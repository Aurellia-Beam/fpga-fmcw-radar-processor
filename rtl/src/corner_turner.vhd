library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Corner Turning Memory: Matrix Transpose for Range-Doppler Processing
-- Writes: Row-wise (Range bins from Range FFT)
-- Reads: Column-wise (for Doppler FFT)
-- Uses ping-pong buffering for continuous streaming

entity corner_turner is
    Generic (
        N_RANGE   : integer := 1024;  -- Range bins (rows)
        N_DOPPLER : integer := 128;   -- Doppler bins / Number of chirps (columns)
        DATA_WIDTH : integer := 32    -- Complex I/Q data (16+16)
    );
    Port (
        aclk      : in  STD_LOGIC;
        aresetn   : in  STD_LOGIC;
        
        -- Input: Range FFT output (row-wise)
        s_axis_tdata  : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;  -- End of one chirp (1024 samples)
        
        -- Output: Column-wise for Doppler FFT
        m_axis_tdata  : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC   -- End of one Doppler FFT frame (128 samples)
    );
end corner_turner;

architecture Behavioral of corner_turner is

    -- Dual-port BRAM type
    type ram_type is array (0 to N_RANGE*N_DOPPLER-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    
    -- Ping-pong buffers
    signal ram_a : ram_type := (others => (others => '0'));
    signal ram_b : ram_type := (others => (others => '0'));
    
    -- Control signals
    type state_type is (WRITE_A_READ_B, WRITE_B_READ_A);
    signal state : state_type := WRITE_A_READ_B;
    signal first_frame : std_logic := '1';  -- Cold start handling
    
    -- Write side counters
    signal wr_chirp_count : integer range 0 to N_DOPPLER-1 := 0;
    signal wr_sample_count : integer range 0 to N_RANGE-1 := 0;
    signal wr_addr : integer range 0 to N_RANGE*N_DOPPLER-1 := 0;
    signal wr_frame_complete : std_logic := '0';
    
    -- Read side counters
    signal rd_range_bin : integer range 0 to N_RANGE-1 := 0;
    signal rd_doppler_bin : integer range 0 to N_DOPPLER-1 := 0;
    signal rd_addr : integer range 0 to N_RANGE*N_DOPPLER-1 := 0;
    signal rd_frame_complete : std_logic := '0';
    signal rd_active : std_logic := '0';
    
    -- Output pipeline
    signal m_tdata_reg : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal m_tvalid_reg : std_logic := '0';
    signal m_tlast_reg : std_logic := '0';
    
    -- Internal signal for tready (can't read from out port)
    signal s_tready_int : std_logic := '0';

begin

    -- Connect internal ready signal to output port
    s_axis_tready <= s_tready_int;

    -- Write Address Calculation (row-wise: chirp*N_RANGE + sample)
    wr_addr <= wr_chirp_count * N_RANGE + wr_sample_count;
    
    -- Read Address Calculation (column-wise: doppler_bin*N_RANGE + range_bin)
    rd_addr <= rd_doppler_bin * N_RANGE + rd_range_bin;

    -- ========================================================================
    -- WRITE PROCESS: Accept Range FFT output, store row-wise
    -- ========================================================================
    process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                wr_chirp_count <= 0;
                wr_sample_count <= 0;
                wr_frame_complete <= '0';
                s_tready_int <= '0';
                
            else
                -- Always ready to accept data (assuming sufficient bandwidth)
                s_tready_int <= '1';
                wr_frame_complete <= '0';
                
                if s_axis_tvalid = '1' and s_tready_int = '1' then
                    -- Write to appropriate bank (calculate address inline to avoid signal delay)
                    if state = WRITE_A_READ_B then
                        ram_a(wr_chirp_count * N_RANGE + wr_sample_count) <= s_axis_tdata;
                    else
                        ram_b(wr_chirp_count * N_RANGE + wr_sample_count) <= s_axis_tdata;
                    end if;
                    
                    -- Advance sample counter
                    if s_axis_tlast = '1' then
                        -- End of chirp
                        wr_sample_count <= 0;
                        
                        if wr_chirp_count = N_DOPPLER - 1 then
                            -- Full frame written
                            wr_chirp_count <= 0;
                            wr_frame_complete <= '1';
                        else
                            wr_chirp_count <= wr_chirp_count + 1;
                        end if;
                    else
                        wr_sample_count <= wr_sample_count + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- READ PROCESS: Read column-wise for Doppler FFT
    -- ========================================================================
    process(aclk)
        variable rd_data : std_logic_vector(DATA_WIDTH-1 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                rd_range_bin <= 0;
                rd_doppler_bin <= 0;
                rd_frame_complete <= '0';
                rd_active <= '0';
                m_tdata_reg <= (others => '0');
                m_tvalid_reg <= '0';
                m_tlast_reg <= '0';
                
            else
                -- Start reading when write frame completes
                if wr_frame_complete = '1' and rd_active = '0' then
                    rd_active <= '1';
                    rd_range_bin <= 0;
                    rd_doppler_bin <= 0;
                end if;
                
                -- Read when active and downstream is ready
                if rd_active = '1' and (m_tvalid_reg = '0' or m_axis_tready = '1') then
                    -- Read from appropriate bank (calculate address inline to avoid signal delay)
                    -- On first frame: read from Bank A (which was just filled)
                    -- On subsequent frames: proper ping-pong operation
                    if first_frame = '1' or state = WRITE_B_READ_A then
                        rd_data := ram_a(rd_doppler_bin * N_RANGE + rd_range_bin);
                    else
                        rd_data := ram_b(rd_doppler_bin * N_RANGE + rd_range_bin);
                    end if;
                    
                    m_tdata_reg <= rd_data;
                    m_tvalid_reg <= '1';
                    
                    -- Generate TLAST at end of each Doppler column
                    if rd_doppler_bin = N_DOPPLER - 1 then
                        m_tlast_reg <= '1';
                    else
                        m_tlast_reg <= '0';
                    end if;
                    
                    -- Advance read counters
                    if rd_doppler_bin = N_DOPPLER - 1 then
                        rd_doppler_bin <= 0;
                        
                        if rd_range_bin = N_RANGE - 1 then
                            -- Full frame read
                            rd_range_bin <= 0;
                            rd_frame_complete <= '1';
                            rd_active <= '0';
                            first_frame <= '0';  -- Clear after first frame read
                        else
                            rd_range_bin <= rd_range_bin + 1;
                        end if;
                    else
                        rd_doppler_bin <= rd_doppler_bin + 1;
                    end if;
                    
                elsif m_axis_tready = '1' then
                    m_tvalid_reg <= '0';
                end if;
            end if;
        end if;
    end process;
    
    -- Output assignments
    m_axis_tdata <= m_tdata_reg;
    m_axis_tvalid <= m_tvalid_reg;
    m_axis_tlast <= m_tlast_reg;

    -- ========================================================================
    -- PING-PONG STATE MACHINE: Toggle banks when frame completes
    -- ========================================================================
    process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                state <= WRITE_A_READ_B;
                first_frame <= '1';
            else
                -- Toggle state when write frame completes
                if wr_frame_complete = '1' then
                    if state = WRITE_A_READ_B then
                        state <= WRITE_B_READ_A;
                    else
                        state <= WRITE_A_READ_B;
                    end if;
                end if;
            end if;
        end if;
    end process;

end Behavioral;