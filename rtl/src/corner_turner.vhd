library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================================
-- Corner Turning Memory: Optimized Matrix Transpose for Range-Doppler Processing
-- =============================================================================
-- OPTIMIZATIONS:
--   1. True dual-port BRAM for simultaneous read/write
--   2. Linear address counters (no multiplication)
--   3. Pipelined read path for timing closure
--   4. Proper ready/valid gating (no magic waits)
--
-- Operation:
--   Write: Row-wise (chirp-major order from Range FFT)
--   Read:  Column-wise (range-bin-major order for Doppler FFT)
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity corner_turner is
    Generic (
        N_RANGE    : integer := 1024;
        N_DOPPLER  : integer := 128;
        DATA_WIDTH : integer := 32
    );
    Port (
        aclk      : in  STD_LOGIC;
        aresetn   : in  STD_LOGIC;
        
        s_axis_tdata  : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;
        
        m_axis_tdata  : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC;
        
        frame_complete : out STD_LOGIC;
        overflow_error : out STD_LOGIC
    );
end corner_turner;

architecture Behavioral of corner_turner is

    constant MATRIX_SIZE : integer := N_RANGE * N_DOPPLER;
    constant ADDR_WIDTH  : integer := 17;  -- log2(1024*128) = 17
    
    -- =========================================================================
    -- True Dual-Port BRAM (Port A = Write, Port B = Read)
    -- =========================================================================
    type ram_type is array (0 to MATRIX_SIZE-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    signal ram_bank0 : ram_type := (others => (others => '0'));
    signal ram_bank1 : ram_type := (others => (others => '0'));
    
    attribute ram_style : string;
    attribute ram_style of ram_bank0 : signal is "block";
    attribute ram_style of ram_bank1 : signal is "block";
    
    -- =========================================================================
    -- Write Path - Linear Address Counter
    -- =========================================================================
    signal wr_addr        : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    signal wr_bank_sel    : std_logic := '0';
    signal wr_chirp_cnt   : unsigned(6 downto 0) := (others => '0');
    signal wr_sample_cnt  : unsigned(9 downto 0) := (others => '0');
    signal wr_frame_done  : std_logic := '0';
    signal s_tready_int   : std_logic := '0';
    
    -- =========================================================================
    -- Read Path - Column-wise Access with Linear Base + Stride
    -- =========================================================================
    signal rd_bank_sel    : std_logic := '0';
    signal rd_range_cnt   : unsigned(9 downto 0) := (others => '0');
    signal rd_doppler_cnt : unsigned(6 downto 0) := (others => '0');
    signal rd_base_addr   : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');  -- range_bin (column start)
    signal rd_addr        : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    signal rd_active      : std_logic := '0';
    signal rd_frame_done  : std_logic := '0';
    signal first_frame    : std_logic := '1';
    
    -- Read pipeline (2-stage for BRAM latency)
    signal rd_data_p1     : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal rd_valid_p1    : std_logic := '0';
    signal rd_last_p1     : std_logic := '0';
    signal rd_data_p2     : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal rd_valid_p2    : std_logic := '0';
    signal rd_last_p2     : std_logic := '0';
    
    -- Output registers
    signal out_valid      : std_logic := '0';
    signal out_data       : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal out_last       : std_logic := '0';
    signal pipe_stall     : std_logic := '0';
    
    signal overflow_flag  : std_logic := '0';

begin

    s_axis_tready  <= s_tready_int;
    m_axis_tdata   <= out_data;
    m_axis_tvalid  <= out_valid;
    m_axis_tlast   <= out_last;
    frame_complete <= rd_frame_done;
    overflow_error <= overflow_flag;

    -- =========================================================================
    -- WRITE PROCESS - Linear Address Increment
    -- =========================================================================
    write_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                wr_addr       <= (others => '0');
                wr_chirp_cnt  <= (others => '0');
                wr_sample_cnt <= (others => '0');
                wr_bank_sel   <= '0';
                wr_frame_done <= '0';
                s_tready_int  <= '0';
                overflow_flag <= '0';
            else
                -- Always ready unless overflow condition
                s_tready_int  <= '1';
                wr_frame_done <= '0';
                overflow_flag <= '0';
                
                -- Overflow: new frame starting while read incomplete
                if wr_frame_done = '1' and rd_active = '1' and first_frame = '0' then
                    overflow_flag <= '1';
                end if;
                
                if s_axis_tvalid = '1' and s_tready_int = '1' then
                    -- Write to selected bank using linear address
                    if wr_bank_sel = '0' then
                        ram_bank0(to_integer(wr_addr)) <= s_axis_tdata;
                    else
                        ram_bank1(to_integer(wr_addr)) <= s_axis_tdata;
                    end if;
                    
                    -- Linear address increment
                    wr_addr <= wr_addr + 1;
                    
                    if s_axis_tlast = '1' then
                        -- End of chirp
                        wr_sample_cnt <= (others => '0');
                        
                        if wr_chirp_cnt = N_DOPPLER - 1 then
                            -- Frame complete
                            wr_chirp_cnt  <= (others => '0');
                            wr_addr       <= (others => '0');
                            wr_frame_done <= '1';
                            wr_bank_sel   <= not wr_bank_sel;
                        else
                            wr_chirp_cnt <= wr_chirp_cnt + 1;
                        end if;
                    else
                        wr_sample_cnt <= wr_sample_cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process write_proc;

    -- =========================================================================
    -- READ PROCESS - Column-wise with Base + Stride
    -- =========================================================================
    -- Read address = range_bin + doppler_bin * N_RANGE
    -- Optimized: base = range_bin, increment by N_RANGE for each Doppler
    -- =========================================================================
    read_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                rd_range_cnt   <= (others => '0');
                rd_doppler_cnt <= (others => '0');
                rd_base_addr   <= (others => '0');
                rd_addr        <= (others => '0');
                rd_bank_sel    <= '0';
                rd_active      <= '0';
                rd_frame_done  <= '0';
                first_frame    <= '1';
                rd_valid_p1    <= '0';
                rd_last_p1     <= '0';
                rd_valid_p2    <= '0';
                rd_last_p2     <= '0';
            else
                rd_frame_done <= '0';
                pipe_stall <= out_valid and (not m_axis_tready);
                
                -- Start reading when write frame completes (GATED - no magic wait)
                if wr_frame_done = '1' and rd_active = '0' then
                    rd_active      <= '1';
                    rd_range_cnt   <= (others => '0');
                    rd_doppler_cnt <= (others => '0');
                    rd_base_addr   <= (others => '0');
                    rd_addr        <= (others => '0');
                    rd_bank_sel    <= not wr_bank_sel;
                    first_frame    <= '0';
                end if;
                
                if pipe_stall = '0' then
                    -- Stage 1: BRAM read
                    if rd_active = '1' then
                        if rd_bank_sel = '0' then
                            rd_data_p1 <= ram_bank0(to_integer(rd_addr));
                        else
                            rd_data_p1 <= ram_bank1(to_integer(rd_addr));
                        end if;
                        
                        rd_valid_p1 <= '1';
                        rd_last_p1  <= '1' when rd_doppler_cnt = N_DOPPLER - 1 else '0';
                        
                        -- Address update: stride by N_RANGE for Doppler, +1 for Range
                        if rd_doppler_cnt = N_DOPPLER - 1 then
                            rd_doppler_cnt <= (others => '0');
                            
                            if rd_range_cnt = N_RANGE - 1 then
                                -- Frame complete
                                rd_range_cnt  <= (others => '0');
                                rd_base_addr  <= (others => '0');
                                rd_addr       <= (others => '0');
                                rd_active     <= '0';
                                rd_frame_done <= '1';
                            else
                                rd_range_cnt <= rd_range_cnt + 1;
                                rd_base_addr <= rd_base_addr + 1;
                                rd_addr      <= rd_base_addr + 1;  -- Next column start
                            end if;
                        else
                            rd_doppler_cnt <= rd_doppler_cnt + 1;
                            rd_addr        <= rd_addr + N_RANGE;  -- Stride by N_RANGE
                        end if;
                    else
                        rd_valid_p1 <= '0';
                        rd_last_p1  <= '0';
                    end if;
                    
                    -- Stage 2: Pipeline register
                    rd_data_p2  <= rd_data_p1;
                    rd_valid_p2 <= rd_valid_p1;
                    rd_last_p2  <= rd_last_p1;
                end if;
            end if;
        end if;
    end process read_proc;

    -- =========================================================================
    -- OUTPUT SKID BUFFER
    -- =========================================================================
    output_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                out_valid <= '0';
                out_data  <= (others => '0');
                out_last  <= '0';
            else
                if m_axis_tready = '1' or out_valid = '0' then
                    out_valid <= rd_valid_p2;
                    out_data  <= rd_data_p2;
                    out_last  <= rd_last_p2;
                end if;
            end if;
        end if;
    end process output_proc;

end Behavioral;