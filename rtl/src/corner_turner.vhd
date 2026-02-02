library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================================
-- Corner Turning Memory: Matrix Transpose for Range-Doppler Processing
-- =============================================================================
-- Optimized for Xilinx FPGA synthesis with proper BRAM inference
-- 
-- Operation:
--   Write: Row-wise (chirp-major order from Range FFT)
--   Read:  Column-wise (range-bin-major order for Doppler FFT)
--
-- Memory Layout:
--   Address = chirp * N_RANGE + sample  (write)
--   Address = doppler * N_RANGE + range (read, transposed access)
--
-- Ping-Pong Architecture:
--   - Bank A and Bank B alternate roles
--   - While writing to one bank, read from the other
--   - First frame has latency (must fill before reading)
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity corner_turner is
    Generic (
        N_RANGE    : integer := 1024;  -- Range bins per chirp
        N_DOPPLER  : integer := 128;   -- Chirps per CPI (Doppler bins)
        DATA_WIDTH : integer := 32     -- Complex I/Q: 16-bit I + 16-bit Q
    );
    Port (
        aclk      : in  STD_LOGIC;
        aresetn   : in  STD_LOGIC;
        
        -- AXI-Stream Slave: Range FFT output (row-wise)
        s_axis_tdata  : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;  -- End of chirp
        
        -- AXI-Stream Master: Column-wise output for Doppler FFT
        m_axis_tdata  : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC;  -- End of Doppler column
        
        -- Status outputs (active high)
        frame_complete : out STD_LOGIC;  -- Pulse when read frame completes
        overflow_error : out STD_LOGIC   -- Write attempted while read incomplete
    );
end corner_turner;

architecture Behavioral of corner_turner is

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant MATRIX_SIZE : integer := N_RANGE * N_DOPPLER;
    
    -- =========================================================================
    -- BRAM Declarations with Xilinx synthesis attributes
    -- =========================================================================
    type ram_type is array (0 to MATRIX_SIZE-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    
    signal ram_a : ram_type := (others => (others => '0'));
    signal ram_b : ram_type := (others => (others => '0'));
    
    -- Synthesis attributes for proper BRAM inference
    attribute ram_style : string;
    attribute ram_style of ram_a : signal is "block";
    attribute ram_style of ram_b : signal is "block";
    
    -- =========================================================================
    -- Write Path Signals
    -- =========================================================================
    signal wr_bank_sel    : std_logic := '0';  -- 0=Bank A, 1=Bank B
    signal wr_chirp_cnt   : unsigned(6 downto 0) := (others => '0');   -- 0 to 127
    signal wr_sample_cnt  : unsigned(9 downto 0) := (others => '0');   -- 0 to 1023
    signal wr_en          : std_logic := '0';
    signal s_tready_int   : std_logic := '0';
    signal wr_frame_done  : std_logic := '0';
    
    -- =========================================================================
    -- Read Path Signals
    -- =========================================================================
    signal rd_bank_sel    : std_logic := '0';  -- 0=Bank A, 1=Bank B
    signal rd_range_cnt   : unsigned(9 downto 0) := (others => '0');   -- 0 to 1023
    signal rd_doppler_cnt : unsigned(6 downto 0) := (others => '0');   -- 0 to 127
    signal rd_active      : std_logic := '0';
    signal rd_frame_done  : std_logic := '0';
    signal first_frame    : std_logic := '1';
    
    -- Read pipeline (2-stage for BRAM latency)
    signal rd_data_p1     : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal rd_valid_p1    : std_logic := '0';
    signal rd_last_p1     : std_logic := '0';
    signal rd_valid_p2    : std_logic := '0';
    signal rd_last_p2     : std_logic := '0';
    signal rd_data_p2     : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    
    -- Output skid buffer for backpressure handling
    signal out_valid      : std_logic := '0';
    signal out_data       : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal out_last       : std_logic := '0';
    signal pipe_stall     : std_logic := '0';
    
    -- Error detection
    signal overflow_flag  : std_logic := '0';

begin

    -- =========================================================================
    -- Output Assignments
    -- =========================================================================
    -- Note: Address calculations are done inline in processes using variables
    s_axis_tready <= s_tready_int;
    m_axis_tdata  <= out_data;
    m_axis_tvalid <= out_valid;
    m_axis_tlast  <= out_last;
    frame_complete <= rd_frame_done;
    overflow_error <= overflow_flag;

    -- =========================================================================
    -- WRITE PROCESS
    -- =========================================================================
    -- Uses variables for bank selection to avoid signal timing issues
    -- =========================================================================
    write_proc: process(aclk)
        variable v_bank_sel : std_logic := '0';
        variable v_wr_addr  : integer range 0 to MATRIX_SIZE-1;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                wr_chirp_cnt  <= (others => '0');
                wr_sample_cnt <= (others => '0');
                wr_frame_done <= '0';
                s_tready_int  <= '0';
                v_bank_sel    := '0';
                wr_bank_sel   <= '0';
                overflow_flag <= '0';
                
            else
                -- Default: ready to accept (can throttle if needed)
                s_tready_int  <= '1';
                wr_frame_done <= '0';
                overflow_flag <= '0';
                
                -- Check for overflow: write frame completing while read still active
                if wr_frame_done = '1' and rd_active = '1' and first_frame = '0' then
                    overflow_flag <= '1';
                end if;
                
                -- Write on valid handshake
                if s_axis_tvalid = '1' and s_tready_int = '1' then
                    -- Calculate address using current counter values
                    v_wr_addr := to_integer(wr_chirp_cnt) * N_RANGE + 
                                 to_integer(wr_sample_cnt);
                    
                    -- Write to selected bank (using variable for immediate update)
                    if v_bank_sel = '0' then
                        ram_a(v_wr_addr) <= s_axis_tdata;
                    else
                        ram_b(v_wr_addr) <= s_axis_tdata;
                    end if;
                    
                    -- Advance counters
                    if s_axis_tlast = '1' then
                        -- End of chirp
                        wr_sample_cnt <= (others => '0');
                        
                        if wr_chirp_cnt = N_DOPPLER - 1 then
                            -- Frame complete - toggle bank IMMEDIATELY (variable)
                            wr_chirp_cnt  <= (others => '0');
                            wr_frame_done <= '1';
                            v_bank_sel    := not v_bank_sel;
                            wr_bank_sel   <= v_bank_sel;  -- Update signal for read side
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
    -- READ PROCESS
    -- =========================================================================
    -- Column-wise read with 2-cycle BRAM latency pipeline
    -- =========================================================================
    read_proc: process(aclk)
        variable v_rd_addr : integer range 0 to MATRIX_SIZE-1;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                rd_range_cnt   <= (others => '0');
                rd_doppler_cnt <= (others => '0');
                rd_active      <= '0';
                rd_frame_done  <= '0';
                rd_bank_sel    <= '0';
                first_frame    <= '1';
                rd_valid_p1    <= '0';
                rd_last_p1     <= '0';
                rd_valid_p2    <= '0';
                rd_last_p2     <= '0';
                
            else
                rd_frame_done <= '0';
                
                -- Pipeline stall when output is full and downstream not ready
                pipe_stall <= out_valid and (not m_axis_tready);
                
                -- Start reading when write frame completes
                if wr_frame_done = '1' and rd_active = '0' then
                    rd_active      <= '1';
                    rd_range_cnt   <= (others => '0');
                    rd_doppler_cnt <= (others => '0');
                    rd_bank_sel    <= not wr_bank_sel;  -- Read from just-completed bank
                    first_frame    <= '0';
                end if;
                
                -- Pipeline advance when not stalled
                if pipe_stall = '0' then
                    -- Stage 1: BRAM read
                    if rd_active = '1' then
                        v_rd_addr := to_integer(rd_doppler_cnt) * N_RANGE + 
                                     to_integer(rd_range_cnt);
                        
                        if rd_bank_sel = '0' then
                            rd_data_p1 <= ram_a(v_rd_addr);
                        else
                            rd_data_p1 <= ram_b(v_rd_addr);
                        end if;
                        
                        rd_valid_p1 <= '1';
                        if rd_doppler_cnt = N_DOPPLER - 1 then
                            rd_last_p1 <= '1';
                        else
                            rd_last_p1 <= '0';
                        end if;
                        
                        -- Advance read counters (column-wise: doppler varies fast)
                        if rd_doppler_cnt = N_DOPPLER - 1 then
                            rd_doppler_cnt <= (others => '0');
                            
                            if rd_range_cnt = N_RANGE - 1 then
                                -- Frame complete
                                rd_range_cnt  <= (others => '0');
                                rd_active     <= '0';
                                rd_frame_done <= '1';
                            else
                                rd_range_cnt <= rd_range_cnt + 1;
                            end if;
                        else
                            rd_doppler_cnt <= rd_doppler_cnt + 1;
                        end if;
                    else
                        rd_valid_p1 <= '0';
                        rd_last_p1  <= '0';
                    end if;
                    
                    -- Stage 2: Output register
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
    -- Handles AXI-Stream backpressure without losing data
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