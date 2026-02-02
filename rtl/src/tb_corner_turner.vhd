library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use std.textio.all;
use IEEE.std_logic_textio.all;

-- =============================================================================
-- Testbench: Corner Turner Verification
-- =============================================================================
-- Comprehensive verification of matrix transpose operation:
--   1. Writes row-wise (chirp-major order)
--   2. Verifies column-wise output (range-major order)
--   3. Checks TLAST timing
--   4. Tests backpressure handling
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity tb_corner_turner is
end tb_corner_turner;

architecture Behavioral of tb_corner_turner is

    -- =========================================================================
    -- DUT Component
    -- =========================================================================
    component corner_turner is
        Generic (
            N_RANGE    : integer := 1024;
            N_DOPPLER  : integer := 128;
            DATA_WIDTH : integer := 32
        );
        Port (
            aclk           : in  STD_LOGIC;
            aresetn        : in  STD_LOGIC;
            s_axis_tdata   : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid  : in  STD_LOGIC;
            s_axis_tready  : out STD_LOGIC;
            s_axis_tlast   : in  STD_LOGIC;
            m_axis_tdata   : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid  : out STD_LOGIC;
            m_axis_tready  : in  STD_LOGIC;
            m_axis_tlast   : out STD_LOGIC;
            frame_complete : out STD_LOGIC;
            overflow_error : out STD_LOGIC
        );
    end component;

    -- =========================================================================
    -- Test Parameters
    -- =========================================================================
    constant N_RANGE   : integer := 1024;
    constant N_DOPPLER : integer := 128;
    constant CLK_PERIOD : time := 10 ns;
    constant MATRIX_SIZE : integer := N_RANGE * N_DOPPLER;
    
    -- Use smaller values for faster simulation (uncomment for quick test)
    -- constant N_RANGE   : integer := 64;
    -- constant N_DOPPLER : integer := 16;

    -- =========================================================================
    -- Signals
    -- =========================================================================
    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';
    
    -- Input interface
    signal s_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_tvalid : std_logic := '0';
    signal s_tready : std_logic;
    signal s_tlast  : std_logic := '0';
    
    -- Output interface
    signal m_tdata  : std_logic_vector(31 downto 0);
    signal m_tvalid : std_logic;
    signal m_tready : std_logic := '1';
    signal m_tlast  : std_logic;
    
    -- Status
    signal frame_complete : std_logic;
    signal overflow_error : std_logic;
    
    -- Verification
    signal write_done   : std_logic := '0';
    signal read_done    : std_logic := '0';
    signal test_failed  : std_logic := '0';
    signal error_count  : integer := 0;
    
    -- Reference memory (shared variable for write/read processes)
    type ref_matrix_t is array (0 to N_DOPPLER-1, 0 to N_RANGE-1) of integer;
    shared variable ref_matrix : ref_matrix_t;

begin

    -- =========================================================================
    -- DUT Instantiation
    -- =========================================================================
    uut: corner_turner
    generic map (
        N_RANGE    => N_RANGE,
        N_DOPPLER  => N_DOPPLER,
        DATA_WIDTH => 32
    )
    port map (
        aclk           => aclk,
        aresetn        => aresetn,
        s_axis_tdata   => s_tdata,
        s_axis_tvalid  => s_tvalid,
        s_axis_tready  => s_tready,
        s_axis_tlast   => s_tlast,
        m_axis_tdata   => m_tdata,
        m_axis_tvalid  => m_tvalid,
        m_axis_tready  => m_tready,
        m_axis_tlast   => m_tlast,
        frame_complete => frame_complete,
        overflow_error => overflow_error
    );

    -- =========================================================================
    -- Clock Generation
    -- =========================================================================
    clk_proc: process
    begin
        aclk <= '0';
        wait for CLK_PERIOD/2;
        aclk <= '1';
        wait for CLK_PERIOD/2;
    end process;

    -- =========================================================================
    -- Stimulus: Write 2 Frames (Row-wise)
    -- =========================================================================
    -- Pattern: test_value = (chirp << 16) | sample
    -- This encodes both indices for easy verification
    -- =========================================================================
    write_proc: process
        variable chirp      : integer;
        variable sample     : integer;
        variable test_value : integer;
    begin
        -- Initialize
        aresetn  <= '0';
        s_tvalid <= '0';
        s_tlast  <= '0';
        s_tdata  <= (others => '0');
        wait for 100 ns;
        aresetn <= '1';
        wait for CLK_PERIOD * 10;
        
        report "===== Starting Corner Turner Test =====";
        report "Matrix size: " & integer'image(N_RANGE) & " x " & integer'image(N_DOPPLER);
        
        -- Write 2 frames for ping-pong verification
        -- Frame 1: Fills bank A
        -- Frame 2: Fills bank B, triggers read of bank A
        for frame in 0 to 1 loop
            report "Writing Frame " & integer'image(frame) & "...";
            
            for chirp in 0 to N_DOPPLER-1 loop
                for sample in 0 to N_RANGE-1 loop
                    -- Encode chirp and sample in test value
                    test_value := (chirp mod N_DOPPLER) * 65536 + sample;
                    
                    -- Store in reference matrix (only frame 0 for verification)
                    if frame = 0 then
                        ref_matrix(chirp, sample) := test_value;
                    end if;
                    
                    -- Drive AXI-Stream
                    s_tvalid <= '1';
                    s_tdata  <= std_logic_vector(to_signed(test_value, 32));
                    
                    -- TLAST at end of each chirp
                    if sample = N_RANGE-1 then
                        s_tlast <= '1';
                    else
                        s_tlast <= '0';
                    end if;
                    
                    -- Wait for handshake
                    wait until rising_edge(aclk);
                    while s_tready = '0' loop
                        wait until rising_edge(aclk);
                    end loop;
                end loop;
                
                -- Progress report every 32 chirps
                if (chirp mod 32) = 31 then
                    report "  Frame " & integer'image(frame) & 
                           ": Written chirp " & integer'image(chirp+1) & "/" & 
                           integer'image(N_DOPPLER);
                end if;
            end loop;
            
            report "Frame " & integer'image(frame) & " write complete.";
        end loop;
        
        s_tvalid <= '0';
        s_tlast  <= '0';
        write_done <= '1';
        
        report "Write phase complete. Sent " & integer'image(2 * MATRIX_SIZE) & " samples.";
        wait;
    end process;

    -- =========================================================================
    -- Monitor: Verify Output (Column-wise)
    -- =========================================================================
    -- Expected order: For each range_bin, output all doppler_bins
    -- Sample[n]: range_bin = n / N_DOPPLER, doppler_bin = n mod N_DOPPLER
    -- =========================================================================
    read_proc: process
        file output_file : text open write_mode is "corner_turner_output.txt";
        variable outline       : line;
        variable sample_count  : integer := 0;
        variable local_errors  : integer := 0;
        variable read_value    : integer;
        variable expected      : integer;
        variable range_bin     : integer;
        variable doppler_bin   : integer;
        variable exp_chirp     : integer;
        variable exp_sample    : integer;
    begin
        -- Wait for reset
        wait until aresetn = '1';
        wait for CLK_PERIOD * 5;
        
        report "Read monitor active...";
        
        -- Main verification loop
        loop
            wait until rising_edge(aclk);
            
            if m_tvalid = '1' and m_tready = '1' then
                read_value := to_integer(signed(m_tdata));
                
                -- Calculate expected position
                -- Output is column-major: range_bin slow, doppler_bin fast
                range_bin   := sample_count / N_DOPPLER;
                doppler_bin := sample_count mod N_DOPPLER;
                
                -- Get expected value from reference
                expected := ref_matrix(doppler_bin, range_bin);
                
                -- Decode for error reporting
                exp_chirp  := expected / 65536;
                exp_sample := expected mod 65536;
                
                -- Verify data
                if read_value /= expected then
                    report "ERROR at sample " & integer'image(sample_count) &
                           " (range=" & integer'image(range_bin) &
                           ", doppler=" & integer'image(doppler_bin) &
                           "): expected " & integer'image(expected) &
                           " (chirp=" & integer'image(exp_chirp) &
                           ", sample=" & integer'image(exp_sample) &
                           "), got " & integer'image(read_value)
                           severity error;
                    local_errors := local_errors + 1;
                    test_failed <= '1';
                end if;
                
                -- Verify TLAST
                if doppler_bin = N_DOPPLER - 1 then
                    if m_tlast /= '1' then
                        report "ERROR: TLAST not asserted at end of range bin " &
                               integer'image(range_bin) severity error;
                        local_errors := local_errors + 1;
                    end if;
                else
                    if m_tlast = '1' then
                        report "ERROR: Spurious TLAST at sample " &
                               integer'image(sample_count) severity error;
                        local_errors := local_errors + 1;
                    end if;
                end if;
                
                -- Log to file
                write(outline, range_bin);
                write(outline, string'(" "));
                write(outline, doppler_bin);
                write(outline, string'(" "));
                write(outline, read_value);
                writeline(output_file, outline);
                
                sample_count := sample_count + 1;
                
                -- Progress report
                if (sample_count mod 10000) = 0 then
                    report "  Read " & integer'image(sample_count) & "/" &
                           integer'image(MATRIX_SIZE) & " samples...";
                end if;
                
                -- Check completion
                if sample_count = MATRIX_SIZE then
                    error_count <= local_errors;
                    read_done <= '1';
                    
                    report "========================================";
                    if local_errors = 0 then
                        report "SUCCESS: Corner Turner Verified!";
                        report "All " & integer'image(MATRIX_SIZE) & 
                               " samples correctly transposed.";
                    else
                        report "FAILURE: " & integer'image(local_errors) & 
                               " errors detected!";
                    end if;
                    report "========================================";
                    
                    file_close(output_file);
                    wait for 1 us;
                    
                    if local_errors = 0 then
                        assert false report "Simulation PASSED" severity failure;
                    else
                        assert false report "Simulation FAILED" severity failure;
                    end if;
                end if;
            end if;
        end loop;
    end process;

    -- =========================================================================
    -- Backpressure Generator (optional stress test)
    -- =========================================================================
    -- Uncomment to test backpressure handling
    -- =========================================================================
    -- backpressure_proc: process(aclk)
    --     variable cnt : integer := 0;
    -- begin
    --     if rising_edge(aclk) then
    --         cnt := cnt + 1;
    --         -- Deassert ready every 7th cycle
    --         if (cnt mod 7) = 0 then
    --             m_tready <= '0';
    --         else
    --             m_tready <= '1';
    --         end if;
    --     end if;
    -- end process;

    -- =========================================================================
    -- Timeout Watchdog
    -- =========================================================================
    watchdog_proc: process
    begin
        wait for 500 ms;  -- Adjust based on matrix size
        if read_done = '0' then
            report "TIMEOUT: Test did not complete in time!" severity failure;
        end if;
        wait;
    end process;

end Behavioral;
