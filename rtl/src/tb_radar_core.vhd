library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;

-- =============================================================================
-- Testbench: Complete Radar Core v2 Verification
-- =============================================================================
-- Simulates FMCW radar receive pipeline with:
--   1. Synthetic chirp generation with targets
--   2. Full CPI processing (128 chirps x 1024 samples)
--   3. Output verification for corner turner transpose
--
-- Signal Model:
--   - Multiple range targets at known bins
--   - Includes Doppler shift for moving targets
--   - Noise floor simulation
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity tb_radar_core is
end tb_radar_core;

architecture Behavioral of tb_radar_core is

    -- =========================================================================
    -- DUT Component
    -- =========================================================================
    component radar_core_v2 is
        Generic (
            N_RANGE   : integer := 1024;
            N_DOPPLER : integer := 128
        );
        Port (
            aclk                   : in  STD_LOGIC;
            aresetn                : in  STD_LOGIC;
            s_axis_tdata           : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid          : in  STD_LOGIC;
            s_axis_tlast           : in  STD_LOGIC;
            s_axis_tready          : out STD_LOGIC;
            m_axis_tdata           : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid          : out STD_LOGIC;
            m_axis_tlast           : out STD_LOGIC;
            m_axis_tready          : in  STD_LOGIC;
            status_fft_config_done : out STD_LOGIC;
            status_frame_complete  : out STD_LOGIC;
            status_window_saturate : out STD_LOGIC;
            status_corner_overflow : out STD_LOGIC;
            diag_chirp_count       : out STD_LOGIC_VECTOR(7 downto 0);
            diag_sample_count      : out STD_LOGIC_VECTOR(9 downto 0)
        );
    end component;

    -- =========================================================================
    -- Test Parameters
    -- =========================================================================
    constant N_RANGE   : integer := 1024;
    constant N_DOPPLER : integer := 128;
    constant CLK_PERIOD : time := 10 ns;  -- 100 MHz
    
    -- Target parameters (for signal generation)
    constant TARGET_1_RANGE : integer := 100;   -- Range bin
    constant TARGET_1_DOPPLER : real := 5.0;    -- Doppler bin (velocity)
    constant TARGET_1_AMP : real := 8000.0;     -- Amplitude
    
    constant TARGET_2_RANGE : integer := 500;
    constant TARGET_2_DOPPLER : real := -10.0;  -- Approaching target
    constant TARGET_2_AMP : real := 5000.0;
    
    constant NOISE_FLOOR : real := 100.0;       -- RMS noise level

    -- =========================================================================
    -- Signals
    -- =========================================================================
    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';
    
    -- Input interface
    signal s_axis_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tlast  : std_logic := '0';
    signal s_axis_tready : std_logic;
    
    -- Output interface
    signal m_axis_tdata  : std_logic_vector(31 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tlast  : std_logic;
    signal m_axis_tready : std_logic := '1';
    
    -- Status signals
    signal status_fft_config_done : std_logic;
    signal status_frame_complete  : std_logic;
    signal status_window_saturate : std_logic;
    signal status_corner_overflow : std_logic;
    signal diag_chirp_count       : std_logic_vector(7 downto 0);
    signal diag_sample_count      : std_logic_vector(9 downto 0);
    
    -- Test control
    signal sim_done : std_logic := '0';
    signal output_sample_count : integer := 0;

begin

    -- =========================================================================
    -- DUT Instantiation
    -- =========================================================================
    uut: radar_core_v2
    generic map (
        N_RANGE   => N_RANGE,
        N_DOPPLER => N_DOPPLER
    )
    port map (
        aclk                   => aclk,
        aresetn                => aresetn,
        s_axis_tdata           => s_axis_tdata,
        s_axis_tvalid          => s_axis_tvalid,
        s_axis_tlast           => s_axis_tlast,
        s_axis_tready          => s_axis_tready,
        m_axis_tdata           => m_axis_tdata,
        m_axis_tvalid          => m_axis_tvalid,
        m_axis_tlast           => m_axis_tlast,
        m_axis_tready          => m_axis_tready,
        status_fft_config_done => status_fft_config_done,
        status_frame_complete  => status_frame_complete,
        status_window_saturate => status_window_saturate,
        status_corner_overflow => status_corner_overflow,
        diag_chirp_count       => diag_chirp_count,
        diag_sample_count      => diag_sample_count
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
    -- FMCW Signal Generator
    -- =========================================================================
    -- Generates synthetic radar returns for multiple targets
    -- Signal model: x[n,m] = sum_k( A_k * exp(j*2*pi*(f_range*n + f_doppler*m)) )
    -- where n = range sample, m = chirp number
    -- =========================================================================
    stimulus_proc: process
        variable seed1, seed2 : positive := 1;
        variable rand : real;
        variable chirp_num : integer;
        variable sample_num : integer;
        variable i_acc, q_acc : real;
        variable phase : real;
        variable i_val, q_val : integer;
    begin
        -- Initialize
        aresetn <= '0';
        s_axis_tvalid <= '0';
        s_axis_tlast <= '0';
        wait for 100 ns;
        aresetn <= '1';
        
        -- Wait for FFT configuration
        report "Waiting for FFT configuration...";
        wait until status_fft_config_done = '1';
        wait for CLK_PERIOD * 10;
        report "FFT configured. Starting data transmission.";
        
        -- Generate 2 complete CPIs (to fill ping-pong and get output)
        for cpi in 0 to 1 loop
            report "=== Generating CPI " & integer'image(cpi) & " ===";
            
            for chirp_num in 0 to N_DOPPLER-1 loop
                for sample_num in 0 to N_RANGE-1 loop
                    -- Initialize accumulators
                    i_acc := 0.0;
                    q_acc := 0.0;
                    
                    -- Target 1: Stationary or slow-moving
                    phase := 2.0 * MATH_PI * (
                        real(TARGET_1_RANGE) * real(sample_num) / real(N_RANGE) +
                        TARGET_1_DOPPLER * real(chirp_num) / real(N_DOPPLER)
                    );
                    i_acc := i_acc + TARGET_1_AMP * cos(phase);
                    q_acc := q_acc + TARGET_1_AMP * sin(phase);
                    
                    -- Target 2: Moving target
                    phase := 2.0 * MATH_PI * (
                        real(TARGET_2_RANGE) * real(sample_num) / real(N_RANGE) +
                        TARGET_2_DOPPLER * real(chirp_num) / real(N_DOPPLER)
                    );
                    i_acc := i_acc + TARGET_2_AMP * cos(phase);
                    q_acc := q_acc + TARGET_2_AMP * sin(phase);
                    
                    -- Add noise
                    uniform(seed1, seed2, rand);
                    i_acc := i_acc + NOISE_FLOOR * (rand - 0.5) * 2.0;
                    uniform(seed1, seed2, rand);
                    q_acc := q_acc + NOISE_FLOOR * (rand - 0.5) * 2.0;
                    
                    -- Quantize to 16-bit
                    i_val := integer(i_acc);
                    q_val := integer(q_acc);
                    
                    -- Clamp to valid range
                    if i_val > 32767 then i_val := 32767;
                    elsif i_val < -32768 then i_val := -32768;
                    end if;
                    if q_val > 32767 then q_val := 32767;
                    elsif q_val < -32768 then q_val := -32768;
                    end if;
                    
                    -- Drive AXI-Stream
                    s_axis_tdata <= std_logic_vector(to_signed(q_val, 16)) &
                                   std_logic_vector(to_signed(i_val, 16));
                    s_axis_tvalid <= '1';
                    s_axis_tlast <= '1' when sample_num = N_RANGE-1 else '0';
                    
                    -- Wait for handshake
                    wait until rising_edge(aclk);
                    while s_axis_tready = '0' loop
                        wait until rising_edge(aclk);
                    end loop;
                end loop;
                
                -- Progress report
                if (chirp_num mod 32) = 31 then
                    report "CPI " & integer'image(cpi) & 
                           ": Sent chirp " & integer'image(chirp_num+1) & "/" &
                           integer'image(N_DOPPLER);
                end if;
            end loop;
        end loop;
        
        s_axis_tvalid <= '0';
        s_axis_tlast <= '0';
        
        report "Data transmission complete. Waiting for processing...";
        
        -- Wait for output to complete
        wait until status_frame_complete = '1';
        wait for 10 us;
        
        sim_done <= '1';
        report "===== Simulation Complete =====";
        report "Total output samples: " & integer'image(output_sample_count);
        
        wait for 1 us;
        assert false report "Simulation Finished" severity failure;
    end process;

    -- =========================================================================
    -- Output Monitor
    -- =========================================================================
    monitor_proc: process
        file output_file : text open write_mode is "radar_output.txt";
        variable outline : line;
        variable out_i, out_q : integer;
        variable range_bin, doppler_bin : integer;
        variable magnitude : real;
    begin
        wait until aresetn = '1';
        
        loop
            wait until rising_edge(aclk);
            exit when sim_done = '1';
            
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                out_i := to_integer(signed(m_axis_tdata(15 downto 0)));
                out_q := to_integer(signed(m_axis_tdata(31 downto 16)));
                
                -- Calculate bin indices
                range_bin := output_sample_count / N_DOPPLER;
                doppler_bin := output_sample_count mod N_DOPPLER;
                
                -- Calculate magnitude (for peak detection)
                magnitude := sqrt(real(out_i*out_i + out_q*out_q));
                
                -- Log to file: range_bin doppler_bin I Q magnitude
                write(outline, range_bin);
                write(outline, string'(" "));
                write(outline, doppler_bin);
                write(outline, string'(" "));
                write(outline, out_i);
                write(outline, string'(" "));
                write(outline, out_q);
                write(outline, string'(" "));
                write(outline, integer(magnitude));
                writeline(output_file, outline);
                
                output_sample_count <= output_sample_count + 1;
                
                -- Report high-magnitude detections (potential targets)
                if magnitude > 50000.0 then
                    report "Strong return at range=" & integer'image(range_bin) &
                           ", doppler=" & integer'image(doppler_bin) &
                           ", mag=" & integer'image(integer(magnitude));
                end if;
            end if;
        end loop;
        
        file_close(output_file);
        wait;
    end process;

    -- =========================================================================
    -- Status Monitor
    -- =========================================================================
    status_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if status_window_saturate = '1' then
                report "WARNING: Window multiplier saturation detected!" 
                       severity warning;
            end if;
            
            if status_corner_overflow = '1' then
                report "ERROR: Corner turner overflow!" severity error;
            end if;
            
            if status_frame_complete = '1' then
                report "Frame complete pulse received.";
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Timeout Watchdog
    -- =========================================================================
    watchdog_proc: process
    begin
        wait for 100 ms;
        if sim_done = '0' then
            report "TIMEOUT: Simulation did not complete!" severity failure;
        end if;
        wait;
    end process;

end Behavioral;
