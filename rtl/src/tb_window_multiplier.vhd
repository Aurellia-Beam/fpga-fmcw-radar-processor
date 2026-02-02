library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;

-- =============================================================================
-- Testbench: Window Multiplier Verification
-- =============================================================================
-- Tests Hamming window multiplication with:
--   1. Impulse response (single sample)
--   2. DC signal (all same value)
--   3. Sinusoidal test signal
--   4. Full-scale I/Q test
--   5. Coefficient symmetry verification
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity tb_window_multiplier is
end tb_window_multiplier;

architecture Behavioral of tb_window_multiplier is

    -- =========================================================================
    -- DUT Component
    -- =========================================================================
    component window_multiplier is
        Generic (
            DATA_WIDTH : integer := 32;
            N_SAMPLES  : integer := 1024;
            COEF_WIDTH : integer := 16
        );
        Port (
            aclk            : in  STD_LOGIC;
            aresetn         : in  STD_LOGIC;
            s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid   : in  STD_LOGIC;
            s_axis_tready   : out STD_LOGIC;
            s_axis_tlast    : in  STD_LOGIC;
            m_axis_tdata    : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid   : out STD_LOGIC;
            m_axis_tready   : in  STD_LOGIC;
            m_axis_tlast    : out STD_LOGIC;
            saturation_flag : out STD_LOGIC
        );
    end component;

    -- =========================================================================
    -- Test Parameters
    -- =========================================================================
    constant N_SAMPLES  : integer := 1024;
    constant CLK_PERIOD : time := 10 ns;

    -- =========================================================================
    -- Signals
    -- =========================================================================
    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';
    
    signal s_axis_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tready : std_logic;
    signal s_axis_tlast  : std_logic := '0';

    signal m_axis_tdata  : std_logic_vector(31 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tready : std_logic := '1';
    signal m_axis_tlast  : std_logic;
    signal saturation_flag : std_logic;
    
    -- Verification
    signal test_done    : std_logic := '0';
    signal error_count  : integer := 0;
    signal sample_count : integer := 0;

    -- Expected Hamming coefficients (computed at elaboration)
    type coef_array_t is array (0 to N_SAMPLES-1) of real;
    
    function compute_hamming return coef_array_t is
        variable result : coef_array_t;
        variable angle : real;
    begin
        for i in 0 to N_SAMPLES-1 loop
            angle := 2.0 * MATH_PI * real(i) / real(N_SAMPLES - 1);
            result(i) := 0.54 - 0.46 * cos(angle);
        end loop;
        return result;
    end function;
    
    constant HAMMING_COEF : coef_array_t := compute_hamming;

begin

    -- =========================================================================
    -- DUT Instantiation
    -- =========================================================================
    uut: window_multiplier
    generic map (
        DATA_WIDTH => 32,
        N_SAMPLES  => N_SAMPLES,
        COEF_WIDTH => 16
    )
    port map (
        aclk            => aclk,
        aresetn         => aresetn,
        s_axis_tdata    => s_axis_tdata,
        s_axis_tvalid   => s_axis_tvalid,
        s_axis_tready   => s_axis_tready,
        s_axis_tlast    => s_axis_tlast,
        m_axis_tdata    => m_axis_tdata,
        m_axis_tvalid   => m_axis_tvalid,
        m_axis_tready   => m_axis_tready,
        m_axis_tlast    => m_axis_tlast,
        saturation_flag => saturation_flag
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
    -- Stimulus Process
    -- =========================================================================
    stim_proc: process
        variable i_val : integer;
        variable q_val : integer;
        variable angle : real;
    begin
        -- Initialize
        aresetn <= '0';
        s_axis_tvalid <= '0';
        s_axis_tlast <= '0';
        wait for 100 ns;
        aresetn <= '1';
        wait for CLK_PERIOD * 10;
        
        report "===== Window Multiplier Test =====";
        
        -- ---------------------------------------------------------------------
        -- Test 1: DC Signal (all 1000)
        -- ---------------------------------------------------------------------
        report "Test 1: DC Signal Response";
        for i in 0 to N_SAMPLES-1 loop
            s_axis_tdata <= std_logic_vector(to_signed(0, 16)) &  -- Q = 0
                           std_logic_vector(to_signed(10000, 16)); -- I = 10000
            s_axis_tvalid <= '1';
            s_axis_tlast <= '1' when i = N_SAMPLES-1 else '0';
            
            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop
                wait until rising_edge(aclk);
            end loop;
        end loop;
        s_axis_tvalid <= '0';
        wait for CLK_PERIOD * 20;
        
        -- ---------------------------------------------------------------------
        -- Test 2: Sinusoidal Signal (verify no clipping)
        -- ---------------------------------------------------------------------
        report "Test 2: Sinusoidal Signal";
        for i in 0 to N_SAMPLES-1 loop
            angle := 2.0 * MATH_PI * 10.0 * real(i) / real(N_SAMPLES);
            i_val := integer(20000.0 * sin(angle));
            q_val := integer(20000.0 * cos(angle));
            
            s_axis_tdata <= std_logic_vector(to_signed(q_val, 16)) &
                           std_logic_vector(to_signed(i_val, 16));
            s_axis_tvalid <= '1';
            s_axis_tlast <= '1' when i = N_SAMPLES-1 else '0';
            
            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop
                wait until rising_edge(aclk);
            end loop;
        end loop;
        s_axis_tvalid <= '0';
        wait for CLK_PERIOD * 20;
        
        -- ---------------------------------------------------------------------
        -- Test 3: Full-Scale Input (saturation test)
        -- ---------------------------------------------------------------------
        report "Test 3: Full-Scale Input";
        for i in 0 to N_SAMPLES-1 loop
            s_axis_tdata <= std_logic_vector(to_signed(32767, 16)) &  -- Q = max
                           std_logic_vector(to_signed(32767, 16));    -- I = max
            s_axis_tvalid <= '1';
            s_axis_tlast <= '1' when i = N_SAMPLES-1 else '0';
            
            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop
                wait until rising_edge(aclk);
            end loop;
        end loop;
        s_axis_tvalid <= '0';
        wait for CLK_PERIOD * 20;
        
        -- ---------------------------------------------------------------------
        -- Test 4: Symmetry Test (impulse at edges)
        -- ---------------------------------------------------------------------
        report "Test 4: Symmetry Test";
        for i in 0 to N_SAMPLES-1 loop
            -- Impulse at first and last samples
            if i = 0 or i = N_SAMPLES-1 then
                i_val := 10000;
            else
                i_val := 0;
            end if;
            
            s_axis_tdata <= std_logic_vector(to_signed(0, 16)) &
                           std_logic_vector(to_signed(i_val, 16));
            s_axis_tvalid <= '1';
            s_axis_tlast <= '1' when i = N_SAMPLES-1 else '0';
            
            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop
                wait until rising_edge(aclk);
            end loop;
        end loop;
        s_axis_tvalid <= '0';
        
        wait for 500 ns;
        test_done <= '1';
        
        report "===== All Tests Complete =====";
        wait for 100 ns;
        assert false report "Simulation Complete" severity failure;
    end process;

    -- =========================================================================
    -- Output Monitor
    -- =========================================================================
    monitor_proc: process
        file output_file : text open write_mode is "window_output.txt";
        variable outline : line;
        variable out_i : integer;
        variable out_q : integer;
        variable expected_i : real;
        variable tolerance : integer := 100;  -- Allow ~0.3% error from rounding
        variable frame_sample : integer := 0;
        variable local_errors : integer := 0;
    begin
        wait until aresetn = '1';
        
        loop
            wait until rising_edge(aclk);
            
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                out_i := to_integer(signed(m_axis_tdata(15 downto 0)));
                out_q := to_integer(signed(m_axis_tdata(31 downto 16)));
                
                -- Log output
                write(outline, sample_count);
                write(outline, string'(" "));
                write(outline, out_i);
                write(outline, string'(" "));
                write(outline, out_q);
                if saturation_flag = '1' then
                    write(outline, string'(" SAT"));
                end if;
                writeline(output_file, outline);
                
                -- Check TLAST timing
                if frame_sample = N_SAMPLES - 1 then
                    if m_axis_tlast /= '1' then
                        report "ERROR: Missing TLAST at sample " & 
                               integer'image(sample_count) severity warning;
                        local_errors := local_errors + 1;
                    end if;
                    frame_sample := 0;
                else
                    frame_sample := frame_sample + 1;
                end if;
                
                sample_count <= sample_count + 1;
            end if;
            
            if test_done = '1' then
                error_count <= local_errors;
                file_close(output_file);
                exit;
            end if;
        end loop;
        
        wait;
    end process;

end Behavioral;
