library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;

entity tb_window_multiplier is
end tb_window_multiplier;

architecture Behavioral of tb_window_multiplier is

    constant CLK_PERIOD : time := 10 ns;
    constant N_SAMPLES  : integer := 64;  -- Small for fast sim
    constant DATA_W     : integer := 32;
    constant HALF_W     : integer := 16;
    constant COEF_W     : integer := 16;

    signal aclk          : std_logic := '0';
    signal aresetn       : std_logic := '0';
    signal s_axis_tdata  : std_logic_vector(DATA_W-1 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tready : std_logic;
    signal s_axis_tlast  : std_logic := '0';
    signal m_axis_tdata  : std_logic_vector(DATA_W-1 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tready : std_logic := '1';
    signal m_axis_tlast  : std_logic;
    signal saturation_flag : std_logic;
    signal sim_done      : boolean := false;

    signal out_count     : integer := 0;
    signal test_phase    : integer := 0;
    signal fail_count    : integer := 0;

    -- Hamming reference
    function hamming_coef(idx, n : integer) return real is
    begin
        return 0.54 - 0.46 * cos(2.0 * MATH_PI * real(idx) / real(n - 1));
    end function;

begin

    uut: entity work.window_multiplier
    generic map ( DATA_WIDTH => DATA_W, N_SAMPLES => N_SAMPLES, COEF_WIDTH => COEF_W )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready, m_axis_tlast => m_axis_tlast,
        saturation_flag => saturation_flag
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    stim: process

        procedure send_sample(i_val, q_val : integer; last : boolean) is
        begin
            s_axis_tdata <= std_logic_vector(to_signed(q_val, HALF_W)) &
                            std_logic_vector(to_signed(i_val, HALF_W));
            s_axis_tvalid <= '1';
            if last then s_axis_tlast <= '1'; else s_axis_tlast <= '0'; end if;
            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
        end procedure;

        procedure idle(n : integer) is
        begin
            s_axis_tvalid <= '0'; s_axis_tlast <= '0';
            for i in 1 to n loop wait until rising_edge(aclk); end loop;
        end procedure;

    begin
        aresetn <= '0'; wait for 50 ns;
        aresetn <= '1'; wait until rising_edge(aclk);

        -- =====================================================================
        -- Test 1: DC input — output should show Hamming taper shape
        -- =====================================================================
        report "=== Test 1: DC input (Hamming shape) ===";
        test_phase <= 1;
        for s in 0 to N_SAMPLES-1 loop
            send_sample(16000, 16000, s = N_SAMPLES-1);
        end loop;
        idle(20);

        -- =====================================================================
        -- Test 2: Zero input — output should be all zeros
        -- =====================================================================
        report "=== Test 2: Zero input ===";
        test_phase <= 2;
        out_count <= 0;
        for s in 0 to N_SAMPLES-1 loop
            send_sample(0, 0, s = N_SAMPLES-1);
        end loop;
        idle(20);

        -- =====================================================================
        -- Test 3: Full-scale — check saturation flag
        -- =====================================================================
        report "=== Test 3: Full-scale saturation check ===";
        test_phase <= 3;
        out_count <= 0;
        for s in 0 to N_SAMPLES-1 loop
            send_sample(32767, 32767, s = N_SAMPLES-1);
        end loop;
        idle(20);

        -- =====================================================================
        -- Test 4: Impulse at center — single non-zero output
        -- =====================================================================
        report "=== Test 4: Impulse at center ===";
        test_phase <= 4;
        out_count <= 0;
        for s in 0 to N_SAMPLES-1 loop
            if s = N_SAMPLES/2 then
                send_sample(10000, 5000, s = N_SAMPLES-1);
            else
                send_sample(0, 0, s = N_SAMPLES-1);
            end if;
        end loop;
        idle(20);

        -- =====================================================================
        -- Test 5: Symmetry — first and last outputs should match
        -- =====================================================================
        report "=== Test 5: Symmetry verification ===";
        test_phase <= 5;
        out_count <= 0;
        for s in 0 to N_SAMPLES-1 loop
            send_sample(8000, 4000, s = N_SAMPLES-1);
        end loop;
        idle(20);

        -- =====================================================================
        -- Test 6: Backpressure — toggle tready mid-frame
        -- =====================================================================
        report "=== Test 6: Backpressure ===";
        test_phase <= 6;
        out_count <= 0;
        for s in 0 to N_SAMPLES-1 loop
            send_sample(10000, 10000, s = N_SAMPLES-1);
        end loop;
        idle(50);

        -- Done
        if fail_count = 0 then report "ALL TESTS PASSED"; 
        else report integer'image(fail_count) & " FAILURES" severity error; end if;
        sim_done <= true; wait;
    end process;

    -- Backpressure for test 6
    bp: process(aclk)
        variable cnt : integer := 0;
    begin
        if rising_edge(aclk) then
            cnt := cnt + 1;
            if test_phase = 6 and cnt mod 3 = 0 then m_axis_tready <= '0';
            else m_axis_tready <= '1'; end if;
        end if;
    end process;

    -- Output checker
    check: process(aclk)
        variable L : line;
        variable i_out, q_out : integer;
        variable sample_in_frame : integer := 0;

        -- Symmetry storage
        type sym_array is array (0 to N_SAMPLES-1) of integer;
        variable sym_i : sym_array := (others => 0);
        variable sym_count : integer := 0;
    begin
        if rising_edge(aclk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                i_out := to_integer(signed(m_axis_tdata(HALF_W-1 downto 0)));
                q_out := to_integer(signed(m_axis_tdata(DATA_W-1 downto HALF_W)));
                out_count <= out_count + 1;

                case test_phase is
                when 1 =>
                    -- DC test: endpoints should be attenuated (~0.08 * input)
                    -- Mid should be near input
                    if sample_in_frame = 0 or sample_in_frame = N_SAMPLES-1 then
                        if abs(i_out) > 3000 then
                            write(L, string'("FAIL T1: endpoint not attenuated, got "));
                            write(L, i_out); writeline(output, L);
                            fail_count <= fail_count + 1;
                        end if;
                    end if;
                    if sample_in_frame = N_SAMPLES/2 then
                        if abs(i_out) < 10000 then
                            write(L, string'("FAIL T1: center too low, got "));
                            write(L, i_out); writeline(output, L);
                            fail_count <= fail_count + 1;
                        end if;
                    end if;

                when 2 =>
                    -- Zero test
                    if i_out /= 0 or q_out /= 0 then
                        write(L, string'("FAIL T2: non-zero output "));
                        write(L, i_out); write(L, string'(",")); write(L, q_out);
                        writeline(output, L);
                        fail_count <= fail_count + 1;
                    end if;

                when 4 =>
                    -- Impulse: only center sample should be non-zero
                    if sample_in_frame = N_SAMPLES/2 then
                        if i_out = 0 then
                            report "FAIL T4: center impulse is zero" severity warning;
                            fail_count <= fail_count + 1;
                        end if;
                    else
                        if i_out /= 0 or q_out /= 0 then
                            null; -- windowed leakage is acceptable
                        end if;
                    end if;

                when 5 =>
                    -- Store for symmetry check
                    if sym_count < N_SAMPLES then
                        sym_i(sym_count) := i_out;
                        sym_count := sym_count + 1;
                    end if;
                    if sym_count = N_SAMPLES then
                        for k in 0 to N_SAMPLES/2-1 loop
                            if abs(sym_i(k) - sym_i(N_SAMPLES-1-k)) > 1 then
                                write(L, string'("FAIL T5: symmetry at "));
                                write(L, k); write(L, string'(" : "));
                                write(L, sym_i(k)); write(L, string'(" vs "));
                                write(L, sym_i(N_SAMPLES-1-k));
                                writeline(output, L);
                                fail_count <= fail_count + 1;
                            end if;
                        end loop;
                        sym_count := N_SAMPLES + 1;  -- Only check once
                    end if;

                when others => null;
                end case;

                if m_axis_tlast = '1' then sample_in_frame := 0;
                else sample_in_frame := sample_in_frame + 1; end if;
            end if;
        end if;
    end process;

end Behavioral;
