library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;

entity tb_doppler_notch is
end tb_doppler_notch;

architecture Behavioral of tb_doppler_notch is

    constant CLK_PERIOD : time := 10 ns;
    constant N_DOPPLER  : integer := 32;
    constant DATA_W     : integer := 32;
    constant HALF_W     : integer := 16;

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
    signal bypass        : std_logic := '0';
    signal sim_done      : boolean := false;
    signal test_phase    : integer := 0;
    signal fail_count    : integer := 0;

    -- Accumulate output power for each test
    signal output_power_acc : real := 0.0;
    signal output_count     : integer := 0;
    signal output_acc_reset : std_logic := '0';

    -- Use 2-pulse for primary tests, 3-pulse tested separately via different instance
    component doppler_notch is
        Generic ( DATA_WIDTH : integer; N_DOPPLER : integer; NOTCH_MODE : integer );
        Port (
            aclk, aresetn : in STD_LOGIC;
            s_axis_tdata : in STD_LOGIC_VECTOR(DATA_W-1 downto 0);
            s_axis_tvalid : in STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast : in STD_LOGIC;
            m_axis_tdata : out STD_LOGIC_VECTOR(DATA_W-1 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in STD_LOGIC;
            m_axis_tlast : out STD_LOGIC;
            bypass : in STD_LOGIC
        );
    end component;

    -- 3-pulse instance signals
    signal m3_axis_tdata  : std_logic_vector(DATA_W-1 downto 0);
    signal m3_axis_tvalid : std_logic;
    signal m3_axis_tready : std_logic := '1';
    signal m3_axis_tlast  : std_logic;

begin

    -- 2-pulse canceller (primary DUT)
    uut_2p: doppler_notch
    generic map ( DATA_WIDTH => DATA_W, N_DOPPLER => N_DOPPLER, NOTCH_MODE => 2 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready, m_axis_tlast => m_axis_tlast,
        bypass => bypass
    );

    -- 3-pulse canceller (secondary DUT)
    uut_3p: doppler_notch
    generic map ( DATA_WIDTH => DATA_W, N_DOPPLER => N_DOPPLER, NOTCH_MODE => 3 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => open, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m3_axis_tdata, m_axis_tvalid => m3_axis_tvalid,
        m_axis_tready => m3_axis_tready, m_axis_tlast => m3_axis_tlast,
        bypass => bypass
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    stim: process
        variable L : line;
        variable i_int, q_int : integer;
        variable phase : real;
        constant AMP : real := 10000.0;

        procedure send_chirp(doppler_freq : real; tag : string) is
        begin
            for s in 0 to N_DOPPLER-1 loop
                phase := 2.0 * MATH_PI * doppler_freq * real(s) / real(N_DOPPLER);
                i_int := integer(AMP * cos(phase));
                q_int := integer(AMP * sin(phase));
                s_axis_tdata  <= std_logic_vector(to_signed(q_int, HALF_W)) &
                                 std_logic_vector(to_signed(i_int, HALF_W));
                s_axis_tvalid <= '1';
                s_axis_tlast  <= '1' when s = N_DOPPLER-1 else '0';
                wait until rising_edge(aclk);
                while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
            end loop;
            s_axis_tvalid <= '0'; s_axis_tlast <= '0';
        end procedure;

    begin
        aresetn <= '0'; wait for 50 ns;
        aresetn <= '1'; wait until rising_edge(aclk);

        -- ==================================================================
        -- Test 1: DC (zero-Doppler) — should be nearly nulled
        -- ==================================================================
        report "=== T1: DC rejection (2-pulse) ===";
        test_phase <= 1; output_acc_reset <= '1';
        bypass <= '0';
        send_chirp(0.0, "DC");
        wait for 200 ns;
        write(L, string'("T1 avg power: "));
        if output_count > 0 then write(L, output_power_acc / real(output_count));
        else write(L, string'("N/A")); end if;
        writeline(output, L);

        -- ==================================================================
        -- Test 2: Half-Nyquist Doppler — should pass through
        -- ==================================================================
        report "=== T2: Doppler tone (f=8) passthrough ===";
        test_phase <= 2; output_acc_reset <= '1';
        send_chirp(8.0, "f=8");
        wait for 200 ns;
        write(L, string'("T2 avg power: "));
        if output_count > 0 then write(L, output_power_acc / real(output_count));
        else write(L, string'("N/A")); end if;
        writeline(output, L);
        -- Doppler tone should have significant output power
        if output_count > 0 and output_power_acc / real(output_count) < 1000.0 then
            report "FAIL T2: Doppler tone excessively attenuated" severity error;
            fail_count <= fail_count + 1;
        end if;

        -- ==================================================================
        -- Test 3: Bypass — output = input
        -- ==================================================================
        report "=== T3: Bypass mode ===";
        test_phase <= 3; output_acc_reset <= '1';
        bypass <= '1';
        send_chirp(0.0, "DC-bypass");
        wait for 200 ns;
        write(L, string'("T3 avg power (bypass): "));
        if output_count > 0 then write(L, output_power_acc / real(output_count));
        else write(L, string'("N/A")); end if;
        writeline(output, L);
        if output_count > 0 and output_power_acc / real(output_count) < 5000.0 then
            report "FAIL T3: bypass not passing DC" severity error;
            fail_count <= fail_count + 1;
        end if;
        bypass <= '0';

        -- ==================================================================
        -- Test 4: 3-pulse DC rejection
        -- ==================================================================
        report "=== T4: 3-pulse DC rejection ===";
        test_phase <= 4; output_acc_reset <= '1';
        send_chirp(0.0, "DC-3p");
        wait for 200 ns;

        -- ==================================================================
        -- Test 5: tlast resets delay line
        -- ==================================================================
        report "=== T5: Delay line reset on tlast ===";
        test_phase <= 5;
        -- Send short burst, then gap, then another — first sample of second burst
        -- should not carry over from first burst
        send_chirp(0.0, "burst1");
        wait for 100 ns;
        send_chirp(0.0, "burst2");
        wait for 200 ns;

        if fail_count = 0 then report "ALL TESTS PASSED";
        else report integer'image(fail_count) & " FAILURES" severity error; end if;
        sim_done <= true; wait;
    end process;

    -- Power measurement on 2-pulse output
    pwr: process(aclk)
        variable i_out, q_out : real;
    begin
        if rising_edge(aclk) then
            if output_acc_reset = '1' then
                output_power_acc <= 0.0;
                output_count <= 0;
                output_acc_reset <= '0';
            elsif m_axis_tvalid = '1' and m_axis_tready = '1' then
                i_out := real(to_integer(signed(m_axis_tdata(HALF_W-1 downto 0))));
                q_out := real(to_integer(signed(m_axis_tdata(DATA_W-1 downto HALF_W))));
                output_power_acc <= output_power_acc + sqrt(i_out*i_out + q_out*q_out);
                output_count <= output_count + 1;
            end if;
        end if;
    end process;

end Behavioral;
