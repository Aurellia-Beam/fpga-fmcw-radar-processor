library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;

-- Tactical Air Defense Scenario Testbench
-- Requires Xilinx FFT IP in project.
--
-- SCENARIO:
--   Fighters: N_FIGHTERS × Su-27/30, Mach 1, hot approach with notch maneuver
--   Attackers: N_ATTACKERS × Su-25, Mach 0.65, steady approach
--   Sea clutter + thermal noise
--   PRF stagger for ambiguity resolution
--
-- CONFIGURATION:
--   Set QUICK_MODE = true for reduced resolution (~30 min sim)
--   Set QUICK_MODE = false for full resolution (hours)

entity tb_tactical is
end tb_tactical;

architecture Behavioral of tb_tactical is

    -- =========================================================================
    -- Configuration (toggle for quick validation vs full run)
    -- =========================================================================
    constant QUICK_MODE : boolean := false;

    -- Derived parameters
    constant N_RANGE     : integer := 1024 when not QUICK_MODE else 128;
    constant N_DOPPLER   : integer := 128  when not QUICK_MODE else 32;
    constant MAX_TRACKS  : integer := 32   when not QUICK_MODE else 16;
    constant N_FIGHTERS  : integer := 6    when not QUICK_MODE else 2;
    constant N_ATTACKERS : integer := 4    when not QUICK_MODE else 1;
    constant NUM_SCANS   : integer := 120  when not QUICK_MODE else 5;
    constant CFAR_REF_R  : integer := 4    when not QUICK_MODE else 2;
    constant CFAR_REF_D  : integer := 4    when not QUICK_MODE else 2;
    constant CFAR_GRD_R  : integer := 2    when not QUICK_MODE else 1;
    constant CFAR_GRD_D  : integer := 1;

    constant CLK_PERIOD  : time := 10 ns;

    -- =========================================================================
    -- Physics
    -- =========================================================================
    constant WAVELENGTH  : real := 0.1;            -- S-band ~10 cm
    constant MAX_RANGE_M : real := 120000.0;
    constant MACH_MPS    : real := 340.29;
    constant NM_TO_M     : real := 1852.0;
    constant FT_TO_M     : real := 0.3048;
    constant SCAN_RATE   : real := 2.0;            -- scans/s
    constant NOTCH_SCAN  : integer := NUM_SCANS / 2;  -- Notch at midpoint

    type prf_array_t is array (0 to 2) of real;
    constant PRF_HZ : prf_array_t := (8000.0, 9000.0, 10000.0);

    -- Clutter/noise
    constant THERMAL_NOISE : real := 50.0;
    constant SEA_CLUTTER   : real := 200.0;
    constant CLUTTER_RNG   : real := 20000.0;
    constant RANGE_RES     : real := 150.0;

    -- =========================================================================
    -- Signals
    -- =========================================================================
    signal aclk, aresetn  : std_logic := '0';
    signal s_axis_tdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axis_tvalid, s_axis_tlast : std_logic := '0';
    signal s_axis_tready  : std_logic;

    signal det_tdata      : std_logic_vector(16 downto 0);
    signal det_tvalid     : std_logic;
    signal det_range_bin  : std_logic_vector(9 downto 0);
    signal det_doppler_bin: std_logic_vector(6 downto 0);

    signal trk_valid      : std_logic;
    signal trk_id         : std_logic_vector(5 downto 0);
    signal trk_range      : std_logic_vector(11 downto 0);
    signal trk_doppler    : std_logic_vector(8 downto 0);
    signal trk_vel_r      : std_logic_vector(9 downto 0);
    signal trk_vel_d      : std_logic_vector(7 downto 0);
    signal trk_quality    : std_logic_vector(3 downto 0);
    signal trk_status     : std_logic_vector(1 downto 0);

    signal active_tracks  : std_logic_vector(5 downto 0);
    signal frame_done, scan_done, overflow : std_logic;
    signal sim_finished   : boolean := false;

    -- =========================================================================
    -- Target state
    -- =========================================================================
    type target_state_t is record
        active      : boolean;
        range_m     : real;
        vel_radial  : real;
        rcs_m2      : real;
        is_notching : boolean;
    end record;
    type target_array_t is array (natural range <>) of target_state_t;

begin

    uut: entity work.radar_core
    generic map (
        N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER, MAX_TRACKS => MAX_TRACKS,
        CFAR_REF_R => CFAR_REF_R, CFAR_REF_D => CFAR_REF_D,
        CFAR_GUARD_R => CFAR_GRD_R, CFAR_GUARD_D => CFAR_GRD_D
    )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tlast => s_axis_tlast, s_axis_tready => s_axis_tready,
        det_tdata => det_tdata, det_tvalid => det_tvalid,
        det_range_bin => det_range_bin, det_doppler_bin => det_doppler_bin,
        trk_valid => trk_valid, trk_id => trk_id, trk_range => trk_range,
        trk_doppler => trk_doppler, trk_vel_r => trk_vel_r, trk_vel_d => trk_vel_d,
        trk_quality => trk_quality, trk_status => trk_status,
        mti_bypass => '0', cfar_scale_ovr => "000",
        active_tracks => active_tracks, frame_complete => frame_done,
        scan_complete => scan_done, status_overflow => overflow
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_finished else '0';

    -- =========================================================================
    -- Main stimulus
    -- =========================================================================
    stim: process
        variable seed1, seed2 : positive := 42;
        variable rand1, rand2 : real;
        variable prf_current  : real;

        variable fighters  : target_array_t(0 to N_FIGHTERS-1);
        variable attackers : target_array_t(0 to N_ATTACKERS-1);

        variable range_bin, doppler_bin : integer;
        variable amplitude, phase : real;
        variable i_acc, q_acc : real;
        variable i_int, q_int : integer;
        variable clutter_amp : real;
        variable scan_period : real := 1.0 / SCAN_RATE;
        variable sim_time    : real;

        -- Fingertip formation offsets (range only, meters)
        type offset_t is array (0 to 5) of real;
        constant FTR_OFFSET : offset_t := (0.0, -50.0, -50.0, -100.0, -100.0, -150.0);

        procedure gauss(variable s1, s2: inout positive; variable g1, g2: out real) is
            variable u1, u2: real;
        begin
            uniform(s1, s2, u1); uniform(s1, s2, u2);
            if u1 < 1.0e-10 then u1 := 1.0e-10; end if;
            g1 := sqrt(-2.0*log(u1)) * cos(2.0*MATH_PI*u2);
            g2 := sqrt(-2.0*log(u1)) * sin(2.0*MATH_PI*u2);
        end procedure;

        function rcs_to_amp(rcs, rng: real) return real is
        begin
            if rng < 1000.0 then return 30000.0; end if;
            return sqrt(rcs) * 20000.0 / sqrt((rng/10000.0)**4);
        end function;

        function vel_to_doppler_bin(vel, prf: real) return integer is
            variable bin : integer;
        begin
            bin := integer((2.0 * vel / WAVELENGTH / prf) * real(N_DOPPLER)) + N_DOPPLER/2;
            if bin < 0 then bin := bin + N_DOPPLER; end if;
            if bin >= N_DOPPLER then bin := bin - N_DOPPLER; end if;
            return bin;
        end function;

        function range_to_bin(rng: real) return integer is
        begin
            return integer((rng / MAX_RANGE_M) * real(N_RANGE));
        end function;

    begin
        aresetn <= '0'; wait for 100 ns;
        aresetn <= '1';
        wait until s_axis_tready = '1';

        -- Initialize fighters
        for i in 0 to N_FIGHTERS-1 loop
            fighters(i).active := true;
            fighters(i).range_m := 45.0 * NM_TO_M + FTR_OFFSET(i mod 6);
            fighters(i).vel_radial := -1.0 * MACH_MPS;
            fighters(i).rcs_m2 := 12.0;
            fighters(i).is_notching := false;
        end loop;

        -- Initialize attackers
        for i in 0 to N_ATTACKERS-1 loop
            attackers(i).active := true;
            attackers(i).range_m := 39.0 * NM_TO_M;
            attackers(i).vel_radial := -0.65 * MACH_MPS;
            attackers(i).rcs_m2 := 20.0;
            attackers(i).is_notching := false;
        end loop;

        report "=== TACTICAL SCENARIO START (" &
               integer'image(N_RANGE) & "×" & integer'image(N_DOPPLER) &
               ", " & integer'image(NUM_SCANS) & " scans) ===";

        -- =================================================================
        -- Scan loop
        -- =================================================================
        for scan in 1 to NUM_SCANS loop

            sim_time := real(scan - 1) / SCAN_RATE;
            prf_current := PRF_HZ((scan - 1) mod 3);

            -- Notch maneuver
            if scan = NOTCH_SCAN then
                report ">>> FIGHTERS NOTCHING (scan " & integer'image(scan) & ") <<<";
                for i in 0 to N_FIGHTERS-1 loop
                    fighters(i).vel_radial := 0.0;
                    fighters(i).is_notching := true;
                end loop;
            elsif scan = NOTCH_SCAN + 3 then
                report ">>> FIGHTERS RESUME HOT <<<";
                for i in 0 to N_FIGHTERS-1 loop
                    fighters(i).vel_radial := -1.0 * MACH_MPS;
                    fighters(i).is_notching := false;
                end loop;
            end if;

            -- Update kinematics
            for i in 0 to N_FIGHTERS-1 loop
                fighters(i).range_m := fighters(i).range_m + fighters(i).vel_radial * scan_period;
                if fighters(i).range_m < 5000.0 then fighters(i).active := false; end if;
            end loop;
            for i in 0 to N_ATTACKERS-1 loop
                attackers(i).range_m := attackers(i).range_m + attackers(i).vel_radial * scan_period;
                if attackers(i).range_m < 5000.0 then attackers(i).active := false; end if;
            end loop;

            -- Status report
            if scan mod 10 = 1 or QUICK_MODE then
                report "Scan " & integer'image(scan) &
                       " t=" & integer'image(integer(sim_time)) & "s" &
                       " F1_R=" & integer'image(integer(fighters(0).range_m/1000.0)) & "km" &
                       " A1_R=" & integer'image(integer(attackers(0).range_m/1000.0)) & "km";
            end if;

            -- Generate radar returns
            for c in 0 to N_DOPPLER-1 loop
                for s in 0 to N_RANGE-1 loop
                    i_acc := 0.0; q_acc := 0.0;

                    -- Fighter returns
                    for t in 0 to N_FIGHTERS-1 loop
                        if fighters(t).active then
                            range_bin := range_to_bin(fighters(t).range_m);
                            doppler_bin := vel_to_doppler_bin(fighters(t).vel_radial, prf_current);
                            if abs(s - range_bin) < 3 then
                                amplitude := rcs_to_amp(fighters(t).rcs_m2, fighters(t).range_m);
                                if s /= range_bin then
                                    amplitude := amplitude * 0.3 / real(abs(s - range_bin));
                                end if;
                                phase := 2.0*MATH_PI*(
                                    real(range_bin)*real(s)/real(N_RANGE) +
                                    real(doppler_bin)*real(c)/real(N_DOPPLER));
                                i_acc := i_acc + amplitude * cos(phase);
                                q_acc := q_acc + amplitude * sin(phase);
                            end if;
                        end if;
                    end loop;

                    -- Attacker returns
                    for t in 0 to N_ATTACKERS-1 loop
                        if attackers(t).active then
                            range_bin := range_to_bin(attackers(t).range_m);
                            doppler_bin := vel_to_doppler_bin(attackers(t).vel_radial, prf_current);
                            if abs(s - range_bin) < 3 then
                                amplitude := rcs_to_amp(attackers(t).rcs_m2, attackers(t).range_m);
                                if s /= range_bin then
                                    amplitude := amplitude * 0.3 / real(abs(s - range_bin));
                                end if;
                                phase := 2.0*MATH_PI*(
                                    real(range_bin)*real(s)/real(N_RANGE) +
                                    real(doppler_bin)*real(c)/real(N_DOPPLER));
                                i_acc := i_acc + amplitude * cos(phase);
                                q_acc := q_acc + amplitude * sin(phase);
                            end if;
                        end if;
                    end loop;

                    -- Sea clutter
                    if real(s) * RANGE_RES < CLUTTER_RNG then
                        uniform(seed1, seed2, rand1);
                        clutter_amp := SEA_CLUTTER * (1.0 - real(s)/real(N_RANGE)) * rand1;
                        uniform(seed1, seed2, rand1);
                        phase := 2.0*MATH_PI*(real(s)*real(s)/real(N_RANGE*10) +
                                 (rand1-0.5)*4.0*real(c)/real(N_DOPPLER));
                        i_acc := i_acc + clutter_amp * cos(phase);
                        q_acc := q_acc + clutter_amp * sin(phase);
                    end if;

                    -- Thermal noise
                    gauss(seed1, seed2, rand1, rand2);
                    i_acc := i_acc + rand1 * THERMAL_NOISE;
                    q_acc := q_acc + rand2 * THERMAL_NOISE;

                    -- Quantize
                    if i_acc >  32000.0 then i_acc :=  32000.0; end if;
                    if i_acc < -32000.0 then i_acc := -32000.0; end if;
                    if q_acc >  32000.0 then q_acc :=  32000.0; end if;
                    if q_acc < -32000.0 then q_acc := -32000.0; end if;

                    s_axis_tdata  <= std_logic_vector(to_signed(integer(q_acc), 16)) &
                                     std_logic_vector(to_signed(integer(i_acc), 16));
                    s_axis_tvalid <= '1';
                    s_axis_tlast  <= '1' when s = N_RANGE-1 else '0';

                    wait until rising_edge(aclk);
                    while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
                end loop;
            end loop;

            s_axis_tvalid <= '0'; s_axis_tlast <= '0';
            wait until frame_done = '1'; wait for 1 us;
        end loop;

        report "=== SCENARIO COMPLETE ===";
        wait for 100 us;
        sim_finished <= true;
        wait;
    end process;

    -- Detection logger
    det_log: process(aclk)
        file f : text open write_mode is "tac_detections.txt";
        variable L : line;
    begin
        if rising_edge(aclk) and det_tvalid = '1' and unsigned(det_tdata) > 0 then
            write(L, to_integer(unsigned(det_range_bin))); write(L, string'(" "));
            write(L, to_integer(unsigned(det_doppler_bin))); write(L, string'(" "));
            write(L, to_integer(unsigned(det_tdata)));
            writeline(f, L);
        end if;
    end process;

    -- Track logger
    trk_log: process(aclk)
        file f : text open write_mode is "tac_tracks.txt";
        variable L : line;
    begin
        if rising_edge(aclk) then
            if trk_valid = '1' then
                write(L, string'("TRK ")); write(L, to_integer(unsigned(trk_id)));
                write(L, string'(" R=")); write(L, to_integer(signed(trk_range)));
                write(L, string'(" D=")); write(L, to_integer(signed(trk_doppler)));
                write(L, string'(" VR=")); write(L, to_integer(signed(trk_vel_r)));
                write(L, string'(" Q=")); write(L, to_integer(unsigned(trk_quality)));
                write(L, string'(" S=")); write(L, trk_status);
                writeline(f, L);
            end if;
            if scan_done = '1' then
                write(L, string'("SCAN_END ACTIVE="));
                write(L, to_integer(unsigned(active_tracks)));
                writeline(f, L);
            end if;
        end if;
    end process;

end Behavioral;
