library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;
use IEEE.std_logic_textio.all;

-- ADR_tb_tactical.vhd
-- Tactical Air Defense Scenario Testbench
-- Based on SAAB Giraffe AMB S-band radar
--
-- SCENARIO:
--   6x Su-27/30 fighters @ Mach 1, 25000ft, 45nm, fingertip formation
--       Loadout: R-77×4, R-73×2 (RCS ~12 m²)
--       Hot approach, notch maneuver at t=30s
--   4x Su-25 attackers @ Mach 0.65, 10000ft, 39nm, line abreast
--       Loadout: FAB-500×4, Kh-29×2 (RCS ~20 m²)
--       Steady approach
--
-- PRF Stagger: 8:9:10 ratio (8kHz, 9kHz, 10kHz)

entity ADR_tb_tactical is
end ADR_tb_tactical;

architecture Behavioral of ADR_tb_tactical is

    component ADR_radar_core is
        Generic (
            N_RANGE : integer := 1024;
            N_DOPPLER : integer := 128;
            MAX_TRACKS : integer := 32
        );
        Port (
            aclk, aresetn : in STD_LOGIC;
            s_axis_tdata : in STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid, s_axis_tlast : in STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            det_tdata : out STD_LOGIC_VECTOR(16 downto 0);
            det_tvalid : out STD_LOGIC;
            det_range_bin : out STD_LOGIC_VECTOR(9 downto 0);
            det_doppler_bin : out STD_LOGIC_VECTOR(6 downto 0);
            trk_valid : out STD_LOGIC;
            trk_id : out STD_LOGIC_VECTOR(5 downto 0);
            trk_range : out STD_LOGIC_VECTOR(11 downto 0);
            trk_doppler : out STD_LOGIC_VECTOR(8 downto 0);
            trk_vel_r : out STD_LOGIC_VECTOR(9 downto 0);
            trk_vel_d : out STD_LOGIC_VECTOR(7 downto 0);
            trk_quality : out STD_LOGIC_VECTOR(3 downto 0);
            trk_status : out STD_LOGIC_VECTOR(1 downto 0);
            mti_bypass : in STD_LOGIC;
            cfar_scale_ovr : in STD_LOGIC_VECTOR(2 downto 0);
            active_tracks : out STD_LOGIC_VECTOR(5 downto 0);
            frame_complete : out STD_LOGIC;
            scan_complete : out STD_LOGIC;
            status_overflow : out STD_LOGIC
        );
    end component;

    -- =========================================================================
    -- RADAR SYSTEM PARAMETERS (Giraffe AMB S-band)
    -- =========================================================================
    constant FREQ_HZ       : real := 3.0e9;           -- S-band 3 GHz
    constant WAVELENGTH_M  : real := 0.1;             -- ~10 cm
    constant C_MPS         : real := 299792458.0;     -- Speed of light
    
    -- PRF Stagger (8:9:10 ratio for ambiguity resolution)
    type prf_array_t is array (0 to 2) of real;
    constant PRF_HZ : prf_array_t := (8000.0, 9000.0, 10000.0);
    
    -- Range parameters
    constant MAX_RANGE_M   : real := 120000.0;        -- 120 km instrumented
    constant RANGE_RES_M   : real := 150.0;           -- ~150m resolution
    
    -- Velocity parameters  
    -- Unambiguous velocity = PRF * wavelength / 4
    -- At 8kHz: ±200 m/s, At 10kHz: ±250 m/s
    constant VEL_MAX_MPS   : real := 500.0;           -- Processing coverage
    
    -- Processing
    constant N_RANGE       : integer := 1024;
    constant N_DOPPLER     : integer := 128;
    constant CLK_PERIOD    : time := 10 ns;           -- 100 MHz
    
    -- Simulation time
    constant SIM_TIME_SEC  : real := 60.0;            -- 60 second scenario
    constant SCAN_RATE_HZ  : real := 2.0;             -- 2 scans per second
    constant SCANS_TOTAL   : integer := integer(SIM_TIME_SEC * SCAN_RATE_HZ);
    constant NOTCH_TIME_S  : real := 30.0;            -- Fighters notch at 30s
    
    -- =========================================================================
    -- TARGET PARAMETERS
    -- =========================================================================
    
    -- Physical constants
    constant NM_TO_M       : real := 1852.0;
    constant FT_TO_M       : real := 0.3048;
    constant MACH_MPS      : real := 340.29;          -- Mach 1 at altitude
    
    -- Fighter group (6x Su-27/30)
    constant N_FIGHTERS    : integer := 6;
    constant FIGHTER_RCS   : real := 12.0;            -- m² (with AAM load)
    constant FIGHTER_SPEED : real := 1.0 * MACH_MPS;  -- Mach 1
    constant FIGHTER_ALT   : real := 25000.0 * FT_TO_M;
    constant FIGHTER_RANGE : real := 45.0 * NM_TO_M;  -- 45 nm initial
    
    -- Fingertip formation offsets (meters)
    type vec3_t is record x, y, z : real; end record;
    type formation_t is array (0 to N_FIGHTERS-1) of vec3_t;
    constant FINGERTIP : formation_t := (
        (x => 0.0,    y => 0.0,    z => 0.0),     -- Lead
        (x => -50.0,  y => 30.0,   z => 5.0),     -- Left wing
        (x => -50.0,  y => -30.0,  z => -5.0),    -- Right wing  
        (x => -100.0, y => 60.0,   z => 10.0),    -- Left trail
        (x => -100.0, y => -60.0,  z => -10.0),   -- Right trail
        (x => -150.0, y => 0.0,    z => 0.0)      -- Slot
    );
    
    -- Attacker group (4x Su-25)
    constant N_ATTACKERS   : integer := 4;
    constant ATTACKER_RCS  : real := 20.0;            -- m² (with bomb load)
    constant ATTACKER_SPEED: real := 0.65 * MACH_MPS; -- Mach 0.65
    constant ATTACKER_ALT  : real := 10000.0 * FT_TO_M;
    constant ATTACKER_RANGE: real := 39.0 * NM_TO_M;  -- 6nm ahead of fighters
    
    -- Line abreast spacing (meters)
    constant LINE_SPACING  : real := 500.0;
    
    -- Noise/clutter
    constant THERMAL_NOISE : real := 50.0;
    constant SEA_CLUTTER   : real := 200.0;
    constant CLUTTER_RANGE : real := 20000.0;         -- Sea clutter extent
    
    -- =========================================================================
    -- Signals
    -- =========================================================================
    signal aclk          : std_logic := '0';
    signal aresetn       : std_logic := '0';
    signal s_axis_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tlast  : std_logic := '0';
    signal s_axis_tready : std_logic;
    
    signal det_tdata     : std_logic_vector(16 downto 0);
    signal det_tvalid    : std_logic;
    signal det_range_bin : std_logic_vector(9 downto 0);
    signal det_doppler_bin : std_logic_vector(6 downto 0);
    
    signal trk_valid     : std_logic;
    signal trk_id        : std_logic_vector(5 downto 0);
    signal trk_range     : std_logic_vector(11 downto 0);
    signal trk_doppler   : std_logic_vector(8 downto 0);
    signal trk_vel_r     : std_logic_vector(9 downto 0);
    signal trk_vel_d     : std_logic_vector(7 downto 0);
    signal trk_quality   : std_logic_vector(3 downto 0);
    signal trk_status    : std_logic_vector(1 downto 0);
    
    signal active_tracks : std_logic_vector(5 downto 0);
    signal frame_done    : std_logic;
    signal scan_done     : std_logic;
    signal overflow      : std_logic;
    
    signal sim_finished  : boolean := false;
    signal current_prf   : integer := 0;              -- PRF stagger index

    -- =========================================================================
    -- Target state
    -- =========================================================================
    type target_state_t is record
        active       : boolean;
        range_m      : real;
        azimuth_rad  : real;
        elevation_rad: real;
        vel_radial   : real;      -- Radial velocity (m/s, + = closing)
        vel_tangent  : real;      -- Tangential velocity
        rcs_m2       : real;
        is_notching  : boolean;
    end record;
    
    type target_array_t is array (natural range <>) of target_state_t;

begin

    -- DUT
    uut: ADR_radar_core
    generic map ( N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER, MAX_TRACKS => 32 )
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

    -- Clock
    aclk <= not aclk after CLK_PERIOD/2 when not sim_finished else '0';

    -- Main stimulus
    stim_proc: process
        variable seed1, seed2 : positive := 42;
        variable rand1, rand2 : real;
        variable sim_time_s   : real := 0.0;
        variable scan_period  : real;
        variable prf_idx      : integer := 0;
        variable prf_current  : real;
        
        -- Target state arrays (as variables)
        variable fighters  : target_array_t(0 to N_FIGHTERS-1);
        variable attackers : target_array_t(0 to N_ATTACKERS-1);
        
        -- Target calculations
        variable tgt_range_m  : real;
        variable tgt_vel_mps  : real;
        variable tgt_rcs      : real;
        variable range_bin    : integer;
        variable doppler_bin  : integer;
        variable amplitude    : real;
        variable phase        : real;
        variable i_acc, q_acc : real;
        variable i_int, q_int : integer;
        
        -- Clutter
        variable clutter_amp  : real;
        variable clutter_dopp : real;
        
        -- Gaussian noise
        procedure gauss(variable s1, s2: inout positive; variable g1, g2: out real) is
            variable u1, u2: real;
        begin
            uniform(s1, s2, u1); uniform(s1, s2, u2);
            if u1 < 1.0e-10 then u1 := 1.0e-10; end if;
            g1 := sqrt(-2.0*log(u1)) * cos(2.0*MATH_PI*u2);
            g2 := sqrt(-2.0*log(u1)) * sin(2.0*MATH_PI*u2);
        end procedure;
        
        -- RCS to amplitude (simplified radar equation)
        function rcs_to_amp(rcs_m2, range_m: real) return real is
            variable r4 : real;
        begin
            if range_m < 1000.0 then return 30000.0; end if;
            r4 := (range_m / 10000.0) ** 4;
            return sqrt(rcs_m2) * 20000.0 / sqrt(r4);
        end function;
        
        -- Velocity to Doppler bin
        function vel_to_doppler(vel_mps, prf: real) return integer is
            variable dopp_hz : real;
            variable bin     : integer;
        begin
            -- fd = 2*v*f/c = 2*v/lambda
            dopp_hz := 2.0 * vel_mps / WAVELENGTH_M;
            -- Normalize to bins (centered at N_DOPPLER/2)
            bin := integer((dopp_hz / prf) * real(N_DOPPLER)) + N_DOPPLER/2;
            if bin < 0 then bin := bin + N_DOPPLER; end if;
            if bin >= N_DOPPLER then bin := bin - N_DOPPLER; end if;
            return bin;
        end function;
        
        -- Range to bin
        function range_to_bin(range_m: real) return integer is
        begin
            return integer((range_m / MAX_RANGE_M) * real(N_RANGE));
        end function;
        
    begin
        aresetn <= '0';
        wait for 100 ns;
        aresetn <= '1';
        wait until s_axis_tready = '1';
        
        -- Initialize targets
        for i in 0 to N_FIGHTERS-1 loop
            fighters(i).active := true;
            fighters(i).range_m := FIGHTER_RANGE + FINGERTIP(i).x;
            fighters(i).azimuth_rad := FINGERTIP(i).y / FIGHTER_RANGE;
            fighters(i).elevation_rad := (FIGHTER_ALT + FINGERTIP(i).z) / FIGHTER_RANGE;
            fighters(i).vel_radial := -FIGHTER_SPEED;  -- Closing (hot)
            fighters(i).vel_tangent := 0.0;
            fighters(i).rcs_m2 := FIGHTER_RCS;
            fighters(i).is_notching := false;
        end loop;
        
        for i in 0 to N_ATTACKERS-1 loop
            attackers(i).active := true;
            attackers(i).range_m := ATTACKER_RANGE;
            attackers(i).azimuth_rad := (real(i) - 1.5) * LINE_SPACING / ATTACKER_RANGE;
            attackers(i).elevation_rad := ATTACKER_ALT / ATTACKER_RANGE;
            attackers(i).vel_radial := -ATTACKER_SPEED;
            attackers(i).vel_tangent := 0.0;
            attackers(i).rcs_m2 := ATTACKER_RCS;
            attackers(i).is_notching := false;
        end loop;

        report "=== TACTICAL SCENARIO START ===";
        report "Fighters: 6x Su-27/30, M1.0, 45nm, hot approach";
        report "Attackers: 4x Su-25, M0.65, 39nm, steady";
        report "Notch maneuver at t=30s";
        
        -- =====================================================================
        -- SCAN LOOP
        -- =====================================================================
        for scan in 1 to SCANS_TOTAL loop
            
            sim_time_s := real(scan - 1) / SCAN_RATE_HZ;
            scan_period := 1.0 / SCAN_RATE_HZ;
            
            -- PRF stagger (rotate through 8:9:10)
            prf_idx := (scan - 1) mod 3;
            prf_current := PRF_HZ(prf_idx);
            current_prf <= prf_idx;
            
            -- ================================================================
            -- UPDATE TARGET KINEMATICS
            -- ================================================================
            
            -- Check for notch maneuver
            if sim_time_s >= NOTCH_TIME_S and sim_time_s < NOTCH_TIME_S + 10.0 then
                -- Fighters execute beam turn (notch)
                for i in 0 to N_FIGHTERS-1 loop
                    if not fighters(i).is_notching then
                        report "t=" & integer'image(integer(sim_time_s)) & 
                               "s: Fighter " & integer'image(i) & " NOTCHING";
                        fighters(i).is_notching := true;
                    end if;
                    -- During notch: radial velocity → 0, tangent velocity = full speed
                    fighters(i).vel_radial := 0.0;
                    fighters(i).vel_tangent := FIGHTER_SPEED;
                end loop;
            elsif sim_time_s >= NOTCH_TIME_S + 10.0 then
                -- Resume hot after notch
                for i in 0 to N_FIGHTERS-1 loop
                    if fighters(i).is_notching then
                        report "t=" & integer'image(integer(sim_time_s)) & 
                               "s: Fighter " & integer'image(i) & " RESUMING HOT";
                        fighters(i).is_notching := false;
                    end if;
                    fighters(i).vel_radial := -FIGHTER_SPEED;
                    fighters(i).vel_tangent := 0.0;
                end loop;
            end if;
            
            -- Update fighter positions
            for i in 0 to N_FIGHTERS-1 loop
                fighters(i).range_m := fighters(i).range_m + 
                                       fighters(i).vel_radial * scan_period;
                -- Clamp range
                if fighters(i).range_m < 5000.0 then 
                    fighters(i).range_m := 5000.0;
                    fighters(i).active := false;
                end if;
            end loop;
            
            -- Update attacker positions
            for i in 0 to N_ATTACKERS-1 loop
                attackers(i).range_m := attackers(i).range_m + 
                                        attackers(i).vel_radial * scan_period;
                if attackers(i).range_m < 5000.0 then
                    attackers(i).range_m := 5000.0;
                    attackers(i).active := false;
                end if;
            end loop;
            
            -- Status report every 10 scans
            if scan mod 10 = 1 then
                report "--- Scan " & integer'image(scan) & 
                       " t=" & integer'image(integer(sim_time_s)) & 
                       "s PRF=" & integer'image(integer(prf_current)) & "Hz ---";
                report "  Fighter lead range: " & 
                       integer'image(integer(fighters(0).range_m/1000.0)) & " km";
                report "  Attacker lead range: " & 
                       integer'image(integer(attackers(0).range_m/1000.0)) & " km";
            end if;
            
            -- ================================================================
            -- GENERATE RADAR RETURNS
            -- ================================================================
            for c in 0 to N_DOPPLER-1 loop
                for s in 0 to N_RANGE-1 loop
                    
                    i_acc := 0.0;
                    q_acc := 0.0;
                    
                    -- Fighter returns
                    for t in 0 to N_FIGHTERS-1 loop
                        if fighters(t).active then
                            range_bin := range_to_bin(fighters(t).range_m);
                            doppler_bin := vel_to_doppler(fighters(t).vel_radial, prf_current);
                            
                            -- Check if this sample contains target
                            if abs(s - range_bin) < 3 then
                                amplitude := rcs_to_amp(fighters(t).rcs_m2, fighters(t).range_m);
                                
                                -- Add range sidelobe spread
                                if s /= range_bin then
                                    amplitude := amplitude * 0.3 / real(abs(s - range_bin));
                                end if;
                                
                                phase := 2.0 * MATH_PI * (
                                    (real(range_bin) * real(s) / real(N_RANGE)) +
                                    (real(doppler_bin) * real(c) / real(N_DOPPLER))
                                );
                                
                                i_acc := i_acc + amplitude * cos(phase);
                                q_acc := q_acc + amplitude * sin(phase);
                            end if;
                        end if;
                    end loop;
                    
                    -- Attacker returns
                    for t in 0 to N_ATTACKERS-1 loop
                        if attackers(t).active then
                            range_bin := range_to_bin(attackers(t).range_m);
                            doppler_bin := vel_to_doppler(attackers(t).vel_radial, prf_current);
                            
                            if abs(s - range_bin) < 3 then
                                amplitude := rcs_to_amp(attackers(t).rcs_m2, attackers(t).range_m);
                                
                                if s /= range_bin then
                                    amplitude := amplitude * 0.3 / real(abs(s - range_bin));
                                end if;
                                
                                phase := 2.0 * MATH_PI * (
                                    (real(range_bin) * real(s) / real(N_RANGE)) +
                                    (real(doppler_bin) * real(c) / real(N_DOPPLER))
                                );
                                
                                i_acc := i_acc + amplitude * cos(phase);
                                q_acc := q_acc + amplitude * sin(phase);
                            end if;
                        end if;
                    end loop;
                    
                    -- Sea clutter (Baltic Sea, low sea state)
                    if real(s) * RANGE_RES_M < CLUTTER_RANGE then
                        uniform(seed1, seed2, rand1);
                        clutter_amp := SEA_CLUTTER * (1.0 - real(s)/real(N_RANGE)) * rand1;
                        
                        -- Near-zero Doppler
                        uniform(seed1, seed2, rand1);
                        clutter_dopp := (rand1 - 0.5) * 4.0;  -- ±2 bins
                        
                        phase := 2.0 * MATH_PI * (
                            (real(s) * real(s) / real(N_RANGE*10)) +
                            (clutter_dopp * real(c) / real(N_DOPPLER))
                        );
                        
                        i_acc := i_acc + clutter_amp * cos(phase);
                        q_acc := q_acc + clutter_amp * sin(phase);
                    end if;
                    
                    -- Thermal noise
                    gauss(seed1, seed2, rand1, rand2);
                    i_acc := i_acc + rand1 * THERMAL_NOISE;
                    q_acc := q_acc + rand2 * THERMAL_NOISE;
                    
                    -- Quantize
                    if i_acc > 32000.0 then i_acc := 32000.0; end if;
                    if i_acc < -32000.0 then i_acc := -32000.0; end if;
                    if q_acc > 32000.0 then q_acc := 32000.0; end if;
                    if q_acc < -32000.0 then q_acc := -32000.0; end if;
                    
                    i_int := integer(i_acc);
                    q_int := integer(q_acc);
                    
                    s_axis_tdata <= std_logic_vector(to_signed(q_int, 16)) &
                                    std_logic_vector(to_signed(i_int, 16));
                    s_axis_tvalid <= '1';
                    s_axis_tlast <= '1' when s = N_RANGE-1 else '0';
                    
                    wait until rising_edge(aclk);
                    while s_axis_tready = '0' loop
                        wait until rising_edge(aclk);
                    end loop;
                    
                end loop;
            end loop;
            
            s_axis_tvalid <= '0';
            s_axis_tlast <= '0';
            
            -- Wait for processing
            wait until frame_done = '1';
            wait for 1 us;
            
        end loop;

        report "=== SCENARIO COMPLETE ===";
        wait for 100 us;
        sim_finished <= true;
        wait;
    end process;

    -- =========================================================================
    -- Detection Logger
    -- =========================================================================
    det_log: process(aclk)
        file detfile : text open write_mode is "ADR_detections.txt";
        variable outline : line;
    begin
        if rising_edge(aclk) then
            if det_tvalid = '1' and unsigned(det_tdata) > 0 then
                write(outline, to_integer(unsigned(det_range_bin)));
                write(outline, string'(" "));
                write(outline, to_integer(unsigned(det_doppler_bin)));
                write(outline, string'(" "));
                write(outline, to_integer(unsigned(det_tdata)));
                writeline(detfile, outline);
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Track Logger
    -- =========================================================================
    trk_log: process(aclk)
        file trkfile : text open write_mode is "ADR_tracks.txt";
        variable outline : line;
    begin
        if rising_edge(aclk) then
            if trk_valid = '1' then
                write(outline, string'("TRK "));
                write(outline, to_integer(unsigned(trk_id)));
                write(outline, string'(" R="));
                write(outline, to_integer(signed(trk_range)));
                write(outline, string'(" D="));
                write(outline, to_integer(signed(trk_doppler)));
                write(outline, string'(" VR="));
                write(outline, to_integer(signed(trk_vel_r)));
                write(outline, string'(" Q="));
                write(outline, to_integer(unsigned(trk_quality)));
                write(outline, string'(" S="));
                write(outline, trk_status);
                writeline(trkfile, outline);
            end if;
            
            if scan_done = '1' then
                write(outline, string'("SCAN_END ACTIVE="));
                write(outline, to_integer(unsigned(active_tracks)));
                writeline(trkfile, outline);
            end if;
        end if;
    end process;

end Behavioral;