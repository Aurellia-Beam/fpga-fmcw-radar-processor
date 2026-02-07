library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;
use IEEE.std_logic_textio.all;

-- ADR_tb_quick.vhd
-- Quick validation testbench (~30 min sim time)
-- Reduced resolution, 5 scans, notch at scan 3

entity ADR_tb_quick is
end ADR_tb_quick;

architecture Behavioral of ADR_tb_quick is

    component ADR_radar_core is
        Generic (
            N_RANGE : integer := 128;
            N_DOPPLER : integer := 32;
            MAX_TRACKS : integer := 16;
            CFAR_REF_R : integer := 2;
            CFAR_REF_D : integer := 2;
            CFAR_GUARD_R : integer := 1;
            CFAR_GUARD_D : integer := 1
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

    -- REDUCED PARAMETERS
    constant N_RANGE    : integer := 128;
    constant N_DOPPLER  : integer := 32;
    constant CLK_PERIOD : time := 10 ns;
    constant NUM_SCANS  : integer := 5;
    constant NOTCH_SCAN : integer := 3;  -- Notch on scan 3
    
    -- Simplified target config
    constant N_FIGHTERS  : integer := 2;   -- Just 2 fighters
    constant N_ATTACKERS : integer := 1;   -- Just 1 attacker
    
    -- Physics (scaled)
    constant MAX_RANGE_M : real := 120000.0;
    constant WAVELENGTH  : real := 0.1;
    constant PRF_HZ      : real := 10000.0;
    constant MACH_MPS    : real := 340.0;
    
    -- Signals
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

    -- Target record
    type target_t is record
        range_m    : real;
        vel_mps    : real;
        rcs        : real;
        is_notch   : boolean;
    end record;

begin

    uut: ADR_radar_core
    generic map (
        N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER, MAX_TRACKS => 16,
        CFAR_REF_R => 2, CFAR_REF_D => 2, CFAR_GUARD_R => 1, CFAR_GUARD_D => 1
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

    stim: process
        variable seed1, seed2 : positive := 42;
        variable rand1, rand2 : real;
        
        -- Targets
        variable ftr1, ftr2 : target_t;
        variable atk1 : target_t;
        
        variable i_acc, q_acc : real;
        variable i_int, q_int : integer;
        variable phase : real;
        variable amplitude : real;
        variable range_bin, doppler_bin : integer;
        
        -- Helpers
        function range_to_bin(r: real) return integer is
        begin
            return integer((r / MAX_RANGE_M) * real(N_RANGE));
        end function;
        
        function vel_to_doppler(v: real) return integer is
            variable fd, bin : real;
        begin
            fd := 2.0 * v / WAVELENGTH;
            bin := (fd / PRF_HZ) * real(N_DOPPLER) + real(N_DOPPLER)/2.0;
            if bin < 0.0 then bin := bin + real(N_DOPPLER); end if;
            if bin >= real(N_DOPPLER) then bin := bin - real(N_DOPPLER); end if;
            return integer(bin);
        end function;
        
        function rcs_to_amp(rcs, rng: real) return real is
        begin
            if rng < 1000.0 then return 25000.0; end if;
            return sqrt(rcs) * 15000.0 / sqrt((rng/10000.0)**4);
        end function;
        
        procedure gauss(variable s1, s2: inout positive; variable g1, g2: out real) is
            variable u1, u2: real;
        begin
            uniform(s1, s2, u1); uniform(s1, s2, u2);
            if u1 < 1.0e-10 then u1 := 1.0e-10; end if;
            g1 := sqrt(-2.0*log(u1)) * cos(2.0*MATH_PI*u2);
            g2 := sqrt(-2.0*log(u1)) * sin(2.0*MATH_PI*u2);
        end procedure;
        
    begin
        aresetn <= '0';
        wait for 100 ns;
        aresetn <= '1';
        wait until s_axis_tready = '1';
        
        -- Init targets
        ftr1 := (range_m => 80000.0, vel_mps => -340.0, rcs => 12.0, is_notch => false);
        ftr2 := (range_m => 82000.0, vel_mps => -340.0, rcs => 12.0, is_notch => false);
        atk1 := (range_m => 70000.0, vel_mps => -220.0, rcs => 20.0, is_notch => false);
        
        report "=== QUICK VALIDATION START ===";
        report "5 scans, notch on scan 3";
        
        for scan in 1 to NUM_SCANS loop
            report "--- Scan " & integer'image(scan) & " ---";
            
            -- Update kinematics
            ftr1.range_m := ftr1.range_m + ftr1.vel_mps * 0.5;
            ftr2.range_m := ftr2.range_m + ftr2.vel_mps * 0.5;
            atk1.range_m := atk1.range_m + atk1.vel_mps * 0.5;
            
            -- Notch maneuver
            if scan = NOTCH_SCAN then
                report ">>> FIGHTERS NOTCHING <<<";
                ftr1.vel_mps := 0.0;
                ftr2.vel_mps := 0.0;
            elsif scan = NOTCH_SCAN + 1 then
                report ">>> FIGHTERS RESUME HOT <<<";
                ftr1.vel_mps := -340.0;
                ftr2.vel_mps := -340.0;
            end if;
            
            report "  F1: R=" & integer'image(integer(ftr1.range_m/1000.0)) & "km V=" & integer'image(integer(ftr1.vel_mps)) & "m/s";
            report "  F2: R=" & integer'image(integer(ftr2.range_m/1000.0)) & "km V=" & integer'image(integer(ftr2.vel_mps)) & "m/s";
            report "  A1: R=" & integer'image(integer(atk1.range_m/1000.0)) & "km V=" & integer'image(integer(atk1.vel_mps)) & "m/s";
            
            -- Generate data
            for c in 0 to N_DOPPLER-1 loop
                for s in 0 to N_RANGE-1 loop
                    i_acc := 0.0;
                    q_acc := 0.0;
                    
                    -- Fighter 1
                    range_bin := range_to_bin(ftr1.range_m);
                    doppler_bin := vel_to_doppler(ftr1.vel_mps);
                    if abs(s - range_bin) < 2 then
                        amplitude := rcs_to_amp(ftr1.rcs, ftr1.range_m);
                        phase := 2.0*MATH_PI*((real(range_bin)*real(s)/real(N_RANGE)) + (real(doppler_bin)*real(c)/real(N_DOPPLER)));
                        i_acc := i_acc + amplitude * cos(phase);
                        q_acc := q_acc + amplitude * sin(phase);
                    end if;
                    
                    -- Fighter 2
                    range_bin := range_to_bin(ftr2.range_m);
                    doppler_bin := vel_to_doppler(ftr2.vel_mps);
                    if abs(s - range_bin) < 2 then
                        amplitude := rcs_to_amp(ftr2.rcs, ftr2.range_m);
                        phase := 2.0*MATH_PI*((real(range_bin)*real(s)/real(N_RANGE)) + (real(doppler_bin)*real(c)/real(N_DOPPLER)));
                        i_acc := i_acc + amplitude * cos(phase);
                        q_acc := q_acc + amplitude * sin(phase);
                    end if;
                    
                    -- Attacker
                    range_bin := range_to_bin(atk1.range_m);
                    doppler_bin := vel_to_doppler(atk1.vel_mps);
                    if abs(s - range_bin) < 2 then
                        amplitude := rcs_to_amp(atk1.rcs, atk1.range_m);
                        phase := 2.0*MATH_PI*((real(range_bin)*real(s)/real(N_RANGE)) + (real(doppler_bin)*real(c)/real(N_DOPPLER)));
                        i_acc := i_acc + amplitude * cos(phase);
                        q_acc := q_acc + amplitude * sin(phase);
                    end if;
                    
                    -- Noise
                    gauss(seed1, seed2, rand1, rand2);
                    i_acc := i_acc + rand1 * 50.0;
                    q_acc := q_acc + rand2 * 50.0;
                    
                    -- Clamp
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
            
            wait until frame_done = '1';
            wait for 1 us;
        end loop;
        
        report "=== QUICK VALIDATION COMPLETE ===";
        wait for 10 us;
        sim_finished <= true;
        wait;
    end process;

    -- Loggers
    det_log: process(aclk)
        file f : text open write_mode is "ADR_quick_det.txt";
        variable L : line;
    begin
        if rising_edge(aclk) and det_tvalid = '1' and unsigned(det_tdata) > 0 then
            write(L, to_integer(unsigned(det_range_bin)));
            write(L, string'(" "));
            write(L, to_integer(unsigned(det_doppler_bin)));
            write(L, string'(" "));
            write(L, to_integer(unsigned(det_tdata)));
            writeline(f, L);
        end if;
    end process;

    trk_log: process(aclk)
        file f : text open write_mode is "ADR_quick_trk.txt";
        variable L : line;
    begin
        if rising_edge(aclk) then
            if trk_valid = '1' then
                write(L, string'("TRK "));
                write(L, to_integer(unsigned(trk_id)));
                write(L, string'(" R="));
                write(L, to_integer(signed(trk_range)));
                write(L, string'(" D="));
                write(L, to_integer(signed(trk_doppler)));
                write(L, string'(" Q="));
                write(L, to_integer(unsigned(trk_quality)));
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
