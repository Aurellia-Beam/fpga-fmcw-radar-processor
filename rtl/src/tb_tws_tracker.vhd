library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use std.textio.all;

entity tb_tws_tracker is
end tb_tws_tracker;

architecture Behavioral of tb_tws_tracker is

    constant CLK_PERIOD  : time := 10 ns;
    constant MAX_TRACKS  : integer := 16;
    constant N_RANGE     : integer := 1024;
    constant N_DOPPLER   : integer := 128;
    constant INIT_HITS   : integer := 2;
    constant COAST_MAX   : integer := 3;
    constant ASSOC_GATE_R : integer := 10;
    constant ASSOC_GATE_D : integer := 5;
    constant NUM_SCANS   : integer := 12;

    signal aclk          : std_logic := '0';
    signal aresetn       : std_logic := '0';
    signal det_valid     : std_logic := '0';
    signal det_range     : std_logic_vector(9 downto 0) := (others => '0');
    signal det_doppler   : std_logic_vector(6 downto 0) := (others => '0');
    signal det_magnitude : std_logic_vector(16 downto 0) := (others => '0');
    signal det_last      : std_logic := '0';

    signal trk_valid     : std_logic;
    signal trk_id        : std_logic_vector(5 downto 0);
    signal trk_range     : std_logic_vector(11 downto 0);
    signal trk_doppler   : std_logic_vector(8 downto 0);
    signal trk_vel_r     : std_logic_vector(9 downto 0);
    signal trk_vel_d     : std_logic_vector(7 downto 0);
    signal trk_quality   : std_logic_vector(3 downto 0);
    signal trk_status    : std_logic_vector(1 downto 0);
    signal active_tracks : std_logic_vector(5 downto 0);
    signal scan_complete : std_logic;

    signal sim_done     : boolean := false;
    signal fail_count   : integer := 0;
    signal scan_number  : integer := 0;
    signal trk_count    : integer := 0;
    signal trk_count_reset : std_logic := '0';

begin

    uut: entity work.tws_tracker
    generic map (
        MAX_TRACKS => MAX_TRACKS, N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER,
        INIT_HITS => INIT_HITS, COAST_MAX => COAST_MAX,
        ASSOC_GATE_R => ASSOC_GATE_R, ASSOC_GATE_D => ASSOC_GATE_D,
        ALPHA_GAIN => 128, BETA_GAIN => 64
    )
    port map (
        aclk => aclk, aresetn => aresetn,
        det_valid => det_valid, det_range => det_range,
        det_doppler => det_doppler, det_magnitude => det_magnitude,
        det_last => det_last,
        trk_valid => trk_valid, trk_id => trk_id, trk_range => trk_range,
        trk_doppler => trk_doppler, trk_vel_r => trk_vel_r, trk_vel_d => trk_vel_d,
        trk_quality => trk_quality, trk_status => trk_status,
        active_tracks => active_tracks, scan_complete => scan_complete
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    stim: process
        variable L : line;

        procedure send_det(r : integer; d : integer; mag : integer; last : boolean) is
        begin
            det_range     <= std_logic_vector(to_unsigned(r, 10));
            det_doppler   <= std_logic_vector(to_unsigned(d, 7));
            det_magnitude <= std_logic_vector(to_unsigned(mag, 17));
            det_valid     <= '1';
            if last then det_last <= '1'; else det_last <= '0'; end if;
            wait until rising_edge(aclk);
            det_valid <= '0'; det_last <= '0';
        end procedure;

        procedure end_scan is
        begin
            det_last <= '1'; det_valid <= '0';
            wait until rising_edge(aclk);
            det_last <= '0';
            -- Wait for tracker to process
            wait until scan_complete = '1';
            wait for 100 ns;
        end procedure;

        procedure log_scan_status(scan : integer) is
        begin
            write(L, string'("Scan ")); write(L, scan);
            write(L, string'(": active=")); write(L, to_integer(unsigned(active_tracks)));
            write(L, string'(" reported_trks=")); write(L, trk_count);
            writeline(output, L);
        end procedure;

        -- Target 1: steady at R=200, D=40, approaching (R decreases by 5/scan)
        -- Target 2: steady at R=600, D=80
        -- Target 3: appears scan 4, disappears scan 8

        variable t1_r, t2_r, t3_r : integer;
        variable max_active_seen : integer := 0;

    begin
        aresetn <= '0'; wait for 50 ns;
        aresetn <= '1'; wait until rising_edge(aclk);

        report "=== TWS Tracker TB: " & integer'image(NUM_SCANS) & " scans ===";

        for scan in 1 to NUM_SCANS loop
            scan_number <= scan;
            trk_count_reset <= '1';
            t1_r := 200 - (scan - 1) * 5;
            t2_r := 600;
            t3_r := 400 + (scan - 4) * 3;

            -- Send detections
            if t1_r > 0 then
                send_det(t1_r, 40, 5000, false);
            end if;

            send_det(t2_r, 80, 8000, false);

            -- Target 3: only scans 4-7
            if scan >= 4 and scan <= 7 then
                send_det(t3_r, 60, 3000, false);
            end if;

            -- Add a false alarm every 3rd scan
            if scan mod 3 = 0 then
                send_det(900, 10, 2000, false);
            end if;

            -- End scan (empty detection with last)
            end_scan;
            log_scan_status(scan);

            if to_integer(unsigned(active_tracks)) > max_active_seen then
                max_active_seen := to_integer(unsigned(active_tracks));
            end if;

            -- ============================================================
            -- Assertions per scan
            -- ============================================================

            -- After scan 2: target 1 and 2 should be tentative
            if scan = 2 then
                if to_integer(unsigned(active_tracks)) < 2 then
                    report "FAIL: scan 2 should have >= 2 tracks" severity error;
                    fail_count <= fail_count + 1;
                end if;
            end if;

            -- After scan 3: targets 1 and 2 should be firm (INIT_HITS=2, 3rd hit)
            if scan = 3 then
                if to_integer(unsigned(active_tracks)) < 2 then
                    report "FAIL: scan 3 should have >= 2 firm tracks" severity error;
                    fail_count <= fail_count + 1;
                end if;
            end if;

            -- After scan 6: should have 3 active targets
            if scan = 6 then
                if to_integer(unsigned(active_tracks)) < 3 then
                    report "FAIL: scan 6 should have >= 3 tracks" severity error;
                    fail_count <= fail_count + 1;
                end if;
            end if;

            -- After scan 11: target 3 gone for 4 scans (>COAST_MAX=3), should be dropped
            if scan = 11 then
                if to_integer(unsigned(active_tracks)) > 3 then
                    report "WARN: scan 11 still has too many tracks (coast not dropping)" severity warning;
                end if;
            end if;

        end loop;

        report "=== Results ===";
        write(L, string'("Max tracks seen: ")); write(L, max_active_seen); writeline(output, L);

        if fail_count = 0 then report "ALL TESTS PASSED";
        else report integer'image(fail_count) & " FAILURES" severity error; end if;

        sim_done <= true; wait;
    end process;

    -- Track output logger
    trk_log: process(aclk)
        variable L : line;
    begin
        if rising_edge(aclk) then
            if trk_count_reset = '1' then
                trk_count <= 0;
                trk_count_reset <= '0';
            elsif trk_valid = '1' then
                trk_count <= trk_count + 1;
                write(L, string'("  TRK id=")); write(L, to_integer(unsigned(trk_id)));
                write(L, string'(" R=")); write(L, to_integer(signed(trk_range)));
                write(L, string'(" D=")); write(L, to_integer(signed(trk_doppler)));
                write(L, string'(" VR=")); write(L, to_integer(signed(trk_vel_r)));
                write(L, string'(" Q=")); write(L, to_integer(unsigned(trk_quality)));
                write(L, string'(" S="));
                case trk_status is
                    when "00" => write(L, string'("FREE"));
                    when "01" => write(L, string'("TENT"));
                    when "10" => write(L, string'("FIRM"));
                    when "11" => write(L, string'("COAST"));
                    when others => write(L, string'("???"));
                end case;
                writeline(output, L);
            end if;
        end if;
    end process;

end Behavioral;
