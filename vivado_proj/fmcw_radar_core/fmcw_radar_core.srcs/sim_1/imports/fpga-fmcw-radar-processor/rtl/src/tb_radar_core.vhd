library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;

-- Full pipeline testbench (requires Xilinx FFT IP in project)
-- Multi-frame with randomized targets, detection & track logging

entity tb_radar_core is
end tb_radar_core;

architecture Behavioral of tb_radar_core is

    constant N_RANGE    : integer := 1024;
    constant N_DOPPLER  : integer := 128;
    constant CLK_PERIOD : time := 10 ns;
    constant NUM_FRAMES : integer := 3;
    constant TIMEOUT    : integer := 10_000_000;

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

    signal sim_done      : boolean := false;
    signal det_count     : integer := 0;

begin

    uut: entity work.radar_core
    generic map (
        N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER, MAX_TRACKS => 16,
        CFAR_REF_R => 4, CFAR_REF_D => 4, CFAR_GUARD_R => 2, CFAR_GUARD_D => 1
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
        mti_bypass => '1', cfar_scale_ovr => "000",
        active_tracks => active_tracks, frame_complete => frame_done,
        scan_complete => scan_done, status_overflow => overflow
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    -- Stimulus: inject randomized single-target frames
    stim: process
        variable seed1, seed2 : positive := 1234;
        variable rand_val : real;
        variable tgt_range, tgt_doppler, tgt_amp : real;
        variable noise_floor : real := 100.0;
        variable phase, i_acc, q_acc : real;
        variable i_int, q_int : integer;
    begin
        aresetn <= '0'; wait for 100 ns;
        aresetn <= '1';
        wait until s_axis_tready = '1' and rising_edge(aclk);
        report "Core ready.";

        for frame in 1 to NUM_FRAMES loop
            uniform(seed1, seed2, rand_val); tgt_range   := rand_val * 900.0 + 50.0;
            uniform(seed1, seed2, rand_val); tgt_doppler := rand_val * 100.0;
            tgt_amp := 20000.0;

            report "Frame " & integer'image(frame) &
                   ": R=" & integer'image(integer(tgt_range)) &
                   " D=" & integer'image(integer(tgt_doppler));

            for c in 0 to N_DOPPLER-1 loop
                for s in 0 to N_RANGE-1 loop
                    phase := 2.0 * MATH_PI * (
                        tgt_range * real(s) / real(N_RANGE) +
                        tgt_doppler * real(c) / real(N_DOPPLER));
                    i_acc := tgt_amp * cos(phase);
                    q_acc := tgt_amp * sin(phase);

                    uniform(seed1, seed2, rand_val);
                    i_acc := i_acc + (rand_val - 0.5) * 2.0 * noise_floor;
                    uniform(seed1, seed2, rand_val);
                    q_acc := q_acc + (rand_val - 0.5) * 2.0 * noise_floor;

                    i_int := integer(i_acc); q_int := integer(q_acc);
                    if i_int >  32767 then i_int :=  32767; end if;
                    if i_int < -32768 then i_int := -32768; end if;
                    if q_int >  32767 then q_int :=  32767; end if;
                    if q_int < -32768 then q_int := -32768; end if;

                    s_axis_tdata  <= std_logic_vector(to_signed(q_int, 16)) &
                                     std_logic_vector(to_signed(i_int, 16));
                    s_axis_tvalid <= '1';
                    s_axis_tlast  <= '1' when s = N_RANGE-1 else '0';

                    wait until rising_edge(aclk);
                    while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
                end loop;
            end loop;

            s_axis_tvalid <= '0'; s_axis_tlast <= '0';
            wait until frame_done = '1'; wait for 1 us;
        end loop;

        report "Complete. Detections: " & integer'image(det_count);
        wait for 10 us;
        sim_done <= true;
        wait;
    end process;

    -- Watchdog
    watchdog: process
        variable cyc : integer := 0;
    begin
        wait until aresetn = '1';
        while not sim_done loop
            wait until rising_edge(aclk);
            cyc := cyc + 1;
            if cyc > TIMEOUT then report "TIMEOUT" severity failure; end if;
        end loop;
        wait;
    end process;

    -- Detection logger
    det_log: process(aclk)
        file f : text open write_mode is "detections.txt";
        variable L : line;
    begin
        if rising_edge(aclk) and det_tvalid = '1' then
            det_count <= det_count + 1;
            write(L, to_integer(unsigned(det_range_bin))); write(L, string'(" "));
            write(L, to_integer(unsigned(det_doppler_bin))); write(L, string'(" "));
            write(L, to_integer(unsigned(det_tdata)));
            writeline(f, L);
        end if;
    end process;

    -- Track logger
    trk_log: process(aclk)
        file f : text open write_mode is "tracks.txt";
        variable L : line;
    begin
        if rising_edge(aclk) then
            if trk_valid = '1' then
                write(L, string'("TRK ")); write(L, to_integer(unsigned(trk_id)));
                write(L, string'(" R=")); write(L, to_integer(signed(trk_range)));
                write(L, string'(" D=")); write(L, to_integer(signed(trk_doppler)));
                write(L, string'(" Q=")); write(L, to_integer(unsigned(trk_quality)));
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
