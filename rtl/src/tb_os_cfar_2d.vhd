library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;

entity tb_os_cfar_2d is
end tb_os_cfar_2d;

architecture Behavioral of tb_os_cfar_2d is

    constant CLK_PERIOD : time := 10 ns;
    constant DATA_W     : integer := 17;
    constant N_RANGE    : integer := 64;
    constant N_DOPPLER  : integer := 32;
    constant REF_R      : integer := 3;
    constant REF_D      : integer := 2;
    constant GUARD_R    : integer := 1;
    constant GUARD_D    : integer := 1;

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
    signal scale_ovr     : std_logic_vector(2 downto 0) := "000";
    signal dbg_threshold : std_logic_vector(DATA_W-1 downto 0);
    signal dbg_scale     : std_logic_vector(2 downto 0);

    signal sim_done   : boolean := false;
    signal det_count  : integer := 0;
    signal out_count  : integer := 0;
    signal test_phase : integer := 0;

    -- Target locations
    constant TGT1_R : integer := 30;
    constant TGT1_D : integer := 16;
    constant TGT2_R : integer := 50;
    constant TGT2_D : integer := 8;
    constant NOISE   : integer := 100;
    constant TGT_AMP : integer := 5000;

    -- Synthetic magnitude map
    type rdm_t is array (0 to N_RANGE-1, 0 to N_DOPPLER-1) of integer;

    function make_map(noise_floor, tgt_amp : integer;
                      t1r, t1d, t2r, t2d : integer) return rdm_t is
        variable m : rdm_t;
    begin
        for r in 0 to N_RANGE-1 loop
            for d in 0 to N_DOPPLER-1 loop
                m(r, d) := noise_floor + ((r * 7 + d * 13) mod 30);  -- Deterministic "noise"
            end loop;
        end loop;
        -- Insert targets (3x3 mainlobe spread)
        for dr in -1 to 1 loop
            for dd in -1 to 1 loop
                if t1r+dr >= 0 and t1r+dr < N_RANGE and t1d+dd >= 0 and t1d+dd < N_DOPPLER then
                    if dr = 0 and dd = 0 then m(t1r+dr, t1d+dd) := tgt_amp;
                    else m(t1r+dr, t1d+dd) := tgt_amp / 3; end if;
                end if;
                if t2r+dr >= 0 and t2r+dr < N_RANGE and t2d+dd >= 0 and t2d+dd < N_DOPPLER then
                    if dr = 0 and dd = 0 then m(t2r+dr, t2d+dd) := tgt_amp;
                    else m(t2r+dr, t2d+dd) := tgt_amp / 3; end if;
                end if;
            end loop;
        end loop;
        return m;
    end function;

begin

    uut: entity work.os_cfar_2d
    generic map (
        DATA_WIDTH => DATA_W, REF_RANGE => REF_R, REF_DOPPLER => REF_D,
        GUARD_RANGE => GUARD_R, GUARD_DOPPLER => GUARD_D,
        RANK_PCT => 75, SCALE_MIN => 2, SCALE_MAX => 6, SCALE_NOM => 4,
        N_DOPPLER => N_DOPPLER
    )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready, m_axis_tlast => m_axis_tlast,
        scale_override => scale_ovr, dbg_threshold => dbg_threshold,
        dbg_scale => dbg_scale
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    stim: process
        variable rdm : rdm_t;
        variable L : line;

        procedure feed_map(m : rdm_t) is
        begin
            for r in 0 to N_RANGE-1 loop
                for d in 0 to N_DOPPLER-1 loop
                    s_axis_tdata  <= std_logic_vector(to_unsigned(m(r, d), DATA_W));
                    s_axis_tvalid <= '1';
                    s_axis_tlast  <= '1' when d = N_DOPPLER-1 else '0';
                    wait until rising_edge(aclk);
                    while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
                end loop;
            end loop;
            s_axis_tvalid <= '0'; s_axis_tlast <= '0';
        end procedure;

    begin
        aresetn <= '0'; wait for 50 ns;
        aresetn <= '1'; wait until rising_edge(aclk);

        -- ==================================================================
        -- Test 1: Two targets in noise (auto scaling)
        -- ==================================================================
        report "=== Test 1: Two targets, auto scaling ===";
        test_phase <= 1;
        det_count  <= 0;
        scale_ovr  <= "000";
        rdm := make_map(NOISE, TGT_AMP, TGT1_R, TGT1_D, TGT2_R, TGT2_D);
        feed_map(rdm);
        wait for 500 ns;

        write(L, string'("T1 detections: ")); write(L, det_count); writeline(output, L);
        if det_count < 2 then
            report "FAIL T1: expected >= 2 detections" severity error;
        end if;

        -- ==================================================================
        -- Test 2: Uniform noise only (should have 0 or very few false alarms)
        -- ==================================================================
        report "=== Test 2: Noise only ===";
        test_phase <= 2;
        det_count  <= 0;
        rdm := make_map(NOISE, NOISE, TGT1_R, TGT1_D, TGT2_R, TGT2_D);
        -- Override targets back to noise
        for r in 0 to N_RANGE-1 loop
            for d in 0 to N_DOPPLER-1 loop
                rdm(r, d) := NOISE + ((r * 7 + d * 13) mod 30);
            end loop;
        end loop;
        feed_map(rdm);
        wait for 500 ns;

        write(L, string'("T2 false alarms: ")); write(L, det_count); writeline(output, L);
        if det_count > 3 then
            report "WARN T2: too many false alarms" severity warning;
        end if;

        -- ==================================================================
        -- Test 3: Scale override (fixed scale = 2, more sensitive)
        -- ==================================================================
        report "=== Test 3: Scale override = 2 ===";
        test_phase <= 3;
        det_count  <= 0;
        scale_ovr  <= "010";
        rdm := make_map(NOISE, TGT_AMP/2, TGT1_R, TGT1_D, TGT2_R, TGT2_D);
        feed_map(rdm);
        wait for 500 ns;

        write(L, string'("T3 detections (sensitive): ")); write(L, det_count); writeline(output, L);

        -- ==================================================================
        -- Test 4: Backpressure during processing
        -- ==================================================================
        report "=== Test 4: Backpressure ===";
        test_phase <= 4;
        det_count  <= 0;
        scale_ovr  <= "000";
        rdm := make_map(NOISE, TGT_AMP, TGT1_R, TGT1_D, TGT2_R, TGT2_D);
        feed_map(rdm);
        wait for 1 us;

        -- ==================================================================
        -- Test 5: tvalid gaps (validates window shift gating fix)
        -- ==================================================================
        report "=== Test 5: tvalid gaps ===";
        test_phase <= 5;
        det_count  <= 0;
        rdm := make_map(NOISE, TGT_AMP, TGT1_R, TGT1_D, TGT2_R, TGT2_D);
        for r in 0 to N_RANGE-1 loop
            for d in 0 to N_DOPPLER-1 loop
                s_axis_tdata  <= std_logic_vector(to_unsigned(rdm(r, d), DATA_W));
                s_axis_tvalid <= '1';
                s_axis_tlast  <= '1' when d = N_DOPPLER-1 else '0';
                wait until rising_edge(aclk);
                while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
                -- Insert idle gap every 4 samples
                if d mod 4 = 3 then
                    s_axis_tvalid <= '0'; s_axis_tlast <= '0';
                    wait until rising_edge(aclk);
                    wait until rising_edge(aclk);
                end if;
            end loop;
        end loop;
        s_axis_tvalid <= '0'; s_axis_tlast <= '0';
        wait for 500 ns;

        write(L, string'("T5 detections (with gaps): ")); write(L, det_count); writeline(output, L);
        if det_count < 2 then
            report "FAIL T5: gap-gating broken, expected >= 2 detections" severity error;
        end if;

        report "=== OS-CFAR-2D TB Complete ===";
        sim_done <= true; wait;
    end process;

    -- Backpressure for test 4
    bp: process(aclk)
        variable cnt : integer := 0;
    begin
        if rising_edge(aclk) then
            cnt := cnt + 1;
            if test_phase = 4 and cnt mod 5 = 0 then m_axis_tready <= '0';
            else m_axis_tready <= '1'; end if;
        end if;
    end process;

    -- Detection counter & logger
    det_mon: process(aclk)
        variable L : line;
    begin
        if rising_edge(aclk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                out_count <= out_count + 1;
                if unsigned(m_axis_tdata) > 0 then
                    det_count <= det_count + 1;
                    write(L, string'("DET phase=")); write(L, test_phase);
                    write(L, string'(" out#=")); write(L, out_count);
                    write(L, string'(" amp=")); write(L, to_integer(unsigned(m_axis_tdata)));
                    write(L, string'(" thr=")); write(L, to_integer(unsigned(dbg_threshold)));
                    write(L, string'(" scl=")); write(L, to_integer(unsigned(dbg_scale)));
                    writeline(output, L);
                end if;
            end if;
        end if;
    end process;

end Behavioral;
