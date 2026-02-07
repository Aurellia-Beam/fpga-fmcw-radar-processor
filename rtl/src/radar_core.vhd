library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Master Radar Signal Processing Core
-- Pipeline: Window → Range FFT → Corner Turn → MTI → Window → Doppler FFT
--           → Magnitude → 2D OS-CFAR → TWS Tracker
--
-- Consolidated from radar_core_v3 + ADR_radar_core with all bugs fixed.

entity radar_core is
    Generic (
        N_RANGE       : integer := 1024;
        N_DOPPLER     : integer := 128;
        MAX_TRACKS    : integer := 32;
        CFAR_REF_R    : integer := 4;
        CFAR_REF_D    : integer := 4;
        CFAR_GUARD_R  : integer := 2;
        CFAR_GUARD_D  : integer := 1
    );
    Port (
        aclk            : in  STD_LOGIC;
        aresetn         : in  STD_LOGIC;

        -- ADC input (I/Q interleaved, 16-bit each)
        s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_tvalid   : in  STD_LOGIC;
        s_axis_tlast    : in  STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;

        -- Detection output (raw CFAR)
        det_tdata       : out STD_LOGIC_VECTOR(16 downto 0);
        det_tvalid      : out STD_LOGIC;
        det_range_bin   : out STD_LOGIC_VECTOR(9 downto 0);
        det_doppler_bin : out STD_LOGIC_VECTOR(6 downto 0);

        -- Track output (TWS)
        trk_valid       : out STD_LOGIC;
        trk_id          : out STD_LOGIC_VECTOR(5 downto 0);
        trk_range       : out STD_LOGIC_VECTOR(11 downto 0);
        trk_doppler     : out STD_LOGIC_VECTOR(8 downto 0);
        trk_vel_r       : out STD_LOGIC_VECTOR(9 downto 0);
        trk_vel_d       : out STD_LOGIC_VECTOR(7 downto 0);
        trk_quality     : out STD_LOGIC_VECTOR(3 downto 0);
        trk_status      : out STD_LOGIC_VECTOR(1 downto 0);

        -- Control
        mti_bypass      : in  STD_LOGIC := '0';
        cfar_scale_ovr  : in  STD_LOGIC_VECTOR(2 downto 0) := "000";

        -- Status
        active_tracks   : out STD_LOGIC_VECTOR(5 downto 0);
        frame_complete  : out STD_LOGIC;
        scan_complete   : out STD_LOGIC;
        status_overflow : out STD_LOGIC
    );
end radar_core;

architecture Behavioral of radar_core is

    -- =========================================================================
    -- Component declarations
    -- =========================================================================

    component window_multiplier is
        Generic ( DATA_WIDTH : integer; N_SAMPLES : integer; COEF_WIDTH : integer );
        Port (
            aclk, aresetn : in STD_LOGIC;
            s_axis_tdata : in STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid, s_axis_tlast : in STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            m_axis_tdata : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in STD_LOGIC;
            m_axis_tlast, saturation_flag : out STD_LOGIC
        );
    end component;

    -- Xilinx FFT IP: Range (wider config for scaling schedule)
    COMPONENT xfft_range
      PORT (
        aclk, aresetn : IN STD_LOGIC;
        s_axis_config_tdata  : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
        s_axis_config_tvalid : IN  STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        s_axis_data_tdata    : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
        s_axis_data_tvalid   : IN  STD_LOGIC;
        s_axis_data_tready   : OUT STD_LOGIC;
        s_axis_data_tlast    : IN  STD_LOGIC;
        m_axis_data_tdata    : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
        m_axis_data_tuser    : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        m_axis_data_tvalid   : OUT STD_LOGIC;
        m_axis_data_tready   : IN  STD_LOGIC;
        m_axis_data_tlast    : OUT STD_LOGIC;
        event_frame_started            : OUT STD_LOGIC;
        event_tlast_unexpected         : OUT STD_LOGIC;
        event_tlast_missing            : OUT STD_LOGIC;
        event_status_channel_halt      : OUT STD_LOGIC;
        event_data_in_channel_halt     : OUT STD_LOGIC;
        event_data_out_channel_halt    : OUT STD_LOGIC
      );
    END COMPONENT;

    -- Xilinx FFT IP: Doppler
    COMPONENT xfft_doppler
      PORT (
        aclk, aresetn : IN STD_LOGIC;
        s_axis_config_tdata  : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
        s_axis_config_tvalid : IN  STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        s_axis_data_tdata    : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
        s_axis_data_tvalid   : IN  STD_LOGIC;
        s_axis_data_tready   : OUT STD_LOGIC;
        s_axis_data_tlast    : IN  STD_LOGIC;
        m_axis_data_tdata    : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
        m_axis_data_tuser    : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        m_axis_data_tvalid   : OUT STD_LOGIC;
        m_axis_data_tready   : IN  STD_LOGIC;
        m_axis_data_tlast    : OUT STD_LOGIC;
        event_frame_started            : OUT STD_LOGIC;
        event_tlast_unexpected         : OUT STD_LOGIC;
        event_tlast_missing            : OUT STD_LOGIC;
        event_status_channel_halt      : OUT STD_LOGIC;
        event_data_in_channel_halt     : OUT STD_LOGIC;
        event_data_out_channel_halt    : OUT STD_LOGIC
      );
    END COMPONENT;

    component corner_turner is
        Generic ( N_RANGE : integer; N_DOPPLER : integer; DATA_WIDTH : integer );
        Port (
            aclk, aresetn : in STD_LOGIC;
            s_axis_tdata : in STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid, s_axis_tlast : in STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            m_axis_tdata : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in STD_LOGIC;
            m_axis_tlast, frame_complete, overflow_error : out STD_LOGIC
        );
    end component;

    component doppler_notch is
        Generic ( DATA_WIDTH : integer; N_DOPPLER : integer; NOTCH_MODE : integer );
        Port (
            aclk, aresetn : in STD_LOGIC;
            s_axis_tdata : in STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid : in STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast : in STD_LOGIC;
            m_axis_tdata : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in STD_LOGIC;
            m_axis_tlast : out STD_LOGIC;
            bypass : in STD_LOGIC
        );
    end component;

    component magnitude_calc is
        Generic ( DATA_WIDTH : integer; OUT_WIDTH : integer );
        Port (
            aclk, aresetn : in STD_LOGIC;
            s_axis_tdata : in STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid, s_axis_tlast : in STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            m_axis_tdata : out STD_LOGIC_VECTOR(16 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in STD_LOGIC;
            m_axis_tlast : out STD_LOGIC
        );
    end component;

    component os_cfar_2d is
        Generic (
            DATA_WIDTH : integer; REF_RANGE : integer; REF_DOPPLER : integer;
            GUARD_RANGE : integer; GUARD_DOPPLER : integer; RANK_PCT : integer;
            SCALE_MIN : integer; SCALE_MAX : integer; SCALE_NOM : integer;
            N_DOPPLER : integer
        );
        Port (
            aclk, aresetn : in STD_LOGIC;
            s_axis_tdata : in STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
            s_axis_tvalid : in STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast : in STD_LOGIC;
            m_axis_tdata : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in STD_LOGIC;
            m_axis_tlast : out STD_LOGIC;
            scale_override : in STD_LOGIC_VECTOR(2 downto 0);
            dbg_threshold : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
            dbg_scale : out STD_LOGIC_VECTOR(2 downto 0)
        );
    end component;

    component tws_tracker is
        Generic (
            MAX_TRACKS : integer; N_RANGE : integer; N_DOPPLER : integer;
            INIT_HITS : integer; COAST_MAX : integer;
            ASSOC_GATE_R : integer; ASSOC_GATE_D : integer;
            ALPHA_GAIN : integer; BETA_GAIN : integer
        );
        Port (
            aclk, aresetn : in STD_LOGIC;
            det_valid : in STD_LOGIC;
            det_range : in STD_LOGIC_VECTOR(9 downto 0);
            det_doppler : in STD_LOGIC_VECTOR(6 downto 0);
            det_magnitude : in STD_LOGIC_VECTOR(16 downto 0);
            det_last : in STD_LOGIC;
            trk_valid : out STD_LOGIC;
            trk_id : out STD_LOGIC_VECTOR(5 downto 0);
            trk_range : out STD_LOGIC_VECTOR(11 downto 0);
            trk_doppler : out STD_LOGIC_VECTOR(8 downto 0);
            trk_vel_r : out STD_LOGIC_VECTOR(9 downto 0);
            trk_vel_d : out STD_LOGIC_VECTOR(7 downto 0);
            trk_quality : out STD_LOGIC_VECTOR(3 downto 0);
            trk_status : out STD_LOGIC_VECTOR(1 downto 0);
            active_tracks : out STD_LOGIC_VECTOR(5 downto 0);
            scan_complete : out STD_LOGIC
        );
    end component;

    -- =========================================================================
    -- Interconnect signals (single set — duplicates removed)
    -- =========================================================================

    signal win1_to_rfft_data, rfft_to_ct_data, ct_to_mti_data  : std_logic_vector(31 downto 0);
    signal mti_to_win2_data, win2_to_dfft_data, dfft_to_mag_data : std_logic_vector(31 downto 0);
    signal win1_to_rfft_valid, win1_to_rfft_ready, win1_to_rfft_last : std_logic;
    signal rfft_to_ct_valid,   rfft_to_ct_ready,   rfft_to_ct_last   : std_logic;
    signal ct_to_mti_valid,    ct_to_mti_ready,    ct_to_mti_last    : std_logic;
    signal mti_to_win2_valid,  mti_to_win2_ready,  mti_to_win2_last  : std_logic;
    signal win2_to_dfft_valid, win2_to_dfft_ready, win2_to_dfft_last : std_logic;
    signal dfft_to_mag_valid,  dfft_to_mag_ready,  dfft_to_mag_last  : std_logic;

    signal mag_to_cfar_data  : std_logic_vector(16 downto 0);
    signal mag_to_cfar_valid, mag_to_cfar_ready, mag_to_cfar_last : std_logic;

    signal cfar_out_data  : std_logic_vector(16 downto 0);
    signal cfar_out_valid, cfar_out_last : std_logic;
    signal det_tvalid_int : std_logic;

    signal win1_saturation, win2_saturation : std_logic;
    signal ct_frame_done, ct_overflow : std_logic;

    -- FIX: FFT config — bit 0 = 1 for FORWARD transform
    constant FFT_FWD_16 : std_logic_vector(15 downto 0) := x"0001";
    constant FFT_FWD_8  : std_logic_vector(7 downto 0)  := x"01";

    type cfg_state_t is (CFG_IDLE, CFG_R, CFG_D, CFG_DONE);
    signal cfg_state : cfg_state_t := CFG_IDLE;
    signal rfft_cfg_valid, dfft_cfg_valid : std_logic := '0';
    signal rfft_cfg_ready, dfft_cfg_ready : std_logic;

    -- Index tracking for detection coordinates
    signal range_idx   : unsigned(9 downto 0) := (others => '0');
    signal doppler_idx : unsigned(6 downto 0) := (others => '0');

    signal overflow_sticky : std_logic := '0';

begin

    -- =========================================================================
    -- Processing chain instantiation
    -- =========================================================================

    u_range_window : window_multiplier
    generic map ( DATA_WIDTH => 32, N_SAMPLES => N_RANGE, COEF_WIDTH => 16 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => win1_to_rfft_data, m_axis_tvalid => win1_to_rfft_valid,
        m_axis_tready => win1_to_rfft_ready, m_axis_tlast => win1_to_rfft_last,
        saturation_flag => win1_saturation
    );

    -- FFT config state machine
    cfg_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                cfg_state <= CFG_IDLE;
                rfft_cfg_valid <= '0'; dfft_cfg_valid <= '0';
            else
                case cfg_state is
                    when CFG_IDLE =>
                        rfft_cfg_valid <= '1'; cfg_state <= CFG_R;
                    when CFG_R =>
                        if rfft_cfg_ready = '1' then
                            rfft_cfg_valid <= '0'; dfft_cfg_valid <= '1'; cfg_state <= CFG_D;
                        end if;
                    when CFG_D =>
                        if dfft_cfg_ready = '1' then
                            dfft_cfg_valid <= '0'; cfg_state <= CFG_DONE;
                        end if;
                    when CFG_DONE => null;
                end case;
            end if;
        end if;
    end process;

    u_range_fft : xfft_range
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_config_tdata => FFT_FWD_16, s_axis_config_tvalid => rfft_cfg_valid,
        s_axis_config_tready => rfft_cfg_ready,
        s_axis_data_tdata => win1_to_rfft_data, s_axis_data_tvalid => win1_to_rfft_valid,
        s_axis_data_tready => win1_to_rfft_ready, s_axis_data_tlast => win1_to_rfft_last,
        m_axis_data_tdata => rfft_to_ct_data, m_axis_data_tuser => open,
        m_axis_data_tvalid => rfft_to_ct_valid, m_axis_data_tready => rfft_to_ct_ready,
        m_axis_data_tlast => rfft_to_ct_last,
        event_frame_started => open, event_tlast_unexpected => open,
        event_tlast_missing => open, event_status_channel_halt => open,
        event_data_in_channel_halt => open, event_data_out_channel_halt => open
    );

    u_corner_turner : corner_turner
    generic map ( N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER, DATA_WIDTH => 32 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => rfft_to_ct_data, s_axis_tvalid => rfft_to_ct_valid,
        s_axis_tready => rfft_to_ct_ready, s_axis_tlast => rfft_to_ct_last,
        m_axis_tdata => ct_to_mti_data, m_axis_tvalid => ct_to_mti_valid,
        m_axis_tready => ct_to_mti_ready, m_axis_tlast => ct_to_mti_last,
        frame_complete => ct_frame_done, overflow_error => ct_overflow
    );

    u_mti : doppler_notch
    generic map ( DATA_WIDTH => 32, N_DOPPLER => N_DOPPLER, NOTCH_MODE => 2 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => ct_to_mti_data, s_axis_tvalid => ct_to_mti_valid,
        s_axis_tready => ct_to_mti_ready, s_axis_tlast => ct_to_mti_last,
        m_axis_tdata => mti_to_win2_data, m_axis_tvalid => mti_to_win2_valid,
        m_axis_tready => mti_to_win2_ready, m_axis_tlast => mti_to_win2_last,
        bypass => mti_bypass
    );

    u_doppler_window : window_multiplier
    generic map ( DATA_WIDTH => 32, N_SAMPLES => N_DOPPLER, COEF_WIDTH => 16 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => mti_to_win2_data, s_axis_tvalid => mti_to_win2_valid,
        s_axis_tready => mti_to_win2_ready, s_axis_tlast => mti_to_win2_last,
        m_axis_tdata => win2_to_dfft_data, m_axis_tvalid => win2_to_dfft_valid,
        m_axis_tready => win2_to_dfft_ready, m_axis_tlast => win2_to_dfft_last,
        saturation_flag => win2_saturation
    );

    u_doppler_fft : xfft_doppler
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_config_tdata => FFT_FWD_8, s_axis_config_tvalid => dfft_cfg_valid,
        s_axis_config_tready => dfft_cfg_ready,
        s_axis_data_tdata => win2_to_dfft_data, s_axis_data_tvalid => win2_to_dfft_valid,
        s_axis_data_tready => win2_to_dfft_ready, s_axis_data_tlast => win2_to_dfft_last,
        m_axis_data_tdata => dfft_to_mag_data, m_axis_data_tuser => open,
        m_axis_data_tvalid => dfft_to_mag_valid, m_axis_data_tready => dfft_to_mag_ready,
        m_axis_data_tlast => dfft_to_mag_last,
        event_frame_started => open, event_tlast_unexpected => open,
        event_tlast_missing => open, event_status_channel_halt => open,
        event_data_in_channel_halt => open, event_data_out_channel_halt => open
    );

    u_magnitude : magnitude_calc
    generic map ( DATA_WIDTH => 16, OUT_WIDTH => 17 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => dfft_to_mag_data, s_axis_tvalid => dfft_to_mag_valid,
        s_axis_tready => dfft_to_mag_ready, s_axis_tlast => dfft_to_mag_last,
        m_axis_tdata => mag_to_cfar_data, m_axis_tvalid => mag_to_cfar_valid,
        m_axis_tready => mag_to_cfar_ready, m_axis_tlast => mag_to_cfar_last
    );

    u_cfar : os_cfar_2d
    generic map (
        DATA_WIDTH => 17, REF_RANGE => CFAR_REF_R, REF_DOPPLER => CFAR_REF_D,
        GUARD_RANGE => CFAR_GUARD_R, GUARD_DOPPLER => CFAR_GUARD_D,
        RANK_PCT => 75, SCALE_MIN => 2, SCALE_MAX => 6, SCALE_NOM => 4,
        N_DOPPLER => N_DOPPLER
    )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => mag_to_cfar_data, s_axis_tvalid => mag_to_cfar_valid,
        s_axis_tready => mag_to_cfar_ready, s_axis_tlast => mag_to_cfar_last,
        m_axis_tdata => cfar_out_data, m_axis_tvalid => cfar_out_valid,
        m_axis_tready => '1', m_axis_tlast => cfar_out_last,
        scale_override => cfar_scale_ovr, dbg_threshold => open, dbg_scale => open
    );

    -- =========================================================================
    -- Detection coordinate tracking
    -- =========================================================================

    idx_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                range_idx <= (others => '0'); doppler_idx <= (others => '0');
            elsif cfar_out_valid = '1' then
                if doppler_idx = N_DOPPLER - 1 then
                    doppler_idx <= (others => '0');
                    if range_idx = N_RANGE - 1 then range_idx <= (others => '0');
                    else range_idx <= range_idx + 1; end if;
                else
                    doppler_idx <= doppler_idx + 1;
                end if;
            end if;
        end if;
    end process;

    -- Detection output (filter: only pass non-zero)
    det_tdata       <= cfar_out_data;
    det_tvalid_int  <= cfar_out_valid when unsigned(cfar_out_data) > 0 else '0';
    det_tvalid      <= det_tvalid_int;
    det_range_bin   <= std_logic_vector(range_idx);
    det_doppler_bin <= std_logic_vector(doppler_idx);

    -- =========================================================================
    -- TWS Tracker
    -- =========================================================================

    u_tws : tws_tracker
    generic map (
        MAX_TRACKS => MAX_TRACKS, N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER,
        INIT_HITS => 2, COAST_MAX => 5, ASSOC_GATE_R => 10, ASSOC_GATE_D => 5,
        ALPHA_GAIN => 128, BETA_GAIN => 64
    )
    port map (
        aclk => aclk, aresetn => aresetn,
        det_valid => det_tvalid_int, det_range => std_logic_vector(range_idx),
        det_doppler => std_logic_vector(doppler_idx), det_magnitude => cfar_out_data,
        det_last => cfar_out_last,
        trk_valid => trk_valid, trk_id => trk_id, trk_range => trk_range,
        trk_doppler => trk_doppler, trk_vel_r => trk_vel_r, trk_vel_d => trk_vel_d,
        trk_quality => trk_quality, trk_status => trk_status,
        active_tracks => active_tracks, scan_complete => scan_complete
    );

    -- =========================================================================
    -- Status
    -- =========================================================================

    frame_complete <= ct_frame_done;

    overflow_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then overflow_sticky <= '0';
            elsif win1_saturation = '1' or win2_saturation = '1' or ct_overflow = '1' then
                overflow_sticky <= '1';
            end if;
        end if;
    end process;
    status_overflow <= overflow_sticky;

end Behavioral;
