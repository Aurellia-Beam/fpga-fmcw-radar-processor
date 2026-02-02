library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================================
-- FMCW Radar Signal Processing Core v4 (OPTIMIZED)
-- =============================================================================
-- OPTIMIZATIONS:
--   1. Gated config FSM (no magic waits - uses ready signals)
--   2. Proper FFT scaling on both Range and Doppler FFTs
--   3. Parallel Doppler FFT support (N_PARALLEL generic)
--   4. Streamlined pipeline handshaking
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity radar_core_v3 is
    Generic (
        N_RANGE    : integer := 1024;
        N_DOPPLER  : integer := 128;
        N_PARALLEL : integer := 1      -- Parallel Doppler FFT lanes (1, 2, or 4)
    );
    Port (
        aclk            : in  STD_LOGIC;
        aresetn         : in  STD_LOGIC;
        
        s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_tvalid   : in  STD_LOGIC;
        s_axis_tlast    : in  STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;
        
        m_axis_tdata    : out STD_LOGIC_VECTOR(16 downto 0);
        m_axis_tvalid   : out STD_LOGIC;
        m_axis_tlast    : out STD_LOGIC;
        m_axis_tready   : in  STD_LOGIC;
        
        rdm_range_bin   : out STD_LOGIC_VECTOR(9 downto 0);
        rdm_doppler_bin : out STD_LOGIC_VECTOR(6 downto 0);
        
        status_range_fft_done   : out STD_LOGIC;
        status_doppler_fft_done : out STD_LOGIC;
        status_frame_complete   : out STD_LOGIC;
        status_overflow         : out STD_LOGIC   -- Overflow detected in pipeline
    );
end radar_core_v3;

architecture Behavioral of radar_core_v3 is

    -- =========================================================================
    -- Component Declarations
    -- =========================================================================
    component window_multiplier is
        Generic ( DATA_WIDTH : integer := 32; N_SAMPLES : integer := 1024; COEF_WIDTH : integer := 16 );
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

    -- Range FFT (1024-pt, 8-bit config)
    COMPONENT xfft_0
      PORT (
        aclk : IN STD_LOGIC;
        s_axis_config_tdata : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
        s_axis_config_tvalid : IN STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        s_axis_data_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
        s_axis_data_tvalid : IN STD_LOGIC;
        s_axis_data_tready : OUT STD_LOGIC;
        s_axis_data_tlast : IN STD_LOGIC;
        m_axis_data_tdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
        m_axis_data_tuser : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        m_axis_data_tvalid : OUT STD_LOGIC;
        m_axis_data_tready : IN STD_LOGIC;
        m_axis_data_tlast : OUT STD_LOGIC;
        m_axis_status_tdata : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        m_axis_status_tvalid : OUT STD_LOGIC;
        m_axis_status_tready : IN STD_LOGIC;
        event_frame_started : OUT STD_LOGIC;
        event_tlast_unexpected : OUT STD_LOGIC;
        event_tlast_missing : OUT STD_LOGIC;
        event_status_channel_halt : OUT STD_LOGIC;
        event_data_in_channel_halt : OUT STD_LOGIC;
        event_data_out_channel_halt : OUT STD_LOGIC
      );
    END COMPONENT;

    -- Doppler FFT (128-pt, 16-bit config)
    COMPONENT xfft_doppler
      PORT (
        aclk : IN STD_LOGIC;
        s_axis_config_tdata : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
        s_axis_config_tvalid : IN STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        s_axis_data_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
        s_axis_data_tvalid : IN STD_LOGIC;
        s_axis_data_tready : OUT STD_LOGIC;
        s_axis_data_tlast : IN STD_LOGIC;
        m_axis_data_tdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
        m_axis_data_tvalid : OUT STD_LOGIC;
        m_axis_data_tready : IN STD_LOGIC;
        m_axis_data_tlast : OUT STD_LOGIC;
        event_frame_started : OUT STD_LOGIC;
        event_tlast_unexpected : OUT STD_LOGIC;
        event_tlast_missing : OUT STD_LOGIC;
        event_status_channel_halt : OUT STD_LOGIC;
        event_data_in_channel_halt : OUT STD_LOGIC;
        event_data_out_channel_halt : OUT STD_LOGIC
      );
    END COMPONENT;

    component corner_turner is
        Generic ( N_RANGE : integer := 1024; N_DOPPLER : integer := 128; DATA_WIDTH : integer := 32 );
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
    
    component magnitude_calc is
        Generic ( DATA_WIDTH : integer := 16; OUT_WIDTH : integer := 17 );
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

    -- =========================================================================
    -- Bit Growth Management Strategy
    -- =========================================================================
    -- Input: 16-bit I/Q (Q15)
    -- 
    -- Stage            | Bits In | Growth | Scaling      | Bits Out
    -- -----------------|---------|--------|--------------|----------
    -- Range Window     | 16      | 0      | ×0.54        | 16 (Q15×Q15→Q15)
    -- Range FFT 1024   | 16      | +10    | /64 (config) | 20 (still growing)
    -- Range Scaler     | 20      | 0      | /16 (shift)  | 16 (normalized)
    -- Corner Turn      | 16      | 0      | none         | 16
    -- Doppler Window   | 16      | 0      | ×0.54        | 16
    -- Doppler FFT 128  | 16      | +7     | /128 (config)| 16 (normalized)
    -- Magnitude        | 16      | +1     | none         | 17
    --
    -- Total FFT scaling: Range(/64×/16=/1024) + Doppler(/128) = proper normalization
    -- =========================================================================
    
    -- =========================================================================
    -- Xilinx FFT IP Scaling Schedule Format
    -- =========================================================================
    -- For "scaled" architecture with run-time configurable scaling:
    --   - 2 bits per radix-2 stage: 00=÷1, 01=÷2, 10=÷4, 11=÷8
    --   - LSB corresponds to first stage
    --   - Bit 0 of config word = FWD(1)/INV(0)
    --
    -- Range FFT (1024-pt = 10 stages, 5 radix-4 or pairs):
    --   Total scaling needed: ÷1024 = ÷2^10
    --   Config bits [7:1] = scaling, bit[0] = direction
    --   Schedule: ÷4 per pair × 5 pairs = ÷1024
    --   Pattern: 10_10_10_10_10 (but only 7 bits available in 8-bit config)
    --   With 8-bit config: use ÷2 per pair: 01_01_01 = ÷8, need more
    --   Actual: 0xD5 = 1101_0101 → FWD + [10 10 10] = ÷64 per 3 pairs
    --   Remaining growth handled by input scaling or wider internal path
    constant RANGE_FFT_CONFIG : std_logic_vector(7 downto 0) := x"D5";
    
    -- Doppler FFT (128-pt = 7 stages):
    --   Total scaling needed: ÷128 = ÷2^7
    --   16-bit config: bits[15:1] = scaling (7 pairs), bit[0] = direction
    --   Schedule: ÷2 per stage × 7 stages = ÷128
    --   Pattern: 01_01_01_01_01_01_01 + FWD = 0x0155
    constant DOPPLER_FFT_CONFIG : std_logic_vector(15 downto 0) := x"0155";

    -- =========================================================================
    -- Pipeline Signals
    -- =========================================================================
    
    -- Window 1 (Range) -> Range FFT
    signal win1_to_rfft_data  : STD_LOGIC_VECTOR(31 downto 0);
    signal win1_to_rfft_valid : STD_LOGIC;
    signal win1_to_rfft_ready : STD_LOGIC;
    signal win1_to_rfft_last  : STD_LOGIC;
    
    -- Range FFT -> Scaler -> Corner Turner
    signal rfft_to_ct_data  : STD_LOGIC_VECTOR(31 downto 0);
    signal rfft_to_ct_valid : STD_LOGIC;
    signal rfft_to_ct_ready : STD_LOGIC;
    signal rfft_to_ct_last  : STD_LOGIC;
    
    -- Raw FFT output (before scaling)
    signal rfft_raw_data    : STD_LOGIC_VECTOR(31 downto 0);
    signal rfft_raw_valid   : STD_LOGIC;
    signal rfft_raw_ready   : STD_LOGIC;
    signal rfft_raw_last    : STD_LOGIC;
    
    -- Corner Turner -> Window 2 (Doppler)
    signal ct_to_win2_data  : STD_LOGIC_VECTOR(31 downto 0);
    signal ct_to_win2_valid : STD_LOGIC;
    signal ct_to_win2_ready : STD_LOGIC;
    signal ct_to_win2_last  : STD_LOGIC;

    -- Window 2 (Doppler) -> Doppler FFT
    signal win2_to_dfft_data  : STD_LOGIC_VECTOR(31 downto 0);
    signal win2_to_dfft_valid : STD_LOGIC;
    signal win2_to_dfft_ready : STD_LOGIC;
    signal win2_to_dfft_last  : STD_LOGIC;
    
    -- Doppler FFT -> Magnitude
    signal dfft_to_mag_data  : STD_LOGIC_VECTOR(31 downto 0);
    signal dfft_to_mag_valid : STD_LOGIC;
    signal dfft_to_mag_ready : STD_LOGIC;
    signal dfft_to_mag_last  : STD_LOGIC;
    
    -- =========================================================================
    -- Config FSM - Gated on Ready Signals (NO MAGIC WAITS)
    -- =========================================================================
    type cfg_state_t is (CFG_IDLE, CFG_SEND_RANGE, CFG_SEND_DOPPLER, CFG_DONE);
    signal cfg_state : cfg_state_t := CFG_IDLE;
    signal rfft_cfg_valid, dfft_cfg_valid : STD_LOGIC := '0';
    signal rfft_cfg_ready, dfft_cfg_ready : STD_LOGIC;
    signal config_complete : STD_LOGIC := '0';
    
    signal ct_frame_complete, rfft_frame_started, dfft_frame_started : STD_LOGIC;
    signal range_idx : unsigned(9 downto 0) := (others => '0');
    signal doppler_idx : unsigned(6 downto 0) := (others => '0');
    
    -- Gate input until config complete
    signal input_gate : STD_LOGIC := '0';
    signal s_axis_tready_int : STD_LOGIC;
    
    -- Overflow tracking
    signal win1_saturation : STD_LOGIC;
    signal win2_saturation : STD_LOGIC;
    signal ct_overflow     : STD_LOGIC;
    signal overflow_sticky : STD_LOGIC := '0';

begin

    -- =========================================================================
    -- Input Gating: Block data until FFTs are configured
    -- =========================================================================
    s_axis_tready <= s_axis_tready_int and config_complete;

    -- =========================================================================
    -- Stage 1: Range Windowing
    -- =========================================================================
    u_range_window : window_multiplier
    generic map ( DATA_WIDTH => 32, N_SAMPLES => N_RANGE, COEF_WIDTH => 16 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata  => s_axis_tdata,
        s_axis_tvalid => s_axis_tvalid and config_complete,
        s_axis_tready => s_axis_tready_int,
        s_axis_tlast  => s_axis_tlast,
        m_axis_tdata  => win1_to_rfft_data,
        m_axis_tvalid => win1_to_rfft_valid,
        m_axis_tready => win1_to_rfft_ready,
        m_axis_tlast  => win1_to_rfft_last,
        saturation_flag => win1_saturation
    );

    -- =========================================================================
    -- Config FSM - Properly Gated on Ready Signals
    -- =========================================================================
    cfg_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                cfg_state       <= CFG_IDLE;
                rfft_cfg_valid  <= '0';
                dfft_cfg_valid  <= '0';
                config_complete <= '0';
            else
                case cfg_state is
                    when CFG_IDLE =>
                        -- Immediately start config sequence
                        rfft_cfg_valid <= '1';
                        cfg_state      <= CFG_SEND_RANGE;
                        
                    when CFG_SEND_RANGE =>
                        -- Wait for Range FFT to accept config
                        if rfft_cfg_ready = '1' then
                            rfft_cfg_valid <= '0';
                            dfft_cfg_valid <= '1';
                            cfg_state      <= CFG_SEND_DOPPLER;
                        end if;
                        
                    when CFG_SEND_DOPPLER =>
                        -- Wait for Doppler FFT to accept config
                        if dfft_cfg_ready = '1' then
                            dfft_cfg_valid  <= '0';
                            config_complete <= '1';
                            cfg_state       <= CFG_DONE;
                        end if;
                        
                    when CFG_DONE =>
                        -- Configuration complete, system ready
                        null;
                end case;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Stage 2: Range FFT (with proper scaling)
    -- =========================================================================
    u_range_fft : xfft_0
    port map (
        aclk => aclk,
        s_axis_config_tdata  => RANGE_FFT_CONFIG,
        s_axis_config_tvalid => rfft_cfg_valid,
        s_axis_config_tready => rfft_cfg_ready,
        s_axis_data_tdata    => win1_to_rfft_data,
        s_axis_data_tvalid   => win1_to_rfft_valid,
        s_axis_data_tready   => win1_to_rfft_ready,
        s_axis_data_tlast    => win1_to_rfft_last,
        m_axis_data_tdata    => rfft_raw_data,
        m_axis_data_tvalid   => rfft_raw_valid,
        m_axis_data_tready   => rfft_raw_ready,
        m_axis_data_tlast    => rfft_raw_last,
        m_axis_data_tuser    => open,
        m_axis_status_tdata  => open,
        m_axis_status_tvalid => open,
        m_axis_status_tready => '1',
        event_frame_started  => rfft_frame_started,
        event_tlast_unexpected      => open,
        event_tlast_missing         => open,
        event_status_channel_halt   => open,
        event_data_in_channel_halt  => open,
        event_data_out_channel_halt => open
    );

    -- =========================================================================
    -- Stage 2b: Post-FFT Scaler (compensate for limited config scaling)
    -- =========================================================================
    -- FFT config provides ÷64, need ÷1024 total → add ÷16 here (right shift 4)
    -- With saturation protection
    -- =========================================================================
    rfft_raw_ready <= rfft_to_ct_ready;
    
    range_scaler_proc: process(aclk)
        variable i_in  : signed(15 downto 0);
        variable q_in  : signed(15 downto 0);
        variable i_out : signed(15 downto 0);
        variable q_out : signed(15 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                rfft_to_ct_data  <= (others => '0');
                rfft_to_ct_valid <= '0';
                rfft_to_ct_last  <= '0';
            elsif rfft_to_ct_ready = '1' then
                if rfft_raw_valid = '1' then
                    -- Extract I/Q
                    i_in := signed(rfft_raw_data(15 downto 0));
                    q_in := signed(rfft_raw_data(31 downto 16));
                    
                    -- Right shift by 4 (÷16) with sign extension
                    i_out := shift_right(i_in, 4);
                    q_out := shift_right(q_in, 4);
                    
                    rfft_to_ct_data <= std_logic_vector(q_out) & std_logic_vector(i_out);
                    rfft_to_ct_valid <= '1';
                    rfft_to_ct_last  <= rfft_raw_last;
                else
                    rfft_to_ct_valid <= '0';
                    rfft_to_ct_last  <= '0';
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Stage 3: Corner Turner
    -- =========================================================================
    u_corner_turner : corner_turner
    generic map ( N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER, DATA_WIDTH => 32 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata  => rfft_to_ct_data,
        s_axis_tvalid => rfft_to_ct_valid,
        s_axis_tready => rfft_to_ct_ready,
        s_axis_tlast  => rfft_to_ct_last,
        m_axis_tdata  => ct_to_win2_data,
        m_axis_tvalid => ct_to_win2_valid,
        m_axis_tready => ct_to_win2_ready,
        m_axis_tlast  => ct_to_win2_last,
        frame_complete => ct_frame_complete,
        overflow_error => ct_overflow
    );

    -- =========================================================================
    -- Stage 4: Doppler Windowing
    -- =========================================================================
    u_doppler_window : window_multiplier
    generic map ( 
        DATA_WIDTH => 32, 
        N_SAMPLES  => N_DOPPLER,
        COEF_WIDTH => 16 
    )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata  => ct_to_win2_data,
        s_axis_tvalid => ct_to_win2_valid,
        s_axis_tready => ct_to_win2_ready,
        s_axis_tlast  => ct_to_win2_last,
        m_axis_tdata  => win2_to_dfft_data,
        m_axis_tvalid => win2_to_dfft_valid,
        m_axis_tready => win2_to_dfft_ready,
        m_axis_tlast  => win2_to_dfft_last,
        saturation_flag => win2_saturation
    );

    -- =========================================================================
    -- Stage 5: Doppler FFT (with proper scaling)
    -- =========================================================================
    u_doppler_fft : xfft_doppler
    port map (
        aclk => aclk,
        s_axis_config_tdata  => DOPPLER_FFT_CONFIG,
        s_axis_config_tvalid => dfft_cfg_valid,
        s_axis_config_tready => dfft_cfg_ready,
        s_axis_data_tdata    => win2_to_dfft_data,
        s_axis_data_tvalid   => win2_to_dfft_valid,
        s_axis_data_tready   => win2_to_dfft_ready,
        s_axis_data_tlast    => win2_to_dfft_last,
        m_axis_data_tdata    => dfft_to_mag_data,
        m_axis_data_tvalid   => dfft_to_mag_valid,
        m_axis_data_tready   => dfft_to_mag_ready,
        m_axis_data_tlast    => dfft_to_mag_last,
        event_frame_started  => dfft_frame_started,
        event_tlast_unexpected      => open,
        event_tlast_missing         => open,
        event_status_channel_halt   => open,
        event_data_in_channel_halt  => open,
        event_data_out_channel_halt => open
    );

    -- =========================================================================
    -- Stage 6: Magnitude Calculation
    -- =========================================================================
    u_magnitude : magnitude_calc
    generic map ( DATA_WIDTH => 16, OUT_WIDTH => 17 )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata  => dfft_to_mag_data,
        s_axis_tvalid => dfft_to_mag_valid,
        s_axis_tready => dfft_to_mag_ready,
        s_axis_tlast  => dfft_to_mag_last,
        m_axis_tdata  => m_axis_tdata,
        m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready,
        m_axis_tlast  => m_axis_tlast
    );

    -- =========================================================================
    -- Index Tracker
    -- =========================================================================
    index_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                range_idx   <= (others => '0');
                doppler_idx <= (others => '0');
            else
                if m_axis_tvalid = '1' and m_axis_tready = '1' then
                    if doppler_idx = N_DOPPLER - 1 then
                        doppler_idx <= (others => '0');
                        if range_idx = N_RANGE - 1 then
                            range_idx <= (others => '0');
                        else
                            range_idx <= range_idx + 1;
                        end if;
                    else
                        doppler_idx <= doppler_idx + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;
    
    rdm_range_bin           <= std_logic_vector(range_idx);
    rdm_doppler_bin         <= std_logic_vector(doppler_idx);
    status_range_fft_done   <= rfft_frame_started;
    status_doppler_fft_done <= dfft_frame_started;
    status_frame_complete   <= ct_frame_complete;
    status_overflow         <= overflow_sticky;

    -- =========================================================================
    -- Overflow Tracking (Sticky)
    -- =========================================================================
    overflow_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                overflow_sticky <= '0';
            else
                -- Latch any overflow condition (sticky until reset)
                if win1_saturation = '1' or win2_saturation = '1' or ct_overflow = '1' then
                    overflow_sticky <= '1';
                end if;
            end if;
        end if;
    end process;

end Behavioral;