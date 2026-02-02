library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================================
-- FMCW Radar Signal Processing Core v3 - Complete RDM Pipeline
-- =============================================================================
-- Full Range-Doppler Map generation:
--   Stage 1: Hamming Window (spectral leakage reduction)
--   Stage 2: Range FFT (1024-point, per-chirp)
--   Stage 3: Corner Turner (matrix transpose)
--   Stage 4: Doppler FFT (128-point, per-range-bin)
--   Stage 5: Magnitude (Alpha-Max Beta-Min approximation)
--
-- Output: Range-Doppler Map magnitude values, ready for CFAR detection
--
-- Required IP:
--   - xfft_0: 1024-point FFT (Range)
--   - xfft_doppler: 128-point FFT (Doppler) -- GENERATE THIS IN VIVADO
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity radar_core_v3 is
    Generic (
        N_RANGE   : integer := 1024;   -- Range bins per chirp
        N_DOPPLER : integer := 128     -- Chirps per CPI (Doppler bins)
    );
    Port (
        aclk            : in  STD_LOGIC;
        aresetn         : in  STD_LOGIC;
        
        -- AXI-Stream Slave: Raw ADC samples (I/Q)
        s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_tvalid   : in  STD_LOGIC;
        s_axis_tlast    : in  STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;
        
        -- AXI-Stream Master: RDM magnitude output (17-bit)
        m_axis_tdata    : out STD_LOGIC_VECTOR(16 downto 0);
        m_axis_tvalid   : out STD_LOGIC;
        m_axis_tlast    : out STD_LOGIC;
        m_axis_tready   : in  STD_LOGIC;
        
        -- Range-Doppler Map indices (active during valid output)
        rdm_range_bin   : out STD_LOGIC_VECTOR(9 downto 0);   -- 0 to 1023
        rdm_doppler_bin : out STD_LOGIC_VECTOR(6 downto 0);   -- 0 to 127
        
        -- Status
        status_range_fft_done   : out STD_LOGIC;
        status_doppler_fft_done : out STD_LOGIC;
        status_frame_complete   : out STD_LOGIC
    );
end radar_core_v3;

architecture Behavioral of radar_core_v3 is

    -- =========================================================================
    -- Component Declarations
    -- =========================================================================
    
    component window_multiplier is
        Generic (
            DATA_WIDTH : integer := 32;
            N_SAMPLES  : integer := 1024;
            COEF_WIDTH : integer := 16
        );
        Port (
            aclk            : in  STD_LOGIC;
            aresetn         : in  STD_LOGIC;
            s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid   : in  STD_LOGIC;
            s_axis_tready   : out STD_LOGIC;
            s_axis_tlast    : in  STD_LOGIC;
            m_axis_tdata    : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid   : out STD_LOGIC;
            m_axis_tready   : in  STD_LOGIC;
            m_axis_tlast    : out STD_LOGIC;
            saturation_flag : out STD_LOGIC
        );
    end component;

    -- 1024-point Range FFT (Xilinx IP)
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

    -- 128-point Doppler FFT (Xilinx IP)
    COMPONENT xfft_doppler
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
        Generic (
            N_RANGE    : integer := 1024;
            N_DOPPLER  : integer := 128;
            DATA_WIDTH : integer := 32
        );
        Port (
            aclk           : in  STD_LOGIC;
            aresetn        : in  STD_LOGIC;
            s_axis_tdata   : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid  : in  STD_LOGIC;
            s_axis_tready  : out STD_LOGIC;
            s_axis_tlast   : in  STD_LOGIC;
            m_axis_tdata   : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid  : out STD_LOGIC;
            m_axis_tready  : in  STD_LOGIC;
            m_axis_tlast   : out STD_LOGIC;
            frame_complete : out STD_LOGIC;
            overflow_error : out STD_LOGIC
        );
    end component;
    
    component magnitude_calc is
        Generic (
            DATA_WIDTH : integer := 16;
            OUT_WIDTH  : integer := 17
        );
        Port (
            aclk          : in  STD_LOGIC;
            aresetn       : in  STD_LOGIC;
            s_axis_tdata  : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid : in  STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast  : in  STD_LOGIC;
            m_axis_tdata  : out STD_LOGIC_VECTOR(16 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in  STD_LOGIC;
            m_axis_tlast  : out STD_LOGIC
        );
    end component;

    -- =========================================================================
    -- Inter-Stage Signals
    -- =========================================================================
    
    -- Window → Range FFT
    signal win_to_rfft_data   : STD_LOGIC_VECTOR(31 downto 0);
    signal win_to_rfft_valid  : STD_LOGIC;
    signal win_to_rfft_ready  : STD_LOGIC;
    signal win_to_rfft_last   : STD_LOGIC;
    
    -- Range FFT → Corner Turner
    signal rfft_to_ct_data    : STD_LOGIC_VECTOR(31 downto 0);
    signal rfft_to_ct_valid   : STD_LOGIC;
    signal rfft_to_ct_ready   : STD_LOGIC;
    signal rfft_to_ct_last    : STD_LOGIC;
    
    -- Corner Turner → Doppler FFT
    signal ct_to_dfft_data    : STD_LOGIC_VECTOR(31 downto 0);
    signal ct_to_dfft_valid   : STD_LOGIC;
    signal ct_to_dfft_ready   : STD_LOGIC;
    signal ct_to_dfft_last    : STD_LOGIC;
    
    -- Doppler FFT → Magnitude
    signal dfft_to_mag_data   : STD_LOGIC_VECTOR(31 downto 0);
    signal dfft_to_mag_valid  : STD_LOGIC;
    signal dfft_to_mag_ready  : STD_LOGIC;
    signal dfft_to_mag_last   : STD_LOGIC;
    
    -- FFT Configuration
    type cfg_state_t is (CFG_IDLE, CFG_WAIT, CFG_SEND_RANGE, CFG_SEND_DOPPLER, CFG_DONE);
    signal cfg_state       : cfg_state_t := CFG_IDLE;
    signal rfft_cfg_valid  : STD_LOGIC := '0';
    signal rfft_cfg_ready  : STD_LOGIC;
    signal dfft_cfg_valid  : STD_LOGIC := '0';
    signal dfft_cfg_ready  : STD_LOGIC;
    signal wait_cnt        : unsigned(5 downto 0) := (others => '0');
    
    -- Status signals
    signal ct_frame_complete : STD_LOGIC;
    signal rfft_frame_started : STD_LOGIC;
    signal dfft_frame_started : STD_LOGIC;
    
    -- RDM index tracking
    signal range_idx   : unsigned(9 downto 0) := (others => '0');
    signal doppler_idx : unsigned(6 downto 0) := (others => '0');

begin

    -- =========================================================================
    -- Stage 1: Hamming Window
    -- =========================================================================
    u_window : window_multiplier
    generic map (
        DATA_WIDTH => 32,
        N_SAMPLES  => N_RANGE,
        COEF_WIDTH => 16
    )
    port map (
        aclk            => aclk,
        aresetn         => aresetn,
        s_axis_tdata    => s_axis_tdata,
        s_axis_tvalid   => s_axis_tvalid,
        s_axis_tready   => s_axis_tready,
        s_axis_tlast    => s_axis_tlast,
        m_axis_tdata    => win_to_rfft_data,
        m_axis_tvalid   => win_to_rfft_valid,
        m_axis_tready   => win_to_rfft_ready,
        m_axis_tlast    => win_to_rfft_last,
        saturation_flag => open
    );

    -- =========================================================================
    -- FFT Configuration State Machine
    -- =========================================================================
    -- Configures both Range and Doppler FFTs after reset
    -- =========================================================================
    cfg_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                cfg_state      <= CFG_IDLE;
                rfft_cfg_valid <= '0';
                dfft_cfg_valid <= '0';
                wait_cnt       <= (others => '0');
            else
                case cfg_state is
                    when CFG_IDLE =>
                        cfg_state <= CFG_WAIT;
                        wait_cnt  <= (others => '0');
                        
                    when CFG_WAIT =>
                        if wait_cnt = 63 then
                            cfg_state      <= CFG_SEND_RANGE;
                            rfft_cfg_valid <= '1';
                        else
                            wait_cnt <= wait_cnt + 1;
                        end if;
                        
                    when CFG_SEND_RANGE =>
                        if rfft_cfg_ready = '1' then
                            rfft_cfg_valid <= '0';
                            dfft_cfg_valid <= '1';
                            cfg_state      <= CFG_SEND_DOPPLER;
                        end if;
                        
                    when CFG_SEND_DOPPLER =>
                        if dfft_cfg_ready = '1' then
                            dfft_cfg_valid <= '0';
                            cfg_state      <= CFG_DONE;
                        end if;
                        
                    when CFG_DONE =>
                        null;
                end case;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Stage 2: Range FFT (1024-point)
    -- =========================================================================
    u_range_fft : xfft_0
    port map (
        aclk => aclk,
        
        s_axis_config_tdata  => x"01",  -- Forward FFT
        s_axis_config_tvalid => rfft_cfg_valid,
        s_axis_config_tready => rfft_cfg_ready,
        
        s_axis_data_tdata    => win_to_rfft_data,
        s_axis_data_tvalid   => win_to_rfft_valid,
        s_axis_data_tready   => win_to_rfft_ready,
        s_axis_data_tlast    => win_to_rfft_last,
        
        m_axis_data_tdata    => rfft_to_ct_data,
        m_axis_data_tvalid   => rfft_to_ct_valid,
        m_axis_data_tready   => rfft_to_ct_ready,
        m_axis_data_tlast    => rfft_to_ct_last,
        
        event_frame_started         => rfft_frame_started,
        event_tlast_unexpected      => open,
        event_tlast_missing         => open,
        event_status_channel_halt   => open,
        event_data_in_channel_halt  => open,
        event_data_out_channel_halt => open
    );

    -- =========================================================================
    -- Stage 3: Corner Turner (1024 x 128 Matrix Transpose)
    -- =========================================================================
    u_corner_turner : corner_turner
    generic map (
        N_RANGE    => N_RANGE,
        N_DOPPLER  => N_DOPPLER,
        DATA_WIDTH => 32
    )
    port map (
        aclk           => aclk,
        aresetn        => aresetn,
        s_axis_tdata   => rfft_to_ct_data,
        s_axis_tvalid  => rfft_to_ct_valid,
        s_axis_tready  => rfft_to_ct_ready,
        s_axis_tlast   => rfft_to_ct_last,
        m_axis_tdata   => ct_to_dfft_data,
        m_axis_tvalid  => ct_to_dfft_valid,
        m_axis_tready  => ct_to_dfft_ready,
        m_axis_tlast   => ct_to_dfft_last,
        frame_complete => ct_frame_complete,
        overflow_error => open
    );

    -- =========================================================================
    -- Stage 4: Doppler FFT (128-point)
    -- =========================================================================
    u_doppler_fft : xfft_doppler
    port map (
        aclk => aclk,
        
        s_axis_config_tdata  => x"01",  -- Forward FFT
        s_axis_config_tvalid => dfft_cfg_valid,
        s_axis_config_tready => dfft_cfg_ready,
        
        s_axis_data_tdata    => ct_to_dfft_data,
        s_axis_data_tvalid   => ct_to_dfft_valid,
        s_axis_data_tready   => ct_to_dfft_ready,
        s_axis_data_tlast    => ct_to_dfft_last,
        
        m_axis_data_tdata    => dfft_to_mag_data,
        m_axis_data_tvalid   => dfft_to_mag_valid,
        m_axis_data_tready   => dfft_to_mag_ready,
        m_axis_data_tlast    => dfft_to_mag_last,
        
        event_frame_started         => dfft_frame_started,
        event_tlast_unexpected      => open,
        event_tlast_missing         => open,
        event_status_channel_halt   => open,
        event_data_in_channel_halt  => open,
        event_data_out_channel_halt => open
    );

    -- =========================================================================
    -- Stage 5: Magnitude Calculation (Alpha-Max Beta-Min)
    -- =========================================================================
    u_magnitude : magnitude_calc
    generic map (
        DATA_WIDTH => 16,
        OUT_WIDTH  => 17
    )
    port map (
        aclk          => aclk,
        aresetn       => aresetn,
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
    -- RDM Index Tracker
    -- =========================================================================
    -- Tracks which range/Doppler bin is currently being output
    -- Output order: for each range_bin, all doppler_bins (column-major from CT)
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
    
    rdm_range_bin   <= std_logic_vector(range_idx);
    rdm_doppler_bin <= std_logic_vector(doppler_idx);

    -- =========================================================================
    -- Status Outputs
    -- =========================================================================
    status_range_fft_done   <= rfft_frame_started;
    status_doppler_fft_done <= dfft_frame_started;
    status_frame_complete   <= ct_frame_complete;

end Behavioral;