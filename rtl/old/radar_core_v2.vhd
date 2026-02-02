library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================================
-- FMCW Radar Signal Processing Core v2
-- =============================================================================
-- Complete Range-Doppler processing pipeline:
--   Stage 1: Hamming Window (spectral leakage reduction)
--   Stage 2: Range FFT (1024-point, per-chirp processing)
--   Stage 3: Corner Turner (matrix transpose for Doppler processing)
--   Stage 4: [Future] Doppler FFT
--   Stage 5: [Future] CFAR Detection
--
-- Timing Budget (@ 200 MHz, 1024 range bins, 128 chirps):
--   - Chirp period: 5.12 µs (1024 samples)
--   - CPI duration: 655.36 µs (128 chirps)
--   - Latency: 1 CPI (first frame fill) + pipeline stages
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity radar_core_v2 is
    Generic (
        N_RANGE   : integer := 1024;   -- Range bins per chirp
        N_DOPPLER : integer := 128;    -- Chirps per CPI
        DATA_WIDTH : integer := 32     -- I/Q packed width
    );
    Port (
        aclk            : in  STD_LOGIC;
        aresetn         : in  STD_LOGIC;
        
        -- AXI-Stream Slave: Raw ADC samples (I/Q)
        s_axis_tdata    : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        s_axis_tvalid   : in  STD_LOGIC;
        s_axis_tlast    : in  STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;
        
        -- AXI-Stream Master: Transposed data for Doppler FFT
        m_axis_tdata    : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        m_axis_tvalid   : out STD_LOGIC;
        m_axis_tlast    : out STD_LOGIC;
        m_axis_tready   : in  STD_LOGIC;
        
        -- Status/Debug Interface
        status_fft_config_done : out STD_LOGIC;  -- FFT configured successfully
        status_frame_complete  : out STD_LOGIC;  -- Pulse: CPI frame complete
        status_window_saturate : out STD_LOGIC;  -- Window multiplier overflow
        status_corner_overflow : out STD_LOGIC;  -- Corner turner overflow
        
        -- Diagnostic counters (active chirp/sample)
        diag_chirp_count  : out STD_LOGIC_VECTOR(7 downto 0);
        diag_sample_count : out STD_LOGIC_VECTOR(9 downto 0)
    );
end radar_core_v2;

architecture Behavioral of radar_core_v2 is

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

    -- Xilinx FFT IP (generated from Vivado)
    COMPONENT xfft_0
      PORT (
        aclk                        : IN  STD_LOGIC;
        s_axis_config_tdata         : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
        s_axis_config_tvalid        : IN  STD_LOGIC;
        s_axis_config_tready        : OUT STD_LOGIC;
        s_axis_data_tdata           : IN  STD_LOGIC_VECTOR(31 DOWNTO 0);
        s_axis_data_tvalid          : IN  STD_LOGIC;
        s_axis_data_tready          : OUT STD_LOGIC;
        s_axis_data_tlast           : IN  STD_LOGIC;
        m_axis_data_tdata           : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
        m_axis_data_tuser           : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        m_axis_data_tvalid          : OUT STD_LOGIC;
        m_axis_data_tready          : IN  STD_LOGIC;
        m_axis_data_tlast           : OUT STD_LOGIC;
        m_axis_status_tdata         : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        m_axis_status_tvalid        : OUT STD_LOGIC;
        m_axis_status_tready        : IN  STD_LOGIC;
        event_frame_started         : OUT STD_LOGIC;
        event_tlast_unexpected      : OUT STD_LOGIC;
        event_tlast_missing         : OUT STD_LOGIC;
        event_status_channel_halt   : OUT STD_LOGIC;
        event_data_in_channel_halt  : OUT STD_LOGIC;
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

    -- =========================================================================
    -- Inter-Stage Signals
    -- =========================================================================
    
    -- Window → Range FFT
    signal win_to_fft_data   : STD_LOGIC_VECTOR(31 downto 0);
    signal win_to_fft_valid  : STD_LOGIC;
    signal win_to_fft_ready  : STD_LOGIC;
    signal win_to_fft_last   : STD_LOGIC;
    signal window_sat        : STD_LOGIC;
    
    -- Range FFT → Corner Turner
    signal fft_to_ct_data    : STD_LOGIC_VECTOR(31 downto 0);
    signal fft_to_ct_valid   : STD_LOGIC;
    signal fft_to_ct_ready   : STD_LOGIC;
    signal fft_to_ct_last    : STD_LOGIC;
    
    -- FFT Events
    signal fft_frame_started : STD_LOGIC;
    signal fft_tlast_unexpected : STD_LOGIC;
    signal fft_tlast_missing : STD_LOGIC;
    
    -- Corner Turner Status
    signal ct_frame_complete : STD_LOGIC;
    signal ct_overflow       : STD_LOGIC;

    -- =========================================================================
    -- FFT Configuration State Machine
    -- =========================================================================
    type fft_config_state_t is (CFG_IDLE, CFG_WAIT_RESET, CFG_SEND, CFG_DONE);
    signal fft_cfg_state : fft_config_state_t := CFG_IDLE;
    signal cfg_tvalid    : STD_LOGIC := '0';
    signal cfg_tready    : STD_LOGIC;
    signal cfg_done      : STD_LOGIC := '0';
    signal reset_cnt     : unsigned(5 downto 0) := (others => '0');
    
    -- Diagnostic counters
    signal chirp_cnt  : unsigned(7 downto 0) := (others => '0');
    signal sample_cnt : unsigned(9 downto 0) := (others => '0');

begin

    -- =========================================================================
    -- Stage 1: Hamming Window
    -- =========================================================================
    u_window : window_multiplier
    generic map (
        DATA_WIDTH => DATA_WIDTH,
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
        m_axis_tdata    => win_to_fft_data,
        m_axis_tvalid   => win_to_fft_valid,
        m_axis_tready   => win_to_fft_ready,
        m_axis_tlast    => win_to_fft_last,
        saturation_flag => window_sat
    );

    -- =========================================================================
    -- FFT Configuration State Machine
    -- =========================================================================
    -- Waits for reset deassertion, then sends forward FFT config once
    -- =========================================================================
    fft_config_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                fft_cfg_state <= CFG_IDLE;
                cfg_tvalid    <= '0';
                cfg_done      <= '0';
                reset_cnt     <= (others => '0');
            else
                case fft_cfg_state is
                    when CFG_IDLE =>
                        -- Wait for stable reset deassertion
                        fft_cfg_state <= CFG_WAIT_RESET;
                        reset_cnt     <= (others => '0');
                        
                    when CFG_WAIT_RESET =>
                        -- Hold-off period after reset
                        if reset_cnt = 63 then
                            fft_cfg_state <= CFG_SEND;
                            cfg_tvalid    <= '1';
                        else
                            reset_cnt <= reset_cnt + 1;
                        end if;
                        
                    when CFG_SEND =>
                        -- Wait for FFT to accept config
                        if cfg_tready = '1' then
                            cfg_tvalid    <= '0';
                            cfg_done      <= '1';
                            fft_cfg_state <= CFG_DONE;
                        end if;
                        
                    when CFG_DONE =>
                        -- Configuration complete, idle
                        cfg_tvalid <= '0';
                end case;
            end if;
        end if;
    end process fft_config_proc;

    -- =========================================================================
    -- Stage 2: Range FFT (1024-point Forward FFT)
    -- =========================================================================
    u_range_fft : xfft_0
    port map (
        aclk => aclk,
        
        -- Config: 0x01 = Forward FFT
        s_axis_config_tdata  => x"01",
        s_axis_config_tvalid => cfg_tvalid,
        s_axis_config_tready => cfg_tready,
        
        -- Data In from Window
        s_axis_data_tdata    => win_to_fft_data,
        s_axis_data_tvalid   => win_to_fft_valid,
        s_axis_data_tready   => win_to_fft_ready,
        s_axis_data_tlast    => win_to_fft_last,
        
        -- Data Out to Corner Turner
        m_axis_data_tdata    => fft_to_ct_data,
        m_axis_data_tuser    => open,  -- Block exponent (could use for scaling)
        m_axis_data_tvalid   => fft_to_ct_valid,
        m_axis_data_tready   => fft_to_ct_ready,
        m_axis_data_tlast    => fft_to_ct_last,
        
        -- Status (always accept)
        m_axis_status_tdata  => open,
        m_axis_status_tvalid => open,
        m_axis_status_tready => '1',
        
        -- Events
        event_frame_started         => fft_frame_started,
        event_tlast_unexpected      => fft_tlast_unexpected,
        event_tlast_missing         => fft_tlast_missing,
        event_status_channel_halt   => open,
        event_data_in_channel_halt  => open,
        event_data_out_channel_halt => open
    );

    -- =========================================================================
    -- Stage 3: Corner Turner (Matrix Transpose)
    -- =========================================================================
    u_corner_turner : corner_turner
    generic map (
        N_RANGE    => N_RANGE,
        N_DOPPLER  => N_DOPPLER,
        DATA_WIDTH => DATA_WIDTH
    )
    port map (
        aclk           => aclk,
        aresetn        => aresetn,
        s_axis_tdata   => fft_to_ct_data,
        s_axis_tvalid  => fft_to_ct_valid,
        s_axis_tready  => fft_to_ct_ready,
        s_axis_tlast   => fft_to_ct_last,
        m_axis_tdata   => m_axis_tdata,
        m_axis_tvalid  => m_axis_tvalid,
        m_axis_tready  => m_axis_tready,
        m_axis_tlast   => m_axis_tlast,
        frame_complete => ct_frame_complete,
        overflow_error => ct_overflow
    );

    -- =========================================================================
    -- Diagnostic Counter
    -- =========================================================================
    diag_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                chirp_cnt  <= (others => '0');
                sample_cnt <= (others => '0');
            else
                -- Track input samples
                if s_axis_tvalid = '1' and s_axis_tready = '1' then
                    if s_axis_tlast = '1' then
                        sample_cnt <= (others => '0');
                        if chirp_cnt = N_DOPPLER - 1 then
                            chirp_cnt <= (others => '0');
                        else
                            chirp_cnt <= chirp_cnt + 1;
                        end if;
                    else
                        sample_cnt <= sample_cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process diag_proc;

    -- =========================================================================
    -- Status Outputs
    -- =========================================================================
    status_fft_config_done <= cfg_done;
    status_frame_complete  <= ct_frame_complete;
    status_window_saturate <= window_sat;
    status_corner_overflow <= ct_overflow;
    diag_chirp_count       <= std_logic_vector(chirp_cnt);
    diag_sample_count      <= std_logic_vector(sample_cnt);

end Behavioral;
