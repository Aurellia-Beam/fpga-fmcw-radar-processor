library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- FPGA FMCW Radar Core - Updated with Corner Turner
-- Pipeline: Window → Range FFT → Corner Turner → (Future: Doppler FFT → CFAR)

entity radar_core_v2 is
    Generic (
        N_RANGE   : integer := 1024;
        N_DOPPLER : integer := 128
    );
    Port (
        aclk            : in  STD_LOGIC;
        aresetn         : in  STD_LOGIC;
        
        -- Input: Raw ADC samples (I/Q)
        s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_tvalid   : in  STD_LOGIC;
        s_axis_tlast    : in  STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;
        
        -- Output: Transposed data ready for Doppler FFT
        m_axis_tdata    : out STD_LOGIC_VECTOR(31 downto 0);
        m_axis_tvalid   : out STD_LOGIC;
        m_axis_tlast    : out STD_LOGIC;
        m_axis_tready   : in  STD_LOGIC;
        
        -- Status outputs
        range_fft_ready : out STD_LOGIC;
        corner_turn_active : out STD_LOGIC
    );
end radar_core_v2;

architecture Behavioral of radar_core_v2 is

    -- ========================================================================
    -- Component Declarations
    -- ========================================================================
    
    component window_multiplier is
        Generic ( DATA_WIDTH : integer := 32 );
        Port (
            aclk          : in  STD_LOGIC;
            aresetn       : in  STD_LOGIC;
            s_axis_tdata  : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid : in  STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast  : in  STD_LOGIC;
            m_axis_tdata  : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in  STD_LOGIC;
            m_axis_tlast  : out STD_LOGIC
        );
    end component;

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
    
    component corner_turner is
        Generic (
            N_RANGE   : integer := 1024;
            N_DOPPLER : integer := 128;
            DATA_WIDTH : integer := 32
        );
        Port (
            aclk      : in  STD_LOGIC;
            aresetn   : in  STD_LOGIC;
            s_axis_tdata  : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid : in  STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast  : in  STD_LOGIC;
            m_axis_tdata  : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in  STD_LOGIC;
            m_axis_tlast  : out STD_LOGIC
        );
    end component;

    -- ========================================================================
    -- Inter-stage signals
    -- ========================================================================
    
    -- Window → Range FFT
    signal window_to_fft_data   : STD_LOGIC_VECTOR(31 downto 0);
    signal window_to_fft_valid  : STD_LOGIC;
    signal window_to_fft_ready  : STD_LOGIC;
    signal window_to_fft_last   : STD_LOGIC;
    
    -- Range FFT → Corner Turner
    signal fft_to_corner_data   : STD_LOGIC_VECTOR(31 downto 0);
    signal fft_to_corner_valid  : STD_LOGIC;
    signal fft_to_corner_ready  : STD_LOGIC;
    signal fft_to_corner_last   : STD_LOGIC;
    
    -- FFT Config
    signal cfg_tvalid : STD_LOGIC := '0';
    signal cfg_tready : STD_LOGIC;
    
    type state_type is (IDLE, SEND_CONFIG, DONE);
    signal state : state_type := IDLE;

begin

    -- ========================================================================
    -- Stage 1: Window Multiplier (Hamming)
    -- ========================================================================
    u_window : window_multiplier
    port map (
        aclk          => aclk,
        aresetn       => aresetn,
        s_axis_tdata  => s_axis_tdata,
        s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready,
        s_axis_tlast  => s_axis_tlast,
        m_axis_tdata  => window_to_fft_data,
        m_axis_tvalid => window_to_fft_valid,
        m_axis_tready => window_to_fft_ready,
        m_axis_tlast  => window_to_fft_last
    );

    -- ========================================================================
    -- FFT Configuration (One-time setup)
    -- ========================================================================
    process(aclk)
        variable startup_count : integer := 0;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                state <= IDLE;
                cfg_tvalid <= '0';
                startup_count := 0;
            else
                case state is
                    when IDLE =>
                        if startup_count < 50 then
                            startup_count := startup_count + 1;
                        else
                            state <= SEND_CONFIG;
                            cfg_tvalid <= '1';
                        end if;
                    when SEND_CONFIG =>
                        if cfg_tready = '1' then
                            cfg_tvalid <= '0';
                            state <= DONE;
                        end if;
                    when DONE =>
                        cfg_tvalid <= '0';
                end case;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Stage 2: Range FFT (1024-point)
    -- ========================================================================
    u_range_fft : xfft_0
    port map (
        aclk => aclk,
        
        -- Config: Forward FFT
        s_axis_config_tdata => x"01",
        s_axis_config_tvalid => cfg_tvalid,
        s_axis_config_tready => cfg_tready,
        
        -- Data In from Window
        s_axis_data_tdata => window_to_fft_data,
        s_axis_data_tvalid => window_to_fft_valid,
        s_axis_data_tready => window_to_fft_ready,
        s_axis_data_tlast => window_to_fft_last,
        
        -- Data Out to Corner Turner
        m_axis_data_tdata => fft_to_corner_data,
        m_axis_data_tuser => open,
        m_axis_data_tvalid => fft_to_corner_valid,
        m_axis_data_tready => fft_to_corner_ready,
        m_axis_data_tlast => fft_to_corner_last,
        
        -- Status
        m_axis_status_tdata => open,
        m_axis_status_tvalid => open,
        m_axis_status_tready => '1',
        
        -- Events
        event_frame_started => range_fft_ready,
        event_tlast_unexpected => open,
        event_tlast_missing => open,
        event_status_channel_halt => open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt => open
    );

    -- ========================================================================
    -- Stage 3: Corner Turner (Matrix Transpose)
    -- ========================================================================
    u_corner_turner : corner_turner
    generic map (
        N_RANGE => N_RANGE,
        N_DOPPLER => N_DOPPLER,
        DATA_WIDTH => 32
    )
    port map (
        aclk => aclk,
        aresetn => aresetn,
        
        -- Input from Range FFT
        s_axis_tdata  => fft_to_corner_data,
        s_axis_tvalid => fft_to_corner_valid,
        s_axis_tready => fft_to_corner_ready,
        s_axis_tlast  => fft_to_corner_last,
        
        -- Output (column-wise, ready for Doppler FFT)
        m_axis_tdata  => m_axis_tdata,
        m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready,
        m_axis_tlast  => m_axis_tlast
    );
    
    -- Status output (indicates data flowing through corner turner)
    corner_turn_active <= fft_to_corner_valid;

end Behavioral;
