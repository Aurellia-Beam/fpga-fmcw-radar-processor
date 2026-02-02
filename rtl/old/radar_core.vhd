library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================================
-- FMCW Radar Signal Processing Core (Basic)
-- =============================================================================
-- Simple pipeline: Window → Range FFT
-- Use radar_core_v2 for complete Range-Doppler processing
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity radar_core is
    Generic (
        N_SAMPLES : integer := 1024  -- FFT size / samples per chirp
    );
    Port (
        aclk            : in  STD_LOGIC;
        aresetn         : in  STD_LOGIC;
        
        -- AXI-Stream Slave: Raw ADC samples
        s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_tvalid   : in  STD_LOGIC;
        s_axis_tlast    : in  STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;
        
        -- AXI-Stream Master: Range FFT output
        m_axis_tdata    : out STD_LOGIC_VECTOR(31 downto 0);
        m_axis_tvalid   : out STD_LOGIC;
        m_axis_tlast    : out STD_LOGIC;
        m_axis_tready   : in  STD_LOGIC;
        
        -- Status
        fft_config_done : out STD_LOGIC
    );
end radar_core;

architecture Behavioral of radar_core is

    -- =========================================================================
    -- Components
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

    -- Xilinx FFT IP
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

    -- =========================================================================
    -- Internal Signals
    -- =========================================================================
    -- Window → FFT
    signal window_out_data  : STD_LOGIC_VECTOR(31 downto 0);
    signal window_out_valid : STD_LOGIC;
    signal window_out_ready : STD_LOGIC;
    signal window_out_last  : STD_LOGIC;
    signal window_sat       : STD_LOGIC;
    
    -- FFT Config
    type cfg_state_t is (CFG_IDLE, CFG_WAIT, CFG_SEND, CFG_DONE);
    signal cfg_state  : cfg_state_t := CFG_IDLE;
    signal cfg_tvalid : STD_LOGIC := '0';
    signal cfg_tready : STD_LOGIC;
    signal cfg_done   : STD_LOGIC := '0';
    signal wait_cnt   : unsigned(5 downto 0) := (others => '0');

begin

    -- =========================================================================
    -- Stage 1: Hamming Window
    -- =========================================================================
    u_window : window_multiplier
    generic map (
        DATA_WIDTH => 32,
        N_SAMPLES  => N_SAMPLES,
        COEF_WIDTH => 16
    )
    port map (
        aclk            => aclk,
        aresetn         => aresetn,
        s_axis_tdata    => s_axis_tdata,
        s_axis_tvalid   => s_axis_tvalid,
        s_axis_tready   => s_axis_tready,
        s_axis_tlast    => s_axis_tlast,
        m_axis_tdata    => window_out_data,
        m_axis_tvalid   => window_out_valid,
        m_axis_tready   => window_out_ready,
        m_axis_tlast    => window_out_last,
        saturation_flag => window_sat
    );

    -- =========================================================================
    -- FFT Configuration State Machine
    -- =========================================================================
    cfg_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                cfg_state  <= CFG_IDLE;
                cfg_tvalid <= '0';
                cfg_done   <= '0';
                wait_cnt   <= (others => '0');
            else
                case cfg_state is
                    when CFG_IDLE =>
                        cfg_state <= CFG_WAIT;
                        wait_cnt  <= (others => '0');
                        
                    when CFG_WAIT =>
                        if wait_cnt = 63 then
                            cfg_state  <= CFG_SEND;
                            cfg_tvalid <= '1';
                        else
                            wait_cnt <= wait_cnt + 1;
                        end if;
                        
                    when CFG_SEND =>
                        if cfg_tready = '1' then
                            cfg_tvalid <= '0';
                            cfg_done   <= '1';
                            cfg_state  <= CFG_DONE;
                        end if;
                        
                    when CFG_DONE =>
                        null;
                end case;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Stage 2: Range FFT
    -- =========================================================================
    u_fft : xfft_0
    port map (
        aclk => aclk,
        
        -- Config: Forward FFT (0x01)
        s_axis_config_tdata  => x"01",
        s_axis_config_tvalid => cfg_tvalid,
        s_axis_config_tready => cfg_tready,
        
        -- Data In
        s_axis_data_tdata    => window_out_data,
        s_axis_data_tvalid   => window_out_valid,
        s_axis_data_tready   => window_out_ready,
        s_axis_data_tlast    => window_out_last,
        
        -- Data Out
        m_axis_data_tdata    => m_axis_tdata,
        m_axis_data_tuser    => open,
        m_axis_data_tvalid   => m_axis_tvalid,
        m_axis_data_tready   => m_axis_tready,
        m_axis_data_tlast    => m_axis_tlast,
        
        -- Status
        m_axis_status_tdata  => open,
        m_axis_status_tvalid => open,
        m_axis_status_tready => '1',
        
        -- Events
        event_frame_started         => open,
        event_tlast_unexpected      => open,
        event_tlast_missing         => open,
        event_status_channel_halt   => open,
        event_data_in_channel_halt  => open,
        event_data_out_channel_halt => open
    );

    -- Status output
    fft_config_done <= cfg_done;

end Behavioral;
