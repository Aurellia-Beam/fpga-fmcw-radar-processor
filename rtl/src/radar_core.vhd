library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity radar_core is
    Port (
        aclk            : in  STD_LOGIC;
        aresetn         : in  STD_LOGIC; -- Used for Window, NOT for FFT
        
        s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_tvalid   : in  STD_LOGIC;
        s_axis_tlast    : in  STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;
        
        m_axis_tdata    : out STD_LOGIC_VECTOR(31 downto 0);
        m_axis_tvalid   : out STD_LOGIC;
        m_axis_tlast    : out STD_LOGIC;
        m_axis_tready   : in  STD_LOGIC
    );
end radar_core;

architecture Behavioral of radar_core is

    component window_multiplier is
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

    -- COMPONENT: FFT (Matches your VHO exactly)
    COMPONENT xfft_0
      PORT (
        aclk : IN STD_LOGIC;
        
        -- Config
        s_axis_config_tdata : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
        s_axis_config_tvalid : IN STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        
        -- Data In
        s_axis_data_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
        s_axis_data_tvalid : IN STD_LOGIC;
        s_axis_data_tready : OUT STD_LOGIC;
        s_axis_data_tlast : IN STD_LOGIC;
        
        -- Data Out
        m_axis_data_tdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
        m_axis_data_tuser : OUT STD_LOGIC_VECTOR(7 DOWNTO 0); -- ADDED (From your VHO)
        m_axis_data_tvalid : OUT STD_LOGIC;
        m_axis_data_tready : IN STD_LOGIC;
        m_axis_data_tlast : OUT STD_LOGIC;
        
        -- Status
        m_axis_status_tdata : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        m_axis_status_tvalid : OUT STD_LOGIC;
        m_axis_status_tready : IN STD_LOGIC;
        
        -- Events
        event_frame_started : OUT STD_LOGIC;
        event_tlast_unexpected : OUT STD_LOGIC;
        event_tlast_missing : OUT STD_LOGIC;
        event_status_channel_halt : OUT STD_LOGIC;
        event_data_in_channel_halt : OUT STD_LOGIC;
        event_data_out_channel_halt : OUT STD_LOGIC
      );
    END COMPONENT;

    signal window_out_data   : STD_LOGIC_VECTOR(31 downto 0);
    signal window_out_valid  : STD_LOGIC;
    signal window_out_ready  : STD_LOGIC;
    signal window_out_last   : STD_LOGIC;
    
    -- Config Signals
    signal cfg_tvalid : STD_LOGIC := '0';
    signal cfg_tready : STD_LOGIC;
    type state_type is (IDLE, SEND_CONFIG, DONE);
    signal state : state_type := IDLE;

begin

    u_window : window_multiplier
    Port Map (
        aclk          => aclk,
        aresetn       => aresetn,
        s_axis_tdata  => s_axis_tdata,
        s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready,
        s_axis_tlast  => s_axis_tlast,
        m_axis_tdata  => window_out_data,
        m_axis_tvalid => window_out_valid,
        m_axis_tready => window_out_ready,
        m_axis_tlast  => window_out_last
    );

    -- PROCESS: Single-Shot Configuration
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

    u_fft : xfft_0
    PORT MAP (
        aclk => aclk,
        
        -- Config
        s_axis_config_tdata => x"01",
        s_axis_config_tvalid => cfg_tvalid,
        s_axis_config_tready => cfg_tready,
        
        -- Data In
        s_axis_data_tdata => window_out_data,
        s_axis_data_tvalid => window_out_valid,
        s_axis_data_tready => window_out_ready,
        s_axis_data_tlast => window_out_last,
        
        -- Data Out
        m_axis_data_tdata => m_axis_tdata,
        m_axis_data_tuser => open, -- We ignore the exponent for now
        m_axis_data_tvalid => m_axis_tvalid,
        m_axis_data_tready => m_axis_tready,
        m_axis_data_tlast => m_axis_tlast,
        
        -- Status
        m_axis_status_tdata => open,
        m_axis_status_tvalid => open,
        m_axis_status_tready => '1', -- Always Ready
        
        -- Events
        event_frame_started => open,
        event_tlast_unexpected => open,
        event_tlast_missing => open,
        event_status_channel_halt => open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt => open
    );

end Behavioral;