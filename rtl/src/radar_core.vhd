library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity radar_core is
    Port (
        aclk            : in  STD_LOGIC;
        aresetn         : in  STD_LOGIC;
        
        -- Slave Interface (Input from ADC/Testbench)
        -- Format: [Imag(16) | Real(16)]
        s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
        s_axis_tvalid   : in  STD_LOGIC;
        s_axis_tlast    : in  STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;
        
        -- Master Interface (Output from FFT)
        -- Format: [Imag(32) | Real(32)] - FFT grows the data width
        m_axis_tdata    : out STD_LOGIC_VECTOR(63 downto 0); 
        m_axis_tvalid   : out STD_LOGIC;
        m_axis_tlast    : out STD_LOGIC;
        m_axis_tready   : in  STD_LOGIC
    );
end radar_core;

architecture Behavioral of radar_core is

    component window_multiplier is
        -- Generic removed to match original entity
        Port (
            aclk          : in  STD_LOGIC;
            aresetn       : in  STD_LOGIC;
            s_axis_tdata  : in  STD_LOGIC_VECTOR(31 downto 0); -- Hardcoded to 32
            s_axis_tvalid : in  STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast  : in  STD_LOGIC;
            m_axis_tdata  : out STD_LOGIC_VECTOR(31 downto 0); -- Hardcoded to 32
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in  STD_LOGIC;
            m_axis_tlast  : out STD_LOGIC
        );
    end component;

    -- 2. Component: The FFT IP Core you just generated
    COMPONENT xfft_0
      PORT (
        aclk : IN STD_LOGIC;
        
        -- Configuration Channel (Required to start the core)
        s_axis_config_tdata : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
        s_axis_config_tvalid : IN STD_LOGIC;
        s_axis_config_tready : OUT STD_LOGIC;
        
        -- Data Input (From Window Multiplier)
        s_axis_data_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
        s_axis_data_tvalid : IN STD_LOGIC;
        s_axis_data_tready : OUT STD_LOGIC;
        s_axis_data_tlast : IN STD_LOGIC;
        
        -- Data Output (To Outside World)
        m_axis_data_tdata : OUT STD_LOGIC_VECTOR(63 DOWNTO 0); -- 32-bit Real + 32-bit Imag
        m_axis_data_tvalid : OUT STD_LOGIC;
        m_axis_data_tready : IN STD_LOGIC;
        m_axis_data_tlast : OUT STD_LOGIC;
        
        -- Status events (Optional, left unconnected)
        event_frame_started : OUT STD_LOGIC;
        event_tlast_unexpected : OUT STD_LOGIC;
        event_tlast_missing : OUT STD_LOGIC;
        event_status_channel_halt : OUT STD_LOGIC;
        event_data_in_channel_halt : OUT STD_LOGIC;
        event_data_out_channel_halt : OUT STD_LOGIC
      );
    END COMPONENT;

    -- Internal Signals (The wires between blocks)
    signal window_out_data   : STD_LOGIC_VECTOR(31 downto 0);
    signal window_out_valid  : STD_LOGIC;
    signal window_out_ready  : STD_LOGIC;
    signal window_out_last   : STD_LOGIC;
    
    -- Config Signals
    signal cfg_tvalid        : STD_LOGIC := '0';
    signal cfg_tready        : STD_LOGIC;
    signal cfg_sent          : STD_LOGIC := '0';

begin

    -- INSTANCE 1: Window Multiplier
    u_window : window_multiplier
    Port Map (
        aclk          => aclk,
        aresetn       => aresetn,
        s_axis_tdata  => s_axis_tdata,
        s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, -- Backpressure up to source
        s_axis_tlast  => s_axis_tlast,
        
        m_axis_tdata  => window_out_data,
        m_axis_tvalid => window_out_valid,
        m_axis_tready => window_out_ready, -- Backpressure from FFT
        m_axis_tlast  => window_out_last
    );

    -- PROCESS: FFT Configuration Logic
    -- We must send ONE valid packet to s_axis_config to tell the FFT to be "Forward" (Mode 1).
    -- Bit 0 = 1 (Forward), Bit 0 = 0 (Inverse).
    process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                cfg_tvalid <= '0';
                cfg_sent   <= '0';
            else
                if cfg_sent = '0' then
                    cfg_tvalid <= '1'; -- Raise Valid
                    if cfg_tready = '1' then
                        cfg_sent <= '1'; -- Done
                        cfg_tvalid <= '0'; -- Drop Valid
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- INSTANCE 2: FFT Core
    u_fft : xfft_0
    PORT MAP (
        aclk => aclk,
        
        -- Config Channel (Driven by process above)
        s_axis_config_tdata => x"0001", -- 1 = Forward FFT
        s_axis_config_tvalid => cfg_tvalid,
        s_axis_config_tready => cfg_tready,
        
        -- Data Input (Connected to Window Output)
        s_axis_data_tdata => window_out_data,
        s_axis_data_tvalid => window_out_valid,
        s_axis_data_tready => window_out_ready,
        s_axis_data_tlast => window_out_last,
        
        -- Data Output (Connected to Top Level Output)
        m_axis_data_tdata => m_axis_tdata,
        m_axis_data_tvalid => m_axis_tvalid,
        m_axis_data_tready => m_axis_tready,
        m_axis_data_tlast => m_axis_tlast,
        
        -- Unused Status Ports
        event_frame_started => open,
        event_tlast_unexpected => open,
        event_tlast_missing => open,
        event_status_channel_halt => open,
        event_data_in_channel_halt => open,
        event_data_out_channel_halt => open
    );

end Behavioral;