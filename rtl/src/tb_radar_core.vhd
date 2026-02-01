library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL; -- For sine generation

entity tb_radar_core is
end tb_radar_core;

architecture Behavioral of tb_radar_core is

    component radar_core is
        Port (
            aclk            : in  STD_LOGIC;
            aresetn         : in  STD_LOGIC;
            s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid   : in  STD_LOGIC;
            s_axis_tlast    : in  STD_LOGIC;
            s_axis_tready   : out STD_LOGIC;
            m_axis_tdata    : out STD_LOGIC_VECTOR(63 downto 0);
            m_axis_tvalid   : out STD_LOGIC;
            m_axis_tlast    : out STD_LOGIC;
            m_axis_tready   : in  STD_LOGIC
        );
    end component;

    -- Signals
    signal aclk : std_logic := '0';
    signal aresetn : std_logic := '0';
    
    -- Input (Time Domain)
    signal s_data : std_logic_vector(31 downto 0) := (others => '0');
    signal s_valid : std_logic := '0';
    signal s_last : std_logic := '0';
    signal s_ready : std_logic;

    -- Output (Frequency Domain)
    signal m_data : std_logic_vector(63 downto 0);
    signal m_valid : std_logic;
    signal m_last : std_logic;
    signal m_ready : std_logic := '1'; -- Always ready to accept data

    -- Simulation Constants
    constant CLK_PERIOD : time := 10 ns;
    constant N_SAMPLES : integer := 1024;

begin

    -- Instantiate the DUT (Device Under Test)
    uut: radar_core
    port map (
        aclk => aclk,
        aresetn => aresetn,
        s_axis_tdata => s_data,
        s_axis_tvalid => s_valid,
        s_axis_tlast => s_last,
        s_axis_tready => s_ready,
        m_axis_tdata => m_data,
        m_axis_tvalid => m_valid,
        m_axis_tlast => m_last,
        m_axis_tready => m_ready
    );

    -- Clock Process
    process
    begin
        aclk <= '0'; wait for CLK_PERIOD/2;
        aclk <= '1'; wait for CLK_PERIOD/2;
    end process;

-- Stimulus Process
    process
        variable i : integer := 0;
        variable real_part : integer;
        variable angle : real;
    begin
        -- 1. Reset System
        aresetn <= '0';
        s_valid <= '0';
        s_last  <= '0';
        wait for 100 ns;
        aresetn <= '1';
        wait for 100 ns;

        -- 2. Inject Sine Wave (Using WHILE loop to handle backpressure)
        i := 0;
        while i < N_SAMPLES loop
            wait until rising_edge(aclk);
            
            -- Check if the core is ready to accept data
            if s_ready = '1' then
                s_valid <= '1';
                
                -- Generate Sine Wave: A * sin(2*pi*f*t)
                angle := 2.0 * MATH_PI * 0.05 * real(i); 
                real_part := integer(10000.0 * sin(angle));
                
                -- Pack Data
                s_data <= std_logic_vector(to_signed(0, 16)) & std_logic_vector(to_signed(real_part, 16));
                
                -- Handle Last Signal
                if i = N_SAMPLES-1 then
                    s_last <= '1';
                else
                    s_last <= '0';
                end if;
                
                -- INCREMENT ONLY ON SUCCESS
                i := i + 1;
            else
                -- Core is busy, wait and try again
                s_valid <= '0';
            end if;
        end loop;

        -- 3. End of Packet
        wait until rising_edge(aclk);
        s_valid <= '0';
        s_last <= '0';

        -- 4. Wait for FFT Output latency
        wait for 50 us;
        
        assert false report "Simulation Completed Successfully" severity failure;
        wait;
    end process;

end Behavioral;