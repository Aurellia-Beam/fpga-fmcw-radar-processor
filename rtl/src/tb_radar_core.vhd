library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

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
            -- OUTPUT: 32 BITS (Matches the new IP setting)
            m_axis_tdata    : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid   : out STD_LOGIC;
            m_axis_tlast    : out STD_LOGIC;
            m_axis_tready   : in  STD_LOGIC
        );
    end component;

    signal aclk : std_logic := '0';
    signal aresetn : std_logic := '0';
    signal s_data : std_logic_vector(31 downto 0) := (others => '0');
    signal s_valid : std_logic := '0';
    signal s_last : std_logic := '0';
    signal s_ready : std_logic;

    -- OUTPUT: 32 BITS
    signal m_data : std_logic_vector(31 downto 0);
    signal m_valid : std_logic;
    signal m_last : std_logic;
    signal m_ready : std_logic := '1';

    constant CLK_PERIOD : time := 10 ns;
    constant N_SAMPLES : integer := 1024;

begin

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

    process
    begin
        aclk <= '0'; wait for CLK_PERIOD/2;
        aclk <= '1'; wait for CLK_PERIOD/2;
    end process;

    process
        variable i : integer := 0;
        variable real_part : integer;
        variable angle : real;
    begin
        -- 1. Reset
        aresetn <= '0';
        s_valid <= '0';
        s_last  <= '0';
        wait for 100 ns;
        aresetn <= '1';
        
        -- 2. Wait for FFT Wakeup (Crucial!)
        wait for 5 us; 

        -- 3. Send Data
        i := 0;
        while i < N_SAMPLES loop
            wait until rising_edge(aclk);
            if s_ready = '1' then
                s_valid <= '1';
                angle := 2.0 * MATH_PI * 0.05 * real(i); 
                real_part := integer(10000.0 * sin(angle));
                s_data <= std_logic_vector(to_signed(0, 16)) & std_logic_vector(to_signed(real_part, 16));
                
                if i = N_SAMPLES-1 then
                    s_last <= '1';
                else
                    s_last <= '0';
                end if;
                i := i + 1;
            else
                s_valid <= '0';
            end if;
        end loop;

        wait until rising_edge(aclk);
        s_valid <= '0';
        s_last <= '0';
        wait for 50 us;
        
        assert false report "Simulation Completed Successfully" severity failure;
        wait;
    end process;

end Behavioral;