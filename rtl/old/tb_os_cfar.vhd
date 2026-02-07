library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use std.textio.all;
use IEEE.std_logic_textio.all;

entity tb_os_cfar is
end tb_os_cfar;

architecture Behavioral of tb_os_cfar is

    component os_cfar is
        Generic (
            DATA_WIDTH   : integer := 17;
            REF_CELLS    : integer := 8;
            GUARD_CELLS  : integer := 2;
            RANK_IDX     : integer := 12;
            SCALING_MULT : integer := 4;
            SCALING_DIV  : integer := 1
        );
        Port (
            aclk          : in  STD_LOGIC;
            aresetn       : in  STD_LOGIC;
            s_axis_tdata  : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
            s_axis_tvalid : in  STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast  : in  STD_LOGIC;
            m_axis_tdata  : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in  STD_LOGIC;
            m_axis_tlast  : out STD_LOGIC
        );
    end component;

    -- Simulation Constants
    constant CLK_PERIOD : time := 10 ns;
    constant DATA_W     : integer := 17;
    
    -- Signals
    signal aclk          : std_logic := '0';
    signal aresetn       : std_logic := '0';
    signal s_axis_tdata  : std_logic_vector(DATA_W-1 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tready : std_logic;
    signal s_axis_tlast  : std_logic := '0';
    
    signal m_axis_tdata  : std_logic_vector(DATA_W-1 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tready : std_logic := '1';
    signal m_axis_tlast  : std_logic;

    signal sim_done : boolean := false;

begin

    -- Instantiate Unit Under Test (UUT)
    uut: os_cfar
    generic map (
        DATA_WIDTH   => DATA_W,
        REF_CELLS    => 8,  -- 8 left + 8 right = 16 reference cells
        GUARD_CELLS  => 2,  -- 2 left + 2 right = 4 guard cells
        RANK_IDX     => 12, -- 12th largest of 16 (approx 75%)
        SCALING_MULT => 3,  -- Threshold = 3 * 12th_rank
        SCALING_DIV  => 1
    )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready, m_axis_tlast => m_axis_tlast
    );

    -- Clock Generation
    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    -- Stimulus Process
    process
        procedure send_sample(val : integer; last : std_logic := '0') is
        begin
            s_axis_tdata  <= std_logic_vector(to_unsigned(val, DATA_W));
            s_axis_tvalid <= '1';
            s_axis_tlast  <= last;
            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop
                wait until rising_edge(aclk);
            end loop;
        end procedure;

    begin
        -- Reset
        aresetn <= '0';
        wait for 50 ns;
        aresetn <= '1';
        wait for 20 ns;
        wait until rising_edge(aclk);

        report "Starting Simulation: Injecting Noise Floor";
        
        -- 1. Inject Noise Floor (Values around 10-20)
        -- The buffer needs to fill up (Ref + Guard + CUT) before valid outputs appear
        for i in 0 to 30 loop
            send_sample(10 + (i mod 5)); -- Varying noise 10-14
        end loop;

        report "Injecting Target 1 (Single clean target)";
        -- 2. Inject Single Strong Target
        send_sample(100); -- Amplitude 100 (should be detected)
        
        -- Return to noise
        for i in 0 to 20 loop
            send_sample(10 + (i mod 3));
        end loop;

        report "Injecting Multi-Target (Masking Test)";
        -- 3. Inject Two Close Targets
        -- Standard CFAR might average these and raise threshold too high.
        -- OS-CFAR sorts them. If Rank=12 (75%), one target shouldn't kill the other.
        send_sample(200); -- Target A
        send_sample(15);  -- 1 cell gap (Guard area)
        send_sample(250); -- Target B
        
        -- Flush with noise
        for i in 0 to 30 loop
            send_sample(12);
        end loop;

        -- End simulation
        s_axis_tvalid <= '0';
        wait for 200 ns;
        sim_done <= true;
        report "Simulation Complete";
        wait;
    end process;

    -- Monitor Process
    process(aclk)
        variable l : line;
    begin
        if rising_edge(aclk) then
            if m_axis_tvalid = '1' then
                if unsigned(m_axis_tdata) > 0 then
                    write(l, string'("DETECTION! Amplitude: "));
                    write(l, to_integer(unsigned(m_axis_tdata)));
                    writeline(output, l);
                else
                    -- Uncomment below to see noise processing
                    -- write(l, string'("Noise... (0)")); 
                    -- writeline(output, l);
                end if;
            end if;
        end if;
    end process;

end Behavioral;