library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use std.textio.all;
use IEEE.std_logic_textio.all;

entity tb_window_multiplier is
    -- Testbenches have no ports
end tb_window_multiplier;

architecture Behavioral of tb_window_multiplier is

    -- DUT Component Declaration
    component window_multiplier is
        Generic ( DATA_WIDTH : integer := 32 );
        Port ( 
            aclk          : in STD_LOGIC;
            aresetn       : in STD_LOGIC;
            s_axis_tdata  : in STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid : in STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast  : in STD_LOGIC;
            m_axis_tdata  : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in STD_LOGIC;
            m_axis_tlast  : out STD_LOGIC
        );
    end component;

    -- Signals
    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';
    
    signal s_axis_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tready : std_logic;
    signal s_axis_tlast  : std_logic := '0';

    signal m_axis_tdata  : std_logic_vector(31 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tready : std_logic := '1'; -- Always ready to accept output
    signal m_axis_tlast  : std_logic;

    -- Clock Period (100 MHz)
    constant CLK_PERIOD : time := 10 ns;

begin

    -- Instantiate DUT
    uut: window_multiplier
    port map (
        aclk => aclk,
        aresetn => aresetn,
        s_axis_tdata => s_axis_tdata,
        s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready,
        s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m_axis_tdata,
        m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready,
        m_axis_tlast => m_axis_tlast
    );

    -- Clock Process
    aclk_process : process
    begin
        aclk <= '0'; wait for CLK_PERIOD/2;
        aclk <= '1'; wait for CLK_PERIOD/2;
    end process;

    -- Main Stimulus Process
    stim_proc: process
        file input_file  : text open read_mode is "golden_input_chirp.txt";
        file output_file : text open write_mode is "output_verification.txt";
        
        variable inline    : line;
        variable outline   : line;
        variable v_re_in   : integer;
        variable v_im_in   : integer;
        variable v_data_in : std_logic_vector(31 downto 0);
        variable v_re_out  : integer;
        variable v_im_out  : integer;
    begin
        -- Reset Hold
        aresetn <= '0';
        wait for 100 ns;
        aresetn <= '1';
        wait for CLK_PERIOD * 5;

        -- Read File and Drive AXI Stream
        while not endfile(input_file) loop
            readline(input_file, inline);
            read(inline, v_re_in);
            read(inline, v_im_in);

            -- Pack Integers to SLV
            v_data_in(15 downto 0)  := std_logic_vector(to_signed(v_re_in, 16));
            v_data_in(31 downto 16) := std_logic_vector(to_signed(v_im_in, 16));

            -- Drive Stream
            s_axis_tvalid <= '1';
            s_axis_tdata  <= v_data_in;

            -- Wait for Handshake
            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop
                wait until rising_edge(aclk);
            end loop;
        end loop;

        -- End of Stream
        s_axis_tvalid <= '0';
        wait for 200 ns;
        
        assert false report "Simulation Finished Successfully" severity failure;
    end process;

    -- Output Monitor Process 
    monitor_proc: process(aclk)
        file output_file : text open write_mode is "output_verification.txt";
        variable outline : line;
        variable v_re : integer;
        variable v_im : integer;
    begin
        if rising_edge(aclk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                v_re := to_integer(signed(m_axis_tdata(15 downto 0)));
                v_im := to_integer(signed(m_axis_tdata(31 downto 16)));
                
                write(outline, v_re);
                write(outline, string'(" "));
                write(outline, v_im);
                writeline(output_file, outline);
            end if;
        end if;
    end process;

end Behavioral;