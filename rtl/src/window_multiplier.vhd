library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity window_multiplier is
    Generic (
        DATA_WIDTH : integer := 32
    );
    Port (
        aclk          : in  STD_LOGIC;
        aresetn       : in  STD_LOGIC;
        
        -- Slave Interface (Input)
        s_axis_tdata  : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;

        -- Master Interface (Output)
        m_axis_tdata  : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC
    );
end window_multiplier;

architecture Behavioral of window_multiplier is

    -- Signals
    signal input_i      : signed(15 downto 0);
    signal input_q      : signed(15 downto 0);
    signal mult_i       : signed(31 downto 0);
    signal mult_q       : signed(31 downto 0);
    
    -- Hamming Window Coefficient (Approx 0.5 for test) -> Fixed Point Q1.15
    constant WINDOW_COEFF : signed(15 downto 0) := to_signed(16384, 16);

begin

    -- I/Q Unpacking
    input_q <= signed(s_axis_tdata(31 downto 16));
    input_i <= signed(s_axis_tdata(15 downto 0));

    -- Backpressure Passthrough
    s_axis_tready <= m_axis_tready;

    process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                m_axis_tvalid <= '0';
                m_axis_tdata  <= (others => '0');
                m_axis_tlast  <= '0';
            else
                if (s_axis_tvalid = '1' and m_axis_tready = '1') then
                    -- Multiplication
                    mult_i <= input_i * WINDOW_COEFF;
                    mult_q <= input_q * WINDOW_COEFF;

                    -- Bit Slicing / Truncation (Shift right 15)
                    m_axis_tdata(15 downto 0)  <= std_logic_vector(mult_i(30 downto 15));
                    m_axis_tdata(31 downto 16) <= std_logic_vector(mult_q(30 downto 15));

                    m_axis_tlast  <= s_axis_tlast;
                    m_axis_tvalid <= '1';
                
                elsif (m_axis_tready = '1') then
                    m_axis_tvalid <= '0';
                end if;
            end if;
        end if;
    end process;

end Behavioral;