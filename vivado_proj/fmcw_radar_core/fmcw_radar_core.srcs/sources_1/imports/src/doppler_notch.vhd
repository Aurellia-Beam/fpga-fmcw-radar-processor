library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- MTI / Doppler Notch Filter
-- Removes zero-Doppler clutter (ground, sea surface).
-- 2-pulse canceller: y[n] = x[n] - x[n-1]
-- 3-pulse canceller: y[n] = x[n] - 2*x[n-1] + x[n-2]

entity doppler_notch is
    Generic (
        DATA_WIDTH : integer := 32;
        N_DOPPLER  : integer := 128;
        NOTCH_MODE : integer := 2    -- 2 or 3 pulse canceller
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
        m_axis_tlast  : out STD_LOGIC;
        bypass        : in  STD_LOGIC := '0'
    );
end doppler_notch;

architecture Behavioral of doppler_notch is
    constant HW : integer := DATA_WIDTH / 2;

    signal delay1_i, delay1_q : signed(HW-1 downto 0) := (others => '0');
    signal delay2_i, delay2_q : signed(HW-1 downto 0) := (others => '0');

    signal pipe_advance : std_logic;
    signal out_valid    : std_logic := '0';
    signal out_last     : std_logic := '0';
    signal out_data     : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');

begin

    pipe_advance  <= m_axis_tready or (not out_valid);
    s_axis_tready <= pipe_advance;

    -- FIX: single output register (was double-registered, adding unnecessary latency)
    m_axis_tdata  <= out_data;
    m_axis_tvalid <= out_valid;
    m_axis_tlast  <= out_last;

    process(aclk)
        variable in_i, in_q     : signed(HW-1 downto 0);
        variable out_i, out_q   : signed(HW-1 downto 0);
        variable diff_i, diff_q : signed(HW downto 0);
        variable diff3_i, diff3_q : signed(HW+1 downto 0);
        constant MAX_V : signed(HW-1 downto 0) := to_signed(2**(HW-1)-1, HW);
        constant MIN_V : signed(HW-1 downto 0) := to_signed(-2**(HW-1), HW);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                delay1_i <= (others => '0'); delay1_q <= (others => '0');
                delay2_i <= (others => '0'); delay2_q <= (others => '0');
                out_valid <= '0'; out_last <= '0'; out_data <= (others => '0');
            elsif pipe_advance = '1' then
                if s_axis_tvalid = '1' then
                    in_i := signed(s_axis_tdata(HW-1 downto 0));
                    in_q := signed(s_axis_tdata(DATA_WIDTH-1 downto HW));

                    if bypass = '1' then
                        out_i := in_i; out_q := in_q;
                    elsif NOTCH_MODE = 2 then
                        diff_i := resize(in_i, HW+1) - resize(delay1_i, HW+1);
                        diff_q := resize(in_q, HW+1) - resize(delay1_q, HW+1);
                        if    diff_i > MAX_V then out_i := MAX_V;
                        elsif diff_i < MIN_V then out_i := MIN_V;
                        else  out_i := resize(diff_i, HW); end if;
                        if    diff_q > MAX_V then out_q := MAX_V;
                        elsif diff_q < MIN_V then out_q := MIN_V;
                        else  out_q := resize(diff_q, HW); end if;
                    else -- 3-pulse
                        diff3_i := resize(in_i, HW+2)
                                 - shift_left(resize(delay1_i, HW+2), 1)
                                 + resize(delay2_i, HW+2);
                        diff3_q := resize(in_q, HW+2)
                                 - shift_left(resize(delay1_q, HW+2), 1)
                                 + resize(delay2_q, HW+2);
                        if    diff3_i > MAX_V then out_i := MAX_V;
                        elsif diff3_i < MIN_V then out_i := MIN_V;
                        else  out_i := resize(diff3_i, HW); end if;
                        if    diff3_q > MAX_V then out_q := MAX_V;
                        elsif diff3_q < MIN_V then out_q := MIN_V;
                        else  out_q := resize(diff3_q, HW); end if;
                    end if;

                    delay2_i <= delay1_i; delay2_q <= delay1_q;
                    delay1_i <= in_i;     delay1_q <= in_q;

                    if s_axis_tlast = '1' then
                        delay1_i <= (others => '0'); delay1_q <= (others => '0');
                        delay2_i <= (others => '0'); delay2_q <= (others => '0');
                    end if;

                    out_data  <= std_logic_vector(out_q) & std_logic_vector(out_i);
                    out_valid <= '1';
                    out_last  <= s_axis_tlast;
                else
                    out_valid <= '0';
                end if;
            end if;
        end if;
    end process;

end Behavioral;
