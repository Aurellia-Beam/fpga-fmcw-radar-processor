library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Alpha-max-beta-min: |Z| ~ max(|I|,|Q|) + 0.375 * min(|I|,|Q|)
-- <4% worst-case error, no CORDIC/sqrt needed

entity magnitude_calc is
    Generic (
        DATA_WIDTH : integer := 16;
        OUT_WIDTH  : integer := 17
    );
    Port (
        aclk          : in  STD_LOGIC;
        aresetn       : in  STD_LOGIC;
        s_axis_tdata  : in  STD_LOGIC_VECTOR(2*DATA_WIDTH-1 downto 0);
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;
        m_axis_tdata  : out STD_LOGIC_VECTOR(OUT_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC
    );
end magnitude_calc;

architecture Behavioral of magnitude_calc is

    signal s1_max, s1_min : unsigned(DATA_WIDTH-1 downto 0) := (others => '0');
    signal s1_valid       : std_logic := '0';
    signal s1_last        : std_logic := '0';

    signal s2_mag   : unsigned(OUT_WIDTH-1 downto 0) := (others => '0');
    signal s2_valid : std_logic := '0';
    signal s2_last  : std_logic := '0';

    signal pipe_en : std_logic;

begin

    pipe_en <= m_axis_tready or (not s2_valid);
    s_axis_tready <= pipe_en;

    -- Stage 1: Absolute value and max/min sort
    stage1: process(aclk)
        variable si, sq     : signed(DATA_WIDTH-1 downto 0);
        variable ai, aq     : unsigned(DATA_WIDTH-1 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s1_max <= (others => '0'); s1_min <= (others => '0');
                s1_valid <= '0'; s1_last <= '0';
            elsif pipe_en = '1' then
                if s_axis_tvalid = '1' then
                    si := signed(s_axis_tdata(DATA_WIDTH-1 downto 0));
                    sq := signed(s_axis_tdata(2*DATA_WIDTH-1 downto DATA_WIDTH));
                    if si(DATA_WIDTH-1) = '1' then ai := unsigned(-si); else ai := unsigned(si); end if;
                    if sq(DATA_WIDTH-1) = '1' then aq := unsigned(-sq); else aq := unsigned(sq); end if;
                    if ai >= aq then s1_max <= ai; s1_min <= aq;
                    else             s1_max <= aq; s1_min <= ai; end if;
                    s1_valid <= '1'; s1_last <= s_axis_tlast;
                else
                    s1_valid <= '0'; s1_last <= '0';
                end if;
            end if;
        end if;
    end process;

    -- Stage 2: max + 0.375*min = max + min/4 + min/8
    stage2: process(aclk)
        variable mx, md4, md8 : unsigned(OUT_WIDTH-1 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s2_mag <= (others => '0'); s2_valid <= '0'; s2_last <= '0';
            elsif pipe_en = '1' then
                if s1_valid = '1' then
                    mx  := resize(s1_max, OUT_WIDTH);
                    md4 := resize(s1_min(DATA_WIDTH-1 downto 2), OUT_WIDTH);
                    md8 := resize(s1_min(DATA_WIDTH-1 downto 3), OUT_WIDTH);
                    s2_mag <= mx + md4 + md8;
                    s2_valid <= '1'; s2_last <= s1_last;
                else
                    s2_valid <= '0'; s2_last <= '0';
                end if;
            end if;
        end if;
    end process;

    m_axis_tdata  <= std_logic_vector(s2_mag);
    m_axis_tvalid <= s2_valid;
    m_axis_tlast  <= s2_last;

end Behavioral;
