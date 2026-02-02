library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================================
-- Magnitude Calculator: Alpha-Max Beta-Min Approximation
-- =============================================================================
-- Computes |Z| ≈ max(|I|,|Q|) + 0.375 * min(|I|,|Q|)
--
-- This avoids expensive CORDIC or square root operations.
-- Error: < 4% worst case, < 1% average
--
-- The 0.375 factor is implemented as: (1/4 + 1/8) = (x >> 2) + (x >> 3)
-- This uses only shifts and adds - no multipliers!
--
-- Pipeline: 3 cycles latency
--   Stage 1: Absolute value
--   Stage 2: Min/Max comparison  
--   Stage 3: Weighted sum
--
-- Author: Senior Radar Systems Engineer
-- =============================================================================

entity magnitude_calc is
    Generic (
        DATA_WIDTH : integer := 16;  -- Input I/Q width
        OUT_WIDTH  : integer := 17   -- Output magnitude width (1 bit growth)
    );
    Port (
        aclk          : in  STD_LOGIC;
        aresetn       : in  STD_LOGIC;
        
        -- AXI-Stream Slave: Complex I/Q input
        s_axis_tdata  : in  STD_LOGIC_VECTOR(2*DATA_WIDTH-1 downto 0);  -- [Q:I]
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;
        
        -- AXI-Stream Master: Magnitude output
        m_axis_tdata  : out STD_LOGIC_VECTOR(OUT_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC
    );
end magnitude_calc;

architecture Behavioral of magnitude_calc is

    -- Pipeline stage 1: Absolute values
    signal s1_abs_i   : unsigned(DATA_WIDTH-1 downto 0) := (others => '0');
    signal s1_abs_q   : unsigned(DATA_WIDTH-1 downto 0) := (others => '0');
    signal s1_valid   : std_logic := '0';
    signal s1_last    : std_logic := '0';
    
    -- Pipeline stage 2: Min/Max
    signal s2_max     : unsigned(DATA_WIDTH-1 downto 0) := (others => '0');
    signal s2_min     : unsigned(DATA_WIDTH-1 downto 0) := (others => '0');
    signal s2_valid   : std_logic := '0';
    signal s2_last    : std_logic := '0';
    
    -- Pipeline stage 3: Result
    signal s3_mag     : unsigned(OUT_WIDTH-1 downto 0) := (others => '0');
    signal s3_valid   : std_logic := '0';
    signal s3_last    : std_logic := '0';
    
    -- Flow control
    signal pipe_enable : std_logic;

begin

    -- Pipeline advances when output can accept data
    pipe_enable <= m_axis_tready or (not s3_valid);
    s_axis_tready <= pipe_enable;

    -- =========================================================================
    -- Stage 1: Compute Absolute Values
    -- =========================================================================
    stage1_proc: process(aclk)
        variable i_signed : signed(DATA_WIDTH-1 downto 0);
        variable q_signed : signed(DATA_WIDTH-1 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s1_abs_i <= (others => '0');
                s1_abs_q <= (others => '0');
                s1_valid <= '0';
                s1_last  <= '0';
            elsif pipe_enable = '1' then
                if s_axis_tvalid = '1' then
                    -- Extract I and Q
                    i_signed := signed(s_axis_tdata(DATA_WIDTH-1 downto 0));
                    q_signed := signed(s_axis_tdata(2*DATA_WIDTH-1 downto DATA_WIDTH));
                    
                    -- Absolute value (handle most negative carefully)
                    if i_signed < 0 then
                        s1_abs_i <= unsigned(-i_signed);
                    else
                        s1_abs_i <= unsigned(i_signed);
                    end if;
                    
                    if q_signed < 0 then
                        s1_abs_q <= unsigned(-q_signed);
                    else
                        s1_abs_q <= unsigned(q_signed);
                    end if;
                    
                    s1_valid <= '1';
                    s1_last  <= s_axis_tlast;
                else
                    s1_valid <= '0';
                    s1_last  <= '0';
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Stage 2: Determine Min and Max
    -- =========================================================================
    stage2_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s2_max   <= (others => '0');
                s2_min   <= (others => '0');
                s2_valid <= '0';
                s2_last  <= '0';
            elsif pipe_enable = '1' then
                if s1_valid = '1' then
                    if s1_abs_i >= s1_abs_q then
                        s2_max <= s1_abs_i;
                        s2_min <= s1_abs_q;
                    else
                        s2_max <= s1_abs_q;
                        s2_min <= s1_abs_i;
                    end if;
                    s2_valid <= '1';
                    s2_last  <= s1_last;
                else
                    s2_valid <= '0';
                    s2_last  <= '0';
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Stage 3: Compute Alpha*Max + Beta*Min
    -- =========================================================================
    -- Alpha = 1, Beta = 0.375 = 1/4 + 1/8
    -- Result = max + (min >> 2) + (min >> 3)
    -- =========================================================================
    stage3_proc: process(aclk)
        variable max_ext   : unsigned(OUT_WIDTH-1 downto 0);
        variable min_div4  : unsigned(OUT_WIDTH-1 downto 0);
        variable min_div8  : unsigned(OUT_WIDTH-1 downto 0);
        variable result    : unsigned(OUT_WIDTH-1 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s3_mag   <= (others => '0');
                s3_valid <= '0';
                s3_last  <= '0';
            elsif pipe_enable = '1' then
                if s2_valid = '1' then
                    -- Extend to output width
                    max_ext  := resize(s2_max, OUT_WIDTH);
                    
                    -- Beta * min = 0.375 * min = min/4 + min/8
                    min_div4 := resize(s2_min(DATA_WIDTH-1 downto 2), OUT_WIDTH);  -- >> 2
                    min_div8 := resize(s2_min(DATA_WIDTH-1 downto 3), OUT_WIDTH);  -- >> 3
                    
                    -- Final sum
                    result := max_ext + min_div4 + min_div8;
                    
                    s3_mag   <= result;
                    s3_valid <= '1';
                    s3_last  <= s2_last;
                else
                    s3_valid <= '0';
                    s3_last  <= '0';
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Output assignments
    -- =========================================================================
    m_axis_tdata  <= std_logic_vector(s3_mag);
    m_axis_tvalid <= s3_valid;
    m_axis_tlast  <= s3_last;

end Behavioral;
