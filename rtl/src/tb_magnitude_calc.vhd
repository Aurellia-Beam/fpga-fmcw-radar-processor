library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;

entity tb_magnitude_calc is
end tb_magnitude_calc;

architecture Behavioral of tb_magnitude_calc is

    constant CLK_PERIOD : time := 10 ns;
    constant DATA_W     : integer := 16;
    constant OUT_W      : integer := 17;
    constant PIPE_DEPTH : integer := 3;  -- 2 stages + 1 margin

    signal aclk          : std_logic := '0';
    signal aresetn       : std_logic := '0';
    signal s_axis_tdata  : std_logic_vector(2*DATA_W-1 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tready : std_logic;
    signal s_axis_tlast  : std_logic := '0';
    signal m_axis_tdata  : std_logic_vector(OUT_W-1 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tready : std_logic := '1';
    signal m_axis_tlast  : std_logic;
    signal sim_done      : boolean := false;
    signal pass_count    : integer := 0;
    signal fail_count    : integer := 0;

    -- Alpha-max-beta-min reference model
    function amb_ref(i_val, q_val : integer) return integer is
        variable ai, aq, mx, mn : integer;
    begin
        if i_val < 0 then ai := -i_val; else ai := i_val; end if;
        if q_val < 0 then aq := -q_val; else aq := q_val; end if;
        if ai >= aq then mx := ai; mn := aq;
        else             mx := aq; mn := ai; end if;
        return mx + mn/4 + mn/8;  -- 0.375 * min
    end function;

    -- Test vector storage
    type test_vec_t is record
        i_val : integer;
        q_val : integer;
    end record;
    type test_array_t is array (natural range <>) of test_vec_t;

    constant TESTS : test_array_t := (
        -- Basic axes
        (i_val => 1000,  q_val => 0),       -- Pure I
        (i_val => 0,     q_val => 1000),     -- Pure Q
        (i_val => -1000, q_val => 0),        -- Negative I
        (i_val => 0,     q_val => -1000),    -- Negative Q
        -- Equal magnitude
        (i_val => 1000,  q_val => 1000),     -- 45 degrees
        (i_val => -1000, q_val => 1000),     -- 135 degrees
        (i_val => -1000, q_val => -1000),    -- 225 degrees
        (i_val => 1000,  q_val => -1000),    -- 315 degrees
        -- Typical radar returns
        (i_val => 5000,  q_val => 3000),
        (i_val => -8000, q_val => 6000),
        (i_val => 100,   q_val => 100),      -- Low SNR
        (i_val => 32000, q_val => 32000),    -- Near full scale
        -- Zero
        (i_val => 0,     q_val => 0),
        -- Small values
        (i_val => 1,     q_val => 0),
        (i_val => 1,     q_val => 1),
        -- Asymmetric
        (i_val => 30000, q_val => 100),
        (i_val => 100,   q_val => 30000)
    );

    constant N_TESTS : integer := TESTS'length;

    -- Expected results queue
    type expect_array_t is array (0 to N_TESTS+PIPE_DEPTH-1) of integer;
    signal expected : expect_array_t := (others => 0);
    signal exp_wr   : integer := 0;
    signal exp_rd   : integer := 0;

begin

    uut: entity work.magnitude_calc
    generic map ( DATA_WIDTH => DATA_W, OUT_WIDTH => OUT_W )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready, m_axis_tlast => m_axis_tlast
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    stim: process
        variable L : line;
        variable ref : integer;
    begin
        aresetn <= '0'; wait for 50 ns;
        aresetn <= '1'; wait until rising_edge(aclk);

        report "=== Magnitude Calc TB: " & integer'image(N_TESTS) & " vectors ===";

        for t in 0 to N_TESTS-1 loop
            s_axis_tdata <= std_logic_vector(to_signed(TESTS(t).q_val, DATA_W)) &
                            std_logic_vector(to_signed(TESTS(t).i_val, DATA_W));
            s_axis_tvalid <= '1';
            s_axis_tlast  <= '1' when t = N_TESTS-1 else '0';

            ref := amb_ref(TESTS(t).i_val, TESTS(t).q_val);
            expected(exp_wr) <= ref;
            exp_wr <= exp_wr + 1;

            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
        end loop;

        s_axis_tvalid <= '0';
        wait for 200 ns;

        write(L, string'("=== RESULTS: "));
        write(L, pass_count); write(L, string'(" pass, "));
        write(L, fail_count); write(L, string'(" fail ==="));
        writeline(output, L);

        if fail_count = 0 then report "ALL TESTS PASSED" severity note;
        else                    report "FAILURES DETECTED" severity error; end if;

        sim_done <= true; wait;
    end process;

    -- Backpressure test: deassert tready every 5th cycle
    bp: process(aclk)
        variable cnt : integer := 0;
    begin
        if rising_edge(aclk) then
            cnt := cnt + 1;
            if cnt mod 7 = 0 then m_axis_tready <= '0';
            else                  m_axis_tready <= '1'; end if;
        end if;
    end process;

    check: process(aclk)
        variable L : line;
        variable got, exp : integer;
        variable tolerance : integer;
    begin
        if rising_edge(aclk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                got := to_integer(unsigned(m_axis_tdata));
                exp := expected(exp_rd);
                exp_rd <= exp_rd + 1;

                -- Allow ±1 LSB for rounding
                tolerance := 1;
                if abs(got - exp) <= tolerance then
                    pass_count <= pass_count + 1;
                else
                    fail_count <= fail_count + 1;
                    write(L, string'("FAIL vec ")); write(L, exp_rd);
                    write(L, string'(": expected ")); write(L, exp);
                    write(L, string'(" got ")); write(L, got);
                    writeline(output, L);
                end if;
            end if;
        end if;
    end process;

end Behavioral;
