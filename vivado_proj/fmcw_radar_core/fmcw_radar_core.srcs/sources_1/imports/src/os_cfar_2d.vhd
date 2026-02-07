library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- 2D Ordered-Statistic CFAR with Adaptive Scaling
-- Processes Range-Doppler map with 2D reference window.
-- Consolidated from os_cfar_2d + ADR_os_cfar_2d with all bugs fixed.

entity os_cfar_2d is
    Generic (
        DATA_WIDTH    : integer := 17;
        REF_RANGE     : integer := 4;   -- Range ref cells per side
        REF_DOPPLER   : integer := 4;   -- Doppler ref cells per side
        GUARD_RANGE   : integer := 2;
        GUARD_DOPPLER : integer := 1;
        RANK_PCT      : integer := 75;  -- Percentile for OS-CFAR rank
        SCALE_MIN     : integer := 2;
        SCALE_MAX     : integer := 6;
        SCALE_NOM     : integer := 4;
        N_DOPPLER     : integer := 128
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
        scale_override: in  STD_LOGIC_VECTOR(2 downto 0) := "000";  -- 0 = auto
        dbg_threshold : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        dbg_scale     : out STD_LOGIC_VECTOR(2 downto 0)
    );
end os_cfar_2d;

architecture Behavioral of os_cfar_2d is

    constant WIN_RANGE   : integer := 2*REF_RANGE + 2*GUARD_RANGE + 1;
    constant WIN_DOPPLER : integer := 2*REF_DOPPLER + 2*GUARD_DOPPLER + 1;
    constant CUT_R       : integer := REF_RANGE + GUARD_RANGE;
    constant CUT_D       : integer := REF_DOPPLER + GUARD_DOPPLER;
    constant GUARD_AREA  : integer := (2*GUARD_RANGE+1) * (2*GUARD_DOPPLER+1);
    constant TOTAL_CELLS : integer := WIN_RANGE * WIN_DOPPLER;
    constant N_REF       : integer := TOTAL_CELLS - GUARD_AREA;

    -- Line buffer: WIN_DOPPLER rows × N_DOPPLER columns
    type line_buffer_t is array (0 to WIN_DOPPLER-1, 0 to N_DOPPLER-1)
                          of unsigned(DATA_WIDTH-1 downto 0);
    signal line_buffer : line_buffer_t := (others => (others => (others => '0')));

    -- 2D processing window
    type range_window_t is array (0 to WIN_DOPPLER-1, 0 to WIN_RANGE-1)
                           of unsigned(DATA_WIDTH-1 downto 0);
    signal range_window : range_window_t := (others => (others => (others => '0')));

    -- Counters
    signal doppler_cnt : unsigned(7 downto 0) := (others => '0');
    signal row_cnt     : unsigned(3 downto 0) := (others => '0');

    -- Control
    signal pipe_advance : std_logic;
    signal window_valid : std_logic := '0';
    signal startup_cnt  : unsigned(15 downto 0) := (others => '0');
    constant STARTUP_DELAY : unsigned(15 downto 0) :=
        to_unsigned((CUT_D + 1) * N_DOPPLER + CUT_R + 2, 16);

    -- Output pipeline
    signal out_valid_p1, out_valid_p2 : std_logic := '0';
    signal out_last_p1,  out_last_p2  : std_logic := '0';
    signal out_data_p2 : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');

    -- Reference array for sorting
    type ref_array_t is array (0 to N_REF-1) of unsigned(DATA_WIDTH-1 downto 0);

begin

    pipe_advance  <= m_axis_tready or (not out_valid_p2);
    s_axis_tready <= pipe_advance;

    process(aclk)
        variable refs       : ref_array_t;
        variable ref_idx    : integer;
        variable temp       : unsigned(DATA_WIDTH-1 downto 0);
        variable cut_val    : unsigned(DATA_WIDTH-1 downto 0);
        variable ranked_val : unsigned(DATA_WIDTH-1 downto 0);
        variable threshold  : unsigned(DATA_WIDTH+3 downto 0);
        variable scale      : integer;
        variable sum_refs   : unsigned(DATA_WIDTH+7 downto 0);
        variable mean_val   : unsigned(DATA_WIDTH-1 downto 0);
        variable in_guard   : boolean;
        variable d_dist, r_dist : integer;
        variable rank_idx   : integer;
        variable wr_row     : integer;
        variable rd_row     : integer;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                line_buffer   <= (others => (others => (others => '0')));
                range_window  <= (others => (others => (others => '0')));
                doppler_cnt   <= (others => '0');
                row_cnt       <= (others => '0');
                startup_cnt   <= (others => '0');
                window_valid  <= '0';
                out_valid_p1  <= '0'; out_valid_p2 <= '0';
                out_last_p1   <= '0'; out_last_p2  <= '0';
                m_axis_tvalid <= '0'; m_axis_tlast <= '0';
                m_axis_tdata  <= (others => '0');
                dbg_threshold <= (others => '0');
                dbg_scale     <= (others => '0');
            elsif pipe_advance = '1' then

                --------------------------------------------------------
                -- 1. Input to line buffer
                --------------------------------------------------------
                if s_axis_tvalid = '1' then
                    wr_row := to_integer(row_cnt);
                    line_buffer(wr_row, to_integer(doppler_cnt)) <= unsigned(s_axis_tdata);

                    if s_axis_tlast = '1' or doppler_cnt = N_DOPPLER - 1 then
                        doppler_cnt <= (others => '0');
                        if row_cnt = WIN_DOPPLER - 1 then row_cnt <= (others => '0');
                        else row_cnt <= row_cnt + 1; end if;
                    else
                        doppler_cnt <= doppler_cnt + 1;
                    end if;

                    if startup_cnt < STARTUP_DELAY then
                        startup_cnt <= startup_cnt + 1;
                        window_valid <= '0';
                    else
                        window_valid <= '1';
                    end if;

                    --------------------------------------------------------
                    -- 2. Build 2D window (FIX: gated by tvalid)
                    --------------------------------------------------------
                    for d in 0 to WIN_DOPPLER-1 loop
                        for r in WIN_RANGE-1 downto 1 loop
                            range_window(d, r) <= range_window(d, r-1);
                        end loop;
                        rd_row := (to_integer(row_cnt) + d) mod WIN_DOPPLER;
                        range_window(d, 0) <= line_buffer(rd_row, to_integer(doppler_cnt));
                    end loop;
                end if;

                --------------------------------------------------------
                -- 3. Extract reference cells (skip guard region)
                --------------------------------------------------------
                ref_idx  := 0;
                sum_refs := (others => '0');

                for d in 0 to WIN_DOPPLER-1 loop
                    for r in 0 to WIN_RANGE-1 loop
                        d_dist := abs(d - CUT_D);
                        r_dist := abs(r - CUT_R);
                        in_guard := (d_dist <= GUARD_DOPPLER) and (r_dist <= GUARD_RANGE);

                        if not in_guard and ref_idx < N_REF then
                            refs(ref_idx) := range_window(d, r);
                            sum_refs := sum_refs + resize(range_window(d, r), sum_refs'length);
                            ref_idx := ref_idx + 1;
                        end if;
                    end loop;
                end loop;

                --------------------------------------------------------
                -- 4. Sort reference cells (bubble sort)
                --------------------------------------------------------
                for i in 0 to N_REF-2 loop
                    for j in i+1 to N_REF-1 loop
                        if refs(i) > refs(j) then
                            temp := refs(i); refs(i) := refs(j); refs(j) := temp;
                        end if;
                    end loop;
                end loop;

                -- Select ranked value
                rank_idx := (N_REF * RANK_PCT) / 100;
                if rank_idx >= N_REF then rank_idx := N_REF - 1; end if;
                ranked_val := refs(rank_idx);

                --------------------------------------------------------
                -- 5. Adaptive scaling
                --------------------------------------------------------
                -- FIX: compute mean by dividing by actual N_REF
                mean_val := resize(sum_refs / to_unsigned(N_REF, sum_refs'length), DATA_WIDTH);

                if scale_override /= "000" then
                    scale := to_integer(unsigned(scale_override));
                elsif ranked_val > mean_val + shift_right(mean_val, 1) then
                    scale := SCALE_MAX;   -- High clutter
                elsif ranked_val < shift_right(mean_val, 1) then
                    scale := SCALE_MIN;   -- Low/uniform noise
                else
                    scale := SCALE_NOM;
                end if;

                --------------------------------------------------------
                -- 6. Threshold & detection
                --------------------------------------------------------
                threshold := resize(ranked_val, threshold'length) * to_unsigned(scale, 4);
                cut_val := range_window(CUT_D, CUT_R);

                out_valid_p1 <= window_valid and s_axis_tvalid;
                out_last_p1  <= s_axis_tlast;

                out_valid_p2 <= out_valid_p1;
                out_last_p2  <= out_last_p1;

                if resize(cut_val, threshold'length) > threshold then
                    out_data_p2 <= std_logic_vector(cut_val);
                else
                    out_data_p2 <= (others => '0');
                end if;

                dbg_threshold <= std_logic_vector(threshold(DATA_WIDTH-1 downto 0));
                dbg_scale     <= std_logic_vector(to_unsigned(scale, 3));

                --------------------------------------------------------
                -- 7. Output
                --------------------------------------------------------
                m_axis_tvalid <= out_valid_p2;
                m_axis_tlast  <= out_last_p2;
                m_axis_tdata  <= out_data_p2;
            end if;
        end if;
    end process;

end Behavioral;
