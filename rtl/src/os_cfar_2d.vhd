library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- 2D OS-CFAR with Adaptive Scaling
-- Processes Range-Doppler map with 2D reference window
-- Dynamic threshold scaling based on local noise variance

entity os_cfar_2d is
    Generic (
        DATA_WIDTH   : integer := 17;
        -- Reference window (per side)
        REF_RANGE    : integer := 4;   -- Range ref cells per side
        REF_DOPPLER  : integer := 4;   -- Doppler ref cells per side
        -- Guard cells (per side)
        GUARD_RANGE  : integer := 2;
        GUARD_DOPPLER: integer := 1;
        -- OS-CFAR rank (percentile of sorted refs)
        RANK_PCT     : integer := 75;  -- 75 = 75th percentile
        -- Adaptive scaling bounds
        SCALE_MIN    : integer := 2;   -- Minimum multiplier
        SCALE_MAX    : integer := 6;   -- Maximum multiplier
        SCALE_NOM    : integer := 4;   -- Nominal multiplier
        -- Doppler dimension for line buffering
        N_DOPPLER    : integer := 128
    );
    Port (
        aclk          : in  STD_LOGIC;
        aresetn       : in  STD_LOGIC;
        s_axis_tdata  : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;  -- End of Doppler row
        m_axis_tdata  : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC;
        -- Debug outputs
        dbg_threshold : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        dbg_scale     : out STD_LOGIC_VECTOR(3 downto 0)
    );
end os_cfar_2d;

architecture Behavioral of os_cfar_2d is

    -- Window dimensions
    constant WIN_RANGE   : integer := 2*REF_RANGE + 2*GUARD_RANGE + 1;
    constant WIN_DOPPLER : integer := 2*REF_DOPPLER + 2*GUARD_DOPPLER + 1;
    constant CUT_R       : integer := REF_RANGE + GUARD_RANGE;  -- CUT position in range
    constant CUT_D       : integer := REF_DOPPLER + GUARD_DOPPLER;  -- CUT position in doppler
    
    -- Total reference cells (excluding guard and CUT)
    -- Full window minus guard region minus CUT
    constant GUARD_AREA  : integer := (2*GUARD_RANGE+1) * (2*GUARD_DOPPLER+1);
    constant TOTAL_CELLS : integer := WIN_RANGE * WIN_DOPPLER;
    constant N_REF       : integer := TOTAL_CELLS - GUARD_AREA;
    
    -- Line buffer type: stores WIN_DOPPLER rows of N_DOPPLER samples each
    type line_buffer_t is array (0 to WIN_DOPPLER-1, 0 to N_DOPPLER-1) 
                          of unsigned(DATA_WIDTH-1 downto 0);
    signal line_buffer : line_buffer_t := (others => (others => (others => '0')));
    
    -- Range shift register for current processing window
    type range_window_t is array (0 to WIN_DOPPLER-1, 0 to WIN_RANGE-1) 
                           of unsigned(DATA_WIDTH-1 downto 0);
    signal range_window : range_window_t := (others => (others => (others => '0')));
    
    -- Position counters
    signal doppler_cnt : unsigned(6 downto 0) := (others => '0');
    signal range_cnt   : unsigned(9 downto 0) := (others => '0');
    signal row_cnt     : unsigned(3 downto 0) := (others => '0');  -- Which row we're writing
    
    -- Pipeline control
    signal pipe_advance : std_logic;
    signal window_valid : std_logic := '0';
    signal startup_cnt  : unsigned(15 downto 0) := (others => '0');
    constant STARTUP_DELAY : unsigned(15 downto 0) := 
        to_unsigned((CUT_D + 1) * N_DOPPLER + CUT_R, 16);
    
    -- Output pipeline
    signal out_valid_p1, out_valid_p2 : std_logic := '0';
    signal out_last_p1, out_last_p2   : std_logic := '0';
    signal out_data_p2  : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    
    -- Reference cell array for sorting
    type ref_array_t is array (0 to N_REF-1) of unsigned(DATA_WIDTH-1 downto 0);
    
    -- Variance estimation
    signal variance_acc : unsigned(DATA_WIDTH+7 downto 0) := (others => '0');
    signal mean_est     : unsigned(DATA_WIDTH-1 downto 0) := (others => '0');
    
begin

    pipe_advance  <= m_axis_tready or (not out_valid_p2);
    s_axis_tready <= pipe_advance;

    -- Main processing
    process(aclk)
        variable refs       : ref_array_t;
        variable ref_idx    : integer;
        variable temp       : unsigned(DATA_WIDTH-1 downto 0);
        variable cut_val    : unsigned(DATA_WIDTH-1 downto 0);
        variable ranked_val : unsigned(DATA_WIDTH-1 downto 0);
        variable threshold  : unsigned(DATA_WIDTH+3 downto 0);
        variable scale      : integer;
        variable sum_refs   : unsigned(DATA_WIDTH+7 downto 0);
        variable sum_sq     : unsigned((DATA_WIDTH*2)+7 downto 0);
        variable variance   : unsigned(DATA_WIDTH-1 downto 0);
        variable in_guard   : boolean;
        variable d_dist, r_dist : integer;
        variable rank_idx   : integer;
        variable wr_row     : integer;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                line_buffer   <= (others => (others => (others => '0')));
                range_window  <= (others => (others => (others => '0')));
                doppler_cnt   <= (others => '0');
                range_cnt     <= (others => '0');
                row_cnt       <= (others => '0');
                startup_cnt   <= (others => '0');
                window_valid  <= '0';
                out_valid_p1  <= '0';
                out_valid_p2  <= '0';
                out_last_p1   <= '0';
                out_last_p2   <= '0';
                m_axis_tvalid <= '0';
                m_axis_tlast  <= '0';
                m_axis_tdata  <= (others => '0');
                dbg_threshold <= (others => '0');
                dbg_scale     <= (others => '0');
            elsif pipe_advance = '1' then
                
                --------------------------------------------------------
                -- 1. Input Stage: Write to line buffer
                --------------------------------------------------------
                if s_axis_tvalid = '1' then
                    wr_row := to_integer(row_cnt);
                    line_buffer(wr_row, to_integer(doppler_cnt)) <= unsigned(s_axis_tdata);
                    
                    -- Update counters
                    if s_axis_tlast = '1' or doppler_cnt = N_DOPPLER - 1 then
                        doppler_cnt <= (others => '0');
                        if row_cnt = WIN_DOPPLER - 1 then
                            row_cnt <= (others => '0');
                        else
                            row_cnt <= row_cnt + 1;
                        end if;
                    else
                        doppler_cnt <= doppler_cnt + 1;
                    end if;
                    
                    -- Track startup
                    if startup_cnt < STARTUP_DELAY then
                        startup_cnt <= startup_cnt + 1;
                        window_valid <= '0';
                    else
                        window_valid <= '1';
                    end if;
                end if;
                
                --------------------------------------------------------
                -- 2. Build 2D Window from Line Buffer
                --------------------------------------------------------
                -- Shift range dimension and load new column
                for d in 0 to WIN_DOPPLER-1 loop
                    for r in WIN_RANGE-1 downto 1 loop
                        range_window(d, r) <= range_window(d, r-1);
                    end loop;
                    -- New sample goes to position 0
                    -- Map line buffer row accounting for circular indexing
                    range_window(d, 0) <= line_buffer(
                        (to_integer(row_cnt) + d) mod WIN_DOPPLER,
                        to_integer(doppler_cnt)
                    );
                end loop;
                
                --------------------------------------------------------
                -- 3. Extract Reference Cells (skip guard region)
                --------------------------------------------------------
                ref_idx := 0;
                sum_refs := (others => '0');
                
                for d in 0 to WIN_DOPPLER-1 loop
                    for r in 0 to WIN_RANGE-1 loop
                        -- Check if in guard region (including CUT)
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
                -- 4. Sort Reference Cells (Partial Sort for OS-CFAR)
                --------------------------------------------------------
                -- Only need to find the k-th element, but full sort is simpler
                -- For synthesis, consider bitonic sort or partial selection
                for i in 0 to N_REF-2 loop
                    for j in i+1 to N_REF-1 loop
                        if refs(i) > refs(j) then
                            temp := refs(i);
                            refs(i) := refs(j);
                            refs(j) := temp;
                        end if;
                    end loop;
                end loop;
                
                -- Select ranked value (k-th percentile)
                rank_idx := (N_REF * RANK_PCT) / 100;
                if rank_idx >= N_REF then
                    rank_idx := N_REF - 1;
                end if;
                ranked_val := refs(rank_idx);
                
                --------------------------------------------------------
                -- 5. Adaptive Scaling Based on Local Statistics
                --------------------------------------------------------
                -- Estimate noise variance from reference cells
                -- Use ratio of max to median as heterogeneity indicator
                -- High ratio = clutter edge = reduce scale (fewer false alarms)
                -- Low ratio = uniform noise = nominal scale
                
                mean_est <= resize(sum_refs / N_REF, DATA_WIDTH);
                
                -- Simple adaptive: compare ranked value to mean
                -- If ranked >> mean, we're near a strong target, reduce sensitivity
                -- If ranked ≈ mean, uniform noise, use nominal
                if ranked_val > shift_right(sum_refs(DATA_WIDTH+7 downto 8), 1) then
                    -- High clutter: use higher threshold
                    scale := SCALE_MAX;
                elsif ranked_val < shift_right(sum_refs(DATA_WIDTH+7 downto 8), 2) then
                    -- Low/uniform noise: can be more sensitive
                    scale := SCALE_MIN;
                else
                    scale := SCALE_NOM;
                end if;
                
                --------------------------------------------------------
                -- 6. Threshold & Detection
                --------------------------------------------------------
                threshold := resize(ranked_val, threshold'length) * to_unsigned(scale, 4);
                cut_val := range_window(CUT_D, CUT_R);
                
                -- Pipeline stage 1
                out_valid_p1 <= window_valid and s_axis_tvalid;
                out_last_p1  <= s_axis_tlast;
                
                -- Pipeline stage 2 with decision
                out_valid_p2 <= out_valid_p1;
                out_last_p2  <= out_last_p1;
                
                if cut_val > threshold(DATA_WIDTH-1 downto 0) then
                    out_data_p2 <= std_logic_vector(cut_val);
                else
                    out_data_p2 <= (others => '0');
                end if;
                
                -- Debug outputs
                dbg_threshold <= std_logic_vector(threshold(DATA_WIDTH-1 downto 0));
                dbg_scale     <= std_logic_vector(to_unsigned(scale, 4));
                
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
