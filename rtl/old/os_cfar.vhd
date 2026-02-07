library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- OS-CFAR Module
-- Function: Sliding window across the data stream.
-- Logic: Sorts reference cells, picks the k-th rank, scales it, 
-- and compares to the CUT (Cell Under Test).
-- Output: Passes input magnitude if Deteced, 0 if Noise.

entity os_cfar is
    Generic (
        DATA_WIDTH   : integer := 17; -- Matches radar_core output width
        REF_CELLS    : integer := 8;  -- Reference cells per side (Total Ref = 16)
        GUARD_CELLS  : integer := 2;  -- Guard cells per side (Total Guard = 4)
        RANK_IDX     : integer := 12; -- Rank 'k': 12th largest of 16 (approx 75%)
        SCALING_MULT : integer := 4;  -- Multiplier for threshold (Alpha)
        SCALING_DIV  : integer := 1   -- Divisor for threshold (Alpha = MULT/DIV)
    );
    Port (
        aclk          : in  STD_LOGIC;
        aresetn       : in  STD_LOGIC;
        
        -- Input (from Magnitude Calc)
        s_axis_tdata  : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;
        
        -- Output (to Radar Core Output)
        m_axis_tdata  : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC
    );
end os_cfar;

architecture Behavioral of os_cfar is

    -- Window definitions
    constant WIN_SIZE : integer := 2*REF_CELLS + 2*GUARD_CELLS + 1;
    constant CUT_IDX  : integer := REF_CELLS + GUARD_CELLS;
    
    type window_t is array (0 to WIN_SIZE-1) of unsigned(DATA_WIDTH-1 downto 0);
    signal window_reg : window_t := (others => (others => '0'));
    
    -- Delay matching for control signals
    signal valid_pipe : std_logic_vector(WIN_SIZE downto 0) := (others => '0');
    signal last_pipe  : std_logic_vector(WIN_SIZE downto 0) := (others => '0');
    
    -- Sorting array (only holds Reference cells)
    constant N_REF : integer := 2 * REF_CELLS;
    type sort_array_t is array (0 to N_REF-1) of unsigned(DATA_WIDTH-1 downto 0);
    
    signal pipe_advance : std_logic;

begin

    -- Simple backpressure flow control
    pipe_advance  <= m_axis_tready; 
    s_axis_tready <= pipe_advance;

    process(aclk)
        variable refs_to_sort : sort_array_t;
        variable temp         : unsigned(DATA_WIDTH-1 downto 0);
        variable threshold    : unsigned(DATA_WIDTH-1 downto 0);
        variable cut_val      : unsigned(DATA_WIDTH-1 downto 0);
        variable k            : integer;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                window_reg    <= (others => (others => '0'));
                valid_pipe    <= (others => '0');
                last_pipe     <= (others => '0');
                m_axis_tvalid <= '0';
                m_axis_tlast  <= '0';
                m_axis_tdata  <= (others => '0');
            elsif pipe_advance = '1' then
                
                ------------------------------------------------------------
                -- 1. Shift Data into Window
                ------------------------------------------------------------
                if s_axis_tvalid = '1' then
                    window_reg(0) <= unsigned(s_axis_tdata);
                    for i in 1 to WIN_SIZE-1 loop
                        window_reg(i) <= window_reg(i-1);
                    end loop;
                end if;
                
                -- Shift control signals regardless of tvalid (flush pipeline)
                valid_pipe <= valid_pipe(WIN_SIZE-1 downto 0) & s_axis_tvalid;
                last_pipe  <= last_pipe(WIN_SIZE-1 downto 0) & s_axis_tlast;
                
                ------------------------------------------------------------
                -- 2. Extract Reference Cells (Skipping Guard & CUT)
                ------------------------------------------------------------
                -- We gather cells from before the CUT and after the CUT
                k := 0;
                -- Left side (Oldest data, at end of shift reg)
                for i in 0 to REF_CELLS-1 loop
                    refs_to_sort(k) := window_reg(WIN_SIZE - 1 - i);
                    k := k + 1;
                end loop;
                
                -- Right side (Newest data, at start of shift reg)
                for i in 0 to REF_CELLS-1 loop
                    refs_to_sort(k) := window_reg(REF_CELLS - 1 - i);
                    k := k + 1;
                end loop;

                ------------------------------------------------------------
                -- 3. Sort Reference Cells (Bubble Sort)
                ------------------------------------------------------------
                -- Note: In a high-speed design (>200MHz), this loop should 
                -- be pipelined. For standard radar processing (<100MHz) or 
                -- simulation, this behavioral description is sufficient.
                for i in 0 to N_REF-2 loop
                    for j in i+1 to N_REF-1 loop
                        if refs_to_sort(i) > refs_to_sort(j) then
                            temp := refs_to_sort(i);
                            refs_to_sort(i) := refs_to_sort(j);
                            refs_to_sort(j) := temp;
                        end if;
                    end loop;
                end loop;

                ------------------------------------------------------------
                -- 4. Threshold & Decision
                ------------------------------------------------------------
                -- Pick the k-th value (RANK_IDX)
                -- Apply scaling factor (Alpha)
                threshold := resize((refs_to_sort(RANK_IDX) * SCALING_MULT) / SCALING_DIV, DATA_WIDTH);
                
                cut_val := window_reg(CUT_IDX);
                
                -- Output Logic: If detection, output magnitude. If noise, output 0.
                if (cut_val > threshold) and (valid_pipe(CUT_IDX) = '1') then
                    m_axis_tdata <= std_logic_vector(cut_val);
                else
                    m_axis_tdata <= (others => '0');
                end if;

                m_axis_tvalid <= valid_pipe(CUT_IDX);
                m_axis_tlast  <= last_pipe(CUT_IDX);
                
            end if;
        end if;
    end process;

end Behavioral;