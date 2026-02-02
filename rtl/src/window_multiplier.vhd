library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity window_multiplier is
    Generic (
        DATA_WIDTH : integer := 32;
        N_SAMPLES  : integer := 1024;
        COEF_WIDTH : integer := 16
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
        saturation_flag : out STD_LOGIC
    );
end window_multiplier;

architecture Behavioral of window_multiplier is

    constant ROM_SIZE : integer := N_SAMPLES / 2;
    type coef_rom_type is array (0 to ROM_SIZE-1) of signed(COEF_WIDTH-1 downto 0);
    
    function init_hamming_rom return coef_rom_type is
        variable rom : coef_rom_type;
        variable angle, coef_real : real;
        variable coef_int : integer;
        constant PI : real := 3.14159265358979323846;
    begin
        for i in 0 to ROM_SIZE-1 loop
            angle := 2.0 * PI * real(i) / real(N_SAMPLES - 1);
            coef_real := 0.54 - 0.46 * cos(angle);
            coef_int := integer(coef_real * 32767.0);
            if coef_int > 32767 then coef_int := 32767; end if;
            if coef_int < 0 then coef_int := 0; end if;
            rom(i) := to_signed(coef_int, COEF_WIDTH);
        end loop;
        return rom;
    end function;
    
    constant HAMMING_ROM : coef_rom_type := init_hamming_rom;
    
    attribute rom_style : string;
    attribute rom_style of HAMMING_ROM : constant is "block";

    signal sample_idx : unsigned(9 downto 0) := (others => '0');
    
    signal s1_i, s1_q   : signed(15 downto 0) := (others => '0');
    signal s1_coef      : signed(COEF_WIDTH-1 downto 0) := (others => '0');
    signal s1_valid     : std_logic := '0';
    signal s1_last      : std_logic := '0';
    
    signal s2_mult_i, s2_mult_q : signed(31 downto 0) := (others => '0');
    signal s2_valid             : std_logic := '0';
    signal s2_last              : std_logic := '0';
    
    signal s3_i_rounded, s3_q_rounded : signed(15 downto 0) := (others => '0');
    signal s3_valid                   : std_logic := '0';
    signal s3_last                    : std_logic := '0';
    
    signal pipe_advance : std_logic;
    signal s_ready_int  : std_logic := '0';
    signal sat_flag     : std_logic := '0';

begin

    pipe_advance  <= m_axis_tready or (not s3_valid);
    s_axis_tready <= s_ready_int;

    stage1_proc: process(aclk)
        variable v_rom_addr : integer range 0 to ROM_SIZE-1;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s1_i        <= (others => '0');
                s1_q        <= (others => '0');
                s1_coef     <= (others => '0');
                s1_valid    <= '0';
                s1_last     <= '0';
                sample_idx  <= (others => '0');
                s_ready_int <= '0';
            else
                s_ready_int <= pipe_advance;
                
                if pipe_advance = '1' then
                    if s_axis_tvalid = '1' and s_ready_int = '1' then
                        s1_i <= signed(s_axis_tdata(15 downto 0));
                        s1_q <= signed(s_axis_tdata(31 downto 16));
                        
                        -- Symmetric window: mirror second half
                        if sample_idx < ROM_SIZE then
                            v_rom_addr := to_integer(sample_idx(8 downto 0));
                        else
                            v_rom_addr := N_SAMPLES - 1 - to_integer(sample_idx);
                        end if;
                        if v_rom_addr > ROM_SIZE - 1 then
                            v_rom_addr := ROM_SIZE - 1;
                        end if;
                        
                        s1_coef <= HAMMING_ROM(v_rom_addr);
                        s1_valid <= '1';
                        s1_last  <= s_axis_tlast;
                        
                        if s_axis_tlast = '1' then
                            sample_idx <= (others => '0');
                        else
                            sample_idx <= sample_idx + 1;
                        end if;
                    else
                        s1_valid <= '0';
                        s1_last  <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process;

    stage2_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s2_mult_i <= (others => '0');
                s2_mult_q <= (others => '0');
                s2_valid  <= '0';
                s2_last   <= '0';
            elsif pipe_advance = '1' then
                s2_mult_i <= s1_i * s1_coef;
                s2_mult_q <= s1_q * s1_coef;
                s2_valid  <= s1_valid;
                s2_last   <= s1_last;
            end if;
        end if;
    end process;

    -- Q15*Q15=Q30, extract Q15 with convergent rounding
    stage3_proc: process(aclk)
        variable i_round, q_round     : signed(31 downto 0);
        variable i_shifted, q_shifted : signed(16 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s3_i_rounded <= (others => '0');
                s3_q_rounded <= (others => '0');
                s3_valid     <= '0';
                s3_last      <= '0';
                sat_flag     <= '0';
            elsif pipe_advance = '1' then
                i_round := s2_mult_i + to_signed(16384, 32);
                q_round := s2_mult_q + to_signed(16384, 32);
                i_shifted := i_round(30 downto 14);
                q_shifted := q_round(30 downto 14);
                
                sat_flag <= '0';
                
                if i_shifted > 32767 then
                    s3_i_rounded <= to_signed(32767, 16);
                    sat_flag <= '1';
                elsif i_shifted < -32768 then
                    s3_i_rounded <= to_signed(-32768, 16);
                    sat_flag <= '1';
                else
                    s3_i_rounded <= i_shifted(15 downto 0);
                end if;
                
                if q_shifted > 32767 then
                    s3_q_rounded <= to_signed(32767, 16);
                    sat_flag <= '1';
                elsif q_shifted < -32768 then
                    s3_q_rounded <= to_signed(-32768, 16);
                    sat_flag <= '1';
                else
                    s3_q_rounded <= q_shifted(15 downto 0);
                end if;
                
                s3_valid <= s2_valid;
                s3_last  <= s2_last;
            end if;
        end if;
    end process;

    m_axis_tdata(15 downto 0)  <= std_logic_vector(s3_i_rounded);
    m_axis_tdata(31 downto 16) <= std_logic_vector(s3_q_rounded);
    m_axis_tvalid              <= s3_valid;
    m_axis_tlast               <= s3_last;
    saturation_flag            <= sat_flag;

end Behavioral;