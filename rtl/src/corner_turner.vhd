library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity corner_turner is
    Generic (
        N_RANGE    : integer := 1024;
        N_DOPPLER  : integer := 128;
        DATA_WIDTH : integer := 32
    );
    Port (
        aclk      : in  STD_LOGIC;
        aresetn   : in  STD_LOGIC;
        s_axis_tdata  : in  STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        s_axis_tvalid : in  STD_LOGIC;
        s_axis_tready : out STD_LOGIC;
        s_axis_tlast  : in  STD_LOGIC;
        m_axis_tdata  : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        m_axis_tvalid : out STD_LOGIC;
        m_axis_tready : in  STD_LOGIC;
        m_axis_tlast  : out STD_LOGIC;
        frame_complete : out STD_LOGIC;
        overflow_error : out STD_LOGIC
    );
end corner_turner;

architecture Behavioral of corner_turner is

    constant MATRIX_SIZE : integer := N_RANGE * N_DOPPLER;
    constant ADDR_WIDTH  : integer := 17;
    
    type ram_type is array (0 to MATRIX_SIZE-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
    signal ram_bank0 : ram_type := (others => (others => '0'));
    signal ram_bank1 : ram_type := (others => (others => '0'));
    
    attribute ram_style : string;
    attribute ram_style of ram_bank0 : signal is "block";
    attribute ram_style of ram_bank1 : signal is "block";
    
    -- Write side
    signal wr_chirp     : unsigned(6 downto 0) := (others => '0');
    signal wr_sample    : unsigned(9 downto 0) := (others => '0');
    signal wr_bank_sel  : std_logic := '0';
    signal wr_frame_done: std_logic := '0';
    signal s_tready_int : std_logic := '0';
    
    -- Read side
    signal rd_range     : unsigned(9 downto 0) := (others => '0');
    signal rd_doppler   : unsigned(6 downto 0) := (others => '0');
    signal rd_bank_sel  : std_logic := '0';
    signal rd_active    : std_logic := '0';
    signal rd_frame_done: std_logic := '0';
    signal first_frame  : std_logic := '1';
    
    -- Pipeline
    signal rd_data_p1   : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal rd_valid_p1  : std_logic := '0';
    signal rd_last_p1   : std_logic := '0';
    signal rd_data_p2   : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal rd_valid_p2  : std_logic := '0';
    signal rd_last_p2   : std_logic := '0';
    
    signal out_valid    : std_logic := '0';
    signal out_data     : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal out_last     : std_logic := '0';
    signal pipe_stall   : std_logic := '0';
    signal overflow_flag: std_logic := '0';
    
    -- Explicit address calculations
    signal wr_addr : integer range 0 to MATRIX_SIZE-1;
    signal rd_addr : integer range 0 to MATRIX_SIZE-1;

begin

    s_axis_tready  <= s_tready_int;
    m_axis_tdata   <= out_data;
    m_axis_tvalid  <= out_valid;
    m_axis_tlast   <= out_last;
    frame_complete <= rd_frame_done;
    overflow_error <= overflow_flag;
    
    -- Write: row-major = chirp * N_RANGE + sample
    wr_addr <= to_integer(wr_chirp) * N_RANGE + to_integer(wr_sample);
    
    -- Read: column-major = range + doppler * N_RANGE
    rd_addr <= to_integer(rd_range) + to_integer(rd_doppler) * N_RANGE;

    write_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                wr_chirp      <= (others => '0');
                wr_sample     <= (others => '0');
                wr_bank_sel   <= '0';
                wr_frame_done <= '0';
                s_tready_int  <= '0';
                overflow_flag <= '0';
            else
                s_tready_int  <= '1';
                wr_frame_done <= '0';
                overflow_flag <= '0';
                
                if wr_frame_done = '1' and rd_active = '1' and first_frame = '0' then
                    overflow_flag <= '1';
                end if;
                
                if s_axis_tvalid = '1' and s_tready_int = '1' then
                    if wr_bank_sel = '0' then
                        ram_bank0(wr_addr) <= s_axis_tdata;
                    else
                        ram_bank1(wr_addr) <= s_axis_tdata;
                    end if;
                    
                    if s_axis_tlast = '1' then
                        wr_sample <= (others => '0');
                        if wr_chirp = N_DOPPLER - 1 then
                            wr_chirp      <= (others => '0');
                            wr_frame_done <= '1';
                            wr_bank_sel   <= wr_bank_sel;
                        else
                            wr_chirp <= wr_chirp + 1;
                        end if;
                    else
                        wr_sample <= wr_sample + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    read_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                rd_range      <= (others => '0');
                rd_doppler    <= (others => '0');
                rd_bank_sel   <= '0';
                rd_active     <= '0';
                rd_frame_done <= '0';
                first_frame   <= '1';
                rd_valid_p1   <= '0';
                rd_last_p1    <= '0';
                rd_valid_p2   <= '0';
                rd_last_p2    <= '0';
            else
                rd_frame_done <= '0';
                pipe_stall <= out_valid and (not m_axis_tready);
                
                if wr_frame_done = '1' and rd_active = '0' then
                    rd_active    <= '1';
                    rd_range     <= (others => '0');
                    rd_doppler   <= (others => '0');
                    rd_bank_sel  <= not wr_bank_sel;
                    first_frame  <= '0';
                end if;
                
                if pipe_stall = '0' then
                    if rd_active = '1' then
                        if rd_bank_sel = '0' then
                            rd_data_p1 <= ram_bank0(rd_addr);
                        else
                            rd_data_p1 <= ram_bank1(rd_addr);
                        end if;
                        
                        rd_valid_p1 <= '1';
                        rd_last_p1  <= '1' when rd_doppler = N_DOPPLER - 1 else '0';
                        
                        -- Advance: doppler varies fast (output one range bin's worth of Doppler samples)
                        if rd_doppler = N_DOPPLER - 1 then
                            rd_doppler <= (others => '0');
                            if rd_range = N_RANGE - 1 then
                                rd_range      <= (others => '0');
                                rd_active     <= '0';
                                rd_frame_done <= '1';
                            else
                                rd_range <= rd_range + 1;
                            end if;
                        else
                            rd_doppler <= rd_doppler + 1;
                        end if;
                    else
                        rd_valid_p1 <= '0';
                        rd_last_p1  <= '0';
                    end if;
                    
                    rd_data_p2  <= rd_data_p1;
                    rd_valid_p2 <= rd_valid_p1;
                    rd_last_p2  <= rd_last_p1;
                end if;
            end if;
        end if;
    end process;

    output_proc: process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                out_valid <= '0';
                out_data  <= (others => '0');
                out_last  <= '0';
            elsif m_axis_tready = '1' or out_valid = '0' then
                out_valid <= rd_valid_p2;
                out_data  <= rd_data_p2;
                out_last  <= rd_last_p2;
            end if;
        end if;
    end process;

end Behavioral;