library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use std.textio.all;
use IEEE.std_logic_textio.all;

entity tb_corner_turner is
end tb_corner_turner;

architecture Behavioral of tb_corner_turner is

    component corner_turner is
        Generic (
            N_RANGE   : integer := 1024;
            N_DOPPLER : integer := 128;
            DATA_WIDTH : integer := 32
        );
        Port (
            aclk      : in  STD_LOGIC;
            aresetn   : in  STD_LOGIC;
            s_axis_tdata  : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid : in  STD_LOGIC;
            s_axis_tready : out STD_LOGIC;
            s_axis_tlast  : in  STD_LOGIC;
            m_axis_tdata  : out STD_LOGIC_VECTOR(31 downto 0);
            m_axis_tvalid : out STD_LOGIC;
            m_axis_tready : in  STD_LOGIC;
            m_axis_tlast  : out STD_LOGIC
        );
    end component;

    constant N_RANGE : integer := 1024;
    constant N_DOPPLER : integer := 128;
    constant CLK_PERIOD : time := 10 ns;

    signal aclk : std_logic := '0';
    signal aresetn : std_logic := '0';
    
    signal s_tdata : std_logic_vector(31 downto 0) := (others => '0');
    signal s_tvalid : std_logic := '0';
    signal s_tready : std_logic;
    signal s_tlast : std_logic := '0';
    
    signal m_tdata : std_logic_vector(31 downto 0);
    signal m_tvalid : std_logic;
    signal m_tready : std_logic := '1';
    signal m_tlast : std_logic;
    
    -- Verification arrays (need 2 frames for ping-pong operation)
    type matrix_type is array (0 to 2*N_DOPPLER-1, 0 to N_RANGE-1) of integer;
    signal write_matrix : matrix_type := (others => (others => 0));
    signal read_matrix : matrix_type := (others => (others => 0));
    
    signal write_done : std_logic := '0';
    signal read_done : std_logic := '0';

begin

    uut: corner_turner
    generic map (
        N_RANGE => N_RANGE,
        N_DOPPLER => N_DOPPLER,
        DATA_WIDTH => 32
    )
    port map (
        aclk => aclk,
        aresetn => aresetn,
        s_axis_tdata => s_tdata,
        s_axis_tvalid => s_tvalid,
        s_axis_tready => s_tready,
        s_axis_tlast => s_tlast,
        m_axis_tdata => m_tdata,
        m_axis_tvalid => m_tvalid,
        m_axis_tready => m_tready,
        m_axis_tlast => m_tlast
    );

    -- Clock generation
    clk_process: process
    begin
        aclk <= '0';
        wait for CLK_PERIOD/2;
        aclk <= '1';
        wait for CLK_PERIOD/2;
    end process;

    -- ========================================================================
    -- WRITE STIMULUS: Generate test pattern and write row-wise
    -- Pattern: Each sample = (chirp_num << 16) | range_bin
    -- This allows easy verification of transpose
    -- ========================================================================
    write_process: process
        variable chirp : integer;
        variable sample : integer;
        variable test_value : integer;
    begin
        aresetn <= '0';
        s_tvalid <= '0';
        s_tlast <= '0';
        wait for 100 ns;
        aresetn <= '1';
        wait for CLK_PERIOD * 10;
        
        report "Starting write phase...";
        
        -- Write 2 frames (2 * N_DOPPLER chirps) for ping-pong operation
        -- Frame 1: Fills Bank A
        -- Frame 2: Fills Bank B while reading Bank A
        for chirp in 0 to (2*N_DOPPLER)-1 loop
            for sample in 0 to N_RANGE-1 loop
                
                -- Create unique test value: chirp_num in upper 16 bits, range_bin in lower 16
                test_value := chirp * 65536 + sample;
                write_matrix(chirp, sample) <= test_value;
                
                -- Drive AXI stream
                s_tvalid <= '1';
                s_tdata <= std_logic_vector(to_signed(test_value, 32));
                
                -- Assert TLAST at end of each chirp
                if sample = N_RANGE-1 then
                    s_tlast <= '1';
                else
                    s_tlast <= '0';
                end if;
                
                -- Wait for handshake
                wait until rising_edge(aclk);
                while s_tready = '0' loop
                    wait until rising_edge(aclk);
                end loop;
            end loop;
            
            report "Written chirp " & integer'image(chirp);
        end loop;
        
        s_tvalid <= '0';
        s_tlast <= '0';
        write_done <= '1';
        
        report "Write phase complete. Wrote " & integer'image(2*N_DOPPLER) & " chirps (2 frames).";
        wait;
    end process;

    -- ========================================================================
    -- READ MONITOR: Capture output every clock cycle
    -- ========================================================================
    read_process: process
        file output_file : text open write_mode is "corner_turner_output.txt";
        variable outline : line;
        variable sample_count : integer := 0;
        variable error_count : integer := 0;
        variable read_value : integer;
        variable expected_value : integer;
        variable range_bin : integer;
        variable doppler_bin : integer;
    begin
        wait until rising_edge(aclk);
        
        -- Capture data every clock when valid and ready (don't wait for write_done)
        if m_tvalid = '1' and m_tready = '1' then
            read_value := to_integer(signed(m_tdata));
            
            -- Calculate which range/doppler bin this sample represents
            -- Output is column-wise: range_bin varies slowly, doppler_bin varies fast
            range_bin := sample_count / N_DOPPLER;
            doppler_bin := sample_count mod N_DOPPLER;
            
            -- Store in read matrix
            read_matrix(doppler_bin, range_bin) <= read_value;
            
            -- Get expected value from write matrix
            -- Corner turner outputs Bank A data (first frame, chirps 0-127)
            -- while Bank B is being written (second frame, chirps 128-255)
            expected_value := write_matrix(doppler_bin, range_bin);
            
            -- Verify
            if read_value /= expected_value then
                report "ERROR at sample " & integer'image(sample_count) &
                       " (range=" & integer'image(range_bin) & 
                       ", doppler=" & integer'image(doppler_bin) & 
                       "): expected " & integer'image(expected_value) & 
                       ", got " & integer'image(read_value)
                       severity error;
                error_count := error_count + 1;
            end if;
            
            -- Write to file
            write(outline, range_bin);
            write(outline, string'(" "));
            write(outline, doppler_bin);
            write(outline, string'(" "));
            write(outline, read_value);
            writeline(output_file, outline);
            
            -- Check TLAST at end of each column
            if doppler_bin = N_DOPPLER-1 then
                if m_tlast /= '1' then
                    report "ERROR: TLAST not asserted at end of column " & 
                           integer'image(range_bin) severity error;
                    error_count := error_count + 1;
                end if;
            end if;
            
            sample_count := sample_count + 1;
            
            -- Progress report
            if sample_count mod 10000 = 0 then
                report "Read " & integer'image(sample_count) & " samples...";
            end if;
            
            -- Check if done
            if sample_count = N_RANGE * N_DOPPLER then
                read_done <= '1';
                
                report "====================================";
                if error_count = 0 then
                    report "SUCCESS: Corner Turner Verified!";
                    report "Matrix transpose correct.";
                else
                    report "FAILURE: " & integer'image(error_count) & " errors detected!";
                end if;
                report "Total samples: " & integer'image(sample_count);
                report "====================================";
                
                file_close(output_file);
                wait for 1 us;
                assert false report "Simulation Complete" severity failure;
            end if;
        end if;
    end process;

end Behavioral;