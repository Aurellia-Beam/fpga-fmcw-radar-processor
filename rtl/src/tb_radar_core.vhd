library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use std.textio.all;
use IEEE.std_logic_textio.all;

entity tb_radar_core is
end tb_radar_core;

architecture Behavioral of tb_radar_core is

    component radar_core_v3 is
        Generic ( N_RANGE : integer := 1024; N_DOPPLER : integer := 128 );
        Port (
            aclk, aresetn   : in  STD_LOGIC;
            s_axis_tdata    : in  STD_LOGIC_VECTOR(31 downto 0);
            s_axis_tvalid   : in  STD_LOGIC;
            s_axis_tlast    : in  STD_LOGIC;
            s_axis_tready   : out STD_LOGIC;
            m_axis_tdata    : out STD_LOGIC_VECTOR(16 downto 0);
            m_axis_tvalid   : out STD_LOGIC;
            m_axis_tlast    : out STD_LOGIC;
            m_axis_tready   : in  STD_LOGIC;
            rdm_range_bin   : out STD_LOGIC_VECTOR(9 downto 0);
            rdm_doppler_bin : out STD_LOGIC_VECTOR(6 downto 0);
            status_range_fft_done   : out STD_LOGIC;
            status_doppler_fft_done : out STD_LOGIC;
            status_frame_complete   : out STD_LOGIC
        );
    end component;

    constant N_RANGE    : integer := 1024;
    constant N_DOPPLER  : integer := 128;
    constant CLK_PERIOD : time := 10 ns;
    
    constant TARGET_1_RANGE   : integer := 100;
    constant TARGET_1_DOPPLER : real := 5.0;
    constant TARGET_1_AMP     : real := 8000.0;
    
    constant TARGET_2_RANGE   : integer := 500;
    constant TARGET_2_DOPPLER : real := -10.0;
    constant TARGET_2_AMP     : real := 5000.0;
    constant NOISE_FLOOR      : real := 20.0;

    signal aclk, aresetn : std_logic := '0';
    signal s_axis_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axis_tvalid, s_axis_tlast : std_logic := '0';
    signal s_axis_tready : std_logic;
    signal m_axis_tdata  : std_logic_vector(16 downto 0);
    signal m_axis_tvalid, m_axis_tlast : std_logic;
    signal m_axis_tready : std_logic := '1';
    signal rdm_range_bin   : std_logic_vector(9 downto 0);
    signal rdm_doppler_bin : std_logic_vector(6 downto 0);
    signal status_range_fft_done, status_doppler_fft_done, status_frame_complete : std_logic;
    signal sim_done : std_logic := '0';
    signal output_sample_count : integer := 0;

    constant TIMEOUT_CYCLES : integer := 10000000;

begin

    uut: radar_core_v3
    generic map ( N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tlast => s_axis_tlast, s_axis_tready => s_axis_tready,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tlast => m_axis_tlast, m_axis_tready => m_axis_tready,
        rdm_range_bin => rdm_range_bin, rdm_doppler_bin => rdm_doppler_bin,
        status_range_fft_done => status_range_fft_done,
        status_doppler_fft_done => status_doppler_fft_done,
        status_frame_complete => status_frame_complete
    );

    clk_proc: process
    begin
        while sim_done = '0' loop
            aclk <= '0'; wait for CLK_PERIOD/2;
            aclk <= '1'; wait for CLK_PERIOD/2;
        end loop;
        wait;
    end process;

    stimulus_proc: process
        variable seed1, seed2 : positive := 1;
        variable rand, i_acc, q_acc, phase : real;
        variable i_val, q_val : integer;
    begin
        aresetn <= '0';
        wait until rising_edge(aclk);
        wait until rising_edge(aclk);
        wait until rising_edge(aclk);
        aresetn <= '1';
        
        report "Waiting for core ready...";
        wait until s_axis_tready = '1' and rising_edge(aclk);
        report "Core ready.";
        
        for cpi in 0 to 0 loop
            report "CPI " & integer'image(cpi);
            
            for chirp_num in 0 to N_DOPPLER-1 loop
                for sample_num in 0 to N_RANGE-1 loop
                    i_acc := 0.0;
                    q_acc := 0.0;
                    
                    phase := 2.0 * MATH_PI * (
                        real(TARGET_1_RANGE) * real(sample_num) / real(N_RANGE) +
                        TARGET_1_DOPPLER * real(chirp_num) / real(N_DOPPLER));
                    i_acc := i_acc + TARGET_1_AMP * cos(phase);
                    q_acc := q_acc + TARGET_1_AMP * sin(phase);
                    
                    phase := 2.0 * MATH_PI * (
                        real(TARGET_2_RANGE) * real(sample_num) / real(N_RANGE) +
                        TARGET_2_DOPPLER * real(chirp_num) / real(N_DOPPLER));
                    i_acc := i_acc + TARGET_2_AMP * cos(phase);
                    q_acc := q_acc + TARGET_2_AMP * sin(phase);
                    
                    uniform(seed1, seed2, rand);
                    i_acc := i_acc + NOISE_FLOOR * (rand - 0.5) * 2.0;
                    uniform(seed1, seed2, rand);
                    q_acc := q_acc + NOISE_FLOOR * (rand - 0.5) * 2.0;
                    
                    i_val := integer(i_acc);
                    q_val := integer(q_acc);
                    if i_val > 32767 then i_val := 32767; elsif i_val < -32768 then i_val := -32768; end if;
                    if q_val > 32767 then q_val := 32767; elsif q_val < -32768 then q_val := -32768; end if;
                    
                    s_axis_tdata <= std_logic_vector(to_signed(q_val, 16)) & std_logic_vector(to_signed(i_val, 16));
                    s_axis_tvalid <= '1';
                    s_axis_tlast <= '1' when sample_num = N_RANGE-1 else '0';
                    
                    wait until rising_edge(aclk);
                    while s_axis_tready = '0' loop
                        wait until rising_edge(aclk);
                    end loop;
                end loop;
            end loop;
        end loop;
        
        s_axis_tvalid <= '0';
        s_axis_tlast <= '0';
        
        report "Waiting for processing...";
        wait until status_frame_complete = '1' and rising_edge(aclk);
        
        for i in 0 to 1000 loop
            wait until rising_edge(aclk);
            exit when m_axis_tvalid = '0';
        end loop;
        
        sim_done <= '1';
        report "Complete. Samples: " & integer'image(output_sample_count);
        wait;
    end process;

    watchdog_proc: process
        variable cycle_count : integer := 0;
    begin
        wait until aresetn = '1';
        while sim_done = '0' loop
            wait until rising_edge(aclk);
            cycle_count := cycle_count + 1;
            if cycle_count > TIMEOUT_CYCLES then
                report "TIMEOUT" severity failure;
            end if;
        end loop;
        wait;
    end process;

    monitor_proc: process
        file output_file : text open write_mode is "radar_output.txt";
        variable outline : line;
        variable mag_val, range_idx, doppler_idx : integer;
    begin
        wait until aresetn = '1';
        
        loop
            wait until rising_edge(aclk);
            exit when sim_done = '1';
            
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                mag_val := to_integer(unsigned(m_axis_tdata));
                range_idx := to_integer(unsigned(rdm_range_bin));
                doppler_idx := to_integer(unsigned(rdm_doppler_bin));
                
                write(outline, range_idx);
                write(outline, string'(" "));
                write(outline, doppler_idx);
                write(outline, string'(" "));
                write(outline, mag_val);
                writeline(output_file, outline);
                
                output_sample_count <= output_sample_count + 1;
                
                if mag_val > 50000 then
                     report "Detection: R=" & integer'image(range_idx) & 
                            " D=" & integer'image(doppler_idx) & 
                            " M=" & integer'image(mag_val);
                end if;
            end if;
        end loop;
        
        file_close(output_file);
        wait;
    end process;

end Behavioral;