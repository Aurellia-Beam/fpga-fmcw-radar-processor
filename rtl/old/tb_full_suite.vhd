library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL; -- Crucial for random generation
use std.textio.all;
use IEEE.std_logic_textio.all;

entity tb_full_suite is
end tb_full_suite;

architecture Behavioral of tb_full_suite is

    -- Component Declaration
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
            status_frame_complete : out STD_LOGIC
        );
    end component;

    -- Constants
    constant N_RANGE    : integer := 1024;
    constant N_DOPPLER  : integer := 128;
    constant CLK_PERIOD : time := 10 ns;
    constant NUM_FRAMES : integer := 4; -- How many frames to simulate

    -- Signals
    signal aclk          : std_logic := '0';
    signal aresetn       : std_logic := '0';
    signal s_axis_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tlast  : std_logic := '0';
    signal s_axis_tready : std_logic;
    
    signal m_axis_tdata    : std_logic_vector(16 downto 0);
    signal m_axis_tvalid   : std_logic;
    signal m_axis_tready   : std_logic := '1';
    signal rdm_range_bin   : std_logic_vector(9 downto 0);
    signal rdm_doppler_bin : std_logic_vector(6 downto 0);
    signal frame_done      : std_logic;
    
    signal sim_finished    : boolean := false;

begin

    -- Instantiate the Full Radar Core (with CFAR inside)
    uut: radar_core_v3
    generic map ( N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tlast => s_axis_tlast, s_axis_tready => s_axis_tready,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tlast => open, m_axis_tready => m_axis_tready,
        rdm_range_bin => rdm_range_bin, rdm_doppler_bin => rdm_doppler_bin,
        status_frame_complete => frame_done
    );

    -- Clock Process
    aclk <= not aclk after CLK_PERIOD/2 when not sim_finished else '0';

    -- Stimulus Process: Injects Random Targets
    process
        variable seed1, seed2 : positive := 1234; -- Seeds for random generator
        variable rand_val     : real;             -- 0.0 to 1.0
        
        -- Target Parameters (Randomized per frame)
        variable tgt_range    : real;
        variable tgt_doppler  : real;
        variable tgt_amp      : real := 20000.0;
        variable noise_floor  : real := 100.0;
        
        variable phase, i_acc, q_acc : real;
        variable i_int, q_int : integer;
        
    begin
        aresetn <= '0';
        wait for 100 ns;
        aresetn <= '1';
        wait until s_axis_tready = '1';

        -- Loop through multiple frames (CPIs)
        for frame in 1 to NUM_FRAMES loop
            report "Simulating Frame " & integer'image(frame);
            
            -- 1. Randomize Target Location for this frame
            uniform(seed1, seed2, rand_val);
            tgt_range := rand_val * 900.0 + 50.0; -- Random Range: 50 to 950
            
            uniform(seed1, seed2, rand_val);
            tgt_doppler := rand_val * 100.0; -- Random Doppler: 0 to 100
            
            report "  > Injecting Target at Range=" & integer'image(integer(tgt_range)) & 
                   " Doppler=" & integer'image(integer(tgt_doppler));

            -- 2. Generate Waveform (Chirps x Samples)
            for c in 0 to N_DOPPLER-1 loop
                for s in 0 to N_RANGE-1 loop
                    
                    -- Calculate Phase: 2*pi * ( (Range*s)/N + (Doppler*c)/M )
                    phase := 2.0 * MATH_PI * ( 
                             (tgt_range * real(s) / real(N_RANGE)) + 
                             (tgt_doppler * real(c) / real(N_DOPPLER)) 
                             );
                             
                    -- Base signal
                    i_acc := tgt_amp * cos(phase);
                    q_acc := tgt_amp * sin(phase);
                    
                    -- Add Random Noise
                    uniform(seed1, seed2, rand_val);
                    i_acc := i_acc + (rand_val - 0.5) * 2.0 * noise_floor;
                    uniform(seed1, seed2, rand_val);
                    q_acc := q_acc + (rand_val - 0.5) * 2.0 * noise_floor;
                    
                    -- Convert to Fixed Point
                    i_int := integer(i_acc);
                    q_int := integer(q_acc);
                    
                    -- Drive AXI Stream
                    s_axis_tdata <= std_logic_vector(to_signed(q_int, 16)) & std_logic_vector(to_signed(i_int, 16));
                    s_axis_tvalid <= '1';
                    
                    if s = N_RANGE-1 then s_axis_tlast <= '1'; else s_axis_tlast <= '0'; end if;
                    
                    wait until rising_edge(aclk);
                    while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
                    
                end loop;
            end loop;
            
            s_axis_tvalid <= '0';
            s_axis_tlast  <= '0';
            
            -- Wait for processing to finish before starting next frame
            wait until frame_done = '1';
            wait for 1 us; 
            
        end loop;

        report "All Frames Complete.";
        sim_finished <= true;
        wait;
    end process;

    -- File Writer Process
    process(aclk)
        file outfile : text open write_mode is "radar_heatmap_data.txt";
        variable outline : line;
        variable r_idx, d_idx, mag_val : integer;
    begin
        if rising_edge(aclk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                r_idx := to_integer(unsigned(rdm_range_bin));
                d_idx := to_integer(unsigned(rdm_doppler_bin));
                mag_val := to_integer(unsigned(m_axis_tdata));
                
                -- Format: Range, Doppler, Magnitude
                -- Only write non-zero values to save space/time (sparse format)
                -- if mag_val > 0 then
                    write(outline, r_idx);
                    write(outline, string'(" "));
                    write(outline, d_idx);
                    write(outline, string'(" "));
                    write(outline, mag_val);
                    writeline(outfile, outline);
                -- end if;
            end if;
        end if;
    end process;

end Behavioral;