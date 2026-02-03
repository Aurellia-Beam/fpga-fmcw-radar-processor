library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

-- ADR_tws_tracker.vhd
-- Track-While-Scan Processor
-- Features:
--   - Multi-target track file (up to MAX_TRACKS simultaneous)
--   - Simplified Kalman filter (alpha-beta tracker)
--   - Track initiation, confirmation, coast, and deletion
--   - Detection-to-track association (nearest neighbor)

entity ADR_tws_tracker is
    Generic (
        MAX_TRACKS    : integer := 32;
        N_RANGE       : integer := 1024;
        N_DOPPLER     : integer := 128;
        -- Track management thresholds
        INIT_HITS     : integer := 2;     -- Hits to confirm track
        COAST_MAX     : integer := 5;     -- Scans before track drop
        ASSOC_GATE_R  : integer := 10;    -- Range association gate (bins)
        ASSOC_GATE_D  : integer := 5;     -- Doppler association gate (bins)
        -- Kalman gains (Q8 fixed point: 256 = 1.0)
        ALPHA_GAIN    : integer := 128;   -- Position gain (0.5)
        BETA_GAIN     : integer := 64     -- Velocity gain (0.25)
    );
    Port (
        aclk          : in  STD_LOGIC;
        aresetn       : in  STD_LOGIC;
        
        -- Detection input (from CFAR)
        det_valid     : in  STD_LOGIC;
        det_range     : in  STD_LOGIC_VECTOR(9 downto 0);
        det_doppler   : in  STD_LOGIC_VECTOR(6 downto 0);
        det_magnitude : in  STD_LOGIC_VECTOR(16 downto 0);
        det_last      : in  STD_LOGIC;  -- End of scan
        
        -- Track output interface
        trk_valid     : out STD_LOGIC;
        trk_id        : out STD_LOGIC_VECTOR(5 downto 0);
        trk_range     : out STD_LOGIC_VECTOR(11 downto 0);  -- Q2 fixed point
        trk_doppler   : out STD_LOGIC_VECTOR(8 downto 0);   -- Q2 fixed point
        trk_vel_r     : out STD_LOGIC_VECTOR(9 downto 0);   -- Range rate (signed)
        trk_vel_d     : out STD_LOGIC_VECTOR(7 downto 0);   -- Doppler rate (signed)
        trk_quality   : out STD_LOGIC_VECTOR(3 downto 0);   -- Track quality
        trk_status    : out STD_LOGIC_VECTOR(1 downto 0);   -- 00=tent, 01=firm, 10=coast
        
        -- Status
        active_tracks : out STD_LOGIC_VECTOR(5 downto 0);
        scan_complete : out STD_LOGIC
    );
end ADR_tws_tracker;

architecture Behavioral of ADR_tws_tracker is

    -- Track status codes
    constant TRK_FREE      : std_logic_vector(1 downto 0) := "00";
    constant TRK_TENTATIVE : std_logic_vector(1 downto 0) := "01";
    constant TRK_FIRM      : std_logic_vector(1 downto 0) := "10";
    constant TRK_COAST     : std_logic_vector(1 downto 0) := "11";
    
    -- Track record (all in Q8 fixed point where noted)
    type track_record is record
        active    : std_logic;
        status    : std_logic_vector(1 downto 0);
        -- Position (Q2: 2 fractional bits)
        range_pos : signed(11 downto 0);
        dopp_pos  : signed(8 downto 0);
        -- Velocity (bins per scan, Q4)
        range_vel : signed(9 downto 0);
        dopp_vel  : signed(7 downto 0);
        -- Track management
        hit_count : unsigned(3 downto 0);
        miss_count: unsigned(3 downto 0);
        quality   : unsigned(3 downto 0);
        age       : unsigned(7 downto 0);
        -- Last measurement
        last_mag  : unsigned(16 downto 0);
    end record;
    
    type track_file_t is array (0 to MAX_TRACKS-1) of track_record;
    signal track_file : track_file_t;
    
    -- Detection buffer for current scan
    constant MAX_DETS_PER_SCAN : integer := 64;
    type det_record is record
        valid     : std_logic;
        range_bin : unsigned(9 downto 0);
        dopp_bin  : unsigned(6 downto 0);
        magnitude : unsigned(16 downto 0);
        associated: std_logic;
    end record;
    type det_buffer_t is array (0 to MAX_DETS_PER_SCAN-1) of det_record;
    signal det_buffer : det_buffer_t;
    signal det_count  : unsigned(5 downto 0) := (others => '0');
    
    -- State machine
    type state_t is (ST_COLLECT, ST_PREDICT, ST_ASSOCIATE, ST_UPDATE, 
                     ST_INITIATE, ST_MAINTAIN, ST_OUTPUT);
    signal state : state_t := ST_COLLECT;
    
    -- Working variables
    signal scan_number   : unsigned(15 downto 0) := (others => '0');
    signal proc_trk_idx  : unsigned(5 downto 0) := (others => '0');
    signal proc_det_idx  : unsigned(5 downto 0) := (others => '0');
    signal best_det_idx  : unsigned(5 downto 0) := (others => '0');
    signal best_distance : unsigned(15 downto 0) := (others => '1');
    signal num_active    : unsigned(5 downto 0) := (others => '0');
    
    -- Output registers
    signal out_valid : std_logic := '0';
    signal out_idx   : unsigned(5 downto 0) := (others => '0');

begin

    active_tracks <= std_logic_vector(num_active);

    process(aclk)
        variable dist_r, dist_d : integer;
        variable distance       : unsigned(15 downto 0);
        variable pred_range     : signed(11 downto 0);
        variable pred_dopp      : signed(8 downto 0);
        variable meas_range     : signed(11 downto 0);
        variable meas_dopp      : signed(8 downto 0);
        variable innov_r        : signed(11 downto 0);
        variable innov_d        : signed(8 downto 0);
        variable new_trk_idx    : integer;
        variable active_cnt     : unsigned(5 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                state <= ST_COLLECT;
                det_count <= (others => '0');
                scan_number <= (others => '0');
                proc_trk_idx <= (others => '0');
                proc_det_idx <= (others => '0');
                out_valid <= '0';
                scan_complete <= '0';
                num_active <= (others => '0');
                
                -- Initialize track file
                for i in 0 to MAX_TRACKS-1 loop
                    track_file(i).active <= '0';
                    track_file(i).status <= TRK_FREE;
                    track_file(i).range_pos <= (others => '0');
                    track_file(i).dopp_pos <= (others => '0');
                    track_file(i).range_vel <= (others => '0');
                    track_file(i).dopp_vel <= (others => '0');
                    track_file(i).hit_count <= (others => '0');
                    track_file(i).miss_count <= (others => '0');
                    track_file(i).quality <= (others => '0');
                    track_file(i).age <= (others => '0');
                end loop;
                
                for i in 0 to MAX_DETS_PER_SCAN-1 loop
                    det_buffer(i).valid <= '0';
                    det_buffer(i).associated <= '0';
                end loop;
                
            else
                scan_complete <= '0';
                out_valid <= '0';
                
                case state is
                
                -- =========================================================
                -- Collect detections for this scan
                -- =========================================================
                when ST_COLLECT =>
                    if det_valid = '1' then
                        if det_count < MAX_DETS_PER_SCAN then
                            det_buffer(to_integer(det_count)).valid <= '1';
                            det_buffer(to_integer(det_count)).range_bin <= unsigned(det_range);
                            det_buffer(to_integer(det_count)).dopp_bin <= unsigned(det_doppler);
                            det_buffer(to_integer(det_count)).magnitude <= unsigned(det_magnitude);
                            det_buffer(to_integer(det_count)).associated <= '0';
                            det_count <= det_count + 1;
                        end if;
                    end if;
                    
                    if det_last = '1' then
                        state <= ST_PREDICT;
                        proc_trk_idx <= (others => '0');
                    end if;
                
                -- =========================================================
                -- Predict track positions
                -- =========================================================
                when ST_PREDICT =>
                    if track_file(to_integer(proc_trk_idx)).active = '1' then
                        -- Predict: x' = x + v*dt (dt=1 scan)
                        pred_range := track_file(to_integer(proc_trk_idx)).range_pos +
                                     resize(track_file(to_integer(proc_trk_idx)).range_vel, 12);
                        pred_dopp := track_file(to_integer(proc_trk_idx)).dopp_pos +
                                    resize(track_file(to_integer(proc_trk_idx)).dopp_vel, 9);
                        
                        track_file(to_integer(proc_trk_idx)).range_pos <= pred_range;
                        track_file(to_integer(proc_trk_idx)).dopp_pos <= pred_dopp;
                        track_file(to_integer(proc_trk_idx)).age <= 
                            track_file(to_integer(proc_trk_idx)).age + 1;
                    end if;
                    
                    if proc_trk_idx = MAX_TRACKS - 1 then
                        state <= ST_ASSOCIATE;
                        proc_trk_idx <= (others => '0');
                    else
                        proc_trk_idx <= proc_trk_idx + 1;
                    end if;
                
                -- =========================================================
                -- Associate detections to tracks (nearest neighbor)
                -- =========================================================
                when ST_ASSOCIATE =>
                    if track_file(to_integer(proc_trk_idx)).active = '1' then
                        -- Find nearest unassociated detection
                        best_distance <= (others => '1');
                        best_det_idx <= (others => '1');
                        
                        for d in 0 to MAX_DETS_PER_SCAN-1 loop
                            if det_buffer(d).valid = '1' and det_buffer(d).associated = '0' then
                                -- Distance calculation (in Q2 space)
                                dist_r := abs(to_integer(track_file(to_integer(proc_trk_idx)).range_pos) - 
                                             to_integer(shift_left(resize(det_buffer(d).range_bin, 12), 2)));
                                dist_d := abs(to_integer(track_file(to_integer(proc_trk_idx)).dopp_pos) - 
                                             to_integer(shift_left(resize(det_buffer(d).dopp_bin, 9), 2)));
                                
                                -- Check gate
                                if dist_r < ASSOC_GATE_R * 4 and dist_d < ASSOC_GATE_D * 4 then
                                    distance := to_unsigned(dist_r + dist_d, 16);
                                    if distance < best_distance then
                                        best_distance <= distance;
                                        best_det_idx <= to_unsigned(d, 6);
                                    end if;
                                end if;
                            end if;
                        end loop;
                    end if;
                    
                    state <= ST_UPDATE;
                
                -- =========================================================
                -- Update tracks with associated detections
                -- =========================================================
                when ST_UPDATE =>
                    if track_file(to_integer(proc_trk_idx)).active = '1' then
                        if best_det_idx < MAX_DETS_PER_SCAN and best_distance < x"FFFF" then
                            -- Association found - update with alpha-beta filter
                            det_buffer(to_integer(best_det_idx)).associated <= '1';
                            
                            -- Measurement (convert to Q2)
                            meas_range := shift_left(resize(signed('0' & det_buffer(to_integer(best_det_idx)).range_bin), 12), 2);
                            meas_dopp := shift_left(resize(signed('0' & det_buffer(to_integer(best_det_idx)).dopp_bin), 9), 2);
                            
                            -- Innovation
                            innov_r := meas_range - track_file(to_integer(proc_trk_idx)).range_pos;
                            innov_d := meas_dopp - track_file(to_integer(proc_trk_idx)).dopp_pos;
                            
                            -- Update position: x = x + alpha * innovation
                            track_file(to_integer(proc_trk_idx)).range_pos <= 
                                track_file(to_integer(proc_trk_idx)).range_pos +
                                resize(shift_right(innov_r * ALPHA_GAIN, 8), 12);
                            track_file(to_integer(proc_trk_idx)).dopp_pos <= 
                                track_file(to_integer(proc_trk_idx)).dopp_pos +
                                resize(shift_right(innov_d * ALPHA_GAIN, 8), 9);
                            
                            -- Update velocity: v = v + beta * innovation
                            track_file(to_integer(proc_trk_idx)).range_vel <= 
                                track_file(to_integer(proc_trk_idx)).range_vel +
                                resize(shift_right(innov_r * BETA_GAIN, 8), 10);
                            track_file(to_integer(proc_trk_idx)).dopp_vel <= 
                                track_file(to_integer(proc_trk_idx)).dopp_vel +
                                resize(shift_right(innov_d * BETA_GAIN, 8), 8);
                            
                            -- Track management
                            track_file(to_integer(proc_trk_idx)).hit_count <= 
                                track_file(to_integer(proc_trk_idx)).hit_count + 1;
                            track_file(to_integer(proc_trk_idx)).miss_count <= (others => '0');
                            track_file(to_integer(proc_trk_idx)).last_mag <= 
                                det_buffer(to_integer(best_det_idx)).magnitude;
                            
                            -- Promote tentative to firm
                            if track_file(to_integer(proc_trk_idx)).status = TRK_TENTATIVE then
                                if track_file(to_integer(proc_trk_idx)).hit_count >= INIT_HITS then
                                    track_file(to_integer(proc_trk_idx)).status <= TRK_FIRM;
                                end if;
                            elsif track_file(to_integer(proc_trk_idx)).status = TRK_COAST then
                                track_file(to_integer(proc_trk_idx)).status <= TRK_FIRM;
                            end if;
                            
                            -- Update quality
                            if track_file(to_integer(proc_trk_idx)).quality < 15 then
                                track_file(to_integer(proc_trk_idx)).quality <= 
                                    track_file(to_integer(proc_trk_idx)).quality + 1;
                            end if;
                        else
                            -- No association - coast
                            track_file(to_integer(proc_trk_idx)).miss_count <= 
                                track_file(to_integer(proc_trk_idx)).miss_count + 1;
                            
                            if track_file(to_integer(proc_trk_idx)).status = TRK_FIRM then
                                track_file(to_integer(proc_trk_idx)).status <= TRK_COAST;
                            end if;
                            
                            -- Drop track if coast too long
                            if track_file(to_integer(proc_trk_idx)).miss_count >= COAST_MAX then
                                track_file(to_integer(proc_trk_idx)).active <= '0';
                                track_file(to_integer(proc_trk_idx)).status <= TRK_FREE;
                            end if;
                            
                            -- Degrade quality
                            if track_file(to_integer(proc_trk_idx)).quality > 0 then
                                track_file(to_integer(proc_trk_idx)).quality <= 
                                    track_file(to_integer(proc_trk_idx)).quality - 1;
                            end if;
                        end if;
                    end if;
                    
                    if proc_trk_idx = MAX_TRACKS - 1 then
                        state <= ST_INITIATE;
                        proc_det_idx <= (others => '0');
                    else
                        proc_trk_idx <= proc_trk_idx + 1;
                        state <= ST_ASSOCIATE;
                    end if;
                
                -- =========================================================
                -- Initiate new tracks from unassociated detections
                -- =========================================================
                when ST_INITIATE =>
                    if det_buffer(to_integer(proc_det_idx)).valid = '1' and 
                       det_buffer(to_integer(proc_det_idx)).associated = '0' then
                        -- Find free track slot
                        new_trk_idx := -1;
                        for t in 0 to MAX_TRACKS-1 loop
                            if track_file(t).active = '0' and new_trk_idx = -1 then
                                new_trk_idx := t;
                            end if;
                        end loop;
                        
                        if new_trk_idx >= 0 then
                            track_file(new_trk_idx).active <= '1';
                            track_file(new_trk_idx).status <= TRK_TENTATIVE;
                            track_file(new_trk_idx).range_pos <= 
                                shift_left(resize(signed('0' & det_buffer(to_integer(proc_det_idx)).range_bin), 12), 2);
                            track_file(new_trk_idx).dopp_pos <= 
                                shift_left(resize(signed('0' & det_buffer(to_integer(proc_det_idx)).dopp_bin), 9), 2);
                            track_file(new_trk_idx).range_vel <= (others => '0');
                            track_file(new_trk_idx).dopp_vel <= (others => '0');
                            track_file(new_trk_idx).hit_count <= to_unsigned(1, 4);
                            track_file(new_trk_idx).miss_count <= (others => '0');
                            track_file(new_trk_idx).quality <= to_unsigned(1, 4);
                            track_file(new_trk_idx).age <= (others => '0');
                            track_file(new_trk_idx).last_mag <= 
                                det_buffer(to_integer(proc_det_idx)).magnitude;
                        end if;
                    end if;
                    
                    if proc_det_idx = det_count - 1 or det_count = 0 then
                        state <= ST_MAINTAIN;
                    else
                        proc_det_idx <= proc_det_idx + 1;
                    end if;
                
                -- =========================================================
                -- Maintenance - count active tracks
                -- =========================================================
                when ST_MAINTAIN =>
                    active_cnt := (others => '0');
                    for t in 0 to MAX_TRACKS-1 loop
                        if track_file(t).active = '1' then
                            active_cnt := active_cnt + 1;
                        end if;
                    end loop;
                    num_active <= active_cnt;
                    
                    state <= ST_OUTPUT;
                    out_idx <= (others => '0');
                
                -- =========================================================
                -- Output confirmed tracks
                -- =========================================================
                when ST_OUTPUT =>
                    if track_file(to_integer(out_idx)).active = '1' and
                       (track_file(to_integer(out_idx)).status = TRK_FIRM or 
                        track_file(to_integer(out_idx)).status = TRK_COAST) then
                        out_valid <= '1';
                        trk_valid <= '1';
                        trk_id <= std_logic_vector(out_idx);
                        trk_range <= std_logic_vector(track_file(to_integer(out_idx)).range_pos);
                        trk_doppler <= std_logic_vector(track_file(to_integer(out_idx)).dopp_pos);
                        trk_vel_r <= std_logic_vector(track_file(to_integer(out_idx)).range_vel);
                        trk_vel_d <= std_logic_vector(track_file(to_integer(out_idx)).dopp_vel);
                        trk_quality <= std_logic_vector(track_file(to_integer(out_idx)).quality);
                        trk_status <= track_file(to_integer(out_idx)).status;
                    else
                        trk_valid <= '0';
                    end if;
                    
                    if out_idx = MAX_TRACKS - 1 then
                        -- Scan complete - reset for next
                        state <= ST_COLLECT;
                        scan_number <= scan_number + 1;
                        scan_complete <= '1';
                        det_count <= (others => '0');
                        
                        -- Clear detection buffer
                        for i in 0 to MAX_DETS_PER_SCAN-1 loop
                            det_buffer(i).valid <= '0';
                            det_buffer(i).associated <= '0';
                        end loop;
                    else
                        out_idx <= out_idx + 1;
                    end if;
                    
                when others =>
                    state <= ST_COLLECT;
                    
                end case;
            end if;
        end if;
    end process;

end Behavioral;
