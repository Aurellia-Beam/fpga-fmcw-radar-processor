library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use std.textio.all;

entity tb_corner_turner is
end tb_corner_turner;

architecture Behavioral of tb_corner_turner is

    -- Small matrix for fast sim
    constant N_RANGE   : integer := 16;
    constant N_DOPPLER : integer := 8;
    constant DATA_W    : integer := 32;
    constant CLK_PERIOD : time := 10 ns;

    signal aclk          : std_logic := '0';
    signal aresetn       : std_logic := '0';
    signal s_axis_tdata  : std_logic_vector(DATA_W-1 downto 0) := (others => '0');
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tready : std_logic;
    signal s_axis_tlast  : std_logic := '0';
    signal m_axis_tdata  : std_logic_vector(DATA_W-1 downto 0);
    signal m_axis_tvalid : std_logic;
    signal m_axis_tready : std_logic := '1';
    signal m_axis_tlast  : std_logic;
    signal frame_complete : std_logic;
    signal overflow_error : std_logic;

    signal sim_done    : boolean := false;
    signal test_phase  : integer := 0;
    signal fail_count  : integer := 0;
    signal out_count   : integer := 0;

    -- Reference matrix: encode (chirp, sample) as unique value = chirp*256 + sample
    function encode_val(chirp, sample : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(chirp * 256 + sample, DATA_W));
    end function;

    function decode_chirp(val : std_logic_vector) return integer is
    begin
        return to_integer(unsigned(val)) / 256;
    end function;

    function decode_sample(val : std_logic_vector) return integer is
    begin
        return to_integer(unsigned(val)) mod 256;
    end function;

begin

    uut: entity work.corner_turner
    generic map ( N_RANGE => N_RANGE, N_DOPPLER => N_DOPPLER, DATA_WIDTH => DATA_W )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready, m_axis_tlast => m_axis_tlast,
        frame_complete => frame_complete, overflow_error => overflow_error
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    stim: process
        variable L : line;

        procedure write_frame(frame_id : integer) is
        begin
            for c in 0 to N_DOPPLER-1 loop
                for s in 0 to N_RANGE-1 loop
                    s_axis_tdata  <= encode_val(c, s);
                    s_axis_tvalid <= '1';
                    s_axis_tlast  <= '1' when s = N_RANGE-1 else '0';
                    wait until rising_edge(aclk);
                    while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
                end loop;
            end loop;
            s_axis_tvalid <= '0'; s_axis_tlast <= '0';
        end procedure;

    begin
        aresetn <= '0'; wait for 50 ns;
        aresetn <= '1'; wait until rising_edge(aclk);

        -- ==================================================================
        -- Test 1: Write frame 1 (fills bank, no output yet — first frame)
        -- ==================================================================
        report "=== T1: First frame fill (no output expected) ===";
        test_phase <= 1;
        write_frame(1);
        wait for 200 ns;

        -- ==================================================================
        -- Test 2: Write frame 2 — should read back transposed frame 1
        -- ==================================================================
        report "=== T2: Second frame → transpose of frame 1 ===";
        test_phase <= 2;
        out_count <= 0;
        write_frame(2);

        -- Wait for readout to complete
        wait until frame_complete = '1';
        wait for 500 ns;

        write(L, string'("T2 outputs: ")); write(L, out_count); writeline(output, L);
        if out_count /= N_RANGE * N_DOPPLER then
            report "FAIL T2: expected " & integer'image(N_RANGE * N_DOPPLER) &
                   " outputs, got " & integer'image(out_count) severity error;
            fail_count <= fail_count + 1;
        end if;

        -- ==================================================================
        -- Test 3: Write frame 3 with backpressure — check overflow flag
        -- ==================================================================
        report "=== T3: Backpressure / overflow ===";
        test_phase <= 3;
        out_count <= 0;
        write_frame(3);
        wait until frame_complete = '1';
        wait for 500 ns;

        -- ==================================================================
        -- Done
        -- ==================================================================
        if fail_count = 0 then report "ALL TESTS PASSED";
        else report integer'image(fail_count) & " FAILURES" severity error; end if;
        sim_done <= true; wait;
    end process;

    -- Backpressure for test 3
    bp: process(aclk)
        variable cnt : integer := 0;
    begin
        if rising_edge(aclk) then
            cnt := cnt + 1;
            if test_phase = 3 and (cnt mod 4 = 0 or cnt mod 4 = 1) then
                m_axis_tready <= '0';
            else
                m_axis_tready <= '1';
            end if;
        end if;
    end process;

    -- Output checker: verify transpose
    -- Input ordering:  chirp c, sample s → address [c][s]
    -- Output ordering: range r, doppler d → value should be encode_val(d, r)
    -- i.e. output[r][d] = input[d][r] → chirp=d, sample=r
    check: process(aclk)
        variable L : line;
        variable exp_range   : integer := 0;
        variable exp_doppler : integer := 0;
        variable got_chirp, got_sample : integer;
    begin
        if rising_edge(aclk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                out_count <= out_count + 1;

                if test_phase = 2 then
                    got_chirp  := decode_chirp(m_axis_tdata);
                    got_sample := decode_sample(m_axis_tdata);

                    -- Expected: transposed read — for output (range=exp_range, doppler=exp_doppler)
                    -- the original write was (chirp=exp_doppler, sample=exp_range)
                    if got_chirp /= exp_doppler or got_sample /= exp_range then
                        write(L, string'("FAIL T2 out#")); write(L, out_count-1);
                        write(L, string'(" R=")); write(L, exp_range);
                        write(L, string'(" D=")); write(L, exp_doppler);
                        write(L, string'(" exp_chirp=")); write(L, exp_doppler);
                        write(L, string'(" exp_sample=")); write(L, exp_range);
                        write(L, string'(" got_chirp=")); write(L, got_chirp);
                        write(L, string'(" got_sample=")); write(L, got_sample);
                        writeline(output, L);
                        fail_count <= fail_count + 1;
                    end if;

                    -- Advance expected indices (output order: range varies slowly, doppler fast)
                    if exp_doppler = N_DOPPLER - 1 then
                        exp_doppler := 0;
                        if exp_range = N_RANGE - 1 then exp_range := 0;
                        else exp_range := exp_range + 1; end if;
                    else
                        exp_doppler := exp_doppler + 1;
                    end if;
                end if;

                -- tlast check: should fire at end of each Doppler row
                if m_axis_tlast = '1' and test_phase = 2 then
                    if exp_doppler /= 0 then
                        report "WARN: tlast at unexpected position" severity warning;
                    end if;
                end if;
            end if;

            -- Overflow flag monitoring
            if overflow_error = '1' then
                report "Overflow flag asserted (test " & integer'image(test_phase) & ")";
            end if;
        end if;
    end process;

end Behavioral;
