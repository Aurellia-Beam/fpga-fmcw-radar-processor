library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use std.textio.all;

-- Standalone testbench for 1D OS-CFAR (no Xilinx IP needed)

entity tb_os_cfar is
end tb_os_cfar;

architecture Behavioral of tb_os_cfar is

    constant CLK_PERIOD : time := 10 ns;
    constant DATA_W     : integer := 17;

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
    signal sim_done      : boolean := false;
    signal det_count     : integer := 0;

begin

    uut: entity work.os_cfar
    generic map (
        DATA_WIDTH => DATA_W, REF_CELLS => 8, GUARD_CELLS => 2,
        RANK_IDX => 12, SCALING_MULT => 3, SCALING_DIV => 1
    )
    port map (
        aclk => aclk, aresetn => aresetn,
        s_axis_tdata => s_axis_tdata, s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready, s_axis_tlast => s_axis_tlast,
        m_axis_tdata => m_axis_tdata, m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready, m_axis_tlast => m_axis_tlast
    );

    aclk <= not aclk after CLK_PERIOD/2 when not sim_done else '0';

    stim: process
        procedure send(val : integer; last : std_logic := '0') is
        begin
            s_axis_tdata  <= std_logic_vector(to_unsigned(val, DATA_W));
            s_axis_tvalid <= '1';
            s_axis_tlast  <= last;
            wait until rising_edge(aclk);
            while s_axis_tready = '0' loop wait until rising_edge(aclk); end loop;
        end procedure;
    begin
        aresetn <= '0'; wait for 50 ns;
        aresetn <= '1'; wait until rising_edge(aclk);

        report "=== OS-CFAR Test Start ===";

        -- Fill noise floor (10-14)
        report "Phase 1: Noise floor";
        for i in 0 to 30 loop send(10 + (i mod 5)); end loop;

        -- Single target
        report "Phase 2: Single target (amp=100)";
        send(100);
        for i in 0 to 20 loop send(10 + (i mod 3)); end loop;

        -- Two close targets (masking test)
        report "Phase 3: Close targets (200, gap, 250)";
        send(200); send(15); send(250);
        for i in 0 to 30 loop send(12); end loop;

        -- Flush
        s_axis_tvalid <= '0';
        wait for 200 ns;
        sim_done <= true;
        report "=== Complete. Detections: " & integer'image(det_count) & " ===";
        wait;
    end process;

    monitor: process(aclk)
        variable l : line;
    begin
        if rising_edge(aclk) then
            if m_axis_tvalid = '1' and unsigned(m_axis_tdata) > 0 then
                det_count <= det_count + 1;
                write(l, string'("DET amp=")); write(l, to_integer(unsigned(m_axis_tdata)));
                writeline(output, l);
            end if;
        end if;
    end process;

end Behavioral;
