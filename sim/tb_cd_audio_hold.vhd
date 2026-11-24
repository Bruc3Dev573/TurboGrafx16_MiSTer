library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

-- Post-load audio hold at the CD unit, SLEEP looped back as in pce_top
-- (SLEEP = walk or SS_REPLAY_HOLD).
--   G_CASE 0: silent D8 + D9 resume, audio late        -> release at ready
--   G_CASE 1: PLAY-mode D8 resume without D9, audio late -> release at ready
--   G_CASE 2: resume whose audio never arrives          -> watchdog release
--   G_CASE 3: AUDIO_ACTIVE = 0 at the save              -> no hold
--   G_CASE 4: silent D8 without D9 (plays nothing)      -> release at the D8
--   G_CASE 5: PLAY-mode D8 whose status the game still awaited at the save;
--             Main answers that D8 after the load       -> slow replay D8,
--             the game gets its status, release at ready
--   G_CASE 6: as 5, but Main has no status left to send -> no replay D8,
--             release when the replay gives up
--   G_CASE 7/8/9: LOOP saved 17 frames before wrap with a SUBQ at STATUS,
--             MSGIN, or a staged DATAIN byte. Only DATAIN needs a new SUBQ.
--   G_CASE 10: the original SUBQ data/GOOD arrived during the save walk;
--             its FIFO is not saved, so wait for the reissued response.
--   G_CASE 11: live SUBQ receives all data and GOOD during the save walk.
-- The model pends a replay D8 status behind SEEK_T unless its fast-seek bit
-- is set; every replay D8 but case 5's must carry that bit.
entity tb_cd_audio_hold is
 generic (G_CASE : natural := 0; G_SKIP : natural := 60);
end entity;

architecture sim of tb_cd_audio_hold is
 constant CDDA_CLKS : integer := (429545 + 440) / 441;
 constant WD_CLKS   : integer := 2**26;
 constant SEEK_T    : time := 4 ms;
 signal clk : std_logic := '0';
 signal rst_n : std_logic := '0';
 signal walk : std_logic := '0';
 signal sleep : std_logic;
 signal ahold : std_logic;
 signal cd_sl, cd_sr : signed(15 downto 0);
 signal ssb_din, ssb_dout : std_logic_vector(63 downto 0) := (others => '0');
 signal ssb_adr : std_logic_vector(9 downto 0) := (others => '0');
 signal ssb_wren, ssb_load : std_logic := '0';
 signal command : std_logic_vector(95 downto 0);
 signal command_send : std_logic;
 signal stat_get : std_logic := '0';
 signal cd_data : std_logic_vector(7 downto 0) := (others => '0');
 signal cd_audio_wr, cd_subcd_wr : std_logic := '0';
 signal cd_data_wr : std_logic := '0';
 signal status, ext_di, ext_do : std_logic_vector(7 downto 0) := x"00";
 signal ext_a : std_logic_vector(20 downto 0) := (others => '0');
 signal wr_n, rd_n : std_logic := '1';
 signal cpu_ce : std_logic := '0';
 signal samples, queries : natural := 0;
 signal message_acked, live_query : boolean := false;
 constant LOOP_FRAMES : natural := 5 * 588;
 signal fast : std_logic;
 signal cyc : natural := 0;
 signal t_rise, t_fall : natural := 0;
 signal n_rise, n_fall : natural := 0;
 signal t_first : natural := 0;
 signal first_val : integer := -1;
 signal dac_in_hold : natural := 0;

 function dac_of(frame : natural) return integer is
 begin
  return (frame * 1023) / 1024;
 end;
begin
 clk <= not clk after 5 ns;
 sleep <= walk or ahold;
 DUT : entity work.cd port map (
  RST_N => rst_n, CLK => clk, EN => '1',
  EXT_A => ext_a, EXT_DI => ext_di, EXT_DO => ext_do,
  EXT_WR_N => wr_n, EXT_RD_N => rd_n, CPU_CE => cpu_ce,
  CPU_WAIT_N => open, SEL_N => open, IRQ_N => open,
  RAM_CS_N => open, BRAM_EN => open,
  CD_STAT => status, CD_MSG => x"00", CD_STAT_GET => stat_get,
  CD_COMM => command, CD_COMM_SEND => command_send,
  CD_DOUT_REQ => '0', CD_DOUT => open, CD_DOUT_SEND => open,
  CD_REGION => '0', CD_RESET => open,
  CD_DATA => cd_data, CD_DATA_WR => cd_data_wr, CD_AUDIO_WR => cd_audio_wr,
  CD_SUBCD_WR => cd_subcd_wr, CD_DATA_END => open, DM => '0',
  CD_SL => cd_sl, CD_SR => cd_sr, AD_S => open, SLEEP => sleep,
  WALK_SLEEP => walk,
  SaveStateBus_Din => ssb_din, SaveStateBus_Adr => ssb_adr,
  SaveStateBus_wren => ssb_wren, SaveStateBus_rst => '0',
  SaveStateBus_load => ssb_load, SaveStateBus_Dout => ssb_dout,
  SS_AD_Addr => (others => '0'), SS_AD_RdEn => '0', SS_AD_WrEn => '0',
  SS_AD_WrData => (others => '0'), SS_AD_RdData => open,
  SS_CD_HOLD => open, SS_REPLAY_HOLD => ahold, CD_FAST_SEEK => fast
 );

 -- t_rise/t_fall and n_rise/n_fall are written on the clock after the edge
 -- of ahold: read them only once n_rise/n_fall has moved.
 mon : process(clk)
  variable h_d : std_logic := '0';
  variable sl_d : signed(15 downto 0) := (others => '0');
 begin
  if rising_edge(clk) then
   cyc <= cyc + 1;
   if ahold = '1' and h_d = '0' then t_rise <= cyc; n_rise <= n_rise + 1; end if;
   if ahold = '0' and h_d = '1' then t_fall <= cyc; n_fall <= n_fall + 1; end if;
   if cd_sl /= sl_d then
    if ahold = '1' then dac_in_hold <= dac_in_hold + 1; end if;
    if first_val < 0 then first_val <= to_integer(cd_sl); t_first <= cyc; end if;
    if G_CASE >= 7 and samples < LOOP_FRAMES - G_SKIP + 16 then
     assert to_integer(cd_sl) = dac_of(1000 + (G_SKIP + samples) mod LOOP_FRAMES)
      report "SUBQ wrap: wrong DAC frame " & natural'image(samples)
             & ", got " & integer'image(to_integer(cd_sl)) severity failure;
     samples <= samples + 1;
    end if;
   end if;
   if G_CASE >= 7 and command_send = '1' then
    if command(7 downto 0) = x"DD" then
     assert live_query or (G_CASE >= 9 and queries = 0)
      report "SUBQ wrap: replayed a query whose response was already presented" severity failure;
     queries <= queries + 1;
    elsif command(7 downto 0) = x"D8" and command(47 downto 32) = x"A00F" then
     assert message_acked and samples = LOOP_FRAMES - G_SKIP
      report "SUBQ wrap: private seek preempted the saved query or its audio tail" severity failure;
    end if;
   end if;
   h_d := ahold;
   sl_d := cd_sl;
  end if;
 end process;

 stim : process
  type words_t is array (53 to 64) of std_logic_vector(63 downto 0);
  variable snapshot : words_t;
  variable t_cmd : natural;
  variable cmd_fast : std_logic;
  variable exp_fast : std_logic := '1';
  alias stat_pend is << signal .tb_cd_audio_hold.DUT.SCSI.STAT_PEND : std_logic >>;
  alias swallow   is << signal .tb_cd_audio_hold.DUT.SCSI.SWALLOW : unsigned(1 downto 0) >>;
  alias req_r     is << signal .tb_cd_audio_hold.DUT.SCSI.REQ_Nr : std_logic >>;
  alias cd_r      is << signal .tb_cd_audio_hold.DUT.SCSI.CD_Nr : std_logic >>;
  alias io_r      is << signal .tb_cd_audio_hold.DUT.SCSI.IO_Nr : std_logic >>;
  alias msg_r     is << signal .tb_cd_audio_hold.DUT.SCSI.MSG_Nr : std_logic >>;
  procedure cycles(n : natural) is
  begin
   for i in 1 to n loop wait until falling_edge(clk); end loop;
  end;
  procedure audio_frame(frame : natural) is
   variable sample : std_logic_vector(15 downto 0);
  begin
   sample := std_logic_vector(to_unsigned(frame, 16));
   for channel in 0 to 1 loop
    cd_data <= sample(7 downto 0);
    cd_audio_wr <= '1'; cycles(2); cd_audio_wr <= '0'; cycles(2);
    cd_data <= sample(15 downto 8);
    cd_audio_wr <= '1'; cycles(2); cd_audio_wr <= '0'; cycles(2);
   end loop;
  end;
  procedure sector(n : natural) is
  begin
   for i in 0 to 587 loop audio_frame(1000 + n * 588 + i); end loop;
   for i in 0 to 97 loop
    cd_data <= std_logic_vector(to_unsigned((64 + n * 98 + i) mod 256, 8));
    cd_subcd_wr <= '1'; cycles(2); cd_subcd_wr <= '0'; cycles(2);
   end loop;
  end;
  procedure good_status is
  begin
   cycles(10); stat_get <= '1'; cycles(1); stat_get <= '0'; cycles(10);
  end;
  procedure expect_cmd(op : std_logic_vector(7 downto 0); tag : string) is
  begin
   wait until command_send = '1' for 40 ms;
   wait until falling_edge(clk);	-- COMM_SEND and CD_FAST_SEEK last one clock
   assert command_send = '1' and command(7 downto 0) = op
    report "AHOLD FAILED: missing replay " & tag severity failure;
   t_cmd := cyc;
   cmd_fast := fast;
   report "AHOLD: replay " & tag & " at cycle " & integer'image(cyc)
          & " hold=" & std_logic'image(ahold) & " fast=" & std_logic'image(fast);
  end;
  procedure put(reg : natural; value : std_logic_vector(7 downto 0)) is
  begin
   ext_a <= std_logic_vector(to_unsigned(16#1FF800# + reg, 21));
   ext_di <= value; wr_n <= '0'; cpu_ce <= '1'; cycles(1);
   wr_n <= '1'; cpu_ce <= '0'; cycles(3);
  end;
  procedure phase(expected : std_logic_vector(7 downto 0)) is
  begin
   ext_a <= std_logic_vector(to_unsigned(16#1FF800#, 21)); rd_n <= '0'; cycles(1);
   if ext_do /= expected then wait until ext_do = expected for 2 ms; end if;
   assert ext_do = expected report "SUBQ wrap: phase " & to_hstring(ext_do)
          & ", expected " & to_hstring(expected) severity failure;
   rd_n <= '1'; cycles(1); -- phase can change just after a rising edge
  end;
  procedure take(expected_phase, value : std_logic_vector(7 downto 0)) is
  begin
   phase(expected_phase);
   ext_a <= std_logic_vector(to_unsigned(16#1FF801#, 21)); rd_n <= '0'; cpu_ce <= '1'; cycles(1);
   assert ext_do = value report "SUBQ wrap: got " & to_hstring(ext_do)
          & ", expected " & to_hstring(value) severity failure;
   rd_n <= '1'; cpu_ce <= '0';
   put(2, x"80"); put(2, x"00");
  end;
  procedure send_query is
  begin
   live_query <= true;
   put(0, x"00");
   for i in 0 to 9 loop
    phase(x"D0");
    if i = 0 then put(1, x"DD"); elsif i = 1 then put(1, x"0A"); else put(1, x"00"); end if;
    put(2, x"80"); put(2, x"00");
   end loop;
  end;
  procedure query_good is
  begin
   for i in 0 to 9 loop
    cd_data <= std_logic_vector(to_unsigned(16#A0# + i, 8));
    cd_data_wr <= '1'; cycles(2); cd_data_wr <= '0'; cycles(2);
   end loop;
   status <= x"00"; stat_get <= '1'; cycles(1); stat_get <= '0'; cycles(1);
  end;
 begin
  cycles(10); rst_n <= '1'; cycles(10);
  if G_CASE = 11 then
   -- Both FIFO data and GOOD are pending before the first CPU ACK.
   send_query; walk <= '1'; cycles(2); query_good; walk <= '0';
   for i in 0 to 9 loop take(x"C8", std_logic_vector(to_unsigned(16#A0# + i, 8))); end loop;
   take(x"D8", x"00"); take(x"F8", x"00"); phase(x"00");
   send_query; status <= x"02"; good_status; -- CHECK CONDITION without any DATAIN
   take(x"D8", x"02"); take(x"F8", x"00"); phase(x"00");
   assert queries = 2 report "SUBQ: wrong number of host commands" severity failure;
   report "AHOLD PASSED case 11: ten live SUBQ bytes before GOOD; no-data error closes";
   std.env.finish;
  end if;
  for i in snapshot'range loop
   ssb_adr <= std_logic_vector(to_unsigned(i, 10)); cycles(3); snapshot(i) := ssb_dout;
  end loop;
  -- Resume point: MSF 00:15:25 (LBA 1000), consumed = G_SKIP frames.
  snapshot(SSREG_INDEX_SCSI_1) := x"05000000000001F0";	-- audio active, MSF, silent D8
  snapshot(SSREG_INDEX_SCSI_2) := x"0025150000000000";
  snapshot(SSREG_INDEX_SCSI_3) := x"0000050300302900";	-- D9 seen, MSF end point
  if G_CASE = 1 then
   snapshot(SSREG_INDEX_SCSI_1)(22 downto 21) := "01";	-- PLAY-mode D8
   snapshot(SSREG_INDEX_SCSI_3)(42) := '0';				-- no D9
  elsif G_CASE = 3 then
   snapshot(SSREG_INDEX_SCSI_1)(58) := '0';				-- audio not active
  elsif G_CASE = 4 then
   snapshot(SSREG_INDEX_SCSI_3)(42) := '0';				-- silent D8, no D9
  elsif G_CASE = 5 or G_CASE = 6 then
   -- the game sent a PLAY-mode D8 and waits for its status (BSY low)
   snapshot(SSREG_INDEX_SCSI_1)(4) := '0';				-- BSY
   snapshot(SSREG_INDEX_SCSI_1)(22 downto 21) := "01";	-- PLAY-mode D8
   snapshot(SSREG_INDEX_SCSI_1)(60) := '1';				-- AUD_OPEN
   snapshot(SSREG_INDEX_SCSI_1)(62 downto 61) := "00";	-- owed: the D8
   snapshot(SSREG_INDEX_SCSI_3)(42) := '0';				-- no D9 yet
   exp_fast := '0';
  elsif G_CASE >= 7 then
   assert G_CASE <= 10 and G_SKIP = LOOP_FRAMES - 17 report "SUBQ wrap: invalid case/offset" severity failure;
   snapshot(SSREG_INDEX_SCSI_1) := (others => '0');
   snapshot(SSREG_INDEX_SCSI_1)(59 downto 58) := "11"; -- query and audio active
   snapshot(SSREG_INDEX_SCSI_1)(47 downto 32) := x"000A";
   if G_CASE = 7 then
    snapshot(SSREG_INDEX_SCSI_1)(3 downto 0) := x"4"; -- SP_STAT_START, CHECK CONDITION staged
    snapshot(SSREG_INDEX_SCSI_1)(5) := '1';
    snapshot(SSREG_INDEX_SCSI_1)(31 downto 24) := x"02";
   elsif G_CASE = 8 then
    snapshot(SSREG_INDEX_SCSI_1)(3 downto 0) := x"7"; -- SP_MSGIN_START
   elsif G_CASE = 9 then
    snapshot(SSREG_INDEX_SCSI_1)(3 downto 0) := x"A"; -- SP_DATAIN_START, A3 staged
    snapshot(SSREG_INDEX_SCSI_1)(6 downto 5) := "11";
    snapshot(SSREG_INDEX_SCSI_1)(47 downto 32) := x"0003";
    snapshot(SSREG_INDEX_SCSI_1)(31 downto 24) := x"A3";
   else
    snapshot(SSREG_INDEX_SCSI_1)(8 downto 5) := "1111"; -- SP_FREE, BSY, still waiting
    snapshot(SSREG_INDEX_SCSI_1)(18) := '1'; -- GOOD latched while SLEEP held the empty phase
    snapshot(SSREG_INDEX_SCSI_1)(47 downto 32) := x"0000";
   end if;
   snapshot(SSREG_INDEX_SCSI_2)(63 downto 32) := x"A00F0000"; -- silent D8 @4000
   snapshot(SSREG_INDEX_SCSI_3) := x"00000401A40F0000"; -- LOOP D9 @4004
   snapshot(SSREG_INDEX_SCSI_COMM_LO) := x"0000000000000ADD";
  end if;
  snapshot(SSREG_INDEX_CD_6)(58 downto 27) := std_logic_vector(to_unsigned(G_SKIP, 32));
  walk <= '1'; cycles(2);
  for i in snapshot'range loop
   ssb_adr <= std_logic_vector(to_unsigned(i, 10));
   ssb_din <= snapshot(i); ssb_wren <= '1'; cycles(2);
  end loop;
  ssb_wren <= '0'; ssb_load <= '1'; cycles(1); ssb_load <= '0'; walk <= '0';
  cycles(10);

  if G_CASE = 3 then
   assert ahold = '0' report "AHOLD FAILED: hold raised without active audio" severity failure;
   expect_cmd(x"DA", "PAUSE");
   good_status; cycles(1000);
   assert n_rise = 0 report "AHOLD FAILED: hold raised without active audio" severity failure;
   report "AHOLD PASSED case 3: no hold when the saved audio was not active";
   std.env.finish;
  end if;

  assert ahold = '1' report "AHOLD FAILED: hold not armed by the load" severity failure;
  if G_CASE = 5 then
   -- Main answers the D8 the game sent before the save.
   wait for 2 ms;
   good_status;
   assert stat_pend = '1' and swallow = 0
    report "AHOLD FAILED: the status of the game's own D8 was not kept for the game"
    severity failure;
  elsif G_CASE = 6 then
   wait until command_send = '1' or n_fall = 1 for 500 ms;
   assert command_send = '0' and n_fall = 1 and ahold = '0'
    report "AHOLD FAILED: case 6 expected a released hold and no replay command"
    severity failure;
   assert t_fall - t_rise < WD_CLKS and dac_in_hold = 0
    report "AHOLD FAILED: case 6 hold ran into the watchdog" severity failure;
   report "AHOLD PASSED case 6: no status from Main, the replay gives up without a D8;"
          & " hold " & integer'image(t_fall - t_rise) & " clocks (watchdog "
          & integer'image(WD_CLKS) & "), game status pending=" & std_logic'image(stat_pend);
   std.env.finish;
  end if;
  expect_cmd(x"D8", "D8");
  assert cmd_fast = exp_fast
   report "AHOLD FAILED: replay D8 fast-seek bit " & std_logic'image(cmd_fast)
          & ", expected " & std_logic'image(exp_fast)
          & " ('0' only when the saved game awaited the status of its own D8)"
   severity failure;
  if G_CASE = 5 then
   cycles(2);	-- RP_SW_INC reaches SWALLOW one clock after the send
   assert swallow = 1
    report "AHOLD FAILED: the replay D8 status is not marked for discard" severity failure;
  end if;
  if G_CASE = 4 then
   cycles(20);
   assert ahold = '0' and n_fall = 1 and t_fall - t_cmd < 10
    report "AHOLD FAILED: silent D8 without D9 kept the machine held" severity failure;
   report "AHOLD PASSED case 4: silent D8 without an end point releases at the D8 ("
          & integer'image(t_fall - t_cmd) & " clocks after it)";
   std.env.finish;
  end if;
  -- Main pends the D8 status behind its seek unless the fast bit is set.
  if cmd_fast = '0' then wait for SEEK_T; end if;
  good_status;
  assert swallow = 0 report "AHOLD FAILED: replay status still owed" severity failure;
  if G_CASE >= 7 then
   assert command(47 downto 24) = x"A40F00" report "SUBQ wrap: wrong resume LBA" severity failure;
   expect_cmd(x"D9", "resume D9"); good_status;
   if G_CASE = 9 then
    -- A slow audio sector leaves the machine asleep when the replacement
    -- response arrives; the saved prefix still has to be discarded.
    expect_cmd(x"DD", "open SUBQ"); cycles(4); query_good;
   end if;
   sector(4); sector(4); -- Main now loops only the final sector until repaired
   if samples < 17 then wait until samples = 17 for 5 ms; end if;
   assert samples = 17 report "SUBQ wrap: missing final DAC samples" severity failure;
   if G_CASE < 9 then wait for 11 ms; end if; -- park beyond the private command gap
   if G_CASE >= 9 then
    if G_CASE = 10 then
     expect_cmd(x"DD", "open SUBQ"); wait for 1 ms; phase(x"80");
     query_good;
    end if;
    for i in 0 to 9 loop
     if G_CASE = 10 or i >= 3 then take(x"C8", std_logic_vector(to_unsigned(16#A0# + i, 8))); end if;
    end loop;
    take(x"D8", x"00");
   elsif G_CASE = 7 then
    take(x"D8", x"02");
   end if;
   take(x"F8", x"00"); message_acked <= true;
   expect_cmd(x"D8", "corrective D8");
   assert command(47 downto 24) = x"A00F00" and command(15 downto 8) = x"00" and cmd_fast = '1'
    report "SUBQ wrap: wrong corrective seek" severity failure;
   good_status; expect_cmd(x"D9", "corrective D9");
   assert command(47 downto 24) = x"A40F00" and command(15 downto 8) = x"01"
    report "SUBQ wrap: wrong corrected loop endpoint" severity failure;
   good_status; sector(0);
   if samples < 33 then wait until samples = 33 for 2 ms; end if;
   assert samples = 33 report "SUBQ wrap: no contiguous end-to-start DAC stream" severity failure;
   phase(x"00");
   report "AHOLD PASSED case " & natural'image(G_CASE)
          & ": query closed once, D8 4004/4000, 17 tail and 16 restart DAC values";
   std.env.finish;
  end if;
  if G_CASE = 0 or G_CASE = 2 then
   expect_cmd(x"D9", "D9");
   good_status;
  end if;
  assert ahold = '1' report "AHOLD FAILED: hold dropped before the audio arrived" severity failure;

  if G_CASE = 2 then
   wait until n_fall = 1 for 1500 ms;
   assert n_fall = 1 and ahold = '0'
    report "AHOLD FAILED: watchdog never released the hold" severity failure;
   report "AHOLD: watchdog released after " & integer'image(t_fall - t_rise)
          & " clocks (2^26 = " & integer'image(WD_CLKS) & ")";
   assert t_fall - t_rise >= WD_CLKS and t_fall - t_rise <= WD_CLKS + 4
    report "AHOLD FAILED: watchdog release outside 2^26 clocks" severity failure;
   assert dac_in_hold = 0 report "AHOLD FAILED: DAC moved during the hold" severity failure;
   -- No deadlock: late audio still resumes at the saved sample.
   cycles(100);
   sector(0); sector(1);
   wait until first_val >= 0 for 5 ms;
   assert first_val = dac_of(1000 + G_SKIP)
    report "AHOLD FAILED: after the watchdog the DAC started at "
           & integer'image(first_val) severity failure;
   report "AHOLD PASSED case 2: watchdog release, late audio resumes at the saved sample";
   std.env.finish;
  end if;

  -- Seek/delivery latency: nothing arrives for 3 ms, then two sectors.
  wait for 3 ms;
  assert ahold = '1' and dac_in_hold = 0
   report "AHOLD FAILED: hold released without audio" severity failure;
  sector(0); sector(1);
  wait until first_val >= 0 for 5 ms;
  report "AHOLD: release at " & integer'image(t_fall) & ", first sample "
         & integer'image(first_val) & " at " & integer'image(t_first)
         & " (gap " & integer'image(t_first - t_fall) & " clocks, one sample = "
         & integer'image(CDDA_CLKS) & ")";
  assert n_fall = 1 and dac_in_hold = 0
   report "AHOLD FAILED: DAC moved during the hold" severity failure;
  assert first_val = dac_of(1000 + G_SKIP)
   report "AHOLD FAILED: resumed at " & integer'image(first_val) & ", expected "
          & integer'image(dac_of(1000 + G_SKIP)) severity failure;
  assert t_first > t_fall and t_first - t_fall <= CDDA_CLKS
   report "AHOLD FAILED: first sample not within one sample period of the release"
   severity failure;
  if G_CASE = 5 then
   wait until req_r = '0' for 1 ms;
   assert req_r = '0' and cd_r = '0' and io_r = '0' and msg_r = '1'
    report "AHOLD FAILED: the game never received the status of its own D8" severity failure;
   report "AHOLD: case 5 status phase for the game at cycle " & integer'image(cyc)
          & "; replay D8 at " & integer'image(t_cmd) & ", hold from " & integer'image(t_rise)
          & " to " & integer'image(t_fall) & " (" & integer'image(t_fall - t_rise) & " clocks)";
  end if;
  report "AHOLD PASSED case " & integer'image(G_CASE) & " skip=" & integer'image(G_SKIP)
         & ": release and first resumed sample within one CDDA period";
  std.env.finish;
  wait;
 end process;
end architecture;
