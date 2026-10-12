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
  EXT_A => (others => '0'), EXT_DI => (others => '0'), EXT_DO => open,
  EXT_WR_N => '1', EXT_RD_N => '1', CPU_CE => '0',
  CPU_WAIT_N => open, SEL_N => open, IRQ_N => open,
  RAM_CS_N => open, BRAM_EN => open,
  CD_STAT => x"00", CD_MSG => x"00", CD_STAT_GET => stat_get,
  CD_COMM => command, CD_COMM_SEND => command_send,
  CD_DOUT_REQ => '0', CD_DOUT => open, CD_DOUT_SEND => open,
  CD_REGION => '0', CD_RESET => open,
  CD_DATA => cd_data, CD_DATA_WR => '0', CD_AUDIO_WR => cd_audio_wr,
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
 begin
  cycles(10); rst_n <= '1'; cycles(10);
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
