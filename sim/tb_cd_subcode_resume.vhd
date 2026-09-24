library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity tb_cd_subcode_resume is
 generic (G_SKIP : natural := 60; G_DELAYED : boolean := true);
end entity;

architecture sim of tb_cd_subcode_resume is
 signal clk : std_logic := '0';
 signal rst_n : std_logic := '0';
 signal en : std_logic := '1';
 signal sleep : std_logic := '0';
 signal ext_a : std_logic_vector(20 downto 0) := (others => '0');
 signal ext_di, ext_do : std_logic_vector(7 downto 0) := (others => '0');
 signal ext_wr_n, ext_rd_n : std_logic := '1';
 signal cpu_ce : std_logic := '0';
 signal irq_n : std_logic;
 signal cd_sl, cd_sr : signed(15 downto 0);
 signal ssb_din, ssb_dout : std_logic_vector(63 downto 0) := (others => '0');
 signal ssb_adr : std_logic_vector(9 downto 0) := (others => '0');
 signal ssb_wren, ssb_load : std_logic := '0';
 signal command : std_logic_vector(95 downto 0);
 signal command_send : std_logic;
 signal stat_get : std_logic := '0';
 signal cd_data : std_logic_vector(7 downto 0) := (others => '0');
 signal cd_audio_wr, cd_subcd_wr : std_logic := '0';
 signal monitor : boolean := false;

 function dac_of(frame : natural) return integer is
 begin
  return (frame * 1023) / 1024;
 end;
begin
 clk <= not clk after 5 ns;
 DUT : entity work.cd port map (
  RST_N => rst_n, CLK => clk, EN => en,
  EXT_A => ext_a, EXT_DI => ext_di, EXT_DO => ext_do,
  EXT_WR_N => ext_wr_n, EXT_RD_N => ext_rd_n, CPU_CE => cpu_ce,
  CPU_WAIT_N => open, SEL_N => open, IRQ_N => irq_n,
  RAM_CS_N => open, BRAM_EN => open,
  CD_STAT => x"00", CD_MSG => x"00", CD_STAT_GET => stat_get,
  CD_COMM => command, CD_COMM_SEND => command_send,
  CD_DOUT_REQ => '0', CD_DOUT => open, CD_DOUT_SEND => open,
  CD_REGION => '0', CD_RESET => open,
  CD_DATA => cd_data, CD_DATA_WR => '0', CD_AUDIO_WR => cd_audio_wr,
  CD_SUBCD_WR => cd_subcd_wr, CD_DATA_END => open, DM => '0',
  CD_SL => cd_sl, CD_SR => cd_sr, AD_S => open, SLEEP => sleep,
  SaveStateBus_Din => ssb_din, SaveStateBus_Adr => ssb_adr,
  SaveStateBus_wren => ssb_wren, SaveStateBus_rst => '0',
  SaveStateBus_load => ssb_load, SaveStateBus_Dout => ssb_dout,
  SS_AD_Addr => (others => '0'), SS_AD_RdEn => '0', SS_AD_WrEn => '0',
  SS_AD_WrData => (others => '0'), SS_AD_RdData => open, SS_CD_HOLD => open
 );
 timeout : process
 begin
  wait for 50 ms;
  assert false report "subcode resume timed out" severity failure;
 end process;

 check_output : process
  variable expected_byte : natural;
 begin
  wait until monitor;
  for i in 0 to 11 loop
   wait on cd_sl for 500 us;
   assert cd_sl'event report "resumed audio stalled" severity failure;
   assert to_integer(cd_sl) = dac_of(1000 + G_SKIP + i) and cd_sl = cd_sr
    report "wrong DAC sample at resumed frame " & integer'image(i)
           & ": got " & integer'image(to_integer(cd_sl)) severity failure;
   wait for 1 ns;
   expected_byte := (64 + (G_SKIP + i) / 6) mod 256;
   assert to_integer(unsigned(ext_do)) = expected_byte
    report "wrong subcode at resumed frame " & integer'image(i)
           & ": got " & integer'image(to_integer(unsigned(ext_do)))
           & ", expected " & integer'image(expected_byte) severity failure;
   if i = 0 then
    wait for 30 ns;
    assert irq_n = '0' report "first visible subcode did not raise IRQ" severity failure;
   end if;
  end loop;
  report "CD SUBCODE RESUME PASSED skip=" & integer'image(G_SKIP)
         & " delayed=" & boolean'image(G_DELAYED);
  std.env.finish;
  wait;
 end process;

 stim : process
  type words_t is array (53 to 64) of std_logic_vector(63 downto 0);
  variable snapshot : words_t;
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
  procedure subcode_byte(index : natural) is
  begin
   cd_data <= std_logic_vector(to_unsigned((64 + index) mod 256, 8));
   cd_subcd_wr <= '1'; cycles(2); cd_subcd_wr <= '0'; cycles(2);
  end;
  procedure good_status is
  begin
   cycles(10); stat_get <= '1'; cycles(1); stat_get <= '0'; cycles(10);
  end;
 begin
  assert G_SKIP < 588 report "skip must be a sector remainder" severity failure;
  assert not G_DELAYED or G_SKIP /= 0
   report "zero-skip control uses buffered input" severity failure;
  cycles(10); rst_n <= '1'; cycles(10);
  for i in snapshot'range loop
   ssb_adr <= std_logic_vector(to_unsigned(i, 10)); cycles(3); snapshot(i) := ssb_dout;
  end loop;
  snapshot(SSREG_INDEX_SCSI_1) := x"05000000000001F0";
  snapshot(SSREG_INDEX_SCSI_2) := x"0025150000000000";
  snapshot(SSREG_INDEX_SCSI_3) := x"0000050300302900";
  snapshot(SSREG_INDEX_CD_6)(58 downto 27) := std_logic_vector(to_unsigned(G_SKIP, 32));
  sleep <= '1'; cycles(2);
  for i in snapshot'range loop
   ssb_adr <= std_logic_vector(to_unsigned(i, 10));
   ssb_din <= snapshot(i); ssb_wren <= '1'; cycles(2);
  end loop;
  ssb_wren <= '0'; ssb_load <= '1'; cycles(1); ssb_load <= '0'; sleep <= '0';
  wait until command_send = '1' for 20 ms;
  assert command_send = '1' and command(7 downto 0) = x"D8"
   report "missing replay D8" severity failure;
  good_status;
  wait until command_send = '1' for 20 ms;
  assert command_send = '1' and command(7 downto 0) = x"D9"
   report "missing replay D9" severity failure;
  good_status;

  -- Enable the CPU subcode IRQ, then acknowledge any pre-replay flag.
  ext_a <= std_logic_vector(to_unsigned(16#1FF802#, 21));
  ext_di <= x"10"; ext_wr_n <= '0'; cpu_ce <= '1'; cycles(1);
  ext_wr_n <= '1'; cpu_ce <= '0'; cycles(3);
  ext_a <= std_logic_vector(to_unsigned(16#1FF807#, 21));
  ext_rd_n <= '0'; cpu_ce <= '1'; cycles(1);
  ext_rd_n <= '1'; cpu_ce <= '0'; cycles(3);
  -- $1807 remains selected for the output observer; no internal forcing.
  monitor <= true;
  if not G_DELAYED then sleep <= '1'; cycles(2); end if;

  -- Actual Main ordering: an entire audio sector precedes its subcode block.
  for sector in 0 to 1 loop
   for i in 0 to 587 loop audio_frame(1000 + sector * 588 + i); end loop;
   if G_DELAYED and sector = 0 then
    wait for 30 us;
    assert cd_sl = 0 and irq_n = '1'
     report "audio/subcode IRQ escaped before delayed subcode arrived" severity failure;
   end if;
   for i in 0 to 97 loop
    subcode_byte(sector * 98 + i);
    if G_DELAYED and sector = 0 and i < G_SKIP / 6 then
     -- Empty the FIFO between prefix bytes to exercise registered EMPTY pacing.
     cycles(100);
     assert cd_sl = 0 and irq_n = '1'
      report "discarded subcode prefix became visible" severity failure;
    end if;
   end loop;
  end loop;
  sleep <= '0';
  wait;
 end process;
end architecture;
