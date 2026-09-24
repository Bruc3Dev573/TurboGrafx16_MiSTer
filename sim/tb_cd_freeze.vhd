library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity tb_cd_freeze is

end entity;

architecture sim of tb_cd_freeze is
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
 signal cd_data_wr, cd_audio_wr, cd_subcd_wr : std_logic := '0';
 signal monitor : boolean := false;
 signal cd_hold : std_logic;
 signal heard : natural := 0;

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
  CD_DATA => cd_data, CD_DATA_WR => cd_data_wr, CD_AUDIO_WR => cd_audio_wr,
  CD_SUBCD_WR => cd_subcd_wr, CD_DATA_END => open, DM => '0',
  CD_SL => cd_sl, CD_SR => cd_sr, AD_S => open, SLEEP => sleep,
  SaveStateBus_Din => ssb_din, SaveStateBus_Adr => ssb_adr,
  SaveStateBus_wren => ssb_wren, SaveStateBus_rst => '0',
  SaveStateBus_load => ssb_load, SaveStateBus_Dout => ssb_dout,
  SS_AD_Addr => (others => '0'), SS_AD_RdEn => '0', SS_AD_WrEn => '0',
  SS_AD_WrData => (others => '0'), SS_AD_RdData => open, SS_CD_HOLD => cd_hold
 );
 timeout : process
 begin
  wait for 50 ms;
  assert false report "freeze FIFO test timed out" severity failure;
 end process;
 check_audio : process
 begin
  wait until monitor;
  for i in 0 to 3224 loop
   wait on cd_sl for 1 ms;
   assert cd_sl'event and cd_sl = cd_sr and to_integer(cd_sl) = dac_of(1000 + 2*i)
    report "freeze lost or reordered audio frame " & integer'image(i) severity failure;
   wait for 1 ns;
   if i < 2352 then
    assert to_integer(unsigned(ext_do)) = (i / 6) mod 256
     report "freeze lost or reordered subcode at frame " & integer'image(i) severity failure;
   end if;
   heard <= i+1;
  end loop;
  wait;
 end process;
 stim : process
  procedure cycles(n : natural) is
  begin
   for i in 1 to n loop wait until falling_edge(clk); end loop;
  end;
  procedure audio_frame(index : natural) is
   variable sample : std_logic_vector(15 downto 0);
  begin
   sample := std_logic_vector(to_unsigned(1000 + 2*index, 16));
   for channel in 0 to 1 loop
    cd_data <= sample(7 downto 0);
    cd_audio_wr <= '1'; cycles(2); cd_audio_wr <= '0'; cycles(2);
    cd_data <= sample(15 downto 8);
    cd_audio_wr <= '1'; cycles(2); cd_audio_wr <= '0'; cycles(2);
   end loop;
  end;
  procedure subcode_byte(index : natural) is
  begin
   cd_data <= std_logic_vector(to_unsigned(index mod 256, 8));
   cd_subcd_wr <= '1'; cycles(2); cd_subcd_wr <= '0'; cycles(2);
  end;
 begin
  -- Freeze immediately after a byte is presented, or after its FIFO pop.
  -- Exercise both the first DATAIN byte and the next-byte handshake path.
  for frozen_byte in 0 to 1 loop
   for delay in 0 to 1 loop
    rst_n <= '0'; sleep <= '0'; cycles(10);
    rst_n <= '1'; sleep <= '1'; cycles(10);
    for i in 0 to 15 loop
     cd_data <= std_logic_vector(to_unsigned(i, 8));
     cd_data_wr <= '1'; cycles(2); cd_data_wr <= '0'; cycles(2);
    end loop;
    ext_a <= std_logic_vector(to_unsigned(16#1FF800#, 21));
    sleep <= '0';
    for i in 0 to 15 loop
     if ext_do(6) /= '1' then wait until ext_do(6) = '1' for 5 us; end if;
     assert ext_do(6) = '1'
      report "DATAIN stalled after freeze at byte " & integer'image(i) severity failure;
     cycles(1);
     if i = frozen_byte then
      cycles(delay); sleep <= '1'; cycles(64); sleep <= '0'; cycles(1);
     end if;
     ext_a <= std_logic_vector(to_unsigned(16#1FF808#, 21)); cycles(1);
     assert to_integer(unsigned(ext_do)) = i
      report "DATAIN freeze lost/reordered byte " & integer'image(i)
       & " got=" & integer'image(to_integer(unsigned(ext_do)))
       & " frozen_byte=" & integer'image(frozen_byte)
       & " delay=" & integer'image(delay) severity failure;
     ext_rd_n <= '0'; cpu_ce <= '1'; cycles(1);
     ext_rd_n <= '1'; cpu_ce <= '0';
     ext_a <= std_logic_vector(to_unsigned(16#1FF800#, 21)); cycles(1);
    end loop;
   end loop;
  end loop;
  report "CD DATA FREEZE PASSED: first and subsequent bytes survive both pop phases";
  rst_n <= '0'; sleep <= '0'; cycles(10);
  cycles(10); rst_n <= '1'; cycles(10); sleep <= '1'; cycles(10);
  for i in 0 to 2047 loop audio_frame(i); end loop;
  for i in 0 to 195 loop subcode_byte(i); end loop;
  cycles(10);
  audio_frame(2048); cycles(10);
  ext_a <= std_logic_vector(to_unsigned(16#1FF807#, 21));
  monitor <= true; sleep <= '0';
  wait until heard = 1 for 100 us;
  sleep <= '1'; cycles(2);
  -- Main keeps delivering while the consumer is frozen: a complete
  -- two-sector startup burst that fits plays in full and in order.
  for i in 2049 to 3224 loop audio_frame(i); end loop;
  for i in 196 to 391 loop subcode_byte(i); end loop;
  cycles(10); sleep <= '0';
  wait until heard = 3225 for 40 ms;
  assert heard = 3225 report "frozen burst did not drain completely" severity failure;
  wait for 100 us;
  -- Delivery keeps pace with the DAC, so a backlog never drains: with SCSI
  -- quiet and no DMA pending it must not veto the freeze boundary.
  sleep <= '1'; cycles(2);
  for i in 0 to 2999 loop audio_frame(i); end loop;
  for i in 0 to 249 loop subcode_byte(i); end loop;
  cycles(10);
  assert cd_hold = '0' report "persistent CDDA/subcode backlog vetoed the freeze" severity failure;
  report "CD FREEZE PASSED: in-flight two-sector burst played in full; a persistent backlog does not veto the freeze";
  std.env.finish;
  wait;
 end process;
end architecture;
