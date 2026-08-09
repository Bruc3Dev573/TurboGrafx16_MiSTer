-----------------------------------------------------------------------------
-- tb_adpcm.vhd — fast standalone unit harness for the CD ADPCM engine.
--
-- Instantiates rtl/cd/cd.vhd directly and drives the ADPCM registers over the
-- EXT bus (no 6280, no pce_top), so the write/play/read sequence and the
-- decoder-state save/load round-trip iterate in seconds instead of the
-- full-system tb_cd's ~40 minutes.  Purpose: nail the sequence that plays a
-- sample (AD_S non-zero) and prove the ADPCM playback state + ADPCM DRAM
-- survive a savestate save/load (hardware "ADPCM crackle after load" bug,
-- hypothesis b).  Once validated here, the sequence is ported into tb_cd's
-- BIOS for the full-engine D scenario.
--
-- CD register window (cd.vhd REG_SEL): EXT_A = 0x1FF800 + reg.
-- Run: sim/run_tb_adpcm.sh
-----------------------------------------------------------------------------
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity tb_adpcm is
end entity;

architecture sim of tb_adpcm is

	constant CLK_PERIOD : time := 10 ns;

	signal clk     : std_logic := '0';
	signal rst_n   : std_logic := '0';

	signal ext_a   : std_logic_vector(20 downto 0) := (others => '0');
	signal ext_di  : std_logic_vector(7 downto 0)  := (others => '0');
	signal ext_do  : std_logic_vector(7 downto 0);
	signal ext_wr_n: std_logic := '1';
	signal ext_rd_n: std_logic := '1';
	signal cpu_ce  : std_logic := '0';

	signal ad_s    : signed(15 downto 0);
	signal cd_sl   : signed(15 downto 0);
	signal cd_sr   : signed(15 downto 0);

	signal sleep      : std_logic := '0';
	signal ssb_din    : std_logic_vector(63 downto 0) := (others => '0');
	signal ssb_adr    : std_logic_vector(9 downto 0)  := (others => '0');
	signal ssb_wren   : std_logic := '0';
	signal ssb_rst    : std_logic := '0';
	signal ssb_load   : std_logic := '0';
	signal ssb_dout   : std_logic_vector(63 downto 0);

	signal ss_ad_addr : std_logic_vector(15 downto 0) := (others => '0');
	signal ss_ad_rden : std_logic := '0';
	signal ss_ad_wren : std_logic := '0';
	signal ss_ad_wrd  : std_logic_vector(7 downto 0) := (others => '0');
	signal ss_ad_rdd  : std_logic_vector(7 downto 0);
	signal ss_cd_hold : std_logic;

	signal n_ce, n_mclk, n_vck, n_prp, n_xti : integer := 0;

begin

	clk <= not clk after CLK_PERIOD / 2;

	-- pulse counters for the ADPCM clock chain (single-cycle strobes)
	mon : process(clk)
		alias ce   is << signal .tb_adpcm.DUT.ADPCM_CE       : std_logic >>;
		alias mclk is << signal .tb_adpcm.DUT.M5205_CLK      : std_logic >>;
		alias vck  is << signal .tb_adpcm.DUT.M5205_VCK_R    : std_logic >>;
		alias prp  is << signal .tb_adpcm.DUT.PLAY_READ_PEND : std_logic >>;
		alias xti  is << signal .tb_adpcm.DUT.MSM5205.XTI    : std_logic >>;
	begin
		if rising_edge(clk) then
			if ce   = '1' then n_ce   <= n_ce   + 1; end if;
			if mclk = '1' then n_mclk <= n_mclk + 1; end if;
			if vck  = '1' then n_vck  <= n_vck  + 1; end if;
			if prp  = '1' then n_prp  <= n_prp  + 1; end if;
			if xti  = '1' then n_xti  <= n_xti  + 1; end if;
		end if;
	end process;

	DUT : entity work.cd
	port map (
		RST_N       => rst_n,
		CLK         => clk,
		EN          => '1',
		EXT_A       => ext_a,
		EXT_DI      => ext_di,
		EXT_DO      => ext_do,
		EXT_WR_N    => ext_wr_n,
		EXT_RD_N    => ext_rd_n,
		CPU_CE      => cpu_ce,
		SEL_N       => open,
		IRQ_N       => open,
		RAM_CS_N    => open,
		BRAM_EN     => open,
		CD_STAT     => x"00",
		CD_MSG      => x"00",
		CD_STAT_GET => '0',
		CD_COMM     => open,
		CD_COMM_SEND=> open,
		CD_DOUT_REQ => '0',
		CD_DOUT     => open,
		CD_DOUT_SEND=> open,
		CD_REGION   => '0',
		CD_RESET    => open,
		CD_DATA     => x"00",
		CD_DATA_WR  => '0',
		CD_AUDIO_WR => '0',
		CD_SUBCD_WR => '0',
		CD_DATA_END => open,
		DM          => '0',
		CD_SL       => cd_sl,
		CD_SR       => cd_sr,
		AD_S        => ad_s,
		SLEEP             => sleep,
		SaveStateBus_Din  => ssb_din,
		SaveStateBus_Adr  => ssb_adr,
		SaveStateBus_wren => ssb_wren,
		SaveStateBus_rst  => ssb_rst,
		SaveStateBus_load => ssb_load,
		SaveStateBus_Dout => ssb_dout,
		SS_AD_Addr        => ss_ad_addr,
		SS_AD_RdEn        => ss_ad_rden,
		SS_AD_WrEn        => ss_ad_wren,
		SS_AD_WrData      => ss_ad_wrd,
		SS_AD_RdData      => ss_ad_rdd,
		SS_CD_HOLD        => ss_cd_hold
	);

	stim : process
		-- observe internals
		alias a_play  is << signal .tb_adpcm.DUT.ADPCM_PLAY  : std_logic >>;
		alias a_len   is << signal .tb_adpcm.DUT.ADPCM_LEN   : std_logic_vector(16 downto 0) >>;
		alias a_rdad  is << signal .tb_adpcm.DUT.ADPCM_RDADDR: std_logic_vector(16 downto 0) >>;
		alias a_wrad  is << signal .tb_adpcm.DUT.ADPCM_WRADDR: std_logic_vector(16 downto 0) >>;
		alias a_wpend is << signal .tb_adpcm.DUT.ADPCM_WRITE_PEND : std_logic >>;
		alias a_end   is << signal .tb_adpcm.DUT.ADPCM_END   : std_logic >>;
		alias a_freq  is << signal .tb_adpcm.DUT.ADPCM_FREQ  : std_logic_vector(3 downto 0) >>;
		alias m_sout  is << signal .tb_adpcm.DUT.M5205_SOUT  : signed(15 downto 0) >>;
		alias m_ccnt  is << signal .tb_adpcm.DUT.MSM5205.CLK_CNT : unsigned(5 downto 0) >>;
		alias m_rstn  is << signal .tb_adpcm.DUT.MSM5205.RST_N : std_logic >>;
		alias m_slp   is << signal .tb_adpcm.DUT.MSM5205.SLEEP : std_logic >>;
		alias m_ssl   is << signal .tb_adpcm.DUT.MSM5205.SS_load : std_logic >>;
		alias m_vck   is << signal .tb_adpcm.DUT.M5205_VCK_R : std_logic >>;
		alias m_sample is << signal .tb_adpcm.DUT.MSM5205.SAMPLE : unsigned(15 downto 0) >>;
		alias m_step   is << signal .tb_adpcm.DUT.MSM5205.STEP : unsigned(5 downto 0) >>;

		-- scenario D: the whole CD register block (CD_1..CD_6 spans the SCSI
		-- registers too, 53..63) and the two decoded-sample tapes
		constant D_K : integer := 48;			-- decoded samples compared
		type snap_t is array (0 to SSREG_INDEX_CD_6 - SSREG_INDEX_CD_1) of std_logic_vector(63 downto 0);
		type tape_t is array (0 to D_K-1) of integer;
		variable snap     : snap_t;
		variable ref_tape : tape_t;
		variable rep_tape : tape_t;

		procedure clks(n : integer) is
		begin
			for i in 1 to n loop wait until rising_edge(clk); end loop;
		end procedure;

		procedure wr(reg : integer; data : integer) is
		begin
			wait until rising_edge(clk);
			ext_a    <= std_logic_vector(to_unsigned(16#1FF800# + reg, 21));
			ext_di   <= std_logic_vector(to_unsigned(data, 8));
			ext_wr_n <= '0'; ext_rd_n <= '1'; cpu_ce <= '1';
			wait until rising_edge(clk);
			cpu_ce <= '0'; ext_wr_n <= '1';
			clks(2);
		end procedure;

		-- write one ADPCM byte ($180A) and wait for the DRAM slot machine to
		-- retire it (WPEND back to 0), so the next write is not dropped
		procedure wr_byte(data : integer) is
			variable t : integer := 0;
		begin
			wr(16#0A#, data);
			while a_wpend = '1' and t < 400 loop
				wait until rising_edge(clk); t := t + 1;
			end loop;
		end procedure;

		-- read one ADPCM DRAM byte over the SS_AD save-walk (SLEEP path): the
		-- exact SS_AD_NIB shuttle the savestate save uses.  The engine paces
		-- one byte per ~8 CLK (memory_slow); mirror that so the registered
		-- ADRAM read + the nibble shuttle both settle.
		procedure ss_read(addr : integer; got : out integer) is
		begin
			ss_ad_addr <= std_logic_vector(to_unsigned(addr, 16));
			ss_ad_rden <= '1';
			clks(8);
			got := to_integer(unsigned(ss_ad_rdd));
			ss_ad_rden <= '0';
			clks(2);
		end procedure;

		-- write one ADPCM DRAM byte over the SS_AD LOAD walk: hi nibble at
		-- 2*addr, lo at 2*addr+1, sequenced by SS_AD_NIB
		procedure ss_write(addr : integer; data : integer) is
		begin
			ss_ad_addr <= std_logic_vector(to_unsigned(addr, 16));
			ss_ad_wrd  <= std_logic_vector(to_unsigned(data, 8));
			ss_ad_wren <= '1';
			clks(8);
			ss_ad_wren <= '0';
			clks(2);
		end procedure;

		-- savestate register access (rtl/bus_savestates.vhd eReg_SavestateV):
		-- reading is combinational off BUS_Adr, writing latches the shadow
		-- buffer that a later BUS_load pulse pushes into the design.
		procedure ss_reg_read(idx : integer; val : out std_logic_vector(63 downto 0)) is
		begin
			ssb_adr <= std_logic_vector(to_unsigned(idx, 10));
			clks(2);
			val := ssb_dout;
		end procedure;

		-- The real engine (savestates.vhd) drives load_done for EXACTLY one clock:
		-- it is cleared at the top of every cycle and set in a single state
		-- transition.  Holding it longer in the testbench is not merely
		-- cosmetic - it let ADPCM_PLAY come back from the blob on the first
		-- cycle and release the decoder's asynchronous reset, so the restore
		-- succeeded on the later cycles and hid a real defect.
		procedure ss_pulse_load is
		begin
			wait until rising_edge(clk);
			ssb_load <= '1';
			wait until rising_edge(clk);
			ssb_load <= '0';
		end procedure;

		procedure ss_reg_write(idx : integer; val : std_logic_vector(63 downto 0)) is
		begin
			wait until rising_edge(clk);
			ssb_adr  <= std_logic_vector(to_unsigned(idx, 10));
			ssb_din  <= val;
			ssb_wren <= '1';
			wait until rising_edge(clk);
			ssb_wren <= '0';
			wait until rising_edge(clk);
		end procedure;

		impure function pat(i : integer) return integer is
		begin
			return (i*17 + 5) mod 256;
		end function;

		variable g : integer;
		variable diff : integer;
		-- 64 bytes = 128 nibbles = 128 decode steps: enough for the pre-roll plus
		-- scenario D's two D_K-sample tapes without LEN running out (LEN=0 raises
		-- ADPCM_END and stops PLAY, which would end the tape early)
		constant N : integer := 64;	-- sample bytes (CPU-write play test)
		-- scenario D2: decode steps until ADPCM_END, with and without a load.
		-- The cap has to exceed the whole sample (N bytes = 2N nibbles) so that
		-- "never ends" is distinguishable from "ends late".
		constant D_MAXSTEP : integer := 4*N;
		variable ref_steps : integer := -1;
		variable rep_steps : integer := -1;
		variable snap_sample, snap_step : integer := 0;
		constant M : integer := 40;	-- bytes for the SS_AD shuttle round-trip
	begin
		rst_n <= '0';
		clks(20);
		rst_n <= '1';
		clks(20);
		-- MSM5205.CLK_CNT (rtl/cd/MSM5205.vhd) has no reset — it relies on the
		-- FPGA powering FFs to 0.  In GHDL it stays 'U' ('U'+1='U'), so VCK_R
		-- never fires.  Pulse BUS_rst then a zero SS_load to define it (the
		-- framework's power-up path).
		ssb_rst <= '1'; clks(2); ssb_rst <= '0'; clks(2);
		ssb_load <= '1'; clks(2); ssb_load <= '0'; clks(8);

		-- ================================================================
		-- SS_AD shuttle round-trip: write a pattern into ADPCM DRAM over the
		-- LOAD walk, read it back over the SAVE walk, require bit-exact.  This
		-- IS the ADPCM-RAM (region 9) save/load path; a dropped/duplicated
		-- nibble corrupts the sample on every load (hardware "crackle after
		-- load" class, hypothesis a).  Independent of the CPU $180A path.
		-- ================================================================
		report "TBADP: === SS_AD shuttle round-trip, M=" & integer'image(M) & " ===";
		sleep <= '1';
		clks(4);
		for i in 0 to M-1 loop
			ss_write(i, pat(i));
		end loop;
		diff := 0;
		for i in 0 to M-1 loop
			ss_read(i, g);
			if g /= pat(i) then
				diff := diff + 1;
				if diff <= 16 then
					report "TBADP SSAD[" & integer'image(i) & "]=" & integer'image(g)
						& " exp=" & integer'image(pat(i)) severity warning;
				end if;
			end if;
		end loop;
		sleep <= '0';
		clks(4);
		if diff = 0 then
			report "TBADP: SS_AD shuttle round-trip OK (" & integer'image(M) & " bytes)";
		else
			report "TBADP: SS_AD shuttle round-trip FAILED in " & integer'image(diff)
				& "/" & integer'image(M) & " bytes" severity warning;
		end if;

		report "TBADP: === write ADPCM RAM, N=" & integer'image(N) & " ===";

		-- reset the ADPCM engine, then arm the write pointer from OFFS=0
		wr(16#0D#, 16#80#);
		wr(16#0D#, 16#00#);
		wr(16#08#, 16#00#);
		wr(16#09#, 16#00#);
		-- Arm the write pointer the way cd.vhd latches it (cd.vhd:567): a $180D
		-- write whose bit0 falls while the OLD CTRL had bit0=1 reloads
		-- WRADDR <= OFFS<<1 when the OLD CTRL bit1 was set.  Doing it with a
		-- $180A write instead is wrong: that write still uses the STALE WRADDR
		-- and only then reloads, so the first byte lands somewhere else and the
		-- whole read-back looks shifted by one byte.
		wr(16#0D#, 16#03#);		-- bit0=1 (arm) + bit1=1 (reload mode)
		wr(16#0D#, 16#00#);		-- bit0 falls -> WRADDR <= OFFS<<1 = 0
		for i in 0 to N-1 loop
			wr_byte(pat(i));
		end loop;
		report "TBADP: after writes WRADDR=" & integer'image(to_integer(unsigned(a_wrad)))
			& " LEN=" & integer'image(to_integer(unsigned(a_len)));

		-- read the ADPCM DRAM back over the SS_AD save-walk and verify content
		report "TBADP: === SS_AD save-walk read-back ===";
		sleep <= '1';
		clks(4);
		diff := 0;
		for i in 0 to N-1 loop
			ss_read(i, g);
			if g /= pat(i) then
				diff := diff + 1;
				if diff <= 12 then
					report "TBADP SSREAD[" & integer'image(i) & "]=" & integer'image(g)
						& " exp=" & integer'image(pat(i)) severity warning;
				end if;
			end if;
		end loop;
		sleep <= '0';
		clks(4);
		if diff = 0 then
			report "TBADP: ADPCM RAM write + SS_AD read round-trip OK (" & integer'image(N) & " bytes)";
		else
			report "TBADP: ADPCM RAM mismatch in " & integer'image(diff) & "/" & integer'image(N)
				& " bytes" severity warning;
		end if;

		-- PLAY: OFFS=0 read-start (bit3), OFFS=N length (bit4), FREQ, PLAY (bit5)
		report "TBADP: === play ===";
		wr(16#0D#, 16#80#);		-- reset engine first
		wr(16#0D#, 16#00#);
		wr(16#08#, 16#00#);
		wr(16#09#, 16#00#);
		wr(16#0D#, 16#08#);		-- bit3: RDADDR<=OFFS<<1
		wr(16#08#, N mod 256);
		wr(16#09#, N / 256);
		wr(16#0D#, 16#18#);		-- bit4 (LEN<=OFFS) + bit3
		wr(16#0E#, 16#0F#);		-- FREQ = fastest
		wr(16#0D#, 16#20#);		-- bit5=1: PLAY
		for i in 1 to 12 loop
			clks(2000);
			report "TBADP play +" & integer'image(i) & " PLAY=" & std_logic'image(a_play)
				& " LEN=" & integer'image(to_integer(unsigned(a_len)))
				& " RDADDR=" & integer'image(to_integer(unsigned(a_rdad)))
				& " END=" & std_logic'image(a_end)
				& " M5205=" & integer'image(to_integer(m_sout))
				& " AD_S=" & integer'image(to_integer(ad_s))
				& " | nCE=" & integer'image(n_ce) & " nMCLK=" & integer'image(n_mclk)
				& " nVCK=" & integer'image(n_vck) & " nPRP=" & integer'image(n_prp)
				& " CLKCNT=" & integer'image(to_integer(m_ccnt))
				& " nXTI=" & integer'image(n_xti)
				& " mSLEEP=" & std_logic'image(m_slp) & " mSSL=" & std_logic'image(m_ssl)
				& " mRSTN=" & std_logic'image(m_rstn);
		end loop;

		-- ================================================================
		-- Scenario D: ADPCM PLAYBACK across a savestate save/load.
		--
		-- A2/the RAM round-trip above only prove the ADPCM DRAM survives.  The
		-- hardware defect class ("crackle/wrong sample after load") also needs
		-- the DECODER state to survive: SAMPLE, STEP, DEC_DATA/DEC_EXEC, the
		-- MSM clock divider, RDADDR/LEN and the pending-read flags.  Any of
		-- those lost and playback continues from a wrong point.
		--
		-- Method: freeze at a mid-playback point and capture every CD savestate
		-- register.  Let the machine run on and record the reference tape (the
		-- next D_K decoded samples, indexed by VCK events so pure clock-phase
		-- jitter cannot make the test flaky).  Then restore the captured
		-- registers and record the tape again.  A complete restore makes the
		-- two tapes bit-identical; anything missing shows up as a divergence,
		-- and the index of the first difference names how far the resumed
		-- stream stayed correct.
		-- ================================================================
		report "TBADP: === scenario D: playback across save/load ===";

		-- restart playback from the top so the tape has plenty of sample left
		wr(16#0D#, 16#80#);
		wr(16#0D#, 16#00#);
		wr(16#08#, 16#00#);
		wr(16#09#, 16#00#);
		wr(16#0D#, 16#0C#);		-- bit2 (arm) + bit3 (reload mode)
		wr(16#0D#, 16#00#);		-- bit2 falls -> RDADDR <= OFFS<<1 = 0
		wr(16#08#, N mod 256);
		wr(16#09#, N / 256);
		wr(16#0D#, 16#10#);		-- bit4: LEN <= OFFS
		wr(16#0D#, 16#00#);
		wr(16#0E#, 16#0F#);		-- FREQ = fastest
		wr(16#0D#, 16#20#);		-- PLAY
		clks(4000);				-- run into the middle of the sample

		-- freeze and capture the CD register file
		sleep <= '1';
		clks(8);
		for i in SSREG_INDEX_CD_1 to SSREG_INDEX_CD_6 loop
			ss_reg_read(i, snap(i - SSREG_INDEX_CD_1));
		end loop;
		report "TBADP D: captured at PLAY=" & std_logic'image(a_play)
			& " RDADDR=" & integer'image(to_integer(unsigned(a_rdad)))
			& " LEN=" & integer'image(to_integer(unsigned(a_len)))
			& " SAMPLE=" & integer'image(to_integer(m_sout));
		sleep <= '0';
		clks(4);

		-- reference tape: the samples the machine produces WITHOUT a load
		for k in 0 to D_K-1 loop
			wait until rising_edge(clk) and m_vck = '1';
			clks(8);
			ref_tape(k) := to_integer(m_sout);
		end loop;

		-- restore the captured registers and replay the same stretch
		sleep <= '1';
		clks(8);
		for i in SSREG_INDEX_CD_1 to SSREG_INDEX_CD_6 loop
			ss_reg_write(i, snap(i - SSREG_INDEX_CD_1));
		end loop;
		ss_pulse_load;
		clks(8);
		report "TBADP D: restored to PLAY=" & std_logic'image(a_play)
			& " RDADDR=" & integer'image(to_integer(unsigned(a_rdad)))
			& " LEN=" & integer'image(to_integer(unsigned(a_len)))
			& " SAMPLE=" & integer'image(to_integer(m_sout));
		sleep <= '0';
		clks(4);

		for k in 0 to D_K-1 loop
			wait until rising_edge(clk) and m_vck = '1';
			clks(8);
			rep_tape(k) := to_integer(m_sout);
		end loop;

		diff := 0;
		for k in 0 to D_K-1 loop
			if ref_tape(k) /= rep_tape(k) then
				diff := diff + 1;
				if diff <= 8 then
					report "TBADP D: sample " & integer'image(k) & " ref="
						& integer'image(ref_tape(k)) & " replay=" & integer'image(rep_tape(k))
						severity error;
				end if;
			end if;
		end loop;
		assert diff = 0
			report "TBADP D FAILED: playback diverges after load in "
				& integer'image(diff) & "/" & integer'image(D_K) & " samples"
			severity failure;
		report "TBADP: scenario D PASSED (" & integer'image(D_K)
			& " decoded samples identical across save/load)";

		-- ================================================================
		-- Scenario D2: playback still TERMINATES where it should after a load.
		--
		-- Scenario D proves the decoded samples match for a stretch; it says
		-- nothing about the end of the sample.  The remaining unexplained
		-- hardware report is "audio went into a loop", and the mechanisms that
		-- would produce it all live in the termination path: ADPCM_LEN counts
		-- the sample down and raises ADPCM_END at zero, but ADPCM_CTRL bit4
		-- reloads LEN from OFFS on a LEVEL, and bit3 reloads RDADDR from OFFS on
		-- every play read.  A blob that restores either bit set - or a LEN /
		-- RDADDR that comes back wrong - gives playback that never ends or that
		-- restarts from the top: exactly "in loop".
		--
		-- Method: from one mid-playback snapshot, measure how many decode steps
		-- remain until ADPCM_END with no load at all, then restore the same
		-- snapshot and measure again.  Equal counts (and END actually reached
		-- both times) means the load did not extend, shorten or restart the
		-- sample.
		-- ================================================================
		report "TBADP: === scenario D2: playback termination across save/load ===";

		wr(16#0D#, 16#80#);
		wr(16#0D#, 16#00#);
		wr(16#08#, 16#00#);
		wr(16#09#, 16#00#);
		wr(16#0D#, 16#0C#);		-- bit2 (arm) + bit3 (reload mode)
		wr(16#0D#, 16#00#);		-- bit2 falls -> RDADDR <= 0
		wr(16#08#, N mod 256);
		wr(16#09#, N / 256);
		wr(16#0D#, 16#10#);		-- bit4: LEN <= OFFS
		wr(16#0D#, 16#00#);
		wr(16#0E#, 16#0F#);
		wr(16#0D#, 16#20#);		-- PLAY
		clks(4000);

		sleep <= '1';
		clks(8);
		for i in SSREG_INDEX_CD_1 to SSREG_INDEX_CD_6 loop
			ss_reg_read(i, snap(i - SSREG_INDEX_CD_1));
		end loop;
		sleep <= '0';
		clks(4);

		-- reference: steps to END with no load
		ref_steps := -1;
		for k in 0 to D_MAXSTEP-1 loop
			wait until rising_edge(clk) and m_vck = '1';
			clks(8);
			if a_end = '1' then
				ref_steps := k;
				exit;
			end if;
		end loop;
		assert ref_steps >= 0
			report "TBADP D2 SETUP FAILED: playback never ended even without a load"
			severity failure;
		report "TBADP D2: reference reached ADPCM_END after "
			& integer'image(ref_steps) & " decode steps";

		-- restore the same mid-playback snapshot and measure again
		sleep <= '1';
		clks(8);
		for i in SSREG_INDEX_CD_1 to SSREG_INDEX_CD_6 loop
			ss_reg_write(i, snap(i - SSREG_INDEX_CD_1));
		end loop;
		ss_pulse_load;
		clks(8);
		sleep <= '0';
		clks(4);

		rep_steps := -1;
		for k in 0 to D_MAXSTEP-1 loop
			wait until rising_edge(clk) and m_vck = '1';
			clks(8);
			if a_end = '1' then
				rep_steps := k;
				exit;
			end if;
		end loop;
		assert rep_steps >= 0
			report "TBADP D2 FAILED: after a load the sample NEVER ends (LEN/CTRL "
				& "restored so playback runs forever) -- this is the 'audio in loop' class"
			severity failure;
		assert rep_steps = ref_steps
			report "TBADP D2 FAILED: sample ends after " & integer'image(rep_steps)
				& " steps with a load but " & integer'image(ref_steps) & " without"
			severity failure;
		report "TBADP: scenario D2 PASSED (ADPCM_END after "
			& integer'image(rep_steps) & " steps, with and without a load)";

		-- ================================================================
		-- Scenario D3: restore a PLAYING state into a STOPPED machine.
		--
		-- D and D2 both restore into a machine that is already playing, so the
		-- decoder's reset (derived from ADPCM_PLAY) happens to be released and
		-- the restore lands.  Real use is the opposite: you load while nothing
		-- is playing.  MSM5205 resets DEC_DATA/STEP/SAMPLE asynchronously, so
		-- that reset used to win over the synchronous SS_load and the decoder
		-- resumed from SAMPLE=0/STEP=0 - and ADPCM being differential, the rest
		-- of the sample came out wrong.  Hardware hit this on a savestate taken
		-- while a streamed sample was playing.
		--
		-- Method: reuse the mid-playback snapshot, but STOP playback before
		-- restoring, so the machine is in the state a real load starts from.
		-- The tape must still match the reference recorded with no load at all.
		-- ================================================================
		report "TBADP: === scenario D3: restore a playing state into a stopped machine ===";

		wr(16#0D#, 16#80#);
		wr(16#0D#, 16#00#);
		wr(16#08#, 16#00#);
		wr(16#09#, 16#00#);
		wr(16#0D#, 16#0C#);
		wr(16#0D#, 16#00#);
		wr(16#08#, N mod 256);
		wr(16#09#, N / 256);
		wr(16#0D#, 16#10#);
		wr(16#0D#, 16#00#);
		wr(16#0E#, 16#0F#);
		wr(16#0D#, 16#20#);		-- PLAY
		clks(4000);

		sleep <= '1';
		clks(8);
		for i in SSREG_INDEX_CD_1 to SSREG_INDEX_CD_6 loop
			ss_reg_read(i, snap(i - SSREG_INDEX_CD_1));
		end loop;
		snap_sample := to_integer(m_sample);
		snap_step   := to_integer(m_step);
		sleep <= '0';
		clks(4);

		for k in 0 to D_K-1 loop
			wait until rising_edge(clk) and m_vck = '1';
			clks(8);
			ref_tape(k) := to_integer(m_sout);
		end loop;

		-- stop playback: this is what makes the decoder's reset assert, exactly
		-- as it is on a machine sitting idle when the user presses load
		wr(16#0D#, 16#00#);		-- clear PLAY
		clks(200);
		assert a_play = '0'
			report "TBADP D3 SETUP FAILED: playback did not stop" severity failure;

		sleep <= '1';
		clks(8);
		for i in SSREG_INDEX_CD_1 to SSREG_INDEX_CD_6 loop
			ss_reg_write(i, snap(i - SSREG_INDEX_CD_1));
		end loop;
		ss_pulse_load;
		clks(8);
		sleep <= '0';
		clks(4);
		assert a_play = '1'
			report "TBADP D3 FAILED: PLAY did not come back from the blob" severity failure;
		-- Did the decoder state actually survive?  MSM5205 resets DEC_DATA/STEP/
		-- SAMPLE asynchronously off ADPCM_PLAY, so a restore taken while the
		-- machine is stopped can be thrown away without the tape necessarily
		-- showing it.  Print the decoder registers so the answer is direct.
		report "TBADP D3: after restore SAMPLE=" & integer'image(to_integer(m_sample))
			& " STEP=" & integer'image(to_integer(m_step))
			& " (snapshot had SAMPLE=" & integer'image(snap_sample)
			& " STEP=" & integer'image(snap_step) & ")";

		for k in 0 to D_K-1 loop
			wait until rising_edge(clk) and m_vck = '1';
			clks(8);
			rep_tape(k) := to_integer(m_sout);
		end loop;

		diff := 0;
		for k in 0 to D_K-1 loop
			if ref_tape(k) /= rep_tape(k) then
				diff := diff + 1;
				if diff <= 8 then
					report "TBADP D3: sample " & integer'image(k) & " ref="
						& integer'image(ref_tape(k)) & " replay=" & integer'image(rep_tape(k))
						severity error;
				end if;
			end if;
		end loop;
		assert diff = 0
			report "TBADP D3 FAILED: the decoder state did not survive a load taken "
				& "from a stopped machine (" & integer'image(diff) & "/"
				& integer'image(D_K) & " samples differ)"
			severity failure;
		report "TBADP: scenario D3 PASSED (" & integer'image(D_K)
			& " samples identical restoring into a stopped machine)";

		report "TBADP: DONE";
		std.env.finish;
	end process;

end architecture;
