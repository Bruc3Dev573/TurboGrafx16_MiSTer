-----------------------------------------------------------------------------
-- CD-subsystem savestate testbench.
--
-- pce_top with CD_EN=1 plus:
--  * a synthetic BIOS whose 6280 program drives the REAL SCSI register
--    protocol ($1800 SEL / $1801 data / $1802 ACK, REQ-polled) to issue
--    SAPSP(D8,silent)+SAPEP(D9,loop) like a real game, copies a routine
--    into CD-RAM and executes it from there, and counts in WRAM;
--  * an HPS drive model with the VERIFIED Main_MiSTer pcecdd.cpp semantics
--    (D8 resets end+mode, statuses on latency-gated poll ticks, CDDA
--    sectors with DETERMINISTIC content: frame n of the track has sample
--    value n mod 65536 on both channels);
--  * a shared ROM/CD-RAM data port matching TurboGrafx16.sv: one data
--    register, rom_rd raddr priority and ce_rom pacing;
--  * savestate watchdogs for freeze and walk/refetch timeouts.
--
-- Scenarios:
--  A. exec-from-CD-RAM determinism: save/run/save/load/run/save + compare.
--  B. CDDA sample-exact resume: save mid-music, keep playing, load; the
--     replayed D8/D9 reposition the model and the first resumed samples
--     must CONTINUE the consumed-count pattern exactly.
--
-- CD-RAM shrunk to 4KB (SS_CDRAM_BYTES) so full walks take sim-ms.
-- Run: sim/run_tb_cd.sh
-----------------------------------------------------------------------------
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;

entity tb_cd is
	generic (
		-- Scenario selection, so the expensive parts can be run separately and
		-- in parallel.  The verified-READ phase costs ~85 ms of simulation per
		-- READ and exists only for scenario C; scenarios B, G and F need none
		-- of it, and with G_READS=0 they start about 280 ms of simulation
		-- earlier.  ghdl -gG_READS=0 -gG_DO_A=false gives the fast music+BRAM
		-- run; the defaults give the full matrix.
		G_READS : integer := 3;
		G_DO_A  : boolean := true;
		-- SAPEP mode (D9 byte1): 1 = LOOP, 2 = INTERRUPT. INTERRUPT replies
		-- at track end, leaving BSY set with SP_FREE while audio plays.
		-- Only non-INTERRUPT SAPEP counts for SWALLOW; only LOOP folds position.
		G_D9_MODE : integer := 1;
		-- Scenario G needs a short loop to exercise wraparound. Mid-track
		-- INTERRUPT tests need a long window so audio outlasts the save.
		G_TRACK_SECT : integer := 4;
		-- Scenario K: save after interrupt-mode audio ends, with a track-end
		-- notification still owed. Use a short window and INTERRUPT mode:
		--   -gG_DO_K=true -gG_D9_MODE=2 -gG_TRACK_SECT=4 \
		--   -gG_DO_B=false -gG_DO_G=false -gG_DO_F=false
		-- (B and G assume LOOP mode and would fail on their own oracle here.)
		G_DO_K       : boolean := false;
		G_DO_L       : boolean := false;
		G_L_PHASE    : integer := 0; -- 0=all (10,11,4,7), 10=DATAIN_START, 11=DATAIN_END, 4=STAT_START, 7=MSGIN_START
		-- Disable these for scenario K, whose INTERRUPT mode ends the audio.
		G_DO_B       : boolean := true;
		G_DO_F       : boolean := true;
		G_DO_G       : boolean := true;
		-- Scenario H: CD-to-ADPCM DMA across save/load. Issue its READ6 before
		-- D8/D9 because READ6 leaves H_PLAY and would stop B/G/F's audio.
		G_DO_H       : boolean := true;
		-- Sectors in scenario H's DMA READ6.  Measured rate: the DMA takes about
		-- 1.35 bytes per simulated microsecond (the drive model's 50 us tick and
		-- its consume-before-push pacing dominate, not the byte handshake), so a
		-- sector costs ~1.5 ms of simulation.  32 sectors = 65536 bytes = exactly
		-- one full pass over the 64 KB ADPCM RAM in ~49 ms, which leaves room for
		-- the freeze and the replay inside a sane run.
		--
		-- Set it to 0 to send count=0, which SCSI defines as 256 sectors: that is
		-- the branch SCSI.vhd's RP_RD_PREP has a special case for, but it costs
		-- ~390 ms of streaming, so it is not the default.
		G_H_SECT     : integer := 32;
		-- Scenario I: an in-flight non-READ6 command across save/load.
		-- $DD (READ SUBQ), issued by the System Card's CD_SUBQ, must complete.
		G_DO_I       : boolean := true;
		-- Drive latency for the ARMED $DD, in 50 us model ticks.  Only applied
		-- while the scenario arms it, so the main loop's ordinary $DD stays fast
		-- and B/G/F keep their timing.  It has to exceed the freeze latency (the
		-- composite boundary waits for VBLANK, up to ~17 ms) or the save would
		-- land after the answer instead of during the wait.
		G_I_LAT      : integer := 800;
		-- Scenario I2: deliver the $DD reply during the freeze, when
		-- REPLAY_START can flush it from the FIFO. Hold it until SLEEP
		-- (i_freeze_ans), avoiding the VBLANK-dependent boundary latency.
		G_DO_I2      : boolean := true;
		-- CD-RAM size for the external-memory walk. Both sizes keep the
		-- hardware byte pacing; Main pauses CDDA delivery during SLEEP.
		-- In-flight sectors may still finish and are checked for overflow.
		-- Set 262144 to exercise the hardware-sized freeze.
		G_CDRAM_BYTES : integer := 4096;
		-- Scenario J: VDC VBLANK interrupts across save/load. The BIOS handler
		-- acknowledges via VDC status and counts in WRAM. Check interrupt rate
		-- against baseline and require the stack pointer to remain bounded.
		G_DO_J       : boolean := true;
		-- Scenario J2: CD IRQ2 across save/load. Match the System Card handler:
		-- disable the source (TRB $1802), CLI inside the handler, acknowledge,
		-- re-enable (TSB), RTI. SUBCD_EN supplies ~7.35 kHz subcode interrupts.
		G_J2         : boolean := false;
		-- Scenario W: save/load mid-READ6 and after all data is consumed but
		-- before final STATUS (RP_RD_PREP's sect>=cnt branch), then verify a
		-- fresh READ6 at another LBA. Requires G_PUSH_STATUS=true.
		G_DO_W       : boolean := false;
		-- Scenario W2: run W, then D8+D9, save/load during audio, and issue
		-- two READ6s back-to-back. Check data, STATUS, MSGIN and the program's
		-- end marker. Requires G_PUSH_STATUS=true.
		G_DO_W2      : boolean := false;
		-- Scenario W3: W2 with G_REAL_HPS and a save requested during D8 seek
		-- latency. The pending status occupies the single deferred slot;
		-- a replay D8 must not overwrite the game's undelivered status.
		G_DO_W3      : boolean := false;
		-- REAL Main_MiSTer HPS semantics (support/pcecd), superseding
		-- G_PUSH_STATUS when true:
		--  * ONE deferred-status slot {stat, has_status}: READ6 completion and
		--    D8 use PendStatus (a second pend before delivery silently
		--    OVERWRITES the first - lost forever); DA / non-INT D9 etc. are
		--    immediate SendStatus.
		--  * delivery quantization: the pended status goes out only on a
		--    13.33 ms tick AND only once the seek latency has expired.
		--  * persistent head position (advanced +1 per pushed sector); READ6
		--    latency from the seek distance (==head & cnt=120 -> 0, the
		--    HuVideo path; <=3 sectors -> 33 ms; <7 -> ~250 ms; else
		--    G_SEEK_MS).  During latency nothing streams and the deferred
		--    slot is frozen.
		--  * D8: latency = seek - G_AUDIODELAY_MS (floor 0), status PENDED.
		--  * command overwrite: a new command silently replaces
		--    state/count/latency; an abandoned op never sends its status.
		--  * cadence: one 2048-byte sector per 16 ms tick while reading and
		--    latency==0, gated per-sector on the previous being consumed
		--    (can_read_next, primed at READ6 issue).
		--  * a savestate load leaves HPS state unchanged.
		G_REAL_HPS   : boolean := false;
		-- Distant-jump seek time in ms. Real seeks take 300-600 ms;
		-- this shortened default does not cover slow seeks.
		G_SEEK_MS    : integer := 50;
		-- the ~220 ms the real SAPSP handler subtracts from its seek
		G_AUDIODELAY_MS : integer := 220;
		-- Stock Main_MiSTer ignores the CD_GET hold bit (hps_ext bit 8): its
		-- audio sector timer keeps running through every freeze. The default
		-- models the build that pauses delivery while SS_SLEEP is high.
		G_STOCK_MAIN : boolean := false;
		-- Seek/delivery latency (ms) between a D8/D9 that starts playback and
		-- its first audio sector. 0 = the first sector follows the command.
		-- A D8 with the fast-seek bit has none; a D9 inherits its D8's.
		G_PLAY_LAT_MS : integer := 0;
		-- W3 positioning knob: pin the D8 seek latency (ms) independently of
		-- G_SEEK_MS.  The real distant audio seek minus audiodelay is
		-- 80-380 ms; with the run-shortened G_SEEK_MS the derived value would
		-- collapse below the save walk and the frozen-slot window would close
		-- before the replay's D8 arrives.  0 = derive from the seek tiers.
		G_W3_D8LAT_MS : integer := 0;
		-- Push-based READ6: fill the 4096-byte SCSI_FIFO under backpressure.
		-- Final STATUS follows the last pushed byte by G_STAT_TICKS ticks,
		-- not the last consumed byte. The default consumption-gated model
		-- cannot open this tail window.
		G_PUSH_STATUS : boolean := false;
		-- Final-STATUS latency in 50 us model ticks after the last pushed byte
		-- (push-status mode only).  The real HPS strobe rides a poll tick.
		G_STAT_TICKS  : integer := 2
	);
end entity;

architecture sim of tb_cd is

	constant CLK_PERIOD : time := 10 ns;
	-- One CDDA sample at this clock: CEGen divides 429545 by 441 (974.03 CLK),
	-- so consecutive DAC ticks are 974 or 975 clocks apart.
	constant CDDA_CLKS  : integer := (429545 + 440) / 441;
	constant BASE_DW    : integer := 16#3800000#;
	constant SLOT_DW    : integer := 16#C0000#;
	constant CDRAM_SIM  : integer := G_CDRAM_BYTES;
	-- Arcade is unused in this bench: omit region 11, retain all 128 eRegs.
	-- DDR model stores 64-bit words; slot stride remains 3MB.
	constant SLOT_WORDS : integer := (2 + 256 + (528576 - 262144 + G_CDRAM_BYTES) / 4) / 2;
	constant TRACK_LBA  : integer := 4000;		-- 0x000FA0
	-- Short loop window for scenario G's consumed-count wrap check.
	constant TRACK_END  : integer := TRACK_LBA + G_TRACK_SECT;
	-- the drive plays the window INCLUSIVE of the end sector, so the loop is
	-- END-START+1 sectors of 588 frames each -- this is the oracle scenario G
	-- measures the replay's fold-back against
	constant LOOP_SAMPLES : integer := (TRACK_END - TRACK_LBA + 1) * 588;

	-- Scenario H sizing.  H_NIB_TOTAL is two nibbles per transferred byte (the
	-- DMA stores each byte as high nibble then low nibble); H_TRIG is the point
	-- at which the scenario takes its savestate -- deliberately at 30%, mid
	-- SECTOR as well as mid transfer, so the replay has to rebuild both the
	-- sector index and the byte offset inside it.
	function h_sect_count return integer is
	begin
		if G_H_SECT = 0 then return 256; else return G_H_SECT; end if;
	end function;
	constant H_SECT_N    : integer := h_sect_count;
	constant H_NIB_TOTAL : integer := H_SECT_N * 2048 * 2;
	constant H_TRIG      : integer := (H_NIB_TOTAL * 3) / 10;

	signal clk      : std_logic := '0';
	signal reset    : std_logic := '1';

	signal ss_save  : std_logic := '0';
	signal ss_load  : std_logic := '0';
	signal ss_slot  : std_logic_vector(1 downto 0) := "00";
	signal ss_busy  : std_logic;
	signal ss_sleep : std_logic;
	signal l_cnt_req, l_cnt_ack : unsigned(15 downto 0) := (others => '0');

	signal ddr_din  : std_logic_vector(63 downto 0);
	signal ddr_dout : std_logic_vector(63 downto 0) := (others => '0');
	signal ddr_adr  : std_logic_vector(25 downto 0);
	signal ddr_rnw  : std_logic;
	signal ddr_ena  : std_logic;
	signal ddr_be   : std_logic_vector(7 downto 0);
	signal ddr_done : std_logic := '0';

	signal rom_rd    : std_logic;
	signal rom_a     : std_logic_vector(21 downto 0);
	signal rom_do    : std_logic_vector(7 downto 0) := (others => '0');
	signal rom_rdy   : std_logic := '1';
	signal rom_clken : std_logic;

	signal cdram_a   : std_logic_vector(21 downto 0);
	signal cdram_do  : std_logic_vector(7 downto 0);
	signal cdram_di  : std_logic_vector(7 downto 0) := (others => '0');
	signal cdram_rd  : std_logic;
	signal cdram_wr  : std_logic;

	signal cd_stat     : std_logic_vector(7 downto 0) := (others => '0');
	signal cd_stat_get : std_logic := '0';
	signal cd_comm     : std_logic_vector(95 downto 0);
	signal cd_comm_send: std_logic;
	signal cd_fast_seek: std_logic;
	signal cd_data     : std_logic_vector(7 downto 0) := (others => '0');
	signal cd_data_wr  : std_logic := '0';
	signal cd_audio_wr : std_logic := '0';
	signal cd_subcd_wr : std_logic := '0';
	signal cd_data_end : std_logic;

	signal cdda_sl  : signed(15 downto 0);
	signal cdda_sr  : signed(15 downto 0);

	-- Model the 2 KB backup RAM (page $F7) supplied by TurboGrafx16.sv.
	-- Scenario F checks that region 7 restores it without persistent data.
	signal brm_a   : std_logic_vector(10 downto 0);
	signal brm_di  : std_logic_vector(7 downto 0);
	signal brm_do  : std_logic_vector(7 downto 0) := (others => '0');
	signal brm_we  : std_logic;

	-- =====================================================================
	-- Synthetic BIOS (reset bank, page $FF00; I/O bank $FF at $4000 via
	-- MPR2, WRAM at $2000 via MPR1, CD-RAM bank $68 at $6000 via MPR3).
	-- CD regs therefore at $5800+ ($4000 window offset $1800).
	--
	--   FF00  LDA #$F8 : TAM1          ; WRAM
	--   FF04  LDA #$FF : TAM2          ; I/O at $4000 (CD at $5800)
	--   FF08  LDA #$68 : TAM3          ; CD-RAM bank $68 at $6000
	--   ; copy INC $2002 : RTS  -> $6000  {EE 02 20 60}
	--   FF0C  LDA #$EE : STA $6000
	--   FF11  LDA #$02 : STA $6001
	--   FF16  LDA #$20 : STA $6002
	--   FF1B  LDA #$60 : STA $6003
	--   FF20  JSR SEND_D8              ; SAPSP silent  (table at FFC0)
	--   FF23  JSR SEND_D9              ; SAPEP loop    (table at FFD0)
	--   loop:
	--   FF26  INC $2000
	--   FF29  JSR $6000                ; exec from CD-RAM
	--   FF2C  INC $2001
	--   FF2F  JMP $FF26
	--
	-- SEND_CMD core (FF40, X = table page offset $C0 or $D0):
	--   sends 10 bytes with REQ/ACK handshake, then eats STATUS+MSGIN
	--   bytes with manual ACK.  REQ = $5800 bit6.
	-- =====================================================================
	type rom_t is array (0 to 8191) of std_logic_vector(7 downto 0);
	function rom_init return rom_t is
		variable r : rom_t := (others => x"EA");
		variable a : integer;
		procedure org(x : integer) is begin a := x; end procedure;
		procedure b(x : integer) is
		begin
			r(a) := std_logic_vector(to_unsigned(x, 8));
			a := a + 1;
		end procedure;
		-- Hand-assembled code uses absolute addresses. Catch layout shifts
		-- at elaboration rather than executing filler or command-table data.
		procedure chk(x : integer) is
		begin
			assert a = x
				report "tb_cd BIOS misaligned: at " & integer'image(a)
					& " (rom offset), expected " & integer'image(x)
				severity failure;
		end procedure;
	begin
		org(16#1F00#);                            -- $FF00
		b(16#A9#); b(16#F8#); b(16#53#); b(16#02#);   -- LDA #$F8 TAM1
		b(16#A9#); b(16#FF#); b(16#53#); b(16#04#);   -- LDA #$FF TAM2
		b(16#A9#); b(16#68#); b(16#53#); b(16#08#);   -- LDA #$68 TAM3
		-- Enable VDC0 display (CR=$CC) for periodic VBLANK boundaries.
		b(16#A9#); b(16#05#); b(16#8D#); b(16#00#); b(16#40#);   -- AR=CR
		b(16#A9#); b(16#CC#); b(16#8D#); b(16#02#); b(16#40#);   -- CR lo=$CC
		b(16#A9#); b(16#00#); b(16#8D#); b(16#03#); b(16#40#);   -- CR hi=0
		b(16#A9#); b(16#EE#); b(16#8D#); b(16#00#); b(16#60#);
		b(16#A9#); b(16#02#); b(16#8D#); b(16#01#); b(16#60#);
		b(16#A9#); b(16#20#); b(16#8D#); b(16#02#); b(16#60#);
		b(16#A9#); b(16#60#); b(16#8D#); b(16#03#); b(16#60#);
		-- $FF20: N verified READs, then D8+D9, then the counter loop.
		-- Repeated reads keep scenario A inside a transfer, then check
		-- alignment across subsequent command boundaries.
		b(16#A9#); b(G_READS);                        -- LDA #G_READS
		b(16#8D#); b(16#16#); b(16#20#);              -- STA $2016
		-- With G_READS = 0 the JSR and the BNE are replaced by NOPs of the SAME
		-- length rather than omitted, so every address below (the JSR $FF52
		-- calls, the main loop, SEND_CMD) stays exactly where it is.
		if G_READS > 0 then
			b(16#20#); b(16#00#); b(16#FE#);          -- $FF34 JSR $FE00 (read+verify)
		else
			b(16#EA#); b(16#EA#); b(16#EA#);
		end if;
		b(16#CE#); b(16#16#); b(16#20#);              -- DEC $2016
		if G_READS > 0 then
			b(16#D0#); b(16#F8#);                     -- BNE -8 -> $FF34
		else
			b(16#EA#); b(16#EA#);
		end if;
		-- $FF3C: the ADPCM-DMA transfer (scenario H).  It has to run HERE,
		-- before the D8/D9: a READ6 puts the drive model into H_READ, so a DMA
		-- issued from the main loop would stop the music and take B, G and F
		-- down with it.  Same NOP-of-equal-length rule as the READ loop above,
		-- so switching H off does not move a single address.
		chk(16#1F3C#);
		if G_DO_H then
			b(16#20#); b(16#00#); b(16#FD#);          -- JSR $FD00 (ADPCM DMA)
		else
			b(16#EA#); b(16#EA#); b(16#EA#);
		end if;
		-- Scenario W replaces the music startup and the main loop with a jump
		-- to its own block at $FAC0 (fresh READ6, then park).  Same rule as
		-- every other generic: the alternative fills the SAME 10 bytes, so no
		-- address below moves.
		if G_DO_W2 or G_DO_W3 then
			b(16#4C#); b(16#D0#); b(16#FA#);          -- JMP $FAD0 (scenario W2/W3)
			for i in 1 to 7 loop b(16#EA#); end loop;
		elsif G_DO_W then
			b(16#4C#); b(16#C0#); b(16#FA#);          -- JMP $FAC0 (scenario W)
			for i in 1 to 7 loop b(16#EA#); end loop;
		elsif G_DO_L then
			b(16#4C#); b(16#00#); b(16#F8#);          -- JMP $F800 (scenario L)
			for i in 1 to 7 loop b(16#EA#); end loop;
		else
			b(16#A2#); b(16#C0#); b(16#20#); b(16#52#); b(16#FF#);   -- D8 (JSR $FF52)
			b(16#A2#); b(16#D0#); b(16#20#); b(16#52#); b(16#FF#);   -- D9 (JSR $FF52)
		end if;
		-- $FF49 jumps to the main loop at $FDC0; SEND_CMD starts at $FF52.
		chk(16#1F49#);
		b(16#4C#); b(16#C0#); b(16#FD#);              -- JMP $FDC0 (loop head)
		chk(16#1F4C#);

		-- ---------------- SEND_CMD @$FF50 ---------------------------------
		-- Sends the 10-byte command whose table starts at $FF00,X (the D8 at
		-- $FFC0 and the D9 at $FFD0), then eats the STATUS and MSGIN bytes.
		org(16#1F52#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FF52 LDA $5800
		b(16#29#); b(16#80#);                         -- AND #$80 (BSY)
		b(16#D0#); b(16#F9#);                         -- BNE -7 (wait bus free)
		b(16#A9#); b(16#81#);                         -- $FF47 LDA #$81
		b(16#8D#); b(16#01#); b(16#58#);              -- STA $5801
		b(16#8D#); b(16#00#); b(16#58#);              -- STA $5800 (SEL)
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#80#);                         -- AND #$80
		b(16#F0#); b(16#F1#);                         -- BEQ -15 -> $FF47 (re-SEL)
		b(16#A0#); b(16#0A#);                         -- LDY #10
		-- byte loop @$FF58
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40 (REQ)
		b(16#F0#); b(16#F9#);                         -- BEQ -7
		b(16#BD#); b(16#00#); b(16#FF#);              -- LDA $FF00,X
		b(16#8D#); b(16#01#); b(16#58#);              -- STA $5801
		b(16#A9#); b(16#80#);                         -- LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              -- STA $5802 (ACK)
		-- REQ release @$FF6A
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#D0#); b(16#F9#);                         -- BNE -7
		b(16#9C#); b(16#02#); b(16#58#);              -- STZ $5802
		b(16#E8#);                                    -- INX
		b(16#88#);                                    -- DEY
		b(16#D0#); b(16#E0#);                         -- BNE -32 -> $FF58
		-- STATUS @$FF78
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#F0#); b(16#F9#);                         -- BEQ -7
		b(16#AD#); b(16#01#); b(16#58#);              -- LDA $5801
		b(16#A9#); b(16#80#);                         -- LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              -- STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- $FF87 LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#D0#); b(16#F9#);                         -- BNE -7
		b(16#9C#); b(16#02#); b(16#58#);              -- STZ $5802
		-- MSGIN @$FF91
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#F0#); b(16#F9#);                         -- BEQ -7
		b(16#AD#); b(16#01#); b(16#58#);              -- LDA $5801
		b(16#A9#); b(16#80#);                         -- LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              -- STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- $FFA0 LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#D0#); b(16#F9#);                         -- BNE -7
		b(16#9C#); b(16#02#); b(16#58#);              -- STZ $5802
		b(16#60#);                                    -- RTS

		-- ---------------- VBL interrupt handler @$FB00 (scenario J) --------
		-- What a real game's IRQ1 handler does, reduced to the essentials: ack
		-- the VDC by reading its status port (the read is what clears IRQ_VBL -
		-- huc6270 clears on a status read with CPU_CE), record it, count it.
		-- The counter at $2030 is the scenario's oracle: it must advance at
		-- FRAME rate.  The storm signature is it advancing at handler rate,
		-- thousands per second, while the stack eats itself.
		org(16#1B00#);
		b(16#48#);                                    -- $FB00 PHA
		b(16#AD#); b(16#00#); b(16#40#);              --       LDA $4000 (VDC status: ack)
		b(16#8D#); b(16#31#); b(16#20#);              --       STA $2031 (last status)
		b(16#EE#); b(16#30#); b(16#20#);              --       INC $2030 (VBLs handled)
		b(16#68#);                                    --       PLA
		b(16#40#);                                    --       RTI
		chk(16#1B0C#);
		-- IRQ2 (CD) handler @$FB20, scenario J2 - deliberately in the System
		-- Card's own shape (offset $074C..): disable the specific enable bit,
		-- THEN CLI while still inside the handler, ack, re-enable, RTI.  This
		-- is the structure that fills the stack with P=$00 frames if anything
		-- in the clear path misbehaves after a restore.
		org(16#1B20#);
		b(16#48#);                                    -- $FB20 PHA
		b(16#A9#); b(16#10#);                         --       LDA #$10 (SUBCD_EN)
		b(16#1C#); b(16#02#); b(16#58#);              --       TRB $5802  (enable OFF)
		b(16#58#);                                    --       CLI        (System Card does)
		b(16#AD#); b(16#07#); b(16#58#);              --       LDA $5807  (subcode byte: ack)
		b(16#8D#); b(16#32#); b(16#20#);              --       STA $2032
		b(16#EE#); b(16#33#); b(16#20#);              --       INC $2033  (IRQ2s handled)
		b(16#A9#); b(16#10#);                         --       LDA #$10
		b(16#0C#); b(16#02#); b(16#58#);              --       TSB $5802  (enable back ON)
		b(16#68#);                                    --       PLA
		b(16#40#);                                    --       RTI
		chk(16#1B37#);
		-- enable @$FB40: unmask IRQ1 only (IRQ2/timer stay masked: the CD
		-- interrupt must not start competing with the polling scenarios), CLI.
		-- Idempotent - the main loop calls it every pass.
		org(16#1B40#);
		b(16#A9#); b(16#05#);                         -- $FB40 LDA #$05
		b(16#8D#); b(16#02#); b(16#54#);              --       STA $5402 (INT_MASK)
		b(16#58#);                                    --       CLI
		b(16#60#);                                    --       RTS
		chk(16#1B47#);
		-- J2 enable @$FB60: unmask IRQ2 only, CLI
		org(16#1B60#);
		b(16#A9#); b(16#06#);                         -- $FB60 LDA #$06
		b(16#8D#); b(16#02#); b(16#54#);              --       STA $5402
		b(16#58#);                                    --       CLI
		b(16#60#);                                    --       RTS
		chk(16#1B67#);
		-- J2 source arm @$FB70: SUBCD_EN on via TSB, preserving the other
		-- $1802 bits (a plain STA would stomp the ACK bit mid-handshake)
		org(16#1B70#);
		b(16#A9#); b(16#10#);                         -- $FB70 LDA #$10
		b(16#0C#); b(16#02#); b(16#58#);              --       TSB $5802
		b(16#60#);                                    --       RTS
		chk(16#1B76#);

		-- ---------------- SUBQ ($DD) @$FC00 (scenario I) -------------------
		-- What CD_SUBQ does in the real System Card (docs/Super CD-ROM2 System
		-- V3.00 (J).ASM, offset 0FBF): put $DD and an allocation length of $0A in
		-- the command buffer, send it, then read the 10-byte answer.  Here the
		-- 10-byte command comes from the table at $FFE6 and the answer is
		-- consumed through $5808 exactly like the READ6 reader does, verifying
		-- the drive model's $A0..$A9 pattern.
		org(16#1C00#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FC00 LDA $5800
		b(16#29#); b(16#80#);                         --       AND #$80 (BSY)
		b(16#D0#); b(16#F9#);                         --       BNE -7 -> $FC00
		chk(16#1C07#);
		b(16#A9#); b(16#81#);                         -- $FC07 LDA #$81
		b(16#8D#); b(16#01#); b(16#58#);              --       STA $5801
		b(16#8D#); b(16#00#); b(16#58#);              --       STA $5800 (SEL)
		b(16#AD#); b(16#00#); b(16#58#);              --       LDA $5800
		b(16#29#); b(16#80#);                         --       AND #$80
		b(16#F0#); b(16#F1#);                         --       BEQ -15 -> $FC07
		b(16#A2#); b(16#E6#);                         --       LDX #$E6 ($FFE6)
		b(16#A0#); b(16#0A#);                         --       LDY #10
		chk(16#1C1A#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FC1A LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40 (REQ)
		b(16#F0#); b(16#F9#);                         --       BEQ -7 -> $FC1A
		b(16#BD#); b(16#00#); b(16#FF#);              --       LDA $FF00,X
		b(16#8D#); b(16#01#); b(16#58#);              --       STA $5801
		b(16#A9#); b(16#80#);                         --       LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              --       STA $5802 (ACK)
		chk(16#1C2C#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FC2C LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#D0#); b(16#F9#);                         --       BNE -7 -> $FC2C
		b(16#9C#); b(16#02#); b(16#58#);              --       STZ $5802
		b(16#E8#);                                    --       INX
		b(16#88#);                                    --       DEY
		b(16#D0#); b(16#E0#);                         --       BNE -32 -> $FC1A
		-- consume the answer @$FC3A: wait REQ, bail out to STATUS on the C/D flag
		chk(16#1C3A#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FC3A LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#F0#); b(16#F9#);                         --       BEQ -7 -> $FC3A
		b(16#AD#); b(16#00#); b(16#58#);              -- $FC41 LDA $5800
		b(16#29#); b(16#10#);                         --       AND #$10 (C/D)
		b(16#D0#); b(16#09#);                         -- $FC46 BNE +9 -> $FC51
		b(16#AD#); b(16#08#); b(16#58#);              -- $FC48 LDA $5808 (data+auto-ack)
		b(16#8D#); b(16#20#); b(16#20#);              --       STA $2020 (last subq byte)
		b(16#4C#); b(16#3A#); b(16#FC#);              -- $FC4E JMP $FC3A
		-- STATUS @$FC51
		chk(16#1C51#);
		b(16#AD#); b(16#01#); b(16#58#);              -- $FC51 LDA $5801
		b(16#A9#); b(16#80#);                         --       LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              --       STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- $FC59 LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#D0#); b(16#F9#);                         --       BNE -7 -> $FC59
		b(16#9C#); b(16#02#); b(16#58#);              --       STZ $5802
		-- MSGIN @$FC63
		chk(16#1C63#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FC63 LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#F0#); b(16#F9#);                         --       BEQ -7 -> $FC63
		b(16#AD#); b(16#01#); b(16#58#);              --       LDA $5801
		b(16#A9#); b(16#80#);                         --       LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              --       STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- $FC72 LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#D0#); b(16#F9#);                         --       BNE -7 -> $FC72
		b(16#9C#); b(16#02#); b(16#58#);              --       STZ $5802
		b(16#EE#); b(16#21#); b(16#20#);              --       INC $2021 (SUBQ replies done)
		b(16#60#);                                    --       RTS
		chk(16#1C80#);

		-- ---------------- ADPCM DMA @$FD00 (scenario H) --------------------
		-- The AD_TRANS / AD_CPLAY path: the CPU points the ADPCM write address
		-- at 0, sets ADPCM_DMA_EN ($180B bit1) and issues a READ6 -- and then
		-- touches NO data at all.  The CD block grabs every DATAIN byte off the
		-- SCSI bus, writes it into ADPCM RAM and acks it itself (cd.vhd: the
		-- DMA_WRITE_PEND -> SLOT_WRITE -> AUTO_ACK chain).
		--
		-- The CPU only waits for the phase to turn to STATUS and then eats the
		-- STATUS and MSGIN bytes, exactly like the reader does.
		org(16#1D00#);
		b(16#A9#); b(16#80#);                         -- $FD00 LDA #$80
		b(16#8D#); b(16#0D#); b(16#58#);              --       STA $580D  reset: OFFS/LEN/WRADDR/RDADDR := 0
		b(16#A9#); b(16#00#);                         --       LDA #$00
		b(16#8D#); b(16#0D#); b(16#58#);              --       STA $580D  release the reset
		b(16#A9#); b(16#02#);                         --       LDA #$02
		b(16#8D#); b(16#0B#); b(16#58#);              --       STA $580B  ADPCM_DMA_EN = 1
		-- send READ6 (same table as the reader, $FFE0)
		chk(16#1D0F#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FD0F LDA $5800
		b(16#29#); b(16#80#);                         --       AND #$80 (BSY)
		b(16#D0#); b(16#F9#);                         --       BNE -7 -> $FD0F
		chk(16#1D16#);
		b(16#A9#); b(16#81#);                         -- $FD16 LDA #$81
		b(16#8D#); b(16#01#); b(16#58#);              --       STA $5801
		b(16#8D#); b(16#00#); b(16#58#);              --       STA $5800 (SEL)
		b(16#AD#); b(16#00#); b(16#58#);              --       LDA $5800
		b(16#29#); b(16#80#);                         --       AND #$80
		b(16#F0#); b(16#F1#);                         --       BEQ -15 -> $FD16 (re-SEL)
		b(16#A2#); b(16#F0#);                         --       LDX #$F0 (READ6 table $FFF0)
		b(16#A0#); b(16#06#);                         --       LDY #6
		chk(16#1D29#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FD29 LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40 (REQ)
		b(16#F0#); b(16#F9#);                         --       BEQ -7 -> $FD29
		b(16#BD#); b(16#00#); b(16#FF#);              --       LDA $FF00,X
		b(16#8D#); b(16#01#); b(16#58#);              --       STA $5801
		b(16#A9#); b(16#80#);                         --       LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              --       STA $5802 (ACK)
		chk(16#1D3B#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FD3B LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#D0#); b(16#F9#);                         --       BNE -7 -> $FD3B
		b(16#9C#); b(16#02#); b(16#58#);              --       STZ $5802
		b(16#E8#);                                    --       INX
		b(16#88#);                                    --       DEY
		b(16#D0#); b(16#E0#);                         --       BNE -32 -> $FD29
		-- Wait for the STATUS phase.  During DATAIN the CPU deliberately does
		-- nothing: if it read $5808 it would race the DMA for the same bytes.
		chk(16#1D49#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FD49 LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40 (REQ)
		b(16#F0#); b(16#F9#);                         --       BEQ -7 -> $FD49
		b(16#AD#); b(16#00#); b(16#58#);              --       LDA $5800
		b(16#29#); b(16#10#);                         --       AND #$10 (C/D)
		b(16#F0#); b(16#F2#);                         --       BEQ -14 -> $FD49
		-- STATUS @$FD57
		chk(16#1D57#);
		b(16#AD#); b(16#01#); b(16#58#);              -- $FD57 LDA $5801
		b(16#A9#); b(16#80#);                         --       LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              --       STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- $FD5F LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#D0#); b(16#F9#);                         --       BNE -7 -> $FD5F
		b(16#9C#); b(16#02#); b(16#58#);              --       STZ $5802
		-- MSGIN @$FD69
		chk(16#1D69#);
		b(16#AD#); b(16#00#); b(16#58#);              -- $FD69 LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#F0#); b(16#F9#);                         --       BEQ -7 -> $FD69
		b(16#AD#); b(16#01#); b(16#58#);              --       LDA $5801
		b(16#A9#); b(16#80#);                         --       LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              --       STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- $FD78 LDA $5800
		b(16#29#); b(16#40#);                         --       AND #$40
		b(16#D0#); b(16#F9#);                         --       BNE -7 -> $FD78
		b(16#9C#); b(16#02#); b(16#58#);              --       STZ $5802
		chk(16#1D82#);
		b(16#9C#); b(16#0B#); b(16#58#);              -- $FD82 STZ $580B (DMA off)
		b(16#EE#); b(16#1A#); b(16#20#);              --       INC $201A (DMA transfers done)
		b(16#60#);                                    --       RTS
		chk(16#1D89#);

		-- ---------------- main loop @$FDC0 --------------------------------
		org(16#1DC0#);
		b(16#EE#); b(16#00#); b(16#20#);              -- INC $2000
		b(16#20#); b(16#00#); b(16#60#);              -- JSR $6000 (exec from CD-RAM)
		b(16#EE#); b(16#01#); b(16#20#);              -- INC $2001
		-- Poll the disc position every pass, the way a game does during
		-- playback.  Unarmed the drive answers in two ticks, so B/G/F keep
		-- their timing; scenario I arms the stall and saves inside it.
		if G_DO_I then
			b(16#20#); b(16#00#); b(16#FC#);          -- JSR $FC00 (SUBQ $DD)
		else
			b(16#EA#); b(16#EA#); b(16#EA#);
		end if;
		-- scenario J / J2: unmask the interrupt under test + CLI (idempotent,
		-- once per pass).  J2 also re-arms SUBCD_EN each pass because the SUBQ
		-- routine's $5802 handshake writes would strip it - which is also why
		-- a J2 run keeps G_DO_I off.
		if G_J2 then
			b(16#20#); b(16#60#); b(16#FB#);          -- JSR $FB60 (IRQ2)
		elsif G_DO_J then
			b(16#20#); b(16#40#); b(16#FB#);          -- JSR $FB40 (IRQ1)
		else
			b(16#EA#); b(16#EA#); b(16#EA#);
		end if;
		if G_J2 then
			b(16#20#); b(16#70#); b(16#FB#);          -- JSR $FB70 (SUBCD_EN on)
		else
			b(16#EA#); b(16#EA#); b(16#EA#);
		end if;
		b(16#4C#); b(16#C0#); b(16#FD#);              -- JMP $FDC0
		chk(16#1DD5#);

		-- ---------------- READER @$FE00 -----------------------------------
		-- Issues READ6 (table @$FFE0, 6 bytes), then consumes the DATAIN
		-- stream via $5808 (auto-ack) verifying byte i == i mod 256; on the
		-- first mismatch records got/exp and sets the fail flag.  Ends when
		-- the phase turns to STATUS (CD flag high), eats STATUS+MSGIN.
		org(16#1E00#);
		-- send READ6: bus-free wait, SEL retry, 6 bytes
		b(16#AD#); b(16#00#); b(16#58#);              -- $FE00 LDA $5800
		b(16#29#); b(16#80#);                         -- AND #$80
		b(16#D0#); b(16#F9#);                         -- BNE -7 (bus free)
		b(16#A9#); b(16#81#);                         -- $FE07 LDA #$81
		b(16#8D#); b(16#01#); b(16#58#);              -- STA $5801
		b(16#8D#); b(16#00#); b(16#58#);              -- STA $5800 (SEL)
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#80#);                         -- AND #$80
		b(16#F0#); b(16#F1#);                         -- BEQ -15 -> $FE07
		b(16#A2#); b(16#E0#);                         -- LDX #$E0 (table $FFE0)
		b(16#A0#); b(16#06#);                         -- LDY #6
		-- cmd byte loop @$FE1B
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#F0#); b(16#F9#);                         -- BEQ -7
		b(16#BD#); b(16#00#); b(16#FF#);              -- LDA $FF00,X
		b(16#8D#); b(16#01#); b(16#58#);              -- STA $5801
		b(16#A9#); b(16#80#);                         -- LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              -- STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- @$FE2D LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#D0#); b(16#F9#);                         -- BNE -7
		b(16#9C#); b(16#02#); b(16#58#);              -- STZ $5802
		b(16#E8#);                                    -- INX
		b(16#88#);                                    -- DEY
		b(16#D0#); b(16#E0#);                         -- BNE -32 -> $FE1B
		b(16#9C#); b(16#10#); b(16#20#);              -- $FE3A STZ $2010 (expected=0)
		-- consume loop @$FE3D: wait REQ
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#F0#); b(16#F9#);                         -- BEQ -7 -> $FE3D
		b(16#AD#); b(16#00#); b(16#58#);              -- $FE44 LDA $5800
		b(16#29#); b(16#10#);                         -- AND #$10 (CD flag)
		b(16#D0#); b(16#1B#);                         -- BNE +27 -> $FE66 (status)
		b(16#AD#); b(16#08#); b(16#58#);              -- $FE4B LDA $5808 (data+auto-ack)
		b(16#CD#); b(16#10#); b(16#20#);              -- CMP $2010
		b(16#F0#); b(16#0D#);                         -- BEQ +13 -> $FE60 (ok)
		b(16#AE#); b(16#14#); b(16#20#);              -- $FE53 LDX $2014
		b(16#D0#); b(16#08#);                         -- BNE +8 -> $FE60
		b(16#8D#); b(16#12#); b(16#20#);              -- STA $2012 (got)
		b(16#A9#); b(16#01#);                         -- LDA #$01
		b(16#8D#); b(16#14#); b(16#20#);              -- STA $2014 (fail flag)
		-- ok @$FE60:
		b(16#EE#); b(16#10#); b(16#20#);              -- INC $2010
		b(16#4C#); b(16#3D#); b(16#FE#);              -- JMP $FE3D
		-- status @$FE66: manual ack STATUS + MSGIN, count the read, RTS
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#F0#); b(16#F9#);                         -- BEQ -7 -> $FE66
		b(16#AD#); b(16#01#); b(16#58#);              -- LDA $5801
		b(16#A9#); b(16#80#);                         -- LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              -- STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- $FE75 LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#D0#); b(16#F9#);                         -- BNE -7 -> $FE75
		b(16#9C#); b(16#02#); b(16#58#);              -- STZ $5802
		-- msgin @$FE7F
		b(16#AD#); b(16#00#); b(16#58#);              -- LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#F0#); b(16#F9#);                         -- BEQ -7 -> $FE7F
		b(16#AD#); b(16#01#); b(16#58#);              -- LDA $5801
		b(16#A9#); b(16#80#);                         -- LDA #$80
		b(16#8D#); b(16#02#); b(16#58#);              -- STA $5802
		b(16#AD#); b(16#00#); b(16#58#);              -- $FE8E LDA $5800
		b(16#29#); b(16#40#);                         -- AND #$40
		b(16#D0#); b(16#F9#);                         -- BNE -7 -> $FE8E
		b(16#9C#); b(16#02#); b(16#58#);              -- STZ $5802
		b(16#EE#); b(16#15#); b(16#20#);              -- INC $2015 (reads done)
		b(16#60#);                                    -- RTS

		-- D8 table @$FFC0: SAPSP silent, LBA mode, start=TRACK_LBA
		org(16#1FC0#);
		b(16#D8#); b(16#00#); b(16#00#);
		b(TRACK_LBA / 65536); b((TRACK_LBA / 256) mod 256); b(TRACK_LBA mod 256);
		b(0); b(0); b(0); b(0);
		-- scenario W2 READ6 #2 table @$FFCA (the 6 free bytes between the D8
		-- and D9 tables): another DIFFERENT lba (0x000A00), 1 sector
		chk(16#1FCA#);
		b(16#08#); b(16#00#); b(16#0A#); b(16#00#); b(16#01#); b(16#00#);
		chk(16#1FD0#);
		-- D9 table @$FFD0: SAPEP loop, LBA mode, end=TRACK_END
		org(16#1FD0#);
		b(16#D9#); b(G_D9_MODE); b(16#00#);
		b(TRACK_END / 65536); b((TRACK_END / 256) mod 256); b(TRACK_END mod 256);
		b(0); b(0); b(0); b(16#00#);
		-- scenario W fresh-READ6 table @$FFDA (the 6 free bytes between the D9
		-- table and the boot READ6 table): a DIFFERENT lba (0x000900), 1 sector.
		chk(16#1FDA#);
		b(16#08#); b(16#00#); b(16#09#); b(16#00#); b(16#01#); b(16#00#);
		chk(16#1FE0#);
		-- READ6 table @$FFE0: lba=0x000400, 2 sectors.
		-- Two sectors exercise pacing across a sector boundary.
		org(16#1FE0#);
		b(16#08#); b(16#00#); b(16#04#); b(16#00#); b(16#02#); b(16#00#);
		-- SUBQ table @$FFE6 (10 bytes, the gap between the two READ6 tables):
		-- $DD with an allocation length of $0A, exactly what CD_SUBQ builds at
		-- $224C/$224D in the System Card.
		chk(16#1FE6#);
		b(16#DD#); b(16#0A#); b(16#00#); b(16#00#); b(16#00#);
		b(16#00#); b(16#00#); b(16#00#); b(16#00#); b(16#00#);
		chk(16#1FF0#);
		-- READ6 table @$FFF0 for the ADPCM DMA (scenario H): lba=0x000800,
		-- G_H_SECT sectors.  The transfer has to be long enough that the freeze
		-- lands INSIDE it -- and because the drive model only pushes sector k
		-- once the DMA has consumed the ones before it, the walk suspends the
		-- stream rather than eating into that window.
		--
		-- Whatever the count, the ADPCM address wraps every 65536 bytes and 65536
		-- is a multiple of the model's 256-byte sector pattern, so the positional
		-- oracle in h_mon stays exact across the wrap.
		org(16#1FF0#);
		b(16#08#); b(16#00#); b(16#08#); b(16#00#); b(G_H_SECT mod 256); b(16#00#);

		r(16#1FFE#) := x"00"; r(16#1FFF#) := x"FF";
		-- IRQ1 (VDC) vector -> the J handler.  Present unconditionally: with
		-- G_DO_J=false interrupts stay masked and it is never taken.
		r(16#1FF8#) := x"00"; r(16#1FF9#) := x"FB";
		-- IRQ2 (CD) vector -> the J2 handler
		r(16#1FF6#) := x"20"; r(16#1FF7#) := x"FB";

		-- ---------------- scenario W: fresh-READ6 reader @$FA00 ------------
		-- Byte-for-byte copy of the $FE00 reader, retargeted at the $FFDA
		-- table (the DIFFERENT lba).  Copying the emitted bytes instead of
		-- re-assembling avoids the address-recount class of mistake entirely;
		-- the only two intra-block absolute references are patched, and the
		-- patch points are guarded so a future reader edit fails loudly here
		-- instead of silently derailing W.  Landmarks inherited by the copy:
		--   $FA58 = the "record first mismatch" store (w_mon fail hook)
		--   $FA98 = the closing INC (w_mon completion hook)
		for i in 0 to 16#9B# loop
			r(16#1A00# + i) := r(16#1E00# + i);
		end loop;
		assert r(16#1A16#) = x"A2" and r(16#1A17#) = x"E0"
			report "tb_cd W: reader LDX patch point moved" severity failure;
		r(16#1A17#) := x"DA";	-- LDX #$DA -> table @$FFDA
		assert r(16#1A63#) = x"4C" and r(16#1A64#) = x"3D" and r(16#1A65#) = x"FE"
			report "tb_cd W: reader JMP patch point moved" severity failure;
		r(16#1A65#) := x"FA";	-- JMP $FE3D -> JMP $FA3D (consume-loop head)
		-- W control block @$FAC0: one fresh READ6, then park.  The park is a
		-- tight self-jump: W's verdicts are counters, and a program that walks
		-- on would fall into filler (the BRK-vector failure mode).
		org(16#1AC0#);
		b(16#20#); b(16#00#); b(16#FA#);              -- $FAC0 JSR $FA00
		b(16#EE#); b(16#42#); b(16#20#);              -- $FAC3 INC $2042
		b(16#4C#); b(16#C6#); b(16#FA#);              -- $FAC6 JMP $FAC6 (park)
		chk(16#1AC9#);

		-- ---------------- scenario W2: READ6 #2 reader @$F900 --------------
		-- Same copy technique as the $FA00 reader, retargeted at the $FFCA
		-- table (lba 0xA00).  Landmarks: $F958 = mismatch store, $F998 = the
		-- closing INC.  (The copy shares the WRAM cells; the PC-edge counters
		-- are what tell the three readers apart.)
		for i in 0 to 16#9B# loop
			r(16#1900# + i) := r(16#1E00# + i);
		end loop;
		assert r(16#1916#) = x"A2" and r(16#1917#) = x"E0"
			report "tb_cd W2: reader LDX patch point moved" severity failure;
		r(16#1917#) := x"CA";	-- LDX #$CA -> table @$FFCA
		assert r(16#1963#) = x"4C" and r(16#1964#) = x"3D" and r(16#1965#) = x"FE"
			report "tb_cd W2: reader JMP patch point moved" severity failure;
		r(16#1965#) := x"F9";	-- JMP $FE3D -> JMP $F93D
		-- W2 control block @$FAD0: the stage-0 loader shape.  Fresh READ6,
		-- then D8+D9 through the boot's own SEND_CMD (the same path a game
		-- takes), then a ~45 ms busy-wait - the AUDIO WINDOW the third save
		-- lands in (a real transition plays music for a while; a poll on
		-- anything savestate-visible would be rewound by the load, a busy
		-- counter just runs its remainder) - then READ6 #2 and #3
		-- back-to-back, end marker, park.
		org(16#1AD0#);
		b(16#20#); b(16#00#); b(16#FA#);              -- $FAD0 JSR $FA00 (READ6 #1, lba 0x900)
		b(16#EE#); b(16#42#); b(16#20#);              -- $FAD3 INC $2042
		b(16#A2#); b(16#C0#);                         -- $FAD6 LDX #$C0
		b(16#20#); b(16#52#); b(16#FF#);              -- $FAD8 JSR $FF52 (D8 SAPSP)
		b(16#A2#); b(16#D0#);                         -- $FADB LDX #$D0
		b(16#20#); b(16#52#); b(16#FF#);              -- $FADD JSR $FF52 (D9 SAPEP loop)
		b(16#EE#); b(16#43#); b(16#20#);              -- $FAE0 INC $2043 (audio marker)
		b(16#A9#); b(16#FF#);                         -- $FAE3 LDA #$FF
		b(16#A0#); b(16#00#);                         -- $FAE5 LDY #$00      (L0)
		b(16#88#);                                    -- $FAE7 DEY           (L1)
		b(16#D0#); b(16#FD#);                         -- $FAE8 BNE L1 (-3)
		b(16#3A#);                                    -- $FAEA DEC A
		b(16#D0#); b(16#F8#);                         -- $FAEB BNE L0 (-8)
		b(16#20#); b(16#00#); b(16#F9#);              -- $FAED JSR $F900 (READ6 #2, lba 0xA00)
		b(16#EE#); b(16#44#); b(16#20#);              -- $FAF0 INC $2044
		b(16#20#); b(16#00#); b(16#FE#);              -- $FAF3 JSR $FE00 (READ6 #3, lba 0x400 x2)
		b(16#EE#); b(16#45#); b(16#20#);              -- $FAF6 INC $2045 (END marker)
		b(16#4C#); b(16#F9#); b(16#FA#);              -- $FAF9 JMP $FAF9 (park)
		chk(16#1AFC#);

		-- ---------------- scenario L: in-flight SCSI phase across save/load --
		-- Starts audio (D8+D9), then issues READ6 #1 (@$F820).  The reader
		-- consumes data, but when the drive signals STATUS, the reader enters
		-- a ~55 ms busy wait BEFORE acknowledging STATUS on $1802.  During
		-- this window the SCSI FSM is in SP_STAT_START (SP=4, REQ_Nr=0,
		-- AUD_OPEN=0, READ_ACTIVE=0, STAT_PEND=0, DOUT_PEND=0, EMPTY=1).
		-- The savestate is taken during this window.  After load, the restored
		-- CPU finishes the busy wait, ACKs STATUS, completes MSGIN, and runs
		-- READ6 #2 (@$FA00) to prove the data stream continues cleanly.
		org(16#1800#);
		-- $F800: Scenario L control block
		b(16#A2#); b(16#C0#);                         -- $F800 LDX #$C0
		b(16#20#); b(16#52#); b(16#FF#);              -- $F802 JSR $FF52 (D8 SAPSP)
		b(16#A2#); b(16#D0#);                         -- $F805 LDX #$D0
		b(16#20#); b(16#52#); b(16#FF#);              -- $F807 JSR $FF52 (D9 SAPEP loop)
		b(16#EE#); b(16#43#); b(16#20#);              -- $F80A INC $2043 (audio marker)
		b(16#20#); b(16#20#); b(16#F8#);              -- $F80D JSR $F820 (READ6 #1 with status stall)
		b(16#EE#); b(16#50#); b(16#20#);              -- $F810 INC $2050 (READ6 #1 done)
		b(16#20#); b(16#00#); b(16#FA#);              -- $F813 JSR $FA00 (READ6 #2, table @$FFDA)
		b(16#EE#); b(16#51#); b(16#20#);              -- $F816 INC $2051 (READ6 #2 done)
		b(16#4C#); b(16#19#); b(16#F8#);              -- $F819 JMP $F819 (park)
		chk(16#181C#);

		-- $F820: Scenario L extended reader (table @$FFCA: lba 0xA00, 1 sector = 2048 bytes)
		-- Handshake stalls in SP_DATAIN_START, SP_DATAIN_END, SP_STAT_START, SP_MSGIN_START
		org(16#1820#);
		b(16#AD#); -- $F820 LDA $5800
		b(16#00#);
		b(16#58#);
		b(16#29#); -- $F823 AND #$80
		b(16#80#);
		b(16#D0#); -- $F825 BNE wait_free
		b(16#F9#);
		b(16#A9#); -- $F827 LDA #$81
		b(16#81#);
		b(16#8D#); -- $F829 STA $5801
		b(16#01#);
		b(16#58#);
		b(16#8D#); -- $F82C STA $5800 (SEL)
		b(16#00#);
		b(16#58#);
		b(16#AD#); -- $F82F LDA $5800
		b(16#00#);
		b(16#58#);
		b(16#29#); -- $F832 AND #$80
		b(16#80#);
		b(16#F0#); -- $F834 BEQ do_sel
		b(16#F1#);
		b(16#A2#); -- $F836 LDX #$CA (table $FFCA)
		b(16#CA#);
		b(16#A0#); -- $F838 LDY #6
		b(16#06#);
		b(16#AD#); -- $F83A LDA $5800
		b(16#00#);
		b(16#58#);
		b(16#29#); -- $F83D AND #$40
		b(16#40#);
		b(16#F0#); -- $F83F BEQ cmd_loop (wait REQ)
		b(16#F9#);
		b(16#BD#); -- $F841 LDA $FF00,X
		b(16#00#);
		b(16#FF#);
		b(16#8D#); -- $F844 STA $5801
		b(16#01#);
		b(16#58#);
		b(16#A9#); -- $F847 LDA #$80
		b(16#80#);
		b(16#8D#); -- $F849 STA $5802 (ACK=1)
		b(16#02#);
		b(16#58#);
		b(16#20#); -- $F84C JSR ack_byte_wait_low
		b(16#EF#);
		b(16#F8#);
		b(16#E8#); -- $F84F INX
		b(16#88#); -- $F850 DEY
		b(16#D0#); -- $F851 BNE cmd_loop
		b(16#E7#);
		b(16#9C#); -- $F853 STZ $2010 (count lo)
		b(16#10#);
		b(16#20#);
		b(16#9C#); -- $F856 STZ $2011 (count hi)
		b(16#11#);
		b(16#20#);
		b(16#AD#); -- $F859 LDA $5800
		b(16#00#);
		b(16#58#);
		b(16#29#); -- $F85C AND #$40
		b(16#40#);
		b(16#F0#); -- $F85E BEQ consume_loop (wait REQ)
		b(16#F9#);
		b(16#AD#); -- $F860 LDA $5800
		b(16#00#);
		b(16#58#);
		b(16#29#); -- $F863 AND #$10 (CD flag)
		b(16#10#);
		b(16#D0#); -- $F865 BNE status_phase
		b(16#57#);
		b(16#AD#); -- $F867 LDA $2011 (count hi)
		b(16#11#);
		b(16#20#);
		b(16#D0#); -- $F86A BNE check_end_phase
		b(16#0C#);
		b(16#AD#); -- $F86C LDA $2010 (count lo)
		b(16#10#);
		b(16#20#);
		b(16#C9#); -- $F86F CMP #$C8 (byte 200)
		b(16#C8#);
		b(16#D0#); -- $F871 BNE check_end_phase
		b(16#05#);
		b(16#20#); -- $F873 JSR busy_wait (~40 ms in SP_DATAIN_START)
		b(16#D5#);
		b(16#F8#);
		b(16#80#); -- $F876 BRA read_auto_ack
		b(16#2B#);
		b(16#AD#); -- $F878 LDA $2011 (count hi)
		b(16#11#);
		b(16#20#);
		b(16#C9#); -- $F87B CMP #$01
		b(16#01#);
		b(16#D0#); -- $F87D BNE read_auto_ack
		b(16#24#);
		b(16#AD#); -- $F87F LDA $2010 (count lo)
		b(16#10#);
		b(16#20#);
		b(16#C9#); -- $F882 CMP #$F4 (byte 500)
		b(16#F4#);
		b(16#D0#); -- $F884 BNE read_auto_ack
		b(16#1D#);
		b(16#AD#); -- $F886 LDA $5801 (manual data read, no auto-ack)
		b(16#01#);
		b(16#58#);
		b(16#CD#); -- $F889 CMP $2010
		b(16#10#);
		b(16#20#);
		b(16#F0#); -- $F88C BEQ end_ack_set
		b(16#08#);
		b(16#8D#); -- $F88E STA $2012 (got)
		b(16#12#);
		b(16#20#);
		b(16#A9#); -- $F891 LDA #$01
		b(16#01#);
		b(16#8D#); -- $F893 STA $2014 (fail flag)
		b(16#14#);
		b(16#20#);
		b(16#A9#); -- $F896 LDA #$80
		b(16#80#);
		b(16#8D#); -- $F898 STA $5802 (ACK=1 -> SCSI in SP_DATAIN_END)
		b(16#02#);
		b(16#58#);
		b(16#20#); -- $F89B JSR busy_wait (~40 ms in SP_DATAIN_END)
		b(16#D5#);
		b(16#F8#);
		b(16#20#); -- $F89E JSR ack_byte_wait_low
		b(16#EF#);
		b(16#F8#);
		b(16#80#); -- $F8A1 BRA inc_counter
		b(16#10#);
		b(16#AD#); -- $F8A3 LDA $5808 (data+auto-ack)
		b(16#08#);
		b(16#58#);
		b(16#CD#); -- $F8A6 CMP $2010
		b(16#10#);
		b(16#20#);
		b(16#F0#); -- $F8A9 BEQ inc_counter
		b(16#08#);
		b(16#8D#); -- $F8AB STA $2012 (got)
		b(16#12#);
		b(16#20#);
		b(16#A9#); -- $F8AE LDA #$01
		b(16#01#);
		b(16#8D#); -- $F8B0 STA $2014 (fail flag)
		b(16#14#);
		b(16#20#);
		b(16#EE#); -- $F8B3 INC $2010
		b(16#10#);
		b(16#20#);
		b(16#D0#); -- $F8B6 BNE no_hi_inc
		b(16#03#);
		b(16#EE#); -- $F8B8 INC $2011
		b(16#11#);
		b(16#20#);
		b(16#4C#); -- $F8BB JMP consume_loop
		b(16#59#);
		b(16#F8#);
		b(16#20#); -- $F8BE JSR busy_wait (~40 ms in SP_STAT_START)
		b(16#D5#);
		b(16#F8#);
		b(16#20#); -- $F8C1 JSR ack_byte (read STATUS and pulse ACK)
		b(16#E0#);
		b(16#F8#);
		b(16#AD#); -- $F8C4 LDA $5800
		b(16#00#);
		b(16#58#);
		b(16#29#); -- $F8C7 AND #$40
		b(16#40#);
		b(16#F0#); -- $F8C9 BEQ wait_msgin_req (wait REQ for MSGIN)
		b(16#F9#);
		b(16#20#); -- $F8CB JSR busy_wait (~40 ms in SP_MSGIN_START)
		b(16#D5#);
		b(16#F8#);
		b(16#20#); -- $F8CE JSR ack_byte (read MESSAGE and pulse ACK)
		b(16#E0#);
		b(16#F8#);
		b(16#EE#); -- $F8D1 INC $2015 (reads done marker)
		b(16#15#);
		b(16#20#);
		b(16#60#); -- $F8D4 RTS
		b(16#A9#); -- $F8D5 LDA #$A0
		b(16#A0#);
		b(16#A0#); -- $F8D7 LDY #$00
		b(16#00#);
		b(16#88#); -- $F8D9 DEY
		b(16#D0#); -- $F8DA BNE bw_loop_y
		b(16#FD#);
		b(16#3A#); -- $F8DC DEC A
		b(16#D0#); -- $F8DD BNE bw_loop
		b(16#F8#);
		b(16#60#); -- $F8DF RTS
		b(16#AD#); -- $F8E0 LDA $5800
		b(16#00#);
		b(16#58#);
		b(16#29#); -- $F8E3 AND #$40
		b(16#40#);
		b(16#F0#); -- $F8E5 BEQ ack_byte_wait_req
		b(16#F9#);
		b(16#AD#); -- $F8E7 LDA $5801 (read byte)
		b(16#01#);
		b(16#58#);
		b(16#A9#); -- $F8EA LDA #$80
		b(16#80#);
		b(16#8D#); -- $F8EC STA $5802 (ACK=1)
		b(16#02#);
		b(16#58#);
		b(16#AD#); -- $F8EF LDA $5800
		b(16#00#);
		b(16#58#);
		b(16#29#); -- $F8F2 AND #$40
		b(16#40#);
		b(16#D0#); -- $F8F4 BNE ack_byte_wait_low (wait REQ=0)
		b(16#F9#);
		b(16#9C#); -- $F8F6 STZ $5802 (ACK=0)
		b(16#02#);
		b(16#58#);
		b(16#60#); -- $F8F9 RTS
		chk(16#18FA#);
		return r;
	end function;
	constant ROM : rom_t := rom_init;

	type mem_t is array (0 to 4*SLOT_WORDS + 64) of std_logic_vector(63 downto 0);

	-- CDDA sample log written by a monitor process, read by the scenario
	type slog_t is array (0 to 262143) of integer;
	type stime_t is array (0 to 262143) of time;
	signal slog_count : integer := 0;

	signal hps_d8_count : integer := 0;

	-- Scenario K: count SAPEP-INTERRUPT completions before and after load.
	signal hps_intstat_count : integer := 0;

	-- scenario I ($DD across a save/load)
	-- Drive latency for a $DD, in model ticks, set by the scenario.  Lowering it
	-- also shortens a stall already in flight, so releasing is immediate.
	signal i_lat       : integer := 2;
	-- Scenario I2: hold $DD until SLEEP, independent of VBLANK timing.
	signal i_freeze_ans : std_logic := '0';
	signal hps_dd_pend : std_logic := '0';	-- $DD accepted, answer not yet sent
	signal hps_dd_done : integer := 0;		-- $DD answers delivered
	signal i_cpu_done  : integer := 0;		-- $DD replies the CPU fully consumed
	-- Count $DD replies delivered during SLEEP. SLEEP can briefly deassert
	-- near walk completion, so edge-sampled baselines can miss the reply.
	signal i_ans_frozen : integer := 0;

	-- instruction-boundary invariant (see st0_mon)
	signal ss_en_leak    : integer := 0;	-- CPU enabled while SLEEP asserted
	signal ss_state_bad  : integer := 0;	-- STATE moved after the freeze
	signal ss_blob_bad   : integer := 0;	-- a SAVED blob carries STATE /= 0

	-- Input already in flight may finish during SLEEP. The model pauses new
	-- audio sectors until the consumer resumes; a sector in flight can still
	-- overflow a nearly full FIFO, and the monitor reports it.
	signal frz_cdda_drop : integer := 0;	-- CDDA samples lost while frozen
	signal frz_subc_drop : integer := 0;	-- subcode bytes lost while frozen
	signal frz_cdda_hi   : integer := 0;	-- peak CDDA FIFO backlog entering a freeze

	-- Sample-tape index at the last CDDA flush. Anchor B/G to a flush:
	-- a looping track eventually reaches any value, even after a bad resume.
	signal flush_mark : integer := -1;
	-- Silent D8/D9 flushes at RP_ISSUE and RP_ISSUE_D9. Each load's
	-- resume anchor is its second flush; the wrap anchor is its fourth.
	type flushlog_t is array (0 to 7) of integer;
	signal flush_log : flushlog_t := (others => -1);
	-- RP_CONS identifies the load owning each flush. A previous load's late
	-- wrap pair can fall after this load's mark, so ordinal alone is unsafe.
	signal flush_cons : flushlog_t := (others => -1);
	signal flush_n   : integer := 0;
	-- Last fall of the core's SLEEP (CPU, video and DAC resume together).
	signal rel_t     : time := 0 ns;

	signal c_fail : std_logic := '0';	-- scenario C: resumed stream misaligned
	signal e_fail : std_logic := '0';	-- scenario E: DTR/CD_DATA_CNT bookkeeping

	-- CD_SL records the post-fader value: FIFO_Q * CDDA_FADE_VOL >> 10.
	-- Full scale is 1023, not 1024 (upstream cd.vhd, about -0.008 dB).
	-- Apply the same scaling to B/G's expected frame index.
	--
	-- Use it for the STARTING value only, never to predict the run that
	-- follows: the tape logs CD_SL changes, and the fader makes consecutive
	-- frames repeat a value now and then (frames 0 and 1 both arrive as 0), so
	-- the repeat leaves no entry.  What the tape holds is therefore the
	-- distinct values in order - consecutive integers - and a resumed run is
	-- checked as "+1 per entry" from wherever it started.
	function dac_of(frame : integer) return integer is
	begin
		return (frame * 1023) / 1024;
	end function;

	-- scenario W (push-status soak + fresh READ6)
	function b2sl(b : boolean) return std_logic is
	begin
		if b then return '1'; else return '0'; end if;
	end function;
	-- Hold W's final READ6 STATUS until the stimulus releases it; otherwise
	-- it arrives ~1.5 ms into read #1 and closes the intended save window.
	signal w_hold_stat  : std_logic := b2sl(G_DO_W or G_DO_W2 or G_DO_W3);
	-- Count ss_load edges because its 3-clock pulse is shorter than the
	-- model's 50 us poll. In simplified push mode a load idles the drive:
	-- the pre-load transaction is assumed complete. Real-HPS mode retains
	-- drive state across loads.
	signal tb_load_count : integer := 0;
	signal w_reads_done : integer := 0;	-- $FE00 reader completions (PC edge $FE98)
	signal w_fresh_done : integer := 0;	-- $FA00 reader completions (PC edge $FA98)
	signal w_fail       : std_logic := '0';	-- fresh-READ6 data mismatch (PC $FA58/$F958)
	signal w_read6_cnt  : integer := 0;	-- fresh-LBA READ6 commands the model saw
	-- scenario W2 program landmarks (PC-edge counters, w_mon)
	signal w2_audio     : integer := 0;	-- $FAE0: D8+D9 both closed, CDDA commanded
	signal w2_read2     : integer := 0;	-- $FAF0: READ6 #2 conversation closed
	signal w2_end       : integer := 0;	-- $FAF6: READ6 #3 closed - the END marker
	-- scenario W3 / real-HPS model observables
	signal hps_slot     : std_logic := '0';	-- the deferred slot holds an undelivered status
	signal w3_lost      : integer := 0;	-- PendStatus OVERWRITES: statuses lost forever
	-- AUD_OPEN defers saves until status arrives. W3 checks state latched
	-- at freeze (w_probe), not state at the save request.
	signal w_frz_slot    : std_logic := '0';
	signal w_frz_audopen : std_logic := '0';

	-- scenario H (ADPCM DMA) verdict signals, raised by h_mon
	signal h_arm    : std_logic := '0';	-- watch the ADPCM RAM write port
	signal h_clr    : std_logic := '0';	-- forget everything seen so far
	signal h_nib    : integer := 0;		-- nibble writes seen (2 per DMA byte)
	signal h_cov    : integer := 0;		-- DISTINCT nibble addresses written
	signal h_after  : integer := 0;		-- nibble writes seen after the load
	signal h_mark   : std_logic := '0';	-- set by the stim right after the load
	signal h_fail   : std_logic := '0';	-- a byte landed with the wrong value
	signal h_bad_a  : integer := -1;
	signal h_bad_v  : integer := -1;
	signal h_bad_e  : integer := -1;
	signal h_jumps  : integer := 0;		-- address discontinuities (1 = the load)
	signal h_ract_frz : std_logic := '0';	-- READ still active at the last freeze
	signal sp_frz     : std_logic_vector(3 downto 0) := x"0";	-- SP phase at last freeze
	-- registered command mailbox (the real TurboGrafx16.sv latches CD_COMM on
	-- the COMM_SEND edge; a bare pulse would be invisible inside the model's
	-- streaming wait loops)
	signal mbox_comm  : std_logic_vector(95 downto 0) := (others => '0');
	signal mbox_count : integer := 0;
	signal mbox_fast  : std_logic := '0';	-- its fast-seek bit (cd_in[111] in TurboGrafx16.sv)

	-- Count one-cycle CD_DATA_END pulses so the polling drive model can
	-- observe consumption between its wakeups.
	signal cd_end_count : integer := 0;

begin

	clk <= not clk after CLK_PERIOD / 2;

	DUT : entity work.pce_top
	generic map ( SS_CDRAM_BYTES => CDRAM_SIM, LITE => 0 )
	port map (
		RESET       => reset,
		COLD_RESET  => '0',
		CLK         => clk,

		ROM_RD      => rom_rd,
		ROM_RDY     => rom_rdy,
		ROM_A       => rom_a,
		ROM_DO      => rom_do,
		ROM_SZ      => x"020",
		ROM_POP     => '0',
		ROM_CLKEN   => rom_clken,

		BRM_A       => brm_a,
		BRM_DI      => brm_di,
		BRM_DO      => brm_do,
		BRM_WE      => brm_we,

		GG_EN       => '0',
		GG_CODE     => (others => '0'),
		GG_RESET    => '0',
		GG_AVAIL    => open,

		SP64        => '0',
		SGX         => '0',

		JOY_OUT     => open,
		JOY_IN      => "1111",

		CD_EN       => '1',
		CD_RAM_A    => cdram_a,
		CD_RAM_DO   => cdram_do,
		CD_RAM_DI   => cdram_di,
		CD_RAM_RD   => cdram_rd,
		CD_RAM_WR   => cdram_wr,
		AC_EN       => '0',

		CD_STAT     => cd_stat,
		CD_MSG      => x"00",
		CD_STAT_GET => cd_stat_get,
		CD_COMM     => cd_comm,
		CD_COMM_SEND=> cd_comm_send,
		CD_DOUT_REQ => '0',
		CD_DOUT     => open,
		CD_DOUT_SEND=> open,
		CD_REGION   => '0',
		CD_RESET    => open,
		CD_FAST_SEEK=> cd_fast_seek,
		CD_DATA     => cd_data,
		CD_DATA_WR  => cd_data_wr,
		CD_AUDIO_WR => cd_audio_wr,
		CD_SUBCD_WR => cd_subcd_wr,
		CD_DATA_END => cd_data_end,
		CD_DM       => '0',

		CDDA_SL     => cdda_sl,
		CDDA_SR     => cdda_sr,
		ADPCM_S     => open,
		PSG_SL      => open,
		PSG_SR      => open,

		BG_EN       => '1',
		SPR_EN      => '1',
		GRID_EN     => "00",
		CPU_PAUSE_EN=> '0',

		SS_SAVE     => ss_save,
		SS_LOAD     => ss_load,
		SS_SLOT     => ss_slot,
		SS_BUSY     => ss_busy,
		SS_FAILED   => open,
		SS_ACTIVE_SLOT => open,
		SS_ACTIVE_LOAD => open,
		SS_SLEEP    => ss_sleep,
		SS_DDR_DIN  => ddr_din,
		SS_DDR_DOUT => ddr_dout,
		SS_DDR_ADDR => ddr_adr,
		SS_DDR_RNW  => ddr_rnw,
		SS_DDR_ENA  => ddr_ena,
		SS_DDR_BE   => ddr_be,
		SS_DDR_DONE => ddr_done,
		SSE_Din     => open,
		SSE_Adr     => open,
		SSE_wren    => open,
		SSE_rst     => open,
		SSE_load    => open,
		SSE_Dout    => (others => '0'),

		BORDER_EN   => '0',
		ReducedVBL  => '1',
		VIDEO_R     => open,
		VIDEO_G     => open,
		VIDEO_B     => open,
		VIDEO_BW    => open,
		VIDEO_CE    => open,
		VIDEO_CE_FS => open,
		VIDEO_VS    => open,
		VIDEO_HS    => open,
		VIDEO_HBL   => open,
		VIDEO_VBL   => open
	);

	-- ------------------------------------------------------------------
	-- Shared ROM / CD-RAM port (TurboGrafx16.sv structure): one data
	-- register serving both sources, rom_rd priority, ce_rom pacing.
	-- ------------------------------------------------------------------
	mem_port : process(clk)
		type cdram_t is array (0 to CDRAM_SIM-1) of std_logic_vector(7 downto 0);
		variable cdram  : cdram_t := (others => x"00");
		variable pend   : integer := 0;
		variable pa     : integer := 0;
		variable p_rom  : boolean := false;
	begin
		if rising_edge(clk) then
			if rom_clken = '1' then
				if cdram_wr = '1' then
					cdram(to_integer(unsigned(cdram_a(17 downto 0))) mod CDRAM_SIM) := cdram_do;
				end if;
				if rom_rd = '1' or cdram_rd = '1' then
					p_rom := rom_rd = '1';			-- raddr priority (the steal)
					if p_rom then
						pa := to_integer(unsigned(rom_a(12 downto 0)));
					else
						pa := to_integer(unsigned(cdram_a(17 downto 0))) mod CDRAM_SIM;
					end if;
					pend := 3;
					rom_rdy <= '0';
				end if;
			elsif pend > 0 then
				pend := pend - 1;
				if pend = 0 then
					if p_rom then
						rom_do   <= ROM(pa);
						cdram_di <= ROM(pa);		-- shared data register
					else
						rom_do   <= cdram(pa);
						cdram_di <= cdram(pa);
					end if;
					rom_rdy <= '1';
				end if;
			end if;
		end if;
	end process;

	mbox : process(clk)
		variable prev : std_logic := '0';
	begin
		if rising_edge(clk) then
			if cd_comm_send = '1' and prev = '0' then
				mbox_comm  <= cd_comm;
				mbox_count <= mbox_count + 1;
				mbox_fast  <= cd_fast_seek;
			end if;
			prev := cd_comm_send;
		end if;
	end process;

	-- latch the sector-consumed pulses for the polling drive model
	end_mon : process(clk)
	begin
		if rising_edge(clk) then
			if cd_data_end = '1' then
				cd_end_count <= cd_end_count + 1;
			end if;
		end if;
	end process;

	-- latch ss_load pulses for the polling drive model (push mode)
	load_mon : process(clk)
		variable prev : std_logic := '0';
	begin
		if rising_edge(clk) then
			if ss_load = '1' and prev = '0' then
				tb_load_count <= tb_load_count + 1;
			end if;
			prev := ss_load;
		end if;
	end process;

	-- ------------------------------------------------------------------
	-- HPS drive model (verified pcecdd.cpp semantics, time-scaled:
	-- mailbox poll 50 us; CDDA sector cadence follows CEGen)
	-- ------------------------------------------------------------------
	hps : process
		type hstate_t is ( H_IDLE, H_PLAY, H_PAUSE, H_READ );
		variable state     : hstate_t := H_IDLE;
		-- head position.  In real-HPS mode it is PERSISTENT (seek distances are
		-- measured from it) and starts parked at the FMV area the scene begins
		-- in, so the boot read does not pay a full distant seek.
		function head0 return integer is
		begin
			if G_REAL_HPS then return 16#400#; else return 0; end if;
		end function;
		variable lba       : integer := head0;
		variable pstart    : integer := 0;
		variable pend_lba  : integer := 100000;
		variable int_pend  : boolean := false;	-- pcecdd: SAPEP-INTERRUPT completion armed
		variable mode      : integer := 0;
		variable comm      : std_logic_vector(95 downto 0);
		variable b0, b1, b9 : integer;
		variable nlba      : integer;
		variable pend_stat : boolean := false;
		variable stat_wait : integer := 0;
		-- push-status mode (G_PUSH_STATUS): the final READ6 STATUS is owed from
		-- the moment the last byte is PUSHED, and strobes rd_wait ticks later
		-- (or when scenario W releases w_hold_stat)
		variable rd_stat   : boolean := false;
		variable rd_wait   : integer := 0;
		variable seen_load : integer := 0;
		-- real-HPS mode (G_REAL_HPS): the single deferred-status slot, the
		-- seek latency (in 13.33 ms ticks) and the per-sector cadence gates
		variable rh_has      : boolean := false;	-- {has_status} - ONE slot
		variable rh_lat      : integer := 0;		-- seek latency, 13.33 ms ticks
		variable rh_can      : boolean := true;		-- can_read_next (per-sector ack)
		variable rh_stat_cnt : integer := 267;		-- 13.33 ms / 50 us model ticks
		variable rh_sect_cnt : integer := 0;		-- 16 ms / 50 us; 0 = may push now
		variable skms        : integer := 0;
		variable seen      : integer := 0;
		variable rdcnt     : integer := 0;
		variable seen_end  : integer := 0;
		-- Consumption-gated mode sends sector k after k*2048 bytes are consumed,
		-- and final STATUS after the whole transfer. CD_DATA_END alone is
		-- insufficient: FIFO empty can also occur partway through a sector.
		alias rd_cons_hw is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		alias rd_skip_hw is << signal .tb_cd.DUT.CD.SCSI.READ_SKIP : unsigned(11 downto 0) >>;
		-- every DATAIN handshake, whatever the command: how the $DD answer is
		-- paced on consumption rather than on the push
		alias din_cnt_hw is << signal .tb_cd.DUT.CD.SCSI.DATAIN_CNT : unsigned(15 downto 0) >>;
		variable pushed    : integer := 0;	-- bytes handed to the FIFO this command
		variable total     : integer := 0;	-- bytes the command will deliver
		-- Rebase READ_CONSUMED at first push: replay preserves its saved count.
		-- Include READ_SKIP drains, which remove bytes without advancing
		-- READ_CONSUMED, to avoid overflowing the 4096-byte FIFO on long reads.
		variable cons_base : integer := 0;
		variable skip0     : integer := 0;
		variable removed   : integer := 0;
		variable dd_pend   : boolean := false;	-- $DD accepted, answer owed
		variable dd_wait   : integer := 0;		-- ticks left before answering it
		-- Gate $DD STATUS on consumption of all 10 bytes. SP_FREE prioritizes
		-- STAT_PEND over FIFO data; early status would strand unread bytes.
		variable dd_stat   : boolean := false;	-- data sent, STATUS owed
		variable dd_target : integer := 0;		-- DATAIN_CNT that means "all 10 taken"
		variable frame_val : integer;
		variable t_next    : time;
		-- Match CEGen's sample rate; mailbox polling does not pace CDDA.
		constant AUDIO_SECTOR_TIME : time := CLK_PERIOD * 429545 * 588 / 441;
		variable audio_next : time := 0 ns;
		variable audio_burst : integer range 1 to 2 := 2;
		-- latency of the last D8: pcecdd.cpp zeroes latency and audiodelay
		-- for a D8 with the fast-seek bit
		variable play_lat : time := 0 ns;
		variable fast     : std_logic := '0';
	begin
		wait until reset = '0';
		t_next := now + 50 us;
		loop
			-- react quickly to commands; otherwise advance to the next tick
			wait until mbox_count /= seen or cd_end_count /= seen_end
			              or now >= t_next for 50 us;

			if mbox_count /= seen then
				seen := mbox_count;
				if true then
				wait for 50 * CLK_PERIOD;			-- ~mailbox pickup delay
				comm := mbox_comm;
				fast := mbox_fast;
				b0 := to_integer(unsigned(comm(7 downto 0)));
				b1 := to_integer(unsigned(comm(15 downto 8)));
				b9 := to_integer(unsigned(comm(79 downto 72)));
				-- pcecdd.cpp CommandExec(): "this->int_pend = false;" runs for
				-- EVERY command before the dispatch - only a SAPEP in INTERRUPT
				-- mode re-arms it.  The armed end-of-playback status is how a
				-- game (Rondo's FMV) learns the track finished.
				int_pend := false;
				if b0 = 16#08# then
					nlba := (to_integer(unsigned(comm(15 downto 8))) mod 32) * 65536
					      + to_integer(unsigned(comm(23 downto 16))) * 256
					      + to_integer(unsigned(comm(31 downto 24)));
					rdcnt := to_integer(unsigned(comm(39 downto 32)));
					if rdcnt = 0 then rdcnt := 256; end if;
					if G_REAL_HPS then
						-- seek latency from the PERSISTENT head (command overwrite
						-- is implicit: state/count/latency all reassigned; a
						-- pended status of an abandoned op stays in the slot)
						if nlba = lba and rdcnt = 120 then
							skms := 0;	-- the HuVideo chained-read path
						elsif abs(nlba - lba) <= 3 then
							skms := 33;
						elsif abs(nlba - lba) < 7 then
							skms := 250;
						else
							skms := G_SEEK_MS;
						end if;
						rh_lat := skms / 13;
						rh_can := true;	-- can_read_next primed at READ6 issue
						report "HPS(real): READ6 seek " & integer'image(skms) & " ms = "
							& integer'image(rh_lat) & " ticks, head "
							& integer'image(lba) & " -> " & integer'image(nlba);
					end if;
					lba := nlba;
					pushed := 0;
					total  := rdcnt * 2048;
					state := H_READ;
					-- The consumption-gated model has no seek latency: start
					-- its first sector relative to this command, not an unrelated
					-- polling tick left over from the previous transaction.
					-- Real-HPS and push-status modes retain their timed serving.
					if not G_REAL_HPS and not G_PUSH_STATUS then
						t_next := now;
					end if;
					-- a new READ6 supersedes any status still owed by the previous
					-- one (push mode): on hardware that status was delivered - and
					-- consumed - in the pre-load timeline; the re-issued command
					-- generates its own
					rd_stat := false;
					report "HPS: READ6 lba=" & integer'image(nlba) & " cnt=" & integer'image(rdcnt);
				elsif b0 = 16#D8# then
					if b9 / 64 = 0 then
						nlba := to_integer(unsigned(comm(31 downto 24))) * 65536
						      + to_integer(unsigned(comm(39 downto 32))) * 256
						      + to_integer(unsigned(comm(47 downto 40)));
					else
						nlba := TRACK_LBA;
					end if;
					if G_REAL_HPS then
						-- SAPSP: seek latency minus the audiodelay (floor 0), or
						-- the W3 positioning override; the status is PENDED into
						-- the single slot - a pend on an occupied slot silently
						-- LOSES the previous status (the real PendStatus)
						if abs(nlba - lba) <= 3 then
							skms := 33;
						elsif abs(nlba - lba) < 7 then
							skms := 250;
						else
							skms := G_SEEK_MS;
						end if;
						skms := skms - G_AUDIODELAY_MS;
						if skms < 0 then skms := 0; end if;
						if G_W3_D8LAT_MS > 0 then skms := G_W3_D8LAT_MS; end if;
						rh_lat := skms / 13;
						if rh_has then
							w3_lost <= w3_lost + 1;
							report "HPS(real): PendStatus OVERWRITE - the previous "
								& "status is LOST (total lost "
								& integer'image(w3_lost + 1) & ") t=" & time'image(now)
								severity warning;
						end if;
						rh_has := true;
						hps_slot <= '1';
						report "HPS(real): D8 seek->latency " & integer'image(skms)
							& " ms = " & integer'image(rh_lat)
							& " ticks, status PENDED (frozen while seeking)";
					else
						pend_stat := true; stat_wait := 2;
					end if;
					lba := nlba; pstart := nlba;
					pend_lba := 100000;				-- D8 resets the end (pcecdd!)
					mode := b1;
					if fast = '1' then play_lat := 0 ns; else play_lat := G_PLAY_LAT_MS * 1 ms; end if;
					audio_next := now + play_lat;
					audio_burst := 2; -- Main preloads two sectors after D8.
					if b1 = 0 then state := H_PAUSE; else state := H_PLAY; end if;
					hps_d8_count <= hps_d8_count + 1;
					report "HPS: D8 lba=" & integer'image(nlba) & " mode=" & integer'image(b1)
						& " fast=" & std_logic'image(fast);
				elsif b0 = 16#D9# then
					if b9 / 64 = 0 then
						nlba := to_integer(unsigned(comm(31 downto 24))) * 65536
						      + to_integer(unsigned(comm(39 downto 32))) * 256
						      + to_integer(unsigned(comm(47 downto 40)));
					else
						nlba := pend_lba;
					end if;
					pend_lba := nlba; mode := b1;
					if state /= H_PLAY then audio_next := now + play_lat; end if;
					if b1 = 0 then state := H_IDLE; else state := H_PLAY; end if;
					if b1 /= 2 then
						pend_stat := true; stat_wait := 0;
					else
						int_pend := true;	-- pcecdd: status deferred to playback end
					end if;
					report "HPS: D9 end=" & integer'image(nlba) & " mode=" & integer'image(b1);
				elsif b0 = 16#DA# then
					state := H_PAUSE;
					pend_stat := true; stat_wait := 0;
					report "HPS: DA pause";
				elsif b0 = 16#DD# then
					-- READ SUBQ.  A real drive answers this WITHOUT disturbing
					-- playback, so `state` is deliberately left alone: the CDDA
					-- stream has to keep flowing underneath it.
					dd_pend := true;
					dd_wait := i_lat;
					hps_dd_pend <= '1';
					report "HPS: DD subq, latency " & integer'image(dd_wait) & " ticks";
				end if;
				end if;	-- rising edge
			end if;

			-- CD_DATA_END only wakes the model promptly; whether to stream the
			-- next sector is decided from READ_CONSUMED below.
			if cd_end_count /= seen_end then
				seen_end := cd_end_count;
			end if;
			-- Simplified push mode assumes the pre-load transaction completed,
			-- so discard its pending READ and status. Real-HPS mode retains
			-- head, state, count, latency and status slot across loads.
			if G_PUSH_STATUS and not G_REAL_HPS and seen_load /= tb_load_count then
				seen_load := tb_load_count;
				if state = H_READ then
					state := H_IDLE;
					report "HPS: load - in-flight READ of the dead timeline dropped";
				end if;
				rd_stat   := false;
				pend_stat := false;
			end if;
			if now >= t_next then
				t_next := now + 50 us;
				if G_REAL_HPS then
					-- 13.33 ms quantization tick: the seek latency counts down
					-- here, and the deferred slot is delivered ONLY here, only
					-- once the latency has expired (and w_hold_stat, the TB
					-- positioning knob, is released)
					if rh_stat_cnt > 0 then
						rh_stat_cnt := rh_stat_cnt - 1;
					else
						rh_stat_cnt := 267;
						if rh_lat > 0 then
							rh_lat := rh_lat - 1;
						elsif rh_has and w_hold_stat = '0' then
							rh_has := false;
							hps_slot <= '0';
							cd_stat <= x"00";
							wait until rising_edge(clk);
							cd_stat_get <= '1';
							wait until rising_edge(clk);
							cd_stat_get <= '0';
							report "HPS(real): deferred status delivered t=" & time'image(now);
						end if;
					end if;
					-- 16 ms sector cadence countdown (push happens below)
					if rh_sect_cnt > 0 then
						rh_sect_cnt := rh_sect_cnt - 1;
					end if;
				end if;
				if pend_stat then
					if stat_wait > 0 then
						stat_wait := stat_wait - 1;
					else
						cd_stat <= x"00";
						wait until rising_edge(clk);
						cd_stat_get <= '1';
						wait until rising_edge(clk);
						cd_stat_get <= '0';
						pend_stat := false;
					end if;
				end if;
				-- $DD answer: 10 bytes of DATAIN, then the STATUS.  The RTL needs
				-- no help to raise the phase - SP_FREE goes to SP_DATAIN as soon
				-- as the SCSI FIFO is non-empty (SCSI.vhd:512-520).
				if dd_pend then
					-- lowering i_lat also shortens a stall already in flight, so a
					-- scenario never has to sit through the rest of one it armed
					if dd_wait > i_lat then
						dd_wait := i_lat;
					end if;
					if i_freeze_ans = '1' and ss_sleep = '0' then
						null;	-- hold it until the machine is frozen (scenario I2)
					elsif dd_wait > 0 then
						dd_wait := dd_wait - 1;
					else
						for i in 0 to 9 loop
							cd_data <= std_logic_vector(to_unsigned(16#A0# + i, 8));
							cd_data_wr <= '1'; wait for 2*CLK_PERIOD;
							cd_data_wr <= '0'; wait for 2*CLK_PERIOD;
						end loop;
						dd_pend := false;
						hps_dd_pend <= '0';
						hps_dd_done <= hps_dd_done + 1;
						if ss_sleep = '1' then
							i_ans_frozen <= i_ans_frozen + 1;
						end if;
						-- STATUS only once the program has taken all ten bytes
						dd_stat   := true;
						dd_target := to_integer(din_cnt_hw) + 10;
					end if;
				end if;
				if dd_stat and to_integer(din_cnt_hw) >= dd_target then
					dd_stat := false;
					pend_stat := true; stat_wait := 1;
				end if;

				-- rebase at each command's first sector: whatever READ_CONSUMED and
				-- READ_SKIP happen to be at that moment is this command's zero
				-- (READ_CONSUMED deliberately does NOT restart after a replayed
				-- READ6, and the drain removes bytes without ever touching it -
				-- both are counted in `removed`)
				if state = H_READ and rdcnt > 0 and pushed = 0 then
					cons_base := to_integer(rd_cons_hw);
					skip0     := to_integer(rd_skip_hw);
				end if;
				removed := (to_integer(rd_cons_hw) - cons_base)
				         + (skip0 - to_integer(rd_skip_hw));
				if G_REAL_HPS then
					-- real cd.cpp serving: one 2048-byte sector per 16 ms tick,
					-- only when the seek latency has expired and the core has
					-- consumed the previous sector (can_read_next).  The re-arm
					-- uses the rebased removed-vs-pushed accounting: the replay
					-- drain consumes bytes without touching READ_CONSUMED and
					-- must count as consumption here too.
					if not rh_can and removed >= pushed then
						rh_can := true;
					end if;
					if state = H_READ and rdcnt > 0 and rh_lat = 0
					   and rh_can and rh_sect_cnt = 0 then
						rh_sect_cnt := 320;	-- next sector no sooner than 16 ms
						pushed := pushed + 2048;
						for i in 0 to 2047 loop
							cd_data <= std_logic_vector(to_unsigned(i mod 256, 8));
							cd_data_wr <= '1'; wait for 2*CLK_PERIOD;
							cd_data_wr <= '0'; wait for 2*CLK_PERIOD;
						end loop;
						lba := lba + 1;
						rdcnt := rdcnt - 1;
						rh_can := false;
						if rdcnt = 0 then
							-- READ6 completion: PendStatus into the SINGLE slot
							if rh_has then
								w3_lost <= w3_lost + 1;
								report "HPS(real): PendStatus OVERWRITE - the previous "
									& "status is LOST (total lost "
									& integer'image(w3_lost + 1) & ") t=" & time'image(now)
									severity warning;
							end if;
							rh_has := true;
							hps_slot <= '1';
							report "HPS(real): READ6 complete, status PENDED t="
								& time'image(now);
						end if;
					end if;
				elsif G_PUSH_STATUS then
					-- Push-based serving (the real cd.cpp): stream sectors subject
					-- ONLY to FIFO-full backpressure.  The SCSI_FIFO holds 4096
					-- bytes, so a sector goes out whenever the in-flight backlog
					-- (pushed minus removed, on the rebased counters above) leaves
					-- room for a whole one.  Consumption gates NOTHING here.
					while state = H_READ and rdcnt > 0
					      and (pushed - removed) <= 4096 - 2048 loop
						pushed := pushed + 2048;
						for i in 0 to 2047 loop
							cd_data <= std_logic_vector(to_unsigned(i mod 256, 8));
							cd_data_wr <= '1'; wait for 2*CLK_PERIOD;
							cd_data_wr <= '0'; wait for 2*CLK_PERIOD;
						end loop;
						lba := lba + 1;
						rdcnt := rdcnt - 1;
						if rdcnt = 0 then
							-- the final STATUS is owed from the LAST PUSHED byte:
							-- it can strobe with up to 4 KB still unconsumed, and,
							-- post-load, while the replay drain is still running
							rd_stat := true;
							rd_wait := G_STAT_TICKS;
							report "HPS: READ6 fully pushed (push mode), status in "
								& integer'image(G_STAT_TICKS) & " ticks t=" & time'image(now);
						end if;
						removed := (to_integer(rd_cons_hw) - cons_base)
						         + (skip0 - to_integer(rd_skip_hw));
					end loop;
					if rd_stat then
						if w_hold_stat = '1' then
							null;	-- scenario W is parking a save in the tail window
						elsif rd_wait > 0 then
							rd_wait := rd_wait - 1;
						else
							rd_stat := false;
							state := H_IDLE;
							pend_stat := true; stat_wait := 0;
							report "HPS: READ6 final status strobed (push mode) t="
								& time'image(now);
						end if;
					end if;
				else
					-- Consumption-gated serving waits for all earlier sectors to drain.
					if state = H_READ and rdcnt > 0
					   and (pushed = 0 or removed >= pushed) then
						pushed := pushed + 2048;
						for i in 0 to 2047 loop
							cd_data <= std_logic_vector(to_unsigned(i mod 256, 8));
							cd_data_wr <= '1'; wait for 2*CLK_PERIOD;
							cd_data_wr <= '0'; wait for 2*CLK_PERIOD;
						end loop;
						lba := lba + 1;
						rdcnt := rdcnt - 1;
					end if;
					-- final STATUS only after the whole transfer has been consumed
					if state = H_READ and rdcnt = 0 and total > 0
					   and removed >= total then
						state := H_IDLE;
						pend_stat := true; stat_wait := 1;
					end if;
				end if;
				-- CD_GET hold pauses Main's sector timer, not an in-flight transfer.
				-- Stock Main ignores the hold bit and keeps delivering.
				if ss_sleep = '1' and not G_STOCK_MAIN then
					audio_next := now + AUDIO_SECTOR_TIME;
				end if;
				if state = H_PLAY and (not G_REAL_HPS or rh_lat = 0)
				   and (ss_sleep = '0' or G_STOCK_MAIN) and now >= audio_next then
					if audio_burst = 2 then audio_next := now; end if;
					audio_next := audio_next + AUDIO_SECTOR_TIME;
					for sector in 1 to audio_burst loop
					exit when state /= H_PLAY;
					-- one CDDA sector: 588 frames, frame value = absolute
					-- frame index within the track (sample-exact oracle)
					-- (real-HPS: nothing streams while the head is seeking)
					for f in 0 to 587 loop
						frame_val := ((lba - TRACK_LBA) * 588 + f) mod 65536;
						cd_data <= std_logic_vector(to_unsigned(frame_val mod 256, 8));
						cd_audio_wr <= '1'; wait for 2*CLK_PERIOD;
						cd_audio_wr <= '0'; wait for 2*CLK_PERIOD;
						cd_data <= std_logic_vector(to_unsigned(frame_val / 256, 8));
						cd_audio_wr <= '1'; wait for 2*CLK_PERIOD;
						cd_audio_wr <= '0'; wait for 2*CLK_PERIOD;
						cd_data <= std_logic_vector(to_unsigned(frame_val mod 256, 8));
						cd_audio_wr <= '1'; wait for 2*CLK_PERIOD;
						cd_audio_wr <= '0'; wait for 2*CLK_PERIOD;
						cd_data <= std_logic_vector(to_unsigned(frame_val / 256, 8));
						cd_audio_wr <= '1'; wait for 2*CLK_PERIOD;
						cd_audio_wr <= '0'; wait for 2*CLK_PERIOD;
					end loop;
					-- Main sends 98 subcode bytes after the complete audio sector.
					for i in 0 to 97 loop
						if i = 0 then cd_data <= x"00";
						elsif i = 1 then cd_data <= x"80";
						else cd_data <= std_logic_vector(to_unsigned((lba + i) mod 256, 8));
						end if;
						cd_subcd_wr <= '1'; wait for 2*CLK_PERIOD;
						cd_subcd_wr <= '0'; wait for 2*CLK_PERIOD;
					end loop;
					lba := lba + 1;
					if lba > pend_lba then
						if mode = 1 then
							lba := pstart;
						else
							state := H_IDLE;
							-- pcecdd playback-end: "if (int_pend) SendStatus(GOOD)"
							-- Deliver the deferred SAPEP-INTERRUPT completion.
							if int_pend then
								pend_stat := true; stat_wait := 0;
								hps_intstat_count <= hps_intstat_count + 1;
								report "HPS: playback end - INTERRUPT status delivered";
							end if;
							int_pend := false;
						end if;
					end if;
					end loop;
					audio_burst := 1;
				end if;
			end if;
		end loop;
	end process;

	bootdbg2 : process
		alias cpu_pc  is << signal .tb_cd.DUT.CPU.CORE.PC : std_logic_vector(15 downto 0) >>;
		alias scsi_pos is << signal .tb_cd.DUT.CD.SCSI.COMM_POS : unsigned(3 downto 0) >>;
		alias scsi_b0  is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		alias q_stat  is << signal .tb_cd.DUT.CD.SCSI.STAT_PEND : std_logic >>;
		alias q_dout  is << signal .tb_cd.DUT.CD.SCSI.DOUT_PEND : std_logic >>;
		alias q_empty is << signal .tb_cd.DUT.CD.SCSI.EMPTY : std_logic >>;
		alias q_swal  is << signal .tb_cd.DUT.CD.SCSI.SWALLOW : unsigned(1 downto 0) >>;
		alias q_dma   is << signal .tb_cd.DUT.CD.DMA_WRITE_PEND : std_logic >>;
		alias q_cons  is << signal .tb_cd.DUT.CD.CDDA_CONSUMED : unsigned(31 downto 0) >>;
		alias q_rsect is << signal .tb_cd.DUT.CD.SCSI.RP_SECT : unsigned(23 downto 0) >>;
	begin
		for i in 1 to 40 loop
			wait for 100 us;
			exit when hps_d8_count > 0;
			report "BOOTDBG t=" & time'image(now) & " PC=" & to_hstring(cpu_pc)
				& " SP=" & to_hstring(scsi_b0) & " POS=" & integer'image(to_integer(scsi_pos));
		end loop;
		-- Sample throughout the run; READ_CONSUMED shows stream progress.
		loop
			wait for 5 ms;
			report "POSTDBG t=" & time'image(now) & " PC=" & to_hstring(cpu_pc)
				& " SP=" & to_hstring(scsi_b0)
				& " stat=" & std_logic'image(q_stat) & " dout=" & std_logic'image(q_dout)
				& " empty=" & std_logic'image(q_empty)
				& " swal=" & integer'image(to_integer(q_swal))
				& " cons=" & integer'image(to_integer(q_cons))
				& " rsect=" & integer'image(to_integer(q_rsect))
				& " rdcons=" & integer'image(to_integer(
					<< signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>))
				& " d8=" & integer'image(hps_d8_count);
		end loop;
	end process;

	-- Log re-issued commands, resume LBA/count and byte drain for scenario C.
	rp_mon : process
		alias rp_send is << signal .tb_cd.DUT.CD.SCSI.RP_SEND : std_logic >>;
		alias rp_comm is << signal .tb_cd.DUT.CD.SCSI.RP_COMM : std_logic_vector(95 downto 0) >>;
		alias rd_cons is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		alias rp_rskip is << signal .tb_cd.DUT.CD.SCSI.RP_RSKIP : unsigned(11 downto 0) >>;
		alias rd_act  is << signal .tb_cd.DUT.CD.SCSI.READ_ACTIVE : std_logic >>;
		alias sp_enc  is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		-- CDDA replay arithmetic, measured rather than inferred: the sample
		-- position a resume lands on is RP_SECT (folded into the loop window
		-- using RP_LEN) plus REPLAY_SKIP, all derived from RP_CONS.  Printing
		-- the four together at the moment the command goes out says which term
		-- is wrong instead of leaving it to be deduced from the audio.
		alias rp_cons  is << signal .tb_cd.DUT.CD.SCSI.RP_CONS : unsigned(31 downto 0) >>;
		alias rp_sect  is << signal .tb_cd.DUT.CD.SCSI.RP_SECT : unsigned(23 downto 0) >>;
		alias rp_len   is << signal .tb_cd.DUT.CD.SCSI.RP_LEN : unsigned(23 downto 0) >>;
		alias rp_skip  is << signal .tb_cd.DUT.CD.REPLAY_SKIP : unsigned(9 downto 0) >>;
		alias cdda_con is << signal .tb_cd.DUT.CD.CDDA_CONSUMED : unsigned(31 downto 0) >>;
	begin
		loop
			wait until rising_edge(clk);
			if rp_send = '1' then
				report "RPSEND cmd=" & to_hstring(rp_comm(7 downto 0))
					& " b1=" & to_hstring(rp_comm(15 downto 8))
					& " lba=" & to_hstring(rp_comm(12 downto 8) & rp_comm(23 downto 16) & rp_comm(31 downto 24))
					& " cnt=" & to_hstring(rp_comm(39 downto 32))
					& " CONS=" & integer'image(to_integer(rd_cons))
					& " RSKIP=" & integer'image(to_integer(rp_rskip))
					& " RDACT=" & std_logic'image(rd_act)
					& " SP=" & to_hstring(sp_enc)
					& " | RP_CONS=" & integer'image(to_integer(rp_cons))
					& " RP_SECT=" & integer'image(to_integer(rp_sect))
					& " RP_LEN=" & integer'image(to_integer(rp_len))
					& " SKIP=" & integer'image(to_integer(rp_skip))
					& " CDDA_CONSUMED=" & integer'image(to_integer(cdda_con));
			end if;
		end loop;
	end process;

	-- Mismatch monitor: fires the first time the reader stores the fail flag
	-- ($FE58 STA $2012), logging the RTL's DATAIN byte position so the
	-- reader-vs-stream desync can be read off directly.
	mm_mon : process
		alias mpc     is << signal .tb_cd.DUT.CPU.CORE.PC : std_logic_vector(15 downto 0) >>;
		alias din_cnt is << signal .tb_cd.DUT.CD.SCSI.DATAIN_CNT : unsigned(15 downto 0) >>;
		alias rd_cons2 is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		alias sp2     is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		alias rskip2  is << signal .tb_cd.DUT.CD.SCSI.READ_SKIP : unsigned(11 downto 0) >>;
		alias aack    is << signal .tb_cd.DUT.CD.AUTO_ACK : std_logic >>;
		variable fired : boolean := false;
		variable aack_p : std_logic := '0';
		variable cons_p : integer := 0;
		variable nreads : integer := 0;	-- CPU $5808 data reads (one AUTO_ACK arm each)
		variable ncons  : integer := 0;	-- RTL DATAIN handshakes (READ_CONSUMED increments)
	begin
		loop
			wait until rising_edge(clk);
			if aack = '1' and aack_p = '0' then nreads := nreads + 1; end if;
			aack_p := aack;
			if to_integer(rd_cons2) = cons_p + 1 then ncons := ncons + 1; end if;
			cons_p := to_integer(rd_cons2);
			if not fired and mpc = x"FE58" then
				report "MISMATCH@FE58 DATAIN_CNT=" & integer'image(to_integer(din_cnt))
					& " CONS=" & integer'image(to_integer(rd_cons2))
					& " NREADS=" & integer'image(nreads)
					& " NCONS=" & integer'image(ncons)
					& " READ_SKIP=" & integer'image(to_integer(rskip2))
					& " SP=" & to_hstring(sp2) & " t=" & time'image(now);
				fired := true;
				-- The reader checks byte i against i mod 256. Reaching $FE58
				-- records a stream mismatch, independent of later blob timing.
				c_fail <= '1';
			end if;
		end loop;
	end process;

	-- Byte trace around the post-replay resume: logs the stream position and the
	-- staged byte for the first handshakes after the drain finishes, plus what
	-- the CPU actually loaded, so a resume misalignment can be read off directly
	-- instead of inferred from the fail flag.
	bt_mon : process
		alias rskip is << signal .tb_cd.DUT.CD.SCSI.READ_SKIP : unsigned(11 downto 0) >>;
		alias cons  is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		alias dbo   is << signal .tb_cd.DUT.CD.SCSI.DBO_r : std_logic_vector(7 downto 0) >>;
		-- drain accounting: every FIFO_RD_REQ that the FIFO actually honours is a
		-- popped byte; comparing that with how far READ_SKIP fell says straight
		-- away whether the drain skipped the number of bytes it claimed to
		alias frdreq is << signal .tb_cd.DUT.CD.SCSI.FIFO_RD_REQ : std_logic >>;
		alias fempty is << signal .tb_cd.DUT.CD.SCSI.EMPTY : std_logic >>;
		variable prev_cons : integer := -1;
		variable armed : boolean := false;
		variable n : integer := 0;
		variable pops, decs, prev_rskip, wrs : integer := 0;
		variable wr_d : std_logic := '0';
	begin
		loop
			wait until rising_edge(clk);
			-- count pops, READ_SKIP decrements and bytes the drive pushed while a
			-- drain is in progress
			if rskip /= 0 or prev_rskip /= 0 then
				if frdreq = '1' and fempty = '0' then pops := pops + 1; end if;
				if cd_data_wr = '1' and wr_d = '0' then wrs := wrs + 1; end if;
				if to_integer(rskip) = prev_rskip - 1 then decs := decs + 1; end if;
				if to_integer(rskip) = 0 and prev_rskip = 1 then
					report "BT DRAIN done: decs=" & integer'image(decs)
						& " pops=" & integer'image(pops)
						& " drive_wrote=" & integer'image(wrs) & " t=" & time'image(now);
					pops := 0; decs := 0; wrs := 0;
				end if;
			end if;
			wr_d := cd_data_wr;
			prev_rskip := to_integer(rskip);
			if rskip /= 0 then armed := true; n := 0; end if;
			if armed then
				if to_integer(cons) /= prev_cons and n < 10 then
					report "BT CONS=" & integer'image(to_integer(cons))
						& " DBO=" & integer'image(to_integer(unsigned(dbo)))
						& " t=" & time'image(now);
					n := n + 1;
				end if;
			end if;
			prev_cons := to_integer(cons);
		end loop;
	end process;

	-- CDDA drain accounting: the B/G resumes land 2-3 samples EARLY, with the
	-- replay arithmetic proven exact (RPSEND: sector + SKIP add up to the
	-- consumed count), so the loss must be in the skip drain itself.  Count,
	-- for every armed skip, the SKIP_LEFT decrements against the pops the
	-- FIFO actually honoured and the samples the DAC branch delivered while
	-- the drain was still open: whichever pair disagrees names the defect.
	cdda_drain_mon : process(clk)
		alias skipl  is << signal .tb_cd.DUT.CD.SKIP_LEFT : unsigned(9 downto 0) >>;
		alias frdreq is << signal .tb_cd.DUT.CD.FIFO_RD_REQ : std_logic >>;
		alias fempty is << signal .tb_cd.DUT.CD.FIFO_EMPTY : std_logic >>;
		alias fwrreq is << signal .tb_cd.DUT.CD.FIFO_WR_REQ : std_logic >>;
		alias fwrd   is << signal .tb_cd.DUT.CD.FIFO_D : std_logic_vector(31 downto 0) >>;
		alias rp_fl  is << signal .tb_cd.DUT.CD.REPLAY_START : std_logic >>;
		alias fq     is << signal .tb_cd.DUT.CD.FIFO_Q : std_logic_vector(31 downto 0) >>;
		variable fl_d : std_logic := '0';
		variable wr_n : integer := 99;
		variable prev_skip : integer := 0;
		variable armed_at  : integer := 0;
		variable tape_at_arm : integer := 0;
		variable decs, pops, reqs, misses : integer := 0;
		variable qlast : integer := -1;
		variable in_drain : boolean := false;
		variable v0, v1, v2, v3 : integer := -1;	-- last four delivered values
		variable prev_out : integer := -1;
		variable post_n : integer := 99;
	begin
		if rising_edge(clk) then
			-- Timestamp output samples around each flush and drain.
			if to_integer(signed(cdda_sl)) /= prev_out then
				prev_out := to_integer(signed(cdda_sl));
				v3 := v2; v2 := v1; v1 := v0; v0 := prev_out;
				if post_n < 6 then
					report "CDDA OUT#" & integer'image(post_n)
						& " val=" & integer'image(prev_out)
						& " t=" & time'image(now);
					post_n := post_n + 1;
				end if;
			end if;
			-- First FIFO writes distinguish a stale tail from resumed data.
			if rp_fl = '1' and fl_d = '0' then
				wr_n := 0;
				report "CDDA FLUSH tape=" & integer'image(slog_count)
					& " last_out=" & integer'image(v3) & " " & integer'image(v2)
					& " " & integer'image(v1) & " " & integer'image(v0)
					& " t=" & time'image(now);
			end if;
			fl_d := rp_fl;
			if fwrreq = '1' and wr_n < 8 then
				report "CDDA WR#" & integer'image(wr_n)
					& " val=" & integer'image(to_integer(unsigned(fwrd(15 downto 0))))
					& " t=" & time'image(now);
				wr_n := wr_n + 1;
			end if;
			if to_integer(skipl) > prev_skip then
				-- the arm (REPLAY_START loads REPLAY_SKIP)
				in_drain := true;
				armed_at := to_integer(skipl);
				tape_at_arm := slog_count;
				decs := 0; pops := 0; reqs := 0; misses := 0; qlast := -1;
			elsif in_drain then
				if frdreq = '1' then
					reqs := reqs + 1;
					if fempty = '0' then
						pops := pops + 1;
						qlast := to_integer(unsigned(fq(15 downto 0)));
					else
						-- a request the FIFO cannot honour: the skip counter
						-- has already paid for a sample that never left
						misses := misses + 1;
						if misses <= 4 then
							report "CDDA DRAIN MISS #" & integer'image(misses)
								& " skipl=" & integer'image(to_integer(skipl))
								& " t=" & time'image(now);
						end if;
					end if;
				end if;
				if to_integer(skipl) = prev_skip - 1 then decs := decs + 1; end if;
				if to_integer(skipl) = 0 and prev_skip /= 0 then
					report "CDDA DRAIN done: armed=" & integer'image(armed_at)
						& " decs=" & integer'image(decs)
						& " reqs=" & integer'image(reqs)
						& " pops=" & integer'image(pops)
						& " misses=" & integer'image(misses)
						& " last_drained=" & integer'image(qlast)
						& " delivered_during=" & integer'image(slog_count - tape_at_arm)
						& " t=" & time'image(now);
					in_drain := false;
					post_n := 0;	-- log the next deliveries with timestamps
				end if;
			end if;
			prev_skip := to_integer(skipl);
		end if;
	end process;

	fl_mon : process(clk)
		alias rp_flush is << signal .tb_cd.DUT.CD.REPLAY_START : std_logic >>;
		alias rp_cons  is << signal .tb_cd.DUT.CD.SCSI.RP_CONS : unsigned(31 downto 0) >>;
		variable d : std_logic := '0';
	begin
		if rising_edge(clk) then
			if rp_flush = '1' and d = '0' then
				flush_log(flush_n mod 8) <= slog_count;
				flush_cons(flush_n mod 8) <= to_integer(rp_cons);
				flush_n <= flush_n + 1;
				flush_mark <= slog_count;
			end if;
			d := rp_flush;
		end if;
	end process;

	rel_mon : process
		alias sleep_eff is << signal .tb_cd.DUT.ss_sleep_eff : std_logic >>;
	begin
		wait until falling_edge(sleep_eff);
		rel_t <= now;
	end process;

	-- ------------------------------------------------------------------
	-- K/L observe the same falling edges and synchronous reset as the
	-- former RTL counters. Edge history is retained across reset and load.
	handshake_mon : process
		alias scsi_clk is << signal .tb_cd.DUT.CD.SCSI.CLK : std_logic >>;
		alias reset_n is << signal .tb_cd.DUT.CD.SCSI.RESET_N : std_logic >>;
		alias req_n is << signal .tb_cd.DUT.CD.SCSI.REQ_Nr : std_logic >>;
		alias ack_n is << signal .tb_cd.DUT.CD.SCSI.ACK_N : std_logic >>;
		variable req_d, ack_d : std_logic := '1';
	begin
		loop
			wait until rising_edge(scsi_clk);
			if reset_n = '0' then
				l_cnt_req <= (others => '0');
				l_cnt_ack <= (others => '0');
			else
				if req_n = '0' and req_d = '1' then
					l_cnt_req <= l_cnt_req + 1;
				end if;
				if ack_n = '0' and ack_d = '1' then
					l_cnt_ack <= l_cnt_ack + 1;
				end if;
				req_d := req_n;
				ack_d := ack_n;
			end if;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- A restored query may legitimately hold BSY while SP_FREE waits for data.
	-- Detect instead a bus with no recorded work that can complete its
	-- transaction. Arm after load; ignore frozen cycles and debounce transient
	-- register updates. Scenario progress checks also cover unfulfilled debts.
	cp_mon : process
		alias bsyn   is << signal .tb_cd.DUT.CD.SCSI.BSY_Nr : std_logic >>;
		alias spenc2 is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		alias cract  is << signal .tb_cd.DUT.CD.SCSI.READ_ACTIVE : std_logic >>;
		alias ssload is << signal .tb_cd.DUT.CD.SCSI.SaveStateBus_load : std_logic >>;
		alias qry    is << signal .tb_cd.DUT.CD.SCSI.QRY_ACTIVE : std_logic >>;
		alias audop  is << signal .tb_cd.DUT.CD.SCSI.AUD_OPEN : std_logic >>;
		alias statp  is << signal .tb_cd.DUT.CD.SCSI.STAT_PEND : std_logic >>;
		alias doutp  is << signal .tb_cd.DUT.CD.SCSI.DOUT_PEND : std_logic >>;
		alias emp    is << signal .tb_cd.DUT.CD.SCSI.EMPTY : std_logic >>;
		alias cmdout is << signal .tb_cd.DUT.CD.SCSI.CMD_OUTST : std_logic >>;
		alias quiet  is << signal .tb_cd.DUT.CD.SCSI.SS_QUIET : std_logic >>;
		alias rskip  is << signal .tb_cd.DUT.CD.SCSI.READ_SKIP : unsigned(11 downto 0) >>;
		alias swal   is << signal .tb_cd.DUT.CD.SCSI.SWALLOW : unsigned(1 downto 0) >>;
		constant DEBOUNCE : integer := 4096;	-- 41 us of tb clock
		variable armed : boolean := false;
		variable held  : integer := 0;
	begin
		loop
			wait until rising_edge(clk);
			if ssload = '1' then armed := true; end if;
			-- With the no-work predicates below, SS_QUIET is true exactly
			-- when replay is idle.
			if armed and ss_sleep = '0'
			   and spenc2 = x"0" and bsyn = '0'
			   and cract = '0' and qry = '0' and audop = '0'
			   and statp = '0' and doutp = '0' and emp = '1'
			   and cmdout = '0' and quiet = '1'
			   and rskip = 0 and swal = 0 then
				held := held + 1;
			else
				held := 0;
			end if;
			assert held < DEBOUNCE
				report "TBCD BUS STUCK: BSY held with the phase free and nothing "
					& "outstanding -- no READ, query, audio command, status, DATAOUT, "
					& "FIFO byte, drain or replay is left to complete the transaction "
					& "(t=" & time'image(now) & ")"
				severity failure;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- Keep recent PCs and dump them when execution leaves the synthetic BIOS
	-- code regions or the four-byte routine copied into CD-RAM at $6000.
	-- ------------------------------------------------------------------
	pc_mon : process
		alias gpc is << signal .tb_cd.DUT.CPU.CORE.PC : std_logic_vector(15 downto 0) >>;
		constant DEPTH : integer := 96;
		-- PC is sampled per cycle, so it legitimately runs a byte or two past the
		-- last opcode of a region while an instruction is being fetched (an RTS at
		-- $FE9B shows $FE9C).  A single out-of-range sample therefore proves
		-- nothing; a SUSTAINED run outside the code does.  RUNAWAY is the number
		-- of consecutive out-of-range PC changes that count as derailed.
		constant RUNAWAY : integer := 16;
		variable badrun : integer := 0;
		type ring_t is array (0 to DEPTH-1) of std_logic_vector(15 downto 0);
		variable ring  : ring_t := (others => (others => '0'));
		variable idx   : integer := 0;
		variable filled : integer := 0;
		variable prev  : std_logic_vector(15 downto 0) := (others => '0');
		variable bad   : boolean;
		variable line  : string(1 to 5*DEPTH);
		variable p     : integer;
		-- the CPU is held during a savestate walk and its PC reads 0000 there,
		-- which is not a derailment: judge only while the machine is awake, and
		-- give it a moment to resume before looking again
		variable hold_until : time := 0 fs;
	begin
		-- let the machine come out of reset before judging the PC
		wait for 100 us;
		loop
			wait until rising_edge(clk);
			if ss_sleep = '1' then
				hold_until := now + 50 us;
			end if;
			if gpc /= prev then
				prev := gpc;
				ring(idx) := gpc;
				idx := (idx + 1) mod DEPTH;
				if filled < DEPTH then filled := filled + 1; end if;

				-- Only BIOS code regions are legal; filler and command tables
				-- must not execute and overwrite the useful PC trace.
				--   VBL handler $FB00-$FB0B, enable $FB40-$FB46 (scenario J)
				--   SUBQ     $FC00-$FC7F
				--   ADPCM DMA $FD00-$FD88, main loop $FDC0-$FDD1
				--   reader   $FE00-$FE9B
				--   boot     $FF00-$FF4B
				--   CD-RAM   $6000-$6003
				bad := false;
				if is_x(gpc) then
					bad := true;
				elsif not ((unsigned(gpc) >= 16#6000# and unsigned(gpc) <= 16#6003#)
				        or (unsigned(gpc) >= 16#F800# and unsigned(gpc) <= 16#F8FA#)
				        or (unsigned(gpc) >= 16#F900# and unsigned(gpc) <= 16#F99B#)
				        or (unsigned(gpc) >= 16#FA00# and unsigned(gpc) <= 16#FAFB#)
				        or (unsigned(gpc) >= 16#FB00# and unsigned(gpc) <= 16#FB0C#)
				        or (unsigned(gpc) >= 16#FB20# and unsigned(gpc) <= 16#FB37#)
				        or (unsigned(gpc) >= 16#FB40# and unsigned(gpc) <= 16#FB47#)
				        or (unsigned(gpc) >= 16#FB60# and unsigned(gpc) <= 16#FB76#)
				        or (unsigned(gpc) >= 16#FC00# and unsigned(gpc) <= 16#FC80#)
				        or (unsigned(gpc) >= 16#FD00# and unsigned(gpc) <= 16#FD89#)
				        or (unsigned(gpc) >= 16#FDC0# and unsigned(gpc) <= 16#FDD5#)
				        or (unsigned(gpc) >= 16#FE00# and unsigned(gpc) <= 16#FE9B#)
				        or (unsigned(gpc) >= 16#FF00# and unsigned(gpc) <= 16#FFBD#)) then
					bad := true;
				end if;

				if bad then
					badrun := badrun + 1;
				else
					badrun := 0;
				end if;

				if badrun >= RUNAWAY and now >= hold_until then
					p := 1;
					for k in 0 to filled-1 loop
						line(p to p+3) := to_hstring(ring((idx + DEPTH - filled + k) mod DEPTH));
						line(p+4) := ' ';
						p := p + 5;
					end loop;
					report "TBCD PC DERAILED at t=" & time'image(now)
						& " -- " & integer'image(badrun) & " consecutive PCs outside the "
						& "program; last " & integer'image(filled) & " (oldest first): "
						& line(1 to p-1)
						severity failure;
				end if;
			end if;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- Fail near the point where a READ stops making progress.
	--
	-- The threshold has to clear the longest LEGITIMATE stall: a savestate load
	-- holds the reader for the replay settle + the ~24 ms command gap + the
	-- drain, so ~40 ms is normal.  ss_sleep covers the walks themselves.
	-- ------------------------------------------------------------------
	wd_mon : process
		alias wcons is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		alias wact  is << signal .tb_cd.DUT.CD.SCSI.READ_ACTIVE : std_logic >>;
		alias wpc   is << signal .tb_cd.DUT.CPU.CORE.PC : std_logic_vector(15 downto 0) >>;
		alias wsp   is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		variable last : integer := -1;
		variable idle : integer := 0;
	begin
		loop
			wait for 1 ms;
			if wact = '1' and ss_sleep = '0' then
				if to_integer(wcons) = last then
					idle := idle + 1;
				else
					idle := 0;
					last := to_integer(wcons);
				end if;
			else
				idle := 0;
				last := -1;
			end if;
			assert idle < 120
				report "TBCD STREAM WEDGED: READ active but READ_CONSUMED stuck at "
					& integer'image(last) & " for " & integer'image(idle)
					& " ms.  PC=" & to_hstring(wpc) & " SP=" & to_hstring(wsp)
					& " t=" & time'image(now)
				severity failure;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- Scenario E: DTR / CD_DATA_CNT bookkeeping across a resume.
	--
	-- CD_DATA_CNT (cd.vhd) counts the bytes the CPU has acked inside the
	-- current 2048-byte sector and drops CD_DTR when it wraps; the SCSI side
	-- counts the same handshakes in READ_CONSUMED.  They must therefore agree
	-- modulo 2048 at every quiet point of a READ.  A savestate stores both
	-- (CD_1 bits 43:33 and SCSI READ_CONSUMED), so a restore that rebuilt one
	-- but not the other - or a replay drain that fed bytes past one of them -
	-- shows up here as a divergence, and the game would lose its sector
	-- boundary (a wrong DTR interrupt, or a sector served 2048 bytes late).
	--
	-- Sampled only in SP_DATAIN_START with no ACK in flight and the CPU-facing
	-- FSM not held: that is the steady point between two handshakes, where the
	-- two counters have both settled.
	-- ------------------------------------------------------------------
	-- ------------------------------------------------------------------
	-- Scenario H monitor: every DMA write into ADPCM RAM, checked against a
	-- PURELY POSITIONAL oracle.
	--
	-- The routine at $FD00 resets the ADPCM address to 0 before enabling the
	-- DMA, and cd.vhd stores byte j of the transfer as two nibbles at
	-- addresses 2j (high) and 2j+1 (low).  The drive model's READ6 payload is
	-- "byte j = j mod 256" and 2048 is a multiple of 256, so the expected
	-- nibble is a function of the ADDRESS ALONE - no running counter, nothing
	-- that a save/load rewind could desynchronise.  That is deliberate: the
	-- machine goes backwards in time in the middle of this transfer, so any
	-- oracle carrying its own state would have to be rewound too, and would
	-- then be checking itself instead of the DUT.
	--
	-- What the three verdicts catch:
	--   h_fail  a byte written at the wrong place, or the wrong byte written
	--           (a resumed stream off by n bytes fails on the first write)
	--   h_cov   every one of the 8192 nibbles written at least once, i.e. the
	--           replay left no HOLE (bytes skipped at the resume point)
	--   h_after requires writes after load, not just before save.
	--
	-- Gated on SLEEP='0': during a walk the same port carries the savestate
	-- shuttle (ADRAM_A = SS_AD_Addr & SS_AD_NIB), which is the twin event that
	-- would otherwise be counted as DMA traffic.
	-- ------------------------------------------------------------------
	h_mon : process
		alias h_adr   is << signal .tb_cd.DUT.CD.ADRAM_A  : std_logic_vector(16 downto 0) >>;
		alias h_di    is << signal .tb_cd.DUT.CD.ADRAM_DI : std_logic_vector(3 downto 0) >>;
		alias h_we    is << signal .tb_cd.DUT.CD.ADRAM_WE : std_logic >>;
		alias h_slp   is << signal .tb_cd.DUT.CD.SLEEP    : std_logic >>;
		alias h_ract  is << signal .tb_cd.DUT.CD.SCSI.READ_ACTIVE : std_logic >>;
		-- ADPCM RAM is 64 KB stored as 2^17 nibbles; ADRAM_A is exactly that
		-- wide, so "address + 1" already wraps the way the oracle needs.
		constant NIBS : integer := 131072;
		type cov_t is array (0 to NIBS-1) of std_logic;
		variable cov  : cov_t := (others => '0');
		variable a, v, e, nbad : integer;
		variable last : integer := -1;
		variable slp_old : std_logic := '0';
	begin
		nbad := 0;
		loop
			wait until rising_edge(clk);
			-- snapshot for the stim: was the READ still in flight when the
			-- machine actually froze?  Without it a save that arrived after the
			-- transfer had finished would still pass every check below, and the
			-- scenario would silently stop testing a mid-DMA save.
			if h_slp = '1' and slp_old = '0' then
				h_ract_frz <= h_ract;
				sp_frz     <= << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
			end if;
			slp_old := h_slp;

			-- Scenario A loads can overlap DMA startup and rewind its address.
			-- Clear counters once H is running so its contiguity check covers
			-- only H's save/load.
			if h_clr = '1' then
				cov  := (others => '0');
				last := -1;
				nbad := 0;
				h_nib <= 0; h_cov <= 0; h_after <= 0; h_jumps <= 0;
				h_fail <= '0'; h_bad_a <= -1; h_bad_v <= -1; h_bad_e <= -1;
			elsif h_arm = '1' and h_slp = '0' and h_we = '1' then
				a := to_integer(unsigned(h_adr));
				v := to_integer(unsigned(h_di));
				if (a mod 2) = 0 then
					e := ((a / 2) mod 256) / 16;
				else
					e := ((a / 2) mod 256) mod 16;
				end if;
				h_nib <= h_nib + 1;
				if h_mark = '1' then
					h_after <= h_after + 1;
				end if;
				if cov(a) = '0' then
					cov(a) := '1';
					h_cov <= h_cov + 1;
				end if;
				-- contiguity: the DMA writes nibble n then n+1 for every byte and
				-- never skips.  The ONLY legal discontinuity is the load rewinding
				-- the write address, so h_jumps is a direct count of holes and
				-- double-writes the replay would have introduced.
				if last >= 0 and a /= ((last + 1) mod NIBS) then
					h_jumps <= h_jumps + 1;
					if h_jumps < 4 then
						report "TBCD H: ADPCM address jump " & integer'image(last)
							& " -> " & integer'image(a) & " t=" & time'image(now);
					end if;
				end if;
				last := a;
				if v /= e then
					nbad := nbad + 1;
					if nbad <= 8 then
						report "TBCD H: ADPCM nibble addr=" & integer'image(a)
							& " got=" & integer'image(v) & " expected=" & integer'image(e)
							& " t=" & time'image(now) severity error;
					end if;
					if h_fail = '0' then
						h_bad_a <= a; h_bad_v <= v; h_bad_e <= e;
					end if;
					h_fail <= '1';
				end if;
			end if;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- Scenario H diagnostics: ACK/AUTO_ACK at freeze distinguish stored
	-- data from a pending byte; READ_SKIP, DATAIN_HELD and DBO at the first
	-- post-load write show what released it. Reports only, no verdict.
	-- ------------------------------------------------------------------
	-- Scenario I: count the $DD replies the PROGRAM actually finishes, by
	-- watching the PC reach the "INC $2021" that closes the SUBQ routine.
	-- Counted on the TRANSITION into that address, not on its presence: the PC
	-- is sampled every cycle and would otherwise be counted many times over -
	-- the same trap pc_mon documents.
	i_mon : process
		alias ipc is << signal .tb_cd.DUT.CPU.CORE.PC : std_logic_vector(15 downto 0) >>;
		variable prev : std_logic_vector(15 downto 0) := (others => '0');
	begin
		loop
			wait until rising_edge(clk);
			if ipc = x"FC7C" and prev /= x"FC7C" then
				i_cpu_done <= i_cpu_done + 1;
			end if;
			prev := ipc;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- Scenario W monitors.
	--
	-- w_mon counts program landmarks on PC transitions:
	--   $FE98  the boot reader's closing INC        -> w_reads_done
	--   $FA98  the fresh-READ6 reader's closing INC -> w_fresh_done
	--   $FA58  the fresh reader's "record first mismatch" store -> w_fail,
	--          with the stream position and the staged byte, so a stale
	--          prepended byte is read off directly (offset+got; expected is
	--          the reader's own counter, i.e. DATAIN_CNT-1 mod 256).
	-- It also counts every READ6 for the FRESH lba (0x000900) that reaches
	-- the drive model: more than one from the same timeline = the program is
	-- RETRYING the command, the hardware black-screen signature.
	-- ------------------------------------------------------------------
	w_mon : process
		alias wpc  is << signal .tb_cd.DUT.CPU.CORE.PC : std_logic_vector(15 downto 0) >>;
		alias wdin is << signal .tb_cd.DUT.CD.SCSI.DATAIN_CNT : unsigned(15 downto 0) >>;
		alias wdbo is << signal .tb_cd.DUT.CD.SCSI.DBO_r : std_logic_vector(7 downto 0) >>;
		alias wskp is << signal .tb_cd.DUT.CD.SCSI.READ_SKIP : unsigned(11 downto 0) >>;
		alias wcns is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		-- W2 counters: REQ/ACK, swallow, status strobes/pends, mailbox/SEL.
		alias wswl is << signal .tb_cd.DUT.CD.SCSI.SWALLOW : unsigned(1 downto 0) >>;
		alias wstp is << signal .tb_cd.DUT.CD.SCSI.STAT_PEND : std_logic >>;
		alias wsel is << signal .tb_cd.DUT.CD.SCSI.SEL_N : std_logic >>;
		variable prev : std_logic_vector(15 downto 0) := (others => '0');
		variable seen : integer := 0;
		variable nstatget : integer := 0;	-- cd_stat_get rising edges (HPS strobes)
		variable nsel     : integer := 0;	-- game SELs (SEL_N falling edges)
		variable sg_d, sel_d : std_logic := '0';
		procedure tally(name : string) is
		begin
			report "TBCD W2 TALLY [" & name & "]: SWALLOW=" & integer'image(to_integer(wswl))
				& " STAT_PEND=" & std_logic'image(wstp)
				& " STAT_GETS=" & integer'image(nstatget)
				& " MBOX=" & integer'image(mbox_count)
				& " SELS=" & integer'image(nsel)
				& " REQ=" & integer'image(to_integer(l_cnt_req))
				& " ACK=" & integer'image(to_integer(l_cnt_ack))
				& " SLOT=" & std_logic'image(hps_slot)
				& " LOST=" & integer'image(w3_lost)
				& " t=" & time'image(now);
		end procedure;
	begin
		loop
			wait until rising_edge(clk);
			if cd_stat_get = '1' and sg_d = '0' then nstatget := nstatget + 1; end if;
			sg_d := cd_stat_get;
			if wsel = '0' and sel_d = '1' then nsel := nsel + 1; end if;
			sel_d := wsel;
			if mbox_count /= seen then
				seen := mbox_count;
				if mbox_comm(7 downto 0) = x"08"
				   and mbox_comm(15 downto 8) = x"00"
				   and mbox_comm(23 downto 16) = x"09" then
					w_read6_cnt <= w_read6_cnt + 1;
					report "TBCD W: fresh READ6 (lba 0x900) reached the model, #"
						& integer'image(w_read6_cnt + 1) & " t=" & time'image(now);
				end if;
			end if;
			if wpc /= prev and not is_x(wpc) then
				if wpc = x"FE98" then
					w_reads_done <= w_reads_done + 1;
				end if;
				if wpc = x"FA98" then
					w_fresh_done <= w_fresh_done + 1;
					report "TBCD W: fresh READ6 conversation closed, DATAIN_CNT="
						& integer'image(to_integer(wdin)) & " t=" & time'image(now);
					if G_DO_W2 then tally("READ6 #1 closed"); end if;
				end if;
				if wpc = x"FA58" or wpc = x"F958" then
					if w_fail = '0' then
						report "TBCD W MISMATCH in the reader at PC=" & to_hstring(wpc)
							& " ($FA58=fresh #1, $F958=#2)"
							& ": offset(DATAIN_CNT)=" & integer'image(to_integer(wdin))
							& " expected=" & integer'image((to_integer(wdin) - 1) mod 256)
							& " DBO=" & to_hstring(wdbo)
							& " READ_SKIP=" & integer'image(to_integer(wskp))
							& " CONS=" & integer'image(to_integer(wcns))
							& " t=" & time'image(now) severity error;
					end if;
					w_fail <= '1';
				end if;
				-- W2 landmarks and boundary counters.
				if wpc = x"FAE0" then
					w2_audio <= w2_audio + 1;
					tally("D8+D9 closed, CDDA commanded");
				end if;
				if wpc = x"FAF0" then
					w2_read2 <= w2_read2 + 1;
					tally("READ6 #2 closed");
				end if;
				if wpc = x"FAF6" then
					w2_end <= w2_end + 1;
					tally("READ6 #3 closed - END");
				end if;
				prev := wpc;
			end if;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- Scenario W diagnostics: capture counters at freeze, FIFO/phase/DBO
	-- at drain end, and the first 12 subsequent DATAIN handshakes.
	-- ------------------------------------------------------------------
	w_probe : process
		alias pskp is << signal .tb_cd.DUT.CD.SCSI.READ_SKIP : unsigned(11 downto 0) >>;
		alias pcns is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		alias pdin is << signal .tb_cd.DUT.CD.SCSI.DATAIN_CNT : unsigned(15 downto 0) >>;
		alias pdbo is << signal .tb_cd.DUT.CD.SCSI.DBO_r : std_logic_vector(7 downto 0) >>;
		alias pemp is << signal .tb_cd.DUT.CD.SCSI.EMPTY : std_logic >>;
		alias pstp is << signal .tb_cd.DUT.CD.SCSI.STAT_PEND : std_logic >>;
		alias psp  is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		alias pact is << signal .tb_cd.DUT.CD.SCSI.READ_ACTIVE : std_logic >>;
		alias paud is << signal .tb_cd.DUT.CD.SCSI.AUDIO_ACTIVE : std_logic >>;
		alias popn is << signal .tb_cd.DUT.CD.SCSI.AUD_OPEN : std_logic >>;
		variable prev_skip : integer := 0;
		variable prev_din  : integer := 0;
		variable logn      : integer := 0;
		variable slp_d     : std_logic := '0';
	begin
		if not (G_DO_W or G_DO_W2 or G_DO_W3) then wait; end if;
		loop
			wait until rising_edge(clk);
			if ss_sleep = '1' and slp_d = '0' then
				w_frz_slot    <= hps_slot;
				w_frz_audopen <= popn;
				report "TBCD W FRZ: CONS=" & integer'image(to_integer(pcns))
					& " SP=" & to_hstring(psp)
					& " RDACT=" & std_logic'image(pact)
					& " AUDACT=" & std_logic'image(paud)
					& " AUDOPEN=" & std_logic'image(popn)
					& " SLOT=" & std_logic'image(hps_slot)
					& " SKIP=" & integer'image(to_integer(pskp))
					& " EMPTY=" & std_logic'image(pemp)
					& " STAT_PEND=" & std_logic'image(pstp)
					& " DATAIN_CNT=" & integer'image(to_integer(pdin))
					& " t=" & time'image(now);
			end if;
			slp_d := ss_sleep;
			if prev_skip /= 0 and to_integer(pskp) = 0 then
				report "TBCD W DRAIN END: EMPTY=" & std_logic'image(pemp)
					& " STAT_PEND=" & std_logic'image(pstp)
					& " SP=" & to_hstring(psp)
					& " DBO=" & to_hstring(pdbo)
					& " CONS=" & integer'image(to_integer(pcns))
					& " DATAIN_CNT=" & integer'image(to_integer(pdin))
					& " t=" & time'image(now);
				logn := 12;
				prev_din := to_integer(pdin);
			end if;
			prev_skip := to_integer(pskp);
			if logn > 0 and to_integer(pdin) /= prev_din then
				report "TBCD W BYTE: DATAIN_CNT=" & integer'image(to_integer(pdin))
					& " DBO=" & to_hstring(pdbo)
					& " SP=" & to_hstring(psp)
					& " t=" & time'image(now);
				logn := logn - 1;
				prev_din := to_integer(pdin);
			end if;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- Saves must capture STATE=0: CPU mid-instruction internals
	-- (AA, DR, T, MC, NEXT_STATE, ALU_*) are not stored.
	-- Watch both enabled CPU cycles during SLEEP and changes in STATE:
	-- a leaked registered CE need not change STATE on every instruction.
	-- ------------------------------------------------------------------
	-- ------------------------------------------------------------------
	-- Freeze-overflow monitor.  A CDDA sample completes when CD_AUDIO_WR rises
	-- on the 4th byte (CD_BYTE_CNT="11"); a subcode byte when CD_SUBCD_WR
	-- rises.  If the corresponding FIFO is FULL at that moment the write
	-- request is suppressed inside cd.vhd and the data is silently gone.
	-- Count only during SLEEP, when consumers freeze but input remains live.
	-- ------------------------------------------------------------------
	frz_mon : process
		alias f_full  is << signal .tb_cd.DUT.CD.FIFO_FULL : std_logic >>;
		alias f_cnt   is << signal .tb_cd.DUT.CD.CD_BYTE_CNT : unsigned(1 downto 0) >>;
		alias s_full  is << signal .tb_cd.DUT.CD.SUBCD_FIFO_FULL : std_logic >>;
		variable aw_old, sw_old : std_logic := '0';
	begin
		loop
			wait until rising_edge(clk);
			if ss_sleep = '1' then
				if cd_audio_wr = '1' and aw_old = '0' and f_cnt = "11" and f_full = '1' then
					frz_cdda_drop <= frz_cdda_drop + 1;
					if frz_cdda_drop = 0 then
						report "TBCD FRZ: CDDA sample DROPPED during the freeze (FIFO full) t="
							& time'image(now) severity error;
					end if;
				end if;
				if cd_subcd_wr = '1' and sw_old = '0' and s_full = '1' then
					frz_subc_drop <= frz_subc_drop + 1;
					if frz_subc_drop = 0 then
						report "TBCD FRZ: SUBCODE byte DROPPED during the freeze (FIFO full) t="
							& time'image(now) severity error;
					end if;
				end if;
			end if;
			aw_old := cd_audio_wr;
			sw_old := cd_subcd_wr;
		end loop;
	end process;

	st0_mon : process
		alias c_en    is << signal .tb_cd.DUT.CPU.EN : std_logic >>;
		alias c_slp   is << signal .tb_cd.DUT.CPU.SLEEP : std_logic >>;
		alias c_state is << signal .tb_cd.DUT.CPU.CORE.STATE : unsigned(4 downto 0) >>;
		alias c_cnt   is << signal .tb_cd.DUT.CPU.CPU_CLK_CNT : unsigned(4 downto 0) >>;
		alias c_cs    is << signal .tb_cd.DUT.CPU.CPU_CS : std_logic >>;
		alias c_ce    is << signal .tb_cd.DUT.CPU.CPU_CE : std_logic >>;
		variable slp_old : std_logic := '0';
		variable st_frz  : integer := -1;
		variable trace   : integer := 0;
	begin
		loop
			wait until rising_edge(clk);
			if c_slp = '1' and slp_old = '0' then
				st_frz := to_integer(c_state);
				trace  := 4;
				-- The phase decides whether a CE is already in flight when the
				-- freeze lands: the generator fires at CPU_CLK_CNT = 5 (CS=1) or
				-- 23 (CS=0), so only a freeze taken on the cycle right before
				-- that can leak one.  Printing it turns "we did not see it" into
				-- "we did not see it AND here is why", instead of luck.
				report "TBCD ST0 freeze: CLK_CNT=" & integer'image(to_integer(c_cnt))
					& " CS=" & std_logic'image(c_cs)
					& " CE=" & std_logic'image(c_ce)
					& " EN=" & std_logic'image(c_en)
					& " STATE=" & integer'image(st_frz)
					& " t=" & time'image(now);
			elsif trace > 0 then
				trace := trace - 1;
				report "TBCD ST0 +" & integer'image(4 - trace)
					& ": CE=" & std_logic'image(c_ce)
					& " EN=" & std_logic'image(c_en)
					& " STATE=" & integer'image(to_integer(c_state))
					& " SLEEP=" & std_logic'image(c_slp);
			end if;
			if c_slp = '1' then
				if c_en = '1' then
					ss_en_leak <= ss_en_leak + 1;
					report "TBCD ST0: the CPU was ENABLED while SLEEP was asserted"
						& " (STATE=" & integer'image(to_integer(c_state))
						& ", frozen at " & integer'image(st_frz)
						& ") t=" & time'image(now) severity error;
				end if;
				if st_frz >= 0 and to_integer(c_state) /= st_frz then
					ss_state_bad <= ss_state_bad + 1;
					report "TBCD ST0: STATE moved AFTER the freeze: "
						& integer'image(st_frz) & " -> "
						& integer'image(to_integer(c_state))
						& " t=" & time'image(now) severity error;
					st_frz := to_integer(c_state);
				end if;
			end if;
			slp_old := c_slp;
		end loop;
	end process;

	h_diag : process
		alias d_we    is << signal .tb_cd.DUT.CD.ADRAM_WE : std_logic >>;
		alias d_adr   is << signal .tb_cd.DUT.CD.ADRAM_A  : std_logic_vector(16 downto 0) >>;
		alias d_di    is << signal .tb_cd.DUT.CD.ADRAM_DI : std_logic_vector(3 downto 0) >>;
		alias d_slp   is << signal .tb_cd.DUT.CD.SLEEP    : std_logic >>;
		alias d_reqn  is << signal .tb_cd.DUT.CD.SCSI_REQ_N : std_logic >>;
		alias d_ackn  is << signal .tb_cd.DUT.CD.SCSI_ACK_N : std_logic >>;
		alias d_auto  is << signal .tb_cd.DUT.CD.AUTO_ACK : std_logic >>;
		alias d_pend  is << signal .tb_cd.DUT.CD.DMA_WRITE_PEND : std_logic >>;
		alias d_nib   is << signal .tb_cd.DUT.CD.ADPCM_WRITE_NIB : std_logic >>;
		alias d_dbo   is << signal .tb_cd.DUT.CD.SCSI_DBO : std_logic_vector(7 downto 0) >>;
		alias d_wrd   is << signal .tb_cd.DUT.CD.ADPCM_WRDATA : std_logic_vector(7 downto 0) >>;
		alias d_wra   is << signal .tb_cd.DUT.CD.ADPCM_WRADDR : std_logic_vector(16 downto 0) >>;
		alias d_held  is << signal .tb_cd.DUT.CD.SCSI_DATAIN_HELD : std_logic >>;
		alias d_skip  is << signal .tb_cd.DUT.CD.SCSI.READ_SKIP : unsigned(11 downto 0) >>;
		alias d_sp    is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		alias d_cons  is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		variable slp_old : std_logic := '0';
		variable shot    : boolean := false;
	begin
		loop
			wait until rising_edge(clk);
			if h_arm = '1' and d_slp = '1' and slp_old = '0' and h_mark = '0' then
				report "TBCD H DIAG @save-freeze:"
					& " WRADDR=" & integer'image(to_integer(unsigned(d_wra)))
					& " CONS=" & integer'image(to_integer(d_cons))
					& " SP=" & to_hstring(d_sp)
					& " REQ_N=" & std_logic'image(d_reqn)
					& " ACK_N=" & std_logic'image(d_ackn)
					& " AUTO_ACK=" & std_logic'image(d_auto)
					& " DMA_PEND=" & std_logic'image(d_pend)
					& " WR_NIB=" & std_logic'image(d_nib)
					& " DBO=" & to_hstring(d_dbo)
					& " WRDATA=" & to_hstring(d_wrd);
			end if;
			slp_old := d_slp;

			if h_mark = '1' and not shot and d_slp = '0' and d_we = '1' then
				shot := true;
				report "TBCD H DIAG @first-write-after-load:"
					& " addr=" & integer'image(to_integer(unsigned(d_adr)))
					& " nib=" & to_hstring(d_di)
					& " WRADDR=" & integer'image(to_integer(unsigned(d_wra)))
					& " CONS=" & integer'image(to_integer(d_cons))
					& " SP=" & to_hstring(d_sp)
					& " HELD=" & std_logic'image(d_held)
					& " READ_SKIP=" & integer'image(to_integer(d_skip))
					& " REQ_N=" & std_logic'image(d_reqn)
					& " ACK_N=" & std_logic'image(d_ackn)
					& " AUTO_ACK=" & std_logic'image(d_auto)
					& " DMA_PEND=" & std_logic'image(d_pend)
					& " DBO=" & to_hstring(d_dbo)
					& " WRDATA=" & to_hstring(d_wrd);
			end if;
		end loop;
	end process;

	e_mon : process
		-- SP itself has an architecture-private type, so the phase is read off
		-- SP_ENC (sp_encode order: SP_DATAIN_START = 10 = 0xA)
		alias dtr      is << signal .tb_cd.DUT.CD.CD_DTR : std_logic >>;
		alias dcnt     is << signal .tb_cd.DUT.CD.CD_DATA_CNT : unsigned(10 downto 0) >>;
		alias rcons    is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		alias ract     is << signal .tb_cd.DUT.CD.SCSI.READ_ACTIVE : std_logic >>;
		alias held     is << signal .tb_cd.DUT.CD.SCSI_DATAIN_HELD : std_logic >>;
		alias ackn     is << signal .tb_cd.DUT.CD.SCSI_ACK_N : std_logic >>;
		alias spenc    is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		variable nbad : integer := 0;
		variable nok  : integer := 0;
	begin
		loop
			wait until rising_edge(clk);
			if ract = '1' and held = '0' and ackn = '1' and spenc = x"A"
			   and dtr = '1' then
				if to_integer(dcnt) /= (to_integer(rcons) mod 2048) then
					nbad := nbad + 1;
					if nbad <= 8 then
						report "TBCD E: CD_DATA_CNT=" & integer'image(to_integer(dcnt))
							& " but READ_CONSUMED mod 2048="
							& integer'image(to_integer(rcons) mod 2048)
							& " t=" & time'image(now) severity error;
					end if;
					e_fail <= '1';
				else
					nok := nok + 1;
				end if;
			end if;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- DDR model + scenario driver
	-- ------------------------------------------------------------------
	ddr_and_stim : block
		shared variable mem  : mem_t := (others => (others => '0'));
		shared variable slog : slog_t := (others => 0);
		-- when each tape entry reached the DAC, and the last SLEEP release then
		shared variable stime : stime_t := (others => 0 ns);
		shared variable srel  : stime_t := (others => 0 ns);
		-- BRAM contents (seeded with a deterministic pattern at t=0; the CPU
		-- program never touches page $F7, so it stays put until the save reads
		-- it and the load rewrites it)
		type brm_t is array (0 to 2047) of std_logic_vector(7 downto 0);
		function brm_init return brm_t is
			variable b : brm_t;
		begin
			for i in b'range loop
				b(i) := std_logic_vector(to_unsigned((i*37 + 11) mod 256, 8));
			end loop;
			return b;
		end function;
		shared variable brm : brm_t := brm_init;
		impure function slot_word(slot, w : integer) return std_logic_vector is
		begin
			return mem(slot*SLOT_WORDS + w);
		end function;
	begin
		process(clk)
			variable a, slot, w : integer;
		begin
			if rising_edge(clk) then
				ddr_done <= '0';
				if ddr_ena = '1' then
					a := to_integer(unsigned(ddr_adr));
					assert a >= BASE_DW and a < BASE_DW + 4*SLOT_DW
						report "DDR access outside 4-slot window" severity failure;
					slot := (a - BASE_DW) / SLOT_DW;
					w    := ((a - BASE_DW) mod SLOT_DW) / 2;
					assert w < SLOT_WORDS report "word index out of range" severity failure;
					if ddr_rnw = '0' then
						mem(slot*SLOT_WORDS + w) := ddr_din;
					else
						ddr_dout <= mem(slot*SLOT_WORDS + w);
					end if;
					ddr_done <= '1';
				end if;
			end if;
		end process;

		-- CDDA sample monitor: log every OUTL change (the pattern makes
		-- L=R=frame index, so a log of L values is the played-position tape)
		sample_mon : process(cdda_sl)
		begin
			if slog_count < slog_t'length - 1 then
				slog(slog_count) := to_integer(unsigned(std_logic_vector(cdda_sl)));
				stime(slog_count) := now;
				srel(slog_count) := rel_t;
				slog_count <= slog_count + 1;
			end if;
		end process;

		-- BRAM block-RAM model (registered read, like the SV dpram port A the
		-- savestate walk borrows during the freeze).  During SAVE the engine
		-- drives brm_a=Save_RAMAddr and samples brm_do after priming; during
		-- LOAD it drives brm_a/brm_di/brm_we for region 7.
		brm_model : process(clk)
		begin
			if rising_edge(clk) then
				if brm_we = '1' then
					brm(to_integer(unsigned(brm_a))) := brm_di;
				end if;
				brm_do <= brm(to_integer(unsigned(brm_a)));
			end if;
		end process;

		stim : process
			procedure wait_cycles(n : integer) is
			begin
				for i in 1 to n loop wait until rising_edge(clk); end loop;
			end procedure;
			-- Freeze deadline in 5 ms steps (default 4 s). W3 tightens this:
			-- AUD_OPEN may defer by ~G_W3_D8LAT_MS, but must eventually release.
			procedure do_ss(slot : integer; is_load : boolean; tag : string;
			                max_wait : integer := 800) is
				variable t0 : time;
			begin
				ss_slot <= std_logic_vector(to_unsigned(slot, 2));
				wait_cycles(2);
				if is_load then ss_load <= '1'; else ss_save <= '1'; end if;
				wait_cycles(3);
				ss_load <= '0'; ss_save <= '0';
				t0 := now;
				-- Wait for freeze, reporting elapsed time every 5 sim-ms.
				for i in 1 to max_wait loop
					wait until ss_sleep = '1' for 5 ms;
					exit when ss_sleep = '1';
					report "TBCD WAIT(" & tag & ") t=" & time'image(now - t0);
				end loop;
				assert ss_sleep = '1'
					report "TBCD HANG(" & tag & "): freeze never engaged" severity failure;
				wait until ss_sleep = '0' for 500 ms;
				assert ss_sleep = '0'
					report "TBCD HANG(" & tag & "): walk/refetch never completed" severity failure;
				report "TBCD: " & tag & " ok (" & time'image(now - t0) & ")";
				if not is_load then
					-- eReg 0 = CPU_1: bits 52:48 STATE, 47:40 IR.  A blob with
					-- STATE /= 0 was captured mid-instruction and cannot be
					-- restored: the mid-instruction internals are not saved.
					if slot_word(slot, 1)(52 downto 48) /= "00000" then
						ss_blob_bad <= ss_blob_bad + 1;
						report "TBCD ST0: blob from '" & tag & "' has CPU STATE="
							& integer'image(to_integer(unsigned(slot_word(slot, 1)(52 downto 48))))
							& " IR=" & to_hstring(slot_word(slot, 1)(47 downto 40))
							& " - captured MID-INSTRUCTION" severity error;
					end if;
				end if;
				if is_load then
					-- tight PC trace right after resume: a wild PC here is a
					-- restore bug, and the first samples show the divergence
					for k in 1 to 12 loop
						wait for 2 us;
						report "RESUME(" & tag & ") +" & integer'image(k*2) & "us PC="
							& to_hstring(<< signal .tb_cd.DUT.CPU.CORE.PC : std_logic_vector(15 downto 0) >>);
					end loop;
				end if;
				wait_cycles(10);
			end procedure;
			-- A resumed CDDA stream must restart with the machine: the release
			-- that wakes CPU and video and the first resumed sample at the DAC
			-- (tape entry idx) lie within one CDDA sample period.
			procedure check_sync(tag : string; idx : integer) is
				variable gap : time;
			begin
				gap := stime(idx) - srel(idx);
				report "TBCD " & tag & " SYNC: release at " & time'image(srel(idx))
					& ", first resumed sample at " & time'image(stime(idx))
					& ", gap " & integer'image(gap / CLK_PERIOD) & " clocks (one CDDA sample = "
					& integer'image(CDDA_CLKS) & " clocks)";
				assert srel(idx) > 0 ns and gap <= CDDA_CLKS * CLK_PERIOD
					report "TBCD " & tag & " FAILED: the first resumed CDDA sample reaches the "
						& "DAC " & time'image(gap) & " after the CPU/video release (limit "
						& time'image(CDDA_CLKS * CLK_PERIOD) & ", one sample)" severity failure;
			end procedure;
			-- With a modelled seek (G_PLAY_LAT_MS > 0): the replay D8 carries
			-- the fast-seek bit, so the first resumed sample (tape entry idx)
			-- follows the end of the load walk (t_ld) by the replay's own
			-- command gaps, not by the seek.
			procedure check_seek(tag : string; idx : integer; t_ld : time) is
			begin
				report "TBCD " & tag & " SEEK: first resumed sample "
					& time'image(stime(idx) - t_ld) & " after the load walk (modelled seek "
					& integer'image(G_PLAY_LAT_MS) & " ms)";
				assert G_PLAY_LAT_MS = 0 or stime(idx) - t_ld < G_PLAY_LAT_MS * 1 ms
					report "TBCD " & tag & " FAILED: the first resumed sample follows the load by "
						& time'image(stime(idx) - t_ld) & ", which includes the modelled "
						& integer'image(G_PLAY_LAT_MS) & " ms seek" severity failure;
			end procedure;
			-- scenario H watches the CPU's own "transfer finished" signal: the
			-- routine at $FD00 clears $180B only after it has taken the STATUS
			alias dma_en_hw is << signal .tb_cd.DUT.CD.ADPCM_DMA_EN : std_logic >>;
			-- scenario W steers its saves off the live SCSI state
			alias w_cons is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
			alias w_sp   is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
			alias w_stp  is << signal .tb_cd.DUT.CD.SCSI.STAT_PEND : std_logic >>;
			alias w_ract is << signal .tb_cd.DUT.CD.SCSI.READ_ACTIVE : std_logic >>;
			alias w_aud  is << signal .tb_cd.DUT.CD.SCSI.AUDIO_ACTIVE : std_logic >>;
			alias w_swl  is << signal .tb_cd.DUT.CD.SCSI.SWALLOW : unsigned(1 downto 0) >>;
			alias w_din  is << signal .tb_cd.DUT.CD.SCSI.DATAIN_CNT : unsigned(15 downto 0) >>;
			-- scenario K gates on the CORE's CDDA consumption, not on the drive
			-- model's own end-of-track: the model closes the window as soon as
			-- ITS position reaches the end point, tens of ms before the core has
			-- drained the samples, and a save taken then is an ordinary mid-track
			-- resume.  CDDA_CONSUMED is the live count the replay latches into
			-- RP_CONS and divides by 588 to rebuild the position, so it is the
			-- same number the RTL decides on.
			alias k_cons is << signal .tb_cd.DUT.CD.CDDA_CONSUMED : unsigned(31 downto 0) >>;
			alias j_pc is << signal .tb_cd.DUT.CPU.CORE.PC : std_logic_vector(15 downto 0) >>;
			-- J/J2 handled-interrupt counter out of the latest slot-3 blob:
			-- $2030 (IRQ1, byte 0) or $2033 (IRQ2, byte 3) of region word 129+6
			impure function j_cnt return integer is
			begin
				if G_J2 then
					return to_integer(unsigned(slot_word(3, 135)(31 downto 24)));
				else
					return to_integer(unsigned(slot_word(3, 135)(7 downto 0)));
				end if;
			end function;
			-- J2's source runs at ~7.35 kHz against J's ~1.3 kHz degenerate VBL:
			-- the window must keep the 8-bit counter well under a wrap
			procedure j_wait is
			begin
				if G_J2 then wait for 10 ms; else wait for 60 ms; end if;
			end procedure;
			variable diff    : integer;
			variable t_dead  : time;
			variable w3_d8base : integer := 0;
			variable still   : integer := 0;
			variable v       : std_logic_vector(63 downto 0);
			variable cons_a  : integer;
			variable anchor  : integer;
			variable werr    : integer;
			variable nf      : integer;
			variable wrapa   : integer;
			variable fl_tmp  : flushlog_t;
			variable fl_swp  : integer;
			variable wrapc   : integer;
			variable cons_g  : integer;
			variable t_ld    : time;	-- end of the last load walk
			variable mark    : integer;
			variable k_int_base : integer;	-- scenario K: completions before the save
			variable req_base, ack_base, cons_base : integer;	-- scenario L: landmarks before save
			variable k_ack_base : integer;	-- scenario K: program ACKs before the load
			variable exp, got : integer;
			variable idx      : integer;
		begin
			wait_cycles(20);
			reset <= '0';
			-- armed from the start: no ADPCM RAM write can happen before the DMA
			-- routine runs (the program never writes $180A), and the walk's own
			-- writes are excluded by the SLEEP gate inside h_mon
			h_arm <= '1';
			-- Anchor A during boot's verified reads to exercise mid-READ save/load.
			wait for 5 ms;

			----------------------------------------------------------------
			if G_DO_A then
			report "TBCD: === scenario A: exec-from-CD-RAM determinism ===";
			-- Two identical load->run->save cycles from the same anchor: the
			-- replay window delays the save by a fixed amount in BOTH cycles,
			-- so the two replay blobs must be BIT-IDENTICAL.  Any live state
			-- that the restore does not rebuild (the CD-RAM class) differs
			-- between the two loads and shows up as a diff.
			----------------------------------------------------------------
			do_ss(0, false, "A anchor save");
			wait_cycles(200_000);
			do_ss(0, true,  "A load #1");
			wait_cycles(200_000);
			do_ss(1, false, "A replay save #1");
			wait_cycles(100_000);
			do_ss(0, true,  "A load #2");
			wait_cycles(200_000);
			do_ss(2, false, "A replay save #2");

			diff := 0;
			for w in 1 to SLOT_WORDS-1 loop
				if w = 63 then
					-- CD_5: mask OUTL (50:35) + CDDA sample-phase bits (11:10):
					-- the drive stream is not phase-locked to the machine (nor
					-- is the real HPS), so the DAC sample at freeze may differ
					if (slot_word(1, w) and x"FFF80007FFFFF3FF")
					 /= (slot_word(2, w) and x"FFF80007FFFFF3FF") then
						diff := diff + 1;
						report "TBCD A: CD_5 non-DAC fields differ" severity error;
					end if;
				elsif w = 64 then
					-- CD_6: mask OUTR (15:0), same reason
					if (slot_word(1, w) and x"FFFFFFFFFFFF0000")
					 /= (slot_word(2, w) and x"FFFFFFFFFFFF0000") then
						diff := diff + 1;
						report "TBCD A: CD_6 non-DAC fields differ" severity error;
					end if;
				elsif w = 25 then
					if (slot_word(1, w) and x"FFF88FFFFFFFF000")
					 /= (slot_word(2, w) and x"FFF88FFFFFFFF000") then
						diff := diff + 1;
					end if;
				elsif w = 56 then
					-- SCSI_1: this comparison excludes audio-response bookkeeping
					-- (AUD_OPEN and AUD_LAST in bits 62:60).
					if (slot_word(1, w) and x"0FFFFFFFFFFFFFFF")
					 /= (slot_word(2, w) and x"0FFFFFFFFFFFFFFF") then
						diff := diff + 1;
						report "TBCD A: SCSI_1 compared fields differ" severity error;
					end if;
				elsif slot_word(1, w) /= slot_word(2, w) then
					diff := diff + 1;
					if diff <= 12 then
						report "TBCD A: word " & integer'image(w) & " differs: ref="
							& to_hstring(slot_word(1, w)) & " replay=" & to_hstring(slot_word(2, w))
							severity error;
					end if;
				end if;
			end loop;
			assert diff = 0
				report "TBCD A FAILED: " & integer'image(diff) & " differing words" severity failure;
			-- anchor-vs-replay on regions the program NEVER writes: a blob-
			-- to-blob compare alone cannot see a SYSTEMATIC save/restore
			-- shift (e.g. the ADPCM nibble shuttle corrupting both replays
			-- identically).  ADPCM RAM (region 9) must round-trip bit-exact
			-- against the anchor.  Region base = 129 + WRAM4096 + VRAM0 8192
			-- + SAT0 64 + PAL128 + VRAM1 8192 + SAT1 64 + PRAM4096 + BRM256
			-- + PSGWF24 = 25241. This program leaves ADPCM RAM zeroed, so A2
			-- checks deterministic transfer, not a patterned-data round-trip.
			diff := 0;
			for w in 25241 to 25241 + 8191 loop
				if slot_word(0, w) /= slot_word(1, w) then
					diff := diff + 1;
					if diff <= 8 then
						report "TBCD A2: ADPCM word " & integer'image(w - 25241)
							& " anchor=" & to_hstring(slot_word(0, w))
							& " replay=" & to_hstring(slot_word(1, w)) severity error;
					end if;
				end if;
			end loop;
			assert diff = 0
				report "TBCD A2 FAILED: ADPCM region does not round-trip ("
					& integer'image(diff) & " words)" severity failure;
			report "TBCD: scenario A PASSED (incl. ADPCM round-trip)";
			end if;  -- G_DO_A

			----------------------------------------------------------------
			if G_DO_W or G_DO_W2 or G_DO_W3 then
			report "TBCD: === scenario W: push-status READ6, save/load mid-stream and "
				& "in the tail window, then a FRESH READ6 ===";
			-- Requires G_PUSH_STATUS=true (the model strobes the READ6 STATUS on
			-- the last PUSHED byte) and w_hold_stat, which started asserted, is
			-- keeping read #1's final STATUS withheld so both save windows stay
			-- open.  The boot program is running its verified READ6 (lba 0x400,
			-- 2 sectors); reuse it as the streaming read under test.
			--
			-- Cycle 1 - mid-stream: save with the reader some hundreds of bytes
			-- into the transfer (RP_RD_PREP sect<cnt branch, partial skip).
			----------------------------------------------------------------
			assert G_PUSH_STATUS or G_REAL_HPS
				report "TBCD W FAILED: scenario W without G_PUSH_STATUS/G_REAL_HPS "
					& "tests nothing" severity failure;
			t_dead := now + 60 ms;
			while to_integer(w_cons) < 600 and now < t_dead loop
				wait for 100 us;
			end loop;
			assert to_integer(w_cons) >= 600
				report "TBCD W FAILED: setup - the boot READ6 never streamed (CONS="
					& integer'image(to_integer(w_cons)) & ")" severity failure;
			do_ss(3, false, "W save #1 mid-read");
			assert h_ract_frz = '1'
				report "TBCD W FAILED: setup - save #1 landed with no READ in flight"
				severity failure;
			wait_cycles(50_000);
			do_ss(3, true, "W load #1 mid-read");
			-- the reader must now carry the replayed stream byte-exact to the END
			-- of the transfer (scenario-C style verdict: any drift trips $FE58 ->
			-- c_fail); "the end" here is every data byte consumed while the final
			-- STATUS is still withheld - which is exactly the TAIL WINDOW.
			t_dead := now + 200 ms;
			while (to_integer(w_cons) < 4096 or w_sp /= x"0") and now < t_dead loop
				wait for 100 us;
			end loop;
			assert to_integer(w_cons) >= 4096 and w_sp = x"0"
				report "TBCD W FAILED: the stream did not complete after load #1 (CONS="
					& integer'image(to_integer(w_cons)) & " SP=" & to_hstring(w_sp) & ")"
				severity failure;
			assert c_fail = '0'
				report "TBCD W FAILED: stream mismatch after load #1 (reader hit $FE58)"
				severity failure;
			report "TBCD W: cycle 1 clean, tail window open (CONS="
				& integer'image(to_integer(w_cons)) & " RDACT=" & std_logic'image(w_ract)
				& " STAT_PEND=" & std_logic'image(w_stp) & ")";
			wait for 200 us;

			----------------------------------------------------------------
			-- Cycle 2 - the tail window: every data byte
			-- consumed, READ_ACTIVE still set, STATUS not yet strobed.  The
			-- replay takes RP_RD_PREP's sect>=cnt branch: re-read the LAST
			-- sector with RP_RSKIP=2048 and drain it WHOLE - the CPU is owed
			-- nothing, so anything the drain leaves staged is an orphan.
			----------------------------------------------------------------
			assert w_ract = '1' and w_stp = '0'
				report "TBCD W FAILED: setup - the tail window is not open (RDACT="
					& std_logic'image(w_ract) & " STAT_PEND=" & std_logic'image(w_stp)
					& "), the save below would not exercise the sect>=cnt branch"
				severity failure;
			do_ss(3, false, "W save #2 tail-window");
			assert h_ract_frz = '1'
				report "TBCD W FAILED: setup - save #2 lost the tail window (READ no "
					& "longer active at the freeze)" severity failure;
			-- let the post-save timeline take its STATUS and close read #1 (the
			-- hardware order: the HPS strobe lands after the save).  The load
			-- below then drops whatever the dead timeline had in flight at the
			-- model (the load-idle rule in the hps process).
			w_hold_stat <= '0';
			if w_reads_done < 1 then
				wait until w_reads_done >= 1 for 60 ms;
			end if;
			assert w_reads_done >= 1
				report "TBCD W FAILED: read #1 never closed once the withheld STATUS "
					& "was released" severity failure;
			do_ss(3, true, "W load #2 tail-window");
			-- Snapshot EVERY landmark counter the post-load waits rely on - and
			-- snapshot them AFTER the walk, not before the do_ss.  The dead
			-- (post-save) timeline keeps running until the load freeze actually
			-- lands - measured >25 ms of runway, enough to close the fresh
			-- READ6 and even the D8/D9 pair - and those increments happen
			-- DURING do_ss's boundary wait, so a pre-do_ss snapshot misses them
			-- and the post-load wait passes without restored execution.
			-- After the walk the counters are final: the machine was frozen,
			-- and the restored timeline needs the whole replay (~25 ms) before
			-- it can possibly fire a marker of its own.
			exp    := w_reads_done;
			cons_a := w_fresh_done;
			cons_g := w2_audio;
			w3_d8base := hps_d8_count;
			-- replay: re-read of the last sector, 2048-byte drain, then the model
			-- strobes the STATUS G_STAT_TICKS after its last push.  The restored
			-- program - parked in the reader's REQ poll waiting for STATUS - must
			-- take exactly STATUS+MSGIN and close; any DATAIN byte it is offered
			-- in between is stale and trips $FE58.
			t_dead := now + 200 ms;
			while w_reads_done <= exp and now < t_dead loop
				wait for 100 us;
			end loop;
			assert w_reads_done > exp
				report "TBCD W FAILED: the tail-window replay never delivered a STATUS "
					& "the program could close on (c_fail=" & std_logic'image(c_fail)
					& " CONS=" & integer'image(to_integer(w_cons))
					& " SP=" & to_hstring(w_sp) & ")" severity failure;
			assert c_fail = '0'
				report "TBCD W FAILED: a stale byte was delivered around the tail-window "
					& "drain (reader hit $FE58 - REPRO, see the MISMATCH/W BYTE reports)"
				severity failure;
			report "TBCD W: cycle 2 (tail window) closed clean, fresh READ6 next";

			----------------------------------------------------------------
			-- Step d - the FRESH READ6 (lba 0x900, a transaction the replay has
			-- never touched).  Any prepended stale byte = mismatch at $FA58; a
			-- retry loop = more than 2 fresh-lba commands at the model; then the
			-- STATUS and MSGIN bytes must be taken and the transaction closed.
			----------------------------------------------------------------
			mark := w_read6_cnt;	-- the pre-load timeline may have issued one
			-- cons_a is the fresh-read completion count snapshot from BEFORE the
			-- load: the dead timeline closed one fresh READ6 in its >25 ms of
			-- runway, so only a count ABOVE the snapshot is the restored one
			t_dead := now + 150 ms;
			while w_fresh_done <= cons_a and now < t_dead loop
				wait for 100 us;
			end loop;
			assert w_read6_cnt - mark <= 2
				report "TBCD W FAILED: retry loop - the program issued the fresh READ6 "
					& integer'image(w_read6_cnt - mark) & " times after the load"
				severity failure;
			assert w_fresh_done > cons_a
				report "TBCD W FAILED: the fresh READ6 conversation never closed "
					& "(issued " & integer'image(w_read6_cnt - mark)
					& "x after the load, w_fail=" & std_logic'image(w_fail)
					& " c_fail=" & std_logic'image(c_fail)
					& " SP=" & to_hstring(w_sp) & ")" severity failure;
			assert w_fail = '0'
				report "TBCD W FAILED: data mismatch on the fresh READ6 - a stale byte "
					& "prepended the new command's data phase (REPRO, see the W "
					& "MISMATCH report)" severity failure;
			assert c_fail = '0'
				report "TBCD W FAILED: the boot reader's mismatch flag was raised "
					& "during W" severity failure;
			if not (G_DO_W2 or G_DO_W3) then
				report "TBCD W PASSED";
				std.env.finish;
			end if;

			if G_DO_W2 then
			----------------------------------------------------------------
			-- W2 transition: after READ6 #1, run D8+D9, save/load in the
			-- busy-wait audio window, then verify two back-to-back READ6s.
			----------------------------------------------------------------
			report "TBCD W2: === stage-0 transition: D8/D9 + audio-window save/load "
				& "+ back-to-back READ6s ===";
			-- cons_g = w2_audio snapshot from before load #2: the dead timeline
			-- already fired the audio marker once, so waiting for ">= 1" here
			-- would trigger save #3 while the restored timeline is still in the
			-- replay - only a count ABOVE the snapshot is the restored D8/D9
			t_dead := now + 100 ms;
			while w2_audio <= cons_g and now < t_dead loop
				wait for 100 us;
			end loop;
			assert w2_audio > cons_g
				report "TBCD W2 FAILED: the D8/D9 pair never closed - the transition "
					& "wedged before audio was even commanded (SP=" & to_hstring(w_sp)
					& " STAT_PEND=" & std_logic'image(w_stp) & ")" severity failure;
			-- save #3 in the AUDIO window: the program is in its ~45 ms busy-wait
			-- with AUDIO_ACTIVE=1 and the model streaming CDDA
			do_ss(3, false, "W2 save #3 audio-window");
			assert w_aud = '1'
				report "TBCD W2 FAILED: setup - save #3 missed the audio window "
					& "(AUDIO_ACTIVE=0 right after the save)" severity failure;
			wait_cycles(50_000);
			do_ss(3, true, "W2 load #3 audio-window");
			-- baselines AFTER the walk (see the load #2 comment): the post-save
			-- dead timeline may run all the way into READ6 #2/#3 during the
			-- boundary wait, so only counts above these post-walk snapshots
			-- prove the RESTORED timeline made it through
			idx := w2_read2;
			got := w2_end;
			-- replay re-issues D8 (repositioned) + D9 (verbatim) and swallows
			-- both statuses; the program finishes its busy-wait remainder and
			-- SELs READ6 #2.
			t_dead := now + 250 ms;
			while w2_read2 <= idx and now < t_dead loop
				wait for 100 us;
			end loop;
			assert w2_read2 > idx
				report "TBCD W2 FAILED: READ6 #2 conversation never closed after the "
					& "audio-window load - the program is parked, the hardware "
					& "wait-for-STATUS symptom (w_fail=" & std_logic'image(w_fail)
					& " c_fail=" & std_logic'image(c_fail)
					& " SP=" & to_hstring(w_sp)
					& " STAT_PEND=" & std_logic'image(w_stp) & ")" severity failure;
			assert w_fail = '0'
				report "TBCD W2 FAILED: data mismatch in a W reader during the "
					& "transition (see the W MISMATCH report)" severity failure;
			t_dead := now + 250 ms;
			while w2_end <= got and now < t_dead loop
				wait for 100 us;
			end loop;
			assert w2_end > got
				report "TBCD W2 FAILED: READ6 #3 / the end marker was never reached "
					& "(SP=" & to_hstring(w_sp) & " STAT_PEND=" & std_logic'image(w_stp)
					& ")" severity failure;
			assert c_fail = '0'
				report "TBCD W2 FAILED: a boot-reader mismatch was recorded during "
					& "the transition (READ6 #3 stream misaligned)" severity failure;
			assert w_fail = '0'
				report "TBCD W2 FAILED: a W-reader mismatch was recorded during the "
					& "transition" severity failure;
			report "TBCD W2 PASSED";
			std.env.finish;
			end if;  -- G_DO_W2

			----------------------------------------------------------------
			-- W3 requests a save while D8 status is held in the deferred slot
			-- during seek. AUD_OPEN must defer freeze until that status arrives,
			-- preventing replay D8 from overwriting the game's pending status.
			----------------------------------------------------------------
			report "TBCD W3: === stage-0 transition on the REAL HPS model "
				& "(single-slot deferred status + seek latency) ===";
			t_dead := now + 400 ms;
			while hps_d8_count <= w3_d8base and now < t_dead loop
				wait for 100 us;
			end loop;
			assert hps_d8_count > w3_d8base
				report "TBCD W3 FAILED: the game's D8 never reached the drive "
					& "(wedged before the transition; SP=" & to_hstring(w_sp)
					& " STAT_PEND=" & std_logic'image(w_stp) & ")" severity failure;
			-- Request during the pending D8 seek; expect AUD_OPEN to defer
			-- freeze until status arrives. The deadline is 100 * 5 ms = 500 ms.
			assert hps_slot = '1'
				report "TBCD W3 FAILED: setup - the D8 status was already "
					& "delivered before the save was even requested; raise "
					& "G_W3_D8LAT_MS" severity failure;
			do_ss(3, false, "W3 save #3 D8-seek-window", 100);
			assert w_frz_slot = '0' and w_frz_audopen = '0'
				report "TBCD W3 FAILED: the freeze landed INSIDE the audio window "
					& "(at-freeze SLOT=" & std_logic'image(w_frz_slot)
					& " AUD_OPEN=" & std_logic'image(w_frz_audopen)
					& ") - AUD_OPEN veto did not hold the boundary"
				severity failure;
			report "TBCD W3: save #3 was requested in the D8 window and correctly "
				& "DEFERRED past it (at-freeze SLOT='0'/AUD_OPEN='0')";
			wait_cycles(50_000);
			do_ss(3, true, "W3 load #3 D8-seek-window");
			-- Baseline after the walk. AUD_OPEN defers freeze past D8/D9 and
			-- the $FAE0 marker, so the next restored landmark is READ6 #2.
			cons_a := w2_read2;
			got    := w2_end;
			-- Allow busy-wait, serialized replay D8 status, D9, game SEL,
			-- G_SEEK_MS seek, 16 ms cadence and ~21 ms consumption (~300 ms).
			-- Fail early only after 200 ms without DATAIN progress or pending
			-- status. After the replay's statuses the quiet interval is the
			-- game's own wait at $FAEx (about 93 ms; the post-load hold runs
			-- it after the replay D8 instead of during it), then the READ6 #2
			-- seek (G_SEEK_MS) and the first 16 ms cadence. Replay
			-- serialization remains exempt while SWALLOW is nonzero. A parked
			-- transition still fails below, at the latest by the deadline.
			t_dead := now + 450 ms;
			still  := 0;
			idx    := to_integer(w_din);
			while w2_read2 <= cons_a and now < t_dead loop
				wait for 100 us;
				if hps_slot = '0' and w_stp = '0' and to_integer(w_swl) = 0
				   and to_integer(w_din) = idx then
					still := still + 1;
				else
					still := 0;
					idx := to_integer(w_din);
				end if;
				exit when still >= 2000;	-- 200 ms of proven stillness
			end loop;
			if w2_read2 <= cons_a then
				report "TBCD W3 REPRO: transition parked - statuses LOST by slot "
					& "overwrite = " & integer'image(w3_lost)
					& "; slot=" & std_logic'image(hps_slot)
					& " SWALLOW=" & integer'image(to_integer(w_swl))
					& " STAT_PEND=" & std_logic'image(w_stp)
					& " SP=" & to_hstring(w_sp)
					& " DATAIN_CNT=" & integer'image(to_integer(w_din));
				assert false
					report "TBCD W3 FAILED: READ6 #2 never closed after the "
						& "audio-window load - the transition is parked in its "
						& "status/data wait.  Statuses lost="
						& integer'image(w3_lost) severity failure;
			end if;
			t_dead := now + 450 ms;
			while w2_end <= got and now < t_dead loop
				wait for 100 us;
			end loop;
			assert w2_end > got
				report "TBCD W3 FAILED: READ6 #3 / the end marker was never reached "
					& "(SP=" & to_hstring(w_sp) & " STAT_PEND="
					& std_logic'image(w_stp) & " lost=" & integer'image(w3_lost)
					& ")" severity failure;
			assert c_fail = '0' and w_fail = '0'
				report "TBCD W3 FAILED: a reader mismatch was recorded during the "
					& "transition" severity failure;
			assert w3_lost = 0
				report "TBCD W3 FAILED: " & integer'image(w3_lost) & " status(es) "
					& "were lost to slot overwrite even though the program "
					& "recovered - latent on this timing, fatal on another"
				severity failure;
			report "TBCD W3 PASSED";
			std.env.finish;
			end if;  -- G_DO_W / G_DO_W2 / G_DO_W3

			----------------------------------------------------------------
			if G_DO_H then
			report "TBCD: === scenario H: ADPCM DMA (AD_TRANS) across a save/load ===";
			-- $FD00 enables DMA and issues G_H_SECT sectors (0 means 256).
			-- Save at H_TRIG, 30% through the transfer and inside a sector.
			----------------------------------------------------------------
			if h_nib < H_TRIG then
				wait until h_nib >= H_TRIG for 600 ms;
			end if;
			assert h_nib >= H_TRIG
				report "TBCD H SETUP FAILED: the ADPCM DMA never streamed (h_nib="
					& integer'image(h_nib) & ").  Either the program never reached "
					& "$FD00 or ADPCM_DMA_EN never took a byte off the bus."
				severity failure;
			report "TBCD H: DMA running, " & integer'image(h_nib)
				& " nibbles written so far";
			-- drop whatever scenario A's own loads did to the ADPCM address, then
			-- take a clean baseline: from here to the end of the scenario the ONLY
			-- legal discontinuity is the one H's own load introduces
			h_clr <= '1';
			wait_cycles(4);
			h_clr <= '0';
			wait_cycles(20_000);
			assert h_nib > 0
				report "TBCD H SETUP FAILED: the DMA stopped writing right after the "
					& "baseline was taken" severity failure;
			assert h_jumps = 0
				report "TBCD H SETUP FAILED: the DMA stream is discontinuous with no "
					& "savestate in flight (" & integer'image(h_jumps) & " jumps)"
				severity failure;

			do_ss(3, false, "H save mid-DMA");
			assert h_ract_frz = '1'
				report "TBCD H SETUP FAILED: the READ had already finished when the "
					& "machine froze, so nothing mid-DMA was saved.  Raise G_H_SECT "
					& "or lower the H_TRIG fraction."
				severity failure;

			wait_cycles(50_000);
			do_ss(3, true, "H load mid-DMA");
			h_mark <= '1';

			-- the replay re-issues the READ6 and the restored DMA has to carry it
			-- to the end; the routine clears $180B only when it gets its STATUS
			if dma_en_hw = '1' then
				wait until dma_en_hw = '0' for 600 ms;
			end if;
			assert dma_en_hw = '0'
				report "TBCD H FAILED: the DMA transfer never completed after the load "
					& "-- the program is still sitting in its own loading routine, which "
					& "is exactly the hardware symptom.  h_nib=" & integer'image(h_nib)
					& " after-load=" & integer'image(h_after)
				severity failure;

			assert h_after > 0
				report "TBCD H FAILED: not a single ADPCM DMA write happened after the "
					& "load, so the check above only ever saw the pre-save stream"
				severity failure;
			assert h_fail = '0'
				report "TBCD H FAILED: the resumed DMA wrote the wrong data -- first bad "
					& "nibble at ADPCM address " & integer'image(h_bad_a)
					& ": got " & integer'image(h_bad_v)
					& ", expected " & integer'image(h_bad_e)
				severity failure;
			assert h_jumps = 1
				report "TBCD H FAILED: " & integer'image(h_jumps)
					& " ADPCM address discontinuities (exactly one is legal, the load "
					& "rewinding the write address).  More means the replay left a hole "
					& "in the stream or re-delivered bytes the DMA had already taken."
				severity failure;
			report "TBCD: scenario H PASSED (" & integer'image(h_nib)
				& " nibbles, " & integer'image(h_after) & " after the load, "
				& integer'image(h_cov) & "/131072 addresses covered)";
			h_arm <= '0';
			end if;  -- G_DO_H

			----------------------------------------------------------------
			if G_DO_I then
			report "TBCD: === scenario I: a $DD (READ SUBQ) in flight across a save/load ===";
			-- Save while the drive still owes the query response, then require
			-- the restored program to complete queries after replay reissues it.
			----------------------------------------------------------------
			i_lat <= G_I_LAT;
			if hps_dd_pend = '0' then
				wait until hps_dd_pend = '1' for 200 ms;
			end if;
			assert hps_dd_pend = '1'
				report "TBCD I SETUP FAILED: the program never issued a $DD, so nothing "
					& "was in flight to save" severity failure;
			mark := hps_dd_done;
			report "TBCD I: $DD in flight (answers so far " & integer'image(mark) & ")";

			do_ss(3, false, "I save mid-$DD");
			assert hps_dd_pend = '1'
				report "TBCD I SETUP FAILED: the drive answered the $DD before the "
					& "machine froze, so nothing in-flight was saved.  Raise G_I_LAT."
				severity failure;
			wait_cycles(50_000);
			do_ss(3, true,  "I load mid-$DD");
			-- let the replay settle, then stop stalling: from here a healthy
			-- machine must be able to get a $DD answered again
			wait_cycles(400_000);
			i_lat <= 2;

			idx := i_cpu_done;
			if i_cpu_done < idx + 2 then
				wait until i_cpu_done >= idx + 2 for 300 ms;
			end if;
			assert i_cpu_done >= idx + 2
				report "TBCD I FAILED: after the load the program never completed "
					& "another $DD ("  & integer'image(i_cpu_done - idx)
					& " in 300 ms) after restoring an outstanding query."
				severity failure;
			report "TBCD: scenario I PASSED (" & integer'image(i_cpu_done - idx)
				& " $DD replies completed after the load).  I is the variant where "
				& "the drive was still holding the answer; I2 below is the one "
				& "where it had already been delivered into the freeze.";

			----------------------------------------------------------------
			if G_DO_I2 then
			report "TBCD: === scenario I2: the $DD answer arrives DURING the freeze ===";
			-- Unlike I, deliver the answer while frozen. The FIFO reply may
			-- be flushed by replay, so require the restored program to progress.
			----------------------------------------------------------------
			i_freeze_ans <= '1';
			if hps_dd_pend = '0' then
				wait until hps_dd_pend = '1' for 200 ms;
			end if;
			assert hps_dd_pend = '1'
				report "TBCD I2 SETUP FAILED: no $DD went in flight" severity failure;
			report "TBCD I2: $DD in flight, held until the machine is frozen";

			mark := i_ans_frozen;
			do_ss(3, false, "I2 save, answer due mid-freeze");
			assert i_ans_frozen > mark
				report "TBCD I2 SETUP FAILED: the drive did NOT answer while the "
					& "machine was frozen (frozen-answers still "
					& integer'image(i_ans_frozen)
					& "), so this is scenario I again, not the case it exists to "
					& "test." severity failure;
			report "TBCD I2: the answer was delivered while the machine was frozen "
				& "(frozen-answers " & integer'image(mark) & " -> "
				& integer'image(i_ans_frozen) & ")";

			i_freeze_ans <= '0';
			wait_cycles(50_000);
			do_ss(3, true,  "I2 load, answer already gone");
			i_lat <= 2;

			idx := i_cpu_done;
			-- Bound query recovery, including replay serialization and its watchdog.
			if i_cpu_done < idx + 2 then
				wait until i_cpu_done >= idx + 2 for 800 ms;
			end if;
			assert i_cpu_done >= idx + 2
				report "TBCD I2 FAILED: after the load the program never completed "
					& "another $DD (" & integer'image(i_cpu_done - idx)
					& " in 800 ms) after restoring a query whose response arrived "
					& "during the save freeze" severity failure;
			report "TBCD: scenario I2 PASSED (" & integer'image(i_cpu_done - idx)
				& " $DD replies completed after the load)";
			end if;  -- G_DO_I2
			end if;  -- G_DO_I

			----------------------------------------------------------------
			if G_DO_J then
			if G_J2 then
				report "TBCD: === scenario J2: CD (IRQ2) interrupt taken THROUGH a save/load ===";
			else
				report "TBCD: === scenario J: VBL interrupt taken THROUGH a save/load ===";
			end if;
			-- $2030 (blob: region word 129+6, low byte) counts VBLs the handler
			-- serviced; CPU_1 bits 31:24 is the stack pointer.  Three saves:
			--   j1 --60ms--> j2   : handler alive, rate sane (frame rate)
			--   load(j2) --60ms--> j3 : the restored pending IRQ_VBL must be
			--   serviced ONCE and cleared - not re-taken forever.  The storm
			--   measured on hardware would give a delta in the THOUSANDS here
			--   (one handler pass is ~20 us), and S walking away as the stack
			--   page fills with identical frames.
			----------------------------------------------------------------
			-- Boot masks interrupts until the verified READs and music startup
			-- finish. Start the baseline only once the selected handler runs.
			if G_J2 then exp := 16#FB20#; else exp := 16#FB00#; end if;
			if unsigned(j_pc) /= exp then
				wait until unsigned(j_pc) = exp for 900 ms;
			end if;
			assert unsigned(j_pc) = exp
				report "TBCD J SETUP FAILED: boot never reached the interrupt handler"
				severity failure;
			report "TBCD J: interrupt handler reached before baseline";
			do_ss(3, false, "J baseline save");
			idx  := to_integer(unsigned(slot_word(3, 1)(31 downto 24)));
			cons_g := j_cnt;
			j_wait;
			do_ss(3, false, "J second save");
			exp := j_cnt;
			got := (exp - cons_g + 256) mod 256;
			report "TBCD J: interrupts in window, baseline = " & integer'image(got)
				& " (S=" & integer'image(to_integer(unsigned(slot_word(3, 1)(31 downto 24)))) & ")";
			-- Boot sets CR but not VDC timing: VBL runs at ~1.3 kHz, not 60 Hz.
			-- Compare against the measured baseline rather than an absolute rate.
			assert got >= 2
				report "TBCD J SETUP FAILED: handler dead before the load ("
					& integer'image(got) & " in 60 ms)" severity failure;
			mark := got;	-- measured baseline rate

			do_ss(3, true, "J load with IRQ history");
			j_wait;
			do_ss(3, false, "J post-load save");
			cons_a := j_cnt;
			got := (cons_a - exp + 256) mod 256;
			report "TBCD J: interrupts in window post-load = " & integer'image(got)
				& " (baseline " & integer'image(mark)
				& ", S=" & integer'image(to_integer(unsigned(slot_word(3, 1)(31 downto 24)))) & ")";
			-- NB: an 8-bit counter wraps at 256; a real storm (~3000/60 ms) still
			-- cannot masquerade as the baseline because S drifts too - both
			-- checks together are the verdict.
			assert got >= mark / 4 and got <= 3 * mark + 8
				report "TBCD J FAILED: " & integer'image(got) & " interrupts in 60 ms "
					& "after the load against a baseline of " & integer'image(mark)
					& ".  Far more = the restored IRQ_VBL is never cleared and the "
					& "handler re-enters forever (the hardware stack was found FILLED "
					& "with identical frames); far fewer = interrupts died across the "
					& "restore." severity failure;
			assert abs (to_integer(unsigned(slot_word(3, 1)(31 downto 24))) - idx) <= 8
				report "TBCD J FAILED: stack pointer drifted "
					& integer'image(to_integer(unsigned(slot_word(3, 1)(31 downto 24))) - idx)
					& " bytes across the save/load - interrupt frames are "
					& "accumulating on the stack" severity failure;
			report "TBCD: scenario J PASSED (interrupts survive the save/load at "
				& "frame rate, stack bounded)";
			end if;  -- G_DO_J

			-- now wait for the program to finish its reads and start music
			-- generous margin: the remaining verified READs run at ~84 ms each
			-- and the freezes above already cost ~130 ms
			-- "wait until" resumes on an EVENT, so it must not be entered when
			-- the condition already holds: with the READ phase switched off the
			-- D8 goes out at ~0.6 ms, well before the 5 ms above, and the wait
			-- would then sit out its whole 900 ms timeout for nothing.
			if hps_d8_count = 0 then
				wait until hps_d8_count > 0 for 900 ms;
			end if;
			assert hps_d8_count > 0
				report "TBCD BOOT FAILED: the program's D8 never reached the drive model"
				severity failure;
			report "TBCD: music commanded by the 6280 program";
			wait for 2 ms;

			if G_DO_K then
			----------------------------------------------------------------
			report "TBCD: === scenario K: interrupt-mode SAPEP debt across a save/load ===";
			-- Save after the core drains the full interrupt-mode track, with
			-- only the track-end notification owed. The model completion count
			-- must advance again after load.
			-- Gate on core consumption, not model end-of-track: the model runs
			-- tens of ms ahead of the CDDA FIFO. RP_CONS is the replay position;
			-- the full window is (end - start + 1) * 588 samples.
			----------------------------------------------------------------
			if k_cons < LOOP_SAMPLES then
				wait until k_cons >= LOOP_SAMPLES for 400 ms;
			end if;
			assert k_cons >= LOOP_SAMPLES
				report "TBCD K SETUP FAILED: the core consumed "
					& integer'image(to_integer(k_cons)) & " of the window's "
					& integer'image(LOOP_SAMPLES) & " samples, so a save here is an "
					& "ordinary mid-track resume and NOT the debt case"
				severity failure;
			assert hps_intstat_count > 0
				report "TBCD K SETUP FAILED: the interrupt-mode track never ended, so "
					& "the debt case is untested (check G_D9_MODE=2 and G_TRACK_SECT)"
				severity failure;
			report "TBCD K: the core drained the whole window ("
				& integer'image(to_integer(k_cons)) & " samples), completions so far = "
				& integer'image(hps_intstat_count);

			k_int_base := hps_intstat_count;
			do_ss(3, false, "K save with the notification owed");
			wait for 1 ms;
			do_ss(3, true, "K load with the notification owed");

			-- The replay pair costs ~2 RP_T_GAP plus the seek, and the re-armed
			-- window is one sector: give it room without making the run long.
			k_ack_base := to_integer(l_cnt_ack);
			wait for 120 ms;
			assert hps_intstat_count > k_int_base
				report "TBCD K FAILED: after the load the drive never reached an end "
					& "point again, so the owed track-end was never re-emitted "
					& "(completions still " & integer'image(hps_intstat_count) & ")"
				severity failure;

			-- Replay requested this status, not the CPU. Consume it internally:
			-- a CPU waiting for DTD will not ACK STATUS via $1802 bit 7.
			-- Require SP_FREE with no additional program ACKs.
			assert w_sp = x"0"
				report "TBCD K FAILED: 120 ms after the load the SCSI phase is still "
					& to_hstring(w_sp) & " instead of SP_FREE: the replay's status was "
					& "presented to the program and the bus is parked"
				severity failure;
			assert to_integer(l_cnt_ack) = k_ack_base
				report "TBCD K FAILED: the program made " 
					& integer'image(to_integer(l_cnt_ack) - k_ack_base) & " ACKs after the "
					& "load: that status should never have reached it"
				severity failure;
			report "TBCD K PASSED: track end re-emitted and consumed by the core "
				& "(completions " & integer'image(k_int_base) & " -> "
				& integer'image(hps_intstat_count) & ", phase SP_FREE, ACKs unchanged at "
				& integer'image(k_ack_base) & ")";
			end if;

			if G_DO_L then
			----------------------------------------------------------------
			report "TBCD: === scenario L: in-flight SCSI phase across save/load ===";
			-- The case: a game running with an in-flight SCSI phase (SP /= SP_FREE),
			-- specifically SP_STAT_START (SP=4), with AUD_OPEN=0, READ_ACTIVE=0,
			-- STAT_PEND=0, DOUT_PEND=0, EMPTY=1.
			-- A parked phase must not prevent the save boundary from opening.
			----------------------------------------------------------------
			if (G_L_PHASE = 0 or G_L_PHASE = 10) then
				t_dead := now + 100 ms;
				while (w_sp /= x"A" or to_integer(w_cons) < 200) and now < t_dead loop
					wait for 100 us;
				end loop;
				assert w_sp = x"A"
					report "TBCD L SETUP FAILED: core never reached SP_DATAIN_START (SP="
						& to_hstring(w_sp) & ")" severity failure;
				report "TBCD L: core reached SP_DATAIN_START (SP=" & to_hstring(w_sp)
					& " CONS=" & integer'image(to_integer(w_cons)) & ")";
				do_ss(3, false, "L save in SP_DATAIN_START", 6);
				assert sp_frz = x"A"
					report "TBCD L FAILED: save did not land with SP_DATAIN_START (SP="
						& to_hstring(sp_frz) & ")" severity failure;
				report "TBCD L: save engaged at SP=" & to_hstring(sp_frz);
				wait for 1 ms;
				do_ss(3, true, "L load in SP_DATAIN_START");
				report "TBCD L: load in SP_DATAIN_START ok";
			end if;

			if (G_L_PHASE = 0 or G_L_PHASE = 11) then
				t_dead := now + 100 ms;
				while (w_sp /= x"B" or to_integer(w_cons) < 500) and now < t_dead loop
					wait for 100 us;
				end loop;
				assert w_sp = x"B"
					report "TBCD L SETUP FAILED: core never reached SP_DATAIN_END (SP="
						& to_hstring(w_sp) & ")" severity failure;
				report "TBCD L: core reached SP_DATAIN_END (SP=" & to_hstring(w_sp)
					& " CONS=" & integer'image(to_integer(w_cons)) & ")";
				do_ss(3, false, "L save in SP_DATAIN_END", 6);
				assert sp_frz = x"B"
					report "TBCD L FAILED: save did not land with SP_DATAIN_END (SP="
						& to_hstring(sp_frz) & ")" severity failure;
				report "TBCD L: save engaged at SP=" & to_hstring(sp_frz);
				wait for 1 ms;
				do_ss(3, true, "L load in SP_DATAIN_END");
				report "TBCD L: load in SP_DATAIN_END ok";
			end if;

			if (G_L_PHASE = 0 or G_L_PHASE = 4) then
				t_dead := now + 100 ms;
				while w_sp /= x"4" and now < t_dead loop
					wait for 100 us;
				end loop;
				assert w_sp = x"4"
					report "TBCD L SETUP FAILED: core never reached SP_STAT_START (SP="
						& to_hstring(w_sp) & ")" severity failure;
				report "TBCD L: core reached SP_STAT_START (SP=" & to_hstring(w_sp)
					& " CONS=" & integer'image(to_integer(w_cons)) & ")";
				do_ss(3, false, "L save in SP_STAT_START", 6);
				assert sp_frz = x"4"
					report "TBCD L FAILED: save did not land with SP_STAT_START (SP="
						& to_hstring(sp_frz) & ")" severity failure;
				report "TBCD L: save engaged at SP=" & to_hstring(sp_frz);
				wait for 1 ms;
				do_ss(3, true, "L load in SP_STAT_START");
				report "TBCD L: load in SP_STAT_START ok";
			end if;

			if (G_L_PHASE = 0 or G_L_PHASE = 7) then
				t_dead := now + 100 ms;
				while w_sp /= x"7" and now < t_dead loop
					wait for 100 us;
				end loop;
				assert w_sp = x"7"
					report "TBCD L SETUP FAILED: core never reached SP_MSGIN_START (SP="
						& to_hstring(w_sp) & ")" severity failure;
				report "TBCD L: core reached SP_MSGIN_START (SP=" & to_hstring(w_sp)
					& " CONS=" & integer'image(to_integer(w_cons)) & ")";
				do_ss(3, false, "L save in SP_MSGIN_START", 6);
				assert sp_frz = x"7"
					report "TBCD L FAILED: save did not land with SP_MSGIN_START (SP="
						& to_hstring(sp_frz) & ")" severity failure;
				report "TBCD L: save engaged at SP=" & to_hstring(sp_frz);
				wait for 1 ms;
				do_ss(3, true, "L load in SP_MSGIN_START");
				report "TBCD L: load in SP_MSGIN_START ok";
			end if;

			-- After load, the restored CPU finishes its busy wait remainder,
			-- completes STATUS + MSGIN handshakes, and returns SP to SP_FREE.
			-- Then READ6 #2 runs and completes (w_fresh_done = 1).
			t_dead := now + 250 ms;
			while (w_fresh_done = 0 or w_sp /= x"0") and now < t_dead loop
				wait for 100 us;
			end loop;

			assert w_sp = x"0"
				report "TBCD L FAILED: after load phase never returned to SP_FREE (SP="
					& to_hstring(w_sp) & ")" severity failure;
			assert to_integer(unsigned(l_cnt_req)) > 0
				report "TBCD L FAILED: REQ counter did not advance after load"
				severity failure;
			assert to_integer(unsigned(l_cnt_ack)) > 0
				report "TBCD L FAILED: ACK counter did not advance after load"
				severity failure;
			assert w_fresh_done > 0
				report "TBCD L FAILED: fresh READ6 #2 never completed after load"
				severity failure;
			assert w_fail = '0' and c_fail = '0'
				report "TBCD L FAILED: data mismatch in restored reader"
				severity failure;

			report "TBCD L PASSED: in-flight SCSI phase(s) restored cleanly ("
				& "SP=" & to_hstring(w_sp)
				& " REQ=" & integer'image(to_integer(unsigned(l_cnt_req)))
				& " ACK=" & integer'image(to_integer(unsigned(l_cnt_ack)))
				& " fresh_done=" & integer'image(w_fresh_done)
				& ")";
			std.env.finish;
			end if;

			----------------------------------------------------------------
			-- B's oracle assumes a LOOPING track (G_D9_MODE=1): an
			-- interrupt-mode run ends the audio before this point, so the gate
			-- exists to let scenario K run without dragging B into a failure
			-- that says nothing about the RTL.
			if G_DO_B then
			report "TBCD: === scenario B: CDDA sample-exact resume ===";
			----------------------------------------------------------------
			-- a modelled seek delays the music's start by as much
			wait for 1 ms + G_PLAY_LAT_MS * 1 ms;
			do_ss(3, false, "B save mid-music");
			v := slot_word(3, 64);				-- CD_6 (eReg 63)
			cons_a := to_integer(unsigned(v(58 downto 27)));
			report "TBCD B: consumed at save = " & integer'image(cons_a);
			wait for 1 ms;						-- music diverges live

			mark := slog_count;
			do_ss(3, true, "B load mid-music");
			t_ld := now;
			-- Allow settle, command gaps, FIFO drain and the later wrap pair.
			-- Search from this load's resume flush, not the stale pre-load tail.
			wait for 140 ms;
			assert slog_count > mark + 64
				report "TBCD B FAILED: no samples after load (replay dead?)" severity failure;

			-- Anchor the search at the replay's last CDDA flush: the resumed
			-- stream starts THERE, so the expected value must appear right
			-- away.  Scanning the whole tape instead proves nothing once the
			-- loop window is shorter than the post-load stream - the track
			-- comes back round to any value on its own.  The tolerance is far
			-- below one sector (588 samples) so a resume that is off by even a
			-- single sector cannot slip through.
			assert flush_mark > mark
				report "TBCD B FAILED: the replay never flushed the CDDA stream"
				severity failure;
			-- Each silent-D8/D9 pair flushes twice: resume = second flush,
			-- wrap = fourth. Filter by latched RP_CONS so a previous load's
			-- late LF_ARM wrap pair cannot shift this load's anchor.
			nf := 0;
			wrapa := -1;
			anchor := flush_mark;
			for i in 0 to 7 loop
				fl_tmp(i) := -1;
			end loop;
			for i in 0 to 7 loop
				if flush_log(i) > mark and flush_log(i) /= -1
				   and flush_cons(i) = cons_a then
					fl_tmp(nf) := flush_log(i);
					nf := nf + 1;
				end if;
			end loop;
			-- flush_log is a ring without order: sort the collected ones
			for i in 0 to 6 loop
				for j in 0 to 6 - i loop
					if fl_tmp(j) > fl_tmp(j+1) and fl_tmp(j+1) /= -1 then
						fl_swp := fl_tmp(j); fl_tmp(j) := fl_tmp(j+1); fl_tmp(j+1) := fl_swp;
					end if;
				end loop;
			end loop;
			report "TBCD B: flush di QUESTO load (RP_CONS=" & integer'image(cons_a)
				& "): nf=" & integer'image(nf)
				& " [" & integer'image(fl_tmp(0)) & " " & integer'image(fl_tmp(1))
				& " " & integer'image(fl_tmp(2)) & " " & integer'image(fl_tmp(3)) & "]";
			assert nf >= 2
				report "TBCD B FAILED: this load produced " & integer'image(nf)
					& " replay flush(es), so there is no resume pair to anchor on"
				severity failure;
			anchor := fl_tmp(1);
			wrapc := -1;
			if nf >= 4 then
				wrapa := fl_tmp(3);	-- post-wrap stream starts here (after the LF D9)
				wrapc := fl_tmp(2);	-- the LF D8 flush = where the fix CUT the stream
			end if;
			exp := cons_a mod LOOP_SAMPLES;
			-- Diagnostic before judging: print what the resumed stream ACTUALLY
			-- starts with, and where the expected value really lands relative to
			-- the flush.  A distance of one sector (588) means the replay
			-- resumed a sector off; a small distance means the sub-sector sample
			-- skip is out; a huge one means the position is unrelated.
			idx := -1;
			for i in anchor to slog_count - 1 loop
				if slog(i) = dac_of(exp) then idx := i; exit; end if;
			end loop;
			report "TBCD B: flush at tape index " & integer'image(anchor)
				& ", exp=" & integer'image(exp)
				& " (dac " & integer'image(dac_of(exp)) & ")"
				& ", first exp at " & integer'image(idx)
				& " (offset " & integer'image(idx - anchor) & ")"
				& ", stream starts: "
				& integer'image(slog(anchor)) & " "
				& integer'image(slog(anchor+1)) & " "
				& integer'image(slog(anchor+2)) & " "
				& integer'image(slog(anchor+3)) & " "
				& integer'image(slog(anchor+4)) & " "
				& integer'image(slog(anchor+5)) & " "
				& integer'image(slog(anchor+6)) & " "
				& integer'image(slog(anchor+7));
			got := -1;
			for i in anchor to minimum(anchor + 64, slog_count - 33) loop
				if slog(i) = dac_of(exp) then
					got := i;
					for k in 1 to 31 loop
						if slog(i + k) /= (slog(i) + k) mod (dac_of(LOOP_SAMPLES - 1) + 1) then
							got := -1;
							exit;
						end if;
					end loop;
					exit when got >= 0;
				end if;
			end loop;
			assert got >= 0
				report "TBCD B FAILED: the resumed stream does not start at "
					& integer'image(exp) & " (searched the 64 samples from the replay "
					& "flush at tape index " & integer'image(anchor) & ")"
					severity failure;
			check_sync("B", got);
			check_seek("B", got, t_ld);
			-- Report offset from the resume anchor; zero means sample-exact.
			-- flush_mark may already point to the later wrap pair.
			report "TBCD: scenario B PASSED (resume at " & integer'image(exp)
				& ", stream started " & integer'image(idx - anchor)
				& " samples from the flush; 0 = sample-exact)";
			-- The wrap pair must restart at the loop start and cut the resumed
			-- stream within ~2 sectors of the window end.
			if wrapa > 0 then
				got := -1;
				for i in wrapa to minimum(wrapa + 64, slog_count - 33) loop
					if slog(i) = dac_of(0) then
						got := i;
						for k in 1 to 31 loop
							if slog(i + k) /= (slog(i) + k) mod (dac_of(LOOP_SAMPLES - 1) + 1) then
								got := -1;
								exit;
							end if;
						end loop;
						exit when got >= 0;
					end if;
				end loop;
				assert got >= 0
					report "TBCD B FAILED: the wrap-fix stream does not restart at the loop start"
					severity failure;
				-- precision: the sample just before the LF-D8 flush is where
				-- the healthy resumed stream was cut - it must sit within ~2
				-- sectors of the window end (samples between the fix's two
				-- flushes are an unphased tail: never measure there).  Both terms
				-- are in DAC units so the fader's scaling cancels.
				werr := dac_of(LOOP_SAMPLES - 1) - slog(wrapc - 1);
				assert werr <= 1200 and werr >= -1200
					report "TBCD B FAILED: the wrap-fix fired " & integer'image(werr)
						& " samples away from the window end"
					severity failure;
				report "TBCD B: wrap-fix verified (restart at the loop start, "
					& integer'image(werr) & " samples before the end)";
			end if;

			end if;	-- G_DO_B


			----------------------------------------------------------------
			if G_DO_G then
			report "TBCD: === scenario G: CDDA resume AFTER a loop wrap ===";
			-- Exercise RP_LOOPMOD after consumption exceeds one loop window.
			--
			-- The drive plays the loop window INCLUSIVE of the D9 end sector
			-- (tb model, pcecdd semantics: "lba+1 > pend_lba -> lba := pstart"),
			-- so the window is END-START+1 sectors.  The played sample value is
			-- the frame index inside that window, which is the oracle here: a
			-- fold-back that uses the wrong window length lands the resumed
			-- stream somewhere else entirely, and the error grows with the
			-- number of wraps the save sits behind.
			----------------------------------------------------------------
			wait for 8 ms;
			do_ss(3, false, "G save after wrap");
			v := slot_word(3, 64);
			cons_g := to_integer(unsigned(v(58 downto 27)));
			report "TBCD G: consumed at save = " & integer'image(cons_g)
				& " (loop window = " & integer'image(LOOP_SAMPLES) & " samples, "
				& integer'image(cons_g / LOOP_SAMPLES) & " wraps behind)";
			assert cons_g > LOOP_SAMPLES
				report "TBCD G SETUP FAILED: the track never wrapped, so the fold-back "
					& "path is still untested (consumed=" & integer'image(cons_g) & ")"
				severity failure;
			wait for 1 ms;

			mark := slog_count;
			do_ss(3, true, "G load after wrap");
			t_ld := now;
			wait for 140 ms;	-- include the wrap pair
			assert slog_count > mark + 64
				report "TBCD G FAILED: no samples after load (replay dead?)" severity failure;

			assert flush_mark > mark
				report "TBCD G FAILED: the replay never flushed the CDDA stream"
				severity failure;
			-- As in B, filter by RP_CONS: a previous load's late wrap pair can
			-- fall after the mark and shift an ordinal-only anchor by two.
			nf := 0;
			wrapa := -1;
			anchor := flush_mark;
			for i in 0 to 7 loop
				fl_tmp(i) := -1;
			end loop;
			for i in 0 to 7 loop
				if flush_log(i) > mark and flush_log(i) /= -1
				   and flush_cons(i) = cons_g then
					fl_tmp(nf) := flush_log(i);
					nf := nf + 1;
				end if;
			end loop;
			-- flush_log is a ring without order: sort the collected ones
			for i in 0 to 6 loop
				for j in 0 to 6 - i loop
					if fl_tmp(j) > fl_tmp(j+1) and fl_tmp(j+1) /= -1 then
						fl_swp := fl_tmp(j); fl_tmp(j) := fl_tmp(j+1); fl_tmp(j+1) := fl_swp;
					end if;
				end loop;
			end loop;
			report "TBCD G: flush di QUESTO load (RP_CONS=" & integer'image(cons_g)
				& "): nf=" & integer'image(nf)
				& " [" & integer'image(fl_tmp(0)) & " " & integer'image(fl_tmp(1))
				& " " & integer'image(fl_tmp(2)) & " " & integer'image(fl_tmp(3)) & "]";
			assert nf >= 2
				report "TBCD G FAILED: this load produced " & integer'image(nf)
					& " replay flush(es), so there is no resume pair to anchor on"
				severity failure;
			anchor := fl_tmp(1);
			wrapc := -1;
			if nf >= 4 then
				wrapa := fl_tmp(3);	-- post-wrap stream starts here (after the LF D9)
				wrapc := fl_tmp(2);	-- the LF D8 flush = where the fix CUT the stream
			end if;
			exp := cons_g mod LOOP_SAMPLES;
			-- Diagnostic before judging: print what the resumed stream ACTUALLY
			-- starts with, and where the expected value really lands relative to
			-- the flush.  A distance of one sector (588) means the replay
			-- resumed a sector off; a small distance means the sub-sector sample
			-- skip is out; a huge one means the position is unrelated.
			idx := -1;
			for i in anchor to slog_count - 1 loop
				if slog(i) = dac_of(exp) then idx := i; exit; end if;
			end loop;
			report "TBCD G: flush at tape index " & integer'image(anchor)
				& ", exp=" & integer'image(exp)
				& " (dac " & integer'image(dac_of(exp)) & ")"
				& ", first exp at " & integer'image(idx)
				& " (offset " & integer'image(idx - anchor) & ")"
				& ", stream starts: "
				& integer'image(slog(anchor)) & " "
				& integer'image(slog(anchor+1)) & " "
				& integer'image(slog(anchor+2)) & " "
				& integer'image(slog(anchor+3)) & " "
				& integer'image(slog(anchor+4)) & " "
				& integer'image(slog(anchor+5)) & " "
				& integer'image(slog(anchor+6)) & " "
				& integer'image(slog(anchor+7));
			got := -1;
			for i in anchor to minimum(anchor + 64, slog_count - 33) loop
				if slog(i) = dac_of(exp) then
					got := i;
					for k in 1 to 31 loop
						if slog(i + k) /= (slog(i) + k) mod (dac_of(LOOP_SAMPLES - 1) + 1) then
							got := -1;
							exit;
						end if;
					end loop;
					exit when got >= 0;
				end if;
			end loop;
			assert got >= 0
				report "TBCD G FAILED: the resumed stream does not start at "
					& integer'image(exp) & " -- the loop fold-back put the track back "
					& "in the wrong place (flush at tape index "
					& integer'image(anchor) & ")"
					severity failure;
			check_sync("G", got);
			check_seek("G", got, t_ld);
			-- Same anchoring as B: measure from the resume flush, not from
			-- flush_mark, which the wrap-fix's later pair may have moved.
			report "TBCD: scenario G PASSED (loop-wrapped resume at " & integer'image(exp)
				& ", stream started " & integer'image(idx - anchor)
				& " samples from the flush; 0 = sample-exact)";
			-- The wrap pair must restart at the loop start and cut the resumed
			-- stream within ~2 sectors of the window end.
			if wrapa > 0 then
				got := -1;
				for i in wrapa to minimum(wrapa + 64, slog_count - 33) loop
					if slog(i) = dac_of(0) then
						got := i;
						for k in 1 to 31 loop
							if slog(i + k) /= (slog(i) + k) mod (dac_of(LOOP_SAMPLES - 1) + 1) then
								got := -1;
								exit;
							end if;
						end loop;
						exit when got >= 0;
					end if;
				end loop;
				assert got >= 0
					report "TBCD G FAILED: the wrap-fix stream does not restart at the loop start"
					severity failure;
				-- precision: the sample just before the LF-D8 flush is where
				-- the healthy resumed stream was cut - it must sit within ~2
				-- sectors of the window end (samples between the fix's two
				-- flushes are an unphased tail: never measure there).  Both terms
				-- are in DAC units so the fader's scaling cancels.
				werr := dac_of(LOOP_SAMPLES - 1) - slog(wrapc - 1);
				assert werr <= 1200 and werr >= -1200
					report "TBCD G FAILED: the wrap-fix fired " & integer'image(werr)
						& " samples away from the window end"
					severity failure;
				report "TBCD G: wrap-fix verified (restart at the loop start, "
					& integer'image(werr) & " samples before the end)";
			end if;

			end if;  -- G_DO_G

			----------------------------------------------------------------
			if G_DO_F then
			report "TBCD: === scenario F: BRAM (page $F7) save/load round-trip ===";
			-- Save patterned 2 KB BRAM, wipe it, then require the blob alone
			-- to restore it bit-exact. BRM occupies blob words 24961..25216;
			-- region byte n is at bits (n mod 8)*8.
			----------------------------------------------------------------
			do_ss(2, false, "F BRAM save");
			-- save-side: the blob's BRM region must carry the seeded pattern,
			-- proving the region-7 READ actually reached the BRAM port
			diff := 0;
			for k in 0 to 255 loop
				v := slot_word(2, 24961 + k);
				for i in 0 to 7 loop
					exp := (((8*k + i)*37 + 11) mod 256);
					got := to_integer(unsigned(v(i*8+7 downto i*8)));
					if got /= exp then
						diff := diff + 1;
						if diff <= 8 then
							report "TBCD F: BRM byte " & integer'image(8*k+i)
								& " in blob=" & integer'image(got)
								& " exp=" & integer'image(exp) severity error;
						end if;
					end if;
				end loop;
			end loop;
			assert diff = 0
				report "TBCD F FAILED (save): BRM not captured into the blob ("
					& integer'image(diff) & " bytes) -- the region-7 read is broken"
					severity failure;
			report "TBCD F: BRM captured into the blob OK";
			-- Wipe live BRAM; only the load's region-7 walk can restore it.
			for i in 0 to 2047 loop brm(i) := x"FF"; end loop;
			do_ss(2, true, "F BRAM load");
			diff := 0;
			for i in 0 to 2047 loop
				exp := ((i*37 + 11) mod 256);
				if to_integer(unsigned(brm(i))) /= exp then
					diff := diff + 1;
					if diff <= 8 then
						report "TBCD F: BRM byte " & integer'image(i)
							& " after load=" & integer'image(to_integer(unsigned(brm(i))))
							& " exp=" & integer'image(exp) severity error;
					end if;
				end if;
			end loop;
			assert diff = 0
				report "TBCD F FAILED: BRAM not restored from the blob ("
					& integer'image(diff) & " bytes wrong) -- load leans on persistent BRAM"
					severity failure;
			report "TBCD: scenario F PASSED (BRAM round-trips through the blob alone)";
			end if;	-- G_DO_F

			----------------------------------------------------------------
			-- Scenario C + E verdicts.  Both are watched by monitors for the
			-- whole run rather than sampled from a blob, so they cover every
			-- load in the run - including the mid-READ ones in scenario A,
			-- which is where the resume-READ misalignment lives.
			----------------------------------------------------------------
			-- the reader must actually have finished its 8 verified READs,
			-- otherwise "no mismatch" would just mean "no bytes checked"
			v := slot_word(3, 131);
			report "TBCD C: reads completed at the B save = "
				& integer'image(to_integer(unsigned(v(47 downto 40))));
			assert hps_d8_count > 0
				report "TBCD C FAILED: the read phase never completed, so the stream was never verified"
				severity failure;
			assert c_fail = '0'
				report "TBCD C FAILED: the reader saw a byte-stream mismatch after a load "
					& "(resume-READ misalignment)" severity failure;
			report "TBCD: scenario C PASSED (stream stayed aligned across every load)";

			assert e_fail = '0'
				report "TBCD E FAILED: CD_DATA_CNT and READ_CONSUMED disagree after a resume"
				severity failure;
			report "TBCD: scenario E PASSED (DTR/CD_DATA_CNT bookkeeping survives the resume)";

			-- Check the instruction-boundary invariant across every save.
			assert ss_en_leak = 0 and ss_state_bad = 0 and ss_blob_bad = 0
				report "TBCD ST0 FAILED: the freeze does not stop the CPU cleanly -- "
					& "CPU enabled while asleep: " & integer'image(ss_en_leak)
					& ", STATE moved after freeze: " & integer'image(ss_state_bad)
					& ", blobs captured mid-instruction: " & integer'image(ss_blob_bad)
				severity failure;
			report "TBCD: instruction-boundary invariant held on every save";

			report "TBCD: ALL SCENARIOS PASSED";
			std.env.finish;
		end process;
	end block;

end architecture;
