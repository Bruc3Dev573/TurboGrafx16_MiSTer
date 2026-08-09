-----------------------------------------------------------------------------
-- tb_cd.vhd — CD-subsystem savestate testbench, v2 (static-verification plan
-- docs/STATIC_VERIFICATION_PLAN.md).
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
--  * a shared ROM/CD-RAM data port reproducing TurboGrafx16.sv (single
--    data register, rom_rd raddr priority, ce_rom pacing) — the structure
--    behind the region-10 read-steal bug;
--  * hang-watchdogs on every savestate wait; on timeout the SS_DBG
--    boundary-block mask is printed (sim analogue of "si impalla").
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
		-- SAPEP play mode (D9 byte1).  1 = LOOP, which every scenario used until
		-- now; 2 = INTERRUPT, which is what Rondo actually issues and which the
		-- drive answers only when the audio reaches the end point - so the bus
		-- stays BSY with the phase back at SP_FREE, and a savestate lands there.
		-- The replay also treats the two differently: only the non-INTERRUPT
		-- SAPEP is counted for SWALLOW, and only LOOP folds the position back.
		G_D9_MODE : integer := 1;
		-- Loop window in sectors.  Scenario G needs it SHORT so the track wraps
		-- within a sane simulation; an INTERRUPT-mode run needs it LONG, because
		-- that mode plays once and stops at the end point, and the hardware case
		-- being reproduced saves in the MIDDLE of a ~4745-sector track, not at its
		-- end.  With a short window the audio simply finishes during the save and
		-- the run says nothing about the defect.
		G_TRACK_SECT : integer := 4;
		G_DO_G       : boolean := true;
		-- Scenario H: ADPCM DMA (the AD_TRANS / AD_CPLAY mechanism) across a
		-- save/load.  This is the CD->ADPCM-RAM streaming path a movie uses and
		-- the only large mechanism no scenario had ever crossed with a
		-- savestate.  The DMA READ6 is issued BEFORE the D8/D9, because a READ6
		-- knocks the drive model out of H_PLAY and would kill B/G/F.
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
		-- Scenario I: a NON-READ6 command in flight across a save/load.  The
		-- replay dispatch (SCSI.vhd, end of RP_SETTLE) has exactly three arms -
		-- AUDIO_ACTIVE, READ_ACTIVE, else RP_PAUSE - so anything that is neither
		-- audio nor a READ6 is not rebuilt at all.  $DD (READ SUBQ) is the case
		-- that matters: the System Card issues it from CD_SUBQ and the handoff
		-- lists a stale disc position as a live suspect.
		G_DO_I       : boolean := true;
		-- Drive latency for the ARMED $DD, in 50 us model ticks.  Only applied
		-- while the scenario arms it, so the main loop's ordinary $DD stays fast
		-- and B/G/F keep their timing.  It has to exceed the freeze latency (the
		-- composite boundary waits for VBLANK, up to ~17 ms) or the save would
		-- land after the answer instead of during the wait.
		G_I_LAT      : integer := 800;
		-- Scenario I2, the variant that actually matters.  In I the drive is
		-- still holding the answer when the machine resumes, so the restored CPU
		-- gets it and carries on - and on hardware that is the LUCKY case.  The
		-- real HPS answers in about a millisecond while the walk takes ~25 ms, so
		-- the answer normally lands DURING the freeze, goes into the SCSI FIFO,
		-- and REPLAY_START flushes it: the restored CPU then waits forever for a
		-- reply that was consumed in a timeline that no longer exists.
		-- I2 does NOT try to time this: the drive holds the answer until SLEEP is
		-- actually asserted (i_freeze_ans).  A first attempt used a short latency
		-- and lost the race against the boundary wait, which varies with the
		-- VBLANK phase - the answer landed before the freeze and the setup guard
		-- rejected the run.
		G_DO_I2      : boolean := true;
		-- CD-RAM size in the DUT.  4 KB keeps every walk at ~4 sim-ms and is
		-- what every scenario always ran with.  The REAL core carries 256 KB
		-- since p11, which makes the freeze ~25 ms - and hardware showed a
		-- whole class of defects that only exist at the long freeze (saving at
		-- the Konami logo worked up to p10's ~12 ms walk and wedges from p11).
		-- The consuming side is frozen for the whole walk while the HPS-facing
		-- FIFO fill paths deliberately stay live: CDDA FIFO holds 4096 samples,
		-- SUBC only 490 bytes, and a full FIFO DROPS silently.  Set 262144 to
		-- reproduce the hardware-sized freeze.
		G_CDRAM_BYTES : integer := 4096;
		-- Scenario J: a VDC VBLANK interrupt taken THROUGH a save/load.  The
		-- hardware wedge signature is the 6280 stack page filled with identical
		-- interrupt frames (PCH=5B PCL=6F P=00) and IRQ_VBL restored pending -
		-- an interrupt storm.  No scenario could ever see it: the synthetic
		-- BIOS had no interrupt handler and never executed CLI, so not one
		-- interrupt was ever taken across a savestate in this bench.  J gives
		-- the BIOS a real VBL handler (ack via VDC status read, count in WRAM),
		-- unmasks IRQ1, and asserts across a save/load that (a) interrupts keep
		-- being taken at frame rate, not storm rate, and (b) the stack pointer
		-- comes back to where it was.
		G_DO_J       : boolean := true;
		-- Scenario J2: the SAME cycle through the OTHER interrupt.  J proved
		-- IRQ1 (VDC) clean; the hardware stack frames point into IRQ2_INT2 -
		-- the System Card's CD interrupt handler (PC cluster $E750-$E7DE).
		-- IRQ2 across a save/load has never been exercised: J masks it.  With
		-- G_J2 the BIOS switches to the storm-prone shape the System Card
		-- actually uses ($074C..): disable the specific enable (TRB $1802),
		-- CLI while still inside the handler, ack, re-enable (TSB), RTI.  The
		-- source is the subcode-byte interrupt (SUBCD_EN, ~7.35 kHz off the
		-- CDDA clock), which is what runs in the wedging menu scene.
		G_J2         : boolean := false;
		-- Scenario W: the hardware CD savestate soak.  Save+load DURING a
		-- streaming READ6 (twice: once mid-sector, once in the TAIL WINDOW where
		-- every data byte has been consumed but the final STATUS has not been
		-- strobed yet - the sect>=cnt branch of RP_RD_PREP, prime suspect), then
		-- a FRESH READ6 at a different LBA whose whole conversation is verified.
		-- Requires G_PUSH_STATUS=true.
		G_DO_W       : boolean := false;
		-- Scenario W2: the residual hardware case - the FMV -> stage-0
		-- TRANSITION.  Runs W's two save/load cycles and its fresh READ6 (which
		-- passes on current RTL), then mimics the stage-0 loader: D8+D9 (CDDA
		-- starts while loading), a THIRD save/load landing in the AUDIO window
		-- (AUDIO_ACTIVE=1, CDDA streaming), then two more READ6s back-to-back.
		-- Every conversation must close byte-exact (data, STATUS, MSGIN) and
		-- the program must reach its end marker.  Implies the W machinery;
		-- requires G_PUSH_STATUS=true.
		G_DO_W2      : boolean := false;
		-- Scenario W3: the W2 transition on the REAL HPS model (G_REAL_HPS),
		-- with the third save/load landing in the D8's SEEK-LATENCY window:
		-- the D8 status is PENDED in the single deferred slot and FROZEN by
		-- the seek when the freeze lands.  Expected on the residual: the
		-- replay's re-issued D8 OVERWRITES the pended slot (the game's own
		-- status is lost forever), the surviving delivery is SWALLOWed as the
		-- replay's, and the restored program parks in its status wait.
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
		--  * a savestate LOAD changes NOTHING here - the real HPS does not
		--    know a load happened.
		G_REAL_HPS   : boolean := false;
		-- seek time for the distant-jump tier, in (real) ms.  The true value
		-- is 300-600 ms; the sim default is chosen to stay well above the
		-- 24 ms replay gap but BELOW the drain's RSKIP_STARVE valve (~84 ms at
		-- the 100 MHz sim clock), which a full-length seek would trip.
		G_SEEK_MS    : integer := 50;
		-- the ~220 ms the real SAPSP handler subtracts from its seek
		G_AUDIODELAY_MS : integer := 220;
		-- W3 positioning knob: pin the D8 seek latency (ms) independently of
		-- G_SEEK_MS.  The real distant audio seek minus audiodelay is
		-- 80-380 ms; with the run-shortened G_SEEK_MS the derived value would
		-- collapse below the save walk and the frozen-slot window would close
		-- before the replay's D8 arrives.  0 = derive from the seek tiers.
		G_W3_D8LAT_MS : integer := 0;
		-- Push-based READ6 serving (the real Main_MiSTer cd.cpp semantics): the
		-- drive pushes sectors as fast as the core's 4096-byte SCSI_FIFO
		-- backpressure allows and strobes the final STATUS G_STAT_TICKS model
		-- ticks after the LAST BYTE IS PUSHED - NOT after consumption.  The
		-- consumption-gated model every existing scenario was written against
		-- can never open the tail window above, which is why the matrix was
		-- blind to the hardware bug.  Default false: existing scenarios keep
		-- the model they were verified with.
		G_PUSH_STATUS : boolean := false;
		-- Final-STATUS latency in 50 us model ticks after the last pushed byte
		-- (push-status mode only).  The real HPS strobe rides a poll tick.
		G_STAT_TICKS  : integer := 2
	);
end entity;

architecture sim of tb_cd is

	constant CLK_PERIOD : time := 10 ns;
	constant BASE_DW    : integer := 16#3800000#;
	constant SLOT_DW    : integer := 16#C0000#;
	constant CDRAM_SIM  : integer := G_CDRAM_BYTES;
	-- STATESIZE with 4KB CD-RAM: 2 + 256 + (528576-262144+4096)/4 = 67890 DW
	-- STATESIZE in 32-bit dwords is 2 + 256 + (528576 - 262144 + CDRAM)/4
	-- (headers + internals + regions, with region 10 scaled to the simulated
	-- CD-RAM); the DDR model stores 64-bit words, hence the /2.  The literal
	-- 33945 this replaces was only valid for 4 KB and silently wrong for any
	-- other size.  For 262144 this gives 66201, matching sim/lint_sizes.py.
	constant SLOT_WORDS : integer := (2 + 256 + (528576 - 262144 + G_CDRAM_BYTES) / 4) / 2;
	constant TRACK_LBA  : integer := 4000;		-- 0x000FA0
	-- 5-sector loop.  The window has to be small enough that the drive actually
	-- WRAPS within a sane simulation: the CDDA stream is consumed at ~62 samples
	-- per sim-ms, so the old 51-sector window needed ~480 ms of play before the
	-- fold-back path scenario G exists to test could even be reached.
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
	signal ss_dbg   : std_logic_vector(8 downto 0);

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
	signal cd_data     : std_logic_vector(7 downto 0) := (others => '0');
	signal cd_data_wr  : std_logic := '0';
	signal cd_audio_wr : std_logic := '0';
	signal cd_subcd_wr : std_logic := '0';
	signal cd_data_end : std_logic;

	signal cdda_sl  : signed(15 downto 0);
	signal cdda_sr  : signed(15 downto 0);

	-- Backup RAM (page $F7) model: the real 2KB block-RAM lives in the SV
	-- top (TurboGrafx16.sv), so tb_cd used to stub BRM_DO=0 and never tested
	-- the region-7 walk.  A real model here lets scenario F reproduce the
	-- hardware "wipe BRAM -> loads break" report.
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
		-- Address assertion.  This BIOS is hand-assembled machine code with
		-- ABSOLUTE addresses in it, and docs/SIMULATION.md records what happens
		-- when an insertion silently shifts everything below it (commit 34a4eca:
		-- the D8 stopped reaching the drive model for the whole life of that
		-- commit and three scenarios were never actually executed).  chk() makes
		-- that class of mistake an elaboration-time failure instead.
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
		-- VDC0 display ON (CR=$CC): makes VBLANK periodic so the composite
		-- boundary lands fast (display-off burst kept !vbl mostly true)
		b(16#A9#); b(16#05#); b(16#8D#); b(16#00#); b(16#40#);   -- AR=CR
		b(16#A9#); b(16#CC#); b(16#8D#); b(16#02#); b(16#40#);   -- CR lo=$CC
		b(16#A9#); b(16#00#); b(16#8D#); b(16#03#); b(16#40#);   -- CR hi=0
		b(16#A9#); b(16#EE#); b(16#8D#); b(16#00#); b(16#60#);
		b(16#A9#); b(16#02#); b(16#8D#); b(16#01#); b(16#60#);
		b(16#A9#); b(16#20#); b(16#8D#); b(16#02#); b(16#60#);
		b(16#A9#); b(16#60#); b(16#8D#); b(16#03#); b(16#60#);
		-- $FF20: N verified READs, then D8+D9, then the counter loop.
		-- Each READ6 is 4 sectors = 8192 bytes and the reader consumes about a
		-- byte every 10 us, so one READ costs ~84 sim-ms (and minutes of wall
		-- clock).  Three is enough: scenario A's saves and loads all land inside
		-- the first one, and the two that follow prove the stream stays aligned
		-- across a full command boundary.  Eight only made the run three times
		-- longer and pushed the D8 past the testbench's own deadline.
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
		else
			b(16#A2#); b(16#C0#); b(16#20#); b(16#52#); b(16#FF#);   -- D8 (JSR $FF52)
			b(16#A2#); b(16#D0#); b(16#20#); b(16#52#); b(16#FF#);   -- D9 (JSR $FF52)
		end if;
		-- $FF49: the main loop no longer fits here (SEND_CMD starts at $FF52
		-- and the DMA call above ate three bytes), so the body lives at $FDC0
		-- and this is just the jump to it.  The JMP used to target $FF37, which
		-- WAS the loop head until commit 34a4eca inserted the verified-READ loop
		-- above and pushed the head down without moving the target: $FF37 is now
		-- the read loop's "DEC $2016", so the program fell back into it, ran
		-- another 255 READ6s and every one of them knocked the drive model out
		-- of H_PLAY -- which is why the CDDA stream died right after the D9 and
		-- scenario B saw no samples.
		chk(16#1F49#);
		b(16#4C#); b(16#C0#); b(16#FD#);              -- JMP $FDC0 (loop head)
		chk(16#1F4C#);

		-- ---------------- SEND_CMD @$FF50 ---------------------------------
		-- Sends the 10-byte command whose table starts at $FF00,X (the D8 at
		-- $FFC0 and the D9 at $FFD0), then eats the STATUS and MSGIN bytes.
		-- Commit 34a4eca dropped this routine when scenario C was added but left
		-- the two "JSR $FF50" calls in the boot: the CPU jumped into $EA filler,
		-- ran up into the command tables, hit a $00 (BRK), took the uninitialised
		-- BRK vector to $EAEA and never came back.  So the D8 has not reached the
		-- drive model since that commit, which is why scenarios B, G and F were
		-- unreachable.
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
		-- DMA_WRITE_PEND -> SLOT_WRITE -> AUTO_ACK chain).  This is the one big
		-- mechanism a savestate had never crossed.
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
		-- Two, not four: one sector would never cross a sector boundary (the
		-- drive's pacing path, which is where the model kept going wrong), and
		-- four just doubled an already hours-long run for no extra coverage.
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
		return r;
	end function;
	constant ROM : rom_t := rom_init;

	type mem_t is array (0 to 4*SLOT_WORDS + 64) of std_logic_vector(63 downto 0);

	-- CDDA sample log written by a monitor process, read by the scenario
	type slog_t is array (0 to 262143) of integer;
	signal slog_count : integer := 0;

	signal hps_d8_count : integer := 0;

	-- scenario I ($DD across a save/load)
	-- Drive latency for a $DD, in model ticks, set by the scenario.  Lowering it
	-- also shortens a stall already in flight, so releasing is immediate.
	signal i_lat       : integer := 2;
	-- Scenario I2: hold any $DD until the machine is actually FROZEN, then
	-- answer.  Tuning a latency instead was a race against the boundary wait,
	-- which varies with the VBLANK phase: the first attempt delivered the answer
	-- BEFORE the freeze and the setup guard correctly rejected the run.  This
	-- states the intent directly instead of trying to time it.
	signal i_freeze_ans : std_logic := '0';
	signal hps_dd_pend : std_logic := '0';	-- $DD accepted, answer not yet sent
	signal hps_dd_done : integer := 0;		-- $DD answers delivered
	signal i_cpu_done  : integer := 0;		-- $DD replies the CPU fully consumed
	-- $DD answers the drive delivered while the machine was FROZEN.  The first
	-- attempt sampled hps_dd_done on the rising edge of SLEEP instead, and read
	-- equal values every time: SLEEP blips low and high again around the end of
	-- a walk (visible as the two @save-freeze snapshots per save in scenario H),
	-- so the baseline was re-sampled AFTER the answer and the difference always
	-- vanished.  Counting the event itself has no such edge to get wrong.
	signal i_ans_frozen : integer := 0;

	-- instruction-boundary invariant (see st0_mon)
	signal ss_en_leak    : integer := 0;	-- CPU enabled while SLEEP asserted
	signal ss_state_bad  : integer := 0;	-- STATE moved after the freeze
	signal ss_blob_bad   : integer := 0;	-- a SAVED blob carries STATE /= 0

	-- FIFO overflow during the freeze (see frz_mon).  The fill paths stay live
	-- during SLEEP by design so nothing the HPS pushes is lost - but a full
	-- FIFO suppresses WR_REQ and the sample/byte is dropped with NO trace.
	-- Invisible at the 4 KB walk; the 256 KB walk freezes the consumer for
	-- ~25 ms while ~1100 CDDA samples and ~190 subcode bytes keep arriving.
	signal frz_cdda_drop : integer := 0;	-- CDDA samples lost while frozen
	signal frz_subc_drop : integer := 0;	-- subcode bytes lost while frozen
	signal frz_cdda_hi   : integer := 0;	-- peak CDDA FIFO backlog entering a freeze

	-- verdict flags raised by the monitors (checked by the stim process)
	-- index into the sample tape at the replay's LAST CDDA flush (REPLAY_START
	-- fires for the re-issued D8 and again for the D9 when the D8 was silent).
	-- Everything logged after it is the genuinely resumed stream, which is what
	-- makes B and G positional: without this anchor they only asked whether the
	-- expected value appears SOMEWHERE in the tape, and since the track loops,
	-- it does - even if the resume landed in the wrong place.
	signal flush_mark : integer := -1;
	-- p31: the silent-D8 pairing flushes TWICE per replay pair (RP_ISSUE and
	-- RP_ISSUE_D9), and the wrap-fix adds a second pair: keep a small history
	-- so the checkers can pick the right anchor (resume = 2nd post-load flush,
	-- wrap = 4th).
	type flushlog_t is array (0 to 7) of integer;
	signal flush_log : flushlog_t := (others => -1);
	signal flush_n   : integer := 0;

	signal c_fail : std_logic := '0';	-- scenario C: resumed stream misaligned
	signal e_fail : std_logic := '0';	-- scenario E: DTR/CD_DATA_CNT bookkeeping

	-- scenario W (push-status soak + fresh READ6)
	function b2sl(b : boolean) return std_logic is
	begin
		if b then return '1'; else return '0'; end if;
	end function;
	-- Hold the final READ6 STATUS strobe.  Starts ASSERTED in a W run: the
	-- push-status strobe would otherwise fire ~1.5 ms into read #1, clear
	-- READ_ACTIVE and close the quiet gate long before the stim process even
	-- reaches the W block (the same "state the intent, don't race it" lesson
	-- as i_freeze_ans).  The stim releases it to close each transaction.
	signal w_hold_stat  : std_logic := b2sl(G_DO_W or G_DO_W2 or G_DO_W3);
	-- ss_load rising edges, counted durably (the pulse is 3 CLK and the model
	-- polls at 50 us - the one-cycle-pulse trap).  In push mode the model uses
	-- it to go idle on a load: on hardware the HPS served the dead timeline's
	-- command and had its status consumed long before the user pressed load,
	-- so a load always finds the drive idle and the replay's re-issued command
	-- is the only one in flight.  Without this the model kept serving the
	-- PRE-load timeline's last command across the load and strobed its status
	-- into the restored machine - a phantom the real system cannot produce.
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
	-- state latched at every freeze (w_probe): p23b's AUD_OPEN veto defers a
	-- save requested in the audio window until the status lands, so the W3
	-- setup asserts judge the FREEZE-time state, not the request-time state
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
	-- registered command mailbox (the real TurboGrafx16.sv latches CD_COMM on
	-- the COMM_SEND edge; a bare pulse would be invisible inside the model's
	-- streaming wait loops)
	signal mbox_comm  : std_logic_vector(95 downto 0) := (others => '0');
	signal mbox_count : integer := 0;

	-- CD_DATA_END is a ONE-CYCLE pulse.  The drive model below is a polling
	-- process that only wakes on its own wait condition, so it could never
	-- observe the pulse directly: it therefore never learned that a sector had
	-- been fully consumed and never sent the next one.  Every multi-sector
	-- READ6 stalled at exactly 2048 bytes.  Counting the pulses here (same
	-- trick as the command mailbox) makes the event durable and wakeable.
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
		SS_SLEEP    => ss_sleep,
		SS_DBG      => ss_dbg,
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
	-- poll tick 50 us; status latency 2 ticks after D8, 0 after D9/DA)
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
		-- Sector pacing.  The drive must not run ahead of the reader: it pushes
		-- sector k only once the reader has actually consumed the k*2048 bytes
		-- before it, and it reports the final STATUS only once the WHOLE transfer
		-- has been consumed.  Pacing on CD_DATA_END instead is wrong - that pulse
		-- means "the FIFO went empty", which also happens mid-sector whenever the
		-- reader briefly outruns the fill.  Doing that made the model shovel all
		-- four sectors out at once, drop rdcnt to zero and raise the end-of-command
		-- STATUS while the reader was still ~2000 bytes from the end; the reader
		-- took the STATUS branch mid-stream and came back from its RTS with a
		-- garbage PC.
		alias rd_cons_hw is << signal .tb_cd.DUT.CD.SCSI.READ_CONSUMED : unsigned(20 downto 0) >>;
		alias rd_skip_hw is << signal .tb_cd.DUT.CD.SCSI.READ_SKIP : unsigned(11 downto 0) >>;
		-- every DATAIN handshake, whatever the command: how the $DD answer is
		-- paced on consumption rather than on the push
		alias din_cnt_hw is << signal .tb_cd.DUT.CD.SCSI.DATAIN_CNT : unsigned(15 downto 0) >>;
		variable pushed    : integer := 0;	-- bytes handed to the FIFO this command
		variable total     : integer := 0;	-- bytes the command will deliver
		-- Bytes the CORE has taken out of the SCSI FIFO for the command in flight.
		-- Pacing used to compare READ_CONSUMED directly against `pushed`, which is
		-- only valid while READ_CONSUMED starts each command at zero.  It does NOT
		-- after a REPLAYED READ6: the replay deliberately keeps the old count (the
		-- skip arithmetic in RP_RD_PREP is built on it), so the comparison was
		-- trivially true and the model shovelled the whole re-read into a 4096-byte
		-- FIFO at once.  Scenario C never saw it because its re-read is at most two
		-- sectors, exactly the FIFO's capacity; scenario H's 19-sector re-read
		-- overflowed it and the stream lost bytes ~4.5 KB in.
		--
		-- So rebase at the first push, and count the DRAINED bytes too: the replay
		-- discards the already-consumed head straight out of the FIFO without ever
		-- touching READ_CONSUMED, but those bytes did leave the FIFO.
		variable cons_base : integer := 0;
		variable skip0     : integer := 0;
		variable removed   : integer := 0;
		variable dd_pend   : boolean := false;	-- $DD accepted, answer owed
		variable dd_wait   : integer := 0;		-- ticks left before answering it
		-- $DD STATUS pacing.  The first version raised the STATUS in the same
		-- breath as the 10 data bytes, and SP_FREE prefers STAT_PEND
		-- (SCSI.vhd:496) over a non-empty FIFO (512): the STATUS won, the program
		-- closed the transaction, and the leftover DATAIN bytes sat in the FIFO
		-- re-raising the phase forever - a wedge that looked exactly like an RTL
		-- defect and was mine.  This is the same mistake docs/SIMULATION.md
		-- already records for the READ path ("lo STATUS finale subito dopo
		-- l'ultimo push invece che dopo l'ultima lettura").  Gate it on the bytes
		-- actually being handshaked away.
		variable dd_stat   : boolean := false;	-- data sent, STATUS owed
		variable dd_target : integer := 0;		-- DATAIN_CNT that means "all 10 taken"
		variable frame_val : integer;
		variable t_next    : time;
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
					if b1 = 0 then state := H_PAUSE; else state := H_PLAY; end if;
					hps_d8_count <= hps_d8_count + 1;
					report "HPS: D8 lba=" & integer'image(nlba) & " mode=" & integer'image(b1);
				elsif b0 = 16#D9# then
					if b9 / 64 = 0 then
						nlba := to_integer(unsigned(comm(31 downto 24))) * 65536
						      + to_integer(unsigned(comm(39 downto 32))) * 256
						      + to_integer(unsigned(comm(47 downto 40)));
					else
						nlba := pend_lba;
					end if;
					pend_lba := nlba; mode := b1;
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
			-- Push mode: a LOAD finds the real HPS idle (the dead timeline's
			-- command was served, and its status consumed, long before the user
			-- pressed load), and the replay's re-issued command is the only one
			-- in flight.  Drop the in-flight READ and any owed status here;
			-- keeping them alive across the load strobed a phantom status into
			-- the restored machine - one the real system cannot produce.
			-- (real-HPS mode is exempt BY DESIGN: the real HPS does not know a
			-- load happened, so head/state/count/latency/slot all survive it)
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
					-- consumption-gated serving: stream the next sector only once
					-- the reader has drained the ones before it (what every pre-W
					-- scenario was verified against)
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
				if state = H_PLAY and (not G_REAL_HPS or rh_lat = 0) then
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
					lba := lba + 1;
					if lba > pend_lba then
						if mode = 1 then
							lba := pstart;
						else
							state := H_IDLE;
							-- pcecdd playback-end: "if (int_pend) SendStatus(GOOD)"
							-- - the deferred SAPEP-INTERRUPT completion the game
							-- is parked on.  Never modelled before this line: every
							-- INTERRUPT-mode wedge was invisible to the matrix.
							if int_pend then
								pend_stat := true; stat_wait := 0;
								report "HPS: playback end - INTERRUPT status delivered";
							end if;
							int_pend := false;
						end if;
					end if;
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
		-- Keep sampling for the WHOLE run at a 5 ms pace.  The old version gave
		-- up after 100 samples, which left the reader's long verification phase
		-- completely dark: when the run later failed on "the D8 never arrived"
		-- there was no way to tell a wedged bus from a testbench deadline that
		-- was simply too short.  READ_CONSUMED is included because it is the
		-- one number that says whether the stream is still making progress.
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

	-- P5b replay monitor: log every re-issued command with the resume math
	-- (READ_CONSUMED, the computed re-read LBA/count, and the byte drain) so
	-- the scenario-C stream misalignment can be diagnosed.
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
				-- Scenario C verdict.  The drive model streams byte i == i mod
				-- 256 and the reader verifies every byte against its own
				-- counter, so $FE58 (the "record the first mismatch" store) can
				-- only ever be reached if the byte stream and the reader have
				-- drifted apart.  With no save/load in the picture that never
				-- happens; reaching it at all therefore IS the failure, whenever
				-- it fires.  Checking the flag instead of a blob word also makes
				-- the test non-vacuous: the old word-131 check could pass simply
				-- because the replay had not delivered a single byte yet by the
				-- time the replay blob was taken.
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

	fl_mon : process(clk)
		alias rp_flush is << signal .tb_cd.DUT.CD.REPLAY_START : std_logic >>;
		variable d : std_logic := '0';
	begin
		if rising_edge(clk) then
			if rp_flush = '1' and d = '0' then
				flush_log(flush_n mod 8) <= slog_count;
				flush_n <= flush_n + 1;
				flush_mark <= slog_count;
			end if;
			d := rp_flush;
		end if;
	end process;

	-- ------------------------------------------------------------------
	-- Boundary invariant: a savestate must never be taken while the SCSI bus
	-- still carries a phase.  The phase itself is already required to be
	-- SP_FREE by the quiet gate; BSY, however, stays asserted from the CPU's
	-- SEL until a status has been handshaked, and a SAPEP in LOOP or INTERRUPT
	-- mode is answered late or not at all.  Landing there used to freeze the
	-- restored machine (see the restore-side release in SCSI.vhd), so this
	-- watches that the restore does leave the bus usable.
	-- ------------------------------------------------------------------
	cp_mon : process
		alias bsyn  is << signal .tb_cd.DUT.CD.SCSI.BSY_Nr : std_logic >>;
		alias spenc2 is << signal .tb_cd.DUT.CD.SCSI.SP_ENC : std_logic_vector(3 downto 0) >>;
		alias cract is << signal .tb_cd.DUT.CD.SCSI.READ_ACTIVE : std_logic >>;
		alias ssload is << signal .tb_cd.DUT.CD.SCSI.SaveStateBus_load : std_logic >>;
		variable slp_d : std_logic := '0';
		-- only a LOAD has to leave the bus usable.  After a SAVE the machine
		-- simply carries on, and BSY asserted there is the perfectly legitimate
		-- state this whole defect is about - checking it then just fails on the
		-- pre-existing condition.
		variable was_load : boolean := false;
	begin
		loop
			wait until rising_edge(clk);
			if ssload = '1' then was_load := true; end if;
			if slp_d = '1' and ss_sleep = '0' and was_load then
				was_load := false;
				assert not (spenc2 = x"0" and cract = '0' and bsyn = '0')
					report "TBCD BUS STUCK: resumed with the phase idle but BSY still "
						& "asserted -- the program cannot select and will hang (t="
						& time'image(now) & ")"
					severity failure;
			end if;
			slp_d := ss_sleep;
		end loop;
	end process;

	-- ------------------------------------------------------------------
	-- Program-counter guard.  The reader was seen returning from its status
	-- routine to a garbage address (PC=D97B, then all-X), which is a derailed
	-- 6280, not a stream problem -- and by the time anything else noticed, the
	-- interesting moment was hundreds of sim-ms in the past.  Keep a ring of
	-- the recent PCs and dump it the instant the PC leaves the code the
	-- synthetic BIOS actually occupies ($FE00-$FFFF, plus the four bytes
	-- copied into CD-RAM at $6000).  The tail of the ring names the
	-- instruction that did it.
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

				-- Only the bytes the BIOS actually fills with CODE are legal.
				-- Everything else in the reset bank is $EA filler and the D8/D9/
				-- READ6 tables, which are DATA: a PC there means the program has
				-- already fallen off its own code, and letting it run on just
				-- flushes the ring with NOPs before the guard notices (the first
				-- attempt only caught it at $EAEA, after a stray $00 in a table
				-- had taken the uninitialised BRK vector).
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
	-- Stream watchdog.  A wedged READ used to cost a whole run: the machine
	-- would sit there for hundreds of sim-ms and only fail hours later on some
	-- distant deadline, with the interesting moment long gone from the log.
	-- Fail fast instead, right where the stream stopped moving.
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
	--   h_after writes actually happened AFTER the load - without it the whole
	--           check would pass on the pre-save part alone, which is exactly
	--           the "check after a SAVE instead of after a LOAD" mistake
	--           docs/SIMULATION.md lists.
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
			end if;
			slp_old := h_slp;

			-- Scenario A also saves and loads, and in the FULL matrix its last
			-- cycles can overlap the start of the DMA transfer -- each of those
			-- loads legitimately rewinds the ADPCM write address.  The scenario
			-- clears the counters once the DMA is unmistakably running, so the
			-- contiguity verdict below covers exactly the H save/load and nothing
			-- else.  (Without this the check would be a matrix-only false alarm,
			-- which is the failure mode docs/SIMULATION.md calls "invarianti
			-- scritte male": the monitor catching the twin event too.)
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
	-- Scenario H diagnostic.  h_mon says the resumed DMA is one byte late and
	-- that the byte sitting in the first post-load slot is the STALE one; this
	-- prints the two states that decide whether that is the staged-DATAIN-byte
	-- path, and whether the blob even carries enough to tell the two cases
	-- apart:
	--
	--   at the SAVE freeze  - was the DMA's ACK round-trip already in flight
	--                         (SCSI_ACK_N='0' / AUTO_ACK='1') for the byte the
	--                         DMA had already stored?  That is the candidate
	--                         discriminator between "the DMA already took this
	--                         byte" and "it still has to", which is exactly what
	--                         a fix has to key on so the CPU path is untouched.
	--   at the FIRST write after the LOAD - what released the hold
	--                         (READ_SKIP just hit 0?  DATAIN_HELD low?) and what
	--                         was in DBO at that instant.
	--
	-- Diagnostic only: it reports, it never fails.  A monitor that also judges
	-- is how this testbench got its false positives.
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
	-- w_mon counts program landmarks off PC TRANSITIONS (the pc_mon lesson):
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
		-- W2 forensics: the tallies the hardware dump exposed (REQ/ACK ledger,
		-- swallow, status strobes vs pends, mailbox sends vs game SELs)
		alias wswl is << signal .tb_cd.DUT.CD.SCSI.SWALLOW : unsigned(1 downto 0) >>;
		alias wstp is << signal .tb_cd.DUT.CD.SCSI.STAT_PEND : std_logic >>;
		alias wnrq is << signal .tb_cd.DUT.CD.SCSI.CNT_REQ : unsigned(15 downto 0) >>;
		alias wnak is << signal .tb_cd.DUT.CD.SCSI.CNT_ACK : unsigned(15 downto 0) >>;
		alias wnsp is << signal .tb_cd.DUT.CD.SCSI.CNT_STATPEND : unsigned(7 downto 0) >>;
		alias wncs is << signal .tb_cd.DUT.CD.SCSI.CNT_COMMSEND : unsigned(7 downto 0) >>;
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
				& " STAT_PENDS=" & integer'image(to_integer(wnsp))
				& " MBOX=" & integer'image(mbox_count)
				& " COMMSENDS=" & integer'image(to_integer(wncs))
				& " SELS=" & integer'image(nsel)
				& " REQ=" & integer'image(to_integer(wnrq))
				& " ACK=" & integer'image(to_integer(wnak))
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
				-- W2 landmarks, with a forensic tally at every boundary
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
	-- Scenario W probe: the drain/status boundary, reported rather than judged.
	--   * at every freeze: the SCSI counters the blob is about to capture;
	--   * at every drain end (READ_SKIP 1->0): EMPTY / STAT_PEND / SP / DBO -
	--     a non-empty FIFO with the skip at zero is the orphan-byte smoking
	--     gun before any program-visible symptom;
	--   * the first 12 DATAIN handshakes after a drain, with the byte the CPU
	--     was actually staged (which bytes were stale, read off directly).
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
	-- The instruction-boundary invariant, which NO scenario checked until the
	-- hardware forced it.  A blob dumped off the MiSTer had the CPU captured at
	-- STATE=1, mid-JSR (IR=$20), with the header's forced flag CLEAR - so the
	-- composite boundary had accepted it.  Everything in the CD block was
	-- healthy; the game wedged on load purely because the CPU's mid-instruction
	-- internals (AA, DR, T, MC, NEXT_STATE, ALU_*) are NOT in the blob.  They
	-- are only don't-care while every save lands at STATE=0, and that is exactly
	-- the assumption that broke.
	--
	-- Mechanism this watches for: CPU_CE is REGISTERED, and HUC6280.vhd drives
	-- the core with `EN <= CPU_CE and CPU_RDY` - no SLEEP term - while
	-- HUC6280_CPU has no SLEEP port at all.  The freeze therefore lets ONE
	-- already-registered CE through after the boundary was accepted, and the CPU
	-- steps off the boundary it was frozen on.
	--
	-- Two independent detectors, because the second one only fires when the
	-- leaked step happens to change STATE:
	--   ss_en_leak   - the core was enabled while SLEEP was asserted
	--   ss_state_bad - STATE moved after the freeze engaged
	-- ------------------------------------------------------------------
	-- ------------------------------------------------------------------
	-- Freeze-overflow monitor.  A CDDA sample completes when CD_AUDIO_WR rises
	-- on the 4th byte (CD_BYTE_CNT="11"); a subcode byte when CD_SUBCD_WR
	-- rises.  If the corresponding FIFO is FULL at that moment the write
	-- request is suppressed inside cd.vhd and the data is silently gone.
	-- Counted only while ss_sleep='1': during normal running a full FIFO is
	-- the HPS's pacing problem, during the freeze it is OUR defect - the
	-- consumer is frozen while the fill is deliberately kept alive.
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
			-- max_wait: freeze-engage deadline in 5 ms steps (default 4 s, the
			-- historical value).  W3 passes a tight one: p23b's AUD_OPEN veto is
			-- EXPECTED to defer the save by ~G_W3_D8LAT_MS, but a veto that
			-- never releases must be a FAIL, not a silent 4 s crawl.
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
				-- wait for the freeze, reporting the boundary-block mask
				-- every 5 sim-ms: the mask is the live diagnosis of WHICH
				-- composite term keeps the boundary rare
				for i in 1 to max_wait loop
					wait until ss_sleep = '1' for 5 ms;
					exit when ss_sleep = '1';
					report "TBCD WAIT(" & tag & ") t=" & time'image(now - t0)
						& " SS_DBG(frc,!st0,res,ce,!vbl,b0,b1,ext,cd)=" & to_string(ss_dbg);
				end loop;
				assert ss_sleep = '1'
					report "TBCD HANG(" & tag & "): freeze never engaged; SS_DBG="
						& to_string(ss_dbg) severity failure;
				wait until ss_sleep = '0' for 500 ms;
				assert ss_sleep = '0'
					report "TBCD HANG(" & tag & "): walk/refetch never completed; SS_DBG="
						& to_string(ss_dbg) severity failure;
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
			variable mark    : integer;
			variable exp, got : integer;
			variable idx      : integer;
		begin
			wait_cycles(20);
			reset <= '0';
			-- armed from the start: no ADPCM RAM write can happen before the DMA
			-- routine runs (the program never writes $180A), and the walk's own
			-- writes are excluded by the SLEEP gate inside h_mon
			h_arm <= '1';
			-- boot: the program runs its 8-read verification phase first;
			-- anchor scenario A right in the middle of it so the symmetric
			-- cycles exercise the mid-READ save/load path (scenario C class)
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
				elsif w = 48 then
					if (slot_word(1, w) and x"000000FFFFFFFFFF")
					 /= (slot_word(2, w) and x"000000FFFFFFFFFF") then
						diff := diff + 1;
					end if;
				elsif w = 56 then
					-- word index = eReg + 1 (word 0 is the header).  eReg 55 =
					-- SCSI_1: mask the forensic starvation taps (63:60), which
					-- track wall-clock history, not machine state
					if (slot_word(1, w) and x"0FFFFFFFFFFFFFFF")
					 /= (slot_word(2, w) and x"0FFFFFFFFFFFFFFF") then
						diff := diff + 1;
						report "TBCD A: SCSI_1 non-forensic fields differ" severity error;
					end if;
				elsif w = 66 or w = 67 then
					-- eRegs 65/66: save-only forensic counters (phase history,
					-- REQ/ACK ledger) - wall-clock history by design, skip
					null;
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
			-- + PSGWF24 = 25241 (the old 25141 was a mis-add: it read 100
			-- words early, inside BRM).  NOTE: this program leaves ADPCM RAM
			-- all-zero, so today A2 only proves the shuttle is DETERMINISTIC
			-- (anchor==replay); scenario D seeds real ADPCM data to make it
			-- a genuine content round-trip.
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
			-- Cycle 2 - the tail window (prime suspect): every data byte
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
			-- and the post-load wait passes vacuously (the I2 baseline lesson).
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
			-- Scenario W2 tail: the FMV -> stage-0 TRANSITION.  READ6 #1 just
			-- closed clean (the hardware forensic says the first fresh READ6 is
			-- perfect).  The program now runs the loader shape: D8+D9 (CDDA
			-- starts), a busy-wait audio window that save #3 lands in, then
			-- READ6 #2 and #3 back-to-back.  The hardware residual parks
			-- forever in the wait-for-STATUS poll right here, with one dangling
			-- REQ in the ledger - the TALLY reports at each boundary are the
			-- sim counterpart of that dump.
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
			-- SELs READ6 #2 - the transaction the hardware dies in
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
			-- Scenario W3 tail: the transition on the REAL HPS model.  The
			-- game's D8 (distant SAPSP) gets a seek latency: its status is
			-- PENDED in the single deferred slot and FROZEN until the seek
			-- expires.  Save #3 lands in that window; the load's replay then
			-- re-issues the D8 against the still-occupied slot.  Expected on
			-- the residual: PendStatus OVERWRITE (the game's status is lost
			-- forever), the survivor is delivered once and SWALLOWed as the
			-- replay's - and the restored program parks in SEND_CMD's status
			-- poll at $FF78 with nothing left to wait for.
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
			-- the save REQUEST goes out while the D8 status is pended + frozen
			-- in the model's slot; p23b's AUD_OPEN veto is then EXPECTED to
			-- defer the actual freeze until the status has landed (~the
			-- G_W3_D8LAT_MS seek), so the deadline is tight: 100 x 5 ms =
			-- 500 ms, and a veto that never releases is a FAIL.
			assert hps_slot = '1'
				report "TBCD W3 FAILED: setup - the D8 status was already "
					& "delivered before the save was even requested; raise "
					& "G_W3_D8LAT_MS" severity failure;
			do_ss(3, false, "W3 save #3 D8-seek-window", 100);
			assert w_frz_slot = '0' and w_frz_audopen = '0'
				report "TBCD W3 FAILED: the freeze landed INSIDE the audio window "
					& "(at-freeze SLOT=" & std_logic'image(w_frz_slot)
					& " AUD_OPEN=" & std_logic'image(w_frz_audopen)
					& ") - p23b's AUD_OPEN veto did not hold the boundary"
				severity failure;
			report "TBCD W3: save #3 was requested in the D8 window and correctly "
				& "DEFERRED past it (at-freeze SLOT='0'/AUD_OPEN='0')";
			wait_cycles(50_000);
			do_ss(3, true, "W3 load #3 D8-seek-window");
			-- post-walk baselines (dead-timeline rule).  NOT w2_audio: p23b's
			-- veto defers the freeze PAST the D8 status, so the blob is taken
			-- with D8+D9 already closed and the $FAE0 marker already executed -
			-- the restored timeline resumes in the busy-wait and can never
			-- re-fire it.  The first landmark the restored program CAN reach
			-- is READ6 #2 closing.
			cons_a := w2_read2;
			got    := w2_end;
			-- Does READ6 #2 ever close for the RESTORED program?  The path is
			-- long and legitimate: busy-wait remainder, then the p23 status
			-- serialization (the replay D8's pended status takes the FULL
			-- G_W3_D8LAT_MS seek before its swallow drains and the D9 goes
			-- out), then the game's SEL, a G_SEEK_MS seek, the 16 ms cadence
			-- and ~21 ms of consumption - measured ~300 ms end to end.  Break
			-- early only on PROVEN death: nothing pended anywhere and the
			-- conversation not moving (DATAIN_CNT static) for 100 ms - longer
			-- than any legitimate quiet stretch (the seek + one cadence tick,
			-- ~70 ms; the serialization window keeps SWALLOW/=0 and is exempt).
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
				exit when still >= 1000;	-- 100 ms of proven stillness
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
			-- The routine at $FD00 has enabled ADPCM_DMA_EN and issued a 256-
			-- sector READ6; the CD block is now shovelling the DATAIN stream
			-- straight into ADPCM RAM with no CPU involvement.  Save about a
			-- fifth of the way in, so the freeze has the remaining ~17 ms of
			-- streaming to land inside.
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
			-- The replay rebuilds an in-flight READ6 and re-issues the audio
			-- commands, and does NOTHING for anything else: at the end of
			-- RP_SETTLE the dispatch is AUDIO_ACTIVE / READ_ACTIVE / else
			-- RP_PAUSE, and RP_PAUSE only flushes.  So a command the drive has
			-- accepted but not yet answered is simply dropped by the restore.
			--
			-- Arm the stall, catch the machine while the drive owes it an
			-- answer, save there, and see whether the program ever gets a $DD
			-- answered again.  If the bus comes back wedged it never will - and
			-- "stuck in its own loading routine" is the hardware symptom.
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
					& " in 300 ms).  The in-flight command was dropped by the restore "
					& "and nothing rebuilds it: the bus never comes back."
				severity failure;
			report "TBCD: scenario I PASSED (" & integer'image(i_cpu_done - idx)
				& " $DD replies completed after the load).  I is the variant where "
				& "the drive was still holding the answer; I2 below is the one "
				& "where it had already been delivered into the freeze.";

			----------------------------------------------------------------
			if G_DO_I2 then
			report "TBCD: === scenario I2: the $DD answer arrives DURING the freeze ===";
			-- I proved the benign variant: the drive still owed the answer when
			-- the machine came back, so the restored CPU got it.  On hardware
			-- that is the exception - the HPS answers in about a millisecond and
			-- the walk takes ~25 ms, so the answer normally lands while the
			-- machine is frozen, goes into the SCSI FIFO, and REPLAY_START
			-- flushes it on the way out.  Nothing re-issues the command, so the
			-- restored CPU is left polling for a reply that was delivered to a
			-- timeline that no longer exists.
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
			-- 800 ms, not 300: SP_STARVE abandons a parked bus phase after about
			-- 780 ms, and that valve has never fired in any test.  A shorter wait
			-- cannot tell "wedged forever" from "wedged until the valve rescues
			-- it", and those call for completely different fixes.
			if i_cpu_done < idx + 2 then
				wait until i_cpu_done >= idx + 2 for 800 ms;
			end if;
			assert i_cpu_done >= idx + 2
				report "TBCD I2 FAILED: after the load the program never completed "
					& "another $DD (" & integer'image(i_cpu_done - idx)
					& " in 800 ms, so not even SP_STARVE rescued it).  The answer was "
					& "consumed by the pre-save timeline "
					& "and flushed with the FIFO; nothing re-issues the command, so "
					& "the program is stuck in its own polling loop - the hardware "
					& "symptom." severity failure;
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
			do_ss(3, false, "J baseline save");
			idx  := to_integer(unsigned(slot_word(3, 1)(31 downto 24)));
			cons_g := j_cnt;
			j_wait;
			do_ss(3, false, "J second save");
			exp := j_cnt;
			got := (exp - cons_g + 256) mod 256;
			report "TBCD J: interrupts in window, baseline = " & integer'image(got)
				& " (S=" & integer'image(to_integer(unsigned(slot_word(3, 1)(31 downto 24)))) & ")";
			-- The boot programs only CR, never the VDC timing registers, so the
			-- frame is degenerate and VBL runs at ~1.3 kHz here, not 60 Hz - the
			-- first version asserted an absolute 2..12 "frame rate" and failed
			-- on a perfectly healthy handler.  The verdict must be a RATIO
			-- against this measured baseline: the storm is x40, not x1.
			-- (Harsher than the real machine, if anything: more interrupts per
			-- save/load boundary, same clear path.)
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

			----------------------------------------------------------------
			report "TBCD: === scenario B: CDDA sample-exact resume ===";
			----------------------------------------------------------------
			wait for 1 ms;
			do_ss(3, false, "B save mid-music");
			v := slot_word(3, 64);				-- CD_6 (eReg 63)
			cons_a := to_integer(unsigned(v(58 downto 27)));
			report "TBCD B: consumed at save = " & integer'image(cons_a);
			wait for 1 ms;						-- music diverges live

			mark := slog_count;
			do_ss(3, true, "B load mid-music");
			-- p31: wait long enough for the wrap-fix pair too
			-- the machine resumes playing the STALE pre-load FIFO first; the
			-- replay repositions the stream ~35ms later (settle + holdgap +
			-- D8 + gap + D9 + drain).  Wait past all of it AND past the
			-- p31 wrap-fix pair (window end + reissue), then SCAN the
			-- sample log for a 32-long consecutive run starting at cons_a:
			-- the stale tail is flushed at the replay D8 long before its
			-- values could reach cons_a, so a match is the genuine resume.
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
			-- p31 anchors: count the post-load flushes.  Silent-D8 pairing
			-- flushes at RP_ISSUE and RP_ISSUE_D9 (2 per pair); the wrap-fix
			-- adds a second pair.  Resume = 2nd post-load flush; wrap = 4th.
			nf := 0;
			wrapa := -1;
			anchor := flush_mark;
			for i in 0 to 7 loop
				fl_tmp(i) := -1;
			end loop;
			for i in 0 to 7 loop
				if flush_log(i) > mark and flush_log(i) /= -1 then
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
			if nf >= 2 then
				anchor := fl_tmp(1);
			elsif nf = 1 then
				anchor := fl_tmp(0);
			end if;
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
				if slog(i) = exp then idx := i; exit; end if;
			end loop;
			report "TBCD B: flush at tape index " & integer'image(anchor)
				& ", exp=" & integer'image(exp)
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
				if slog(i) = exp then
					got := i;
					for k in 1 to 31 loop
						if slog(i + k) /= (exp + k) mod LOOP_SAMPLES then
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
			-- Report the actual offset: "passed" with a non-zero offset is NOT
			-- sample-exact, and the requirement for this project is that it be.
			report "TBCD: scenario B PASSED (resume at " & integer'image(exp)
				& ", stream started " & integer'image(idx - flush_mark)
				& " samples from the flush; 0 = sample-exact)";
			-- p31: the loop-fix re-issues the audio pair at the wrap point,
			-- adding a SECOND flush.  Verify it: the post-wrap stream must
			-- restart at the loop START (the broken-record fix) and the fix
			-- must have fired within ~2 sectors of the true window end.
			if wrapa > 0 then
				got := -1;
				for i in wrapa to minimum(wrapa + 64, slog_count - 33) loop
					if slog(i) = 0 then
						got := i;
						for k in 1 to 31 loop
							if slog(i + k) /= k then
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
				-- flushes are an unphased tail: never measure there)
				werr := LOOP_SAMPLES - 1 - slog(wrapc - 1);
				assert werr <= 1200 and werr >= -1200
					report "TBCD B FAILED: the wrap-fix fired " & integer'image(werr)
						& " samples away from the window end"
					severity failure;
				report "TBCD B: wrap-fix verified (restart at the loop start, "
					& integer'image(werr) & " samples before the end)";
			end if;


			----------------------------------------------------------------
			if G_DO_G then
			report "TBCD: === scenario G: CDDA resume AFTER a loop wrap ===";
			-- B saves a few sectors into the track, so the replay's fold-back
			-- path (RP_LOOPMOD) never runs: the consumed position is still
			-- inside the first pass.  A game left on a looping track for a
			-- while is the untested case, and it is the remaining candidate for
			-- the hardware report "audio went into a loop".
			--
			-- The drive plays the loop window INCLUSIVE of the D9 end sector
			-- (tb model, pcecdd semantics: "lba+1 > pend_lba -> lba := pstart"),
			-- so the window is END-START+1 sectors.  The played sample value is
			-- the frame index inside that window, which is the oracle here: a
			-- fold-back that uses the wrong window length lands the resumed
			-- stream somewhere else entirely, and the error grows with the
			-- number of wraps the save sits behind.
			----------------------------------------------------------------
			wait for 8 ms;						-- a wrap costs ~2.5 ms of model time
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
			wait for 140 ms;	-- p31: include the wrap-fix pair
			assert slog_count > mark + 64
				report "TBCD G FAILED: no samples after load (replay dead?)" severity failure;

			assert flush_mark > mark
				report "TBCD G FAILED: the replay never flushed the CDDA stream"
				severity failure;
			-- p31 anchors: count the post-load flushes.  Silent-D8 pairing
			-- flushes at RP_ISSUE and RP_ISSUE_D9 (2 per pair); the wrap-fix
			-- adds a second pair.  Resume = 2nd post-load flush; wrap = 4th.
			nf := 0;
			wrapa := -1;
			anchor := flush_mark;
			for i in 0 to 7 loop
				fl_tmp(i) := -1;
			end loop;
			for i in 0 to 7 loop
				if flush_log(i) > mark and flush_log(i) /= -1 then
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
			if nf >= 2 then
				anchor := fl_tmp(1);
			elsif nf = 1 then
				anchor := fl_tmp(0);
			end if;
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
				if slog(i) = exp then idx := i; exit; end if;
			end loop;
			report "TBCD G: flush at tape index " & integer'image(anchor)
				& ", exp=" & integer'image(exp)
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
				if slog(i) = exp then
					got := i;
					for k in 1 to 31 loop
						if slog(i + k) /= (exp + k) mod LOOP_SAMPLES then
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
			report "TBCD: scenario G PASSED (loop-wrapped resume at " & integer'image(exp)
				& ", stream started " & integer'image(idx - flush_mark)
				& " samples from the flush; 0 = sample-exact)";
			-- p31: the loop-fix re-issues the audio pair at the wrap point,
			-- adding a SECOND flush.  Verify it: the post-wrap stream must
			-- restart at the loop START (the broken-record fix) and the fix
			-- must have fired within ~2 sectors of the true window end.
			if wrapa > 0 then
				got := -1;
				for i in wrapa to minimum(wrapa + 64, slog_count - 33) loop
					if slog(i) = 0 then
						got := i;
						for k in 1 to 31 loop
							if slog(i + k) /= k then
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
				-- flushes are an unphased tail: never measure there)
				werr := LOOP_SAMPLES - 1 - slog(wrapc - 1);
				assert werr <= 1200 and werr >= -1200
					report "TBCD G FAILED: the wrap-fix fired " & integer'image(werr)
						& " samples away from the window end"
					severity failure;
				report "TBCD G: wrap-fix verified (restart at the loop start, "
					& integer'image(werr) & " samples before the end)";
			end if;

			end if;  -- G_DO_G

			----------------------------------------------------------------
			report "TBCD: === scenario F: BRAM (page $F7) save/load round-trip ===";
			-- The CD testbench used to stub BRM_DO=0, so a broken region-7
			-- walk was invisible and reached hardware: wiping the SD .brm
			-- broke loads (the snapshot was leaning on persistent BRAM).
			-- Model a real 2KB BRAM, save it, WIPE it, load, and require the
			-- blob ALONE to restore it bit-exact.  BRM region = blob words
			-- 24961..25216 (256 words; region byte n is at bits (n mod 8)*8).
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
			-- WIPE the live BRAM (the user's SD .brm delete): only the load's
			-- region-7 walk can bring it back now
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
					& "(P5b resume-READ misalignment)" severity failure;
			report "TBCD: scenario C PASSED (stream stayed aligned across every load)";

			assert e_fail = '0'
				report "TBCD E FAILED: CD_DATA_CNT and READ_CONSUMED disagree after a resume"
				severity failure;
			report "TBCD: scenario E PASSED (DTR/CD_DATA_CNT bookkeeping survives the resume)";

			-- The instruction-boundary invariant, checked over every save the run
			-- took.  This is what the hardware caught and the matrix did not.
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
