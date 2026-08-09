library STD;
use STD.TEXTIO.ALL;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_TEXTIO.all;
use IEEE.NUMERIC_STD.ALL;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity SCSI is
	port(
		RESET_N		: in std_logic;
		CLK 			: in std_logic;
		
		DBI			: in std_logic_vector(7 downto 0);
		DBO			: out std_logic_vector(7 downto 0);
		SEL_N			: in std_logic;
		ACK_N			: in std_logic;
		RST_N			: in std_logic;
		BSY_N			: out std_logic;
		REQ_N			: out std_logic;
		MSG_N			: out std_logic;
		CD_N			: out std_logic;
		IO_N			: out std_logic;
		
		STATUS		: in std_logic_vector(7 downto 0);
		MESSAGE		: in std_logic_vector(7 downto 0);
		STAT_GET		: in std_logic;
		
		COMMAND		: out std_logic_vector(95 downto 0);
		COMM_SEND	: out std_logic;
		
		DOUT_REQ		: in std_logic;
		DOUT			: out std_logic_vector(79 downto 0);
		DOUT_SEND	: out std_logic;
		
		CD_DATA		: in std_logic_vector(7 downto 0);
		CD_WR			: in std_logic;
		CD_DATA_END	: out std_logic;
		STOP_CD_SND	: out std_logic;
		
		DBG_DATAIN_CNT: out unsigned(15 downto 0);

		-- P5b: the CPU-facing FSM is held (replay in flight / stream drain) with
		-- a DATAIN byte staged.  cd.vhd must hide "data ready" from the program
		-- for the duration - see the DATAIN_HOLD comment below.
		DATAIN_HELD	: out std_logic;

		-- Savestates (plan P5a; slots 55-59 in pce_savestates_pkg)
		SLEEP				: in std_logic := '0';
		-- Raw savestate sleep REQUEST (pce_top sleep_ss): high while the engine
		-- is waiting for the composite boundary.  Arms the phase-starvation
		-- valve - the valve exists only to unwedge the SS_QUIET gate, and armed
		-- on the live machine it was tearing down legitimately parked phases
		-- (391 ms, not the commented 780) during ordinary CD loading.
		SS_PEND			: in std_logic := '0';
		SaveStateBus_Din  : in  std_logic_vector(63 downto 0) := (others => '0');
		SaveStateBus_Adr  : in  std_logic_vector(9 downto 0) := (others => '0');
		SaveStateBus_wren : in  std_logic := '0';
		SaveStateBus_rst  : in  std_logic := '0';
		SaveStateBus_load : in  std_logic := '0';
		SaveStateBus_Dout : out std_logic_vector(63 downto 0);
		-- CD-quiet: safe to take the composite boundary
		SS_QUIET				: out std_logic;
		-- P5c CDDA replay: sample-exact resume of the audio stream
		CDDA_CONSUMED		: in  unsigned(31 downto 0) := (others => '0');
		AUDIO_CMD_SET		: out std_logic;	-- pulse: D8 accepted, reset the consumed counter
		REPLAY_SKIP			: out unsigned(9 downto 0);	-- samples to discard after replay
		REPLAY_START		: out std_logic;	-- pulse: flush CDDA/SUBC FIFOs, arm the skip
		-- P5d: "the DATAIN byte staged at the save had ALREADY been taken by the
		-- ADPCM DMA".  Latched by cd.vhd straight out of the blob at the load
		-- pulse - see the comment there for why it cannot be sampled live.
		DMA_STAGED			: in  std_logic := '0'
	);
end SCSI;

architecture rtl of SCSI is
	
	type SCSIPhase_t is (
		SP_FREE,
		SP_COMM_BEFOREREQ,
		SP_COMM_START,
		SP_COMM_END,
		SP_STAT_START,
		SP_STAT_END,
		SP_STAT_HOLD,
		SP_MSGIN_START,
		SP_MSGIN_END,
		SP_MSGIN_HOLD,
		SP_DATAIN_START,
		SP_DATAIN_END,
		SP_DATAOUT_START,
		SP_DATAOUT_END
	);
	signal SP 			: SCSIPhase_t; 
	
	signal BSY_Nr 		: std_logic;
	signal MSG_Nr 		: std_logic;
	signal CD_Nr 		: std_logic;
	signal IO_Nr 		: std_logic;
	signal REQ_Nr 		: std_logic;
--	signal TR_DONE		: std_logic;
--	signal TR_RDY		: std_logic;
	
	type CommBuf_t is array (0 to 11) of std_logic_vector(7 downto 0);
	signal COMM 		: CommBuf_t;
	signal COMM_POS 	: unsigned(3 downto 0);
	signal COMM_OUT 	: std_logic;
	type CommLen_t is array (0 to 15) of unsigned(3 downto 0);
	constant COMM_LEN : CommLen_t :=
	("0110", "0110", "1010", "1010", "1010", "1010", "1010", "1010", "1010", "1010", "1100", "1100", "1010", "1010", "1010", "1010"); 

	type DataBuf_t is array (0 to 9) of std_logic_vector(7 downto 0);
	signal DATA_BUF 	: DataBuf_t;
	signal DATA_POS	: unsigned(3 downto 0);
	signal DATA_OUT	: std_logic;
	
	signal FULL 		: std_logic;
	signal EMPTY		: std_logic;
	signal FIFO_RD_REQ: std_logic;
	signal FIFO_WR_REQ: std_logic;
	signal FIFO_D 		: std_logic_vector(7 downto 0);
	signal FIFO_Q 		: std_logic_vector(7 downto 0);
	signal CD_WR_OLD 	: std_logic;
	signal STAT_PEND 	: std_logic;
	signal STOP_CD_SND_r : std_logic;
	signal DBO_r        : std_logic_vector(7 downto 0);
	signal DOUT_PEND  : std_logic;
	
	signal DATAIN_CNT 	: unsigned(15 downto 0);

	signal STAT_COUNT    : unsigned(15 downto 0);
	signal DELAY_COUNT   : unsigned(16 downto 0);

	--P5c CDDA replay
	signal AUDIO_ACTIVE  : std_logic;
	signal AUDIO_MODE    : std_logic_vector(1 downto 0);	-- COMM(9)(7:6): 00=LBA 01=MSF 10=track
	signal AUDIO_B1      : std_logic_vector(1 downto 0);	-- D8 byte1: CDDA play mode (values 0-3)
	signal AUDIO_B2, AUDIO_B3, AUDIO_B4, AUDIO_B5 : std_logic_vector(7 downto 0);
	--P5c2: the effective audio state is D8 (start) + the last D9 (end point and
	--play mode) — the HPS rebuilds CDDAEnd/CDDAMode from BOTH, so both are
	--latched raw and both are re-issued on load.
	signal D9_SEEN       : std_logic;
	signal D9_B1         : std_logic_vector(7 downto 0);	-- D9 byte1: end play mode
	signal D9_MODE       : std_logic_vector(1 downto 0);	-- D9 byte9(7:6)
	signal D9_B2, D9_B3, D9_B4, D9_B5 : std_logic_vector(7 downto 0);
	--P5b READ6 replay
	signal READ_ACTIVE   : std_logic;
	-- P5e: a QUERY command ($DD READ SUBQ, $DE CD_DINFO, ...) is outstanding.
	-- Those are neither audio nor READ6, and the replay dispatch has no arm for
	-- them: the restore dropped them silently and the program sat in its own
	-- polling loop forever with the bus idle - the scenario I signature.
	signal QRY_ACTIVE    : std_logic;
	signal READ_CONSUMED : unsigned(20 downto 0);	-- bytes handed to the CPU since the 08 accept
	signal READ_SKIP     : unsigned(11 downto 0);	-- replayed-stream bytes still to drain
	signal SWALLOW       : unsigned(1 downto 0);	-- replay-generated statuses still to eat
	-- relief valves: a lost replay command must not wedge the boundary forever
	signal RSKIP_STARVE  : unsigned(23 downto 0);	-- drain waited too long for stream data
	signal RSKIP_GAP     : std_logic := '0';	-- idle cycle after a drain pop (EMPTY is registered)
	signal SWAL_AGE      : unsigned(27 downto 0);	-- swallowed status never arrived
	signal SP_STARVE     : unsigned(24 downto 0);	-- bus phase with no CPU progress
	signal SP_STARVE_HIT : unsigned(2 downto 0) := (others => '0');	-- forensic: live overflows (valve NOT armed), saturating
	signal SEL_FLUSH     : std_logic := '0';	-- 1-CLK FIFO flush: game SEL while a replay drain was still pending
	-- forensic eReg 65 (phase-history + counters, save-only)
	signal SP_HIST        : std_logic_vector(31 downto 0) := (others => '0');
	signal SP_ENC_D2      : std_logic_vector(3 downto 0) := (others => '0');
	signal CNT_STATGET    : unsigned(7 downto 0) := (others => '0');
	signal CNT_SEL        : unsigned(7 downto 0) := (others => '0');
	signal CNT_MSGIN      : unsigned(7 downto 0) := (others => '0');
	signal CNT_STATPEND   : unsigned(7 downto 0) := (others => '0');
	signal DBG_STATGET_D  : std_logic := '0';
	signal DBG_SEL_D      : std_logic := '1';
	signal DBG_STATPEND_D : std_logic := '0';
	signal SS_V_Dout_DBG  : std_logic_vector(63 downto 0);
	-- forensic eReg 66: handshake ledger
	signal CNT_REQ        : unsigned(15 downto 0) := (others => '0');
	signal CNT_ACK        : unsigned(15 downto 0) := (others => '0');
	signal CNT_FIFOWR     : unsigned(15 downto 0) := (others => '0');
	signal CNT_COMMSEND   : unsigned(7 downto 0) := (others => '0');
	signal LAST_ACK_SP    : std_logic_vector(3 downto 0) := (others => '0');
	signal LAST_REQ_SP    : std_logic_vector(3 downto 0) := (others => '0');
	signal DBG_REQ_D      : std_logic := '1';
	signal DBG_ACK_D      : std_logic := '1';
	signal DBG_DE_D       : std_logic := '0';
	signal DBG_CS_D       : std_logic := '0';
	signal SS_V_Dout_DBG2 : std_logic_vector(63 downto 0);
	signal SP_STARVE_ABT : std_logic := '0';	-- forensic: armed valve actually dropped a phase
	signal STAT_GET_D    : std_logic := '0';	-- edge detect: one status per strobe
	type Replay_t is ( RP_IDLE, RP_WAIT, RP_CVT_START, RP_DIV588, RP_CVT_END, RP_LOOPMOD,
	                   RP_HOLDGAP, RP_ISSUE, RP_GAP, RP_ISSUE_D9, RP_LF_CALC,
	                   RP_RD_PREP, RP_RD_ISSUE, RP_PAUSE,
	                   RP_QRY_PREP, RP_QRY_ISSUE );
	signal RP            : Replay_t := RP_IDLE;
	signal RP_AFTER      : Replay_t := RP_IDLE;	-- successor after RP_HOLDGAP
	signal RP_DELAY      : unsigned(20 downto 0);
	signal RP_ACC        : unsigned(31 downto 0);	-- working accumulator
	signal RP_SECT       : unsigned(23 downto 0);	-- consumed sectors, then resume LBA
	signal RP_START_LBA  : unsigned(23 downto 0);
	signal RP_CONS       : unsigned(31 downto 0);	-- consumed AT RESTORE (pre stale-tail drift)
	signal RP_LEN        : unsigned(23 downto 0);	-- loop length in sectors (0 = unknown)
	signal RP_TRACKMODE  : std_logic;
	signal RP_COMM_OVR   : std_logic := '0';
	signal RP_COMM       : std_logic_vector(95 downto 0);
	signal RP_SEND       : std_logic := '0';
	signal RP_START_r    : std_logic := '0';
	signal RP_FIFO_HOLD  : std_logic := '0';	-- clear the data FIFO until the replay command is out
	signal RP_SW_INC     : std_logic := '0';
	signal RP_RSKIP      : unsigned(11 downto 0);
	signal RP_RSKIP_SET  : std_logic := '0';
	signal RP_TAILDRAIN  : std_logic := '0';	-- tail-case drain: CPU owed nothing, flush at end
	-- Rescue retry: one shot per load.
	--
	-- A replayed audio pair can leave the drive not playing.  When that
	-- happens no status ever arrives again, and a game waiting for the end of
	-- the track waits forever - either parked in the System Card's wait-for-$D8
	-- poll, or waiting the IRQ2/DTD track-end event a stopped drive can never
	-- raise.  Re-issuing the pair restarts the drive, and the end status it
	-- eventually produces satisfies both of those waits.
	--
	-- The arming condition is total CD silence, because that is what separates
	-- a stall from slow progress: a healthy load exchanges statuses and
	-- commands within milliseconds, so seconds of nothing at all, with an
	-- interrupt-mode audio event still outstanding, cannot be anything else.
	signal RP_RETRY_AGE  : unsigned(27 downto 0) := (others => '0');	-- bit27 = ~3.1 s
	signal RP_RETRIED    : std_logic := '0';
	signal RP_AUD_DONE   : std_logic := '0';	-- this load's replay included the audio pair
	-- The retry pass swallows nothing.
	--
	-- Elsewhere a replayed command's own status is swallowed, so the game is
	-- not handed a reply to a command it never sent.  That reasoning does not
	-- survive here: statuses carry no identity and are all GOOD, so a swallow
	-- armed during the rescue can just as easily eat the one status the game
	-- has been starving for.  After seconds of proven silence every status is
	-- a lifeline - the System Card poll accepts any $D8, and the data-transfer
	-- interrupt fires on any status request - and none of them can collide
	-- with anything, because nothing else is in flight.
	signal RP_RETRY_RUN  : std_logic := '0';
	signal RP_SG_D       : std_logic := '0';
	-- Loop repair: a looping track resumed part-way inherits the wrong start.
	--
	-- The drive loops back to the last position it was told to seek to, and a
	-- replay necessarily seeks to where playback is being resumed, not to where
	-- the track began.  Left alone the music would repeat from the save point
	-- to the end of the track forever, which is what a save taken close to the
	-- end sounds like: a stuck record.
	--
	-- So the wrap is watched rather than the seek corrected.  When the consumed
	-- samples reach the end of the window, the pair is re-issued once with the
	-- original start position; the loop that arms from it has the right start,
	-- and since resume and start now coincide, it does not arm this again.
	-- Any command or selection from the game disarms it - the game's own audio
	-- decisions always supersede a replayed one.
	signal LF_ARM        : std_logic := '0';
	signal LF_TARGET     : unsigned(31 downto 0) := (others => '0');
	signal LF_ACC        : unsigned(31 downto 0) := (others => '0');
	signal LF_CNT        : unsigned(20 downto 0) := (others => '0');
	signal RP_STWD       : unsigned(25 downto 0) := (others => '0');	-- status-serialization watchdog (~1.5s)
	signal AUD_OPEN      : std_logic := '0';	-- audio-family command sent, its status not yet arrived
	signal AUD_AGE       : unsigned(25 downto 0) := (others => '0');	-- AUD_OPEN relief valve (~1.5s)
	signal AUD_LAST      : std_logic_vector(1 downto 0) := "00";	-- which audio command is owed: 00=D8 01=D9 10=DA
	signal TAIL_FLUSH    : std_logic := '0';	-- 1-CLK FIFO aclr on the tail drain's last pop
	signal RP_FREEZE     : std_logic;
	signal DATAIN_HOLD   : std_logic;	-- mask REQ towards the CPU while the FSM is held
	-- HPS command pickup is ~1ms but statuses ride a 13-16ms poll tick: two
	-- replay commands (or a flush and the first new data) must sit one full
	-- tick apart.  2^20 CLK at 42.95 MHz = ~24 ms.
	constant RP_T_SETTLE : unsigned(20 downto 0) := to_unsigned(65535, 21);	-- ~1.5ms
	constant RP_T_GAP    : unsigned(20 downto 0) := to_unsigned(1048575, 21);	-- ~24ms

	--Savestates
	type slv64_array6 is array (0 to 5) of std_logic_vector(63 downto 0);
	signal SS_V      : slv64_array6;
	signal SS_V_BACK : slv64_array6 := (others => (others => '0'));
	signal SS_V_Dout : slv64_array6;
	signal SP_ENC    : std_logic_vector(3 downto 0);

	function sp_encode(x : SCSIPhase_t) return std_logic_vector is
	begin
		return std_logic_vector(to_unsigned(SCSIPhase_t'pos(x), 4));
	end function;
	function sp_decode(v : std_logic_vector(3 downto 0)) return SCSIPhase_t is
	begin
		return SCSIPhase_t'val(to_integer(unsigned(v)));
	end function;

begin

	process( RESET_N, CLK )
	begin
		if RESET_N = '0' then
			FIFO_D <= (others => '0');
			FIFO_WR_REQ <= '0';
			--CD_WR_OLD <= '0';
		elsif rising_edge(CLK) then
			FIFO_WR_REQ <= '0';
			if SaveStateBus_load = '1' then
				FIFO_D    <= SS_V(0)(55 downto 48);
				CD_WR_OLD <= SS_V(0)(20);
			else
				CD_WR_OLD <= CD_WR;
				if CD_WR = '1' and CD_WR_OLD = '0' then
					FIFO_D <= CD_DATA;
					if FULL = '0' then
						FIFO_WR_REQ <= '1';
					end if;
				end if;
			end if;
		end if;
	end process;

	
	-- Flushed on savestate load: it may hold bytes pushed by the PRE-load
	-- live stream during the freeze — stale sector data for the restored
	-- state (the CD-quiet gate guarantees the SAVED state expects it empty).
	FIFO : entity work.SCSI_FIFO 
	port map(
		-- SEL_FLUSH: replay-scoped only (READ_SKIP/=0) - a game SEL landing
		-- mid-drain must not leave the abandoned re-read's bytes queued in
		-- front of the new command's data.  Live behavior untouched.
		aclr     => (not RESET_N) or SaveStateBus_load or RP_FIFO_HOLD or SEL_FLUSH or TAIL_FLUSH,

		wrclk		=> CLK,
		data		=> FIFO_D,
		wrreq		=> FIFO_WR_REQ,
		wrfull	=> FULL,
		
		rdclk		=> CLK,
		rdreq		=> FIFO_RD_REQ,
		rdempty	=> EMPTY,
		q			=> FIFO_Q
	);

	process( CLK, RESET_N ) begin
		if RESET_N = '0' then
			AUDIO_ACTIVE <= '0';
			AUDIO_MODE <= (others => '0');
			AUDIO_B1 <= (others => '0');
			D9_SEEN <= '0';
			D9_B1 <= (others => '0');
			D9_MODE <= (others => '0');
			D9_B2 <= (others => '0');
			D9_B3 <= (others => '0');
			D9_B4 <= (others => '0');
			D9_B5 <= (others => '0');
			READ_ACTIVE <= '0';
			QRY_ACTIVE <= '0';
			READ_CONSUMED <= (others => '0');
			READ_SKIP <= (others => '0');
			SWALLOW <= (others => '0');
			RSKIP_STARVE <= (others => '0');
			RSKIP_GAP <= '0';
			SWAL_AGE <= (others => '0');
			SP_STARVE <= (others => '0');
			SP_STARVE_HIT <= (others => '0');
			SP_STARVE_ABT <= '0';
			AUD_OPEN <= '0';
			AUDIO_B2 <= (others => '0');
			AUDIO_B3 <= (others => '0');
			AUDIO_B4 <= (others => '0');
			AUDIO_B5 <= (others => '0');
			DBO_r <= (others => '0');
			BSY_Nr <= '1';
			MSG_Nr <= '1';
			CD_Nr <= '1';
			IO_Nr <= '1';
			REQ_Nr <= '1';
			COMM <= (others => (others => '0'));
			COMM_POS <= (others => '0');
			DATA_BUF <= (others => (others => '0'));
			DATA_POS <= (others => '0');
			SP <= SP_FREE;
			STOP_CD_SND_r <= '0';
			
			COMM_OUT <= '0';
			DATA_OUT <= '0';
			CD_DATA_END <= '0';
			STAT_PEND <= '0';
			DOUT_PEND <= '0';
			FIFO_RD_REQ <= '0';
			
			STAT_COUNT  <= (others => '0');
			DELAY_COUNT <= (others => '0');
			
			DATAIN_CNT  <= (others => '0');

		elsif rising_edge( CLK ) then
			if SaveStateBus_load = '1' then
				-- Savestate restore (registers driven by this process)
				SP       <= sp_decode(SS_V(0)(3 downto 0));
				BSY_Nr   <= SS_V(0)(4);
				MSG_Nr   <= SS_V(0)(5);
				CD_Nr    <= SS_V(0)(6);
				IO_Nr    <= SS_V(0)(7);
				REQ_Nr   <= SS_V(0)(8);
				COMM_POS <= unsigned(SS_V(0)(12 downto 9));
				DATA_POS <= unsigned(SS_V(0)(16 downto 13));
				STOP_CD_SND_r <= SS_V(0)(17);
				STAT_PEND   <= SS_V(0)(18);
				DOUT_PEND   <= SS_V(0)(19);
				DBO_r       <= SS_V(0)(31 downto 24);
				DATAIN_CNT  <= unsigned(SS_V(0)(47 downto 32));
				STAT_COUNT  <= unsigned(SS_V(1)(15 downto 0));
				DELAY_COUNT <= resize(unsigned(SS_V(1)(31 downto 16)), 17);
				AUDIO_MODE   <= SS_V(0)(57 downto 56);
				AUDIO_ACTIVE <= SS_V(0)(58);
				AUDIO_B1     <= SS_V(0)(22 downto 21);
				READ_ACTIVE  <= SS_V(0)(23);
				QRY_ACTIVE   <= SS_V(0)(59);
				-- Release the bus when the restored phase is idle.
				--
				-- BSY is asserted by the CPU's own SEL and is only ever deasserted again
				-- in SP_MSGIN_HOLD, i.e. after a status has been delivered and
				-- handshaken.  A SAPEP in LOOP or INTERRUPT mode is answered late or not
				-- at all (the System Card knows this and does not wait - see $0992 in
				-- docs/Super CD-ROM2 System V3.00 (J).ASM), so the machine legitimately
				-- sits with the phase back at SP_FREE and BSY still asserted, and a
				-- savestate lands there.  After a load nothing can ever release it: the
				-- status that would have belonged to that command was consumed by the
				-- pre-save timeline, and the phase-starvation valve deliberately does
				-- not count while SP is SP_FREE.  The System Card checks BSY before
				-- selecting ($0908: TST $80,$1800) and, finding it stuck, spins in its
				-- recovery path with the data-transfer interrupt switched off - so no
				-- further command is ever issued, the video stream never resumes and the
				-- picture freezes while CDDA, driven by the drive, keeps playing.
				-- Confirmed on hardware: of two savestates taken in the same scene, only
				-- the one whose blob had BSY asserted froze.
				--
				-- With the phase idle there is no handshake to preserve, so presenting a
				-- free bus is both safe and what lets the program carry on.  An in-flight
				-- READ6 is left alone: the replay rebuilds that one.
				if SS_V(0)(3 downto 0) = x"0" and SS_V(0)(23) = '0' then
					BSY_Nr <= '1';
					MSG_Nr <= '1';
					CD_Nr  <= '1';
					IO_Nr  <= '1';
					REQ_Nr <= '1';
				end if;
				D9_B2   <= SS_V(5)(7 downto 0);
				D9_B3   <= SS_V(5)(15 downto 8);
				D9_B4   <= SS_V(5)(23 downto 16);
				D9_B5   <= SS_V(5)(31 downto 24);
				D9_B1   <= SS_V(5)(39 downto 32);
				D9_MODE <= SS_V(5)(41 downto 40);
				D9_SEEN <= SS_V(5)(42);
				READ_CONSUMED <= unsigned(SS_V(5)(63 downto 43));
				-- never captured non-zero (the quiet gate forbids it): just clear
				READ_SKIP <= (others => '0');
				SWALLOW   <= (others => '0');
				RSKIP_STARVE <= (others => '0');
				RSKIP_GAP <= '0';
				SWAL_AGE  <= (others => '0');
				SP_STARVE <= (others => '0');
				AUDIO_B2     <= SS_V(1)(39 downto 32);
				AUDIO_B3     <= SS_V(1)(47 downto 40);
				AUDIO_B4     <= SS_V(1)(55 downto 48);
				AUDIO_B5     <= SS_V(1)(63 downto 56);
				for i in 0 to 7 loop
					COMM(i) <= SS_V(2)(i*8+7 downto i*8);
				end loop;
				for i in 0 to 3 loop
					COMM(8+i)   <= SS_V(3)(i*8+7 downto i*8);
					DATA_BUF(i) <= SS_V(3)(32+i*8+7 downto 32+i*8);
				end loop;
				for i in 0 to 5 loop
					DATA_BUF(4+i) <= SS_V(4)(i*8+7 downto i*8);
				end loop;
			else
			-- HPS-facing latches stay live even during the savestate freeze:
			-- with P5b a READ stream can complete mid-walk and its status must
			-- not be lost for the live (post-save) machine.  Statuses generated
			-- by replay commands are eaten here (SWALLOW).
			-- relief valve: a swallowed status that never arrives (replay
			-- command lost on the HPS mailbox) would block the quiet gate
			-- forever; statuses are late by at most the emulated seek (~1s),
			-- so after ~6s give up (a later stray status may leak - harmless
			-- next to a permanently wedged boundary)
			if SWALLOW /= 0 then
				SWAL_AGE <= SWAL_AGE + 1;
				if SWAL_AGE(27) = '1' then
					SWALLOW  <= (others => '0');
					SWAL_AGE <= (others => '0');
				end if;
			else
				SWAL_AGE <= (others => '0');
			end if;
			-- The open-audio flag must never stick.  A status lost outright by
			-- the drive's last-writer-wins mailbox would otherwise veto the
			-- quiet gate for good: no save and no load until reset, which is a
			-- far worse failure than the one the veto prevents.  The longest
			-- legitimate wait is a single emulated seek, under 700 ms, so the
			-- flag ages out at ~1.5 s.  A restore clears it too: it describes a
			-- timeline that no longer exists.
			if AUD_OPEN = '1' then
				AUD_AGE <= AUD_AGE + 1;
				if AUD_AGE(25) = '1' then
					AUD_OPEN <= '0';
					AUD_AGE <= (others => '0');
				end if;
			else
				AUD_AGE <= (others => '0');
			end if;
			if SaveStateBus_load = '1' then
				-- Whether a status is still owed is state, not a hint, so it is
				-- saved and restored: a game frozen while waiting for an audio
				-- status must come back still waiting for it.  The replay then
				-- lets the re-issued command's status through instead of
				-- swallowing it.  Swallowing assumes the pre-save game had
				-- already been served; when it had not, that assumption steals
				-- the reply it was still waiting for and parks it forever.
				AUD_OPEN <= SS_V(0)(60);
				AUD_LAST <= SS_V(0)(62 downto 61);
			end if;
			STAT_GET_D <= STAT_GET;
			if STAT_GET = '1' and STAT_GET_D = '0' then	-- rising edge: exactly one
				READ_ACTIVE <= '0';	-- a status closes the in-flight READ
				AUD_OPEN <= '0';	-- the awaited audio status arrived: window closed
				if SWALLOW /= 0 then
					SWALLOW <= SWALLOW - 1;
					SWAL_AGE <= (others => '0');
				else
					STAT_PEND <= '1';
				end if;
			end if;
			if RP_SW_INC = '1' then
				SWALLOW <= SWALLOW + 1;
			end if;
			
			if DOUT_REQ = '1' then
				DOUT_PEND <= '1';
			end if;
			
			if SLEEP = '0' then

			COMM_OUT <= '0';
			DATA_OUT <= '0';
			CD_DATA_END <= '0';
			FIFO_RD_REQ <= '0';
			AUDIO_CMD_SET <= '0';
			SEL_FLUSH <= '0';
			TAIL_FLUSH <= '0';
			
			if RST_N = '0' then
				BSY_Nr <= '1';
				MSG_Nr <= '1';
				CD_Nr <= '1';
				IO_Nr <= '1';
				REQ_Nr <= '1';
			else
				if RP_RSKIP_SET = '1' then
					READ_SKIP <= RP_RSKIP;
					RSKIP_GAP <= '0';
				elsif READ_SKIP /= 0 then
					-- post-replay drain: discard the already-consumed head of the
					-- re-issued stream; the CPU-facing FSM is held meanwhile so
					-- the CPU cannot race the drain for FIFO bytes.
					-- Relief valve: if the re-issued READ was lost (HPS mailbox
					-- collision) the stream never comes - after ~400ms without a
					-- byte give up instead of freezing the FSM (and the boundary)
					-- forever; the game will retry its own READ.
					--
					-- RSKIP_GAP leaves one idle cycle after every pop.  The FIFO's
					-- EMPTY is registered, so the cycle right after our own read
					-- request it still reports the PRE-pop state: draining on
					-- consecutive cycles therefore decrements READ_SKIP once more
					-- than it actually pops whenever the FIFO runs dry - and it
					-- runs dry constantly, because the drive feeds roughly a byte
					-- every four clocks while the drain would take one per clock.
					-- The skip then expires hundreds of bytes early and the CPU
					-- resumes on the wrong part of the stream, which a streamed
					-- cutscene shows as picture garbage that grows as it plays
					-- (scenario C in the testbench matrix).  With
					-- the gap, EMPTY is always settled when it is sampled, so one
					-- decrement is exactly one byte.  Cost: 2 clocks per drained
					-- byte, i.e. ~40 us for a full 2048-byte sector.
					if RSKIP_GAP = '1' then
						RSKIP_GAP <= '0';
					elsif EMPTY = '0' then
						FIFO_RD_REQ <= '1';
						READ_SKIP <= READ_SKIP - 1;
						RSKIP_GAP <= '1';
						RSKIP_STARVE <= (others => '0');
						-- Tail-window drain (RP_RD_PREP sect_v>=cnt: the CPU had
						-- consumed the WHOLE transfer, only the STATUS was owed):
						-- NOTHING in the re-read belongs to the CPU.  Without this
						-- flush the sector's last byte survived the drain and was
						-- served as a spurious 4097th DATAIN byte before the STATUS
						-- (scenario W repro; on hardware: the FMV-soak black screen,
						-- the stage-0 READ6 stream shifted and the BIOS retrying
						-- forever).  Flush on the LAST pop so the in-flight byte
						-- dies in the aclr too.
						if READ_SKIP = 1 and RP_TAILDRAIN = '1' then
							TAIL_FLUSH <= '1';	-- RP_TAILDRAIN is cleared by its owner (replay process)
						end if;
					else
						RSKIP_STARVE <= RSKIP_STARVE + 1;
						if RSKIP_STARVE(23) = '1' then
							READ_SKIP    <= (others => '0');
							RSKIP_STARVE <= (others => '0');
						end if;
					end if;
				elsif RP_FREEZE = '0' then
				-- phase-starvation valve: a bus phase (STATUS/MSGIN/DATAIN/
				-- COMMAND) parked with no CPU handshake for ~391ms (2^24 CLK)
				-- means the program is not listening - e.g. a stray status
				-- raised a phase nobody serves and the quiet gate would stay
				-- blocked forever.  ARMED ONLY while a savestate is pending
				-- (SS_PEND): unwedging SS_QUIET is the valve's sole purpose,
				-- and armed on the live machine it tore down legitimately
				-- parked phases during ordinary CD loading - dropping the
				-- staged DATAIN byte / a pending STATUS and shifting or
				-- starving the stream the game was still consuming (the
				-- Konami-logo transient BRK storms).  When it would have
				-- fired live, only count it (SP_STARVE_HIT, saved in
				-- SS_V(0) bits 63:61 - readable from any blob dump).
				if SP = SP_FREE or ACK_N = '0' or SEL_N = '0' then
					SP_STARVE <= (others => '0');
				else
					SP_STARVE <= SP_STARVE + 1;
					if SP_STARVE(24) = '1' then
						SP_STARVE <= (others => '0');
						if SS_PEND = '1' then
							SP_STARVE_ABT <= '1';
							SP <= SP_FREE;
							BSY_Nr <= '1';
							MSG_Nr <= '1';
							CD_Nr  <= '1';
							IO_Nr  <= '1';
							REQ_Nr <= '1';
							COMM_POS <= (others => '0');
							DATA_POS <= (others => '0');
						elsif SP_STARVE_HIT /= "111" then
							SP_STARVE_HIT <= SP_STARVE_HIT + 1;
						end if;
					end if;
				end if;
				case SP is
					when SP_FREE =>
						if SEL_N = '0' then
							-- a new command abandons the tracked READ stream
							READ_ACTIVE <= '0';
							QRY_ACTIVE <= '0';
							-- A selection by the game retires all replay
							-- bookkeeping.  The drive's mailbox is serial and the
							-- replay paces its own commands so that their statuses
							-- have landed before the machine resumes; by the time
							-- the game selects, therefore, a leftover swallow can
							-- only ever eat the game's own next status - and a
							-- game waiting on a completion it will never receive
							-- polls for it forever.
							SWALLOW  <= (others => '0');
							SWAL_AGE <= (others => '0');
							if READ_SKIP /= 0 then
								SEL_FLUSH <= '1';	-- drop the abandoned re-read's queued bytes
							end if;
							READ_SKIP    <= (others => '0');
							RSKIP_STARVE <= (others => '0');
							RSKIP_GAP    <= '0';
							BSY_Nr <= '0';
							MSG_Nr <= '1';
							CD_Nr <= '0';
							IO_Nr <= '1';
							SP <= SP_COMM_BEFOREREQ;
							DELAY_COUNT <= to_unsigned(1700, DELAY_COUNT'LENGTH);		-- Wait 40 microseconds after control signals are set up, before triggering REQ in COMMAND phase
							DATAIN_CNT <= (others => '0');
						elsif STAT_PEND = '1' then
							STAT_COUNT <= STAT_COUNT + 1;

							if (STAT_COUNT = 45000) then		-- CLK is 42.95 MHz; this gives ~1.05 millisec delay before transitioning to STATUS phase
																		-- this is empirical and may not be correct but it solves
																		-- the Sailor Moon hang issue
								STAT_COUNT <= (others => '0');
								STAT_PEND <= '0';
								DBO_r <= STATUS;
								BSY_Nr <= '0';
								MSG_Nr <= '1';
								CD_Nr <= '0';
								IO_Nr <= '0';
								REQ_Nr <= '0';
								SP <= SP_STAT_START;
							end if;
						-- The drain guards are not optional.  During a tail-window
						-- drain the phase sits free, and without them this arm
						-- claims the byte of the drain's last pop on the same
						-- cycle, one cycle before the flush can clear the FIFO:
						-- the game is then handed one byte more than the sector
						-- holds and retries the transfer forever.  The guards
						-- exclude the whole drain window and its exit cycle, where
						-- the registered EMPTY is still stale.  The
						-- mid-sector owed-byte path does not pass through here (SP
						-- is restored in DATAIN and served by its own arm), and on
						-- the live machine both signals are constant '0'.
						elsif EMPTY = '0' and READ_SKIP = 0 and TAIL_FLUSH = '0' then
							DBO_r <= FIFO_Q;
							BSY_Nr <= '0';
							MSG_Nr <= '1';
							CD_Nr <= '1';
							IO_Nr <= '0';
							REQ_Nr <= '0';
							FIFO_RD_REQ <= '1';
							SP <= SP_DATAIN_START;
						elsif DOUT_PEND = '1' then
							DOUT_PEND <= '0';
							BSY_Nr <= '0';
							MSG_Nr <= '1';
							CD_Nr <= '1';
							IO_Nr <= '1';
							REQ_Nr <= '0';
							SP <= SP_DATAOUT_START;
						end if;
						
					when SP_COMM_BEFOREREQ =>
						if (DELAY_COUNT = 0) then
							REQ_Nr <= '0';
							SP <= SP_COMM_START;
						else
							DELAY_COUNT <= DELAY_COUNT - 1;
						end if;

					when SP_COMM_START =>
						if REQ_Nr = '0' and ACK_N = '0' then
							REQ_Nr <= '1';
							COMM(to_integer(COMM_POS)) <= DBI;
							COMM_POS <= COMM_POS + 1;
							SP <= SP_COMM_END;
						end if;
					
					when SP_COMM_END =>
						if REQ_Nr = '1' and ACK_N = '1' then
							if COMM_POS = COMM_LEN(to_integer(unsigned(COMM(0)(7 downto 4)))) then
								COMM_POS <= (others => '0');
								COMM_OUT <= '1';
								CD_Nr <= '1';
								SP <= SP_FREE;
								if ((COMM(0) = x"08") or (COMM(0) = x"DA")) then	-- READ6 and PAUSE commands should mute sound, but still drain FIFO
									STOP_CD_SND_r <= '1';
									AUDIO_ACTIVE <= '0';	-- P5c: audio no longer resumable as-is
								end if;
								if (COMM(0) = x"08") then	-- P5b: track the data stream for replay
									READ_ACTIVE   <= '1';
									READ_CONSUMED <= (others => '0');
								end if;
								-- P5e: anything that is neither a READ6 nor one of the
								-- three audio commands is a QUERY the drive still owes an
								-- answer to ($DD READ SUBQ, $DE CD_DINFO...).  Track it so
								-- the replay can re-issue it: nothing else will, and the
								-- program polls for that answer forever.
								if COMM(0) /= x"08" and COMM(0) /= x"D8"
								   and COMM(0) /= x"D9" and COMM(0) /= x"DA" then
									QRY_ACTIVE <= '1';
								end if;
								if (COMM(0) = x"D8") then	-- P5c: SAPSP — latch the raw start point
									AUDIO_ACTIVE <= '1';
									AUDIO_MODE   <= COMM(9)(7 downto 6);
									AUDIO_B1     <= COMM(1)(1 downto 0);
									D9_SEEN      <= '0';	-- a new start point invalidates the old end
									AUDIO_B2     <= COMM(2);
									AUDIO_B3     <= COMM(3);
									AUDIO_B4     <= COMM(4);
									AUDIO_B5     <= COMM(5);
									AUDIO_CMD_SET <= '1';
								end if;
								if (COMM(0) = x"D9") then	-- P5c2: SAPEP — latch the raw end point
									D9_SEEN <= '1';
									D9_B1   <= COMM(1);
									D9_MODE <= COMM(9)(7 downto 6);
									D9_B2   <= COMM(2);
									D9_B3   <= COMM(3);
									D9_B4   <= COMM(4);
									D9_B5   <= COMM(5);
									if COMM(1) = x"00" then	-- SILENT end = playback stops
										AUDIO_ACTIVE <= '0';
									end if;
								end if;
								if ((COMM(0) = x"D8") or (COMM(0) = x"D9")) then	-- SAPSP and SAPEP commands should unmute sound (FIFO should be empty by now)
									STOP_CD_SND_r <= '0';
								end if;
								-- Audio-family commands answer with a bare STATUS -
								-- for a D8 it is PENDED HPS-side behind the emulated
								-- seek (hundreds of ms).  A save landing in that
								-- window captures a machine whose awaited status
								-- lives only in the HPS slot: unrestorable, and the
								-- replay's first command would overwrite it (W3).
								-- Track the window and veto the quiet gate for its
								-- duration; the freeze FSM simply retries.
								if COMM(0) = x"D8" or COMM(0) = x"D9" or COMM(0) = x"DA" then
									AUD_OPEN <= '1';
									if COMM(0) = x"D8" then
										AUD_LAST <= "00";
									elsif COMM(0) = x"D9" then
										AUD_LAST <= "01";
									else
										AUD_LAST <= "10";
									end if;
								end if;
							else
								SP <= SP_COMM_BEFOREREQ;
								DELAY_COUNT <= to_unsigned(5370, DELAY_COUNT'LENGTH);	-- Wait 125 microseconds after ACK, before next REQ in COMMAND phase
							end if;
						end if;

					when SP_STAT_START =>
						if REQ_Nr = '0' and ACK_N = '0' then
							REQ_Nr <= '1';
							SP <= SP_STAT_END;
						end if;
					
					when SP_STAT_END =>
						if REQ_Nr = '1' and ACK_N = '1' then
							SP <= SP_STAT_HOLD;
							DELAY_COUNT <= to_unsigned(49400, DELAY_COUNT'LENGTH);	-- wait 1.15 milliseconds after ACK in STATUS pahse before transitioning to next phase (MSGIN)
						end if;

					when SP_STAT_HOLD =>
						if (DELAY_COUNT = 0) then
							DBO_r <= MESSAGE;
							BSY_Nr <= '0';
							MSG_Nr <= '0';
							CD_Nr <= '0';
							IO_Nr <= '0';
							REQ_Nr <= '0';
							SP <= SP_MSGIN_START;
						else
							DELAY_COUNT <= DELAY_COUNT - 1;
						end if;
					
					when SP_MSGIN_START =>
						if REQ_Nr = '0' and ACK_N = '0' then
							REQ_Nr <= '1';
							SP <= SP_MSGIN_END;
						end if;
					
					when SP_MSGIN_END =>
						if REQ_Nr = '1' and ACK_N = '1' then
							SP <= SP_MSGIN_HOLD;
							DELAY_COUNT <= to_unsigned(6600, DELAY_COUNT'LENGTH);		-- wait 154 microseconds after ACK in STATUS phase before transitioning to next phase/disconnecting
						end if;

					when SP_MSGIN_HOLD =>
						if (DELAY_COUNT = 0) then
							-- the transaction is fully over: nothing left to re-issue
							QRY_ACTIVE <= '0';
							BSY_Nr <= '1';
							MSG_Nr <= '1';
							CD_Nr <= '1';
							IO_Nr <= '1';
							REQ_Nr <= '1';
							SP <= SP_FREE;
						else
							DELAY_COUNT <= DELAY_COUNT - 1;
						end if;

					when SP_DATAIN_START =>
						if REQ_Nr = '0' and ACK_N = '0' then
							REQ_Nr <= '1';
							SP <= SP_DATAIN_END;
							STOP_CD_SND_r <= '0';		-- unmute
						end if;
					
					when SP_DATAIN_END =>
						if REQ_Nr = '1' and ACK_N = '1' then
							if EMPTY = '0' then
								DBO_r <= FIFO_Q;
								REQ_Nr <= '0';
								FIFO_RD_REQ <= '1';
								SP <= SP_DATAIN_START;
							else
								CD_DATA_END <= '1';
								SP <= SP_FREE;
							end if;
							DATAIN_CNT <= DATAIN_CNT + 1;
							if READ_ACTIVE = '1' then
								READ_CONSUMED <= READ_CONSUMED + 1;
							end if;
						end if;
						
					when SP_DATAOUT_START =>
						if REQ_Nr = '0' and ACK_N = '0' then
							REQ_Nr <= '1';
							DATA_BUF(to_integer(DATA_POS)) <= DBI;
							DATA_POS <= DATA_POS + 1;
							SP <= SP_DATAOUT_END;
						end if;
					
					when SP_DATAOUT_END =>
						if REQ_Nr = '1' and ACK_N = '1' then
							if DATA_POS = 10 then
								DATA_POS <= (others => '0');
								DATA_OUT <= '1';
								SP <= SP_FREE;
							else
								REQ_Nr <= '0';
								SP <= SP_DATAOUT_START;
							end if;
						end if;
						
					when others => null;
				end case;
				end if;	-- drain / replay freeze
			end if;	-- RST_N
			end if;	-- SLEEP
			end if;	-- savestate load
		end if;
	end process;

	STOP_CD_SND <= STOP_CD_SND_r;
	DBO <= DBO_r;

	BSY_N <= BSY_Nr;
	MSG_N <= MSG_Nr;
	CD_N <= CD_Nr;
	IO_N <= IO_Nr;

	REQ_N <= REQ_Nr;

	-- P5b DATAIN hold (scenario C / the hardware FMV-streaming garble).
	-- After a load lands mid-READ6 the CPU-facing FSM is held for the whole
	-- replay sequence (RP_FREEZE: settle + gap + re-issue, tens of ms) and then
	-- for the byte drain (READ_SKIP /= 0).  The restored phase, however, still
	-- presents REQ asserted with the pre-save byte staged in DBO, so the
	-- restored program sees "a byte is ready", reads the data register, gets the
	-- SAME byte again, and — because the frozen FSM never answers the handshake
	-- — spins, counting one staged byte as hundreds.  Its stream position runs
	-- away from the drive's: exactly the misalignment scenario C reproduces, and
	-- the growing garble the hardware shows in a streamed cutscene.
	-- This flag tells cd.vhd to hide "a byte is ready" (the $5800 REQ bit, the
	-- $1803 DTR bit and its interrupt) and to hold the ADPCM DMA trigger off for
	-- the duration.  It is deliberately NOT applied to REQ_N itself: the raw REQ
	-- still drives the internal handshake, so an ACK caught in flight by the
	-- save completes normally when the FSM unfreezes instead of deadlocking.
	-- Restricted to the DATAIN phase: a held STATUS/MSGIN byte is idempotent,
	-- and leaving those REQ edges alone keeps cd.vhd's CD_DTD edge detector
	-- (which keys on MSG_N) from seeing a spurious transition.
	DATAIN_HOLD <= '1' when (RP_FREEZE = '1' or READ_SKIP /= 0)
	                    and (SP = SP_DATAIN_START or SP = SP_DATAIN_END)
	               else '0';
	DATAIN_HELD <= DATAIN_HOLD;

	COMMAND <= RP_COMM when RP_COMM_OVR = '1' else
	           COMM(11) & COMM(10) & COMM(9) & COMM(8) & COMM(7) & COMM(6) & COMM(5) & COMM(4) & COMM(3) & COMM(2) & COMM(1) & COMM(0);
	COMM_SEND <= COMM_OUT or RP_SEND;
	REPLAY_START <= RP_START_r;

	-- ============================================================================
	-- P5c2 replay FSM (audio + data), driven from the verified HPS behaviour
	-- (Main_MiSTer support/pcecd/pcecdd.cpp):
	--   * SAPSP (D8) also RESETS CDDAEnd to disc end and CDDAMode to its byte 1,
	--     so a lone D8 loses the loop/end set by the game's last SAPEP (D9).
	--     The replay therefore re-issues D8 (repositioned, LBA mode) AND, when
	--     one was latched, the game's last D9 verbatim — 24 ms apart, because
	--     commands share one HPS mailbox slot and statuses ride a 13-16 ms poll.
	--   * The consumed-sample position is wrapped into the loop window
	--     (start + (consumed/588 mod (end-start))) for LOOP-mode audio; for
	--     ended non-loop audio a PAUSE is issued instead of a stale replay.
	--   * Track-mode (byte9=8x) positions have no LBA visible to the FPGA: the
	--     D8 is re-issued verbatim (track restarts — documented limitation).
	--   * READ6 (0x08) in flight is re-issued from the first unconsumed sector
	--     (P5b): count' = count - consumed/2048, and consumed mod 2048 bytes of
	--     the re-streamed sector are drained before the CPU-facing FSM resumes.
	--   * Every replayed D8/D9/DA generates a status the restored CPU never
	--     asked for: SWALLOW eats them.  A replayed 08's final status is the
	--     one the restored CPU is legitimately waiting for — not swallowed.
	--   * The data FIFO is held cleared (RP_FIFO_HOLD) from the load until the
	--     replay command goes out, so sectors pushed by the PRE-load live
	--     stream cannot leak into the restored machine (the HPS keeps pushing
	--     for up to one 16 ms tick until the replayed command re-seeds it).
	--   * The CPU-facing FSM is frozen (RP_FREEZE) while the sequence runs so
	--     a restored mid-command handshake cannot interleave its own COMM_SEND
	--     with the replay's on the single HPS mailbox.  Worst case ~50 ms once
	--     per load.  All division is iterative subtraction.
	-- ============================================================================
	process( CLK, RESET_N )
		variable msf_bin : unsigned(23 downto 0);
		variable end_bin : unsigned(23 downto 0);
		variable nxt_v   : unsigned(20 downto 0);
		variable sect_v  : unsigned(9 downto 0);
		variable cnt9_v  : unsigned(8 downto 0);
		variable lba_v   : unsigned(20 downto 0);
	begin
		if RESET_N = '0' then
			RP <= RP_IDLE;
			RP_AFTER <= RP_IDLE;
			RP_COMM_OVR <= '0';
			RP_SEND <= '0';
			RP_START_r <= '0';
			RP_FIFO_HOLD <= '0';
			RP_SW_INC <= '0';
			RP_RSKIP_SET <= '0';
			RP_TRACKMODE <= '0';
			RP_TAILDRAIN <= '0';
			RP_STWD <= (others => '0');
			RP_RETRY_AGE <= (others => '0');
			RP_RETRIED <= '0';
			RP_AUD_DONE <= '0';
			RP_RETRY_RUN <= '0';
			LF_ARM <= '0';
			REPLAY_SKIP <= (others => '0');
		elsif rising_edge(CLK) then
			RP_SEND <= '0';
			RP_START_r <= '0';
			RP_SW_INC <= '0';
			RP_RSKIP_SET <= '0';

			case RP is
				when RP_IDLE =>
					RP_COMM_OVR <= '0';
					-- end-of-drain (or drain given up): the tail flag is stale.
					-- RP_RSKIP_SET qualifier: on the first IDLE cycle after
					-- RP_RD_ISSUE the main FSM has not loaded READ_SKIP yet.
					if READ_SKIP = 0 and RP_RSKIP_SET = '0' and RP_TAILDRAIN = '1' then
						RP_TAILDRAIN <= '0';
					end if;
					-- Rescue retry (see the signal declaration).  Armed only when
					-- this load replayed the audio pair; disarmed by ANY sign of
					-- life (a status from the HPS, a command or select from the
					-- game).  The 3.1 s of total CD silence it requires never
					-- happens on a healthy machine.  Bus must be idle: the retry
					-- freezes the CPU-facing FSM (RP_FREEZE) for its ~50 ms.
					RP_RETRY_RUN <= '0';	-- back in IDLE: the retry pass is over
					RP_SG_D <= STAT_GET;
					-- The game's own commands always supersede a replayed one
					if COMM_OUT = '1' or SEL_N = '0' then
						LF_ARM <= '0';
					elsif LF_ARM = '1' and CDDA_CONSUMED >= LF_TARGET
					   and SP = SP_FREE and STAT_PEND = '0' then
						LF_ARM <= '0';
						REPLAY_SKIP <= (others => '0');
						RP_SECT <= resize(RP_START_LBA, RP_SECT'length);
						RP_TRACKMODE <= '0';
						RP <= RP_ISSUE;	-- D8 @ original start, then the D9 pair
					end if;
					if (STAT_GET = '1' and RP_SG_D = '0') or COMM_OUT = '1'
					   or SEL_N = '0' then
						RP_RETRY_AGE <= (others => '0');
					elsif RP_AUD_DONE = '1' and RP_RETRIED = '0' then
						RP_RETRY_AGE <= RP_RETRY_AGE + 1;
						if RP_RETRY_AGE(27) = '1'
						   and D9_SEEN = '1' and D9_B1(1 downto 0) = "10"
						   and SP = SP_FREE and STAT_PEND = '0' then
							RP_RETRY_AGE <= (others => '0');
							RP_RETRIED <= '1';
							RP_RETRY_RUN <= '1';	-- this pass swallows nothing
							RP_CONS <= CDDA_CONSUMED;
							RP_TRACKMODE <= '0';
							RP <= RP_CVT_START;
						end if;
					end if;
					if SaveStateBus_load = '1' then
						RP <= RP_WAIT;
						RP_DELAY <= RP_T_SETTLE;
						RP_FIFO_HOLD <= '1';
						RP_TRACKMODE <= '0';
						RP_RETRIED <= '0';
						RP_AUD_DONE <= '0';
						RP_RETRY_RUN <= '0';
						LF_ARM <= '0';
						RP_RETRY_AGE <= (others => '0');
					end if;

				when RP_WAIT =>
					-- latch the consumed counter a few cycles into the settle:
					-- the restore has landed, but the machine then keeps playing
					-- the STALE pre-load FIFO tail and would inflate the counter
					-- by the settle time (~1.5ms of forward drift) before the
					-- conversion samples it
					if RP_DELAY = RP_T_SETTLE - 32 then
						RP_CONS <= CDDA_CONSUMED;
					end if;
					if RP_DELAY = 0 then
						if AUDIO_ACTIVE = '1' then
							RP <= RP_CVT_START;
						elsif READ_ACTIVE = '1' then
							RP <= RP_RD_PREP;
						else
							REPLAY_SKIP <= (others => '0');	-- pause path: flush only, no drain
							RP_DELAY <= RP_T_GAP;
							RP_AFTER <= RP_PAUSE;
							RP <= RP_HOLDGAP;
						end if;
					else
						RP_DELAY <= RP_DELAY - 1;
					end if;

				when RP_CVT_START =>	-- D8 start point -> binary LBA
					if AUDIO_MODE = "00" then		-- LBA in B3..B5
						RP_START_LBA <= unsigned(AUDIO_B3) & unsigned(AUDIO_B4) & unsigned(AUDIO_B5);
					elsif AUDIO_MODE = "01" then	-- MSF (BCD) in B2..B4
						msf_bin := resize(
							( (resize(unsigned(AUDIO_B2(7 downto 4)),16)*10 + resize(unsigned(AUDIO_B2(3 downto 0)),16)) * 60
							+ (resize(unsigned(AUDIO_B3(7 downto 4)),16)*10 + resize(unsigned(AUDIO_B3(3 downto 0)),16)) ) * 75
							+ (resize(unsigned(AUDIO_B4(7 downto 4)),16)*10 + resize(unsigned(AUDIO_B4(3 downto 0)),16))
							, 24) - 150;
						RP_START_LBA <= msf_bin;
					else							-- track mode: no LBA visible, restart the track
						RP_TRACKMODE <= '1';
						RP_START_LBA <= (others => '0');
					end if;
					RP_ACC  <= RP_CONS;
					RP_SECT <= (others => '0');
					RP <= RP_DIV588;

				when RP_DIV588 =>	-- consumed samples -> sectors + sub-sector skip
					if RP_ACC >= 588 then
						RP_ACC  <= RP_ACC - 588;
						RP_SECT <= RP_SECT + 1;
					else
						REPLAY_SKIP <= RP_ACC(9 downto 0);
						RP <= RP_CVT_END;
					end if;

				-- The drive plays the SAPEP window INCLUSIVE of the end sector
				-- (pcecdd: the position wraps only once it passes pend_lba), so a
				-- window of start..end is end-start+1 sectors long.  Using
				-- end-start folded the resumed position into a window one sector
				-- too short, and the error grows with the number of loops the
				-- track has already made: a save taken 16 sectors in came back at
				-- 16 mod 4 = 0 instead of 16 mod 5 = 1, i.e. a whole sector of
				-- audio early.  That is the "audio went into a loop" report.
				when RP_CVT_END =>	-- last D9 end point -> loop length (when computable)
					RP_LEN <= (others => '0');
					if RP_TRACKMODE = '1' then
						REPLAY_SKIP <= (others => '0');	-- verbatim restart: nothing to skip
					end if;
					if RP_TRACKMODE = '0' and D9_SEEN = '1' then
						if D9_MODE = "00" then
							end_bin := unsigned(D9_B3) & unsigned(D9_B4) & unsigned(D9_B5);
							if end_bin >= RP_START_LBA then
								RP_LEN <= end_bin - RP_START_LBA + 1;
							end if;
						elsif D9_MODE = "01" then
							end_bin := resize(
								( (resize(unsigned(D9_B2(7 downto 4)),16)*10 + resize(unsigned(D9_B2(3 downto 0)),16)) * 60
								+ (resize(unsigned(D9_B3(7 downto 4)),16)*10 + resize(unsigned(D9_B3(3 downto 0)),16)) ) * 75
								+ (resize(unsigned(D9_B4(7 downto 4)),16)*10 + resize(unsigned(D9_B4(3 downto 0)),16))
								, 24) - 150;
							if end_bin >= RP_START_LBA then
								RP_LEN <= end_bin - RP_START_LBA + 1;
							end if;
						end if;
					end if;
					RP <= RP_LOOPMOD;

				when RP_LOOPMOD =>	-- wrap the position into the loop window
					if RP_LEN /= 0 and RP_SECT >= RP_LEN then
						if D9_B1(1 downto 0) = "01" then	-- LOOP: fold back
							RP_SECT <= RP_SECT - RP_LEN;
						else
							-- non-loop audio had already finished before the save:
							-- nothing to resume, just silence the live stream
							REPLAY_SKIP <= (others => '0');
							RP_DELAY <= RP_T_GAP;
							RP_AFTER <= RP_PAUSE;
							RP <= RP_HOLDGAP;
						end if;
					else
						RP_SECT <= RP_START_LBA + RP_SECT;	-- absolute resume LBA
						RP_DELAY <= RP_T_GAP;
						RP_AFTER <= RP_ISSUE;
						RP <= RP_HOLDGAP;
					end if;

				when RP_HOLDGAP =>	-- absorb the tail of the pre-load live stream
					if RP_DELAY = 0 then
						-- SERIALIZE ON STATUSES (scenario W3, from the real
						-- pcecd source): the HPS status slot is SINGLE and its
						-- delivery is gated on the emulated seek latency - a
						-- distant jump freezes the previous command's GOOD for
						-- hundreds of ms.  Issuing the next command on a fixed
						-- 24ms gap OVERWRITES that frozen status: it is lost
						-- forever, the armed SWALLOW then eats the survivor
						-- the game is genuinely waiting for, and the BIOS
						-- parks in its status poll (the FMV->stage-0 black
						-- screen).  Wait until every pending swallow drained
						-- (= the previous command's status really arrived);
						-- ~1.5s watchdog in case the HPS dropped it outright.
						if (SWALLOW = 0 and AUD_OPEN = '0') or RP_STWD(25) = '1' then
							RP_STWD <= (others => '0');
							RP <= RP_AFTER;
						else
							RP_STWD <= RP_STWD + 1;
						end if;
					else
						RP_DELAY <= RP_DELAY - 1;
					end if;

				when RP_ISSUE =>	-- re-issue the D8 at the resume position
					RP_AUD_DONE <= '1';	-- the rescue may re-run this pair once
					RP_COMM <= (others => '0');
					RP_COMM(7 downto 0) <= x"D8";
					RP_COMM(9 downto 8) <= AUDIO_B1;	-- byte1: original CDDA play mode
					if RP_TRACKMODE = '1' then		-- verbatim (track restarts)
						RP_COMM(23 downto 16) <= AUDIO_B2;
						RP_COMM(31 downto 24) <= AUDIO_B3;
						RP_COMM(39 downto 32) <= AUDIO_B4;
						RP_COMM(47 downto 40) <= AUDIO_B5;
						RP_COMM(79 downto 72) <= AUDIO_MODE & "000000";
					else							-- always LBA mode: bytes 3..5
						RP_COMM(31 downto 24) <= std_logic_vector(RP_SECT(23 downto 16));
						RP_COMM(39 downto 32) <= std_logic_vector(RP_SECT(15 downto 8));
						RP_COMM(47 downto 40) <= std_logic_vector(RP_SECT(7 downto 0));
						RP_COMM(79 downto 72) <= (others => '0');	-- byte9 = 00 (LBA)
					end if;
					RP_COMM_OVR <= '1';
					RP_SEND <= '1';
					RP_START_r <= '1';	-- flush CDDA/SUBC FIFOs + arm the sample skip
					-- Swallow the re-issued command's status only if the pre-save
				-- game had already consumed its own - if it was still owed
				-- (AUD_OPEN with AUD_LAST=D8), the status must reach the game
				if not (AUD_OPEN = '1' and AUD_LAST = "00")
				   and RP_RETRY_RUN = '0' then
					RP_SW_INC <= '1';	-- the D8 pends one status
				end if;
					RP_FIFO_HOLD <= '0';
					if D9_SEEN = '1' then
						RP_DELAY <= RP_T_GAP;
						RP <= RP_GAP;
					elsif QRY_ACTIVE = '1' then
						RP <= RP_QRY_PREP;
					else
						RP <= RP_IDLE;
					end if;

				when RP_GAP =>		-- one HPS poll tick between the two commands
					if RP_DELAY = 0 then
						-- same status-serialization gate as RP_HOLDGAP: the
						-- D8's status is pended behind the seek latency
						if (SWALLOW = 0 and AUD_OPEN = '0') or RP_STWD(25) = '1' then
							RP_STWD <= (others => '0');
							RP <= RP_ISSUE_D9;
						else
							RP_STWD <= RP_STWD + 1;
						end if;
					else
						RP_DELAY <= RP_DELAY - 1;
					end if;

				when RP_ISSUE_D9 =>	-- re-issue the game's last SAPEP verbatim
					RP_COMM <= (others => '0');
					RP_COMM(7 downto 0) <= x"D9";
					RP_COMM(15 downto 8) <= D9_B1;
					RP_COMM(23 downto 16) <= D9_B2;
					RP_COMM(31 downto 24) <= D9_B3;
					RP_COMM(39 downto 32) <= D9_B4;
					RP_COMM(47 downto 40) <= D9_B5;
					RP_COMM(79 downto 72) <= D9_MODE & "000000";
					RP_COMM_OVR <= '1';
					RP_SEND <= '1';
					-- SILENT D8 (the common pairing): nothing has streamed during the
					-- gap, so flush again and re-arm the sample skip for the clean
					-- stream that starts only now.  A PLAY-mode D8 already started the
					-- stream and its skip is (partly) consumed: re-arming would skip
					-- twice, so leave it alone (a stale sector from before the HPS
					-- processed the D8 remains possible there — minor, documented).
					if AUDIO_B1 = "00" then
						RP_START_r <= '1';
					end if;
					if D9_B1(1 downto 0) /= "10"
					   and not (AUD_OPEN = '1' and AUD_LAST = "01")
					   and RP_RETRY_RUN = '0' then
						RP_SW_INC <= '1';	-- non-INTERRUPT SAPEP answers immediately
					end if;
					-- Arm the loop repair: a looping track resumed past its start
					-- point would inherit a wrong wrap target (see LF_ARM)
					if D9_B1(1 downto 0) = "01" and RP_LEN /= 0
					   and RP_SECT > RP_START_LBA then
						LF_CNT <= resize(RP_LEN - (RP_SECT - RP_START_LBA), 21);
						LF_ACC <= (others => '0');
						RP <= RP_LF_CALC;
					elsif QRY_ACTIVE = '1' then
						RP <= RP_QRY_PREP;
					else
						RP <= RP_IDLE;
					end if;

				when RP_LF_CALC =>	-- sectors-to-end -> samples-to-wrap (x588)
					if LF_CNT /= 0 then
						LF_ACC <= LF_ACC + 588;
						LF_CNT <= LF_CNT - 1;
					else
						-- base = consumed NOW (the D9 just went out): using the
						-- The base is the consumed count at this instant, not the one
						-- from the load: using the older value fires the repair early
						-- by the whole drift the replay accumulated (~20 ms).
						LF_TARGET <= resize(CDDA_CONSUMED, 32) + LF_ACC;
						LF_ARM <= '1';
						if QRY_ACTIVE = '1' then
							RP <= RP_QRY_PREP;
						else
							RP <= RP_IDLE;
						end if;
					end if;

				when RP_RD_PREP =>	-- P5b: rebuild the in-flight READ6
					-- Bytes already handed over, plus the one sitting in DBO when
					-- the save caught a DATAIN handshake mid-flight.
					--
					-- nxt_v is "one past the last byte the consumer already has",
					-- because the drain discards nxt_v mod 2048 MINUS ONE bytes:
					-- when READ_SKIP reaches 0 the FSM resumes before the last pop
					-- has settled through the FIFO's registered output, so the last
					-- nominally-skipped byte is the one that lands in DBO.  For the
					-- CPU that off-by-one is load-bearing and must NOT be "fixed"
					-- on its own: its staged byte is still owed to it (the CPU takes
					-- a DATAIN byte by reading $1808), so counting it here and
					-- getting it back from the drain is a wash.  Scenarios A, C and
					-- E depend on exactly that.
					--
					-- The ADPCM DMA has the OPPOSITE consumption boundary: it takes
					-- the byte off SCSI_DBO the moment REQ rises and has already
					-- written it into ADPCM RAM, while READ_CONSUMED still does not
					-- count it.  Handing it back duplicates it and slides the whole
					-- rest of the transfer one byte late (scenario H).
					-- So when the blob says the DMA had already taken the staged
					-- byte, step one further.
					if SP = SP_DATAIN_START or SP = SP_DATAIN_END then
						if DMA_STAGED = '1' then
							nxt_v := READ_CONSUMED + 2;
						else
							nxt_v := READ_CONSUMED + 1;
						end if;
					else
						nxt_v := READ_CONSUMED;
					end if;
					sect_v := nxt_v(20 downto 11);
					if COMM(4) = x"00" then
						cnt9_v := to_unsigned(256, 9);
					else
						cnt9_v := resize(unsigned(COMM(4)), 9);
					end if;
					lba_v := unsigned(COMM(1)(4 downto 0)) & unsigned(COMM(2)) & unsigned(COMM(3));
					RP_COMM <= (others => '0');
					RP_COMM(7 downto 0) <= x"08";
					if sect_v >= resize(cnt9_v, 10) then
						-- everything consumed, only the final STATUS is missing: the
						-- HPS sends it strictly after re-streaming, so re-read the
						-- last sector and drain it whole
						lba_v := lba_v + resize(cnt9_v, 21) - 1;
						RP_COMM(39 downto 32) <= x"01";
						RP_RSKIP <= to_unsigned(2048, 12);
						-- tail case: the CPU is owed NOTHING from the re-read -
						-- arm the end-of-drain FIFO flush (scenario W orphan)
						RP_TAILDRAIN <= '1';
					else
						lba_v := lba_v + resize(sect_v, 21);
						RP_COMM(39 downto 32) <= std_logic_vector(resize(cnt9_v - resize(sect_v, 9), 8));
						RP_RSKIP <= resize(nxt_v(10 downto 0), 12);
						RP_TAILDRAIN <= '0';
					end if;
					RP_COMM(15 downto 8)  <= COMM(1)(7 downto 5) & std_logic_vector(lba_v(20 downto 16));
					RP_COMM(23 downto 16) <= std_logic_vector(lba_v(15 downto 8));
					RP_COMM(31 downto 24) <= std_logic_vector(lba_v(7 downto 0));
					RP_COMM(47 downto 40) <= COMM(5);
					RP_DELAY <= RP_T_GAP;
					RP_AFTER <= RP_RD_ISSUE;
					RP <= RP_HOLDGAP;

				when RP_RD_ISSUE =>
					RP_COMM_OVR <= '1';
					RP_SEND <= '1';
					RP_RSKIP_SET <= '1';	-- arm the byte drain in the main FSM
					RP_FIFO_HOLD <= '0';	-- from here on the FIFO carries the new stream
					-- no RP_SW_INC: the replayed READ's final status is the one the
					-- restored CPU is genuinely waiting for
					RP <= RP_IDLE;

				-- P5e: re-issue the query the drive still owed an answer to.
				--
				-- It has to come AFTER the audio/pause command, for two reasons:
				-- the data FIFO is held cleared until then (RP_FIFO_HOLD feeds the
				-- FIFO's aclr), so an answer arriving earlier is discarded - that
				-- is exactly how the original one was lost - and the HPS mailbox
				-- takes one command at a time.
				--
				-- The skip mirrors RP_RD_PREP: DATAIN_CNT counts the answer bytes
				-- the program already took (it is reset per command, at the SEL in
				-- SP_FREE), and the drain discards one less than it is given, so
				-- the staged byte the CPU has not read yet comes back to it.
				when RP_QRY_PREP =>
					RP_COMM <= COMM(11) & COMM(10) & COMM(9) & COMM(8)
					         & COMM(7) & COMM(6) & COMM(5) & COMM(4)
					         & COMM(3) & COMM(2) & COMM(1) & COMM(0);
					if SP = SP_DATAIN_START or SP = SP_DATAIN_END then
						RP_RSKIP <= resize(DATAIN_CNT(10 downto 0), 12) + 1;
					else
						RP_RSKIP <= resize(DATAIN_CNT(10 downto 0), 12);
					end if;
					RP_DELAY <= RP_T_GAP;
					RP_AFTER <= RP_QRY_ISSUE;
					RP <= RP_HOLDGAP;

				when RP_QRY_ISSUE =>
					RP_COMM_OVR <= '1';
					RP_SEND <= '1';
					RP_RSKIP_SET <= '1';	-- arm the byte drain in the main FSM
					RP_FIFO_HOLD <= '0';	-- the answer must land in a LIVE FIFO
					-- no RP_SW_INC: the re-issued query's status is the one the
					-- restored program is genuinely waiting for
					RP <= RP_IDLE;

				when RP_PAUSE =>	-- nothing to resume: silence whatever streams
					RP_COMM <= (others => '0');
					RP_COMM(7 downto 0) <= x"DA";
					RP_COMM_OVR <= '1';
					RP_SEND <= '1';
					-- flush + realign the CDDA/SUBC assemblers here too: a stale
					-- mid-frame phase would poison the game's own next SAPSP
					RP_START_r <= '1';
					if not (AUD_OPEN = '1' and AUD_LAST = "10")
				   and RP_RETRY_RUN = '0' then
					RP_SW_INC <= '1';	-- PAUSE answers with an immediate status
				end if;
					RP_FIFO_HOLD <= '0';
					if QRY_ACTIVE = '1' then
						RP <= RP_QRY_PREP;
					else
						RP <= RP_IDLE;
					end if;
			end case;
		end if;
	end process;
	
	-- The CPU-facing FSM must stay held not only while the replay sequence runs,
	-- but until the statuses it generated have been SWALLOWed.  Statuses carry no
	-- identity: if the program issues a command of its own in that window, its
	-- status arrives first and is eaten in place of the replay's.  Seen on
	-- hardware: after a load the game recovered, issued a $DD (read subcode Q),
	-- received its 10 data bytes and then hung forever in the System Card's
	-- bus-phase poll at $09FB waiting for a MESSAGE IN that had been swallowed
	-- (see docs/Super CD-ROM2 System V3.00 (J).ASM).  Holding the FSM until
	-- SWALLOW drains keeps the program off the bus while those statuses are
	-- outstanding; SWAL_AGE still bounds the wait if one never arrives.
	RP_FREEZE <= '0' when RP = RP_IDLE and SWALLOW = 0 else '1';

	DOUT <= DATA_BUF(9) & DATA_BUF(8) & DATA_BUF(7) & DATA_BUF(6) & DATA_BUF(5) & DATA_BUF(4) & DATA_BUF(3) & DATA_BUF(2) & DATA_BUF(1) & DATA_BUF(0);
	DOUT_SEND <= DATA_OUT;
	
	DBG_DATAIN_CNT <= DATAIN_CNT;

	--------------------------------------------------------------------------------
	-- SAVESTATES (plan P5a; slot map: pce_savestates_pkg.vhd 55-59)
	--------------------------------------------------------------------------------
	-- NOT saved: SCSI_FIFO content (the CD-quiet boundary guarantees EMPTY),
	-- FIFO_RD/WR_REQ + COMM_OUT/DATA_OUT/CD_DATA_END strobes (quiet at boundary).

	SP_ENC <= sp_encode(SP);

	SS_V_BACK(0)(3 downto 0)   <= SP_ENC;
	SS_V_BACK(0)(4)            <= BSY_Nr;
	SS_V_BACK(0)(5)            <= MSG_Nr;
	SS_V_BACK(0)(6)            <= CD_Nr;
	SS_V_BACK(0)(7)            <= IO_Nr;
	SS_V_BACK(0)(8)            <= REQ_Nr;
	SS_V_BACK(0)(12 downto 9)  <= std_logic_vector(COMM_POS);
	SS_V_BACK(0)(16 downto 13) <= std_logic_vector(DATA_POS);
	SS_V_BACK(0)(17)           <= STOP_CD_SND_r;
	SS_V_BACK(0)(18)           <= STAT_PEND;
	SS_V_BACK(0)(19)           <= DOUT_PEND;
	SS_V_BACK(0)(20)           <= CD_WR_OLD;
	SS_V_BACK(0)(31 downto 24) <= DBO_r;
	SS_V_BACK(0)(47 downto 32) <= std_logic_vector(DATAIN_CNT);
	SS_V_BACK(0)(55 downto 48) <= FIFO_D;

	SS_V_BACK(0)(57 downto 56) <= AUDIO_MODE;
	SS_V_BACK(0)(58)           <= AUDIO_ACTIVE;
	SS_V_BACK(0)(22 downto 21) <= AUDIO_B1;
	SS_V_BACK(0)(23)           <= READ_ACTIVE;
	SS_V_BACK(0)(59)           <= QRY_ACTIVE;	-- P5e (bit 59 was spare: no STATESIZE change)
	-- 62:60 = real state, restored on load: AUD_LAST(62:61),
	-- AUD_OPEN(60) - the owed-audio-status marker the replay honors.
	-- 63 = forensic only: any live phase-starvation overflow or armed abort.
	SS_V_BACK(0)(63)           <= SP_STARVE_ABT or SP_STARVE_HIT(0)
	                              or SP_STARVE_HIT(1) or SP_STARVE_HIT(2);
	SS_V_BACK(0)(62 downto 61) <= AUD_LAST;
	SS_V_BACK(0)(60)           <= AUD_OPEN;

	-- P5c2/P5b (slot SCSI_3 = 64): last SAPEP raw + in-flight READ position.
	-- READ_SKIP/SWALLOW are never non-zero at a quiet boundary: not saved.
	SS_V_BACK(5)(7 downto 0)   <= D9_B2;
	SS_V_BACK(5)(15 downto 8)  <= D9_B3;
	SS_V_BACK(5)(23 downto 16) <= D9_B4;
	SS_V_BACK(5)(31 downto 24) <= D9_B5;
	SS_V_BACK(5)(39 downto 32) <= D9_B1;
	SS_V_BACK(5)(41 downto 40) <= D9_MODE;
	SS_V_BACK(5)(42)           <= D9_SEEN;
	SS_V_BACK(5)(63 downto 43) <= std_logic_vector(READ_CONSUMED);

	SS_V_BACK(1)(15 downto 0)  <= std_logic_vector(STAT_COUNT);
	-- DELAY_COUNT is declared 17b but its max programmed value is 49400:
	-- 16 bits suffice on the wire (restored with resize).
	SS_V_BACK(1)(31 downto 16) <= std_logic_vector(DELAY_COUNT(15 downto 0));
	SS_V_BACK(1)(39 downto 32) <= AUDIO_B2;
	SS_V_BACK(1)(47 downto 40) <= AUDIO_B3;
	SS_V_BACK(1)(55 downto 48) <= AUDIO_B4;
	SS_V_BACK(1)(63 downto 56) <= AUDIO_B5;

	GEN_SS_COMM_LO : for i in 0 to 7 generate
		SS_V_BACK(2)(i*8+7 downto i*8) <= COMM(i);
	end generate;
	GEN_SS_COMM_HI : for i in 0 to 3 generate
		SS_V_BACK(3)(i*8+7 downto i*8)       <= COMM(8+i);
		SS_V_BACK(3)(32+i*8+7 downto 32+i*8) <= DATA_BUF(i);
	end generate;
	GEN_SS_DATA : for i in 0 to 5 generate
		SS_V_BACK(4)(i*8+7 downto i*8) <= DATA_BUF(4+i);
	end generate;

	iSS_SCSI3 : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_SCSI_3, def => SSREG_DEFAULT_CD )
	port map (
		clk      => CLK,
		BUS_Din  => SaveStateBus_Din,
		BUS_Adr  => SaveStateBus_Adr,
		BUS_wren => SaveStateBus_wren,
		BUS_rst  => SaveStateBus_rst,
		BUS_Dout => SS_V_Dout(5),
		Din      => SS_V_BACK(5),
		Dout     => SS_V(5)
	);

	GEN_SS_EREGS : for k in 0 to 4 generate
		iSS_SCSI : entity work.eReg_SavestateV
		generic map ( Adr => SSREG_INDEX_SCSI_1 + k, def => SSREG_DEFAULT_CD )
		port map (
			clk      => CLK,
			BUS_Din  => SaveStateBus_Din,
			BUS_Adr  => SaveStateBus_Adr,
			BUS_wren => SaveStateBus_wren,
			BUS_rst  => SaveStateBus_rst,
			BUS_Dout => SS_V_Dout(k),
			Din      => SS_V_BACK(k),
			Dout     => SS_V(k)
		);
	end generate;

	-- Forensic eReg 65 (slots 65-127 are free internals: saved by the walk,
	-- never restored, zero STATESIZE impact).  One blob dump shows the tail
	-- of the last SCSI conversation:
	--   31:0  = last 8 SP phases, 4-bit sp_encode each, newest in 3:0
	--   39:32 = STAT_GET strobes seen (saturating)
	--   47:40 = game SELs seen
	--   55:48 = SP_MSGIN_START entries
	--   63:56 = STAT_PEND rises
	process(CLK) begin
		if rising_edge(CLK) then
			if RESET_N = '0' then
				SP_HIST <= (others => '0');
				CNT_STATGET <= (others => '0'); CNT_SEL <= (others => '0');
				CNT_MSGIN <= (others => '0'); CNT_STATPEND <= (others => '0');
			else
				SP_ENC_D2 <= SP_ENC;
				if SP_ENC /= SP_ENC_D2 then
					SP_HIST <= SP_HIST(27 downto 0) & SP_ENC;
					if SP = SP_MSGIN_START and CNT_MSGIN /= x"FF" then
						CNT_MSGIN <= CNT_MSGIN + 1;
					end if;
				end if;
				DBG_STATGET_D <= STAT_GET;
				if STAT_GET = '1' and DBG_STATGET_D = '0' and CNT_STATGET /= x"FF" then
					CNT_STATGET <= CNT_STATGET + 1;
				end if;
				DBG_SEL_D <= SEL_N;
				if SEL_N = '0' and DBG_SEL_D = '1' and CNT_SEL /= x"FF" then
					CNT_SEL <= CNT_SEL + 1;
				end if;
				DBG_STATPEND_D <= STAT_PEND;
				if STAT_PEND = '1' and DBG_STATPEND_D = '0' and CNT_STATPEND /= x"FF" then
					CNT_STATPEND <= CNT_STATPEND + 1;
				end if;
			end if;
		end if;
	end process;

	iSS_SCSI_DBG : entity work.eReg_SavestateV
	generic map ( Adr => 65, def => SSREG_DEFAULT_CD )
	port map (
		clk      => CLK,
		BUS_Din  => SaveStateBus_Din,
		BUS_Adr  => SaveStateBus_Adr,
		BUS_wren => SaveStateBus_wren,
		BUS_rst  => SaveStateBus_rst,
		BUS_Dout => SS_V_Dout_DBG,
		Din      => std_logic_vector(CNT_STATPEND) & std_logic_vector(CNT_MSGIN)
		          & std_logic_vector(CNT_SEL) & std_logic_vector(CNT_STATGET) & SP_HIST,
		Dout     => open
	);

	-- Forensic eReg 66: handshake ledger (save-only, like eReg 65).
	--   15:0  REQ assert edges (REQ_Nr falling)     47:40 drive bytes / 256
	--   31:16 ACK assert edges (ACK_N falling)      55:48 COMM_SEND pulses
	--   35:32 SP at the last ACK                    59:56 SP at the last REQ
	process(CLK) begin
		if rising_edge(CLK) then
			if RESET_N = '0' then
				CNT_REQ <= (others => '0'); CNT_ACK <= (others => '0');
				CNT_FIFOWR <= (others => '0'); CNT_COMMSEND <= (others => '0');
			else
				DBG_REQ_D <= REQ_Nr;
				if REQ_Nr = '0' and DBG_REQ_D = '1' then
					CNT_REQ <= CNT_REQ + 1; LAST_REQ_SP <= SP_ENC;
				end if;
				DBG_ACK_D <= ACK_N;
				if ACK_N = '0' and DBG_ACK_D = '1' then
					CNT_ACK <= CNT_ACK + 1; LAST_ACK_SP <= SP_ENC;
				end if;
				-- Quartus 17 cannot read out ports: use internal drivers only.
				DBG_DE_D <= FIFO_WR_REQ;
				if FIFO_WR_REQ = '1' and DBG_DE_D = '0' then
					CNT_FIFOWR <= CNT_FIFOWR + 1;	-- bytes pushed by the drive (wraps)
				end if;
				DBG_CS_D <= COMM_OUT or RP_SEND;
				if (COMM_OUT or RP_SEND) = '1' and DBG_CS_D = '0' and CNT_COMMSEND /= x"FF" then
					CNT_COMMSEND <= CNT_COMMSEND + 1;
				end if;
			end if;
		end if;
	end process;

	iSS_SCSI_DBG2 : entity work.eReg_SavestateV
	generic map ( Adr => 66, def => SSREG_DEFAULT_CD )
	port map (
		clk      => CLK,
		BUS_Din  => SaveStateBus_Din,
		BUS_Adr  => SaveStateBus_Adr,
		BUS_wren => SaveStateBus_wren,
		BUS_rst  => SaveStateBus_rst,
		BUS_Dout => SS_V_Dout_DBG2,
		Din      => x"0" & LAST_REQ_SP & std_logic_vector(CNT_COMMSEND)
		          & std_logic_vector(CNT_FIFOWR(15 downto 8)) & x"0" & LAST_ACK_SP
		          & std_logic_vector(CNT_ACK) & std_logic_vector(CNT_REQ),
		Dout     => open
	);

	SaveStateBus_Dout <= SS_V_Dout(0) or SS_V_Dout(1) or SS_V_Dout(2)
	                  or SS_V_Dout(3) or SS_V_Dout(4) or SS_V_Dout(5)
	                  or SS_V_Dout_DBG or SS_V_Dout_DBG2;

	-- CD-quiet (plan P5a, relaxed by P5b): an in-flight READ6 is fully
	-- replayable (command + consumed position saved, stream re-issued on
	-- load), so it no longer blocks the boundary — FMV/streaming saves land
	-- immediately.  Everything else still requires the bus idle and the FIFO
	-- drained.  The replay machinery itself must be quiescent in the blob.
	SS_QUIET <= '1' when RP = RP_IDLE and READ_SKIP = 0 and SWALLOW = 0
	                 and AUD_OPEN = '0'
	                 and ( READ_ACTIVE = '1'
	                    or (SP = SP_FREE and STAT_PEND = '0' and DOUT_PEND = '0'
	                        and EMPTY = '1') )
	            else '0';

end rtl;
