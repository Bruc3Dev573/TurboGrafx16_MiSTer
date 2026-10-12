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

		-- '1' while the CPU-facing FSM is held with a DATAIN byte staged:
		-- cd.vhd must not advertise "data ready" meanwhile
		DATAIN_HELD	: out std_logic;

		-- Savestates
		SLEEP				: in std_logic := '0';
		DTD_EN			: in std_logic := '0';	-- program's CD data-transfer interrupt enable
		SaveStateBus_Din  : in  std_logic_vector(63 downto 0) := (others => '0');
		SaveStateBus_Adr  : in  std_logic_vector(9 downto 0) := (others => '0');
		SaveStateBus_wren : in  std_logic := '0';
		SaveStateBus_rst  : in  std_logic := '0';
		SaveStateBus_load : in  std_logic := '0';
		SaveStateBus_Dout : out std_logic_vector(63 downto 0);
		-- SCSI state is ready for a savestate capture.
		SS_QUIET				: out std_logic;
		-- CDDA replay: sample-exact resume of the audio stream
		CDDA_CONSUMED		: in  unsigned(31 downto 0) := (others => '0');
		AUDIO_CMD_SET		: out std_logic;	-- pulse: D8 accepted, reset the consumed counter
		REPLAY_SKIP			: out unsigned(9 downto 0);	-- samples to discard after replay
		REPLAY_START		: out std_logic;	-- pulse: flush CDDA/SUBC FIFOs, arm the skip
		REPLAY_CONSUMED	: out unsigned(31 downto 0) := (others => '0'); -- counter base before skip
		CD_RESET_DONE		: in std_logic := '0';
		REPLAY_RESET		: out std_logic := '0';
		REPLAY_FAST			: out std_logic := '0';
		-- Post-load audio hold: REPLAY_AUDIO is high from a load that restores
		-- active CDDA until the replay flushes the stream that resumes it
		-- (REPLAY_RESUME, a pulse coincident with that REPLAY_START) or
		-- abandons the resume (pause, stop, abort).
		REPLAY_AUDIO		: out std_logic := '0';
		REPLAY_RESUME		: out std_logic := '0';
		-- The staged DATAIN byte was already consumed by ADPCM DMA.
		-- Latched by cd.vhd from the savestate on load.
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
	signal SAVED_STATUS, SAVED_MESSAGE : std_logic_vector(7 downto 0);
	signal SAVED_RESPONSE : std_logic := '0';
	signal STOP_CD_SND_r : std_logic;
	signal DBO_r        : std_logic_vector(7 downto 0);
	signal DOUT_PEND  : std_logic;
	
	signal DATAIN_CNT 	: unsigned(15 downto 0);

	signal STAT_COUNT    : unsigned(15 downto 0);
	signal DELAY_COUNT   : unsigned(16 downto 0);

	--CDDA replay
	signal AUDIO_ACTIVE  : std_logic;
	signal AUDIO_MODE    : std_logic_vector(1 downto 0);	-- COMM(9)(7:6): 00=LBA 01=MSF 10=track
	signal AUDIO_B1      : std_logic_vector(1 downto 0);	-- D8 byte1: CDDA play mode (values 0-3)
	signal AUDIO_B2, AUDIO_B3, AUDIO_B4, AUDIO_B5 : std_logic_vector(7 downto 0);
	-- The effective audio state is the D8 start point plus the last D9 end
	-- point/mode: both are latched raw and both are re-issued on load.
	signal D9_SEEN       : std_logic;
	signal D9_B1         : std_logic_vector(7 downto 0);	-- D9 byte1: end play mode
	signal D9_MODE       : std_logic_vector(1 downto 0);	-- D9 byte9(7:6)
	signal D9_B2, D9_B3, D9_B4, D9_B5 : std_logic_vector(7 downto 0);
	--READ6 replay
	signal READ_ACTIVE   : std_logic;
	-- a query command ($DD READ SUBQ, $DE CD_DINFO, ...) is outstanding
	signal QRY_ACTIVE    : std_logic;
	-- The HPS mailbox holds one command; wait for status before reusing it.
	signal CMD_OUTST     : std_logic := '0';
	signal CMD_SEND_D    : std_logic := '0';
	signal READ_CONSUMED : unsigned(20 downto 0);	-- bytes handed to the CPU since the 08 accept
	signal READ_SKIP     : unsigned(11 downto 0);	-- replayed-stream bytes still to drain
	signal SWALLOW       : unsigned(1 downto 0);	-- pending replay statuses to discard
	signal RSKIP_GAP     : std_logic := '0';	-- idle cycle after a drain pop (EMPTY is registered)
	signal SWAL_AGE      : unsigned(27 downto 0);	-- wait time for pending replay statuses
	signal SEL_FLUSH     : std_logic := '0';	-- 1-CLK FIFO flush: game SEL while a replay drain was still pending
	signal STAT_GET_D    : std_logic := '0';	-- edge detect: one status per strobe
	type Replay_t is ( RP_IDLE, RP_WAIT, RP_CVT_START, RP_DIV588, RP_CVT_END, RP_LOOPMOD,
	                   RP_HOLDGAP, RP_ISSUE, RP_GAP, RP_ISSUE_D9, RP_LF_CALC,
	                   RP_RD_PREP, RP_RD_ISSUE, RP_PAUSE,
	                   RP_QRY_PREP, RP_QRY_ISSUE, RP_RESET_WAIT,
	                   RP_TOC_ISSUE, RP_TOC_WAIT, RP_TOC_CHECK, RP_TOC_END );
	signal RP            : Replay_t := RP_IDLE;
	signal RP_AFTER      : Replay_t := RP_IDLE;	-- successor after RP_HOLDGAP
	signal RP_DELAY      : unsigned(20 downto 0);
	signal RP_ACC        : unsigned(31 downto 0);	-- working accumulator
	signal RP_SECT       : unsigned(23 downto 0);	-- consumed sectors, then resume LBA
	signal RP_START_LBA  : unsigned(23 downto 0);
	signal RP_CONS       : unsigned(31 downto 0);	-- source count before this replay's stale-tail drift
	signal RP_LEN        : unsigned(23 downto 0);	-- loop length in sectors (0 = unknown)
	signal RP_PRIVATE    : std_logic := '0';
	signal RP_TOC_READY  : std_logic := '0';
	signal RP_TOC_FAILED : std_logic := '0';
	signal RP_TOC_KIND   : natural range 0 to 2 := 0; -- bounds, start, end
	signal RP_TOC_COUNT  : natural range 0 to 5 := 0; -- fifth byte is invalid
	signal RP_TOC_DATA   : std_logic_vector(31 downto 0);
	signal RP_TOC_GOOD   : std_logic;
	signal RP_TOC_LAST   : natural range 0 to 99;
	signal RP_END_LBA    : unsigned(23 downto 0);
	signal RP_COMM_OVR   : std_logic := '0';
	signal RP_COMM       : std_logic_vector(95 downto 0);
	signal RP_SEND       : std_logic := '0';
	signal RP_START_r    : std_logic := '0';
	signal RP_AUDIO      : std_logic := '0';	-- this load's audio resume is still pending
	signal RP_RESUME_r   : std_logic := '0';
	-- The loaded game still awaits the status of its own D8 (saved AUD_OPEN
	-- with AUD_LAST = D8). Latched at load: the replay clears AUD_OPEN.
	signal RP_D8_OWED    : std_logic := '0';
	signal RP_SKIP      : unsigned(9 downto 0);
	signal RP_FIFO_HOLD  : std_logic := '0';	-- clear the data FIFO until the replay command is out
	signal RP_SW_INC     : std_logic := '0';
	signal RP_RSKIP      : unsigned(11 downto 0);
	signal RP_RSKIP_SET  : std_logic := '0';
	signal RP_TAILDRAIN  : std_logic := '0';	-- tail-case drain: CPU owed nothing, flush at end
	-- Retry the audio pair once per load after prolonged silence when an
	-- interrupt-mode track-end notification is still pending.
	signal RP_RETRY_AGE  : unsigned(27 downto 0) := (others => '0');	-- bit27 = 3.1 s
	signal RP_RETRIED    : std_logic := '0';
	signal RP_AUD_DONE   : std_logic := '0';	-- this load's replay included the audio pair
	-- Replaying a completed interrupt-mode track generates another end status.
	-- Discard it unless the restored game still awaits that D9 response.
	signal RP_DEBT_SW    : std_logic := '0';
	-- Retry preserves the D9 completion status for the game.
	-- Its D8 seek status still follows normal replay status accounting.
	signal RP_RETRY_RUN  : std_logic := '0';
	signal RP_SG_D       : std_logic := '0';
	-- Loop repair: the drive loops back to the last seek target, so a looping
	-- track resumed mid-way is re-issued once from its original start on wrap.
	signal LF_ARM        : std_logic := '0';
	signal LF_REPLAY     : std_logic := '0';
	signal LF_TARGET     : unsigned(31 downto 0) := (others => '0');
	signal LF_ACC        : unsigned(31 downto 0) := (others => '0');
	signal LF_CNT        : unsigned(20 downto 0) := (others => '0');
	signal RP_STWD       : unsigned(25 downto 0) := (others => '0');	-- status-serialization watchdog (bit25 = 0.78 s)
	signal AUD_OPEN      : std_logic := '0';	-- audio-family command sent, its status not yet arrived
	signal AUD_AGE       : unsigned(25 downto 0) := (others => '0');	-- AUD_OPEN timeout (bit25 = 0.78 s)
	signal AUD_LAST      : std_logic_vector(1 downto 0) := "00";	-- which audio command is owed: 00=D8 01=D9 10=DA
	signal TAIL_FLUSH    : std_logic := '0';	-- 1-CLK FIFO aclr on the tail drain's last pop
	signal RP_FREEZE     : std_logic;
	signal DATAIN_HOLD   : std_logic;	-- mask CPU-visible data-ready while the FSM is held
	-- Two replay commands must sit one HPS status poll tick (13-16 ms) apart:
	-- they share a single mailbox slot.
	constant RP_T_SETTLE : unsigned(20 downto 0) := to_unsigned(65535, 21);	-- 1.5 ms
	constant RP_T_GAP    : unsigned(20 downto 0) := to_unsigned(1048575, 21);	-- 24 ms

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

	function bcd_value(v : std_logic_vector(7 downto 0)) return natural is
	begin
		return to_integer(unsigned(v(7 downto 4))) * 10
		     + to_integer(unsigned(v(3 downto 0)));
	end function;

	function bcd_valid(v : std_logic_vector(7 downto 0)) return boolean is
	begin
		return unsigned(v(7 downto 4)) <= 9 and unsigned(v(3 downto 0)) <= 9;
	end function;

	function to_bcd(v : natural) return std_logic_vector is
	begin
		return std_logic_vector(to_unsigned(v / 10, 4))
		     & std_logic_vector(to_unsigned(v mod 10, 4));
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

	
	-- Flushed on load: it may hold bytes pushed by the pre-load stream during
	-- the freeze.  SEL_FLUSH drops an abandoned replay drain's bytes.
	FIFO : entity work.SCSI_FIFO 
	port map(
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
			RSKIP_GAP <= '0';
			SWAL_AGE <= (others => '0');
			AUD_OPEN <= '0';
			AUD_AGE <= (others => '0');
			AUD_LAST <= "00";
			SAVED_RESPONSE <= '0';
			SAVED_STATUS <= (others => '0');
			SAVED_MESSAGE <= (others => '0');
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
			-- The FIFO stays clocked during SLEEP. Complete a pending pop once,
			-- then clear its strobe even while the protocol state is frozen.
			FIFO_RD_REQ <= '0';
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
				D9_B2   <= SS_V(5)(7 downto 0);
				D9_B3   <= SS_V(5)(15 downto 8);
				D9_B4   <= SS_V(5)(23 downto 16);
				D9_B5   <= SS_V(5)(31 downto 24);
				D9_B1   <= SS_V(5)(39 downto 32);
				D9_MODE <= SS_V(5)(41 downto 40);
				D9_SEEN <= SS_V(5)(42);
				AUD_OPEN <= SS_V(0)(60);
				AUD_LAST <= SS_V(0)(62 downto 61);
				AUD_AGE <= (others => '0');
				SAVED_RESPONSE <= '0';
				if SS_V(0)(58) = '1'
				   and (SS_V(0)(57 downto 56) = "10"
				        or (SS_V(5)(42) = '1' and SS_V(5)(41 downto 40) = "10"))
				   and (SS_V(0)(18) = '1'
				        or (unsigned(SS_V(0)(3 downto 0)) >= 4
				            and unsigned(SS_V(0)(3 downto 0)) <= 9)) then
					SAVED_RESPONSE <= '1';
				end if;
				READ_CONSUMED <= unsigned(SS_V(5)(63 downto 43));
				-- never captured non-zero (the quiet gate forbids it): just clear
				READ_SKIP <= (others => '0');
				SWALLOW   <= (others => '0');
				RSKIP_GAP <= '0';
				SWAL_AGE  <= (others => '0');
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
			-- Top-level status registers restore on the load edge too: sample
			-- their restored values before private GOOD responses overwrite them.
			if RP = RP_WAIT and RP_DELAY = RP_T_SETTLE - 1 then
				SAVED_STATUS <= STATUS;
				SAVED_MESSAGE <= MESSAGE;
			end if;
			-- Runs during the freeze too: a status arriving mid-walk must not be
			-- lost.  A swallow whose status never arrives ages out after 3.1 s.
			if SWALLOW /= 0 then
				SWAL_AGE <= SWAL_AGE + 1;
				if SWAL_AGE(27) = '1' then
					SWALLOW  <= (others => '0');
					SWAL_AGE <= (others => '0');
				end if;
			else
				SWAL_AGE <= (others => '0');
			end if;
			-- AUD_OPEN ages out after 0.78 s (the longest legitimate wait is one
			-- emulated seek): a lost status must not veto the quiet gate forever.
			if AUD_OPEN = '1' and RP_PRIVATE = '0'
			   and not (RP_TOC_READY = '1' and RP /= RP_IDLE) then
				AUD_AGE <= AUD_AGE + 1;
				if AUD_AGE(25) = '1' then
					AUD_OPEN <= '0';
					AUD_AGE <= (others => '0');
				end if;
			else
				AUD_AGE <= (others => '0');
			end if;
			STAT_GET_D <= STAT_GET;
			if STAT_GET = '1' and STAT_GET_D = '0' and RP_PRIVATE = '0' then
				READ_ACTIVE <= '0';	-- a status closes the in-flight READ
				if RP_TOC_READY = '0' or SWALLOW = 0 then
					AUD_OPEN <= '0'; -- a swallowed D8 does not own the saved D9 debt
				end if;
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
			
			if DOUT_REQ = '1' and RP_PRIVATE = '0' then
				DOUT_PEND <= '1';
			end if;
			
			if SLEEP = '0' then

			COMM_OUT <= '0';
			DATA_OUT <= '0';
			CD_DATA_END <= '0';
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
					-- Post-replay drain: discard the already-consumed head of the
					-- re-issued stream. A seek delay must not cancel the byte debt.
					-- EMPTY is registered: one idle cycle per pop (RSKIP_GAP) keeps
					-- one decrement equal to one byte.
					if RSKIP_GAP = '1' then
						RSKIP_GAP <= '0';
					elsif EMPTY = '0' then
						FIFO_RD_REQ <= '1';
						READ_SKIP <= READ_SKIP - 1;
						RSKIP_GAP <= '1';
						-- Tail drain: nothing in the re-read belongs to the CPU, so
						-- flush on the last pop (the in-flight byte dies in the aclr).
						if READ_SKIP = 1 and RP_TAILDRAIN = '1' then
							TAIL_FLUSH <= '1';	-- RP_TAILDRAIN is cleared by its owner (replay process)
						end if;
					end if;
				elsif RP_FREEZE = '0' then
				case SP is
					when SP_FREE =>
						if SEL_N = '0' then
							SAVED_RESPONSE <= '0';
							-- a new command abandons the tracked READ stream
							READ_ACTIVE <= '0';
							QRY_ACTIVE <= '0';
							-- A game selection retires all replay bookkeeping: a leftover
							-- swallow could only eat the game's own next status.
							SWALLOW  <= (others => '0');
							SWAL_AGE <= (others => '0');
							if READ_SKIP /= 0 then
								SEL_FLUSH <= '1';	-- drop the abandoned re-read's queued bytes
							end if;
							READ_SKIP    <= (others => '0');
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
								if SAVED_RESPONSE = '1' then
									DBO_r <= SAVED_STATUS;
								else
									DBO_r <= STATUS;
								end if;
								BSY_Nr <= '0';
								MSG_Nr <= '1';
								CD_Nr <= '0';
								IO_Nr <= '0';
								REQ_Nr <= '0';
								SP <= SP_STAT_START;
							end if;
						-- READ_SKIP/TAIL_FLUSH guards: during a tail drain the phase is
						-- free and this arm must not claim the last pop's byte.
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
									AUDIO_ACTIVE <= '0';	-- audio no longer resumable as-is
								end if;
								if (COMM(0) = x"08") then	-- track the data stream for replay
									READ_ACTIVE   <= '1';
									READ_CONSUMED <= (others => '0');
								end if;
								-- Anything else is a query the drive still owes an answer to
								-- ($DD READ SUBQ, $DE CD_DINFO, ...): tracked for the replay.
								if COMM(0) /= x"08" and COMM(0) /= x"D8"
								   and COMM(0) /= x"D9" and COMM(0) /= x"DA" then
									QRY_ACTIVE <= '1';
								end if;
								if (COMM(0) = x"D8") then	-- SAPSP: latch the raw start point
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
								if (COMM(0) = x"D9") then	-- SAPEP: latch the raw end point
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
								-- An audio command's status may be pended HPS-side behind the
								-- emulated seek: veto the quiet gate until it arrives.
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
							if SAVED_RESPONSE = '1' then
								DBO_r <= SAVED_MESSAGE;
							else
								DBO_r <= MESSAGE;
							end if;
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
							SAVED_RESPONSE <= '0';
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

	STOP_CD_SND <= STOP_CD_SND_r or RP_TOC_FAILED;
	DBO <= DBO_r;
	
	BSY_N <= BSY_Nr;
	MSG_N <= MSG_Nr;
	CD_N <= CD_Nr;
	IO_N <= IO_Nr;

	REQ_N <= REQ_Nr;
	
	-- Only the CPU-visible "data ready" is masked, never REQ_N itself: an ACK
	-- caught in flight must still complete once the FSM resumes.
	DATAIN_HOLD <= '1' when (RP_FREEZE = '1' or READ_SKIP /= 0)
	                    and (SP = SP_DATAIN_START or SP = SP_DATAIN_END)
	               else '0';
	DATAIN_HELD <= DATAIN_HOLD;

	COMMAND <= RP_COMM when RP_COMM_OVR = '1' else
	           COMM(11) & COMM(10) & COMM(9) & COMM(8) & COMM(7) & COMM(6) & COMM(5) & COMM(4) & COMM(3) & COMM(2) & COMM(1) & COMM(0);
	COMM_SEND <= COMM_OUT or RP_SEND;
	REPLAY_START <= RP_START_r;
	REPLAY_AUDIO <= RP_AUDIO;
	REPLAY_RESUME <= RP_RESUME_r;
	REPLAY_SKIP <= RP_SKIP;
	REPLAY_CONSUMED <= RP_CONS - resize(RP_SKIP, 32);
	-- Replayed D8s reposition the drive without Main's physical-seek delay,
	-- except the load replay (and its retry) of a D8 whose status the saved
	-- game still awaits: that seek keeps the latency the game was waiting
	-- through. Private TOC reconstruction and LOOP repair are always fast.
	REPLAY_FAST <= '1' when (RP_D8_OWED = '0' or RP_TOC_READY = '1' or LF_REPLAY = '1')
	                       and RP_SEND = '1' and RP_COMM(7 downto 0) = x"D8" else '0';

	-- Replay restores the drive state after a load. Commands share one HPS
	-- mailbox; wait for each status before issuing the next command.
	-- RP_WAIT settles restored registers and captures CDDA_CONSUMED.
	-- RP_RESET_WAIT/TOC_* reset the drive and resolve track addresses.
	-- RP_CVT_*/DIV588/LOOPMOD convert the resume position to LBA and sample skip.
	-- RP_ISSUE/D9 restore audio; RP_LF_CALC restores its original loop target.
	-- RP_RD_*/QRY_* reissue reads and queries, draining already-consumed bytes.
	-- RP_PAUSE stops a stream with nothing left to resume.
	-- Discard replay-only audio statuses; preserve responses still owed to the
	-- game, including READ6/query completion. RP_FREEZE holds the CPU-facing
	-- FSM, and RP_FIFO_HOLD clears stale data until the replacement stream starts.
	process( CLK, RESET_N )
		variable msf_bin : unsigned(23 downto 0);
		variable end_bin : unsigned(23 downto 0);
		variable nxt_v   : unsigned(20 downto 0);
		variable sect_v  : unsigned(9 downto 0);
		variable cnt9_v  : unsigned(8 downto 0);
		variable lba_v   : unsigned(20 downto 0);
		variable track_v : natural range 0 to 165;
		variable frames_v : natural range 0 to 742499;
		variable toc_ok : boolean;
	begin
		if RESET_N = '0' then
			RP <= RP_IDLE;
			RP_AFTER <= RP_IDLE;
			RP_COMM_OVR <= '0';
			RP_SEND <= '0';
			RP_START_r <= '0';
			RP_AUDIO <= '0';
			RP_RESUME_r <= '0';
			RP_FIFO_HOLD <= '0';
			RP_SW_INC <= '0';
			RP_RSKIP_SET <= '0';
			RP_PRIVATE <= '0';
			RP_TOC_READY <= '0';
			RP_TOC_FAILED <= '0';
			REPLAY_RESET <= '0';
			RP_TAILDRAIN <= '0';
			RP_STWD <= (others => '0');
			RP_RETRY_AGE <= (others => '0');
			RP_RETRIED <= '0';
			RP_AUD_DONE <= '0';
			RP_RETRY_RUN <= '0';
			LF_ARM <= '0';
			LF_REPLAY <= '0';
			RP_DEBT_SW <= '0';
			RP_D8_OWED <= '0';
			RP_SKIP <= (others => '0');
		elsif rising_edge(CLK) then
			RP_SEND <= '0';
			RP_START_r <= '0';
			RP_RESUME_r <= '0';
			RP_SW_INC <= '0';
			RP_RSKIP_SET <= '0';
			REPLAY_RESET <= '0';

			case RP is
				when RP_IDLE =>
					RP_COMM_OVR <= '0';
					LF_REPLAY <= '0';
					RP_AUDIO <= '0';
					-- The tail flag is stale once the drain is over (RP_RSKIP_SET:
					-- READ_SKIP is not loaded yet on the first IDLE cycle).
					if READ_SKIP = 0 and RP_RSKIP_SET = '0' and RP_TAILDRAIN = '1' then
						RP_TAILDRAIN <= '0';
					end if;
					-- Retry requires 3.1 s without a status, command or selection,
					-- an idle bus, and an interrupt-mode track end still pending.
					RP_RETRY_RUN <= '0';	-- back in IDLE: the retry pass is over
					RP_SG_D <= STAT_GET;
					-- The game's own commands always supersede a replayed one
					if COMM_OUT = '1' or SEL_N = '0' then
						if COMM_OUT = '1' then
							RP_PRIVATE <= '0';
							RP_TOC_FAILED <= '0';
							RP_TOC_READY <= '0';
							RP_FIFO_HOLD <= '0';
						end if;
						LF_ARM <= '0';
					elsif LF_ARM = '1' and CDDA_CONSUMED >= LF_TARGET
					   and STAT_PEND = '0'
					   and (SP = SP_FREE or
					        (SP = SP_STAT_START and ACK_N = '1' and AUD_OPEN = '0')) then
						-- A presented, unacknowledged status can outlive the track.
						-- Freeze that reply while repairing only the physical drive.
						LF_ARM <= '0';
						LF_REPLAY <= '1';
						RP_SKIP <= (others => '0');
						RP_CONS <= CDDA_CONSUMED;
						RP_SECT <= resize(RP_START_LBA, RP_SECT'length);
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
							RP_RETRY_RUN <= '1';	-- preserve the retry's D9 completion
							RP_CONS <= CDDA_CONSUMED;
							RP <= RP_CVT_START;
						end if;
					end if;
					if SaveStateBus_load = '1' then
						RP <= RP_WAIT;
						RP_DELAY <= RP_T_SETTLE;
						RP_FIFO_HOLD <= '1';
						RP_AUDIO <= SS_V(0)(58);	-- restored AUDIO_ACTIVE
						RP_D8_OWED <= '0';
						if SS_V(0)(60) = '1' and SS_V(0)(62 downto 61) = "00" then
							RP_D8_OWED <= '1';
						end if;
						RP_PRIVATE <= '0';
						if SS_V(0)(58) = '1'
						   and (SS_V(0)(57 downto 56) = "10"
						        or (SS_V(5)(42) = '1' and SS_V(5)(41 downto 40) = "10")) then
							RP_PRIVATE <= '1';
						end if;
						RP_TOC_READY <= '0';
						RP_TOC_FAILED <= '0';
						RP_STWD <= (others => '0');
						RP_RETRIED <= '0';
						RP_AUD_DONE <= '0';
						RP_RETRY_RUN <= '0';
						LF_ARM <= '0';
						LF_REPLAY <= '0';
						RP_RETRY_AGE <= (others => '0');
					end if;

				when RP_WAIT =>
					-- Latch CDDA_CONSUMED a few cycles in: the stale pre-load FIFO
					-- tail is still playing and would inflate it by the settle time.
					if RP_DELAY = RP_T_SETTLE - 32 then
						RP_CONS <= CDDA_CONSUMED;
					end if;
					if RP_DELAY = 0 then
						if AUDIO_ACTIVE = '1' then
							if RP_PRIVATE = '1' then
								REPLAY_RESET <= '1';
								RP_STWD <= (others => '0');
								RP <= RP_RESET_WAIT;
							else
								RP <= RP_CVT_START;
							end if;
						elsif READ_ACTIVE = '1' then
							RP <= RP_RD_PREP;
						else
							RP_SKIP <= (others => '0');	-- pause path: flush only, no drain
							RP_DELAY <= RP_T_GAP;
							RP_AFTER <= RP_PAUSE;
							RP <= RP_HOLDGAP;
						end if;
					else
						RP_DELAY <= RP_DELAY - 1;
					end if;

				when RP_RESET_WAIT =>
					if CD_RESET_DONE = '1' then
						RP_STWD <= (others => '0');
						RP_TOC_KIND <= 0;
						RP <= RP_TOC_ISSUE;
					end if;

				when RP_TOC_ISSUE =>
					-- Main strips the transfer header: exactly four raw CD_DATA
					-- bytes followed by GOOD, without a CD_DATA_END handshake.
					RP_COMM <= (others => '0');
					RP_COMM(7 downto 0) <= x"DE";
					if RP_TOC_KIND = 1 then
						track_v := bcd_value(AUDIO_B2);
						if track_v = 0 then track_v := 1; end if;
						if track_v > RP_TOC_LAST then track_v := RP_TOC_LAST; end if;
						RP_COMM(15 downto 8) <= x"02";
						RP_COMM(23 downto 16) <= to_bcd(track_v);
					elsif RP_TOC_KIND = 2 then
						track_v := bcd_value(D9_B2);
						if track_v = 0 then track_v := 1; end if;
						if D9_SEEN = '0' or track_v > RP_TOC_LAST then
							RP_COMM(15 downto 8) <= x"01"; -- leadout
						else
							RP_COMM(15 downto 8) <= x"02";
							RP_COMM(23 downto 16) <= to_bcd(track_v);
						end if;
					end if;
					RP_TOC_COUNT <= 0;
					RP_TOC_GOOD <= '0';
					RP_TOC_DATA <= (others => '0');
					RP_COMM_OVR <= '1';
					RP_SEND <= '1';
					RP_STWD <= (others => '0');
					RP <= RP_TOC_WAIT;

				when RP_TOC_WAIT =>
					if CD_WR = '1' and CD_WR_OLD = '0' then
						if RP_TOC_COUNT < 4 then
							RP_TOC_DATA(RP_TOC_COUNT * 8 + 7 downto RP_TOC_COUNT * 8) <= CD_DATA;
						end if;
						if RP_TOC_COUNT < 5 then
							RP_TOC_COUNT <= RP_TOC_COUNT + 1;
						end if;
					end if;
					if STAT_GET = '1' and STAT_GET_D = '0' then
						if STATUS = x"00" then RP_TOC_GOOD <= '1'; end if;
						RP_STWD <= (others => '0');
						RP <= RP_TOC_CHECK;
					end if;

				when RP_TOC_CHECK =>
					toc_ok := false;
					if RP_TOC_COUNT = 4 and RP_TOC_GOOD = '1' then
						if RP_TOC_KIND = 0 then
							if RP_TOC_DATA(7 downto 0) = x"01"
							   and bcd_valid(RP_TOC_DATA(15 downto 8))
							   and bcd_value(RP_TOC_DATA(15 downto 8)) /= 0 then
								RP_TOC_LAST <= bcd_value(RP_TOC_DATA(15 downto 8));
								toc_ok := true;
								if AUDIO_MODE = "10" then
									RP_TOC_KIND <= 1;
									RP <= RP_TOC_ISSUE;
								else
									RP <= RP_TOC_END;
								end if;
							end if;
						elsif bcd_valid(RP_TOC_DATA(7 downto 0))
						  and bcd_valid(RP_TOC_DATA(15 downto 8))
						  and bcd_valid(RP_TOC_DATA(23 downto 16))
						  and bcd_value(RP_TOC_DATA(15 downto 8)) < 60
						  and bcd_value(RP_TOC_DATA(23 downto 16)) < 75 then
							frames_v := (bcd_value(RP_TOC_DATA(7 downto 0)) * 60
							             + bcd_value(RP_TOC_DATA(15 downto 8))) * 75
							             + bcd_value(RP_TOC_DATA(23 downto 16));
							if frames_v >= 150 then
								toc_ok := true;
								if RP_TOC_KIND = 1 then
									RP_START_LBA <= to_unsigned(frames_v - 150, 24);
									RP <= RP_TOC_END;
								else
									RP_END_LBA <= to_unsigned(frames_v - 150, 24);
									RP_TOC_READY <= '1';
									RP_PRIVATE <= '0';
									RP <= RP_CVT_START;
								end if;
							end if;
						end if;
					end if;
					if not toc_ok then
						-- Never seek using an unverified TOC. Keep late private
						-- traffic quarantined until the game starts a new command.
						RP_TOC_FAILED <= '1';
						RP_SKIP <= (others => '0');
						RP_START_r <= '1';
						RP <= RP_IDLE;
					end if;

				when RP_TOC_END =>
					if D9_SEEN = '0' or D9_MODE = "10" then
						RP_TOC_KIND <= 2;
						RP <= RP_TOC_ISSUE;
					else
						RP_TOC_READY <= '1';
						RP_PRIVATE <= '0';
						RP <= RP_CVT_START;
					end if;

				when RP_CVT_START =>	-- D8 start point -> binary LBA
					if AUDIO_MODE = "01" then	-- MSF (BCD) in B2..B4
						msf_bin := to_unsigned(
							(bcd_value(AUDIO_B2) * 60 + bcd_value(AUDIO_B3)) * 75
							+ bcd_value(AUDIO_B4), 24) - 150;
						RP_START_LBA <= msf_bin;
					elsif AUDIO_MODE /= "10" then	-- Main's default, including mode 11
						RP_START_LBA <= unsigned(AUDIO_B3) & unsigned(AUDIO_B4) & unsigned(AUDIO_B5);
					end if; -- track mode keeps the privately resolved original start
					RP_ACC  <= RP_CONS;
					RP_SECT <= (others => '0');
					RP <= RP_DIV588;

				when RP_DIV588 =>	-- consumed samples -> sectors + sub-sector skip
					if RP_ACC >= 588 then
						RP_ACC  <= RP_ACC - 588;
						RP_SECT <= RP_SECT + 1;
					else
						RP_SKIP <= RP_ACC(9 downto 0);
						RP <= RP_CVT_END;
					end if;

				-- The SAPEP window includes the end sector: length = end - start + 1.
				when RP_CVT_END =>	-- last D9 end point -> loop length (when computable)
					RP_LEN <= (others => '0');
					if D9_SEEN = '1' then
						if D9_MODE = "10" then
							end_bin := RP_END_LBA;
						elsif D9_MODE = "01" then
							end_bin := to_unsigned(
								(bcd_value(D9_B2) * 60 + bcd_value(D9_B3)) * 75
								+ bcd_value(D9_B4), 24) - 150;
						else -- Main's default, including mode 11
							end_bin := unsigned(D9_B3) & unsigned(D9_B4) & unsigned(D9_B5);
						end if;
						if end_bin >= RP_START_LBA then
							RP_LEN <= end_bin - RP_START_LBA + 1;
						end if;
					elsif AUDIO_MODE = "10" then
						if RP_END_LBA >= RP_START_LBA then
							RP_LEN <= RP_END_LBA - RP_START_LBA + 1;
						end if;
					end if;
					RP <= RP_LOOPMOD;

				when RP_LOOPMOD =>	-- wrap the position into the loop window
					if RP_LEN /= 0 and RP_SECT >= RP_LEN then
						if (D9_SEEN = '1' and D9_B1(1 downto 0) = "01")
						   or (D9_SEEN = '0' and AUDIO_B1 = "01") then -- LOOP
							RP_SECT <= RP_SECT - RP_LEN;
						elsif D9_SEEN = '1' and D9_B1(1 downto 0) = "10" then
							-- Audio has ended, but interrupt-mode SAPEP still needs
							-- a drive-generated completion. Replay the last sector.
							-- RP_DEBT_SW discards the duplicate D9 status unless the
							-- saved game still awaits it or this is the retry pass.
							RP_SECT <= RP_START_LBA + RP_LEN - 1;
							RP_DEBT_SW <= '1';
							RP_AUDIO <= '0';	-- nothing audible to resume
							RP_SKIP <= (others => '0');
							RP_DELAY <= RP_T_GAP;
							RP_AFTER <= RP_ISSUE;
							RP <= RP_HOLDGAP;
						else
							-- non-loop audio with no debt: nothing to resume, just
							-- silence the live stream
							RP_SKIP <= (others => '0');
							RP_AUDIO <= '0';
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
						-- Serialize on statuses: the single HPS status slot is overwritten
						-- by the next command; RP_STWD(25) = 0.78 s if it never arrives.
						if SWALLOW = 0 and (AUD_OPEN = '0' or RP_TOC_READY = '1') and CMD_OUTST = '0' then
							RP_STWD <= (others => '0');
							RP <= RP_AFTER;
						elsif RP_STWD(25) = '1' then
							-- Abort replay after 0.78 s without status rather than
							-- overwrite an outstanding drive command.
							RP_STWD <= (others => '0');
							RP <= RP_IDLE;
						else
							RP_STWD <= RP_STWD + 1;
						end if;
					else
						RP_DELAY <= RP_DELAY - 1;
					end if;

				when RP_ISSUE =>	-- re-issue the D8 at the resume position
					RP_AUD_DONE <= '1';	-- allow one retry of this audio pair
					RP_COMM <= (others => '0');
					RP_COMM(7 downto 0) <= x"D8";
					RP_COMM(9 downto 8) <= AUDIO_B1;	-- byte1: original CDDA play mode
					RP_COMM(31 downto 24) <= std_logic_vector(RP_SECT(23 downto 16));
					RP_COMM(39 downto 32) <= std_logic_vector(RP_SECT(15 downto 8));
					RP_COMM(47 downto 40) <= std_logic_vector(RP_SECT(7 downto 0));
					RP_COMM_OVR <= '1';
					RP_SEND <= '1';
					RP_START_r <= '1';	-- flush CDDA/SUBC FIFOs + arm the sample skip
					-- A PLAY-mode D8 streams from here: this flush resumes the
					-- audio. A silent D8 waits for its D9; without one it plays
					-- nothing.
					if AUDIO_B1 /= "00" then
						RP_RESUME_r <= RP_AUDIO;
						RP_AUDIO <= '0';
					elsif D9_SEEN = '0' then
						RP_AUDIO <= '0';
					end if;
					-- Preserve D8 status only when the saved game still awaits
					-- that seek response. Retry does not change its ownership:
					-- a replay-only D8 status is not the game's D9 completion.
					if not (AUD_OPEN = '1' and AUD_LAST = "00") then
						RP_SW_INC <= '1';	-- the D8 pends one status
					end if;
					RP_FIFO_HOLD <= '0';
					if D9_SEEN = '1' then
						RP_DELAY <= RP_T_GAP;
						RP <= RP_GAP;
					elsif AUDIO_MODE = "10" and AUDIO_B1 = "01" and RP_LEN /= 0
					      and RP_SECT > RP_START_LBA then
						-- Without D9, Main loops D8 to the disc leadout.
						LF_CNT <= resize(RP_LEN - (RP_SECT - RP_START_LBA), 21);
						LF_ACC <= (others => '0');
						RP <= RP_LF_CALC;
					elsif QRY_ACTIVE = '1' then
						RP <= RP_QRY_PREP;
					else
						RP <= RP_IDLE;
					end if;

				when RP_GAP =>		-- one HPS poll tick between the two commands
					if RP_DELAY = 0 then
						-- same status-serialization gate as RP_HOLDGAP: the
						-- D8's status is pended behind the seek latency
						if SWALLOW = 0 and (AUD_OPEN = '0' or RP_TOC_READY = '1') and CMD_OUTST = '0' then
							RP_STWD <= (others => '0');
							RP <= RP_ISSUE_D9;
						elsif RP_STWD(25) = '1' then
							RP_STWD <= (others => '0');
							RP <= RP_IDLE;	-- no D9 while D8 remains unanswered
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
					RP_AUDIO <= '0';
					-- A SILENT D8 streams nothing during the gap: flush and re-arm the
					-- skip.  A PLAY-mode D8 has already consumed (part of) its skip.
					if AUDIO_B1 = "00" then
						RP_START_r <= '1';
						RP_RESUME_r <= RP_AUDIO;
					end if;
					-- Discard immediate D9 status and duplicate track-end status
					-- marked by RP_DEBT_SW. Preserve status for an outstanding
					-- game D9 or the retry pass.
					if (D9_B1(1 downto 0) /= "10" or RP_DEBT_SW = '1')
					   and not (AUD_OPEN = '1' and AUD_LAST = "01")
					   and RP_RETRY_RUN = '0' then
						RP_SW_INC <= '1';
					end if;
					RP_DEBT_SW <= '0';	-- applies only to this replayed pair
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
						-- Keep the original lap count; remove the replayed sector's
						-- skip, then add its remaining sectors through the loop end.
						LF_TARGET <= RP_CONS - resize(RP_SKIP, 32) + LF_ACC;
						LF_ARM <= '1';
						if QRY_ACTIVE = '1' then
							RP <= RP_QRY_PREP;
						else
							RP <= RP_IDLE;
						end if;
					end if;

				when RP_RD_PREP =>	-- rebuild the in-flight READ6
					-- nxt_v is one past the last byte already held by the consumer.
					-- Keep the restored DBO byte; drain its replayed copy.
					if SP = SP_DATAIN_START or SP = SP_DATAIN_END then
						if DMA_STAGED = '1' then	-- the DMA already stored the staged byte
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
						-- Only the final STATUS is owed and the HPS sends it after
						-- streaming: re-read the last sector and drain it whole
						lba_v := lba_v + resize(cnt9_v, 21) - 1;
						RP_COMM(39 downto 32) <= x"01";
						RP_RSKIP <= to_unsigned(2048, 12);
						-- the CPU is owed nothing from the re-read: flush at the end
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

				-- Re-issued after the audio/pause command: the data FIFO is held
				-- clear until then and the mailbox takes one command at a time.
				when RP_QRY_PREP =>
					RP_COMM <= COMM(11) & COMM(10) & COMM(9) & COMM(8)
					         & COMM(7) & COMM(6) & COMM(5) & COMM(4)
					         & COMM(3) & COMM(2) & COMM(1) & COMM(0);
					-- DATAIN_CNT counts consumed answer bytes since SEL.
					-- Also drain the replayed copy of any restored staged byte.
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
					RP_AUDIO <= '0';
					-- flush + realign the CDDA/SUBC assemblers here too: a stale
					-- mid-frame phase would poison the game's own next SAPSP
					RP_START_r <= '1';
					-- Only active audio can still owe its interrupt completion.
					-- A game PAUSE/READ retires that debt but leaves D9 history intact.
					if AUDIO_ACTIVE = '1' and D9_SEEN = '1'
					   and D9_B1(1 downto 0) = "10" then
						RP_AUD_DONE <= '1';
					elsif not (AUD_OPEN = '1' and AUD_LAST = "10")
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
			if (RP = RP_RESET_WAIT and CD_RESET_DONE = '0')
			   or (RP = RP_TOC_WAIT and not (STAT_GET = '1' and STAT_GET_D = '0')) then
				RP_STWD <= RP_STWD + 1;
				if RP_STWD(25) = '1' then
					RP_TOC_FAILED <= '1';
					RP_SKIP <= (others => '0');
					RP_START_r <= '1';
					RP_STWD <= (others => '0');
					RP <= RP_IDLE;
				end if;
			end if;
		end if;
	end process;
	
	-- Held until the replay's statuses have been swallowed too: statuses carry
	-- no identity, so a game command issued meanwhile would lose its own.
	RP_FREEZE <= '0' when RP = RP_IDLE and SWALLOW = 0 else '1';
	
	DOUT <= DATA_BUF(9) & DATA_BUF(8) & DATA_BUF(7) & DATA_BUF(6) & DATA_BUF(5) & DATA_BUF(4) & DATA_BUF(3) & DATA_BUF(2) & DATA_BUF(1) & DATA_BUF(0);
	DOUT_SEND <= DATA_OUT;
	
	DBG_DATAIN_CNT <= DATAIN_CNT;

	-- Savestates (slot map: pce_savestates_pkg.vhd).
	-- FIFO contents are not saved: READ6 replay reconstructs unread data.
	-- Transfer strobes are transient and are not saved.

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
	SS_V_BACK(0)(59)           <= QRY_ACTIVE;
	SS_V_BACK(0)(60)           <= AUD_OPEN;
	SS_V_BACK(0)(62 downto 61) <= AUD_LAST;
	SS_V_BACK(0)(63)           <= '0';

	-- SCSI_3: last SAPEP raw + in-flight READ position.
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

	-- Track the command awaiting HPS status to serialize mailbox writes.
	-- On load, restore any pending audio response from AUD_OPEN.
	-- Drive reset clears physical work, not the game's saved response.
	process(CLK) begin
		if rising_edge(CLK) then
			if RESET_N = '0' then
				CMD_OUTST <= '0'; CMD_SEND_D <= '0';
			else
				-- Quartus 17 cannot read out ports; use the internal drivers.
				CMD_SEND_D <= COMM_OUT or RP_SEND;
				if SaveStateBus_load = '1' then
					CMD_OUTST <= SS_V(0)(60);
				elsif CD_RESET_DONE = '1' and RP = RP_RESET_WAIT then
					CMD_OUTST <= '0'; -- live drive work is gone; AUD_OPEN is FPGA debt
				elsif (COMM_OUT or RP_SEND) = '1' and CMD_SEND_D = '0' then
					CMD_OUTST <= '1';
				elsif STAT_GET = '1' and STAT_GET_D = '0' then
					CMD_OUTST <= '0';
				end if;
			end if;
		end if;
	end process;

	SaveStateBus_Dout <= SS_V_Dout(0) or SS_V_Dout(1) or SS_V_Dout(2)
	                  or SS_V_Dout(3) or SS_V_Dout(4) or SS_V_Dout(5);

	-- Replay, drains and pending replay/audio statuses block capture.
	-- An active READ6 can be reconstructed even with unread FIFO data.
	-- Otherwise require no pending status/data-out request and an empty FIFO.
	-- The phase and handshake registers are saved, so SP need not be SP_FREE.
	SS_QUIET <= '1' when RP = RP_IDLE and READ_SKIP = 0 and SWALLOW = 0
	                 and AUD_OPEN = '0'
	                 and ( READ_ACTIVE = '1'
	                    or (STAT_PEND = '0' and DOUT_PEND = '0'
	                        and EMPTY = '1') )
	            else '0';

end rtl;
