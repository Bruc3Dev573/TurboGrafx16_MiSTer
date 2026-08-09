library STD;
use STD.TEXTIO.ALL;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_TEXTIO.all;
use IEEE.NUMERIC_STD.ALL;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity cd is
	port(
		RST_N			: in  std_logic;
		CLK 			: in  std_logic;
		EN 			: in  std_logic;

		EXT_A 		: in  std_logic_vector(20 downto 0);
		EXT_DI 		: in  std_logic_vector(7 downto 0);
		EXT_DO 		: out std_logic_vector(7 downto 0);
		EXT_WR_N		: in  std_logic;
		EXT_RD_N		: in  std_logic;
		CPU_CE		: in  std_logic;
		
		SEL_N			: out std_logic;
		IRQ_N			: out std_logic;
		
		RAM_CS_N		: out std_logic;
		BRAM_EN		: out std_logic;
		
		CD_STAT		: in std_logic_vector(7 downto 0);
		CD_MSG		: in std_logic_vector(7 downto 0);
		CD_STAT_GET	: in std_logic;
		
		CD_COMM		: out std_logic_vector(95 downto 0);
		CD_COMM_SEND: out std_logic;
		
		CD_DOUT_REQ	: in std_logic;
		CD_DOUT		: out std_logic_vector(79 downto 0);
		CD_DOUT_SEND: out std_logic;
		
		CD_REGION   : in  std_logic;
		CD_RESET		: out std_logic;
		
		CD_DATA		: in std_logic_vector(7 downto 0);
		CD_DATA_WR	: in std_logic;
		CD_AUDIO_WR	: in std_logic;
		CD_SUBCD_WR	: in std_logic;
		CD_DATA_END	: out std_logic;
		
		DM				: in std_logic;
		
		CD_SL			: out signed(15 downto 0);
		CD_SR			: out signed(15 downto 0);
		AD_S			: out signed(15 downto 0);

		-- Savestates (plan P5a; slots 53-54 + 60-63, SCSI owns 55-59).
		-- SLEEP freezes the consuming/decoding side; the CDDA/SUBC FIFO fill
		-- paths stay live so no sample pushed by the HPS during the freeze is
		-- lost (the FIFOs absorb ~10ms easily).
		SLEEP				: in std_logic := '0';
		-- raw sleep REQUEST (boundary wait in progress) - arms the SCSI
		-- phase-starvation valve; see SS_PEND in SCSI.vhd
		SS_PEND			: in std_logic := '0';
		SaveStateBus_Din  : in  std_logic_vector(63 downto 0) := (others => '0');
		SaveStateBus_Adr  : in  std_logic_vector(9 downto 0) := (others => '0');
		SaveStateBus_wren : in  std_logic := '0';
		SaveStateBus_rst  : in  std_logic := '0';
		SaveStateBus_load : in  std_logic := '0';
		SaveStateBus_Dout : out std_logic_vector(63 downto 0);
		-- ADPCM DRAM walk (SAVETYPE_ADPCM = 9): byte addr, 2 nibbles per byte
		SS_AD_Addr        : in  std_logic_vector(15 downto 0) := (others => '0');
		SS_AD_RdEn        : in  std_logic := '0';
		SS_AD_WrEn        : in  std_logic := '0';
		SS_AD_WrData      : in  std_logic_vector(7 downto 0) := (others => '0');
		SS_AD_RdData      : out std_logic_vector(7 downto 0);
		-- CD-quiet veto for the composite boundary
		SS_CD_HOLD        : out std_logic
	);
end cd;

architecture rtl of cd is

	signal REG_SEL 			: std_logic;
	signal RAM_SEL 			: std_logic;
	signal CDRAM_DO			: std_logic_vector(7 downto 0);
	
	signal SCSI_DBI			: std_logic_vector(7 downto 0);
	signal SCSI_DBO			: std_logic_vector(7 downto 0);
	signal SCSI_ACK_N			: std_logic;
	signal SCSI_RST_N			: std_logic;
	signal SCSI_SEL_N			: std_logic;
	signal SCSI_BSY_N			: std_logic;
	signal SCSI_REQ_N			: std_logic;
	signal SCSI_DATAIN_HELD	: std_logic;	-- P5b: hide the staged DATAIN byte from the CPU
	signal SCSI_MSG_N			: std_logic;
	signal SCSI_CD_N			: std_logic;
	signal SCSI_IO_N			: std_logic;
	signal SCSI_COMM_SEND_i	: std_logic;	-- internal tap: Quartus 17 cannot read out ports
	signal CD_STOP_CD_SND	: std_logic;

	-- Forensic eReg 68 (save-only): post-load CD event timeline in seconds.
	-- RESET_N pulses on a savestate load (proven by the zeroed eReg 66
	-- ledger), so second 0 = the load itself.
	signal DBG_SECDIV  : unsigned(25 downto 0) := (others => '0');
	signal DBG_SEC     : unsigned(15 downto 0) := (others => '0');
	signal DBG_T_STAT  : unsigned(7 downto 0) := (others => '0');
	signal DBG_T_COMM  : unsigned(7 downto 0) := (others => '0');
	signal DBG_T_CDDA  : unsigned(7 downto 0) := (others => '0');
	signal DBG_T_LOAD  : unsigned(7 downto 0) := (others => '0');
	signal DBG_RSTCNT  : unsigned(7 downto 0) := (others => '0');
	signal DBG_STATCNT : unsigned(7 downto 0) := (others => '0');
	signal DBG_SG_D    : std_logic := '0';
	signal DBG_CS_D    : std_logic := '0';
	signal DBG_RN_D    : std_logic := '1';
	signal SS_V_Dout_DBG68 : std_logic_vector(63 downto 0);
	
	signal CD_DTD				: std_logic;	--CD data transfer done flag
	signal CD_DTR				: std_logic;	--CD data transfer ready flag
	signal CD_SUBCD			: std_logic;	--CD Subcode data ready flag
	signal CD_DTD_EN			: std_logic;
	signal CD_DTR_EN			: std_logic;
--	signal CD_MOTOR			: std_logic;
	signal CD_SUBCD_EN		: std_logic;
	signal ADPCM_END_EN		: std_logic;
	signal ADPCM_HALF_EN		: std_logic;
	signal CH_SEL				: std_logic;
	signal BRAM_LOCK			: std_logic;
	
	signal AUTO_ACK			: std_logic;
	signal TR_DONE_OLD		: std_logic;
	signal TR_RDY_OLD			: std_logic;
	signal SCSI_REQ_N_OLD	: std_logic;
	signal SCSI_BSY_N_OLD	: std_logic;
	signal SCSI_ACK_N_OLD	: std_logic;
	signal CD_DATA_CNT		: unsigned(10 downto 0);
	
	signal R1802_0				: std_logic;
	signal R1802_1				: std_logic;
	signal R180E_7_4			: std_logic_vector(3 downto 0);
	signal R180F_0				: std_logic;
	signal R180F_7_4			: std_logic_vector(3 downto 0);
	
	signal CDDA_VOL			: std_logic_vector(15 downto 0);
	
	--ADPCM controller
	signal ADPCM_OFFS			: std_logic_vector(15 downto 0);
	signal ADPCM_LEN			: std_logic_vector(16 downto 0);
	signal ADPCM_RDADDR		: std_logic_vector(16 downto 0);
	signal ADPCM_WRADDR		: std_logic_vector(16 downto 0);
	signal ADPCM_CTRL			: std_logic_vector(7 downto 0);
	signal ADPCM_DMA_EN		: std_logic;
	signal ADPCM_DMA_RUN		: std_logic;
	signal ADPCM_END			: std_logic;							--ADPCM end reached flag
	signal ADPCM_HALF			: std_logic;							--ADPCM half reached flag
	signal ADPCM_PLAY			: std_logic;
	signal ADPCM_FREQ			: std_logic_vector(3 downto 0);
	signal ADPCM_FADER		: std_logic_vector(2 downto 0) := (others => '0');
	signal ADPCM_RDDATA		: std_logic_vector(7 downto 0);
	signal ADPCM_WRDATA		: std_logic_vector(7 downto 0);
	signal ADPCM_WRITE_PEND	: std_logic;
	signal ADPCM_READ_PEND	: std_logic;
	signal PLAY_READ_PEND	: std_logic;
	signal DMA_WRITE_PEND	: std_logic := '0';
	signal DMA_STAGED			: std_logic := '0';	-- P5d: staged DATAIN byte was the DMA's
	signal ADPCM_WRITE_NIB	: std_logic;
	signal ADPCM_READ_NIB	: std_logic;
	signal WRITE_PEND			: std_logic;
	signal READ_PEND			: std_logic;
	
	--ADPCM DRAM
	signal ADRAM_A				: std_logic_vector(16 downto 0);
	signal ADRAM_DI			: std_logic_vector(3 downto 0);
	signal ADRAM_DO			: std_logic_vector(3 downto 0);
	signal ADRAM_WE			: std_logic;
	
	type DRAMSlot_t is (
		SLOT_REFRESH,
		SLOT_READ,
		SLOT_WRITE
	);
	signal DRAM_SLOT 			: DRAMSlot_t; 
	signal DRAM_CLK_CNT		: unsigned(4 downto 0);
	signal DRAM_CLKEN			: std_logic;
	signal DRAM_SLOT_CNT		: unsigned(1 downto 0);
	
	--ADPCM decoder
	signal M5205_D				: std_logic_vector(3 downto 0);
	signal M5205_CLK			: std_logic;
	signal M5205_VCK_R		: std_logic;
	signal M5205_VCK_F		: std_logic;
	signal M5205_SOUT			: signed(15 downto 0);
	signal M5205_CLK_CNT 	: integer range 0 to 511;
	type M5205ClockTable_t is array (0 to 15) of integer range 0 to 511;
	constant ACT : M5205ClockTable_t :=
	(443,	--96712Hz
	 415,	--103199Hz
	 387,	--110619Hz
	 360,	--119048Hz
	 332,	--129032Hz
	 304,	--140647Hz
	 276,	--154799Hz
	 249,	--171821Hz
	 221,	--193424Hz
	 193,	--221239Hz
	 166,	--257732Hz
	 138,	--309598Hz
	 110,	--386100Hz
	 82,	--515464Hz
	 55,	--769231Hz
	 27	--1538462Hz
	);
	
	--CDDA
	signal CD_WR_OLD 			: std_logic;
	signal CD_BYTE_CNT		: unsigned(1 downto 0);
	signal FIFO_FULL 			: std_logic;
	signal FIFO_EMPTY 		: std_logic;
	signal FIFO_RD_REQ		: std_logic;
	signal FIFO_WR_REQ		: std_logic;
	signal FIFO_D 				: std_logic_vector(31 downto 0);
	signal FIFO_Q 				: std_logic_vector(31 downto 0);
	signal FIFO_SCLR			: std_logic;
	signal CDDA_CE 			: std_logic;
	signal ADPCM_CE         : std_logic;
	signal CDDA_SAMPLE		: std_logic;
	signal CDDA_SAMPLE_OLD	: std_logic;
	signal OUTL 				: signed(15 downto 0);
	signal OUTR 				: signed(15 downto 0);
	
	--CD SUBCODE
	signal SUBCD_WR_OLD 			: std_logic;
	signal SUBCD_FIFO_FULL 		: std_logic;
	signal SUBCD_FIFO_EMPTY 	: std_logic;
	signal SUBCD_FIFO_RD_REQ	: std_logic;
	signal SUBCD_FIFO_WR_REQ	: std_logic;
	signal SUBCD_FIFO_D 			: std_logic_vector(7 downto 0);
	signal SUBCD_FIFO_Q 			: std_logic_vector(7 downto 0);
	signal SUBCD_FIFO_SCLR		: std_logic;
	signal SUBCD_CE			: std_logic;
	signal SUBCD_CE_OLD		: std_logic;
	signal SUBCD_CNT			: unsigned(3 downto 0);
	signal SUBCD_BYTE			: std_logic_vector(7 downto 0);
	signal SUBCD_BYTENUM		: unsigned(7 downto 0);
	
	--Fader
	signal FADE_VOL 			: unsigned(10 downto 0);
	signal FADE_CNT 			: unsigned(7 downto 0);
	signal CDDA_FADE_VOL		: unsigned(10 downto 0);
	signal ADPCM_FADE_VOL	: unsigned(10 downto 0);

	--Savestates
	type slv64_array6 is array (0 to 5) of std_logic_vector(63 downto 0);
	signal SS_V      : slv64_array6;	-- 0=CD_1 1=CD_2 2=CD_3 3=CD_4 4=CD_5 5=CD_6
	signal SS_V_BACK : slv64_array6 := (others => (others => '0'));
	signal SS_V_Dout : slv64_array6;
	signal SS_Dout_SCSI : std_logic_vector(63 downto 0);
	signal SCSI_SS_QUIET : std_logic;
	signal M5205_SS_DOUT : std_logic_vector(38 downto 0);
	--P5c CDDA replay
	signal CDDA_CONSUMED : unsigned(31 downto 0);
	signal AUDIO_CMD_SET : std_logic;
	signal REPLAY_SKIP   : unsigned(9 downto 0);
	signal REPLAY_START  : std_logic;
	signal SKIP_LEFT     : unsigned(9 downto 0) := (others => '0');
	signal SKIP_GAP      : std_logic := '0';	-- idle cycle after a skip read (FIFO_EMPTY is registered)
	-- ADPCM walk shuttle: two nibble accesses per Save_RAM byte window
	signal SS_AD_NIB   : std_logic := '0';
	signal SS_AD_PH    : std_logic := '0';
	signal SS_AD_LO    : std_logic_vector(3 downto 0);
	signal SS_AD_PREV  : std_logic_vector(15 downto 0);

begin

	REG_SEL <= '1' when EXT_A(20 downto 13) = x"FF" and EXT_A(12 downto 8) = "11000" else '0';

	process( CLK, RST_N ) 
	variable NEW_ADPCM_LEN : std_logic_vector(16 downto 0);
	begin
		if RST_N = '0' then
			SCSI_DBI <= (others => '0');
			SCSI_ACK_N <= '1';
			SCSI_RST_N <= '1';
			SCSI_SEL_N <= '1';
			CD_DTD <= '0';
			CD_DTR <= '0';
			CD_SUBCD <= '0';
			CH_SEL <= '0';
			BRAM_LOCK <= '1';
			CD_DTD_EN <= '0';
			CD_DTR_EN <= '0';
--			CD_MOTOR <= '0';
			CD_SUBCD_EN <= '0';
			ADPCM_END_EN <= '0';
			ADPCM_HALF_EN <= '0';
			AUTO_ACK <= '0';
			
			CDDA_VOL <= (others => '0');
			
			SUBCD_CE_OLD <= '0';
			ADPCM_OFFS <= (others => '0');
			ADPCM_LEN <= (others => '0');
			ADPCM_RDADDR <= (others => '0');
			ADPCM_WRADDR <= (others => '0');
			ADPCM_CTRL <= (others => '0');
			ADPCM_DMA_EN <= '0';
			ADPCM_DMA_RUN <= '0';
			DMA_STAGED <= '0';
			ADPCM_END <= '0';
			ADPCM_HALF <= '0';
			ADPCM_PLAY <= '0';
			ADPCM_FREQ <= (others => '0');
			ADPCM_WRDATA <= (others => '0');
			ADPCM_RDDATA <= (others => '0');
			ADPCM_WRITE_PEND <= '0';
			ADPCM_READ_PEND <= '0';
			PLAY_READ_PEND <= '0';
			ADPCM_WRITE_NIB <= '0';
			ADPCM_READ_NIB <= '0';
			M5205_D <= (others => '0');
		elsif rising_edge( CLK ) then
			if SaveStateBus_load = '1' then
				-- P5d: tell the replay which consumer owns the staged DATAIN byte.
				--
				-- The ADPCM DMA takes a DATAIN byte the moment REQ rises (it copies
				-- SCSI_DBO into ADPCM_WRDATA/DMA_WRITE_PEND above), writes it into
				-- ADPCM RAM, and only THEN starts the ACK round-trip; READ_CONSUMED
				-- on the SCSI side counts it when that handshake completes.  A save
				-- landing in between leaves the byte already stored but uncounted.
				-- The CPU's boundary is the opposite: its staged byte is still owed
				-- to it until it reads $1808.  The replay has to tell the two apart.
				--
				-- It must be computed HERE, from the blob, and not sampled when
				-- RP_RD_PREP runs: by then AUTO_ACK has long released and ACK_N is
				-- back high, so the evidence is gone.  Measured on the failing case:
				-- DMA_WRITE_PEND='0', AUTO_ACK='1', ACK_N='0'.
				DMA_STAGED <= (SS_V(1)(55) or SS_V(1)(56))          -- DMA_EN / DMA_RUN
				              and (SS_V(1)(63)                       -- DMA_WRITE_PEND
				                   or SS_V(0)(21)                    -- AUTO_ACK
				                   or not SS_V(0)(8));               -- ACK asserted
				-- Savestate restore (registers driven by this process)
				SCSI_DBI      <= SS_V(0)(7 downto 0);
				SCSI_ACK_N    <= SS_V(0)(8);
				SCSI_RST_N    <= SS_V(0)(9);
				SCSI_SEL_N    <= SS_V(0)(10);
				CD_DTD        <= SS_V(0)(11);
				CD_DTR        <= SS_V(0)(12);
				CD_SUBCD      <= SS_V(0)(13);
				CH_SEL        <= SS_V(0)(14);
				BRAM_LOCK     <= SS_V(0)(15);
				CD_DTD_EN     <= SS_V(0)(16);
				CD_DTR_EN     <= SS_V(0)(17);
				CD_SUBCD_EN   <= SS_V(0)(18);
				ADPCM_END_EN  <= SS_V(0)(19);
				ADPCM_HALF_EN <= SS_V(0)(20);
				AUTO_ACK      <= SS_V(0)(21);
				R1802_0       <= SS_V(0)(22);
				R1802_1       <= SS_V(0)(23);
				R180E_7_4     <= SS_V(0)(27 downto 24);
				R180F_0       <= SS_V(0)(28);
				R180F_7_4     <= SS_V(0)(32 downto 29);
				CD_DATA_CNT   <= unsigned(SS_V(0)(43 downto 33));
				SCSI_REQ_N_OLD <= SS_V(0)(44);
				SCSI_BSY_N_OLD <= SS_V(0)(45);
				SCSI_ACK_N_OLD <= SS_V(0)(46);
				CDDA_VOL      <= SS_V(0)(63 downto 48);
				ADPCM_OFFS    <= SS_V(1)(15 downto 0);
				ADPCM_LEN     <= SS_V(1)(32 downto 16);
				ADPCM_CTRL    <= SS_V(1)(47 downto 40);
				ADPCM_FREQ    <= SS_V(1)(51 downto 48);
				ADPCM_FADER   <= SS_V(1)(54 downto 52);
				ADPCM_DMA_EN  <= SS_V(1)(55);
				ADPCM_DMA_RUN <= SS_V(1)(56);
				ADPCM_END     <= SS_V(1)(57);
				ADPCM_HALF    <= SS_V(1)(58);
				ADPCM_PLAY    <= SS_V(1)(59);
				ADPCM_WRITE_PEND <= SS_V(1)(60);
				ADPCM_READ_PEND  <= SS_V(1)(61);
				PLAY_READ_PEND   <= SS_V(1)(62);
				DMA_WRITE_PEND   <= SS_V(1)(63);
				ADPCM_RDADDR  <= SS_V(2)(16 downto 0);
				ADPCM_WRADDR  <= SS_V(2)(36 downto 20);
				ADPCM_RDDATA  <= SS_V(2)(47 downto 40);
				ADPCM_WRDATA  <= SS_V(2)(55 downto 48);
				ADPCM_WRITE_NIB <= SS_V(2)(56);
				ADPCM_READ_NIB  <= SS_V(2)(57);
				M5205_D       <= SS_V(2)(61 downto 58);
				CDDA_SAMPLE_OLD <= SS_V(4)(11);
			elsif EN = '1' and SLEEP = '0' then
			if CPU_CE = '1' then
				SCSI_SEL_N <= '1';
				if REG_SEL = '1' and EXT_WR_N = '0' then
					case EXT_A(7 downto 0) is
						when x"00" =>
							SCSI_SEL_N <= '0';
							CD_DTD <= '0';
							CD_DTR <= '0';
--							if SCSI_DBI = x"00" then
--								CD_MOTOR <= '1';
--							end if;
						when x"01" =>
							SCSI_DBI <= EXT_DI;
						when x"02" =>
							SCSI_ACK_N <= not EXT_DI(7);
							CD_DTR_EN <= EXT_DI(6);
							CD_DTD_EN <= EXT_DI(5);
							CD_SUBCD_EN <= EXT_DI(4);
							ADPCM_END_EN <= EXT_DI(3);
							ADPCM_HALF_EN <= EXT_DI(2);
							R1802_1 <= EXT_DI(1);
							R1802_0 <= EXT_DI(0);
						when x"04" =>
							SCSI_RST_N <= not EXT_DI(1);
							if EXT_DI(1) = '1' then
								CD_DTD <= '0';
								CD_DTR <= '0';
							end if;
						when x"05" =>
							if CDDA_SAMPLE = '1' and CDDA_SAMPLE_OLD = '0' then
								if CH_SEL = '0' then
									CDDA_VOL <= std_logic_vector(OUTL);   -- CDDA_VOL is returned by a different process in *the same cycle*, so we assign the opposite of what documentation says
								else
									CDDA_VOL <= std_logic_vector(OUTR);
								end if;
								CH_SEL <= not CH_SEL;
							end if;
							CDDA_SAMPLE_OLD <= CDDA_SAMPLE;
							
						when x"07" =>
							BRAM_LOCK <= not EXT_DI(7);  -- unlock when bit 7 is '1', but also lock if bit 7 is '0'
						when x"08" =>
							ADPCM_OFFS(7 downto 0) <= EXT_DI;
						when x"09" =>
							ADPCM_OFFS(15 downto 8) <= EXT_DI;
						when x"0A" =>
							ADPCM_WRDATA <= EXT_DI;
							ADPCM_WRITE_NIB <= '0';
							ADPCM_WRITE_PEND <= '1';
						when x"0B" =>
							ADPCM_DMA_EN <= EXT_DI(1);
							ADPCM_DMA_RUN <= EXT_DI(0);
						when x"0D" =>
							ADPCM_CTRL <= EXT_DI;
							if EXT_DI(5) = '1' and ADPCM_CTRL(5) = '0' then
								ADPCM_PLAY <= '1';
								ADPCM_HALF <= '0';
							elsif EXT_DI(5) = '0' and ADPCM_CTRL(5) = '1' then
								ADPCM_PLAY <= '0';
							end if;
						when x"0E" =>
							R180E_7_4 <= EXT_DI(7 downto 4);
							ADPCM_FREQ <= EXT_DI(3 downto 0);
						when x"0F" =>
							R180F_7_4 <= EXT_DI(7 downto 4);
							ADPCM_FADER <= EXT_DI(3 downto 1);
							R180F_0 <= EXT_DI(0);
						when others => null;
					end case;
				elsif REG_SEL = '1' and EXT_RD_N = '0' then
					case EXT_A(7 downto 0) is
						when x"03" =>
							BRAM_LOCK <= '1';
						when x"07" =>
							CD_SUBCD <= '0';
						when x"08" =>
							if SCSI_REQ_N = '0' and SCSI_IO_N = '0' and SCSI_CD_N = '1' and SCSI_ACK_N = '1' then 
								SCSI_ACK_N <= '0';
								AUTO_ACK <= '1';
							end if;
						when x"0A" =>
							ADPCM_READ_NIB <= '0';
							ADPCM_READ_PEND <= '1';
						when others => null;
					end case;
				end if;
			end if;
			
			if AUTO_ACK = '1' and SCSI_REQ_N = '1' then
				SCSI_ACK_N <= '1';
				AUTO_ACK <= '0';
			end if;
			
			if (ADPCM_DMA_EN = '1' or ADPCM_DMA_RUN = '1') and DMA_WRITE_PEND = '0' and SCSI_DATAIN_HELD = '0' then
				-- P5b: same hold - a DMA fired against the staged byte would
				-- shovel that one byte into ADPCM RAM over and over
				if SCSI_REQ_N = '0' and SCSI_IO_N = '0' and SCSI_CD_N = '1' and SCSI_ACK_N = '1' then
					ADPCM_WRDATA <= SCSI_DBO;
					DMA_WRITE_PEND <= '1';
				end if;
			end if;
			
			SCSI_REQ_N_OLD <= SCSI_REQ_N;
			SCSI_BSY_N_OLD <= SCSI_BSY_N;
			if CD_DTD = '0' and SCSI_REQ_N = '0' and SCSI_REQ_N_OLD = '1' and SCSI_BSY_N = '0' and SCSI_IO_N = '0' and SCSI_CD_N = '0' then
				CD_DTD <= '1';
			elsif CD_DTD = '1' and ((SCSI_REQ_N = '1' and SCSI_REQ_N_OLD = '0' and SCSI_MSG_N = '0') or (SCSI_BSY_N = '1' and SCSI_BSY_N_OLD = '0')) then
				CD_DTD <= '0';
			end if;
			
			SCSI_ACK_N_OLD <= SCSI_ACK_N;
			if CD_DTR = '0' and SCSI_REQ_N = '0' and SCSI_BSY_N = '0' and SCSI_CD_N = '1' and SCSI_MSG_N = '1' and SCSI_IO_N = '0' then
				CD_DTR <= '1';
				CD_DATA_CNT <= (others => '0');
			elsif CD_DTR = '1' and SCSI_ACK_N = '1' and SCSI_ACK_N_OLD = '0' and SCSI_BSY_N = '0' and SCSI_CD_N = '1' and SCSI_MSG_N = '1' and SCSI_IO_N = '0' then
				CD_DATA_CNT <= CD_DATA_CNT + 1;
				if CD_DATA_CNT = 2047 then
					CD_DTR <= '0';
					ADPCM_DMA_RUN <= '0';
				end if;
			elsif CD_DTR = '1' and (SCSI_BSY_N = '1' and SCSI_BSY_N_OLD = '0') then
				CD_DTR <= '0';
			end if;

			SUBCD_CE_OLD <= SUBCD_CE;
			if CD_SUBCD = '0' and SUBCD_CE = '1' and SUBCD_CE_OLD = '0' then
				CD_SUBCD <= '1';
			end if;
			
			if M5205_VCK_R = '1' and ADPCM_PLAY = '1' then
				PLAY_READ_PEND <= '1';
			end if;
			
			if DRAM_CLKEN = '1' and ADPCM_CTRL(7) = '0' then
				case DRAM_SLOT is
					when SLOT_READ =>
						NEW_ADPCM_LEN := std_logic_vector(unsigned(ADPCM_LEN) - 1);
						if ADPCM_READ_PEND = '1' or PLAY_READ_PEND = '1' then
							M5205_D <= ADRAM_DO;
							if PLAY_READ_PEND = '1' then
								PLAY_READ_PEND <= '0';
							end if;
							
							ADPCM_READ_NIB <= not ADPCM_READ_NIB;
							if ADPCM_READ_NIB = '0' then
								ADPCM_RDDATA(7 downto 4) <= ADRAM_DO;
							else
								ADPCM_RDDATA(3 downto 0) <= ADRAM_DO;
							end if;
							if ADPCM_READ_NIB = '1' then
								if ADPCM_LEN /= "0"&x"0000" then
									ADPCM_LEN <= NEW_ADPCM_LEN;
								end if;
								if ADPCM_READ_PEND = '1' then
									ADPCM_READ_PEND <= '0';
								end if;
							end if;
							if ADPCM_LEN < "0"&x"8000" then
								ADPCM_HALF <= '1';
							elsif ADPCM_LEN = "0"&x"8000" and ADPCM_CTRL(4) = '0' then
								ADPCM_HALF <= '1';
							else
								ADPCM_HALF <= '0';
							end if;
							if ADPCM_LEN = "0"&x"0000" then
								ADPCM_END <= '1';
								if ADPCM_READ_PEND = '1' and ADPCM_CTRL(4) = '0' then
									ADPCM_HALF <= '0';
								end if;
								if ADPCM_CTRL(6) = '1' and ADPCM_PLAY = '1' then
									ADPCM_PLAY <= '0';
									M5205_D <= (others => '0');
									ADPCM_CTRL(5) <= '0';
								end if;
							end if;
						end if;
						
					when SLOT_WRITE =>
						NEW_ADPCM_LEN := std_logic_vector(unsigned(ADPCM_LEN) + 1);
						if ADPCM_WRITE_PEND = '1' or DMA_WRITE_PEND = '1' then
							ADPCM_WRITE_NIB <= not ADPCM_WRITE_NIB;
							if ADPCM_WRITE_NIB = '1' then
								ADPCM_LEN <= NEW_ADPCM_LEN;
								if ADPCM_WRITE_PEND = '1' then
									ADPCM_WRITE_PEND <= '0';
								end if;
								if DMA_WRITE_PEND = '1' then
									DMA_WRITE_PEND <= '0';
									SCSI_ACK_N <= '0';
									AUTO_ACK <= '1';
								end if;
							end if;
							ADPCM_HALF <= not ADPCM_LEN(15) and not ADPCM_LEN(16);
							if ADPCM_LEN = "0"&x"0000" then
								ADPCM_END <= '1';
							end if;
						end if;
						
					when others => null;
				end case;
			end if;
			
			if ADPCM_CTRL(4) = '1' then
				ADPCM_LEN <= "0"&ADPCM_OFFS;
				ADPCM_END <= '0';
			end if;
			
			if ADPCM_CTRL(7) = '1' then
				ADPCM_OFFS <= (others => '0');
				ADPCM_LEN <= (others => '0');
				ADPCM_WRADDR <= (others => '0');
				ADPCM_RDADDR <= (others => '0');
				ADPCM_END <= '0';
				ADPCM_HALF <= '0';
			end if;
			
			if DRAM_CLKEN = '1' and DRAM_SLOT = SLOT_WRITE and (ADPCM_WRITE_PEND = '1' or DMA_WRITE_PEND = '1') then
				if ADPCM_CTRL(1) = '1' then
					ADPCM_WRADDR <= ADPCM_OFFS & "0";
				else
					ADPCM_WRADDR <= std_logic_vector(unsigned(ADPCM_WRADDR) + 1);
				end if;
			elsif CPU_CE = '1' and REG_SEL = '1' and EXT_WR_N = '0' and EXT_A(7 downto 0) = x"0D" and EXT_DI(0) = '0' and ADPCM_CTRL(0) = '1' then
				if ADPCM_CTRL(1) = '1' then
					ADPCM_WRADDR <= ADPCM_OFFS & "0";
				else
					ADPCM_WRADDR <= std_logic_vector(unsigned(ADPCM_WRADDR) + 1);
				end if;
			end if;
			
			if DRAM_CLKEN = '1' and DRAM_SLOT = SLOT_READ and (ADPCM_READ_PEND = '1' or PLAY_READ_PEND = '1') then
				if ADPCM_CTRL(3) = '1' then
					ADPCM_RDADDR <= ADPCM_OFFS & "0";
				else
					ADPCM_RDADDR <= std_logic_vector(unsigned(ADPCM_RDADDR) + 1);
				end if;
			elsif CPU_CE = '1' and REG_SEL = '1' and EXT_WR_N = '0' and EXT_A(7 downto 0) = x"0D" and EXT_DI(2) = '0' and ADPCM_CTRL(2) = '1' then
				if ADPCM_CTRL(3) = '1' then
					ADPCM_RDADDR <= ADPCM_OFFS & "0";
				else
					ADPCM_RDADDR <= std_logic_vector(unsigned(ADPCM_RDADDR) + 1);
				end if;
			end if;
			end if;
		end if;
	end process;

	WRITE_PEND <= ADPCM_WRITE_PEND or DMA_WRITE_PEND;
	READ_PEND <= ADPCM_READ_PEND or PLAY_READ_PEND;
	process( REG_SEL, EXT_A, SCSI_DBO, SCSI_DBI, SCSI_BSY_N, SCSI_REQ_N, SCSI_DATAIN_HELD, SCSI_MSG_N, SCSI_CD_N, SCSI_IO_N, SCSI_ACK_N, SCSI_RST_N,
				CD_DTR, CD_DTD, CD_SUBCD, SUBCD_BYTE, CD_DTR_EN, CD_DTD_EN, CD_SUBCD_EN, R1802_0, R1802_1, CH_SEL, ADPCM_RDDATA, ADPCM_DMA_EN, ADPCM_DMA_RUN, ADPCM_END, ADPCM_HALF, ADPCM_END_EN, ADPCM_HALF_EN, 
				ADPCM_CTRL, ADPCM_FREQ, R180E_7_4, ADPCM_PLAY, ADPCM_FADER, R180F_0, R180F_7_4, READ_PEND, WRITE_PEND, CDDA_VOL, CD_REGION) 
	begin
		EXT_DO <= x"00";
		if REG_SEL = '1' then
			case EXT_A(7 downto 0) is
				when x"00" =>
					-- P5b: while the SCSI FSM is held after a load (replay in
					-- flight / stream drain) the staged DATAIN byte must not be
					-- advertised, or the program re-reads it in a tight loop and
					-- its stream position runs away from the drive's.
					EXT_DO <= not SCSI_BSY_N & (not SCSI_REQ_N and not SCSI_DATAIN_HELD)
					          & not SCSI_MSG_N & not SCSI_CD_N & not SCSI_IO_N & "000";
				when x"01" =>
					if SCSI_BSY_N = '0' then
						EXT_DO <= SCSI_DBO;
					else
						EXT_DO <= SCSI_DBI;
					end if;
				when x"02" =>
					EXT_DO <= not SCSI_ACK_N & CD_DTR_EN & CD_DTD_EN & CD_SUBCD_EN & ADPCM_END_EN & ADPCM_HALF_EN & R1802_1 & R1802_0;
				when x"03" =>
					-- CD_DTR ("a data byte is ready") is hidden for the same
					-- reason: games that poll $1803 instead of $1800 would
					-- otherwise chew through the one staged byte just as fast.
					EXT_DO <= "0" & (CD_DTR and not SCSI_DATAIN_HELD) & CD_DTD & CD_SUBCD & ADPCM_END & ADPCM_HALF & CH_SEL & "0";--TODO
				when x"04" =>
					EXT_DO <= "000000" & not SCSI_RST_N & "0";
				when x"05" =>
					EXT_DO <= CDDA_VOL(7 downto 0);
				when x"06" =>
					EXT_DO <= CDDA_VOL(15 downto 8);
				when x"07" =>
					EXT_DO <= SUBCD_BYTE;
				when x"08" =>
					if SCSI_BSY_N = '0' then
						EXT_DO <= SCSI_DBO;
					else
						EXT_DO <= SCSI_DBI;
					end if;
					
				when x"0A" =>
					EXT_DO <= ADPCM_RDDATA;
				when x"0B" =>
					EXT_DO <= "000000" & ADPCM_DMA_EN & ADPCM_DMA_RUN;
				when x"0C" =>
					EXT_DO <= READ_PEND & "000" & ADPCM_PLAY & WRITE_PEND & "0" & ADPCM_END;
				when x"0D" =>
					EXT_DO <= ADPCM_CTRL;
				when x"0E" =>
					EXT_DO <= R180E_7_4 & ADPCM_FREQ;
				when x"0F" =>
					EXT_DO <= R180F_7_4 & ADPCM_FADER & R180F_0;
					
				when x"C1" =>
					EXT_DO <= x"AA";
				when x"C2" =>
					EXT_DO <= x"55";
				when x"C3" =>
					EXT_DO <= x"00";

				when x"C5" =>
					if CD_REGION = '1' then
						EXT_DO <= x"55";
					else
						EXT_DO <= x"AA";
					end if;
				when x"C6" =>
					if CD_REGION = '1' then
						EXT_DO <= x"AA";
					else
						EXT_DO <= x"55";
					end if;
				when x"C7" =>
					if CD_REGION = '1' then
						EXT_DO <= x"C0";
					else
						EXT_DO <= x"03";
					end if;
				when others => null;
			end case;
		end if;
	end process;
	
	SEL_N <= not (REG_SEL and EN);
	-- P5b: the DTR interrupt is masked along with the DTR status bit, otherwise
	-- a held DATAIN byte keeps re-triggering the game's data-transfer ISR.
	IRQ_N <= not ((CD_DTR_EN and CD_DTR and not SCSI_DATAIN_HELD) or (CD_DTD_EN and CD_DTD) or (CD_SUBCD_EN and CD_SUBCD) or (ADPCM_END_EN and ADPCM_END) or (ADPCM_HALF_EN and ADPCM_HALF));
	
	RAM_SEL <= '1' when EXT_A(20 downto 13) >= x"68" and EXT_A(20 downto 13) <= x"87" else '0';
	RAM_CS_N <= not (RAM_SEL and EN);
	
	BRAM_EN <= not BRAM_LOCK or not EN;
	CD_RESET <= not SCSI_RST_N;
	
	SCSI : entity work.SCSI
	port map (
		-- Same trap as the MSM5205 reset below: SCSI_RST_N is RESTORED state and
		-- SCSI resets asynchronously, so a load taken while the game holds the CD
		-- reset asserted would throw the whole restored SCSI state away.  Hold the
		-- block out of reset for the savestate walk; afterwards the restored
		-- SCSI_RST_N governs again.  The global RST_N is deliberately still able
		-- to reset it at any time.
		RESET_N		=> RST_N and (SCSI_RST_N or SLEEP),
		CLK			=> CLK,
		
		DBI			=> SCSI_DBI,
		DBO			=> SCSI_DBO,
		SEL_N			=> SCSI_SEL_N,
		ACK_N			=> SCSI_ACK_N,
		RST_N			=> '1', --SCSI_RST_N,
		BSY_N			=> SCSI_BSY_N,
		REQ_N			=> SCSI_REQ_N,
		MSG_N			=> SCSI_MSG_N,
		CD_N			=> SCSI_CD_N,
		IO_N			=> SCSI_IO_N,
		
		STATUS		=> CD_STAT,
		MESSAGE		=> CD_MSG,
		STAT_GET		=> CD_STAT_GET,
		
		COMMAND		=> CD_COMM,
		COMM_SEND	=> SCSI_COMM_SEND_i,
		
		DOUT_REQ		=> CD_DOUT_REQ,
		DOUT			=> CD_DOUT,
		DOUT_SEND	=> CD_DOUT_SEND,
		STOP_CD_SND	=> CD_STOP_CD_SND,
		
		CD_DATA		=> CD_DATA,
		CD_WR			=> CD_DATA_WR,
		CD_DATA_END	=> CD_DATA_END,
		DATAIN_HELD	=> SCSI_DATAIN_HELD,

		SLEEP				=> SLEEP,
		SS_PEND			=> SS_PEND,
		SaveStateBus_Din  => SaveStateBus_Din,
		SaveStateBus_Adr  => SaveStateBus_Adr,
		SaveStateBus_wren => SaveStateBus_wren,
		SaveStateBus_rst  => SaveStateBus_rst,
		SaveStateBus_load => SaveStateBus_load,
		SaveStateBus_Dout => SS_Dout_SCSI,
		SS_QUIET				=> SCSI_SS_QUIET,
		CDDA_CONSUMED	=> CDDA_CONSUMED,
		AUDIO_CMD_SET	=> AUDIO_CMD_SET,
		REPLAY_SKIP		=> REPLAY_SKIP,
		DMA_STAGED		=> DMA_STAGED,
		REPLAY_START	=> REPLAY_START
	);

	CD_COMM_SEND <= SCSI_COMM_SEND_i;

	-- Forensic eReg 68: second-resolution timeline of the CD interface.
	-- Answers, from one corpse dump, the questions the HPS side cannot be
	-- asked (release Main has no console): did a status arrive after the
	-- load and WHEN; when was the last replay/game command; did the game
	-- reset the drive; until when did the HPS stream CDDA samples.
	process( CLK )
	begin
		if rising_edge(CLK) then
			if RST_N = '0' then
				DBG_SECDIV <= (others => '0'); DBG_SEC <= (others => '0');
				DBG_T_STAT <= (others => '0'); DBG_T_COMM <= (others => '0');
				DBG_T_CDDA <= (others => '0'); DBG_T_LOAD <= (others => '0');
				DBG_RSTCNT <= (others => '0'); DBG_STATCNT <= (others => '0');
			else
				DBG_SECDIV <= DBG_SECDIV + 1;
				if DBG_SECDIV = 42954544 then
					DBG_SECDIV <= (others => '0');
					if DBG_SEC /= x"FFFF" then
						DBG_SEC <= DBG_SEC + 1;
					end if;
				end if;
				DBG_SG_D <= CD_STAT_GET;
				if CD_STAT_GET = '1' and DBG_SG_D = '0' then
					DBG_T_STAT <= DBG_SEC(7 downto 0);
					if DBG_STATCNT /= x"FF" then
						DBG_STATCNT <= DBG_STATCNT + 1;
					end if;
				end if;
				DBG_CS_D <= SCSI_COMM_SEND_i;
				if SCSI_COMM_SEND_i = '1' and DBG_CS_D = '0' then
					DBG_T_COMM <= DBG_SEC(7 downto 0);
				end if;
				DBG_RN_D <= SCSI_RST_N;
				if SCSI_RST_N = '0' and DBG_RN_D = '1' then
					if DBG_RSTCNT /= x"FF" then
						DBG_RSTCNT <= DBG_RSTCNT + 1;
					end if;
				end if;
				if CD_AUDIO_WR = '1' then
					DBG_T_CDDA <= DBG_SEC(7 downto 0);
				end if;
				if SaveStateBus_load = '1' then
					DBG_T_LOAD <= DBG_SEC(7 downto 0);
				end if;
			end if;
		end if;
	end process;

	iSS_CD_DBG68 : entity work.eReg_SavestateV
	generic map ( Adr => 68, def => SSREG_DEFAULT_CD )
	port map ( CLK, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren,
	           SaveStateBus_rst, SS_V_Dout_DBG68,
	           std_logic_vector(DBG_STATCNT) & std_logic_vector(DBG_RSTCNT)
	         & std_logic_vector(DBG_T_LOAD) & std_logic_vector(DBG_T_CDDA)
	         & std_logic_vector(DBG_T_COMM) & std_logic_vector(DBG_T_STAT)
	         & std_logic_vector(DBG_SEC), open );


	--ADPCM DRAM
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			DRAM_CLK_CNT <= (others => '0');
			DRAM_CLKEN <= '0';
			DRAM_SLOT_CNT <= (others => '0');
		elsif rising_edge(CLK) then
			if EN = '1' and SLEEP = '0' then
				DRAM_CLKEN <= '0';
				DRAM_CLK_CNT <= DRAM_CLK_CNT + 1;
				if DRAM_CLK_CNT = 18-1 then
					DRAM_CLK_CNT <= (others => '0');
					DRAM_CLKEN <= '1';
				end if;
				
				if DRAM_CLKEN = '1' then
					DRAM_SLOT_CNT <= DRAM_SLOT_CNT + 1;
				end if;
			end if;
		end if;
	end process;
	
	process( DRAM_SLOT_CNT )
	begin
		case DRAM_SLOT_CNT is
			when "00" => DRAM_SLOT <= SLOT_REFRESH;
			when "01" => DRAM_SLOT <= SLOT_WRITE;
			when "10" => DRAM_SLOT <= SLOT_WRITE;
			when others => DRAM_SLOT <= SLOT_READ;
		end case;
	end process;
	
	ADPCM_DRAM : entity work.dpram generic map (17,4)
	port map (
		clock		=> CLK,
		address_a=> ADRAM_A,
		data_a	=> ADRAM_DI,
		wren_a	=> ADRAM_WE,
		q_a		=> ADRAM_DO
	);
	-- Savestate walk borrows the (single) DRAM port during the freeze: the
	-- slot machine is SLEEP-held.  Byte address -> two nibble accesses,
	-- sequenced by SS_AD_NIB inside the engine's 8-cycle byte window
	-- (hi nibble first: byte = ADRAM[2a] & ADRAM[2a+1]).
	ADRAM_A <= SS_AD_Addr & SS_AD_NIB when SLEEP = '1' else
	           ADPCM_WRADDR when DRAM_SLOT = SLOT_WRITE else ADPCM_RDADDR;
	ADRAM_DI <= SS_AD_WrData(7 downto 4) when SLEEP = '1' and SS_AD_NIB = '0' else
	            SS_AD_WrData(3 downto 0) when SLEEP = '1' else
	            ADPCM_WRDATA(3 downto 0) when ADPCM_WRITE_NIB = '1' else ADPCM_WRDATA(7 downto 4);
	ADRAM_WE <= SS_AD_WrEn when SLEEP = '1' else
	            DRAM_CLKEN when DRAM_SLOT = SLOT_WRITE and (ADPCM_WRITE_PEND = '1' or DMA_WRITE_PEND = '1') else '0';

	-- Walk shuttle: nibble 0 for the first half of the engine's per-byte
	-- window, then latch the registered read data and switch to nibble 1.
	-- The ADPCM DRAM (dpram, altsyncram) has a 1-CYCLE registered read: after
	-- SS_AD_NIB drops to 0 the address (2*addr) is only presented THIS cycle,
	-- so ADRAM_DO does not carry the hi nibble until the NEXT cycle.  Latching
	-- SS_AD_LO immediately (the old code) captured the stale lo nibble, so the
	-- save read back (lo,lo) per byte and every loaded ADPCM sample was
	-- corrupted (the "ADPCM crackle after load" bug).  SS_AD_PH inserts the
	-- one settle cycle so SS_AD_LO latches the real hi nibble.
	process( CLK ) begin
		if rising_edge( CLK ) then
			if SLEEP = '0' then
				SS_AD_NIB  <= '0';
				SS_AD_PH   <= '0';
				SS_AD_PREV <= (others => '1');
			else
				if SS_AD_Addr /= SS_AD_PREV then
					SS_AD_PREV <= SS_AD_Addr;
					SS_AD_NIB  <= '0';
					SS_AD_PH   <= '0';		-- restart: this cycle presents 2*addr
				elsif SS_AD_NIB = '0' and (SS_AD_RdEn = '1' or SS_AD_WrEn = '1') then
					if SS_AD_PH = '0' then
						SS_AD_PH <= '1';		-- wait for the registered read (hi half)
					else
						SS_AD_LO  <= ADRAM_DO;	-- now ADRAM_DO = hi nibble at 2*addr
						SS_AD_NIB <= '1';
					end if;
				end if;
			end if;
		end if;
	end process;
	SS_AD_RdData <= SS_AD_LO & ADRAM_DO;

	
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			M5205_CLK_CNT <= 0;
			M5205_CLK <= '0';
		elsif rising_edge(CLK) then
			M5205_CLK <= '0';
			if SaveStateBus_load = '1' then
				M5205_CLK_CNT <= to_integer(unsigned(SS_V(3)(46 downto 38)));
			elsif EN = '1' and SLEEP = '0' then
				if ADPCM_CE = '1' then
					-- M5205 CLK = 42954545Hz / (ACT(n)+1)
					M5205_CLK_CNT <= M5205_CLK_CNT + 1;
					if M5205_CLK_CNT >= 15 - unsigned(ADPCM_FREQ) then
						M5205_CLK_CNT <= 0;
						M5205_CLK <= '1';
					end if;
				end if;
			end if;
		end if;
	end process;
	
	MSM5205 : entity work.MSM5205
	port map (
		-- The decoder's reset is derived from ADPCM_PLAY, which is itself restored
		-- state, and MSM5205 resets DEC_DATA/STEP/SAMPLE ASYNCHRONOUSLY - so the
		-- async reset wins over the synchronous SS_load restore.  Loading a state
		-- in which ADPCM was playing while the machine currently has PLAY=0 (the
		-- ordinary case) therefore threw the restored decoder state away and
		-- resumed from SAMPLE=0/STEP=0.  ADPCM is differential, so everything
		-- after that point is wrong.  Holding the decoder out of reset for the
		-- whole savestate walk (SLEEP) lets the restore land; once SLEEP drops,
		-- the RESTORED ADPCM_PLAY governs again, so a state that was not playing
		-- still ends up properly reset.
		RST_N		=> ((not ADPCM_CTRL(7) and ADPCM_PLAY) or SLEEP) and EN,
		CLK		=> CLK,
		
		XTI		=> M5205_CLK,
		D			=> M5205_D,
		VCK_R		=> M5205_VCK_R,
		--VCK_F		=> M5205_VCK_F,
		
		SOUT		=> M5205_SOUT,

		SLEEP		=> SLEEP,
		SS_load	=> SaveStateBus_load,
		SS_DIN	=> SS_V(3)(38 downto 0),
		SS_DOUT	=> M5205_SS_DOUT
	);

	
	--CDDA
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			CD_BYTE_CNT <= (others => '0');
			FIFO_D <= (others => '0');
			FIFO_WR_REQ <= '0';
			CD_WR_OLD <= '0';
		elsif rising_edge(CLK) then
			FIFO_WR_REQ <= '0';
			if SaveStateBus_load = '1' then
				CD_WR_OLD   <= SS_V(0)(47);
				CD_BYTE_CNT <= unsigned(SS_V(4)(9 downto 8));
			elsif EN = '1' then
				CD_WR_OLD <= CD_AUDIO_WR;
				if REPLAY_START = '1' then
					-- replayed stream starts at a frame boundary: realign the
					-- assembler (the restored/live-tail phase is meaningless)
					CD_BYTE_CNT <= (others => '0');
				elsif DM = '1' then
					CD_BYTE_CNT <= (others => '0');
				elsif CD_AUDIO_WR = '1' and CD_WR_OLD = '0' then
					CD_BYTE_CNT <= CD_BYTE_CNT + 1;
					case CD_BYTE_CNT is
						when "00" => FIFO_D(7 downto 0) <= CD_DATA;
						when "01" => FIFO_D(15 downto 8) <= CD_DATA;
						when "10" => FIFO_D(23 downto 16) <= CD_DATA;
						when others => 
							FIFO_D(31 downto 24) <= CD_DATA;
							if FIFO_FULL = '0' and DM = '0' then
								FIFO_WR_REQ <= '1';
							end if;
					end case;
				end if;
			end if;
		end if;
	end process;
	
	FIFO : entity work.CDDA_FIFO 
	port map(
		clock		=> CLK,
		data		=> FIFO_D,
		wrreq		=> FIFO_WR_REQ,
		full		=> FIFO_FULL,
		sclr		=> FIFO_SCLR,
		rdreq		=> FIFO_RD_REQ,
		empty		=> FIFO_EMPTY,
		q			=> FIFO_Q
	);
	
	CDDA_CLK_GEN : entity work.CEGen
	port map(
		CLK   		=> CLK,
		RST_N       => RST_N,		
		IN_CLK   	=> 429545,
		OUT_CLK   	=> 441,
		CE   			=> CDDA_CE
	);

	ADPCM_CLK_GEN : entity work.CEGen
	port map(
		CLK         => CLK,
		RST_N       => RST_N,
		IN_CLK      => 42954545,
		OUT_CLK     => 1540200,
		CE          => ADPCM_CE
	);
	
	--CD SUBCODE
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			SUBCD_FIFO_D <= (others => '0');
			SUBCD_FIFO_WR_REQ <= '0';
			SUBCD_WR_OLD <= '0';
		elsif rising_edge(CLK) then
			SUBCD_FIFO_WR_REQ <= '0';
			if SaveStateBus_load = '1' then
				SUBCD_WR_OLD <= SS_V(4)(34);
			elsif EN = '1' then
				SUBCD_WR_OLD <= CD_SUBCD_WR;
				if CD_SUBCD_WR = '1' and SUBCD_WR_OLD = '0' then
					SUBCD_FIFO_D(7 downto 0) <= CD_DATA;
					if SUBCD_FIFO_FULL = '0' and DM = '0' then
						SUBCD_FIFO_WR_REQ <= '1';
					end if;
				end if;
			end if;
		end if;
	end process;
	
	CDSUBC_FIFO : entity work.CDSUBC_FIFO 
	port map(
		clock		=> CLK,
		data		=> SUBCD_FIFO_D,
		wrreq		=> SUBCD_FIFO_WR_REQ,
		full		=> SUBCD_FIFO_FULL,
		sclr		=> SUBCD_FIFO_SCLR,
		rdreq		=> SUBCD_FIFO_RD_REQ,
		empty		=> SUBCD_FIFO_EMPTY,
		q			=> SUBCD_FIFO_Q
	);
	
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			FIFO_SCLR <= '1';
			FIFO_RD_REQ <= '0';
			OUTL <= (others => '0');
			OUTR <= (others => '0');
			SUBCD_CE <= '0';
			SUBCD_CNT <= (others => '0');
			SUBCD_BYTE <= (others => '0');
			SUBCD_BYTENUM <= (others => '0');
			SUBCD_FIFO_SCLR <= '1';
			SUBCD_FIFO_RD_REQ <= '0';

		elsif rising_edge(CLK) then
			FIFO_RD_REQ <= '0';
			FIFO_SCLR <= '0';
			SUBCD_FIFO_RD_REQ <= '0';
			SUBCD_FIFO_SCLR <= '0';

			if REPLAY_START = '1' then
				-- the FIFOs hold the WRONG-position live stream: flush them,
				-- and realign the subcode position to the new sector start
				FIFO_SCLR <= '1';
				SUBCD_FIFO_SCLR <= '1';
				SUBCD_BYTENUM <= (others => '0');
				SUBCD_CNT <= (others => '0');
			elsif SKIP_LEFT /= 0 and SKIP_GAP = '0' and FIFO_EMPTY = '0' and SLEEP = '0' then
				-- fast-drain the skip remainder (sub-sector position);
				-- SKIP_GAP paces it at one frame every two clocks (see below)
				FIFO_RD_REQ <= '1';
			elsif SaveStateBus_load = '1' then
				CDDA_SAMPLE   <= SS_V(4)(10);
				SUBCD_CNT     <= unsigned(SS_V(4)(15 downto 12));
				SUBCD_BYTE    <= SS_V(4)(23 downto 16);
				SUBCD_BYTENUM <= unsigned(SS_V(4)(31 downto 24));
				SUBCD_CE      <= SS_V(4)(32);
				OUTL          <= signed(SS_V(4)(50 downto 35));
				OUTR          <= signed(SS_V(5)(15 downto 0));
			elsif CDDA_CE = '1' and EN = '1' and SLEEP = '0' then	-- ~44.1kHz
				CDDA_SAMPLE <= not CDDA_SAMPLE;
				if FIFO_EMPTY = '0' then
					FIFO_RD_REQ <= '1';
					if (CD_STOP_CD_SND = '0') then
						OUTL <= resize(shift_right(signed(FIFO_Q(15 downto 0)) * signed(CDDA_FADE_VOL), 10), OUTL'length);
						OUTR <= resize(shift_right(signed(FIFO_Q(31 downto 16)) * signed(CDDA_FADE_VOL), 10), OUTR'length);
					else
						OUTL <= (others => '0');
						OUTR <= (others => '0');
						FIFO_SCLR <= '1';
						SUBCD_FIFO_SCLR <= '1';
					end if;
				end if;


				if SUBCD_CNT = 0 then
					SUBCD_CE <= '1';									-- set interrupt flag
					
					if SUBCD_FIFO_EMPTY = '0' then
						SUBCD_FIFO_RD_REQ <= '1';
						SUBCD_BYTE <= SUBCD_FIFO_Q(7 downto 0);
					end if;

					-- Note that there are 96 bytes in a subcode sector, PLUS 2 bytes as a 'synchronization word'
					-- The sync word bytes are "0x00, 0x80" when the motor is running, or "0x1F, 0xFD" when it is not running
						--> Currently, Main_MiSTer send these two bytes, and only implements the 'motor on' version (0x00, 0x80)
					-- When paused, the last sector (with correct SUNCODEQ timing information) should be repeated constantly
						--> Not yet implemented

					if (SUBCD_BYTENUM = 97) then
						SUBCD_BYTENUM <= (others => '0');		-- 98 bytes in sector
					else
						SUBCD_BYTENUM <= SUBCD_BYTENUM + 1;
					end if;
				else
					SUBCD_CE <= '0';									-- SUBCODE 1 sample every 6 CDDA sample intervals
				end if;

				if SUBCD_CNT = 5 then
					SUBCD_CNT <= (others => '0');
				else
					SUBCD_CNT <= SUBCD_CNT + 1;
				end if;

			end if;
		end if;
	end process;

			
	-- P5c: consumed-sample counter (reset on every accepted SAPSP) and the
	-- post-replay skip: drain consumed-mod-588 samples fast (one per CLK)
	-- so the DAC resumes on the exact sample.  Restored from CD_6(58:27).
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			CDDA_CONSUMED <= (others => '0');
			SKIP_LEFT <= (others => '0');
			SKIP_GAP  <= '0';
		elsif rising_edge(CLK) then
			if SaveStateBus_load = '1' then
				CDDA_CONSUMED <= unsigned(SS_V(5)(58 downto 27));
				SKIP_LEFT <= (others => '0');
				SKIP_GAP  <= '0';
			else
				if AUDIO_CMD_SET = '1' then
					CDDA_CONSUMED <= (others => '0');
				elsif FIFO_RD_REQ = '1' and SLEEP = '0' then
					CDDA_CONSUMED <= CDDA_CONSUMED + 1;
				end if;
				-- FIFO_EMPTY is a REGISTERED output, so in the cycle right after
				-- our own read request it still reports the pre-pop state.
				-- Draining on consecutive clocks therefore decrements SKIP_LEFT
				-- once more than it discards every time the FIFO runs dry - and it
				-- runs dry constantly, because the drain takes a frame per clock
				-- while the drive delivers far more slowly.  The skip expired at
				-- roughly half its length and the stream resumed early (581
				-- requested, 290 discarded; 38 requested, 17 discarded).  SKIP_GAP
				-- leaves one idle cycle after every read so EMPTY is settled when
				-- sampled.  Same defect and same fix as RSKIP_GAP in SCSI.vhd.
				if REPLAY_START = '1' then
					SKIP_LEFT <= REPLAY_SKIP;
					SKIP_GAP  <= '0';
				elsif SKIP_GAP = '1' then
					SKIP_GAP <= '0';
				elsif SKIP_LEFT /= 0 and FIFO_EMPTY = '0' and SLEEP = '0' then
					SKIP_LEFT <= SKIP_LEFT - 1;
					SKIP_GAP  <= '1';
					-- no CDDA_CONSUMED here: the drain's FIFO_RD_REQ already
					-- counts in the increment above (it double-counted)
				end if;
			end if;
		end if;
	end process;

	--Fader
	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			FADE_VOL <= "01111111111";
			FADE_CNT <= (others => '0');
		elsif rising_edge(CLK) then
			if SaveStateBus_load = '1' then
				FADE_CNT <= unsigned(SS_V(4)(7 downto 0));
				FADE_VOL <= unsigned(SS_V(5)(26 downto 16));
			elsif CDDA_CE = '1' and EN = '1' and SLEEP = '0' then
				if FADE_VOL(9 downto 0) > 0 and ADPCM_FADER(2) = '1' then
					FADE_CNT <= FADE_CNT + 1;
					if (FADE_CNT = 107 and ADPCM_FADER(1) = '1') or 	--2.5s
						(FADE_CNT = 255 and ADPCM_FADER(1) = '0') then	--6s
						FADE_CNT <= (others => '0');
						FADE_VOL <= "0" & (FADE_VOL(9 downto 0) - 1);
					end if;
				elsif ADPCM_FADER(2) = '0' then
					FADE_VOL <= "01111111111";
				end if;
			end if;
		end if;
	end process;
	
	CDDA_FADE_VOL <= FADE_VOL when ADPCM_FADER(0) = '0' else "01111111111";
	ADPCM_FADE_VOL <= FADE_VOL when ADPCM_FADER(0) = '1' else "01111111111";
	
	CD_SL <= OUTL;
	CD_SR <= OUTR;

	AD_S <= x"0000" when ADPCM_PLAY = '0' else resize(shift_right(M5205_SOUT * signed(ADPCM_FADE_VOL), 10), AD_S'length);

	--------------------------------------------------------------------------------
	-- SAVESTATES (plan P5a; slot map: pce_savestates_pkg.vhd 53-63)
	--------------------------------------------------------------------------------
	-- NOT saved: CDDA/SUBC FIFO content (refilled by the stream; the fill
	-- paths run through the freeze so no pushed sample is lost), CEGen
	-- CLK_SUM phases and the M5205 divider phase between XTI edges
	-- (sub-sample reseed), SCSI_FIFO (drained by the CD-quiet boundary).

	SS_V_BACK(0)(7 downto 0)   <= SCSI_DBI;
	SS_V_BACK(0)(8)            <= SCSI_ACK_N;
	SS_V_BACK(0)(9)            <= SCSI_RST_N;
	SS_V_BACK(0)(10)           <= SCSI_SEL_N;
	SS_V_BACK(0)(11)           <= CD_DTD;
	SS_V_BACK(0)(12)           <= CD_DTR;
	SS_V_BACK(0)(13)           <= CD_SUBCD;
	SS_V_BACK(0)(14)           <= CH_SEL;
	SS_V_BACK(0)(15)           <= BRAM_LOCK;
	SS_V_BACK(0)(16)           <= CD_DTD_EN;
	SS_V_BACK(0)(17)           <= CD_DTR_EN;
	SS_V_BACK(0)(18)           <= CD_SUBCD_EN;
	SS_V_BACK(0)(19)           <= ADPCM_END_EN;
	SS_V_BACK(0)(20)           <= ADPCM_HALF_EN;
	SS_V_BACK(0)(21)           <= AUTO_ACK;
	SS_V_BACK(0)(22)           <= R1802_0;
	SS_V_BACK(0)(23)           <= R1802_1;
	SS_V_BACK(0)(27 downto 24) <= R180E_7_4;
	SS_V_BACK(0)(28)           <= R180F_0;
	SS_V_BACK(0)(32 downto 29) <= R180F_7_4;
	SS_V_BACK(0)(43 downto 33) <= std_logic_vector(CD_DATA_CNT);
	SS_V_BACK(0)(44)           <= SCSI_REQ_N_OLD;
	SS_V_BACK(0)(45)           <= SCSI_BSY_N_OLD;
	SS_V_BACK(0)(46)           <= SCSI_ACK_N_OLD;
	SS_V_BACK(0)(47)           <= CD_WR_OLD;
	SS_V_BACK(0)(63 downto 48) <= CDDA_VOL;

	SS_V_BACK(1)(15 downto 0)  <= ADPCM_OFFS;
	SS_V_BACK(1)(32 downto 16) <= ADPCM_LEN;
	SS_V_BACK(1)(47 downto 40) <= ADPCM_CTRL;
	SS_V_BACK(1)(51 downto 48) <= ADPCM_FREQ;
	SS_V_BACK(1)(54 downto 52) <= ADPCM_FADER;
	SS_V_BACK(1)(55)           <= ADPCM_DMA_EN;
	SS_V_BACK(1)(56)           <= ADPCM_DMA_RUN;
	SS_V_BACK(1)(57)           <= ADPCM_END;
	SS_V_BACK(1)(58)           <= ADPCM_HALF;
	SS_V_BACK(1)(59)           <= ADPCM_PLAY;
	SS_V_BACK(1)(60)           <= ADPCM_WRITE_PEND;
	SS_V_BACK(1)(61)           <= ADPCM_READ_PEND;
	SS_V_BACK(1)(62)           <= PLAY_READ_PEND;
	SS_V_BACK(1)(63)           <= DMA_WRITE_PEND;

	SS_V_BACK(2)(16 downto 0)  <= ADPCM_RDADDR;
	SS_V_BACK(2)(36 downto 20) <= ADPCM_WRADDR;
	SS_V_BACK(2)(47 downto 40) <= ADPCM_RDDATA;
	SS_V_BACK(2)(55 downto 48) <= ADPCM_WRDATA;
	SS_V_BACK(2)(56)           <= ADPCM_WRITE_NIB;
	SS_V_BACK(2)(57)           <= ADPCM_READ_NIB;
	SS_V_BACK(2)(61 downto 58) <= M5205_D;

	SS_V_BACK(3)(38 downto 0)  <= M5205_SS_DOUT;

	SS_V_BACK(4)(7 downto 0)   <= std_logic_vector(FADE_CNT);
	SS_V_BACK(4)(9 downto 8)   <= std_logic_vector(CD_BYTE_CNT);
	SS_V_BACK(4)(10)           <= CDDA_SAMPLE;
	SS_V_BACK(4)(11)           <= CDDA_SAMPLE_OLD;
	SS_V_BACK(4)(15 downto 12) <= std_logic_vector(SUBCD_CNT);
	SS_V_BACK(4)(23 downto 16) <= SUBCD_BYTE;
	SS_V_BACK(4)(31 downto 24) <= std_logic_vector(SUBCD_BYTENUM);
	SS_V_BACK(4)(32)           <= SUBCD_CE;
	SS_V_BACK(4)(33)           <= SUBCD_CE_OLD;
	SS_V_BACK(4)(34)           <= SUBCD_WR_OLD;
	SS_V_BACK(4)(50 downto 35) <= std_logic_vector(OUTL);

	SS_V_BACK(5)(15 downto 0)  <= std_logic_vector(OUTR);
	SS_V_BACK(5)(26 downto 16) <= std_logic_vector(FADE_VOL);
	SS_V_BACK(5)(58 downto 27) <= std_logic_vector(CDDA_CONSUMED);

	GEN_SS_CD1 : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_CD_1, def => SSREG_DEFAULT_CD )
	port map ( CLK, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren,
	           SaveStateBus_rst, SS_V_Dout(0), SS_V_BACK(0), SS_V(0) );
	GEN_SS_CD2 : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_CD_2, def => SSREG_DEFAULT_CD )
	port map ( CLK, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren,
	           SaveStateBus_rst, SS_V_Dout(1), SS_V_BACK(1), SS_V(1) );
	GEN_SS_CD3 : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_CD_3, def => SSREG_DEFAULT_CD )
	port map ( CLK, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren,
	           SaveStateBus_rst, SS_V_Dout(2), SS_V_BACK(2), SS_V(2) );
	GEN_SS_CD4 : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_CD_4, def => SSREG_DEFAULT_CD )
	port map ( CLK, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren,
	           SaveStateBus_rst, SS_V_Dout(3), SS_V_BACK(3), SS_V(3) );
	GEN_SS_CD5 : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_CD_5, def => SSREG_DEFAULT_CD )
	port map ( CLK, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren,
	           SaveStateBus_rst, SS_V_Dout(4), SS_V_BACK(4), SS_V(4) );
	GEN_SS_CD6 : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_CD_6, def => SSREG_DEFAULT_CD )
	port map ( CLK, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren,
	           SaveStateBus_rst, SS_V_Dout(5), SS_V_BACK(5), SS_V(5) );

	SaveStateBus_Dout <= SS_V_Dout(0) or SS_V_Dout(1) or SS_V_Dout(2)
	                  or SS_V_Dout(3) or SS_V_Dout(4) or SS_V_Dout(5)
	                  or SS_V_Dout_DBG68 or SS_Dout_SCSI;

	-- CD-quiet veto (plan P5a): SCSI fully drained AND no ADPCM-DMA byte in
	-- flight between the SCSI layer and the DRAM.
	SS_CD_HOLD <= EN and not (SCSI_SS_QUIET and not DMA_WRITE_PEND);

end rtl;
