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
		CPU_WAIT_N	: out std_logic;
		
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
		CD_RESET_DONE : in std_logic := '0';
		CD_FAST_SEEK : out std_logic := '0';
		
		CD_DATA		: in std_logic_vector(7 downto 0);
		CD_DATA_WR	: in std_logic;
		CD_AUDIO_WR	: in std_logic;
		CD_SUBCD_WR	: in std_logic;
		CD_DATA_END	: out std_logic;
		
		DM				: in std_logic;
		
		CD_SL			: out signed(15 downto 0);
		CD_SR			: out signed(15 downto 0);
		AD_S			: out signed(15 downto 0);

		-- Savestates: slots 53-54 and 60-63; SCSI owns 55-59.
		-- SLEEP freezes the consuming/decoding side; the CDDA/SUBC FIFO fill
		-- paths stay live, so samples the HPS pushes during the freeze are kept
		-- while the FIFOs have room.
		SLEEP				: in std_logic := '0';
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
		SS_CD_HOLD        : out std_logic;
		-- Post-load audio hold: freeze the machine until resumed CDDA is ready
		SS_AUDIO_HOLD     : out std_logic
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
	signal SCSI_DATAIN_HELD	: std_logic;	-- staged DATAIN byte held during replay
	signal SCSI_MSG_N			: std_logic;
	signal SCSI_CD_N			: std_logic;
	signal SCSI_IO_N			: std_logic;
	signal CD_STOP_CD_SND	: std_logic;
	signal SCSI_REPLAY_RESET : std_logic;

	
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
	signal DMA_STAGED			: std_logic := '0';	-- staged DATAIN byte owned by DMA
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
	signal SUBCD_SKIP_LEFT	: unsigned(6 downto 0) := (others => '0');
	signal SUBCD_RESUME		: std_logic := '0';
	
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
	-- CDDA replay
	signal CDDA_CONSUMED : unsigned(31 downto 0);
	signal AUDIO_CMD_SET : std_logic;
	signal REPLAY_SKIP   : unsigned(9 downto 0);
	signal REPLAY_START  : std_logic;
	signal REPLAY_CONSUMED : unsigned(31 downto 0);
	signal SKIP_LEFT     : unsigned(9 downto 0) := (others => '0');
	signal SKIP_GAP      : std_logic := '0';	-- idle cycle after a skip read (FIFO_EMPTY is registered)
	signal REPLAY_AUDIO  : std_logic;
	signal REPLAY_RESUME : std_logic;
	signal REPLAY_AUDIO_D : std_logic := '0';
	-- Post-load audio hold. AH freezes the machine (SLEEP via pce_top) while
	-- the replay restarts CDDA; AH_GO marks the resuming flush; AH_WD bounds it.
	signal AH            : std_logic := '0';
	signal AH_GO         : std_logic := '0';
	signal AH_WD         : unsigned(26 downto 0) := (others => '0');	-- bit26 = 1.56 s
	signal AH_READY      : std_logic;
	signal SKIP_EN       : std_logic;	-- skip drains and their count run during the hold
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
				-- DMA stores the byte before completing ACK; the CPU consumes
				-- it on its register read. Preserve ownership from the saved
				-- DMA/ACK state before replay clears the live handshake.
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
				-- Do not consume the staged byte again while replay holds it.
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
					-- Hide staged DATAIN until replay restores the stream.
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
	-- Sector readers may consume $1808 without polling between bytes.
	-- Stall the CPU until the restored stream is ready, not the CD replay.
	CPU_WAIT_N <= not SCSI_DATAIN_HELD;
	-- Mask both data-ready status and IRQ while the staged byte is held.
	IRQ_N <= not ((CD_DTR_EN and CD_DTR and not SCSI_DATAIN_HELD) or (CD_DTD_EN and CD_DTD) or (CD_SUBCD_EN and CD_SUBCD) or (ADPCM_END_EN and ADPCM_END) or (ADPCM_HALF_EN and ADPCM_HALF));
	
	RAM_SEL <= '1' when EXT_A(20 downto 13) >= x"68" and EXT_A(20 downto 13) <= x"87" else '0';
	RAM_CS_N <= not (RAM_SEL and EN);
	
	BRAM_EN <= not BRAM_LOCK or not EN;
	CD_RESET <= (not SCSI_RST_N) or SCSI_REPLAY_RESET;
	
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
		COMM_SEND	=> CD_COMM_SEND,
		
		DOUT_REQ		=> CD_DOUT_REQ,
		DOUT			=> CD_DOUT,
		DOUT_SEND	=> CD_DOUT_SEND,
		STOP_CD_SND	=> CD_STOP_CD_SND,
		CD_RESET_DONE => CD_RESET_DONE,
		REPLAY_RESET => SCSI_REPLAY_RESET,
		
		CD_DATA		=> CD_DATA,
		CD_WR			=> CD_DATA_WR,
		CD_DATA_END	=> CD_DATA_END,
		DATAIN_HELD	=> SCSI_DATAIN_HELD,

		SLEEP				=> SLEEP,
		DTD_EN			=> CD_DTD_EN,
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
		REPLAY_START	=> REPLAY_START,
		REPLAY_CONSUMED => REPLAY_CONSUMED,
		REPLAY_FAST => CD_FAST_SEEK,
		REPLAY_AUDIO => REPLAY_AUDIO,
		REPLAY_RESUME => REPLAY_RESUME
	);

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

	-- The ADPCM RAM read is registered. Present nibble 0, wait one clock,
	-- latch its data, then select nibble 1 for the second half of the byte.
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
					-- Divide ADPCM_CE by (16 - ADPCM_FREQ).
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
			SUBCD_SKIP_LEFT <= (others => '0');
			SUBCD_RESUME <= '0';

		elsif rising_edge(CLK) then
			FIFO_RD_REQ <= '0';
			FIFO_SCLR <= '0';
			SUBCD_FIFO_RD_REQ <= '0';
			SUBCD_FIFO_SCLR <= '0';
			
			if REPLAY_START = '1' then
				-- Discard the live tail and restart at the replayed sector.
				FIFO_SCLR <= '1';
				SUBCD_FIFO_SCLR <= '1';
				SUBCD_BYTENUM <= (others => '0');
				SUBCD_CNT <= (others => '0');
				SUBCD_CE <= '0';
				SUBCD_SKIP_LEFT <= (others => '0');
				SUBCD_RESUME <= '0';
				if REPLAY_SKIP /= 0 then
					SUBCD_RESUME <= '1';
				end if;
			elsif SaveStateBus_load = '1' then
				CDDA_SAMPLE   <= SS_V(4)(10);
				SUBCD_CNT     <= unsigned(SS_V(4)(15 downto 12));
				SUBCD_BYTE    <= SS_V(4)(23 downto 16);
				SUBCD_BYTENUM <= unsigned(SS_V(4)(31 downto 24));
				SUBCD_CE      <= SS_V(4)(32);
				OUTL          <= signed(SS_V(4)(50 downto 35));
				OUTR          <= signed(SS_V(5)(15 downto 0));
				SUBCD_SKIP_LEFT <= (others => '0');
				SUBCD_RESUME <= '0';
			elsif SUBCD_RESUME = '1' and (CD_STOP_CD_SND = '1' or SCSI_RST_N = '0' or AUDIO_CMD_SET = '1') then
				SUBCD_SKIP_LEFT <= (others => '0');
				SUBCD_RESUME <= '0';
				SUBCD_CNT <= (others => '0');
				SUBCD_BYTENUM <= (others => '0');
				SUBCD_CE <= '0';
			elsif SKIP_LEFT /= 0 or SKIP_GAP = '1' then
				SUBCD_CE <= '0';
				if SKIP_LEFT /= 0 and SKIP_GAP = '0' and FIFO_EMPTY = '0' and SKIP_EN = '1' then
					FIFO_RD_REQ <= '1';
					-- Main sends audio before subcode. Keep the byte debt until
					-- its FIFO fills; advancing empty reads would lose the prefix.
					if SUBCD_CNT = 5 then
						SUBCD_CNT <= (others => '0');
						SUBCD_BYTENUM <= SUBCD_BYTENUM + 1;
						SUBCD_SKIP_LEFT <= SUBCD_SKIP_LEFT + 1;
					else
						SUBCD_CNT <= SUBCD_CNT + 1;
					end if;
				end if;
			elsif SUBCD_SKIP_LEFT /= 0 or SUBCD_FIFO_RD_REQ = '1' then
				SUBCD_CE <= '0';
				-- Leave a clock after each pop for the registered EMPTY flag.
				if SUBCD_SKIP_LEFT /= 0 and SUBCD_FIFO_RD_REQ = '0' and SUBCD_FIFO_EMPTY = '0' and SKIP_EN = '1' then
					SUBCD_FIFO_RD_REQ <= '1';
					SUBCD_SKIP_LEFT <= SUBCD_SKIP_LEFT - 1;
				end if;
			elsif SUBCD_RESUME = '1' and SUBCD_FIFO_EMPTY = '1' then
				-- Do not start the audible remainder ahead of its subcode.
				SUBCD_CE <= '0';
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


				if SUBCD_CNT = 0 or SUBCD_RESUME = '1' then
					SUBCD_CE <= '1';									-- set interrupt flag
					
					if SUBCD_FIFO_EMPTY = '0' then
						SUBCD_FIFO_RD_REQ <= '1';
						SUBCD_BYTE <= SUBCD_FIFO_Q(7 downto 0);
						SUBCD_RESUME <= '0';
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

			
	-- Count consumed samples and discard the restored sub-sector remainder.
	-- The counter is restored from CD_6(58:27).
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
				elsif REPLAY_START = '1' then
					-- Drop stale pops and count the replay drain back up to the
					-- saved position, rather than adding its skip a second time.
					CDDA_CONSUMED <= REPLAY_CONSUMED;
				elsif FIFO_RD_REQ = '1' and SKIP_EN = '1' then
					CDDA_CONSUMED <= CDDA_CONSUMED + 1;
				end if;
				-- Leave an idle clock after each pop: FIFO_EMPTY is registered.
				if REPLAY_START = '1' then
					SKIP_LEFT <= REPLAY_SKIP;
					SKIP_GAP  <= '0';
				elsif SUBCD_RESUME = '1' and (CD_STOP_CD_SND = '1' or SCSI_RST_N = '0' or AUDIO_CMD_SET = '1') then
					SKIP_LEFT <= (others => '0');
					SKIP_GAP <= '0';
				elsif SKIP_GAP = '1' then
					SKIP_GAP <= '0';
				elsif SKIP_LEFT /= 0 and FIFO_EMPTY = '0' and SKIP_EN = '1' then
					SKIP_LEFT <= SKIP_LEFT - 1;
					SKIP_GAP  <= '1';
					-- FIFO_RD_REQ already accounts for the discarded sample.
				end if;
			end if;
		end if;
	end process;

	-- Post-load audio hold. A load that restores active CDDA keeps the whole
	-- machine frozen (pce_top ORs AH into SLEEP) until the replay's resuming
	-- flush has run, both skip drains are done and the saved sample heads the
	-- FIFO with its subcode present. CPU, video and the DAC then restart on
	-- the same clock, the DAC on its next 44.1 kHz tick. Nothing but the skip
	-- drains advances during the hold. The hold ends at once when the replay
	-- resumes nothing, and after 2^26 clocks (1.56 s) regardless.
	SKIP_EN <= not SLEEP or AH;
	AH_READY <= '1' when REPLAY_START = '0' and FIFO_SCLR = '0' and FIFO_RD_REQ = '0'
	                     and SKIP_LEFT = 0 and SKIP_GAP = '0'
	                     and SUBCD_SKIP_LEFT = 0 and SUBCD_FIFO_RD_REQ = '0'
	                     and FIFO_EMPTY = '0'
	                     and (SUBCD_RESUME = '0' or SUBCD_FIFO_EMPTY = '0') else '0';

	process( RST_N, CLK )
	begin
		if RST_N = '0' then
			AH <= '0';
			AH_GO <= '0';
			AH_WD <= (others => '0');
			REPLAY_AUDIO_D <= '0';
		elsif rising_edge(CLK) then
			REPLAY_AUDIO_D <= REPLAY_AUDIO;
			if SaveStateBus_load = '1' then
				AH <= '0';
				AH_GO <= '0';
			elsif REPLAY_AUDIO = '1' and REPLAY_AUDIO_D = '0' then
				-- raised by SCSI on the load that restores active CDDA
				AH <= EN;
				AH_GO <= '0';
				AH_WD <= (others => '0');
			elsif AH = '1' then
				AH_WD <= AH_WD + 1;
				if REPLAY_RESUME = '1' then
					AH_GO <= '1';
				end if;
				if AH_WD(26) = '1' then
					AH <= '0';
				elsif AH_GO = '0' and REPLAY_AUDIO = '0' and REPLAY_RESUME = '0' then
					AH <= '0';	-- the replay resumes nothing
				elsif AH_GO = '1' and AH_READY = '1' then
					AH <= '0';
				end if;
			end if;
		end if;
	end process;

	SS_AUDIO_HOLD <= AH;

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
	-- Savestates (slot map in pce_savestates_pkg.vhd)
	--------------------------------------------------------------------------------
	-- NOT saved: CDDA/SUBC FIFO content (refilled by the stream; the fill
	-- paths run through the freeze so no pushed sample is lost), CEGen
	-- CLK_SUM phases, SCSI_FIFO (drained by the CD-quiet boundary).

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

	-- MSM5205 bit 38 is padding, not decoder state; CD owns bits 46:38.
	SS_V_BACK(3)(37 downto 0)  <= M5205_SS_DOUT(37 downto 0);
	SS_V_BACK(3)(46 downto 38) <= std_logic_vector(to_unsigned(M5205_CLK_CNT, 9));

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
	                  or SS_Dout_SCSI;

	-- The boundary waits for a quiet SCSI, no pending ADPCM DMA write and the
	-- end of the post-load audio hold. CDDA/subcode FIFO levels do not veto
	-- it: Main delivers as fast as the DAC consumes, so a backlog left by a
	-- freeze would never drain.
	SS_CD_HOLD <= EN when SCSI_SS_QUIET = '0' or DMA_WRITE_PEND = '1' or AH = '1' else '0';

end rtl;
