library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.STD_LOGIC_TEXTIO.all;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity pce_top is
	generic (
		-- Bind these BY NAME from the SV top.  A positional #(LITE) lands on
		-- SS_CDRAM_BYTES and silently drops the CD work RAM from the blob.
		SS_CDRAM_BYTES : integer := 262144;
		LITE : integer := 0
	);
	port(
		RESET			: in  std_logic;
		COLD_RESET	: in  std_logic;
		CLK 			: in  std_logic;

		ROM_RD		: out std_logic;
		ROM_RDY		: in  std_logic;
		ROM_A 		: out std_logic_vector(21 downto 0);
		ROM_DO 		: in  std_logic_vector(7 downto 0);
		ROM_SZ 		: in  std_logic_vector(11 downto 0);
		ROM_POP		: in  std_logic;
		ROM_CLKEN	: out std_logic;

		BRM_A 		: out std_logic_vector(10 downto 0);
		BRM_DI 		: out std_logic_vector(7 downto 0);
		BRM_DO 		: in  std_logic_vector(7 downto 0);
		BRM_WE 		: out std_logic;

		GG_EN			: in  std_logic;
		GG_CODE		: in  std_logic_vector(128 downto 0);
		GG_RESET		: in  std_logic;
		GG_AVAIL		: out std_logic;

		SP64			: in  std_logic;
		SGX			: in  std_logic;

		JOY_OUT     : out std_logic_vector(1 downto 0);
		JOY_IN      : in  std_logic_vector(3 downto 0);

		CD_EN			: in  std_logic;
		CD_RAM_A 	: out std_logic_vector(21 downto 0);
		CD_RAM_DO 	: out std_logic_vector(7 downto 0);
		CD_RAM_DI 	: in  std_logic_vector(7 downto 0);
		CD_RAM_RD	: out std_logic;
		CD_RAM_WR	: out std_logic;
		AC_EN			: in  std_logic;

		CD_STAT		: in  std_logic_vector(7 downto 0);
		CD_MSG		: in  std_logic_vector(7 downto 0);
		CD_STAT_GET	: in  std_logic;

		CD_COMM		: out std_logic_vector(95 downto 0);
		CD_COMM_SEND: out std_logic;

		CD_DOUT_REQ	: in  std_logic;
		CD_DOUT		: out std_logic_vector(79 downto 0);
		CD_DOUT_SEND: out std_logic;

		CD_REGION   : in  std_logic;
		CD_RESET		: out std_logic;
		CD_RESET_DONE : in std_logic := '0';
		CD_FAST_SEEK : out std_logic := '0';

		CD_DATA		: in  std_logic_vector(7 downto 0);
		CD_DATA_WR	: in  std_logic;
		CD_AUDIO_WR	: in  std_logic;
		CD_SUBCD_WR	: in  std_logic;			-- subcode data
		CD_DATA_END	: out std_logic;
		CD_DM			: in  std_logic;

		CDDA_SL		: out signed(15 downto 0);
		CDDA_SR		: out signed(15 downto 0);
		ADPCM_S		: out signed(15 downto 0);
		PSG_SL		: out signed(15 downto 0);
		PSG_SR		: out signed(15 downto 0);

		BG_EN			: in  std_logic;
		SPR_EN		: in  std_logic;
		GRID_EN		: in  std_logic_vector(1 downto 0);
		CPU_PAUSE_EN: in  std_logic;

		-- Savestates (Robert Peip framework)
		SS_SAVE		: in  std_logic := '0';						-- save request pulse (savestate_ui)
		SS_LOAD		: in  std_logic := '0';						-- load request pulse
		SS_SLOT		: in  std_logic_vector(1 downto 0) := "00";
		SS_BUSY		: out std_logic;
		SS_FAILED	: out std_logic;					-- save failed or load cancelled before restore
		SS_ACTIVE_SLOT : out std_logic_vector(1 downto 0);
		SS_ACTIVE_LOAD : out std_logic := '0';
		SS_SLEEP		: out std_logic;								-- core frozen (for top-level muxes)
		-- 64-bit DDR3 channel (wire to ddram ch1 in TurboGrafx16.sv)
		SS_DDR_DIN	: out std_logic_vector(63 downto 0);
		SS_DDR_DOUT	: in  std_logic_vector(63 downto 0) := (others => '0');
		SS_DDR_ADDR	: out std_logic_vector(25 downto 0);	-- DWORD address
		SS_DDR_RNW	: out std_logic;
		SS_DDR_ENA	: out std_logic;
		SS_DDR_BE	: out std_logic_vector(7 downto 0);
		SS_DDR_DONE	: in  std_logic := '0';
		-- External hold veto: while '1' the composite boundary is not taken
		-- (e.g. an external serial transfer in flight)
		SS_EXT_HOLD	: in  std_logic := '0';
		SS_INHIBIT	: in  std_logic := '0';	-- reject uncommitted save/load requests
		SS_TO_SD	: in  std_logic := '1';	-- '0': saves keep the slot change counter, so Main leaves them off SD
		-- SaveStateBus export for SystemVerilog-side eRegs (TOP_EXT)
		SSE_Din		: out std_logic_vector(63 downto 0);
		SSE_Adr		: out std_logic_vector(9 downto 0);
		SSE_wren		: out std_logic;
		SSE_rst		: out std_logic;
		SSE_load		: out std_logic;
		SSE_Dout		: in  std_logic_vector(63 downto 0) := (others => '0');
		-- boundary-block telemetry: one-hot of the composite terms that were
		-- blocking at the last failed check while a freeze was pending, plus
		-- the forced flag - turns a wedged boundary into a one-dump diagnosis
		SS_DBG		: out std_logic_vector(8 downto 0);
		-- sticky: the game actually touched the Arcade Card (regs or RAM)
		SS_AC_USED	: out std_logic;

		BORDER_EN	: in  std_logic;
		ReducedVBL	: in  std_logic;
		VIDEO_R		: out std_logic_vector(2 downto 0);
		VIDEO_G		: out std_logic_vector(2 downto 0);
		VIDEO_B		: out std_logic_vector(2 downto 0);
		VIDEO_BW		: out std_logic;
		VIDEO_CE		: out std_logic;
		VIDEO_CE_FS	: out std_logic;
		VIDEO_VS		: out std_logic;
		VIDEO_HS		: out std_logic;
		VIDEO_HBL	: out std_logic;
		VIDEO_VBL	: out std_logic
	);
end pce_top;

architecture rtl of pce_top is

signal RESET_N			: std_logic := '0';

-- CPU signals
signal CPU_CE			: std_logic;
signal CPU_CE2			: std_logic;
signal CPU_RD_N		: std_logic;
signal CPU_WR_N		: std_logic;
signal CPU_DI			: std_logic_vector(7 downto 0);
signal CPU_DO			: std_logic_vector(7 downto 0);
signal CPU_A			: std_logic_vector(20 downto 0);
signal CPU_CLKEN		: std_logic;
signal CPU_VCE_SEL_N	: std_logic;
signal CPU_VDC_SEL_N	: std_logic;
signal CPU_RAM_SEL_N	: std_logic;
signal CPU_BRM_SEL_N	: std_logic;
signal CPU_IO_DO		: std_logic_vector(7 downto 0);

signal CPU_VDC0_SEL_N: std_logic;
signal CPU_VDC1_SEL_N: std_logic;
signal CPU_VPC_SEL_N	: std_logic;

signal CPU_ROM_SEL_N	: std_logic;

-- RAM signals
signal RAM_DO			: std_logic_vector(7 downto 0);
signal RAM_A			: std_logic_vector(14 downto 0);

signal PRAM_DO			: std_logic_vector(7 downto 0);
signal CPU_PRAM_SEL_N: std_logic;

-- VCE signals
signal VCE_DO			: std_logic_vector(7 downto 0);

-- VDC signals
signal VDC0_DO			: std_logic_vector(15 downto 0);		-- only lower 8 bits are used in 8-bit mode
alias  VDC0_DO_LO		: std_logic_vector(7 downto 0) is VDC0_DO(7 downto 0);
signal VDC0_BUSY_N	: std_logic;
signal VDC0_IRQ_N		: std_logic;
signal VDC0_COLNO		: std_logic_vector(8 downto 0);
signal VDC1_DO			: std_logic_vector(15 downto 0);		-- only lower 8 bits are used in 8-bit mode
alias  VDC1_DO_LO		: std_logic_vector(7 downto 0) is VDC1_DO(7 downto 0);
signal VDC1_BUSY_N	: std_logic;
signal VDC1_IRQ_N		: std_logic;
signal VDC1_COLNO		: std_logic_vector(8 downto 0);
signal VDC_CLKEN		: std_logic;
signal VDC_CLKEN_F	: std_logic;
signal VPC_DO			: std_logic_vector(7 downto 0);
signal VDCNUM    		: std_logic;
signal VDC_COLNO		: std_logic_vector(8 downto 0);

-- CD signals
signal CD_SEL_N		: std_logic;
signal CD_DO			: std_logic_vector(7 downto 0);
signal CD_IRQ_N    	: std_logic;

-- NTSC/RGB Video Output
signal VS_N				: std_logic;
signal HS_N				: std_logic;

signal PCE_SL			: std_logic_vector(23 downto 0);
signal PCE_SR			: std_logic_vector(23 downto 0);

signal rombank			: std_logic_vector(1 downto 0);

signal gamepad_out	: std_logic_vector(1 downto 0);
signal gamepad_port	: unsigned(2 downto 0);
signal gamepad_nibble: std_logic;

signal GENIE		: boolean;
signal GENIE_DO	: std_logic_vector(7 downto 0);
signal GENIE_DI   : std_logic_vector(7 downto 0);

component CODES is
	generic(
		ADDR_WIDTH  : in integer := 16;
		DATA_WIDTH  : in integer := 8
	);
	port(
		clk         : in  std_logic;
		reset       : in  std_logic;
		enable      : in  std_logic;
		addr_in     : in  std_logic_vector(20 downto 0);
		data_in     : in  std_logic_vector(7 downto 0);
		code        : in  std_logic_vector(128 downto 0);
		available   : out std_logic;
		genie_ovr   : out boolean;
		genie_data  : out std_logic_vector(7 downto 0)
	);
end component;

signal VCE_HSYNC_F, VCE_HSYNC_R, VCE_VSYNC_F, VCE_VSYNC_R: std_logic;
signal VRAM0_A	   : std_logic_vector(15 downto 0);
signal VRAM0_DI	: std_logic_vector(15 downto 0);
signal VRAM0_DO	: std_logic_vector(15 downto 0);
signal VRAM0_WE	: std_logic;
signal VRAM1_A	   : std_logic_vector(15 downto 0);
signal VRAM1_DI	: std_logic_vector(15 downto 0);
signal VRAM1_DO	: std_logic_vector(15 downto 0);
signal VRAM1_WE	: std_logic;
signal CLR_A	   : std_logic_vector(14 downto 0);
signal CLR_WE		: std_logic;
signal VDC0_BORDER: std_logic;
signal VDC0_GRID	: std_logic_vector(1 downto 0);
signal CPU_PRE_RD	: std_logic;
signal CPU_PRE_WR	: std_logic;
signal CD_RAM_CS_N: std_logic;
signal CD_BRAM_EN	: std_logic;

signal BORDER		: std_logic;
signal GRID			: std_logic_vector(1 downto 0);

signal AC_SEL_N   : std_logic;
signal AC_RAM_CS_N: std_logic;
signal AC_RAM_A   : std_logic_vector(20 downto 0);
signal AC_DO      : std_logic_vector(7 downto 0);

-- Savestate framework signals
signal sleep_ss          : std_logic;
signal ss_paused         : std_logic := '0';
signal ss_busy_i         : std_logic;
signal ss_save_failed    : std_logic;
signal ss_reset          : std_logic;
signal ss_load_done      : std_logic;
signal ss_req_save       : std_logic;
signal ss_req_load       : std_logic;
signal ss_addr_int       : integer;
signal ss_request_slot  : integer range 0 to 3;
signal SaveStateBus_Din  : std_logic_vector(63 downto 0);
signal SaveStateBus_Adr  : std_logic_vector(9 downto 0);
signal SaveStateBus_wren : std_logic;
signal SaveStateBus_rst  : std_logic;
signal SaveStateBus_Dout : std_logic_vector(63 downto 0);
signal Save_RAMAddr      : std_logic_vector(24 downto 0);
signal Save_RAMRdEn      : std_logic;
signal Save_RAMWrEn      : std_logic;
signal Save_RAMWriteData : std_logic_vector(7 downto 0);
signal Save_RAMReadData  : std_logic_vector(7 downto 0);
signal Save_RAMType      : unsigned(3 downto 0);
signal SS_TOP_BACK       : std_logic_vector(63 downto 0);
signal SS_TOP            : std_logic_vector(63 downto 0);
signal SS_Dout_TOP       : std_logic_vector(63 downto 0);
signal SS_Dout_VCE       : std_logic_vector(63 downto 0);
signal SS_Dout_CPU       : std_logic_vector(63 downto 0);
signal SS_Dout_VDC0      : std_logic_vector(63 downto 0);
signal SS_Dout_VDC1      : std_logic_vector(63 downto 0);
signal SS_Dout_VPC       : std_logic_vector(63 downto 0);
signal ss_pal_rd         : std_logic_vector(8 downto 0);
signal ss_pal_we         : std_logic;
signal ss_wf_rd          : std_logic_vector(4 downto 0);
signal ss_wf_we          : std_logic;
signal ss_sat0_rd        : std_logic_vector(15 downto 0);
signal ss_sat0_we        : std_logic;
signal ss_sat1_rd        : std_logic_vector(15 downto 0);
signal ss_sat1_we        : std_logic;
signal ss_busy0          : std_logic;	-- VDC0 DMA busy (DMA_EXEC or DMAS_EXEC)
signal ss_busy1          : std_logic;	-- VDC1 (SGX)
signal SS_Dout_CD        : std_logic_vector(63 downto 0);
signal ss_ad_rd          : std_logic_vector(7 downto 0);
signal ss_ad_we          : std_logic;
signal ss_cd_hold        : std_logic;
signal cd_cpu_wait_n     : std_logic;
signal ss_bnd_idle       : std_logic;	-- CPU at STATE=0 with no pending reset
signal video_vbl_i       : std_logic;
signal ss_boundary       : std_logic;
signal ss_frozen         : std_logic := '0';
signal ss_wd             : unsigned(27 downto 0) := (others => '0');
signal ss_sleep_eff      : std_logic;
signal ss_walk_sleep     : std_logic;	-- freeze, restore and refetch; exported as SS_SLEEP
signal ss_audio_hold     : std_logic;	-- post-load CDDA hold from the CD unit
signal ss_saving         : std_logic;
signal ss_forced         : std_logic := '0';
signal ss_dbg_block      : std_logic_vector(7 downto 0) := (others => '0');
signal ss_ac_used_r      : std_logic := '0';
signal ss_arcade_loaded  : std_logic;
signal ss_save_abort     : std_logic;
signal ss_refetch        : std_logic := '0';
signal ss_refetch_cnt    : unsigned(7 downto 0) := (others => '0');
signal ss_sleep_d        : std_logic := '0';
signal ss_saving_d       : std_logic := '0';
signal ss_kick           : std_logic;
-- Shared CD/Arcade work-RAM walk; borrows the physical SDRAM port.
signal ss_cdr_act        : std_logic;
signal ss_cdr_addr       : std_logic_vector(21 downto 0);
signal ss_cdr_data       : std_logic_vector(7 downto 0);
signal ss_cdr_wdata      : std_logic_vector(7 downto 0);
signal ss_cdr_rd         : std_logic;
signal ss_cdr_wr         : std_logic;
signal ss_cdr_kick       : std_logic;
signal ss_cdr_dbg_ns     : unsigned(15 downto 0) := (others => '0');	-- reads sampled with no rdy dip seen
signal ss_cdr_dbg_to     : unsigned(7 downto 0) := (others => '0');	-- 255-CLK timeouts
signal ss_cdr_dbg_max    : unsigned(7 downto 0) := (others => '0');	-- slowest byte (cnt at sample)
signal SS_Dout_CDRDBG    : std_logic_vector(63 downto 0);
signal SS_Dout_AC        : std_logic_vector(63 downto 0);
signal ss_ram_rdy        : std_logic;
signal VRAM0_QB          : std_logic_vector(15 downto 0);
signal VRAM1_QB          : std_logic_vector(15 downto 0);
signal WRAM_QB           : std_logic_vector(7 downto 0);
signal PRAM_QB           : std_logic_vector(7 downto 0);
signal ss_vram_lo        : std_logic_vector(7 downto 0);
signal vram0_b_addr      : std_logic_vector(14 downto 0);
signal vram0_b_data      : std_logic_vector(15 downto 0);
signal vram0_b_we        : std_logic;
signal vram1_b_addr      : std_logic_vector(14 downto 0);
signal vram1_b_data      : std_logic_vector(15 downto 0);
signal vram1_b_we        : std_logic;
signal ram_b_addr        : std_logic_vector(14 downto 0);
signal ram_b_data        : std_logic_vector(7 downto 0);
signal ram_b_we          : std_logic;
signal pram_b_addr       : std_logic_vector(14 downto 0);
signal pram_b_data       : std_logic_vector(7 downto 0);
signal pram_b_we         : std_logic;

component ARCADE_CARD is
	port(
		CLK     : in  std_logic;
		RST_N   : in  std_logic;
		SLEEP   : in  std_logic;
		SaveStateBus_Din : in std_logic_vector(63 downto 0);
		SaveStateBus_Adr : in std_logic_vector(9 downto 0);
		SaveStateBus_wren : in std_logic;
		SaveStateBus_rst : in std_logic;
		SaveStateBus_load : in std_logic;
		SaveStateBus_Dout : out std_logic_vector(63 downto 0);

		EN      : in  std_logic;
		WR_N    : in  std_logic;
		RD_N    : in  std_logic;
		A       : in  std_logic_vector(20 downto 0);
		DI      : in  std_logic_vector(7 downto 0);
		DO      : out std_logic_vector(7 downto 0);

		SEL_N   : out std_logic;

		RAM_CS_N: out std_logic;
		RAM_A   : out std_logic_vector(20 downto 0)
	);
end component;

begin

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

generate_CHEAT: if (LITE = 0) generate begin

-- Game Genie
GAMEGENIE : component CODES
generic map(
	ADDR_WIDTH => 21,
	DATA_WIDTH => 8
)
port map(
	clk => CLK,
	reset => GG_RESET,
	enable => not GG_EN,
	addr_in => CPU_A,
	data_in => CPU_DI,
	code => GG_CODE,
	available => GG_AVAIL,
	genie_ovr => GENIE,
	genie_data => GENIE_DO
);

GENIE_DI <= GENIE_DO when GENIE else CPU_DI;

end generate;

generate_NOCHEAT: if (LITE /= 0) generate begin
	GENIE_DI <= CPU_DI;
	GG_AVAIL <= '0';
end generate;

CPU : entity work.HUC6280
port map(
	CLK 		=> CLK,
	RST_N		=> RESET_N,
	WAIT_N	=> ROM_RDY and not CPU_PAUSE_EN and cd_cpu_wait_n,
	SLEEP		=> ss_sleep_eff,

	IRQ1_N	=> VDC0_IRQ_N and VDC1_IRQ_N,
	IRQ2_N	=> CD_IRQ_N,
	NMI_N		=> '1',

	DI			=> GENIE_DI,
	DO 		=> CPU_DO,

	A 			=> CPU_A,
	WR_N 		=> CPU_WR_N,
	RD_N		=> CPU_RD_N,

	RDY		=> VDC0_BUSY_N and VDC1_BUSY_N,

	CE			=> CPU_CE,
	CEK_N		=> CPU_VCE_SEL_N,
	CE7_N		=> CPU_VDC_SEL_N,
	CER_N		=> CPU_RAM_SEL_N,
	PRE_RD   => CPU_PRE_RD,
	PRE_WR   => CPU_PRE_WR,

	K			=> not CD_EN & "011" & JOY_IN,
	O			=> CPU_IO_DO,

	VDCNUM   => VDCNUM,

	AUD_LDATA=> PCE_SL,
	AUD_RDATA=> PCE_SR,

	-- Savestates
	SaveStateBus_Din  => SaveStateBus_Din,
	SaveStateBus_Adr  => SaveStateBus_Adr,
	SaveStateBus_wren => SaveStateBus_wren,
	SaveStateBus_rst  => SaveStateBus_rst,
	SaveStateBus_load => ss_load_done,
	SaveStateBus_Dout => SS_Dout_CPU,
	SS_WF_Addr        => Save_RAMAddr(7 downto 0),
	SS_WF_WrEn        => ss_wf_we,
	SS_WF_WrData      => Save_RAMWriteData(4 downto 0),
	SS_WF_RdData      => ss_wf_rd,
	SS_IDLE           => ss_bnd_idle
);

ss_wf_we <= Save_RAMWrEn when (sleep_ss = '1' and Save_RAMType = 8) else '0';
ss_ad_we <= Save_RAMWrEn when (sleep_ss = '1' and Save_RAMType = 9) else '0';

JOY_OUT <= CPU_IO_DO(1 downto 0);

CPU_CLKEN <= CPU_CE when rising_edge( CLK );

VIDEO_CE <= VDC_CLKEN;
VIDEO_VS <= not VS_N;
VIDEO_HS <= not HS_N;

VCE : entity work.huc6260
port map(
	CLK 		=> CLK,
	RESET_N	=> RESET_N,
	SLEEP		=> ss_sleep_eff,

	-- CPU Interface
	A			=> CPU_A(2 downto 0),
	CE_N		=> CPU_VCE_SEL_N,
	WR_N		=> CPU_WR_N,
	RD_N		=> CPU_RD_N,
	DI			=> CPU_DO,
	DO 		=> VCE_DO,

	-- VDC Interface
	COLNO		=> VDC_COLNO,
	CLKEN		=> VDC_CLKEN,
	CLKEN_F  => VDC_CLKEN_F,
	HSYNC_F	=> VCE_HSYNC_F,
	HSYNC_R	=> VCE_HSYNC_R,
	VSYNC_F	=> VCE_VSYNC_F,
	VSYNC_R	=> VCE_VSYNC_R,
	CLKEN_FS => VIDEO_CE_FS,
	RVBL		=> ReducedVBL,
	
	GRID_EN	=> GRID_EN,
	BORDER_EN=> BORDER_EN,
	BORDER	=> BORDER,
	GRID		=> GRID,
		
	-- NTSC/RGB Video Output
	R			=> VIDEO_R,
	G			=> VIDEO_G,
	B			=> VIDEO_B,
	BW			=> VIDEO_BW,
	VS_N		=> VS_N,
	HS_N		=> HS_N,
	HBL		=> VIDEO_HBL,
	VBL		=> video_vbl_i,

	-- Savestates
	SaveStateBus_Din  => SaveStateBus_Din,
	SaveStateBus_Adr  => SaveStateBus_Adr,
	SaveStateBus_wren => SaveStateBus_wren,
	SaveStateBus_rst  => SaveStateBus_rst,
	SaveStateBus_load => ss_load_done,
	SaveStateBus_Dout => SS_Dout_VCE,
	SS_PAL_Addr       => Save_RAMAddr(9 downto 0),
	SS_PAL_WrEn       => ss_pal_we,
	SS_PAL_WrData     => Save_RAMWriteData(0) & ss_vram_lo,
	SS_PAL_RdData     => ss_pal_rd
);

-- Palette write: 16-bit assembled via ss_vram_lo latch, strobed on the odd byte.
ss_pal_we <= Save_RAMWrEn and Save_RAMAddr(0) when (sleep_ss = '1' and Save_RAMType = 3) else '0';

VDC0 : entity work.HUC6270
port map(
	CLK 		=> CLK,
	RST_N		=> RESET_N,
	CLR_MEM  => COLD_RESET,

	-- CPU Interface
	CPU_CE	=> CPU_CE,
	BYTEWORD => '1',						-- 8-bit access
	A			=> CPU_A(1 downto 0),
	CS_N		=> CPU_VDC0_SEL_N,
	WR_N		=> CPU_WR_N,
	RD_N		=> CPU_RD_N,
	DI			=> "00000000" & CPU_DO,
	DO 		=> VDC0_DO,
	BUSY_N	=> VDC0_BUSY_N,
	IRQ_N		=> VDC0_IRQ_N,

	-- VCE Interface
	DCK_CE	=> VDC_CLKEN,
	DCK_CE_F => VDC_CLKEN_F,
	HSYNC_F	=> VCE_HSYNC_F,
	HSYNC_R	=> VCE_HSYNC_R,
	VSYNC_F	=> VCE_VSYNC_F,
	VSYNC_R	=> VCE_VSYNC_R,
	VD			=> VDC0_COLNO,
	
	BORDER	=> VDC0_BORDER,
	GRID		=> VDC0_GRID,
	SP64     => SP64,

	RAM_A		=> VRAM0_A,
	RAM_DI	=> VRAM0_DI,
	RAM_DO	=> VRAM0_DO,
	RAM_WE	=> VRAM0_WE,
	
	BG_EN		=> BG_EN,
	SPR_EN	=> SPR_EN,

	-- Savestates (slot base 25 = default generic)
	SaveStateBus_Din  => SaveStateBus_Din,
	SaveStateBus_Adr  => SaveStateBus_Adr,
	SaveStateBus_wren => SaveStateBus_wren,
	SaveStateBus_rst  => SaveStateBus_rst,
	SaveStateBus_load => ss_load_done,
	SaveStateBus_Dout => SS_Dout_VDC0,
	SLEEP             => ss_sleep_eff,
	SS_SAT_Addr       => Save_RAMAddr(8 downto 0),
	SS_SAT_WrEn       => ss_sat0_we,
	SS_SAT_WrData     => Save_RAMWriteData & ss_vram_lo,
	SS_SAT_RdData     => ss_sat0_rd,
	SS_DMA_BUSY       => ss_busy0
);

ss_sat0_we <= Save_RAMWrEn and Save_RAMAddr(0) when (sleep_ss = '1' and Save_RAMType = 2) else '0';

VRAM0 : entity work.dpram generic map (addr_width => 15, data_width => 16, disable_value => '0')
port map (
	clock		=> CLK,
	address_a=> VRAM0_A(14 downto 0),
	data_a	=> VRAM0_DO,
	cs_a		=> not VRAM0_A(15),
	wren_a	=> VRAM0_WE,
	q_a		=> VRAM0_DI,

	address_b=> vram0_b_addr,
	data_b	=> vram0_b_data,
	wren_b	=> vram0_b_we,
	q_b		=> VRAM0_QB
);

-- Port B: cold-reset clear normally; savestate DMA while frozen.
vram0_b_addr <= Save_RAMAddr(15 downto 1)          when sleep_ss = '1' else CLR_A;
vram0_b_data <= Save_RAMWriteData & ss_vram_lo     when sleep_ss = '1' else (others => '0');
vram0_b_we   <= (Save_RAMWrEn and Save_RAMAddr(0)) when (sleep_ss = '1' and Save_RAMType = 1) else
                CLR_WE                             when sleep_ss = '0' else '0';

CLR_A  <= CLR_A + 1  when rising_edge(CLK);
CLR_WE <= COLD_RESET when rising_edge(CLK);

generate_SGX: if (LITE = 0) generate begin

	VDC1 : entity work.HUC6270
	generic map( SS_BASE => SSREG_INDEX_VDC1 )
	port map(
		CLK 		=> CLK,
		CLR_MEM  => COLD_RESET,
		RST_N		=> RESET_N,

		-- CPU Interface
		CPU_CE	=> CPU_CE,
		BYTEWORD => '1',						-- 8-bit access
		A			=> CPU_A(1 downto 0),
		CS_N		=> CPU_VDC1_SEL_N,
		WR_N		=> CPU_WR_N,
		RD_N		=> CPU_RD_N,
		DI			=> "00000000" & CPU_DO,
		DO 		=> VDC1_DO,
		BUSY_N	=> VDC1_BUSY_N,
		IRQ_N		=> VDC1_IRQ_N,

		-- VCE Interface
		DCK_CE	=> VDC_CLKEN,
		DCK_CE_F => VDC_CLKEN_F,
		HSYNC_F	=> VCE_HSYNC_F,
		HSYNC_R	=> VCE_HSYNC_R,
		VSYNC_F	=> VCE_VSYNC_F,
		VSYNC_R	=> VCE_VSYNC_R,
		VD			=> VDC1_COLNO,
		--GRID		=> VDC1_GRID,
		
		SP64     => SP64,
		
		RAM_A		=> VRAM1_A,
		RAM_DI	=> VRAM1_DI,
		RAM_DO	=> VRAM1_DO,
		RAM_WE	=> VRAM1_WE,

		BG_EN		=> BG_EN,
		SPR_EN	=> SPR_EN,

		-- Savestates (slot base 35 = VDC1; LITE=1 build simply omits these
		-- eRegs, the on-disk layout stays identical — slots read back 0)
		SaveStateBus_Din  => SaveStateBus_Din,
		SaveStateBus_Adr  => SaveStateBus_Adr,
		SaveStateBus_wren => SaveStateBus_wren,
		SaveStateBus_rst  => SaveStateBus_rst,
		SaveStateBus_load => ss_load_done,
		SaveStateBus_Dout => SS_Dout_VDC1,
		SLEEP             => ss_sleep_eff,
		SS_SAT_Addr       => Save_RAMAddr(8 downto 0),
		SS_SAT_WrEn       => ss_sat1_we,
		SS_SAT_WrData     => Save_RAMWriteData & ss_vram_lo,
		SS_SAT_RdData     => ss_sat1_rd,
		SS_DMA_BUSY       => ss_busy1
	);

	ss_sat1_we <= Save_RAMWrEn and Save_RAMAddr(0) when (sleep_ss = '1' and Save_RAMType = 5) else '0';

	VRAM1 : entity work.dpram generic map (addr_width => 15, data_width => 16, disable_value => '0')
	port map (
		clock		=> CLK,
		address_a=> VRAM1_A(14 downto 0),
		data_a	=> VRAM1_DO,
		cs_a		=> not VRAM1_A(15),
		wren_a	=> VRAM1_WE and not VRAM1_A(15),
		q_a		=> VRAM1_DI,

		address_b=> vram1_b_addr,
		data_b	=> vram1_b_data,
		wren_b	=> vram1_b_we,
		q_b		=> VRAM1_QB
	);

	-- Port B: cold-reset clear normally; savestate DMA while frozen.
	vram1_b_addr <= Save_RAMAddr(15 downto 1)          when sleep_ss = '1' else CLR_A;
	vram1_b_data <= Save_RAMWriteData & ss_vram_lo     when sleep_ss = '1' else (others => '0');
	vram1_b_we   <= (Save_RAMWrEn and Save_RAMAddr(0)) when (sleep_ss = '1' and Save_RAMType = 4) else
	                CLR_WE                             when sleep_ss = '0' else '0';

	VPC : entity work.huc6202
	port map(
		CLK 		=> CLK,
		CLKEN		=> VDC_CLKEN,
		RESET_N	=> RESET_N,

		-- CPU Interface
		A			=> CPU_A(2 downto 0),
		WR_N		=> CPU_WR_N or CPU_VPC_SEL_N or not CPU_CE,
		DI			=> CPU_DO,
		DO 		=> VPC_DO,
		
		HS_F		=> VCE_HSYNC_F,
		VDC0_IN  => VDC0_COLNO,
		VDC1_IN  => VDC1_COLNO,
		VDC_OUT  => VDC_COLNO,
		
		SGX		=> SGX,

		VDCNUM   => VDCNUM,

		SaveStateBus_Din  => SaveStateBus_Din,
		SaveStateBus_Adr  => SaveStateBus_Adr,
		SaveStateBus_wren => SaveStateBus_wren,
		SaveStateBus_rst  => SaveStateBus_rst,
		SaveStateBus_load => ss_load_done,
		SaveStateBus_Dout => SS_Dout_VPC
	);

	CPU_VDC0_SEL_N <= CPU_VDC_SEL_N or     CPU_A(3) or     CPU_A(4) when SGX = '1' else CPU_VDC_SEL_N;
	CPU_VDC1_SEL_N <= CPU_VDC_SEL_N or     CPU_A(3) or not CPU_A(4) when SGX = '1' else '1';
	CPU_VPC_SEL_N  <= CPU_VDC_SEL_N or not CPU_A(3) or     CPU_A(4) when SGX = '1' else '1';
	
	process( CLK )
	begin
		if rising_edge( CLK ) then
			if VDC_CLKEN = '1' then
				BORDER <= VDC0_BORDER;
				GRID <= VDC0_GRID;
			end if;
		end if;
	end process;

end generate;

generate_NOSGX: if (LITE /= 0) generate begin

	CPU_VDC0_SEL_N <= CPU_VDC_SEL_N;
	CPU_VDC1_SEL_N <= '1';
	CPU_VPC_SEL_N  <= '1';
	VDC1_BUSY_N <= '1';
	VDC1_IRQ_N <= '1';
	VRAM1_QB <= (others => '0');	-- savestate read mux default (no VDC1 in LITE build)
	SS_Dout_VDC1 <= (others => '0');
	SS_Dout_VPC  <= (others => '0');
	ss_sat1_rd   <= (others => '0');
	ss_busy1     <= '0';

	VDCNUM <= '0';
	VDC1_DO <= (others => '1');
	VPC_DO <= (others => '1');
	VDC_COLNO <= VDC0_COLNO;
	
	BORDER <= VDC0_BORDER;
	GRID <= VDC0_GRID;

end generate;

--TODO: check address mirroring for HuCard games
CPU_BRM_SEL_N <= '0' when CPU_A(20 downto 11) = x"F7"&"00" and CD_BRAM_EN = '1' else '1'; -- BRM : Page $F7

CPU_ROM_SEL_N <= CPU_A(20);

-- CPU data bus
CPU_DI <= RAM_DO         when CPU_RAM_SEL_N  = '0'
			else CD_DO      when CD_SEL_N       = '0'
			else CD_RAM_DI  when CD_RAM_CS_N    = '0' or AC_RAM_CS_N = '0'
			else AC_DO      when AC_SEL_N       = '0'
			else BRM_DO     when CPU_BRM_SEL_N  = '0'
			else PRAM_DO    when CPU_PRAM_SEL_N = '0'
			else ROM_DO     when CPU_ROM_SEL_N  = '0'
			else VCE_DO     when CPU_VCE_SEL_N  = '0'
			else VDC0_DO_LO when CPU_VDC0_SEL_N = '0'
			else VDC1_DO_LO when CPU_VDC1_SEL_N = '0'
			else VPC_DO     when CPU_VPC_SEL_N  = '0'
			else X"FF";

-- Perform address mangling to mimic HuCard chip mapping.
-- 384K ROM, split in 3, mapped ABABCCCC
	                                     -- bits 19 downto 16
	-- 00000 -> 20000  => 00000 -> 20000		0000 -> 0000
	-- 20000 -> 40000  => 20000 -> 40000		0010 -> 0010
	-- 40000 -> 60000  => 00000 -> 20000		0100 -> 0000
	-- 60000 -> 80000  => 20000 -> 40000		0110 -> 0010
	-- 80000 -> A0000  => 40000 -> 60000		1000 -> 0100
	-- A0000 -> C0000  => 40000 -> 60000		1010 -> 0100
	-- C0000 -> E0000  => 40000 -> 60000		1100 -> 0100
	-- E0000 ->100000  => 40000 -> 60000		1110 -> 0100

-- 768K ROM, split in 6, mapped ABCDEFEF
				                            -- bits 19 downto 16
	-- 00000 -> 20000  => 00000 -> 20000		0000 -> 0000
	-- 20000 -> 40000  => 20000 -> 40000		0010 -> 0010
	-- 40000 -> 60000  => 40000 -> 60000		0100 -> 0100
	-- 60000 -> 80000  => 60000 -> 80000		0110 -> 0110
	-- 80000 -> A0000  => 80000 -> A0000		1000 -> 1000
	-- A0000 -> C0000  => A0000 -> C0000		1010 -> 1010
	-- C0000 -> E0000  => 80000 -> A0000		1100 -> 1000
	-- E0000 ->100000  => A0000 -> C0000		1110 -> 1010

--2560K ROM, ABCDEFGH, ABCDIJKL, ABCDMNOP, ABCDQRST = SF2
                                      -- bits 21 downto 19 (bank)
	-- 00000 -> 80000 XX => 00000 -> 80000		0 XX -> 000
	-- 80000 ->100000 00 => 80000 ->100000		1 00 -> 001
	-- 80000 ->100000 01 =>100000 ->180000		1 01 -> 010
	-- 80000 ->100000 10 =>180000 ->200000		1 10 -> 011
	-- 80000 ->100000 11 =>200000 ->280000		1 11 -> 100

-- 128K ROM, mapped AAAAAAAA -> simple repeat
-- 256K ROM, mapped ABABABAB -> simple repeat
-- 512K ROM, mapped ABCDABCD -> simple repeat
-- 1MB and others            -> Straight mapping

ROM_A <=   "00000"&CPU_A(16 downto 0)                                       when rom_sz = X"020" -- 128K
      else "0000"&CPU_A(17 downto 0)                                        when rom_sz = X"040" -- 256K
      else "000"&CPU_A(19)&(CPU_A(17) and not CPU_A(19))&CPU_A(16 downto 0) when rom_sz = X"060" -- 384K
      else "000"&CPU_A(18 downto 0)                                         when rom_sz = X"080" -- 512K
      else "00" &CPU_A(19)&(CPU_A(18) and not CPU_A(19))&CPU_A(17 downto 0) when rom_sz = X"0C0" -- 768K
      else (CPU_A(19) and (rombank(0) and rombank(1)))
          &(CPU_A(19) and (rombank(0) xor rombank(1)))
          &(CPU_A(19) and not rombank(0))&CPU_A(18 downto 0)                when rom_sz = X"280" -- SF2
      else "00"&CPU_A(19 downto 0);                                                             -- 1MB and others

-- ss_cdr_act: the region-10 walk owns the SDRAM read port; the CPU is
-- frozen but CPU_PRE_RD is a LEVEL - a fetch pending at freeze time would
-- otherwise steal every walk beat (rom_rd has raddr priority) and the
-- whole CD-RAM region saved as stale ROM data.  The post-save/load
-- refetch re-issues the suppressed fetch.
ROM_RD    <= CPU_PRE_RD and not CPU_ROM_SEL_N and CPU_PRAM_SEL_N and ((AC_RAM_CS_N and CD_RAM_CS_N) or not CD_EN)
             and not ss_cdr_act;
ROM_CLKEN <= CPU_CLKEN or ss_kick or ss_cdr_kick;	-- ss_kick: post-load refetch; ss_cdr_kick: region-10 walk pacing

process( CLK ) begin
	if rising_edge( CLK ) then
		if RESET = '1' or ss_reset = '1' then
			-- T65-style reset-branch restore: on a plain reset SS_TOP holds the
			-- eReg default ("00"); on a savestate load it holds the loaded value.
			rombank <= SS_TOP(1 downto 0);
		elsif CPU_CE = '1' then
			-- CPU_A(12 downto 2) = X"7FC" means CPU_A & 0x1FFC = 0x1FF0
			if CPU_A(20) = '0' and ('0' & CPU_A(12 downto 2)) = X"7FC" and CPU_WR_N = '0' then
				rombank <= CPU_A(1 downto 0);
			end if;
		end if;
		RESET_N <= not (RESET or ss_reset);
	end if;
end process;

PRAM : entity work.dpram generic map (15,8)
port map (
	clock		=> CLK,
	address_a=> CPU_A(14 downto 0),
	data_a	=> CPU_DO,
	wren_a	=> CPU_CE and not CPU_PRAM_SEL_N and not CPU_WR_N,
	q_a		=> PRAM_DO,

	address_b=> pram_b_addr,
	data_b	=> pram_b_data,
	wren_b	=> pram_b_we,
	q_b		=> PRAM_QB
);

pram_b_addr <= Save_RAMAddr(14 downto 0) when sleep_ss = '1' else CLR_A;
pram_b_data <= Save_RAMWriteData         when sleep_ss = '1' else (others => '0');
pram_b_we   <= Save_RAMWrEn when (sleep_ss = '1' and Save_RAMType = 6) else
               CLR_WE       when sleep_ss = '0' else '0';

CPU_PRAM_SEL_N <= CPU_A(20) or not CPU_A(19) or not ROM_POP;


RAM : entity work.dpram generic map (15,8)
port map (
	clock		=> CLK,
	address_a=> RAM_A(14 downto 0),
	data_a	=> CPU_DO,
	wren_a	=> CPU_CE and not CPU_RAM_SEL_N and not CPU_WR_N,
	q_a		=> RAM_DO,

	address_b=> ram_b_addr,
	data_b	=> ram_b_data,
	wren_b	=> ram_b_we,
	q_b		=> WRAM_QB
);

ram_b_addr <= Save_RAMAddr(14 downto 0) when sleep_ss = '1' else CLR_A;
ram_b_data <= Save_RAMWriteData         when sleep_ss = '1' else (others => '0');
ram_b_we   <= Save_RAMWrEn when (sleep_ss = '1' and Save_RAMType = 0) else
              CLR_WE       when sleep_ss = '0' else '0';

RAM_A(12 downto 0)  <= CPU_A(12 downto 0);
RAM_A(14 downto 13) <= CPU_A(14 downto 13) when SGX = '1' else "00";

-- Backup RAM
-- The frozen CPU's BRAM port serves the byte-wise savestate walk.
-- SD backup retains port B.
BRM_A  <= Save_RAMAddr(10 downto 0) when ss_sleep_eff = '1' else CPU_A(10 downto 0);
BRM_DI <= Save_RAMWriteData         when ss_sleep_eff = '1' else CPU_DO;
BRM_WE <= Save_RAMWrEn when (ss_sleep_eff = '1' and Save_RAMType = 7) else
          (CPU_CE and not CPU_BRM_SEL_N and not CPU_WR_N) when ss_sleep_eff = '0' else '0';


CD : entity work.cd
port map(
	CLK 			=> CLK,
	RST_N			=> RESET_N,
	EN				=> '1',

	EXT_A			=> CPU_A,
	EXT_DI		=> CPU_DO,
	EXT_DO		=> CD_DO,
	EXT_WR_N		=> CPU_WR_N,
	EXT_RD_N		=> CPU_RD_N,
	CPU_CE		=> CPU_CE,
	CPU_WAIT_N	=> cd_cpu_wait_n,
	
	RAM_CS_N		=> CD_RAM_CS_N,
	BRAM_EN		=> CD_BRAM_EN,
	
	SEL_N			=> CD_SEL_N,
	IRQ_N			=> CD_IRQ_N,
	
	CD_STAT		=> CD_STAT,
	CD_MSG		=> CD_MSG,
	CD_STAT_GET	=> CD_STAT_GET,
	CD_COMM		=> CD_COMM,
	CD_COMM_SEND=> CD_COMM_SEND,
	CD_DOUT_REQ	=> CD_DOUT_REQ,
	CD_DOUT		=> CD_DOUT,
	CD_DOUT_SEND=> CD_DOUT_SEND,
	
	CD_DATA		=> CD_DATA,
	CD_DATA_WR	=> CD_DATA_WR,
	CD_AUDIO_WR	=> CD_AUDIO_WR,
	CD_SUBCD_WR	=> CD_SUBCD_WR,
	CD_DATA_END	=> CD_DATA_END,
	
	CD_REGION   => CD_REGION,
	CD_RESET		=> CD_RESET,
	CD_RESET_DONE => CD_RESET_DONE,
	CD_FAST_SEEK => CD_FAST_SEEK,
	
	DM				=> CD_DM,
	
	SLEEP				=> ss_sleep_eff,
	SaveStateBus_Din  => SaveStateBus_Din,
	SaveStateBus_Adr  => SaveStateBus_Adr,
	SaveStateBus_wren => SaveStateBus_wren,
	SaveStateBus_rst  => SaveStateBus_rst,
	SaveStateBus_load => ss_load_done,
	SaveStateBus_Dout => SS_Dout_CD,
	SS_AD_Addr        => Save_RAMAddr(15 downto 0),
	SS_AD_RdEn        => Save_RAMRdEn,
	SS_AD_WrEn        => ss_ad_we,
	SS_AD_WrData      => Save_RAMWriteData,
	SS_AD_RdData      => ss_ad_rd,
	SS_CD_HOLD        => ss_cd_hold,
	SS_AUDIO_HOLD     => ss_audio_hold,
	
	CD_SL			=> CDDA_SL,
	CD_SR			=> CDDA_SR,
	AD_S			=> ADPCM_S
);

-- Frozen region 10/11 walks own this port, never concurrent CPU accesses.
CD_RAM_A  <= ss_cdr_addr when ss_cdr_act = '1' else
             '0' & AC_RAM_A when AC_RAM_CS_N = '0' else "1000" & CPU_A(17 downto 0);
CD_RAM_DO <= ss_cdr_wdata when ss_cdr_act = '1' else CPU_DO;
CD_RAM_RD <= ss_cdr_rd when ss_cdr_act = '1' else
             (CPU_PRE_RD and not (CD_RAM_CS_N and AC_RAM_CS_N));
CD_RAM_WR <= ss_cdr_wr when ss_cdr_act = '1' else
             (CPU_PRE_WR and not (CD_RAM_CS_N and AC_RAM_CS_N));

SS_EXT_RAM : entity work.ss_external_ram
port map (
	clk => CLK, sleep_ss => sleep_ss, frozen => ss_frozen, saving => ss_saving,
	region => Save_RAMType, walk_addr => Save_RAMAddr,
	walk_wr => Save_RAMWrEn, walk_data => Save_RAMWriteData,
	walk_q => ss_cdr_data, walk_ready => ss_ram_rdy, active => ss_cdr_act,
	ram_addr => ss_cdr_addr, ram_data => ss_cdr_wdata, ram_q => CD_RAM_DI,
	ram_ready => ROM_RDY, ram_rd => ss_cdr_rd, ram_wr => ss_cdr_wr,
	kick => ss_cdr_kick, debug_no_dip => ss_cdr_dbg_ns,
	debug_timeout => ss_cdr_dbg_to, debug_max => ss_cdr_dbg_max
);

AC : ARCADE_CARD
port map(
	CLK     => CLK,
	RST_N   => RESET_N,
	SLEEP   => ss_sleep_eff,
	SaveStateBus_Din => SaveStateBus_Din,
	SaveStateBus_Adr => SaveStateBus_Adr,
	SaveStateBus_wren => SaveStateBus_wren,
	SaveStateBus_rst => SaveStateBus_rst,
	SaveStateBus_load => ss_load_done,
	SaveStateBus_Dout => SS_Dout_AC,

	EN      => CD_EN and AC_EN,
	WR_N    => CPU_WR_N,
	RD_N    => CPU_RD_N,
	A       => CPU_A,
	DI      => CPU_DO,
	DO      => AC_DO,
	SEL_N   => AC_SEL_N,

	RAM_CS_N=> AC_RAM_CS_N,
	RAM_A   => AC_RAM_A
);

PSG_SR <= signed(PCE_SR(23 downto 8));
PSG_SL <= signed(PCE_SL(23 downto 8));

--------------------------------------------------------------------------------
-- Savestates (Robert Peip framework)
--------------------------------------------------------------------------------

SS_SLEEP <= ss_walk_sleep;
SS_BUSY  <= ss_busy_i;
SS_FAILED <= ss_save_failed;
SS_ACTIVE_SLOT <= std_logic_vector(to_unsigned(ss_request_slot, 2));
process(CLK) begin
	if rising_edge(CLK) then
		if ss_req_save = '1' then
			SS_ACTIVE_LOAD <= '0';
		elsif ss_req_load = '1' then
			SS_ACTIVE_LOAD <= '1';
		end if;
	end if;
end process;
VIDEO_VBL <= video_vbl_i;

-- Freeze between CPU instructions and CE pulses, inside VBLANK, with both
-- VDC DMA engines idle and neither external nor CD ownership holding.
-- Posted VDC transactions need not drain: their address, data and handshake
-- state are serialized and complete after restore.
ss_boundary <= '1' when ss_bnd_idle = '1' and CPU_CE = '0'
                    and video_vbl_i = '1'
                    and ss_busy0 = '0' and ss_busy1 = '0'
                    and SS_EXT_HOLD = '0'
                    and ss_cd_hold = '0'
               else '0';

-- Wait for the safe boundary before gating CEs and acknowledging the engine.
-- After 2^27 clocks, force the freeze; a forced save aborts without writing.
process( CLK ) begin
	if rising_edge( CLK ) then
		if sleep_ss = '0' then
			ss_frozen <= '0';
			ss_wd <= (others => '0');
		elsif ss_frozen = '0' then
			-- The machine continues running while the boundary is unavailable.
			ss_wd <= ss_wd + 1;
			-- snapshot WHICH terms block ('1' = this term is the blocker)
			-- bit 7 = CPU not idle; bit 6 retired with SS_RES_INT, kept at 0 so
			-- the other blockers keep their eReg 47 bit positions
			ss_dbg_block <= (not ss_bnd_idle) & '0' & CPU_CE
			              & (not video_vbl_i)
			              & ss_busy0 & ss_busy1
			              & SS_EXT_HOLD & ss_cd_hold;
			if ss_boundary = '1' or ss_wd(27) = '1' then
				ss_frozen <= '1';
				ss_forced <= ss_wd(27);	-- watchdog-forced (non-boundary) freeze
			end if;
		end if;
		if sleep_ss = '0' then
			ss_forced <= '0';
		end if;
		ss_paused <= ss_frozen;
	end if;
end process;

-- Refetch ROM data after PC/MPR restore or a shared-port memory walk.
-- Keep the CPU frozen while read kicks refresh the data register; ROM_RDY
-- may still describe the old address. Allow startup latency before checking
-- readiness, with a 255-cycle limit.
process( CLK ) begin
	if rising_edge( CLK ) then
		ss_sleep_d  <= sleep_ss;
		ss_saving_d <= ss_saving;
		-- Both load and save can replace the shared ROM/CD-RAM data register.
		-- Delay saving_savestate because it falls with sleep_ss.
		if ss_load_done = '1' or (ss_sleep_d = '1' and sleep_ss = '0' and ss_saving_d = '1') then
			ss_refetch <= '1';
			ss_refetch_cnt <= (others => '0');
		elsif ss_refetch = '1' then
			ss_refetch_cnt <= ss_refetch_cnt + 1;
			if (ss_refetch_cnt > 40 and ROM_RDY = '1') or ss_refetch_cnt = 255 then
				ss_refetch <= '0';
			end if;
		end if;
	end if;
end process;

ss_kick <= '1' when ss_refetch = '1' and ss_refetch_cnt(3 downto 0) = "1000" else '0';

-- SLEEP stays asserted seamlessly through restore (ss_load_done) and the
-- ROM refetch window, so the restored registers are held until the fetch
-- path is fresh.
ss_walk_sleep <= (sleep_ss and ss_frozen) or ss_load_done or ss_refetch;
-- After a load that resumes CDDA the CD unit extends the freeze until the
-- saved sample is ready at its DAC; the refetch window covers the two
-- clocks the hold needs to arm. The hold is not exported: Main must keep
-- delivering CD data while it lasts.
ss_sleep_eff <= ss_walk_sleep or ss_audio_hold;

-- Abort forced saves before the slot header is touched. Do not gate this
-- with saving_savestate: that signal rises only after the abort decision.
-- Load states do not consume save_abort.
ss_save_abort <= ss_forced;

SSMANAGER : entity work.statemanager
generic map (
	Softmap_SaveState_ADDR => 16#3800000#,	-- DWORD addr = byte 0x0E000000 in the 0x30000000 window = DDR3 0x3E000000
	Softmap_Rewind_ADDR    => 16#3800000#	-- rewind unused
)
port map (
	clk               => CLK,
	reset             => RESET or SS_INHIBIT,
	rewind_on         => '0',
	rewind_active     => '0',
	savestate_number  => to_integer(unsigned(SS_SLOT)),
	save              => SS_SAVE,
	load              => SS_LOAD,
	sleep_rewind      => open,
	vsync             => not VS_N,
	request_savestate => ss_req_save,
	request_loadstate => ss_req_load,
	request_address   => ss_addr_int,
	request_slot      => ss_request_slot,
	request_busy      => ss_busy_i
);

SSENGINE : entity work.savestates
generic map ( CDRAM_BYTES => SS_CDRAM_BYTES )
port map (
	clk                   => CLK,
	reset_in              => RESET,
	reset_ss              => ss_reset,
	reset_delay           => open,
	load_done             => ss_load_done,
	arcade_used           => ss_ac_used_r,
	arcade_loaded         => ss_arcade_loaded,
	increaseSSHeaderCount => SS_TO_SD,
	save                  => ss_req_save,
	save_abort            => ss_save_abort,
	inhibit               => SS_INHIBIT,
	load                  => ss_req_load,
	savestate_address     => ss_addr_int,
	savestate_busy        => ss_busy_i,
	save_failed           => ss_save_failed,
	paused                => ss_paused,
	BUS_Din               => SaveStateBus_Din,
	BUS_Adr               => SaveStateBus_Adr,
	BUS_wren              => SaveStateBus_wren,
	BUS_rst               => SaveStateBus_rst,
	BUS_Dout              => SaveStateBus_Dout,
	loading_savestate     => open,
	saving_savestate      => ss_saving,
	sleep_savestate       => sleep_ss,
	Save_RAMAddr          => Save_RAMAddr,
	Save_RAMRdEn          => Save_RAMRdEn,
	Save_RAMWrEn          => Save_RAMWrEn,
	Save_RAMWriteData     => Save_RAMWriteData,
	Save_RAMReadData      => Save_RAMReadData,
	Save_RAMRdy           => ss_ram_rdy,
	Save_RAMType          => Save_RAMType,
	bus_out_Din           => SS_DDR_DIN,
	bus_out_Dout          => SS_DDR_DOUT,
	bus_out_Adr           => SS_DDR_ADDR,
	bus_out_rnw           => SS_DDR_RNW,
	bus_out_ena           => SS_DDR_ENA,
	bus_out_be            => SS_DDR_BE,
	bus_out_done          => SS_DDR_DONE
);

-- SF2 ROM bank state.
iREG_SS_TOP : entity work.eReg_SavestateV
generic map ( Adr => SSREG_INDEX_TOP, def => SSREG_DEFAULT_TOP )
port map (
	clk      => CLK,
	BUS_Din  => SaveStateBus_Din,
	BUS_Adr  => SaveStateBus_Adr,
	BUS_wren => SaveStateBus_wren,
	BUS_rst  => SaveStateBus_rst,
	BUS_Dout => SS_Dout_TOP,
	Din      => SS_TOP_BACK,
	Dout     => SS_TOP
);

-- Save-only region-10 diagnostics: bits 15:0 count reads without a ROM_RDY
-- dip, 23:16 count 255-clock timeouts, 31:24 hold the longest byte latency.
iREG_SS_CDRDBG : entity work.eReg_SavestateV
generic map ( Adr => 67, def => (63 downto 0 => '0') )
port map (
	clk      => CLK,
	BUS_Din  => SaveStateBus_Din,
	BUS_Adr  => SaveStateBus_Adr,
	BUS_wren => SaveStateBus_wren,
	BUS_rst  => SaveStateBus_rst,
	BUS_Dout => SS_Dout_CDRDBG,
	Din      => x"00000000" & std_logic_vector(ss_cdr_dbg_max)
	          & std_logic_vector(ss_cdr_dbg_to) & std_logic_vector(ss_cdr_dbg_ns),
	Dout     => open
);

-- Wired-OR of all module BUS_Dout contributors (eReg drives 0 when not addressed).
SaveStateBus_Dout <= SS_Dout_TOP or SS_Dout_VCE or SS_Dout_CPU
                  or SS_Dout_VDC0 or SS_Dout_VDC1 or SS_Dout_VPC
                  or SS_Dout_CD or SS_Dout_AC or SSE_Dout or SS_Dout_CDRDBG;

-- SaveStateBus export towards TurboGrafx16.sv (TOP_EXT input-block eReg)
SSE_Din  <= SaveStateBus_Din;
SS_DBG   <= ss_forced & ss_dbg_block;

-- Capture Arcade Card state once the game writes a register or accesses RAM.
-- Restore the saved usage flag so subsequent saves retain the same regions.
process( CLK ) begin
	if rising_edge( CLK ) then
		if ss_load_done = '1' then
			ss_ac_used_r <= ss_arcade_loaded;
		elsif RESET = '1' then
			ss_ac_used_r <= '0';
		elsif ss_sleep_eff = '0' and
		      ((AC_SEL_N = '0' and CPU_WR_N = '0') or AC_RAM_CS_N = '0') then
			-- register WRITES or any AC-RAM access; plain reads excluded
			-- (many CD games probe the AC ident register at boot)
			ss_ac_used_r <= '1';
		end if;
	end if;
end process;
SS_AC_USED <= ss_ac_used_r;
SSE_Adr  <= SaveStateBus_Adr;
SSE_wren <= SaveStateBus_wren;
SSE_rst  <= SaveStateBus_rst;
SSE_load <= ss_load_done;

SS_TOP_BACK(63 downto 2) <= (others => '0');
SS_TOP_BACK(1 downto 0)  <= rombank;

-- Byte assembler for 16-bit RAMs (VRAM0/VRAM1): even byte latched, word
-- written on the odd byte (little-endian within the .ss blob).
process( CLK ) begin
	if rising_edge( CLK ) then
		if Save_RAMWrEn = '1' and Save_RAMAddr(0) = '0' then
			ss_vram_lo <= Save_RAMWriteData;
		end if;
	end if;
end process;

-- Region read mux (Save_RAMType per savestates.vhd table).
process( Save_RAMType, Save_RAMAddr, WRAM_QB, VRAM0_QB, VRAM1_QB, PRAM_QB, ss_pal_rd, ss_wf_rd, ss_sat0_rd, ss_sat1_rd, BRM_DO, ss_ad_rd, ss_cdr_data ) begin
	case to_integer(Save_RAMType) is
		when 0      => Save_RAMReadData <= WRAM_QB;
		when 1      => if Save_RAMAddr(0) = '1' then Save_RAMReadData <= VRAM0_QB(15 downto 8);
		               else                          Save_RAMReadData <= VRAM0_QB(7 downto 0); end if;
		when 2      => if Save_RAMAddr(0) = '1' then Save_RAMReadData <= ss_sat0_rd(15 downto 8);
		               else                          Save_RAMReadData <= ss_sat0_rd(7 downto 0); end if;
		when 3      => if Save_RAMAddr(0) = '1' then Save_RAMReadData <= "0000000" & ss_pal_rd(8);
		               else                          Save_RAMReadData <= ss_pal_rd(7 downto 0); end if;
		when 4      => if Save_RAMAddr(0) = '1' then Save_RAMReadData <= VRAM1_QB(15 downto 8);
		               else                          Save_RAMReadData <= VRAM1_QB(7 downto 0); end if;
		when 5      => if Save_RAMAddr(0) = '1' then Save_RAMReadData <= ss_sat1_rd(15 downto 8);
		               else                          Save_RAMReadData <= ss_sat1_rd(7 downto 0); end if;
		when 6      => Save_RAMReadData <= PRAM_QB;
		when 7      => Save_RAMReadData <= BRM_DO;
		when 8      => Save_RAMReadData <= "000" & ss_wf_rd;
		when 9      => Save_RAMReadData <= ss_ad_rd;
		when 10 | 11 => Save_RAMReadData <= ss_cdr_data;
		when others => Save_RAMReadData <= x"00";
	end case;
end process;

end rtl;
