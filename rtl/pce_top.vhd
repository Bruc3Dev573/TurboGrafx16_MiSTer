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

		-- Savestates (Robert Peip framework — see docs/SAVESTATE_IMPLEMENTATION_PLAN.md)
		SS_SAVE		: in  std_logic := '0';						-- save request pulse (savestate_ui)
		SS_LOAD		: in  std_logic := '0';						-- load request pulse
		SS_SLOT		: in  std_logic_vector(1 downto 0) := "00";
		SS_BUSY		: out std_logic;
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
		-- (e.g. MB128 serial transfer in flight, plan §5.9 option 1)
		SS_EXT_HOLD	: in  std_logic := '0';
		-- SaveStateBus export for SystemVerilog-side eRegs (TOP_EXT, plan §5.6)
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
signal ss_reset          : std_logic;
signal ss_load_done      : std_logic;
signal ss_req_save       : std_logic;
signal ss_req_load       : std_logic;
signal ss_addr_int       : integer;
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
signal ss_busy0          : std_logic_vector(3 downto 0);
signal ss_busy1          : std_logic_vector(3 downto 0);
signal SS_Dout_CD        : std_logic_vector(63 downto 0);
signal ss_ad_rd          : std_logic_vector(7 downto 0);
signal ss_ad_we          : std_logic;
signal ss_cd_hold        : std_logic;
signal ss_bnd_state0     : std_logic;
signal ss_bnd_res_int    : std_logic;
signal video_vbl_i       : std_logic;
signal ss_boundary       : std_logic;
signal ss_frozen         : std_logic := '0';
signal ss_wd             : unsigned(27 downto 0) := (others => '0');
signal ss_sleep_eff      : std_logic;
signal ss_saving         : std_logic;
signal ss_forced         : std_logic := '0';
signal ss_dbg_block      : std_logic_vector(7 downto 0) := (others => '0');
signal ss_ac_used_r      : std_logic := '0';
signal ss_save_abort     : std_logic;
signal ss_refetch        : std_logic := '0';
signal ss_refetch_cnt    : unsigned(7 downto 0) := (others => '0');
signal ss_sleep_d        : std_logic := '0';
signal ss_saving_d       : std_logic := '0';
signal ss_kick           : std_logic;
-- P5d: CD/SCD work-RAM walk (region 10) - borrows the CD_RAM SDRAM port
-- during the freeze; each byte gets a 22-CLK window with its own clkref
-- kick (the SDRAM is otherwise idle while the core sleeps).
signal ss_cdr_act        : std_logic;
signal ss_cdr_cnt        : unsigned(7 downto 0);
signal ss_cdr_addr       : std_logic_vector(17 downto 0);
signal ss_cdr_prev       : std_logic_vector(24 downto 0);
signal ss_cdr_data       : std_logic_vector(7 downto 0);
signal ss_cdr_wdata      : std_logic_vector(7 downto 0);
signal ss_cdr_rd         : std_logic;
signal ss_cdr_wr         : std_logic;
signal ss_cdr_kick       : std_logic;
signal ss_cdr_rdy        : std_logic;
signal ss_cdr_busy       : std_logic;
signal ss_cdr_seen       : std_logic;	-- ROM_RDY dipped after the kick (forensic only)
signal ss_cdr_dbg_ns     : unsigned(15 downto 0) := (others => '0');	-- reads sampled with no rdy dip seen
signal ss_cdr_dbg_to     : unsigned(7 downto 0) := (others => '0');	-- 255-CLK timeouts
signal ss_cdr_dbg_max    : unsigned(7 downto 0) := (others => '0');	-- slowest byte (cnt at sample)
signal SS_Dout_CDRDBG    : std_logic_vector(63 downto 0);
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
	WAIT_N	=> ROM_RDY and not CPU_PAUSE_EN,
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
	SS_STATE0         => ss_bnd_state0,
	SS_RES_INT        => ss_bnd_res_int
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
	SS_SLEEP          => ss_sleep_eff,
	SS_SAT_Addr       => Save_RAMAddr(8 downto 0),
	SS_SAT_WrEn       => ss_sat0_we,
	SS_SAT_WrData     => Save_RAMWriteData & ss_vram_lo,
	SS_SAT_RdData     => ss_sat0_rd,
	SS_BUSY_FLAGS     => ss_busy0
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

-- Port B: cold-reset clear normally; savestate DMA while frozen (plan §5.3).
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
		SS_SLEEP          => ss_sleep_eff,
		SS_SAT_Addr       => Save_RAMAddr(8 downto 0),
		SS_SAT_WrEn       => ss_sat1_we,
		SS_SAT_WrData     => Save_RAMWriteData & ss_vram_lo,
		SS_SAT_RdData     => ss_sat1_rd,
		SS_BUSY_FLAGS     => ss_busy1
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

	-- Port B: cold-reset clear normally; savestate DMA while frozen (plan §5.3).
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
	ss_busy1     <= (others => '0');

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
-- BRM: during the savestate freeze the CPU side is quiescent — borrow the
-- core-facing port for the SAVETYPE_BRM walk (plan §5.9; the SD/backup side
-- keeps its own port B).  8-bit region: byte-wise, no assembler needed.
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
	
	DM				=> CD_DM,
	
	SLEEP				=> ss_sleep_eff,
	SS_PEND			=> sleep_ss,
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

	CD_SL			=> CDDA_SL,
	CD_SR			=> CDDA_SR,
	AD_S			=> ADPCM_S
);

-- P5d: the savestate walk borrows the port while the core sleeps
CD_RAM_A  <= "1000" & ss_cdr_addr when ss_cdr_act = '1' else
             '0' & AC_RAM_A when AC_RAM_CS_N = '0' else "1000" & CPU_A(17 downto 0);
CD_RAM_DO <= ss_cdr_wdata when ss_cdr_act = '1' else CPU_DO;
CD_RAM_RD <= ss_cdr_rd when ss_cdr_act = '1' else
             (CPU_PRE_RD and not (CD_RAM_CS_N and AC_RAM_CS_N));
CD_RAM_WR <= ss_cdr_wr when ss_cdr_act = '1' else
             (CPU_PRE_WR and not (CD_RAM_CS_N and AC_RAM_CS_N));

-- Gated on the FROZEN walk window (sleep_ss AND ss_frozen), not on the
-- raw sleep request: sleep_ss rises the moment a save is REQUESTED,
-- while the core keeps running until the boundary lands - and
-- Save_RAMType still holds 10 from the previous walk, so the borrow
-- would hijack the CD-RAM port from the RUNNING CPU and crash the game
-- before the save even landed (the on-hardware 'freezes on save' bug).
-- ss_sleep_eff is equally wrong on the other side: it stays up through
-- the post-load refetch, blocking the CPU's first CD-RAM fetch.
ss_cdr_act <= '1' when sleep_ss = '1' and ss_frozen = '1' and Save_RAMType = 10 else '0';
ss_ram_rdy <= ss_cdr_rdy when Save_RAMType = 10 else '1';

-- Region-10 byte engine: one 22-CLK window per byte.  Reads: triggered by a
-- walk address change (the engine polls Save_RAMRdy every 8 CLK and samples
-- ss_cdr_data once ready).  Writes: triggered by the engine's Save_RAMWrEn
-- byte strobe; Rdy gates the next byte until the write went out.  The kick
-- gives the external SDRAM controller its clkref edge; by CLK 21 the
-- round-trip (a few 10.7 MHz clkref periods) has long completed.
process( CLK )
begin
	if rising_edge( CLK ) then
		ss_cdr_kick <= '0';
		if ss_cdr_act = '0' then
			ss_cdr_prev <= (others => '1');
			ss_cdr_rd   <= '0';
			ss_cdr_wr   <= '0';
			ss_cdr_busy <= '0';
			ss_cdr_rdy  <= '0';
			ss_cdr_seen <= '0';
		else
			if ss_cdr_busy = '1' then
				ss_cdr_cnt <= ss_cdr_cnt + 1;
				-- EMPIRICAL, hardware-proven pacing (p18): blind re-kick every
				-- 16 CLK, sample at cnt>=21 with ROM_RDY high.  Two attempts
				-- to make this "smarter" both re-broke the lane-0 echo
				-- corruption on hardware (p19: rdy-low-only re-kick, 64/64;
				-- p20: rdy-dip 'seen' flag, still 64/64) while the blind
				-- re-kick measures clean (12/64 = chance floor, 30/30 soak).
				-- Some pipeline in the shared read path is still unmodeled -
				-- ss_cdr_seen stays as a forensic tap (eReg 67), not a gate.
				-- Cost: the extra kick re-issues served accesses and the walk
				-- runs slower than the pre-p16 fixed count; scenario A's
				-- wall-clock calibration reads that as a determinism diff
				-- (known signature: constant few-cycle offset, documented in
				-- HANDOFF).  Correctness beats the calibration.
				if ROM_RDY = '0' then
					ss_cdr_seen <= '1';
				end if;
				if ss_cdr_cnt(3 downto 0) = "0010" then
					ss_cdr_kick <= '1';
				end if;
				if ss_cdr_cnt >= 21 and (ROM_RDY = '1' or ss_cdr_cnt = 255) then
					if ss_cdr_rd = '1' then
						ss_cdr_data <= CD_RAM_DI;
					end if;
					-- forensic tallies (eReg 67): how this byte completed
					if ss_cdr_rd = '1' and ss_cdr_seen = '0' and ss_cdr_dbg_ns /= x"FFFF" then
						ss_cdr_dbg_ns <= ss_cdr_dbg_ns + 1;	-- sampled without ever seeing rdy dip
					end if;
					if ss_cdr_cnt = 255 and ss_cdr_dbg_to /= x"FF" then
						ss_cdr_dbg_to <= ss_cdr_dbg_to + 1;	-- timeout backstop hit
					end if;
					if ss_cdr_cnt > ss_cdr_dbg_max then
						ss_cdr_dbg_max <= ss_cdr_cnt;	-- slowest byte
					end if;
					ss_cdr_rd   <= '0';
					ss_cdr_wr   <= '0';
					ss_cdr_busy <= '0';
					ss_cdr_rdy  <= '1';
				end if;
			elsif ss_saving = '1' and Save_RAMAddr /= ss_cdr_prev then
				ss_cdr_prev <= Save_RAMAddr;
				ss_cdr_addr <= Save_RAMAddr(17 downto 0);
				ss_cdr_rd   <= '1';
				ss_cdr_cnt  <= (others => '0');
				ss_cdr_busy <= '1';
				ss_cdr_rdy  <= '0';
				ss_cdr_seen <= '0';
			elsif Save_RAMWrEn = '1' then
				ss_cdr_addr  <= Save_RAMAddr(17 downto 0);
				ss_cdr_wdata <= Save_RAMWriteData;
				ss_cdr_wr    <= '1';
				ss_cdr_cnt   <= (others => '0');
				ss_cdr_busy  <= '1';
				ss_cdr_rdy   <= '0';
				ss_cdr_seen  <= '0';
			else
				ss_cdr_rdy <= '1';
			end if;
		end if;
	end if;
end process;

AC : ARCADE_CARD
port map(
	CLK     => CLK,
	RST_N   => RESET_N,

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
-- SAVESTATES (Robert Peip framework — plan: docs/SAVESTATE_IMPLEMENTATION_PLAN.md)
--------------------------------------------------------------------------------

SS_SLEEP <= ss_sleep_eff;
SS_BUSY  <= ss_busy_i;
VIDEO_VBL <= video_vbl_i;

-- Composite safe boundary (plan §2.3, amended on SGX hardware evidence):
-- CPU at instruction boundary (STATE=0, no reset vector in progress, between
-- CE pulses), inside VBLANK, no VDC-VRAM/SATB DMA executing on either VDC,
-- CD excluded from scope (menu is hidden with CD mounted anyway).
--
-- The CPUWR_PEND/CPURD_PEND terms were REMOVED from the gate: SGX titles
-- (1941) stream posted writes to both VDCs through VBLANK too, so the full
-- conjunction never occurred within the watchdog window and saves were
-- silently watchdog-forced MID-INSTRUCTION (blob captured with STATE=1 —
-- corrupt by design, the game wedged on load).  Dropping PEND is safe by
-- construction: the whole posted FSM (PEND/PEND2/EXEC + CPU_VRAM_ADDR/DATA)
-- is saved and restored verbatim in the HSK slot, which per plan §7.3 is
-- what actually guarantees handshake coherence; an in-flight transaction
-- simply completes after the restore.
ss_boundary <= '1' when ss_bnd_state0 = '1' and ss_bnd_res_int = '0' and CPU_CE = '0'
                    and video_vbl_i = '1'
                    and ss_busy0(1 downto 0) = "00" and ss_busy1(1 downto 0) = "00"
                    and SS_EXT_HOLD = '0'
                    and ss_cd_hold = '0'
               else '0';

-- Freeze FSM: on an engine sleep request, wait for the composite boundary
-- before actually gating the CEs (ss_frozen), then ack with ss_paused so the
-- engine's SETTLECOUNT drain starts.  Watchdog: a 64KB block-move holds
-- STATE/=0 for 3+ frames (plan §2.3) — after ~2^23 CLK (~12 frames at 42.95MHz)
-- the boundary is forced rather than hanging the savestate engine forever.
-- Deferred-VBLANK unfreeze is intentionally omitted: capture is VBL-gated, so
-- restore resumes inside VBLANK and the (reset) render pipelines rebuild before
-- the first visible line; only a watchdog-forced save can show a one-frame
-- glitch.
process( CLK ) begin
	if rising_edge( CLK ) then
		if sleep_ss = '0' then
			ss_frozen <= '0';
			ss_wd <= (others => '0');
		elsif ss_frozen = '0' then
			-- The core keeps RUNNING while we wait for the boundary, so a long
			-- wait is invisible to the player — the save simply lands at the
			-- first safe moment.  ~3.1s (2^27 CLK) of continuous retry before
			-- the pathological-case force+abort.
			ss_wd <= ss_wd + 1;
			-- snapshot WHICH terms block ('1' = this term is the blocker)
			ss_dbg_block <= (not ss_bnd_state0) & ss_bnd_res_int & CPU_CE
			              & (not video_vbl_i)
			              & (ss_busy0(1) or ss_busy0(0)) & (ss_busy1(1) or ss_busy1(0))
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

-- Post-load ROM refetch (hardware-found bug).  rom_sdata/rom_ddata are
-- registers refreshed only by a read, and reads are paced by clkref=ce_rom
-- (= CPU_CLKEN): after the restore rewrites PC/MPR no ce_rom pulse has
-- occurred for the new address, ROM_RDY is still '1' from the stale fetch,
-- so the first CPU_CE would execute the PRE-LOAD PC's byte at the restored
-- PC -> wrong opcode -> game crashes into its reset vector (observed on
-- hardware as a reboot to a stuck boot screen; save/resume is immune since
-- the stale byte is the frozen PC's own).  Hold the freeze after load_done
-- and pulse ROM_CLKEN (a fake ce_rom kick every 16 CLK) so ddram/sdram
-- refetch the restored address; release once ROM_RDY has re-asserted (or
-- immediately for a non-ROM PC), with a 255-cycle timeout backstop.
process( CLK ) begin
	if rising_edge( CLK ) then
		ss_sleep_d  <= sleep_ss;
		ss_saving_d <= ss_saving;
		-- refetch after LOAD (restored PC) and, since P5d, after SAVE too:
		-- the region-10 walk reads the CD-RAM through the shared ROM data
		-- register, clobbering the frozen PC's prefetched byte (ss_saving
		-- falls together with sleep_ss, hence the delayed copy)
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
ss_sleep_eff <= (sleep_ss and ss_frozen) or ss_load_done or ss_refetch;

-- A watchdog-forced SAVE captured a mid-instruction machine: abort it (the
-- engine skips the control word, the slot keeps its previous valid blob).
-- Forced LOADs are harmless — the restore rewrites the machine completely.
ss_save_abort <= ss_forced and ss_saving;

SSMANAGER : entity work.statemanager
generic map (
	Softmap_SaveState_ADDR => 16#3800000#,	-- DWORD addr = byte 0x0E000000 in the 0x30000000 window = DDR3 0x3E000000
	Softmap_Rewind_ADDR    => 16#3800000#	-- rewind unused
)
port map (
	clk               => CLK,
	reset             => RESET,
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
	increaseSSHeaderCount => '1',
	save                  => ss_req_save,
	save_abort            => ss_save_abort,
	load                  => ss_req_load,
	savestate_address     => ss_addr_int,
	savestate_busy        => ss_busy_i,
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

-- First instrumented register: SF2 rombank (also proves the SS bus end-to-end).
-- NOTE: with more eRegs in child modules, OR-combine their BUS_Dout here.
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

-- Forensic eReg 67: region-10 walk health (save-only, never restored).
--   15:0 = reads sampled without a ROM_RDY dip, 23:16 = 255-CLK timeouts,
--   31:24 = slowest byte's cnt at sample.  One blob dump shows whether the
--   walk's accesses start, dip and how late they complete on this hardware.
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
                  or SS_Dout_CD or SSE_Dout or SS_Dout_CDRDBG;

-- SaveStateBus export towards TurboGrafx16.sv (TOP_EXT input-block eReg)
SSE_Din  <= SaveStateBus_Din;
SS_DBG   <= ss_forced & ss_dbg_block;

-- AC 2MB RAM is not a blob region until P4: flag any real AC use so the
-- top level can veto savestates for that session (option alone is not
-- enough - most users leave it enabled while few games touch it).
process( CLK ) begin
	if rising_edge( CLK ) then
		if RESET = '1' then
			ss_ac_used_r <= '0';
		elsif (AC_SEL_N = '0' and CPU_WR_N = '0') or AC_RAM_CS_N = '0' then
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
		when 10     => Save_RAMReadData <= ss_cdr_data;
		when others => Save_RAMReadData <= x"00";
	end case;
end process;

end rtl;
