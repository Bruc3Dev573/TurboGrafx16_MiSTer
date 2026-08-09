-----------------------------------------------------------------------------
-- pce_savestates_pkg.vhd
-- Savestate register-slot map for the TurboGrafx16 / PC Engine core.
--
-- Companion of the Robert Peip savestate framework (bus_savestates.vhd,
-- savestates.vhd, statemanager.vhd — vendored verbatim from NES_MiSTer).
-- SystemVerilog mirror: rtl/pce_savestates.sv (keep the two in sync!).
--
-- Bus: 64-bit data, 10-bit address (pBus_savestates). INTERNALSCOUNT = 64,
-- so only slots 0..63 are streamed to/from DDR3.
--
-- Full design rationale: docs/SAVESTATE_IMPLEMENTATION_PLAN.md §5 (per-module
-- change list) and §4.2 (DDR3 slot layout).
--
-- Bit-packing convention: field(N downto M) noted per slot. Unlisted upper
-- bits are zero. Defaults are power-on values (BUS_rst loads them).
-----------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

package pPCE_savestates is

   ---------------------------------------------------------------------------
   -- HuC6280 CPU core (HUC6280_CPU.vhd + HUC6280_AG.vhd)
   ---------------------------------------------------------------------------
   -- CPU_1: A(7:0), X(15:8), Y(23:16), SP(31:24), P(39:32), IR(47:40),
   --        STATE(52:48), CS(53)  [CS = speed, HUC6280.vhd]
   constant SSREG_INDEX_CPU_1     : integer := 0;
   constant SSREG_DEFAULT_CPU_1   : std_logic_vector(63 downto 0) := (others => '0');
   -- CPU_2: PCr(15:0) [from HUC6280_AG], MPR_LAST(23:16), O(31:24) [KO port],
   --        IO_BUF(39:32), GOT_INT(40), RES_INT(41), NMI_INT(42), IRQ1_INT(43),
   --        IRQ2_INT(44), IRQT_INT(45), OLD_NMI_N(46), NMI_SYNC(47), NMI_ACTIVE(48)
   constant SSREG_INDEX_CPU_2     : integer := 1;
   constant SSREG_DEFAULT_CPU_2   : std_logic_vector(63 downto 0) := (others => '0');
   -- CPU_MPR: MPR0(7:0) .. MPR7(63:56) — exactly one full slot
   constant SSREG_INDEX_CPU_MPR   : integer := 2;
   constant SSREG_DEFAULT_CPU_MPR : std_logic_vector(63 downto 0) := (others => '0');
   -- CPU_TIMER (HUC6280.vhd): TMR_VALUE(6:0), TMR_LATCH(14:8), TMR_PRE_CNT(25:16),
   --        TMR_EN(26), TMR_RELOAD(27), TMR_IRQ(28), TMR_IRQ_ACK(29),
   --        INT_MASK(34:32), INT_MASK_PRE(38:36)
   constant SSREG_INDEX_CPU_TIMER   : integer := 3;
   constant SSREG_DEFAULT_CPU_TIMER : std_logic_vector(63 downto 0) := (others => '0');

   ---------------------------------------------------------------------------
   -- PSG (rtl/HUC6280/psg.vhd)
   ---------------------------------------------------------------------------
   -- PSG_GLOBAL: CHSEL(2:0), LMAL(7:4), RMAL(11:8), LDATA_FF(35:12), RDATA_FF(59:36)
   constant SSREG_INDEX_PSG_GLOBAL   : integer := 4;
   constant SSREG_DEFAULT_PSG_GLOBAL : std_logic_vector(63 downto 0) := (others => '0');
   -- Per channel i (0..5), base slot = 5 + i*3, three slots:
   --  +0 (CHx_A): FREQ(11:0), DDA(12), CHON(13), AL(20:16), LAL(27:24), RAL(31:28),
   --             NG_FREQ(36:32), NE(37), WF_ADDR(44:40), DA_OUT(52:48), WF_OUT(60:56)
   --  +1 (CHx_B): WF_CNT(12:0), LFSR(33:16), NG_CNT(47:36), NG_OUT(52:48), GL_OUT(60:56)
   --  +2 (CHx_C): LFO_FREQ(7:0), LFCTL(9:8), LFTRG(10), LFO_CNT(23:16), LFO_ADD(43:32)
   -- (WF_DATA 6x32x5b waveform tables travel over the Save_RAM walk,
   --  Save_RAMType = SAVETYPE_PSGWF — NOT over register slots.)
   constant SSREG_INDEX_PSG_CH0   : integer := 5;   -- 5,6,7
   constant SSREG_INDEX_PSG_CH1   : integer := 8;   -- 8,9,10
   constant SSREG_INDEX_PSG_CH2   : integer := 11;  -- 11,12,13
   constant SSREG_INDEX_PSG_CH3   : integer := 14;  -- 14,15,16
   constant SSREG_INDEX_PSG_CH4   : integer := 17;  -- 17,18,19
   constant SSREG_INDEX_PSG_CH5   : integer := 20;  -- 20,21,22
   constant SSREG_DEFAULT_PSG_CH  : std_logic_vector(63 downto 0) := (others => '0');

   ---------------------------------------------------------------------------
   -- HuC6260 VCE (rtl/huc6260.vhd)
   ---------------------------------------------------------------------------
   -- VCE_1: CR(7:0), DOTCLOCK(9:8), BW(10), RAM_A(20:12) [9b], RAM_DI(29:21) [9b],
   --        RAM_WE(30), CTRL(33:32) [ctrl_t enc: 00=IDLE 01=WAIT 10=INCR],
   --        DO(43:36), PREV_A(46:44) [3b]
   constant SSREG_INDEX_VCE_1     : integer := 23;
   constant SSREG_DEFAULT_VCE_1   : std_logic_vector(63 downto 0) := (others => '0');
   -- VCE_2: H_CNT(11:0), V_CNT(25:16), END_LINE(41:32) [integer -> to_unsigned(,10)],
   --        CLKEN_CNT(46:44), CLKEN_FS_CNT(50:48), CLKEN_FF(51), CLKEN_FF_F(52),
   --        CLKEN_FS(53), MULTIRES_FF(54), MULTIRES(55)
   -- POLICY: CLKEN_CNT / CLKEN_FS_CNT are SAVED VERBATIM, never reseeded (plan §5.4).
   constant SSREG_INDEX_VCE_2     : integer := 24;
   constant SSREG_DEFAULT_VCE_2   : std_logic_vector(63 downto 0) := (others => '0');
   -- (Palette RAM 512x9 travels over Save_RAM walk, SAVETYPE_PALETTE, 9->16 padded.)

   ---------------------------------------------------------------------------
   -- HuC6270 VDC — instance 0 (PCE/SGX) slots 25..34, instance 1 (SGX) 35..44
   -- Per instance, offset from base:
   --  +0..+4 (REGS_0..4): REGS[0..19] packed 4 regs x 16b per slot
   --          (REGS(4k)(15:0), REGS(4k+1)(31:16), REGS(4k+2)(47:32), REGS(4k+3)(63:48))
   --  +5 (CORE): AR(4:0), VRR(23:8),
   --          shadow CM(24), SM(26:25), VM(28:27), SCREEN(31:29), BB(32), SB(33),
   --          IRQ flags {DMA(38),COL(39),OVF(40),RCR(41),DMAS(42),VBL(43)},
   --          SR_LATCH(50:44) [7b], CPU_BUSY(52), CPU_BUSY_CLEAR(53)
   --  +6 (HSK): CPURD_PEND(0), CPUWR_PEND(1), CPURD_PEND2(2), CPUWR_PEND2(3),
   --          CPURD_EXEC(4), CPUWR_EXEC(5), CPU_VRAM_ADDR(21:6), CPU_VRAM_DATA(37:22),
   --          BYR/BXR write latches {IO_BYRL_SET,IO_BYRH_SET,IO_BYRL_WR,IO_BYRH_WR,
   --          BXR_SET,BYRL_SET,BYRH_SET}(44:38)
   --  +7 (DMA): DMA_PEND(0), DMA_EXEC(1), DMA_WR(2), DMA_BUF(18:3),
   --          DMAS_PEND(19), DMAS_EXEC(20), DMAS_SAT_ADDR(31:24), DMAS_VRAM_ADDR(47:32)
   --  +8 (RASTER): DOT_CNT(2:0), TILE_CNT(10:4), DISP_CNT(25:16), RC_CNT(41:32),
   --          DOTS_REMAIN(46:44), RC_CNT_UPDATED(47) [promote process variable!],
   --          DISP_CNT_INC(48), DISP_BREAK_EN(49), DISP_BREAK_LATCH(50), TILE_ZERO(51),
   --          BURST(52), VDISP(53), VDISP_OLD(54), RD_N_OLD(55), RES7M(56)
   --  +9 (TIMING): HSW(4:0), HDS(14:8), HDW(22:16), HDE(30:24), VSW(36:32),
   --          VDS(47:40), VDW(56:48)... VDE packs (63:57) low 7 of 8 — NOTE:
   --          VDE is 8b: put VDE(7:0) at (63:56) and move VDW to (55:48)? FINAL:
   --          HSW(4:0), HDS(14:8), HDW(22:16), HDE(30:24), VSW(36:32), VDS(45:38),
   --          VDW(55:47) [9b], VDE(63:56) [8b]
   -- OFS_X: NOT stored — restored as OFS_X <= REGS(7) on load_done (plan §5.3).
   -- Sprite/BG pipelines, SPR_CACHE, line buffers: NOT stored — ss_reconstruct.
   -- SAT 256x16: Save_RAM walk, SAVETYPE_SAT0/SAT1, verbatim.
   ---------------------------------------------------------------------------
   constant SSREG_INDEX_VDC0      : integer := 25;  -- 25..34
   constant SSREG_INDEX_VDC1      : integer := 35;  -- 35..44 (SGX only, LITE=0)
   constant SSREG_DEFAULT_VDC     : std_logic_vector(63 downto 0) := (others => '0');

   ---------------------------------------------------------------------------
   -- HuC6202 VPC (SGX only, LITE=0)
   -- VPC: PRI0(7:0), PRI1(15:8), WIN1(25:16), WIN2(35:26), VDCNUM(36),
   --      X(49:40) [10b], DO(57:50)
   ---------------------------------------------------------------------------
   constant SSREG_INDEX_VPC       : integer := 45;
   constant SSREG_DEFAULT_VPC     : std_logic_vector(63 downto 0) := (others => '0');

   ---------------------------------------------------------------------------
   -- Top glue (pce_top.vhd)
   -- TOP: rombank(1:0) [SF2 mapper]
   ---------------------------------------------------------------------------
   constant SSREG_INDEX_TOP       : integer := 46;
   constant SSREG_DEFAULT_TOP     : std_logic_vector(63 downto 0) := (others => '0');

   ---------------------------------------------------------------------------
   -- SystemVerilog input-block state (TurboGrafx16.sv, via eReg instantiated
   -- from SV — NES "EXT" pattern):
   -- TOP_EXT: high_buttons(0), joy_port(3:1), joy_latch(7:4), scan_counter(11:8),
   --          joyrept_0(13:12), joyrept_1(15:14), joyrept_2(17:16),
   --          joyrept_3(19:18), joyrept_4(21:20), last_gp(23:22)
   ---------------------------------------------------------------------------
   constant SSREG_INDEX_TOP_EXT   : integer := 47;
   constant SSREG_DEFAULT_TOP_EXT : std_logic_vector(63 downto 0) := (others => '0');

   ---------------------------------------------------------------------------
   -- Arcade Card (rtl/arcade.sv) — phase P4. One slot per port + one control.
   -- AC_PORTx: base(23:0), offset(39:24), increment(55:40), control(62:56)
   -- AC_CTRL:  ena(0), shift_latch(32:1), shift_bits(40:33), rotate_bits(48:41),
   --           old_acc(49)
   ---------------------------------------------------------------------------
   constant SSREG_INDEX_AC_PORT0  : integer := 48;
   constant SSREG_INDEX_AC_PORT1  : integer := 49;
   constant SSREG_INDEX_AC_PORT2  : integer := 50;
   constant SSREG_INDEX_AC_PORT3  : integer := 51;
   constant SSREG_INDEX_AC_CTRL   : integer := 52;

   ---------------------------------------------------------------------------
   -- CD subsystem (rtl/cd/* — phase P5a).  ADPCM DRAM travels over the
   -- Save_RAM walk, SAVETYPE_ADPCM = 9 (64KB, 128K nibbles packed 2/byte).
   --
   -- 53 CD_1 (cd.vhd interface):
   --   SCSI_DBI(7:0), SCSI_ACK_N(8), SCSI_RST_N(9), SCSI_SEL_N(10),
   --   CD_DTD(11), CD_DTR(12), CD_SUBCD(13), CH_SEL(14), BRAM_LOCK(15),
   --   CD_DTD_EN(16), CD_DTR_EN(17), CD_SUBCD_EN(18), ADPCM_END_EN(19),
   --   ADPCM_HALF_EN(20), AUTO_ACK(21), R1802_0(22), R1802_1(23),
   --   R180E_7_4(27:24), R180F_0(28), R180F_7_4(32:29), CD_DATA_CNT(43:33),
   --   SCSI_REQ_N_OLD(44), SCSI_BSY_N_OLD(45), SCSI_ACK_N_OLD(46),
   --   CDDA CD_WR_OLD(47), CDDA_VOL(63:48)
   -- 54 CD_2 (ADPCM control):
   --   ADPCM_OFFS(15:0), ADPCM_LEN(32:16), ADPCM_CTRL(47:40),
   --   ADPCM_FREQ(51:48), ADPCM_FADER(54:52), ADPCM_DMA_EN(55),
   --   ADPCM_DMA_RUN(56), ADPCM_END(57), ADPCM_HALF(58), ADPCM_PLAY(59),
   --   ADPCM_WRITE_PEND(60), ADPCM_READ_PEND(61), PLAY_READ_PEND(62),
   --   DMA_WRITE_PEND(63)
   -- 55 SCSI_1: SP(3:0) [enum pos], BSY_Nr(4), MSG_Nr(5), CD_Nr(6), IO_Nr(7),
   --   REQ_Nr(8), COMM_POS(12:9), DATA_POS(16:13), STOP_CD_SND(17),
   --   STAT_PEND(18), DOUT_PEND(19), CD_WR_OLD(20), DBO(31:24),
   --   DATAIN_CNT(47:32), FIFO_D(55:48), AUDIO_MODE(57:56), AUDIO_ACTIVE(58)
   -- 56 SCSI_2: STAT_COUNT(15:0), DELAY_COUNT(31:16) [17b reg, max 49400],
   --   AUDIO_B2..B5 (raw SAPSP bytes 2-5) at (39:32)(47:40)(55:48)(63:56)
   -- 57 SCSI_COMM_LO: COMM(0)..COMM(7), byte i at (i*8+7 : i*8)
   -- 58 SCSI_COMM_HI: COMM(8..11)(31:0), DATA_BUF(0..3)(63:32)
   -- 59 SCSI_DATA:    DATA_BUF(4..9)(47:0)
   -- 60 CD_3 (ADPCM datapath): ADPCM_RDADDR(16:0), ADPCM_WRADDR(36:20),
   --   ADPCM_RDDATA(47:40), ADPCM_WRDATA(55:48), ADPCM_WRITE_NIB(56),
   --   ADPCM_READ_NIB(57), M5205_D(61:58)
   -- 61 CD_4 (MSM5205): SAMPLE(15:0), STEP(21:16), DEC_DATA(27:24),
   --   DEC_EXEC(28), CLK_CNT(37:32), M5205_CLK_CNT(46:38)
   -- 62 CD_5 (CDDA/subcode): FADE_CNT(7:0), CD_BYTE_CNT(9:8),
   --   CDDA_SAMPLE(10), CDDA_SAMPLE_OLD(11), SUBCD_CNT(15:12),
   --   SUBCD_BYTE(23:16), SUBCD_BYTENUM(31:24), SUBCD_CE(32),
   --   SUBCD_CE_OLD(33), SUBCD_WR_OLD(34), OUTL(50:35)
   -- 63 CD_6: OUTR(15:0), FADE_VOL(26:16), CDDA_CONSUMED(58:27)
   --
   -- NOT saved: the three FIFOs (CD-quiet boundary guarantees SCSI_FIFO
   -- empty; CDDA/SUBC refill from the stream), FIFO_* strobes, CEGen
   -- CLK_SUM phases and MSM inner divider phase (sub-sample reseed).
   ---------------------------------------------------------------------------
   constant SSREG_INDEX_CD_1        : integer := 53;
   constant SSREG_INDEX_CD_2        : integer := 54;
   constant SSREG_INDEX_SCSI_1      : integer := 55;
   constant SSREG_INDEX_SCSI_2      : integer := 56;
   constant SSREG_INDEX_SCSI_COMM_LO: integer := 57;
   constant SSREG_INDEX_SCSI_COMM_HI: integer := 58;
   constant SSREG_INDEX_SCSI_DATA   : integer := 59;
   constant SSREG_INDEX_CD_3        : integer := 60;
   constant SSREG_INDEX_CD_4        : integer := 61;
   constant SSREG_INDEX_CD_5        : integer := 62;
   constant SSREG_INDEX_CD_6        : integer := 63;
   -- P5c2/P5b (SCSI, slot 64):
   --   (7:0)   D9_B2          last SAPEP byte2 (raw)
   --   (15:8)  D9_B3          last SAPEP byte3
   --   (23:16) D9_B4          last SAPEP byte4
   --   (31:24) D9_B5          last SAPEP byte5
   --   (39:32) D9_B1          last SAPEP byte1 (end play mode)
   --   (41:40) D9_MODE        last SAPEP byte9(7:6)
   --   (42)    D9_SEEN
   --   (63:43) READ_CONSUMED  bytes handed to the CPU since the 08 accept
   -- (SCSI_1 additions: (22:21) AUDIO_B1 = D8 byte1(1:0), (23) READ_ACTIVE)
   constant SSREG_INDEX_SCSI_3      : integer := 64;
   constant SSREG_DEFAULT_CD        : std_logic_vector(63 downto 0) := (others => '0');

end package;
