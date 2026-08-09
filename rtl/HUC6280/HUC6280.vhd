library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.HUC6280_PKG.all;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity HUC6280 is
	port(
		CLK		: in std_logic;
		RST_N		: in std_logic;
		WAIT_N	: in std_logic;
		SLEEP		: in std_logic := '0';	-- savestate freeze: halts CPU/IO/PSG clock-enable generation
		  
		A			: out std_logic_vector(20 downto 0);
		DI			: in std_logic_vector(7 downto 0);
		DO			: out std_logic_vector(7 downto 0);
		WR_N  	: out std_logic;
		RD_N  	: out std_logic;
		RDY		: in std_logic;
		NMI_N		: in std_logic;  
		IRQ1_N	: in std_logic;
		IRQ2_N	: in std_logic;
		
		CE			: out std_logic;
		CEK_N		: out std_logic;
		CE7_N		: out std_logic;
		CER_N		: out std_logic;
		PRE_RD	: out std_logic; -- for MiSTer sdram/ddram read
		PRE_WR	: out std_logic;
		
		HSM		: out std_logic;
		
		O			: out std_logic_vector(7 downto 0);
		K			: in std_logic_vector(7 downto 0);
		
		VDCNUM	: in std_logic;
		
		AUD_LDATA: out std_logic_vector(23 downto 0);
		AUD_RDATA: out std_logic_vector(23 downto 0);

		-- Savestates (plan §5.1): CPU_2 + CPU_TIMER eRegs live here; the bus is
		-- also forwarded to the CPU core (CPU_1/CPU_MPR) and the PSG.
		SaveStateBus_Din  : in  std_logic_vector(63 downto 0) := (others => '0');
		SaveStateBus_Adr  : in  std_logic_vector(9 downto 0) := (others => '0');
		SaveStateBus_wren : in  std_logic := '0';
		SaveStateBus_rst  : in  std_logic := '0';
		SaveStateBus_load : in  std_logic := '0';
		SaveStateBus_Dout : out std_logic_vector(63 downto 0);
		-- PSG WF_DATA walk (SAVETYPE_PSGWF) passthrough
		SS_WF_Addr        : in  std_logic_vector(7 downto 0) := (others => '0');
		SS_WF_WrEn        : in  std_logic := '0';
		SS_WF_WrData      : in  std_logic_vector(4 downto 0) := (others => '0');
		SS_WF_RdData      : out std_logic_vector(4 downto 0);
		-- Composite safe-boundary status (plan §2.3)
		SS_STATE0         : out std_logic;
		SS_RES_INT        : out std_logic
	);
end HUC6280;

architecture rtl of HUC6280 is

	signal CPU_CE 			: std_logic;
	signal CPU_CE_G		: std_logic;	-- CPU_CE with SLEEP removed (savestate freeze)
	signal CPU_CER 		: std_logic;
	signal IO_CE 			: std_logic;
	signal EN 				: std_logic;
	
	signal CPU_DI 			: std_logic_vector(7 downto 0);
	signal CPU_DO 			: std_logic_vector(7 downto 0);
	signal CPU_A 			: std_logic_vector(20 downto 0);
	signal CPU_WE_N 		: std_logic;
	signal CPU_CS 			: std_logic;
	signal CPU_MCYCLE		: std_logic;
	signal CPU_IRQ1_N 	: std_logic;
	signal CPU_IRQ2_N 	: std_logic;
	signal CPU_IRQT_N 	: std_logic;
	signal CPU_RDY 		: std_logic;
	
	signal CPU_CLK_CNT 	: unsigned(4 downto 0);
	signal IO_CLK_CNT 	: unsigned(2 downto 0);
	signal VDC_SEL_OLD	: std_logic;
	
	--IO
	signal IO_BUF 			: std_logic_vector(7 downto 0);
	signal RAM_SEL 		: std_logic;
	signal VDC_SEL 		: std_logic;
	signal VCE_SEL 		: std_logic;
	signal IOP_SEL 		: std_logic;
	signal PSG_SEL 		: std_logic;
	signal TMR_SEL 		: std_logic;
	signal INT_SEL 		: std_logic;
	signal IO_SEL 			: std_logic;
	
	signal INT_MASK_PRE	: std_logic_vector(2 downto 0);
	signal INT_MASK 		: std_logic_vector(2 downto 0);
	signal TMR_PRE_CNT 	: unsigned(9 downto 0);
	signal TMR_VALUE 		: std_logic_vector(6 downto 0); 
	signal TMR_LATCH 		: std_logic_vector(6 downto 0);
	signal TMR_EN 			: std_logic;
	signal TMR_RELOAD		: std_logic;
	signal TMR_IRQ 		: std_logic;
	signal TMR_IRQ_ACK 	: std_logic;

	--Savestates
	signal O_FF				: std_logic_vector(7 downto 0);
	signal SS_CPU_2			: std_logic_vector(63 downto 0);
	signal SS_TIMER			: std_logic_vector(63 downto 0);
	signal SS_CPU_2_BACK	: std_logic_vector(63 downto 0);
	signal SS_CPU2_PART		: std_logic_vector(63 downto 0);
	signal SS_TIMER_BACK	: std_logic_vector(63 downto 0) := (others => '0');
	signal SS_Dout_CPU		: std_logic_vector(63 downto 0);
	signal SS_Dout_CPU2		: std_logic_vector(63 downto 0);
	signal SS_Dout_TIMER	: std_logic_vector(63 downto 0);
	signal SS_Dout_PSG		: std_logic_vector(63 downto 0);

begin

	
	process(CLK, RST_N)
	begin
		if RST_N = '0' then
			CPU_CLK_CNT <= (others=>'0');
			CPU_CE <= '0';
			CPU_CER <= '0';
			IO_CLK_CNT <= (others=>'0');
			IO_CE <= '0';
		elsif rising_edge(CLK) then
			if SLEEP = '0' then		-- savestate freeze: hold counters and CEs
				CPU_CE <= '0';
				if (CPU_CLK_CNT = 5 and CPU_CS = '1') or (CPU_CLK_CNT = 23 and CPU_CS = '0') then
					if WAIT_N = '1' then
						CPU_CLK_CNT <= (others=>'0');
						CPU_CE <= '1';
					end if;
				else
					CPU_CLK_CNT <= CPU_CLK_CNT + 1;
				end if;

				CPU_CER <= '0';
				if CPU_CLK_CNT = 1 then
					CPU_CER <= '1';
				end if;

				IO_CE <= '0';
				IO_CLK_CNT <= IO_CLK_CNT + 1;
				if IO_CLK_CNT = 5 then
					IO_CLK_CNT <= (others=>'0');
					IO_CE <= '1';
				end if;
			else
				CPU_CE <= '0';
				CPU_CER <= '0';
				IO_CE <= '0';
			end if;
		end if;
	end process;
	
	CE <= CPU_CE_G and CPU_RDY;	-- SLEEP-gated: see CPU_CE_G below
	
	-- The SLEEP term is NOT redundant with the gating in the CE generator above.
	-- CPU_CE is REGISTERED, and ss_frozen (which drives SLEEP) is latched on the
	-- SAME clock edge, so a freeze accepted on the cycle before the generator
	-- fires - CPU_CLK_CNT = 5 with CS='1', or 23 with CS='0' - leaves one CE
	-- already in flight.  Without this term that CE reaches the core, which has
	-- no SLEEP port of its own, and the CPU takes one more step AFTER the
	-- composite boundary accepted the freeze.
	--
	-- Consequence when it happened: the blob was captured at STATE=1 (seen on
	-- hardware mid-JSR, IR=$20, with the header's forced flag clear), and the
	-- mid-instruction internals - AA, DR, T, MC, NEXT_STATE, ALU_* - are not in
	-- the savestate.  They are don't-care only while every save lands at
	-- STATE=0; this is what guarantees that.  Dropping the leaked CE is safe: at
	-- a boundary it merely delays the next instruction on a machine that is
	-- about to stop anyway.  Covered by sim/tb_cpuslp.vhd, which sweeps every
	-- phase of the generator instead of waiting for the 1-in-24 accident.
	-- Gate ONCE, here, and feed everything from it.  The first version of this
	-- fix gated only EN (the core), and that was not enough: the CE output port
	-- drives VDC0, VDC1 and the CD block too (pce_top: CE => CPU_CE, then
	-- CPU_CE => those three), so the leaked pulse still reached them and they
	-- ran one extra bus cycle while the machine was supposed to be frozen.
	-- Observed on hardware as: CPU now captured correctly at STATE=0, but the
	-- VDC came back with IRQ_VBL stuck pending and the game drowned in an
	-- interrupt storm.  During SLEEP nothing must see a CPU clock enable.
	CPU_CE_G <= CPU_CE and not SLEEP;
	EN <= CPU_CE_G and CPU_RDY;
	
	
	CORE : entity work.HUC6280_CPU
	port map (
		CLK 		=> CLK,
		RST_N 	=> RST_N,
		-- MUST be the SLEEP-gated enable.  This port is the core's only clock
		-- enable (HUC6280_CPU: EN <= RDY and CE) and the core has no SLEEP input
		-- of its own, so wiring the raw CPU_CE here is what let the CPU take one
		-- more step after the savestate boundary had accepted the freeze.
		-- NB: the `EN` signal in this file is NOT the core's enable - it only
		-- feeds the PSG write strobe and the IO/timer registers.  Gating that one
		-- alone looks like a fix and changes nothing for the CPU.
		CE 		=> CPU_CE_G,
		
		A_OUT 	=> CPU_A,
		DI 		=> CPU_DI,
		DO 		=> CPU_DO,
		WE_N 		=> CPU_WE_N,
		RDY 		=> CPU_RDY,
		IRQ1_N 	=> CPU_IRQ1_N,
		IRQ2_N 	=> CPU_IRQ2_N,
		IRQT_N 	=> CPU_IRQT_N,
		NMI_N 	=> NMI_N,
		MCYCLE	=> CPU_MCYCLE,
		CS 		=> CPU_CS,
		VDCNUM   => VDCNUM,

		SaveStateBus_Din  => SaveStateBus_Din,
		SaveStateBus_Adr  => SaveStateBus_Adr,
		SaveStateBus_wren => SaveStateBus_wren,
		SaveStateBus_rst  => SaveStateBus_rst,
		SaveStateBus_load => SaveStateBus_load,
		SaveStateBus_Dout => SS_Dout_CPU,
		SS_CPU2_PART      => SS_CPU2_PART,
		SS_CPU2_Din       => SS_CPU_2,
		SS_STATE0         => SS_STATE0,
		SS_RES_INT        => SS_RES_INT
	);
	
	CPU_IRQ1_N <= IRQ1_N or INT_MASK(1);
	CPU_IRQ2_N <= IRQ2_N or INT_MASK(0);
	CPU_IRQT_N <= not TMR_IRQ or INT_MASK(2);
	
	RAM_SEL <= '1' when CPU_A(20 downto 15) = "111110" else '0'; -- RAM : Page $F8 - $FB
	VDC_SEL <= '1' when CPU_A(20 downto 13) = x"FF" and CPU_A(12 downto 10) = "000" else '0'; -- VDC : $0000 - $03FF
	VCE_SEL <= '1' when CPU_A(20 downto 13) = x"FF" and CPU_A(12 downto 10) = "001" else '0'; -- VCE : $0400 - $07FF
	
	process(CLK, RST_N)
	begin
		if RST_N = '0' then
			WR_N <= '1';
			RD_N <= '1';
			CPU_RDY <= '1';
			VDC_SEL_OLD <= '0';
		elsif rising_edge(CLK) then
			if CPU_CER = '1' then
				if CPU_MCYCLE = '1' then
					WR_N <= CPU_WE_N;
					RD_N <= not CPU_WE_N;
				end if; 
				
				VDC_SEL_OLD <= VDC_SEL or VCE_SEL;
				if (VDC_SEL = '1' or VCE_SEL = '1') and VDC_SEL_OLD = '0' then
					CPU_RDY <= '0';
				end if;
			elsif CPU_CE = '1' then
				if CPU_RDY = '1' then
					WR_N <= '1';
					RD_N <= '1';
				end if; 
				CPU_RDY <= RDY;
			end if; 
		end if;
	end process;
	
	PRE_RD <= CPU_WE_N and CPU_MCYCLE and RST_N;
	PRE_WR <= not CPU_WE_N and CPU_MCYCLE and RST_N;
	
	A <= CPU_A;
	DO <= CPU_DO;
	CER_N <= not RAM_SEL;
	CE7_N <= not VDC_SEL;
	CEK_N <= not VCE_SEL;
	HSM <= CPU_CS;
	
	
	
	--KO port
	IOP_SEL <= '1' when CPU_A(20 downto 13) = x"FF" and CPU_A(12 downto 10) = "100" else '0'; -- IOP : $1000 - $13FF
	process(CLK, RST_N)
	begin
		if RST_N = '0' then
			O_FF <= (others=>'0');
		elsif rising_edge(CLK) then
			if SaveStateBus_load = '1' then
				O_FF <= SS_CPU_2(31 downto 24);
			elsif EN = '1' then
				if IOP_SEL = '1' and CPU_WE_N = '0' then
					O_FF <= CPU_DO;
				end if;
			end if;
		end if;
	end process;

	O <= O_FF;
	
	--Interrupts register
	INT_SEL <= '1' when CPU_A(20 downto 13) = x"FF" and CPU_A(12 downto 10) = "101" else '0'; -- INT : $1400 - $17FF
	process(CLK, RST_N)
	begin
		if RST_N = '0' then
			INT_MASK_PRE <= (others=>'0');
			TMR_IRQ_ACK <= '0';
		elsif rising_edge(CLK) then
			if SaveStateBus_load = '1' then
				INT_MASK     <= SS_TIMER(34 downto 32);
				INT_MASK_PRE <= SS_TIMER(38 downto 36);
				TMR_IRQ_ACK  <= SS_TIMER(29);
			else
			TMR_IRQ_ACK <= '0';

			if CPU_CE = '1' then
				INT_MASK <= INT_MASK_PRE;	-- Delay interrupt mask usage until 1 cycle after it is updated
			end if;

			if INT_SEL = '1' and CPU_CER = '1' then
				if CPU_WE_N = '0' then
					case CPU_A(1 downto 0) is
						when "10" =>
							INT_MASK_PRE <= CPU_DO(2 downto 0);
						when "11" =>
							TMR_IRQ_ACK <= '1';
						when others => null;
					end case;
				end if;
			end if;
			end if;	-- savestate load
		end if;
	end process;


	-- Timer
	TMR_SEL <= '1' when CPU_A(20 downto 13) = x"FF" and CPU_A(12 downto 10) = "011" else '0'; -- TMR : $0C00 - $0FFF
	process( CLK, RST_N )
	begin
		if RST_N = '0' then
			TMR_VALUE <= (others => '0');			
			TMR_PRE_CNT <= (others => '1'); 
			TMR_LATCH <= (others => '0');
			TMR_EN <= '0';
			TMR_RELOAD <= '0';
			TMR_IRQ <= '0';
		elsif rising_edge(CLK) then
			if SaveStateBus_load = '1' then
				TMR_VALUE   <= SS_TIMER(6 downto 0);
				TMR_LATCH   <= SS_TIMER(14 downto 8);
				TMR_PRE_CNT <= unsigned(SS_TIMER(25 downto 16));
				TMR_EN      <= SS_TIMER(26);
				TMR_RELOAD  <= SS_TIMER(27);
				TMR_IRQ     <= SS_TIMER(28);
			elsif TMR_SEL = '1' and CPU_WE_N = '0' and CPU_CER = '1' then
				if CPU_A(0) = '0' then
					-- Timer latch
					TMR_LATCH <= CPU_DO(6 downto 0);
				else
					-- Timer enable
					TMR_EN <= CPU_DO(0);
					if TMR_EN = '0' and CPU_DO(0) = '1' then
						TMR_VALUE <= TMR_LATCH;
						TMR_PRE_CNT <= (others => '1'); 
					end if;
				end if;	
			end if; 
			
			if TMR_IRQ_ACK = '1' then
				TMR_IRQ <= '0';
			end if;
			
			if IO_CE = '1' then
				TMR_RELOAD <= '0';
				if TMR_EN = '1' then
					TMR_PRE_CNT <= TMR_PRE_CNT - 1;
					if TMR_PRE_CNT = 0 then
						TMR_VALUE <= std_logic_vector( unsigned(TMR_VALUE) - 1 );
						if TMR_VALUE = "0000000" then
							TMR_RELOAD <= '1';
							TMR_IRQ <= '1';
						end if;
					end if;
				end if; 
				
				if TMR_RELOAD = '1' then
					TMR_VALUE <= TMR_LATCH;
				end if; 
			end if;
		end if;
	end process;
	
	-- PSG
	PSG_SEL <= '1' when CPU_A(20 downto 13) = x"FF" and CPU_A(12 downto 10) = "010" else '0'; -- PSG : $0800 - $0BFF
	PSG : entity work.psg port map (
		CLK		=> CLK,
		CLKEN		=> IO_CE,	-- 7.16 Mhz clock
		RESET_N	=> RST_N,

		DI			=> CPU_DO,
		A			=> CPU_A(3 downto 0),
		WE			=> not CPU_WE_N and EN and PSG_SEL,

		DAC_LATCH=> '1',
		LDATA		=> AUD_LDATA,
		RDATA		=> AUD_RDATA,

		SaveStateBus_Din  => SaveStateBus_Din,
		SaveStateBus_Adr  => SaveStateBus_Adr,
		SaveStateBus_wren => SaveStateBus_wren,
		SaveStateBus_rst  => SaveStateBus_rst,
		SaveStateBus_load => SaveStateBus_load,
		SaveStateBus_Dout => SS_Dout_PSG,
		SS_WF_Addr        => SS_WF_Addr,
		SS_WF_WrEn        => SS_WF_WrEn,
		SS_WF_WrData      => SS_WF_WrData,
		SS_WF_RdData      => SS_WF_RdData
	);
		
	IO_SEL <= IOP_SEL or INT_SEL or TMR_SEL or PSG_SEL;
	process(CLK, RST_N)
	begin
		if RST_N = '0' then
			IO_BUF <= (others=>'1');
		elsif rising_edge(CLK) then
			if SaveStateBus_load = '1' then
				IO_BUF <= SS_CPU_2(39 downto 32);
			elsif EN = '1' then
				if IO_SEL = '1' then
					if CPU_WE_N = '0' then
						IO_BUF <= CPU_DO;
					else
						IO_BUF <= CPU_DI;
					end if; 
				end if; 
			end if; 
		end if;
	end process;
	
	process(CLK)
	begin
		if rising_edge(CLK) then
			if IO_SEL = '0' then
				CPU_DI <= DI;
			elsif PSG_SEL = '1' then
				CPU_DI <= x"00";
			elsif IOP_SEL = '1' then
				CPU_DI <= K;
			elsif INT_SEL = '1' then
				case CPU_A(1 downto 0) is
					when "10" =>
						CPU_DI <= IO_BUF(7 downto 3) & INT_MASK;
					when "11" =>
						CPU_DI <= IO_BUF(7 downto 3) & TMR_IRQ & not IRQ1_N & not IRQ2_N;
					when others =>
						CPU_DI <= IO_BUF;
				end case;
			elsif TMR_SEL = '1' then
				CPU_DI <= IO_BUF(7) & TMR_VALUE;
			else
				CPU_DI <= IO_BUF;
			end if;
		end if;
	end process;

	--------------------------------------------------------------------------------
	-- SAVESTATES (plan §5.1; slot map: pce_savestates_pkg.vhd)
	--------------------------------------------------------------------------------
	-- NOT saved (reseeded to canonical phase by reset_ss, plan §7.2):
	-- CPU_CLK_CNT, IO_CLK_CNT, CPU_CE/CER, IO_CE, CPU_RDY, WR_N, RD_N, VDC_SEL_OLD.
	-- CPU_DI re-registers from the bus every CLK and self-heals before unfreeze.

	-- CPU_2: PC/MPR_LAST/int latches pre-placed by the core; O and IO_BUF here.
	SS_CPU_2_BACK(23 downto 0)  <= SS_CPU2_PART(23 downto 0);
	SS_CPU_2_BACK(31 downto 24) <= O_FF;
	SS_CPU_2_BACK(39 downto 32) <= IO_BUF;
	SS_CPU_2_BACK(63 downto 40) <= SS_CPU2_PART(63 downto 40);

	-- CPU_TIMER: TMR_VALUE(6:0), TMR_LATCH(14:8), TMR_PRE_CNT(25:16), TMR_EN(26),
	--            TMR_RELOAD(27), TMR_IRQ(28), TMR_IRQ_ACK(29), INT_MASK(34:32),
	--            INT_MASK_PRE(38:36)
	SS_TIMER_BACK(6 downto 0)   <= TMR_VALUE;
	SS_TIMER_BACK(14 downto 8)  <= TMR_LATCH;
	SS_TIMER_BACK(25 downto 16) <= std_logic_vector(TMR_PRE_CNT);
	SS_TIMER_BACK(26)           <= TMR_EN;
	SS_TIMER_BACK(27)           <= TMR_RELOAD;
	SS_TIMER_BACK(28)           <= TMR_IRQ;
	SS_TIMER_BACK(29)           <= TMR_IRQ_ACK;
	SS_TIMER_BACK(34 downto 32) <= INT_MASK;
	SS_TIMER_BACK(38 downto 36) <= INT_MASK_PRE;

	iSS_CPU_2 : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_CPU_2, def => SSREG_DEFAULT_CPU_2 )
	port map (
		clk      => CLK,
		BUS_Din  => SaveStateBus_Din,
		BUS_Adr  => SaveStateBus_Adr,
		BUS_wren => SaveStateBus_wren,
		BUS_rst  => SaveStateBus_rst,
		BUS_Dout => SS_Dout_CPU2,
		Din      => SS_CPU_2_BACK,
		Dout     => SS_CPU_2
	);

	iSS_CPU_TIMER : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_CPU_TIMER, def => SSREG_DEFAULT_CPU_TIMER )
	port map (
		clk      => CLK,
		BUS_Din  => SaveStateBus_Din,
		BUS_Adr  => SaveStateBus_Adr,
		BUS_wren => SaveStateBus_wren,
		BUS_rst  => SaveStateBus_rst,
		BUS_Dout => SS_Dout_TIMER,
		Din      => SS_TIMER_BACK,
		Dout     => SS_TIMER
	);

	SaveStateBus_Dout <= SS_Dout_CPU or SS_Dout_CPU2 or SS_Dout_TIMER or SS_Dout_PSG;

end rtl;
	