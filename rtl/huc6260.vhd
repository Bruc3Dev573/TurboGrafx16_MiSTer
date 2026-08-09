library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.STD_LOGIC_TEXTIO.all;
library STD;
use STD.TEXTIO.ALL;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity huc6260 is
	port (
		CLK 		: in std_logic;
		RESET_N	: in std_logic;
		SLEEP		: in std_logic := '0';	-- savestate freeze: halts video counters and CLKEN generation

		-- CPU Interface
		A			: in std_logic_vector(2 downto 0);
		CE_N		: in std_logic;
		WR_N		: in std_logic;
		RD_N		: in std_logic;
		DI			: in std_logic_vector(7 downto 0);
		DO 		: out std_logic_vector(7 downto 0);

		-- VDC Interface
		COLNO		: in std_logic_vector(8 downto 0);
		CLKEN		: out std_logic;
		CLKEN_F	: out std_logic;
		HSYNC_F	: out std_logic;
		HSYNC_R	: out std_logic;
		VSYNC_F	: out std_logic;
		VSYNC_R	: out std_logic;
		CLKEN_FS	: out std_logic;
		RVBL		: in std_logic;
		
		GRID_EN	: in std_logic_vector(1 downto 0);
		BORDER_EN: in std_logic;
		BORDER	: in std_logic;
		GRID		: in std_logic_vector(1 downto 0);

		-- NTSC/RGB Video Output
		R			: out std_logic_vector(2 downto 0);
		G			: out std_logic_vector(2 downto 0);
		B			: out std_logic_vector(2 downto 0);
		BW			: out std_logic;

		VS_N		: out std_logic;
		HS_N		: out std_logic;
		HBL		: out std_logic;
		VBL		: out std_logic;

		-- Savestates (plan §5.4)
		SaveStateBus_Din  : in  std_logic_vector(63 downto 0) := (others => '0');
		SaveStateBus_Adr  : in  std_logic_vector(9 downto 0) := (others => '0');
		SaveStateBus_wren : in  std_logic := '0';
		SaveStateBus_rst  : in  std_logic := '0';
		SaveStateBus_load : in  std_logic := '0';	-- load_done strobe: restore registers
		SaveStateBus_Dout : out std_logic_vector(63 downto 0);
		-- Palette RAM walk (SAVETYPE_PALETTE): port A borrowed during freeze
		SS_PAL_Addr       : in  std_logic_vector(9 downto 0) := (others => '0');	-- region byte addr; (9:1)=entry
		SS_PAL_WrEn       : in  std_logic := '0';	-- strobed on odd byte with assembled 9-bit word
		SS_PAL_WrData     : in  std_logic_vector(8 downto 0) := (others => '0');
		SS_PAL_RdData     : out std_logic_vector(8 downto 0)
	);
end huc6260;

architecture rtl of huc6260 is

-- CPU Interface
signal PREV_A	: std_logic_vector(2 downto 0);

type ctrl_t is ( CTRL_IDLE, CTRL_WAIT, CTRL_INCR );
signal CTRL		: ctrl_t;
signal CR		: std_logic_vector(7 downto 0);

-- VCE Registers
signal DOTCLOCK	: std_logic_vector(1 downto 0);

-- CPU Color RAM Interface
signal RAM_A	: std_logic_vector(8 downto 0);
signal RAM_DI	: std_logic_vector(8 downto 0);
signal RAM_WE	: std_logic := '0';
signal RAM_DO	: std_logic_vector(8 downto 0);

-- CPU conflict color latching
signal R_FF	: std_logic_vector(2 downto 0);
signal G_FF	: std_logic_vector(2 downto 0);
signal B_FF	: std_logic_vector(2 downto 0);
signal CE_N_FF : std_logic := '1';

-- Color RAM Output
signal COLOR	: std_logic_vector(8 downto 0);

constant LEFT_BL_CLOCKS	: integer := 456;
constant DISP_CLOCKS	   : integer := 2160;
constant LINE_CLOCKS	   : integer := 2730;
constant HS_CLOCKS		: integer := 192;
constant HS_OFF			: integer := 46;

constant TOTAL_LINES		: integer := 263;  -- 525
constant VS_LINES			: integer := 3; 	 -- pcetech.txt
constant TOP_BL_LINES_E	: integer := 19;   -- pcetech.txt (must include VS_LINES in current implementation)
constant DISP_LINES_E	: integer := 242;	 -- same as in mednafen
signal TOP_BL_LINES		: integer;
signal DISP_LINES			: integer;
signal END_LINE			: integer := TOTAL_LINES;
signal HSYNC_START_POS	: integer;
signal HSYNC_END_POS 	: integer;

signal H_CNT	: std_logic_vector(11 downto 0);
signal V_CNT	: std_logic_vector(9 downto 0);

signal HBL_FF, HBL_FF2	: std_logic;
signal VBL_FF, VBL_FF2	: std_logic;

-- Clock generation
signal CLKEN_CNT	: std_logic_vector(2 downto 0);
signal CLKEN_FS_CNT: std_logic_vector(2 downto 0);
signal CLKEN_FF	: std_logic;
signal CLKEN_FF_F	: std_logic;
signal MULTIRES_FF : std_logic;
signal MULTIRES   : std_logic;

-- Savestates: internal shadows of write-only out ports
signal DO_FF        : std_logic_vector(7 downto 0);
signal BW_FF        : std_logic := '0';
signal CLKEN_FS_FF  : std_logic := '0';
signal CTRL_ENC     : std_logic_vector(1 downto 0);
-- eReg slot values (restored) and back-capture vectors (saved)
signal SS_VCE_1, SS_VCE_2           : std_logic_vector(63 downto 0);
signal SS_VCE_1_BACK, SS_VCE_2_BACK : std_logic_vector(63 downto 0) := (others => '0');
signal SS_Dout_1, SS_Dout_2         : std_logic_vector(63 downto 0);
-- Palette port A borrow mux (during freeze the CPU FSM is quiescent: CTRL_IDLE)
signal RAM_A_MUX    : std_logic_vector(8 downto 0);
signal RAM_DI_MUX   : std_logic_vector(8 downto 0);
signal RAM_WE_MUX   : std_logic;

begin

TOP_BL_LINES <= TOP_BL_LINES_E when RVBL = '1' else TOP_BL_LINES_E+4;
DISP_LINES   <= DISP_LINES_E   when RVBL = '1' else DISP_LINES_E-11;
	 
-- Color RAM
-- Savestate palette walk borrows port A while frozen (plan §5.4): the CPU-side
-- FSM is guaranteed CTRL_IDLE at the composite boundary, and on unfreeze q_a
-- re-primes from RAM_A within 1-2 CLK, long before the CPU can access the VCE.
RAM_A_MUX  <= SS_PAL_Addr(9 downto 1) when SLEEP = '1' else RAM_A;
RAM_DI_MUX <= SS_PAL_WrData           when SLEEP = '1' else RAM_DI;
RAM_WE_MUX <= SS_PAL_WrEn             when SLEEP = '1' else RAM_WE;
SS_PAL_RdData <= RAM_DO;

ram : entity work.dpram generic map (addr_width => 9, data_width => 9, mem_init_file =>"huc6260_palette_init.mif")
port map(
	clock			=> CLK,

	address_a	=> RAM_A_MUX,
	data_a		=> RAM_DI_MUX,
	wren_a		=> RAM_WE_MUX,
	q_a			=> RAM_DO,

	address_b	=> COLNO,
	q_b			=> COLOR
);

process( CLK )
begin
	if rising_edge( CLK ) then
		if RESET_N = '0' then
			RAM_A <= (others => '0');
			RAM_DI <= (others => '0');
			RAM_WE <= '0';
			CR <= x"00";

			PREV_A <= (others => '0');
			CTRL <= CTRL_IDLE;
		elsif SaveStateBus_load = '1' then
			-- Savestate restore (strobed on load_done, while still frozen)
			CR     <= SS_VCE_1(7 downto 0);
			RAM_A  <= SS_VCE_1(20 downto 12);
			RAM_DI <= SS_VCE_1(29 downto 21);
			RAM_WE <= SS_VCE_1(30);
			DO_FF  <= SS_VCE_1(43 downto 36);
			PREV_A <= SS_VCE_1(46 downto 44);
			case SS_VCE_1(33 downto 32) is
				when "01"   => CTRL <= CTRL_WAIT;
				when "10"   => CTRL <= CTRL_INCR;
				when others => CTRL <= CTRL_IDLE;
			end case;
		elsif SLEEP = '0' then	-- freeze the CPU FSM too: a watchdog-forced (non-boundary)
										-- freeze may land mid-access with CE_N/WR_N held low; letting
										-- the FSM run against the borrowed palette port would lose the
										-- write / latch walk garbage into DO (audit P1 finding 1)
			case CTRL is
			
			when CTRL_IDLE =>
				RAM_WE <= '0';
				if CE_N = '0' and WR_N = '0' then
					-- CPU Write
					PREV_A <= A;
					CTRL <= CTRL_WAIT;
					case A is
					when "000" =>
						CR <= DI;
					when "010" =>
						RAM_A(7 downto 0) <= DI;
					when "011" =>
						RAM_A(8) <= DI(0);
					when "100" =>
						RAM_WE <= '1';
						RAM_DI <= RAM_DO(8) & DI;
					when "101" =>
						RAM_WE <= '1';
						RAM_DI <= DI(0) & RAM_DO(7 downto 0);
						CTRL <= CTRL_INCR;
					when others => null;
					end case;
					
				elsif CE_N = '0' and RD_N = '0' then
					-- CPU Read
					PREV_A <= A;
					CTRL <= CTRL_WAIT;
					DO_FF <= x"FF";
					case A is
					when "100" =>
						DO_FF <= RAM_DO(7 downto 0);
					when "101" =>
						DO_FF <= "1111111" & RAM_DO(8);
						CTRL <= CTRL_INCR;
					when others => null;
					end case;
				end if;
			
			when CTRL_INCR =>
				RAM_WE <= '0';
				RAM_A <= RAM_A + 1;
				CTRL <= CTRL_WAIT;
			
			when CTRL_WAIT =>
				RAM_WE <= '0';
				-- Wait for the CPU to "release" the VCE.
				-- I don't know what happens in the case of an address change
				-- however it can be achieved only with addresses read/write cycles,
				-- so it seems unlikely. The case has been handled, though.
				-- HuC6280 Rmw instructions are safe, as there is a "dummy cycle"
				-- between the read cycle and the write cycle.
				CTRL <= CTRL_IDLE;
				if CE_N = '0' and (WR_N = '0' or RD_N = '0') and PREV_A = A then
					CTRL <= CTRL_WAIT;
				end if;
			
			when others => null;
			end case;
		end if;
	end if;
end process;

-- Video counting, register loading and clock generation
process(CLK, RESET_N)
begin
	if RESET_N = '0' then
		CLKEN_CNT <= (others=>'0');
		H_CNT <= (others=>'0');
		V_CNT <= (others=>'0');
		CLKEN_FF <= '0';
		MULTIRES_FF <= '0';
		BW_FF <= '0';
	elsif rising_edge(CLK) then
		if SaveStateBus_load = '1' then
			-- Savestate restore. CLKEN_CNT restored VERBATIM, never reseeded (plan §5.4).
			H_CNT       <= SS_VCE_2(11 downto 0);
			V_CNT       <= SS_VCE_2(25 downto 16);
			END_LINE    <= to_integer(unsigned(SS_VCE_2(41 downto 32)));
			CLKEN_CNT   <= SS_VCE_2(46 downto 44);
			CLKEN_FF    <= SS_VCE_2(51);
			CLKEN_FF_F  <= SS_VCE_2(52);
			MULTIRES_FF <= SS_VCE_2(54);
			MULTIRES    <= SS_VCE_2(55);
			DOTCLOCK    <= SS_VCE_1(9 downto 8);
			BW_FF       <= SS_VCE_1(10);
		elsif SLEEP = '0' then	-- savestate freeze: hold H/V counters, dividers and CLKENs
		H_CNT <= H_CNT + 1;

		CLKEN_FF <= '0';
		CLKEN_CNT <= CLKEN_CNT + 1;
		if DOTCLOCK = "00" and CLKEN_CNT = "111" and H_CNT < LINE_CLOCKS-2-1 then
			CLKEN_CNT <= (others => '0');
			CLKEN_FF <= '1';
		elsif DOTCLOCK = "01" and CLKEN_CNT = "101" then
			CLKEN_CNT <= (others => '0');
			CLKEN_FF <= '1';				
		elsif DOTCLOCK(1) = '1' and CLKEN_CNT = "011" and H_CNT < LINE_CLOCKS-2-1 then
			CLKEN_CNT <= (others => '0');
			CLKEN_FF <= '1';				
		end if;
		
		if DOTCLOCK = "00" and CLKEN_CNT = "011" then
			CLKEN_FF_F <= '1';
		elsif DOTCLOCK = "01" and CLKEN_CNT = "010" then
			CLKEN_FF_F <= '1';				
		elsif DOTCLOCK(1) = '1' and CLKEN_CNT = "001" then
			CLKEN_FF_F <= '1';				
		end if;

		if H_CNT = LINE_CLOCKS-1 then
			CLKEN_CNT <= (others => '0');
			CLKEN_FF <= '1';				
			H_CNT <= (others => '0');
			V_CNT <= V_CNT + 1;
			if V_CNT >= END_LINE-1 then
				V_CNT <= (others => '0');
				if CR(2) = '1' then			-- artifact bit affects number of lines per field; check at start of field
				  END_LINE <= TOTAL_LINES;
				else
				  END_LINE <= TOTAL_LINES - 1;
				end if;
			end if;
			-- Reload registers
			BW_FF <= CR(7);
			DOTCLOCK <= CR(1 downto 0);

			if V_CNT >= TOP_BL_LINES and V_CNT < TOP_BL_LINES + DISP_LINES and DOTCLOCK /= CR(1 downto 0) then 
				MULTIRES_FF <= '1';
			end if;
				
			if V_CNT = TOP_BL_LINES + DISP_LINES then
				MULTIRES <= MULTIRES_FF;
				MULTIRES_FF <= '0';
			end if;
		end if;
		end if;	-- SLEEP guard
	end if;
end process;

HSYNC_START_POS <= 8-1 when DOTCLOCK = "00" else 
                   LINE_CLOCKS-6-1 when DOTCLOCK = "01" else 
                   LINE_CLOCKS-26-1;
HSYNC_END_POS   <= 8+464-1 when DOTCLOCK = "00" else 
                   468-6-1 when DOTCLOCK = "01" else 
                   468-26-1;
process( CLK )
begin
	if rising_edge( CLK ) then
		HSYNC_F <= '0';
		HSYNC_R <= '0';
		VSYNC_F <= '0';
		VSYNC_R <= '0';
		if H_CNT = HSYNC_START_POS then HSYNC_F <= '1'; end if;
		if H_CNT = HSYNC_START_POS + 1 and DOTCLOCK = "01" then HSYNC_F <= '1'; end if;
		if H_CNT = HSYNC_END_POS   then HSYNC_R <= '1'; end if;
		if V_CNT = END_LINE-1    and H_CNT = LINE_CLOCKS-1 then VSYNC_F <= '1'; end if;
		if V_CNT = VS_LINES-1    and H_CNT = LINE_CLOCKS-1 then VSYNC_R <= '1'; end if;
	end if;
end process;

process(CLK, RESET_N)
begin
	if RESET_N = '0' then
		CLKEN_FS_CNT <= (others=>'0');
		CLKEN_FS_FF <= '0';
	elsif rising_edge(CLK) then
		if SaveStateBus_load = '1' then
			-- Savestate restore. CLKEN_FS_CNT restored VERBATIM, never reseeded (plan §5.4).
			CLKEN_FS_CNT <= SS_VCE_2(50 downto 48);
			CLKEN_FS_FF  <= SS_VCE_2(53);
		elsif SLEEP = '0' then	-- savestate freeze: hold divider (output CE gated low below)
		CLKEN_FS_FF <= '0';
		CLKEN_FS_CNT <= CLKEN_FS_CNT + 1;
		if (MULTIRES = '1' or DOTCLOCK(1) = '1') and CLKEN_FS_CNT = "011" and H_CNT < LINE_CLOCKS-2-1 then
			CLKEN_FS_CNT <= (others => '0');
			CLKEN_FS_FF <= '1';
		elsif DOTCLOCK = "00" and CLKEN_FS_CNT = "111" and H_CNT < LINE_CLOCKS-2-1 then
			CLKEN_FS_CNT <= (others => '0');
			CLKEN_FS_FF <= '1';
		elsif DOTCLOCK = "01" and CLKEN_FS_CNT = "101" then
			CLKEN_FS_CNT <= (others => '0');
			CLKEN_FS_FF <= '1';
		end if;

		if H_CNT = LINE_CLOCKS-1 then
			 CLKEN_FS_CNT <= (others => '0');
			 CLKEN_FS_FF <= '1';
		end if;
		end if;	-- SLEEP guard
	end if;
end process;

-- Sync.  SLEEP-gated: during a savestate load reset_ss zeroes V_CNT/H_CNT
-- while these processes would otherwise free-run — V_CNT=0 would latch VS_N
-- ACTIVE for the whole multi-ms load window (a continuous fake vsync into
-- the scaler).  Holding them keeps the frozen sync levels until resume.
process( CLK )
begin
	if rising_edge( CLK ) then
		if SLEEP = '0' then
			if H_CNT = HS_OFF             then HS_N <= '0'; end if;
			if H_CNT = HS_OFF + HS_CLOCKS then HS_N <= '1'; end if;
			if V_CNT = 0                  then VS_N <= '0'; end if;
			if V_CNT = VS_LINES           then VS_N <= '1'; end if;
		end if;
	end if;
end process;

-- Blank (SLEEP-gated for the same reason)
process( CLK )
begin
	if rising_edge( CLK ) then
		if SLEEP = '0' then
			if H_CNT = LEFT_BL_CLOCKS               then HBL_FF <= '0'; end if;
			if H_CNT = LEFT_BL_CLOCKS + DISP_CLOCKS then HBL_FF <= '1'; end if;
			if V_CNT = TOP_BL_LINES                 then VBL_FF <= '0'; end if;
			if V_CNT = TOP_BL_LINES + DISP_LINES    then VBL_FF <= '1'; end if;
		end if;
	end if;
end process;

-- Final output
process( CLK )
begin
	if rising_edge( CLK ) then
		if CLKEN_FF = '1' then

			-- compensate HUC6202 delay
			VBL_FF2 <= VBL_FF;
			HBL_FF2 <= HBL_FF;

			VBL <= VBL_FF2;
			HBL <= HBL_FF2;

			if BORDER = '1' and BORDER_EN = '0' then
				G <= (others => '0');
				R <= (others => '0');
				B <= (others => '0');
				G_FF <= (others => '0');
				R_FF <= (others => '0');
				B_FF <= (others => '0');
				CE_N_FF  <= '1';

			elsif (CE_N = '0') then
				G <= G_FF;
				R <= R_FF;
				B <= B_FF;
				CE_N_FF <= '0';
			elsif (CE_N_FF = '0') then
				G <= G_FF;
				R <= R_FF;
				B <= B_FF;
				CE_N_FF <= '1';

			elsif (GRID(0) = '1' and GRID_EN(0) = '1') or (GRID(1) = '1' and GRID_EN(1) = '1') then
				G <= (others => '1');
				R <= (others => '1');
				B <= (others => '1');
			else
				G <= COLOR(8 downto 6);
				R <= COLOR(5 downto 3);
				B <= COLOR(2 downto 0);
				G_FF <= COLOR(8 downto 6);
				R_FF <= COLOR(5 downto 3);
				B_FF <= COLOR(2 downto 0);
			end if;
		end if;
	end if;
end process;

-- Gated with SLEEP: a freeze landing while CLKEN_FF=1 must NOT leave the
-- VDC clock-enable stuck high (the VDCs would then free-run at CLK speed).
CLKEN <= CLKEN_FF and not SLEEP;
CLKEN_F <= CLKEN_FF_F and not SLEEP;
CLKEN_FS <= CLKEN_FS_FF and not SLEEP;

-- Out-port shadows
DO <= DO_FF;
BW <= BW_FF;

--------------------------------------------------------------------------------
-- SAVESTATES (plan §5.4; slot map: pce_savestates_pkg.vhd)
--------------------------------------------------------------------------------

CTRL_ENC <= "01" when CTRL = CTRL_WAIT else
            "10" when CTRL = CTRL_INCR else
            "00";

-- VCE_1: CR(7:0), DOTCLOCK(9:8), BW(10), RAM_A(20:12), RAM_DI(29:21), RAM_WE(30),
--        CTRL(33:32), DO(43:36), PREV_A(46:44)
SS_VCE_1_BACK(7 downto 0)   <= CR;
SS_VCE_1_BACK(9 downto 8)   <= DOTCLOCK;
SS_VCE_1_BACK(10)           <= BW_FF;
SS_VCE_1_BACK(20 downto 12) <= RAM_A;
SS_VCE_1_BACK(29 downto 21) <= RAM_DI;
SS_VCE_1_BACK(30)           <= RAM_WE;
SS_VCE_1_BACK(33 downto 32) <= CTRL_ENC;
SS_VCE_1_BACK(43 downto 36) <= DO_FF;
SS_VCE_1_BACK(46 downto 44) <= PREV_A;

-- VCE_2: H_CNT(11:0), V_CNT(25:16), END_LINE(41:32), CLKEN_CNT(46:44),
--        CLKEN_FS_CNT(50:48), CLKEN_FF(51), CLKEN_FF_F(52), CLKEN_FS(53),
--        MULTIRES_FF(54), MULTIRES(55)
SS_VCE_2_BACK(11 downto 0)  <= H_CNT;
SS_VCE_2_BACK(25 downto 16) <= V_CNT;
SS_VCE_2_BACK(41 downto 32) <= std_logic_vector(to_unsigned(END_LINE, 10));
SS_VCE_2_BACK(46 downto 44) <= CLKEN_CNT;
SS_VCE_2_BACK(50 downto 48) <= CLKEN_FS_CNT;
SS_VCE_2_BACK(51)           <= CLKEN_FF;
SS_VCE_2_BACK(52)           <= CLKEN_FF_F;
SS_VCE_2_BACK(53)           <= CLKEN_FS_FF;
SS_VCE_2_BACK(54)           <= MULTIRES_FF;
SS_VCE_2_BACK(55)           <= MULTIRES;

iSS_VCE_1 : entity work.eReg_SavestateV
generic map ( Adr => SSREG_INDEX_VCE_1, def => SSREG_DEFAULT_VCE_1 )
port map (
	clk      => CLK,
	BUS_Din  => SaveStateBus_Din,
	BUS_Adr  => SaveStateBus_Adr,
	BUS_wren => SaveStateBus_wren,
	BUS_rst  => SaveStateBus_rst,
	BUS_Dout => SS_Dout_1,
	Din      => SS_VCE_1_BACK,
	Dout     => SS_VCE_1
);

iSS_VCE_2 : entity work.eReg_SavestateV
generic map ( Adr => SSREG_INDEX_VCE_2, def => SSREG_DEFAULT_VCE_2 )
port map (
	clk      => CLK,
	BUS_Din  => SaveStateBus_Din,
	BUS_Adr  => SaveStateBus_Adr,
	BUS_wren => SaveStateBus_wren,
	BUS_rst  => SaveStateBus_rst,
	BUS_Dout => SS_Dout_2,
	Din      => SS_VCE_2_BACK,
	Dout     => SS_VCE_2
);

SaveStateBus_Dout <= SS_Dout_1 or SS_Dout_2;

end rtl;
