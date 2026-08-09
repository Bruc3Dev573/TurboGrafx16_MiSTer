library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.NUMERIC_STD.ALL;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity psg is
	port (
		CLK 	: in std_logic;
		CLKEN	: in std_logic;
		RESET_N	: in std_logic;

		-- CPU Interface
		DI		: in std_logic_vector(7 downto 0);
		A 		: in std_logic_vector(3 downto 0);
		WE		: in std_logic;

		-- DAC Interface
		DAC_LATCH	: in std_logic;
		LDATA		: out std_logic_vector(23 downto 0);
		RDATA		: out std_logic_vector(23 downto 0);

		-- Savestates (plan §5.2)
		SaveStateBus_Din  : in  std_logic_vector(63 downto 0) := (others => '0');
		SaveStateBus_Adr  : in  std_logic_vector(9 downto 0) := (others => '0');
		SaveStateBus_wren : in  std_logic := '0';
		SaveStateBus_rst  : in  std_logic := '0';
		SaveStateBus_load : in  std_logic := '0';
		SaveStateBus_Dout : out std_logic_vector(63 downto 0);
		-- WF_DATA walk (SAVETYPE_PSGWF): addr[7:5]=channel, addr[4:0]=entry.
		-- The SS write path BYPASSES the DDA=0 gate of the CPU write path.
		SS_WF_Addr        : in  std_logic_vector(7 downto 0) := (others => '0');
		SS_WF_WrEn        : in  std_logic := '0';
		SS_WF_WrData      : in  std_logic_vector(4 downto 0) := (others => '0');
		SS_WF_RdData      : out std_logic_vector(4 downto 0)
	);
end psg;

architecture rtl of psg is

-- R0 - Channel Selection
signal CHSEL	: integer range 0 to 7;
-- R1 - Main Volume Adjustement
signal LMAL		: std_logic_vector(3 downto 0);
signal RMAL		: std_logic_vector(3 downto 0);

-- R2-R7 - Channel specific registers
type wavedata_t is array(0 to 31) of std_logic_vector(4 downto 0);
type chan_t is
	record
		-- Registers
		FREQ		: std_logic_vector(11 downto 0);
		DDA		: std_logic;
		CHON		: std_logic;
		AL			: std_logic_vector(4 downto 0);
		LAL		: std_logic_vector(3 downto 0);
		RAL		: std_logic_vector(3 downto 0);

		NG_FREQ	: std_logic_vector(4 downto 0);
		NE			: std_logic;

		-- Waveform generator
		WF_DATA	: wavedata_t;
		WF_ADDR	: std_logic_vector(4 downto 0);
		WF_CNT	: std_logic_vector(12 downto 0);

		WF_RES	: std_logic;
		WF_INC	: std_logic;

		-- Noise generator
		LFSR		: std_logic_vector(17 downto 0);
		NG_CNT	: std_logic_vector(11 downto 0);

		-- Outputs
		DA_OUT	: std_logic_vector(4 downto 0);
		WF_OUT	: std_logic_vector(4 downto 0);
		NG_OUT	: std_logic_vector(4 downto 0);
		-- Global output
		GL_OUT	: std_logic_vector(4 downto 0);

		-- LFO
		LFO_FREQ	: std_logic_vector(7 downto 0);
		LFCTL		: std_logic_vector(1 downto 0);
		LFTRG		: std_logic;
		LFO_CNT	: std_logic_vector(7 downto 0);
		LFO_ADD 	: std_logic_vector(11 downto 0);
	end record;
type chanarray_t is array(0 to 5) of chan_t;
signal CH		: chanarray_t;

-- Channels mixing
signal LACC		: std_logic_vector(23 downto 0);
signal RACC		: std_logic_vector(23 downto 0);

signal VT_ADDR	: std_logic_vector(11 downto 0);
signal VT_DATA	: std_logic_vector(23 downto 0);

type mix_t is ( MIX_WAIT, MIX_NEXT, MIX_LREAD, MIX_LNEXT, MIX_RREAD, MIX_RNEXT, MIX_END );
signal MIX		: mix_t;
signal MIX_CNT	: std_logic_vector(2 downto 0);

signal LDATA_FF	: std_logic_vector(23 downto 0);
signal RDATA_FF	: std_logic_vector(23 downto 0);

-- Savestates
type slv64_array6 is array(0 to 5) of std_logic_vector(63 downto 0);
signal SS_GLOBAL      : std_logic_vector(63 downto 0);
signal SS_GLOBAL_BACK : std_logic_vector(63 downto 0) := (others => '0');
signal SS_Dout_GLOBAL : std_logic_vector(63 downto 0);
signal SS_CH_A,      SS_CH_B,      SS_CH_C      : slv64_array6;
signal SS_CH_A_BACK, SS_CH_B_BACK, SS_CH_C_BACK : slv64_array6 := (others => (others => '0'));
signal SS_Dout_A,    SS_Dout_B,    SS_Dout_C    : slv64_array6;

begin

-- CPU Interface
process( CLK )
begin
	if rising_edge( CLK ) then

		for i in 0 to 5 loop
			CH(i).WF_RES <= '0';
			CH(i).WF_INC <= '0';
		end loop;

		if RESET_N = '0' then

			CHSEL <= 0;
			LMAL <= (others => '0');
			RMAL <= (others => '0');

			for i in 0 to 5 loop
				CH(i).FREQ <= (others => '0');
				CH(i).DDA <= '0';
				CH(i).CHON <= '0';
				CH(i).LAL <= (others => '0');
				CH(i).RAL <= (others => '0');
				CH(i).NG_FREQ <= (others => '0');
				CH(i).NE <= '0';
				CH(i).DA_OUT <= (others => '0');
				CH(i).LFO_FREQ <= (others => '0');
				CH(i).LFCTL <= "00";
				CH(i).LFTRG <= '0';
			end loop;
		elsif SaveStateBus_load = '1' then
			-- Savestate restore (registers driven by this process)
			CHSEL <= conv_integer(SS_GLOBAL(2 downto 0));
			LMAL  <= SS_GLOBAL(7 downto 4);
			RMAL  <= SS_GLOBAL(11 downto 8);
			for i in 0 to 5 loop
				CH(i).FREQ     <= SS_CH_A(i)(11 downto 0);
				CH(i).DDA      <= SS_CH_A(i)(12);
				CH(i).CHON     <= SS_CH_A(i)(13);
				CH(i).AL       <= SS_CH_A(i)(20 downto 16);
				CH(i).LAL      <= SS_CH_A(i)(27 downto 24);
				CH(i).RAL      <= SS_CH_A(i)(31 downto 28);
				CH(i).NG_FREQ  <= SS_CH_A(i)(36 downto 32);
				CH(i).NE       <= SS_CH_A(i)(37);
				CH(i).DA_OUT   <= SS_CH_A(i)(52 downto 48);
				CH(i).LFO_FREQ <= SS_CH_C(i)(7 downto 0);
				CH(i).LFCTL    <= SS_CH_C(i)(9 downto 8);
				CH(i).LFTRG    <= SS_CH_C(i)(10);
			end loop;
		else
			-- WF_DATA walk write (freeze only; bypasses the DDA=0 gate)
			if SS_WF_WrEn = '1' then
				CH(conv_integer(SS_WF_Addr(7 downto 5))).WF_DATA(conv_integer(SS_WF_Addr(4 downto 0))) <= SS_WF_WrData;
			end if;
			if WE = '1' then
				case A is
				when "0000" =>
					CHSEL <= conv_integer(DI(2 downto 0));

				when "0001" =>
					LMAL <= DI(7 downto 4);
					RMAL <= DI(3 downto 0);

				when "0010" =>
					CH(CHSEL).FREQ(7 downto 0) <= DI;

				when "0011" =>
					CH(CHSEL).FREQ(11 downto 8) <= DI(3 downto 0);

				when "0100" =>
					CH(CHSEL).CHON <= DI(7);
					CH(CHSEL).DDA <= DI(6);
					CH(CHSEL).AL <= DI(4 downto 0);
					if CH(CHSEL).DDA = '1' and DI(6) = '0' then
						CH(CHSEL).WF_RES <= '1';
					end if;

				when "0101" =>
					CH(CHSEL).LAL <= DI(7 downto 4);
					CH(CHSEL).RAL <= DI(3 downto 0);

				when "0110" =>
					if CH(CHSEL).DDA = '0' then
						CH(CHSEL).WF_DATA(conv_integer(CH(CHSEL).WF_ADDR)) <= DI(4 downto 0);
					end if;
					if CH(CHSEL).CHON = '1' then
						CH(CHSEL).DA_OUT <= DI(4 downto 0);
					end if;
					if CH(CHSEL).DDA = '0' and CH(CHSEL).CHON = '0' then
						CH(CHSEL).WF_INC <= '1';
					end if;

				when "0111" =>
					if CHSEL = 4 or CHSEL =5 then
						CH(CHSEL).NE <= DI(7);
						CH(CHSEL).NG_FREQ <= DI(4 downto 0);
					end if;

				when "1000" =>
					CH(1).LFO_FREQ <= DI;

				when "1001" =>
					CH(1).LFCTL <= DI(1 downto 0);
					CH(1).LFTRG <= DI(7);

				when others => null;
				end case;
			end if;
		end if;
	end if;
end process;


process( CLK ) begin
	if rising_edge( CLK ) then
		for i in 0 to 5 loop
			if RESET_N = '0' then
				CH(i).GL_OUT <= (others => '0');
				CH(i).WF_CNT <= (others => '0');
				CH(i).LFSR <= (others => '0');
				CH(i).NG_CNT <= (others => '0');
				CH(i).LFO_CNT <= (others => '0');
			elsif SaveStateBus_load = '1' then
				-- Savestate restore (registers driven by this process)
				CH(i).WF_ADDR <= SS_CH_A(i)(44 downto 40);
				CH(i).WF_OUT  <= SS_CH_A(i)(60 downto 56);
				CH(i).WF_CNT  <= SS_CH_B(i)(12 downto 0);
				CH(i).LFSR    <= SS_CH_B(i)(33 downto 16);
				CH(i).NG_CNT  <= SS_CH_B(i)(47 downto 36);
				CH(i).NG_OUT  <= SS_CH_B(i)(52 downto 48);
				CH(i).GL_OUT  <= SS_CH_B(i)(60 downto 56);
				CH(i).LFO_CNT <= SS_CH_C(i)(23 downto 16);
			else
				if CH(i).WF_RES = '1' then
					CH(i).WF_ADDR <= (others => '0');
				end if;
				if CH(i).WF_INC = '1' then
					CH(i).WF_ADDR <= CH(i).WF_ADDR + 1;
				end if;

				if CH(i).LFCTL /= "00" then
					CH(i).WF_OUT <= CH(i).WF_DATA(conv_integer(CH(i).WF_ADDR));

					if CH(i).LFTRG = '1' then
						CH(i).WF_ADDR <= (others => '0');
						CH(i).WF_CNT <= (CH(i).FREQ - 1) & "1";
						CH(i).LFO_CNT <= CH(i).LFO_FREQ - 1;
					else
						if CLKEN = '1' then
							CH(i).LFO_CNT <= CH(i).LFO_CNT - 1;
							if CH(i).LFO_CNT = 0 then
								CH(i).LFO_CNT <= CH(i).LFO_FREQ - 1;
								CH(i).WF_CNT <= CH(i).WF_CNT - 1;
								if CH(i).WF_CNT = 0 then
									CH(i).WF_CNT <= (CH(i).FREQ - 1) & "1";
									CH(i).WF_ADDR <= CH(i).WF_ADDR + 1;
								end if;
							end if;
						end if;
					end if;
				elsif CH(i).CHON = '0' then
					CH(i).WF_CNT <= (CH(i).FREQ - 1 + CH(i).LFO_ADD) & "1";
				elsif CH(i).DDA = '0' then
					CH(i).WF_OUT <= CH(i).WF_DATA(conv_integer(CH(i).WF_ADDR));

					if CLKEN = '1' then
						CH(i).WF_CNT <= CH(i).WF_CNT - 1;
						if CH(i).WF_CNT = 0 then
							CH(i).WF_CNT <= (CH(i).FREQ - 1 + CH(i).LFO_ADD) & "1";
							CH(i).WF_ADDR <= CH(i).WF_ADDR + 1;
						end if;
					end if;
				end if;

				if CH(i).NE = '0' then
				if CH(i).NG_FREQ = "11111" then
						CH(i).NG_CNT <= "000000111111";
					else
						CH(i).NG_CNT <= ( not(CH(i).NG_FREQ) - 1) & "1111111";
					end if;
				else
					if CH(i).LFSR(0) = '0' then
						CH(i).NG_OUT <= "00000";
					else
						CH(i).NG_OUT <= "11111";
					end if;

					if CLKEN = '1' then
						CH(i).NG_CNT <= CH(i).NG_CNT - 1;
						if CH(i).NG_CNT = 0 then
							if CH(i).NG_FREQ = "11111" then
								CH(i).NG_CNT <= "000000111111";
							else
								CH(i).NG_CNT <= ( not(CH(i).NG_FREQ) - 1) & "1111111";
							end if;
							if CH(i).LFSR = 0 then
								CH(i).LFSR(0) <= '1';
							else
								CH(i).LFSR <= (CH(i).LFSR(0) xor CH(i).LFSR(1) xor CH(i).LFSR(11) xor CH(i).LFSR(12) xor CH(i).LFSR(17)) & CH(i).LFSR(17 downto 1);
							end if;
						end if;
					end if;
				end if;

				if CH(i).CHON = '0' then
					CH(i).GL_OUT <= "10000";		-- Not zero; this should be midpoint in the range to reduce 'pop' sound
				elsif CH(i).DDA = '1' then
					CH(i).GL_OUT <= CH(i).DA_OUT;
				elsif CH(i).NE = '1' then
					CH(i).GL_OUT <= CH(i).NG_OUT;
				else
					CH(i).GL_OUT <= CH(i).WF_OUT;
				end if;
			end if;
		end loop;
	end if;
end process;

process( CLK )
	variable DATA: std_logic_vector(4 downto 0);
begin
	if rising_edge( CLK ) then
		if RESET_N = '0' then
			for i in 0 to 5 loop
				CH(i).LFO_ADD <= (others => '0');
			end loop;
		elsif SaveStateBus_load = '1' then
			for i in 0 to 5 loop
				CH(i).LFO_ADD <= SS_CH_C(i)(43 downto 32);
			end loop;
		else
			DATA := CH(1).WF_DATA(conv_integer(CH(1).WF_ADDR)) xor "1000";
			CH(0).LFO_ADD(11 downto 8) <= DATA(4) & DATA(4) & DATA(4) & DATA(4);
			case CH(1).LFCTL is
			when "01" =>   CH(0).LFO_ADD(7 downto 0) <= DATA(4) & DATA(4) & DATA(4) & DATA;
			when "10" =>   CH(0).LFO_ADD(7 downto 0) <= DATA(4) & DATA & "00";
			when "11" =>   CH(0).LFO_ADD(7 downto 0) <= DATA(3 downto 0) & "0000";
			when others => CH(0).LFO_ADD <= (others => '0');
			end case;
		end if;
	end if;
end process;

-- Channels mixing
VT : entity work.dpram generic map (12,24,"HUC6280/voltab.mif")
port map (
	clock		=> CLK,
	address_a=> VT_ADDR,
	q_a		=> VT_DATA
);

process( CLK )
begin
	if rising_edge( CLK ) then
		if RESET_N = '0' then
			LDATA_FF <= (others => '0');
			RDATA_FF <= (others => '0');
			MIX <= MIX_WAIT;
		elsif SaveStateBus_load = '1' then
			-- Savestate restore. The mixer FSM itself is NOT saved: it free-runs
			-- and self-reconstructs in ~13 cycles; LDATA_FF/RDATA_FF cover the
			-- DAC latch meanwhile (plan §5.2 rationale).
			LDATA_FF <= SS_GLOBAL(35 downto 12);
			RDATA_FF <= SS_GLOBAL(59 downto 36);
		else
			case MIX is
			when MIX_WAIT =>
				LACC <= (others => '0');
				RACC <= (others => '0');
				MIX_CNT <= (others => '0');
				VT_ADDR <= (others => '1');
				if DAC_LATCH = '1' then
					MIX <= MIX_NEXT;
				end if;

			when MIX_NEXT =>
				VT_ADDR <= CH(conv_integer(MIX_CNT)).GL_OUT
					& ( "1011101" - CH(conv_integer(MIX_CNT)).AL - (CH(conv_integer(MIX_CNT)).LAL & "1") - (LMAL & "1") );
				MIX <= MIX_LREAD;

			when MIX_LREAD =>
				MIX <= MIX_LNEXT;

			when MIX_LNEXT =>
				LACC <= LACC + VT_DATA;
				VT_ADDR <= CH(conv_integer(MIX_CNT)).GL_OUT
					& ( "1011101" - CH(conv_integer(MIX_CNT)).AL - (CH(conv_integer(MIX_CNT)).RAL & "1") - (RMAL & "1") );
				MIX <= MIX_RREAD;

			when MIX_RREAD =>
				MIX <= MIX_RNEXT;

			when MIX_RNEXT =>
				RACC <= RACC + VT_DATA;
				if MIX_CNT = "101" then
					MIX <= MIX_END;
				else
					MIX_CNT <= MIX_CNT + 1;
					MIX <= MIX_NEXT;
				end if;

			when MIX_END =>
				LDATA_FF <= LACC;
				RDATA_FF <= RACC;
				MIX <= MIX_WAIT;

			when others => null;
			end case;
		end if;
	end if;
end process;

LDATA <= LDATA_FF;
RDATA <= RDATA_FF;

--------------------------------------------------------------------------------
-- SAVESTATES (plan §5.2; slot map: pce_savestates_pkg.vhd)
--------------------------------------------------------------------------------
-- NOT saved: LACC/RACC/VT_ADDR/VT_DATA/MIX/MIX_CNT (mixer self-reconstructs),
-- WF_RES/WF_INC (single-cycle strobes), voltab (constant dpram).

-- WF_DATA walk read (async mux over the flop array).  The address bus is
-- shared with the other Save_RAM regions, whose larger addresses would index
-- channels 6/7: clamp to a valid channel (result only sampled for type PSGWF).
process( SS_WF_Addr, CH )
	variable c : integer range 0 to 7;
begin
	c := conv_integer(SS_WF_Addr(7 downto 5));
	if c > 5 then
		c := 0;
	end if;
	SS_WF_RdData <= CH(c).WF_DATA(conv_integer(SS_WF_Addr(4 downto 0)));
end process;

-- PSG_GLOBAL: CHSEL(2:0), LMAL(7:4), RMAL(11:8), LDATA_FF(35:12), RDATA_FF(59:36)
SS_GLOBAL_BACK(2 downto 0)   <= std_logic_vector(to_unsigned(CHSEL, 3));
SS_GLOBAL_BACK(7 downto 4)   <= LMAL;
SS_GLOBAL_BACK(11 downto 8)  <= RMAL;
SS_GLOBAL_BACK(35 downto 12) <= LDATA_FF;
SS_GLOBAL_BACK(59 downto 36) <= RDATA_FF;

iSS_PSG_GLOBAL : entity work.eReg_SavestateV
generic map ( Adr => SSREG_INDEX_PSG_GLOBAL, def => SSREG_DEFAULT_PSG_GLOBAL )
port map (
	clk      => CLK,
	BUS_Din  => SaveStateBus_Din,
	BUS_Adr  => SaveStateBus_Adr,
	BUS_wren => SaveStateBus_wren,
	BUS_rst  => SaveStateBus_rst,
	BUS_Dout => SS_Dout_GLOBAL,
	Din      => SS_GLOBAL_BACK,
	Dout     => SS_GLOBAL
);

GEN_SS_CH : for i in 0 to 5 generate
	-- CHx_A: FREQ(11:0), DDA(12), CHON(13), AL(20:16), LAL(27:24), RAL(31:28),
	--        NG_FREQ(36:32), NE(37), WF_ADDR(44:40), DA_OUT(52:48), WF_OUT(60:56)
	SS_CH_A_BACK(i)(11 downto 0)  <= CH(i).FREQ;
	SS_CH_A_BACK(i)(12)           <= CH(i).DDA;
	SS_CH_A_BACK(i)(13)           <= CH(i).CHON;
	SS_CH_A_BACK(i)(20 downto 16) <= CH(i).AL;
	SS_CH_A_BACK(i)(27 downto 24) <= CH(i).LAL;
	SS_CH_A_BACK(i)(31 downto 28) <= CH(i).RAL;
	SS_CH_A_BACK(i)(36 downto 32) <= CH(i).NG_FREQ;
	SS_CH_A_BACK(i)(37)           <= CH(i).NE;
	SS_CH_A_BACK(i)(44 downto 40) <= CH(i).WF_ADDR;
	SS_CH_A_BACK(i)(52 downto 48) <= CH(i).DA_OUT;
	SS_CH_A_BACK(i)(60 downto 56) <= CH(i).WF_OUT;

	-- CHx_B: WF_CNT(12:0), LFSR(33:16), NG_CNT(47:36), NG_OUT(52:48), GL_OUT(60:56)
	SS_CH_B_BACK(i)(12 downto 0)  <= CH(i).WF_CNT;
	SS_CH_B_BACK(i)(33 downto 16) <= CH(i).LFSR;
	SS_CH_B_BACK(i)(47 downto 36) <= CH(i).NG_CNT;
	SS_CH_B_BACK(i)(52 downto 48) <= CH(i).NG_OUT;
	SS_CH_B_BACK(i)(60 downto 56) <= CH(i).GL_OUT;

	-- CHx_C: LFO_FREQ(7:0), LFCTL(9:8), LFTRG(10), LFO_CNT(23:16), LFO_ADD(43:32)
	SS_CH_C_BACK(i)(7 downto 0)   <= CH(i).LFO_FREQ;
	SS_CH_C_BACK(i)(9 downto 8)   <= CH(i).LFCTL;
	SS_CH_C_BACK(i)(10)           <= CH(i).LFTRG;
	SS_CH_C_BACK(i)(23 downto 16) <= CH(i).LFO_CNT;
	SS_CH_C_BACK(i)(43 downto 32) <= CH(i).LFO_ADD;

	iSS_CH_A : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_PSG_CH0 + i*3, def => SSREG_DEFAULT_PSG_CH )
	port map (
		clk      => CLK,
		BUS_Din  => SaveStateBus_Din,
		BUS_Adr  => SaveStateBus_Adr,
		BUS_wren => SaveStateBus_wren,
		BUS_rst  => SaveStateBus_rst,
		BUS_Dout => SS_Dout_A(i),
		Din      => SS_CH_A_BACK(i),
		Dout     => SS_CH_A(i)
	);

	iSS_CH_B : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_PSG_CH0 + i*3 + 1, def => SSREG_DEFAULT_PSG_CH )
	port map (
		clk      => CLK,
		BUS_Din  => SaveStateBus_Din,
		BUS_Adr  => SaveStateBus_Adr,
		BUS_wren => SaveStateBus_wren,
		BUS_rst  => SaveStateBus_rst,
		BUS_Dout => SS_Dout_B(i),
		Din      => SS_CH_B_BACK(i),
		Dout     => SS_CH_B(i)
	);

	iSS_CH_C : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_PSG_CH0 + i*3 + 2, def => SSREG_DEFAULT_PSG_CH )
	port map (
		clk      => CLK,
		BUS_Din  => SaveStateBus_Din,
		BUS_Adr  => SaveStateBus_Adr,
		BUS_wren => SaveStateBus_wren,
		BUS_rst  => SaveStateBus_rst,
		BUS_Dout => SS_Dout_C(i),
		Din      => SS_CH_C_BACK(i),
		Dout     => SS_CH_C(i)
	);
end generate;

SaveStateBus_Dout <= SS_Dout_GLOBAL
	or SS_Dout_A(0) or SS_Dout_B(0) or SS_Dout_C(0)
	or SS_Dout_A(1) or SS_Dout_B(1) or SS_Dout_C(1)
	or SS_Dout_A(2) or SS_Dout_B(2) or SS_Dout_C(2)
	or SS_Dout_A(3) or SS_Dout_B(3) or SS_Dout_C(3)
	or SS_Dout_A(4) or SS_Dout_B(4) or SS_Dout_C(4)
	or SS_Dout_A(5) or SS_Dout_B(5) or SS_Dout_C(5);

end rtl;

