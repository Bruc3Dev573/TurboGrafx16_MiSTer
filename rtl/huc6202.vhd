--============================================================================
--  HUC6202
--  Copyright (C) 2018 Sorgelig
--
--  This program is free software; you can redistribute it and/or modify it
--  under the terms of the GNU General Public License as published by the Free
--  Software Foundation; either version 2 of the License, or (at your option)
--  any later version.
--
--  This program is distributed in the hope that it will be useful, but WITHOUT
--  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
--  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
--  more details.
--
--  You should have received a copy of the GNU General Public License along
--  with this program; if not, write to the Free Software Foundation, Inc.,
--  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
--============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.NUMERIC_STD.ALL;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity huc6202 is
	port (
		CLK 		: in std_logic;
		CLKEN		: in std_logic;
		RESET_N	: in std_logic;

		A			: in std_logic_vector(2 downto 0);
		WR_N		: in std_logic;
		DI			: in std_logic_vector(7 downto 0);
		DO 		: out std_logic_vector(7 downto 0);

		HS_F		: in std_logic;
		VDC0_IN	: in std_logic_vector(8 downto 0);
		VDC1_IN	: in std_logic_vector(8 downto 0);
		VDC_OUT	: out std_logic_vector(8 downto 0);

		SGX		: in std_logic;

		VDCNUM	: out std_logic;

		-- Savestates (plan §5.5)
		SaveStateBus_Din  : in  std_logic_vector(63 downto 0) := (others => '0');
		SaveStateBus_Adr  : in  std_logic_vector(9 downto 0) := (others => '0');
		SaveStateBus_wren : in  std_logic := '0';
		SaveStateBus_rst  : in  std_logic := '0';
		SaveStateBus_load : in  std_logic := '0';
		SaveStateBus_Dout : out std_logic_vector(63 downto 0)
	);
end huc6202;

architecture rtl of huc6202 is

signal PRI0 : std_logic_vector(7 downto 0);
signal PRI1 : std_logic_vector(7 downto 0);
signal WIN1 : std_logic_vector(9 downto 0);
signal WIN2 : std_logic_vector(9 downto 0);
signal X    : std_logic_vector(9 downto 0);
signal PRIN : std_logic_vector(1 downto 0);
signal PRI  : std_logic_vector(3 downto 0);

signal VDC_PRI		: std_logic_vector(8 downto 0);

--Savestates
signal VDCNUM_FF : std_logic;
signal DO_FF     : std_logic_vector(7 downto 0);
signal SS_VPC      : std_logic_vector(63 downto 0);
signal SS_VPC_BACK : std_logic_vector(63 downto 0) := (others => '0');

begin

PRIN(0) <= '1' when WIN1 <= x"40" or X >= WIN1 else '0';
PRIN(1) <= '1' when WIN2 <= x"40" or X >= WIN2 else '0';
PRI <= PRI0(3 downto 0) when PRIN = "00" else
		 PRI0(7 downto 4) when PRIN = "01" else
		 PRI1(3 downto 0) when PRIN = "10" else
		 PRI1(7 downto 4);

VDC_PRI <= VDC0_IN when VDC0_IN(3 downto 0) /= "0000" else VDC1_IN;

process( CLK )
	variable VDCDATA : std_logic_vector(8 downto 0);
begin
	if rising_edge(CLK) then
		if SaveStateBus_load = '1' then
			-- Savestate restore.  VDC_OUT (video pipeline stage) is NOT saved:
			-- re-derived on the first CLKEN from the two frozen VDC outputs.
			X <= SS_VPC(49 downto 40);
		elsif CLKEN = '1' then
			X <= X + 1;
			if HS_F = '1' then
				X <= (others => '0');
			end if;

			case PRI(1 downto 0) is
				when "00" =>
					VDCDATA := (others => '0');
				when "01" =>
					VDCDATA := VDC0_IN;
				when "10" =>
					VDCDATA := VDC1_IN;
				when others =>
					VDCDATA := VDC_PRI;
					case PRI(3 downto 2) is
						when "01" =>
							if VDC1_IN(8) = '1' and VDC0_IN(8) = '0' and VDC1_IN(3 downto 0) /= "0000" then
								VDCDATA := VDC1_IN;
							end if;
						when "10" =>
							if VDC1_IN(8) = '0' and VDC0_IN(8) = '1' and VDC1_IN(3 downto 0) /= "0000" then
								VDCDATA := VDC1_IN;
							end if;
						when others => null;
					end case;
			end case;

			VDC_OUT <= VDCDATA;
		end if;
	end if;
end process;

process( CLK ) begin
	if rising_edge(CLK) then
		if RESET_N = '0' then
			PRI0 <= "00010001";
			PRI1 <= "00010001";
			WIN1 <= (others => '0');
			WIN2 <= (others => '0');
			VDCNUM_FF <= '0';
			DO_FF <= X"FF";
		elsif SaveStateBus_load = '1' then
			-- Savestate restore
			PRI0      <= SS_VPC(7 downto 0);
			PRI1      <= SS_VPC(15 downto 8);
			WIN1      <= SS_VPC(25 downto 16);
			WIN2      <= SS_VPC(35 downto 26);
			VDCNUM_FF <= SS_VPC(36);
			DO_FF     <= SS_VPC(57 downto 50);
		else
			if WR_N = '0' then
				case A is
					when "000" => PRI0 <= DI;
					when "001" => PRI1 <= DI;
					when "010" => WIN1(7 downto 0) <= DI;
					when "011" => WIN1(9 downto 8) <= DI(1 downto 0);
					when "100" => WIN2(7 downto 0) <= DI;
					when "101" => WIN2(9 downto 8) <= DI(1 downto 0);
					when "110" => VDCNUM_FF <= DI(0);
					when others => null;
				end case;
			end if;
			case A is
				when "000" => DO_FF <= PRI0;
				when "001" => DO_FF <= PRI1;
				when "010" => DO_FF <= WIN1(7 downto 0);
				when "011" => DO_FF <= "000000" & WIN1(9 downto 8);
				when "100" => DO_FF <= WIN2(7 downto 0);
				when "101" => DO_FF <= "000000" & WIN2(9 downto 8);
				when others => DO_FF <= X"00";
			end case;
		end if;
	end if;
end process;

DO     <= DO_FF;
VDCNUM <= VDCNUM_FF;

--------------------------------------------------------------------------------
-- SAVESTATES (plan §5.5; slot map: pce_savestates_pkg.vhd)
--------------------------------------------------------------------------------
-- VPC: PRI0(7:0), PRI1(15:8), WIN1(25:16), WIN2(35:26), VDCNUM(36), X(49:40),
--      DO(57:50).  PRI/PRIN/VDC_PRI are combinational (correctly excluded).
SS_VPC_BACK(7 downto 0)   <= PRI0;
SS_VPC_BACK(15 downto 8)  <= PRI1;
SS_VPC_BACK(25 downto 16) <= WIN1;
SS_VPC_BACK(35 downto 26) <= WIN2;
SS_VPC_BACK(36)           <= VDCNUM_FF;
SS_VPC_BACK(49 downto 40) <= X;
SS_VPC_BACK(57 downto 50) <= DO_FF;

iSS_VPC : entity work.eReg_SavestateV
generic map ( Adr => SSREG_INDEX_VPC, def => SSREG_DEFAULT_VPC )
port map (
	clk      => CLK,
	BUS_Din  => SaveStateBus_Din,
	BUS_Adr  => SaveStateBus_Adr,
	BUS_wren => SaveStateBus_wren,
	BUS_rst  => SaveStateBus_rst,
	BUS_Dout => SaveStateBus_Dout,
	Din      => SS_VPC_BACK,
	Dout     => SS_VPC
);

end rtl;
