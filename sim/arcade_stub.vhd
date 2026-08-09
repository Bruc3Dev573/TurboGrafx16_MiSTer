-- Sim stub for ARCADE_CARD (the real one is rtl/arcade.sv, SystemVerilog,
-- invisible to GHDL: an unbound component drives every output to 'U' and,
-- with CD_EN=1, the metavalue reaches ROM_RD through the AC_RAM_CS_N term
-- and kills the instruction fetch).  Benign inactive-card behaviour.
library IEEE;
use IEEE.std_logic_1164.all;

entity ARCADE_CARD is
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
end entity;

architecture stub of ARCADE_CARD is
begin
	DO       <= x"FF";
	SEL_N    <= '1';
	RAM_CS_N <= '1';
	RAM_A    <= (others => '0');
end architecture;
