-- Simulation stub for the SystemVerilog CODES (game genie) module.
-- Pass-through disabled: no overrides, no availability.
library IEEE;
use IEEE.std_logic_1164.all;

entity CODES is
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
end entity;

architecture sim of CODES is
begin
	available  <= '0';
	genie_ovr  <= false;
	genie_data <= (others => '0');
end architecture;
