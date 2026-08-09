-- Targeted test: does the to_integer(unsigned(SS_SLOT)) PORT-MAP conversion
-- (pce_top.vhd -> statemanager) actually pass the slot bits through?
-- Replicates the exact construct, drives every slot value, checks the
-- resulting request_address.
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity slotaddr_wrap is
	port(
		clk      : in  std_logic;
		reset    : in  std_logic;
		SS_SLOT  : in  std_logic_vector(1 downto 0);
		save     : in  std_logic;
		req_save : out std_logic;
		req_addr : out integer
	);
end entity;

architecture rtl of slotaddr_wrap is
begin
	-- the construct under test, verbatim from pce_top.vhd
	SSMANAGER : entity work.statemanager
	generic map (
		Softmap_SaveState_ADDR => 16#3800000#,
		Softmap_Rewind_ADDR    => 16#3800000#
	)
	port map (
		clk               => clk,
		reset             => reset,
		rewind_on         => '0',
		rewind_active     => '0',
		savestate_number  => to_integer(unsigned(SS_SLOT)),
		save              => save,
		load              => '0',
		sleep_rewind      => open,
		vsync             => '0',
		request_savestate => req_save,
		request_loadstate => open,
		request_address   => req_addr,
		request_busy      => '0'
	);
end architecture;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity tb_slotaddr is
end entity;

architecture sim of tb_slotaddr is
	signal clk      : std_logic := '0';
	signal reset    : std_logic := '1';
	signal slot     : std_logic_vector(1 downto 0) := "00";
	signal save     : std_logic := '0';
	signal req_save : std_logic;
	signal req_addr : integer;
	signal addr_seen : integer := -1;
begin
	latch : process(clk)
	begin
		if rising_edge(clk) then
			if req_save = '1' then addr_seen <= req_addr; end if;
		end if;
	end process;
	clk <= not clk after 10 ns;

	uut : entity work.slotaddr_wrap
	port map (clk => clk, reset => reset, SS_SLOT => slot, save => save,
	          req_save => req_save, req_addr => req_addr);

	process
		variable errors : integer := 0;
		procedure try(n : integer) is
		begin
			slot <= std_logic_vector(to_unsigned(n, 2));
			wait for 100 ns;
			save <= '1';  wait for 60 ns;  save <= '0';
			wait for 500 ns;
			assert addr_seen /= -1 report "no save request for slot " & integer'image(n) severity failure;
			if addr_seen /= 16#3800000# + n * 16#C0000# then
				errors := errors + 1;
				report "SLOT " & integer'image(n) & " WRONG ADDRESS: got " &
					integer'image(addr_seen) & " expected " &
					integer'image(16#3800000# + n * 16#C0000#) severity error;
			else
				report "slot " & integer'image(n) & " ok: addr=" & integer'image(addr_seen);
			end if;
			wait for 200 ns;
		end procedure;
	begin
		wait for 50 ns;
		reset <= '0';
		wait for 50 ns;
		try(0); try(1); try(2); try(3);
		if errors = 0 then
			report "TB_SLOTADDR PASS";
		else
			report "TB_SLOTADDR FAIL" severity failure;
		end if;
		std.env.finish;
	end process;
end architecture;
