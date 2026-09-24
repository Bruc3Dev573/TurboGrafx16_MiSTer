library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

-- Shared CD/Arcade SDRAM access; payload stays in external RAM.
-- Byte pacing is calibrated for the external CD memory interface.
entity ss_external_ram is
	port (
		clk, sleep_ss, frozen, saving : in std_logic;
		region : in unsigned(3 downto 0);
		walk_addr : in std_logic_vector(24 downto 0);
		walk_wr : in std_logic;
		walk_data : in std_logic_vector(7 downto 0);
		walk_q : out std_logic_vector(7 downto 0);
		walk_ready, active : out std_logic;
		ram_addr : out std_logic_vector(21 downto 0);
		ram_data : out std_logic_vector(7 downto 0);
		ram_q : in std_logic_vector(7 downto 0);
		ram_ready : in std_logic;
		ram_rd : buffer std_logic;
		ram_wr, kick : out std_logic;
		-- eReg 67 holds CD-only diagnostics and is not restored.
		debug_no_dip : buffer unsigned(15 downto 0) := (others => '0');
		debug_timeout, debug_max : buffer unsigned(7 downto 0) := (others => '0')
	);
end entity;

architecture rtl of ss_external_ram is
	signal borrowing : std_logic;
	signal count : unsigned(7 downto 0);
	signal previous : std_logic_vector(24 downto 0);
	signal busy, ready, seen : std_logic;
begin
	-- Neither the pending freeze request nor post-load refetch owns this port.
	borrowing <= '1' when sleep_ss = '1' and frozen = '1' and
	                         (region = 10 or region = 11) else '0';
	active <= borrowing;
	walk_ready <= ready when region = 10 or region = 11 else '1';

	process(clk)
	begin
		if rising_edge(clk) then
			kick <= '0';
			if borrowing = '0' then
				previous <= (others => '1');
				ram_rd <= '0';
				ram_wr <= '0';
				busy <= '0';
				ready <= '0';
				seen <= '0';
			else
				if busy = '1' then
					count <= count + 1;
					-- Reissue every 16 clocks regardless of ram_ready or seen.
					-- Sample when count >= 21 and ram_ready is high, or at timeout.
					-- seen is diagnostic only; gating kicks on it can corrupt lane 0.
					if ram_ready = '0' then seen <= '1'; end if;
					if count(3 downto 0) = "0010" then kick <= '1'; end if;
					if count >= 21 and (ram_ready = '1' or count = 255) then
						if ram_rd = '1' then walk_q <= ram_q; end if;
						if region = 10 then
							if ram_rd = '1' and seen = '0' and debug_no_dip /= x"FFFF" then
								debug_no_dip <= debug_no_dip + 1;
							end if;
							if count = 255 and debug_timeout /= x"FF" then
								debug_timeout <= debug_timeout + 1;
							end if;
							if count > debug_max then debug_max <= count; end if;
						end if;
						ram_rd <= '0';
						ram_wr <= '0';
						busy <= '0';
						ready <= '1';
					end if;
				elsif saving = '1' and walk_addr /= previous then
					previous <= walk_addr;
					if region = 10 then ram_addr <= "1000" & walk_addr(17 downto 0);
					else ram_addr <= '0' & walk_addr(20 downto 0); end if;
					ram_rd <= '1';
					count <= (others => '0');
					busy <= '1';
					ready <= '0';
					seen <= '0';
				elsif walk_wr = '1' then
					if region = 10 then ram_addr <= "1000" & walk_addr(17 downto 0);
					else ram_addr <= '0' & walk_addr(20 downto 0); end if;
					ram_data <= walk_data;
					ram_wr <= '1';
					count <= (others => '0');
					busy <= '1';
					ready <= '0';
					seen <= '0';
				else
					ready <= '1';
				end if;
			end if;
		end if;
	end process;
end architecture;
