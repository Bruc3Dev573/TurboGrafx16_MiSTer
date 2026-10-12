library ieee;
use ieee.std_logic_1164.all;

entity tb_statemanager is end entity;

architecture sim of tb_statemanager is
	constant BASE : integer := 16#3800000#;
	constant STRIDE : integer := 16#C0000#;
	signal clk : std_logic := '0';
	signal reset : std_logic := '1';
	signal save, load : std_logic := '0';
	signal busy : std_logic := '1';
	signal slot : integer := 0;
	signal req_save, req_load : std_logic;
	signal req_slot : integer range 0 to 3;
	signal req_addr : integer;
begin
	clk <= not clk after 5 ns;
	DUT : entity work.statemanager
	generic map (Softmap_SaveState_ADDR => BASE)
	port map (
		clk => clk, reset => reset,
		savestate_number => slot, save => save, load => load,
		request_savestate => req_save,
		request_loadstate => req_load, request_address => req_addr,
		request_slot => req_slot, request_busy => busy
	);

	watchdog : process begin
		wait for 10 us;
		assert false report "statemanager test timed out" severity failure;
	end process;

	stim : process
		procedure tick(n : positive := 1) is begin
			for i in 1 to n loop
				wait until rising_edge(clk); wait for 1 ns;
			end loop;
		end procedure;
		procedure pulse(signal button : out std_logic) is begin
			button <= '1'; tick; button <= '0'; tick;
		end procedure;
		procedure expect_request(is_save : boolean; expected_slot : integer) is begin
			assert (is_save and req_save = '1' and req_load = '0') or
				(not is_save and req_load = '1' and req_save = '0')
				report "missing request or wrong queue priority" severity failure;
			assert req_slot = expected_slot and req_addr = BASE + expected_slot * STRIDE
				report "request lost its captured slot/address" severity failure;
		end procedure;
		procedure expect_idle(n : positive := 1) is begin
			for i in 1 to n loop
				tick;
				assert req_save = '0' and req_load = '0'
					report "unexpected or repeated request" severity failure;
			end loop;
		end procedure;
		procedure acknowledge is begin
			busy <= '1'; expect_idle(2);
			busy <= '0'; tick;
		end procedure;
	begin
		tick(2); reset <= '0'; tick;

		-- Both entries retain their first edge, not the latest selector value.
		slot <= 0; pulse(save);
		slot <= 1; pulse(load);
		slot <= 2; pulse(save); pulse(load);
		slot <= 3; busy <= '0'; tick;
		expect_request(true, 0);
		-- Delay the busy acknowledgement beyond the dispatch pulse.
		expect_idle(5);
		assert req_slot = 0 and req_addr = BASE
			report "active metadata changed before acknowledgement" severity failure;
		busy <= '1'; expect_idle(4);
		assert req_slot = 0 and req_addr = BASE
			report "active metadata changed while busy" severity failure;
		busy <= '0'; tick;
		expect_request(false, 1);
		acknowledge; expect_idle(4);

		-- Dequeue plus a same-kind edge must leave the new entry queued.
		for operation in 0 to 1 loop
			busy <= '1'; slot <= 0; tick;
			if operation = 0 then pulse(save); else pulse(load); end if;
			slot <= 2; busy <= '0';
			if operation = 0 then save <= '1'; else load <= '1'; end if;
			tick; expect_request(operation = 0, 0);
			save <= '0'; load <= '0';
			expect_idle(3);
			acknowledge; expect_request(operation = 0, 2);
			acknowledge; expect_idle(3);
		end loop;

		-- An opposite-kind edge during dispatch also survives the busy window.
		for operation in 0 to 1 loop
			busy <= '1'; slot <= 1; tick;
			if operation = 0 then pulse(save); else pulse(load); end if;
			slot <= 3; busy <= '0';
			if operation = 0 then load <= '1'; else save <= '1'; end if;
			tick; expect_request(operation = 0, 1);
			save <= '0'; load <= '0'; expect_idle(3);
			acknowledge; expect_request(operation /= 0, 3);
			acknowledge; expect_idle(3);
		end loop;

		-- Reset/inhibit cancels queued entries and edges held through release.
		busy <= '1'; pulse(save); pulse(load);
		reset <= '1'; save <= '1'; load <= '1'; tick(2);
		reset <= '0'; busy <= '0'; expect_idle(4);
		save <= '0'; load <= '0'; expect_idle(2);

		-- Cancellation also releases an unacknowledged dispatch for later input.
		slot <= 0; pulse(save); expect_request(true, 0);
		reset <= '1'; tick;
		reset <= '0'; expect_idle(2);
		slot <= 2; pulse(load); expect_request(false, 2);
		acknowledge; expect_idle(3);

		report "tb_statemanager PASSED" severity note;
		std.env.finish;
	end process;
end architecture;
