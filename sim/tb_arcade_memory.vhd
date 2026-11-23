library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

-- Real savestates + the same physical-port bridge used by pce_top.
-- Synthetic SDRAM/DDR timing, not proof of either real memory controller.
-- Character arrays keep the 2MB payload compact on GHDL mcode. No AC stub.
entity tb_arcade_memory is
	generic (CD_BYTES : positive := 262144; G_FAULTS : boolean := false);
end entity;
architecture sim of tb_arcade_memory is
	constant AC_BYTES : integer := 2097152;
	constant CD_BASE : integer := 16#200000#;
	constant SLOT_BASE : integer := 16#3980000#; -- slot 2, DWORD address
	constant SLOT_BYTES : integer := 16#300000#;
	constant FULL_DWORDS : integer := 16#A0532# - (262144 - CD_BYTES) / 4;
	constant SHORT_DWORDS : integer := FULL_DWORDS - AC_BYTES / 4;
	constant CD_OFFSET : integer := 8 + 128 * 8 + 266432;
	constant AC_OFFSET : integer := CD_OFFSET + CD_BYTES;
	constant SENTINEL : character := character'val(16#D7#);
	type bytes_t is array(integer range <>) of character;
	function pattern(a : natural) return character is
		variable v : unsigned(31 downto 0) := to_unsigned(a + 1, 32);
	begin
		-- All physical address bits influence the payload, unlike a low-byte ramp.
		v := v xor shift_left(v, 13);
		v := v xor shift_right(v, 17);
		v := v xor shift_right(v, 8) xor shift_right(v, 16) xor shift_right(v, 24);
		v := v xor shift_left(v, 5);
		return character'val(to_integer(v(7 downto 0)));
	end function;
	function byte(c : character) return std_logic_vector is
	begin return std_logic_vector(to_unsigned(character'pos(c), 8)); end function;

	signal clk : std_logic := '0';
	signal reset : std_logic := '1';
	signal save, load, abort_save : std_logic := '0';
	signal busy, failed, sleeping, saving, loaded, committed, reset_delay : std_logic;
	signal arcade_used : std_logic := '0';
	signal arcade_loaded, loaded_mode : std_logic := '0';
	signal expected_arcade : std_logic := '0';
	signal expected_dwords : natural := SHORT_DWORDS;
	signal expected_header_count : natural := 2;
	signal frozen : std_logic := '0';
	signal addr : std_logic_vector(24 downto 0);
	signal region : unsigned(3 downto 0);
	signal wr, rd, ready, walk_error : std_logic;
	signal data, q, ext_q : std_logic_vector(7 downto 0);
	signal ram_addr : std_logic_vector(21 downto 0);
	signal ram_data, ram_q : std_logic_vector(7 downto 0) := (others => '0');
	signal ram_rd, ram_wr, kick, active : std_logic;
	signal ram_ready : std_logic := '1';
	signal fault_addr : integer := -1;
	signal release_fault : boolean := false;
	signal restored_cd_bytes : natural := CD_BYTES;
	signal last_reads, last_writes, console_resets : natural := 0;
	signal ddr_adr : std_logic_vector(25 downto 0);
	signal ddr_din, ddr_dout : std_logic_vector(63 downto 0) := (others => '0');
	signal ddr_be : std_logic_vector(7 downto 0);
	signal ddr_ena, ddr_rnw : std_logic;
	signal ddr_done : std_logic := '0';
	signal seed_memory, overwrite, check_memory, check_blob : std_logic := '0';
	signal memory_checked, blob_checked : std_logic := '0';
	signal header : std_logic_vector(63 downto 0) := (others => '0');
	signal cycles, transfer_cycles, ddr_writes, ddr_reads, mem_reads, mem_writes : natural := 0;
	signal load_count : natural := 0;
begin
	clk <= not clk after 5 ns;
	q <= ext_q when region = 10 or region = 11 else x"00";

	engine : entity work.savestates
	generic map (CDRAM_BYTES => CD_BYTES)
	port map (
		clk => clk, reset_in => reset, reset_ss => open, reset_delay => reset_delay,
		load_done => loaded, increaseSSHeaderCount => '1', save => save,
		save_abort => abort_save, load => load, savestate_address => SLOT_BASE,
		arcade_used => arcade_used, arcade_loaded => arcade_loaded,
		savestate_busy => busy, save_failed => failed, paused => frozen,
		BUS_Din => open, BUS_Adr => open, BUS_wren => open, BUS_rst => open,
		BUS_Dout => (others => '0'), loading_savestate => committed,
		saving_savestate => saving, sleep_savestate => sleeping,
		Save_RAMAddr => addr, Save_RAMRdEn => rd, Save_RAMWrEn => wr,
		Save_RAMWriteData => data, Save_RAMReadData => q, Save_RAMRdy => ready,
		Save_RAMError => walk_error,
		Save_RAMType => region, bus_out_Din => ddr_din, bus_out_Dout => ddr_dout,
		bus_out_Adr => ddr_adr, bus_out_rnw => ddr_rnw, bus_out_ena => ddr_ena,
		bus_out_be => ddr_be, bus_out_done => ddr_done
	);
	bridge : entity work.ss_external_ram
	port map (
		clk => clk, sleep_ss => sleeping, frozen => frozen, saving => saving,
		region => region, walk_addr => addr, walk_wr => wr, walk_data => data,
		walk_q => ext_q, walk_ready => ready, walk_error => walk_error, active => active,
		ram_addr => ram_addr, ram_data => ram_data, ram_q => ram_q,
		ram_ready => ram_ready, ram_rd => ram_rd, ram_wr => ram_wr, kick => kick
	);

	process(clk)
		variable freeze_delay : natural := 0;
	begin
		if rising_edge(clk) then
			cycles <= cycles + 1;
			if active = '1' then transfer_cycles <= transfer_cycles + 1; end if;
			if sleeping = '0' then frozen <= '0'; freeze_delay := 0;
			elsif freeze_delay = 4 then frozen <= '1';
			else freeze_delay := freeze_delay + 1; end if;
			if loaded = '1' then
				load_count <= load_count + 1;
				loaded_mode <= arcade_loaded;
			end if;
			assert active /= '1' or (sleeping = '1' and frozen = '1')
				report "port borrowed before freeze/after walk" severity failure;
			-- pce_top samples reset_delay to cold-reset the CPU, ROM bank and AC flag.
			if reset_delay = '1' then console_resets <= console_resets + 1; end if;
		end if;
	end process;

	physical_memory : process
		-- AC [0,2MB), CD [2MB,2.25MB), plus two untouched window sentinels.
		variable mem : bytes_t(0 to 16#3FFFFF#) := (others => SENTINEL);
		variable pending, served : boolean := false;
		variable latency, a, byte_cycles : natural := 0;
		variable ac_span : positive := AC_BYTES;
		variable complete : boolean;
		variable write_access : boolean := false;
		variable wdata : character;
	begin
		-- The faulting full save must stop inside the first Arcade word.
		if G_FAULTS then ac_span := 8; end if;
		for i in 0 to ac_span - 1 loop mem(i) := pattern(i); end loop;
		for i in 0 to CD_BYTES - 1 loop mem(CD_BASE + i) := pattern(CD_BASE + i); end loop;
		loop
			wait until rising_edge(clk);
			-- Idle busy pulses as well as address-dependent transaction latency.
			if cycles mod 29 < 3 then ram_ready <= '0'; else ram_ready <= '1'; end if;
			if ram_rd = '0' and ram_wr = '0' then
				assert not pending or a = fault_addr report "external byte cancelled before completion" severity failure;
				if served and a = CD_BASE + CD_BYTES - 1 and a /= fault_addr then
					assert byte_cycles = 256 and ready = '1' and walk_error = '0'
						report "did not accept the last legal acknowledgement" severity failure;
					if write_access then last_writes <= last_writes + 1;
					else last_reads <= last_reads + 1; end if;
				end if;
				byte_cycles := 0; served := false;
			else
				byte_cycles := byte_cycles + 1;
				assert byte_cycles <= 256 report "external byte exceeded timeout" severity failure;
			end if;
			if pending then
				assert active = '1' report "memory ownership lost during byte" severity failure;
				ram_ready <= '0';
				if a = fault_addr then
					-- A transaction already accepted by the controller retires even after request drops.
					complete := release_fault;
				elsif a = CD_BASE + CD_BYTES - 1 then
					-- Drive ready/data before the 256th sampling edge (counter 255).
					complete := byte_cycles = 255;
				else
					complete := latency = 0;
					if not complete then latency := latency - 1; end if;
				end if;
				if complete then
					if write_access then mem(a) := wdata; mem_writes <= mem_writes + 1;
					else ram_q <= byte(mem(a)); mem_reads <= mem_reads + 1; end if;
					pending := false;
					ram_ready <= '1';
				end if;
			elsif kick = '1' and (ram_rd = '1' or ram_wr = '1') and not served then
				assert active = '1' and ram_rd /= ram_wr report "invalid external request" severity failure;
				a := to_integer(unsigned(ram_addr));
				if region = 10 then
					assert a >= CD_BASE and a < CD_BASE + CD_BYTES report "CD window alias" severity failure;
				else
					assert region = 11 and a < ac_span report "AC access outside expected window" severity failure;
					assert expected_arcade = '1' report "short snapshot accessed AC RAM" severity failure;
				end if;
				latency := 5 + (a mod 5) * 7;
				if a = AC_BYTES - 1 then latency := 43; end if;
				write_access := ram_wr = '1';
				if write_access then wdata := character'val(to_integer(unsigned(ram_data))); end if;
				pending := true; served := true; ram_ready <= '0';
			end if;
			if seed_memory = '1' or overwrite = '1' then
				assert busy = '0' and not pending severity failure;
				for i in 0 to ac_span - 1 loop
					if overwrite = '1' then mem(i) := character'val(255 - character'pos(pattern(i)));
					else mem(i) := pattern(i); end if;
				end loop;
				for i in 0 to CD_BYTES - 1 loop
					if overwrite = '1' then
						mem(CD_BASE + i) := character'val(255 - character'pos(pattern(CD_BASE + i)));
					else mem(CD_BASE + i) := pattern(CD_BASE + i); end if;
				end loop;
			end if;
			if check_memory = '1' then
				for i in 0 to ac_span - 1 loop
					if expected_arcade = '1' then
						assert mem(i) = pattern(i) report "AC roundtrip mismatch at " & integer'image(i) severity failure;
					else
						assert mem(i) = character'val(255 - character'pos(pattern(i)))
							report "short load changed AC RAM at " & integer'image(i) severity failure;
					end if;
				end loop;
				for i in 0 to CD_BYTES - 1 loop
					if i < restored_cd_bytes then
						assert mem(CD_BASE + i) = pattern(CD_BASE + i)
							report "CD restore mismatch at " & integer'image(i) severity failure;
					else
						assert mem(CD_BASE + i) = character'val(255 - character'pos(pattern(CD_BASE + i)))
							report "failed load overwrote CD suffix at " & integer'image(i) severity failure;
					end if;
				end loop;
				assert mem(CD_BASE + CD_BYTES) = SENTINEL and mem(16#3FFFFF#) = SENTINEL
					report "external window sentinel overwritten" severity failure;
				memory_checked <= '1';
			else
				memory_checked <= '0';
			end if;
		end loop;
	end process;

	ddr_memory : process(clk)
		-- Empty slot with change counter 1; a failed save must not advance it.
		variable mem : bytes_t(-8 to SLOT_BYTES + 7) :=
			(0 => character'val(1), 1 to 7 => character'val(0), others => SENTINEL);
		variable pending : boolean := false;
		variable delay, index : natural := 0;
		variable write_access : boolean;
		variable wdata : std_logic_vector(63 downto 0);
		variable enables : std_logic_vector(7 downto 0);
	begin
		if rising_edge(clk) then
			ddr_done <= '0';
			if pending then
				assert busy = '1' report "engine returned idle with a slot grant outstanding" severity failure;
				if delay = 0 then
					for lane in 0 to 7 loop
						if write_access then
							if enables(lane) = '1' then
								mem(index + lane) := character'val(to_integer(unsigned(wdata(lane*8+7 downto lane*8))));
							end if;
							if index = 0 then header(lane*8+7 downto lane*8) <= byte(mem(lane)); end if;
						else
							ddr_dout(lane*8+7 downto lane*8) <= byte(mem(index + lane));
						end if;
					end loop;
					if write_access then
						ddr_writes <= ddr_writes + 1;
					else
						ddr_reads <= ddr_reads + 1;
					end if;
					ddr_done <= '1'; pending := false;
				else delay := delay - 1; end if;
			end if;
			if ddr_ena = '1' then
				assert not pending report "overlapping DDR transaction" severity failure;
				assert unsigned(ddr_adr) >= SLOT_BASE and unsigned(ddr_adr) < SLOT_BASE + expected_dwords
					report "DDR transfer outside blob/into adjacent slot" severity failure;
				assert ddr_adr(0) = '0' report "unaligned DDR transfer" severity failure;
				index := (to_integer(unsigned(ddr_adr)) - SLOT_BASE) * 4;
				enables := ddr_be;
				if index /= 0 and ddr_rnw = '0' then
					assert header(63 downto 32) = x"00000000" report "payload write to a valid slot" severity failure;
				end if;
				if index = 0 and ddr_rnw = '0' and ddr_din(63 downto 32) = x"00000000" then
					assert enables = x"F0" report "invalidation did not preserve slot counter" severity failure;
				else
					assert enables = x"FF" report "unexpected DDR byte enables" severity failure;
				end if;
				delay := 1 + ((index / 8) mod 7);
				-- Byte 7 can time out while the following slot word is being prefetched.
				if ddr_rnw = '1' and index = CD_OFFSET + 8 and fault_addr = CD_BASE + 7 then delay := 700; end if;
				write_access := ddr_rnw = '0'; wdata := ddr_din; pending := true;
			end if;
			if check_blob = '1' then
				assert unsigned(header(63 downto 32)) = expected_dwords
					and unsigned(header(31 downto 0)) = expected_header_count
					report "wrong size/count header" severity failure;
				for i in 0 to CD_BYTES - 1 loop
					assert mem(CD_OFFSET + i) = pattern(CD_BASE + i)
						report "saved CD payload mismatch at " & integer'image(i) severity failure;
				end loop;
				if expected_arcade = '1' then
					for i in 0 to AC_BYTES - 1 loop
						assert mem(AC_OFFSET + i) = pattern(i)
							report "saved AC payload mismatch at " & integer'image(i) severity failure;
					end loop;
				end if;
				for i in -8 to -1 loop assert mem(i) = SENTINEL report "preceding slot sentinel" severity failure; end loop;
				for i in expected_dwords * 4 to SLOT_BYTES + 7 loop
					assert mem(i) = SENTINEL report "blob tail/adjacent slot sentinel" severity failure;
				end loop;
				blob_checked <= '1';
			else
				blob_checked <= '0';
			end if;
		end if;
	end process;

	stimulus : process
		constant fault_prefixes : integer_vector := (3, 8, CD_BYTES); -- mid-word, prefetch, final region drain
		variable start_cycle, start_transfer, writes_before, reads_before : natural;
		variable mem_reads_before, mem_writes_before, loads_before : natural;
		variable payload_bytes : natural;
		variable start_time : time;
		variable saved_header : std_logic_vector(63 downto 0);
		procedure tick is begin wait until rising_edge(clk); wait for 1 ns; end procedure;
		procedure start(signal request : out std_logic) is
		begin
			request <= '1'; tick; request <= '0';
			assert busy = '1' and failed = '0' report "new request not accepted" severity failure;
		end procedure;
		procedure timeout_byte(signal request : out std_logic; target : natural; is_load : boolean) is
			constant physical : std_logic_vector(ram_addr'range) := std_logic_vector(to_unsigned(target, ram_addr'length));
			variable resets_before, completions_before : natural;
		begin
			fault_addr <= target;
			completions_before := load_count;
			start(request);
			wait until ram_addr = physical and (ram_rd = '1' or ram_wr = '1') for 500 ms;
			assert ram_addr = physical and
				((is_load and ram_wr = '1' and committed = '1') or (not is_load and ram_rd = '1' and saving = '1'))
				report "did not reach the intended byte of the committed walk" severity failure;
			resets_before := console_resets;
			for i in 1 to 255 loop
				tick;
				assert ready = '0' and walk_error = '0' and (ram_rd = '1' or ram_wr = '1')
					report "unacknowledged byte retired before its deadline" severity failure;
			end loop;
			-- Keep the controller unavailable for longer than another counter wrap.
			for i in 0 to 300 loop
				tick;
				assert busy = '1' and sleeping = '1' and frozen = '1' and active = '1'
					and ram_ready = '0' and ready = '0' and walk_error = '1'
					report "timeout did not quarantine the undrained port" severity failure;
				assert ram_rd = '0' and ram_wr = '0' and kick = '0' and ddr_ena = '0' and loaded = '0'
					report "transfer or load completion after timeout" severity failure;
			end loop;
			release_fault <= true;
			wait until ready = '1' and walk_error = '1' for 1 us;
			assert ready = '1' and walk_error = '1' and active = '1'
				report "late acknowledgement was not reported as a drained error" severity failure;
			for i in 1 to 512 loop
				tick;
				assert (active = '0' or walk_error = '1') and rd = '0' and wr = '0' and ddr_ena = '0' and loaded = '0'
					report "late acknowledgement advanced the failed walk" severity failure;
				if is_load and sleeping = '0' then
					assert console_resets > resets_before report "partial restore woke before console reset" severity failure;
				end if;
				exit when busy = '0';
			end loop;
			assert busy = '0' and failed = '1' and sleeping = '0' and committed = '0'
				report "drained error did not finish as a failure" severity failure;
			tick; tick;
			assert walk_error = '0' and active = '0' and load_count = completions_before
				report "stale error/ownership or false load_done" severity failure;
			if not is_load then
				assert console_resets = resets_before report "failed save reset the console" severity failure;
			end if;
			fault_addr <= -1; release_fault <= false;
		end procedure;
	begin
		assert CD_BYTES mod 8 = 0 and CD_BYTES <= 262144 report "invalid simulation CD window" severity failure;
		assert not G_FAULTS or CD_BYTES >= 16 report "fault suite needs two CD words" severity failure;
		tick; tick; reset <= '0'; tick;
		-- Short first: the entire omitted payload must retain the DDR sentinel.
		-- Only one full 2MiB save/load; timing races piggyback on those walks.
		for mode in 0 to 1 loop
			if mode = 0 then
				expected_arcade <= '0'; expected_dwords <= SHORT_DWORDS;
				payload_bytes := CD_BYTES;
			else
				expected_arcade <= '1'; expected_dwords <= FULL_DWORDS;
				payload_bytes := CD_BYTES + AC_BYTES;
				seed_memory <= '1'; tick; seed_memory <= '0';
			end if;
			expected_header_count <= mode + 2;
			arcade_used <= '1'; tick;
			if mode = 1 then arcade_used <= '0'; tick; end if;
			start_cycle := cycles; start_transfer := transfer_cycles; start_time := now;
			writes_before := ddr_writes; mem_reads_before := mem_reads;
			mem_writes_before := mem_writes;
			report "Arcade memory SAVE mode=" & integer'image(mode) & " (0=short, 1=full)";
			save <= '1'; tick; save <= '0';
			if mode = 0 then
				wait until frozen = '1';
				-- Not at request or paused assertion: sample after paused settle.
				for i in 1 to 50 loop tick; end loop;
				arcade_used <= '0';
			else
				-- An AC access arriving after request but before freeze belongs
				-- to this snapshot, not the next one.
				assert frozen = '0' severity failure;
				arcade_used <= '1';
			end if;
			wait until ddr_writes > writes_before;
			-- The selected format must remain fixed throughout the walk.
			arcade_used <= not expected_arcade;
			wait until busy = '0'; tick;
			report "SAVE cycles=" & integer'image(cycles - start_cycle) &
				" transfercycles=" & integer'image(transfer_cycles - start_transfer) & " elapsed=" & time'image(now - start_time);
			assert failed = '0' and mem_reads - mem_reads_before = payload_bytes
				and mem_writes = mem_writes_before
				report "incomplete/incorrect save: failed=" & std_logic'image(failed) &
					" reads=" & integer'image(mem_reads - mem_reads_before) &
					" expected=" & integer'image(payload_bytes) &
					" writes=" & integer'image(mem_writes - mem_writes_before) severity failure;
			assert ddr_writes - writes_before = expected_dwords / 2 + 1
				report "save word count" severity failure;
			check_blob <= '1'; tick; check_blob <= '0';
			assert blob_checked = '1' severity failure;
			saved_header := header; writes_before := ddr_writes; mem_reads_before := mem_reads;
			-- Rejected saves cannot invalidate either format or touch its RAM.
			abort_save <= '1'; save <= '1'; tick; save <= '0';
			wait until busy = '0'; tick; abort_save <= '0';
			assert failed = '1' and ddr_writes = writes_before and header = saved_header
				and mem_reads = mem_reads_before and mem_writes = mem_writes_before
				report "rejected save changed slot/memory" severity failure;
			if G_FAULTS then
				report "Timeout reading Arcade byte 3 after invalidating a valid short slot";
				expected_arcade <= '1'; expected_dwords <= FULL_DWORDS; arcade_used <= '1';
				timeout_byte(save, 3, false);
				assert header = (x"00000000" & saved_header(31 downto 0))
					and ddr_writes - writes_before = AC_OFFSET / 8
					report "failed save published a header or stored a partial Arcade word" severity failure;
				reads_before := ddr_reads; writes_before := ddr_writes; loads_before := load_count;
				start(load); wait until busy = '0'; tick;
				assert failed = '1' and load_count = loads_before and ddr_reads = reads_before + 1
					and ddr_writes = writes_before and mem_writes = mem_writes_before
					report "load did not reject the invalidated slot" severity failure;

				report "Recover with a short save; exercise three committed-load failure boundaries";
				expected_arcade <= '0'; expected_dwords <= SHORT_DWORDS; arcade_used <= '0';
				expected_header_count <= 3;
				start(save); wait until busy = '0'; tick;
				check_blob <= '1'; tick; check_blob <= '0';
				assert failed = '0' and blob_checked = '1' and ddr_writes - writes_before = SHORT_DWORDS / 2 + 1
					report "save after timeout did not publish the recovered slot" severity failure;
				arcade_used <= '1'; saved_header := header; writes_before := ddr_writes; mem_reads_before := mem_reads;
				overwrite <= '1'; tick; overwrite <= '0';
				for i in fault_prefixes'range loop
					report "Timeout writing CD byte " & integer'image(fault_prefixes(i) - 1);
					timeout_byte(load, CD_BASE + fault_prefixes(i) - 1, true);
					assert header = saved_header and ddr_writes = writes_before and mem_reads = mem_reads_before
						and mem_writes - mem_writes_before = fault_prefixes(i)
						report "late physical write did not drain exactly once or load modified slot" severity failure;
					restored_cd_bytes <= fault_prefixes(i); check_memory <= '1'; tick; check_memory <= '0';
					assert memory_checked = '1' severity failure;
					mem_writes_before := mem_writes;
				end loop;
			end if;
			overwrite <= '1'; tick; overwrite <= '0'; tick;
			start_cycle := cycles; start_transfer := transfer_cycles; start_time := now;
			reads_before := ddr_reads; loads_before := load_count;
			report "Arcade memory LOAD mode=" & integer'image(mode) & " with opposite current flag";
			assert arcade_used /= expected_arcade severity failure;
			load <= '1'; tick; load <= '0';
			wait until busy = '0'; tick; tick;
			report "LOAD cycles=" & integer'image(cycles - start_cycle) &
				" transfercycles=" & integer'image(transfer_cycles - start_transfer) & " elapsed=" & time'image(now - start_time);
			assert load_count = loads_before + 1 and loaded_mode = expected_arcade and failed = '0'
				report "load completion/selected format" severity failure;
			assert mem_writes - mem_writes_before = payload_bytes and mem_reads = mem_reads_before
				report "incomplete/incorrect load" severity failure;
			assert ddr_reads - reads_before = expected_dwords / 2 and ddr_writes = writes_before
				report "load word count/slot changed" severity failure;
			assert last_reads > 0 and last_writes > 0 report "last-count read/write not exercised" severity failure;
			check_memory <= '1'; check_blob <= '1'; tick;
			assert memory_checked = '1' and blob_checked = '1' severity failure;
			check_memory <= '0'; check_blob <= '0'; tick;
			exit when G_FAULTS;
		end loop;
		if G_FAULTS then
			report "TB ARCADE TIMEOUT PASSED: last-count ack, invalid slot, reset before wake, clean save/load after errors";
		else
			report "TB ARCADE MEMORY PASSED: short/full payloads, header-selected loads, late flag, unchanged short AC, boundaries";
		end if;
		std.env.finish;
	end process;
	watchdog : process begin
		wait for 5 sec;
		assert false report "TB ARCADE MEMORY watchdog" severity failure;
	end process;
end architecture;
