-----------------------------------------------------------------------------
-- GHDL testbench for the PCE savestate engine chain:
--   statemanager -> savestates -> (eReg_SavestateV, Save_RAM model, DDR model)
--
-- Verifies:
--   * slot address math (base 0x3800000 DW + slot*0xC0000)
--   * SAVE: internals stream (128 slots from DWord base+2), 11 memory regions
--     in table order with exact byte counts, control word (STATESIZE|count)
--     written last at the slot base
--   * LOAD roundtrip: eReg value restored, memory bytes reproduced exactly,
--     reset_ss pulsed, load_done pulsed
--   * LOAD reject: corrupted STATESIZE header -> no reset_ss, no load_done
--   * Savestates to SDCard off: no header write moves the change counter,
--     even when the DDR grants it late; the slot still loads byte-exact
--
-- Run: sim/run_tb.sh
-----------------------------------------------------------------------------
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;
use work.pBus_savestates.all;
use work.pPCE_savestates.all;

entity tb_savestates is
end entity;

architecture sim of tb_savestates is

	constant CLK_PERIOD : time := 10 ns;

	constant SLOT            : integer := 2;
	constant SLOT_BASE       : integer := 16#3800000# + SLOT * 16#C0000#;
	constant EXP_STATESIZE   : integer := 132402;              -- 0x20532, no Arcade RAM payload
	constant EXP_MEM_BYTES   : integer := 528576;              -- sum of included region sizes
	constant EXP_MEM_WORDS   : integer := EXP_MEM_BYTES / 8;
	constant EXP_SAVE_WRITES : integer := 2 + 128 + EXP_MEM_WORDS;	-- header-invalidate + control + internals + memory

	type sizes_t is array (0 to 11) of integer;
	constant REGION_BYTES : sizes_t := (32768, 65536, 512, 1024, 65536, 512, 32768, 2048, 192, 65536, 262144, 0);

	constant LIVE_VALUE : std_logic_vector(63 downto 0) := x"AA55AA55AA55CAFE";

	signal clk       : std_logic := '0';
	signal reset     : std_logic := '1';
	signal save_btn  : std_logic := '0';
	signal load_btn  : std_logic := '0';
	signal selected_slot : integer := SLOT;
	signal save_abort : std_logic;
	signal inhibit : std_logic := '0';
	signal manager_hold : std_logic := '0';
	signal hold_header : std_logic := '0';
	signal to_sd      : std_logic := '1';	-- OSD "Savestates to SDCard"
	signal late_grant : std_logic := '0';	-- DDR grants header writes late
	signal boundary_open : std_logic := '1';	-- '0' keeps the safe boundary closed
	signal frozen     : std_logic := '0';
	signal forced     : std_logic := '0';
	signal wd         : unsigned(9 downto 0) := (others => '0');
	constant WD_BIT   : integer := 8;	-- 256 clocks; hardware uses bit 27 (~3.1 s)
	signal save_failed : std_logic;
	signal saving_ss   : std_logic;
	signal saw_saving  : std_logic := '0';

	-- statemanager <-> savestates
	signal req_save  : std_logic;
	signal req_load  : std_logic;
	signal req_addr  : integer;
	signal req_slot  : integer range 0 to 3;
	signal ss_busy   : std_logic;

	-- savestates <-> world
	signal reset_ss          : std_logic;
	signal load_done         : std_logic;
	signal sleep_ss          : std_logic;
	signal ss_paused         : std_logic := '0';
	signal BUS_Din           : std_logic_vector(63 downto 0);
	signal BUS_Adr           : std_logic_vector(9 downto 0);
	signal BUS_wren          : std_logic;
	signal BUS_rst           : std_logic;
	signal BUS_Dout          : std_logic_vector(63 downto 0);
	signal Save_RAMAddr      : std_logic_vector(24 downto 0);
	signal Save_RAMRdEn      : std_logic;
	signal Save_RAMWrEn      : std_logic;
	signal Save_RAMWriteData : std_logic_vector(7 downto 0);
	signal Save_RAMReadData  : std_logic_vector(7 downto 0);
	signal Save_RAMType      : unsigned(3 downto 0);
	signal ddr_din           : std_logic_vector(63 downto 0);
	signal ddr_dout          : std_logic_vector(63 downto 0) := (others => '0');
	signal ddr_adr           : std_logic_vector(25 downto 0);
	signal ddr_rnw           : std_logic;
	signal ddr_ena           : std_logic;
	signal ddr_be            : std_logic_vector(7 downto 0);
	signal ddr_done          : std_logic := '0';

	-- eReg under test (slot SSREG_INDEX_TOP)
	signal live_reg : std_logic_vector(63 downto 0) := LIVE_VALUE;
	signal reg_out  : std_logic_vector(63 downto 0);

	-- instrumentation
	signal corrupt_header : std_logic := '0';
	signal main_prefetch : std_logic := '0';
	signal saved_count : unsigned(31 downto 0) := (others => '0');
	signal wr_count       : integer := 0;                        -- DDR words written (save)
	signal rd_count       : integer := 0;                        -- DDR words read (load)
	signal first_wr_adr   : integer := -1;
	signal ctrl_written   : std_logic := '0';
	type cnt_t is array (0 to 15) of integer;
	signal region_wr      : cnt_t := (others => 0);              -- Save_RAM bytes written per type (load)
	signal saw_reset_ss   : std_logic := '0';
	signal saw_load_done  : std_logic := '0';
	signal clr_flags      : std_logic := '0';
	signal data_errors    : integer := 0;

	-- deterministic Save_RAM content: f(type, byte-address)
	function ram_pattern(t : unsigned(3 downto 0); a : std_logic_vector(24 downto 0)) return std_logic_vector is
		variable r : unsigned(7 downto 0);
	begin
		r := unsigned(a(7 downto 0)) xor (t & x"0");
		return std_logic_vector(r);
	end function;

begin

	clk <= not clk after CLK_PERIOD / 2;

	watchdog : process begin
		wait for 2 sec;
		assert false report "savestate engine test timed out" severity failure;
	end process;

	-- Model the top-level pause: freeze at the safe boundary, or force a
	-- freeze when the watchdog expires. Derive abort from the forced freeze
	-- rather than driving it directly, so the test covers the abort wiring.
	freeze_model : process(clk) begin
		if rising_edge(clk) then
			if sleep_ss = '0' then
				frozen  <= '0';
				forced  <= '0';
				wd      <= (others => '0');
			elsif frozen = '0' then
				wd <= wd + 1;
				if boundary_open = '1' or wd(WD_BIT) = '1' then
					frozen <= '1';
					forced <= wd(WD_BIT);
				end if;
			end if;
			ss_paused <= frozen;
		end if;
	end process;

	-- Match pce_top: a forced freeze aborts the save.
	save_abort <= forced;

	SSMAN : entity work.statemanager
	generic map ( Softmap_SaveState_ADDR => 16#3800000#, Softmap_Rewind_ADDR => 16#3800000# )
	port map (
		clk => clk, reset => reset or inhibit,
		rewind_on => '0', rewind_active => '0',
		savestate_number => selected_slot,
		save => save_btn, load => load_btn,
		sleep_rewind => open, vsync => '0',
		request_savestate => req_save,
		request_loadstate => req_load,
		request_address   => req_addr,
		request_slot      => req_slot,
		request_busy      => ss_busy or manager_hold
	);

	DUT : entity work.savestates
	port map (
		clk => clk, reset_in => reset,
		reset_ss => reset_ss, reset_delay => open,
		load_done => load_done,
		increaseSSHeaderCount => to_sd,
		save => req_save, load => req_load,
		savestate_address => req_addr,
		savestate_busy => ss_busy,
		save_abort => save_abort, save_failed => save_failed,
		inhibit => inhibit,
		paused => ss_paused,
		BUS_Din => BUS_Din, BUS_Adr => BUS_Adr, BUS_wren => BUS_wren,
		BUS_rst => BUS_rst, BUS_Dout => BUS_Dout,
		loading_savestate => open, saving_savestate => saving_ss,
		sleep_savestate => sleep_ss,
		Save_RAMAddr => Save_RAMAddr, Save_RAMRdEn => Save_RAMRdEn,
		Save_RAMWrEn => Save_RAMWrEn, Save_RAMWriteData => Save_RAMWriteData,
		Save_RAMReadData => Save_RAMReadData, Save_RAMType => Save_RAMType,
		bus_out_Din => ddr_din, bus_out_Dout => ddr_dout, bus_out_Adr => ddr_adr,
		bus_out_rnw => ddr_rnw, bus_out_ena => ddr_ena, bus_out_be => ddr_be,
		bus_out_done => ddr_done
	);

	EREG : entity work.eReg_SavestateV
	generic map ( Adr => SSREG_INDEX_TOP, def => SSREG_DEFAULT_TOP )
	port map (
		clk => clk,
		BUS_Din => BUS_Din, BUS_Adr => BUS_Adr, BUS_wren => BUS_wren,
		BUS_rst => BUS_rst, BUS_Dout => BUS_Dout,
		Din => live_reg, Dout => reg_out
	);

	-- Save_RAM model: combinational deterministic pattern for reads;
	-- write checker for loads (count per region + data compare).
	Save_RAMReadData <= ram_pattern(Save_RAMType, Save_RAMAddr);

	-- WrEn is held high for several cycles per byte (memory_slow pacing) —
	-- count and check on the rising edge only, like a RAM write would land once.
	process(clk)
		variable wr_prev : std_logic := '0';
	begin
		if rising_edge(clk) then
			if Save_RAMWrEn = '1' and wr_prev = '0' then
				region_wr(to_integer(Save_RAMType)) <= region_wr(to_integer(Save_RAMType)) + 1;
				if Save_RAMWriteData /= ram_pattern(Save_RAMType, Save_RAMAddr) then
					data_errors <= data_errors + 1;
					report "Save_RAM write data mismatch, type=" & integer'image(to_integer(Save_RAMType))
						& " addr=" & integer'image(to_integer(unsigned(Save_RAMAddr))) severity error;
				end if;
			end if;
			wr_prev := Save_RAMWrEn;
		end if;
	end process;

	-- An aborted save must never assert saving_savestate. The abort decision
	-- precedes that signal, so gating abort with it would miss the refusal.
	process(clk) begin
		if rising_edge(clk) then
			if clr_flags = '1' then saw_saving <= '0';
			elsif saving_ss = '1' then saw_saving <= '1'; end if;
		end if;
	end process;

	-- flag monitors (single driver; stim clears via clr_flags)
	process(clk) begin
		if rising_edge(clk) then
			if clr_flags = '1' then
				saw_reset_ss  <= '0';
				saw_load_done <= '0';
			else
				if reset_ss = '1' and reset = '0' then saw_reset_ss <= '1'; end if;
				if load_done = '1' then saw_load_done <= '1'; end if;
			end if;
		end if;
	end process;

	-- DDR3 model: 1-cycle latency, word storage indexed relative to SLOT_BASE.
	-- Like ddram.sv, a write stores the data and byte enables present at its
	-- grant; late_grant delays header-write grants, as DDR contention can.
	ddr_model : process(clk)
		type mem_t is array (0 to 16#C0000# / 2 - 1) of std_logic_vector(63 downto 0);
		variable mem   : mem_t;
		variable idx   : integer;
		variable a     : integer;
		variable pend  : std_logic := '0';
		variable wpend : integer := 0;	-- clocks until a delayed header write is granted

		procedure store(i : integer; adr : integer) is
			variable w : std_logic_vector(63 downto 0) := mem(i);
		begin
			for b in 0 to 7 loop
				if ddr_be(b) = '1' then w(8 * b + 7 downto 8 * b) := ddr_din(8 * b + 7 downto 8 * b); end if;
			end loop;
			wr_count <= wr_count + 1;
			if first_wr_adr = -1 then first_wr_adr <= adr; end if;
			if adr /= SLOT_BASE then
				assert ddr_be = x"FF" report "unexpected byte-enable on save write" severity failure;
			else
				-- Main copies a slot to SD when this counter changes (process_ss).
				assert to_sd = '1' or w(31 downto 0) = mem(i)(31 downto 0)
					report "change counter moved with Savestates to SDCard off" severity failure;
				if w(63 downto 32) = x"00000000" then
					-- header-invalidate at save start: slot must be
					-- rejectable while the body is being written
					assert ctrl_written = '0'
						report "header invalidate AFTER control word" severity failure;
				else
					ctrl_written <= '1';
					saved_count <= unsigned(w(31 downto 0));
					assert to_integer(unsigned(w(63 downto 32))) = EXP_STATESIZE
						report "control word STATESIZE mismatch: got "
							& integer'image(to_integer(unsigned(w(63 downto 32))))
							& " expected " & integer'image(EXP_STATESIZE) severity failure;
				end if;
			end if;
			mem(i) := w;
		end procedure;
	begin
		if rising_edge(clk) then
			ddr_done <= '0';
			-- Reset the control-word ordering check for each attempt.
			if clr_flags = '1' then ctrl_written <= '0'; end if;
			-- Main marks prefetched SD slots with this change-counter sentinel.
			if main_prefetch = '1' then mem(0)(31 downto 0) := x"FFFFFFFF"; end if;
			if ddr_ena = '1' then
				assert wpend = 0 report "DDR request while a write awaits its grant" severity failure;
				a   := to_integer(unsigned(ddr_adr));
				assert a >= SLOT_BASE and a < SLOT_BASE + 16#C0000#
					report "DDR access outside slot window: " & integer'image(a) severity failure;
				assert (a mod 2) = 0 report "DDR address not word-aligned" severity failure;
				idx := (a - SLOT_BASE) / 2;
				if ddr_rnw = '0' then
					if a = SLOT_BASE and late_grant = '1' then
						wpend := 4;
					else
						store(idx, a);
						ddr_done <= '1';
					end if;
				else
					if idx = 0 and corrupt_header = '1' then
						ddr_dout <= std_logic_vector(to_unsigned(EXP_STATESIZE + 2, 32)) & mem(0)(31 downto 0);
					else
						ddr_dout <= mem(idx);
					end if;
					rd_count <= rd_count + 1;
					if a = SLOT_BASE and hold_header = '1' then
						pend := '1';
					else
						ddr_done <= '1';
					end if;
				end if;
			elsif wpend > 0 then
				wpend := wpend - 1;
				if wpend = 0 then
					store(0, SLOT_BASE);
					ddr_done <= '1';
				end if;
			elsif pend = '1' and hold_header = '0' then
				ddr_done <= '1';
				pend := '0';
			end if;
		end if;
	end process;

	stim : process
		variable wr_before : integer := 0;
		variable rd_before : integer := 0;
		variable regions_before : cnt_t;
		procedure pulse(signal button : out std_logic) is begin
			button <= '1';
			wait until rising_edge(clk);
			wait until rising_edge(clk);
			button <= '0';
		end procedure;
		procedure wait_cycles(n : integer) is begin
			for i in 1 to n loop wait until rising_edge(clk); end loop;
		end procedure;
	begin
		wait_cycles(10);
		reset <= '0';
		wait_cycles(10);

		----------------------------------------------------------------
		report "TB: SAVE to slot " & integer'image(SLOT);
		save_btn <= '1'; wait_cycles(3); save_btn <= '0';
		wait until sleep_ss = '1';
		wait until sleep_ss = '0';
		wait_cycles(10);

		-- the save must invalidate the slot header BEFORE writing the body
		assert first_wr_adr = SLOT_BASE
			report "first write at " & integer'image(first_wr_adr)
				& " expected header-invalidate at " & integer'image(SLOT_BASE) severity failure;
		assert wr_count = EXP_SAVE_WRITES
			report "save word count " & integer'image(wr_count)
				& " expected " & integer'image(EXP_SAVE_WRITES) severity failure;
		assert ctrl_written = '1' report "control word never written" severity failure;
		report "TB: SAVE ok (" & integer'image(wr_count) & " words)";

		----------------------------------------------------------------
		-- change the live value: the load must restore the OLD (saved) one
		live_reg <= x"0123456789ABCDEF";
		wait_cycles(2);
		assert reg_out = SSREG_DEFAULT_TOP report "reg_out should still be default before load" severity failure;

		report "TB: LOAD roundtrip";
		load_btn <= '1'; wait_cycles(3); load_btn <= '0';
		wait until sleep_ss = '1';
		wait until sleep_ss = '0';
		wait_cycles(10);

		assert saw_load_done = '1' report "load_done never pulsed" severity failure;
		assert saw_reset_ss = '1' report "reset_ss never pulsed during load" severity failure;
		assert reg_out = LIVE_VALUE
			report "eReg not restored to saved value" severity failure;
		for t in REGION_BYTES'range loop
			assert region_wr(t) = REGION_BYTES(t)
				report "region " & integer'image(t) & " wrote " & integer'image(region_wr(t))
					& " bytes, expected " & integer'image(REGION_BYTES(t)) severity failure;
		end loop;
		assert data_errors = 0 report "memory data mismatches: " & integer'image(data_errors) severity failure;
		report "TB: LOAD ok (regions verified byte-exact)";

		----------------------------------------------------------------
		report "TB: LOAD reject (corrupted header)";
		clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
		corrupt_header <= '1';
		rd_before := rd_count; regions_before := region_wr;
		wait_cycles(2);
		load_btn <= '1'; wait_cycles(3); load_btn <= '0';
		wait until sleep_ss = '1';
		wait until sleep_ss = '0';
		wait_cycles(10);
		assert saw_load_done = '0' report "load_done pulsed on corrupted header" severity failure;
		assert saw_reset_ss = '0' report "reset_ss pulsed on corrupted header" severity failure;
		assert reg_out = LIVE_VALUE report "eReg clobbered by rejected load" severity failure;
		assert save_failed = '1' and rd_count = rd_before + 1 and region_wr = regions_before
			report "malformed load did not report failure without restoring memory" severity failure;
		report "TB: reject ok";

		-- Both queued opcodes must reach the engine, even if the save aborts.
		report "TB: queued save then load with independent slots";
		manager_hold <= '1'; boundary_open <= '0';
		selected_slot <= 0;
		pulse(save_btn); wait_cycles(2);
		selected_slot <= SLOT;
		pulse(load_btn); wait_cycles(2);
		selected_slot <= 3;
		wr_before := wr_count; rd_before := rd_count;
		manager_hold <= '0';
		wait until req_save = '1'; wait for 1 ns;
		assert req_slot = 0 and req_addr = 16#3800000#
			report "queued save lost its request-time slot" severity failure;
		wait until ss_busy = '1';
		wait until ss_busy = '0'; wait for 1 ns;
		assert save_failed = '1' and wr_count = wr_before
			report "queued save did not abort without writing" severity failure;
		wait until req_load = '1'; wait for 1 ns;
		assert req_slot = SLOT and req_addr = SLOT_BASE
			report "queued load lost its independent request-time slot" severity failure;
		wait until ss_busy = '1';
		wait until ss_busy = '0'; wait_cycles(10);
		assert rd_count = rd_before + 1 and save_failed = '1'
			and saw_load_done = '0' and saw_reset_ss = '0' and region_wr = regions_before
			report "queued load was dropped or restored malformed data" severity failure;
		selected_slot <= SLOT; boundary_open <= '1';
		----------------------------------------------------------------
		-- A refused save must preserve the previous slot. Load it afterward
		-- to check the stored state, not just the absence of write requests.
		report "TB: forced SAVE (save_abort) - the slot must stay untouched";
		corrupt_header <= '0';
		clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
		wr_before := wr_count;
		boundary_open <= '0';	-- Keep the boundary closed until the watchdog expires.
		save_btn <= '1'; wait_cycles(3); save_btn <= '0';
		wait until sleep_ss = '1';
		wait until sleep_ss = '0';
		wait_cycles(10);
		assert wr_count = wr_before
			report "a forced save wrote " & integer'image(wr_count - wr_before)
				& " words: it had to exit without touching the slot" severity failure;
		assert saw_saving = '0'
			report "saving_savestate rose during an aborted attempt: the"
				& " engine passed SAVE_WAITSETTLE, so the early exit"
				& " did not work" severity failure;
		assert save_failed = '1'
			report "save_failed low after a forced attempt: the UI"
				& " would announce a save that does not exist" severity failure;
		boundary_open <= '1';

		live_reg <= x"DEADDEADDEADDEAD";
		wait_cycles(2);
		load_btn <= '1'; wait_cycles(3); load_btn <= '0';
		wait until sleep_ss = '1';
		wait until sleep_ss = '0';
		wait_cycles(10);
		assert saw_load_done = '1'
			report "the previous save no longer loads: the forced"
				& " attempt destroyed it" severity failure;
		assert reg_out = LIVE_VALUE
			report "slot survived but its content is wrong" severity failure;
		assert save_failed = '0'
			report "save_failed still high after a successful load: it would"
				& " silence the restore notice" severity failure;
		report "TB: forced save ok (slot intact, save_failed correct)";

		----------------------------------------------------------------
		-- An inhibit clears both queued operations; neither can replay later.
		report "TB: MB128 queued save/load cancellation";
		wr_before := wr_count; rd_before := rd_count;
		manager_hold <= '1';
		pulse(save_btn); pulse(load_btn);
		inhibit <= '1'; wait_cycles(4);
		inhibit <= '0'; manager_hold <= '0'; wait_cycles(20);
		assert ss_busy = '0' and wr_count = wr_before and rd_count = rd_before
			report "cancelled queued request replayed after inhibit cleared" severity failure;

		-- Cancel while the game still runs, and while the settle drain runs.
		for phase in 0 to 1 loop
			for operation in 0 to 1 loop
				clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
				if phase = 0 then boundary_open <= '0'; else boundary_open <= '1'; end if;
				wr_before := wr_count; rd_before := rd_count;
				if operation = 0 then pulse(save_btn); else pulse(load_btn); end if;
				wait until sleep_ss = '1';
				wait_cycles(20);
				inhibit <= '1'; wait_cycles(6);
				assert ss_busy = '0' and sleep_ss = '0' and save_failed = '1'
					report "pending operation was not promptly refused" severity failure;
				assert wr_count = wr_before and rd_count = rd_before
					and saw_reset_ss = '0' and saw_load_done = '0' and reg_out = LIVE_VALUE
					report "pending refusal modified slot or restored machine state" severity failure;
				inhibit <= '0'; boundary_open <= '1'; wait_cycles(10);
			end loop;
		end loop;

		-- Last save cancellation point: accepted freeze, but no header write.
		report "TB: MB128 inhibit before header invalidation";
		wr_before := wr_count;
		pulse(save_btn);
		wait until saving_ss = '1';
		inhibit <= '1'; wait_cycles(6);
		assert ss_busy = '0' and saving_ss = '0' and save_failed = '1'
			and wr_count = wr_before report "late refusal destroyed the saved slot" severity failure;
		inhibit <= '0'; wait_cycles(10);

		-- A header response may arrive after the inhibit; no eReg may be loaded.
		report "TB: MB128 inhibit with header read outstanding";
		clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
		hold_header <= '1'; rd_before := rd_count;
		pulse(load_btn);
		wait until rd_count > rd_before;
		inhibit <= '1'; wait_cycles(4);
		hold_header <= '0'; wait_cycles(10);
		assert ss_busy = '0' and save_failed = '1' and reg_out = LIVE_VALUE
			and saw_reset_ss = '0' and saw_load_done = '0'
			report "inhibited header response began a restore" severity failure;
		inhibit <= '0'; wait_cycles(10);

		-- Load the original source after all rejected operations.
		pulse(load_btn);
		wait until sleep_ss = '1'; wait until sleep_ss = '0'; wait_cycles(10);
		assert reg_out = LIVE_VALUE and saw_load_done = '1' and data_errors = 0
			report "refusals damaged the original snapshot" severity failure;

		-- Once a transfer is committed, finish it rather than leave a torn slot
		-- or a partly restored machine. The inhibit blocks following requests.
		report "TB: MB128 mode change during committed save/load";
		live_reg <= x"C0DEC0DEC0DEC0DE";
		clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
		wr_before := wr_count;
		pulse(save_btn);
		wait until wr_count > wr_before;
		inhibit <= '1';
		wait until sleep_ss = '0'; wait_cycles(10);
		assert ctrl_written = '1' and save_failed = '0'
			report "inhibit tore a committed save" severity failure;
		inhibit <= '0'; live_reg <= (others => '0'); wait_cycles(10);
		clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
		rd_before := rd_count;
		pulse(load_btn);
		wait until rd_count >= rd_before + 2;
		inhibit <= '1';
		wait until sleep_ss = '0'; wait_cycles(10);
		assert saw_load_done = '1' and reg_out = x"C0DEC0DEC0DEC0DE" and data_errors = 0
			report "inhibit interrupted a committed restore" severity failure;
		report "TB: MB128 policy PASS (queue, boundary, settle, header, committed transfer)";

		-- Loading a prefetched SD slot imports FFFFFFFF. The next complete
		-- save wraps to zero; size, not the change counter, validates a load.
		inhibit <= '0';
		main_prefetch <= '1'; wait_cycles(2); main_prefetch <= '0';
		clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
		pulse(load_btn);
		wait until sleep_ss = '1'; wait until sleep_ss = '0'; wait_cycles(10);
		assert saw_load_done = '1' and reg_out = x"C0DEC0DEC0DEC0DE"
			report "Main-prefetched slot failed to restore" severity failure;

		for expected_count in 0 to 1 loop
			live_reg <= x"123456789ABCDEF0";
			clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
			wr_before := wr_count;
			pulse(save_btn);
			wait until sleep_ss = '1'; wait until sleep_ss = '0'; wait_cycles(10);
			assert saved_count = to_unsigned(expected_count, 32)
				report "unexpected counter after Main-prefetched load" severity failure;
			assert ctrl_written = '1' and save_failed = '0'
				and wr_count - wr_before = EXP_SAVE_WRITES
				report "counter-wrap save did not complete the payload" severity failure;
			live_reg <= (others => '0');
			clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
			rd_before := rd_count;
			pulse(load_btn);
			wait until sleep_ss = '1'; wait until sleep_ss = '0'; wait_cycles(10);
			assert saw_load_done = '1' and saw_reset_ss = '1'
				and reg_out = x"123456789ABCDEF0"
				and rd_count - rd_before = EXP_SAVE_WRITES - 1 and data_errors = 0
				report "counter-wrap slot failed byte-exact restore" severity failure;
			report "TB: Main-prefetched count=" & integer'image(expected_count)
				& " save and byte-exact load PASS";
		end loop;

		-- Savestates to SDCard off: Main must never see the change counter
		-- move (the model checks every header write, granted late here), and
		-- the slot must still load.
		report "TB: Savestates to SDCard off";
		to_sd <= '0'; late_grant <= '1';
		live_reg <= x"5D0FF5D0FF5D0FF0";
		clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
		wr_before := wr_count;
		pulse(save_btn);
		wait until sleep_ss = '1'; wait until sleep_ss = '0'; wait_cycles(10);
		assert ctrl_written = '1' and save_failed = '0'
			and wr_count - wr_before = EXP_SAVE_WRITES
			report "SD-off save did not complete the payload" severity failure;
		live_reg <= (others => '0');
		clr_flags <= '1'; wait_cycles(2); clr_flags <= '0';
		pulse(load_btn);
		wait until sleep_ss = '1'; wait until sleep_ss = '0'; wait_cycles(10);
		assert saw_load_done = '1' and saw_reset_ss = '1'
			and reg_out = x"5D0FF5D0FF5D0FF0" and data_errors = 0
			report "SD-off slot failed byte-exact restore" severity failure;
		to_sd <= '1'; late_grant <= '0';
		report "TB: Savestates to SDCard off PASS (counter unchanged, slot restorable)";

		report "TB PASSED" severity note;
		std.env.finish;
	end process;

end architecture;
