-----------------------------------------------------------------------------
-- tb_savestates.vhd — GHDL testbench for the PCE savestate engine chain:
--   statemanager -> savestates -> (eReg_SavestateV, Save_RAM model, DDR model)
--
-- Verifies (see docs/SAVESTATE_IMPLEMENTATION_PLAN.md §4.2):
--   * slot address math (base 0x3800000 DW + slot*0xC0000)
--   * SAVE: internals stream (64 slots from DWord base+2), 9 memory regions
--     in table order with exact byte counts, control word (STATESIZE|count)
--     written last at the slot base
--   * LOAD roundtrip: eReg value restored, memory bytes reproduced exactly,
--     reset_ss pulsed, load_done pulsed
--   * LOAD reject: corrupted STATESIZE header -> no reset_ss, no load_done
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
	constant EXP_STATESIZE   : integer := 132402;               -- must match savestates.vhd
	constant EXP_MEM_BYTES   : integer := 528576;               -- sum of region sizes
	constant EXP_MEM_WORDS   : integer := EXP_MEM_BYTES / 8;    -- 33304
	constant EXP_SAVE_WRITES : integer := 2 + 128 + EXP_MEM_WORDS;	-- header-invalidate + control + internals + memory

	type sizes_t is array (0 to 10) of integer;
	constant REGION_BYTES : sizes_t := (32768, 65536, 512, 1024, 65536, 512, 32768, 2048, 192, 65536, 262144);

	constant LIVE_VALUE : std_logic_vector(63 downto 0) := x"AA55AA55AA55CAFE";

	signal clk       : std_logic := '0';
	signal reset     : std_logic := '1';
	signal save_btn  : std_logic := '0';
	signal load_btn  : std_logic := '0';

	-- statemanager <-> savestates
	signal req_save  : std_logic;
	signal req_load  : std_logic;
	signal req_addr  : integer;
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

	-- P0-style pause acknowledge (1 cycle after sleep)
	process(clk) begin
		if rising_edge(clk) then
			ss_paused <= sleep_ss;
		end if;
	end process;

	SSMAN : entity work.statemanager
	generic map ( Softmap_SaveState_ADDR => 16#3800000#, Softmap_Rewind_ADDR => 16#3800000# )
	port map (
		clk => clk, reset => reset,
		rewind_on => '0', rewind_active => '0',
		savestate_number => SLOT,
		save => save_btn, load => load_btn,
		sleep_rewind => open, vsync => '0',
		request_savestate => req_save,
		request_loadstate => req_load,
		request_address   => req_addr,
		request_busy      => ss_busy
	);

	DUT : entity work.savestates
	port map (
		clk => clk, reset_in => reset,
		reset_ss => reset_ss, reset_delay => open,
		load_done => load_done,
		increaseSSHeaderCount => '1',
		save => req_save, load => req_load,
		savestate_address => req_addr,
		savestate_busy => ss_busy,
		paused => ss_paused,
		BUS_Din => BUS_Din, BUS_Adr => BUS_Adr, BUS_wren => BUS_wren,
		BUS_rst => BUS_rst, BUS_Dout => BUS_Dout,
		loading_savestate => open, saving_savestate => open,
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
	ddr_model : process(clk)
		type mem_t is array (0 to 131071) of std_logic_vector(63 downto 0);
		variable mem  : mem_t;
		variable idx  : integer;
		variable a    : integer;
		variable pend : std_logic := '0';
	begin
		if rising_edge(clk) then
			ddr_done <= '0';
			if ddr_ena = '1' then
				a   := to_integer(unsigned(ddr_adr));
				assert a >= SLOT_BASE and a < SLOT_BASE + 16#C0000#
					report "DDR access outside slot window: " & integer'image(a) severity failure;
				assert (a mod 2) = 0 report "DDR address not word-aligned" severity failure;
				idx := (a - SLOT_BASE) / 2;
				if ddr_rnw = '0' then
					assert ddr_be = x"FF" report "unexpected byte-enable on save write" severity failure;
					mem(idx) := ddr_din;
					wr_count <= wr_count + 1;
					if first_wr_adr = -1 then first_wr_adr <= a; end if;
					if a = SLOT_BASE then
						if ddr_din = x"0000000000000000" then
							-- header-invalidate at save start: slot must be
							-- rejectable while the body is being written
							assert ctrl_written = '0'
								report "header invalidate AFTER control word" severity failure;
						else
							ctrl_written <= '1';
							assert to_integer(unsigned(ddr_din(63 downto 32))) = EXP_STATESIZE
								report "control word STATESIZE mismatch: got "
									& integer'image(to_integer(unsigned(ddr_din(63 downto 32))))
									& " expected " & integer'image(EXP_STATESIZE) severity failure;
						end if;
					end if;
				else
					if idx = 0 and corrupt_header = '1' then
						ddr_dout <= x"DEADBEEF" & mem(0)(31 downto 0);
					else
						ddr_dout <= mem(idx);
					end if;
					rd_count <= rd_count + 1;
				end if;
				ddr_done <= '1';
			end if;
		end if;
	end process;

	stim : process
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
		for t in 0 to 9 loop
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
		wait_cycles(2);
		load_btn <= '1'; wait_cycles(3); load_btn <= '0';
		wait until sleep_ss = '1';
		wait until sleep_ss = '0';
		wait_cycles(10);
		assert saw_load_done = '0' report "load_done pulsed on corrupted header" severity failure;
		assert saw_reset_ss = '0' report "reset_ss pulsed on corrupted header" severity failure;
		assert reg_out = LIVE_VALUE report "eReg clobbered by rejected load" severity failure;
		report "TB: reject ok";

		report "TB PASSED" severity note;
		std.env.finish;
	end process;

end architecture;
