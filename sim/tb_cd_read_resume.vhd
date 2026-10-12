-- Focused READ6 replay regression; public CD ports and savestate bus only.
-- A fixed-length $1808 copy uses WAIT, not per-byte readiness polling. Delivery
-- uses CD_DATA_END, never READ_CONSUMED or other private consumption counters.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
use work.pPCE_savestates.all;

entity tb_cd_read_resume is
	generic (G_NEXT_DELAY_US : natural := 100; G_TAIL : boolean := false;
	         G_ERROR : boolean := false);
end entity;

architecture sim of tb_cd_read_resume is
	constant CLK_PERIOD : time := 24 ns;
	constant SOURCE_LBA : natural := 3872;
	constant SOURCE_COUNT : natural := 169;
	constant SAVED_CONSUMED : natural := 342037; -- 167 sectors + offset 21
	constant RESUME_LBA : natural := 4039;
	constant FRESH_LBA : natural := 5003;
	constant REMAINING : natural := SOURCE_COUNT * 2048 - SAVED_CONSUMED;
	subtype byte_t is std_logic_vector(7 downto 0);
	type regs_t is array (53 to 64) of std_logic_vector(63 downto 0);
	signal clk : std_logic := '0';
	signal rst_n : std_logic := '0';
	signal ext_a : std_logic_vector(20 downto 0) := (others => '0');
	signal ext_di, ext_do : byte_t := (others => '0');
	signal wr_n, rd_n : std_logic := '1';
	signal cpu_ce : std_logic := '0';
	signal cpu_wait_n : std_logic;
	signal cd_comm : std_logic_vector(95 downto 0);
	signal cd_comm_send, cd_reset, cd_reset_done : std_logic := '0';
	signal cd_data : byte_t := (others => '0');
	signal cd_data_wr, cd_stat_get : std_logic := '0';
	signal cd_status : byte_t := x"00";
	signal cd_data_end : std_logic;
	signal sleep, ss_load, ss_wren, ss_rst : std_logic := '0';
	signal walk_sleep, replay_hold : std_logic := '0';
	signal capture_veto : std_logic;
	signal ss_adr : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(55, 10));
	signal ss_din, ss_dout : std_logic_vector(63 downto 0) := (others => '0');
	signal mailbox : std_logic_vector(95 downto 0) := (others => '0');
	signal command_count, end_count, reset_count : natural := 0;
	signal host_done : natural := 0;
	signal copy_active, tail_active : boolean := false;
	signal copy_expected, copied, stalled : natural := 0;

	-- Includes both LBA and non-byte-periodic offset terms: unlike i mod 256,
	-- repeating/skipping a sector or a 256-byte block changes the oracle.
	function payload(lba, offset : natural) return byte_t is
	begin
		return std_logic_vector(to_unsigned((offset * 73 + (offset / 7) * 19
			+ (offset / 251) * 101 + lba * 29
			+ (lba mod 17) * (offset mod 31)) mod 256, 8));
	end function;
begin
	clk <= not clk after CLK_PERIOD / 2;
	sleep <= walk_sleep or replay_hold;
	DUT : entity work.cd
	port map (
		RST_N => rst_n, CLK => clk, EN => '1', EXT_A => ext_a,
		EXT_DI => ext_di, EXT_DO => ext_do, EXT_WR_N => wr_n,
		EXT_RD_N => rd_n, CPU_CE => cpu_ce, CPU_WAIT_N => cpu_wait_n,
		SEL_N => open, IRQ_N => open, RAM_CS_N => open, BRAM_EN => open,
		CD_STAT => cd_status, CD_MSG => x"00", CD_STAT_GET => cd_stat_get,
		CD_COMM => cd_comm, CD_COMM_SEND => cd_comm_send,
		CD_DOUT_REQ => '0', CD_DOUT => open, CD_DOUT_SEND => open,
		CD_REGION => '0', CD_RESET => cd_reset, CD_RESET_DONE => cd_reset_done,
		CD_FAST_SEEK => open, CD_DATA => cd_data, CD_DATA_WR => cd_data_wr,
		CD_AUDIO_WR => '0', CD_SUBCD_WR => '0', CD_DATA_END => cd_data_end,
		DM => '0', CD_SL => open, CD_SR => open, AD_S => open,
		SLEEP => sleep, SaveStateBus_Din => ss_din, SaveStateBus_Adr => ss_adr,
		WALK_SLEEP => walk_sleep,
		SaveStateBus_wren => ss_wren, SaveStateBus_rst => ss_rst,
		SaveStateBus_load => ss_load, SaveStateBus_Dout => ss_dout,
		SS_AD_RdData => open, SS_CD_HOLD => capture_veto, SS_REPLAY_HOLD => replay_hold
	);

	-- Mailbox and one-cycle request pulses survive a blocking sector burst.
	monitor : process(clk)
		variable old_send, old_reset : std_logic := '0';
		variable phase : natural;
	begin
		if rising_edge(clk) then
			if cd_comm_send = '1' and old_send = '0' then
				mailbox <= cd_comm;
				command_count <= command_count + 1;
			end if;
			if cd_data_end = '1' then end_count <= end_count + 1; end if;
			if cd_reset = '1' and old_reset = '0' then
				reset_count <= reset_count + 1;
			end if;
			old_send := cd_comm_send;
			old_reset := cd_reset;
			if ss_adr = std_logic_vector(to_unsigned(55, 10)) and ss_load = '0'
				and sleep = '0' and not is_x(ss_dout(3 downto 0)) then
				phase := to_integer(unsigned(ss_dout(3 downto 0)));
				if copy_active and phase >= 4 and phase <= 9 then
					assert copied = copy_expected
						report "READ_RESUME premature STATUS: copied=" & natural'image(copied)
						& " expected=" & natural'image(copy_expected) severity failure;
				end if;
				if tail_active then
					assert phase /= 10 and phase /= 11
						report "READ_RESUME tail exposed discarded DATAIN" severity failure;
				end if;
			end if;
		end if;
	end process;

	host : process
		variable seen_command, seen_reset, seen_end : natural := 0;
		variable lba, sectors : natural;
		variable interrupted : boolean;
	begin
		loop
			if command_count = seen_command and reset_count = seen_reset then
				wait until command_count /= seen_command or reset_count /= seen_reset;
			end if;
			if reset_count /= seen_reset then
				seen_reset := reset_count;
				seen_command := command_count; -- discard the old mailbox timeline
				cd_data_wr <= '0';
				cd_stat_get <= '0';
				-- Acknowledge reset only after cancelling the old burst/status.
				wait until falling_edge(clk);
				cd_reset_done <= '1';
				wait until falling_edge(clk);
				wait until falling_edge(clk);
				cd_reset_done <= '0';
			end if;
			if command_count /= seen_command then
				seen_command := command_count;
				assert mailbox(7 downto 0) = x"08"
					report "READ_RESUME unexpected host command" severity failure;
				lba := to_integer(unsigned(mailbox(12 downto 8))) * 65536
					+ to_integer(unsigned(mailbox(23 downto 16))) * 256
					+ to_integer(unsigned(mailbox(31 downto 24)));
				sectors := to_integer(unsigned(mailbox(39 downto 32)));
				assert (seen_command = 1 and ((not G_TAIL and lba = RESUME_LBA and sectors = 2)
					or (G_TAIL and lba = RESUME_LBA + 1 and sectors = 1)))
					or (seen_command = 2 and lba = FRESH_LBA and sectors = 1)
					report "READ_RESUME wrong reissued/fresh READ6 LBA or count" severity failure;
				seen_end := end_count;
				interrupted := false;
				cd_status <= x"00";
				if G_ERROR and seen_command = 1 then
					cd_status <= x"02"; -- failed tail reread, no payload
				else
				for sector in 0 to sectors - 1 loop
					if sector /= 0 then
						if end_count = seen_end then
							wait until end_count /= seen_end or reset_count /= seen_reset for 5 ms;
						end if;
						assert end_count /= seen_end or reset_count /= seen_reset
							report "READ_RESUME host never received CD_DATA_END" severity failure;
						seen_end := end_count;
						if G_NEXT_DELAY_US /= 0 then
							wait on reset_count for G_NEXT_DELAY_US * 1 us;
						end if;
					end if;
					if reset_count /= seen_reset then interrupted := true; exit; end if;
					for offset in 0 to 2047 loop
						if seen_command = 2 and offset = 2 and G_NEXT_DELAY_US /= 0 then
							-- A host pause inside a sector must not become a missing byte.
							wait on reset_count for G_NEXT_DELAY_US * 1 us;
						end if;
						wait until falling_edge(clk);
						cd_data <= payload(lba + sector, offset);
						cd_data_wr <= '1';
						wait until falling_edge(clk);
						wait until falling_edge(clk);
						cd_data_wr <= '0';
						wait until falling_edge(clk);
						if reset_count /= seen_reset then interrupted := true; exit; end if;
					end loop;
					exit when interrupted;
				end loop;
				end if;
				if not interrupted then
					-- Main can deliver GOOD with almost a full sector still queued.
					-- Do not wait for consumer progress before delivering completion.
					wait until falling_edge(clk);
					cd_stat_get <= '1';
					wait until falling_edge(clk);
					cd_stat_get <= '0';
					host_done <= host_done + 1;
				end if;
			end if;
		end loop;
	end process;

	stimulus : process
		procedure clocks(n : natural) is
		begin
			for i in 1 to n loop wait until falling_edge(clk); end loop;
		end procedure;
		procedure bus_access(address : natural; writing : boolean;
			value : inout byte_t) is
			variable deadline : time;
		begin
			ext_a <= std_logic_vector(to_unsigned(16#1FF800# + address, 21));
			ext_di <= value;
			if writing then wr_n <= '0'; else rd_n <= '0'; end if;
			clocks(1); -- settle address-dependent WAIT before the CPU edge
			deadline := now + 50 ms;
			loop
				if cpu_wait_n = '1' then
					cpu_ce <= '1';
					wait until rising_edge(clk);
					if not writing then value := ext_do; end if;
					wait until falling_edge(clk);
					cpu_ce <= '0';
					exit;
				end if;
				stalled <= stalled + 1;
				assert now < deadline report "READ_RESUME CPU_WAIT_N never released" severity failure;
				clocks(1);
			end loop;
			wr_n <= '1'; rd_n <= '1';
			clocks(30); -- fixed CPU instruction/copy cadence, not REQ polling
		end procedure;
		procedure put(address : natural; value : byte_t) is
			variable b : byte_t := value;
		begin bus_access(address, true, b); end procedure;
		procedure get(address : natural; variable value : out byte_t) is
			variable b : byte_t := x"00";
		begin bus_access(address, false, b); value := b; end procedure;
		procedure wait_phase(expected : byte_t) is
			variable b : byte_t;
			variable deadline : time := now + 50 ms;
		begin
			loop
				get(0, b);
				exit when b = expected;
				assert now < deadline report "READ_RESUME phase timeout, got "
					& to_hstring(b) & " expected " & to_hstring(expected) severity failure;
			end loop;
		end procedure;
		procedure ack is
		begin put(2, x"80"); put(2, x"00"); end procedure;
		procedure finish_read(expected_status : byte_t := x"00") is
			variable b : byte_t;
		begin
			wait_phase(x"D8"); -- BSY REQ C/D I/O, STATUS
			get(1, b);
			assert b = expected_status report "READ_RESUME wrong STATUS" severity failure;
			ack;
			wait_phase(x"F8"); -- MSGIN
			get(1, b);
			assert b = x"00" report "READ_RESUME nonzero MESSAGE" severity failure;
			ack;
			wait_phase(x"00");
		end procedure;
		procedure copy_bytes(lba, offset, length : natural) is
			variable b, expected : byte_t;
		begin
			copy_expected <= length; copied <= 0; copy_active <= true;
			for i in 0 to length - 1 loop
				if lba = FRESH_LBA and i = 2 and G_NEXT_DELAY_US >= 100 then
					get(0, b); -- status must remain readable during the forced gap
					assert b(7) = '1' and b(6) = '0'
						report "READ_RESUME status poll waited for DATAIN" severity failure;
				end if;
				get(8, b); -- fixed-length copy: no per-byte readiness polling
				expected := payload(lba + (offset + i) / 2048, (offset + i) mod 2048);
				assert b = expected
					report "READ_RESUME byte mismatch index=" & natural'image(i)
					& " got=" & to_hstring(b) & " expected=" & to_hstring(expected)
					severity failure;
				copied <= i + 1;
			end loop;
		end procedure;
		variable regs : regs_t := (others => (others => '0'));
		variable b : byte_t;
		variable consumed : natural;
	begin
		clocks(10); rst_n <= '1'; clocks(10);
		walk_sleep <= '1'; ss_rst <= '1'; clocks(2); ss_rst <= '0';
		regs(53)(10 downto 8) := "111"; -- SEL/RST/ACK idle
		regs(53)(46) := '1'; -- ACK_N_OLD
		regs(55)(23) := '1'; -- READ_ACTIVE
		regs(57)(7 downto 0) := x"08";
		regs(57)(15 downto 8) := std_logic_vector(to_unsigned(SOURCE_LBA / 65536, 8));
		regs(57)(23 downto 16) := std_logic_vector(to_unsigned((SOURCE_LBA / 256) mod 256, 8));
		regs(57)(31 downto 24) := std_logic_vector(to_unsigned(SOURCE_LBA mod 256, 8));
		regs(57)(39 downto 32) := std_logic_vector(to_unsigned(SOURCE_COUNT, 8));
		if G_TAIL then
			consumed := SOURCE_COUNT * 2048;
			regs(55)(8 downto 4) := "11111"; -- SP_FREE, bus idle; final status owed
			tail_active <= true;
		else
			consumed := SAVED_CONSUMED;
			regs(53)(12) := '1'; -- CD_DTR preserves the partial sector counter
			regs(53)(43 downto 33) := std_logic_vector(to_unsigned(21, 11));
			regs(55)(3 downto 0) := x"A"; -- SP_DATAIN_START
			regs(55)(6 downto 5) := "11"; -- C/D_N, MSG_N
			regs(55)(31 downto 24) := payload(RESUME_LBA, 21);
			copy_expected <= REMAINING; copy_active <= true;
		end if;
		regs(55)(47 downto 32) := std_logic_vector(to_unsigned(consumed mod 65536, 16));
		regs(64)(63 downto 43) := std_logic_vector(to_unsigned(consumed, 21));
		for slot in regs'range loop
			ss_adr <= std_logic_vector(to_unsigned(slot, 10));
			ss_din <= regs(slot); ss_wren <= '1'; clocks(1);
		end loop;
		ss_wren <= '0'; ss_adr <= std_logic_vector(to_unsigned(55, 10));
		ss_load <= '1'; clocks(1); ss_load <= '0'; clocks(4); walk_sleep <= '0';
		if not G_TAIL then
			-- Begin at the restored CPU's next data read, including preserved DBO.
			copy_bytes(RESUME_LBA, 21, REMAINING);
		end if;
		if G_TAIL and not G_ERROR then
			wait until host_done = 1 for 50 ms;
			if capture_veto /= '0' then wait until capture_veto = '0' for 5 ms; end if;
			assert capture_veto = '0' report "READ_RESUME tail cannot be saved" severity failure;
			-- A save walk must freeze the READ's pending STATUS.
			walk_sleep <= '1'; clocks(3);
			assert ss_dout(23) = '1' and ss_dout(3 downto 0) = x"0"
				report "READ_RESUME tail capture missed pending completion" severity failure;
			wait for 2 ms; clocks(1);
			assert ss_dout(23) = '1' and ss_dout(3 downto 0) = x"0"
				report "READ_RESUME STATUS advanced during save walk" severity failure;
			walk_sleep <= '0';
		end if;
		if G_ERROR then finish_read(x"02"); else finish_read; end if;
		copy_active <= false; tail_active <= false;
		assert host_done = 1 and command_count = 1
			report "READ_RESUME replay failed to close exactly once" severity failure;
		if not G_TAIL and G_NEXT_DELAY_US /= 0 then
			assert stalled > 0 report "READ_RESUME delayed gap did not exercise CPU_WAIT_N" severity failure;
		end if;
		-- A cleanly closed replay must not prepend stale data or lose fresh GOOD.
		put(0, x"00");
		for i in 0 to 5 loop
			wait_phase(x"D0"); -- command REQ, C/D, not I/O
			case i is
				when 0 => b := x"08";
				when 1 => b := x"00";
				when 2 => b := std_logic_vector(to_unsigned(FRESH_LBA / 256, 8));
				when 3 => b := std_logic_vector(to_unsigned(FRESH_LBA mod 256, 8));
				when 4 => b := x"01";
				when others => b := x"00";
			end case;
			put(1, b); ack;
		end loop;
		wait_phase(x"C8"); -- only the initial fresh DATAIN byte is polled
		copy_bytes(FRESH_LBA, 0, 2048);
		finish_read; copy_active <= false;
		assert command_count = 2 and host_done = 2
			report "READ_RESUME fresh READ6 retried or never completed" severity failure;
		report "READ_RESUME PASS delay_us=" & natural'image(G_NEXT_DELAY_US)
			& " tail=" & boolean'image(G_TAIL) & " error=" & boolean'image(G_ERROR);
		stop;
		wait;
	end process;

	watchdog : process
	begin
		wait for 100 ms;
		assert false report "READ_RESUME overall completion timeout" severity failure;
	end process;
end architecture;
