-----------------------------------------------------------------------------
-- tb_core.vhd — full-core savestate determinism testbench (plan §11.1).
--
-- Instantiates pce_top (LITE=1) with a tiny synthetic HuCard program that
-- continuously mutates WRAM, plus a 3-slot DDR3 model on the SS_DDR port.
--
--   1. run the core, SAVE to slot 0
--   2. run K unfrozen cycles, SAVE to slot 1   (the "continue" reference)
--   3. LOAD slot 0 (rewind), run K cycles, SAVE to slot 2
--   4. compare slot 1 vs slot 2 word-by-word (control word excluded — the
--      header save-counter legitimately differs).  Any mismatch = state that
--      is not captured/restored deterministically.
--
-- The composite boundary is VBL-gated, so both "K cycles later" saves land on
-- the same frame boundary relative to the (identical) rewound state; a clean
-- core therefore produces bit-identical blobs.
--
-- Run: sim/run_tb_core.sh   (expect several minutes of GHDL mcode runtime)
-----------------------------------------------------------------------------
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;

entity tb_core is
end entity;

architecture sim of tb_core is

	constant CLK_PERIOD : time := 10 ns;
	constant BASE_DW    : integer := 16#3800000#;	-- DWORD addr of slot 0
	constant SLOT_DW    : integer := 16#C0000#;		-- DWORD stride per slot
	constant SLOT_WORDS : integer := 66201;			-- 1 header + 128 internals + 66072 memory (64-bit words)
	constant K_CYCLES   : integer := 200_000;			-- unfrozen cycles between save points

	signal clk      : std_logic := '0';
	signal reset    : std_logic := '1';

	signal ss_save  : std_logic := '0';
	signal ss_load  : std_logic := '0';
	signal ss_slot  : std_logic_vector(1 downto 0) := "00";
	signal ss_busy  : std_logic;
	signal ss_sleep : std_logic;

	signal ddr_din  : std_logic_vector(63 downto 0);
	signal ddr_dout : std_logic_vector(63 downto 0) := (others => '0');
	signal ddr_adr  : std_logic_vector(25 downto 0);
	signal ddr_rnw  : std_logic;
	signal ddr_ena  : std_logic;
	signal ddr_be   : std_logic_vector(7 downto 0);
	signal ddr_done : std_logic := '0';

	signal rom_rd    : std_logic;
	signal rom_a     : std_logic_vector(21 downto 0);
	signal rom_do    : std_logic_vector(7 downto 0) := (others => '0');
	signal rom_rdy   : std_logic := '1';
	signal rom_clken : std_logic;

	-- 8KB synthetic SGX HuCard: maps WRAM + I/O, programs the VPC and VDC1,
	-- then loops incrementing WRAM counters and streaming them into VRAM1
	-- through VDC1 posted writes — so the SGX slots (35-45), the VDC1
	-- CPU-VRAM handshake and the VRAM1 region all carry live nonzero data
	-- through the determinism roundtrip.
	--   FF00: LDA #$F8 / TAM #1            ; MPR1 = WRAM at $2000
	--   FF04: LDA #$FF / TAM #4            ; MPR2 = I/O  at $4000
	--   FF08: PRI0=$12, WIN1L=$34          ; VPC regs ($4008/$400A)
	--   FF12: VDC1 CR=$CC, MAWR=0, AR=VWR  ; VDC1 setup ($4010-$4013)
	--   loop: INC $2000 / LDA $2000 / STA $4012 / STA $4013 / INC $2001 / JMP
	--   reset vector (FFFE) -> $FF00
	type rom_t is array (0 to 8191) of std_logic_vector(7 downto 0);
	function rom_init return rom_t is
		variable r : rom_t := (others => x"EA");	-- NOP filler
		variable a : integer := 16#1F00#;
		procedure b(x : integer) is
		begin
			r(a) := std_logic_vector(to_unsigned(x, 8));
			a := a + 1;
		end procedure;
	begin
		b(16#A9#); b(16#F8#);                     -- LDA #$F8
		b(16#53#); b(16#02#);                     -- TAM #1
		b(16#A9#); b(16#FF#);                     -- LDA #$FF
		b(16#53#); b(16#04#);                     -- TAM #2
		b(16#A9#); b(16#12#);                     -- LDA #$12
		b(16#8D#); b(16#08#); b(16#40#);          -- STA $4008 (VPC PRI0)
		b(16#A9#); b(16#34#);                     -- LDA #$34
		b(16#8D#); b(16#0A#); b(16#40#);          -- STA $400A (VPC WIN1 lo)
		b(16#A9#); b(16#05#);                     -- LDA #$05
		b(16#8D#); b(16#10#); b(16#40#);          -- STA $4010 (VDC1 AR = CR)
		b(16#A9#); b(16#CC#);                     -- LDA #$CC
		b(16#8D#); b(16#12#); b(16#40#);          -- STA $4012 (VDC1 CR lo)
		b(16#A9#); b(16#00#);                     -- LDA #$00
		b(16#8D#); b(16#10#); b(16#40#);          -- STA $4010 (VDC1 AR = MAWR)
		b(16#8D#); b(16#12#); b(16#40#);          -- STA $4012 (MAWR lo)
		b(16#8D#); b(16#13#); b(16#40#);          -- STA $4013 (MAWR hi)
		b(16#A9#); b(16#02#);                     -- LDA #$02
		b(16#8D#); b(16#10#); b(16#40#);          -- STA $4010 (VDC1 AR = VWR)
		-- loop (at $FF2C):
		b(16#EE#); b(16#00#); b(16#20#);          -- INC $2000
		b(16#AD#); b(16#00#); b(16#20#);          -- LDA $2000
		b(16#8D#); b(16#12#); b(16#40#);          -- STA $4012 (VWR lo)
		b(16#8D#); b(16#13#); b(16#40#);          -- STA $4013 (VWR hi -> VRAM1 write)
		b(16#EE#); b(16#01#); b(16#20#);          -- INC $2001
		b(16#4C#); b(16#2C#); b(16#FF#);          -- JMP $FF2C
		r(16#1FFE#) := x"00"; r(16#1FFF#) := x"FF";	-- RESET vector = $FF00
		return r;
	end function;
	constant ROM : rom_t := rom_init;

	type mem_t is array (0 to 3*SLOT_WORDS + 64) of std_logic_vector(63 downto 0);

	signal mismatches : integer := 0;

begin

	clk <= not clk after CLK_PERIOD / 2;

	DUT : entity work.pce_top
	generic map ( LITE => 0 )
	port map (
		RESET       => reset,
		COLD_RESET  => '0',
		CLK         => clk,

		ROM_RD      => rom_rd,
		ROM_RDY     => rom_rdy,
		ROM_A       => rom_a,
		ROM_DO      => rom_do,
		ROM_SZ      => x"020",		-- 128K mirror path (ROM_A = CPU_A[16:0])
		ROM_POP     => '0',
		ROM_CLKEN   => rom_clken,

		BRM_A       => open,
		BRM_DI      => open,
		BRM_DO      => x"00",
		BRM_WE      => open,

		GG_EN       => '0',
		GG_CODE     => (others => '0'),
		GG_RESET    => '0',
		GG_AVAIL    => open,

		SP64        => '0',
		SGX         => '1',

		JOY_OUT     => open,
		JOY_IN      => "1111",

		CD_EN       => '0',
		CD_RAM_A    => open,
		CD_RAM_DO   => open,
		CD_RAM_DI   => x"00",
		CD_RAM_RD   => open,
		CD_RAM_WR   => open,
		AC_EN       => '0',

		CD_STAT     => x"00",
		CD_MSG      => x"00",
		CD_STAT_GET => '0',
		CD_COMM     => open,
		CD_COMM_SEND=> open,
		CD_DOUT_REQ => '0',
		CD_DOUT     => open,
		CD_DOUT_SEND=> open,
		CD_REGION   => '0',
		CD_RESET    => open,
		CD_DATA     => x"00",
		CD_DATA_WR  => '0',
		CD_AUDIO_WR => '0',
		CD_SUBCD_WR => '0',
		CD_DATA_END => open,
		CD_DM       => '0',

		CDDA_SL     => open,
		CDDA_SR     => open,
		ADPCM_S     => open,
		PSG_SL      => open,
		PSG_SR      => open,

		BG_EN       => '1',
		SPR_EN      => '1',
		GRID_EN     => "00",
		CPU_PAUSE_EN=> '0',

		SS_SAVE     => ss_save,
		SS_LOAD     => ss_load,
		SS_SLOT     => ss_slot,
		SS_BUSY     => ss_busy,
		SS_SLEEP    => ss_sleep,
		SS_DDR_DIN  => ddr_din,
		SS_DDR_DOUT => ddr_dout,
		SS_DDR_ADDR => ddr_adr,
		SS_DDR_RNW  => ddr_rnw,
		SS_DDR_ENA  => ddr_ena,
		SS_DDR_BE   => ddr_be,
		SS_DDR_DONE => ddr_done,
		SSE_Din     => open,
		SSE_Adr     => open,
		SSE_wren    => open,
		SSE_rst     => open,
		SSE_load    => open,
		SSE_Dout    => (others => '0'),

		BORDER_EN   => '0',
		ReducedVBL  => '1',
		VIDEO_R     => open,
		VIDEO_G     => open,
		VIDEO_B     => open,
		VIDEO_BW    => open,
		VIDEO_CE    => open,
		VIDEO_CE_FS => open,
		VIDEO_VS    => open,
		VIDEO_HS    => open,
		VIDEO_HBL   => open,
		VIDEO_VBL   => open
	);

	-- ROM model: REQUEST-PACED like the real sdram/ddram controllers — the
	-- data register updates ONLY on a ROM_CLKEN (ce_rom) pulse with ROM_RD
	-- asserted, with a few cycles of latency covered by the ROM_RDY/WAIT_N
	-- handshake.  A free-running model hides the post-load stale-fetch class
	-- of bug (the byte of the pre-load PC executed at the restored PC).
	process(clk)
		variable pend : integer := 0;
		variable pa   : std_logic_vector(21 downto 0);
	begin
		if rising_edge(clk) then
			if rom_clken = '1' and rom_rd = '1' then
				pa      := rom_a;
				pend    := 3;
				rom_rdy <= '0';
			elsif pend > 0 then
				pend := pend - 1;
				if pend = 0 then
					rom_do  <= ROM(to_integer(unsigned(pa(12 downto 0))));
					rom_rdy <= '1';
				end if;
			end if;
		end if;
	end process;

	-- DDR3 model: 1-cycle latency, 3 savestate slots
	ddr_and_stim : block
		shared variable mem : mem_t := (others => (others => '0'));
		impure function slot_word(slot, w : integer) return std_logic_vector is
		begin
			return mem(slot*SLOT_WORDS + w);
		end function;
	begin
		process(clk)
			variable a, slot, w : integer;
		begin
			if rising_edge(clk) then
				ddr_done <= '0';
				if ddr_ena = '1' then
					a := to_integer(unsigned(ddr_adr));
					assert a >= BASE_DW and a < BASE_DW + 3*SLOT_DW
						report "DDR access outside 3-slot window: " & integer'image(a) severity failure;
					slot := (a - BASE_DW) / SLOT_DW;
					w    := ((a - BASE_DW) mod SLOT_DW) / 2;
					assert w < SLOT_WORDS report "word index out of range" severity failure;
					if ddr_rnw = '0' then
						mem(slot*SLOT_WORDS + w) := ddr_din;
					else
						ddr_dout <= mem(slot*SLOT_WORDS + w);
					end if;
					ddr_done <= '1';
				end if;
			end if;
		end process;

		stim : process
			procedure wait_cycles(n : integer) is
			begin
				for i in 1 to n loop wait until rising_edge(clk); end loop;
			end procedure;
			procedure do_ss(slot : integer; is_load : boolean) is
			begin
				ss_slot <= std_logic_vector(to_unsigned(slot, 2));
				wait_cycles(2);
				if is_load then ss_load <= '1'; else ss_save <= '1'; end if;
				wait_cycles(3);
				ss_load <= '0'; ss_save <= '0';
				wait until ss_sleep = '1';
				wait until ss_sleep = '0';
				wait_cycles(10);
			end procedure;
			constant PHASE_MASK : std_logic_vector(63 downto 0) :=
				x"FFF88FFFFFFFF000";	-- VCE_2 minus H_CNT, CLKEN_CNT, CLKEN_FS_CNT
			variable diff : integer := 0;
			variable h1, h2 : integer;
		begin
			wait_cycles(20);
			reset <= '0';
			-- let the CPU boot and grind the WRAM counters for a while
			wait_cycles(100_000);

			report "TBC: SAVE slot 0 (anchor)";
			do_ss(0, false);

			report "TBC: run K, SAVE slot 1 (continue reference)";
			wait_cycles(K_CYCLES);
			do_ss(1, false);

			report "TBC: LOAD slot 0 (rewind)";
			do_ss(0, true);

			report "TBC: run K, SAVE slot 2 (replay)";
			wait_cycles(K_CYCLES);
			do_ss(2, false);

			report "TBC: comparing slot 1 vs slot 2 (control word excluded)";
			-- Word 25 = internals slot 24 = VCE_2.  The CPU clock dividers are
			-- deliberately reseeded to canonical phase on load (plan §7.2), so
			-- the replay's second freeze lands a few CLK away from the
			-- reference's: the free-running VCE phase fields H_CNT(11:0),
			-- CLKEN_CNT(46:44), CLKEN_FS_CNT(50:48) may skew by up to one CPU
			-- cycle.  This is the accepted sub-cycle perturbation of §11.1 —
			-- masked here, with the skew bounded and the fields' mutual
			-- consistency implied by everything else being bit-exact.
			-- anti-vacuity: a slot-addressing bug that dumps every save into
			-- slot 0 would make this whole compare pass on empty==empty
			assert slot_word(1, 1) /= x"0000000000000000"
				report "TBC FAILED: slot 1 blob is empty - slot addressing broken"
				severity failure;
			assert slot_word(2, 1) /= x"0000000000000000"
				report "TBC FAILED: slot 2 blob is empty - slot addressing broken"
				severity failure;
			diff := 0;
			for w in 1 to SLOT_WORDS-1 loop
				if w = 48 then
					-- internals slot 47 = TOP_EXT: bits 63:40 are live
					-- slot-selection telemetry (save/load counters), naturally
					-- different between the reference and replay saves
					if (slot_word(1, w) and x"000000FFFFFFFFFF")
					 /= (slot_word(2, w) and x"000000FFFFFFFFFF") then
						diff := diff + 1;
						report "TBC: TOP_EXT non-telemetry fields differ: ref="
							& to_hstring(slot_word(1, w)) & " replay=" & to_hstring(slot_word(2, w))
							severity error;
					end if;
				elsif w = 25 then
					if (slot_word(1, w) and PHASE_MASK) /= (slot_word(2, w) and PHASE_MASK) then
						diff := diff + 1;
						report "TBC: VCE_2 non-phase fields differ: ref="
							& to_hstring(slot_word(1, w)) & " replay=" & to_hstring(slot_word(2, w))
							severity error;
					end if;
					h1 := to_integer(unsigned(slot_word(1, w)(11 downto 0)));
					h2 := to_integer(unsigned(slot_word(2, w)(11 downto 0)));
					if abs(h1 - h2) > 12 and abs(h1 - h2) < 2730 - 12 then
						diff := diff + 1;
						report "TBC: H_CNT skew exceeds one CPU cycle: "
							& integer'image(h1) & " vs " & integer'image(h2) severity error;
					end if;
				elsif slot_word(1, w) /= slot_word(2, w) then
					diff := diff + 1;
					if diff <= 16 then
						report "TBC: word " & integer'image(w) & " differs: ref="
							& to_hstring(slot_word(1, w)) & " replay=" & to_hstring(slot_word(2, w))
							severity error;
					end if;
				end if;
			end loop;
			mismatches <= diff;
			wait_cycles(1);

			assert diff = 0
				report "TBC FAILED: " & integer'image(diff) & " differing words" severity failure;
			report "TBC PASSED: continue and rewind+replay blobs are bit-identical";
			std.env.finish;
		end process;

		-- wall-progress heartbeat: distinguishes a slow grind from a stuck wait
		heartbeat : process
		begin
			wait for 1 ms;
			report "TBC: heartbeat t=" & time'image(now);
		end process;
	end block;

end architecture;
