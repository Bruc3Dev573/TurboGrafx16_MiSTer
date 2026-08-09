-----------------------------------------------------------------------------
-- tb_brm.vhd — fast Backup-RAM (page $F7) savestate round-trip test.
--
-- pce_top with CD_EN=1 and a MINIMAL BIOS (MPR setup + display-on + a tight
-- INC/JMP loop — no CD reads, no music), so the composite save boundary lands
-- almost immediately and the run finishes in sim-ms.  Models a real 2KB BRAM
-- at the BRM_* ports, seeds a pattern, saves, WIPES it, loads, and requires
-- the blob ALONE to restore it bit-exact.  This is scenario F from tb_cd but
-- without the slow CD-streaming scenarios in front of it — it answers the
-- hardware "wipe BRAM -> loads break" report directly.
--
-- If this PASSES, the pce_top-side region-7 walk is correct and the hardware
-- bug is in the SV top (dpram_difclk / bk_loading / defbram).  If it FAILS,
-- the bug is in pce_top and is fixed here.
-- Run: sim/run_tb_brm.sh
-----------------------------------------------------------------------------
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;

entity tb_brm is
end entity;

architecture sim of tb_brm is

	constant CLK_PERIOD : time := 10 ns;
	constant BASE_DW    : integer := 16#3800000#;
	constant SLOT_DW    : integer := 16#C0000#;
	constant CDRAM_SIM  : integer := 4096;
	constant SLOT_WORDS : integer := 33945;	-- same blob layout as tb_cd

	signal clk      : std_logic := '0';
	signal reset    : std_logic := '1';

	signal ss_save  : std_logic := '0';
	signal ss_load  : std_logic := '0';
	signal ss_slot  : std_logic_vector(1 downto 0) := "00";
	signal ss_busy  : std_logic;
	signal ss_sleep : std_logic;
	signal ss_dbg   : std_logic_vector(8 downto 0);

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

	signal cdram_a   : std_logic_vector(21 downto 0);
	signal cdram_do  : std_logic_vector(7 downto 0);
	signal cdram_di  : std_logic_vector(7 downto 0) := (others => '0');
	signal cdram_rd  : std_logic;
	signal cdram_wr  : std_logic;

	signal brm_a   : std_logic_vector(10 downto 0);
	signal brm_di  : std_logic_vector(7 downto 0);
	signal brm_do  : std_logic_vector(7 downto 0) := (others => '0');
	signal brm_we  : std_logic;

	-- Minimal BIOS: MPR setup, VDC display ON (periodic VBLANK), INC/JMP loop.
	type rom_t is array (0 to 8191) of std_logic_vector(7 downto 0);
	function rom_init return rom_t is
		variable r : rom_t := (others => x"EA");
		variable a : integer;
		procedure org(x : integer) is begin a := x; end procedure;
		procedure b(x : integer) is
		begin
			r(a) := std_logic_vector(to_unsigned(x, 8));
			a := a + 1;
		end procedure;
	begin
		org(16#1F00#);                                -- $FF00
		b(16#A9#); b(16#F8#); b(16#53#); b(16#02#);   -- LDA #$F8 TAM1 (WRAM $2000)
		b(16#A9#); b(16#FF#); b(16#53#); b(16#04#);   -- LDA #$FF TAM2 (I/O $4000)
		b(16#A9#); b(16#68#); b(16#53#); b(16#08#);   -- LDA #$68 TAM3 (CD-RAM $6000)
		b(16#A9#); b(16#05#); b(16#8D#); b(16#00#); b(16#40#);   -- AR=CR (reg 5)
		b(16#A9#); b(16#CC#); b(16#8D#); b(16#02#); b(16#40#);   -- CR lo=$CC (display on)
		b(16#A9#); b(16#00#); b(16#8D#); b(16#03#); b(16#40#);   -- CR hi=0
		-- seed WRAM[0..255] = i (STA $2000,X with A=X), then idle loop.  The
		-- save-side check verifies the blob captured this pattern bit-exact
		-- (a WRAM save-read bug would corrupt game state on every load).
		b(16#A2#); b(16#00#);                         -- $FF1B LDX #$00
		b(16#8A#);                                    -- $FF1D TXA
		b(16#9D#); b(16#00#); b(16#20#);              -- STA $2000,X
		b(16#E8#);                                    -- INX
		b(16#D0#); b(16#F9#);                         -- BNE $FF1D (-7)
		b(16#4C#); b(16#24#); b(16#FF#);              -- $FF24 JMP $FF24 (idle; WRAM untouched)
		r(16#1FFE#) := x"00"; r(16#1FFF#) := x"FF";   -- reset vector $FF00
		return r;
	end function;
	constant ROM : rom_t := rom_init;

	type mem_t is array (0 to 4*SLOT_WORDS + 64) of std_logic_vector(63 downto 0);

begin

	clk <= not clk after CLK_PERIOD / 2;

	DUT : entity work.pce_top
	generic map ( SS_CDRAM_BYTES => CDRAM_SIM, LITE => 0 )
	port map (
		RESET       => reset,
		COLD_RESET  => '0',
		CLK         => clk,
		ROM_RD      => rom_rd,
		ROM_RDY     => rom_rdy,
		ROM_A       => rom_a,
		ROM_DO      => rom_do,
		ROM_SZ      => x"020",
		ROM_POP     => '0',
		ROM_CLKEN   => rom_clken,
		BRM_A       => brm_a,
		BRM_DI      => brm_di,
		BRM_DO      => brm_do,
		BRM_WE      => brm_we,
		GG_EN       => '0',
		GG_CODE     => (others => '0'),
		GG_RESET    => '0',
		GG_AVAIL    => open,
		SP64        => '0',
		SGX         => '0',
		JOY_OUT     => open,
		JOY_IN      => "1111",
		CD_EN       => '1',
		CD_RAM_A    => cdram_a,
		CD_RAM_DO   => cdram_do,
		CD_RAM_DI   => cdram_di,
		CD_RAM_RD   => cdram_rd,
		CD_RAM_WR   => cdram_wr,
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
		SS_DBG      => ss_dbg,
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

	-- Shared ROM / CD-RAM port (TurboGrafx16.sv structure)
	mem_port : process(clk)
		type cdram_t is array (0 to CDRAM_SIM-1) of std_logic_vector(7 downto 0);
		variable cdram  : cdram_t := (others => x"00");
		variable pend   : integer := 0;
		variable pa     : integer := 0;
		variable p_rom  : boolean := false;
	begin
		if rising_edge(clk) then
			if rom_clken = '1' then
				if cdram_wr = '1' then
					cdram(to_integer(unsigned(cdram_a(17 downto 0))) mod CDRAM_SIM) := cdram_do;
				end if;
				if rom_rd = '1' or cdram_rd = '1' then
					p_rom := rom_rd = '1';
					if p_rom then
						pa := to_integer(unsigned(rom_a(12 downto 0)));
					else
						pa := to_integer(unsigned(cdram_a(17 downto 0))) mod CDRAM_SIM;
					end if;
					pend := 3;
					rom_rdy <= '0';
				end if;
			elsif pend > 0 then
				pend := pend - 1;
				if pend = 0 then
					if p_rom then
						rom_do   <= ROM(pa);
						cdram_di <= ROM(pa);
					else
						rom_do   <= cdram(pa);
						cdram_di <= cdram(pa);
					end if;
					rom_rdy <= '1';
				end if;
			end if;
		end if;
	end process;

	ddr_and_stim : block
		shared variable mem  : mem_t := (others => (others => '0'));
		type brm_t is array (0 to 2047) of std_logic_vector(7 downto 0);
		function brm_init return brm_t is
			variable bb : brm_t;
		begin
			for i in bb'range loop
				bb(i) := std_logic_vector(to_unsigned((i*37 + 11) mod 256, 8));
			end loop;
			return bb;
		end function;
		shared variable brm : brm_t := brm_init;
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
					assert a >= BASE_DW and a < BASE_DW + 4*SLOT_DW
						report "DDR access outside 4-slot window" severity failure;
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

		brm_model : process(clk)
		begin
			if rising_edge(clk) then
				if brm_we = '1' then
					brm(to_integer(unsigned(brm_a))) := brm_di;
				end if;
				brm_do <= brm(to_integer(unsigned(brm_a)));
			end if;
		end process;

		stim : process
			procedure wait_cycles(n : integer) is
			begin
				for i in 1 to n loop wait until rising_edge(clk); end loop;
			end procedure;
			procedure do_ss(slot : integer; is_load : boolean; tag : string) is
			begin
				ss_slot <= std_logic_vector(to_unsigned(slot, 2));
				wait_cycles(2);
				if is_load then ss_load <= '1'; else ss_save <= '1'; end if;
				wait_cycles(3);
				ss_load <= '0'; ss_save <= '0';
				for i in 1 to 400 loop
					wait until ss_sleep = '1' for 5 ms;
					exit when ss_sleep = '1';
					report "TBBRM WAIT(" & tag & ") SS_DBG=" & to_string(ss_dbg);
				end loop;
				assert ss_sleep = '1'
					report "TBBRM HANG(" & tag & "): freeze never engaged; SS_DBG="
						& to_string(ss_dbg) severity failure;
				wait until ss_sleep = '0' for 500 ms;
				assert ss_sleep = '0'
					report "TBBRM HANG(" & tag & "): walk never completed" severity failure;
				report "TBBRM: " & tag & " ok";
				wait_cycles(20);
			end procedure;
			variable v        : std_logic_vector(63 downto 0);
			variable diff, exp, got : integer;
		begin
			wait_cycles(20);
			reset <= '0';
			wait for 2 ms;			-- let the loop BIOS settle

			report "TBBRM: === BRAM (page $F7) save/WIPE/load round-trip ===";
			do_ss(0, false, "BRAM save");
			-- WRAM save-read check: the BIOS seeded WRAM[i]=i; blob region 0
			-- (words 129..) must carry it bit-exact.  A WRAM save-read bug
			-- (the ADPCM-shuttle class) would corrupt game state on every load.
			diff := 0;
			for k in 0 to 31 loop
				v := slot_word(0, 129 + k);
				for i in 0 to 7 loop
					exp := (8*k + i) mod 256;
					got := to_integer(unsigned(v(i*8+7 downto i*8)));
					if got /= exp then
						diff := diff + 1;
						if diff <= 12 then
							report "TBBRM WRAM byte " & integer'image(8*k+i)
								& " blob=" & integer'image(got) & " exp=" & integer'image(exp)
								severity warning;
						end if;
					end if;
				end loop;
			end loop;
			assert diff = 0
				report "TBBRM FAILED (WRAM save): " & integer'image(diff)
					& " bytes wrong -- WRAM save-read corrupts game state" severity failure;
			report "TBBRM: WRAM save-read OK (256 bytes)";
			-- save-side: blob BRM region (words 24961..25216) must carry the seed
			diff := 0;
			for k in 0 to 255 loop
				v := slot_word(0, 24961 + k);
				for i in 0 to 7 loop
					exp := (((8*k + i)*37 + 11) mod 256);
					got := to_integer(unsigned(v(i*8+7 downto i*8)));
					if got /= exp then
						diff := diff + 1;
						if diff <= 8 then
							report "TBBRM save: BRM byte " & integer'image(8*k+i)
								& " blob=" & integer'image(got) & " exp=" & integer'image(exp)
								severity warning;
						end if;
					end if;
				end loop;
			end loop;
			assert diff = 0
				report "TBBRM FAILED (save): BRM not captured (" & integer'image(diff)
					& " bytes) -- region-7 read broken" severity failure;
			report "TBBRM: BRM captured into blob OK";

			for i in 0 to 2047 loop brm(i) := x"FF"; end loop;	-- wipe
			do_ss(0, true, "BRAM load");
			diff := 0;
			for i in 0 to 2047 loop
				exp := ((i*37 + 11) mod 256);
				if to_integer(unsigned(brm(i))) /= exp then
					diff := diff + 1;
					if diff <= 8 then
						report "TBBRM load: BRM byte " & integer'image(i) & " ="
							& integer'image(to_integer(unsigned(brm(i))))
							& " exp=" & integer'image(exp) severity warning;
					end if;
				end if;
			end loop;
			assert diff = 0
				report "TBBRM FAILED: BRAM not restored from blob (" & integer'image(diff)
					& " bytes) -- load leans on persistent BRAM" severity failure;
			report "TBBRM: PASSED (BRAM round-trips through the blob alone)";
			std.env.finish;
		end process;
	end block;

end architecture;
