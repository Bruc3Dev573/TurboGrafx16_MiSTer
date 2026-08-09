-- Behavioral altera_mf models for GHDL core-level simulation.
-- Compile into library altera_mf AFTER altera_mf_stub.vhd (same component
-- declarations; these entities provide the default binding).
--   * altsyncram: equal-width two-port RAM, zero-initialized (matches
--     power_up_uninitialized=FALSE), read-during-write = new data.
--     init_file is IGNORED (voltab/palette read as zeros — deterministic,
--     which is all the determinism TB needs).
--   * scfifo/dcfifo: permanently-empty stubs (CD path unused, CD_EN=0).
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity altsyncram is
	generic (
		address_aclr_a                 : string := "NONE";
		address_aclr_b                 : string := "NONE";
		address_reg_b                  : string := "CLOCK0";
		byte_size                      : natural := 8;
		clock_enable_input_a           : string := "NORMAL";
		clock_enable_input_b           : string := "NORMAL";
		clock_enable_output_a          : string := "NORMAL";
		clock_enable_output_b          : string := "NORMAL";
		indata_aclr_a                  : string := "NONE";
		indata_reg_b                   : string := "CLOCK1";
		init_file                      : string := "UNUSED";
		init_file_layout               : string := "PORT_A";
		intended_device_family         : string := "Cyclone V";
		lpm_hint                       : string := "UNUSED";
		lpm_type                       : string := "altsyncram";
		maximum_depth                  : natural := 0;
		numwords_a                     : natural := 0;
		numwords_b                     : natural := 0;
		operation_mode                 : string := "BIDIR_DUAL_PORT";
		outdata_aclr_a                 : string := "NONE";
		outdata_aclr_b                 : string := "NONE";
		outdata_reg_a                  : string := "UNREGISTERED";
		outdata_reg_b                  : string := "UNREGISTERED";
		power_up_uninitialized         : string := "FALSE";
		ram_block_type                 : string := "AUTO";
		rdcontrol_reg_b                : string := "CLOCK1";
		read_during_write_mode_mixed_ports : string := "DONT_CARE";
		read_during_write_mode_port_a  : string := "NEW_DATA_NO_NBE_READ";
		read_during_write_mode_port_b  : string := "NEW_DATA_NO_NBE_READ";
		widthad_a                      : natural := 1;
		widthad_b                      : natural := 1;
		width_a                        : natural := 1;
		width_b                        : natural := 1;
		width_byteena_a                : natural := 1;
		width_byteena_b                : natural := 1;
		wrcontrol_aclr_a               : string := "NONE";
		wrcontrol_wraddress_reg_b      : string := "CLOCK1"
	);
	port (
		aclr0          : in std_logic := '0';
		aclr1          : in std_logic := '0';
		address_a      : in std_logic_vector(widthad_a-1 downto 0);
		address_b      : in std_logic_vector(widthad_b-1 downto 0) := (others => '0');
		addressstall_a : in std_logic := '0';
		addressstall_b : in std_logic := '0';
		byteena_a      : in std_logic_vector(width_byteena_a-1 downto 0) := (others => '1');
		byteena_b      : in std_logic_vector(width_byteena_b-1 downto 0) := (others => '1');
		clock0         : in std_logic := '1';
		clock1         : in std_logic := '1';
		clocken0       : in std_logic := '1';
		clocken1       : in std_logic := '1';
		clocken2       : in std_logic := '1';
		clocken3       : in std_logic := '1';
		data_a         : in std_logic_vector(width_a-1 downto 0) := (others => '0');
		data_b         : in std_logic_vector(width_b-1 downto 0) := (others => '0');
		eccstatus      : out std_logic_vector(2 downto 0) := (others => '0');
		q_a            : out std_logic_vector(width_a-1 downto 0);
		q_b            : out std_logic_vector(width_b-1 downto 0);
		rden_a         : in std_logic := '1';
		rden_b         : in std_logic := '1';
		wren_a         : in std_logic := '0';
		wren_b         : in std_logic := '0'
	);
end entity;

architecture sim of altsyncram is
	type mem_t is array (0 to 2**widthad_a - 1) of std_logic_vector(width_a-1 downto 0);
	shared variable mem : mem_t := (others => (others => '0'));
begin

	assert width_a = width_b and widthad_a = widthad_b
		report "altsyncram sim model supports equal-width ports only" severity failure;

	-- Port A (clock0)
	process(clock0)
		variable ai : integer;
	begin
		if rising_edge(clock0) then
			if clocken0 = '1' then
				ai := to_integer(unsigned(address_a));
				if wren_a = '1' then
					mem(ai) := data_a;
				end if;
				q_a <= mem(ai);	-- new-data read-during-write
			end if;
		end if;
	end process;

	-- Port B (clock1)
	process(clock1)
		variable bi : integer;
	begin
		if rising_edge(clock1) then
			if clocken1 = '1' then
				bi := to_integer(unsigned(address_b));
				if wren_b = '1' then
					mem(bi) := data_b;
				end if;
				q_b <= mem(bi);
			end if;
		end if;
	end process;

end architecture;

library IEEE;
use IEEE.std_logic_1164.all;

entity scfifo is
	generic (
		add_ram_output_register : string := "OFF";
		almost_empty_value      : natural := 0;
		almost_full_value       : natural := 0;
		intended_device_family  : string := "Cyclone V";
		lpm_numwords            : natural := 0;
		lpm_showahead           : string := "OFF";
		lpm_type                : string := "scfifo";
		lpm_width               : natural := 1;
		lpm_widthu              : natural := 1;
		overflow_checking       : string := "ON";
		underflow_checking      : string := "ON";
		use_eab                 : string := "ON"
	);
	port (
		aclr         : in std_logic := '0';
		almost_empty : out std_logic;
		almost_full  : out std_logic;
		clock        : in std_logic;
		data         : in std_logic_vector(lpm_width-1 downto 0);
		empty        : out std_logic;
		full         : out std_logic;
		q            : out std_logic_vector(lpm_width-1 downto 0);
		rdreq        : in std_logic;
		sclr         : in std_logic := '0';
		usedw        : out std_logic_vector(lpm_widthu-1 downto 0);
		wrreq        : in std_logic
	);
end entity;

architecture sim of scfifo is
	-- functional ring buffer, showahead-ON semantics (q = head; rdreq pops)
begin
	process(clock)
		type mem_t is array (0 to lpm_numwords-1) of std_logic_vector(lpm_width-1 downto 0);
		variable mem : mem_t;
		variable rp, wp, cnt : integer := 0;
	begin
		if rising_edge(clock) then
			if aclr = '1' or sclr = '1' then
				rp := 0; wp := 0; cnt := 0;
			else
				if wrreq = '1' and cnt < lpm_numwords then
					mem(wp) := data;
					wp := (wp + 1) mod lpm_numwords;
					cnt := cnt + 1;
				end if;
				if rdreq = '1' and cnt > 0 then
					rp := (rp + 1) mod lpm_numwords;
					cnt := cnt - 1;
				end if;
			end if;
			if cnt > 0 then
				q <= mem(rp);
			else
				q <= (others => '0');
			end if;
			if cnt = 0 then empty <= '1'; else empty <= '0'; end if;
			if cnt >= lpm_numwords then full <= '1'; else full <= '0'; end if;
		end if;
	end process;
	almost_empty <= '0';
	almost_full  <= '0';
	usedw        <= (others => '0');
end architecture;

library IEEE;
use IEEE.std_logic_1164.all;

entity dcfifo is
	generic (
		add_usedw_msb_bit       : string := "OFF";
		clocks_are_synchronized : string := "FALSE";
		delay_rdusedw           : natural := 1;
		delay_wrusedw           : natural := 1;
		intended_device_family  : string := "Cyclone V";
		lpm_numwords            : natural := 0;
		lpm_showahead           : string := "OFF";
		lpm_type                : string := "dcfifo";
		lpm_width               : natural := 1;
		lpm_widthu              : natural := 1;
		overflow_checking       : string := "ON";
		rdsync_delaypipe        : natural := 0;
		read_aclr_synch         : string := "OFF";
		underflow_checking      : string := "ON";
		use_eab                 : string := "ON";
		write_aclr_synch        : string := "OFF";
		wrsync_delaypipe        : natural := 0
	);
	port (
		aclr    : in std_logic := '0';
		data    : in std_logic_vector(lpm_width-1 downto 0);
		q       : out std_logic_vector(lpm_width-1 downto 0);
		rdclk   : in std_logic;
		rdempty : out std_logic;
		rdfull  : out std_logic;
		rdreq   : in std_logic;
		rdusedw : out std_logic_vector(lpm_widthu-1 downto 0);
		wrclk   : in std_logic;
		wrempty : out std_logic;
		wrfull  : out std_logic;
		wrreq   : in std_logic;
		wrusedw : out std_logic_vector(lpm_widthu-1 downto 0)
	);
end entity;

architecture sim of dcfifo is
	-- functional model, single-clock assumption (rdclk = wrclk in this core)
begin
	process(wrclk)
		type mem_t is array (0 to lpm_numwords-1) of std_logic_vector(lpm_width-1 downto 0);
		variable mem : mem_t;
		variable rp, wp, cnt : integer := 0;
	begin
		if rising_edge(wrclk) then
			if aclr = '1' then
				rp := 0; wp := 0; cnt := 0;
			else
				if wrreq = '1' and cnt < lpm_numwords then
					mem(wp) := data;
					wp := (wp + 1) mod lpm_numwords;
					cnt := cnt + 1;
				end if;
				if rdreq = '1' and cnt > 0 then
					rp := (rp + 1) mod lpm_numwords;
					cnt := cnt - 1;
				end if;
			end if;
			if cnt > 0 then
				q <= mem(rp);
			else
				q <= (others => '0');
			end if;
			if cnt = 0 then rdempty <= '1'; else rdempty <= '0'; end if;
			if cnt >= lpm_numwords then wrfull <= '1'; else wrfull <= '0'; end if;
		end if;
	end process;
	rdfull  <= '0';
	rdusedw <= (others => '0');
	wrempty <= '0';
	wrusedw <= (others => '0');
end architecture;

library IEEE;
use IEEE.std_logic_1164.all;

entity dcfifo_mixed_widths is
	generic (
		intended_device_family  : string := "Cyclone V";
		lpm_numwords            : natural := 0;
		lpm_showahead           : string := "OFF";
		lpm_type                : string := "dcfifo_mixed_widths";
		lpm_width               : natural := 1;
		lpm_widthu              : natural := 1;
		lpm_widthu_r            : natural := 1;
		lpm_width_r             : natural := 1;
		overflow_checking       : string := "ON";
		rdsync_delaypipe        : natural := 0;
		read_aclr_synch         : string := "OFF";
		underflow_checking      : string := "ON";
		use_eab                 : string := "ON";
		write_aclr_synch        : string := "OFF";
		wrsync_delaypipe        : natural := 0
	);
	port (
		aclr    : in std_logic := '0';
		data    : in std_logic_vector(lpm_width-1 downto 0);
		rdclk   : in std_logic;
		rdreq   : in std_logic;
		wrfull  : out std_logic;
		q       : out std_logic_vector(lpm_width_r-1 downto 0);
		rdempty : out std_logic;
		wrclk   : in std_logic;
		wrreq   : in std_logic
	);
end entity;

architecture sim of dcfifo_mixed_widths is
	-- functional model for the equal-width use in this core (8 -> 8),
	-- single-clock, showahead-ON
begin
	assert lpm_width = lpm_width_r
		report "dcfifo_mixed_widths sim model supports equal widths only" severity failure;
	process(wrclk)
		type mem_t is array (0 to lpm_numwords-1) of std_logic_vector(lpm_width-1 downto 0);
		variable mem : mem_t;
		variable rp, wp, cnt : integer := 0;
	begin
		if rising_edge(wrclk) then
			if aclr = '1' then
				rp := 0; wp := 0; cnt := 0;
			else
				if wrreq = '1' and cnt < lpm_numwords then
					mem(wp) := data;
					wp := (wp + 1) mod lpm_numwords;
					cnt := cnt + 1;
				end if;
				if rdreq = '1' and cnt > 0 then
					rp := (rp + 1) mod lpm_numwords;
					cnt := cnt - 1;
				end if;
			end if;
			if cnt > 0 then
				q <= mem(rp);
			else
				q <= (others => '0');
			end if;
			if cnt = 0 then rdempty <= '1'; else rdempty <= '0'; end if;
			if cnt >= lpm_numwords then wrfull <= '1'; else wrfull <= '0'; end if;
		end if;
	end process;
end architecture;
