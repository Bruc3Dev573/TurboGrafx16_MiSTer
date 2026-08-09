-- Minimal altera_mf stub for GHDL static analysis of the core VHDL tree.
-- NOT for simulation of RAM contents — analysis/elaboration only.
library IEEE;
use IEEE.std_logic_1164.all;

package altera_mf_components is
	component altsyncram
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
		eccstatus      : out std_logic_vector(2 downto 0);
		q_a            : out std_logic_vector(width_a-1 downto 0);
		q_b            : out std_logic_vector(width_b-1 downto 0);
		rden_a         : in std_logic := '1';
		rden_b         : in std_logic := '1';
		wren_a         : in std_logic := '0';
		wren_b         : in std_logic := '0'
	);
	end component;

	component scfifo
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
	end component;

	component dcfifo
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
	end component;
end package;
