// pce_savestates.sv
// SystemVerilog mirror of rtl/pce_savestates_pkg.vhd — KEEP IN SYNC.
// Slot/bit layout documentation lives in the VHDL package; this file only
// exposes the indices/defaults needed on the SV side (TurboGrafx16.sv).

package pce_savestates;

	// HuC6280 CPU
	parameter [9:0] SSREG_INDEX_CPU_1      = 10'd0;
	parameter [9:0] SSREG_INDEX_CPU_2      = 10'd1;
	parameter [9:0] SSREG_INDEX_CPU_MPR    = 10'd2;
	parameter [9:0] SSREG_INDEX_CPU_TIMER  = 10'd3;

	// PSG
	parameter [9:0] SSREG_INDEX_PSG_GLOBAL = 10'd4;
	parameter [9:0] SSREG_INDEX_PSG_CH0    = 10'd5;   // 3 slots per channel
	parameter [9:0] SSREG_INDEX_PSG_CH5    = 10'd20;  // last base (20,21,22)

	// VCE
	parameter [9:0] SSREG_INDEX_VCE_1      = 10'd23;
	parameter [9:0] SSREG_INDEX_VCE_2      = 10'd24;

	// VDC instances (10 slots each)
	parameter [9:0] SSREG_INDEX_VDC0       = 10'd25;  // 25..34
	parameter [9:0] SSREG_INDEX_VDC1       = 10'd35;  // 35..44 (SGX, LITE=0)

	// VPC (SGX)
	parameter [9:0] SSREG_INDEX_VPC        = 10'd45;

	// pce_top glue (rombank)
	parameter [9:0] SSREG_INDEX_TOP        = 10'd46;

	// SV input-block state (high_buttons/joy_port/joy_latch/scan_counter/joyrept/last_gp)
	parameter [9:0] SSREG_INDEX_TOP_EXT    = 10'd47;
	parameter [63:0] SSREG_DEFAULT_TOP_EXT = 64'h0000000000000000;

	// Arcade Card (P4)
	parameter [9:0] SSREG_INDEX_AC_PORT0   = 10'd48;
	parameter [9:0] SSREG_INDEX_AC_CTRL    = 10'd52;

	// CD (P5) reserved 53..59; free 60..63.
	parameter [9:0] SSREG_INDEX_CD_BASE    = 10'd53;

endpackage
