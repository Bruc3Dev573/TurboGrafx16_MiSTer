#!/bin/bash
# Real Arcade SV + the production VHDL eRegs. Run with hdlc/sim:osvb.
set -euo pipefail
for tool in ghdl verilator timeout; do
    command -v "$tool" >/dev/null || { echo "$tool not found" >&2; exit 1; }
done
R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d /tmp/arcade_snapshot.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

ghdl -a --std=08 "$R/rtl/bus_savestates.vhd"
# GHDL specializes generics during synthesis. These temporary wrappers expose
# each real specialization; the SV adapter only matches mixed-language names.
cat > ereg_adapter.sv <<'SV'
module eReg_SavestateV #(parameter integer Adr = 0, parameter [63:0] def = 0)
(
    input clk, input [63:0] BUS_Din, input [9:0] BUS_Adr,
    input BUS_wren, BUS_rst, output [63:0] BUS_Dout,
    input [63:0] Din, output [63:0] Dout
);
    initial if (def != 0 || Adr < 48 || Adr > 52)
        $fatal(1, "unsupported Arcade eReg specialization");
SV
for slot in 48 49 50 51 52; do
    cat > "ereg_${slot}.vhd" <<VHDL
library ieee;
use ieee.std_logic_1164.all;
entity arcade_ereg_${slot} is
    port (clk : in std_logic;
          bus_din : in std_logic_vector(63 downto 0);
          bus_adr : in std_logic_vector(9 downto 0);
          bus_wren, bus_rst : in std_logic;
          bus_dout : out std_logic_vector(63 downto 0);
          din : in std_logic_vector(63 downto 0);
          dout : out std_logic_vector(63 downto 0));
end entity;
architecture bridge of arcade_ereg_${slot} is
begin
    actual : entity work.eReg_SavestateV
        generic map (Adr => ${slot}, def => (63 downto 0 => '0'))
        port map (clk, bus_din, bus_adr, bus_wren, bus_rst, bus_dout, din, dout);
end architecture;
VHDL
    ghdl -a --std=08 "ereg_${slot}.vhd"
    timeout 30s ghdl --synth --std=08 --out=verilog "arcade_ereg_${slot}" > "ereg_${slot}.v"
    cat >> ereg_adapter.sv <<SV
    if (Adr == ${slot}) begin : slot_${slot}
        arcade_ereg_${slot} actual (
            .clk(clk), .bus_din(BUS_Din), .bus_adr(BUS_Adr),
            .bus_wren(BUS_wren), .bus_rst(BUS_rst), .bus_dout(BUS_Dout),
            .din(Din), .dout(Dout));
    end
SV
done
printf 'endmodule\n' >> ereg_adapter.sv

timeout 180s verilator --binary --timing -j 2 -Wno-fatal "$@" \
    --top-module tb_arcade_snapshot --Mdir "$WORK/obj" \
    "$R/rtl/arcade.sv" "$R/sim/tb_arcade_snapshot.sv" \
    ereg_adapter.sv ereg_48.v ereg_49.v ereg_50.v ereg_51.v ereg_52.v
timeout 30s "$WORK/obj/Vtb_arcade_snapshot"
