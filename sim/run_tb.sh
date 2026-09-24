#!/bin/bash
# Run the savestate-engine testbench with GHDL.
# Usage: sim/run_tb.sh   (from the repo root or from sim/)
set -e

GHDL="$(command -v ghdl || true)"
if [ -z "$GHDL" ]; then
	# Homebrew cask fallback (macOS)
	GHDL="$(ls -d /opt/homebrew/Caskroom/ghdl/*/ghdl-*/bin/ghdl 2>/dev/null | head -1)"
fi
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work"
mkdir -p "$WORK"
cd "$WORK"

FLAGS="--std=08 -frelaxed-rules -fexplicit --workdir=. -P."

$GHDL -a $FLAGS "$R/rtl/bus_savestates.vhd"
$GHDL -a $FLAGS "$R/rtl/pce_savestates_pkg.vhd"
$GHDL -a $FLAGS "$R/rtl/savestates.vhd"
$GHDL -a $FLAGS "$R/rtl/statemanager.vhd"
$GHDL -a $FLAGS "$R/sim/tb_savestates.vhd"
$GHDL -e $FLAGS tb_savestates
$GHDL -r $FLAGS tb_savestates --max-stack-alloc=0
