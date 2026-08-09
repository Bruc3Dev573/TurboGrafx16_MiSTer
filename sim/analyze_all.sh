#!/bin/bash
# GHDL --std=08 static analysis of the whole core VHDL tree (syntax/type gate).
# Uses sim/altera_mf_stub.vhd in place of the Quartus altera_mf library.
# Usage: sim/analyze_all.sh
set -e

GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work_full"
mkdir -p "$WORK"
cd "$WORK"

FLAGS="--std=08 -frelaxed-rules -fexplicit -fsynopsys --workdir=. -P."

# altera_mf stub compiled into library altera_mf
mkdir -p altera_mf
$GHDL -a --std=08 --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_stub.vhd"

FLAGS="$FLAGS -Paltera_mf"

# Order matters (packages first, leaf entities before users).
FILES="
rtl/bus_savestates.vhd
rtl/pce_savestates_pkg.vhd
rtl/savestates.vhd
rtl/statemanager.vhd
rtl/dpram.vhd
rtl/CEGen.vhd
rtl/HUC6280/HUC6280_PKG.vhd
rtl/HUC6280/AddSubBCD.vhd
rtl/HUC6280/HUC6280_MC.vhd
rtl/HUC6280/HUC6280_ALU.vhd
rtl/HUC6280/HUC6280_AG.vhd
rtl/HUC6280/HUC6280_CPU.vhd
rtl/HUC6280/psg.vhd
rtl/HUC6280/HUC6280.vhd
rtl/huc6260.vhd
rtl/huc6270.vhd
rtl/huc6202.vhd
rtl/cd/CDDA_FIFO.vhd
rtl/cd/CDSUBC_FIFO.vhd
rtl/cd/SCSI_FIFO.vhd
rtl/cd/MSM5205.vhd
rtl/cd/SCSI.vhd
rtl/cd/cd.vhd
sim/codes_stub.vhd
sim/arcade_stub.vhd
rtl/pce_top.vhd
"

for f in $FILES; do
	echo "-- $f"
	$GHDL -a $FLAGS "$R/$f"
done
if command -v python3 >/dev/null 2>&1; then
  python3 "$R/sim/lint_sizes.py" || exit 1
else
  echo "LINT SKIPPED (no python3 in container - run sim/lint_sizes.py on the host)"
fi
echo "ANALYSIS OK"
