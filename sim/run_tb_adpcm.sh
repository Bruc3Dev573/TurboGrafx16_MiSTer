#!/bin/bash
# Fast standalone ADPCM unit harness (tb_adpcm.vhd). Seconds of runtime.
# Usage: sim/run_tb_adpcm.sh
set -e

GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work_adpcm"
rm -rf "$WORK"
mkdir -p "$WORK/altera_mf"
cd "$WORK"

FLAGS="--std=08 -frelaxed-rules -fexplicit -fsynopsys --workdir=. -P. -Paltera_mf"

$GHDL -a --std=08 -frelaxed-rules --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_stub.vhd"
$GHDL -a --std=08 -frelaxed-rules --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_sim.vhd"

FILES="
rtl/bus_savestates.vhd
rtl/pce_savestates_pkg.vhd
rtl/dpram.vhd
rtl/CEGen.vhd
rtl/cd/CDDA_FIFO.vhd
rtl/cd/CDSUBC_FIFO.vhd
rtl/cd/SCSI_FIFO.vhd
rtl/cd/MSM5205.vhd
rtl/cd/SCSI.vhd
rtl/cd/cd.vhd
sim/tb_adpcm.vhd
"

for f in $FILES; do
	$GHDL -a $FLAGS "$R/$f"
done
$GHDL -e $FLAGS tb_adpcm
exec $GHDL -r $FLAGS tb_adpcm --max-stack-alloc=0 --ieee-asserts=disable
