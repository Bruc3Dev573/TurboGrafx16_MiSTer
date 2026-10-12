#!/bin/bash
# Focused public-port READ6 replay regression; no full-console/savestate walk.
# Usage: bash sim/run_tb_cd_read_resume.sh
set -e

GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work_cd_read_resume"
mkdir -p "$WORK/altera_mf"
cd "$WORK"
FLAGS="--std=08 -frelaxed-rules -fexplicit -fsynopsys --workdir=. -P. -Paltera_mf"

$GHDL -a --std=08 -frelaxed-rules --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_stub.vhd"
$GHDL -a --std=08 -frelaxed-rules --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_sim.vhd"
for f in rtl/bus_savestates.vhd rtl/pce_savestates_pkg.vhd rtl/dpram.vhd rtl/CEGen.vhd \
	rtl/cd/CDDA_FIFO.vhd rtl/cd/CDSUBC_FIFO.vhd rtl/cd/SCSI_FIFO.vhd \
	rtl/cd/MSM5205.vhd rtl/cd/SCSI.vhd rtl/cd/cd.vhd sim/tb_cd_read_resume.vhd; do
	$GHDL -a $FLAGS "$R/$f"
done
$GHDL -e $FLAGS tb_cd_read_resume
$GHDL -r $FLAGS tb_cd_read_resume --max-stack-alloc=0 --ieee-asserts=disable -gG_NEXT_DELAY_US=0 "$@"
$GHDL -r $FLAGS tb_cd_read_resume --max-stack-alloc=0 --ieee-asserts=disable -gG_NEXT_DELAY_US=100 "$@"
$GHDL -r $FLAGS tb_cd_read_resume --max-stack-alloc=0 --ieee-asserts=disable -gG_TAIL=true "$@"
exec $GHDL -r $FLAGS tb_cd_read_resume --max-stack-alloc=0 --ieee-asserts=disable -gG_TAIL=true -gG_ERROR=true "$@"
