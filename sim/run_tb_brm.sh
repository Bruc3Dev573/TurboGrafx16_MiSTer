#!/bin/bash
# Fast Backup-RAM savestate round-trip test (tb_brm.vhd). Seconds of runtime.
set -e
GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }
R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work_brm"
rm -rf "$WORK"; mkdir -p "$WORK/altera_mf"; cd "$WORK"
FLAGS="--std=08 -frelaxed-rules -fexplicit -fsynopsys --workdir=. -P. -Paltera_mf"
$GHDL -a --std=08 -frelaxed-rules --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_stub.vhd"
$GHDL -a --std=08 -frelaxed-rules --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_sim.vhd"
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
sim/tb_brm.vhd
"
for f in $FILES; do $GHDL -a $FLAGS "$R/$f"; done
$GHDL -e $FLAGS tb_brm
exec $GHDL -r $FLAGS tb_brm --max-stack-alloc=0 --ieee-asserts=disable
