#!/bin/bash
# tb_cd with the verified-READ phase and scenario A switched OFF, in its own
# workdir.  Those two are what make the full run take hours: each verified
# READ6 costs ~85 ms of simulation and scenario A adds five savestate walks.
# Scenarios B (CDDA sample-exact resume), G (resume after a loop wrap) and F
# (BRAM round-trip) need none of it, so this configuration reaches B about
# 280 ms of simulation earlier -- minutes instead of an hour per iteration.
#
# Run the full matrix with sim/run_tb_cd.sh; run this one alongside it (they
# use separate workdirs) when iterating on the CDDA/BRAM scenarios.
#   sim/run_tb_cd_fast.sh [stop-time]
set -e

GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }

STOP="${1:-}"

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work_cdfast"
mkdir -p "$WORK/altera_mf"
cd "$WORK"

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
sim/tb_cd.vhd
"

for f in $FILES; do
	$GHDL -a $FLAGS "$R/$f"
done
$GHDL -e $FLAGS tb_cd

# G_DO_H=false: scenario H runs BEFORE the music (its READ6 would knock the
# drive model out of H_PLAY), so leaving it on means a failing H aborts the run
# before B, G and F are ever reached -- this script would then silently test
# nothing.  Use sim/run_tb_cd_dma.sh for H.
RUNFLAGS="--max-stack-alloc=0 --ieee-asserts=disable -gG_READS=0 -gG_DO_A=false -gG_DO_H=false"
[ -n "$STOP" ] && RUNFLAGS="$RUNFLAGS --stop-time=$STOP"

exec $GHDL -r $FLAGS tb_cd $RUNFLAGS
