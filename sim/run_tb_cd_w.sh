#!/bin/bash
# Scenario W: the hardware CD savestate soak, on the PUSH-STATUS drive model.
#
# The real MiSTer HPS (cd.cpp) pushes READ6 sectors as fast as the core's
# 4096-byte SCSI_FIFO backpressure allows and strobes the final STATUS after
# the last byte is PUSHED - potentially with ~4KB still unconsumed, and,
# post-load, while the replay drain is still eating the skip prefix.  Every
# other runner uses the consumption-gated model, which can never open the
# tail window (all data consumed, STATUS not yet strobed, READ_ACTIVE=1)
# where RP_RD_PREP takes its sect>=cnt branch: re-read the LAST sector with
# RP_RSKIP=2048 and drain it whole.
#
# W does: save/load mid-stream, save/load in the tail window, then a FRESH
# READ6 at a different LBA whose whole conversation is verified.  A stale
# prepended byte, a retry loop or a wedge is the REPRO this scenario exists
# to catch; it prints "TBCD W PASSED" or "TBCD W FAILED: <reason>" and
# terminates either way (plus the --stop-time backstop below).
#
#   sim/run_tb_cd_w.sh [stop-time]     (default 300ms; expect the verdict
#                                       around ~200 sim-ms, ~35-50 min wall)
set -e

GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }

STOP="${1:-300ms}"

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work_cdw"
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

# G_READS=1: the boot's single verified READ6 is the streaming read under
# test (both W saves land inside it).  Everything else off: W finishes with
# its own std.env.finish, and the music/DMA/SUBQ scenarios only stand between
# the boot and the verdict.  G_STAT_TICKS=10 (500 us) keeps the drain-end /
# status gap wide open so the drain boundary is observable.
RUNFLAGS="--max-stack-alloc=0 --ieee-asserts=disable"
RUNFLAGS="$RUNFLAGS -gG_READS=1 -gG_DO_A=false -gG_DO_H=false -gG_DO_I=false"
RUNFLAGS="$RUNFLAGS -gG_DO_I2=false -gG_DO_J=false -gG_DO_G=false"
RUNFLAGS="$RUNFLAGS -gG_DO_W=true -gG_PUSH_STATUS=true -gG_STAT_TICKS=10"
RUNFLAGS="$RUNFLAGS --stop-time=$STOP"

exec $GHDL -r $FLAGS tb_cd $RUNFLAGS
