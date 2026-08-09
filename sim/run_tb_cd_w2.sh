#!/bin/bash
# Scenario W2: the residual hardware case - the FMV -> stage-0 TRANSITION.
#
# Runs scenario W's soak (push-status drive model, save/load mid-stream and in
# the tail window, fresh READ6 #1 - all of which passes on current RTL), then
# mimics the stage-0 loader: D8+D9 (CDDA starts while loading), a THIRD
# save/load landing in the AUDIO window (AUDIO_ACTIVE=1, CDDA streaming, the
# program in a busy-wait), then READ6 #2 and #3 back-to-back.  Every
# conversation must close byte-exact (data, STATUS, MSGIN) and the program
# must reach its end marker; "TBCD W2 TALLY" lines report SWALLOW, STAT_GET
# strobes vs STAT_PENDs, mailbox sends vs game SELs and the REQ/ACK ledger at
# every transaction boundary - the sim counterpart of the hardware dump
# (parked in wait-for-STATUS, REQ=ACK+1, 3 sends vs 1 SEL).
#
# Prints "TBCD W2 PASSED" or "TBCD W2 FAILED: <reason>" and terminates either
# way (plus the --stop-time backstop).
#
#   sim/run_tb_cd_w2.sh [stop-time]    (default 600ms; expect the verdict
#                                       around ~420 sim-ms, ~70-100 min wall)
set -e

GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }

STOP="${1:-600ms}"

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work_cdw2"
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

# Same configuration as run_tb_cd_w.sh plus G_DO_W2.  G_READS=1: the boot's
# single verified READ6 is the streaming read the first two saves land in.
RUNFLAGS="--max-stack-alloc=0 --ieee-asserts=disable"
RUNFLAGS="$RUNFLAGS -gG_READS=1 -gG_DO_A=false -gG_DO_H=false -gG_DO_I=false"
RUNFLAGS="$RUNFLAGS -gG_DO_I2=false -gG_DO_J=false -gG_DO_G=false"
RUNFLAGS="$RUNFLAGS -gG_DO_W2=true -gG_PUSH_STATUS=true -gG_STAT_TICKS=10"
RUNFLAGS="$RUNFLAGS --stop-time=$STOP"

exec $GHDL -r $FLAGS tb_cd $RUNFLAGS
