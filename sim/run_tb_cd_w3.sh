#!/bin/bash
# Scenario W3: the FMV -> stage-0 transition on the REAL Main_MiSTer HPS model
# (G_REAL_HPS - support/pcecd semantics):
#   * ONE deferred-status slot: READ6-completion and D8 use PendStatus, and a
#     second pend before delivery silently OVERWRITES the first (lost forever);
#     DA / non-INT D9 are immediate.
#   * delivery only on a 13.33 ms tick and only once the seek latency expired;
#   * persistent head position, tiered seek latencies (<=3 sect -> 33 ms,
#     <7 -> 250 ms, else G_SEEK_MS), one sector per 16 ms with per-sector ack;
#   * a savestate load changes NOTHING model-side (the HPS cannot see it).
#
# Two save/loads inside streaming READ6, then a third request during the D8
# seek-latency window. AUD_OPEN must defer that freeze until the pending
# status arrives. After restore, both subsequent READ6 transactions must
# finish without payload mismatch or lost status. TALLY lines track the
# status slot and REQ/ACK ledger at each boundary.
#
# Knobs: G_SEEK_MS=50 shortens READ seeks; this run is NOT slow-seek
# coverage. G_W3_D8LAT_MS=200 pins the D8 seek so the frozen-slot window
# comfortably outlasts save walk + load walk + the 24 ms replay gap.
#
#   sim/run_tb_cd_w3.sh [stop-time]    (default 1500ms; backstop, not PASS)
# Require TBCD W3 PASSED and natural simulation completion.
set -e

GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found"; exit 1; }

STOP="${1:-1500ms}"

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$R/sim/ghdl_work_cdw3"
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
rtl/ss_external_ram.vhd
rtl/pce_top.vhd
sim/tb_cd.vhd
"

for f in $FILES; do
	$GHDL -a $FLAGS "$R/$f"
done
$GHDL -e $FLAGS tb_cd

RUNFLAGS="--max-stack-alloc=0 --ieee-asserts=disable"
RUNFLAGS="$RUNFLAGS -gG_READS=1 -gG_DO_A=false -gG_DO_H=false -gG_DO_I=false"
RUNFLAGS="$RUNFLAGS -gG_DO_I2=false -gG_DO_J=false -gG_DO_G=false"
RUNFLAGS="$RUNFLAGS -gG_DO_W3=true -gG_REAL_HPS=true"
RUNFLAGS="$RUNFLAGS -gG_SEEK_MS=50 -gG_AUDIODELAY_MS=0 -gG_W3_D8LAT_MS=200"
RUNFLAGS="$RUNFLAGS --stop-time=$STOP"

exec $GHDL -r $FLAGS tb_cd $RUNFLAGS
