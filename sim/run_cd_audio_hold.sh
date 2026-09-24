#!/bin/bash
# Post-load audio hold at the CD unit: resume, abandon, no-hold, watchdog,
# and the replay D8 fast-seek bit (slow only for a D8 the game still awaited).
#   sim/run_cd_audio_hold.sh [cases]   (default "0:60 0:1 1:60 3:0 4:60 5:60 6:60 2:60")
# Each case is G_CASE:G_SKIP; case 6 simulates ~0.35 s, case 2 (watchdog) ~0.7 s.
set -euo pipefail
GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found" >&2; exit 1; }
R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/ghdl_cd_audio_hold.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/altera_mf"
cd "$WORK"
FLAGS=(--std=08 -frelaxed-rules -fexplicit -fsynopsys --workdir=. -P. -Paltera_mf)
"$GHDL" -a --std=08 -frelaxed-rules --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_stub.vhd" "$R/sim/altera_mf_sim.vhd"
for f in rtl/bus_savestates.vhd rtl/pce_savestates_pkg.vhd rtl/dpram.vhd rtl/CEGen.vhd rtl/cd/CDDA_FIFO.vhd rtl/cd/CDSUBC_FIFO.vhd rtl/cd/SCSI_FIFO.vhd rtl/cd/MSM5205.vhd rtl/cd/SCSI.vhd rtl/cd/cd.vhd sim/tb_cd_audio_hold.vhd; do
    "$GHDL" -a "${FLAGS[@]}" "$R/$f"
done
"$GHDL" -e "${FLAGS[@]}" tb_cd_audio_hold
for pair in ${1:-0:60 0:1 1:60 3:0 4:60 5:60 6:60 2:60}; do
    echo "=== case ${pair%:*} skip ${pair#*:} $(date -u +%T)"
    "$GHDL" -r "${FLAGS[@]}" tb_cd_audio_hold -gG_CASE="${pair%:*}" -gG_SKIP="${pair#*:}" --max-stack-alloc=0 --ieee-asserts=disable --assert-level=error
done
echo "ALL AHOLD CASES PASSED"
