#!/bin/bash
# Short + full-size real-engine/shared-bridge roundtrips; GHDL mcode, no core/stub/ROM.
# Usage: bash sim/run_arcade_memory.sh [--faults] [GHDL run options]
# --faults uses 16 CD bytes for the deadline, failed save/load and recovery cases.
set -euo pipefail
GHDL="${GHDL:-ghdl}"
case "$("$GHDL" --version)" in
    *mcode*) ;;
    *) echo "Arcade memory probe requires GHDL mcode" >&2; exit 1 ;;
esac
R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/arcade_memory.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"
FLAGS=(--std=08 -frelaxed-rules -fexplicit --workdir=. -P.)
"$GHDL" -a "${FLAGS[@]}" "$R/rtl/bus_savestates.vhd"
"$GHDL" -a "${FLAGS[@]}" "$R/rtl/savestates.vhd"
"$GHDL" -a "${FLAGS[@]}" "$R/rtl/ss_external_ram.vhd"
"$GHDL" -a "${FLAGS[@]}" "$R/sim/tb_arcade_memory.vhd"
"$GHDL" -e "${FLAGS[@]}" tb_arcade_memory
if [[ ${1:-} == --faults ]]; then
    shift
    set -- -gG_FAULTS=true -gCD_BYTES=16 "$@"
fi
"$GHDL" -r "${FLAGS[@]}" tb_arcade_memory --max-stack-alloc=0 --assert-level=error "$@"
