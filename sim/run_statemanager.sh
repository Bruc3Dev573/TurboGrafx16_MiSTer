#!/bin/bash
# Bounded request queues, slot capture, busy acknowledgement and cancellation.
set -euo pipefail
GHDL="${GHDL:-ghdl}"
R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/statemanager.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"
FLAGS=(--std=08 -frelaxed-rules -fexplicit --workdir=. -P.)
"$GHDL" -a "${FLAGS[@]}" "$R/rtl/statemanager.vhd"
"$GHDL" -a "${FLAGS[@]}" "$R/sim/tb_statemanager.vhd"
"$GHDL" -e "${FLAGS[@]}" tb_statemanager
"$GHDL" -r "${FLAGS[@]}" tb_statemanager --assert-level=error "$@"
