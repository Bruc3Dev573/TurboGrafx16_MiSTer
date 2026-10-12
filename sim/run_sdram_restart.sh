#!/bin/bash
set -euo pipefail
for tool in verilator timeout; do
    command -v "$tool" >/dev/null || { echo "$tool not found" >&2; exit 1; }
done
R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d /tmp/sdram_restart.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"
timeout 180s verilator --binary --timing -j 2 -Wno-fatal "$@" \
    --top-module tb_sdram_restart --Mdir "$WORK/obj" \
    "$R/rtl/sdram.v" "$R/sim/tb_sdram_restart.sv"
timeout 30s "$WORK/obj/Vtb_sdram_restart"
