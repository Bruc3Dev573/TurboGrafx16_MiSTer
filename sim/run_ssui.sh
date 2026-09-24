#!/bin/bash
# Keyboard, slot selection and operation completion.
set -euo pipefail
R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/ssui.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
iverilog -g2012 -s tb_ssui -o "$WORK/tb_ssui" "$R/rtl/savestate_ui.sv" "$R/sim/tb_ssui.v"
vvp "$WORK/tb_ssui"
