#!/bin/bash
# Real CD replay: DAC position, subcode byte/phase, and delayed FIFO filling.
set -euo pipefail

GHDL="$(command -v ghdl || true)"
[ -n "$GHDL" ] || { echo "ghdl not found" >&2; exit 1; }
R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/ghdl_cd_subcode_resume.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/altera_mf"
cd "$WORK"
FLAGS=(--std=08 -frelaxed-rules -fexplicit -fsynopsys --workdir=. -P. -Paltera_mf)

"$GHDL" -a --std=08 -frelaxed-rules --work=altera_mf --workdir=altera_mf "$R/sim/altera_mf_stub.vhd" "$R/sim/altera_mf_sim.vhd"
for f in rtl/bus_savestates.vhd rtl/pce_savestates_pkg.vhd rtl/dpram.vhd rtl/CEGen.vhd rtl/cd/CDDA_FIFO.vhd rtl/cd/CDSUBC_FIFO.vhd rtl/cd/SCSI_FIFO.vhd rtl/cd/MSM5205.vhd rtl/cd/SCSI.vhd rtl/cd/cd.vhd; do
    "$GHDL" -a "${FLAGS[@]}" "$R/$f"
done
"$GHDL" -a "${FLAGS[@]}" "$R/sim/tb_cd_subcode_resume.vhd"
"$GHDL" -e "${FLAGS[@]}" tb_cd_subcode_resume
for pair in 0:false 60:false 1:true 60:true 61:true 65:true 587:false; do
    "$GHDL" -r "${FLAGS[@]}" tb_cd_subcode_resume -gG_SKIP="${pair%:*}" -gG_DELAYED="${pair#*:}" --max-stack-alloc=0 --ieee-asserts=disable --assert-level=error
done
"$GHDL" -a "${FLAGS[@]}" "$R/sim/tb_cd_freeze.vhd"
"$GHDL" -e "${FLAGS[@]}" tb_cd_freeze
"$GHDL" -r "${FLAGS[@]}" tb_cd_freeze --max-stack-alloc=0 --ieee-asserts=disable --assert-level=error
echo "ALL CD SUBCODE RESUME AND FREEZE SCENARIOS PASSED"
