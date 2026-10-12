#!/bin/bash
# Real HuC6280 blind READ6 -> VDC regression, using tb_cd's normal libraries,
# memory model and DDR savestate walk. No private ROM/state payload is needed.
# Usage: bash sim/run_tb_cd_blind.sh [additional GHDL run arguments]
# Require TBCD BLIND PASSED and natural completion; --stop-time is not PASS.
# Sector starts poll once; the fixed-size LDA $5808 / STA VDC low/high loop
# never polls per byte. An odd low-byte latch is saved at offset21/phase10.
set -e
R="$(cd "$(dirname "$0")/.." && pwd)"
exec bash "$R/sim/run_tb_cd.sh" \
    -gG_BLIND_READ=true -gG_READS=0 \
    -gG_DO_A=false -gG_DO_B=false -gG_DO_F=false -gG_DO_G=false \
    -gG_DO_H=false -gG_DO_I=false -gG_DO_I2=false -gG_DO_J=false \
    -gG_REAL_HPS=false -gG_PUSH_STATUS=false "$@"
