#!/usr/bin/env python3
"""Cross-check savestate size/coverage invariants across the tree.

Catches the drift classes that produced real hardware bugs:
  - an eReg slot index >= INTERNALSCOUNT (slot 64 was silently never walked)
  - testbench size constants stale after a format change
"""
import re, sys, os

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def read(p):
    return open(os.path.join(R, p), encoding="utf-8", errors="replace").read()

errors = []

eng = read("rtl/savestates.vhd")
internals = int(re.search(r"INTERNALSCOUNT\s*:\s*integer\s*:=\s*(\d+)", eng).group(1))
header    = int(re.search(r"HEADERCOUNT\s*:\s*integer\s*:=\s*(\d+)", eng).group(1))
cdram_default = int(re.search(r"CDRAM_BYTES\s*:\s*integer\s*:=\s*(\d+)", eng).group(1))
sizes = [int(m.group(1)) if m.group(1).isdigit() else cdram_default
         for m in re.finditer(r"\(16#[0-9A-Fa-f]+#,\s*(\w+)\)", eng)]
ntypes = int(re.search(r"SAVETYPESCOUNT\s*:\s*integer\s*:=\s*(\d+)", eng).group(1))
if len(sizes) != ntypes:
    errors.append(f"savetypes table has {len(sizes)} entries, SAVETYPESCOUNT={ntypes}")
statesize = header + internals * 2 + sum(sizes) // 4
words64   = statesize // 2

pkg = read("rtl/pce_savestates_pkg.vhd")
slot_max = max(int(m.group(1)) for m in re.finditer(r"SSREG_INDEX_\w+\s*:\s*integer\s*:=\s*(\d+)", pkg))
if slot_max >= internals:
    errors.append(f"pkg uses eReg slot {slot_max} but the walk covers only 0..{internals-1}")

for f, pat, expect, what in [
    ("sim/tb_savestates.vhd", r"EXP_STATESIZE\s*:\s*integer\s*:=\s*(\d+)", statesize, "EXP_STATESIZE"),
    ("sim/tb_savestates.vhd", r"EXP_MEM_BYTES\s*:\s*integer\s*:=\s*(\d+)", sum(sizes), "EXP_MEM_BYTES"),
    ("sim/tb_core.vhd",       r"SLOT_WORDS\s*:\s*integer\s*:=\s*(\d+)", words64, "tb_core SLOT_WORDS"),
    ("sim/tb_broken.vhd",     r"SLOT_WORDS\s*:\s*integer\s*:=\s*(\d+)", words64, "tb_broken SLOT_WORDS"),
]:
    got = int(re.search(pat, read(f)).group(1))
    if got != expect:
        errors.append(f"{f}: {what}={got}, expected {expect}")

dec = read("sim/decode_ss.py")
regions_block = re.search(r"REGIONS = \[(.*?)\]", dec, re.S).group(1)
nreg = len(re.findall(r'\("\w+",\s*\d+\)', regions_block))
if nreg != ntypes:
    errors.append(f"decode_ss.py REGIONS has {nreg} entries, engine has {ntypes}")
off = re.search(r"off = \(1 \+ (\d+)\) \* 8", dec)
if off and int(off.group(1)) != internals:
    errors.append(f"decode_ss.py memory offset uses {off.group(1)} internals, engine has {internals}")

if errors:
    for e in errors:
        print("LINT FAIL:", e)
    sys.exit(1)
print(f"LINT OK: STATESIZE={statesize} DW ({words64} w64), internals={internals}, max slot={slot_max}, regions={ntypes}")
