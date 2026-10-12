#!/usr/bin/env python3
"""Decode a TurboGrafx16_MiSTer savestate blob (pce_savestates_pkg map).

Usage:
  decode_ss.py <blob.bin>            # raw dump of one slot

A blob comes either from a slot file written by the core or from a dump of the
slot's DDR window; this script only reads bytes, it never talks to a board.

Blob layout (see the savestate documentation, section "slot layout"):
  word 0     : STATESIZE(63:32) | header_count(31:0)
  words 1-128 : internals = eReg slots 0..127
  words 129.. : memory regions listed in REGIONS below
"""
import sys, struct

REGIONS = [("WRAM",32768),("VRAM0",65536),("SAT0",512),("PALETTE",1024),
           ("VRAM1",65536),("SAT1",512),("PRAM",32768),("BRM",2048),("PSGWF",192),
           ("ADPCM",65536),("CDRAM",262144),("ACRAM",2097152)]
STATESIZE_FULL = (8 + 128 * 8 + sum(size for _, size in REGIONS)) // 4
STATESIZE_SHORT = STATESIZE_FULL - REGIONS[-1][1] // 4

def regions_for_size(size):
    if size == STATESIZE_FULL:
        return REGIONS
    if size == STATESIZE_SHORT:
        return REGIONS[:-1]
    raise ValueError(f"unknown/invalid STATESIZE {size:#x}")

def bits(v, hi, lo): return (v >> lo) & ((1 << (hi-lo+1)) - 1)

def decode(words):
    if not words:
        raise ValueError("truncated slot header")
    if len(words) < 129:
        raise ValueError(f"truncated internals: {len(words) - 1} of 128 words available")
    w = words
    print(f"header: count={bits(w[0],31,0)} STATESIZE={bits(w[0],63,32)}")
    s = lambda i: w[1+i]
    c1, c2, mpr, tmr = s(0), s(1), s(2), s(3)
    print(f"CPU: A={bits(c1,7,0):02X} X={bits(c1,15,8):02X} Y={bits(c1,23,16):02X} "
          f"SP={bits(c1,31,24):02X} P={bits(c1,39,32):02X} IR={bits(c1,47,40):02X} "
          f"STATE={bits(c1,52,48):02d} CS={bits(c1,53,53)}")
    print(f"     PC={bits(c2,15,0):04X} MPR_LAST={bits(c2,23,16):02X} O={bits(c2,31,24):02X} "
          f"IO_BUF={bits(c2,39,32):02X} GOT/RES/NMI/IRQ1/IRQ2/IRQT={bits(c2,40,40)}{bits(c2,41,41)}"
          f"{bits(c2,42,42)}{bits(c2,43,43)}{bits(c2,44,44)}{bits(c2,45,45)}")
    print(f"     MPR=" + " ".join(f"{bits(mpr,i*8+7,i*8):02X}" for i in range(8)))
    print(f"TIMER: VAL={bits(tmr,6,0):02X} LATCH={bits(tmr,14,8):02X} PRE={bits(tmr,25,16)} "
          f"EN={bits(tmr,26,26)} RELOAD={bits(tmr,27,27)} IRQ={bits(tmr,28,28)} "
          f"INT_MASK={bits(tmr,34,32):03b} PRE_MASK={bits(tmr,38,36):03b}")
    g = s(4)
    print(f"PSG: CHSEL={bits(g,2,0)} LMAL={bits(g,7,4):X} RMAL={bits(g,11,8):X} "
          f"L={bits(g,35,12):06X} R={bits(g,59,36):06X}")
    for ch in range(6):
        a, b = s(5+ch*3), s(5+ch*3+1)
        print(f"  ch{ch}: FREQ={bits(a,11,0):03X} DDA={bits(a,12,12)} ON={bits(a,13,13)} "
              f"AL={bits(a,20,16):02X} WF_ADDR={bits(a,44,40):02d} WF_CNT={bits(b,12,0):04X} "
              f"LFSR={bits(b,33,16):05X}")
    v1, v2 = s(23), s(24)
    print(f"VCE: CR={bits(v1,7,0):02X} DOTCLK={bits(v1,9,8)} BW={bits(v1,10,10)} "
          f"RAM_A={bits(v1,20,12):03X} CTRL={bits(v1,33,32)} DO={bits(v1,43,36):02X}")
    print(f"     H_CNT={bits(v2,11,0)} V_CNT={bits(v2,25,16)} END_LINE={bits(v2,41,32)} "
          f"CLKEN_CNT={bits(v2,46,44)} FS_CNT={bits(v2,50,48)} MULTIRES={bits(v2,55,55)}")
    for inst, base in (("VDC0",25),("VDC1",35)):
        regs = [bits(s(base+k), j*16+15, j*16) for k in range(5) for j in range(4)]
        core, hsk, dma, ras, tim = s(base+5), s(base+6), s(base+7), s(base+8), s(base+9)
        if inst == "VDC1" and all(r == 0 for r in regs) and core == 0:
            print(f"{inst}: (empty)"); continue
        print(f"{inst}: REGS " + " ".join(f"{r:04X}" for r in regs))
        print(f"     AR={bits(core,4,0):02d} VRR={bits(core,23,8):04X} "
              f"IRQ[dma,col,ovf,rcr,dmas,vbl]={bits(core,43,38):06b} SR={bits(core,50,44):02X} "
              f"BUSY={bits(core,52,52)}")
        print(f"     HSK: RD_P={bits(hsk,0,0)} WR_P={bits(hsk,1,1)} P2={bits(hsk,3,2):02b} "
              f"EX={bits(hsk,5,4):02b} VA={bits(hsk,21,6):04X} VD={bits(hsk,37,22):04X}")
        print(f"     DMA: pend={bits(dma,0,0)} exec={bits(dma,1,1)} DMAS pend={bits(dma,19,19)} "
              f"exec={bits(dma,20,20)} SATa={bits(dma,31,24):02X} VRAMa={bits(dma,47,32):04X}")
        print(f"     RASTER: DOT={bits(ras,2,0)} TILE={bits(ras,10,4)} DISP={bits(ras,25,16)} "
              f"RC={bits(ras,41,32)} BURST={bits(ras,52,52)} VDISP={bits(ras,53,53)}")
    vpc, top, ext = s(45), s(46), s(47)
    print(f"VPC: PRI0={bits(vpc,7,0):02X} PRI1={bits(vpc,15,8):02X} W1={bits(vpc,25,16):03X} "
          f"W2={bits(vpc,35,26):03X} VDCNUM={bits(vpc,36,36)} X={bits(vpc,49,40)}")
    print(f"TOP: rombank={bits(top,1,0)}  EXT: high_btn={bits(ext,0,0)} joy_port={bits(ext,3,1)} "
          f"latch={bits(ext,7,4):X} scan={bits(ext,11,8):X}")

def cd_registers(words):
    """Raw functional CD/SCSI savestate registers."""
    print("\nCD/SCSI registers (raw):")
    for index in range(53, 65):
        word = words[1 + index] if 1 + index < len(words) else 0
        print(f"  eReg {index:>3} {word:016X}")


def region_summary(data):
    if len(data) < 8:
        raise ValueError("truncated slot header")
    regions = regions_for_size(struct.unpack_from("<I", data, 4)[0])
    off = (1 + 128) * 8
    for name, size in regions:
        chunk = data[off:off+size]
        if len(chunk) != size:
            raise ValueError(f"region {name:8s} incomplete: {len(chunk)} of {size} bytes available")
        nz = sum(1 for b in chunk if b)
        print(f"region {name:8s} {size:6d}B  nonzero={nz}")
        off += size

def load(path):
    """Read a slot as 64-bit words, from a savestate file or a memory dump.

    Two shapes exist in practice: the binary blob (a slot file, or a raw copy of
    the slot's DDR window) and the text dump the remote helpers produce, one
    32-bit word per line as 0x........ - low half first.
    """
    raw = open(path, "rb").read()
    if raw[:2] in (b"0x", b"0X"):
        # Remote forensic dumps may end with an interrupted line; retain the
        # complete words so the register evidence remains readable.
        halves = []
        for token in raw.decode("ascii", "replace").split():
            try:
                halves.append(int(token, 16))
            except ValueError:
                continue
        words = [halves[i] | (halves[i + 1] << 32)
                 for i in range(0, len(halves) - 1, 2)]
        return words, None
    count = min(len(raw) // 8, 129)
    return list(struct.unpack(f"<{count}Q", raw[:count * 8])), raw


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    try:
        words, data = load(sys.argv[1])
        decode(words)
        if "--regs" in sys.argv:
            cd_registers(words)
        if data is not None:
            region_summary(data)
    except ValueError as error:
        raise SystemExit(f"invalid/incomplete snapshot: {error}")
