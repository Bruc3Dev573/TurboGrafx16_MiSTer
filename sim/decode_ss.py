#!/usr/bin/env python3
"""Decode a TurboGrafx16_MiSTer savestate blob (pce_savestates_pkg map).

Usage:
  decode_ss.py <blob.bin>            # raw dump of one slot (>= 25177*8 bytes)

A blob comes either from a slot file written by the core or from a dump of the
slot's DDR window; this script only reads bytes, it never talks to a board.

Blob layout (see the savestate documentation, section "slot layout"):
  word 0     : STATESIZE(63:32) | header_count(31:0)
  words 1-128 : internals = eReg slots 0..127
  words 129.. : memory regions (WRAM, VRAM0, SAT0, PALETTE, VRAM1, SAT1, PRAM, BRM, PSGWF)
"""
import sys, struct

REGIONS = [("WRAM",32768),("VRAM0",65536),("SAT0",512),("PALETTE",1024),
           ("VRAM1",65536),("SAT1",512),("PRAM",32768),("BRM",2048),("PSGWF",192),
           ("ADPCM",65536),("CDRAM",262144)]

def bits(v, hi, lo): return (v >> lo) & ((1 << (hi-lo+1)) - 1)

def decode(words):
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

def cd_timeline(words):
    """Decode the CD interface timeline that every savestate carries (eReg 68).

    This is what makes a blocked machine diagnosable from a plain save: the CD
    block keeps a second-resolution record of when a status last arrived, when
    a command was last sent, how long the drive kept streaming audio, and when
    the last restore happened. Comparing those four instants against each other
    says which side of the interface stopped talking, without any access to the
    board.
    """
    def s(i):
        return words[1 + i] if 1 + i < len(words) else 0

    dbg = s(68)
    if dbg == 0:
        print("\nCD timeline: not present (blob predates the forensic register)")
        return

    sec = bits(dbg, 15, 0)
    t_stat, t_comm = bits(dbg, 23, 16), bits(dbg, 31, 24)
    t_cdda, t_load = bits(dbg, 39, 32), bits(dbg, 47, 40)
    rstcnt, statcnt = bits(dbg, 55, 48), bits(dbg, 63, 56)

    # Each instant is stored as the low 8 bits of the second counter, so it is
    # only meaningful as a distance back from the save, and only within the last
    # 256 seconds. That is the window a hang lives in, so it is enough - but an
    # absolute "T+83s" would be a lie whenever the machine has been up longer.
    def ago(stamp):
        return (sec - stamp) & 0xFF

    def show(label, stamp, seen=None):
        # Second 0 is before the machine has run at all, so a zero stamp means
        # the event never happened rather than "at the very beginning".
        if stamp == 0:
            print(f"  {label:<28} never")
            return
        age = ago(stamp)
        note = f"   ({seen} in all)" if seen is not None else ""
        print(f"  {label:<28} {age:>3}s before the save{note}")

    print(f"\nCD timeline (eReg 68), machine up {sec}s, instants below are "
          f"distances back from the save (8-bit seconds, wraps at 256):")
    show("status from the drive", t_stat, f"{statcnt} arrived")
    show("command onto the bus", t_comm)
    show("CDDA sample streamed", t_cdda)
    show("restore", t_load)
    print(f"  {'drive resets by the game':<28} {rstcnt}")

    if t_load == 0:
        print("  -> no restore has happened yet: this is a source state, not a corpse")
        return
    if ago(t_stat) < ago(t_load):
        print("  -> a status did arrive after the last restore")
    else:
        print("  -> NO status arrived after the last restore: the drive went "
              "silent across the load")
    if ago(t_comm) > 3 and ago(t_stat) > 3:
        print(f"  -> both sides quiet for {min(ago(t_stat), ago(t_comm))}s before "
              f"the save: the interface was parked, not busy")


def cd_registers(words):
    """Raw CD/SCSI savestate registers, for when the timeline is not enough."""
    print("\nCD/SCSI registers (raw):")
    for index in range(53, 69):
        word = words[1 + index] if 1 + index < len(words) else 0
        print(f"  eReg {index:>3} {word:016X}")


def region_summary(data):
    off = (1 + 128) * 8
    for name, size in REGIONS:
        chunk = data[off:off+size]
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
        # A dump pulled over ssh can carry a truncated line where the remote
        # shell was interrupted; skip what is not a number rather than dying on
        # the one blob that was expensive to capture.
        halves = []
        for token in raw.decode("ascii", "replace").split():
            try:
                halves.append(int(token, 16))
            except ValueError:
                continue
        words = [halves[i] | (halves[i + 1] << 32)
                 for i in range(0, len(halves) - 1, 2)]
        return words, None
    count = min(len(raw) // 8, 25177)
    return list(struct.unpack(f"<{count}Q", raw[:count * 8])), raw


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    words, data = load(sys.argv[1])
    decode(words)
    cd_timeline(words)
    if "--regs" in sys.argv:
        cd_registers(words)
    if data is not None and len(data) >= 25177 * 8:
        region_summary(data)
