# Savestates for the TurboGrafx16 / PC Engine core

Save the machine at any instant and restore it later, identically — including
CD-ROM² and Super CD-ROM² titles, which is the part that took the work.

## Using them

| | |
|---|---|
| `Alt` + `F1`…`F4` | save to slot 1…4 |
| `F1`…`F4` | load slot 1…4 |
| savestate button + `Down` / `Up` | save / load with a pad |
| savestate button + `Left` / `Right` | change slot |

The slot can also be picked from the core's menu. The pad combination needs no
other modifier: `SELECT` is used in play by a good part of the library, and
`RUN` pauses the game, so neither is required here.

## What is in a slot

The slot is a flat blob: a header, then 128 internal registers, then the memory
regions in a fixed order.

| region | size |
|---|---|
| WRAM | 32 KB |
| VRAM0, VRAM1 | 64 KB each |
| SAT0, SAT1 | 512 B each |
| palette | 1 KB |
| PRAM (Populous) | 32 KB |
| backup RAM | 2 KB |
| PSG waveforms | 192 B |
| ADPCM RAM | 64 KB |
| CD work RAM | 256 KB |

`STATESIZE` is **derived** from that table rather than written by hand, so the
header can never drift from the layout it describes. On load it is compared
against the size the running core expects, and a slot that disagrees is
rejected rather than half-applied.

A save clears the slot header *before* writing the body. An aborted or torn
save therefore leaves a slot that reads as invalid, never a stale valid header
in front of an inconsistent body.

> The CD work RAM is 256 KB of the blob, and the size generic that carries it
> must be bound **by name**. A positional binding silently substitutes it, the
> region vanishes from the blob, and everything still appears to work in
> simulation while failing on hardware. `sim/lint_sizes.py` prints the expected
> `STATESIZE`; comparing it against a real slot header costs a second.

## How a save is taken

The machine is paused at an instruction boundary and left to settle, then a
walk engine streams every region out to DDR through a single port, eight bytes
at a time, handshaking with the memory controller for each word.

That handshake is not a formality. An early version sampled the data bus a
fixed number of cycles after the request; when DDR was still busy with the
blob's own write, byte lane 0 of every eighth word came back holding the
previous value. The corruption was perfectly deterministic, so it survived
every blob-against-blob comparison, was invisible in simulation — where the
model always answered in time — and only showed up as the restored program
executing an operand as an opcode.

## How a CD game is restored

The drive is not inside the FPGA. It is emulated on the ARM side, outside the
savestate, with a single pending-status slot and emulated seek latency. It
cannot be frozen and thawed with the rest of the machine.

So a restore does not try. It **replays** the conversation instead: the audio
command pair is re-issued with the saved position, and the real drive is put
back into the state the game believes it is in. Everything below exists because
that replay meets a machine that has kept running in the meantime.

### The valves

| valve | what it does | why it exists |
|---|---|---|
| phase-starvation abort | drops a SCSI phase parked longer than ~391 ms, armed only while a savestate is pending | a phase left parked across the boundary never completes on its own |
| tail-drain flush | flushes the read FIFO when a save lands in the tail window | otherwise the first post-load pop is claimed twice and a phantom byte appears, and the BIOS retries forever |
| status serialisation | the replay waits rather than overlapping its own status with the game's | the ARM side holds one status at a time; the second overwrites the first |
| open-audio veto | refuses to quiesce while an audio command has been accepted but its status has not arrived | that window is unsaveable by construction; the veto ages out so a lost status cannot block saving forever |
| rescue retry | after 3.1 s of total CD silence following a load, re-issues the audio pair once | covers the case where the drive stopped and the game is waiting for an end-of-track event that can now never come |
| loop re-issue | at the wrap of a track resumed mid-way, re-issues the pair with the original start | otherwise the drive loops back to the resume point and the music repeats its own tail forever |

The rescue pass deliberately swallows nothing: after several seconds of proven
starvation every status is a lifeline and can collide with nothing.

## Forensics

Register 68 records, at one-second resolution, when a status last arrived from
the drive, when a command last went out, how long audio kept streaming, and
when the last restore happened. It costs nothing and stays in the build, so any
future hang documents itself: take a savestate while the machine is stuck and

    python sim/decode_ss.py <slot file>

prints the timeline and what it implies — including whether any status arrived
at all after the restore, which is usually the whole diagnosis.

Registers 47, 65, 66 and 67 carry boundary and walk instrumentation from
earlier investigations and are equally free.

## Verifying a change

`sim/run_tb_cd.sh` runs the CD scenario matrix; `run_tb_cd_w.sh`, `_w2` and
`_w3` add the push-based and drive-faithful models of the ARM side, which is
where the hard defects show. `run_tb_savestates.sh` covers the walk itself.
The benches are self-contained: no BIOS image and no game data is needed.

A green matrix is necessary and not sufficient. Three of the defects fixed here
were invisible in simulation until the model was corrected, and one of them was
a property of the real drive that no testbench had ever represented.

## Known limits

- The blob carries no content checksum yet. Structural corruption is caught by
  the size check, and a torn save by the header rule, but silent corruption of
  the payload would not be. A checksum is the next change, and it will change
  the slot format.
- The rescue valve covers starvation with an outstanding interrupt-mode audio
  event. A post-load stall of a different shape is not covered by design; if a
  title hangs for more than ten seconds without recovering, a savestate taken
  during the hang classifies it (see above).
- Coverage is uneven: the CD path, the walk and the CPU have hardware evidence,
  while the PSG, the Arcade Card and the peripherals have none recorded.
- The slot format is not stable across versions yet. Assume savestates do not
  survive an upgrade.
