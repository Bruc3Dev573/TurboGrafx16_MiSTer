# Savestates

This source adds four savestate slots for HuCard and CD games. CD savestates
require the matching companion Main: it pauses sector delivery while the core
is frozen and binds SD slots to the disc. HuCard savestates do not require it.
Back up battery saves separately.

## Install and use

1. Copy the built core `.rbf` to the SD card's `_Console` directory under a
   distinct filename, keeping your usual core available.
2. For CD savestates, back up the original SD-root `MiSTer` executable and
   your `savestates`, `saves` and relevant `config` files outside the card.
   Shut MiSTer down and power it off before replacing `MiSTer` with the companion
   executable (see Source and build) using a computer. Restore the original the
   same way to roll back.
3. Boot MiSTer, load the core and select your game or disc and BIOS. Leave MB128
   disabled. Choose the required hardware options before starting the game.
4. Set `Savestates to SDCard` to `On` for persistent slots. With `Off`, new
   saves stay in memory until another game is loaded, and slots already on SD
   are still loaded with the game. A system updater can replace the companion
   Main; reinstall the matching version before using CD savestates.

|Action|Keyboard|
|---|---|
|Save slot 1-4|Alt + F1-F4|
|Load slot 1-4|F1-F4|

The core OSD also provides slot selection and save/restore actions. Hold the
mapped `Savestates` controller button and press Down to save, Up to load, or
Left/Right to change slot.

## Limits

- Restore with the same game/disc, BIOS and compatible hardware settings.
  Slot compatibility between core versions is not guaranteed.
- MB128 state is not captured. Enabling MB128 blocks savestates until the core
  is reloaded, even if the option is disabled again. Reload with MB128 disabled.
- Arcade Card RAM and registers are included when the game has used the card.
- There is no payload CRC or power-loss guarantee. Interrupted or failed writes
  can lose a slot; a save request or existing file does not prove completion.
- A zero change counter is valid after rollover, including a save following a
  Main-prefetched SD load. It does not by itself indicate an invalid slot.
- Writing a slot to SD pauses Main briefly. The first save after a CD track
  starts, or after a load, can drop a few tens of milliseconds of CD audio, and
  a later save whose SD write takes longer than the earlier ones can drop more
  (seen once as a half-second gap). The picture is not affected and restoring
  those slots is clean. With `Savestates to SDCard` set to `Off` nothing is
  written to SD.
- Without the companion Main, CD slots are not tied to the disc or saved under
  its name. After a HuCard in the same session, CD saves overwrite that
  HuCard's SD slots, and loading a slot left from another game can hang the
  machine. CD music also runs ahead of the picture after each save.

## Design

The core freezes the whole machine at a safe boundary and walks its state
into a DDR3 scratch area. Main mirrors that area to the SD card. A load
writes the state back and then resynchronizes the CD drive. Main emulates
the drive and cannot see the load.

### Requests

`TurboGrafx16.sv` declares four slots of 3 MiB at DDR3 `0x3E000000` with the
`SS3E000000:300000` config-string token. Main already serves that token for
other cores. `rtl/savestate_ui.sv` turns keyboard, pad and OSD actions into
save and load pulses. It reports the completion of the operation that ran,
not of the last key pressed. `rtl/statemanager.vhd` keeps one pending request
per type together with its slot. It dispatches saves before loads and holds
the slot address stable until the engine is busy. The UI is offered only with
a game or disc loaded and no backup RAM transfer in flight. Enabling MB128
vetoes requests at the engine. The veto stays latched until the next
cartridge or core load, because the MB128 RAM is not captured.

### Freeze

`rtl/savestates.vhd` comes from the NES core's savestate framework by Robert
Peip. It asks `rtl/pce_top.vhd` to sleep. The top waits for a boundary: CPU
between instructions with no reset pending, `CPU_CE` low, VBLANK, both VDC
DMA engines idle, and neither the external RAM bridge nor the CD unit holding.
Posted VDC transactions do not need to drain. Their address, data and
handshake state are saved and they complete after the restore. At the
boundary `SLEEP` gates every clock enable (CPU, PSG, VCE, VDCs, ADPCM, CD
unit). The machine stops on one edge and restarts on one edge. The freeze is
forced if no boundary arrives within 2^27 clocks. A forced load proceeds. A
forced save is abandoned before the slot is touched.

### Format

A slot holds one 64-bit header (`STATESIZE` and a change counter), 128
64-bit registers transferred over the savestate bus, and twelve byte regions
walked through spare RAM ports: WRAM, VRAM0, SAT0, palette, VRAM1, SAT1, PRAM,
backup RAM, PSG waveforms, ADPCM DRAM, the 256 KiB CD work RAM and the 2 MiB
Arcade RAM. The Arcade RAM is written only when the game has used the card.
The two sizes are `0x20532` and `0xA0532` DWORDs. A load accepts nothing else.
It picks the format from the header, not from the running game, and it
restores the Arcade flag with it. The save clears the header before the first
write and rewrites it after the last write, so an interrupted save leaves a
slot that the load rejects. The save skips the counter word when
`Savestates to SDCard` is off, and Main then does not copy the slot.

`rtl/bus_savestates.vhd` gives every module a shadow register per address.
The live value is read during a save. The loaded value is parked and the
module applies it on `load_done`, so the restore takes effect on one clock
after every register has arrived. `rtl/pce_savestates_pkg.vhd` is the map:
CPU 0-3, PSG 4-22, VCE 23-24, VDC0 25-34, VDC1 35-44, VPC 45, mapper 46,
input 47, Arcade Card 48-52, CD 53-64. Higher addresses are diagnostics.

### Saved and rebuilt state

Each module saves the state a program can observe and rebuilds its transients
after the restore. The CPU saves registers, MPRs, sequencer state, interrupt
latches, timer and I/O ports. Its data register is fetched again while the
top keeps the freeze. The PSG saves every channel counter, LFSR and LFO. The
mixer restarts. The VDCs save registers, pending CPU accesses, DMA and raster
counters. Sprite caches and line buffers are rebuilt from VRAM. The VCE saves
the phase of its clock-enable dividers so the VDCs restart in step.

RAM regions use existing second ports (WRAM, VRAM, PRAM, palette, SAT,
waveforms) or borrow the CPU port while frozen (backup RAM). CD work RAM and
Arcade RAM live in SDRAM behind one port. `rtl/ss_external_ram.vhd` drives
that port byte by byte from the walk, only while frozen, and paces the engine
with `Save_RAMRdy`. `rtl/ddram.sv` adds a direct 64-bit channel to DDR3 for
the slot itself, outside the ROM cache.

### CD

`rtl/cd/SCSI.vhd` saves the SCSI phase, the command in progress, the audio
range and the number of CDDA samples consumed since the play command. The
CDDA and subcode FIFOs are not saved. After a load SCSI replays the drive
state to Main through the ordinary mailbox, one command per status. It resets
the drive, runs the TOC queries it needs, restarts play at the saved position
(sector address plus a sample skip within the sector, loop end restored),
reissues any open READ6 and drains the bytes the game had already consumed,
then reissues any pending query. It discards the statuses the replay itself
produces and delivers the ones the game still waits for. Replayed play
commands set the fast-seek bit, except when the game was itself waiting
through that seek. The CD unit holds the whole machine asleep until the saved
sample heads the CDDA FIFO, so audio and picture resume on the same clock. A
2^26-clock watchdog releases the hold.

`rtl/hps_ext.v` shows the freeze to Main as bit 8 of the `CD_GET` reply. The
companion Main stops its timed sector delivery while the bit is set, keeps
serving the mailbox, and resumes on the schedule it had. The stock Main keeps
delivering: sectors sent into a frozen core are dropped and CD audio runs
ahead of the picture after a save.

### Changing the core later

A change to the core touches the savestate only when it adds state that
outlives an instruction. Three cases cover it.

**A new register in a chip that already has a savestate slot.** Add the
field to that chip's `SS_*_BACK` word for the save, and to the
`SaveStateBus_load` branch of the process that owns the register for the
restore. `rtl/huc6202.vhd` is the smallest example: one eReg, one
`SS_VPC_BACK` assignment per field, one load branch. The packing comments in
`rtl/pce_savestates_pkg.vhd` say which bits are free in each word.

**A new register with no room left.** Instantiate another `eReg_SavestateV`
with the next free address. Addresses 0-64 are the map above, 65-79 hold
diagnostics, 80-127 are free. Add the index to `rtl/pce_savestates_pkg.vhd`.
The engine walks all 128 words, so nothing else changes.

**A new RAM.** Give it a second port or a mux like the ones in
`rtl/pce_top.vhd`, add a row to the `savetypes` table in
`rtl/savestates.vhd` and a case to the read mux at the end of `pce_top.vhd`.
The slot format changes with it: `STATESIZE` is derived from the table, and
old slots are rejected by size.

State that can be rebuilt (pipelines, caches, line buffers) does not need a
field. Clear it on `reset_ss` and let it refill before the machine wakes, as
the VDC sprite evaluator does.

`bash sim/run_tb_core.sh` catches a forgotten register: it saves, runs on,
saves again, then loads the first slot, runs the same span and compares the
two later slots word by word. A field that is not restored shows up as a
differing word. `bash sim/run_tb_cd.sh` does the same for the CD path.

## Source and build

Build from the repository root with Quartus Prime Lite 17.0.2:

```sh
quartus_sh --flow compile TurboGrafx16
```

Generated build-date text can affect the binary; bit-identical builds are not
promised. Preserve source licenses and attributions when redistributing the
core or Main. BIOS and game images are not included.

The companion Main is tag `tg16-savestates-v0.2.0-pre1` of
https://github.com/Bruc3Dev573/Main_MiSTer: MiSTer Main commit
`f6a3caa601c22fe1c34647b48ecbff6b9c7ddc09` plus three commits, all in
`support/pcecd/pcecd.cpp`. While the core is frozen it sets bit 8 of the CD_GET
reply; this Main then pauses its timed sector delivery, keeps serving the
command mailbox, and resumes on its pre-freeze schedule. It also loads and
writes CD slots on SD under the disc's name when a CD is mounted.

## Standalone simulations

Run from the repository root with Bash and GHDL (mcode). The Arcade register
check also needs Verilator, GNU `timeout` and a C++ toolchain; the UI check
uses Icarus Verilog. The benches stub the Altera memories and use synthetic
programs and disc models: no BIOS or game image is needed.

|Command|Coverage|
|---|---|
|`bash sim/analyze_all.sh`|GHDL analysis of the whole VHDL tree|
|`bash sim/run_tb.sh`|Engine roundtrip, refusal, queued requests, counter rollover, SD option|
|`bash sim/run_statemanager.sh`|Request queues, slot capture, acknowledgement and cancellation|
|`bash sim/run_ssui.sh`|Keyboard modifiers, slot selection and completion notices|
|`bash sim/run_tb_brm.sh`|Backup RAM roundtrip|
|`bash sim/run_tb_adpcm.sh`|ADPCM unit: divider phase and first-sample timing|
|`bash sim/run_tb_core.sh`|Whole core: determinism after a load, Main pause modelled|
|`bash sim/run_tb_cd.sh`|CD subsystem save/load matrix (hours)|
|`bash sim/run_tb_cd_fast.sh`|CDDA sample-exact resume, loop wrap and backup RAM only (minutes)|
|`bash sim/run_tb_cd_w.sh`, `_w2.sh`, `_w3.sh`|Save/load inside streaming READ6 on push-status and Main-accurate drive models|
|`bash sim/run_cd_subcode_resume.sh`|CDDA/subcode resume, delayed FIFO data, in-flight burst, persistent backlog|
|`bash sim/run_cd_audio_hold.sh`|Post-load audio hold: resume, abandon, no hold, watchdog, fast-seek bit|
|`bash sim/run_arcade_memory.sh`|Arcade RAM roundtrip through the engine and shared bridge|
|`bash sim/run_arcade_snapshot.sh`|Arcade registers with the production savestate registers|

These are source-level checks, not a substitute for hardware testing.
