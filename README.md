# Bluebridge

Use an Iomega ZIP 100 **parallel-port** drive on any modern computer over USB.

## What it is

The ZIP 100 parallel drive (Z100P) was built for the PC printer port, which
modern machines no longer have, and USB-to-parallel adapters only handle
printers. Bluebridge is a small board built around a Raspberry Pi Pico W that sits
between the drive and a USB port. The firmware in this repository emulates a PC
parallel port in software and talks to the drive with the same protocol as the
Linux `ppa` driver. To the host it is an ordinary USB mass storage device:
insert a disk and it appears as a removable drive, with no drivers needed on
Windows, macOS or Linux. Disk changes, write protection and eject are passed
through. A second USB interface provides a serial console for diagnostics and
low-level testing.

### The hardware

```
Host PC  <--USB-->  Pico W  <--3.3 V-->  3x SN74LVC4245A  <--5 V-->  DB25  <--cable-->  ZIP 100 (Z100P)
```

- **Raspberry Pi Pico W (RP2040)** runs the firmware. The wireless chip is not
  used.
- **Three SN74LVC4245A level shifters** translate between the Pico's 3.3 V
  GPIOs and the 5 V parallel port: U2 for the bidirectional data lines, U3 for
  the status lines (drive to Pico) and U4 for the control lines (Pico to drive).
- **A DB25 connector** takes a standard parallel cable to the drive.
- **An activity LED** stays lit while a disk is inserted and blinks during
  transfers.

The board is powered over USB. The drive keeps its own power supply, and the
two share ground through DB25 pins 18–25. The drive is run in PS/2 (8-bit
bidirectional) mode; nibble mode and EPP are not implemented.

## Performance

About 375 KB/s sustained read, slightly faster than Iomega's rating for the
parallel drive (20 MB/min, about 0.33 MB/s). The parallel link is the
bottleneck, not USB.

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
(tested with 2.1.1), CMake, Ninja and an `arm-none-eabi` toolchain.

```
export PICO_SDK_PATH=~/pico/pico-sdk
cmake -B build -G Ninja                      # PICO_BOARD defaults to pico_w
ninja -C build
```

## Flashing

Hold BOOTSEL while plugging the Pico in, then copy `build/bluebridge.uf2` to the
`RPI-RP2` drive that appears. After that, the console command `bootsel`
reboots into the bootloader without replugging.

## Usage

Connect the drive to the board, power the drive on, then plug the board into
the host. The ZIP disk shows up as a removable USB drive.

The board also provides a USB serial console (CDC ACM), any baud rate:

```
picocom /dev/ttyACM0
```

Type `help` for the full command list. The main groups:

| Commands | Purpose |
|---|---|
| `init`, `inq [len]`, `tur`, `sense`, `cap` | Basic SCSI commands to the drive. `inq 255` also shows the drive's firmware date |
| `read <lba>`, `wtest <lba> yes` | Hexdump a sector; write/verify/restore one sector |
| `bench`, `sum`, `dtune`, `cache` | Read throughput, CRC of a disk area, data-delay sweep, read-ahead statistics |
| `eject`, `msc on\|off` | Eject the disk; enable or disable USB mass storage access |
| `pins`, `ctr`, `dtr`, `str`, `rdtr`, `dir`, `oe`, `walk` | Raw GPIO and parallel-port register access |
| `connect`, `disconnect`, `rpulse`, `reset`, `echo`, `trace` | Low-level transport steps, cable check, bus trace |
| `delay` | Per-access I/O delay for control and data cycles |
| `bootsel` | Reboot into the USB bootloader |

Raw register commands pause USB mass storage access so the host's polling
can't interleave with manual pokes. Resume it with `msc on`.

The USB VID/PID (`0xCafe:0x5a10`) are development test IDs, not registered
ones.

## Source layout

| File | Role |
|---|---|
| `src/board.h` | Pin map and timing constants |
| `src/lpt.c` | PC parallel-port register emulation (`w_ctr`, `r_str`, `w_dtr`, `r_dtr`), signal inversions, bus direction, trace buffer |
| `src/ppa.c` | Port of Linux `drivers/scsi/ppa.c`, PS/2 mode only, same function names |
| `src/zip.c` | SCSI layer: auto REQUEST SENSE, capacity/write-protect cache, media-change handling |
| `src/engine.c` | Core1 worker. The only code that touches the parallel port after boot |
| `src/msc.c` | TinyUSB mass storage callbacks, 32 KB drive commands, read-ahead cache |
| `src/shell.c` | Diagnostic console |
| `src/wdt.c` | Hardware watchdog and reset-cause reporting |
| `src/usb_descriptors.c` | Composite USB device: CDC console + MSC |

## Hardware details

### Pin map

| GPIO | Signal | Direction | Notes |
|---|---|---|---|
| GP0–GP7 | D0–D7 | bidirectional | through U2 |
| GP8 | nStrobe | out | |
| GP9 | nAutoFd | out | per-byte handshake line |
| GP10 | nInit | out | |
| GP11 | nSelectIn | out | |
| GP12 | DATA_DIR | out | U2 `DIR`: 1 = drive → Pico, 0 = Pico → drive |
| GP13 | nAck | in | |
| GP14 | Busy | in | |
| GP15 | PError | in | |
| GP16 | Select | in | |
| GP17 | nFault | in | |
| GP18 | nOE | out | U2 `OE#`: 1 = data buffer isolated |
| GP19 | LED | out | active high |

Pull-ups hold DATA_DIR and nOE high while the RP2040 is in reset, so the data
buffer stays isolated until the firmware has put every line in a known state.

### Register emulation

A PC's parallel port hardware inverts nStrobe, nAutoFd and nSelectIn in the
control register and Busy in the status register. This board's buffers don't,
so `src/lpt.c` applies those inversions in software. Control register bit 5,
the PC's data-direction bit, drives DATA_DIR directly. With that in place,
every register value from the Linux driver can be used unchanged.

### Implementation notes

- **Direction-flip order.** Going inbound: pads to input, then DATA_DIR = 1,
  then control lines. Going outbound: control lines first, then DATA_DIR = 0,
  then pads to output. This way the Pico and U2 never drive the Pico-side data
  lines at the same time.
- **Settle time per register access.** `ppa.c` relies on ISA `inb`/`outb`
  taking about 1 µs each. Control sequences and every bus turnaround keep
  1000 ns (`delay ctl <ns>`). The 512-byte data bursts use 100 ns
  (`delay data <ns>`), found with `dtune`: reads verified down to 0 ns, and
  100 ns keeps a margin.
- **Cable check.** After connect, nFault must follow nAutoFd (ctr 0x0e gives
  nFault = 1, 0x0c gives nFault = 0). The console command `echo` runs this test.
- **Target selection.** The SCSI ID is found by scanning 0–7. A ZIP drive is
  normally ID 6.
- **Watchdog.** If the drive back-powers the board through the cable while USB
  is unplugged, the RP2040 can start or hang at low voltage. A hardware
  watchdog restarts it, and the console banner reports the last reset cause.

## Bring-up and troubleshooting

Step-by-step checks for a freshly built board:

| Step | How | Pass |
|---|---|---|
| 1 | Power up | LED lights briefly at boot |
| 2 | `picocom /dev/ttyACM0` | Banner, `pins` works |
| 3 | Drive **disconnected**: `walk 0`, `walk 1`, `walk 2`, `walk 3` | Multimeter at DB25: pin 1 nStrobe, 14 nAutoFd, 16 nInit, 17 nSelectIn toggle at 1 Hz. `ctr 04` = all high |
| 4 | `dir out` + `dtr 55` / `dtr aa`, `dir in`, `oe 0`/`oe 1` | Data pins 2–9 follow in `out`; U2 isolated with `oe 0` |
| 5 | Drive attached + powered: `echo` | `cable check PASS` |
| 6 | `init` | `drive at SCSI ID 6` |
| 7 | `inq` | `IOMEGA  ZIP 100` |
| 8 | `cap`, `read 0` | ~196608 blocks × 512; signature `55aa` |
| 9 | `wtest <lba> yes` on a scratch disk | Pattern readback OK, original restored OK |
| 10 | `msc on`, then use from the host | `lsblk`, mount, `dd \| md5sum` twice, eject |
| 11 | `bench`, `dtune`, `sum`, `cache` | ~375 KB/s read |

If the drive doesn't answer:

1. Check the inversions in `src/lpt.c` against your wiring. A wrong inversion
   looks exactly like a protocol bug.
2. Ring out the cable. A missing data line or a broken ground shows up as a
   protocol failure.
3. Run `trace on`, repeat the command, then `trace dump`, and compare the
   register sequence against `ppa.c` line by line.

## Licence

Bluebridge is free software, licensed under the GNU General Public License
version 2. See [LICENSE](LICENSE).

- Original Bluebridge sources are GPL-2.0-or-later.
- `src/ppa.c` and `src/ppa.h` are derived from the Linux kernel's
  `drivers/scsi/ppa.c` and are GPL-2.0-only. Because of them, the firmware
  as a whole is distributed under GPL-2.0.
- `pico_sdk_import.cmake` is copied from the Raspberry Pi Pico SDK
  (BSD-3-Clause).

Each source file states its license with an SPDX identifier.
