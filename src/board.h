// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// Pin map and board constants.
#pragma once

#define PIN_D0          0   // D0-D7 on GP0-GP7
#define PIN_NSTROBE     8
#define PIN_NAUTOFD     9
#define PIN_NINIT       10
#define PIN_NSELECTIN   11
#define PIN_DATA_DIR    12  // U2 DIR: 1 = drive->Pico, 0 = Pico->drive
#define PIN_NACK        13
#define PIN_BUSY        14
#define PIN_PERROR      15
#define PIN_SELECT      16
#define PIN_NFAULT      17
#define PIN_NOE         18  // U2 OE#: 1 = isolated
#define PIN_LED         19  // active high

#define LPT_DATA_MASK   (0xffu << PIN_D0)
#define LPT_CTL_MASK    ((1u << PIN_NSTROBE) | (1u << PIN_NAUTOFD) | \
                         (1u << PIN_NINIT) | (1u << PIN_NSELECTIN))

// Settle time after every emulated port access. ppa.c was written against
// ISA inb/outb which take ~1us each. The control/handshake sequences keep
// that; the 512-byte data bursts get their own (tunable) value, since ppa.c
// notes the transfer loop tolerates faster I/O than the rest.
#define LPT_DEFAULT_DELAY_NS      1000
// Found with 'dtune': reads verify down to 0 ns (the loop overhead alone is
// enough for the drive); 100 ns keeps a margin at ~96% of the top speed.
#define LPT_DEFAULT_DATA_DELAY_NS 100
