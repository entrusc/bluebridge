// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      0
#endif

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_PICO
#endif

#define CFG_TUSB_DEBUG        0
#define CFG_TUD_ENABLED       1
#define CFG_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN    __attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC           1
#define CFG_TUD_MSC           1
#define CFG_TUD_HID           0
#define CFG_TUD_MIDI          0
#define CFG_TUD_VENDOR        0

#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 2048
#define CFG_TUD_CDC_EP_BUFSIZE 64

// Up to 64 sectors per READ(10)/WRITE(10) to the drive. TinyUSB hands us
// min(this, bytes left in the host command), so a host command of up to
// 32 KB becomes a single drive command. Must stay <= 32768 (uint16 lengths
// inside TinyUSB).
#define CFG_TUD_MSC_EP_BUFSIZE 32768

#endif
