// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// Bluebridge: Iomega ZIP 100 parallel drive as a USB mass storage device.

#include "console.h"
#include "engine.h"
#include "led.h"
#include "lpt.h"
#include "msc.h"
#include "shell.h"
#include "tusb.h"
#include "wdt.h"

int main(void) {
    // First thing: put the parallel port into its safe idle state.
    lpt_init();
    wdt_init();
    led_init();

    // Core1 owns the parallel port from here on.
    engine_init();

    msc_init(2000);
    tud_init(BOARD_TUD_RHPORT);
    console_init();

    for (;;) {
        usb_task();
        shell_task();
        led_task();
    }
}
