// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger

#include "wdt.h"

#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/watchdog.h"

#define WDT_TIMEOUT_MS 2000

static const char *reason = "unknown";

void wdt_init(void) {
    // Must be read before watchdog_enable(), which rewrites the scratch
    // registers watchdog_enable_caused_reboot() looks at.
    uint32_t chip = vreg_and_chip_reset_hw->chip_reset;

    if (watchdog_enable_caused_reboot())
        reason = "WATCHDOG (firmware stopped responding)";
    else if (watchdog_caused_reboot())
        reason = "software reboot (e.g. after flashing)";
    else if (chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS)
        reason = "power-on / brown-out";
    else if (chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS)
        reason = "RUN pin";
    else if (chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS)
        reason = "debugger";

    watchdog_enable(WDT_TIMEOUT_MS, true);   // paused while a debugger halts us
}

void wdt_feed(void) {
    watchdog_update();
}

const char *wdt_reset_reason(void) {
    return reason;
}
