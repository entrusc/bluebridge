// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// Hardware watchdog and reset-cause reporting.
//
// Mainly a guard against the back-powering problem: if the drive half-powers
// the board while USB is unplugged, the RP2040 may start (or crash) at low
// voltage. The watchdog restarts it once it stops running normally.
#pragma once

void wdt_init(void);   // record the reset cause, then start the watchdog
void wdt_feed(void);   // call from every place core0 can wait for long
const char *wdt_reset_reason(void);
