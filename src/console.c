// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger

#include "console.h"

#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "tusb.h"
#include "wdt.h"

static bool in_task;

void usb_task(void) {
    wdt_feed();
    if (in_task)
        return;
    in_task = true;
    tud_task();
    in_task = false;
}

static void cdc_out_chars(const char *buf, int len) {
    if (!tud_cdc_connected())
        return;

    absolute_time_t tmo = make_timeout_time_ms(500);
    while (len > 0) {
        uint32_t n = tud_cdc_write(buf, (uint32_t)len);
        buf += n;
        len -= (int)n;
        if (len > 0) {
            tud_cdc_write_flush();
            // Inside a USB callback the FIFO cannot drain: drop the rest.
            if (in_task || time_reached(tmo))
                break;
            usb_task();
        }
    }
    tud_cdc_write_flush();
}

static stdio_driver_t cdc_stdio = {
    .out_chars = cdc_out_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = true,
#endif
};

void console_init(void) {
    stdio_set_driver_enabled(&cdc_stdio, true);
}
