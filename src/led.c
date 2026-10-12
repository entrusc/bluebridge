// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger

#include "led.h"

#include "board.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#define BLINK_HOLD_US   150000   // keep blinking this long after the last transfer
#define BLINK_PERIOD_US 60000

static bool media;
static uint64_t last_activity;

void led_init(void) {
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);   // lit during the boot probe, then led_task takes over
}

void led_media(bool present) {
    media = present;
}

void led_activity(void) {
    last_activity = time_us_64();
    led_task();
}

void led_task(void) {
    uint64_t now = time_us_64();
    bool on = media;
    if (last_activity && now - last_activity < BLINK_HOLD_US)
        on = (now / BLINK_PERIOD_US) & 1;
    gpio_put(PIN_LED, on);
}
