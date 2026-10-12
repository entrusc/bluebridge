// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// PC parallel port register emulation.
//
// The PC's LPT hardware inverts nStrobe, nAutoFeed, nSelectIn and Busy; this
// board's buffers don't, so the inversions happen here. With that done every
// register value in ppa.c can be used verbatim.

#include "lpt.h"

#include "board.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/time.h"

static uint8_t ctr_shadow;
static bool inbound;
static bool oe_enabled;
// The shell changes these from core0 while core1 transfers; each is a
// single aligned word and io_delay() picks one per access, so a change just
// takes effect with the next access.
static volatile uint32_t delay_ns, data_delay_ns;
static volatile uint32_t ctl_cycles, data_cycles;
static bool data_mode;   // core1 only

#define TRACE_N 2048
static lpt_trace_t tbuf[TRACE_N];
static uint32_t thead, tcount;
static bool trace_on;

static inline void io_delay(void) {
    uint32_t c = data_mode ? data_cycles : ctl_cycles;
    if (c)
        busy_wait_at_least_cycles(c);
}

// Bus turnaround always gets the safe delay: the drive must have released
// the data lines before U2 starts driving them.
static inline void turnaround_delay(void) {
    uint32_t c = ctl_cycles;
    if (c)
        busy_wait_at_least_cycles(c);
}

static uint32_t ns_to_cycles(uint32_t ns) {
    return (uint32_t)(((uint64_t)ns * clock_get_hz(clk_sys) + 999999999u) / 1000000000u);
}

static void trace(char op, uint8_t v) {
    if (!trace_on)
        return;
    if (tcount && (op == 'S' || op == 'R')) {
        lpt_trace_t *last = &tbuf[(thead + TRACE_N - 1) % TRACE_N];
        if (last->op == op && last->val == v && last->rep < 0xffff) {
            last->rep++;
            return;
        }
    }
    tbuf[thead] = (lpt_trace_t){ .t_us = time_us_32(), .rep = 1, .op = op, .val = v };
    thead = (thead + 1) % TRACE_N;
    if (tcount < TRACE_N)
        tcount++;
}

static inline uint32_t ctl_pins(uint8_t v) {
    uint32_t o = 0;
    if (!(v & 0x01)) o |= 1u << PIN_NSTROBE;     // nStrobe    inverted
    if (!(v & 0x02)) o |= 1u << PIN_NAUTOFD;     // nAutoFeed  inverted
    if (v & 0x04)    o |= 1u << PIN_NINIT;       // nInit      not inverted
    if (!(v & 0x08)) o |= 1u << PIN_NSELECTIN;   // nSelectIn  inverted
    return o;
}

void lpt_init(void) {
    // Order matters: nothing may drive the cable until the data
    // buffer direction and the control lines are in a known state.

    // 1. D0-D7 as inputs
    for (uint p = PIN_D0; p < PIN_D0 + 8; p++) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_disable_pulls(p);
    }

    // 2. DATA_DIR high (inbound). N_OE is also taken over high so it keeps
    //    the level its pull-up gave it until step 4.
    gpio_init(PIN_DATA_DIR);
    gpio_put(PIN_DATA_DIR, 1);
    gpio_set_dir(PIN_DATA_DIR, GPIO_OUT);
    gpio_init(PIN_NOE);
    gpio_put(PIN_NOE, 1);
    gpio_set_dir(PIN_NOE, GPIO_OUT);

    // 3. control lines idle high (deasserted)
    const uint ctl[] = { PIN_NSTROBE, PIN_NAUTOFD, PIN_NINIT, PIN_NSELECTIN };
    for (uint i = 0; i < 4; i++) {
        gpio_init(ctl[i]);
        gpio_put(ctl[i], 1);
        gpio_set_dir(ctl[i], GPIO_OUT);
    }

    // status inputs
    const uint st[] = { PIN_NACK, PIN_BUSY, PIN_PERROR, PIN_SELECT, PIN_NFAULT };
    for (uint i = 0; i < 5; i++) {
        gpio_init(st[i]);
        gpio_set_dir(st[i], GPIO_IN);
        gpio_disable_pulls(st[i]);
    }

    // The register view of the state above: all lines idle, bit 5 = input.
    ctr_shadow = 0x24;
    inbound = true;
    gpio_put_masked(LPT_DATA_MASK, 0);   // output latch, not driven yet

    lpt_set_delay_ns(LPT_DEFAULT_DELAY_NS);
    lpt_set_data_delay_ns(LPT_DEFAULT_DATA_DELAY_NS);

    // 4. only now enable the data buffer
    lpt_set_oe(true);
}

void w_ctr(uint8_t v) {
    bool want_in = (v & 0x20) != 0;
    uint32_t ctl = ctl_pins(v);

    if (want_in && !inbound) {
        // Stop driving the B side before U2 starts driving it, then let
        // the control lines tell the drive it may talk.
        gpio_set_dir_in_masked(LPT_DATA_MASK);
        gpio_put(PIN_DATA_DIR, 1);
        inbound = true;
        gpio_put_masked(LPT_CTL_MASK, ctl);
    } else if (!want_in && inbound) {
        // Tell the drive to release the bus first, then turn U2 around and
        // drive the latched data.
        gpio_put_masked(LPT_CTL_MASK, ctl);
        turnaround_delay();
        gpio_put(PIN_DATA_DIR, 0);
        gpio_set_dir_out_masked(LPT_DATA_MASK);
        inbound = false;
    } else {
        gpio_put_masked(LPT_CTL_MASK, ctl);
    }

    ctr_shadow = v;
    trace('C', v);
    io_delay();
}

void w_dtr(uint8_t v) {
    // Like a PC: the latch is always written, it only reaches the cable
    // while the port is in output mode.
    gpio_put_masked(LPT_DATA_MASK, (uint32_t)v << PIN_D0);
    trace('D', v);
    io_delay();
}

uint8_t r_str(void) {
    uint32_t g = gpio_get_all();
    uint8_t s = 0;
    if (!(g & (1u << PIN_BUSY)))  s |= 0x80;   // Busy   inverted
    if (g & (1u << PIN_NACK))     s |= 0x40;   // nAck
    if (g & (1u << PIN_PERROR))   s |= 0x20;   // PError
    if (g & (1u << PIN_SELECT))   s |= 0x10;   // Select
    if (g & (1u << PIN_NFAULT))   s |= 0x08;   // nFault
    trace('S', s);
    io_delay();
    return s;
}

uint8_t r_dtr(void) {
    uint8_t v = (uint8_t)(gpio_get_all() >> PIN_D0);
    trace('R', v);
    io_delay();
    return v;
}

uint8_t lpt_ctr(void) {
    return ctr_shadow;
}

void lpt_set_oe(bool enable) {
    gpio_put(PIN_NOE, !enable);
    oe_enabled = enable;
}

bool lpt_oe(void) {
    return oe_enabled;
}

void lpt_set_delay_ns(uint32_t ns) {
    delay_ns = ns;
    ctl_cycles = ns_to_cycles(ns);
}

uint32_t lpt_delay_ns(void) {
    return delay_ns;
}

void lpt_set_data_delay_ns(uint32_t ns) {
    data_delay_ns = ns;
    data_cycles = ns_to_cycles(ns);
}

uint32_t lpt_data_delay_ns(void) {
    return data_delay_ns;
}

void lpt_data_mode(bool on) {
    data_mode = on;
}

void lpt_trace_enable(bool on) {
    trace_on = on;
}

bool lpt_trace_enabled(void) {
    return trace_on;
}

void lpt_trace_clear(void) {
    thead = tcount = 0;
}

void lpt_trace_mark(char op, uint8_t val) {
    trace(op, val);
}

uint32_t lpt_trace_count(void) {
    return tcount;
}

bool lpt_trace_get(uint32_t idx, lpt_trace_t *out) {
    if (idx >= tcount)
        return false;
    *out = tbuf[(thead + TRACE_N - tcount + idx) % TRACE_N];
    return true;
}
