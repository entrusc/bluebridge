// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// PC parallel port register emulation.
// Only core1 (the engine) may call the register functions after boot.
#pragma once
#include <stdbool.h>
#include <stdint.h>

void lpt_init(void);

void w_ctr(uint8_t v);
void w_dtr(uint8_t v);
uint8_t r_str(void);
uint8_t r_dtr(void);

uint8_t lpt_ctr(void);          // last value written to the control register
void lpt_set_oe(bool enable);   // U2 N_OE, for bring-up only
bool lpt_oe(void);

// Delay for control/handshake sequences, also used for every bus turnaround.
void lpt_set_delay_ns(uint32_t ns);
uint32_t lpt_delay_ns(void);
// Delay used while lpt_data_mode(true), i.e. inside 512-byte data bursts.
void lpt_set_data_delay_ns(uint32_t ns);
uint32_t lpt_data_delay_ns(void);
void lpt_data_mode(bool on);

// Trace ring buffer: every register access as (op, value, time).
// op: 'C' w_ctr, 'D' w_dtr, 'S' r_str, 'R' r_dtr, anything else = marker.
// Consecutive identical reads are collapsed into one entry with rep > 1.
typedef struct {
    uint32_t t_us;
    uint16_t rep;
    char op;
    uint8_t val;
} lpt_trace_t;

void lpt_trace_enable(bool on);
bool lpt_trace_enabled(void);
void lpt_trace_clear(void);
void lpt_trace_mark(char op, uint8_t val);
uint32_t lpt_trace_count(void);
bool lpt_trace_get(uint32_t idx, lpt_trace_t *out);   // idx 0 = oldest
