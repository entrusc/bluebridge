// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// USB Mass Storage front end: forwards to the drive through the engine.
#pragma once
#include <stdbool.h>
#include <stdint.h>

// Starts the first drive probe and waits up to probe_ms for it.
void msc_init(uint32_t probe_ms);

// When disabled the host sees an empty drive and the parallel port is left
// alone (used while poking registers from the shell).
void msc_set_enabled(bool on);
bool msc_enabled(void);

typedef struct {
    uint32_t host_reads;     // host read chunks served
    uint32_t demand_reads;   // drive reads for a host request (cache miss)
    uint32_t ahead_reads;    // drive reads started by read-ahead
    uint32_t ahead_used;     // read-ahead chunks the host then asked for
    uint32_t ahead_wasted;   // read-ahead chunks dropped unused
    uint32_t writes;         // host write chunks
} msc_stats_t;

void msc_get_stats(msc_stats_t *out);
void msc_reset_stats(void);
