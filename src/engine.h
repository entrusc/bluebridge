// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// Core1 worker. It is the only code that touches the parallel port after
// boot; core0 (USB, shell) hands it jobs through a queue.
#pragma once
#include <stdbool.h>

typedef struct job job_t;
typedef void (*job_fn_t)(job_t *job);

// Embed as the first member of a larger job struct.
struct job {
    job_fn_t fn;
    volatile bool done;
};

void engine_init(void);   // launches core1

// Queue a job and return immediately; poll job->done.
void engine_post(job_t *job);

// Queue a job and wait for it. idle (may be NULL) is called while waiting.
void engine_run(job_t *job, void (*idle)(void));
