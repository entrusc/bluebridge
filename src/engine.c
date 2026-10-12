// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger

#include "engine.h"

#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "wdt.h"

static queue_t jobs;

static void engine_main(void) {
    for (;;) {
        job_t *job;
        queue_remove_blocking(&jobs, &job);
        job->fn(job);
        __dmb();
        job->done = true;
    }
}

void engine_init(void) {
    queue_init(&jobs, sizeof(job_t *), 8);
    multicore_launch_core1(engine_main);
}

void engine_post(job_t *job) {
    job->done = false;
    __dmb();
    queue_add_blocking(&jobs, &job);
}

void engine_run(job_t *job, void (*idle)(void)) {
    engine_post(job);
    while (!job->done) {
        wdt_feed();
        if (idle)
            idle();
        else
            tight_loop_contents();
    }
    __dmb();
}
