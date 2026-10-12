// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// SCSI layer on top of the PPA transport. All zip_* functions run on core1;
// core0 reaches them through zjob_t jobs.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "engine.h"
#include "ppa.h"

#define ZIP_BLOCK_SIZE 512

typedef enum {
    ZIP_GOOD = 0,
    ZIP_CHECK,    // CHECK CONDITION, sense fetched
    ZIP_BUSY,     // BUSY / reservation conflict
    ZIP_XPORT,    // parallel transport failed (see zip_last_xport)
    ZIP_NODEV,    // no drive found (yet)
} zip_rc_t;

typedef struct {
    uint8_t key, asc, ascq;
} zip_sense_t;

const char *zip_rc_str(zip_rc_t rc);

bool zip_present(void);
int zip_target(void);
ppa_err_t zip_last_xport(void);
// Standard 36 bytes plus the Iomega firmware date (bytes 36..43, "MM/DD/YY").
#define ZIP_INQUIRY_LEN 44

const uint8_t *zip_inquiry_data(void);   // ZIP_INQUIRY_LEN bytes, NULL if not read yet

typedef enum {
    ZOP_INIT,         // ppa_init + device_check + INQUIRY
    ZOP_TUR,
    ZOP_CAPACITY,     // cached; flag = force refresh
    ZOP_READ,         // lba, n, buf
    ZOP_WRITE,        // lba, n, buf
    ZOP_START_STOP,   // flag = start, flag2 = load/eject
    ZOP_PREVENT,      // flag = prevent removal
    ZOP_CMD,          // raw: cdb, cdb_len, buf, len, dir, tmo_ms
} zop_t;

typedef struct {
    job_t job;
    zop_t op;
    // arguments
    uint32_t lba;
    uint16_t n;
    void *buf;
    uint32_t len;
    ppa_dir_t dir;
    uint32_t tmo_ms;
    uint8_t cdb[16];
    uint8_t cdb_len;
    bool flag, flag2;
    // results
    zip_rc_t rc;
    zip_sense_t sense;
    uint32_t blocks, bsize;
    bool wp;
    uint32_t xferred;
} zjob_t;

// Prepare a job for op (clears arguments and results).
void zjob_setup(zjob_t *j, zop_t op);
