// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger

#include "zip.h"

#include <string.h>

#define TMO_DEFAULT_MS      10000
#define TMO_RW_MS           20000
#define TMO_START_STOP_MS   30000

// SCSI sense keys used when the transport itself failed
#define SENSE_NOT_READY         0x02
#define SENSE_HARDWARE_ERROR    0x04

static volatile int target = -1;
static ppa_err_t last_xport;
static uint8_t inquiry[ZIP_INQUIRY_LEN];
static volatile bool inquiry_valid;

static bool cap_valid;
static uint32_t cap_blocks, cap_bsize;
static bool cap_wp;

const char *zip_rc_str(zip_rc_t rc) {
    switch (rc) {
    case ZIP_GOOD:  return "GOOD";
    case ZIP_CHECK: return "CHECK CONDITION";
    case ZIP_BUSY:  return "BUSY";
    case ZIP_XPORT: return "TRANSPORT ERROR";
    case ZIP_NODEV: return "NO DEVICE";
    }
    return "?";
}

bool zip_present(void) {
    return target >= 0;
}

int zip_target(void) {
    return target;
}

ppa_err_t zip_last_xport(void) {
    return last_xport;
}

const uint8_t *zip_inquiry_data(void) {
    return inquiry_valid ? inquiry : NULL;
}

static uint8_t cdb_length(uint8_t opcode) {
    switch (opcode >> 5) {
    case 0:  return 6;
    case 1:
    case 2:  return 10;
    case 4:  return 16;
    case 5:  return 12;
    default: return 10;
    }
}

static void note_sense(const zip_sense_t *s) {
    // Medium changed, no medium, or reset: capacity and WP must be re-read.
    if (s->asc == 0x28 || s->asc == 0x3a || s->asc == 0x29)
        cap_valid = false;
}

static void fetch_sense(zip_sense_t *sense) {
    static const uint8_t cdb[6] = { 0x03, 0, 0, 0, 18, 0 };
    uint8_t buf[18] = { 0 };
    uint8_t st, msg;
    uint32_t n;

    ppa_err_t e = ppa_command((uint8_t)target, cdb, 6, buf, sizeof buf, PPA_DIR_IN,
                              TMO_DEFAULT_MS, &st, &msg, &n);
    if (e != PPA_OK || (st & 0x3e) != 0 || n < 14) {
        *sense = (zip_sense_t){ SENSE_HARDWARE_ERROR, 0x44, 0x00 };
        return;
    }
    *sense = (zip_sense_t){ buf[2] & 0x0f, buf[12], buf[13] };
    note_sense(sense);
}

static zip_rc_t zip_command(const uint8_t *cdb, uint8_t cdb_len, void *buf, uint32_t len,
                            ppa_dir_t dir, uint32_t tmo_ms, uint32_t *xferred,
                            zip_sense_t *sense) {
    uint8_t st, msg;

    *sense = (zip_sense_t){ 0, 0, 0 };
    if (target < 0) {
        *sense = (zip_sense_t){ SENSE_NOT_READY, 0x3a, 0x00 };
        return ZIP_NODEV;
    }

    ppa_err_t e = ppa_command((uint8_t)target, cdb, cdb_len, buf, len, dir, tmo_ms,
                              &st, &msg, xferred);
    if (e != PPA_OK) {
        last_xport = e;
        cap_valid = false;
        *sense = (zip_sense_t){ SENSE_HARDWARE_ERROR, 0x44, 0x00 };
        if (e == PPA_E_CABLE || e == PPA_E_NOSELECT) {
            // Drive switched off or unplugged: forget it, MSC will re-probe.
            target = -1;
            inquiry_valid = false;
        } else {
            ppa_reset();
        }
        return ZIP_XPORT;
    }

    switch (st & 0x3e) {
    case 0x00:
        return ZIP_GOOD;
    case 0x02:
        fetch_sense(sense);
        return ZIP_CHECK;
    default:   // BUSY, RESERVATION CONFLICT, ...
        *sense = (zip_sense_t){ SENSE_NOT_READY, 0x04, 0x01 };
        return ZIP_BUSY;
    }
}

static zip_rc_t zip_init(zjob_t *j) {
    target = -1;
    inquiry_valid = false;
    cap_valid = false;

    ppa_err_t e = ppa_init();
    if (e != PPA_OK) {
        last_xport = e;
        return ZIP_XPORT;
    }
    int id = ppa_device_check();
    if (id < 0) {
        last_xport = (ppa_err_t)-id;
        return ZIP_NODEV;
    }
    target = id;
    last_xport = PPA_OK;

    static const uint8_t cdb[6] = { 0x12, 0, 0, 0, sizeof inquiry, 0 };
    memset(inquiry, 0, sizeof inquiry);
    uint32_t n;
    if (zip_command(cdb, 6, inquiry, sizeof inquiry, PPA_DIR_IN, TMO_DEFAULT_MS, &n,
                    &j->sense) == ZIP_GOOD)
        inquiry_valid = true;
    return ZIP_GOOD;
}

static zip_rc_t zip_capacity(zjob_t *j) {
    if (j->flag)
        cap_valid = false;
    if (!cap_valid) {
        static const uint8_t rc10[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        uint8_t buf[8] = { 0 };
        uint32_t n;
        zip_rc_t rc = zip_command(rc10, 10, buf, sizeof buf, PPA_DIR_IN, TMO_DEFAULT_MS, &n,
                                  &j->sense);
        if (rc != ZIP_GOOD)
            return rc;
        if (n < 8)
            return ZIP_XPORT;
        uint32_t last = (uint32_t)buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
        cap_bsize = (uint32_t)buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7];
        cap_blocks = last + 1;

        // Write protect flag from the mode parameter header. If the drive
        // refuses MODE SENSE, assume writable.
        static const uint8_t ms6[6] = { 0x1a, 0, 0x3f, 0, 4, 0 };
        uint8_t hdr[4] = { 0 };
        zip_sense_t ignored;
        cap_wp = zip_command(ms6, 6, hdr, sizeof hdr, PPA_DIR_IN, TMO_DEFAULT_MS, &n,
                             &ignored) == ZIP_GOOD && n >= 3 && (hdr[2] & 0x80);
        cap_valid = true;
    }
    j->blocks = cap_blocks;
    j->bsize = cap_bsize;
    j->wp = cap_wp;
    return ZIP_GOOD;
}

static zip_rc_t zip_rw(zjob_t *j, bool write) {
    uint8_t cdb[10] = {
        write ? 0x2a : 0x28, 0,
        (uint8_t)(j->lba >> 24), (uint8_t)(j->lba >> 16), (uint8_t)(j->lba >> 8), (uint8_t)j->lba,
        0, (uint8_t)(j->n >> 8), (uint8_t)j->n, 0,
    };
    uint32_t len = (uint32_t)j->n * ZIP_BLOCK_SIZE;
    zip_rc_t rc = zip_command(cdb, 10, j->buf, len, write ? PPA_DIR_OUT : PPA_DIR_IN,
                              TMO_RW_MS, &j->xferred, &j->sense);
    if (rc == ZIP_GOOD && j->xferred != len) {
        // GOOD status with a short transfer should not happen.
        j->sense = (zip_sense_t){ SENSE_HARDWARE_ERROR, 0x44, 0x00 };
        return ZIP_XPORT;
    }
    return rc;
}

static void zjob_exec(job_t *job) {
    zjob_t *j = (zjob_t *)job;
    uint32_t n;

    switch (j->op) {
    case ZOP_INIT:
        j->rc = zip_init(j);
        break;
    case ZOP_TUR: {
        static const uint8_t cdb[6] = { 0 };
        j->rc = zip_command(cdb, 6, NULL, 0, PPA_DIR_NONE, TMO_DEFAULT_MS, &n, &j->sense);
        break;
    }
    case ZOP_CAPACITY:
        j->rc = zip_capacity(j);
        break;
    case ZOP_READ:
        j->rc = zip_rw(j, false);
        break;
    case ZOP_WRITE:
        j->rc = zip_rw(j, true);
        break;
    case ZOP_START_STOP: {
        uint8_t cdb[6] = { 0x1b, 0, 0, 0, (uint8_t)((j->flag2 ? 2 : 0) | (j->flag ? 1 : 0)), 0 };
        j->rc = zip_command(cdb, 6, NULL, 0, PPA_DIR_NONE, TMO_START_STOP_MS, &n, &j->sense);
        if (j->flag2 && !j->flag)
            cap_valid = false;
        break;
    }
    case ZOP_PREVENT: {
        uint8_t cdb[6] = { 0x1e, 0, 0, 0, j->flag ? 1 : 0, 0 };
        j->rc = zip_command(cdb, 6, NULL, 0, PPA_DIR_NONE, TMO_DEFAULT_MS, &n, &j->sense);
        break;
    }
    case ZOP_CMD:
        if (!j->cdb_len)
            j->cdb_len = cdb_length(j->cdb[0]);
        j->rc = zip_command(j->cdb, j->cdb_len, j->buf, j->len, j->dir,
                            j->tmo_ms ? j->tmo_ms : TMO_DEFAULT_MS, &j->xferred, &j->sense);
        break;
    }
}

void zjob_setup(zjob_t *j, zop_t op) {
    memset(j, 0, sizeof *j);
    j->job.fn = zjob_exec;
    j->job.done = true;
    j->op = op;
}
