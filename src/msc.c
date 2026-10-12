// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// TinyUSB MSC callbacks.
//
// Everything that touches the drive runs on core1. READ(10)/WRITE(10) are
// asynchronous: the callback posts a job and returns 0 ("busy"), and TinyUSB
// calls it again until the job is done. The short commands (TEST UNIT READY,
// READ CAPACITY, ...) wait for their job synchronously.
//
// Reads go through a small read-ahead cache, see the READ(10) section below.

#include "msc.h"

#include <string.h>

#include "led.h"
#include "pico/time.h"
#include "tusb.h"
#include "wdt.h"
#include "zip.h"

#define INIT_RETRY_MS 3000

static volatile bool enabled = true;

static zjob_t init_job;
static bool init_pending;
static absolute_time_t next_init;

static zjob_t sync_job;

static uint32_t disk_blocks;   // from READ CAPACITY, 0 = unknown

static void cache_invalidate(void);

static uint8_t cmd_buf[256] __attribute__((aligned(4)));   // pass-through data in

static bool write_protected;

// TinyUSB replaces the sense of a failed READ(10)/WRITE(10) with "medium not
// present". Remember the drive's real sense and put it back in REQUEST SENSE.
static struct {
    bool pending;
    zip_sense_t s;
} sense_ovr;

void msc_init(uint32_t probe_ms) {
    // Probe before USB comes up so INQUIRY can report the drive's own
    // vendor/product strings. A slow probe just carries on in the background.
    zjob_setup(&init_job, ZOP_INIT);
    init_pending = true;
    next_init = make_timeout_time_ms(INIT_RETRY_MS);
    engine_post(&init_job.job);
    absolute_time_t tmo = make_timeout_time_ms(probe_ms);
    while (!init_job.job.done && !time_reached(tmo))
        wdt_feed();
}

void msc_set_enabled(bool on) {
    enabled = on;
    if (!on) {
        led_media(false);
        cache_invalidate();
    }
}

bool msc_enabled(void) {
    return enabled;
}

static void set_sense(zip_sense_t s) {
    tud_msc_set_sense(0, s.key, s.asc, s.ascq);
    sense_ovr.s = s;
    sense_ovr.pending = true;
}

static void set_not_ready(void) {
    set_sense((zip_sense_t){ SCSI_SENSE_NOT_READY, 0x3a, 0x00 });
}

// Report a failed job's sense. Transport problems look like an empty drive,
// which the host handles more gracefully than a hardware error on TUR.
static void set_job_sense(const zjob_t *j, bool xport_as_not_ready) {
    if ((j->rc == ZIP_XPORT || j->rc == ZIP_NODEV) && xport_as_not_ready)
        set_not_ready();
    else
        set_sense(j->sense);
}

// True if the drive has been found. Otherwise (re)start probing in the
// background every few seconds so a drive powered on later is picked up.
static bool drive_usable(void) {
    if (!enabled)
        return false;
    if (init_pending) {
        if (!init_job.job.done)
            return false;
        init_pending = false;
    }
    if (zip_present())
        return true;
    if (time_reached(next_init)) {
        next_init = make_timeout_time_ms(INIT_RETRY_MS);
        zjob_setup(&init_job, ZOP_INIT);
        init_pending = true;
        engine_post(&init_job.job);
    }
    return false;
}

static void run_sync(zjob_t *j) {
    engine_run(&j->job, NULL);
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16],
                        uint8_t product_rev[4]) {
    (void)lun;
    const uint8_t *inq = zip_inquiry_data();
    if (inq) {
        memcpy(vendor_id, &inq[8], 8);
        memcpy(product_id, &inq[16], 16);
        memcpy(product_rev, &inq[32], 4);
    } else {
        memcpy(vendor_id, "Bluebrdg", 8);
        memcpy(product_id, "no drive        ", 16);
        memcpy(product_rev, "0.1 ", 4);
    }
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    if (!drive_usable()) {
        set_not_ready();
        led_media(false);
        return false;
    }
    zjob_setup(&sync_job, ZOP_TUR);
    run_sync(&sync_job);
    led_media(sync_job.rc == ZIP_GOOD);
    if (sync_job.rc == ZIP_GOOD)
        return true;
    cache_invalidate();
    set_job_sense(&sync_job, true);
    return false;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void)lun;
    *block_count = 0;
    *block_size = 0;
    if (!drive_usable()) {
        set_not_ready();
        return;
    }
    zjob_setup(&sync_job, ZOP_CAPACITY);
    run_sync(&sync_job);
    if (sync_job.rc != ZIP_GOOD) {
        cache_invalidate();
        disk_blocks = 0;
        set_job_sense(&sync_job, true);
        return;
    }
    if (sync_job.blocks != disk_blocks) {
        cache_invalidate();
        disk_blocks = sync_job.blocks;
    }
    *block_count = sync_job.blocks;
    *block_size = (uint16_t)sync_job.bsize;
    write_protected = sync_job.wp;
}

bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;
    return !write_protected;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
    (void)lun;
    (void)power_condition;
    if (!drive_usable()) {
        set_not_ready();
        return false;
    }
    cache_invalidate();
    zjob_setup(&sync_job, ZOP_START_STOP);
    sync_job.flag = start;
    sync_job.flag2 = load_eject;
    run_sync(&sync_job);
    if (load_eject && !start)
        led_media(false);
    if (sync_job.rc == ZIP_GOOD)
        return true;
    set_job_sense(&sync_job, false);
    return false;
}

bool tud_msc_prevent_allow_medium_removal_cb(uint8_t lun, uint8_t prohibit_removal,
                                             uint8_t control) {
    (void)lun;
    (void)control;
    if (!drive_usable()) {
        set_not_ready();
        return false;
    }
    zjob_setup(&sync_job, ZOP_PREVENT);
    sync_job.flag = prohibit_removal & 1;
    run_sync(&sync_job);
    if (sync_job.rc == ZIP_GOOD)
        return true;
    set_job_sense(&sync_job, false);
    return false;
}

int32_t tud_msc_request_sense_cb(uint8_t lun, void *buffer, uint16_t bufsize) {
    (void)lun;
    uint8_t *b = buffer;
    // Only replace TinyUSB's generic "medium not present" with what the
    // drive actually said; leave TinyUSB's own senses (bad CDB etc.) alone.
    if (sense_ovr.pending && b[2] == SCSI_SENSE_NOT_READY && b[12] == 0x3a) {
        b[2] = sense_ovr.s.key;
        b[12] = sense_ovr.s.asc;
        b[13] = sense_ovr.s.ascq;
    }
    sense_ovr.pending = false;
    return bufsize < 18 ? bufsize : 18;
}

// ---- READ(10) ------------------------------------------------------------
//
// Reads go through two 32 KB slots. A slot is filled either on demand (the
// host asked for data that isn't cached) or by read-ahead: while the host
// reads sequentially, the chunk after the one just served is fetched into the
// other slot while USB is still sending the current one. The drive thus gets
// fewer, larger commands back to back instead of losing a disk revolution
// between small ones. Random reads never trigger read-ahead, so they cost no
// extra drive time.
//
// Cached data is dropped on overlapping writes, media change, eject, errors,
// and in any case after SLOT_TTL_MS.

#define CHUNK_SECTORS   (CFG_TUD_MSC_EP_BUFSIZE / ZIP_BLOCK_SIZE)
#define SLOT_TTL_MS     1000

typedef enum { SLOT_EMPTY, SLOT_BUSY, SLOT_VALID, SLOT_FAILED } slot_state_t;

typedef struct {
    zjob_t job;           // job.lba / job.n describe the slot's range
    slot_state_t state;
    bool demand;          // read for a host request (its errors get reported)
    bool stale;           // invalidated while the read was still in flight
    bool used;            // served at least one request
    uint32_t gen;
    absolute_time_t expires;
    uint8_t buf[CFG_TUD_MSC_EP_BUFSIZE] __attribute__((aligned(4)));
} slot_t;

static slot_t slots[2];
static uint32_t cache_gen;
static uint32_t last_end = UINT32_MAX;   // lba following the previous host read
static msc_stats_t stats;

static inline uint32_t slot_end(const slot_t *s) {
    return s->job.lba + s->job.n;
}

static bool slot_covers(const slot_t *s, uint32_t lba, uint16_t n) {
    return lba >= s->job.lba && lba + n <= slot_end(s);
}

static bool slot_overlaps(const slot_t *s, uint32_t lba, uint16_t n) {
    return lba < slot_end(s) && s->job.lba < lba + n;
}

static void slot_drop(slot_t *s) {
    if (s->state == SLOT_VALID && !s->demand && !s->used)
        stats.ahead_wasted++;
    s->state = SLOT_EMPTY;
}

static void cache_invalidate(void) {
    // Reads still in flight are discarded when they finish (gen mismatch).
    cache_gen++;
    for (int i = 0; i < 2; i++)
        if (slots[i].state != SLOT_BUSY)
            slot_drop(&slots[i]);
    last_end = UINT32_MAX;
}

static void cache_invalidate_range(uint32_t lba, uint16_t n) {
    for (int i = 0; i < 2; i++) {
        slot_t *s = &slots[i];
        if (s->state == SLOT_EMPTY || !slot_overlaps(s, lba, n))
            continue;
        if (s->state == SLOT_BUSY)
            s->stale = true;
        else
            slot_drop(s);
    }
}

// Collect finished reads and expire old data.
static void slots_poll(void) {
    for (int i = 0; i < 2; i++) {
        slot_t *s = &slots[i];
        if (s->state == SLOT_BUSY && s->job.job.done) {
            if (s->stale || s->gen != cache_gen)
                s->state = SLOT_EMPTY;
            else if (s->job.rc == ZIP_GOOD) {
                s->state = SLOT_VALID;
                s->expires = make_timeout_time_ms(SLOT_TTL_MS);
            } else
                // A failed read-ahead is simply forgotten; the host's own
                // request will re-read it and report the error.
                s->state = s->demand ? SLOT_FAILED : SLOT_EMPTY;
        }
        if (s->state == SLOT_VALID && time_reached(s->expires))
            slot_drop(s);
    }
}

// A slot whose buffer may be refilled, other than `keep`.
static slot_t *free_slot(const slot_t *keep) {
    slot_t *best = NULL;
    for (int i = 0; i < 2; i++) {
        slot_t *s = &slots[i];
        if (s == keep || s->state == SLOT_BUSY)
            continue;
        if (!best || s->state != SLOT_VALID)
            best = s;
    }
    return best;
}

static void start_read(slot_t *s, uint32_t lba, uint16_t n, bool demand) {
    slot_drop(s);
    zjob_setup(&s->job, ZOP_READ);
    s->job.lba = lba;
    s->job.n = n;
    s->job.buf = s->buf;
    s->state = SLOT_BUSY;
    s->demand = demand;
    s->stale = false;
    s->used = false;
    s->gen = cache_gen;
    if (demand)
        stats.demand_reads++;
    else
        stats.ahead_reads++;
    engine_post(&s->job.job);
}

// Fetch the chunk after slot s unless it is cached or on its way already.
static void read_ahead(const slot_t *s) {
    uint32_t next = slot_end(s);
    if (!disk_blocks || next >= disk_blocks)
        return;
    for (int i = 0; i < 2; i++)
        if ((slots[i].state == SLOT_BUSY || slots[i].state == SLOT_VALID) &&
            slot_covers(&slots[i], next, 1))
            return;
    slot_t *t = free_slot(s);
    if (!t)
        return;
    uint32_t left = disk_blocks - next;
    start_read(t, next, (uint16_t)(left < CHUNK_SECTORS ? left : CHUNK_SECTORS), false);
}

static int32_t read_step(uint32_t lba, uint16_t n, void *buffer) {
    slots_poll();

    // Cached?
    for (int i = 0; i < 2; i++) {
        slot_t *s = &slots[i];
        if (s->state != SLOT_VALID || !slot_covers(s, lba, n))
            continue;
        memcpy(buffer, s->buf + (lba - s->job.lba) * ZIP_BLOCK_SIZE, n * ZIP_BLOCK_SIZE);
        if (!s->demand && !s->used)
            stats.ahead_used++;
        s->used = true;
        stats.host_reads++;
        // Read ahead only for a sequential stream, or for a full-size chunk
        // (the host command probably continues).
        bool sequential = lba == last_end;
        last_end = lba + n;
        if (sequential || n == CHUNK_SECTORS)
            read_ahead(s);
        led_activity();
        return (int32_t)n * ZIP_BLOCK_SIZE;
    }

    // On its way?
    for (int i = 0; i < 2; i++)
        if (slots[i].state == SLOT_BUSY && slot_covers(&slots[i], lba, n))
            return 0;

    // Our own read failed?
    for (int i = 0; i < 2; i++) {
        slot_t *s = &slots[i];
        if (s->state == SLOT_FAILED && s->job.lba == lba && s->job.n == n) {
            s->state = SLOT_EMPTY;
            last_end = UINT32_MAX;
            set_job_sense(&s->job, false);
            return -1;
        }
    }

    // Miss: read exactly what the host asked for.
    if (!drive_usable()) {
        set_not_ready();
        return -1;
    }
    slot_t *s = free_slot(NULL);
    if (!s)
        return 0;   // both slots busy, wait for one
    start_read(s, lba, n, true);
    led_activity();
    return 0;
}

// ---- WRITE(10) -----------------------------------------------------------
//
// Write-through: success is only reported once the drive accepted the data.
// The data is copied because TinyUSB reuses its buffer as soon as a new
// command arrives (e.g. after a host reset abandoned this one).

static zjob_t wr_job;
static bool wr_busy;
static uint8_t wr_buf[CFG_TUD_MSC_EP_BUFSIZE] __attribute__((aligned(4)));

static int32_t write_step(uint32_t lba, uint16_t n, const uint8_t *buffer) {
    if (wr_busy) {
        if (!wr_job.job.done)
            return 0;
        wr_busy = false;
        if (wr_job.lba == lba && wr_job.n == n) {
            led_activity();
            if (wr_job.rc != ZIP_GOOD) {
                cache_invalidate();
                set_job_sense(&wr_job, false);
                return -1;
            }
            stats.writes++;
            return (int32_t)n * ZIP_BLOCK_SIZE;
        }
        // Result of an earlier, abandoned request (host reset): start over.
    }

    if (!drive_usable()) {
        set_not_ready();
        return -1;
    }
    cache_invalidate_range(lba, n);
    last_end = UINT32_MAX;
    zjob_setup(&wr_job, ZOP_WRITE);
    wr_job.lba = lba;
    wr_job.n = n;
    wr_job.buf = wr_buf;
    memcpy(wr_buf, buffer, (size_t)n * ZIP_BLOCK_SIZE);
    wr_busy = true;
    engine_post(&wr_job.job);
    led_activity();
    return 0;
}

static bool rw_args_ok(uint32_t offset, uint32_t bufsize) {
    return offset == 0 && bufsize != 0 && bufsize % ZIP_BLOCK_SIZE == 0 &&
           bufsize <= CFG_TUD_MSC_EP_BUFSIZE;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer,
                          uint32_t bufsize) {
    (void)lun;
    // TinyUSB keeps calling us while a job runs, without returning to the
    // main loop.
    wdt_feed();
    if (!rw_args_ok(offset, bufsize))
        return -1;
    return read_step(lba, (uint16_t)(bufsize / ZIP_BLOCK_SIZE), buffer);
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer,
                           uint32_t bufsize) {
    (void)lun;
    wdt_feed();
    if (!rw_args_ok(offset, bufsize))
        return -1;
    return write_step(lba, (uint16_t)(bufsize / ZIP_BLOCK_SIZE), buffer);
}

void msc_get_stats(msc_stats_t *out) {
    *out = stats;
}

void msc_reset_stats(void) {
    memset(&stats, 0, sizeof stats);
}

// Commands TinyUSB does not handle itself.
int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer, uint16_t bufsize) {
    (void)lun;
    switch (scsi_cmd[0]) {
    case 0x35:   // SYNCHRONIZE CACHE(10): writes are already write-through
        return 0;

    case 0x5a:   // MODE SENSE(10): data in, pass through
    case 0x2f:   // VERIFY(10) without BYTCHK: no data, pass through
        if (scsi_cmd[0] == 0x2f && (scsi_cmd[1] & 0x02))
            break;
        if (!drive_usable()) {
            set_not_ready();
            return -1;
        }
        zjob_setup(&sync_job, ZOP_CMD);
        memcpy(sync_job.cdb, scsi_cmd, 10);
        sync_job.cdb_len = 10;
        if (scsi_cmd[0] == 0x5a) {
            sync_job.buf = cmd_buf;
            sync_job.len = bufsize < sizeof cmd_buf ? bufsize : sizeof cmd_buf;
            sync_job.dir = PPA_DIR_IN;
        }
        run_sync(&sync_job);
        if (sync_job.rc != ZIP_GOOD) {
            set_job_sense(&sync_job, false);
            return -1;
        }
        if (sync_job.xferred)
            memcpy(buffer, cmd_buf, sync_job.xferred);
        return (int32_t)sync_job.xferred;
    }

    tud_msc_set_sense(0, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}
