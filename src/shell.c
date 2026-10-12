// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// Bring-up console. Every hardware access is a job on core1, so
// the engine stays the single owner of the parallel port.

#include "shell.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "console.h"
#include "engine.h"
#include "hardware/gpio.h"
#include "lpt.h"
#include "msc.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "ppa.h"
#include "tusb.h"
#include "wdt.h"
#include "zip.h"

#define LINE_MAX 96
#define MAX_ARGS 6

static char line[LINE_MAX];
static int line_len;
#define SBUF_SECTORS 64
static uint8_t sbuf[SBUF_SECTORS * ZIP_BLOCK_SIZE] __attribute__((aligned(4)));

// ---- raw register jobs ----------------------------------------------------

typedef enum {
    R_CTR, R_DTR, R_STR, R_RDTR, R_OE, R_CONNECT, R_DISCONNECT, R_RESET_PULSE,
    R_ECHO, R_RESET, R_WALK,
} rop_t;

typedef struct {
    job_t job;
    rop_t op;
    uint32_t a, b;
    uint32_t r0, r1;
} rjob_t;

static void rjob_exec(job_t *job) {
    rjob_t *j = (rjob_t *)job;
    uint8_t e, c;

    switch (j->op) {
    case R_CTR:         w_ctr((uint8_t)j->a); break;
    case R_DTR:         w_dtr((uint8_t)j->a); break;
    case R_STR:         j->r0 = r_str(); break;
    case R_RDTR:        j->r0 = r_dtr(); break;
    case R_OE:          lpt_set_oe(j->a != 0); break;
    case R_CONNECT:     ppa_connect(); break;
    case R_DISCONNECT:  ppa_disconnect(); break;
    case R_RESET_PULSE: ppa_reset_pulse(); break;
    case R_RESET:       ppa_reset(); break;
    case R_ECHO:
        ppa_connect();
        j->r0 = ppa_echo(&e, &c);
        j->r1 = (uint32_t)e << 8 | c;
        ppa_disconnect();
        break;
    case R_WALK: {
        uint8_t base = lpt_ctr();
        for (uint32_t i = 0; i < j->b * 2; i++) {
            w_ctr((i & 1) ? base : base ^ (uint8_t)j->a);
            sleep_ms(500);
        }
        w_ctr(base);
        break;
    }
    }
}

static uint32_t raw(rop_t op, uint32_t a, uint32_t b) {
    static rjob_t j;
    j = (rjob_t){ .job.fn = rjob_exec, .op = op, .a = a, .b = b };
    engine_run(&j.job, usb_task);
    return j.r0;
}

// Raw access and a live MSC session would interleave on the cable.
static void pause_msc(void) {
    if (msc_enabled()) {
        msc_set_enabled(false);
        printf("(MSC paused, 'msc on' to resume)\n");
    }
}

static void run_zjob(zjob_t *j) {
    engine_run(&j->job, usb_task);
}

// ---- helpers --------------------------------------------------------------

static void print_str(uint8_t s) {
    printf("str=%02x  Busy%c nAck=%d PError=%d Select=%d nFault=%d  phase=%02x\n", s,
           (s & 0x80) ? '-' : '+', !!(s & 0x40), !!(s & 0x20), !!(s & 0x10), !!(s & 0x08),
           s & 0xf0);
}

static void print_result(const zjob_t *j) {
    printf("%s", zip_rc_str(j->rc));
    if (j->rc == ZIP_CHECK || j->rc == ZIP_BUSY)
        printf("  sense key=%x asc=%02x ascq=%02x", j->sense.key, j->sense.asc, j->sense.ascq);
    if (j->rc == ZIP_XPORT || j->rc == ZIP_NODEV)
        printf("  (%s, last phase %02x)", ppa_err_str(zip_last_xport()), ppa_last_phase());
    printf("\n");
}

static void hexdump(const uint8_t *p, uint32_t n) {
    for (uint32_t i = 0; i < n; i += 16) {
        printf("%04lx ", (unsigned long)i);
        for (uint32_t k = 0; k < 16; k++)
            printf(i + k < n ? " %02x" : "   ", p[i + k]);
        printf("  ");
        for (uint32_t k = 0; k < 16 && i + k < n; k++)
            putchar(p[i + k] >= 32 && p[i + k] < 127 ? p[i + k] : '.');
        printf("\n");
    }
}

static void print_ascii(const char *label, const uint8_t *p, int n) {
    printf("%s'", label);
    for (int i = 0; i < n; i++)
        putchar(p[i] >= 32 && p[i] < 127 ? p[i] : '.');
    printf("'");
}

// Iomega puts the firmware date as "MM/DD/YY" right after the revision.
static bool is_fw_date(const uint8_t *p) {
    for (int i = 0; i < 8; i++) {
        bool slash = i == 2 || i == 5;
        if (slash ? p[i] != '/' : (p[i] < '0' || p[i] > '9'))
            return false;
    }
    return true;
}

static uint32_t num(const char *s) {
    return strtoul(s, NULL, 0);
}

static uint32_t hex(const char *s) {
    return strtoul(s, NULL, 16);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t n) {
    static uint32_t table[256];
    if (!table[1]) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++)
                c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
    }
    crc = ~crc;
    while (n--)
        crc = table[(crc ^ *p++) & 0xff] ^ (crc >> 8);
    return ~crc;
}

static zip_rc_t sector_io(bool write, uint32_t lba, uint16_t n, uint8_t *buf, zjob_t *j) {
    zjob_setup(j, write ? ZOP_WRITE : ZOP_READ);
    j->lba = lba;
    j->n = n;
    j->buf = buf;
    run_zjob(j);
    return j->rc;
}

// ---- commands -------------------------------------------------------------

static void cmd_help(int argc, char **argv);

static void cmd_pins(int argc, char **argv) {
    (void)argc; (void)argv;
    uint32_t g = gpio_get_all();
    printf("ctr=%02x (last written)  N_OE=%d DATA_DIR=%d(%s)  delay ctl=%luns data=%luns\n",
           lpt_ctr(), !!(g & (1u << PIN_NOE)), !!(g & (1u << PIN_DATA_DIR)),
           (g & (1u << PIN_DATA_DIR)) ? "in" : "out", (unsigned long)lpt_delay_ns(),
           (unsigned long)lpt_data_delay_ns());
    printf("pins: D=%02x nStrobe=%d nAutoFd=%d nInit=%d nSelectIn=%d\n",
           (unsigned)((g >> PIN_D0) & 0xff), !!(g & (1u << PIN_NSTROBE)),
           !!(g & (1u << PIN_NAUTOFD)), !!(g & (1u << PIN_NINIT)),
           !!(g & (1u << PIN_NSELECTIN)));
    printf("pins: nAck=%d Busy=%d PError=%d Select=%d nFault=%d\n", !!(g & (1u << PIN_NACK)),
           !!(g & (1u << PIN_BUSY)), !!(g & (1u << PIN_PERROR)), !!(g & (1u << PIN_SELECT)),
           !!(g & (1u << PIN_NFAULT)));
    printf("drive: %s", zip_present() ? "found" : "not found");
    if (zip_present())
        printf(" at ID %d", zip_target());
    printf(", MSC %s\n", msc_enabled() ? "on" : "off");
}

static void cmd_ctr(int argc, char **argv) {
    pause_msc();
    if (argc > 1)
        raw(R_CTR, hex(argv[1]), 0);
    printf("ctr=%02x\n", lpt_ctr());
}

static void cmd_dtr(int argc, char **argv) {
    if (argc < 2) {
        printf("usage: dtr <hex>\n");
        return;
    }
    pause_msc();
    raw(R_DTR, hex(argv[1]), 0);
}

static void cmd_str(int argc, char **argv) {
    (void)argc; (void)argv;
    print_str((uint8_t)raw(R_STR, 0, 0));
}

static void cmd_rdtr(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("dtr=%02lx\n", (unsigned long)raw(R_RDTR, 0, 0));
}

static void cmd_dir(int argc, char **argv) {
    if (argc < 2 || (strcmp(argv[1], "in") && strcmp(argv[1], "out"))) {
        printf("usage: dir in|out\n");
        return;
    }
    pause_msc();
    uint8_t c = lpt_ctr();
    raw(R_CTR, strcmp(argv[1], "in") ? (c & ~0x20u) : (c | 0x20u), 0);
    printf("ctr=%02x\n", lpt_ctr());
}

static void cmd_oe(int argc, char **argv) {
    if (argc < 2) {
        printf("usage: oe 0|1  (1 = buffer enabled)\n");
        return;
    }
    pause_msc();
    raw(R_OE, num(argv[1]), 0);
}

static void cmd_walk(int argc, char **argv) {
    if (argc < 2 || num(argv[1]) > 5) {
        printf("usage: walk <ctr bit 0-5> [seconds]\n");
        return;
    }
    uint32_t bit = num(argv[1]);
    uint32_t secs = argc > 2 ? num(argv[2]) : 10;
    pause_msc();
    printf("toggling ctr bit %lu at 1 Hz for %lus...\n", (unsigned long)bit, (unsigned long)secs);
    raw(R_WALK, 1u << bit, secs);
    printf("done, ctr=%02x\n", lpt_ctr());
}

static void cmd_delay(int argc, char **argv) {
    if (argc == 2) {
        lpt_set_delay_ns(num(argv[1]));
        lpt_set_data_delay_ns(num(argv[1]));
    } else if (argc == 3 && !strcmp(argv[1], "ctl")) {
        lpt_set_delay_ns(num(argv[2]));
    } else if (argc == 3 && !strcmp(argv[1], "data")) {
        lpt_set_data_delay_ns(num(argv[2]));
        if (msc_enabled() && lpt_data_delay_ns() < LPT_DEFAULT_DATA_DELAY_NS)
            printf("note: the host now gets data read with an unverified delay\n");
    } else if (argc != 1) {
        printf("usage: delay [ns] | delay ctl <ns> | delay data <ns>\n");
        return;
    }
    printf("io delay: ctl %luns, data %luns\n", (unsigned long)lpt_delay_ns(),
           (unsigned long)lpt_data_delay_ns());
}

static void cmd_trace(int argc, char **argv) {
    const char *sub = argc > 1 ? argv[1] : "";
    if (!strcmp(sub, "on")) {
        lpt_trace_clear();
        lpt_trace_enable(true);
    } else if (!strcmp(sub, "off")) {
        lpt_trace_enable(false);
    } else if (!strcmp(sub, "clear")) {
        lpt_trace_clear();
    } else if (!strcmp(sub, "dump")) {
        uint32_t cnt = lpt_trace_count();
        uint32_t max = argc > 2 ? num(argv[2]) : cnt;
        uint32_t first = cnt > max ? cnt - max : 0;
        lpt_trace_t t, t0;
        lpt_trace_get(first, &t0);
        for (uint32_t i = first; i < cnt && lpt_trace_get(i, &t); i++) {
            printf("%8lu %c %02x", (unsigned long)(t.t_us - t0.t_us), t.op, t.val);
            if (t.rep > 1)
                printf(" x%u", t.rep);
            printf("\n");
        }
        return;
    } else {
        printf("usage: trace on|off|clear|dump [n]\n");
        printf("ops: C=w_ctr D=w_dtr S=r_str R=r_dtr; markers c=connect d=disconnect "
               "s=select r=reset >=command <=result\n");
    }
    printf("trace %s, %lu entries\n", lpt_trace_enabled() ? "on" : "off",
           (unsigned long)lpt_trace_count());
}

static void cmd_connect(int argc, char **argv) {
    (void)argc; (void)argv;
    pause_msc();
    raw(R_CONNECT, 0, 0);
    cmd_str(0, NULL);
}

static void cmd_disconnect(int argc, char **argv) {
    (void)argc; (void)argv;
    pause_msc();
    raw(R_DISCONNECT, 0, 0);
    cmd_str(0, NULL);
}

static void cmd_rpulse(int argc, char **argv) {
    (void)argc; (void)argv;
    pause_msc();
    raw(R_RESET_PULSE, 0, 0);
}

static void cmd_reset(int argc, char **argv) {
    (void)argc; (void)argv;
    pause_msc();
    raw(R_RESET, 0, 0);
    printf("drive reset (connect, reset pulse, disconnect)\n");
}

static void cmd_echo(int argc, char **argv) {
    (void)argc; (void)argv;
    pause_msc();
    static rjob_t j;
    j = (rjob_t){ .job.fn = rjob_exec, .op = R_ECHO };
    engine_run(&j.job, usb_task);
    uint8_t e = (uint8_t)(j.r1 >> 8), c = (uint8_t)j.r1;
    printf("ctr=0e: ");
    print_str(e);
    printf("ctr=0c: ");
    print_str(c);
    printf("cable check %s (expect nFault=1 then 0)\n", j.r0 ? "PASS" : "FAIL");
}

static void cmd_init(int argc, char **argv) {
    (void)argc; (void)argv;
    static zjob_t j;
    zjob_setup(&j, ZOP_INIT);
    run_zjob(&j);
    print_result(&j);
    if (j.rc == ZIP_GOOD) {
        printf("drive at SCSI ID %d", zip_target());
        const uint8_t *inq = zip_inquiry_data();
        if (inq) {
            print_ascii("  ", &inq[8], 8);
            print_ascii(" ", &inq[16], 16);
            print_ascii(" ", &inq[32], 4);
            if (is_fw_date(&inq[36]))
                print_ascii(" ", &inq[36], 8);
        }
        printf("\n");
    }
}

static void cmd_inq(int argc, char **argv) {
    uint32_t len = argc > 1 ? num(argv[1]) : 36;
    if (len < 5 || len > 255) {
        printf("usage: inq [len]  (5..255, default 36)\n");
        return;
    }
    static zjob_t j;
    zjob_setup(&j, ZOP_CMD);
    j.cdb[0] = 0x12;
    j.cdb[4] = len;
    j.buf = sbuf;
    j.len = len;
    j.dir = PPA_DIR_IN;
    memset(sbuf, 0, len);
    run_zjob(&j);
    print_result(&j);
    if (j.xferred) {
        hexdump(sbuf, j.xferred);
        if (j.xferred >= 5)
            printf("additional length %u (%u bytes total)\n", sbuf[4], sbuf[4] + 5u);
        if (j.xferred >= 36) {
            print_ascii("vendor ", &sbuf[8], 8);
            print_ascii(" product ", &sbuf[16], 16);
            print_ascii(" rev ", &sbuf[32], 4);
            printf("\n");
        }
        if (j.xferred >= 44 && is_fw_date(&sbuf[36])) {
            print_ascii("firmware date ", &sbuf[36], 8);
            printf("\n");
        }
        if (j.xferred > 96) {
            print_ascii("copyright ", &sbuf[96], j.xferred - 96);
            printf("\n");
        }
    }
}

static void cmd_tur(int argc, char **argv) {
    (void)argc; (void)argv;
    static zjob_t j;
    zjob_setup(&j, ZOP_TUR);
    run_zjob(&j);
    print_result(&j);
}

static void cmd_sense(int argc, char **argv) {
    (void)argc; (void)argv;
    static zjob_t j;
    zjob_setup(&j, ZOP_CMD);
    j.cdb[0] = 0x03;
    j.cdb[4] = 18;
    j.buf = sbuf;
    j.len = 18;
    j.dir = PPA_DIR_IN;
    run_zjob(&j);
    print_result(&j);
    if (j.xferred) {
        hexdump(sbuf, j.xferred);
        if (j.xferred >= 14)
            printf("key=%x asc=%02x ascq=%02x\n", sbuf[2] & 0x0f, sbuf[12], sbuf[13]);
    }
}

static void cmd_cap(int argc, char **argv) {
    (void)argc; (void)argv;
    static zjob_t j;
    zjob_setup(&j, ZOP_CAPACITY);
    j.flag = true;   // bypass cache
    run_zjob(&j);
    print_result(&j);
    if (j.rc == ZIP_GOOD)
        printf("%lu blocks x %lu bytes = %lu KiB%s\n", (unsigned long)j.blocks,
               (unsigned long)j.bsize, (unsigned long)((uint64_t)j.blocks * j.bsize / 1024),
               j.wp ? ", write protected" : "");
}

static void cmd_read(int argc, char **argv) {
    if (argc < 2) {
        printf("usage: read <lba>\n");
        return;
    }
    static zjob_t j;
    uint32_t lba = num(argv[1]);
    memset(sbuf, 0, ZIP_BLOCK_SIZE);
    sector_io(false, lba, 1, sbuf, &j);
    print_result(&j);
    if (j.rc == ZIP_GOOD) {
        hexdump(sbuf, ZIP_BLOCK_SIZE);
        if (lba == 0)
            printf("signature %02x%02x %s\n", sbuf[510], sbuf[511],
                   (sbuf[510] == 0x55 && sbuf[511] == 0xaa) ? "(valid MBR/boot sector)" : "");
    }
}

// Write a pattern to one sector, read it back, then restore the original.
static void cmd_wtest(int argc, char **argv) {
    if (argc < 3 || strcmp(argv[2], "yes")) {
        printf("usage: wtest <lba> yes   (overwrites and then restores one sector)\n");
        return;
    }
    static zjob_t j;
    uint32_t lba = num(argv[1]);
    uint8_t *orig = sbuf, *pat = sbuf + 512, *back = sbuf + 1024;

    if (sector_io(false, lba, 1, orig, &j) != ZIP_GOOD) {
        printf("read original: ");
        print_result(&j);
        return;
    }
    for (int i = 0; i < 512; i++)
        pat[i] = (uint8_t)((i * 7 + lba) ^ 0xa5);
    if (sector_io(true, lba, 1, pat, &j) != ZIP_GOOD) {
        printf("write pattern: ");
        print_result(&j);
        return;
    }
    memset(back, 0, 512);
    bool ok = sector_io(false, lba, 1, back, &j) == ZIP_GOOD && !memcmp(pat, back, 512);
    printf("pattern readback %s\n", ok ? "OK" : "MISMATCH");
    if (!ok && j.rc == ZIP_GOOD) {
        for (int i = 0; i < 512; i++)
            if (pat[i] != back[i]) {
                printf("first difference at %d: wrote %02x read %02x\n", i, pat[i], back[i]);
                break;
            }
    }
    if (sector_io(true, lba, 1, orig, &j) != ZIP_GOOD) {
        printf("RESTORE FAILED: ");
        print_result(&j);
        return;
    }
    memset(back, 0, 512);
    ok = sector_io(false, lba, 1, back, &j) == ZIP_GOOD && !memcmp(orig, back, 512);
    printf("original restored %s\n", ok ? "OK" : "MISMATCH");
}

static void cmd_bench(int argc, char **argv) {
    uint32_t lba = argc > 1 ? num(argv[1]) : 0;
    uint32_t count = argc > 2 ? num(argv[2]) : 2048;
    uint32_t chunk = argc > 3 ? num(argv[3]) : SBUF_SECTORS;
    if (chunk < 1 || chunk > SBUF_SECTORS) {
        printf("chunk must be 1..%d sectors\n", SBUF_SECTORS);
        return;
    }
    static zjob_t j;
    uint64_t t0 = time_us_64();
    uint32_t done = 0;
    while (done < count) {
        uint16_t n = (uint16_t)(count - done > chunk ? chunk : count - done);
        if (sector_io(false, lba + done, n, sbuf, &j) != ZIP_GOOD) {
            printf("at lba %lu: ", (unsigned long)(lba + done));
            print_result(&j);
            break;
        }
        done += n;
    }
    uint64_t us = time_us_64() - t0;
    if (us)
        printf("%lu sectors, %lu per command, in %lu ms = %lu KB/s\n", (unsigned long)done,
               (unsigned long)chunk,
               (unsigned long)(us / 1000), (unsigned long)((uint64_t)done * 512 * 1000 / us));
}

// CRC32 over count sectors from lba, read in chunks of `chunk` sectors.
// Returns false on a drive error (printed). *kbs gets the throughput.
static bool area_crc(uint32_t lba, uint32_t count, uint32_t chunk, uint32_t *crc,
                     uint32_t *kbs) {
    static zjob_t j;
    uint32_t c = 0, done = 0;
    uint64_t t0 = time_us_64();
    while (done < count) {
        uint16_t n = (uint16_t)(count - done > chunk ? chunk : count - done);
        if (sector_io(false, lba + done, n, sbuf, &j) != ZIP_GOOD) {
            printf("at lba %lu: ", (unsigned long)(lba + done));
            print_result(&j);
            return false;
        }
        c = crc32_update(c, sbuf, (uint32_t)n * ZIP_BLOCK_SIZE);
        done += n;
    }
    uint64_t us = time_us_64() - t0;
    *crc = c;
    *kbs = us ? (uint32_t)((uint64_t)count * 512 * 1000 / us) : 0;
    return true;
}

static void cmd_sum(int argc, char **argv) {
    uint32_t lba = argc > 1 ? num(argv[1]) : 0;
    uint32_t count = argc > 2 ? num(argv[2]) : 2048;
    uint32_t chunk = argc > 3 ? num(argv[3]) : SBUF_SECTORS;
    if (!count || chunk < 1 || chunk > SBUF_SECTORS) {
        printf("usage: sum [lba] [count] [chunk 1..%d]\n", SBUF_SECTORS);
        return;
    }
    uint32_t crc, kbs;
    if (area_crc(lba, count, chunk, &crc, &kbs))
        printf("crc32 %08lx over %lu sectors from %lu, %lu KB/s (data delay %luns)\n",
               (unsigned long)crc, (unsigned long)count, (unsigned long)lba,
               (unsigned long)kbs, (unsigned long)lpt_data_delay_ns());
}

// Sweep the data delay downwards and check every step against a reference
// CRC taken at the (safe) control delay. Read-only; restores the delay.
static void cmd_dtune(int argc, char **argv) {
    static const uint32_t steps[] = { 750, 500, 400, 300, 250, 200, 150, 100, 50, 0 };
    const int runs = 3;
    uint32_t lba = argc > 1 ? num(argv[1]) : 0;
    uint32_t count = argc > 2 ? num(argv[2]) : 2048;
    uint32_t saved = lpt_data_delay_ns();
    uint32_t safe = lpt_delay_ns();
    bool was_on = msc_enabled();
    uint32_t ref, crc, kbs, best = safe;
    bool all_ok = true;

    // The host must not get data read at an unverified delay.
    msc_set_enabled(false);

    lpt_set_data_delay_ns(safe);
    if (!area_crc(lba, count, SBUF_SECTORS, &ref, &kbs)) {
        printf("reference read failed\n");
        goto out;
    }
    printf("reference at %luns: crc32 %08lx, %lu KB/s\n", (unsigned long)safe,
           (unsigned long)ref, (unsigned long)kbs);

    for (size_t i = 0; i < sizeof steps / sizeof steps[0]; i++) {
        if (steps[i] >= safe)
            continue;
        lpt_set_data_delay_ns(steps[i]);
        int good = 0;
        uint32_t speed = 0;
        for (int r = 0; r < runs; r++) {
            if (area_crc(lba, count, SBUF_SECTORS, &crc, &kbs) && crc == ref) {
                good++;
                speed = kbs;
            }
        }
        printf("%4luns: %d/%d ok", (unsigned long)steps[i], good, runs);
        if (good)
            printf(", %lu KB/s", (unsigned long)speed);
        printf("\n");
        if (good == runs && all_ok)
            best = steps[i];
        else
            all_ok = false;
    }
    printf("fastest delay with it and every slower step passing: %luns\n",
           (unsigned long)best);
    printf("pick a margin above that, set it with 'delay data <ns>', then run 'wtest'\n");

out:
    lpt_set_data_delay_ns(saved);
    msc_set_enabled(was_on);
    printf("data delay restored to %luns\n", (unsigned long)saved);
}

static void cmd_cache(int argc, char **argv) {
    if (argc > 1 && !strcmp(argv[1], "reset")) {
        msc_reset_stats();
        printf("counters reset\n");
        return;
    }
    msc_stats_t s;
    msc_get_stats(&s);
    printf("host reads %lu, drive reads: %lu on demand + %lu read-ahead\n",
           (unsigned long)s.host_reads, (unsigned long)s.demand_reads,
           (unsigned long)s.ahead_reads);
    printf("read-ahead used %lu, wasted %lu; host writes %lu\n", (unsigned long)s.ahead_used,
           (unsigned long)s.ahead_wasted, (unsigned long)s.writes);
}

static void cmd_eject(int argc, char **argv) {
    (void)argc; (void)argv;
    static zjob_t j;
    zjob_setup(&j, ZOP_START_STOP);
    j.flag = false;
    j.flag2 = true;
    run_zjob(&j);
    print_result(&j);
}

static void cmd_msc(int argc, char **argv) {
    if (argc > 1)
        msc_set_enabled(!strcmp(argv[1], "on"));
    printf("MSC %s\n", msc_enabled() ? "on" : "off");
}

static void cmd_bootsel(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("rebooting into BOOTSEL...\n");
    sleep_ms(100);
    reset_usb_boot(0, 0);
}

static const struct {
    const char *name;
    void (*fn)(int argc, char **argv);
    const char *help;
} cmds[] = {
    { "help",       cmd_help,       "this list" },
    { "pins",       cmd_pins,       "raw GPIO levels and state" },
    { "ctr",        cmd_ctr,        "[hex]  write/show control register (PC semantics)" },
    { "dtr",        cmd_dtr,        "<hex>  write data register" },
    { "str",        cmd_str,        "read status register" },
    { "rdtr",       cmd_rdtr,       "read data register" },
    { "dir",        cmd_dir,        "in|out  set ctr bit 5 (data direction)" },
    { "oe",         cmd_oe,         "0|1  data buffer enable" },
    { "walk",       cmd_walk,       "<bit> [s]  toggle a ctr bit at 1 Hz (multimeter)" },
    { "delay",      cmd_delay,      "[ns] | ctl <ns> | data <ns>  per-access io delay" },
    { "trace",      cmd_trace,      "on|off|clear|dump [n]" },
    { "connect",    cmd_connect,    "ppa_connect" },
    { "disconnect", cmd_disconnect, "ppa_disconnect" },
    { "rpulse",     cmd_rpulse,     "ppa_reset_pulse" },
    { "reset",      cmd_reset,      "connect + reset pulse + disconnect" },
    { "echo",       cmd_echo,       "connect + cable check + disconnect" },
    { "init",       cmd_init,       "ppa_init + device scan + INQUIRY" },
    { "inq",        cmd_inq,        "[len]  INQUIRY (default 36, max 255)" },
    { "tur",        cmd_tur,        "TEST UNIT READY" },
    { "sense",      cmd_sense,      "REQUEST SENSE" },
    { "cap",        cmd_cap,        "READ CAPACITY(10) + write protect" },
    { "read",       cmd_read,       "<lba>  hexdump one sector" },
    { "wtest",      cmd_wtest,      "<lba> yes  write/verify/restore one sector" },
    { "bench",      cmd_bench,      "[lba] [count] [chunk]  read throughput" },
    { "cache",      cmd_cache,      "[reset]  USB read-ahead statistics" },
    { "sum",        cmd_sum,        "[lba] [count] [chunk]  CRC32 of a disk area + speed" },
    { "dtune",      cmd_dtune,      "[lba] [count]  sweep data delay, verify by CRC" },
    { "eject",      cmd_eject,      "START STOP UNIT (eject)" },
    { "msc",        cmd_msc,        "on|off  USB mass storage access" },
    { "bootsel",    cmd_bootsel,    "reboot into the USB bootloader" },
};

static void cmd_help(int argc, char **argv) {
    (void)argc; (void)argv;
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++)
        printf("  %-10s %s\n", cmds[i].name, cmds[i].help);
    printf("register values are hex, lba/count/ns decimal (or 0x..)\n");
}

static void execute(char *s) {
    char *argv[MAX_ARGS];
    int argc = 0;
    for (char *tok = strtok(s, " \t"); tok && argc < MAX_ARGS; tok = strtok(NULL, " \t"))
        argv[argc++] = tok;
    if (!argc)
        return;
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++) {
        if (!strcmp(argv[0], cmds[i].name)) {
            cmds[i].fn(argc, argv);
            return;
        }
    }
    printf("unknown command '%s', try 'help'\n", argv[0]);
}

void shell_task(void) {
    static bool was_connected;
    bool connected = tud_cdc_connected();
    if (connected && !was_connected)
        printf("\nBluebridge bring-up console. 'help' for commands.\n"
               "last reset: %s\n> ", wdt_reset_reason());
    was_connected = connected;

    static bool last_cr;
    while (tud_cdc_available()) {
        char c;
        if (tud_cdc_read(&c, 1) != 1)
            break;
        if (c == '\n' && last_cr) {   // CR LF counts as one line end
            last_cr = false;
            continue;
        }
        last_cr = c == '\r';
        if (c == '\r' || c == '\n') {
            printf("\n");
            line[line_len] = 0;
            execute(line);
            line_len = 0;
            printf("> ");
        } else if (c == 0x7f || c == '\b') {
            if (line_len) {
                line_len--;
                printf("\b \b");
            }
        } else if (c >= 32 && line_len < LINE_MAX - 1) {
            line[line_len++] = c;
            putchar(c);
        }
    }
}
