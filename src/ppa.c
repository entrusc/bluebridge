// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2026 Florian Frankenberger (port)
//
// Iomega PPA3 (ZIP 100 parallel) transport.
//
// Ported from Linux drivers/scsi/ppa.c:
//   (c) 1995,1996 Grant R. Guenther, grant@torque.net
//   and David Campbell et al., under the GNU General Public License.
//
// Only the PS/2 (8-bit bidirectional) paths are kept. The register sequences
// are verbatim; function names match the original so the two can be compared
// side by side. Loop-count timeouts are replaced with time-based ones, and
// the scheduler-driven ppa_engine state machine becomes one blocking call.

#include "ppa.h"

#include "lpt.h"
#include "pico/time.h"

#define PPA_BURST_SIZE      512
#define PPA_SELECT_TMO_US   5000
#define PPA_SPIN_TMO_US     50000
#define PPA_RECON_TMO_US    500
#define PPA_POLL_US         50      // re-poll interval where Linux reschedules
#define PPA_SLICE_US        10000   // ppa_completion "one jiffy" budget

static ppa_err_t failed;
static uint8_t last_phase;

const char *ppa_err_str(ppa_err_t e) {
    switch (e) {
    case PPA_OK:         return "ok";
    case PPA_E_CABLE:    return "cable check failed (nFault does not follow nAutoFd)";
    case PPA_E_NOSELECT: return "no answer to selection";
    case PPA_E_TIMEOUT:  return "timeout";
    case PPA_E_PROTO:    return "unexpected phase";
    case PPA_E_BUSBUSY:  return "transfer failed";
    case PPA_E_OVERRUN:  return "data overrun";
    case PPA_E_DIR:      return "wrong data direction";
    case PPA_E_NODEV:    return "no device found";
    }
    return "?";
}

uint8_t ppa_last_phase(void) {
    return last_phase;
}

// Keep the first error: it is the most informative (e.g. a timeout inside
// ppa_wait, rather than the phase error ppa_out reports right after it).
static void ppa_fail(ppa_err_t e) {
    if (!failed)
        failed = e;
}

// Wait for bits 6 and 7 (nAck and ready), return the phase nibble:
//   0xc0 = ZIP wants more data
//   0xd0 = ZIP wants to send more data
//   0xe0 = ZIP is expecting SCSI command data
//   0xf0 = end of transfer, ZIP is sending status
static uint8_t ppa_wait(void) {
    absolute_time_t tmo = make_timeout_time_us(PPA_SPIN_TMO_US);
    uint8_t r = r_str();

    while ((r & 0xc0) != 0xc0) {
        if (time_reached(tmo)) {
            last_phase = r;
            ppa_fail(PPA_E_TIMEOUT);
            return 0;
        }
        busy_wait_us_32(1);
        r = r_str();
    }
    last_phase = r & 0xf0;
    return r & 0xf0;
}

static int ppa_byte_out(const uint8_t *buffer, int len) {
    for (int i = len; i; i--) {
        w_dtr(*buffer++);
        w_ctr(0xe);
        w_ctr(0xc);
    }
    return 1;
}

static int ppa_byte_in(uint8_t *buffer, int len) {
    for (int i = len; i; i--) {
        *buffer++ = r_dtr();
        w_ctr(0x27);
        w_ctr(0x25);
    }
    return 1;
}

static int ppa_out(const uint8_t *buffer, int len) {
    uint8_t r = ppa_wait();

    if ((r & 0x50) != 0x40) {
        ppa_fail(PPA_E_PROTO);
        return 0;
    }
    return ppa_byte_out(buffer, len);
}

static int ppa_in(uint8_t *buffer, int len) {
    uint8_t r = ppa_wait();

    if ((r & 0x50) != 0x50) {
        ppa_fail(PPA_E_PROTO);
        return 0;
    }
    w_ctr(0x25);
    int ret = ppa_byte_in(buffer, len);
    w_ctr(0x4);
    w_ctr(0xc);
    return ret;
}

static void ppa_d_pulse(uint8_t b) {
    w_dtr(b);
    w_ctr(0xc);
    w_ctr(0xe);
    w_ctr(0xc);
    w_ctr(0x4);
    w_ctr(0xc);
}

void ppa_disconnect(void) {
    lpt_trace_mark('d', 0);
    ppa_d_pulse(0);
    ppa_d_pulse(0x3c);
    ppa_d_pulse(0x20);
    ppa_d_pulse(0xf);
}

static void ppa_c_pulse(uint8_t b) {
    w_dtr(b);
    w_ctr(0x4);
    w_ctr(0x6);
    w_ctr(0x4);
    w_ctr(0xc);
}

void ppa_connect(void) {
    lpt_trace_mark('c', 0);
    ppa_c_pulse(0);
    ppa_c_pulse(0x3c);
    ppa_c_pulse(0x20);
    ppa_c_pulse(0x8f);   // CONNECT_NORMAL / not EPP
}

// Bit 6 (0x40) of the status register is the "device selected" bit.
static int ppa_select(int target) {
    absolute_time_t tmo;
    bool ok;

    lpt_trace_mark('s', (uint8_t)target);

    // First wait until the current device goes off line.
    tmo = make_timeout_time_us(PPA_SELECT_TMO_US);
    do {
        busy_wait_us_32(1);
        ok = !(r_str() & 0x40);
    } while (!ok && !time_reached(tmo));
    if (!ok)
        return 0;

    w_dtr(1 << target);
    w_ctr(0xe);
    w_ctr(0xc);
    w_dtr(0x80);   // This is NOT the initiator
    w_ctr(0x8);

    tmo = make_timeout_time_us(PPA_SELECT_TMO_US);
    do {
        busy_wait_us_32(1);
        ok = (r_str() & 0x40) != 0;
    } while (!ok && !time_reached(tmo));

    return ok;
}

void ppa_reset_pulse(void) {
    lpt_trace_mark('r', 0);
    w_dtr(0x40);
    w_ctr(0x8);
    busy_wait_us_32(30);
    w_ctr(0xc);
}

// Cable sanity check from ppa_init / ppa_engine phase 1: while connected,
// the drive echoes nAutoFeed on nFault.
bool ppa_echo(uint8_t *str_e, uint8_t *str_c) {
    int retv = 2;
    uint8_t s;

    w_ctr(0xe);
    s = r_str();
    if (str_e)
        *str_e = s;
    if ((s & 0x08) == 0x08)
        retv--;

    w_ctr(0xc);
    s = r_str();
    if (str_c)
        *str_c = s;
    if ((s & 0x08) == 0x00)
        retv--;

    return retv == 0;
}

// Based on a trace of the Iomega DOS 'guest' driver (see ppa.c).
ppa_err_t ppa_init(void) {
    w_ctr(0x0c);   // done by __ppa_attach before ppa_init

    ppa_disconnect();
    ppa_connect();

    bool ok = ppa_echo(NULL, NULL);
    if (ok)
        ppa_reset_pulse();
    sleep_us(1000);   // Allow devices to settle down
    ppa_disconnect();
    sleep_us(1000);   // Another delay to allow devices to settle

    return ok ? PPA_OK : PPA_E_CABLE;
}

static void ppa_reset_sequence(void) {
    ppa_disconnect();
    ppa_connect();
    ppa_reset_pulse();
    sleep_us(1000);
    ppa_disconnect();
    sleep_us(1000);
}

int ppa_device_check(void) {
    static const uint8_t cmd[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    for (int loop = 0; loop < 8; loop++) {
        ppa_connect();
        if (!ppa_select(loop)) {
            ppa_disconnect();
            continue;
        }

        // Send SCSI command (TEST UNIT READY)
        failed = PPA_OK;
        int status = 1;
        w_ctr(0x0c);
        for (int l = 0; (l < 6) && status; l++)
            status = ppa_out(cmd, 1);

        if (!status) {
            ppa_reset_sequence();
            return -PPA_E_PROTO;
        }

        w_ctr(0x0c);
        absolute_time_t tmo = make_timeout_time_us(1000000);   // 1 second
        uint8_t l;
        do {
            l = r_str();
            busy_wait_us_32(1);
        } while (!(l & 0x80) && !time_reached(tmo));

        l &= 0xf0;
        last_phase = l;

        if (l != 0xf0) {
            ppa_reset_sequence();
            return -PPA_E_PROTO;
        }

        // Communication established
        ppa_reset_sequence();
        return loop;
    }
    return -PPA_E_NODEV;
}

void ppa_reset(void) {
    ppa_reset_sequence();
}

static int ppa_send_command(const uint8_t *cdb, uint8_t cdb_len) {
    w_ctr(0x0c);

    for (int k = 0; k < cdb_len; k++)
        if (!ppa_out(&cdb[k], 1))
            return 0;
    return 1;
}

typedef struct {
    uint8_t *ptr;
    int32_t residual;
    ppa_dir_t dir;
    bool bulk;
} xfer_t;

// Return codes: -1 error, 0 "told to schedule" (poll again), 1 finished.
static int ppa_completion(xfer_t *x) {
    absolute_time_t slice = make_timeout_time_us(PPA_SLICE_US);
    uint8_t r = r_str() & 0xf0;

    while (r != 0xf0) {
        // If we have been running for more than a full timer tick then
        // take a rest.
        if (time_reached(slice))
            return 0;

        if (x->residual <= 0) {
            last_phase = r;
            ppa_fail(PPA_E_OVERRUN);
            return -1;
        }

        // Some hardware drops the connected bit (6) for ~100us; busy wait
        // a little for it to come back before giving up this slice.
        if ((r & 0xc0) != 0xc0) {
            absolute_time_t recon = make_timeout_time_us(PPA_RECON_TMO_US);
            while (((r = (r_str() & 0xf0)) & 0xc0) != 0xc0) {
                if (time_reached(recon))
                    return 0;
                busy_wait_us_32(1);
            }
        }

        // determine if we should use burst I/O
        int fast = (x->bulk && x->residual >= PPA_BURST_SIZE) ? PPA_BURST_SIZE : 1;
        bool burst = fast == PPA_BURST_SIZE;
        int status;

        // ppa.c: "The driver appears to remain stable if we speed up the
        // parallel port i/o in this function, but not elsewhere." Only the
        // 512-byte bursts use the (tunable) data delay.
        if (r == 0xc0) {
            if (x->dir != PPA_DIR_OUT) {
                last_phase = r;
                ppa_fail(PPA_E_DIR);
                return -1;
            }
            lpt_data_mode(burst);
            status = ppa_out(x->ptr, fast);
            lpt_data_mode(false);
        } else {
            if (x->dir != PPA_DIR_IN) {
                last_phase = r;
                ppa_fail(PPA_E_DIR);
                return -1;
            }
            lpt_data_mode(burst);
            status = ppa_in(x->ptr, fast);
            lpt_data_mode(false);
        }

        x->ptr += fast;
        x->residual -= fast;

        if (!status) {
            ppa_fail(PPA_E_BUSBUSY);
            return -1;
        }

        // Now check to see if the drive is ready to communicate
        r = r_str() & 0xf0;
        // If not, drop back down and poll again
        if (!(r & 0x80))
            return 0;
    }
    return 1;
}

ppa_err_t ppa_command(uint8_t target, const uint8_t *cdb, uint8_t cdb_len,
                      void *buf, uint32_t len, ppa_dir_t dir, uint32_t timeout_ms,
                      uint8_t *status, uint8_t *msg, uint32_t *xferred) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    xfer_t x = {
        .ptr = buf,
        .residual = (int32_t)len,
        .dir = dir,
        .bulk = cdb[0] == 0x08 || cdb[0] == 0x28 || cdb[0] == 0x0a || cdb[0] == 0x2a,
    };
    int phase;

    failed = PPA_OK;
    *status = 0xff;
    *msg = 0;
    lpt_trace_mark('>', cdb[0]);

    // Phase 1 - Connected. Perform a sanity check for cable unplugged,
    // retrying for up to a second.
    phase = 1;
    absolute_time_t cable_tmo = make_timeout_time_ms(1000);
    for (;;) {
        ppa_connect();
        if (ppa_echo(NULL, NULL))
            break;
        if (time_reached(cable_tmo)) {
            ppa_fail(PPA_E_CABLE);
            goto out;
        }
        ppa_disconnect();
        sleep_ms(1);
    }

    // Phase 2 - We are now talking to the scsi bus
    phase = 2;
    if (!ppa_select(target)) {
        ppa_fail(PPA_E_NOSELECT);
        goto out;
    }

    // Phase 3 - Ready to accept a command
    phase = 3;
    for (;;) {
        w_ctr(0x0c);
        if (r_str() & 0x80)
            break;
        if (time_reached(deadline)) {
            ppa_fail(PPA_E_TIMEOUT);
            goto out;
        }
        sleep_us(PPA_POLL_US);
    }
    if (!ppa_send_command(cdb, cdb_len))
        goto out;

    // Phase 5 - Data transfer stage
    phase = 5;
    for (;;) {
        w_ctr(0x0c);
        if (r_str() & 0x80) {
            int rc = ppa_completion(&x);
            if (rc < 0)
                goto out;
            if (rc > 0)
                break;
        }
        if (time_reached(deadline)) {
            ppa_fail(PPA_E_TIMEOUT);
            goto out;
        }
        sleep_us(PPA_POLL_US);
    }

    // Phase 6 - Read status/message
    phase = 6;
    if (ppa_wait() != 0xf0) {   // Check for data overrun
        ppa_fail(PPA_E_PROTO);
        goto out;
    }
    uint8_t l = 0, h = 0;
    if (ppa_in(&l, 1)) {   // read status byte
        // Check for optional message byte. A timeout here does not fail
        // the command (Linux overwrites the result afterwards).
        ppa_err_t saved = failed;
        if (ppa_wait() == 0xf0)
            ppa_in(&h, 1);
        failed = saved;
        *status = l;
        *msg = h;
    }

out:
    if (phase > 1)
        ppa_disconnect();
    if (xferred)
        *xferred = len - (uint32_t)(x.residual > 0 ? x.residual : 0);
    lpt_trace_mark('<', (uint8_t)failed);
    return failed;
}
