// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2026 Florian Frankenberger (port)
// Iomega PPA3 (ZIP 100 parallel) transport, ported from Linux drivers/scsi/ppa.c.
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PPA_OK = 0,
    PPA_E_CABLE,      // nFault did not echo nAutoFeed after connect
    PPA_E_NOSELECT,   // target did not answer selection (DID_NO_CONNECT)
    PPA_E_TIMEOUT,    // ppa_wait or command deadline expired (DID_TIME_OUT)
    PPA_E_PROTO,      // unexpected phase (DID_ERROR)
    PPA_E_BUSBUSY,    // transfer failed mid-way (DID_BUS_BUSY)
    PPA_E_OVERRUN,    // drive wanted more data than the buffer holds
    PPA_E_DIR,        // drive wanted the opposite data direction
    PPA_E_NODEV,      // device_check found nothing
} ppa_err_t;

typedef enum { PPA_DIR_NONE, PPA_DIR_IN, PPA_DIR_OUT } ppa_dir_t;

const char *ppa_err_str(ppa_err_t e);

// Low-level sequences, exposed for the bring-up shell.
void ppa_connect(void);
void ppa_disconnect(void);
void ppa_reset_pulse(void);
bool ppa_echo(uint8_t *str_e, uint8_t *str_c);   // cable check, after connect

// ppa_init(): port reset, cable check and SCSI bus reset (ppa_init in Linux).
ppa_err_t ppa_init(void);
// Scan SCSI IDs 0-7 with TEST UNIT READY. Returns the ID or -ppa_err_t.
int ppa_device_check(void);
// Recovery after a failed command (ppa_reset in Linux).
void ppa_reset(void);

// One full SCSI command (ppa_engine phases 1-6), blocking.
// status/msg are the SCSI status and message bytes. xferred counts data bytes.
ppa_err_t ppa_command(uint8_t target, const uint8_t *cdb, uint8_t cdb_len,
                      void *buf, uint32_t len, ppa_dir_t dir, uint32_t timeout_ms,
                      uint8_t *status, uint8_t *msg, uint32_t *xferred);

// Status nibble last seen by the protocol code (for diagnostics).
uint8_t ppa_last_phase(void);
