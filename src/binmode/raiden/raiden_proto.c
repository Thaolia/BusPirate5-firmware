/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_proto.c
 * @brief Wire-format primitives for the raiden-dialect binmode.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pirate.h"
#include "usb_tx.h"

#include "raiden_proto.h"

// One line of hex dump is 78 characters; STATUS prints longer single lines.
#define RP_FMT_MAX 256

// A full TX FIFO means the host has stopped reading. Spinning forever there is
// the firmware-side twin of pyserial's write_timeout=None, which froze a
// campaign with no message at all (measured 2026-09-21). We drop instead: a
// truncated reply makes the host time out and say so, a wedged firmware does
// not.
#define RP_PUT_TRIES 20000

// Set when rp_put() had to give up on a byte. A dropped byte corrupts the
// reply, and a corrupt reply that still looks well-formed is the one outcome
// this bench must never produce -- so the drop is reported instead of hidden.
static bool tx_dropped = false;

static void rp_put(char c) {
    for (uint32_t i = 0; i < RP_PUT_TRIES; i++) {
        if (bin_tx_fifo_try_put(c)) {
            return;
        }
        tight_loop_contents();
    }
    tx_dropped = true;
}

bool rp_take_truncated(void) {
    bool was = tx_dropped;
    tx_dropped = false;
    return was;
}

void rp_send(const char* s) {
    for (; *s; s++) {
        rp_put(*s);
    }
}

static void rp_vprintf(const char* fmt, va_list args) {
    char buf[RP_FMT_MAX];
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    if (n < 0) {
        return;
    }
    rp_send(buf);
}

void rp_printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    rp_vprintf(fmt, args);
    va_end(args);
}

void rp_ok(const char* fmt, ...) {
    va_list args;
    rp_send("OK: ");
    va_start(args, fmt);
    rp_vprintf(fmt, args);
    va_end(args);
    rp_send("\r\n");
}

void rp_err(const char* fmt, ...) {
    va_list args;
    rp_send("ERROR: ");
    va_start(args, fmt);
    rp_vprintf(fmt, args);
    va_end(args);
    rp_send("\r\n");
}

void rp_prompt(void) {
    rp_send("> ");
}

void rp_hexdump_begin(uint32_t addr, uint32_t nbytes) {
    rp_printf("Reading %u bytes from 0x%08X:\r\n", (unsigned)nbytes, (unsigned)addr);
}

void rp_hexdump_end(void) {
    rp_send("OK: Read complete\r\n");
}

void rp_hexdump_lines(uint32_t addr, const uint8_t* buf, uint32_t nbytes) {
    for (uint32_t i = 0; i < nbytes; i += 16) {
        rp_printf("0x%08X:", (unsigned)(addr + i));
        for (uint32_t j = i; j < i + 16 && j < nbytes; j++) {
            rp_printf(" %02X", buf[j]);
        }
        rp_send("  ");
        for (uint32_t j = i; j < i + 16 && j < nbytes; j++) {
            char c = (char)buf[j];
            if (c < 32 || c > 126) {
                c = '.';
            }
            rp_printf("%c", c);
        }
        rp_send("\r\n");
    }
}

void rp_words_line(uint32_t addr, const uint32_t* words, uint32_t nwords) {
    rp_printf("0x%08X:", (unsigned)addr);
    for (uint32_t i = 0; i < nwords; i++) {
        rp_printf(" %08X", (unsigned)words[i]);
    }
    rp_send("\r\n");
}

void rp_reg_dp(uint8_t addr, uint32_t value) {
    rp_ok("DP[0x%X] = 0x%08X", (unsigned)addr, (unsigned)value);
}

void rp_reg_ap(uint8_t apsel, uint8_t addr, uint32_t value) {
    if (apsel == 0) {
        rp_ok("AP[0x%02X] = 0x%08X", (unsigned)addr, (unsigned)value);
    } else {
        rp_ok("AP%u[0x%02X] = 0x%08X", (unsigned)apsel, (unsigned)addr, (unsigned)value);
    }
}
