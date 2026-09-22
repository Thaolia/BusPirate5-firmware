/**
 * @file raiden_binmode.c
 * @brief Binary mode speaking the raiden-pico command dialect.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pirate.h"
#include "system_config.h"
#include "usb_rx.h"
#include "usb_tx.h"

#include "raiden_binmode.h"
#include "raiden_clock.h"
#include "raiden_cmd.h"
#include "raiden_proto.h"

#define RAIDEN_VERSION "v0.1-bp5"

// raiden's own parser uses a 600-byte buffer. Nothing the host sends comes
// close: the longest command in src/nrf52/ is "SWD WRITE AP 1 0x04 0x...".
#define RAIDEN_LINE_MAX 128

const char raiden_binmode_name[] = "Raiden glitch/SWD (raiden dialect)";

static char line[RAIDEN_LINE_MAX];
static uint32_t line_len = 0;
static bool line_overflow = false;

bool raiden_parse_u32(const char* s, uint32_t* out) {
    if (s == NULL || *s == '\0') {
        return false;
    }
    int base = 10;
    if (s[0] == '0' && (s[1] == 'X' || s[1] == 'x')) {
        base = 16;
        s += 2;
        if (*s == '\0') {
            return false;
        }
    }
    uint32_t value = 0;
    for (; *s; s++) {
        uint32_t digit;
        if (*s >= '0' && *s <= '9') {
            digit = (uint32_t)(*s - '0');
        } else if (base == 16 && *s >= 'A' && *s <= 'F') {
            digit = (uint32_t)(*s - 'A') + 10u;
        } else if (base == 16 && *s >= 'a' && *s <= 'f') {
            digit = (uint32_t)(*s - 'a') + 10u;
        } else {
            return false;
        }
        if (value > (0xFFFFFFFFu - digit) / (uint32_t)base) {
            return false; // overflow: refuse rather than wrap into a valid-looking address
        }
        value = value * (uint32_t)base + digit;
    }
    *out = value;
    return true;
}

static void cmd_version(void) {
    rp_printf("Bus Pirate raiden-dialect binmode %s\r\n", RAIDEN_VERSION);
    // The clock is half the meaning of any PAUSE or WIDTH in a campaign log:
    // the same integer denotes a different instant on a different clk_sys.
    rp_printf("Clock: %u kHz, glitch step %u ps\r\n",
              (unsigned)(raiden_clock_hz() / 1000u), (unsigned)raiden_clock_step_ps());
}

static void cmd_status(void) {
    rp_send("=== System Status ===\r\n\r\n");
    rp_send("== System ==\r\n");
    rp_printf("Model:        Bus Pirate 5 (raiden dialect %s)\r\n", RAIDEN_VERSION);
    rp_printf("Clock:        %u kHz (step %u ps)\r\n",
              (unsigned)(raiden_clock_hz() / 1000u), (unsigned)raiden_clock_step_ps());
    raiden_glitch_status();
    raiden_power_status();
}

static void dispatch(int argc, char* argv[]) {
    const char* verb = argv[0];

    if (strcmp(verb, "VERSION") == 0) {
        cmd_version();
    } else if (strcmp(verb, "STATUS") == 0) {
        cmd_status();
    } else if (strcmp(verb, "SWD") == 0) {
        raiden_swd_command(argc, argv);
    } else if (strcmp(verb, "TARGET") == 0) {
        raiden_power_command(argc, argv);
    } else if (strcmp(verb, "SET") == 0 || strcmp(verb, "GET") == 0 ||
               strcmp(verb, "ARM") == 0 || strcmp(verb, "GLITCH") == 0 ||
               strcmp(verb, "TRIGGER") == 0 || strcmp(verb, "TRACE") == 0) {
        raiden_glitch_command(argc, argv);
    } else {
        // Never fall through to a default branch: an unrecognised verb that
        // silently no-ops would let a campaign keep shooting into the void.
        rp_err("Unknown command '%s'", verb);
    }
}

static void execute(char* buf) {
    char* argv[RAIDEN_MAX_ARGS];
    int argc = 0;

    for (char* p = buf; *p != '\0' && argc < RAIDEN_MAX_ARGS;) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        argv[argc++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') {
            if (*p >= 'a' && *p <= 'z') {
                *p = (char)(*p - 'a' + 'A'); // raiden upper-cases every token
            }
            p++;
        }
        if (*p != '\0') {
            *p++ = '\0';
        }
    }

    if (argc == 0) {
        return; // blank line: stay silent, see feed()
    }
    dispatch(argc, argv);
    // A dropped byte left a well-formed-looking but incomplete reply on the
    // wire. Say so: the host treats the literal word ERROR as failure, so this
    // turns a silent corruption into a shot the campaign refuses to trust.
    if (rp_take_truncated()) {
        rp_err("Response truncated: host stopped reading the binary interface");
    }
    rp_prompt();
}

static void feed(char c) {
    // The host sends CRLF (raiden_client.py writes c + "\r\n"). Treating both
    // terminators as end-of-line and staying SILENT on the resulting empty line
    // is what keeps one command from producing two replies -- a stray second
    // prompt could end the NEXT read early, before its data arrived.
    if (c == '\r' || c == '\n') {
        if (line_overflow) {
            line_overflow = false;
            line_len = 0;
            rp_err("Command too long (max %u)", (unsigned)(RAIDEN_LINE_MAX - 1));
            rp_prompt();
            return;
        }
        line[line_len] = '\0';
        line_len = 0;
        execute(line);
        return;
    }
    if (line_len >= RAIDEN_LINE_MAX - 1) {
        line_overflow = true; // keep eating until the terminator
        return;
    }
    line[line_len++] = c;
}

void raiden_binmode_setup(void) {
    line_len = 0;
    line_overflow = false;
    // Claim the binary FIFOs explicitly rather than trusting the defaults.
    // SUMP turns both off and hands USB straight to TinyUSB; if it ever exits
    // without restoring them, this mode would come up MUTE -- no error, no
    // reply, just a bench that never answers. Setting them here costs nothing
    // and removes a silent failure mode.
    system_config.binmode_usb_rx_queue_enable = true;
    system_config.binmode_usb_tx_queue_enable = true;
    raiden_clock_apply();
    raiden_power_init();
    raiden_swd_init();
    raiden_glitch_init();
}

void raiden_binmode_setup_message(void) {
    printf("Raiden dialect on the binary interface. Point the host client at the\r\n");
    printf("second CDC port ('Bus Pirate BIN'), not this terminal.\r\n");
    printf("Clock %u kHz, glitch step %u ps.\r\n",
           (unsigned)(raiden_clock_hz() / 1000u), (unsigned)raiden_clock_step_ps());
}

void raiden_binmode_service(void) {
    char c;
    // Bounded per call: binmode_service() is the cooperative slot of the main
    // loop, so it has to hand control back.
    for (uint32_t i = 0; i < 64; i++) {
        if (!bin_rx_fifo_try_get(&c)) {
            return;
        }
        feed(c);
    }
}

void raiden_binmode_cleanup(void) {
    raiden_glitch_deinit();
    raiden_swd_deinit();
    raiden_power_deinit();
    // Last, and never skipped: leaving the part overclocked would hand the
    // next mode a clock it never asked for.
    raiden_clock_restore();
}
