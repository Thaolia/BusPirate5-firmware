/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_target.c
 * @brief Target family selector and the nRST pulse.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pirate.h"
#include "command_struct.h"
#include "system_config.h"
#include "pirate/bio.h"

#include "raiden_cmd.h"
#include "raiden_proto.h"
#include "raiden_swd.h"
#include "raiden_target.h"

/* Low time of the nRST pulse. The BAT32G135 datasheet asks for tRSL >= 10 us
 * (DS 6.6); ten milliseconds is four orders of magnitude of margin on a pulse
 * issued once per shot, and it costs nothing next to the 12 ms a supply cycle
 * takes. Short pulses are where a reset that did not take hides. */
#define RESET_LOW_MS 10u

/* Settling after release, for the target to finish booting before SWD CONNECT
 * clocks a line reset at it. */
#define RESET_SETTLE_MS 10u

static raiden_target_t family = RAIDEN_TARGET_NRF52;

raiden_target_t raiden_target_family(void) {
    return family;
}

void raiden_target_init(void) {
    family = RAIDEN_TARGET_NRF52;
}

/** Pulse nRST low, then release it.
 *
 * Released rather than driven high: RESETB wants ONE driver at a time, and on
 * BAT32G135 the datasheet has it tied to VDD through a resistor when nobody
 * drives it (DS p. 19). That pull-up is what brings the target back, and
 * leaving the pin released means the bench is not fighting it between shots.
 */
static void pulse_nrst(void) {
    // Drop the link first. The DP does not survive the reset, and a SWCLK or
    // SWDIO left driven feeds the target through its ESD diodes -- which is
    // how a reset stops being a reset without anything reporting a failure.
    raiden_swd_forget_connection();

    bio_put(RAIDEN_BIO_NRST, false); // value before direction: low at turn-on
    bio_output(RAIDEN_BIO_NRST);
    busy_wait_ms(RESET_LOW_MS);
    bio_input(RAIDEN_BIO_NRST);
    busy_wait_ms(RESET_SETTLE_MS);
}

static void cmd_reset(void) {
    if (family == RAIDEN_TARGET_NRF52) {
        // Not an error -- the pin really is pulsed, and on a validation target
        // that is sometimes what is wanted. But a campaign that reached for
        // this INSTEAD of a supply cut would fire every shot against a debug
        // port that was never rearmed, and score every one of them.
        rp_printf("[TARGET] WARNING: on nRF52 a pin reset does NOT rearm the debug "
                  "port (PS 5.3.6.8) -- only POR or brownout does\r\n");
    }
    pulse_nrst();
    rp_ok("nRST pulsed low %u ms on BIO%u", (unsigned)RESET_LOW_MS,
          (unsigned)RAIDEN_BIO_NRST);
}

static void cmd_family(raiden_target_t f, const char* label) {
    // Selecting a family drops the link on purpose. The two families disagree
    // about what a reset does and about whether the AHB-AP can be trusted, so
    // carrying a connection across the switch would carry assumptions with it.
    if (f != family) {
        raiden_swd_forget_connection();
    }
    family = f;
    rp_ok("Target family %s", label);
}

void raiden_target_command(int argc, char* argv[]) {
    if (argc < 2) {
        rp_err("Usage: TARGET <BAT32|NRF52|RESET|POWER ...>");
        return;
    }
    const char* sub = argv[1];

    if (strcmp(sub, "POWER") == 0) {
        raiden_power_command(argc, argv);
    } else if (strcmp(sub, "BAT32") == 0) {
        cmd_family(RAIDEN_TARGET_BAT32, "BAT32G135 (Cortex-M0+)");
    } else if (strcmp(sub, "NRF52") == 0) {
        cmd_family(RAIDEN_TARGET_NRF52, "nRF52 (Cortex-M4)");
    } else if (strcmp(sub, "RESET") == 0) {
        cmd_reset();
    } else {
        // Never a default branch. raiden's TARGET verb also takes SYNC, BL and
        // a dozen chip-family selectors; none of them is implemented here, and
        // one that was quietly accepted would let a campaign believe it had
        // configured something.
        rp_err("TARGET %s is not implemented in the raiden binmode "
               "(BAT32, NRF52, RESET, POWER)", sub);
    }
}

void raiden_target_status(void) {
    rp_send("\r\n== Family ==\r\n");
    if (family == RAIDEN_TARGET_BAT32) {
        rp_send("Family:       BAT32G135 (Cortex-M0+)\r\n");
        rp_send("Rearm:        TARGET RESET rearms the debug port -- no supply cut per shot\r\n");
    } else {
        rp_send("Family:       nRF52 (Cortex-M4)\r\n");
        rp_send("Rearm:        POR or brownout ONLY -- every shot needs a real supply cut\r\n");
    }
}
