/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_clock.c
 * @brief System-clock control for the raiden-dialect binmode.
 */
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pirate.h"

#include "raiden_clock.h"

// Frequency applied when the binmode starts. 0 = leave the part alone, and
// that is the default on purpose: an overclock nobody measured is exactly the
// kind of plausible default that produces a whole campaign of shots that were
// never really fired. Set it from the build once the board in hand has been
// walked up step by step and the ceiling written into docs/BP5_FACTS.md.
#ifndef RAIDEN_SYS_CLOCK_KHZ
#define RAIDEN_SYS_CLOCK_KHZ 0
#endif

#ifndef RAIDEN_CORE_MV
#define RAIDEN_CORE_MV 0
#endif

#define VREG_MV_MIN 850u
#define VREG_MV_MAX 1300u
#define VREG_MV_STEP 50u
// vreg_voltage encodes 0.85 V as 0b0110; each step above is +50 mV.
#define VREG_CODE_AT_MIN 6u

static uint32_t entry_khz = 0;
static uint32_t entry_core_mv = 0;
static bool entered = false;

static bool vreg_apply(uint32_t core_mv) {
    if (core_mv < VREG_MV_MIN || core_mv > VREG_MV_MAX) {
        return false;
    }
    uint32_t code = ((core_mv - VREG_MV_MIN) / VREG_MV_STEP) + VREG_CODE_AT_MIN;
    vreg_set_voltage((enum vreg_voltage)code);
    // The regulator needs a moment before the core may be clocked faster.
    busy_wait_ms(2);
    return true;
}

bool raiden_clock_set(uint32_t khz, uint32_t core_mv) {
    // Voltage first when going up, so the core is never fast AND undervolted.
    if (core_mv != 0 && !vreg_apply(core_mv)) {
        return false;
    }
    if (khz == 0) {
        return true;
    }
    return set_sys_clock_khz(khz, false);
}

void raiden_clock_apply(void) {
    if (!entered) {
        entry_khz = clock_get_hz(clk_sys) / 1000u;
        // The SDK exposes no getter for the current vreg setting, so the entry
        // voltage is the power-on default unless this module changed it.
        entry_core_mv = 1100u;
        entered = true;
    }
    if (RAIDEN_SYS_CLOCK_KHZ != 0) {
        (void)raiden_clock_set(RAIDEN_SYS_CLOCK_KHZ, RAIDEN_CORE_MV);
    }
}

void raiden_clock_restore(void) {
    if (!entered) {
        return;
    }
    // Clock down first, then voltage down: the reverse order would leave the
    // core briefly fast and undervolted.
    (void)set_sys_clock_khz(entry_khz, false);
    (void)vreg_apply(entry_core_mv);
    entered = false;
}

uint32_t raiden_clock_hz(void) {
    return clock_get_hz(clk_sys);
}

uint32_t raiden_clock_step_ps(void) {
    uint32_t hz = clock_get_hz(clk_sys);
    if (hz == 0) {
        return 0;
    }
    return (uint32_t)(1000000000000ull / (uint64_t)hz);
}
