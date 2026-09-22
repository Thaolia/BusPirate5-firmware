/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_power.c
 * @brief Target supply and power mode for the raiden-dialect binmode.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pirate.h"
#include "command_struct.h"
#include "system_config.h"
#include "pirate/amux.h"
#include "pirate/bio.h"
#include "pirate/psu.h"
#include "commands/global/w_psu.h"

#include "raiden_clock.h"
#include "raiden_cmd.h"
#include "raiden_power.h"
#include "raiden_proto.h"

#define DEFAULT_VOLTS 3.3f
// 0 mA means "no fuse", and that is deliberate as the default on a glitch
// bench. See cmd_on() for the full reasoning -- it is a real trade, not an
// oversight.
#define DEFAULT_MA 0.0f
// 100 % disables undervoltage protection. NOT optional here: a voltage-glitch
// campaign IS a sequence of undervoltage events, so an armed protection would
// trip on every shot and the bench would return no_dp shots that look like a
// chip refusing to open.
#define DEFAULT_UV 100u

#define DEFAULT_OFF_MS 50u
#define MAX_OFF_MS 10000u

#define VOLTS_MIN_MV 800u
#define VOLTS_MAX_MV 5000u
#define MA_MAX 500u

static raiden_power_mode_t mode = RAIDEN_POWER_INTERNAL;
static bool gate_active_high = true;
static bool powered = false;

static raiden_supply_t supply = RAIDEN_SUPPLY_PSU;
// No default polarity for the MOSFET enable, and that is the whole point.
// A reversed enable leaves the target powered through every single shot: the
// campaign then logs tens of thousands of reproducible no_dp with not one
// fault actually attempted, and nothing anywhere says so. power.py refuses a
// default for --cmd-off/--cmd-on for exactly this reason; so does this.
static bool supply_active_high = false;
static bool supply_polarity_known = false;

// 0.8 V is the lowest the programmable supply will produce, so the firmware's
// own preflight calls anything under 790 mV "not powered" (ui_help.c).
#define VREF_MIN_MV 790u

static float cfg_volts = DEFAULT_VOLTS;
static float cfg_ma = DEFAULT_MA;
static uint8_t cfg_uv = DEFAULT_UV;

raiden_power_mode_t raiden_power_mode(void) {
    return mode;
}

raiden_supply_t raiden_power_supply(void) {
    return supply;
}

bool raiden_power_vref_ok(uint32_t* mv_out) {
    amux_sweep();
    uint32_t mv = hw_adc_voltage[HW_ADC_MUX_VREF_VOUT];
    if (mv_out != NULL) {
        *mv_out = mv;
    }
    return mv >= VREF_MIN_MV;
}

/** Park the supply enable at its OFF level, driven rather than floating.
 *
 * Driven, because a floating enable is not an off enable -- it is a gate
 * waiting for the first bit of coupling to decide for it.
 */
static void supply_idle(void) {
    bio_output(RAIDEN_BIO_SUPPLY);
    bio_put(RAIDEN_BIO_SUPPLY, !supply_active_high);
}

static void supply_assert(bool on) {
    bio_output(RAIDEN_BIO_SUPPLY);
    bio_put(RAIDEN_BIO_SUPPLY, on == supply_active_high);
}

bool raiden_power_gate_active_high(void) {
    return gate_active_high;
}

void raiden_power_gate_idle(void) {
    bio_output(RAIDEN_BIO_CROWBAR);
    // Idle is the level that keeps the MOSFET OFF: low for an active-high
    // gate, high for an active-low one. A floating gate is not idle -- it is
    // a crowbar waiting to close on its own.
    bio_put(RAIDEN_BIO_CROWBAR, !gate_active_high);
}

static const char* psu_error_text(uint32_t code) {
    switch (code) {
        case PSU_ERROR_FUSE_TRIPPED:
            return "current limit tripped (target draws more than the fuse)";
        case PSU_ERROR_VOUT_LOW:
            return "VOUT below the undervoltage limit";
        case PSU_ERROR_BACKFLOW:
            return "backflow -- another source is pushing the rail";
        default:
            return "unknown PSU failure";
    }
}

static void power_off(void) {
    if (supply == RAIDEN_SUPPLY_MOSFET) {
        supply_assert(false);
    } else {
        psucmd_disable();
    }
    powered = false;
}

static bool power_on(void) {
    if (supply == RAIDEN_SUPPLY_MOSFET) {
        if (!supply_polarity_known) {
            rp_err("Supply polarity unknown: TARGET POWER SOURCE MOSFET <AHIGH|ALOW>");
            return false;
        }
        // Checked HERE and not on the PSU path, because on the PSU path this
        // rail is the thing being switched on -- testing it first would always
        // fail. On the MOSFET path the rail comes from upstream of the switch
        // and must already be up, or the level shifters cannot drive the very
        // pin about to be asserted.
        uint32_t mv = 0;
        if (!raiden_power_vref_ok(&mv)) {
            rp_err("VOUT/VREF reads %u mV: the I/O buffers have no rail, so BIO%u "
                   "drives nothing. Wire VREF UPSTREAM of the MOSFET, never to the "
                   "switched side", (unsigned)mv, (unsigned)RAIDEN_BIO_SUPPLY);
            return false;
        }
        supply_assert(true);
        powered = true;
        return true;
    }
    // current_limit_override == true DISABLES limiting. The parameter is named
    // current_limit_enabled in psu.h and current_limit_override in w_psu.c;
    // psu.c:308 and its use at psu.c:320/337 settle it -- true skips the fuse
    // AND the 500 ms settling wait. Getting this backwards costs 500 ms a shot
    // and trips on every glitch, with no message naming the cause.
    bool no_fuse = (cfg_ma <= 0.0f);
    uint32_t r = psucmd_enable(cfg_volts, cfg_ma, no_fuse, cfg_uv);
    if (r != PSU_OK) {
        rp_err("PSU: %s", psu_error_text(r));
        powered = false;
        return false;
    }
    powered = true;
    return true;
}

static void cmd_on(int argc, char* argv[]) {
    // TARGET POWER ON [<volts> [<mA> [<uv%>]]]
    // raiden has no voltage argument -- its supply is a fixed 3.3 V GPIO. Here
    // the supply is programmable, and the campaign driver passes these strings
    // verbatim through --cmd-on, so the operator sets the rail from there.
    if (argc >= 4) {
        uint32_t mv;
        // Millivolts, not a float: "2.2" through a scanf on this toolchain is
        // one more thing to get subtly wrong. TARGET POWER ON 2200 is exact.
        if (!raiden_parse_u32(argv[3], &mv) || mv < VOLTS_MIN_MV || mv > VOLTS_MAX_MV) {
            rp_err("Usage: TARGET POWER ON [<mV> [<mA> [<uv%%>]]] (mV %u-%u)",
                   (unsigned)VOLTS_MIN_MV, (unsigned)VOLTS_MAX_MV);
            return;
        }
        cfg_volts = (float)mv / 1000.0f;
    }
    if (argc >= 5) {
        uint32_t ma;
        if (!raiden_parse_u32(argv[4], &ma) || ma > MA_MAX) {
            rp_err("Current limit out of range (0-%u mA, 0 disables the fuse)", (unsigned)MA_MAX);
            return;
        }
        cfg_ma = (float)ma;
    }
    if (argc >= 6) {
        uint32_t uv;
        if (!raiden_parse_u32(argv[5], &uv) || uv < 1u || uv > 100u) {
            rp_err("Undervoltage percent out of range (1-100, 100 disables)");
            return;
        }
        cfg_uv = (uint8_t)uv;
    }

    if (!power_on()) {
        return;
    }
    rp_ok("Target power ON, %u mV, %s, undervoltage %u%%",
          (unsigned)(cfg_volts * 1000.0f),
          (cfg_ma <= 0.0f) ? "no fuse" : "fuse armed",
          (unsigned)cfg_uv);
}

static void cmd_cycle(int argc, char* argv[]) {
    uint32_t off_ms = DEFAULT_OFF_MS;
    if (argc >= 4 && (!raiden_parse_u32(argv[3], &off_ms) || off_ms > MAX_OFF_MS)) {
        rp_err("Off time out of range (0-%u ms)", (unsigned)MAX_OFF_MS);
        return;
    }
    power_off();
    // The rail has to actually reach zero: a reservoir capacitor holds it up
    // well after the supply is cut, and without a real POR the debug port does
    // not rearm. Measure the fall time on a scope -- do not assume this value.
    busy_wait_ms(off_ms);
    if (!power_on()) {
        return;
    }
    rp_ok("Target power cycled, off %u ms", (unsigned)off_ms);
}

static void cmd_mode_external(int argc, char* argv[]) {
    if (argc >= 4) {
        if (strcmp(argv[3], "AHIGH") == 0) {
            gate_active_high = true;
        } else if (strcmp(argv[3], "ALOW") == 0) {
            gate_active_high = false;
        } else {
            rp_err("Usage: TARGET POWER EXTERNAL [AHIGH|ALOW]");
            return;
        }
    }
    mode = RAIDEN_POWER_EXTERNAL;
    raiden_power_gate_idle();
    // The host requires the literal words "Power mode" in this reply
    // (src/nrf52/campaign.py checks for them before it will start a campaign).
    rp_ok("Power mode EXTERNAL (PSU VOUT supply, BIO%u crowbar gate %s/idle-%s)",
          (unsigned)RAIDEN_BIO_CROWBAR,
          gate_active_high ? "active-HIGH" : "active-LOW",
          gate_active_high ? "LOW" : "HIGH");
}

static void cmd_mode_internal(void) {
    mode = RAIDEN_POWER_INTERNAL;
    raiden_power_gate_idle();
    rp_ok("Power mode INTERNAL (PSU VOUT supply, no crowbar gate driven)");
}

static void cmd_source(int argc, char* argv[]) {
    if (argc < 4) {
        rp_printf("Supply source: %s\r\n",
                  (supply == RAIDEN_SUPPLY_MOSFET) ? "MOSFET" : "PSU");
        return;
    }
    if (strcmp(argv[3], "PSU") == 0) {
        if (supply != RAIDEN_SUPPLY_PSU) {
            // Cut the outgoing source BEFORE switching. Two sources on one rail
            // is what the Bus Pirate names "backflow", and on a glitch bench it
            // would also mean the cut never actually cuts.
            power_off();
            supply = RAIDEN_SUPPLY_PSU;
        }
        rp_ok("Supply source PSU (VOUT, %u mV, %s)",
              (unsigned)(cfg_volts * 1000.0f),
              (cfg_ma <= 0.0f) ? "no fuse" : "fuse armed");
        return;
    }
    if (strcmp(argv[3], "MOSFET") != 0) {
        rp_err("Unknown supply source '%s' (use PSU or MOSFET)", argv[3]);
        return;
    }
    // Polarity is REQUIRED, never defaulted: a reversed enable leaves the target
    // powered through every shot, and a campaign of reproducible no_dp with no
    // fault attempted looks exactly like a chip that refuses to open.
    if (argc < 5) {
        rp_err("TARGET POWER SOURCE MOSFET needs its polarity: AHIGH (inverter "
               "topology, pin HIGH = target powered) or ALOW. No default -- a "
               "wrong one would not be visible anywhere");
        return;
    }
    bool want_high;
    if (strcmp(argv[4], "AHIGH") == 0) {
        want_high = true;
    } else if (strcmp(argv[4], "ALOW") == 0) {
        want_high = false;
    } else {
        rp_err("Unknown polarity '%s' (use AHIGH or ALOW)", argv[4]);
        return;
    }
    if (supply != RAIDEN_SUPPLY_MOSFET) {
        power_off(); // still the PSU here: cut it before handing the rail over
    }
    supply = RAIDEN_SUPPLY_MOSFET;
    supply_active_high = want_high;
    supply_polarity_known = true;
    supply_idle();
    uint32_t mv = 0;
    bool vref = raiden_power_vref_ok(&mv);
    rp_ok("Supply source MOSFET (BIO%u enable, active-%s, idle-%s), VREF %u mV%s",
          (unsigned)RAIDEN_BIO_SUPPLY,
          want_high ? "HIGH" : "LOW",
          want_high ? "LOW" : "HIGH",
          (unsigned)mv,
          vref ? "" : " -- TOO LOW: the I/O buffers have no rail, wire VREF "
                      "UPSTREAM of the MOSFET");
}

static void cmd_report(void) {
    rp_printf("Power mode:   %s\r\n",
              (mode == RAIDEN_POWER_EXTERNAL) ? "EXTERNAL" : "INTERNAL");
    if (supply == RAIDEN_SUPPLY_MOSFET) {
        rp_printf("Supply src:   MOSFET on BIO%u, %s\r\n",
                  (unsigned)RAIDEN_BIO_SUPPLY,
                  supply_polarity_known ? (supply_active_high ? "active-HIGH" : "active-LOW")
                                        : "POLARITY NOT SET");
        rp_printf("Supply:       %s\r\n", powered ? "ON" : "OFF");
    } else {
        rp_printf("Supply src:   PSU on VOUT\r\n");
        rp_printf("Supply:       %s, %u mV\r\n",
                  powered ? "ON" : "OFF", (unsigned)(cfg_volts * 1000.0f));
        if (powered) {
            rp_printf("Measured:     %u mV, %u mA\r\n",
                      (unsigned)psu_measure_vout(), (unsigned)psu_measure_current());
        }
    }
    // Always reported, both sources: this rail powers the I/O buffers, so a bench
    // that reads low here drives nothing at all -- and says nothing about it.
    uint32_t mv = 0;
    bool vref = raiden_power_vref_ok(&mv);
    rp_printf("VOUT/VREF:    %u mV%s\r\n", (unsigned)mv,
              vref ? "" : "  (TOO LOW -- I/O buffers unpowered)");
}

static void cmd_power(int argc, char* argv[]) {
    if (argc < 3) {
        cmd_report();
        return;
    }
    const char* sub = argv[2];
    if (strcmp(sub, "ON") == 0) {
        cmd_on(argc, argv);
    } else if (strcmp(sub, "OFF") == 0) {
        power_off();
        rp_ok("Target power OFF");
    } else if (strcmp(sub, "CYCLE") == 0) {
        cmd_cycle(argc, argv);
    } else if (strcmp(sub, "EXTERNAL") == 0 || strcmp(sub, "EXT") == 0) {
        cmd_mode_external(argc, argv);
    } else if (strcmp(sub, "INTERNAL") == 0 || strcmp(sub, "INT") == 0) {
        cmd_mode_internal();
    } else if (strcmp(sub, "SOURCE") == 0) {
        cmd_source(argc, argv);
    } else {
        rp_err("Unknown TARGET POWER sub-command '%s' "
               "(use ON/OFF/CYCLE/EXTERNAL/INTERNAL/SOURCE)", sub);
    }
}

void raiden_power_command(int argc, char* argv[]) {
    if (argc < 2) {
        rp_err("Usage: TARGET POWER <ON|OFF|CYCLE|EXTERNAL|INTERNAL|SOURCE>");
        return;
    }
    // raiden_target.c routes TARGET and only forwards POWER here; anything
    // else it refuses by name. Kept as an assertion rather than as a second
    // opinion: two modules answering for the same verb is how they drift.
    if (strcmp(argv[1], "POWER") != 0) {
        rp_err("TARGET %s did not belong to raiden_power (internal routing bug)",
               argv[1]);
        return;
    }
    cmd_power(argc, argv);
}

void raiden_power_status(void) {
    rp_send("\r\n== Target ==\r\n");
    cmd_report();
}

void raiden_power_init(void) {
    mode = RAIDEN_POWER_INTERNAL;
    gate_active_high = true;
    powered = false;
    cfg_volts = DEFAULT_VOLTS;
    cfg_ma = DEFAULT_MA;
    cfg_uv = DEFAULT_UV;
    supply = RAIDEN_SUPPLY_PSU;
    supply_active_high = false;
    supply_polarity_known = false;
    bio_init();
    raiden_power_gate_idle();
    // The enable stays an INPUT until a polarity is declared. Driving it before
    // then would pick a level at random, and on this pin one of the two levels
    // is "target powered".
    bio_input(RAIDEN_BIO_SUPPLY);
    system_bio_update_purpose_and_label(true, RAIDEN_BIO_CROWBAR, BP_PIN_MODE, "CROW");
    system_bio_update_purpose_and_label(true, RAIDEN_BIO_SUPPLY, BP_PIN_MODE, "VEN");
}

void raiden_power_deinit(void) {
    power_off();
    raiden_power_gate_idle();
    // Park the enable at OFF while we still know the polarity, and only then
    // release it: the external pull-down takes over and holds the target down,
    // which is where hardware and firmware agree.
    if (supply_polarity_known) {
        supply_idle();
    }
    bio_input(RAIDEN_BIO_SUPPLY);
    system_bio_update_purpose_and_label(false, RAIDEN_BIO_CROWBAR, BP_PIN_MODE, 0);
    system_bio_update_purpose_and_label(false, RAIDEN_BIO_SUPPLY, BP_PIN_MODE, 0);
}
