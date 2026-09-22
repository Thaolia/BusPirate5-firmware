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

static float cfg_volts = DEFAULT_VOLTS;
static float cfg_ma = DEFAULT_MA;
static uint8_t cfg_uv = DEFAULT_UV;

raiden_power_mode_t raiden_power_mode(void) {
    return mode;
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
    psucmd_disable();
    powered = false;
}

static bool power_on(void) {
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

static void cmd_report(void) {
    rp_printf("Power mode:   %s\r\n",
              (mode == RAIDEN_POWER_EXTERNAL) ? "EXTERNAL" : "INTERNAL");
    rp_printf("Supply:       %s, %u mV\r\n",
              powered ? "ON" : "OFF", (unsigned)(cfg_volts * 1000.0f));
    if (powered) {
        rp_printf("Measured:     %u mV, %u mA\r\n",
                  (unsigned)psu_measure_vout(), (unsigned)psu_measure_current());
    }
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
    } else {
        rp_err("Unknown TARGET POWER sub-command '%s' "
               "(use ON/OFF/CYCLE/EXTERNAL/INTERNAL)", sub);
    }
}

void raiden_power_command(int argc, char* argv[]) {
    if (argc < 2) {
        rp_err("Usage: TARGET POWER <ON|OFF|CYCLE|EXTERNAL|INTERNAL>");
        return;
    }
    if (strcmp(argv[1], "POWER") == 0) {
        cmd_power(argc, argv);
        return;
    }
    // Everything else raiden's TARGET verb accepts -- RESET, SYNC, BL, the
    // chip-family selectors -- is out of scope here. Say so instead of
    // accepting it: a TARGET RESET that silently did nothing would look like a
    // reset that failed to open the chip.
    rp_err("TARGET %s is not implemented in the raiden binmode (only TARGET POWER)",
           argv[1]);
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
    bio_init();
    raiden_power_gate_idle();
    system_bio_update_purpose_and_label(true, RAIDEN_BIO_CROWBAR, BP_PIN_MODE, "CROW");
}

void raiden_power_deinit(void) {
    power_off();
    raiden_power_gate_idle();
    system_bio_update_purpose_and_label(false, RAIDEN_BIO_CROWBAR, BP_PIN_MODE, 0);
}
