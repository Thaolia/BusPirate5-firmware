/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_glitch.c
 * @brief Glitch pulse and trigger for the raiden-dialect binmode.
 *
 * One PIO state machine on PIO_MODE_PIO drives the whole shot: trigger edge,
 * delay loop, assert, width loop, deassert. The program is in
 * raiden_glitch.pio; this file owns the pin handover, the arming protocol and
 * the command surface.
 *
 * What this file does NOT do: it never sets the gate's idle level or its
 * polarity by itself -- both come from raiden_power.c. It borrows the gate pad
 * from the CPU at ARM ON and gives it back the moment the shot is accounted
 * for, so the resting state of the crowbar is always CPU-driven and idle.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pirate.h"
#include "command_struct.h"
#include "system_config.h"
#include "pirate/bio.h"
#include "pio_config.h"
#include "raiden_glitch.pio.h"

#include "raiden_clock.h"
#include "raiden_cmd.h"
#include "raiden_glitch.h"
#include "raiden_power.h"
#include "raiden_proto.h"

// pio0 SM0 and SM1 belong to the logic analyser and the RGB LED (pirate.h);
// PIO_MODE_PIO is the instance reserved for modes, and this engine takes one
// state machine of it. SWD is bit-banged precisely so it needs none.
#define GLITCH_SM 0u

// Ceiling for the busy wait of a manual GLITCH. binmode_service() is the
// cooperative slot of the main loop: it has to hand control back. A PAUSE that
// would exceed this is refused up front rather than silently truncated.
#define MANUAL_MAX_US 500000u

// Recorded, never acted upon in this build -- see cmd_trace().
#define TRACE_SAMPLES_MAX 4096u

static uint32_t g_pause = 0;
static uint32_t g_width = 0;
static uint32_t g_gap = 0;
static uint32_t g_count = 1;

static uint32_t g_shots = 0;
static bool g_armed = false;
static raiden_trigger_t g_edge = RAIDEN_TRIGGER_NONE;

static raiden_trace_state_t g_trace = RAIDEN_TRACE_IDLE;
static uint32_t g_trace_samples = 0;
static uint32_t g_trace_pre = 0;

static struct _pio_config g_pio;
static bool g_program_loaded = false;
static raiden_trigger_t g_program_edge = RAIDEN_TRIGGER_RISING;

/* ------------------------------------------------------------------ naming -- */

static const char* edge_name(raiden_trigger_t edge) {
    switch (edge) {
        case RAIDEN_TRIGGER_RISING:
            return "RISING";
        case RAIDEN_TRIGGER_FALLING:
            return "FALLING";
        default:
            return "NONE";
    }
}

static const char* trace_name(void) {
    switch (g_trace) {
        case RAIDEN_TRACE_RUNNING:
            return "RUNNING";
        case RAIDEN_TRACE_COMPLETE:
            return "COMPLETE";
        default:
            return "IDLE";
    }
}

/* ------------------------------------------------------- PIO program memory -- */

static uint gate_gpio(void) {
    // One BIO costs two RP2040 GPIOs: the buffered data pin and the buffer
    // direction pin. Only the data pin is ever handed to the PIO; the direction
    // pin stays on SIO, set outward by raiden_power_gate_idle().
    return bio2bufiopin[RAIDEN_BIO_CROWBAR];
}

static uint trigger_gpio(void) {
    return bio2bufiopin[RAIDEN_BIO_TRIGGER];
}

/** Make the program for @p edge the resident one. false = no room. */
static bool program_load(raiden_trigger_t edge) {
    // TRIGGER NONE still needs a resident program for the manual GLITCH entry
    // point, and that entry has no `wait`, so the polarity does not matter.
    raiden_trigger_t want =
        (edge == RAIDEN_TRIGGER_FALLING) ? RAIDEN_TRIGGER_FALLING : RAIDEN_TRIGGER_RISING;

    if (g_program_loaded && g_program_edge == want) {
        return true;
    }
    // Only one polarity is ever resident. Two 16-instruction programs DO fit in
    // the 32 slots of a PIO -- the point is not that the second add would fail,
    // it is that it would leave zero headroom, so any other user of
    // PIO_MODE_PIO (a protocol mode selected alongside this binmode) could no
    // longer load anything. Releasing the unused polarity keeps the cost at 16.
    // pio_can_add_program() below is what turns "no room" into an error line
    // instead of the hard assert pio_add_program() raises on a full PIO.
    if (g_program_loaded) {
        pio_remove_program(g_pio.pio, g_pio.program, g_pio.offset);
        g_program_loaded = false;
        g_pio.program = NULL;
    }
    const pio_program_t* prog = (want == RAIDEN_TRIGGER_FALLING) ? &raiden_glitch_falling_program
                                                                 : &raiden_glitch_rising_program;
    if (!pio_can_add_program(g_pio.pio, prog)) {
        return false;
    }
    g_pio.program = prog;
    g_pio.offset = (uint)pio_add_program(g_pio.pio, prog);
    g_program_edge = want;
    g_program_loaded = true;
    return true;
}

static uint entry_armed(void) {
    return g_pio.offset + ((g_program_edge == RAIDEN_TRIGGER_FALLING)
                               ? raiden_glitch_falling_offset_armed
                               : raiden_glitch_rising_offset_armed);
}

static uint entry_now(void) {
    return g_pio.offset + ((g_program_edge == RAIDEN_TRIGGER_FALLING)
                               ? raiden_glitch_falling_offset_now
                               : raiden_glitch_rising_offset_now);
}

static pio_sm_config program_config(void) {
    return (g_program_edge == RAIDEN_TRIGGER_FALLING)
               ? raiden_glitch_falling_program_get_default_config(g_pio.offset)
               : raiden_glitch_rising_program_get_default_config(g_pio.offset);
}

/* ------------------------------------------------------------ gate handover -- */

/** Pin the pad at its idle level through the IO bank, whoever owns it. */
static void gate_freeze_idle(void) {
    // The output override acts after the function select, so the pad holds this
    // level while the peripheral underneath is swapped. Without it there is a
    // window in every SIO <-> PIO handover where the pad is driven by the new
    // owner with the old polarity -- and on an active-LOW gate that window is a
    // closed crowbar on the target's rail.
    gpio_set_outover(gate_gpio(),
                     raiden_power_gate_active_high() ? GPIO_OVERRIDE_LOW : GPIO_OVERRIDE_HIGH);
}

static void gate_to_pio(void) {
    uint pin = gate_gpio();
    raiden_power_gate_idle(); // idle level + buffer outward, still CPU-owned
    gate_freeze_idle();
    pio_sm_set_pins_with_mask(g_pio.pio, GLITCH_SM, 0, 1u << pin);
    (void)pio_sm_set_consecutive_pindirs(g_pio.pio, GLITCH_SM, pin, 1, true);
    pio_gpio_init(g_pio.pio, pin);
    // Release the freeze straight into the polarity the program expects. The
    // program always asserts with `set pins, 1`; an active-LOW gate is obtained
    // by inverting the pad, not by a second pair of PIO programs. At this
    // instant the PIO output register holds 0, so the pad does not move.
    gpio_set_outover(pin, raiden_power_gate_active_high() ? GPIO_OVERRIDE_NORMAL
                                                          : GPIO_OVERRIDE_INVERT);
}

static void gate_to_cpu(void) {
    uint pin = gate_gpio();
    gate_freeze_idle();
    // bio_output() sets the direction but does NOT reselect SIO -- only
    // bio_init() does. Without this line raiden_power_gate_idle() would write a
    // level that never reaches a pad still owned by the PIO, and deinit would
    // park nothing at all while reporting success.
    gpio_set_function(pin, GPIO_FUNC_SIO);
    raiden_power_gate_idle();
    gpio_set_outover(pin, GPIO_OVERRIDE_NORMAL);
}

/* ------------------------------------------------------------ state machine -- */

static void sm_stop(void) {
    if (!g_program_loaded) {
        return;
    }
    pio_sm_set_enabled(g_pio.pio, GLITCH_SM, false);
    pio_sm_restart(g_pio.pio, GLITCH_SM);
    pio_sm_clear_fifos(g_pio.pio, GLITCH_SM);
}

/** Load the program, hand over the pins, push the parameters, run.
 *
 * Every arming path goes through here, unconditionally: stop, re-init the PC,
 * push, enable. Re-enabling a state machine without setting its PC would resume
 * it at whatever instruction it wrapped to, and the pulse would leave at an
 * instant nobody chose.
 */
static void sm_start(uint initial_pc, bool drive_gate, uint32_t pause, uint32_t width) {
    sm_stop();
    if (drive_gate) {
        gate_to_pio();
    } else {
        gate_to_cpu();
    }

    pio_sm_config c = program_config();
    // Divider exactly 1.0 -- and 1.0f is exactly representable in the 16.8
    // divider, so nothing is rounded. One PIO instruction is then one clk_sys
    // cycle, which makes a PAUSE unit a clk_sys cycle at ANY frequency. That is
    // a stronger property than deriving a divider from clock_get_hz(): the
    // dependency on the real clock lives where it belongs, in
    // raiden_clock_step_ps(), through which every reported TIME figure passes.
    sm_config_set_clkdiv(&c, 1.0f);
    sm_config_set_in_pins(&c, trigger_gpio());
    // SET_COUNT 0 in trace mode: `set pins` then reaches no pin at all. Together
    // with the pad being left on SIO by gate_to_cpu(), that is two independent
    // reasons why no pulse can leave during a trace. One reason would be a
    // promise; two is a guarantee.
    sm_config_set_set_pins(&c, gate_gpio(), drive_gate ? 1u : 0u);

    // The trigger pad stays on SIO: PIO reads any GPIO regardless of its
    // function select, and leaving it on SIO keeps bio_get() honest on BIO1.
    (void)pio_sm_set_consecutive_pindirs(g_pio.pio, GLITCH_SM, trigger_gpio(), 1, false);

    pio_sm_init(g_pio.pio, GLITCH_SM, initial_pc, &c);
    pio_sm_put(g_pio.pio, GLITCH_SM, pause);
    pio_sm_put(g_pio.pio, GLITCH_SM, width);
    pio_sm_set_enabled(g_pio.pio, GLITCH_SM, true);
}

/** Account for a shot the state machine has already fired.
 *
 * The witness is the word the program pushes after deasserting: a non-empty RX
 * FIFO means the program reached its end. The TX FIFO could NOT serve -- both
 * `pull`s drain at ARM time, before the trigger, so an empty TX FIFO proves
 * nothing at all.
 *
 * Upstream raiden refreshes this flag from its main loop
 * (glitch_update_flags()), which is why its bare ARM lags. This binmode has no
 * such hook: raiden_binmode_service() only feeds the parser. So the refresh
 * happens here, on every verb and before STATUS prints. Leaving it out would
 * make ARM answer ARMED for ever after the first shot, and a campaign driven by
 * --fire-probe arm would record every single shot as never fired.
 */
static void refresh(void) {
    if (!g_program_loaded || pio_sm_is_rx_fifo_empty(g_pio.pio, GLITCH_SM)) {
        return;
    }
    if (g_armed) {
        g_armed = false;
        g_shots++;
        // Pad back to the CPU BEFORE the state machine is stopped, always, and
        // in that order. gate_to_cpu() pins the pad at idle as its very first
        // action, whereas disabling a state machine leaves the PIO holding
        // whatever level it last wrote. Here the witness proves `set pins, 0`
        // already ran, so the level is idle either way -- but the same two
        // calls appear on the manual GLITCH timeout path, where the state
        // machine really can be stopped mid-pulse. One order for both.
        gate_to_cpu();
        sm_stop();
    } else if (g_trace == RAIDEN_TRACE_RUNNING) {
        g_trace = RAIDEN_TRACE_COMPLETE;
        sm_stop();
    } else {
        pio_sm_clear_fifos(g_pio.pio, GLITCH_SM);
    }
}

/* ------------------------------------------------------------- SET  /  GET -- */

static void report_param(const char* name, uint32_t cycles) {
    // Cycles are reported as cycles. The picosecond figure is informative
    // only: this project refuses to convert a cycle into a time in a campaign
    // log, because the conversion depends on a clk_sys that differs between
    // benches.
    uint64_t ps = (uint64_t)cycles * (uint64_t)raiden_clock_step_ps();
    rp_ok("%s set to %u cycles (%u ps)", name, (unsigned)cycles, (unsigned)ps);
}

static void cmd_set(int argc, char* argv[]) {
    if (argc < 3) {
        rp_err("Usage: SET <PAUSE|WIDTH|GAP|COUNT> <value>");
        return;
    }
    if (g_armed) {
        // The state machine latched PAUSE and WIDTH into X and Y at ARM ON. A
        // SET accepted now would not reach the pending shot, and the campaign
        // log would carry parameters that were never fired.
        rp_err("SET refused while armed: the pending shot latched its parameters at ARM ON");
        return;
    }
    uint32_t value;
    if (!raiden_parse_u32(argv[2], &value)) {
        rp_err("Invalid value '%s' for SET %s", argv[2], argv[1]);
        return;
    }
    if (strcmp(argv[1], "PAUSE") == 0) {
        g_pause = value;
        report_param("PAUSE", value);
    } else if (strcmp(argv[1], "WIDTH") == 0) {
        g_width = value;
        report_param("WIDTH", value);
    } else if (strcmp(argv[1], "GAP") == 0) {
        // GAP is the spacing BETWEEN pulses, so it can only ever apply when
        // COUNT > 1 -- which the next branch refuses. It is therefore stored
        // but never silently applied. Whoever lifts the COUNT cap has to
        // implement GAP in the same change, or this becomes a quiet lie.
        g_gap = value;
        report_param("GAP", value);
    } else if (strcmp(argv[1], "COUNT") == 0) {
        // The PIO program emits ONE pulse. Accepting COUNT > 1 and quietly
        // firing once would put a number in the campaign log that never
        // happened -- refuse instead.
        if (value > 1) {
            rp_err("COUNT > 1 is not supported: this engine emits a single pulse");
            return;
        }
        g_count = value;
        rp_ok("COUNT set to %u", (unsigned)value);
    } else {
        rp_err("Unknown variable '%s' (use PAUSE/WIDTH/GAP/COUNT)", argv[1]);
    }
}

static void cmd_get(int argc, char* argv[]) {
    if (argc < 2) {
        rp_printf("PAUSE: %u cycles\r\n", (unsigned)g_pause);
        rp_printf("WIDTH: %u cycles\r\n", (unsigned)g_width);
        rp_printf("GAP:   %u cycles\r\n", (unsigned)g_gap);
        rp_printf("COUNT: %u\r\n", (unsigned)g_count);
        return;
    }
    if (strcmp(argv[1], "PAUSE") == 0) {
        rp_printf("%u cycles\r\n", (unsigned)g_pause);
    } else if (strcmp(argv[1], "WIDTH") == 0) {
        rp_printf("%u cycles\r\n", (unsigned)g_width);
    } else if (strcmp(argv[1], "GAP") == 0) {
        rp_printf("%u cycles\r\n", (unsigned)g_gap);
    } else if (strcmp(argv[1], "COUNT") == 0) {
        rp_printf("%u\r\n", (unsigned)g_count);
    } else {
        rp_err("Unknown variable '%s' (use PAUSE/WIDTH/GAP/COUNT)", argv[1]);
    }
}

/* ----------------------------------------------------------------- TRIGGER -- */

static void cmd_trigger(int argc, char* argv[]) {
    if (argc < 2) {
        if (g_edge == RAIDEN_TRIGGER_NONE) {
            rp_send("Trigger: NONE\r\n");
        } else {
            rp_printf("Trigger: GPIO BIO%u %s edge\r\n", (unsigned)RAIDEN_BIO_TRIGGER,
                      edge_name(g_edge));
        }
        return;
    }
    if (g_armed || g_trace != RAIDEN_TRACE_IDLE) {
        // The host already sends ARM OFF before changing the trigger, because
        // the upstream firmware refuses it too. Accepting it here would change
        // a polarity the running state machine cannot see.
        rp_err("Disarm before changing the trigger (ARM OFF, TRACE RESET)");
        return;
    }
    if (strcmp(argv[1], "NONE") == 0) {
        g_edge = RAIDEN_TRIGGER_NONE;
        rp_ok("Trigger disabled");
        return;
    }
    if (strcmp(argv[1], "GPIO") != 0 || argc < 3) {
        rp_err("Usage: TRIGGER [NONE|GPIO <RISING|FALLING>]");
        return;
    }
    raiden_trigger_t edge;
    if (strcmp(argv[2], "RISING") == 0) {
        edge = RAIDEN_TRIGGER_RISING;
    } else if (strcmp(argv[2], "FALLING") == 0) {
        edge = RAIDEN_TRIGGER_FALLING;
    } else {
        rp_err("Unknown edge '%s' (use RISING or FALLING)", argv[2]);
        return;
    }
    if (!program_load(edge)) {
        rp_err("PIO instruction memory full: cannot load the %s program", edge_name(edge));
        return;
    }
    g_edge = edge;
    rp_ok("GPIO trigger on BIO%u, %s edge", (unsigned)RAIDEN_BIO_TRIGGER, edge_name(edge));
}

/* --------------------------------------------------------------------- ARM -- */

static bool require_external_mode(const char* what) {
    if (raiden_power_mode() == RAIDEN_POWER_EXTERNAL) {
        return true;
    }
    // In INTERNAL mode the gate is not meant to be driven at all. Handing the
    // pad to a state machine anyway would close a crowbar on the target's own
    // rail -- a measured failure mode, and one that looks like a dead target
    // rather than a configuration mistake.
    rp_err("%s refused: power mode is INTERNAL, BIO%u must not be driven "
           "(TARGET POWER EXTERNAL first)",
           what, (unsigned)RAIDEN_BIO_CROWBAR);
    return false;
}

static void arm_on(void) {
    if (g_armed) {
        rp_err("Already armed (ARM OFF first)");
        return;
    }
    if (g_trace != RAIDEN_TRACE_IDLE) {
        rp_err("A trace holds the state machine: send TRACE RESET first");
        return;
    }
    if (!require_external_mode("ARM ON")) {
        return;
    }
    if (g_edge == RAIDEN_TRIGGER_NONE) {
        rp_err("No trigger configured: send TRIGGER GPIO <RISING|FALLING> first");
        return;
    }
    if (!program_load(g_edge)) {
        rp_err("PIO instruction memory full: cannot load the %s program", edge_name(g_edge));
        return;
    }
    sm_start(entry_armed(), true, g_pause, g_width);
    g_armed = true;
    rp_ok("System armed");
}

static void arm_off(void) {
    if (g_trace != RAIDEN_TRACE_IDLE) {
        // Never take the state machine away from a trace: the trace would die
        // without a word, TRACE STATUS would read RUNNING for ever, and at the
        // host that is indistinguishable from a trigger edge that never came.
        g_armed = false;
        rp_ok("System disarmed (a trace still holds the state machine; TRACE RESET releases it)");
        return;
    }
    sm_stop();
    gate_to_cpu();
    g_armed = false;
    rp_ok("System disarmed");
}

static void arm_trace(void) {
    // The exact prefix "Failed to arm trace" is what the host reports back to
    // the operator; the reason after the colon is for the human.
    if (g_armed) {
        rp_err("Failed to arm trace: the glitch engine is armed (ARM OFF first)");
        return;
    }
    if (g_trace == RAIDEN_TRACE_COMPLETE) {
        rp_err("Failed to arm trace: a completed trace is still held (TRACE RESET first)");
        return;
    }
    if (g_edge == RAIDEN_TRIGGER_NONE) {
        rp_err("Failed to arm trace: no trigger configured");
        return;
    }
    if (!program_load(g_edge)) {
        rp_err("Failed to arm trace: PIO instruction memory full");
        return;
    }
    // PAUSE and WIDTH are pushed as zero. A trace witnesses the EDGE, nothing
    // else, and a 4.5 M-cycle PAUSE would hold the witness back by 36 ms at
    // 125 MHz -- longer than the host's settle, so a perfectly good edge would
    // read as absent. Power mode is deliberately NOT required here: no pulse
    // can leave, which is what makes the trace the one trigger check that risks
    // nothing on a live target.
    sm_start(entry_armed(), false, 0u, 0u);
    g_trace = RAIDEN_TRACE_RUNNING;
    rp_ok("Trace armed (trigger only, no glitch pulse)");
}

static void cmd_arm(int argc, char* argv[]) {
    if (argc < 2) {
        // These two words, alone, are the whole contract.
        // "DISARMED" CONTAINS "ARMED": the host tests DISARMED first, and any
        // decoration around them would make a shot that left read as a shot
        // that never did -- classifying an entire campaign backwards.
        rp_send(g_armed ? "ARMED\r\n" : "DISARMED\r\n");
        return;
    }
    if (strcmp(argv[1], "ON") == 0) {
        arm_on();
    } else if (strcmp(argv[1], "OFF") == 0) {
        arm_off();
    } else if (strcmp(argv[1], "TRACE") == 0) {
        arm_trace();
    } else {
        rp_err("Usage: ARM [ON|OFF|TRACE]");
    }
}

/* ------------------------------------------------------------------ GLITCH -- */

static void cmd_glitch(int argc, char* argv[]) {
    if (argc > 1) {
        // raiden's GLITCH takes no argument: the parameters come from SET. An
        // argument silently ignored here would fire a pulse nobody described.
        rp_err("GLITCH takes no argument (use SET PAUSE / SET WIDTH)");
        return;
    }
    if (g_armed) {
        rp_err("Refused: the engine is armed for a triggered shot (ARM OFF first)");
        return;
    }
    if (g_trace != RAIDEN_TRACE_IDLE) {
        rp_err("Refused: a trace holds the state machine (TRACE RESET first)");
        return;
    }
    if (!require_external_mode("GLITCH")) {
        return;
    }
    if (!program_load(g_edge)) {
        rp_err("PIO instruction memory full: cannot load the pulse program");
        return;
    }

    uint32_t hz = raiden_clock_hz();
    uint64_t cycles = (uint64_t)g_pause + (uint64_t)g_width + 16u; // + program overhead
    uint64_t us = (hz != 0u) ? (cycles * 1000000ull) / (uint64_t)hz : 0ull;
    if (us + 1000ull > (uint64_t)MANUAL_MAX_US) {
        rp_err("PAUSE+WIDTH is about %u ms at this clock: a manual GLITCH would hold the "
               "command loop past its %u ms ceiling. Use ARM ON with a trigger",
               (unsigned)(us / 1000ull), (unsigned)(MANUAL_MAX_US / 1000u));
        return;
    }
    uint32_t budget_us = (uint32_t)us + 1000u;

    sm_start(entry_now(), true, g_pause, g_width);

    // Bounded, always: a lost pulse must not turn binmode_service() into a
    // dead loop. The host reading a timeout is a bench that says so; a bench
    // that stops answering is a bench that has to be power-cycled to diagnose.
    absolute_time_t deadline = make_timeout_time_us(budget_us);
    while (pio_sm_is_rx_fifo_empty(g_pio.pio, GLITCH_SM)) {
        if (time_reached(deadline)) {
            // ORDER MATTERS, and only on this path. A timeout means the state
            // machine is stuck SOMEWHERE -- possibly between `set pins, 1` and
            // `set pins, 0`, i.e. with the crowbar closed. Disabling it first
            // would freeze the gate asserted on a live rail until the next
            // call got round to parking it. gate_to_cpu() pins the pad at idle
            // before anything else moves, so the window does not exist.
            gate_to_cpu();
            sm_stop();
            rp_err("Glitch did not complete within %u us: the state machine never reached the "
                   "end of its program",
                   (unsigned)budget_us);
            return;
        }
        tight_loop_contents();
    }
    gate_to_cpu();
    sm_stop();
    g_shots++;
    rp_ok("Glitch executed");
}

/* ------------------------------------------------------------------- TRACE -- */

static void cmd_trace(int argc, char* argv[]) {
    if (argc < 2) {
        rp_err("Usage: TRACE <samples> <pre%%> | TRACE STATUS | TRACE RESET");
        return;
    }
    if (strcmp(argv[1], "STATUS") == 0) {
        switch (g_trace) {
            case RAIDEN_TRACE_RUNNING:
                rp_send("Trace: RUNNING (waiting for trigger)\r\n");
                break;
            case RAIDEN_TRACE_COMPLETE:
                rp_send("Trace: COMPLETE (ready for TRACE DUMP)\r\n");
                break;
            default:
                rp_send("Trace: IDLE\r\n");
                break;
        }
        return;
    }
    if (strcmp(argv[1], "RESET") == 0) {
        // Touch the state machine ONLY if a trace holds it: a TRACE RESET sent
        // while the glitch engine is armed must not disarm it behind the
        // operator's back.
        if (g_trace != RAIDEN_TRACE_IDLE) {
            sm_stop();
            gate_to_cpu();
            g_trace = RAIDEN_TRACE_IDLE;
        }
        rp_ok("Trace reset");
        return;
    }
    if (strcmp(argv[1], "DUMP") == 0) {
        // Loud, not silent. Returning an empty or fabricated waveform here
        // would be read as a measurement of the glitch node.
        rp_err("TRACE DUMP is not implemented in this build: the trace captures no samples, "
               "it is a trigger witness only (see TRACE STATUS)");
        return;
    }

    uint32_t samples;
    uint32_t pre;
    if (argc < 3 || !raiden_parse_u32(argv[1], &samples) || !raiden_parse_u32(argv[2], &pre)) {
        rp_err("Usage: TRACE <samples> <pre%%> | TRACE STATUS | TRACE RESET");
        return;
    }
    if (samples == 0u || samples > TRACE_SAMPLES_MAX) {
        rp_err("Sample count out of range (1-%u)", (unsigned)TRACE_SAMPLES_MAX);
        return;
    }
    if (pre > 100u) {
        rp_err("Pre-trigger percentage out of range (0-100)");
        return;
    }
    if (g_armed) {
        rp_err("Refused: the glitch engine is armed (ARM OFF first)");
        return;
    }
    if (g_trace != RAIDEN_TRACE_IDLE) {
        rp_err("A trace is already armed: send TRACE RESET first");
        return;
    }
    if (g_edge == RAIDEN_TRIGGER_NONE) {
        rp_err("No trigger configured: send TRIGGER GPIO <RISING|FALLING> first");
        return;
    }
    if (!program_load(g_edge)) {
        rp_err("PIO instruction memory full: cannot load the %s program", edge_name(g_edge));
        return;
    }
    // Both numbers are recorded and reported by STATUS, and nothing samples
    // anything: this build has no ADC capture. They are kept so a later capture
    // implementation reads the same command, and TRACE DUMP says out loud that
    // no waveform exists.
    g_trace_samples = samples;
    g_trace_pre = pre;
    sm_start(entry_armed(), false, 0u, 0u);
    g_trace = RAIDEN_TRACE_RUNNING;
    rp_send("Trace armed, waiting\r\n");
}

/* ---------------------------------------------------------------- dispatch -- */

void raiden_glitch_command(int argc, char* argv[]) {
    refresh(); // see refresh(): there is no main-loop hook to do this for us

    if (strcmp(argv[0], "SET") == 0) {
        cmd_set(argc, argv);
    } else if (strcmp(argv[0], "GET") == 0) {
        cmd_get(argc, argv);
    } else if (strcmp(argv[0], "ARM") == 0) {
        cmd_arm(argc, argv);
    } else if (strcmp(argv[0], "GLITCH") == 0) {
        cmd_glitch(argc, argv);
    } else if (strcmp(argv[0], "TRIGGER") == 0) {
        cmd_trigger(argc, argv);
    } else if (strcmp(argv[0], "TRACE") == 0) {
        cmd_trace(argc, argv);
    } else {
        rp_err("Unknown glitch command '%s'", argv[0]);
    }
}

void raiden_glitch_status(void) {
    // BEFORE printing, never after. This is the witness the host trusts by
    // construction; the bare ARM is the cheap one. Printing a stale flag here
    // would make STATUS the same as ARM and leave the host with no safe probe.
    refresh();

    rp_send("\r\n== Glitch Parameters ==\r\n");
    rp_printf("Armed:        %s\r\n", g_armed ? "YES" : "NO");
    rp_printf("Glitch Count: %u\r\n", (unsigned)g_shots);
    rp_printf("Pause:        %u cycles\r\n", (unsigned)g_pause);
    rp_printf("Width:        %u cycles\r\n", (unsigned)g_width);
    rp_printf("Gap:          %u cycles\r\n", (unsigned)g_gap);
    // "Pulses", not "Count": STATUS already carries "Glitch Count:", which is the
    // number of shots FIRED, while this is the COUNT parameter -- pulses per shot,
    // capped at 1. Two different numbers, and a line spelled "Count:" here would be
    // a substring collision waiting for a host parser to pick the wrong one.
    rp_printf("Pulses:       %u per shot\r\n", (unsigned)g_count);
    if (g_edge == RAIDEN_TRIGGER_NONE) {
        rp_send("Trigger:      NONE\r\n");
    } else {
        rp_printf("Trigger:      GPIO BIO%u %s edge\r\n", (unsigned)RAIDEN_BIO_TRIGGER,
                  edge_name(g_edge));
    }
    // Named "Trace state" and not "Trace": the host scans for a line starting
    // with "Trace:" to read TRACE STATUS, and two commands must not answer the
    // same question in two places.
    rp_printf("Trace state:  %s (%u samples, %u%% pre, no capture in this build)\r\n",
              trace_name(), (unsigned)g_trace_samples, (unsigned)g_trace_pre);
    rp_printf("Latency:      PAUSE + %u cycles (%u ps here) -- DERIVED, scope it\r\n",
              (unsigned)RAIDEN_GLITCH_LATENCY_CYCLES,
              (unsigned)(RAIDEN_GLITCH_LATENCY_CYCLES * raiden_clock_step_ps()));
    rp_printf("Pulse:        WIDTH + %u cycles asserted\r\n",
              (unsigned)RAIDEN_GLITCH_WIDTH_OVERHEAD_CYCLES);
}

void raiden_glitch_init(void) {
    g_pause = 0;
    g_width = 0;
    g_gap = 0;
    g_count = 1;
    g_shots = 0;
    g_armed = false;
    // No default edge is invented: the host always sends TRIGGER GPIO, and a
    // guessed polarity would arm a detector on the wrong transition without
    // anyone being told.
    g_edge = RAIDEN_TRIGGER_NONE;
    g_trace = RAIDEN_TRACE_IDLE;
    g_trace_samples = 0;
    g_trace_pre = 0;

    g_pio.pio = PIO_MODE_PIO;
    g_pio.sm = GLITCH_SM;
    g_pio.offset = 0;
    g_pio.program = NULL;
    g_program_loaded = false;
    g_program_edge = RAIDEN_TRIGGER_RISING;

    // BIO1 is an INPUT: this engine watches the target's rail come back up, it
    // never drives it. Driving it would fight the very supply it measures.
    bio_input(RAIDEN_BIO_TRIGGER);
    system_bio_update_purpose_and_label(true, RAIDEN_BIO_TRIGGER, BP_PIN_MODE, "TRIG");

    // raiden_power_init() runs first and already parked the gate; take nothing
    // until ARM ON, and make the resting ownership explicit.
    gate_to_cpu();
}

void raiden_glitch_deinit(void) {
    if (g_program_loaded) {
        pio_sm_set_enabled(g_pio.pio, GLITCH_SM, false);
        pio_sm_restart(g_pio.pio, GLITCH_SM);
        pio_sm_clear_fifos(g_pio.pio, GLITCH_SM);
        pio_remove_program(g_pio.pio, g_pio.program, g_pio.offset);
        g_program_loaded = false;
        g_pio.program = NULL;
    }
    g_armed = false;
    g_trace = RAIDEN_TRACE_IDLE;
    // The gate last, and never skipped. A pad left under a state machine is not
    // at rest: it is a crowbar waiting for whatever the next mode writes there.
    // This also has to happen BEFORE raiden_power_deinit(), which parks the gate
    // through SIO -- on a pad still owned by the PIO that write goes nowhere.
    gate_to_cpu();
    system_bio_update_purpose_and_label(false, RAIDEN_BIO_TRIGGER, BP_PIN_MODE, 0);
}
