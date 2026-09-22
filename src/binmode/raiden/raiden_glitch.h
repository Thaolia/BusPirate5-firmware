/**
 * @file raiden_glitch.h
 * @brief Glitch pulse and trigger for the raiden-dialect binmode.
 *
 * One PIO state machine on PIO_MODE_PIO (pio1 on BP5): wait for the trigger
 * edge, delay loop, assert, width loop, deassert. Divider 1.0, so one loop
 * iteration is one clk_sys cycle and raiden_clock.c decides its length.
 *
 * What this file does NOT own: the crowbar gate's IDLE level and its polarity
 * belong to raiden_power.c. This module owns only the pulse, and it borrows the
 * gate pad from the CPU for exactly as long as it is armed.
 */
#ifndef RAIDEN_GLITCH_H
#define RAIDEN_GLITCH_H

#include <stdbool.h>
#include <stdint.h>

/** BIO watched for the trigger edge, held as an input. */
#define RAIDEN_BIO_TRIGGER 1u

typedef enum {
    RAIDEN_TRIGGER_NONE = 0,
    RAIDEN_TRIGGER_RISING,
    RAIDEN_TRIGGER_FALLING,
} raiden_trigger_t;

/** Trace is a trigger witness only: it arms the edge detector and emits NO
 *  pulse. It captures no samples in this build -- TRACE DUMP says so loudly. */
typedef enum {
    RAIDEN_TRACE_IDLE = 0,
    RAIDEN_TRACE_RUNNING,
    RAIDEN_TRACE_COMPLETE,
} raiden_trace_state_t;

/** Fixed cost, in clk_sys cycles, between the trigger edge appearing at the pad
 *  and the gate pin changing state, on top of PAUSE. Counted in the program:
 *
 *    2  the PIO input synchroniser on the trigger GPIO (two flip-flops)
 *    1  the cycle in which `wait` retires once its condition holds
 *    1  the single `jmp x-- delay` still executed when X reached 0
 *    1  `set pins, 1`, whose pin update lands at the end of its cycle
 *
 *  Convention: from the clk_sys edge at which the pad level is presented to the
 *  synchroniser, to the clk_sys edge at which the PIO output register drives
 *  the new level.
 *
 *  WARNING: DERIVED, NOT MEASURED. The BIO level shifter and the crowbar
 *  MOSFET's turn-on add delay that is not a PIO-cycle quantity and that
 *  dominates these five cycles. The real trigger-to-dip latency has to be read
 *  on a scope before any campaign -- that measurement is what fixes the shot
 *  budget, not this constant. Nothing in the firmware compensates for it: a
 *  built-in PAUSE offset would make SET PAUSE 3 mean something other than 3 in
 *  the campaign log, which is exactly the class of silent lie this project
 *  refuses. The host may subtract it once it has been measured.
 */
#define RAIDEN_GLITCH_LATENCY_CYCLES 5u

/** Asserted time for WIDTH = n is n + this, in clk_sys cycles: `set pins, 1`,
 *  then the n+1 executions of `jmp y-- width`, then `set pins, 0`. */
#define RAIDEN_GLITCH_WIDTH_OVERHEAD_CYCLES 2u

#endif
