/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_power.h
 * @brief Target supply and power mode for the raiden-dialect binmode.
 *
 * Two supply sources, selected at runtime, never both live:
 *
 *   PSU     the Bus Pirate's own programmable supply on VOUT. A real switched
 *           supply, so the cut is clean rather than a rail sagging through CMOS
 *           pads -- and it measures the current it delivers. Costs ~12 ms a
 *           cycle in firmware sequencing.
 *   MOSFET  an external rail switched by a high-side FET on BIO5. One GPIO
 *           write, so the edge is sharp and its timing deterministic, which is
 *           what a trigger reference needs.
 *
 * On nRF52 only POR and brownout reset the debug port, so every attempt needs a
 * real cut either way.
 *
 * What this file does NOT do: it never fires the crowbar. It owns the gate
 * pin's IDLE state and its polarity; the pulse itself belongs to the PIO in
 * raiden_glitch.c.
 */
#ifndef RAIDEN_POWER_H
#define RAIDEN_POWER_H

#include <stdbool.h>
#include <stdint.h>

/** BIO carrying the external crowbar gate. Also M_UART_GLITCH_TRG upstream. */
#define RAIDEN_BIO_CROWBAR 0u

/** BIO driving the external high-side supply switch.
 *
 * The topology this matches is the project's own schematic: an N-channel
 * inverter whose drain pulls the gate of a P-channel high-side switch. The
 * inverter is NOT optional -- a P-channel conducts with its gate LOW, so wiring
 * this pin straight to it would invert the whole bench and make TARGET POWER
 * OFF switch the target ON.
 *
 * Safe state by construction: pin released, the external pull-down holds the
 * inverter off, the pull-up holds the P-channel gate at the rail, target
 * unpowered. Hardware and firmware fall on the same side.
 */
#define RAIDEN_BIO_SUPPLY 5u

typedef enum {
    RAIDEN_POWER_INTERNAL = 0, /**< No crowbar gate driven. */
    RAIDEN_POWER_EXTERNAL,     /**< BIO0 is the crowbar gate. */
} raiden_power_mode_t;

/** Where the target's rail comes from. */
typedef enum {
    RAIDEN_SUPPLY_PSU = 0, /**< The Bus Pirate's programmable supply, on VOUT. */
    RAIDEN_SUPPLY_MOSFET,  /**< An external rail switched by a FET on BIO5. */
} raiden_supply_t;

raiden_power_mode_t raiden_power_mode(void);
raiden_supply_t raiden_power_supply(void);

/** true when the gate asserts HIGH (idle LOW). Set by TARGET POWER EXTERNAL. */
bool raiden_power_gate_active_high(void);

/** Park the gate in its idle level, whatever the polarity. */
void raiden_power_gate_idle(void);

/** Read the VOUT/VREF rail, and say whether it can power the I/O buffers.
 *
 * WARNING, and it decides a wiring choice: the level shifters on the eight BIO
 * pins take their target-side rail from VOUT/VREF. With no voltage there they
 * drive nothing, which the firmware's own preflight states in as many words
 * (ui_help.c, "VOUT/VREF pin is not powered. Use W to enable power, or attach
 * an external supply").
 *
 * So a bench whose VREF hangs off the SWITCHED side of the MOSFET cuts its own
 * hand off: killing the target also kills the pin that drives the MOSFET, the
 * crowbar gate and the trigger input. VREF belongs UPSTREAM of the switch.
 *
 * The same rail feeds the trigger input's buffer, so on the PSU source the
 * trigger buffer wakes up DURING the rise it is meant to observe. The edge is
 * late and ill-defined rather than absent -- scope it before trusting a delay
 * sweep taken that way.
 *
 * @param mv_out  optional, receives the measured millivolts
 */
bool raiden_power_vref_ok(uint32_t* mv_out);

#endif
