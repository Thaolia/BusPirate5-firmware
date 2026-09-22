/**
 * @file raiden_power.h
 * @brief Target supply and power mode for the raiden-dialect binmode.
 *
 * On raiden the target supply is a GPIO through a load switch. Here it is the
 * Bus Pirate's programmable PSU on VOUT, which is a better instrument: a real
 * switched supply, so the cut is clean rather than a rail sagging through CMOS
 * pads. On nRF52 only POR and brownout reset the debug port, which is exactly
 * what a clean cut provides.
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

typedef enum {
    RAIDEN_POWER_INTERNAL = 0, /**< PSU supplies VOUT; no crowbar gate driven. */
    RAIDEN_POWER_EXTERNAL,     /**< PSU supplies VOUT; BIO0 is the crowbar gate. */
} raiden_power_mode_t;

raiden_power_mode_t raiden_power_mode(void);

/** true when the gate asserts HIGH (idle LOW). Set by TARGET POWER EXTERNAL. */
bool raiden_power_gate_active_high(void);

/** Park the gate in its idle level, whatever the polarity. */
void raiden_power_gate_idle(void);

#endif
