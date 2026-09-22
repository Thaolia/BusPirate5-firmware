/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_target.h
 * @brief Which silicon family the bench is talking to, and the reset pin.
 *
 * The family is not decoration. Two of its properties are OPPOSITE between the
 * two parts this binmode serves, and getting either backwards produces a bench
 * that answers normally while doing nothing:
 *
 *   nRF52820  only POR and brownout rearm the debug port (PS 5.3.6.8). A pin
 *             reset buys nothing, so every attempt needs a real supply cut.
 *   BAT32G135 TARGET RESET rearms the debug port. No supply cut per shot --
 *             which is the whole difference in what a campaign costs.
 *
 * A campaign copied from one family to the other therefore fires blanks, and
 * says nothing about it. That is why TARGET RESET warns out loud when the
 * selected family is the one it cannot help.
 *
 * What this file does NOT do: it holds no memory map, no protection level and
 * no flash controller. A family that needs those gets its own module -- see
 * raiden_bat32.c. This one owns the selector and the reset pin, nothing else.
 */
#ifndef RAIDEN_TARGET_H
#define RAIDEN_TARGET_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RAIDEN_TARGET_NRF52 = 0, /**< Default: the bench this binmode was built for. */
    RAIDEN_TARGET_BAT32,     /**< Cmsemicon BAT32G135, Cortex-M0+. */
} raiden_target_t;

/** The selected family. Defaults to nRF52 at mode entry. */
raiden_target_t raiden_target_family(void);

/** TARGET verb: BAT32 / NRF52 / RESET, and POWER routed to raiden_power.c. */
void raiden_target_command(int argc, char* argv[]);

/** STATUS section: the family, and what it implies for the shot loop. */
void raiden_target_status(void);

/** Reset to the default family. Called on mode entry. */
void raiden_target_init(void);

#endif
