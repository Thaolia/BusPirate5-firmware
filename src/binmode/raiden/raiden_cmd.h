/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_cmd.h
 * @brief The one contract every raiden-dialect command module implements.
 *
 * Tokens arrive already upper-cased and split on whitespace, raiden-style.
 * argv[0] is the verb the dispatcher matched.
 *
 * A handler returns once it has emitted its ENTIRE reply -- success or
 * failure. The dispatcher adds only the trailing prompt. A handler that
 * returns without printing anything leaves the host reading until its own
 * timeout, which on a campaign loop looks like a dead bench rather than a bug.
 */
#ifndef RAIDEN_CMD_H
#define RAIDEN_CMD_H

#include <stdbool.h>
#include <stdint.h>

#define RAIDEN_MAX_ARGS 8

/** Parse a decimal or 0x-prefixed token. false leaves *out untouched. */
bool raiden_parse_u32(const char* s, uint32_t* out);

/* Command entry points. Each lives in its own translation unit so the modules
 * can be written independently; only the dispatch table below joins them. */
void raiden_swd_command(int argc, char* argv[]);     /* raiden_swd.c    */
void raiden_glitch_command(int argc, char* argv[]);  /* raiden_glitch.c */
void raiden_power_command(int argc, char* argv[]);   /* raiden_power.c  */

/* Lifecycle hooks the dispatcher calls on mode entry and exit. Each module
 * owns its own pins and PIO resources and must release them here. */
void raiden_swd_init(void);
void raiden_swd_deinit(void);
void raiden_glitch_init(void);
void raiden_glitch_deinit(void);
void raiden_power_init(void);
void raiden_power_deinit(void);

/** STATUS is assembled from every module; each appends its own section. */
void raiden_glitch_status(void);
void raiden_power_status(void);

#endif
