/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence. The
 * payload below was reassembled from its six ARMv6-M mnemonics, each encoded
 * from its format in the ARM ARM; the encodings were then checked against the
 * semantics the dialect documents in prose.
 */
/**
 * @file raiden_bat32.h
 * @brief SWD BAT32 -- the Level 1 read bypass, and nothing that writes flash.
 *
 * ONE verb: RAMREAD. The dialect this speaks has eight, and the other seven --
 * PROGRAM, WRITE, PATTERN, SECTORERASE, CHIPERASE, ARM, DISARM -- are
 * DELIBERATELY absent. They are the ones that change the part, and the dialect
 * itself marks them by demanding a literal CONFIRM; RAMREAD is the only one
 * without that gate, because it is the only one that does not modify flash.
 *
 * Two consequences, to be stated rather than discovered:
 *   - Arming Level 1 on a validation part, and recovering one by chip erase,
 *     are NOT possible from a Bus Pirate. That still needs the raiden.
 *   - RAMREAD is read-only on FLASH but OVERWRITES target SRAM
 *     0x20000000-0x2000101F, where the .data copied from flash at boot lives.
 *     At Level 1 that copy is readable flash content obtained without writing
 *     anything -- the fallback if the bypass fails -- and RAMREAD destroys it.
 *     A reset restores only about 93 % of it (measured at the bench,
 *     2026-09-06): roughly 282 bytes keep whatever the payload left, because
 *     boot only reinitialises what .data and .bss cover.
 *     The host owns that policy: dump.py snapshots SRAM first and stops if the
 *     snapshot fails. This module does not second-guess it, it states it.
 *
 * What this file does NOT do: it never writes flash, never erases, never
 * touches an option byte. It owns no protection-level logic either -- the lock
 * oracle is the host's, built from the contrast between a flash read and an
 * SRAM read, and it needs no firmware support beyond plain SWD READ.
 */
#ifndef RAIDEN_BAT32_H
#define RAIDEN_BAT32_H

/** SWD BAT32 <...>. argv[0] is "SWD", argv[1] is "BAT32". */
void raiden_bat32_command(int argc, char* argv[]);

/** Forget the once-per-session notices. Called on mode entry. */
void raiden_bat32_init(void);

#endif
