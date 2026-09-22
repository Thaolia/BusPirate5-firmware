/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_proto.h
 * @brief Wire-format primitives for the raiden-dialect binmode.
 *
 * Every byte this binmode sends to the host goes through here. The formats are
 * NOT free choices: they are the contract consumed by the fault_injection host
 * tooling (src/common/raiden_client.py, src/common/hexdump.py,
 * src/nrf52/ctrl_ap.py, src/nrf52/campaign.py). Changing a single space breaks
 * a parser on the other side, silently.
 *
 * What this file does NOT do: it never talks to SWD, the PSU or the PIO. It
 * formats and emits, nothing else.
 */
#ifndef RAIDEN_PROTO_H
#define RAIDEN_PROTO_H

#include <stdbool.h>
#include <stdint.h>

/** Raw bytes to the binary CDC, no framing added. */
void rp_send(const char* s);

/** printf into the binary CDC. Lines must carry their own "\r\n". */
void rp_printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/** "OK: " + text + CRLF. */
void rp_ok(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/** "ERROR: " + text + CRLF.
 *
 * The literal word ERROR is the ONLY failure signal the host understands:
 * ~25 call sites test `"ERROR" in out`. Never invent "ERR:" or "FAIL:".
 */
void rp_err(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/** The "> " prompt, emitted after every command.
 *
 * Not decoration. MinimalRaiden.cmd() ends a read on an OK:/ERROR: marker OR
 * on a trailing '>'. Replies that carry no marker -- ARM, GET, STATUS,
 * TRACE STATUS -- would otherwise stall until the caller's timeout expires.
 */
void rp_prompt(void);

/** True once if a byte had to be dropped since the last call, and clears.
 *
 * A full TX FIFO means the host stopped reading. Dropping the byte keeps the
 * firmware alive -- spinning forever would be the firmware-side twin of
 * pyserial's write_timeout=None, which froze a campaign with no message at all
 * (measured 2026-09-21) -- but a silently truncated reply is worse than a
 * noisy failure. The dispatcher calls this after every command and turns a
 * drop into an ERROR line the host already knows how to read.
 */
bool rp_take_truncated(void);

/** Hex dump in the exact shape of raiden's `SWD READ`, trailer included.
 *
 * Reading %u bytes from 0x%08X:
 * 0x%08X: 00 11 22 ...  ASCII
 * OK: Read complete
 *
 * 16 bytes per line; non-printables render as '.'. A short final line is NOT
 * padded out to 16 columns -- neither here nor in raiden, whose padding branch
 * only fires on a partial bus read, never at the end of a dump.
 *
 * The TWO spaces between the hex column and the ASCII column are load-bearing,
 * not cosmetic. The host parser ends the byte run at the first run of two
 * spaces, which is the only thing that stops an ASCII column beginning with a
 * hex pair followed by a space ("ab cdef") from being read as data. Emit one
 * space between bytes and exactly two before the ASCII column.
 *
 * WARNING: this is the 16-bytes-per-line format that hexdump.parse_bytes()
 * accepts. It is NOT the 4-words-per-line RAMREAD format -- the host refuses
 * to mix them on purpose, because confusing them reverses byte order.
 */
void rp_hexdump(uint32_t addr, const uint8_t* buf, uint32_t nbytes);

/** One RAMREAD line: an address prefix and up to four 32-bit words.
 *
 * The OTHER dump format, and it is not interchangeable with rp_hexdump(). That
 * one is 16 BYTES per line and the host reassembles it byte by byte; this one
 * is 4 WORDS per line and the host reassembles it word by word, little-endian.
 * Feeding either output to the other's parser reverses byte order within every
 * word -- silently, because both look like a hexdump.
 *
 * The host's parser is anchored: nothing may precede the "0x", and nothing may
 * follow the last word but the line terminator. It also keys each word on THIS
 * line's own address, so the caller passes the address the data came FROM, not
 * the address it was read from.
 *
 * No trailer here, deliberately. The host stops reading a reply at its first
 * marker, so "OK:" belongs after the last line of a whole pass -- never once
 * per block, the way rp_hexdump() emits it.
 */
void rp_words_line(uint32_t addr, const uint32_t* words, uint32_t nwords);

/** Debug-port register read-back: "OK: DP[0x4] = 0x50000040". */
void rp_reg_dp(uint8_t addr, uint32_t value);

/** Access-port register read-back.
 *
 * AP 0 prints WITHOUT an index ("OK: AP[0x0C] = ..."), any other APSEL prints
 * the index glued to the name ("OK: AP1[0x0C] = ..."). The host regex accepts
 * both spellings; emitting the wrong one for APSEL 0 would still parse, but it
 * would diverge from raiden and make two benches disagree on a stored trace.
 */
void rp_reg_ap(uint8_t apsel, uint8_t addr, uint32_t value);

#endif
