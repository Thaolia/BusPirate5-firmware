/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_ctrlap.h
 * @brief nRF52 CTRL-AP -- the access port that answers when the AHB-AP is dead.
 *
 * APSEL 1. That index is NOT in the Product Specification: it comes from
 * LimitedResults' OpenOCD (`nrf52.dap apreg 1 0x0c`), so any sequence must
 * re-read IDR and compare it -- otherwise a wrong index would serve the AHB-AP
 * under another name.
 *
 * WARNING: APPROTECTSTATUS polarity is inverted against intuition --
 * 0 = protection ACTIVE, 1 = protection LIFTED. Getting it backwards makes the
 * oracle exactly wrong, and never noisily.
 *
 * WARNING: nothing in this firmware ever writes ERASEALL on its own. The
 * constant is defined so the register map is complete and so the write-only
 * rule below can name it; the only way to fire it is for the host to spell out
 * `SWD WRITE AP 1 0x04 0x1`, which its own tooling gates behind an explicit
 * confirmation. ERASEALL wipes flash, UICR and RAM, and there is no undo.
 */
#ifndef RAIDEN_CTRLAP_H
#define RAIDEN_CTRLAP_H

#include <stdbool.h>
#include <stdint.h>

#define RAIDEN_CTRLAP_APSEL 1u

#define RAIDEN_CTRLAP_RESET 0x000u
#define RAIDEN_CTRLAP_ERASEALL 0x004u
#define RAIDEN_CTRLAP_ERASEALLSTATUS 0x008u
#define RAIDEN_CTRLAP_APPROTECTSTATUS 0x00Cu
#define RAIDEN_CTRLAP_IDR 0x0FCu
#define RAIDEN_CTRLAP_IDR_EXPECTED 0x02880000u

/** Read one CTRL-AP register, proving once per connection that it IS the
 * CTRL-AP.
 *
 * The proof is a silent IDR read; only a MISMATCH prints, as a plain warning
 * line that deliberately carries no OK: or ERROR: marker. The value asked for
 * is still returned: a bench that refused to show a wrong IDR could not be
 * used to diagnose the wrong index it is warning about.
 */
bool raiden_ctrlap_read(uint8_t addr, uint32_t* out);

/** Write one CTRL-AP register, with the same one-per-connection IDR proof. */
bool raiden_ctrlap_write(uint8_t addr, uint32_t value);

/** True for a register that must NOT be read back after a write.
 *
 * ERASEALL is write-only, and reading it back would be worse than useless: the
 * erase takes ~173 ms during which the port may not answer, so a failed
 * read-back would report an erase that DID happen as an erase that was
 * refused. On an irreversible operation that is the dangerous direction.
 */
bool raiden_ctrlap_write_only(uint8_t addr);

/** Forget the IDR proof. Called on every connect: a new link proves nothing. */
void raiden_ctrlap_forget(void);

#endif
