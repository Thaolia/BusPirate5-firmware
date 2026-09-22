/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_swd.h
 * @brief Bit-banged ADIv5 SWD for the raiden-dialect binmode.
 *
 * Bit-banged on purpose: it needs no PIO at all, which is what leaves the PIO
 * budget free for the glitch engine. Nothing in this firmware could be reused
 * -- blueTag stops at reading DPIDR, with no MEM-AP, no memory read and no
 * halt, and there is no CMSIS-DAP or OpenOCD adapter in the tree.
 *
 * WARNING, and it decides whether the bench has an oracle at all: on a chip
 * locked by APPROTECT the AHB-AP is disabled and it does NOT fault. It answers
 * ACK OK and returns the constant 0x23000000 on every one of its registers,
 * its own IDR included. This stack passes that value through verbatim -- no
 * error, no special case. The host profile lists it as a dead word; a stack
 * that raised an error here would make a locked chip indistinguishable from a
 * dead bench, and every shot of a campaign would be misclassified.
 */
#ifndef RAIDEN_SWD_H
#define RAIDEN_SWD_H

#include <stdbool.h>
#include <stdint.h>

/** BIO assignments. Each costs two RP2040 GPIOs (BUFIO data + BUFDIR). */
#define RAIDEN_BIO_SWCLK 2u
#define RAIDEN_BIO_SWDIO 3u
#define RAIDEN_BIO_NRST 4u

/** ACK field of a SWD transaction. */
typedef enum {
    RAIDEN_ACK_OK = 1,
    RAIDEN_ACK_WAIT = 2,
    RAIDEN_ACK_FAULT = 4,
    RAIDEN_ACK_NONE = 7, /**< line idle / no target */
} raiden_swd_ack_t;

/** ACK of the last transaction, for error reporting. */
raiden_swd_ack_t raiden_swd_last_ack(void);

/** Raise CSYSPWRUPREQ|CDBGPWRUPREQ and wait for both acknowledges.
 *
 * MANDATORY before ANY access port, the CTRL-AP included (nRF52 PS 4.8.4).
 * Without it the port answers WRONG rather than answering with an error, so
 * every AP entry point in this module calls it first.
 *
 * @param stat_out receives CTRL/STAT as last read, or NULL.
 */
bool raiden_swd_power_up_debug(uint32_t* stat_out);

/** One AP register read. SELECT is rewritten on every call -- never cached.
 *
 * A SELECT cached across a power cycle would serve the AHB-AP under the
 * CTRL-AP's name, ACK OK, and return plausible values: exactly the confusion
 * the IDR check exists to catch. One extra transaction buys that away.
 */
bool raiden_swd_ap_read(uint8_t apsel, uint8_t addr, uint32_t* out);

/** One AP register write. Same SELECT rule as raiden_swd_ap_read(). */
bool raiden_swd_ap_write(uint8_t apsel, uint8_t addr, uint32_t value);

/* --- Cortex-M core debug, exposed to the target-family modules -------
 *
 * ARM architectural, not nRF52 or BAT32 specific: any family whose core is a
 * Cortex-M reaches its registers this way. They live here rather than in a
 * family module so a second family cannot quietly grow its own copy with a
 * different halt policy -- two halt policies would differ exactly where it is
 * hardest to notice, on a part that answers but does not stop.
 */

/** DCRSR register selectors. Only the ones a payload sequence needs. */
#define RAIDEN_REG_R0 0u
#define RAIDEN_REG_R1 1u
#define RAIDEN_REG_R2 2u
#define RAIDEN_REG_SP 13u
#define RAIDEN_REG_PC 15u
#define RAIDEN_REG_XPSR 16u

/** DHCSR status bits a caller has to tell apart. */
#define RAIDEN_DHCSR_S_HALT (1u << 17)
#define RAIDEN_DHCSR_S_LOCKUP (1u << 19)

/** Debug Exception and Monitor Control, and the Debug Fault Status register.
 *
 * WARNING on the bit number: VC_HARDERR is bit 10. Bit 0 is VC_CORERESET, and
 * the two get conflated -- ARMv6-M implements exactly VC_CORERESET, VC_HARDERR
 * and TRCENA, so a "correction" to bit 0 compiles, runs, and silently catches
 * the wrong event.
 */
#define RAIDEN_CM_DEMCR 0xE000EDFCu
#define RAIDEN_CM_DFSR 0xE000ED30u
#define RAIDEN_DEMCR_TRCENA (1u << 24)
#define RAIDEN_DEMCR_VC_HARDERR (1u << 10)

/** Words per MEM-AP block, the unit a streaming caller should buffer.
 *
 * Sized against the 1 KB TX FIFO, not against SWD: see the definition in
 * raiden_swd.c, which a static assertion keeps in step with this one.
 */
#define RAIDEN_MEM_BLOCK_WORDS 16u

/** Connect if not already connected. false means the target did not answer. */
bool raiden_swd_ensure_connected(void);

/** Clear the DP sticky error bits.
 *
 * MANDATORY before touching a part whose flash has just faulted. On BAT32G135
 * at Level 1 a flash read answers ACK=FAULT, which latches STICKYERR and makes
 * EVERY later AP transaction fail. A caller that probed the chip and then
 * halted would watch the halt fail for a reason that has nothing to do with
 * halting -- which is how a Level 1 recovery attempt was first misdiagnosed.
 */
bool raiden_swd_abort_clear(void);

/** How many words may be moved from @p addr without rewriting TAR.
 *
 * Never more than RAIDEN_MEM_BLOCK_WORDS, and never across a 1 KB boundary.
 */
uint32_t raiden_swd_block_words(uint32_t addr, uint32_t remaining);

/** One MEM-AP block read. @p nwords must come from raiden_swd_block_words(). */
bool raiden_swd_mem_read_block(uint32_t addr, uint32_t* buf, uint32_t nwords);

/** Memory write of any length: it re-windows TAR itself.
 *
 * This bench reads targets. The one thing it writes into a target is a code
 * payload in SRAM, and only where a family module asks for it by name.
 */
bool raiden_swd_mem_write(uint32_t addr, const uint32_t* values, uint32_t nwords);

/** Write ONE byte. Required for the BAT32 option bytes, which share a 32-bit
 *  word with the WDT/LVD/HOCO settings -- a word write would clobber them. */
bool raiden_swd_mem_write_byte(uint32_t addr, uint8_t value);

/** Read or write one core register through DCRSR+DCRDR. Core must be halted. */
bool raiden_swd_core_reg_read(uint8_t regsel, uint32_t* out);
bool raiden_swd_core_reg_write(uint8_t regsel, uint32_t value);

/** Read DHCSR once. */
bool raiden_swd_read_dhcsr(uint32_t* out);

/** Halt the core, retrying until @p timeout_ms has passed.
 *
 * Retries because one request does not always take on a core that is running,
 * and re-issues it whenever S_RESET_ST shows a reset swallowed the last one.
 *
 * But it gives up AT ONCE on a DHCSR that cannot be a DHCSR: 0xFFFFFFFF from a
 * floating line, or the 0x23000000 a disabled AHB-AP returns. Those are
 * CONSTANTS -- waiting cannot change them. That is what keeps SWD HALT on a
 * locked nRF52 as quick as it was before this loop existed, and the nRF52
 * campaign pays for one halt per shot (0.65 s, measured 2026-09-21).
 *
 * @param dhcsr_out optional, receives DHCSR as last read, for diagnostics.
 */
bool raiden_swd_halt(uint32_t timeout_ms, uint32_t* dhcsr_out);

/** Resume from halt: DHCSR = DBGKEY | C_DEBUGEN, with C_HALT cleared. */
bool raiden_swd_resume(void);

/** Drop the link and park the lines: the DP state is about to become invalid.
 *
 * Called before a reset or a family switch. Without it, ensure_connected()
 * would see its stale flag, skip the reconnect and fail on a DP that no longer
 * holds what it was told -- a failure reported against the first read after
 * the reset rather than against the reset.
 */
void raiden_swd_forget_connection(void);

#endif
