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

#endif
