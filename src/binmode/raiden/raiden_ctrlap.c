/**
 * @file raiden_ctrlap.c
 * @brief nRF52 CTRL-AP -- the access port that answers when the AHB-AP is dead.
 *
 * This module exists for one reason the generic AP path cannot serve: APSEL 1
 * is not a documented number. It comes from LimitedResults' OpenOCD invocation
 * (`nrf52.dap apreg 1 0x0c`), not from the Product Specification, so every
 * connection has to PROVE that index 1 really lands on the CTRL-AP instead of
 * assuming it. The proof is one IDR read compared against the reset value the
 * PS does give.
 *
 * What this file does NOT do: it never starts an erase, it never decides
 * whether a chip is locked, and it never rewrites what the caller asked for.
 */
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pirate.h"

#include "raiden_ctrlap.h"
#include "raiden_proto.h"
#include "raiden_swd.h"

/* Three states, not two. UNKNOWN means the question has not been asked since
 * the last connect; WRONG means it was asked and answered badly, which must be
 * said once and not once per register. */
typedef enum {
    IDR_UNKNOWN = 0,
    IDR_CONFIRMED,
    IDR_WRONG,
} idr_proof_t;

static idr_proof_t proof = IDR_UNKNOWN;

void raiden_ctrlap_forget(void) {
    proof = IDR_UNKNOWN;
}

bool raiden_ctrlap_write_only(uint8_t addr) {
    // ERASEALL only. ERASEALLSTATUS (0x008) next door is read-only and reading
    // it is how the host watches the erase finish -- the two must not be
    // confused, they are one nibble apart.
    return addr == (uint8_t)RAIDEN_CTRLAP_ERASEALL;
}

/** Ask once per connection whether APSEL 1 is really the CTRL-AP.
 *
 * Returns whatever the caller asked for either way. Refusing to serve a read
 * when the IDR is wrong would make the bench useless for diagnosing exactly
 * the wrong index it is complaining about -- and the host runs its own IDR
 * check, which is the one that decides anything.
 */
static void prove_once(void) {
    if (proof != IDR_UNKNOWN) {
        return;
    }
    uint32_t idr = 0;
    // Silent on success: the host reads back the LAST "OK: AP..." line of a
    // reply, so an extra register line here would be picked up as the answer
    // to a question nobody asked.
    if (!raiden_swd_ap_read((uint8_t)RAIDEN_CTRLAP_APSEL, (uint8_t)RAIDEN_CTRLAP_IDR, &idr)) {
        // Leave it UNKNOWN: a transaction that did not complete has not
        // disproved anything, and the next access should ask again.
        return;
    }
    if (idr == RAIDEN_CTRLAP_IDR_EXPECTED) {
        proof = IDR_CONFIRMED;
        return;
    }
    proof = IDR_WRONG;
    // A plain line, carrying neither OK: nor the word ERROR: the access itself
    // is about to succeed, and calling it a failure would turn a diagnosable
    // wrong index into a bench that looks mute.
    rp_printf("WARNING: AP%u IDR = 0x%08X, expected 0x%08X -- this may not be the CTRL-AP\r\n",
              (unsigned)RAIDEN_CTRLAP_APSEL, (unsigned)idr, (unsigned)RAIDEN_CTRLAP_IDR_EXPECTED);
}

bool raiden_ctrlap_read(uint8_t addr, uint32_t* out) {
    prove_once();
    // APPROTECTSTATUS reads 0 for protection ACTIVE and 1 for protection
    // LIFTED -- inverted against every intuition, and the one polarity that
    // would make the oracle exactly wrong without ever being noisy about it.
    // Nothing here interprets it: the value goes up as it came off the wire.
    return raiden_swd_ap_read((uint8_t)RAIDEN_CTRLAP_APSEL, addr, out);
}

bool raiden_ctrlap_write(uint8_t addr, uint32_t value) {
    prove_once();
    // No guard on ERASEALL, and that is the considered choice: the host owns
    // that decision and gates it behind an explicit confirmation string, while
    // a firmware that silently refused a write it had ACKed would be worse
    // than one that performs it. What this firmware never does is write
    // ERASEALL on its own initiative -- no path here composes that value.
    return raiden_swd_ap_write((uint8_t)RAIDEN_CTRLAP_APSEL, addr, value);
}
