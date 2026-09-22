/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Thaolia
 *
 * Written for the Bus Pirate 5 firmware (MIT, (c) 2023 Ian Lesnet, Where Labs
 * LLC). Independent reimplementation of a command surface -- no code was copied
 * from the project whose dialect it speaks; that project ships no licence.
 */
/**
 * @file raiden_swd.c
 * @brief Bit-banged ADIv5 SWD and the command surface of the raiden dialect.
 *
 * Nothing in this firmware could be reused: blueTag stops at swdReadDPIDR(),
 * with no MEM-AP, no memory read and no halt, and there is no CMSIS-DAP or
 * OpenOCD adapter in the tree. The bit-level convention below IS blueTag's,
 * because that convention is the one piece of this that has been seen to read
 * a DPIDR off real silicon through these buffers (same repository, MIT).
 *
 * Bit-banged on purpose: it needs no PIO at all, which keeps the PIO budget
 * free for the glitch engine.
 *
 * WARNING, the single fact this whole file is shaped around: a chip locked by
 * APPROTECT has its AHB-AP DISABLED, and a disabled AP does not fault. It
 * answers ACK OK and returns the constant 0x23000000 on every register it has,
 * its own IDR included. Every path here passes that through unchanged. There
 * is no test for it anywhere in this file, and adding one would break the
 * campaign oracle: the host lists the value as a dead word and needs to see it.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pirate.h"
#include "command_struct.h"
#include "system_config.h"
#include "pirate/bio.h"

#include "raiden_bat32.h"
#include "raiden_cmd.h"
#include "raiden_ctrlap.h"
#include "raiden_proto.h"
#include "raiden_swd.h"

/* --- Debug port register map (ADIv5, IHI0031) ------------------------ */
#define DP_DPIDR 0x0u      /* read side of address 0x0 */
#define DP_ABORT 0x0u      /* write side of address 0x0 -- a DIFFERENT register */
#define DP_CTRL_STAT 0x4u
#define DP_SELECT 0x8u     /* write only */
#define DP_RDBUFF 0xCu     /* read only */

/* STKCMPCLR|STKERRCLR|WDERRCLR|ORUNERRCLR. Deliberately NOT DAPABORT (bit 0):
 * clearing the sticky bits recovers the port, aborting the current transfer
 * would also discard a transaction that may still be in flight. */
#define DP_ABORT_STICKY_CLEAR 0x1Eu

#define DP_PWRUP_REQ 0x50000000u /* CSYSPWRUPREQ|CDBGPWRUPREQ */
#define DP_PWRUP_ACK 0xA0000000u /* CSYSPWRUPACK|CDBGPWRUPACK */

/* --- MEM-AP ---------------------------------------------------------- */
#define AP_CSW 0x00u
#define AP_TAR 0x04u
#define AP_DRW 0x0Cu

/* MasterType=Debug (bit 29), HPROT[1:0]=privileged data (bits 25/24),
 * AddrInc=increment single (bits 5:4 = 0b01), Size=word (bits 2:0 = 0b010).
 * The upper half is the same 0x23000000 a disabled AHB-AP returns, which is a
 * coincidence of the Cortex-M default Prot field and nothing more -- do not
 * read one as evidence about the other. */
#define AP_CSW_WORD_INC 0x23000012u
/* Size=000 (byte), AddrInc=00 (off). Same DbgSwEnable/Prot bits as the word
 * variant -- only the access size and the auto-increment differ. A byte write
 * exists for ONE reason: the BAT32 option bytes share a 32-bit word with the
 * WDT, LVD and HOCO settings, so writing OCDEN as a word would silently
 * overwrite three unrelated configurations on the very part being locked. */
#define AP_CSW_BYTE 0x23000000u

/* TAR auto-increment is only guaranteed inside a 1 KB window (ADIv5): past it
 * the increment may wrap instead of carrying, so TAR is rewritten at every
 * boundary. Silently reading the same 1 KB twice is the failure this avoids. */
#define AP_TAR_WINDOW 0x400u

/* --- Cortex-M debug -------------------------------------------------- */
#define CM_CPUID 0xE000ED00u
#define CM_DHCSR 0xE000EDF0u
#define DHCSR_HALT_REQ 0xA05F0003u /* DBGKEY | C_DEBUGEN | C_HALT */
#define DHCSR_RESUME_REQ 0xA05F0001u /* DBGKEY | C_DEBUGEN, C_HALT cleared */
#define DHCSR_S_HALT (1u << 17)
#define DHCSR_S_REGRDY (1u << 16)
#define DHCSR_S_RESET_ST (1u << 25)

/* A value that cannot BE a DHCSR: bits 4-15 and 28-31 are reserved and read as
 * zero on every Cortex-M. 0xFFFFFFFF from a floating line trips it, and so does
 * the 0x23000000 a disabled AHB-AP returns. A genuine LOCKUP reading,
 * 0x01080001, does NOT -- that one is a real answer and has to stay readable,
 * because it is the difference between "this core died" and "this bus is not
 * there", and the BAT32 payload path reports the two differently. */
#define DHCSR_IMPLAUSIBLE 0xF000FFF0u

#define CM_DCRSR 0xE000EDF4u
#define CM_DCRDR 0xE000EDF8u
#define DCRSR_WRITE (1u << 16)

/* S_REGRDY after a DCRSR access. The ARM ARM gives no bound; a halted core
 * completes the transfer in a few cycles, so this is already generous. */
#define REGRDY_TRIES 40u
#define REGRDY_POLL_US 50u

#define HALT_POLL_US 200u

/* Timeout of the CLI halt. A running core stops in microseconds; this only has
 * to cover a part still coming out of reset. The BAT32 payload path asks for
 * ten times more, because there it halts a core executing application firmware
 * -- and asks for it explicitly rather than raising this one for everybody. */
#define HALT_CLI_TIMEOUT_MS 50u

/* --- Line-level sequences -------------------------------------------- */
#define SWD_JTAG_TO_SWD 0xE79Eu
#define SWD_DORMANT_ACTIVATION 0x1Au
#define SWD_LINE_RESET_CLOCKS 56u /* spec floor is 50; blueTag uses 62 */
#define SWD_IDLE_CLOCKS 8u

/* --- Tuning ---------------------------------------------------------- */
#define SWD_XFER_TRIES 20u
#define SWD_SPEED_DEFAULT_US 4u

/* Cap chosen from the host's own timing, not from taste. The host ends a reply
 * on a 0.2 s serial read timeout; between two hexdump blocks this firmware is
 * silent for one block's worth of bit-banging. A block is 16 words, a word is
 * 46 clocks, a clock is 2 x delay: at 100 us that silence is 147 ms, which
 * still fits. At 150 us it is 221 ms and the host would end the reply in the
 * middle of a dump, leaving the parser with holes. */
#define SWD_SPEED_MAX_US 100u

/* Minimum settling after a SWDIO direction flip, INDEPENDENT of clk_delay_us.
 * At SWD SPEED 0 there is no clock delay at all, and the level shifter would
 * be sampled before it has turned around -- which shows up as intermittent
 * ACK=0b111, i.e. as a bench that looks unplugged. */
#define SWD_DIR_SETTLE_US 1u

/* 16 words = 64 bytes = 4 hexdump lines, roughly 390 bytes of text. The binmode
 * TX FIFO is 1024 bytes (usb_tx.c) and is drained by core 1 while core 0 bangs
 * bits; rp_send() DROPS characters rather than blocking forever when it fills.
 * raiden uses 64 words, but raiden is not writing into a 1 KB FIFO shared with
 * a cooperative main loop: 64 words would emit ~1.3 KB in one burst with no
 * bit-banging pause to drain it. */
#define MEM_BLOCK_WORDS 16u
_Static_assert(MEM_BLOCK_WORDS == RAIDEN_MEM_BLOCK_WORDS,
               "the header's block size must match the one reasoned about here");

/* 1 MiB of words: the largest nRF52 flash. A count beyond this is a typo, and
 * a typo that reads for an hour looks exactly like a hung bench. */
#define SWD_READ_MAX_WORDS 0x40000u

#define PWRUP_POLL_TRIES 50u
#define PWRUP_POLL_US 200u

/* --- State ----------------------------------------------------------- */
static uint32_t clk_delay_us = SWD_SPEED_DEFAULT_US;
static bool connected = false;
static bool bus_awake = false;
static bool swdio_is_output = false;
static uint32_t last_dpidr = 0;
static raiden_swd_ack_t last_ack = RAIDEN_ACK_NONE;

/* One block, reused. 64 bytes of .bss rather than 64 bytes of stack on a part
 * that is already at 87 % RAM. */
static uint32_t mem_buf[MEM_BLOCK_WORDS];

/* ARM selection alert, 0x19BC0EA2 E3DDAFE9 86852D95 6209F392, in the order the
 * line takes it: least significant byte of the least significant word first. */
static const uint8_t dormant_alert[16] = {
    0x92, 0xF3, 0x09, 0x62, 0x95, 0x2D, 0x85, 0x86,
    0xE9, 0xAF, 0xDD, 0xE3, 0xA2, 0x0E, 0xBC, 0x19,
};

raiden_swd_ack_t raiden_swd_last_ack(void) {
    return last_ack;
}

/* Bit-level, defined below: swd_bus_wake() needs idle clocks to flush the DP. */
static void write_bits(uint32_t value, uint32_t count);

/* --- Pin level ------------------------------------------------------- */

static inline void swd_delay(void) {
    if (clk_delay_us != 0u) {
        // Timer-based, not clk_sys-based: it stays honest when raiden_clock.c
        // overclocks the part for the glitch engine.
        busy_wait_us_32(clk_delay_us);
    }
}

static void swdio_dir(bool output) {
    if (output == swdio_is_output) {
        return;
    }
    if (output) {
        bio_output(RAIDEN_BIO_SWDIO);
    } else {
        bio_input(RAIDEN_BIO_SWDIO);
    }
    swdio_is_output = output;
    busy_wait_us_32(SWD_DIR_SETTLE_US);
}

/** Park every SWD line in high impedance.
 *
 * Not tidiness. SWD lines left driven high re-feed the target through its ESD
 * diodes, so cutting its supply is no longer a cut: no brownout, no POR, and
 * on nRF52 only POR and brownout rearm the debug port. The campaign then fires
 * blanks and says nothing -- the bench notes call it "the cut that does not
 * cut". Both lines go low BEFORE being released, so the transient through the
 * direction flip is low rather than high.
 */
static void swd_park_pins(void) {
    // Value before direction: bio_put() only loads the output latch, so the
    // line is already low the instant the buffer turns around.
    bio_put(RAIDEN_BIO_SWCLK, false);
    bio_output(RAIDEN_BIO_SWCLK);
    bio_put(RAIDEN_BIO_SWDIO, false);
    bio_output(RAIDEN_BIO_SWDIO);
    bio_input(RAIDEN_BIO_SWCLK);
    bio_input(RAIDEN_BIO_SWDIO);
    // nRST is claimed so the pin display shows it reserved, and then never
    // driven: on nRF52820 a pin reset does NOT reset the debug port (PS
    // 5.3.6.8), so driving it would buy nothing and could fight the target.
    bio_input(RAIDEN_BIO_NRST);
    swdio_is_output = false;
    bus_awake = false;
}

/** Take the lines back after a park.
 *
 * The order is not cosmetic. Parking releases SWCLK, and taking it back means
 * driving it high again -- a RISING EDGE, which is the edge the DP samples
 * SWDIO on. Raise it with SWDIO high and the DP reads a START bit, then takes
 * the next seven bits of the real request as the rest of that phantom one, and
 * every transaction afterwards is off by one for as long as the link lasts.
 * So SWDIO is driven LOW first, the phantom edge clocks in an idle bit, and
 * eight more idle bits flush the DP to a known resting point.
 */
static void swd_bus_wake(void) {
    if (bus_awake) {
        return;
    }
    bio_put(RAIDEN_BIO_SWDIO, false);
    bio_output(RAIDEN_BIO_SWDIO);
    bio_put(RAIDEN_BIO_SWCLK, false);
    bio_output(RAIDEN_BIO_SWCLK);
    swdio_is_output = true;
    bus_awake = true;
    busy_wait_us_32(SWD_DIR_SETTLE_US);
    bio_put(RAIDEN_BIO_SWCLK, true); /* the phantom edge, with SWDIO low */
    swd_delay();
    write_bits(0, SWD_IDLE_CLOCKS);
}

/* --- Bit level -------------------------------------------------------
 *
 * SWCLK idles HIGH. One pulse is: drive low, wait, drive high, wait. A written
 * bit is placed before its pulse; a read bit is sampled before its pulse, which
 * is the high phase that follows the rising edge on which the target drove it.
 * That is blueTag's convention, and the count works out: after the 8 request
 * bits and the single turnaround pulse, the first sample reads ACK[0].
 */

static void clk_pulse(void) {
    bio_put(RAIDEN_BIO_SWCLK, false);
    swd_delay();
    bio_put(RAIDEN_BIO_SWCLK, true);
    swd_delay();
}

static void write_bit(bool value) {
    bio_put(RAIDEN_BIO_SWDIO, value);
    clk_pulse();
}

static bool read_bit(void) {
    bool value = bio_get(RAIDEN_BIO_SWDIO);
    clk_pulse();
    return value;
}

static void write_bits(uint32_t value, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        write_bit(((value >> i) & 1u) != 0u);
    }
}

static uint32_t read_bits(uint32_t count) {
    uint32_t value = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (read_bit()) {
            value |= 1u << i;
        }
    }
    return value;
}

static bool parity32(uint32_t v) {
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (v & 1u) != 0u;
}

/* --- Transaction level ----------------------------------------------- */

static bool swd_xfer_once(bool ap, bool rnw, uint8_t addr, uint32_t* data) {
    uint32_t a = ((uint32_t)addr >> 2) & 0x3u;
    uint32_t request = 0x81u                     /* start = 1, park = 1 */
                       | ((uint32_t)ap << 1)
                       | ((uint32_t)rnw << 2)
                       | (a << 3);
    uint32_t parity = (uint32_t)ap ^ (uint32_t)rnw ^ (a & 1u) ^ ((a >> 1) & 1u);
    request |= parity << 5;

    swdio_dir(true);
    write_bits(request, 8);

    swdio_dir(false);
    clk_pulse(); /* turnaround */
    uint32_t ack = read_bits(3);
    last_ack = (raiden_swd_ack_t)ack;

    bool ok = false;
    if (ack == (uint32_t)RAIDEN_ACK_OK) {
        if (rnw) {
            uint32_t value = read_bits(32);
            bool par = read_bit();
            clk_pulse(); /* turnaround back to host */
            swdio_dir(true);
            if (par == parity32(value)) {
                *data = value;
                ok = true;
            } else {
                // A marginal turnaround returns plausible garbage. Unchecked it
                // would be filed as a perturbed shot and fabricate a gradient
                // that does not exist. The frame is also out of step now, so
                // the link is dropped and the next command re-runs a line reset.
                rp_printf("Data parity mismatch, frame desynchronised\r\n");
                connected = false;
            }
        } else {
            clk_pulse(); /* turnaround back to host */
            swdio_dir(true);
            write_bits(*data, 32);
            write_bit(parity32(*data));
            ok = true;
        }
    } else {
        // WAIT and FAULT carry no data phase (overrun detection is left off),
        // so the line turns around straight after the ACK.
        clk_pulse();
        swdio_dir(true);
        if (ack == (uint32_t)RAIDEN_ACK_NONE) {
            // Nobody drove the line. Drop the link so the next command starts
            // with a line reset instead of talking to a chip that rebooted.
            connected = false;
        }
    }

    // Idle clocks with the line low: a DP write is not complete until the host
    // keeps clocking, and a bus parked low is a bus that is not driving the
    // target's pins high.
    write_bits(0, SWD_IDLE_CLOCKS);
    return ok;
}

static bool swd_abort_clear(void) {
    uint32_t value = DP_ABORT_STICKY_CLEAR;
    return swd_xfer_once(false, false, DP_ABORT, &value);
}

static bool swd_xfer(bool ap, bool rnw, uint8_t addr, uint32_t* data) {
    bool fault_cleared = false;
    for (uint32_t tries = 0; tries < SWD_XFER_TRIES; tries++) {
        if (swd_xfer_once(ap, rnw, addr, data)) {
            return true;
        }
        if (last_ack == RAIDEN_ACK_WAIT) {
            continue; /* the AP is busy; bounded by SWD_XFER_TRIES */
        }
        if (last_ack == RAIDEN_ACK_FAULT && !fault_cleared) {
            // A sticky error fails every AP transaction after it until ABORT
            // clears it. One clear, one retry -- looping on it would hide a
            // target that faults every time.
            (void)swd_abort_clear();
            fault_cleared = true;
            continue;
        }
        break;
    }
    return false;
}

static bool dp_read(uint8_t addr, uint32_t* out) {
    return swd_xfer(false, true, addr, out);
}

static bool dp_write(uint8_t addr, uint32_t value) {
    return swd_xfer(false, false, addr, &value);
}

/** SELECT for one AP register. DPBANKSEL stays 0: no DP bank 1 register is
 * used here, and a stray bank would silently retarget CTRL/STAT. */
static bool ap_select(uint8_t apsel, uint8_t addr) {
    uint32_t select = ((uint32_t)apsel << 24) | ((uint32_t)addr & 0xF0u);
    return dp_write(DP_SELECT, select);
}

bool raiden_swd_power_up_debug(uint32_t* stat_out) {
    uint32_t stat = 0;
    bool ok = dp_read(DP_CTRL_STAT, &stat);
    if (ok && (stat & DP_PWRUP_ACK) != DP_PWRUP_ACK) {
        if (dp_write(DP_CTRL_STAT, DP_PWRUP_REQ)) {
            for (uint32_t i = 0; i < PWRUP_POLL_TRIES; i++) {
                ok = dp_read(DP_CTRL_STAT, &stat);
                if (!ok || (stat & DP_PWRUP_ACK) == DP_PWRUP_ACK) {
                    break;
                }
                busy_wait_us_32(PWRUP_POLL_US);
            }
        }
    }
    if (stat_out != NULL) {
        *stat_out = stat;
    }
    return ok && (stat & DP_PWRUP_ACK) == DP_PWRUP_ACK;
}

bool raiden_swd_ap_read(uint8_t apsel, uint8_t addr, uint32_t* out) {
    if (!raiden_swd_power_up_debug(NULL)) {
        return false;
    }
    if (!ap_select(apsel, addr)) {
        return false;
    }
    uint32_t posted = 0;
    // An AP read is posted: this transaction starts the read, and RDBUFF hands
    // back its result. Reading the AP twice instead would return the value of
    // the previous access and shift every reported register by one.
    if (!swd_xfer(true, true, addr, &posted)) {
        return false;
    }
    return dp_read(DP_RDBUFF, out);
}

bool raiden_swd_ap_write(uint8_t apsel, uint8_t addr, uint32_t value) {
    if (!raiden_swd_power_up_debug(NULL)) {
        return false;
    }
    if (!ap_select(apsel, addr)) {
        return false;
    }
    if (!swd_xfer(true, false, addr, &value)) {
        return false;
    }
    // An AP write is posted too: its real ACK lands on the next transaction.
    // RDBUFF is the cheapest way to make a late failure visible.
    uint32_t rdbuff = 0;
    return dp_read(DP_RDBUFF, &rdbuff);
}

/** Route by APSEL. AP 1 goes through the CTRL-AP module, which proves once per
 * connection that the index really does reach the CTRL-AP. */
static bool ap_read_routed(uint8_t apsel, uint8_t addr, uint32_t* out) {
    if (apsel == (uint8_t)RAIDEN_CTRLAP_APSEL) {
        return raiden_ctrlap_read(addr, out);
    }
    return raiden_swd_ap_read(apsel, addr, out);
}

static bool ap_write_routed(uint8_t apsel, uint8_t addr, uint32_t value) {
    if (apsel == (uint8_t)RAIDEN_CTRLAP_APSEL) {
        return raiden_ctrlap_write(addr, value);
    }
    return raiden_swd_ap_write(apsel, addr, value);
}

/* --- Connection ------------------------------------------------------ */

static void line_reset(void) {
    swdio_dir(true);
    bio_put(RAIDEN_BIO_SWDIO, true);
    for (uint32_t i = 0; i < SWD_LINE_RESET_CLOCKS; i++) {
        clk_pulse();
    }
}

static void dormant_to_swd(void) {
    swdio_dir(true);
    bio_put(RAIDEN_BIO_SWDIO, true);
    for (uint32_t i = 0; i < 8u; i++) {
        clk_pulse();
    }
    for (uint32_t i = 0; i < sizeof(dormant_alert); i++) {
        write_bits(dormant_alert[i], 8);
    }
    write_bits(0, 4);
    write_bits(SWD_DORMANT_ACTIVATION, 8);
}

static bool try_read_dpidr(uint32_t* dpidr) {
    if (!dp_read(DP_DPIDR, dpidr)) {
        return false;
    }
    // A DPIDR of zero is a line that answered without a target behind it. The
    // host treats "connected" as proof the chip is alive, so a zero must fail.
    return *dpidr != 0u;
}

static bool swd_connect(void) {
    connected = false;
    raiden_ctrlap_forget();
    swd_bus_wake();

    uint32_t dpidr = 0;
    line_reset();
    write_bits(SWD_JTAG_TO_SWD, 16);
    line_reset();
    write_bits(0, SWD_IDLE_CLOCKS);
    bool ok = try_read_dpidr(&dpidr);

    if (!ok) {
        // Second attempt through the dormant state. Some DPs boot dormant and
        // ignore the JTAG-to-SWD sequence entirely; the order below is
        // blueTag's, which is the one seen to work through these buffers.
        dormant_to_swd();
        line_reset();
        write_bits(SWD_JTAG_TO_SWD, 16);
        line_reset();
        write_bits(0, SWD_IDLE_CLOCKS);
        ok = try_read_dpidr(&dpidr);
    }
    if (!ok) {
        return false;
    }

    connected = true;
    last_dpidr = dpidr;
    (void)swd_abort_clear();

    uint32_t stat = 0;
    if (!raiden_swd_power_up_debug(&stat)) {
        // Loud, but deliberately NOT the word ERROR and deliberately not fatal:
        // the DP answered, so the chip is not mute. Failing the connect here
        // would file a live DP as no_dp and hide the real fault, which is the
        // one confusion this bench cannot afford.
        rp_printf("WARNING: debug domain did not acknowledge power-up (CTRL/STAT=0x%08X)\r\n",
                  (unsigned)stat);
    }
    return true;
}

/** Every sub-command except CONNECT and SPEED goes through here. */
static bool ensure_connected(void) {
    swd_bus_wake();
    if (connected) {
        // Same as raiden's parser: clear the sticky bits before each
        // sub-command, so one faulted shot cannot poison the classification of
        // every shot after it. ABORT is the one register a DP accepts whatever
        // state it is in, so a refusal here means the LINK is gone -- the chip
        // was power-cycled, or the idle line was disturbed while parked. Fall
        // through and reconnect instead of reporting a failure whose cause was
        // the gap between two commands.
        if (swd_abort_clear()) {
            return true;
        }
        if (last_ack != RAIDEN_ACK_NONE) {
            // The DP answered, it just did not answer OK. A WAIT or a FAULT is
            // proof the link is alive -- the OPPOSITE of a lost one. Tearing
            // the link down here would spend a line reset in the middle of a
            // shot and never show up in the log as anything but a slow shot.
            return true;
        }
        connected = false;
    }
    if (swd_connect()) {
        return true;
    }
    rp_err("SWD connection failed (check target power and wiring)");
    return false;
}

/* --- MEM-AP ---------------------------------------------------------- */

static uint32_t block_words(uint32_t addr, uint32_t remaining) {
    uint32_t n = (remaining > MEM_BLOCK_WORDS) ? MEM_BLOCK_WORDS : remaining;
    uint32_t to_boundary = (AP_TAR_WINDOW - (addr & (AP_TAR_WINDOW - 1u))) / 4u;
    return (n > to_boundary) ? to_boundary : n;
}

static bool mem_read_block(uint32_t addr, uint32_t* buf, uint32_t nwords) {
    uint32_t csw = AP_CSW_WORD_INC;
    uint32_t tar = addr;

    if (!raiden_swd_power_up_debug(NULL)) {
        return false;
    }
    // One SELECT for the whole block: CSW, TAR and DRW all live in AP bank 0,
    // and nothing else runs between these transactions. What is never done is
    // carrying a SELECT across commands -- a stale one would serve the AHB-AP
    // under another AP's name, with an OK ACK and plausible values.
    if (!ap_select(0u, AP_CSW)) {
        return false;
    }
    if (!swd_xfer(true, false, AP_CSW, &csw)) {
        return false;
    }
    if (!swd_xfer(true, false, AP_TAR, &tar)) {
        return false;
    }
    uint32_t posted = 0;
    if (!swd_xfer(true, true, AP_DRW, &posted)) {
        return false;
    }
    for (uint32_t i = 0; i + 1u < nwords; i++) {
        if (!swd_xfer(true, true, AP_DRW, &buf[i])) {
            return false;
        }
    }
    return dp_read(DP_RDBUFF, &buf[nwords - 1u]);
}

/** Memory write of one TAR window's worth. @p nwords must fit the window. */
static bool mem_write_block(uint32_t addr, const uint32_t* values, uint32_t nwords) {
    uint32_t csw = AP_CSW_WORD_INC;
    uint32_t tar = addr;
    uint32_t rdbuff = 0;

    if (!raiden_swd_power_up_debug(NULL)) {
        return false;
    }
    if (!ap_select(0u, AP_CSW)) {
        return false;
    }
    if (!swd_xfer(true, false, AP_CSW, &csw)) {
        return false;
    }
    if (!swd_xfer(true, false, AP_TAR, &tar)) {
        return false;
    }
    for (uint32_t i = 0; i < nwords; i++) {
        uint32_t v = values[i]; // swd_xfer takes a mutable slot even to write
        if (!swd_xfer(true, false, AP_DRW, &v)) {
            return false;
        }
    }
    // RDBUFF flushes the last posted write. Without it a write that faulted
    // would surface on whatever transaction came next -- reported against the
    // wrong address, which reads as a second, unrelated failure.
    return dp_read(DP_RDBUFF, &rdbuff);
}

/** Single-word memory write: the halt request, and the core-debug registers. */
static bool mem_write_word(uint32_t addr, uint32_t value) {
    return mem_write_block(addr, &value, 1u);
}

/** Single-BYTE memory write. See AP_CSW_BYTE for why this exists at all.
 *
 * ADIv5 routes a sub-word access through the byte lane selected by the low
 * address bits, so the value is shifted into its lane before the DRW write.
 * Getting that shift wrong writes the right byte to the wrong quarter of the
 * word -- which, on an option byte, is indistinguishable from a part that
 * refused the write.
 */
bool raiden_swd_mem_write_byte(uint32_t addr, uint8_t value) {
    uint32_t csw = AP_CSW_BYTE;
    uint32_t tar = addr;
    uint32_t drw = (uint32_t)value << (8u * (addr & 3u));
    uint32_t rdbuff = 0;

    if (!raiden_swd_power_up_debug(NULL)) {
        return false;
    }
    if (!ap_select(0u, AP_CSW)) {
        return false;
    }
    if (!swd_xfer(true, false, AP_CSW, &csw)) {
        return false;
    }
    if (!swd_xfer(true, false, AP_TAR, &tar)) {
        return false;
    }
    if (!swd_xfer(true, false, AP_DRW, &drw)) {
        return false;
    }
    return dp_read(DP_RDBUFF, &rdbuff);
}

/* --- The seam the target-family modules use -------------------------- *
 *
 * Thin by design. A family module never sees a DP register, an APSEL or a CSW
 * field: it asks for memory, core registers, a halt or a resume. That is what
 * lets raiden_bat32.c be read for what it does to a BAT32 rather than for how
 * it drives ADIv5, and it is the only reason two families can share this file
 * without either one growing its own private variant of a transaction.
 */

bool raiden_swd_ensure_connected(void) {
    return ensure_connected();
}

bool raiden_swd_abort_clear(void) {
    return swd_abort_clear();
}

uint32_t raiden_swd_block_words(uint32_t addr, uint32_t remaining) {
    return block_words(addr, remaining);
}

bool raiden_swd_mem_read_block(uint32_t addr, uint32_t* buf, uint32_t nwords) {
    return mem_read_block(addr, buf, nwords);
}

bool raiden_swd_mem_write(uint32_t addr, const uint32_t* values, uint32_t nwords) {
    uint32_t done = 0;
    while (done < nwords) {
        uint32_t at = addr + done * 4u;
        uint32_t n = block_words(at, nwords - done);
        if (!mem_write_block(at, &values[done], n)) {
            return false;
        }
        done += n;
    }
    return true;
}

bool raiden_swd_read_dhcsr(uint32_t* out) {
    return mem_read_block(CM_DHCSR, out, 1u);
}

/** Wait for the core to finish a DCRSR transfer. */
static bool wait_regrdy(void) {
    for (uint32_t i = 0; i < REGRDY_TRIES; i++) {
        uint32_t dhcsr = 0;
        if (!mem_read_block(CM_DHCSR, &dhcsr, 1u)) {
            return false;
        }
        if ((dhcsr & DHCSR_IMPLAUSIBLE) != 0u) {
            return false; // a constant, not a status: polling it cannot help
        }
        if ((dhcsr & DHCSR_S_REGRDY) != 0u) {
            return true;
        }
        busy_wait_us_32(REGRDY_POLL_US);
    }
    return false;
}

bool raiden_swd_core_reg_write(uint8_t regsel, uint32_t value) {
    // DCRDR first. DCRSR is what STARTS the transfer, so writing it before the
    // data would hand the core whatever the previous access left in DCRDR --
    // a wrong register value that the readback of a different register would
    // never contradict.
    if (!mem_write_word(CM_DCRDR, value)) {
        return false;
    }
    if (!mem_write_word(CM_DCRSR, DCRSR_WRITE | (uint32_t)regsel)) {
        return false;
    }
    return wait_regrdy();
}

bool raiden_swd_core_reg_read(uint8_t regsel, uint32_t* out) {
    if (!mem_write_word(CM_DCRSR, (uint32_t)regsel)) {
        return false;
    }
    if (!wait_regrdy()) {
        return false;
    }
    return mem_read_block(CM_DCRDR, out, 1u);
}

void raiden_swd_forget_connection(void) {
    connected = false;
    raiden_ctrlap_forget();
    swd_park_pins();
}

bool raiden_swd_resume(void) {
    return mem_write_word(CM_DHCSR, DHCSR_RESUME_REQ);
}

bool raiden_swd_halt(uint32_t timeout_ms, uint32_t* dhcsr_out) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    if (!mem_write_word(CM_DHCSR, DHCSR_HALT_REQ)) {
        return false;
    }
    for (;;) {
        uint32_t dhcsr = 0;
        if (!mem_read_block(CM_DHCSR, &dhcsr, 1u)) {
            return false;
        }
        if (dhcsr_out != NULL) {
            *dhcsr_out = dhcsr;
        }
        if ((dhcsr & DHCSR_IMPLAUSIBLE) != 0u) {
            // Where a locked part lands. Giving up AT ONCE rather than after
            // the timeout is what keeps the nRF52 campaign's per-shot halt
            // costing what it costed before this loop existed.
            return false;
        }
        if ((dhcsr & DHCSR_S_HALT) != 0u) {
            return true;
        }
        if ((dhcsr & DHCSR_S_RESET_ST) != 0u) {
            // The core was in reset, so nobody is holding the request any
            // more. Re-issue it instead of waiting out a timeout on a request
            // that no longer exists.
            if (!mem_write_word(CM_DHCSR, DHCSR_HALT_REQ)) {
                return false;
            }
        }
        if (time_reached(deadline)) {
            return false;
        }
        busy_wait_us_32(HALT_POLL_US);
    }
}

/* --- Commands -------------------------------------------------------- */

/** Addresses and values MUST carry the 0x prefix.
 *
 * raiden_parse_u32() defaults to base 10 while raiden's own parser defaults to
 * base 16 for addresses, so a bare token does not fail -- it succeeds at the
 * wrong address. `SWD READ 10000100` would read 0x989680 and report it without
 * a word of complaint, and on an anchor check that reads exactly like a glitch
 * effect. Word counts and AP indices keep the plain parser: raiden reads those
 * base 0, so decimal is what they are meant to be.
 */
static bool parse_hex_arg(const char* s, uint32_t* out) {
    return raiden_parse_hex32(s, out);
}

static void cmd_connect(void) {
    if (swd_connect()) {
        rp_ok("Connected, DPIDR=0x%08X", (unsigned)last_dpidr);
    } else {
        rp_err("SWD connect failed (check target power and wiring)");
    }
}

static void cmd_speed(int argc, char* argv[]) {
    if (argc < 3) {
        if (clk_delay_us == 0u) {
            rp_printf("SWD clock delay: 0 (MAX, no delay)\r\n");
        } else {
            rp_printf("SWD clock delay: %lu us (~%lu kHz)\r\n",
                      (unsigned long)clk_delay_us, (unsigned long)(1000u / (2u * clk_delay_us)));
        }
        return;
    }
    uint32_t delay = 0;
    if (!raiden_parse_u32(argv[2], &delay) || delay > SWD_SPEED_MAX_US) {
        rp_err("Invalid SWD speed (0 for max, 1-%u us)", (unsigned)SWD_SPEED_MAX_US);
        return;
    }
    clk_delay_us = delay;
    if (delay == 0u) {
        rp_ok("SWD speed set to 0 (MAX, no delay)");
    } else {
        rp_ok("SWD speed set to %lu us (~%lu kHz)",
              (unsigned long)delay, (unsigned long)(1000u / (2u * delay)));
    }
}

static void cmd_idcode(void) {
    uint32_t dpidr = 0;
    if (!dp_read(DP_DPIDR, &dpidr) || dpidr == 0u) {
        rp_err("Could not read DPIDR");
        return;
    }
    last_dpidr = dpidr;
    unsigned designer = (unsigned)((dpidr >> 1) & 0x7FFu);
    rp_printf("DPIDR:    0x%08X\r\n", (unsigned)dpidr);
    rp_printf("  Designer: 0x%03X%s, PartNo: 0x%02X, Rev: %u, Ver: %u\r\n",
              designer, (designer == 0x23Bu) ? " (ARM)" : "",
              (unsigned)((dpidr >> 20) & 0xFFu),
              (unsigned)((dpidr >> 28) & 0xFu),
              (unsigned)((dpidr >> 12) & 0xFu));

    uint32_t cpuid = 0;
    if (!mem_read_block(CM_CPUID, &cpuid, 1u)) {
        // Only a failed TRANSACTION reaches this. A locked chip does not: its
        // disabled AHB-AP answers OK with 0x23000000, which is printed below as
        // the value it is. The host maps any ERROR here to "bench mute", and
        // mute-versus-locked is the confusion the whole project is built to
        // avoid -- so this must never fire because of a value.
        rp_err("Could not read CPUID/debug registers");
        return;
    }
    unsigned implementer = (unsigned)((cpuid >> 24) & 0xFFu);
    unsigned part = (unsigned)((cpuid >> 4) & 0xFFFu);
    const char* core = "Unknown";
    if (part == 0xC23u) {
        core = "Cortex-M3";
    } else if (part == 0xC24u) {
        core = "Cortex-M4";
    } else if (part == 0xC27u) {
        core = "Cortex-M7";
    } else if (part == 0xC60u) {
        core = "Cortex-M0+";
    }
    rp_printf("CPUID:    0x%08X\r\n", (unsigned)cpuid);
    rp_printf("  Implementer: 0x%02X%s\r\n", implementer, (implementer == 0x41u) ? " (ARM)" : "");
    rp_printf("  Core: %s r%up%u\r\n", core,
              (unsigned)((cpuid >> 20) & 0xFu), (unsigned)(cpuid & 0xFu));
}

/** SWD PHY -- accepte BITBANG, refuse PIO bruyamment.
 *
 * Le dialecte a deux couches physiques ; ce binmode n'en a qu'une, et c'est
 * un choix : le budget PIO appartient au moteur de glitch, et le SWD est
 * bit-bange pour ne rien lui prendre. Refuser PIO A VOIX HAUTE plutot que de
 * l'accepter en ne faisant rien -- scripts/bat32_dump.py teste la reponse de
 * `SWD PHY PIO` et s'arrete proprement sur un refus, alors qu'un acquittement
 * menteur lui ferait dumper des heures en croyant tourner a 2,5 MHz.
 */
static void cmd_phy(int argc, char* argv[]) {
    if (argc < 3) {
        rp_ok("PHY: BITBANG (the only physical layer in this binmode)");
        return;
    }
    if (strcmp(argv[2], "BITBANG") == 0) {
        rp_ok("PHY: BITBANG");
    } else if (strcmp(argv[2], "PIO") == 0) {
        rp_err("SWD PHY PIO is not implemented here: the PIO budget belongs to "
               "the glitch engine and this SWD is bit-banged on purpose. Use "
               "SWD PHY BITBANG with SWD SPEED <n>.");
    } else {
        rp_err("Unknown SWD PHY '%s' (BITBANG only in this binmode)", argv[2]);
    }
}

static void cmd_halt(void) {
    // One request does not always take on a core that is running -- which is
    // the BAT32 case, where SWD HALT is issued against application firmware in
    // mid-execution and a single shot was enough only by luck. Both replies
    // are unchanged: a locked part still fails, and still fails immediately.
    if (!raiden_swd_halt(HALT_CLI_TIMEOUT_MS, NULL)) {
        rp_err("Halt failed");
        return;
    }
    rp_ok("Target halted");
}

static void cmd_read_mem(uint32_t addr, uint32_t nwords) {
    uint32_t done = 0;

    // UN entete et UN trailer par COMMANDE, jamais par bloc. Le bloc de 16 mots
    // est un detail de transport -- il existe pour ne pas noyer la FIFO TX, pas
    // pour decouper la reponse. Une version anterieure emettait le hexdump
    // COMPLET par bloc : une lecture de 256 mots rendait alors SEIZE
    // "OK: Read complete". Un hote qui arrete sa lecture peu apres le premier
    // marqueur -- ce que fait scripts/bat32_dump.py du raiden, 50 ms -- perdait
    // tout le reste et ecrivait un dump silencieusement INCOMPLET. Le faux banc
    // des tests n'en emettait qu'un seul, donc rien ne signalait l'ecart.
    rp_hexdump_begin(addr, nwords * 4u);
    while (done < nwords) {
        uint32_t at = addr + done * 4u;
        uint32_t n = block_words(at, nwords - done);
        if (!mem_read_block(at, mem_buf, n)) {
            // One recovery attempt, as raiden does: a sticky error left by the
            // previous transaction would otherwise fail every read after it.
            (void)swd_abort_clear();
            if (!mem_read_block(at, mem_buf, n)) {
                rp_err("Read failed at 0x%08X (ACK=0x%X)", (unsigned)at, (unsigned)last_ack);
                return;
            }
        }
        rp_hexdump_lines(at, (const uint8_t*)mem_buf, n * 4u);
        done += n;
    }
    rp_hexdump_end();
}

static void cmd_read(int argc, char* argv[]) {
    if (argc < 3) {
        rp_err("Usage: SWD READ <addr> [<words>] | SWD READ DP <addr> | SWD READ AP [<n>] <addr>");
        return;
    }

    if (strcmp(argv[2], "DP") == 0) {
        uint32_t addr = 0;
        if (argc < 4 || !parse_hex_arg(argv[3], &addr)) {
            rp_err("Invalid address (0x prefix required). Usage: SWD READ DP <addr>");
            return;
        }
        addr &= 0xCu;
        if (!ensure_connected()) {
            return;
        }
        uint32_t value = 0;
        if (dp_read((uint8_t)addr, &value)) {
            rp_reg_dp((uint8_t)addr, value);
        } else {
            rp_err("DP read failed (ACK=0x%X)", (unsigned)last_ack);
        }
        return;
    }

    if (strcmp(argv[2], "AP") == 0) {
        uint32_t apsel = 0;
        uint32_t addr = 0;
        const char* addr_token = (argc >= 4) ? argv[3] : NULL;
        if (argc >= 5) {
            // The AP index is resolved by ARGUMENT COUNT, never by value.
            // `SWD READ AP 1` reads AP[0x01] of AP 0; `SWD READ AP 1 0xFC`
            // reads the IDR of AP 1. Reading the index out of the value would
            // make the CTRL-AP unreachable, and with it every dialogue with a
            // locked chip -- the only dialogue that still works on one.
            if (!raiden_parse_u32(argv[3], &apsel) || apsel > 0xFFu) {
                rp_err("Invalid AP index (0-255). Usage: SWD READ AP [<n>] <addr>");
                return;
            }
            addr_token = argv[4];
        }
        if (!parse_hex_arg(addr_token, &addr) || addr > 0xFFu) {
            rp_err("Invalid address (0x prefix required, 0x00-0xFC). "
                   "Usage: SWD READ AP [<n>] <addr>");
            return;
        }
        if (!ensure_connected()) {
            return;
        }
        uint32_t value = 0;
        if (ap_read_routed((uint8_t)apsel, (uint8_t)addr, &value)) {
            rp_reg_ap((uint8_t)apsel, (uint8_t)addr, value);
        } else {
            rp_err("AP read failed (ACK=0x%X)", (unsigned)last_ack);
        }
        return;
    }

    uint32_t addr = 0;
    uint32_t nwords = 1;
    if (!parse_hex_arg(argv[2], &addr)) {
        rp_err("Invalid address (0x prefix required). Usage: SWD READ <addr> [<words>]");
        return;
    }
    // The count is in WORDS, not bytes, and defaults to one -- same as raiden,
    // whose host multiplies by four on the other side.
    if (argc >= 4 && (!raiden_parse_u32(argv[3], &nwords) || nwords == 0u)) {
        rp_err("Invalid count (number of 32-bit words, 1 or more)");
        return;
    }
    if (nwords > SWD_READ_MAX_WORDS) {
        rp_err("Word count too large (max %u words)", (unsigned)SWD_READ_MAX_WORDS);
        return;
    }
    if ((addr & 3u) != 0u) {
        rp_err("Address 0x%08X is not word aligned (32-bit accesses only)", (unsigned)addr);
        return;
    }
    if (nwords - 1u > (0xFFFFFFFFu - addr) / 4u) {
        rp_err("Read runs past the end of the address space");
        return;
    }
    if (!ensure_connected()) {
        return;
    }
    cmd_read_mem(addr, nwords);
}

static void cmd_write(int argc, char* argv[]) {
    if (argc < 3) {
        rp_err("Usage: SWD WRITE DP <addr> <value> | SWD WRITE AP [<n>] <addr> <value>");
        return;
    }

    if (strcmp(argv[2], "DP") == 0) {
        uint32_t addr = 0;
        uint32_t value = 0;
        if (argc < 5 || !parse_hex_arg(argv[3], &addr) || !parse_hex_arg(argv[4], &value)) {
            rp_err("Invalid argument (0x prefix required). Usage: SWD WRITE DP <addr> <value>");
            return;
        }
        addr &= 0xCu;
        if (!ensure_connected()) {
            return;
        }
        rp_printf("Writing DP[0x%X] = 0x%08X\r\n", (unsigned)addr, (unsigned)value);
        if (!dp_write((uint8_t)addr, value)) {
            rp_err("DP write failed (ACK=0x%X)", (unsigned)last_ack);
            return;
        }
        if (addr == DP_ABORT) {
            // Address 0x0 is ABORT on write and DPIDR on read: two different
            // registers. Reading it back would print a DPIDR nobody wrote,
            // under the name DP[0x0]. Echo what was written, as the line above
            // already announced it.
            rp_reg_dp((uint8_t)addr, value);
            return;
        }
        uint32_t back = 0;
        if (!dp_read((uint8_t)addr, &back)) {
            rp_err("DP[0x%X] write ACKed but read-back failed (ACK=0x%X), the write may have landed",
                   (unsigned)addr, (unsigned)last_ack);
            return;
        }
        rp_reg_dp((uint8_t)addr, back);
        return;
    }

    if (strcmp(argv[2], "AP") == 0) {
        uint32_t apsel = 0;
        uint32_t addr = 0;
        uint32_t value = 0;
        const char* addr_token = (argc >= 4) ? argv[3] : NULL;
        const char* value_token = (argc >= 5) ? argv[4] : NULL;
        if (argc >= 6) {
            // Same rule as SWD READ AP, and for the same reason: three
            // arguments mean an explicit APSEL, two mean AP 0.
            if (!raiden_parse_u32(argv[3], &apsel) || apsel > 0xFFu) {
                rp_err("Invalid AP index (0-255). Usage: SWD WRITE AP [<n>] <addr> <value>");
                return;
            }
            addr_token = argv[4];
            value_token = argv[5];
        }
        if (!parse_hex_arg(addr_token, &addr) || addr > 0xFFu ||
            !parse_hex_arg(value_token, &value)) {
            rp_err("Invalid argument (0x prefix required). "
                   "Usage: SWD WRITE AP [<n>] <addr> <value>");
            return;
        }
        if (!ensure_connected()) {
            return;
        }
        if (apsel == 0u) {
            rp_printf("Writing AP[0x%02X] = 0x%08X\r\n", (unsigned)addr, (unsigned)value);
        } else {
            rp_printf("Writing AP%u[0x%02X] = 0x%08X\r\n",
                      (unsigned)apsel, (unsigned)addr, (unsigned)value);
        }
        if (!ap_write_routed((uint8_t)apsel, (uint8_t)addr, value)) {
            rp_err("AP write failed (ACK=0x%X)", (unsigned)last_ack);
            return;
        }
        if (apsel == (uint32_t)RAIDEN_CTRLAP_APSEL && raiden_ctrlap_write_only((uint8_t)addr)) {
            // See raiden_ctrlap.h: reading ERASEALL back would report an erase
            // that DID happen as one that was refused.
            rp_reg_ap((uint8_t)apsel, (uint8_t)addr, value);
            return;
        }
        uint32_t back = 0;
        if (!ap_read_routed((uint8_t)apsel, (uint8_t)addr, &back)) {
            rp_err("AP write ACKed but read-back failed (ACK=0x%X), the write may have landed",
                   (unsigned)last_ack);
            return;
        }
        rp_reg_ap((uint8_t)apsel, (uint8_t)addr, back);
        return;
    }

    // Deliberately absent. This bench reads a target and classifies it; a
    // memory write here would be one typo away from a flash controller
    // register, and the project keeps everything irreversible behind a
    // different door.
    rp_err("Memory writes are not implemented in this binmode "
           "(SWD WRITE DP <addr> <value> or SWD WRITE AP [<n>] <addr> <value> only)");
}

void raiden_swd_command(int argc, char* argv[]) {
    if (argc < 2) {
        rp_err("Usage: SWD <CONNECT|IDCODE|HALT|SPEED|PHY|OPT|READ|WRITE|BAT32>");
        swd_park_pins();
        return;
    }
    const char* sub = argv[1];

    if (strcmp(sub, "BAT32") == 0) {
        // Its own module: a family's memory map and its bypass sequence do not
        // belong in the transport that carries them.
        raiden_bat32_command(argc, argv);
    } else if (strcmp(sub, "CONNECT") == 0) {
        cmd_connect();
    } else if (strcmp(sub, "SPEED") == 0) {
        cmd_speed(argc, argv);
    } else if (strcmp(sub, "IDCODE") == 0) {
        if (ensure_connected()) {
            cmd_idcode();
        }
    } else if (strcmp(sub, "PHY") == 0) {
        cmd_phy(argc, argv);
    } else if (strcmp(sub, "OPT") == 0) {
        raiden_bat32_opt();
    } else if (strcmp(sub, "HALT") == 0) {
        if (ensure_connected()) {
            cmd_halt();
        }
    } else if (strcmp(sub, "READ") == 0) {
        cmd_read(argc, argv);
    } else if (strcmp(sub, "WRITE") == 0) {
        cmd_write(argc, argv);
    } else {
        // Never a default branch. A sub-command that quietly did nothing would
        // let a campaign keep shooting and keep scoring, against a bench that
        // was not doing what the operator believes.
        rp_err("Unknown SWD sub-command '%s' "
               "(use CONNECT/IDCODE/HALT/SPEED/PHY/OPT/READ/WRITE/BAT32)", sub);
    }

    // Released after EVERY command, not just at mode exit: the power cycle of
    // the next shot happens between two commands, and lines left driven would
    // keep the target alive through its ESD diodes.
    swd_park_pins();
}

void raiden_swd_init(void) {
    clk_delay_us = SWD_SPEED_DEFAULT_US;
    connected = false;
    last_ack = RAIDEN_ACK_NONE;
    last_dpidr = 0;
    raiden_ctrlap_forget();
    // No bio_init() here. raiden_power_init() has already run it, and running
    // it again would put EVERY BIO back to input -- BIO0 included, whose idle
    // level is the only thing keeping the crowbar MOSFET open.
    swd_park_pins();
    system_bio_update_purpose_and_label(true, RAIDEN_BIO_SWCLK, BP_PIN_MODE, "SCLK");
    system_bio_update_purpose_and_label(true, RAIDEN_BIO_SWDIO, BP_PIN_MODE, "SDIO");
    system_bio_update_purpose_and_label(true, RAIDEN_BIO_NRST, BP_PIN_MODE, "NRST");
}

void raiden_swd_deinit(void) {
    connected = false;
    raiden_ctrlap_forget();
    swd_park_pins();
    system_bio_update_purpose_and_label(false, RAIDEN_BIO_SWCLK, BP_PIN_MODE, 0);
    system_bio_update_purpose_and_label(false, RAIDEN_BIO_SWDIO, BP_PIN_MODE, 0);
    system_bio_update_purpose_and_label(false, RAIDEN_BIO_NRST, BP_PIN_MODE, 0);
}
