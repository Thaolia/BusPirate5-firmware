/**
 * @file raiden_clock.h
 * @brief System-clock control for the raiden-dialect binmode.
 *
 * The glitch step is one PIO instruction, and one PIO instruction is one
 * clk_sys cycle once the divider is 1.0. So clk_sys IS the resolution:
 *
 *   125 MHz (stock) -> 8.00 ns      200 MHz -> 5.00 ns
 *   133 MHz (spec)  -> 7.52 ns      250 MHz -> 4.00 ns
 *
 * Above 133 MHz the RP2040 is out of datasheet specification. That is allowed
 * here, but it must be MEASURED on the board in hand and written down, never
 * assumed: a part that browns out mid-campaign does not raise an error, it
 * returns no_dp shots that look exactly like a chip refusing to open.
 *
 * What this file does NOT do: it does not touch the flash divider. Above
 * ~266 MHz, clk_sys/2 exceeds the usual 133 MHz QSPI ceiling and
 * PICO_FLASH_SPI_CLKDIV must go to 4 -- a build-time change, not a runtime one.
 */
#ifndef RAIDEN_CLOCK_H
#define RAIDEN_CLOCK_H

#include <stdbool.h>
#include <stdint.h>

/** Remember the entry frequency and apply the configured one, if any. */
void raiden_clock_apply(void);

/** Put back the frequency and core voltage found at entry.
 *
 * Mandatory. The in-tree precedent forgets it: sump_logic_analyzer_cleanup()
 * leaves the part at whatever TURBO_200MHZ set, so the NEXT mode runs
 * overclocked without anyone having asked -- and the failure that follows looks
 * unrelated to the mode that caused it.
 */
void raiden_clock_restore(void);

/** Change frequency (and optionally core voltage) at runtime.
 *
 * @param khz     target clk_sys in kHz
 * @param core_mv core voltage, 850..1300 mV; 0 leaves it alone
 * @return false if the PLL cannot produce this frequency -- nothing changed
 */
bool raiden_clock_set(uint32_t khz, uint32_t core_mv);

/** Current clk_sys in Hz. The PIO divider and every reported figure use this. */
uint32_t raiden_clock_hz(void);

/** Picoseconds per PIO instruction at the current clk_sys.
 *
 * Picoseconds, not nanoseconds: at 250 MHz a step is 4 ns, and integer
 * nanoseconds would quantise the very number the operator is trying to read.
 */
uint32_t raiden_clock_step_ps(void);

#endif
