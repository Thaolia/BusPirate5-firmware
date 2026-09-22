/**
 * @file raiden_binmode.h
 * @brief Binary mode speaking the raiden-pico command dialect.
 *
 * Why a binmode and not a protocol mode: the binary CDC (interface 1) carries
 * no VT100 terminal, no line editor, no screen-size handshake and no HiZ
 * gate -- so the dialect goes out in clear text and the unmodified host client
 * attaches to it. The terminal CDC could not do that; driving it from a script
 * cost this project four separate measured failures.
 *
 * This is an INDEPENDENT REIMPLEMENTATION of a command surface, not a port:
 * raiden-pico ships no licence at all, so none of its code may be copied into
 * this MIT tree. What is reproduced here is the interface -- roughly 25 command
 * strings and their reply formats -- because that is what the host parses.
 */
#ifndef RAIDEN_BINMODE_H
#define RAIDEN_BINMODE_H

void raiden_binmode_setup(void);
void raiden_binmode_setup_message(void);
void raiden_binmode_service(void);
void raiden_binmode_cleanup(void);

extern const char raiden_binmode_name[];

#endif
