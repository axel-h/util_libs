/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <platsupport/chardev.h>
#include <platsupport/plat/serial.h>

/*****************************
 **** Serial device flags ****
 *****************************/

/* Auto-send CR (Carriage Return, "\r") before each "\n". All UART drivers
 * should set this flag by default, so the UART can be used as a console.
 */
#define SERIAL_AUTO_CR          BIT(0)

/* Do not block if the TX FIFO is full, but return an error. When SERIAL_AUTO_CR
 * is enabled, CR+LF is considered as an atom, ie either nothing is sent or both
 * CR and LF are sent. If the underlying UART implementation can't ensure the TX
 * FIFO has space for both chars, it is allowed to block after CR has been sent
 * to ensure LF can also be sent. Rational for this is, that SERIAL_AUTO_CR
 * usually implies that the UART is used as a console.  Blocking in this corner
 * case can be neglected considering the issues caused by a missing LF and a CR
 * getting sent twice then.
 */
#define SERIAL_TX_NONBLOCKING   BIT(1)

/*****************************/

enum serial_parity {
    PARITY_NONE,
    PARITY_EVEN,
    PARITY_ODD
};

/*
 * Initialiase a device
 * @param  id: the id of the character device
 * @param ops: a structure containing OS specific operations for memory access
 * @param dev: a character device structure to populate
 * @return   : 0 on success
 */
int serial_init(enum chardev_id id,
                ps_io_ops_t* ops,
                ps_chardevice_t* dev);

/*
 * Performs line configuration of a serial port
 * @param       dev: an initialised character device structure
 * @param       bps: The desired boad rate
 * @param char_size: The desired character size
 * @param stop_bits: The number of stop bits
 */
int serial_configure(ps_chardevice_t* dev,
                     long bps,
                     int  char_size,
                     enum serial_parity parity,
                     int  stop_bits);

