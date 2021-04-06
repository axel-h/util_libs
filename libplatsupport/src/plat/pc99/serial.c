/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdlib.h>
#include <string.h>
#include <utils/util.h>

#include "../../chardev.h"

/*
 * Port offsets
 * W    - write
 * R    - read
 * RW   - read and write
 * DLAB - Alternate register function bit
 */

#define SERIAL_THR  0 /* Transmitter Holding Buffer (W ) DLAB = 0 */
#define SERIAL_RBR  0 /* Receiver Buffer            (R ) DLAB = 0 */
#define SERIAL_DLL  0 /* Divisor Latch Low Byte     (RW) DLAB = 1 */
#define SERIAL_IER  1 /* Interrupt Enable Register  (RW) DLAB = 0 */
#define SERIAL_DLH  1 /* Divisor Latch High Byte    (RW) DLAB = 1 */
#define SERIAL_IIR  2 /* Interrupt Identification   (R ) */
#define SERIAL_FCR  2 /* FIFO Control Register      (W ) */
#define SERIAL_LCR  3 /* Line Control Register      (RW) */
#define SERIAL_MCR  4 /* Modem Control Register     (RW) */
#define SERIAL_LSR  5 /* Line Status Register       (R ) */
#define SERIAL_MSR  6 /* Modem Status Register      (R ) */
#define SERIAL_SR   7 /* Scratch Register           (RW) */

#define SERIAL_DLAB BIT(7)
#define SERIAL_LSR_DATA_READY BIT(0)
#define SERIAL_LSR_TRANSMITTER_EMPTY BIT(5)

/*
 *******************************************************************************
 * UART access primitives
 *******************************************************************************
 */

static uint32_t get_console_io_port(
    ps_chardevice_t *device,
    unsigned int offset)
{
    /* Casting points to a specific integer directly is not allowed, must cast
     * to uintptr_t and then cast to a specific integer type.
     */
    return (uint32_t)((uintptr_t)device->vaddr) + offset;
}

static void console_io_port_read(
    ps_chardevice_t *device,
    unsigned int port_offset,
    uint32_t *data)
{
    return ps_io_port_in(
                &device->ioops.io_port_ops,
                get_console_io_port(device, port_offset),
                1, /* io_size */
                data);
}

static void console_io_port_write(
    ps_chardevice_t *device,
    unsigned int port_offset,
    uint32_t data)
{
    return ps_io_port_out(
                &device->ioops.io_port_ops,
                get_console_io_port(device, port_offset),
                1, /* io_size */
                data);
}

static int serial_is_tx_ready(ps_chardevice_t* device)
{
    uint32_t data;
    int ret = console_io_port_read(device, SERIAL_LSR, &data);
    if (ret != 0) {
        return 0; /* claim transmitter is not ready */
    }
    return data & SERIAL_LSR_TRANSMITTER_EMPTY;
}

static int serial_tx_byte(ps_chardevice_t* device, uint8_t byte)
{
    return console_io_port_write(device, SERIAL_THR, byte);
}

/*
 *******************************************************************************
 * UART access API
 *******************************************************************************
 */

int uart_getchar(ps_chardevice_t *device)
{
    int ret;
    uint32_t data;

    /* Check if character is available. */
    ret = console_io_port_read(device, SERIAL_LSR, &data);
    if (ret != 0) {
        return -1;
    }
    if (!(data & SERIAL_LSR_DATA_READY)) {
        return -1;
    }

    /* retrieve character */
    ret = console_io_port_read(device, SERIAL_RBR, &data);
    if (ret != 0) {
        return -1;
    }

    return (uint8_t)data;
}

int uart_putchar(ps_chardevice_t* device, int c)
{
    /* Check if serial transmitter is ready. */
    if (!serial_is_tx_ready(device)) {
        return -1;
    }

    /* Extract the byte to send, drop any flags. */
    uint8_t byte = (uint8_t)c;

    if (byte == '\n') {
        /* If we output immediately then odds are the transmit buffer will be
         * full, so we have to wait. */
        (void)serial_tx_byte(device, '\r');
        while (!serial_is_tx_ready(device)) {
            /* busy waiting loop */
        }
    }

    /* Write out the character, ignore return code. */
    (void)serial_tx_byte(device, byte);

    return byte;
}

static void uart_handle_irq(ps_chardevice_t* device UNUSED)
{
    /* No IRQ handling required here. */
}

int
uart_init(const struct dev_defn* defn, const ps_io_ops_t* ops, ps_chardevice_t* dev)
{
    memset(dev, 0, sizeof(*dev));
    /* Set up all the  device properties. */
    dev->id         = defn->id;
    dev->vaddr      = (void*) defn->paddr; /* Save the IO port base number. */
    dev->read       = &uart_read;
    dev->write      = &uart_write;
    dev->handle_irq = &uart_handle_irq;
    dev->irqs       = defn->irqs;
    dev->ioops      = *ops;

    /* Initialise the device. */

    /* clear DLAB - Divisor Latch Access Bit */
    if (console_io_port_write(dev, SERIAL_LCR, 0x00 & ~SERIAL_DLAB) != 0) {
        return -1;
    }

    /* disable generating interrupts */
    if (console_io_port_write(dev, SERIAL_IER, 0x00) != 0) {
        return -1;
    }

    /* set DLAB to*/
    if (console_io_port_write(dev, SERIAL_LCR, 0x00 | SERIAL_DLAB) != 0) {
        return -1;
    }
    /* set low byte of divisor to 0x01 = 115200 baud */
    if (console_io_port_write(dev, SERIAL_DLL, 0x01) != 0) {
        return -1;
    }
    /* set high byte of divisor to 0x00 */
    if (console_io_port_write(dev, SERIAL_DLH, 0x00) != 0) {
        return -1;
    }

    /* line control register: set 8 bit, no parity, 1 stop bit; clear DLAB */
    if (console_io_port_write(dev, SERIAL_LCR, 0x03 & ~SERIAL_DLAB) != 0) {
        return -1;
    }
    /* modem control register: set DTR/RTS/OUT2 */
    if (console_io_port_write(dev, SERIAL_MCR, 0x0b) != 0) {
        return -1;
    }

    uint32_t temp;
    /* clear receiver port */
    if (console_io_port_write(dev, SERIAL_RBR, &temp) != 0) {
        return -1;
    }
    /* clear line status port */
    if (console_io_port_write(dev, SERIAL_LSR, &temp) != 0) {
        return -1;
    }
    /* clear modem status port */
    if (console_io_port_write(dev, SERIAL_MSR, &temp) != 0) {
        return -1;
    }

    /* Enable the receiver interrupt. */
    if (console_io_port_write(dev, SERIAL_IER, 0x01) != 0) {
        return -1;
    }

    return 0;
}
