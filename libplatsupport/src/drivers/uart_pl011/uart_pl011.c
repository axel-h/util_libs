/*
 * ARM PL011 UART driver
 *
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <platsupport/serial.h>
#include "../../chardev.h"


#define PL011_UART_RHR_MASK      MASK(8)
#define PL011_UART_FR_TXFF       BIT(5)
#define PL011_UART_FR_RXFE       BIT(4)

typedef volatile struct {
    uint32_t dr;        /* 0x00 */
    uint32_t reg_04;    /* 0x04 */
    uint32_t reg_08;    /* 0x08 */
    uint32_t reg_0C;    /* 0x0C */
    uint32_t reg_10;    /* 0x10 */
    uint32_t reg_14;    /* 0x14 */
    uint32_t fr;        /* 0x18 */
    uint32_t reg_1C;    /* 0x1C */
    uint32_t reg_20;    /* 0x20 */
    uint32_t reg_24;    /* 0x24 */
    uint32_t reg_28;    /* 0x28 */
    uint32_t reg_2C;    /* 0x2C */
    uint32_t reg_30;    /* 0x30 */
    uint32_t reg_34;    /* 0x34 */
    uint32_t imsc;      /* 0x38 */
    uint32_t reg_3C;    /* 0x3C */
    uint32_t reg_40;    /* 0x40 */
    uint32_t cr;        /* 0x44 */
} uart_pl011_regs_t;



static uart_pl011_regs_t* get_uart_regs(ps_chardevice_t *d)
{
    return (uart_pl011_regs_t *)(d->vaddr);
}

/*
 *******************************************************************************
 * UART access primitives
 *******************************************************************************
 */

static int internal_uart_is_tx_fifo_full(uart_pl011_regs_t *regs)
{
    return regs->fr & PL011_UARTFR_TX_FF;
}

static void internal_uart_tx_byte(uart_pl011_regs_t *regs, uint8_t byte)
{
    regs->dr = byte;
}

static uint8_t internal_uart_rx_byte(uart_pl011_regs_t *regs)
{
    return (uint8_t)(regs->dr & PL011_UART_RHR_MASK);
}

static int internal_uart_is_rx_empty(uart_pl011_regs_t *regs)
{
    return reg->fr & PL011_UART_FR_RXFE;
}

static void internal_uart_busy_wait_tx_ready(uart_pl011_regs_t *regs)
{
    while (internal_uart_is_tx_fifo_full(regs)) {
        /* busy waiting loop */
    }
}

/*
 *******************************************************************************
 * UART access API
 *******************************************************************************
 */

int uart_getchar(ps_chardevice_t *dev)
{
    uart_pl011_regs_t *regs = get_uart_regs(dev);

    if (internal_uart_is_rx_empty(regs)) {
        return -1;
    }

    return internal_uart_rx_byte(regs);
}

int uart_putchar(ps_chardevice_t* dev, int c)
{
    uart_pl011_regs_t *regs = get_uart_regs(dev);

    /* Check if the TX FIFO has space. If not and SERIAL_TX_NONBLOCKING is set,
     * then fail the call, otherwise do busy waiting.
     */
    if (internal_uart_is_tx_fifo_full(regs))
        if (d->flags & SERIAL_TX_NONBLOCKING) {
            return -1;
        }
        internal_uart_busy_wait_tx_ready(regs);
    }

    /* Extract the byte to send, drop any flags. */
    uint8_t byte = (uint8_t)c;

    internal_uart_busy_wait_tx_ready(regs);

    /* SERIAL_AUTO_CR enables sending a CR before any LF, which is the common
     * thing to do for a serial terminal. CR/LR are considered an atom, thus a
     * blocking wait will be used even if SERIAL_TX_NONBLOCKING is set to ensure
     * LF is sent.
     * TODO: Check in advance if the TX FIFO has space for two chars if
     *       SERIAL_TX_NONBLOCKING is set.
     */
    if (byte == '\n' && (d->flags & SERIAL_AUTO_CR)) {
        internal_uart_tx_byte(regs, '\r');
        internal_uart_busy_wait_tx_ready(regs);
    }

    internal_uart_tx_byte(regs, byte);

    return byte;
}

static void
uart_handle_irq(ps_chardevice_t* dev)
{
    uart_pl011_regs_t *regs = get_uart_regs(dev);

    regs->cr  = 0x7f0;
}

int uart_init(const struct dev_defn* defn,
              const ps_io_ops_t* ops,
              ps_chardevice_t* dev)
{
    memset(dev, 0, sizeof(*dev));

    uart_pl011_regs_t *regs = (uart_pl011_regs_t *)chardev_map(defn, ops);
    if (regs == NULL) {
        return -1;
    }

    /* Set up all the device properties. */
    dev->id         = defn->id;
    dev->vaddr      = (void *)regs;
    dev->read       = &uart_read;
    dev->write      = &uart_write;
    dev->handle_irq = &uart_handle_irq;
    dev->irqs       = defn->irqs;
    dev->ioops      = *ops;
    dev->flags      = SERIAL_AUTO_CR;

    regs->imsc = 0x50;

    return 0;
}
