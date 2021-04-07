/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <autoconf.h>

#include <stdlib.h>
#include <platsupport/serial.h>
#include <platsupport/plat/serial.h>
#include <string.h>

#include "../../chardev.h"

#define UART_TX_DATA_MASK  0xFF
#define UART_TX_DATA_FULL  BIT(31)

#define UART_RX_DATA_MASK   0xFF
#define UART_RX_DATA_EMPTY  BIT(31)

#define UART_TX_INT_EN     BIT(0)
#define UART_RX_INT_EN     BIT(1)

#define UART_TX_INT_PEND     BIT(0)
#define UART_RX_INT_PEND     BIT(1)
#define UART_BAUD_DIVISOR 4340
struct uart {
    uint32_t txdata;
    uint32_t rxdata;
    uint32_t txctrl;
    uint32_t rxctrl;
    uint32_t ie;
    uint32_t ip;
    uint32_t div;
};
typedef volatile struct uart uart_regs_t;

/*
 *******************************************************************************
 * UART access primitives
 *******************************************************************************
 */

static uart_regs_t* uart_get_regs(ps_chardevice_t *d)
{
    return (uart_regs_t*)d->vaddr;
}

static int internal_uart_is_tx_fifo_full(uart_regs_t* regs)
{
    return regs->txdata & UART_TX_DATA_FULL;
}

static int internal_uart_tx_byte(uart_regs_t* regs, uint8_t c)
{
    regs->txdata = c & UART_TX_DATA_MASK;
}

static void internal_uart_busy_wait_tx_ready(uart_regs_t* regs)
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

int uart_getchar(ps_chardevice_t *d)
{
    uart_regs_t* regs = uart_get_regs(d);

    uint32_t rxdata = regs->rxdata;
    if (rxdata & UART_RX_DATA_EMPTY)
    {
        return -1;
    }

    /* return only the lowest 8 bits */
    return (uint8_t)(rxdata & UART_RX_DATA_MASK);
}

int uart_putchar(ps_chardevice_t* d, int c)
{
    uart_regs_t* regs = uart_get_regs(d);

    /* Check if the TX FIFO has space. If not and SERIAL_TX_NONBLOCKING is set,
     * then fail the call, otherwise do busy waiting.
     */
    if (internal_uart_is_tx_fifo_full(regs)) {
        if (d->flags & SERIAL_TX_NONBLOCKING) {
            return -1;
        }
        internal_uart_busy_wait_tx_ready(regs);
    }

    /* Extract the byte to send, drop any flags. */
    uint8_t byte = (uint8_t)c;

    /* SERIAL_AUTO_CR enables sending a CR before any LF, which is the common
     * thing to do for a serial terminal. CR/LR are considered an atom, thus a
     * blocking wait will be used even if SERIAL_TX_NONBLOCKING is set to ensure
     * LF is sent.
     * TODO: Check in advance if the TX FIFO has space for two chars if
     *       SERIAL_TX_NONBLOCKING is set.
     */
    if ((byte == '\n') && (d->flags & SERIAL_AUTO_CR)) {
        internal_uart_tx_byte(regs, '\r');
        internal_uart_busy_wait_tx_ready(regs);
    }

    internal_uart_tx_byte(regs, byte);

    return byte;
}

static void
uart_handle_irq(ps_chardevice_t* d UNUSED)
{
    // IRQs are cleared when the TX/RX watermark conditions are no longer met
    // so there is nothing to do here.
}

int uart_init(const struct dev_defn* defn,
              const ps_io_ops_t* ops,
              ps_chardevice_t* dev)
{
    /* Attempt to map the virtual address, assure this works */
    void* vaddr = chardev_map(defn, ops);
    if (vaddr == NULL) {
        return -1;
    }

    memset(dev, 0, sizeof(*dev));

    /* Set up all the  device properties. */
    dev->id         = defn->id;
    dev->vaddr      = (void*)vaddr;
    dev->read       = &uart_read;
    dev->write      = &uart_write;
    dev->handle_irq = &uart_handle_irq;
    dev->irqs       = defn->irqs;
    dev->ioops      = *ops;
    dev->flags      = SERIAL_AUTO_CR;

    uart_regs_t* regs = uart_get_regs(dev);

    /*
     * Enable TX and RX and don't set any watermark levels.
     * 0 watermark on RX indicates an IRQ when more than 0 chars in RX buffer
     * O watermark on TX indicates no IRQ (less than 0 chars in TX buffer)
     */
    regs->txctrl = 0x00001;
    regs->rxctrl = 0x00001;
    /* Enable RX IRQs.  We don't enable TX IRQs as we don't expect any. */
    regs->ie = 0x2;

    if (regs->div != UART_BAUD_DIVISOR) {
        ZF_LOGW("Warning: We require a target baud of 115200 and assume an input clk freq 500MHz.");
        ZF_LOGW("Warning: However an incorrect divisor is set: %d, expected %d", regs->div, UART_BAUD_DIVISOR);
    }

    return 0;
}
