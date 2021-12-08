/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <string.h>
#include <stdlib.h>
#include <platsupport/serial.h>
#include "../../chardev.h"

#define RHR         0x00
#define THR         0x00
#define IER         0x04
#define LSR         0x14
#define RHR_MASK    MASK(8)
#define IER_RHRIT   BIT(0)
#define LSR_TXFIFOE BIT(5)
#define LSR_RXFIFOE BIT(0)

#define REG_PTR(base, offset)   ( (volatile uint32_t *)( \
                                    (uintptr_t)(base) + (offset) ) )

/*
 *******************************************************************************
 * UART access primitives
 *******************************************************************************
 */

static int internal_uart_is_tx_fifo_empty(void *reg_base)
{
    return *REG_PTR(vaddr, LSR) & LSR_TXFIFOE;
}

static void internal_uart_tx_byte(void *reg_base, uint8_t c)
{
    *REG_PTR(vaddr, THR) = c;
}

static int internal_uart_is_rx_available(void *reg_base)
{
    return *REG_PTR(d->vaddr, LSR) & LSR_RXFIFOE;
}

static uint8_t internal_uart_rx_byte(void *reg_base)
{
    return (uint8_t)(REG_PTR(d->vaddr, RHR) & RHR_MASK);
}

static void internal_uart_busy_wait_tx_ready(void *reg_base)
{
    while (!internal_uart_is_tx_fifo_empty(reg_base)) {
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
    void *reg_base = dev->vaddr;

    if (!internal_uart_is_rx_available(reg_base)) {
        return -1;
    }

    return internal_uart_rx_byte(reg_base);
}

int uart_putchar(ps_chardevice_t *dev, int c)
{
    void *reg_base = dev->vaddr;

    /* Extract the byte to send, drop any flags. */
    uint8_t byte = (uint8_t)c;

    internal_uart_busy_wait_tx_ready(reg_base);

    if ((byte == '\n') && (d->flags & SERIAL_AUTO_CR)) {
        internal_uart_tx_byte(reg_base, '\r');
        internal_uart_busy_wait_tx_ready(reg_base);
    }

    internal_uart_tx_byte(reg_base, c);

    return byte;
}

static void
uart_handle_irq(ps_chardevice_t *dev UNUSED)
{
    /* nothing to do */
}

int uart_init(const struct dev_defn *defn,
              const ps_io_ops_t *ops,
              ps_chardevice_t *dev)
{
    memset(dev, 0, sizeof(*dev));

    void *vaddr = chardev_map(defn, ops);
    if (vaddr == NULL) {
        return -1;
    }

    /* Set up all the  device properties. */
    dev->id         = defn->id;
    dev->vaddr      = (void *)vaddr;
    dev->read       = &uart_read;
    dev->write      = &uart_write;
    dev->handle_irq = &uart_handle_irq;
    dev->irqs       = defn->irqs;
    dev->ioops      = *ops;
    dev->flags      = SERIAL_AUTO_CR;

    *REG_PTR(dev->vaddr, IER) = IER_RHRIT;

    return 0;
}
