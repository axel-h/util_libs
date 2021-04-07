/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <string.h>
#include <stdlib.h>
#include <platsupport/serial.h>
#include "chardev.h"

ssize_t uart_write(
    ps_chardevice_t *d,
    const void *vdata,
    size_t count,
    chardev_callback_t rcb UNUSED,
    void *token UNUSED)
{
    const unsigned char *data = (const unsigned char *)vdata;
    for (unsigned int i = 0; i < count; i++) {
        /* Call the UART driver, it is supposed to implement the handling for
         * the flags SERIAL_TX_NONBLOCKING and SERIAL_AUTO_CR properly.
         */
        int ret = uart_putchar(d, data[i]);
        if (ret < 0) {
            /* There is nothing we can do, so abort and return how much data we
             * could send. Unfortunately, we can return the actual error code,
             * so the caller wont know what exactly failed. However, when
             * SERIAL_TX_NONBLOCKING is enabled, it's likely that the TX FIFO is
             * full so the caller should wait and send the remaining data.
             */
            return i;
        }
    }
    return count;
}

ssize_t uart_read(
    ps_chardevice_t *d,
    void *vdata,
    size_t count,
    chardev_callback_t rcb UNUSED,
    void *token UNUSED)
{
    char *data = (char *)vdata;
    for (unsigned int i = 0; i < count; i++) {
        int ret = uart_getchar(d);
        if (EOF == ret) {
            return i;
        }
        data[i] = ret;
    }
    return count;
}
