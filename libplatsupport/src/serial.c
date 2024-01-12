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
    for (int i = 0; i < count; i++) {
        /* TODO: Some UART drivers implement a busy waiting until there is space
         *       in the TX FIFO, others return an error in this case. We should
         *       have a mechanism to allow the user to specify the desired
         *       behavior, so we don't have to try to guess it. For a standard
         *       logging console, the behavior is likely to do a blocking wait,
         *       while a data UART may prefer nog to block and let the caller
         *       handle the waiting.
         */
        if (uart_putchar(d, data[i]) < 0) {
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
    for (int i = 0; i < count; i++) {
        int ret = uart_getchar(d);
        if (EOF == ret) {
            return i;
        }
        data[i] = ret;
    }
    return count;
}
