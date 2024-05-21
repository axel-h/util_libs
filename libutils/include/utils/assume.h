/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <utils/builtin.h>

/* This idiom is a way of passing extra information or hints to GCC. It is only
 * occasionally successful, so don't think of it as a silver optimisation
 * bullet.
 */
#define ASSUME(x) \
    do { \
        if (!(x)) { \
            __builtin_unreachable(); \
        } \
    } while (0)
