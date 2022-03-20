/*
 * Copyright 2022, Axel Heider <axelheider@gmx.de>
 * Copyright 2018, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <utils/attribute.h>

#define UD_INSTRUCTION_SIZE 4

static inline void ALWAYS_INLINE utils_undefined_instruction(void) {
    asm volatile (".word 0x00000000");
}
