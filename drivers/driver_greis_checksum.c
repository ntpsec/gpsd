/*
 * Checksum for the GNSS Receiver External Interface Specification (GREIS).
 *
 * This file is Copyright 2017 Virgin Orbit
 * This file is Copyright 2017 the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <limits.h>

#include "../include/driver_greis.h"

static inline unsigned char greis_rotate_left(unsigned char val)
{
    /* left circular rotation by two bits */
    return (val << 2) | (val >> (CHAR_BIT - 2));
}

unsigned char greis_checksum(const unsigned char *src, int count)
{
    unsigned char res = 0;
    while (count--)
        res = greis_rotate_left(res) ^ *src++;
    return greis_rotate_left(res);
}
// vim: set expandtab shiftwidth=4
