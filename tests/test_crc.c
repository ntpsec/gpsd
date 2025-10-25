/*
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#include "../include/gpsd_config.h"  // must be before all includes

#include <ctype.h>
#include <errno.h>          // for errno
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>       // for open()
#include <sys/types.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/crc24q.h"

static int verbose = 0;

#define NUM_TESTS 2
static unsigned  char crc_good[NUM_TESTS][10] = {
    {0},
    {33, 33, 34, 0, 0, 0, 0, 0x46, 0x56, 0x4f},
};

static unsigned  char crc_bad[NUM_TESTS][10] = {
    {1},
    {3, 33, 34, 0, 0, 0, 0, 0x46, 0x56, 0x4f},
};

int main(int argc, char *argv[])
{
    int failcount = 0;
    int option;
    unsigned loop;

    verbose = 0;
    while ((option = getopt(argc, argv, "v:")) != -1) {
        switch (option) {
        case 'v':
            verbose = atoi(optarg);
            break;
        default:
            break;
        }
    }

    // thses should pass
    for (loop = 0; NUM_TESTS > loop; loop++) {
        if (!crc24q_check(crc_good[loop], 10)) {
            printf("FAILED data crc failure, %0x against %02x %02x %02x\n",
                     crc24q_hash(crc_good[loop], 7),
                     crc_good[loop][7], crc_good[loop][8], crc_good[loop][9]);
            failcount++;
        }
    }
    // thses should fail
    for (loop = 0; NUM_TESTS > loop; loop++) {
        if (crc24q_check(crc_bad[loop], 10)) {
            printf("FAILED data crc failure, %0x against %02x %02x %02x\n",
                     crc24q_hash(crc_bad[loop], 7),
                     crc_bad[loop][7], crc_bad[loop][8], crc_bad[loop][9]);
            failcount++;
        }
    }
    exit(failcount > 0 ? EXIT_FAILURE : EXIT_SUCCESS);
}
// vim: set expandtab shiftwidth=4
