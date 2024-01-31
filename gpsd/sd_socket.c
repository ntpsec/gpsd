/*
 * This file is Copyright 2011 by Eckhart Wörner
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <errno.h>                   // for errno
#include <limits.h>
#include <stdlib.h>                  // for strtoul()
#include <unistd.h>

#include "../include/sd_socket.h"

int sd_get_socket_count(void) {
    unsigned long n;
    const char* env;

    env = getenv("LISTEN_PID");
    if (!env) {
        return 0;
    }

    errno = 0;
    n = strtoul(env, NULL, 10);
    if (0 != errno ||
        getpid() != (pid_t)n) {
        return 0;
    }

    env = getenv("LISTEN_FDS");
    if (!env) {
        return 0;
    }

    errno = 0;
    n = strtoul(env, NULL, 10);
    if (0 != errno) {
        return 0;
    }

    return (int)n;
}

// vim: set expandtab shiftwidth=4
