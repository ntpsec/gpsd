/*
 * Copyright 2005 Alfredo Pironti
 *
 * This file is Copyright 2005 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <assert.h>                  // for assert()
#include <cstdlib>
#include "../include/libgpsmm.h"

struct gps_data_t* gpsmm::gps_inner_open(const char *host, const char *port)
{
    if (0 != (gps_open(host, port, gps_state()))) {
        to_user = NULL;
        return NULL;
    }
    // else, connection successfully opened
    to_user = new struct gps_data_t;

    // prevent CWE-690 warning: dereference of possibly-NULL pinter
    assert(NULL != to_user);
    return backup(); // we return the backup of our internal structure
}

struct gps_data_t* gpsmm::stream(int flags)
{
    if (NULL == to_user) {
        return NULL;
    }
    if (-1 == gps_stream(gps_state(),flags, NULL)) {
        return NULL;
    }
    // else
    return backup();
}

struct gps_data_t* gpsmm::send(const char *request)
{
    if (-1 == gps_send(gps_state(),request)) {
        return NULL;
    }
    // else
    return backup();
}

struct gps_data_t* gpsmm::read(void)
{
    if (0 >= gps_read(gps_state(), NULL, 0)) {
        // we return null if there was a read() error, or
        // if no data is ready in POLL_NOBLOCK (default) mode, or
        // if the connection is closed by gpsd
        return NULL;
    }
    // else
    return backup();
}

bool gpsmm::waiting(int timeout)
{
    return gps_waiting(gps_state(), timeout);
}

const char *gpsmm::data(void)
{
    return gps_data(gps_state());
}

// cppcheck-suppress unusedFunction
void gpsmm::clear_fix(void)
{
    gps_clear_fix(&(gps_state()->fix));
}

// cppcheck-suppress unusedFunction
void gpsmm::enable_debug(int level, FILE *fp)
{
    gps_enable_debug(level, fp);
}

// cppcheck-suppress unusedFunction
bool gpsmm::is_open(void)
{
        return to_user != NULL;
}

gpsmm::~gpsmm()
{
    if (NULL != to_user) {
        (void)gps_close(gps_state());
        delete to_user;
    }
}
// vim: set expandtab shiftwidth=4
