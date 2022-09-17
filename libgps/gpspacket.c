/*
 * Foreign function interface binding for the packet module.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include "../include/gps.h"           // for gps_visibilize()
#include "../include/gpsd.h"
#include "../include/strfuncs.h"

#define LOG_SHOUT 0

struct gps_lexer_t *ffi_Lexer_init(void);   // For FFI Python interface.
void gpsd_vlog(const struct gpsd_errout_t*, const int, char*,
               size_t, const char*, va_list);

// assemble msg in vprintf(3) style, use errout hook or syslog for delivery
// FIXME: duplicated in gpsd/libgpsd_core.c
void gpsd_vlog(const struct gpsd_errout_t *errout, const int errlevel,
               char *outbuf, size_t outlen, const char *fmt, va_list ap)
{
    if (errout->debug >= errlevel) {
        char buf[BUFSIZ];
        char *err_str;
        const char *label;

        switch (errlevel) {
        case LOG_ERROR:
            err_str = "ERROR: ";
            break;
        case LOG_SHOUT:
            err_str = "SHOUT: ";
            break;
        case LOG_WARN:
            err_str = "WARN: ";
            break;
        case LOG_CLIENT:
            err_str = "CLIENT: ";
            break;
        case LOG_INF:
            err_str = "INFO: ";
            break;
        case LOG_DATA:
            err_str = "DATA: ";
            break;
        case LOG_PROG:
            err_str = "PROG: ";
            break;
        case LOG_IO:
            err_str = "IO: ";
            break;
        case LOG_SPIN:
            err_str = "SPIN: ";
            break;
        case LOG_RAW:
            err_str = "RAW: ";
            break;
        case LOG_RAW1:       // 9, rediculous
            err_str = "RAW1";
            break;
        case LOG_RAW2:       // 10, insane
            err_str = "RAW2";
            break;
        default:
            err_str = "UNK: ";
            break;
        }

        if (NULL == errout->label) {
            label = "MISSING";
        } else {
            label = errout->label;
        }

        snprintf(buf, sizeof(buf), "%s:%s", label, err_str);
        str_vappendf(buf, sizeof(buf), fmt, ap);

        gps_visibilize(outbuf, outlen, buf, strlen(buf));

        if (getpid() == getsid(getpid())) {
            syslog((errlevel <= LOG_SHOUT) ? LOG_ERR : LOG_NOTICE,
                   "%s", outbuf);
        } else if (NULL != errout->report) {
            errout->report(outbuf);
        } else {
            (void)fputs(outbuf, stderr);
        }
    }
}

// assemble msg in printf(3) style, use errout hook or syslog for delivery
void gpsd_log(const int errlevel, const struct gpsd_errout_t *errout,
              const char *fmt, ...)
{
    char buf[BUFSIZ];
    va_list ap;

    // cppcheck wants is to check for NULL == errout.  Should never happen?
    if (NULL == errout ||
        errout->debug < errlevel) {
        // nothing to do, get out
        return;
    }

    buf[0] = '\0';
    va_start(ap, fmt);
    gpsd_vlog(errout, errlevel, buf, sizeof(buf), fmt, ap);
    va_end(ap);
}

static void basic_report(const char *buf) {
    (void)fputs(buf, stderr);
}

void errout_reset(struct gpsd_errout_t *errout) {
    errout->debug = LOG_SHOUT;
    errout->report = basic_report;
}

size_t fvi_size_lexer = sizeof(struct gps_lexer_t);
size_t fvi_size_buffer = (MAX_PACKET_LENGTH * 2) + 1;

struct gps_lexer_t *ffi_Lexer_init() {
    struct gps_lexer_t *result;

    result = calloc(1, fvi_size_lexer);
    if (NULL == result) {
        return NULL;
    }
    packet_reset(result);
    return result;
}
// vim: set expandtab shiftwidth=4
