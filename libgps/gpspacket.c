/*
 * Foreign function interface binding for the packet module.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include "../include/gps.h"           // for gps_visibilize()
#include "../include/gpsd.h"
#include "../include/strfuncs.h"

#define LOG_SHOUT 0

// Add prototypes for FFI interfaces (not just Python).
struct gps_device_t *ffi_Device_init(int);
struct gps_lexer_t *ffi_Device_Lexer(struct gps_device_t *);
void ffi_Device_fini(struct gps_device_t *);
struct gps_lexer_t *ffi_Lexer_init(void);
void ffi_Lexer_fini(struct gps_lexer_t *);

// Add non FFI prototypes
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

/* Export structure sizes for FFI. Two of these are unused now.
 * Pythons gps.packet.lexer_t storage class needs fvi_size_buffer */
const size_t fvi_size_device = sizeof(struct gps_device_t);
const size_t fvi_size_lexer = sizeof(struct gps_lexer_t);
const size_t fvi_size_buffer = (MAX_PACKET_LENGTH * 2) + 1;

/* Attempt to allocate and prepare a bare lexer instance for FFI.
 * Returns NULL on failure. */
struct gps_lexer_t *ffi_Lexer_init(void) {
    struct gps_lexer_t *result;

    result = calloc(1, fvi_size_lexer);
    if (NULL == result) {
        return NULL;
    }
    packet_reset(result);
    return result;
}

// Free allocated memory for lexer struct.
void ffi_Lexer_fini(struct gps_lexer_t *lexer) {
    free(lexer);
}

/* Attempt to allocate and prepare a wrapped lexer instance for FFI.
 * Returns NULL on failure */
struct gps_device_t *ffi_Device_init(int fd) {
    struct gps_device_t *result;

    result = calloc(1, fvi_size_device);
    if (NULL == result) {
        return NULL;
    }
    result->gpsdata.gps_fd = fd;
    packet_reset(&result->lexer);
    return result;
}

// Free allocated memory for device struct.
void ffi_Device_fini(struct gps_device_t *device) {
    free(device);
}

// Get the lexer member of a gps_device_t for FFI.
struct gps_lexer_t *ffi_Device_Lexer(struct gps_device_t *dev) {
    return &dev->lexer;
}
// vim: set expandtab shiftwidth=4
