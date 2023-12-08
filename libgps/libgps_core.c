/* libgps_core.c -- client interface library for the gpsd daemon
 *
 * Core portion of client library.  Cals helpers to handle different eports.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>                    // open()
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/libgps.h"
#include "../include/gps_json.h"
#include "../include/strfuncs.h"

int libgps_debuglevel = 0;

static FILE *debugfp;

// control the level and destination of debug trace messages
void gps_enable_debug(int level, FILE * fp)
{
    libgps_debuglevel = level;
    debugfp = fp;
    json_enable_debug(level - DEBUG_JSON, fp);
}

// assemble command in printf(3) style
void libgps_trace(int errlevel, const char *fmt, ...)
{
    if (errlevel <= libgps_debuglevel) {
        char buf[BUFSIZ];
        va_list ap;

        (void)strlcpy(buf, "libgps: ", sizeof(buf));
        va_start(ap, fmt);
        str_vappendf(buf, sizeof(buf), fmt, ap);
        va_end(ap);

        (void)fputs(buf, debugfp);
    }
}

#if defined(SHM_EXPORT_ENABLE) || defined(SOCKET_EXPORT_ENABLE)
#define CONDITIONALLY_UNUSED UNUSED
#else
#define CONDITIONALLY_UNUSED UNUSED
#endif  // SOCKET_EXPORT_ENABLE

/* gps_open(host,) -- open a connection for reading from gpsd
 *
 * host can be:
 *     a host name or host ip - to conenct to host
 *        port is numeric or symbolic port to connect to
 *     GPSD_DBUS_EXPORT "DBUS export"     - to connect to local DBUS
 *     GPSD_FILE_LOCAL "local file"       - to read to local file
 *        port is file name to open
 *     GPSD_SHARED_MEMORY "shared memory" - to connect to local chared memory
 *
 * Return: 0 osnuccess
 *         less than zero on failure
 */
int gps_open(const char *host, const char *port,
             struct gps_data_t *gpsdata)
{
    int status = -100;

    if (!gpsdata) {
        return NL_NOHOST;
    }

    // save for later
    gpsdata->source.server = host;
    gpsdata->source.port = port;

    if (NULL != host &&
        0 == strcmp(host, GPSD_LOCAL_FILE)) {
        int fd;

        libgps_debug_trace((DEBUG_CALLS, "INFO: gps_open(FILE)\n"));
        if (NULL == port) {
            libgps_debug_trace((DEBUG_CALLS,
                               "ERROR: gps_open(FILE) missing port\n"));
            return FILE_FAIL;
        }
        fd = open(port, O_RDONLY);
        if (0 > fd) {
            libgps_debug_trace((DEBUG_CALLS, "ERROR: gps_open(%s) %d\n",
                                port,  errno));
            return FILE_FAIL;
        }
#ifdef USE_QT
        gpsdata->gps_fd = (void *)(intptr_t)fd;   // Huh?
#else
        gpsdata->gps_fd = fd;
#endif
        status = 0;
        // set up for line-buffered I/O over the daemon socket
        gpsdata->privdata =
            (struct privdata_t *)calloc(1, sizeof(struct privdata_t));
        if (NULL == gpsdata->privdata) {
            return -1;
        }
    }
#ifdef SHM_EXPORT_ENABLE
    else if (NULL != host &&
        0 == strcmp(host, GPSD_SHARED_MEMORY)) {
        status = gps_shm_open(gpsdata);
        if (0 != status) {
            if (-2 == status ) {
                return SHM_NOATTACH;
            }
            if (-3 == status ) {
                return SHM_CALLOC;
            }
            // -1, or other error
            return SHM_NOSHARED;
        }
    }
#define USES_HOST
#endif  // SHM_EXPORT_ENABLE

#ifdef DBUS_EXPORT_ENABLE
    else if (NULL != host &&
        0 == strcmp(host, GPSD_DBUS_EXPORT)) {
        status = gps_dbus_open(gpsdata);
        if (0 != status ) {
            return DBUS_FAILURE;
        }
    }
#define USES_HOST
#endif  // DBUS_EXPORT_ENABLE

#ifdef SOCKET_EXPORT_ENABLE
    else {
        // last shot, try host:port
        status = gps_sock_open(host, port, gpsdata);
    }
#define USES_HOST
#endif  // SOCKET_EXPORT_ENABLE

#ifndef USES_HOST
    (void)fprintf(stderr,
                  "No methods available for connecting to %s!\n",
                  host);
#endif  // USES_HOST
#undef USES_HOST

    gpsdata->set = 0;
    gpsdata->satellites_used = 0;
    gps_clear_att(&(gpsdata->attitude));
    gps_clear_dop(&(gpsdata->dop));
    gps_clear_fix(&(gpsdata->fix));
    gps_clear_log(&(gpsdata->log));

    return status;
}

// close a gpsd connection
int gps_close(struct gps_data_t *gpsdata CONDITIONALLY_UNUSED)
{
    int status = -1;

    libgps_debug_trace((DEBUG_CALLS, "gps_close()\n"));

#ifdef SHM_EXPORT_ENABLE
    if (BAD_SOCKET((intptr_t)(gpsdata->gps_fd))) {
        gps_shm_close(gpsdata);
        status = 0;
    }
#endif  // SHM_EXPORT_ENABLE

#ifdef SOCKET_EXPORT_ENABLE
    if (-1 == status ) {
        status = gps_sock_close(gpsdata);
    }
#endif  // SOCKET_EXPORT_ENABLE

    return status;
}

/* wait for and read data from the daemon or file
 *
 * parameters:
 *    gps_data_t *gpsdata   -- structure for GPS data
 *    char *message         -- NULL, or optional buffer for received JSON
 *    int message_len       -- zero, or sizeof(message)
 *
 * Return:
 *    -1 == error
 *    -2 == EOF
 *    zero OK
 */
int gps_read(struct gps_data_t *gpsdata, char *message, int message_len)
{
    int status = -1;

    libgps_debug_trace((DEBUG_CALLS, "gps_read() begins\n"));
    if ((NULL != message) &&
        (0 < message_len)) {
        // be sure message is zero length
        // we do not memset() as this is time critical input path
        *message = '\0';
    }
    if (NULL == PRIVATE(gpsdata)) {
        char err[] = "gps_read() NULL == privdata";
        libgps_debug_trace((DEBUG_CALLS, "%s\n", err));
        strlcpy(gpsdata->error, err, sizeof(gpsdata->error));
        gpsdata->set = ERROR_SET;
        return -1;
    }

    if (NULL != gpsdata->source.server &&
        0 == strcmp(gpsdata->source.server, GPSD_LOCAL_FILE)) {
        // local file read
        char *eol, *eptr;
        ssize_t read_ret, message_len;

        errno = 0;

        // scan to find end of message (\n), or end of buffer
        eol = PRIVATE(gpsdata)->buffer;
        eptr = eol + PRIVATE(gpsdata)->waiting;

        // fill the buffer
#ifdef USE_QT
        read_ret = -1;  // TBD
#else
        read_ret = read(gpsdata->gps_fd, eptr,
                        sizeof(PRIVATE(gpsdata)->buffer) -
                        PRIVATE(gpsdata)->waiting - 1);
#endif
        if (0 >= read_ret) {
            int ret;

            // EOL, or error
            if ( 0 == read_ret) {
                strlcpy(gpsdata->error, "EOF", sizeof(gpsdata->error));
                ret = -2;
            } else {
                strlcpy(gpsdata->error, "ERROR", sizeof(gpsdata->error));
                ret = -1;
            }
            gpsdata->set = ERROR_SET;
            libgps_debug_trace((DEBUG_CALLS, "%s\n", gpsdata->error));
            return ret;
        }
        gpsdata->set &= ~PACKET_SET;
        PRIVATE(gpsdata)->waiting += read_ret;
        eptr = eol + PRIVATE(gpsdata)->waiting;

        while ((eptr > eol) &&
               ('\n' != *eol)) {
            eol++;
        }

        if (eol >= eptr) {
            /* Buffer is full but still didn't get a message.
             * discard and try again. */
            libgps_debug_trace((DEBUG_CALLS,
                                "gps_read() buffer full, but no message\n"));
            PRIVATE(gpsdata)->buffer[0] = '\0';
            PRIVATE(gpsdata)->waiting = 0;
            return -1;
        }
        /* else  have a full message in buffer
         * eol now points to trailing \n in a full message */
        *eol = '\0';
        /* why the 1?  We want the NUL.
         *          |0|1|2|3|4|5| 6|7|
         *          |1|2|3|4|5|6|\n|X|
         *     buffer^         eol^
         *  buffer = 0
         *  eol = 6
         *  eol-buffer = 6-0 = 6, size of the line data is 7 bytes with \n
         *  eol-buffer+1 = 6-0+1 = 7
         */
        message_len = 1 + eol - PRIVATE(gpsdata)->buffer;

        if (NULL != message) {
            // user wants a copy, including NUL
            memcpy(message, PRIVATE(gpsdata)->buffer, message_len);
        }
        (void)clock_gettime(CLOCK_REALTIME, &gpsdata->online);
        // unpack the JSON message
        status = gps_unpack(PRIVATE(gpsdata)->buffer, gpsdata);

        // calculate length of good data still in buffer
        PRIVATE(gpsdata)->waiting -= message_len;

        if (0 >= PRIVATE(gpsdata)->waiting) {
            // no waiting data, or overflow, clear the buffer, just in case
            *PRIVATE(gpsdata)->buffer = '\0';
            PRIVATE(gpsdata)->waiting = 0;
        } else {
            // shift the remaing data to the fron.
            memmove(PRIVATE(gpsdata)->buffer,
                    PRIVATE(gpsdata)->buffer + message_len,
                    PRIVATE(gpsdata)->waiting);
        }
        gpsdata->set |= PACKET_SET;
    }
#ifdef SHM_EXPORT_ENABLE
    else if (BAD_SOCKET((intptr_t)(gpsdata->gps_fd))) {
        status = gps_shm_read(gpsdata);
    }
#endif  // SHM_EXPORT_ENABLE

#ifdef SOCKET_EXPORT_ENABLE
    else if (-1 == status &&
        !BAD_SOCKET((intptr_t)(gpsdata->gps_fd))) {
        status = gps_sock_read(gpsdata, message, message_len);
    }
#endif  // SOCKET_EXPORT_ENABLE

    libgps_debug_trace((DEBUG_CALLS, "gps_read() -> %d (%s)\n",
                        status, gps_maskdump(gpsdata->set)));

    return status;
}

/* send a command to the gpsd instance
 *
 * Return: 0 -- success
 * Return: negative -- fail
 */
int gps_send(struct gps_data_t *gpsdata CONDITIONALLY_UNUSED,
             const char *fmt CONDITIONALLY_UNUSED, ...)
{
    int status = -1;
    char buf[BUFSIZ];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    // codacy deos not like strlen()
    if ('\n' != buf[strnlen(buf, sizeof(buf)) - 1]) {
        (void)strlcat(buf, "\n", sizeof(buf));
    }

#ifdef SOCKET_EXPORT_ENABLE
    status = gps_sock_send(gpsdata, buf);
#endif  // SOCKET_EXPORT_ENABLE

    return status;
}

/* setup a stream
 *
 * FIXME: works on socket streams, but not on shared memory stream.
 *
 * Return: 0 -- success
 * Return: negative -- fail
 */
int gps_stream(struct gps_data_t *gpsdata, watch_t flags,
        const char *d CONDITIONALLY_UNUSED)
{
    int status = -1;

    if (NULL != gpsdata->source.server &&
        0 == strcmp(gpsdata->source.server, GPSD_LOCAL_FILE)) {
        // lodal cile, read-only
        flags |= WATCH_READONLY;
    }
    gpsdata->watch = flags;
    if (WATCH_READONLY & flags) {
        // read only
        return 0;
    }
#ifdef SOCKET_EXPORT_ENABLE
    status = gps_sock_stream(gpsdata, flags, d);
#endif  // SOCKET_EXPORT_ENABLE

    return status;
}

// return the contents of the client data buffer
const char *gps_data(const struct gps_data_t *gpsdata CONDITIONALLY_UNUSED)
{
    const char *bufp = NULL;

#ifdef SOCKET_EXPORT_ENABLE
    bufp = gps_sock_data(gpsdata);
#endif  // SOCKET_EXPORT_ENABLE

    return bufp;
}

/* is there input waiting from the GPS?
 * timeout is in uSec
 */
bool gps_waiting(const struct gps_data_t *gpsdata,
                 int timeout CONDITIONALLY_UNUSED)
{
    // this is bogus, but I can't think of a better solution yet
    bool waiting = true;

    if (NULL != gpsdata->source.server &&
        0 == strcmp(gpsdata->source.server, GPSD_LOCAL_FILE)) {
        // always ready, until EOF
        return true;
    }
#ifdef SHM_EXPORT_ENABLE
    if (SHM_PSEUDO_FD == (intptr_t)(gpsdata->gps_fd)) {
        waiting = gps_shm_waiting(gpsdata, timeout);
        return waiting;
    }
#endif  // SHM_EXPORT_ENABLE

#ifdef SOCKET_EXPORT_ENABLE
    if (0 <= (intptr_t)(gpsdata->gps_fd)) {
        waiting = gps_sock_waiting(gpsdata, timeout);
    }
#endif  // SOCKET_EXPORT_ENABLE

    return waiting;
}

/* run a main loop with a specified handler
 *
 * Returns: -1 on timeout or read error
 *          -2 read error
 * FIXME: read error should return different than timeout
 */
int gps_mainloop(struct gps_data_t *gpsdata CONDITIONALLY_UNUSED,
                 int timeout CONDITIONALLY_UNUSED,
                 void (*hook)(struct gps_data_t *gpsdata) CONDITIONALLY_UNUSED)
{
    int status = -1;

    libgps_debug_trace((DEBUG_CALLS, "gps_mainloop() begins\n"));

#ifdef SHM_EXPORT_ENABLE
    if (SHM_PSEUDO_FD == (intptr_t)(gpsdata->gps_fd)) {
        libgps_debug_trace((DEBUG_CALLS, "gps_shm_mainloop() begins\n"));
        status = gps_shm_mainloop(gpsdata, timeout, hook);
    }
#endif  // SHM_EXPORT_ENABLE
#ifdef DBUS_EXPORT_ENABLE
    if (DBUS_PSEUDO_FD == (intptr_t)(gpsdata->gps_fd)) {
        libgps_debug_trace((DEBUG_CALLS, "gps_dbus_mainloop() begins\n"));
        status = gps_dbus_mainloop(gpsdata, timeout, hook);
    }
#endif  // DBUS_EXPORT_ENABLE
#ifdef SOCKET_EXPORT_ENABLE
    if (0 <= (intptr_t)(gpsdata->gps_fd)) {
        libgps_debug_trace((DEBUG_CALLS, "gps_sock_mainloop() begins\n"));
        status = gps_sock_mainloop(gpsdata, timeout, hook);
    }
#endif  // SOCKET_EXPORT_ENABLE

    libgps_debug_trace((DEBUG_CALLS, "gps_mainloop() -> %d (%s)\n",
                        status, gps_maskdump(gpsdata->set)));

    return status;
}

extern const char *gps_errstr(const int err)
{
    /*
     * We might add our own error codes in the future, e.g for
     * protocol compatibility checks
     */
#ifndef USE_QT
    static char buf[32];

    (void)snprintf(buf, sizeof(buf), "Qt error %d", err);
    return buf;
#else // USE_QT
#ifdef SHM_EXPORT_ENABLE
    if (SHM_NOSHARED == err) {
        return "no shared-memory segment or daemon not running";
    }
    if (SHM_NOATTACH == err) {
        return "attach failed for unknown reason";
    }
#endif  // SHM_EXPORT_ENABLE
#ifdef DBUS_EXPORT_ENABLE
    if (DBUS_FAILURE == err) {
        return "DBUS initialization failure";
    }
#endif  // DBUS_EXPORT_ENABLE
    return netlib_errstr(err);
#endif  // USE_QT
}

void libgps_dump_state(struct gps_data_t *collect)
{
    char ts_buf[TIMESPEC_LEN];

    // no need to dump the entire state, this is a sanity check
#ifndef USE_QT
    (void)fprintf(debugfp, "flags: (0x%04x) %s\n",
                  (unsigned int)collect->set, gps_maskdump(collect->set));
#endif
    if (ONLINE_SET & collect->set) {
        (void)fprintf(debugfp, "ONLINE: %s\n",
                      timespec_str(&collect->online, ts_buf, sizeof(ts_buf)));
    }
    if (TIME_SET & collect->set) {
        (void)fprintf(debugfp, "TIME: %s\n",
                     timespec_str(&collect->fix.time, ts_buf, sizeof(ts_buf)));
    }
    // NOTE: %.7f needed for cm level accurate GPS
    if (LATLON_SET & collect->set) {
        (void)fprintf(debugfp, "LATLON: lat/lon: %.7lf %.7lf\n",
                      collect->fix.latitude, collect->fix.longitude);
    }
    if (ALTITUDE_SET & collect->set) {
        (void)fprintf(debugfp, "ALTITUDE: altHAE: %lf  U: climb: %lf\n",
                      collect->fix.altHAE, collect->fix.climb);
    }
    if (SPEED_SET & collect->set) {
        (void)fprintf(debugfp, "SPEED: %lf\n", collect->fix.speed);
    }
    if (TRACK_SET & collect->set) {
        (void)fprintf(debugfp, "TRACK: track: %lf\n", collect->fix.track);
    }
    if (MAGNETIC_TRACK_SET & collect->set) {
        (void)fprintf(debugfp, "MAGNETIC_TRACK: magtrack: %lf\n",
                      collect->fix.magnetic_track);
    }
    if (CLIMB_SET & collect->set) {
        (void)fprintf(debugfp, "CLIMB: climb: %lf\n", collect->fix.climb);
    }
    if (STATUS_SET & collect->set) {
        // FIXME! add missing status values.  range check status!
        const char *status_values[] = { "NO_FIX", "FIX", "DGPS_FIX" };
        (void)fprintf(debugfp, "STATUS: status: %d (%s)\n",
                      collect->fix.status, status_values[collect->fix.status]);
    }
    if (MODE_SET & collect->set) {
        const char *mode_values[] = { "", "NO_FIX", "MODE_2D", "MODE_3D" };
        (void)fprintf(debugfp, "MODE: mode: %d (%s)\n",
                      collect->fix.mode, mode_values[collect->fix.mode]);
    }
    if (SATELLITE_SET & collect->set) {
        (void)fprintf(debugfp,
                      "DOP: satellites %d, pdop=%lf, hdop=%lf, vdop=%lf\n",
                      collect->satellites_used, collect->dop.pdop,
                      collect->dop.hdop, collect->dop.vdop);
    }
    if (VERSION_SET & collect->set) {
        (void)fprintf(debugfp, "VERSION: release=%s rev=%s proto=%d.%d\n",
                      collect->version.release,
                      collect->version.rev,
                      collect->version.proto_major,
                      collect->version.proto_minor);
    }
    if (POLICY_SET & collect->set) {
        (void)fprintf(debugfp,
                      "POLICY: watcher=%s nmea=%s raw=%d scaled=%s timing=%s, "
                      "split24=%s pps=%s, devpath=%s\n",
                      collect->policy.watcher ? "true" : "false",
                      collect->policy.nmea ? "true" : "false",
                      collect->policy.raw,
                      collect->policy.scaled ? "true" : "false",
                      collect->policy.timing ? "true" : "false",
                      collect->policy.split24 ? "true" : "false",
                      collect->policy.pps ? "true" : "false",
                      collect->policy.devpath);
    }
    if (SATELLITE_SET & collect->set) {
        struct satellite_t *sp;

        (void)fprintf(debugfp, "SKY: satellites in view: %d\n",
                      collect->satellites_visible);
        for (sp = collect->skyview;
             sp < collect->skyview + collect->satellites_visible;
             sp++) {
            (void)fprintf(debugfp, "  %2.2d: %4.1f %5.1f %3.0f %c\n",
                          sp->PRN, sp->elevation,
                          sp->azimuth, sp->ss,
                          sp->used ? 'Y' : 'N');
        }
    }
    if (RAW_SET & collect->set) {
        (void)fprintf(debugfp, "RAW: got raw data\n");
    }
    if (DEVICE_SET & collect->set) {
        (void)fprintf(debugfp, "DEVICE: Device is '%s', driver is '%s'\n",
                      collect->dev.path, collect->dev.driver);
    }
    if (DEVICELIST_SET & collect->set) {
        int i;
        (void)fprintf(debugfp, "DEVICELIST:%d devices:\n",
                      collect->devices.ndevices);
        for (i = 0; i < collect->devices.ndevices; i++) {
            (void)fprintf(debugfp, "%d: path='%s' driver='%s'\n",
                          collect->devices.ndevices,
                          collect->devices.list[i].path,
                          collect->devices.list[i].driver);
        }
    }
}

// vim: set expandtab shiftwidth=4
