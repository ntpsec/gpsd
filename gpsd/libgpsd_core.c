/* libgpsd_core.c -- manage access to sensors
 *
 * Access to the driver layer goes through the entry points in this file.
 * The idea is to present a session as an abstraction from which you get
 * fixes (and possibly other data updates) by calling gpsd_multipoll(). The
 * rest is setup and teardown. (For backward compatibility the older gpsd_poll()
 * entry point has been retained.)
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/select.h>    // for pselect() per POSIX
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/matrix.h"
#include "../include/strfuncs.h"
#include "../include/timespec.h"
#if defined(NMEA2000_ENABLE)
    #include "../include/driver_nmea2000.h"
#endif  // defined(NMEA2000_ENABLE)

// pass low-level data to devices straight through
ssize_t gpsd_write(struct gps_device_t *session,
                   const char *buf,
                   const size_t len)
{
    return session->context->serial_write(session, buf, len);
}

static void basic_report(const char *buf)
{
    (void)fputs(buf, stderr);
}

void errout_reset(struct gpsd_errout_t *errout)
{
    errout->debug = LOG_SHOUT;
    errout->report = basic_report;
}

static pthread_mutex_t report_mutex;

void gpsd_acquire_reporting_lock(void)
{
    int err;

    // pthread_mutex_lock() returns zero, or an error code.
    err = pthread_mutex_lock(&report_mutex);
    if (0 != err) {
        /* POSIX says pthread_mutex_lock() should only fail if the
        thread holding the lock has died.  Best for gpsd to just die
        because things are FUBAR. */

        (void)fprintf(stderr,"pthread_mutex_lock() failed: %s(%d)\n",
                      strerror(err), err);
        exit(EXIT_FAILURE);
    }
}

void gpsd_release_reporting_lock(void)
{
    int err;

    // pthread_mutex_lock() returns zero, or an error code.
    err = pthread_mutex_unlock(&report_mutex);
    if (0 != err) {
        /* POSIX says pthread_mutex_unlock() should only fail when
        trying to unlock a lock that does not exist, or is not owned by
        this thread.  This should never happen, so best for gpsd to die
        because things are FUBAR. */

        (void)fprintf(stderr,"pthread_mutex_unlock() failed: %s(%d)\n",
                      strerror(err), err);
        exit(EXIT_FAILURE);
    }
}

// assemble msg in vprintf(3) style, use errout hook or syslog for delivery
// FIXME: duplicated in gpsd/libgpsd_core.c
static void gpsd_vlog(const int errlevel,
                      const struct gpsd_errout_t *errout,
                      char *outbuf, size_t outlen,
                      const char *fmt, va_list ap)
{
#ifdef SQUELCH_ENABLE
    (void)errout;
    (void)errlevel;
    (void)fmt;
#else
    char buf[BUFSIZ];
    const char *err_str;
    const char *label;
    int level = LOG_ERR;

    gpsd_acquire_reporting_lock();
    switch (errlevel) {
    case LOG_ERROR:      // -1, cannot turn off
            err_str = "ERROR";
            level = LOG_CRIT;
            break;
    case LOG_SHOUT:      // 0, cannot turn off
            err_str = "SHOUT";
            level = LOG_ERR;
            break;
    case LOG_WARN:       // 1
            err_str = "WARN";
            level = LOG_WARNING;
            break;
    case LOG_CLIENT:     // 2, log JSON to clients
            err_str = "CLIENT";
            level = LOG_NOTICE;
            break;
    case LOG_INF:        // 3, informative info
            err_str = "INFO";
            level = LOG_INFO;
            break;
    case LOG_PROG:       // 4, program progress messages
            err_str = "PROG";
            level = LOG_DEBUG;
            break;
    case LOG_IO:         // 5, device IO
            err_str = "IO";
            level = LOG_DEBUG;
            break;
    case LOG_DATA:       // 6, decoded data
            err_str = "DATA";
            level = LOG_DEBUG;
            break;
    case LOG_SPIN:       // 7, spin logging
            err_str = "SPIN";
            level = LOG_DEBUG;
            break;
    case LOG_RAW:        // 8, low level IO
            err_str = "RAW";
            level = LOG_DEBUG;
            break;
    case LOG_RAW1:       // 9, ridiculous
            err_str = "RAW1";
            level = LOG_DEBUG;
            break;
    case LOG_RAW2:       // 10, insane
            err_str = "RAW2";
            level = LOG_DEBUG;
            break;
    default:             // WTF?
            err_str = "UNK";
            level = LOG_CRIT;
            break;
    }

    if (NULL == errout->label) {
        label = "MISSING";
    } else {
        label = errout->label;
    }
    snprintf(buf, sizeof(buf), "%s:%s: %s", label, err_str, fmt);
    vsnprintf(outbuf, outlen, buf, ap);

    // this was crazy expensive, just fix the bad log calls
    // gps_visibilize(outbuf, outlen, buf, strlen(buf));

    if (getpid() == getsid(getpid())) {
        // I think this calls syslog() only when daemonized
        syslog(level, "%s",  outbuf);
    } else if (NULL != errout->report) {
        // we are a thread, use report()?
        // FIXME: is POSIX syslog() thread safe?
        errout->report(outbuf);
    } else {
        // foreground, use stderr?
        (void)fputs(outbuf, stderr);
    }
    gpsd_release_reporting_lock();
#endif  // !SQUELCH_ENABLE
}

// assemble msg in printf(3) style, use errout hook or syslog for delivery
void gpsd_log(const int errlevel, const struct gpsd_errout_t *errout,
              const char *fmt, ...)
{
    char buf[BUFSIZ];
    va_list ap;

    // errout should never be NULL, but some code analyzers complain anyway
    if (NULL == errout ||
        errout->debug < errlevel) {
        return;
    }

    buf[0] = '\0';
    va_start(ap, fmt);
    gpsd_vlog(errlevel, errout, buf, sizeof(buf), fmt, ap);
    va_end(ap);
}

// dump the current packet in a form optimised for eyeballs
const char *gpsd_prettydump(struct gps_device_t *session)
{
    return gpsd_packetdump(session->msgbuf, sizeof(session->msgbuf),
                           session->lexer.outbuffer,
                           session->lexer.outbuflen);
}

// Define the possible hook strings here so we can get the length
#define HOOK_ACTIVATE "ACTIVATE"
#define HOOK_DEACTIVATE "DEACTIVATE"

#define HOOK_CMD_MAX (sizeof(DEVICEHOOKPATH) + GPS_PATH_MAX \
                      + sizeof(HOOK_DEACTIVATE))

static void gpsd_run_device_hook(struct gpsd_errout_t *errout,
                                 char *device_name, char *hook)
{
    struct stat statbuf;
    int status;
    char buf[HOOK_CMD_MAX];

    if (-1 == stat(DEVICEHOOKPATH, &statbuf)) {
        GPSD_LOG(LOG_PROG, errout,
                 "CORE: no %s present, skipped running %s hook. %s(%d)\n",
                 DEVICEHOOKPATH, hook, strerror(errno), errno);
        return;
    }

    (void)snprintf(buf, sizeof(buf), "%s %s %s",
                   DEVICEHOOKPATH, device_name, hook);
    GPSD_LOG(LOG_INF, errout, "CORE: running %s\n", buf);
    status = system(buf);
    if (-1 == status) {
        GPSD_LOG(LOG_ERROR, errout, "CORE: error %s(%d) running %s\n",
                 strerror(errno), errno, buf);
    } else {
        GPSD_LOG(LOG_INF, errout,
                 "CORE: %s returned %d\n", DEVICEHOOKPATH,
                 WEXITSTATUS(status));
    }
}

int gpsd_switch_driver(struct gps_device_t *session, char *type_name)
{
    const struct gps_type_t **dp;
    bool first_sync = (NULL != session->device_type);
    unsigned int i;

    if (first_sync &&
        0 == strcmp(session->device_type->type_name, type_name)) {
        // no need to switch driver
        return 0;
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CORE: switch_driver(%s) called...\n", type_name);
    for (dp = gpsd_drivers, i = 0; *dp; dp++, i++)
        if (0 == strcmp((*dp)->type_name, type_name)) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "CORE: selecting %s driver...\n",
                     (*dp)->type_name);
            gpsd_assert_sync(session);
            session->device_type = *dp;
            session->driver_index = i;
            session->gpsdata.dev.mincycle = session->device_type->min_cycle;
            // reconfiguration might be required
            if (first_sync &&
                NULL != session->device_type->event_hook) {
                session->device_type->event_hook(session,
                                                 EVENT_DRIVER_SWITCH);
            }
            if (STICKY(*dp)) {
                session->last_controller = *dp;
            }
            return 1;
        }
    GPSD_LOG(LOG_ERROR, &session->context->errout,
             "CORE: invalid GPS type \"%s\".\n", type_name);
    return 0;
}

void gps_context_init(struct gps_context_t *context,
                      const char *label)
{
    (void)memset(context, '\0', sizeof(struct gps_context_t));
    //context.readonly = false;
    context->leap_notify    = LEAP_NOWARNING;
    context->serial_write = gpsd_serial_write;

    errout_reset(&context->errout);
    context->errout.label = label;

    (void)pthread_mutex_init(&report_mutex, NULL);
}

// initialize GPS polling
void gpsd_init(struct gps_device_t *session, struct gps_context_t *context,
               const char *device)
{
    (void)memset(session, 0, sizeof(struct gps_device_t));

    if (device != NULL) {
        (void)strlcpy(session->gpsdata.dev.path, device,
                      sizeof(session->gpsdata.dev.path));
    }

    /* with memset(), no need to set NULLs, or zeros
     *
     * session->device_type = NULL;        // start by hunting packets
     * session->last_controller = NULL;
     * session->observed = 0;
     * memset(session->subtype, 0, sizeof(session->subtype));
     * memset(session->subtype1, 0, sizeof(session->subtype1));
     * memset(&(session->nmea), 0, sizeof(session->nmea));
     * session->gpsdata.set = 0;
     * session->sor = (timespec_t){0, 0};
     * session->ts_startCurrentBaud = (timespec_t){0, 0};
     * session->chars = 0;
     *
     */
    session->context = context;
    session->gpsdata.dev.cycle =(timespec_t){1, 0};
    session->gpsdata.dev.mincycle = (timespec_t){1, 0};
    session->gpsdata.dev.parity = ' ';          // will be E, N, or O
    session->servicetype = SERVICE_UNKNOWN;     // gpsd_open() sets this
    session->shm_clock_unit = -1;
    session->shm_pps_unit = -1;
    session->sourcetype = SOURCE_UNKNOWN;       // gpsd_open() sets this
    gps_clear_att(&session->gpsdata.attitude);
    gps_clear_dop(&session->gpsdata.dop);
    gps_clear_fix(&session->gpsdata.fix);
    gps_clear_fix(&session->lastfix);
    gps_clear_fix(&session->newdata);
    gps_clear_fix(&session->oldfix);
    gps_clear_gst(&session->gpsdata.gst);
    gps_clear_log(&session->gpsdata.log);
    // tty-level initialization
    gpsd_tty_init(session);
    // necessary in case we start reading in the middle of a GPGSV sequence
    gpsd_zero_satellites(&session->gpsdata);

    // initialize things for the packet parser
    packet_reset(&session->lexer);
}

// temporarily release the GPS device
void gpsd_deactivate(struct gps_device_t *session)
{
    if (!session->context->readonly &&
        NULL != session->device_type  &&
        NULL != session->device_type->event_hook) {
        session->device_type->event_hook(session, EVENT_DEACTIVATE);
    }
    // cast for 32-bit ints.
    GPSD_LOG(LOG_INF, &session->context->errout,
             "CORE: closing %s, fd %ld\n",
             session->gpsdata.dev.path, (long)session->gpsdata.gps_fd);
    if (SERVICE_NTRIP == session->servicetype) {
        ntrip_close(session);
    } else
#if defined(NMEA2000_ENABLE)
    if (SOURCE_CAN == session->sourcetype) {
        (void)nmea2000_close(session);
    } else
#endif  // NMEA2000_ENABLE
    {
        // could be serial, udp://, tcp://, etc.
        gpsd_close(session);
    }
    if (O_OPTIMIZE == session->mode) {
        gpsd_run_device_hook(&session->context->errout,
                             session->gpsdata.dev.path,
                             HOOK_DEACTIVATE);
    }
    // tell any PPS-watcher thread to die
    session->pps_thread.report_hook = NULL;
    // mark it inactivated
    session->gpsdata.online.tv_sec = 0;
    session->gpsdata.online.tv_nsec = 0;
}

// shim function to decouple PPS monitor code from the session structure
static void ppsthread_log(volatile struct pps_thread_t *pps_thread,
                          int loglevel, const char *fmt, ...)
{
    struct gps_device_t *device = (struct gps_device_t *)pps_thread->context;
    char buf[BUFSIZ];
    va_list ap;

    switch (loglevel) {
    case THREAD_ERROR:
        loglevel = LOG_ERROR;
        break;
    case THREAD_WARN:
        loglevel = LOG_WARN;
        break;
    case THREAD_INF:
        loglevel = LOG_INF;
        break;
    case THREAD_PROG:
        loglevel = LOG_PROG;
        break;
    case THREAD_RAW:
        loglevel = LOG_RAW;
        break;
    }

    if (device->context->errout.debug < loglevel) {
        // skip it
        return;
    }

    buf[0] = '\0';
    va_start(ap, fmt);
    gpsd_vlog(loglevel, &device->context->errout, buf, sizeof(buf), fmt, ap);
    va_end(ap);
}

/* gpsd_clear().- set and clear some data storage fields.
 * device has been opened.
 * So some things like path and gpsd_fd are already set.
 *
 * Return: void
 */
void gpsd_clear(struct gps_device_t *session)
{
    (void)clock_gettime(CLOCK_REALTIME, &session->gpsdata.online);
    lexer_init(&session->lexer, &session->context->errout);
    // session->gpsdata.online = 0;
    gps_clear_att(&session->gpsdata.attitude);
    gps_clear_dop(&session->gpsdata.dop);
    gps_clear_fix(&session->gpsdata.fix);
    gps_clear_gst(&session->gpsdata.gst);
    session->releasetime = (time_t)0;
    session->badcount = 0;

    // clear the private data union
    memset((void *)&session->driver, '\0', sizeof(session->driver));
    // set up the context structure for the PPS thread monitor
    memset_volatile(&session->pps_thread, 0, sizeof(session->pps_thread));
    session->pps_thread.devicefd = session->gpsdata.gps_fd;
    session->pps_thread.devicename = session->gpsdata.dev.path;
    session->pps_thread.log_hook = ppsthread_log;
    session->pps_thread.context = (void *)session;

    session->opentime = time(NULL);
}

/* split s into host and service parts
 * if service is not specified, *service is assigned to NULL
 * device is currently always assigned to NULL
 * return: -1 on error, 0 otherwise
 */
int parse_uri_dest(char *s, char **host, char **service, char **device)
{
    char *search = s;

    if ('[' == s[0]) {
        // IPv6 literal
        char *cb = strchr(s, ']');
        if (NULL == cb) {
            // missing terminating ]
            return -1;
        }
        *cb = '\0';
        *host = s + 1;
        search = cb + 1;
    } else {
        // IPv4 literal, or hostname
        *host = s;
    }
    s = strchr(search, ':');
    if (NULL != s) {
        // found a colon, remove it from host
        *s = '\0';
        search = s + 1;
        if ('\0' != s[1] && ':' != s[1]) {
            // s[1] start port/service
            *service = s + 1;
        } else {
            *service = NULL;
        }
    } else {
        *service = NULL;
    }
    s = strchr(search, ':');
    if (NULL != s) {
        // found a colon, remove it
        *s = '\0';
        if ('\0' != s[1]) {
            *device = s + 1;
        } else {
            *device = NULL;
        }
    } else {
        *device = NULL;
    }
    /* Support trailing / in URIs, e.g. tcp://192.168.100.90:1234/
     * Assumes / is not valid _inside_ host or service parts of URI
     * Accepts strange input, e.g. service part with / but no digits
     */
    s = strchr(*host, '/');
    if (NULL != s) {
        // found a backslash, remove it
        *s = '\0';
    }
    /* Don't enforce that / should not be in both host and service
     * Or that if host has /, there shouldn't be any service at all */
    if (NULL != *service) {
        s = strchr(*service, '/');
        if (NULL != s) {
            // found a backslash, remove it
            *s = '\0';
        }
        // nothing in service besides / --> trigger use of default port
        if ('\0' == *service[0]) {
            *service = NULL;
        }
    }
    return 0;
}

/* open a device for access to its data *
 * return: the opened file descriptor
 *         PLACEHOLDING_FD (-2) - for /dev/ppsX, ntrip waiting reconnect, etc.
 *         UNALLOCATED_FD (-1) - for open failure
 */
int gpsd_open(struct gps_device_t *session)
{
    // cast for 32-bit ints
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CORE: gpsd_open(%s) fld %ld\n",
             session->gpsdata.dev.path,
             (long)session->gpsdata.gps_fd);

    // special case: source may be a URI to a remote GNSS or DGPS service
    if (netgnss_uri_check(session->gpsdata.dev.path)) {
        session->gpsdata.gps_fd = netgnss_uri_open(session,
                                                   session->gpsdata.dev.path);
        session->sourcetype = SOURCE_TCP;
        // cast for 32-bit ints.
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CORE: netgnss_uri_open(%s) returns socket on fd %ld\n",
                 session->gpsdata.dev.path, (long)session->gpsdata.gps_fd);
        return (int)session->gpsdata.gps_fd;
    // otherwise, could be an TCP data feed
    } else if (str_starts_with(session->gpsdata.dev.path, "tcp://")) {
        char server[GPS_PATH_MAX], *host, *port, *device;
        socket_t dsock;
        char addrbuf[50];    // INET6_ADDRSTRLEN

        session->sourcetype = SOURCE_TCP;
        (void)strlcpy(server, session->gpsdata.dev.path + 6, sizeof(server));
        INVALIDATE_SOCKET(session->gpsdata.gps_fd);
        if (-1 == parse_uri_dest(server, &host, &port, &device) ||
            !port) {
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "CORE: Missing service in TCP feed spec %s\n",
                     session->gpsdata.dev.path);
            return UNALLOCATED_FD ;
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CORE: opening TCP feed at %s, port %s.\n", host,
                 port);
        // open non-blocking
        dsock = netlib_connectsock1(AF_UNSPEC, host, port, "tcp",
                                    1, false, addrbuf, sizeof(addrbuf));
        if (0 > dsock) {
            // cast for 32-bit ints.
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "CORE: TCP %s IP %s, open error %s(%ld).\n",
                     session->gpsdata.dev.path, addrbuf,
                     netlib_errstr(dsock), (long)dsock);
        } else {
            // cast for 32-bit ints.
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "CORE: TCP %s IP %s opened on fd %ld\n",
                     session->gpsdata.dev.path, addrbuf, (long)dsock);
        }
        session->gpsdata.gps_fd = dsock;
        return session->gpsdata.gps_fd;
    // or could be UDP
    } else if (str_starts_with(session->gpsdata.dev.path, "udp://")) {
        char server[GPS_PATH_MAX], *host, *port, *device;
        socket_t dsock;

        session->sourcetype = SOURCE_UDP;
        (void)strlcpy(server, session->gpsdata.dev.path + 6, sizeof(server));
        INVALIDATE_SOCKET(session->gpsdata.gps_fd);
        if (-1 == parse_uri_dest(server, &host, &port, &device) ||
            !port) {
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "CORE: Missing service in UDP feed spec.\n");
            return -1;
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CORE: opening UDP feed at %s, port %s.\n", host,
                 port);
        if (0 > (dsock = netlib_connectsock1(AF_UNSPEC, host, port, "udp",
                                             1, true, NULL, 0))) {
            // cast for 32-bit ints.
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "CORE: UDP device open error %s(%ld).\n",
                     netlib_errstr(dsock), (long)dsock);
            return -1;
        } else {
            // cast for 32-bit ints.
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "CORE: UDP device opened on fd %ld\n", (long)dsock);
        }
        session->gpsdata.gps_fd = dsock;
        return session->gpsdata.gps_fd;
    }
    if (str_starts_with(session->gpsdata.dev.path, "gpsd://")) {
        /* could be:
         *    gpsd://[ipv6]
         *    gpsd://ipv4
         *    gpsd://hostname
         *    gpsd://[ipv6]:port
         *    gpsd://ipv4:port
         *    gpsd://hostname:port
         *    gpsd://[ipv6]:port:/device
         *    gpsd://ipv4:port:/device
         *    gpsd://hostname:port:/device
         *    gpsd://[ipv6]::/device
         *    gpsd://ipv4::/device
         *    gpsd://hostname::/device
         */
        char server[GPS_PATH_MAX], *host, *port, *device;
        socket_t dsock;

        session->sourcetype = SOURCE_GPSD;
        (void)strlcpy(server, session->gpsdata.dev.path + 7, sizeof(server));
        INVALIDATE_SOCKET(session->gpsdata.gps_fd);
        if (-1 == parse_uri_dest(server, &host, &port, &device)) {
                GPSD_LOG(LOG_ERROR, &session->context->errout,
                        "CORE: Malformed URI specified.\n");
                return -1;
        }
        if (!port) {
            port = DEFAULT_GPSD_PORT;
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CORE: opening remote gpsd feed at %s, port %s.\n",
                 host, port);
        if (0 > (dsock = netlib_connectsock(AF_UNSPEC, host, port, "tcp"))) {
            // cast for 32-bit ints.
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "CORE: remote gpsd device open error %s(%ld).\n",
                     netlib_errstr(dsock), (long)dsock);
            return -1;
        } // else
        // cast for 32-bit ints.
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CORE: remote gpsd feed opened on fd %ld\n", (long) dsock);

        // watch to remote is issued when WATCH is
        session->gpsdata.gps_fd = dsock;
        return session->gpsdata.gps_fd;
    }
#if defined(NMEA2000_ENABLE)
    if (str_starts_with(session->gpsdata.dev.path, "nmea2000://")) {
        return nmea2000_open(session);
    }
#endif  // defined(NMEA2000_ENABLE)
    /* fall through to plain serial open.
     * could be a naked /dev/ppsX */
    return gpsd_serial_open(session);
}

/* acquire a connection to the GPS device
 * could be serial, udp://, tcp://, etc.
 *
 * Return: fd on success
 *         less than zero on failure
 *         UNALLOCATED_FD (-1)  -- give up
 *         PLACEHOLDING_FD (-2) -- retry possible
 */
int gpsd_activate(struct gps_device_t *session, const int mode)
{
    // cast for 32-bit ints
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CORE: gpsd_activate(%s, %d) fd %ld\n",
             session->gpsdata.dev.path, mode,
             (long)session->gpsdata.gps_fd);

    if (O_OPTIMIZE == mode) {
        gpsd_run_device_hook(&session->context->errout,
                             session->gpsdata.dev.path, HOOK_ACTIVATE);
    }
    session->gpsdata.gps_fd = (gps_fd_t)gpsd_open(session);
    if (O_CONTINUE != mode) {
        session->mode = mode;
    }

    if (0 > session->gpsdata.gps_fd) {
        // return could be -1, PLACEHOLDING_FD, of UNALLOCATED_FD
        // could be ntrip:// reconnect in progress
        if (PLACEHOLDING_FD == session->gpsdata.gps_fd &&
            SOURCE_PPS == session->sourcetype &&
            NULL == session->pps_thread.report_hook) {
            /* it is /dev/ppsX, need to set devicename, etc.
             * check report_hook to ensure not already running
             * cast for 32-bit ints */
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "CORE: to gpsd_clear() fd %ld\n",
                     (long)session->gpsdata.gps_fd);
            gpsd_clear(session);
        }
        return session->gpsdata.gps_fd;
    }

    // if it's a sensor, it must be probed
    if ((SERVICE_SENSOR == session->servicetype) &&
        (SOURCE_CAN != session->sourcetype)) {
        const struct gps_type_t **dp;

        for (dp = gpsd_drivers; *dp; dp++) {
            if (NULL != (*dp)->probe_detect) {
                GPSD_LOG(LOG_PROG, &session->context->errout,
                         "CORE: Probing \"%s\" driver...\n",
                         (*dp)->type_name);
                // toss stale data
                (void)tcflush(session->gpsdata.gps_fd, TCIOFLUSH);
                if (0 != (*dp)->probe_detect(session)) {
                    GPSD_LOG(LOG_PROG, &session->context->errout,
                             "CORE: Probe found \"%s\" driver...\n",
                             (*dp)->type_name);
                    session->device_type = *dp;
                    gpsd_assert_sync(session);
                    goto foundit;
                } else {
                    GPSD_LOG(LOG_PROG, &session->context->errout,
                             "CORE: Probe not found \"%s\" driver...\n",
                             (*dp)->type_name);
                }
            }
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CORE: no probe matched...\n");
    }
foundit:

    gpsd_clear(session);
    /*
     * We might know the device's type, but we shouldn't assume it has
     * retained its settings.  A revert hook might well have undone
     * them on the previous close.  Fire a reactivate event so drivers
     * can do something about this if they choose.
     */
    if (NULL != session->device_type &&
        NULL != session->device_type->event_hook) {
        session->device_type->event_hook(session, EVENT_REACTIVATE);
    }
    // cast for 32-bit ints
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CORE: activate fd %ld done\n",
             (long)session->gpsdata.gps_fd);

    return session->gpsdata.gps_fd;
}


/*****************************************************************************

Carl Carter of SiRF supplied this algorithm for computing DOPs from
a list of visible satellites (some typos corrected)...

For satellite n, let az(n) = azimuth angle from North and el(n) be elevation.
Let:

    a(k, 1) = sin az(k) * cos el(k)
    a(k, 2) = cos az(k) * cos el(k)
    a(k, 3) = sin el(k)

Then form the line-of-sight matrix A for satellites used in the solution:

    | a(1,1) a(1,2) a(1,3) 1 |
    | a(2,1) a(2,2) a(2,3) 1 |
    |   :       :      :   : |
    | a(n,1) a(n,2) a(n,3) 1 |

And its transpose A~:

    |a(1, 1) a(2, 1) .  .  .  a(n, 1) |
    |a(1, 2) a(2, 2) .  .  .  a(n, 2) |
    |a(1, 3) a(2, 3) .  .  .  a(n, 3) |
    |    1       1   .  .  .     1    |

Compute the covariance matrix (A~*A)^-1, which is guaranteed symmetric:

    | s(x)^2    s(x)*s(y)  s(x)*s(z)  s(x)*s(t) |
    | s(y)*s(x) s(y)^2     s(y)*s(z)  s(y)*s(t) |
    | s(z)*s(x) s(z)*s(y)  s(z)^2     s(z)*s(t) |
    | s(t)*s(x) s(t)*s(y)  s(t)*s(z)  s(t)^2    |

Then:

GDOP = sqrt(s(x)^2 + s(y)^2 + s(z)^2 + s(t)^2)
TDOP = sqrt(s(t)^2)
PDOP = sqrt(s(x)^2 + s(y)^2 + s(z)^2)
HDOP = sqrt(s(x)^2 + s(y)^2)
VDOP = sqrt(s(z)^2)

Here's how we implement it...

First, each compute element P(i,j) of the 4x4 product A~*A.
If S(k=1,k=n): f(...) is the sum of f(...) as k varies from 1 to n, then
applying the definition of matrix product tells us:

P(i,j) = S(k=1,k=n): B(i, k) * A(k, j)

But because B is the transpose of A, this reduces to

P(i,j) = S(k=1,k=n): A(k, i) * A(k, j)

This is not, however, the entire algorithm that SiRF uses.  Carl writes:

> As you note, with rounding accounted for, most values agree exactly, and
> those that don't agree our number is higher.  That is because we
> deweight some satellites and account for that in the DOP calculation.
> If a satellite is not used in a solution at the same weight as others,
> it should not contribute to DOP calculation at the same weight.  So our
> internal algorithm does a compensation for that which you would have no
> way to duplicate on the outside since we don't output the weighting
> factors.  In fact those are not even available to API users.

Queried about the deweighting, Carl says:

> In the SiRF tracking engine, each satellite track is assigned a quality
> value based on the tracker's estimate of that signal.  It includes C/No
> estimate, ability to hold onto the phase, stability of the I vs. Q phase
> angle, etc.  The navigation algorithm then ranks all the tracks into
> quality order and selects which ones to use in the solution and what
> weight to give those used in the solution.  The process is actually a
> bit of a "trial and error" method -- we initially use all available
> tracks in the solution, then we sequentially remove the lowest quality
> ones until the solution stabilizes.  The weighting is inherent in the
> Kalman filter algorithm.  Once the solution is stable, the DOP is
> computed from those SVs used, and there is an algorithm that looks at
> the quality ratings and determines if we need to deweight any.
> Likewise, if we use altitude hold mode for a 3-SV solution, we deweight
> the phantom satellite at the center of the Earth.

So we cannot exactly duplicate what SiRF does internally.  We'll leave
HDOP alone and use our computed values for VDOP and PDOP.  Note, this
may have to change in the future if this code is used by a non-SiRF
driver.

******************************************************************************/


static gps_mask_t fill_dop(const struct gpsd_errout_t *errout,
                           const struct gps_data_t * gpsdata,
                           struct dop_t * dop)
{
    double prod[4][4] = {0};
    double inv[4][4] = {0};
    double satpos[MAXCHANNELS][4] = {0};
    double xdop, ydop, hdop, vdop, pdop, tdop, gdop;
    int i, j, k, n;

    for (n = k = 0; k < gpsdata->satellites_visible; k++) {
        // This double counts single sats where we got 2 signals from them.
#ifdef __UNUSED__
        GPSD_LOG(LOG_IO, errout,
                 "CORE: PRN %d used %d az %.1f el %.1f\n",
                gpsdata->skyview[k].PRN,
                gpsdata->skyview[k].used,
                gpsdata->skyview[k].azimuth,
                gpsdata->skyview[k].elevation);
#endif  // __UNUSED__
        if (!gpsdata->skyview[k].used) {
             // skip unused sats
             continue;
        }
        if (1 > gpsdata->skyview[k].PRN) {
             // skip bad PRN
             continue;
        }
        if (0 == isfinite(gpsdata->skyview[k].azimuth) ||
            0 > gpsdata->skyview[k].azimuth ||
            359 < gpsdata->skyview[k].azimuth) {
             // skip bad azimuth
             continue;
        }
        if (0 == isfinite(gpsdata->skyview[k].elevation) ||
            90 < fabs(gpsdata->skyview[k].elevation)) {
             // skip bad elevation
             continue;
        }
        const struct satellite_t *sp = &gpsdata->skyview[k];
        satpos[n][0] = sin(sp->azimuth * DEG_2_RAD)
            * cos(sp->elevation * DEG_2_RAD);
        satpos[n][1] = cos(sp->azimuth * DEG_2_RAD)
            * cos(sp->elevation * DEG_2_RAD);
        satpos[n][2] = sin(sp->elevation * DEG_2_RAD);
        satpos[n][3] = 1;
        GPSD_LOG(LOG_INF, errout,
                 "CORE: PRN %3d az %5.1f el %4.1f (%9.6f, %9.6f, %9.6f)\n",
                 gpsdata->skyview[k].PRN,
                 gpsdata->skyview[k].azimuth,
                 gpsdata->skyview[k].elevation,
                 satpos[n][0], satpos[n][1], satpos[n][2]);
        n++;
    }
    /* can't use gpsdata->satellites_used as that is a counter for xxGSA,
     * and gets cleared at odd times */
    GPSD_LOG(LOG_INF, errout, "CORE: Sats used %d visible %d:\n",
             n, gpsdata->satellites_visible);

    /* If we don't have 4 satellites then we don't have enough
     * information to calculate DOPS */
    if (n < 4) {
#ifdef __UNUSED__
        GPSD_LOG(LOG_RAW, errout,
                 "CORE: Not enough satellites available %d < 4:\n", n);
#endif  // __UNUSED__
        // Is this correct return code here? or should it be ERROR_SET
        return 0;
    }

#ifdef __UNUSED__
    GPSD_LOG(LOG_INF, errout, "CORE: Line-of-sight matrix:\n");
    for (k = 0; k < n; k++) {
        GPSD_LOG(LOG_INF, errout, "CORE: %f %f %f %f\n",
                 satpos[k][0], satpos[k][1], satpos[k][2], satpos[k][3]);
    }
#endif  // __UNUSED__

    for (i = 0; i < 4; ++i) {           // < rows
        for (j = 0; j < 4; ++j) {       // < cols
            prod[i][j] = 0.0;
            for (k = 0; k < n; ++k) {
                prod[i][j] += satpos[k][i] * satpos[k][j];
            }
        }
    }

#ifdef __UNUSED__
    GPSD_LOG(LOG_INF, errout, "CORE: product:\n");
    for (k = 0; k < 4; k++) {
        GPSD_LOG(LOG_INF, errout, "CORE: %f %f %f %f\n",
                 prod[k][0], prod[k][1], prod[k][2], prod[k][3]);
    }
#endif  // __UNUSED__

    if (matrix_invert(prod, inv)) {
#ifdef __UNUSED__
        /*
         * Note: this will print garbage unless all the subdeterminants
         * are computed in the invert() function.
         */
        GPSD_LOG(LOG_RAW, errout, "CORE: inverse:\n");
        for (k = 0; k < 4; k++) {
            GPSD_LOG(LOG_RAW, errout,
                     "CORE: %f %f %f %f\n",
                     inv[k][0], inv[k][1], inv[k][2], inv[k][3]);
        }
#endif  // __UNUSED__
    } else {
        GPSD_LOG(LOG_DATA, errout,
                 "CORE: LOS matrix singular, DOPs fail - source '%s'\n",
                 gpsdata->dev.path);
        return 0;
    }

    xdop = sqrt(inv[0][0]);
    ydop = sqrt(inv[1][1]);
    hdop = sqrt(inv[0][0] + inv[1][1]);
    vdop = sqrt(inv[2][2]);
    pdop = sqrt(inv[0][0] + inv[1][1] + inv[2][2]);
    tdop = sqrt(inv[3][3]);
    gdop = sqrt(inv[0][0] + inv[1][1] + inv[2][2] + inv[3][3]);

    GPSD_LOG(LOG_DATA, errout,
             "CORE: DOPS computed/reported: X=%f/%f Y=%f/%f H=%f/%f "
             "V=%f/%f P=%f/%f T=%f/%f G=%f/%f\n",
             xdop, dop->xdop, ydop, dop->ydop, hdop, dop->hdop, vdop,
             dop->vdop, pdop, dop->pdop, tdop, dop->tdop, gdop, dop->gdop);

    /* Check to see which DOPs we already have.  Save values if no value
     * from the GPS.  Do not overwrite values which came from the GPS */
    if (0 == isfinite(dop->xdop)) {
        dop->xdop = xdop;
    }
    if (0 == isfinite(dop->ydop)) {
        dop->ydop = ydop;
    }
    if (0 == isfinite(dop->hdop)) {
        dop->hdop = hdop;
    }
    if (0 == isfinite(dop->vdop)) {
        dop->vdop = vdop;
    }
    if (0 == isfinite(dop->pdop)) {
        dop->pdop = pdop;
    }
    if (0 == isfinite(dop->tdop)) {
        dop->tdop = tdop;
    }
    if (0 == isfinite(dop->gdop)) {
        dop->gdop = gdop;
    }

    return DOP_SET;
}

/* compute errors and derived quantities
 * also a handy place to do final sanity checking */
static void gpsd_error_model(struct gps_device_t *session)
{
    struct gps_fix_t *fix;           // current fix
    struct gps_fix_t *lastfix;       // last fix, maybe same time stamp
    struct gps_fix_t *oldfix;        // old fix, previous time stamp
    struct gps_fix_t *newfix;        // new fix (just merged)
    double deltatime = -1.0;         // time span to compute rates

    /*
     * Now we compute derived quantities.  This is where the tricky error-
     * modeling stuff goes. Presently we don't know how to derive
     * time error.
     *
     * Some drivers set the error fields.  No NMEA 183 reports climb error.
     * $GPXTE and $PSRFEPE can report track error, but are rare.  Whenever
     * possible, we step aside and allow the GNSS receiver error estimates
     * to be used.  But even they are only Wild Ass Guesses (WAGs).
     *
     * The UERE constants are our assumption about the base error of
     * GPS fixes in different directions.
     *
     * UERE is actually a variable sent in the Almanac, so assuming
     * a UERE constant is bogus, as is using it this way.
     *
     * Assuming that DGPS has substantially better accuracy than plain
     * GPS is also a fallacy.  Extending this to RTK is building false
     * conjecture on top of misplaced wishful thinking.
     */
#define H_UERE_NO_DGPS          15.0    // meters, 95% confidence
#define H_UERE_WITH_DGPS        3.75    // meters, 95% confidence
#define V_UERE_NO_DGPS          23.0    // meters, 95% confidence
#define V_UERE_WITH_DGPS        5.75    // meters, 95% confidence
#define P_UERE_NO_DGPS          19.0    // meters, 95% confidence
#define P_UERE_WITH_DGPS        4.75    // meters, 95% confidence
    double h_uere, v_uere, p_uere;

    if (NULL == session) {
        return;
    }

    fix = &session->gpsdata.fix;
    lastfix = &session->lastfix;
    oldfix = &session->oldfix;
    newfix = &session->newdata;         // For whether rcvr supplies values

    if (0 < fix->time.tv_sec) {
        // we have a time for this merge data

        deltatime = TS_SUB_D(&fix->time, &lastfix->time);

        if (0.0099 < fabs(deltatime)) {
            /* Time just moved, probably forward at least 10 ms.
             * Lastfix is now the previous (old) fix. */
            *oldfix = *lastfix;
        } else {
            // compute delta from old fix
            deltatime = TS_SUB_D(&fix->time, &oldfix->time);
        }
    }
    // Sanity check for negative delta?

    // adjusting UERE for DGPS is dodgy...
    h_uere =
        (session->gpsdata.fix.status ==
         STATUS_DGPS ? H_UERE_WITH_DGPS : H_UERE_NO_DGPS);
    v_uere =
        (session->gpsdata.fix.status ==
         STATUS_DGPS ? V_UERE_WITH_DGPS : V_UERE_NO_DGPS);
    p_uere =
        (session->gpsdata.fix.status ==
         STATUS_DGPS ? P_UERE_WITH_DGPS : P_UERE_NO_DGPS);

    if (0 == isfinite(fix->latitude) ||
        0 == isfinite(fix->longitude) ||  // both lat/lon, or none
        90.0 < fabs(fix->latitude) ||     // lat out of range
        180.0 < fabs(fix->longitude)) {   // lon out of range
        fix->latitude = fix->longitude = NAN;
    }
    // validate ECEF
    if (0 == isfinite(fix->ecef.x) ||
        0 == isfinite(fix->ecef.y) ||
        0 == isfinite(fix->ecef.z) ||
        10.0 >= (fabs(fix->ecef.x) +
                 fabs(fix->ecef.y) +
                 fabs(fix->ecef.z))) {    // all zeros
        fix->ecef.x = fix->ecef.y = fix->ecef.z = NAN;
    }

    // if we have not lat/lon, but do have ECEF, calculate lat/lon
    if ((0 == isfinite(fix->longitude) ||
         0 == isfinite(fix->latitude)) &&
        0 != isfinite(fix->ecef.x)) {
        session->gpsdata.set |= ecef_to_wgs84fix(fix,
                                                 fix->ecef.x, fix->ecef.y,
                                                 fix->ecef.z, fix->ecef.vx,
                                                 fix->ecef.vy, fix->ecef.vz);
    }

    /* If you are in a rocket, and your GPS is ITAR unlocked, then
     * triple check these sanity checks.
     *
     * u-blox 8: Max altitude: 50,000m
     *           Max horizontal speed: 250 m/s
     *           Max climb: 100 m/s
     *
     * u-blox ZED-F9P: Max Velocity: 500 m/s
     */

    /* sanity check the speed, 10,000 m/s should be a nice max
     * Low Earth Orbit (LEO) is about 7,800 m/s */
    if (9999.9 < fabs(fix->speed))
        fix->speed = NAN;

    if (9999.9 < fabs(fix->NED.velN))
        fix->NED.velN = NAN;
    if (9999.9 < fabs(fix->NED.velE))
        fix->NED.velE = NAN;
    if (9999.9 < fabs(fix->NED.velD))
        fix->NED.velD = NAN;

    // sanity check the climb, 10,000 m/s should be a nice max
    if (9999.9 < fabs(fix->climb))
        fix->climb = NAN;
    if (0 != isfinite(fix->NED.velD) &&
        0 == isfinite(fix->climb)) {
        // have good velD, use it for climb
        fix->climb = -fix->NED.velD;
    }

    // compute speed and track from velN and velE if needed and possible
    if (0 != isfinite(fix->NED.velN) &&
        0 != isfinite(fix->NED.velE)) {
        if (0 == isfinite(fix->speed)) {
            fix->speed = hypot(fix->NED.velN, fix->NED.velE);
        }
        if (0 == isfinite(fix->track)) {
            fix->track = atan2(fix->NED.velE, fix->NED.velN) * RAD_2_DEG;
            // normalized later
        }
    }

    /*
     * OK, this is not an error computation, but we're at the right
     * place in the architecture for it.  Compute geoid separation
     * and altHAE and altMSL in the simplest possible way.
     */

    // geoid (ellipsoid) separation and variation
    if (0 != isfinite(fix->latitude) &&
        0 != isfinite(fix->longitude)) {
        if (0 == isfinite(fix->geoid_sep)) {
            fix->geoid_sep = wgs84_separation(fix->latitude,
                                              fix->longitude);
        }
        if (0 == isfinite(fix->magnetic_var) ||
            0.09 >= fabs(fix->magnetic_var)) {
            // some GPS set 0.0,E, or 0,W instead of blank
            fix->magnetic_var = mag_var(fix->latitude,
                                        fix->longitude);
        }
    }

    if (0 != isfinite(fix->magnetic_var)) {
        if (0 == isfinite(fix->magnetic_track) &&
            0 != isfinite(fix->track)) {

            // calculate mag track, normalized later
            fix->magnetic_track = fix->track + fix->magnetic_var;
        } else if (0 != isfinite(fix->magnetic_track) &&
                   0 == isfinite(fix->track)) {

            // calculate true track, normalized later
            fix->track = fix->magnetic_track - fix->magnetic_var;
        }
    }
    if (0 != isfinite(fix->track)) {
            // normalize true track
            DEG_NORM(fix->track);
    }

    if (0 != isfinite(fix->magnetic_track)) {
            // normalize mag track
            DEG_NORM(fix->magnetic_track);
    }

    if (0 != isfinite(fix->geoid_sep)) {
        if (0 != isfinite(fix->altHAE) &&
            0 == isfinite(fix->altMSL)) {
            // compute missing altMSL
            fix->altMSL = fix->altHAE - fix->geoid_sep;
        } else if (0 == isfinite(fix->altHAE) &&
                   0 != isfinite(fix->altMSL)) {
            // compute missing altHAE
            fix->altHAE = fix->altMSL + fix->geoid_sep;
        }
    }

    /*
     * OK, this is not an error computation, but we're at the right
     * place in the architecture for it.  Compute speed over ground
     * and climb/sink in the simplest possible way.
     */

#ifdef  __UNUSED__
    // debug code
    {
        char tbuf[JSON_DATE_MAX+1];
        GPSD_LOG(LOG_SHOUT, &session->context->errout,
                 "CORE: time %s deltatime %f\n",
                 timespec_to_iso8601(fix->time, tbuf, sizeof(tbuf)),
                 deltatime);
    }
#endif // __UNUSED__

    if (0 < deltatime) {
        // have a valid time duration
        // FIXME! ignore if large.  maybe > 1 hour?

        if (MODE_2D <= fix->mode &&
            MODE_2D <= oldfix->mode) {

            if (0 == isfinite(fix->speed)) {
                fix->speed = earth_distance(fix->latitude,
                                            fix->longitude,
                                            oldfix->latitude,
                                            oldfix->longitude) / deltatime;
                // sanity check
                if (9999.9 < fabs(fix->speed))
                    fix->speed = NAN;
            }

            if (MODE_3D <= fix->mode &&
                MODE_3D <= oldfix->mode &&
                0 == isfinite(fix->climb) &&
                0 != isfinite(fix->altHAE) &&
                0 != isfinite(oldfix->altHAE)) {
                    fix->climb = (fix->altHAE - oldfix->altHAE) / deltatime;

                    // sanity check the climb
                    if (9999.9 < fabs(fix->climb))
                        fix->climb = NAN;
            }
        }
    }

    /*
     * Field reports match the theoretical prediction that
     * expected time error should be half the resolution of
     * the GPS clock, so we put the bound of the error
     * in as a constant pending getting it from each driver.
     *
     * In an ideal world, we'd increase this if no leap-second has
     * been seen and it's less than 750s (one almanac load cycle) from
     * device powerup. Alas, we have no way to know when device
     * powerup occurred - depending on the receiver design it could be
     * when the hardware was first powered up or when it was first
     * opened.  Also, some devices (notably plain NMEA0183 receivers)
     * never ship an indication of when they have valid leap second.
     */
    if (0 < fix->time.tv_sec &&
        0 == isfinite(fix->ept)) {
        // can we compute ept from tdop?
        fix->ept = 0.005;
    }

    // Other error computations depend on having a valid fix
    if (MODE_2D <= fix->mode) {
        if (0 == isfinite(newfix->epx) &&
            0 != isfinite(session->gpsdata.dop.xdop)) {
            fix->epx = session->gpsdata.dop.xdop * h_uere;
        }

        if (0 == isfinite(newfix->epy) &&
            0 != isfinite(session->gpsdata.dop.ydop)) {
            fix->epy = session->gpsdata.dop.ydop * h_uere;
        }

        if (MODE_3D <= fix->mode &&
            0 == isfinite(fix->epv) &&
            0 != isfinite(session->gpsdata.dop.vdop)) {
            fix->epv = session->gpsdata.dop.vdop * v_uere;
        }

        // 2D error
        if (0 == isfinite(fix->eph) &&
            0 != isfinite(session->gpsdata.dop.hdop)) {
            fix->eph = session->gpsdata.dop.hdop * p_uere;
        }

        // 3D error
        if (0 == isfinite(fix->sep) &&
            0 != isfinite(session->gpsdata.dop.pdop)) {
            fix->sep = session->gpsdata.dop.pdop * p_uere;
        }

        /*
         * If we have a current fix and an old fix, and the packet handler
         * didn't set the speed error, climb error or track error members
         * itself, try to compute them now.
         */

#define EMAX(x, y)     (((x) > (y)) ? (x) : (y))

        if (0 < deltatime &&
            MODE_2D <= oldfix->mode) {

            if (0 == isfinite(newfix->eps) &&
                0 != isfinite(oldfix->epx) &&
                0 != isfinite(oldfix->epy)) {
                    fix->eps = (EMAX(oldfix->epx, oldfix->epy) +
                                EMAX(fix->epx, fix->epy)) / deltatime;
            }

            if (0 == isfinite(fix->epd)) {
                /*
                 * We compute a track error estimate solely from the
                 * position of this fix and the last one.  The maximum
                 * track error, as seen from the position of last fix, is
                 * the angle subtended by the two most extreme possible
                 * error positions of the current fix; the expected track
                 * error is half that.  Let the position of the old fix be
                 * A and of the new fix B.  We model the view from A as
                 * two right triangles ABC and ABD with BC and BD both
                 * having the length of the new fix's estimated error.
                 * adj = len(AB), opp = len(BC) = len(BD), hyp = len(AC) =
                 * len(AD). This leads to spurious uncertainties
                 * near 180 when we're moving slowly; to avoid reporting
                 * garbage, throw back NaN if the distance from the previous
                 * fix is less than the error estimate.
                 */
                double adj = earth_distance(oldfix->latitude, oldfix->longitude,
                                            fix->latitude, fix->longitude);
                double opp = EMAX(fix->epx, fix->epy);
                if (isfinite(adj) != 0 && adj > opp) {
                    double hyp = sqrt(adj * adj + opp * opp);
                    fix->epd = RAD_2_DEG * 2 * asin(opp / hyp);
                }
            }

            if (0 == isfinite(newfix->epc) &&
                0 != isfinite(fix->epv) &&
                0 != isfinite(oldfix->epv)) {
                    // Is this really valid?
                    // if vertical uncertainties are zero this will be too
                    fix->epc = (oldfix->epv + fix->epv) / deltatime;
            }
        }
    }

#ifdef  __UNUSED__
    {
        // Debug code.
        char tbuf[JSON_DATE_MAX+1];
        GPSD_LOG(&session->context->errout, 0,
                 "CORE: %s deltatime %.3f, speed %0.3f climb %.3f "
                 "epc %.3f fixHAE %.3f oldHAE %.3f\n",
                 timespec_to_iso8601(fix->time, tbuf, sizeof(tbuf)),
                 deltatime, fix->speed, fix->climb, fix->epc,
                 fix->altHAE, oldfix->altHAE);
    }
#endif // __UNUSED__

    if (0 < fix->time.tv_sec) {
        // save lastfix, not yet oldfix, for later error computations
        *lastfix = *fix;
    }
}

/* await data from any socket in the all_fds set
 *
 * return: AWAIT_ value
 */
int gpsd_await_data(fd_set *rfds,
                    fd_set *efds,
                    int maxfd,
                    fd_set *all_fds,
                    struct gpsd_errout_t *errout,
                    timespec_t ts_timeout)
{
    int status;

    FD_ZERO(efds);
    *rfds = *all_fds;
    GPSD_LOG(LOG_RAW1, errout, "CORE: select waits, maxfd %d\n", maxfd);
    /*
     * Poll for user commands or GPS data.  The timeout doesn't
     * actually matter here since select returns whenever one of
     * the file descriptors in the set goes ready.  The point
     * of tracking maxfd is to keep the set of descriptors that
     * pselect(2) has to poll here as small as possible (for
     * low-clock-rate SBCs and the like).
     *
     * As used here, there is no difference between pselect()
     * or select().  A timeout is used, this adds a bit
     * of power consumption, but prevents infinite hang during autobaud,
     * or select.  pselect() may, or may not, modify ts_timeout.
     */
    errno = 0;

    status = pselect(maxfd + 1, rfds, NULL, NULL, &ts_timeout, NULL);
    if (-1 == status) {
        if (EINTR == errno) {
            // caught a signal
            return AWAIT_NOT_READY;
        }

        if (EBADF == errno) {
            // Invalid file descriptor.
            int fd;
            for (fd = 0; fd < (int)FD_SETSIZE; fd++) {
                /*
                 * All we care about here is a cheap, fast, uninterruptible
                 * way to check if a file descriptor is valid.
                 */
                if (FD_ISSET(fd, all_fds) && -1 == fcntl(fd, F_GETFL, 0)) {
                    FD_CLR(fd, all_fds);
                    FD_SET(fd, efds);
                }
            }
            return AWAIT_NOT_READY;
        }
        //  else
        GPSD_LOG(LOG_ERROR, errout, "CORE: pselect: %s(%d)\n",
                 strerror(errno), errno);
        return AWAIT_FAILED;
    }
    if (0 == status) {
        // pselect timeout
        GPSD_LOG(LOG_PROG, errout, "CORE: pselect: timeout\n");
        return AWAIT_TIMEOUT;
    }

    if (LOG_SPIN <= errout->debug) {
        int i;
        char dbuf[BUFSIZ];
        timespec_t ts_now;
        char ts_str[TIMESPEC_LEN];

        dbuf[0] = '\0';
        for (i = 0; i < (int)FD_SETSIZE; i++) {
            if (FD_ISSET(i, all_fds)) {
                str_appendf(dbuf, sizeof(dbuf), "%d ", i);
            }
        }
        str_rstrip_char(dbuf, ' ');
        (void)strlcat(dbuf, "} -> {", sizeof(dbuf));
        for (i = 0; i < (int)FD_SETSIZE; i++) {
            if (FD_ISSET(i, rfds)) {
                str_appendf(dbuf, sizeof(dbuf), " %d ", i);
            }
        }

        (void)clock_gettime(CLOCK_REALTIME, &ts_now);
        GPSD_LOG(LOG_SPIN, errout,
                 "CORE: pselect() {%s} at %s, %s(%d)\n",
                 dbuf,
                 timespec_str(&ts_now, ts_str, sizeof(ts_str)),
                 strerror(errno), errno);
    }

    return AWAIT_GOT_INPUT;
}

/* Return false - don't go to next hunt setting
 *        true - time to go to next hunt setting
 */
static bool hunt_failure(struct gps_device_t *session)
{
    /*
     * After a bad packet, what should cue us to go to next autobaud setting?
     * We have tried three different tests here.
     *
     * The first was session->badcount++>1.  This worked very well on
     * ttys for years and years, but caused failure to sync on TCP/IP
     * sources, which have I/O boundaries in mid-packet more often
     * than RS232 ones.  There's a test for this at
     * test/daemon/tcp-torture.log.
     *
     * The second was session->badcount++>1 && session->lexer.state==0.
     * Fail hunt only if we get a second consecutive bad packet
     * and the lexer is in ground state.  We don't want to fail on
     * a first bad packet because the source might have a burst of
     * leading garbage after open.  We don't want to fail if the
     * lexer is not in ground state, because that means the read
     * might have picked up a valid partial packet - better to go
     * back around the loop and pick up more data.
     *
     * The "&& session->lexer.state==0" guard causes an intermittent
     * hang while autobauding on SiRF IIIs (but not on SiRF-IIs, oddly
     * enough).  Removing this conjunct resurrected the failure
     * of test/daemon/tcp-torture.log.
     *
     * Our third attempt, isatty(session->gpsdata.gps_fd) != 0
     * && session->badcount++ > 1, reverts to the old test that worked
     * well on ttys for ttys and prevents non-tty devices from *ever*
     * having hunt failures. This has the cost that non-tty devices
     * will never get kicked off for presenting bad packets.
     *
     * Slightly refactored, but equivalent, in 3.23.1
     *
     * This test may need further revision.
     */
    if (0 >= gpsd_serial_isatty(session)) {
        // Not a tty, so can't hunt.
        return false;
    }
    // It is a tty, but don't hunt if speed is fixed.
    if (0 != session->context->fixed_port_speed) {
        return false;
    }
    return 1 < session->badcount++;
}

// update the stuff in the scoreboard structure
// Also used by gpsdecode.c
gps_mask_t gpsd_poll(struct gps_device_t *session)
{
    ssize_t newlen;
    bool driver_change = false;
    timespec_t ts_now;
    timespec_t delta;
    char ts_buf[TIMESPEC_LEN];

    // Maybe only clear when we actually get a new packet?  How?
    gps_clear_fix(&session->newdata);

    /*
     * Input just became available from a sensor, but no read from the
     * device has yet been done.
     *
     * What we actually do here is trickier.  For latency-timing
     * purposes, we want to know the time at the start of the current
     * recording cycle. We rely on the fact that even at 4800bps
     * there's a quiet time perceptible to the human eye in gpsmon
     * between when the last character of the last packet in a
     * 1-second cycle ships and when the next reporting cycle
     * ships. Because the cycle time is fixed, higher baud rates will
     * make this gap larger.
     *
     * Thus, we look for an inter-character delay much larger than an
     * average 4800bps sentence time.  How should this delay be set?  Well,
     * counting framing bits and erring on the side of caution, it's
     * about 480 characters per second or 2083 microeconds per character;
     * that's almost exactly 0.125 seconds per average 60-char sentence.
     * Doubling this to avoid false positives, we look for an inter-character
     * delay of greater than 0.250s.
     *
     * The above assumes a cycle time of 1 second.  To get the minimum size of
     * the quiet period, we multiply by the device cycle time.
     *
     * We can sanity-check these calculation by watching logs. If we have set
     * MINIMUM_QUIET_TIME correctly, the "transmission pause" message below
     * will consistently be emitted just before the sentence that shows up
     * as start-of-cycle in gpsmon, and never emitted at any other point
     * in the cycle.
     *
     * In practice, it seems that edge detection succeeds at 9600bps but
     * fails at 4800bps.  This is not surprising, as previous profiling has
     * indicated that at 4800bps some devices overrun a 1-second cycle time
     * with the data they transmit.
     */
#define MINIMUM_QUIET_TIME      0.25
    if (0 == session->lexer.outbuflen) {
        /* beginning of a new packet, or not...
         * 0 == lexer.outbuf just means the last read was not a full packet.
         * that works on serial lines that dribble data.
         * usb tends to only send complete packets.
         * Worse, we do not know if we have a full packet this time.
         */
        (void)clock_gettime(CLOCK_REALTIME, &ts_now);
        if (NULL != session->device_type &&
            (0 < session->lexer.start_time.tv_sec ||
             0 < session->lexer.start_time.tv_nsec)) {
            const double min_cycle = TSTONS(&session->device_type->min_cycle);
            double quiet_time = (MINIMUM_QUIET_TIME * min_cycle);
            double gap;

            gap = TS_SUB_D(&ts_now, &session->lexer.start_time);

            // used to compare gap > min_cycle, but min_cycle is now
            // so variable as to be not helpful.  Some GPS models can
            // vary from 20Hz to 1Hz.
            if (gap > quiet_time) {
                // quiet_time is getting less useful as GNSS receivers
                // have more data to send.
                GPSD_LOG(LOG_PROG, &session->context->errout,
                         "CORE: transmission pause. gap %f quiet_time %f\n",
                         gap, quiet_time);
                session->sor = ts_now;
                session->lexer.start_char = session->lexer.char_counter;
            }
        }
        session->lexer.start_time = ts_now;
    }

    if (COMMENT_PACKET <= session->lexer.type) {
        session->observed |= PACKET_TYPEMASK(session->lexer.type);
    }

    // can we get a full packet from the device/NTRIP/DGPS/tcp/etc.?
    if (NULL != session->device_type &&
        NULL != session->device_type->get_packet) {
        newlen = session->device_type->get_packet(session);
        // coverity[deref_ptr]
        GPSD_LOG(LOG_RAW, &session->context->errout,
                 "CORE: %s is known to be %s, packet type %d\n",
                 session->gpsdata.dev.path,
                 session->device_type->type_name,
                 session->lexer.type);
    } else {
        newlen = packet_get1(session);
    }

    // update the scoreboard structure from the GPS
    GPSD_LOG(LOG_RAW1, &session->context->errout,
             "CORE: %s sent %zd new characters\n",
             session->gpsdata.dev.path, newlen);

    (void)clock_gettime(CLOCK_REALTIME, &ts_now);
    TS_SUB(&delta, &ts_now, &session->gpsdata.online);
    if (0 > newlen) {           // read error
        GPSD_LOG(LOG_INF, &session->context->errout,
                 "CORE: %s returned error %zd (%s sec since data)\n",
                 session->gpsdata.dev.path, newlen,
                 timespec_str(&delta, ts_buf, sizeof(ts_buf)));
        session->gpsdata.online.tv_sec = 0;
        session->gpsdata.online.tv_nsec = 0;
        return ERROR_SET;
    }
    if (0 == newlen) {           // zero length read, possible EOF
        /*
         * Multiplier is 2 to avoid edge effects due to sampling at the exact
         * wrong time...
         * leave TCP network connection alone, let the TCP link timer expire
         * and throw an error.
         */
        if (0 < session->gpsdata.online.tv_sec &&
            SOURCE_TCP != session->sourcetype &&
            // FIXME: do this with integer math...
            TSTONS(&delta) >= (TSTONS(&session->gpsdata.dev.cycle) * 2)) {
            GPSD_LOG(LOG_INF, &session->context->errout,
                     "CORE: %s is offline (%s sec since data) cycle %lld "
                     "srctype %d\n",
                     session->gpsdata.dev.path,
                     timespec_str(&delta, ts_buf, sizeof(ts_buf)),
                     (long long)session->gpsdata.dev.cycle.tv_sec,
                     session->sourcetype);
            session->gpsdata.online.tv_sec = 0;
            session->gpsdata.online.tv_nsec = 0;
        }
        return NODATA_IS;
    }
    // else (0 < newlen), got at least something.
    session->lexer.pkt_time = ts_now;

    GPSD_LOG(LOG_RAW, &session->context->errout,
             "CORE: packet sniff on %s finds type %d\n",
             session->gpsdata.dev.path, session->lexer.type);
    if (COMMENT_PACKET == session->lexer.type) {
        // deal with regression test helper macros
        const char date_str[] = "# Date: ";

        session->badcount = 0;
        if (0 == strcmp((const char *)session->lexer.outbuffer,
                        "# EOF\n")) {
            // undocumented, used by gpsfake to signal EOF
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "CORE: synthetic EOF\n");
            return EOF_IS;
        }
        if (0 == strncmp((const char *)session->lexer.outbuffer,
                         date_str, sizeof(date_str) - 1)) {
            // # Date: yyyy-mm-dd
            // used by regression tests to correct
            // change start time, gps weeks, etc.
            gpsd_set_century(session);
            session->regression = 1;

            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "CORE: start_time %lld\n",
                     (long long)session->context->start_time);
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CORE: comment, sync lock deferred: >%s<\n",
                 session->lexer.outbuffer);
    } else if (COMMENT_PACKET < session->lexer.type) {
        if (NULL == session->device_type) {
            driver_change = true;
        } else {
            int newtype = session->lexer.type;
            /*
             * Are we seeing a new packet type? Then we probably
             * want to change drivers.
             */
            bool new_packet_type =
                (newtype != session->device_type->packet_type);
            /*
             * Possibly the old driver has a mode-switcher method, in
             * which case we know it can handle NMEA itself and may
             * want to do special things (like tracking whether a
             * previous mode switch to binary succeeded in suppressing
             * NMEA).
             */
            // QQQ: use STICKY() instead?
            bool dependent_nmea = (NMEA_PACKET == newtype &&
                               NULL != session->device_type->mode_switcher);

            /*
             * Compute whether to switch drivers.
             * If the previous driver type was sticky and this one
             * isn't, we'll revert after processing the packet.
             */
            driver_change = new_packet_type && !dependent_nmea;
        }
        if (driver_change) {
            const struct gps_type_t **dp;

            for (dp = gpsd_drivers; *dp; dp++) {
                if (session->lexer.type == (*dp)->packet_type) {
                    GPSD_LOG(LOG_PROG, &session->context->errout,
                             "CORE: switching to match packet type %d: %s\n",
                             session->lexer.type, gpsd_prettydump(session));
                    (void)gpsd_switch_driver(session, (*dp)->type_name);
                    break;
                }
            }
            if (NULL == *dp) {
                GPSD_LOG(LOG_WARN, &session->context->errout,
                         "CORE: no matching packet type %d\n",
                         session->lexer.type);
            }
        }
        session->badcount = 0;
        session->gpsdata.dev.driver_mode =
            (session->lexer.type > NMEA_PACKET) ? MODE_BINARY : MODE_NMEA;
    } else if (hunt_failure(session) && !gpsd_next_hunt_setting(session)) {
        (void)clock_gettime(CLOCK_REALTIME, &ts_now);
        TS_SUB(&delta, &ts_now, &session->gpsdata.online);
        GPSD_LOG(LOG_INF, &session->context->errout,
                 "CORE: hunt on %s failed (%s sec since data)\n",
                 session->gpsdata.dev.path,
                 timespec_str(&delta, ts_buf, sizeof(ts_buf)));
        return ERROR_SET;
    }

    if (0 == session->lexer.outbuflen) {      // got new data, but no packet
        GPSD_LOG(LOG_RAW1, &session->context->errout,
                 "CORE: New data on %s, not yet a packet\n",
                 session->gpsdata.dev.path);
        return ONLINE_SET;
    }

    // we have recognized a packet
    session->badcount = 0;
    gps_mask_t received = PACKET_SET;
    (void)clock_gettime(CLOCK_REALTIME, &session->gpsdata.online);

    GPSD_LOG(LOG_RAW1, &session->context->errout,
             "CORE: Accepted packet on %s.\n",
             session->gpsdata.dev.path);

    // track the packet count since achieving sync on the device
    if (driver_change &&
        0 == (session->drivers_identified & (1 << session->driver_index))) {

        // coverity[var_deref_op]
        GPSD_LOG(LOG_INF, &session->context->errout,
                 "CORE: %s identified as type %s, %ld sec\n",
                 session->gpsdata.dev.path,
                 session->device_type->type_name,
                 (long)(time(NULL) - session->opentime));

        if (0 < gpsd_serial_isatty(session)) {
            GPSD_LOG(LOG_INF, &session->context->errout,
                     "CORE: %s %ubps\n",
                     session->gpsdata.dev.path,
                     (unsigned int)gpsd_get_speed(session));
        }

        // fire the init_query method
        if (NULL != session->device_type &&
            NULL != session->device_type->init_query) {
            /*
             * We can force readonly off knowing this method does
             * not alter device state.
             */
            bool saved = session->context->readonly;
            session->context->readonly = false;
            session->device_type->init_query(session);
            session->context->readonly = saved;
        }

        // fire the identified hook
        if (NULL != session->device_type &&
            NULL != session->device_type->event_hook) {
            session->device_type->event_hook(session, EVENT_IDENTIFIED);
        }
        session->lexer.counter = 0;

        // let clients know about this.
        received |= DRIVER_IS;

        // mark the fact that this driver has been seen
        session->drivers_identified |= (1 << session->driver_index);
    } else {
        session->lexer.counter++;
    }

    // fire the configure hook, on every packet.  Seems excessive...
    if (NULL != session->device_type &&
        NULL != session->device_type->event_hook) {
        session->device_type->event_hook(session, EVENT_CONFIGURE);
    }

    GPSD_LOG(LOG_RAW, &session->context->errout,
             "CORE: raw packet of type %d, %zd:%s\n",
             session->lexer.type,
             session->lexer.outbuflen,
             gpsd_prettydump(session));

    // Get data from current packet into the fix structure
    if (COMMENT_PACKET != session->lexer.type &&
        BAD_PACKET != session->lexer.type &&
        NULL != session->device_type &&
        NULL != session->device_type->parse_packet) {
            received |= session->device_type->parse_packet(session);
            GPSD_LOG(LOG_SPIN, &session->context->errout,
                     "CORE: parse_packet() = %s\n", gps_maskdump(received));
    }

    /*
     * We may want to revert to the last driver that was marked
     * sticky.  What this accomplishes is that if we've just
     * processed something like AIVDM, but a driver with control
     * methods or an event hook had been active before that, we
     * keep the information about those capabilities.
     */
    if (!STICKY(session->device_type) &&
        NULL != session->last_controller &&
        STICKY(session->last_controller)) {
        session->device_type = session->last_controller;
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CORE: reverted to %s driver...\n",
                 session->device_type->type_name);
    }

    // are we going to generate a report? if so, count characters
    if (0 != (received & REPORT_IS)) {
        session->chars = session->lexer.char_counter -
                             session->lexer.start_char;
    }

    session->gpsdata.set = ONLINE_SET | received;

    // copy/merge device data into staging buffers
    if (0 != (session->gpsdata.set & CLEAR_IS)) {
        // CLEAR_IS should only be set on first sentence of cycle
        gps_clear_att(&session->gpsdata.attitude);
        if (0 == (session->gpsdata.set & DOP_SET)) {
            // FIXME: put gpsdata.dop in newdata.dop
            gps_clear_dop(&session->gpsdata.dop);
        }
        gps_clear_fix(&session->gpsdata.fix);
    }
#ifdef __UNUSED
    // debug
    GPSD_LOG(LOG_SHOUT, &session->context->errout,
             "CORE: before  alt %f new %f\n",
             session->gpsdata.fix.altMSL,
             session->newdata.altMSL);

    GPSD_LOG(LOG_SHOUT, &session->context->errout,
             "CORE: transfer mask: %s\n",
             gps_maskdump(session->gpsdata.set));
    GPSD_LOG(LOG_SHOUT, &session->context->errout,
             "CORE: SNARD before: status old %d new %d\n",
             session->gpsdata.fix.status,
             session->newdata.status);
#endif  // __UNUSED
    gps_merge_fix(&session->gpsdata.fix,
                  session->gpsdata.set, &session->newdata);

    /*
     * Compute fix-quality data from the satellite positions.
     * These will not overwrite any DOPs reported from the packet
     * we just got.
     */
    if (0 != (received & SATELLITE_SET) &&
        0 < session->gpsdata.satellites_visible) {
        session->gpsdata.set |= fill_dop(&session->context->errout,
                                         &session->gpsdata,
                                         &session->gpsdata.dop);
    }

    gpsd_error_model(session);

    /*
     * Count good fixes. We used to check
     *      session->gpsdata.fix.status > STATUS_UNK
     * here, but that wasn't quite right.  That tells us whether
     * we think we have a valid fix for the current cycle, but remains
     * true while following non-fix packets are received.  What we
     * really want to know is whether the last packet received was a
     * fix packet AND held a valid fix. We must ignore non-fix packets
     * AND packets which have fix data but are flagged as invalid. Some
     * devices output fix packets on a regular basis, even when unable
     * to derive a good fix. Such bad packets should set MODE_NO_FIX
     */
    if (0 != (session->gpsdata.set & (LATLON_SET|ECEF_SET))) {
        if (MODE_NO_FIX < session->gpsdata.fix.mode) {
            session->context->fixcnt++;
            session->fixcnt++;
        } else {
            session->context->fixcnt = 0;
            session->fixcnt = 0;
        }
    } else if (0 != (session->gpsdata.set & (MODE_SET))) {
        if (MODE_NO_FIX == session->gpsdata.fix.mode) {
            session->context->fixcnt = 0;
            session->fixcnt = 0;
        }
    }
    /*
     * Sanity check.  This catches a surprising number of port and
     * driver errors, including 32-vs.-64-bit problems.
     */
    if (0 != (session->gpsdata.set & TIME_SET)) {
        if (session->newdata.time.tv_sec >
            (time(NULL) + (60 * 60 * 24 * 365))) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "CORE: date (%lld) more than a year in the future!\n",
                     (long long)session->newdata.time.tv_sec);
        } else if (0 > session->newdata.time.tv_sec) {
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "CORE: date (%lld) is negative!\n",
                     (long long)session->newdata.time.tv_sec);
        }
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "CORE: gpsd_poll(%s) %s\n",
             session->gpsdata.dev.path,
             gps_maskdump(session->gpsdata.set));
    return session->gpsdata.set;
}

// consume and handle packets from a specified device
int gpsd_multipoll(const bool data_ready,
                   struct gps_device_t *device,
                   void (*handler)(struct gps_device_t *, gps_mask_t),
                   float reawake_time)
{
    if (data_ready) {
        int fragments;

        // cast for 32-bit ints
        GPSD_LOG(LOG_RAW1, &device->context->errout,
                 "CORE: polling %ld\n", (long)device->gpsdata.gps_fd);

        /*
         * Strange special case - the opening transaction on an NTRIP
         * connection may not yet be completed.
         * Try to ratchet things forward.
         */
        if (SERVICE_NTRIP == device->servicetype &&
            NTRIP_CONN_ESTABLISHED != device->ntrip.conn_state) {

            timespec_t ts_now;
            double step;

            (void)clock_gettime(CLOCK_REALTIME, &ts_now);

            step = TS_SUB_D(&ts_now, &device->ntrip.stream.stream_time);
            // wait 6 seconds between hitting ntrip_open()
            if (6 > fabs(step)) {
                return DEVICE_UNCHANGED;
            }
            device->ntrip.stream.stream_time = ts_now;
            (void)ntrip_open(device, "");
            if (NTRIP_CONN_ERR == device->ntrip.conn_state) {
                GPSD_LOG(LOG_WARN, &device->context->errout,
                         "CORE: connection to ntrip server failed\n");
                // FIXME: next stat after error should depend on if
                // initial connect or reconnect...
                device->ntrip.conn_state = NTRIP_CONN_CLOSED;
                return DEVICE_ERROR;
            }
            //  else
            return DEVICE_READY;
        }

        for (fragments = 0; ; fragments++) {
            gps_mask_t changed = gpsd_poll(device);

            if (EOF_IS == changed) {
                GPSD_LOG(LOG_WARN, &device->context->errout,
                         "CORE: device signed off %s\n",
                         device->gpsdata.dev.path);
                return DEVICE_EOF;
            }
            if (ERROR_SET == changed) {
                GPSD_LOG(LOG_WARN, &device->context->errout,
                         "CORE: device read of %s returned error or "
                         "packet sniffer failed sync (flags %s)\n",
                         device->gpsdata.dev.path,
                         gps_maskdump(changed));
                return DEVICE_ERROR;
            }
            if (NODATA_IS == changed) {
                /*
                 * No data on the first fragment read means the device
                 * fd may have been in an end-of-file condition on select.
                 */
                if (0 == fragments) {
                    GPSD_LOG(LOG_DATA, &device->context->errout,
                             "CORE: %s returned zero bytes\n",
                             device->gpsdata.dev.path);
                    if (device->zerokill) {
                        // failed timeout-and-reawake, kill it
                        gpsd_deactivate(device);
                        if (device->ntrip.works) {
                            // reset so we try this once only
                            device->ntrip.works = false;
                            if (0 > gpsd_activate(device, O_CONTINUE)) {
                                GPSD_LOG(LOG_WARN, &device->context->errout,
                                         "CORE: reconnect to ntrip server "
                                         "failed\n");
                                return DEVICE_ERROR;
                            }
                            // else
                            GPSD_LOG(LOG_INF, &device->context->errout,
                                     "CORE: reconnecting to ntrip server\n");
                            return DEVICE_READY;
                        }
                    } else if (0 == reawake_time) {
                        return DEVICE_ERROR;
                    } else {
                        /*
                         * Disable listening to this fd for long enough
                         * that the buffer can fill up again.
                         */
                        GPSD_LOG(LOG_DATA, &device->context->errout,
                                 "CORE: %s will be repolled in %f seconds\n",
                                 device->gpsdata.dev.path, reawake_time);
                        device->reawake = time(NULL) + reawake_time;
                        return DEVICE_UNREADY;
                    }
                }
                /*
                 * No data on later fragment reads just means the
                 * input buffer is empty.  In this case break out
                 * of the fragment-processing loop but consider
                 * the device still good.
                 */
                break;
            }

            // we got actual data, head off the reawake special case
            device->zerokill = false;
            device->reawake = (time_t)0;

            // must have a full packet to continue
            if (0 == (changed & PACKET_SET)) {
                break;
            }

            // conditional prevents mask dumper from eating CPU
            if (LOG_DATA <= device->context->errout.debug) {
                if (BAD_PACKET == device->lexer.type) {
                    GPSD_LOG(LOG_DATA, &device->context->errout,
                             "CORE: packet with bad checksum from %s\n",
                             device->gpsdata.dev.path);
                } else {
                    GPSD_LOG(LOG_DATA, &device->context->errout,
                             "CORE: packet type %d from %s with %s\n",
                             device->lexer.type,
                             device->gpsdata.dev.path,
                             gps_maskdump(device->gpsdata.set));
                }
            }


            // handle data contained in this packet
            if (BAD_PACKET != device->lexer.type) {
                handler(device, changed);
            }

#ifdef __future__
            // this breaks: test/daemon/passthrough.log ??
            /*
             * Bernd Ocklin suggests:
             * Exit when a full packet was received and parsed.
             * This allows other devices to be serviced even if
             * this device delivers a full packet at every single
             * read.
             * Otherwise we can sit here for a long time without
             * any for-loop exit condition being met.
             * It might also reduce the latency from a received packet to
             * it being output by gpsd.
             */
            if (0 != (changed & PACKET_SET)) {
               break;
            }
#endif  // __future__
        }
    } else if (0 < device->reawake &&
               time(NULL) > device->reawake) {
        // FIXME: what if time went backward?
        // device may have had a zero-length read
        GPSD_LOG(LOG_DATA, &device->context->errout,
                 "CORE: %s reawakened after zero-length read\n",
                 device->gpsdata.dev.path);
        device->reawake = (time_t)0;
        device->zerokill = true;
        return DEVICE_READY;
    } else if (SERVICE_NTRIP == device->servicetype &&
               NTRIP_CONN_INPROGRESS == device->ntrip.conn_state) {

            timespec_t ts_now;
            double step;

            (void)clock_gettime(CLOCK_REALTIME, &ts_now);

            step = TS_SUB_D(&ts_now, &device->ntrip.stream.stream_time);
            // wait 6 seconds between hitting ntrip_open()
            if (6 > fabs(step)) {
                return DEVICE_UNCHANGED;
            }
            device->ntrip.stream.stream_time = ts_now;
            (void)ntrip_open(device, "");
            if (NTRIP_CONN_ERR == device->ntrip.conn_state) {
                GPSD_LOG(LOG_WARN, &device->context->errout,
                         "CORE: 2 connection to ntrip server failed\n");
                // FIXME: next stat after error should depend on if
                // initial connect or reconnect...
                device->ntrip.conn_state = NTRIP_CONN_CLOSED;
                return DEVICE_ERROR;
            }
            //  else
            return DEVICE_READY;
        }

    // no change in device descriptor state
    return DEVICE_UNCHANGED;
}

// end-of-session wrapup
void gpsd_wrap(struct gps_device_t *session)
{
    if (!BAD_SOCKET(session->gpsdata.gps_fd)) {
        gpsd_deactivate(session);
    }
}

/* gpsd_zero_satellites(), initialize the skyview (satellite_t)
 *
 * return; void
 */
void gpsd_zero_satellites( struct gps_data_t *out)
{
    unsigned long  sat;

    memset(out->skyview, 0, sizeof(out->skyview));
    out->satellites_visible = 0;
    // zero is good inbound data for ss, elevation, and azimuth.
    // we need to set them to invalid values
    for (sat = 0; sat < ROWS(out->skyview); sat++) {
        out->skyview[sat].azimuth = NAN;
        out->skyview[sat].elevation = NAN;
        out->skyview[sat].ss = NAN;
        out->skyview[sat].prRes = NAN;
        out->skyview[sat].prRate = NAN;
        out->skyview[sat].pr = NAN;
        out->skyview[sat].freqid = -1;
        out->skyview[sat].qualityInd = -1;
    }
#if 0
    /*
     * We used to clear DOPs here, but this causes misbehavior on some
     * combined GPS/GLONASS/QZSS receivers like the Telit SL869; the
     * symptom is that the "satellites_used" field in a struct gps_data_t
     * filled in by gps_read() is always zero.
     */
    gps_clear_dop(&out->dop);
#endif
}

/* Latch the fact that we've saved a fix.
 * And add in the device fudge */
void ntp_latch(struct gps_device_t *device, struct timedelta_t *td)
{
    // this should be an invariant of the way this function is called
    if (0 >= device->newdata.time.tv_sec) {
        return;
    }

    (void)clock_gettime(CLOCK_REALTIME, &td->clock);
    // structure copy of time from GPS
    td->real = device->newdata.time;

    // is there an offset method?
    if (NULL != device->device_type &&
        NULL != device->device_type->time_offset) {
        double integral;
        double offset = device->device_type->time_offset(device);

        // add in offset which is double
        td->real.tv_nsec += (long)(modf(offset, &integral) * 1e9);
        td->real.tv_sec += (time_t)integral;
        TS_NORM(&td->real);
    }

    // thread-safe update
    pps_thread_fixin(&device->pps_thread, td);
}

// end
// vim: set expandtab shiftwidth=4
