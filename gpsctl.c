/* gpsctl.c -- tweak the control settings on a GPS
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "include/gpsd_config.h"  // must be before all includes

#include <assert.h>
#include <errno.h>
#ifdef HAVE_GETOPT_LONG
   #include <getopt.h>
#endif
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>       // for strlcat() and strlcpy()
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>       // for _exit()

#include "include/gpsd.h"

#ifdef SHM_EXPORT_ENABLE
#include <sys/ipc.h>
#include <sys/shm.h>
#endif  // SHM_EXPORT_ENABLE

#define HIGH_LEVEL_TIMEOUT 8

static int debuglevel;
static bool explicit_timeout = false;
static unsigned int timeout = 0;        // no timeout
static struct gps_context_t context;
static bool hunting = true;

/*
 * Set this as high or higher than the maximum number of subtype
 * probes in drivers.c.
 */
#define REDIRECT_SNIFF 15

// allow the device to settle after a control operation
static void settle(struct gps_device_t *session)
{
    struct timespec delay;

    /*
     * See the 'deep black magic' comment in serial.c:set_serial().
     */
    (void)tcdrain(session->gpsdata.gps_fd);

    // wait 50,000 uSec
    delay.tv_sec = 0;
    delay.tv_nsec = 50000000L;
    nanosleep(&delay, NULL);

    (void)tcdrain(session->gpsdata.gps_fd);
}

/*
 * Allows any response other than ERROR.  Use it for queries where a
 * failure return (due to, for example, a missing driver method) is
 * immediate, but successful responses have unpredictable lag.
 */
#define NON_ERROR 0       // must be distinct from any gps_mask_t value

// ship a command and wait on an expected response type
static bool gps_query(struct gps_data_t *gpsdata,
                      gps_mask_t expect,
                      const int timeout,
                      const char *fmt, ... )
{
    static fd_set rfds;
    char buf[BUFSIZ];
    va_list ap;
    time_t starttime;
    struct timespec tv;
    sigset_t oldset, blockset;

    (void)sigemptyset(&blockset);
    (void)sigaddset(&blockset, SIGHUP);
    (void)sigaddset(&blockset, SIGINT);
    (void)sigaddset(&blockset, SIGTERM);
    (void)sigaddset(&blockset, SIGQUIT);
    (void)sigprocmask(SIG_BLOCK, &blockset, &oldset);

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf)-2, fmt, ap);
    va_end(ap);
    // codacy does not like strlen()
    if ('\n' != buf[strnlen(buf, sizeof(buf) - 1) - 1]) {
        (void)strlcat(buf, "\n", sizeof(buf));
    }
    if (0 <= write(gpsdata->gps_fd, buf, strnlen(buf, sizeof(buf)))) {
        GPSD_LOG(LOG_ERROR, &context.errout, "gps_query(), write failed\n");
        return false;
    }
    GPSD_LOG(LOG_PROG, &context.errout, "gps_query(), wrote, %s\n", buf);

    FD_ZERO(&rfds);
    starttime = time(NULL);
    for (;;) {
        FD_CLR(gpsdata->gps_fd, &rfds);

        GPSD_LOG(LOG_PROG, &context.errout, "waiting...\n");

        tv.tv_sec = 2;
        tv.tv_nsec = 0;
        // (socket_t) to pacify codacy.
        if (-1 == pselect((socket_t)gpsdata->gps_fd + 1, &rfds, NULL, NULL,
                          &tv, &oldset)) {
            if (EINTR == errno ||
                !FD_ISSET(gpsdata->gps_fd, &rfds)) {
                continue;
            }
            GPSD_LOG(LOG_ERROR, &context.errout, "select %s(%d)\n",
                     strerror(errno), errno);
            exit(EXIT_FAILURE);
        }

        GPSD_LOG(LOG_PROG, &context.errout, "reading...\n");

        (void)gps_read(gpsdata, NULL, 0);
        if (ERROR_SET & gpsdata->set) {
            GPSD_LOG(LOG_ERROR, &context.errout, "error '%s'\n",
                     gpsdata->error);
            return false;
        }

        if ((NON_ERROR == expect) ||
            0 != (expect & gpsdata->set)) {
            return true;
        } else if (0 < timeout &&
                   (time(NULL) - starttime > timeout)) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "timed out after %d seconds\n",
                     timeout);
            return false;
        }
    }

    return false;
}

static void onsig(int sig)
{
    // CWE-479: Signal Handler Use of a Non-reentrant Function
    // See: The C Standard, 7.14.1.1, paragraph 5 [ISO/IEC 9899:2011]
    // Can't log in a signal handler.  Can't even call exit().
    if (sig == SIGALRM) {
        _exit(EXIT_FAILURE);
    } else {
        _exit(EXIT_SUCCESS);
    }
}

// full ID of the device for reports, including subtype
static char *gpsd_id(struct gps_device_t *session)
{
    static char buf[128];
    if (NULL == session ||
        NULL == session->device_type ||
        NULL == session->device_type->type_name) {
        return "unknown,";
    }
    (void)strlcpy(buf, session->device_type->type_name, sizeof(buf));
    if ('\0' != session->subtype[0]) {
        (void)strlcat(buf, " ", sizeof(buf));
        (void)strlcat(buf, session->subtype, sizeof(buf));
    }
    return (buf);
}

// recognize when we've achieved sync
static void ctlhook(struct gps_device_t *device UNUSED,
                    gps_mask_t changed UNUSED)
{
    static int packet_counter = 0;

    /*
     * If it's NMEA, go back around enough times for the type probes to
     * reveal any secret identity (like SiRF or UBX) the chip might have.
     * If it's not, getting more packets might fetch subtype information.
     */
    if (REDIRECT_SNIFF <= packet_counter++) {
        hunting = false;
        (void)alarm(0);
    }
}

static void usage(void)
{
    (void)printf("usage: gpsctl [OPTIONS] [device]\n\n"
#ifdef HAVE_GETOPT_LONG
         "  --binary            Switch device to native binary mode.\n"
         "  --debug DEBUGLEVEL  Set debug level to DEBUGLEVEL.\n"
         "  --direct            Force direct access to the device.\n"
         "  --echo              Echo specified control string with wrapper.\n"
         "  --help              Show this help, then exit\n"
         "  --list              List known device types and exit.\n"
         "  --nmea              Switch device to NMEA mode.\n"
         "  --rate RATE         Change receiver cyclte time to RATE.\n"
         "  --reset             Force reset to default mode.\n"
#ifdef SHM_EXPORT_ENABLE
         "  --rmshm             Remove the SHM export segment and exit.\n"
#endif   // SHM_EXPORT_ENABLE
         "  --ship CONTROL      Ship specified control string.\n"
         "  --speed SPEED       Set device speed to SPEED.\n"
         "  --timeout TIMEOUT   Set the timeout on packet recognition.\n"
         "  --type DEVTYPE      Force the device type.\n"
         "  --version           Show version, then exit\n"
#endif   // HAVE_GETOPT_LONG
         "  -?                  Show this help, then exit\n"
         "  -b                  Switch device to native binary mode.\n"
         "  -c RATE             Change receiver cyclte time to RATE.\n"
         "  -D DEBUGLEVEL       Set debug level to DEBUGLEVEL.\n"
         "  -e                  Echo specified control string with wrapper.\n"
         "  -f                  Force direct access to the device.\n"
         "  -h                  Show this help, then exit\n"
         "  -l                  List known device types and exit.\n"
         "  -n                  Switch device to NMEA mode.\n"
#ifdef SHM_EXPORT_ENABLE
         "  -R                  Remove the SHM export segment and exit.\n"
#endif   // SHM_EXPORT_ENABLE
         "  -r                  Force reset to default mode.\n"
         "  -s SPEED            Set device speed to SPEED.\n"
         "  -t DEVTYPE          Force the device type.\n"
         "  -T TIMEOUT          Set the timeout on packet recognition.\n"
         "  -V                  Show version, then exit\n"
         "  -x CONTROL          Ship specified control string.\n");
}

int main(int argc, char **argv)
{
    int ch, status;
    char *device = NULL, *devtype = NULL;
    char *speed = NULL, *control = NULL, *rate = NULL;
    bool to_binary = false, to_nmea = false, reset = false;
    bool control_stdout = false;
    bool lowlevel=false, echo=false;
    struct gps_data_t gpsdata;
    const struct gps_type_t *forcetype = NULL;
    const struct gps_type_t **dp;
    char cooked[BUFSIZ];
    ssize_t cooklen = 0;
    const char *optstring = "?bec:fhlnrs:t:x:D:RT:V";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"binary", no_argument, NULL, 'b'},
        {"debug", required_argument, NULL, 'D'},
        {"direct", no_argument, NULL, 'f'},
        {"echo", no_argument, NULL, 'e'},
        {"help", no_argument, NULL, 'h'},
        {"list", no_argument, NULL, 'l'},
        {"nmea", no_argument, NULL, 'n'},
        {"rate", required_argument, NULL, 'c'},
#ifdef SHM_EXPORT_ENABLE
        {"rmshm", required_argument, NULL, 'R'},
#endif
        {"ship", required_argument, NULL, 'x'},
        {"speed", required_argument, NULL, 's'},
        {"timeout", required_argument, NULL, 'T'},
        {"type", required_argument, NULL, 't'},
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif

    // We need this before any logging happens (for report_mutex)
    gps_context_init(&context, "gpsctl");

    while (1) {
#ifdef HAVE_GETOPT_LONG
        ch = getopt_long(argc, argv, optstring, long_options, &option_index);
#else
        ch = getopt(argc, argv, optstring);
#endif

        if (-1 == ch) {
            break;
        }

        switch (ch) {
        case 'b':               // switch to vendor binary mode
            to_binary = true;
            break;
        case 'c':
            rate = optarg;
            break;
        case 'D':               // set debugging level
            debuglevel = atoi(optarg);
            gps_enable_debug(debuglevel, stderr);
            break;
        case 'e':               // echo specified control string with wrapper
            lowlevel = true;
            control_stdout = true;  /* Prevent message going to stdout */
            echo = true;
            break;
        case 'f':               // force direct access to the device
            lowlevel = true;
            break;
        case 'l':               // list known device types
            for (dp = gpsd_drivers; *dp; dp++) {
                if ((*dp)->mode_switcher != NULL)
                    (void)fputs("-[bn]\t", stdout);
                else
                    (void)fputc('\t', stdout);
                if ((*dp)->speed_switcher != NULL)
                    (void)fputs("-s\t", stdout);
                else
                    (void)fputc('\t', stdout);
                if ((*dp)->rate_switcher != NULL)
                    (void)fputs("-c\t", stdout);
                else
                    (void)fputc('\t', stdout);
                if ((*dp)->control_send != NULL)
                    (void)fputs("-x\t", stdout);
                else
                    (void)fputc('\t', stdout);
                (void)puts((*dp)->type_name);
            }
            exit(EXIT_SUCCESS);
        case 'n':               /* switch to NMEA mode */
            to_nmea = true;
            break;
#ifdef SHM_EXPORT_ENABLE
        case 'R':               /* remove the SHM export segment */
            status = shmget(getenv("GPSD_SHM_KEY") ?
                            (key_t)strtol(getenv("GPSD_SHM_KEY"), NULL, 0) :
                            (key_t)GPSD_SHM_KEY, 0, 0);
            if (status == -1) {
                GPSD_LOG(LOG_WARN, &context.errout,
                         "GPSD SHM segment does not exist.\n");
                exit(1);
            } else {
                status = shmctl(status, IPC_RMID, NULL);
                if (status == -1) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "shmctl failed, errno = %d (%s)\n",
                             errno, strerror(errno));
                    exit(1);
                }
            }
            exit(0);
#endif /* SHM_EXPORT_ENABLE */
        case 'r':               /* force-switch to default mode */
            reset = true;
            lowlevel = false;   /* so we'll abort if the daemon is running */
            break;
        case 's':               /* change output baud rate */
            speed = optarg;
            break;
        case 'T':               /* set the timeout on packet recognition */
            timeout = (unsigned)atoi(optarg);
            explicit_timeout = true;
            break;
        case 't':               /* force the device type */
            devtype = optarg;
            /* experimental kluge */
            if (strcmp(devtype, "u-blox") == 0)
                timeout = 2;
            break;
        case 'x':               /* ship specified control string */
            control = optarg;
            lowlevel = true;
            if ((cooklen = hex_escapes(cooked, control)) <= 0) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "invalid escape string (error %d)\n", (int)cooklen);
                exit(EXIT_FAILURE);
            }
            break;
        case 'V':
            (void)fprintf(stderr, "%s: version %s (revision %s)\n",
                          argv[0], VERSION, REVISION);
            exit(EXIT_SUCCESS);
        case '?':
            FALLTHROUGH
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
        default:
            usage();
            exit(EXIT_FAILURE);
        }
    }

    if (optind < argc)
        device = argv[optind];

    if (devtype != NULL) {
        int matchcount = 0;
        for (dp = gpsd_drivers; *dp; dp++) {
            if (strstr((*dp)->type_name, devtype) != NULL) {
                forcetype = *dp;
                matchcount++;
            }
        }
        if (matchcount == 0)
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "no driver type name matches '%s'.\n", devtype);
        else if (matchcount == 1) {
            assert(forcetype != NULL);
            GPSD_LOG( LOG_PROG,&context.errout,
                     "%s driver selected.\n", forcetype->type_name);
        } else {
            forcetype = NULL;
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "%d driver type names match '%s'.\n",
                     matchcount, devtype);
        }
    }

    if (((int)to_nmea + (int)to_binary + (int)reset) > 1) {
        GPSD_LOG(LOG_ERROR, &context.errout, "make up your mind, would you?\n");
        exit(EXIT_SUCCESS);
    }

    (void) signal(SIGINT, onsig);
    (void) signal(SIGTERM, onsig);
    (void) signal(SIGQUIT, onsig);

    if (!lowlevel) {
        /* Try to open the stream to gpsd. */
        if (gps_open(NULL, NULL, &gpsdata) != 0) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "no gpsd running or network error: %s.\n",
                     gps_errstr(errno));
            lowlevel = true;
        }
    }

    if (!lowlevel) {
        int i, devcount;

        if (!explicit_timeout)
            timeout = HIGH_LEVEL_TIMEOUT;

        /* what devices have we available? */
        if (!gps_query(&gpsdata, DEVICELIST_SET, (int)timeout,
                       "?DEVICES;\r\n")) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "no DEVICES response received.\n");
            (void)gps_close(&gpsdata);
            exit(EXIT_FAILURE);
        }
        if (gpsdata.devices.ndevices == 0) {
            GPSD_LOG(LOG_ERROR, &context.errout, "no devices connected.\n");
            (void)gps_close(&gpsdata);
            exit(EXIT_FAILURE);
        } else if (gpsdata.devices.ndevices > 1 && device == NULL) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "multiple devices and no device specified.\n");
            (void)gps_close(&gpsdata);
            exit(EXIT_FAILURE);
        }
        GPSD_LOG(LOG_PROG, &context.errout,
                 "%d device(s) found.\n",gpsdata.devices.ndevices);

        /* try to mine the devicelist return for the data we want */
        if (gpsdata.devices.ndevices == 1 && device == NULL) {
            device = gpsdata.dev.path;
            i = 0;
        } else {
            assert(device != NULL);
            for (i = 0; i < gpsdata.devices.ndevices; i++)
                if (strcmp(device, gpsdata.devices.list[i].path) == 0) {
                    goto devicelist_entry_matches;
                }
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "specified device not found in device list.\n");
            (void)gps_close(&gpsdata);
            exit(EXIT_FAILURE);
        devicelist_entry_matches:;
        }
        gpsdata.dev = gpsdata.devices.list[i];
        devcount = gpsdata.devices.ndevices;

        /* if the device has not identified, watch it until it does so */
        if (gpsdata.dev.driver[0] == '\0') {
            if (gps_stream(&gpsdata, WATCH_ENABLE|WATCH_JSON, NULL) == -1) {
                GPSD_LOG(LOG_ERROR, &context.errout, "stream set failed.\n");
                (void)gps_close(&gpsdata);
                exit(EXIT_FAILURE);
            }

            while (devcount > 0) {
                /* Wait for input data */
                if (!gps_waiting(&gpsdata, timeout * 1000000)) {
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "timed out waiting for device\n");
                        (void)gps_close(&gpsdata);
                        exit(EXIT_FAILURE);
                }
                errno = 0;
                if (gps_read(&gpsdata, NULL, 0) == -1) {
                    GPSD_LOG(LOG_ERROR, &context.errout, "data read failed.\n");
                    (void)gps_close(&gpsdata);
                    exit(EXIT_FAILURE);
                }

                if (gpsdata.set & DEVICE_SET) {
                    --devcount;
                    assert(gpsdata.dev.path[0]!='\0' &&
                           gpsdata.dev.driver[0]!='\0');
                    if (strcmp(gpsdata.dev.path, device) == 0) {
                        goto matching_device_seen;
                    }
                }
            }
            GPSD_LOG(LOG_ERROR, &context.errout, "data read failed.\n");
            (void)gps_close(&gpsdata);
            exit(EXIT_FAILURE);
        matching_device_seen:;
        }

        /* sanity check */
        if (gpsdata.dev.driver[0] == '\0') {
            GPSD_LOG(LOG_SHOUT, &context.errout,
                     "%s can't be identified.\n",
                     gpsdata.dev.path);
            (void)gps_close(&gpsdata);
            exit(EXIT_SUCCESS);
        }

        /* if no control operation was specified, just ID the device */
        if (speed==NULL && rate == NULL && !to_nmea && !to_binary && !reset) {
            (void)printf("%s identified as a %s",
                         gpsdata.dev.path, gpsdata.dev.driver);
            if (gpsdata.dev.subtype[0] != '\0') {
                (void)fputc(' ', stdout);
                (void)fputs(gpsdata.dev.subtype, stdout);
            }
            if (gpsdata.dev.baudrate > 0)
                (void)printf(" at %u baud", gpsdata.dev.baudrate);
            (void)fputc('.', stdout);
            (void)fputc('\n', stdout);
        }

        status = 0;
        if (reset)
        {
            GPSD_LOG(LOG_PROG, &context.errout,
                     "cannot reset with gpsd running.\n");
            exit(EXIT_SUCCESS);
        }

        /*
         * We used to wait on DEVICE_SET here.  That doesn't work
         * anymore because when the demon generates its response it
         * sets the mode bit in the response from the current packet
         * type, which may not have changed (probably will not have
         * changed) even though the command to switch modes has been
         * sent and will shortly take effect.
         */
        if (to_nmea) {
            if (!gps_query(&gpsdata, NON_ERROR, (int)timeout,
                           "?DEVICE={\"path\":\"%s\",\"native\":0}\r\n",
                           device)) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "%s mode change to NMEA failed\n",
                         gpsdata.dev.path);
                status = 1;
            } else
                GPSD_LOG(LOG_PROG, &context.errout,
                         "%s mode change succeeded\n", gpsdata.dev.path);
        }
        else if (to_binary) {
            if (!gps_query(&gpsdata, NON_ERROR, (int)timeout,
                           "?DEVICE={\"path\":\"%s\",\"native\":1}\r\n",
                           device)) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "%s mode change to native mode failed\n",
                         gpsdata.dev.path);
                status = 1;
            } else
                GPSD_LOG(LOG_PROG, &context.errout,
                         "%s mode change succeeded\n",
                         gpsdata.dev.path);
        }
        if (speed != NULL) {
            char parity = 'N';
            char stopbits = '1';
            if (strchr(speed, ':') == NULL)
                (void)gps_query(&gpsdata,
                                DEVICE_SET, (int)timeout,
                                "?DEVICE={\"path\":\"%s\",\"bps\":%s}\r\n",
                                 device, speed);
            else {
                char *modespec = strchr(speed, ':');
                status = 0;
                if (modespec!=NULL) {
                    *modespec = '\0';
                    if (strchr("78", *++modespec) == NULL) {
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "No support for that word length.\n");
                        status = 1;
                    }
                    parity = *++modespec;
                    if (strchr("NOE", parity) == NULL) {
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "What parity is '%c'?\n", parity);
                        status = 1;
                    }
                    stopbits = *++modespec;
                    if (strchr("12", stopbits) == NULL) {
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "Stop bits must be 1 or 2.\n");
                        status = 1;
                    }
                }
                if (0 == status)
                    (void)gps_query(&gpsdata,
                                    DEVICE_SET, (int)timeout,
                                     "?DEVICE={\"path\":\"%s\",\"bps\":%s,"
                                     "\"parity\":\"%c\",\"stopbits\":%c}\r\n",
                                     device, speed, parity, stopbits);
            }
            if (atoi(speed) != (int)gpsdata.dev.baudrate) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "%s driver won't support %s%c%c\n",
                         gpsdata.dev.path,
                         speed, parity, stopbits);
                status = 1;
            } else
                GPSD_LOG(LOG_PROG, &context.errout,
                         "%s change to %s%c%c succeeded\n",
                         gpsdata.dev.path,
                         speed, parity, stopbits);
        }
        if (NULL != rate) {
            (void)gps_query(&gpsdata,
                            DEVICE_SET, (int)timeout,
                            "?DEVICE={\"path\":\"%s\",\"cycle\":%s}\r\n",
                            device, rate);
        }
        (void)gps_close(&gpsdata);
        exit(status);
    } else if (reset) {
        // hard reset will go through lower-level operations
        // FIXME: missing speeds, should come from a header.
        const int speeds[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400,
                              460800, 921600};
        static struct gps_device_t      session;        // zero this too
        int i;

        if (NULL == device ||
            NULL == forcetype) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "device and type must be specified for the "
                         "reset operation.\n");
                exit(EXIT_FAILURE);
            }

        context.errout.debug = debuglevel;
        session.context = &context;
        gpsd_tty_init(&session);
        (void)strlcpy(session.gpsdata.dev.path, device,
                      sizeof(session.gpsdata.dev.path));
        session.device_type = forcetype;
        (void)gpsd_open(&session);
        (void)gpsd_set_raw(&session);
        (void)session.device_type->speed_switcher(&session, 4800, 'N', 1);
        (void)tcdrain(session.gpsdata.gps_fd);
        for(i = 0; i < (int)(sizeof(speeds) / sizeof(speeds[0])); i++) {
            (void)gpsd_set_speed(&session, speeds[i], 'N', 1);
            (void)session.device_type->speed_switcher(&session, 4800, 'N', 1);
            (void)tcdrain(session.gpsdata.gps_fd);
        }
        gpsd_set_speed(&session, 4800, 'N', 1);
        for (i = 0; i < 3; i++) {
            if (session.device_type->mode_switcher) {
                session.device_type->mode_switcher(&session, MODE_NMEA);
            }
        }
        gpsd_wrap(&session);
        exit(EXIT_SUCCESS);
    } else {
        // access to the daemon failed, use the low-level facilities
        static struct gps_device_t      session;        // zero this too
        fd_set all_fds;
        fd_set rfds;

        /*
         * Unless the user explicitly requested it, always run to end of
         * hunt rather than timing out. Otherwise we can easily get messages
         * that spuriously look like failure at high baud rates.
         */

        gps_context_init(&context, "gpsctl");
        context.errout.debug = debuglevel;
        session.context = &context;     // in case gps_init isn't called

        if (echo) {
            context.readonly = true;
        }

        if (timeout > 0) {
            (void) alarm(timeout);
            (void) signal(SIGALRM, onsig);
        }
        /*
         * Unless the user has forced a type and only wants to see the
         * string (not send it) we now need to try to open the device
         * and find out what is actually there.
         */
        if (NULL == forcetype ||
            !echo) {
            socket_t maxfd = 0;
            int activated = -1;

            if (device == NULL) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "device must be specified for low-level access.\n");
                exit(EXIT_FAILURE);
            }

            gpsd_init(&session, &context, device);
            activated = gpsd_activate(&session, O_PROBEONLY);
            if ( 0 > activated ) {
                if ( PLACEHOLDING_FD == activated ) {
                    (void)printf("%s identified as a %s.\n",
                       device, gpsd_id(&session));
                    exit(EXIT_SUCCESS);
                }
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "initial GPS device %s open failed\n",
                         device);
                exit(EXIT_FAILURE);
            }
            GPSD_LOG(LOG_INF, &context.errout,
                     "device %s activated\n", session.gpsdata.dev.path);
            FD_SET(session.gpsdata.gps_fd, &all_fds);
            if (session.gpsdata.gps_fd > maxfd) {
                 maxfd = session.gpsdata.gps_fd;
            }

            // initialize the GPS context's time fields
            gpsd_time_init(&context, time(NULL));

            // grab packets until we time out, get sync, or fail sync
            for (hunting = true; hunting; ) {
                fd_set efds;
                timespec_t ts_timeout = {2, 0};   // timeout for pselect()
                switch(gpsd_await_data(&rfds, &efds, maxfd, &all_fds,
                                       &context.errout, ts_timeout)) {
                case AWAIT_GOT_INPUT:
                    FALLTHROUGH
                case AWAIT_TIMEOUT:
                    break;
                case AWAIT_NOT_READY:
                    /* no recovery from bad fd is possible */
                    if (FD_ISSET(session.gpsdata.gps_fd, &efds))
                        exit(EXIT_FAILURE);
                    continue;
                case AWAIT_FAILED:
                    exit(EXIT_FAILURE);
                }

                switch(gpsd_multipoll(FD_ISSET(session.gpsdata.gps_fd, &rfds),
                                               &session, ctlhook, 0))
                {
                case DEVICE_READY:
                    FD_SET(session.gpsdata.gps_fd, &all_fds);
                    break;
                case DEVICE_UNREADY:
                    FD_CLR(session.gpsdata.gps_fd, &all_fds);
                    break;
                case DEVICE_ERROR:
                    /* this is where a failure to sync lands */
                    GPSD_LOG(LOG_WARN, &context.errout,
                             "device error, bailing out.\n");
                    exit(EXIT_FAILURE);
                case DEVICE_EOF:
                    GPSD_LOG(LOG_WARN, &context.errout,
                             "device signed off, bailing out.\n");
                    exit(EXIT_SUCCESS);
                default:
                    break;
                }
            }

            GPSD_LOG(LOG_PROG, &context.errout,
                     "%s looks like a %s at %d.\n",
                     device, gpsd_id(&session),
                     session.gpsdata.dev.baudrate);

            if (forcetype!=NULL &&
                strcmp("NMEA0183", session.device_type->type_name) != 0 &&
                strcmp(forcetype->type_name,
                       session.device_type->type_name) != 0) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "'%s' doesn't match non-generic type '%s' "
                         "of selected device.\n",
                         forcetype->type_name,
                         session.device_type->type_name);
            }
        }

        if(!control_stdout)
            (void)printf("%s identified as a %s at %u baud.\n",
                         device, gpsd_id(&session),
                         session.gpsdata.dev.baudrate);

        /* if no control operation was specified, we're done */
        if (speed==NULL && !to_nmea && !to_binary && control==NULL)
            exit(EXIT_SUCCESS);

        /* maybe user wants to see the packet rather than send it */
        if (echo)
            session.gpsdata.gps_fd = fileno(stdout);

        /* control op specified; maybe we forced the type */
        if (forcetype != NULL)
            (void)gpsd_switch_driver(&session, forcetype->type_name);

        /* now perform the actual control function */
        status = 0;
        if (to_nmea || to_binary) {
            bool write_enable = context.readonly;
            context.readonly = false;
            if (session.device_type->mode_switcher == NULL) {
                GPSD_LOG(LOG_SHOUT, &context.errout,
                         "%s devices have no mode switch.\n",
                         session.device_type->type_name);
                status = 1;
            } else {
                int target_mode = to_nmea ? MODE_NMEA : MODE_BINARY;

                GPSD_LOG(LOG_SHOUT, &context.errout,
                         "switching to mode %s.\n",
                         to_nmea ? "NMEA" : "BINARY");
                session.device_type->mode_switcher(&session, target_mode);
                settle(&session);
            }
            context.readonly = write_enable;
        }
        if (speed) {
            char parity = echo ? 'N': session.gpsdata.dev.parity;
            int stopbits = echo ? 1 : session.gpsdata.dev.stopbits;
            char *modespec;

            modespec = strchr(speed, ':');
            status = 0;
            if (modespec!=NULL) {
                *modespec = '\0';
                if (strchr("78", *++modespec) == NULL) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "No support for that word lengths.\n");
                    status = 1;
                }
                parity = *++modespec;
                if (strchr("NOE", parity) == NULL) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "What parity is '%c'?\n", parity);
                    status = 1;
                }
                stopbits = *++modespec;
                if (strchr("12", parity) == NULL) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "Stop bits must be 1 or 2.\n");
                    status = 1;
                }
                stopbits = (int)(stopbits-'0');
            }
            if (status == 0) {
                if (session.device_type->speed_switcher == NULL) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "%s devices have no speed switch.\n",
                             session.device_type->type_name);
                    status = 1;
                }
                else if (session.device_type->speed_switcher(&session,
                                                           (speed_t)atoi(speed),
                                                           parity,
                                                           stopbits)) {
                    settle(&session);
                    GPSD_LOG(LOG_PROG, &context.errout,
                             "%s change to %s%c%d succeeded\n",
                             session.gpsdata.dev.path,
                             speed, parity, stopbits);
                } else {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "%s driver won't support %s%c%d.\n",
                             session.gpsdata.dev.path,
                             speed, parity, stopbits);
                    status = 1;
                }
            }
        }
        if (rate) {
            bool write_enable = context.readonly;
            context.readonly = false;
            if (session.device_type->rate_switcher == NULL) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "%s devices have no rate switcher.\n",
                         session.device_type->type_name);
                status = 1;
            } else {
                double rate_dbl = strtod(rate, NULL);

                if (!session.device_type->rate_switcher(&session, rate_dbl)) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "rate switch failed.\n");
                    status = 1;
                }
                settle(&session);
            }
            context.readonly = write_enable;
        }
        if (control) {
            bool write_enable = context.readonly;
            context.readonly = false;
            if (session.device_type->control_send == NULL) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "%s devices have no control sender.\n",
                         session.device_type->type_name);
                status = 1;
            } else {
                if (session.device_type->control_send(&session,
                                                      cooked,
                                                      (size_t)cooklen) == -1) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "control transmission failed.\n");
                    status = 1;
                }
                settle(&session);
            }
            context.readonly = write_enable;
        }

        exit(status);
    }
}

// end
// vim: set expandtab shiftwidth=4
