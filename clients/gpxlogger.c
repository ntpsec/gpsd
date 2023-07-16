/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <assert.h>
#include <errno.h>
#include <libgen.h>
#include <limits.h>          // for PATH_MAX
#include <math.h>
#ifdef HAVE_GETOPT_LONG
   #include <getopt.h>       // for getopt_long()
#endif
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>          // for atexit()
#include <string.h>
#include <time.h>
#include <unistd.h>          // for _exit()

#include "../include/gps.h"
#include "../include/gpsdclient.h"
#include "../include/os_compat.h"
#include "../include/timespec.h"

static char *progname;
static struct fixsource_t source;

/**************************************************************************
 *
 * Transport-layer-independent functions
 *
 **************************************************************************/

static int debug;                         // debug level
// The garmin extensions are optional, because they cause Google maps to barf.
static bool garmin = false;               // enable garmin depth extension
static struct gps_data_t gpsdata;
static bool intrack = false;
static FILE *gpxlogfile = NULL;           // file to write gpx log to
static double minmove = 0;                // meters
static int sig_flag = 0;
static long timeout = 5;                  // seconds

static void print_gpx_header(void)
{
    char tbuf[CLIENT_DATE_MAX+1];

    (void)fprintf(gpxlogfile,
         "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
         "<gpx version=\"1.1\" creator=\"GPSD %s - %s\"\n"
         "  xmlns=\"http://www.topografix.com/GPX/1/1\"\n"
         "  xmlns:xsi=\"https://www.w3.org/2001/XMLSchema-instance\"\n",
         VERSION, GPSD_URL);

    if (garmin) {
        (void)fputs(
             "  xmlns:gpxx=\"http://www8.garmin.com/xmlschemas/"
             "GpxExtensions/v3\"\n"
             "  xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 "
             "http://www.topografix.com/GPX/1/1/gpx.xsd "
             "https://www8.garmin.com/xmlschemas/GpxExtensions/v3 "
             "https://www8.garmin.com/xmlschemas/GpxExtensions/v3/"
             "GpxExtensionsv3.xsd\"",
             gpxlogfile);
    } else {
        (void)fputs(
             "  xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1\n"
             "  http://www.topografix.com/GPX/1/1/gpx.xsd\"",
             gpxlogfile);
    }

    (void)fprintf(gpxlogfile,
         "\n>\n"
         " <metadata>\n"
         "  <time>%s</time>\n"
         " </metadata>\n",
         now_to_iso8601(tbuf, sizeof(tbuf)));
    (void)fflush(gpxlogfile);
}

static void print_gpx_trk_end(void)
{
    (void)fputs("  </trkseg>\n"
                " </trk>\n", gpxlogfile);
    (void)fflush(gpxlogfile);
}

static void print_gpx_footer(void)
{
    if (intrack) {
        print_gpx_trk_end();
    }
    (void)fputs("</gpx>\n", gpxlogfile);
    (void)fclose(gpxlogfile);
}

static void print_gpx_trk_start(void)
{
    (void)fputs(" <trk>\n"
                "  <src>GPSD " VERSION "</src>\n"
                "  <trkseg>\n", gpxlogfile);
    (void)fflush(gpxlogfile);
}

static void print_fix(struct gps_data_t *gpsdata, timespec_t ts_time)
{
    char tbuf[CLIENT_DATE_MAX + 1];

    (void)fprintf(gpxlogfile, "   <trkpt lat=\"%.9f\" lon=\"%.9f\">\n",
                  gpsdata->fix.latitude, gpsdata->fix.longitude);

    /*
     * From the specification at https://www.topografix.com/GPX/1/1/gpx.xsd
     * the <ele> tag is defined as "Elevation (in meters) of the point."
     * This is ambiguous between HAE and orthometric height (above geoid,
     * aka MSL).
     * gpsd has historically used HAE and MSL randomly for altitude.
     * gpsd now explicitly supports distinct HAE and MSL.
     */
    if (0 != isfinite(gpsdata->fix.altHAE)) {
        (void)fprintf(gpxlogfile,
             "    <ele>%.4f</ele>\n", gpsdata->fix.altHAE);
    }

    (void)fprintf(gpxlogfile, "    <time>%s</time>\n",
                  timespec_to_iso8601(ts_time, tbuf, sizeof(tbuf)));
    if (STATUS_DGPS == gpsdata->fix.status) {
        // FIXME: other status values?
        (void)fputs("    <fix>dgps</fix>\n", gpxlogfile);
    } else {
        switch (gpsdata->fix.mode) {
        case MODE_3D:
            (void)fputs("    <fix>3d</fix>\n", gpxlogfile);
            break;
        case MODE_2D:
            (void)fputs("    <fix>2d</fix>\n", gpxlogfile);
            break;
        case MODE_NO_FIX:
            (void)fputs("    <fix>none</fix>\n", gpxlogfile);
            break;
        default:
            // don't print anything if no fix indicator
            break;
        }
    }

    if (MODE_NO_FIX < gpsdata->fix.mode &&
        0 < gpsdata->satellites_used) {
        (void)fprintf(gpxlogfile,
             "    <sat>%d</sat>\n", gpsdata->satellites_used);
    }
    if (0 != isfinite(gpsdata->dop.hdop)) {
        (void)fprintf(gpxlogfile,
             "    <hdop>%.1f</hdop>\n", gpsdata->dop.hdop);
    }
    if (0 != isfinite(gpsdata->dop.vdop)) {
        (void)fprintf(gpxlogfile,
             "    <vdop>%.1f</vdop>\n", gpsdata->dop.vdop);
    }
    if (0 != isfinite(gpsdata->dop.pdop)) {
        (void)fprintf(gpxlogfile,
             "    <pdop>%.1f</pdop>\n", gpsdata->dop.pdop);
    }

    if (true == garmin &&
        0 != isfinite(gpsdata->fix.depth)) {
        // garmin extentions cause google maps to crash
        (void)fprintf(gpxlogfile,
             "    <extensions>\n"
             "       <gpxx:TrackPointExtension>\n"
             "           <gpxx:Depth>%.2f</gpxx:Depth>\n"
             "       </gpxx:TrackPointExtension>\n"
             "    </extensions>\n",
             gpsdata->fix.depth);
    }
    (void)fputs("   </trkpt>\n", gpxlogfile);
    (void)fflush(gpxlogfile);
}

// cleanup as an atexit() handler
static void cleanup(void)
{
        print_gpx_footer();
        (void)gps_close(&gpsdata);
        // don't clutter the logs on Ctrl-C
        if (0 != sig_flag &&
            SIGINT != sig_flag) {
            syslog(LOG_INFO, "exiting, signal %d received", sig_flag);
        }
}

/* called by gps_mainloop() to maybe log a fix.
 *
 * Return void
 */
static void conditionally_log_fix(struct gps_data_t *gpsdata)
{
    static timespec_t ts_time, old_ts_time, ts_diff;
    static double old_lat, old_lon;
    static bool first = true;

    if (0 != sig_flag) {
        if (SIGINT != sig_flag) {
            exit(EXIT_FAILURE);
        }
        exit(EXIT_SUCCESS);
    }

    // FIXME: check for good time?
    ts_time = gpsdata->fix.time;
    if (TS_EQ(&ts_time, &old_ts_time) ||
        MODE_2D > gpsdata->fix.mode) {
        return;
    }

    // may not be worth logging if we've moved only a very short distance
    if (0 < minmove &&
        !first &&
        earth_distance(gpsdata->fix.latitude, gpsdata->fix.longitude,
                       old_lat, old_lon) < minmove) {
        return;
    }

    /*
     * Make new track if the jump in time is above
     * timeout.  Handle jumps both forward and
     * backwards in time.  The clock sometimes jumps
     * backward when gpsd is submitting junk on the
     * dbus.
     */
    TS_SUB(&ts_diff, &ts_time, &old_ts_time);
    if (labs((long)ts_diff.tv_sec) > timeout &&
        !first) {
        print_gpx_trk_end();
        intrack = false;
    }

    if (!intrack) {
        print_gpx_trk_start();
        intrack = true;
        if (first) {
            first = false;
        }
    }

    old_ts_time = ts_time;
    if (0 < minmove) {
        old_lat = gpsdata->fix.latitude;
        old_lon = gpsdata->fix.longitude;
    }
    print_fix(gpsdata, ts_time);
}

static void quit_handler(int signum)
{
    // CWE-479: Signal Handler Use of a Non-reentrant Function
    // See: The C Standard, 7.14.1.1, paragraph 5 [ISO/IEC 9899:2011]
    // Can't log in a signal handler.  Can't even call exit().
    sig_flag = signum;
    return;
}

/**************************************************************************
 *
 * Main sequence
 *
 **************************************************************************/

static void usage(void)
{
    (void)fprintf(stderr,
         "Usage: %s [OPTIONS] [server[:port:[device]]]\n\n"
         "  -?                  Show this help, then exit\n"
#ifdef HAVE_GETOPT_LONG
         "  --daemonize         Daemonize\n"
         "  --debug LVL         Set debug level.\n"
         "  --export EXPORTMETHOD  Default %s\n"
         "  --exports           List available exports, then exit\n"
         "  --filein INFILE     Read from INFILE, not gpsd\n"
         "  --garmin            Enable Garmin depth output\n"
         "  --help              Show this help, then exit\n"
         "  --interval TIMEOUT  Create new track after TIMEOUT seconds. "
         "Default 5\n"
         "  --minmove MINMOVE   Minimum move in meters to log\n"
         "  --output OUTFILE    Send gpx output to file OUTFILE\n"
         "  --reconnect         Retry when gpsd loses the fix.\n"
         "  --version           Show version, then exit\n"
#endif
         "  -D LVL              Set debug level.\n"
         "  -d                  Daemonize\n"
         "  -e EXPORTMETHOD     Default %s \n"
         "  -f OUTFILE          Send gpx output to file OUTFILE\n"
         "  -F INFILE           Read *gpsd* JSON from INFILE, not gpsd\n"
         "  -g                  Enable Garmin depth output\n"
         "  -h                  Show this help, then exit\n"
         "  -i TIMEOUT          Create new track after TIMEOUT seconds. "
         "Default 5\n"
         "  -l                  List available exports, then exit\n"
         "  -m MINMOVE          Minimum move in meters to log\n"
         "  -r                  Retry when gpsd loses the fix.\n"
         "  -V                  Show version and exit\n",
         progname,
#ifdef HAVE_GETOPT_LONG
         export_default()->name,
#endif
         export_default()->name);
    exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
    int ch;
    bool daemonize = false;
    bool reconnect = false;
    unsigned int flags = WATCH_ENABLE;
    struct exportmethod_t *method = NULL;
    char   *file_in = NULL;
    const char *optstring = "?dD:e:f:F:ghi:lm:rV";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"daemonize", no_argument, NULL, 'd'},
        {"debug", required_argument, NULL, 'D'},
        {"export", required_argument, NULL, 'e'},
        {"exports", no_argument, NULL, 'l'},
        {"filein", required_argument, NULL, 'F' },
        {"garmin", no_argument, NULL, 'g'},
        {"help", no_argument, NULL, 'h'},
        {"interval", required_argument, NULL, 'i'},
        {"minmove", required_argument, NULL, 'm'},
        {"output", required_argument, NULL, 'f'},
        {"reconnect", no_argument, NULL, 'r' },
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif

    progname = argv[0];

    method = export_default();
    if (NULL == method) {
        (void)fprintf(stderr, "%s: no export methods.\n", progname);
        exit(EXIT_FAILURE);
    }

    gpxlogfile = stdout;
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
        case 'd':
            openlog(basename(progname), LOG_PID | LOG_PERROR, LOG_DAEMON);
            daemonize = true;
            break;
        case 'D':
            debug = atoi(optarg);
            gps_enable_debug(debug, gpxlogfile);
            break;
        case 'e':
            method = export_lookup(optarg);
            if (NULL == method) {
                (void)fprintf(stderr,
                              "%s: %s is not a known export method.\n",
                              progname, optarg);
                exit(EXIT_FAILURE);
            }
            break;
       case 'f':       // Output file name.
            {
                char   fname[PATH_MAX];
                time_t t = time(NULL);

                size_t s = strftime(fname, sizeof(fname) - 11, optarg,
                                    localtime(&t));
                fname[s] = '\0';      // paranoia
                if (0 < s) {
                    gpxlogfile = fopen(fname, "w");
                    if (NULL == gpxlogfile) {
                        syslog(LOG_ERR,
                               "Failed to open %s: %s, logging to stdout.",
                               fname, strerror(errno));
                        gpxlogfile = stdout;
                    }
                } else {
                    syslog(LOG_ERR, "strftime() failed, logging to stdout.");
                }
            }
            break;
        case 'F':       // input file name.
            file_in = strdup(optarg);
            break;
        case 'g':
            garmin = true;
            break;
        case 'i':               // set polling interval
            timeout = atoi(optarg);
            if (1 > timeout) {
                // set zero, and negative, timeouts to one.
                timeout = 1;
            } else if (3600 <= timeout) {
                (void)fputs("WARNING: track timeout is an hour or more!\n",
                            stderr);
            }
            break;
        case 'l':
            export_list(stderr);
            exit(EXIT_SUCCESS);
        case 'm':
            minmove = (double )atoi(optarg);
            break;
        case 'r':
            reconnect = true;
            break;
        case 'V':
            (void)fprintf(stderr, "%s: version %s (revision %s)\n",
                          progname, VERSION, REVISION);
            exit(EXIT_SUCCESS);
        case '?':
        case 'h':
        default:
            usage();
            // NOTREACHED
        }
    }

    if (daemonize &&
        stdout ==  gpxlogfile) {
        syslog(LOG_ERR,
            "Daemon mode with no valid gpxlogfile name - exiting.");
        exit(EXIT_FAILURE);
    }

    memset(&source, 0, sizeof(source));
    if (NULL != file_in) {
        // read from file, not a gpsd
        if (optind < argc) {
            (void)fprintf(stderr,
                "ERROR: local file and gpsd source both requested\n");
            exit(EXIT_FAILURE);
        }
        source.server = GPSD_LOCAL_FILE;
        source.port = file_in;
    } else if (NULL != method->magic) {
        source.server = method->magic;
    } else {
        source.server = (char *)"localhost";
        source.port = (char *)DEFAULT_GPSD_PORT;
    }

    if (optind < argc) {
        // in this case, switch to the method "socket" always
        gpsd_source_spec(argv[optind], &source);
    }
#if 0
    (void)fprintf(gpxlogfile, "<!-- server: %s port: %s  device: %s -->\n",
                 source.server, source.port, source.device);
#endif

    // catch all interesting signals
    (void)signal(SIGTERM, quit_handler);
    (void)signal(SIGQUIT, quit_handler);
    (void)signal(SIGINT, quit_handler);

    // might be time to daemonize
    if (daemonize) {
        errno = 0;
        // not SuS/POSIX portable, but we have our own fallback version
        if (0 != os_daemon(0, 0)) {
            (void)fprintf(stderr, "daemonization failed: %s(%d)\n",
                          strerror(errno), errno);
        }
    }

    // syslog (LOG_INFO, "---------- STARTED ----------");

    if (0 != gps_open(source.server, source.port, &gpsdata)) {
        (void)fprintf(stderr,
                      "%s: no gpsd running or network error: %d, %s\n",
                      progname, errno, gps_errstr(errno));
        exit(EXIT_FAILURE);
    }

    if (NULL != source.device) {
        flags |= WATCH_DEVICE;
    }
    if (NULL != source.port &&
        NULL == file_in) {
        // only to sockets, not infiles, shared memory or dbus
        if (0 > gps_stream(&gpsdata, flags, source.device)) {
            syslog(LOG_ERR, "gps_stream() failed");
            exit(EXIT_FAILURE);
        }
    }

    print_gpx_header();
    // make sure footer added on exit
    if (0 != atexit(cleanup)) {
        syslog(LOG_ERR, "atexit() failed");
        exit(EXIT_FAILURE);
    }

    while (0 > gps_mainloop(&gpsdata, timeout * 1000000,
                            conditionally_log_fix)) {
        // fell out of mainloop, some sort of error, or just a timeout
        if (!reconnect || 0 != sig_flag) {
            // give up
            break;
        }
        // avoid banging on reconnect
        // (unsigned int) cast to shut up Coverity CID 355859
        (void)sleep((unsigned int)timeout);
        syslog(LOG_INFO, "timeout; about to reconnect");
    }

    if (0 != sig_flag &&
        SIGINT != sig_flag) {
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}

// vim: set expandtab shiftwidth=4
