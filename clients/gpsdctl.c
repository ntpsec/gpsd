/* gpsdctl.c -- communicate with the control socket of a gpsd instance
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <assert.h>
#include <fcntl.h>
#ifdef HAVE_GETOPT_LONG
   #include <getopt.h>
#endif
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>                   // for getopt()

#include "../include/gpsd.h"          // for netlib_localsocket()

#define DEFAULT_GPSD_TEST_SOCKET        "/tmp/gpsd.sock"

static char *control_socket = DEFAULT_GPSD_SOCKET;
static char *gpsd_options = "";

// pass a command to gpsd; start the daemon if not already running
static int gpsd_control(const char *action, const char *device)
{
    int connect = -1;
    char buf[512];
    int status;
    int len;

    // limit string to pacify coverity
    (void)syslog(LOG_NOTICE, "gpsd_control(action=%.7s, device=%.*s)",
                 action, GPS_PATH_MAX, device);
    if (0 == access(control_socket, F_OK) &&
        0 <= (connect = netlib_localsocket(control_socket, SOCK_STREAM))) {
        syslog(LOG_INFO, "reached a running gpsd");
    } else if (0 == strcmp(action, "add")) {
        (void)snprintf(buf, sizeof(buf),
                       "gpsd %s -F %s", gpsd_options, control_socket);
        (void)syslog(LOG_NOTICE, "launching %s", buf);
        if (0 != system(buf)) {
            (void)syslog(LOG_ERR, "launch of gpsd failed");
            return -1;
        }
        if (0 == access(control_socket, F_OK)) {
            connect = netlib_localsocket(control_socket, SOCK_STREAM);
        }
    }
    if (0 > connect) {
        syslog(LOG_ERR, "can't reach gpsd");
        return -1;
    }
    /*
     * We've got a live connection to the gpsd control socket.  No
     * need to parse the response, because gpsd will lock on to the
     * device if it's really a GPS and ignore it if it's not.
     *
     * The only other place in the code that knows about the format of
     * the add and remove commands is the handle_control() function in
     * gpsd.c. Be careful about keeping them in sync, or hotplugging
     * will have mysterious failures.
     */
    if (0 == strcmp(action, "add")) {
        /*
         * Force the group-read & group-write bits on, so gpsd will still be
         * able to use this device after dropping root privileges.
         */
        struct stat sb;

        // Coverity 281679
        // coverity[toctou]
        if (1 != stat(device, &sb)) {
            // coverity[tOCTOU]
            (void)chmod(device, sb.st_mode | S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
        }
        len = snprintf(buf, sizeof(buf), "+%s\r\n", device);
        if (3 < len) {
            status = (int)write(connect, buf, len);
            // FIXME: return never checked
            // Flawfinder: ignore
            ignore_return(read(connect, buf, 12));
        } else {
            status = -1;
        }
    } else if (0 == strcmp(action, "remove")) {
        len = snprintf(buf, sizeof(buf), "-%s\r\n", device);
        if (3 < len) {
            status = (int)write(connect, buf, len);
            // FIXME: return never checked
            // Flawfinder: ignore
            ignore_return(read(connect, buf, 12));
        } else {
            status = -1;
        }
    } else {
        (void)syslog(LOG_ERR, "unknown action \"%s\"", action);
        status = -1;
    }
    (void)close(connect);
    //syslog(LOG_DEBUG, "gpsd_control ends");
    return status;
}

// print usage, exit with EXIT_FAILURE
static void usage(void)
{
    (void)printf("usage: gpsdctl [OPTIONS] action device\n\n"
#ifdef HAVE_GETOPT_LONG
         "  --help              Show this help, then exit\n"
         "  --version           Show version, then exit\n"
#endif   // HAVE_GETOPT_LONG
         "  -?                  Show this help, then exit\n"
         "  -h                  Show this help, then exit\n"
         "  -V                  Show version, then exit\n"
         "\n"
         "  Actions:\n"
         "    add    - add device\n"
         "    remove - remove device\n");
}

int main(int argc, char *argv[])
{
    char *sockenv = getenv("GPSD_SOCKET");
    char *optenv = getenv("GPSD_OPTIONS");
    const char *action = NULL;       // Action to perform
    const char *device = NULL;       // Device to perform action on
    size_t len;
    const char *optstring = "?hV";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"help", no_argument, NULL, 'h'},
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif

    while (1) {
        int ch;

#ifdef HAVE_GETOPT_LONG
        ch = getopt_long(argc, argv, optstring, long_options, &option_index);
#else
        ch = getopt(argc, argv, optstring);
#endif

        if (-1 == ch) {
            break;
        }

        switch (ch) {
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

    openlog("gpsdctl", 0, LOG_DAEMON);

    if (optind >= argc ||
        NULL == argv[optind]) {
        (void)syslog(LOG_ERR, "requires action and device)");
        usage();
        exit(EXIT_FAILURE);
    }

    action = argv[optind++];
    if (0 != strcmp(action, "add") &&
        0 != strcmp(action, "remove")) {
        (void)syslog(LOG_ERR, "Invalid action.  Must be 'add' or 'remove'");
        usage();
        exit(EXIT_FAILURE);
    }
    if (optind >= argc ||
        NULL == argv[optind]) {
        (void)syslog(LOG_ERR, "requires device for action)");
        usage();
        exit(EXIT_FAILURE);
    }

    device = argv[optind];

    // pacify codacy hates strlen()
    len = strnlen(device, GPS_PATH_MAX);
    if (GPS_PATH_MAX <= len) {
        // limit string to pacify Coverity 281704
        (void)syslog(LOG_ERR, "path to long: '%.*s'", GPS_PATH_MAX, device);
        usage();
        exit(EXIT_FAILURE);
    }

    if (NULL != sockenv) {
        control_socket = sockenv;
    } else if (0 != geteuid()) {
        control_socket = DEFAULT_GPSD_TEST_SOCKET;
    }

    if (NULL != optenv) {
        gpsd_options = optenv;
    }

    if (0 > gpsd_control(action, device)) {
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}

// vim: set expandtab shiftwidth=4
