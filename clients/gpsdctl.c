/* gpsdctl.c -- communicate with the control socket of a gpsd instance
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <assert.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include "../include/gpsd.h"          // for netlib_localsocket()

#define DEFAULT_GPSD_TEST_SOCKET        "/tmp/gpsd.sock"

static char *control_socket = DEFAULT_GPSD_SOCKET;
static char *gpsd_options = "";

// pass a command to gpsd; start the daemon if not already running
static int gpsd_control(const char *action, const char *argument)
{
    int connect = -1;
    char buf[512];
    int status;
    int len;

    // limit string to pacify coverity
    (void)syslog(LOG_ERR, "gpsd_control(action=%.7s, arg=%.*s)",
                 action, GPS_PATH_MAX, argument);
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

        // coverity[toctou]
        if (1 != stat(argument, &sb)) {
            (void)chmod(argument, sb.st_mode | S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
        }
        len = snprintf(buf, sizeof(buf), "+%s\r\n", argument);
        if (3 < len) {
            status = (int)write(connect, buf, len);
            // FIXME: return never checked
            ignore_return(read(connect, buf, 12));
        } else {
            status = -1;
        }
    } else if (0 == strcmp(action, "remove")) {
        len = snprintf(buf, sizeof(buf), "-%s\r\n", argument);
        if (3 < len) {
            status = (int)write(connect, buf, len);
            // FIXME: return never checked
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
    (void)printf("usage: gpsdctl action argument\n\n"
                 "  Actions:\n"
                 "    add    - add device\n"
                 "    remove - remove device\n");
    exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
    char *sockenv = getenv("GPSD_SOCKET");
    char *optenv = getenv("GPSD_OPTIONS");
    size_t len;

    // FIXME: add usage()
    openlog("gpsdctl", 0, LOG_DAEMON);
    if (3 != argc) {
        (void)syslog(LOG_ERR, "requires action and argument (%d)", argc);
        usage();
    }
    // pacify coverity, codacy hates strlen()
    len = strnlen(argv[1], 8);
    if (3 > len ||
        7 < len) {
        (void)syslog(LOG_ERR, "invalid action '%s'", argv[1]);
        usage();
    }

    // pacify coverity, codacy hates strlen()
    len = strnlen(argv[2], GPS_PATH_MAX);
    if (GPS_PATH_MAX <= len) {
        // limit string to pacify Coverity
        (void)syslog(LOG_ERR, "invalid path '%.*s'", GPS_PATH_MAX, argv[2]);
        usage();
    }

    if (NULL != sockenv) {
        control_socket = sockenv;
    } else if (0 != geteuid()) {
        control_socket = DEFAULT_GPSD_TEST_SOCKET;
    }

    if (NULL != optenv) {
        gpsd_options = optenv;
    }

    if (0 > gpsd_control(argv[1], argv[2])) {
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}

// vim: set expandtab shiftwidth=4
