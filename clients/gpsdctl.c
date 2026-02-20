/* gpsdctl.c -- communicate with the control socket of a gpsd instance
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <assert.h>
#include <errno.h>                   // for errno, strerrno()
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
static bool log_stderr = false;

static void client_log(int level, const char *msg, ...)
{
    va_list ap;
    int len;

    va_start(ap, msg);
    if (log_stderr) {
        len = vfprintf(stderr, msg, ap);
        fputs("\n", stderr);
        if (1 > len) {
            // uh, oh
            fprintf(stderr, "format error %s\n", msg);
            exit(EXIT_FAILURE);
        }
    } else {
        (void)vsyslog(level, msg, ap);
    }
    va_end(ap);
}

// pass a command to gpsd; start the daemon if not already running
static ssize_t gpsd_control(const char *action, const char *device)
{
    int connect = -1;
    char buf[GPS_JSON_RESPONSE_MAX];
    ssize_t status;
    int len = 0;
    bool do_write = false;
    struct stat sb;

    // limit string to pacify coverity
    client_log(LOG_NOTICE, "NOTICE: gpsd_control(action=%.7s, device=%.*s)",
                 action, GPS_PATH_MAX, device);
    if (NET_LOCAL != netgnss_uri_type(device)) {
        // don't confuse stat().
    } else if (0 != stat(device, &sb)) {
        client_log(LOG_ERR, "ERR: stat() device=%.*s) %s(%d)",
                     GPS_PATH_MAX, device, strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
    if (0 == access(control_socket, R_OK | W_OK) &&
        0 <= (connect = netlib_localsocket(control_socket, SOCK_STREAM))) {
        client_log(LOG_INFO, "INFO: reached a running gpsd at %s",
                   control_socket);
    } else if (0 == strcmp(action, "add")) {
        (void)snprintf(buf, sizeof(buf),
                       "gpsd %s -F %s", gpsd_options, control_socket);
        // FIXME: malicious gpsd_options possible, use exec()
        client_log(LOG_NOTICE, "NOTICE: launching %s", buf);
        if (0 != system(buf)) {
            client_log(LOG_ERR, "ERR: launch of gpsd failed");
            return -1;
        }
        if (0 == access(control_socket, R_OK | W_OK)) {
            connect = netlib_localsocket(control_socket, SOCK_STREAM);
        }
    }
    if (0 > connect) {
        client_log(LOG_ERR, "ERR: can't reach gpsd control socket.  %s(%d)",
               strerror(errno), errno);
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

        // Coverity 281679
        // coverity[toctou]
        if (0 != chmod(device, sb.st_mode | S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP)) {
            client_log(LOG_WARNING, "WARNING: chnod() device=%.*s) %s(%d)",
                         GPS_PATH_MAX, device, strerror(errno), errno);
        }

        len = snprintf(buf, sizeof(buf), "+%s\r\n", device);
        do_write = true;
    } else if (0 == strcmp(action, "remove")) {
        len = snprintf(buf, sizeof(buf), "-%s\r\n", device);
        do_write = true;
    } else {
        client_log(LOG_ERR, "ERR: unknown action \"%s\"", action);
        status = -1;
    }

    if (do_write) {
        bool do_read = false;

        if (3 < len) {
            status = write(connect, buf, len);
            do_read = true;
            if (0 > status) {
                client_log(LOG_ERR,
                           "ERR: Could not write gpsd control socket. %s(%d)",
                       strerror(errno), errno);
                do_read = false;    // don't bother to read
            }
        } else {
            status = -1;
        }
        if (do_read) {
            ssize_t status1;

            // timeout??, wait for it??
            status1 = read(connect, buf, sizeof(buf) - 2);
            if (0 == status1) {
                // EOF
            } else if (0 > status1) {
                client_log(LOG_ERR,
                           "ERR: Could not read gpsd control socket");
            } else {
                // force NUL terminated
                buf[status1] = '\0';
                if (NULL == strstr(buf, "ACK")) {
                    client_log(LOG_ERR, "ERR: gpsd returned %s", buf);
                    status = -1;
                } else {
                    // Succcss!
                    client_log(LOG_INFO, "INFO: gpsd returned %s", buf);
                    status = 0;
                }
            }
        }
    }
    (void)close(connect);
    //client_log(LOG_DEBUG, "DEBUG: gpsd_control ends");
    return status;
}

// print usage, exit with EXIT_FAILURE
static void usage(void)
{
    (void)printf("usage: gpsdctl [OPTIONS] action device\n\n"
#ifdef HAVE_GETOPT_LONG
         "  --help              Show this help, then exit\n"
         "  --log               log to stderr instead of syslog\n"
         "  --version           Show version, then exit\n"
#endif   // HAVE_GETOPT_LONG
         "  -?                  Show this help, then exit\n"
         "  -h                  Show this help, then exit\n"
         "  -l                  log to stderr instead of syslog\n"
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
    char action[10] = "";            // Action to perform
    const char *device = NULL;       // Device to perform action on
    size_t len;
    const char *optstring = "?hlV";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"help", no_argument, NULL, 'h'},
        {"log", no_argument, NULL, 'l'},
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
        case 'l':
            log_stderr  =  true;
            break;
        default:
            usage();
            exit(EXIT_FAILURE);
        }
    }

    if (!log_stderr) {
        openlog("gpsdctl", 0, LOG_DAEMON);
    }

    if (optind >= argc ||
        NULL == argv[optind]) {
        client_log(LOG_ERR, "ERROR: requires action and device)");
        usage();
        exit(EXIT_FAILURE);
    }

    // Pacify Coverity 281704, remove taint from argv[]
    strlcpy(action, argv[optind++], sizeof(action));
    if (0 != strcmp(action, "add") &&
        0 != strcmp(action, "remove")) {
        client_log(LOG_ERR,
                   "ERROR: Invalid action.  Must be 'add' or 'remove'");
        usage();
        exit(EXIT_FAILURE);
    }
    if (optind >= argc ||
        NULL == argv[optind]) {
        client_log(LOG_ERR, "ERROR: requires device for action)");
        usage();
        exit(EXIT_FAILURE);
    }
    device = argv[optind];

    // pacify codacy hates strlen()
    len = strnlen(device, GPS_PATH_MAX);
    if (GPS_PATH_MAX <= len) {
        // limit string to pacify Coverity 281704
        client_log(LOG_ERR,
                   "ERROR: path to long: '%.*s'", GPS_PATH_MAX, device);
        usage();
        exit(EXIT_FAILURE);
    }
    if (0 == len) {
        // empty device
        client_log(LOG_ERR, "ERROR: No device given");
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
