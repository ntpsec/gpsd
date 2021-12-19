/*
 * gpspipe
 *
 * a simple program to connect to a gpsd daemon and dump the received data
 * to stdout
 *
 * This will dump the raw NMEA from gpsd to stdout
 *      gpspipe -r
 *
 * This will dump the super-raw data (gps binary) from gpsd to stdout
 *      gpspipe -R
 *
 * This will dump the GPSD sentences from gpsd to stdout
 *      gpspipe -w
 *
 * This will dump the GPSD and the NMEA sentences from gpsd to stdout
 *      gpspipe -wr
 *
 * Original code by: Gary E. Miller <gem@rellim.com>.  Cleanup by ESR.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <errno.h>
#include <fcntl.h>
#ifdef HAVE_GETOPT_LONG
    #include <getopt.h>   // for getopt_long()
#endif
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>               /* for time_t */
#include <unistd.h>

#ifdef HAVE_SYS_SOCKET_H
    #include <sys/socket.h>
#endif /* HAVE_SYS_SOCKET_H */
#include <termios.h>            /* for speed_t, and cfmakeraw() on some OS */
#ifdef HAVE_WINSOCK2_H
    #include <winsock2.h>
#endif /* HAVE_WINSOCK2_H */

#include "../include/gpsd.h"            // for os_daemon()

#include "../include/gpsdclient.h"

static struct gps_data_t gpsdata;
static void spinner(unsigned int, unsigned int);

/* NMEA-0183 standard baud rate */
#define BAUDRATE B4800

/* Serial port variables */
static struct termios oldtio, newtio;
static int fd_out = STDOUT_FILENO; /* output initially goes to standard output */
static char serbuf[255];
static int debug;

/* open the serial port and set it up */
static void open_serial(char *device)
{
    /*
     * Open modem device for reading and writing and not as controlling
     * tty.
     */
    if ((fd_out = open(device, O_RDWR | O_NOCTTY)) == -1) {
        (void)fprintf(stderr, "gpspipe: error opening serial port\n");
        exit(EXIT_FAILURE);
    }

    /* Save current serial port settings for later */
    if (tcgetattr(fd_out, &oldtio) != 0) {
        (void)fprintf(stderr, "gpspipe: error reading serial port settings\n");
        exit(EXIT_FAILURE);
    }

    /* Clear struct for new port settings. */
    memset(&newtio, 0, sizeof(newtio));

    /* make it raw */
    (void)cfmakeraw(&newtio);
    /* set speed */
    (void)cfsetospeed(&newtio, BAUDRATE);

    /* Clear the modem line and activate the settings for the port. */
    (void)tcflush(fd_out, TCIFLUSH);
    if (tcsetattr(fd_out, TCSANOW, &newtio) != 0) {
        (void)fprintf(stderr, "gpspipe: error configuring serial port\n");
        exit(EXIT_FAILURE);
    }
}

static void usage(void)
{
    (void)fprintf(stderr,
                  "Usage: gpspipe [OPTIONS] [server[:port[:device]]]\n\n"
#ifdef HAVE_GETOPT_LONG
                  "  --count COUNT    Exit after COUNT packets.\n"
                  "  --daemonize      Run as daemon.\n"
                  "  --debug LVL      Set debug level to LVL.\n"
                  "  --help           Show this help and exit.\n"
                  "  --json           Dump gpsd native JSON data.\n"
                  "  --nmea           Dump (pseudo) NMEA.\n"
                  "  --output FILE    Write output to FILE.\n"
                  "  --pps            Include PPS JSON in NMEA or raw mode.\n"
                  "  --profile        Include profiling info in the JSON.\n"
                  "  --raw            Dump super-raw mode, GPS binary and "
                  "NMEA.\n"
                  "  --scaled         Set scaled flag. For AIS and subframe "
                  "data.\n"
                  "  --seconds SEC    Exit after SEC seconds delay.\n"
                  "  --serial DEV     Emulate a 4800bps NMEA GPS on "
                  "DEV (use with '-r').\n"
                  "  --sleep          Sleep for ten seconds before "
                  "connecting to gpsd.\n"
                  "  --spinner        Print a little spinner.\n"
                  "  --split24        Set the split24 flag.\n"
                  "  --timefmt FORMAT Set the timestamp format "
                  "(strftime(3)-like; implies '-t').\n"
                  "  --timestamp      Time stamp the data.\n"
                  "  --usec           Time stamp in usec, implies '-t'. "
                  "Use twice for sec.usec.\n"
                  "  --version        Print version and exit.\n"
                  "  --zulu           Set the timestamp format to iso8601, "
                  "implies '-t'\n"
#endif
                  "  -2               Set the split24 flag.\n"
                  "  -d               Run as a daemon.\n"
                  "  -D LVL           Set debug level to LVL.\n"
                  "  -h               Show this help and exit.\n"
                  "  -l               Sleep for ten seconds before "
                  "connecting to gpsd.\n"
                  "  -n COUNT         Exit after count packets.\n"
                  "  -o FILE          Write output to FILE.\n"
                  "  -P               Include PPS JSON in NMEA or raw mode.\n"
                  "  -p               Include profiling info in the JSON.\n"
                  "  -r               Dump (pseudo) NMEA.\n"
                  "  -R               Dump super-raw mode, GPS binary and "
                  "NMEA.\n"
                  "  -s DEV           Emulate a 4800bps NMEA GPS on "
                  "DEV (use with '-r').\n"
                  "  -S               Set scaled flag. For AIS and subframe "
                  "data.\n"
                  "  -T FORMAT        Set the timestamp format "
                  "(strftime(3)-like; implies '-t')\n"
                  "  -t               Time stamp the data.\n"
                  "  -u               Time stamp in usec, implies '-t'. "
                  "Use -uu to output sec.usec.\n"
                  "  -v               Print a little spinner.\n"
                  "  -V               Print version and exit.\n"
                  "  -w               Dump gpsd native JSON data.\n"
                  "  -x SEC           Exit after SEC seconds delay.\n"
                  "  -Z               Set the timestamp format to iso8601, "
                  "implies '-t'.\n\n"
                  "You must specify one, or more, of: "
                  "--json, --nmea, --raw, -r, -R, or -w\n"
                  "You must use -o if you use -d.\n");
}

int main(int argc, char **argv)
{
    char buf[4096];
    bool timestamp = false;
    bool iso8601 = false;
    char *format = "%F %T";
    char *zulu_format = "%FT%T";
    char tmstr[200];
    bool daemonize = false;
    bool binary = false;
    bool sleepy = false;
    bool new_line = true;
    bool raw = false;
    bool watch = false;
    int option_u = 0;                   // option to show uSeconds
    long count = -1;
    time_t exit_timer = 0;
    unsigned int vflag = 0, l = 0;
    FILE *fp;
    unsigned int flags;
    fd_set fds;

    struct fixsource_t source;
    char *serialport = NULL;
    char *outfile = NULL;
    const char *optstring = "2?dD:hln:o:pPrRwSs:tT:uvVx:Z";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"count", required_argument, NULL, 'n'},
        {"daemonize", no_argument, NULL, 'd'},
        {"debug", required_argument, NULL, 'D'},
        {"help", no_argument, NULL, 'h'},
        {"json", no_argument, NULL, 'w'},
        {"nmea", no_argument, NULL, 'r' },
        {"output", required_argument, NULL, 'o'},
        {"pps", no_argument, NULL, 'P' },
        {"profile", no_argument, NULL, 'p' },
        {"scaled", no_argument, NULL, 'S' },
        {"seconds", required_argument, NULL, 'x'},
        {"serial", no_argument, NULL, 'r' },
        {"sleep", required_argument, NULL, 'i'},
        {"spinner", no_argument, NULL, 'v' },
        {"split24", no_argument, NULL, '2'},
        {"raw", no_argument, NULL, 'R' },
        {"timefmt", required_argument, NULL, 'T'},
        {"timestamp", required_argument, NULL, 't'},
        {"usec", no_argument, NULL, 'u'},
        {"version", no_argument, NULL, 'V' },
        {"zulu", no_argument, NULL, 'Z'},
        {NULL, 0, NULL, 0},
    };
#endif

    flags = WATCH_ENABLE;
    while (1) {
        int ch;
#ifdef HAVE_GETOPT_LONG
        ch = getopt_long(argc, argv, optstring, long_options, &option_index);
#else
        ch = getopt(argc, argv, optstring);
#endif

        if (ch == -1) {
            break;
        }

        switch (ch) {
        case '2':
            flags |= WATCH_SPLIT24;
            break;
        case 'D':
            debug = atoi(optarg);
            gps_enable_debug(debug, stderr);
            break;
        case 'd':
            daemonize = true;
            break;
        case 'l':
            sleepy = true;
            break;
        case 'n':
            count = strtol(optarg, 0, 0);
            break;
        case 'o':
            outfile = optarg;
            break;
        case 'P':
            flags |= WATCH_PPS;
            break;
        case 'p':
            flags |= WATCH_TIMING;
            break;
        case 'R':
            flags |= WATCH_RAW;
            binary = true;
            break;
        case 'r':
            raw = true;
            /*
             * Yes, -r invokes NMEA mode rather than proper raw mode.
             * This emulates the behavior under the old protocol.
             */
            flags |= WATCH_NMEA;
            break;
        case 'S':
            flags |= WATCH_SCALED;
            break;
        case 's':
            serialport = optarg;
            break;
        case 'T':
            timestamp = true;
            format = optarg;
            break;
        case 't':
            timestamp = true;
            break;
        case 'u':
            timestamp = true;
            option_u++;
            break;
        case 'V':
            (void)fprintf(stderr, "%s: %s (revision %s)\n",
                          argv[0], VERSION, REVISION);
            exit(EXIT_SUCCESS);
        case 'v':
            vflag++;
            break;
        case 'w':
            flags |= WATCH_JSON;
            watch = true;
            break;
        case 'x':
            exit_timer = time(NULL) + strtol(optarg, 0, 0);
            break;
        case 'Z':
            timestamp = true;
            format = zulu_format;
            iso8601 = true;
            break;
        case '?':
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
        default:
            usage();
            exit(EXIT_FAILURE);
        }
    }

    /* Grok the server, port, and device. */
    if (optind < argc) {
        gpsd_source_spec(argv[optind], &source);
    } else {
        gpsd_source_spec(NULL, &source);
    }

    if (serialport != NULL && !raw) {
        (void)fprintf(stderr, "gpspipe: use of '-s' requires '-r'.\n");
        exit(EXIT_FAILURE);
    }

    if (outfile == NULL && daemonize) {
        (void)fprintf(stderr, "gpspipe: use of '-d' requires '-o'.\n");
        exit(EXIT_FAILURE);
    }

    if (!raw && !watch && !binary) {
        (void)fprintf(stderr,
                      "gpspipe: one of '-R', '-r', or '-w' is required.\n");
        exit(EXIT_FAILURE);
    }

    /* Daemonize if the user requested it. */
    if (daemonize) {
        if (os_daemon(0, 0) != 0) {
            (void)fprintf(stderr,
                          "gpspipe: daemonization failed: %s\n",
                          strerror(errno));
        }
    }

    /* Sleep for ten seconds if the user requested it. */
    if (sleepy) {
        (void)sleep(10);
    }

    /* Open the output file if the user requested it. If the user
     * requested '-R', we use the 'b' flag in fopen() to "do the right
     * thing" in non-linux/unix OSes. */
    if (outfile == NULL) {
        fp = stdout;
    } else {
        fp = fopen(outfile, binary ? "wb" : "w");
        if (fp == NULL) {
            (void)fprintf(stderr,
                          "gpspipe: unable to open output file: %s\n",
                          outfile);
            exit(EXIT_FAILURE);
        }
    }

    /* Open the serial port and set it up. */
    if (serialport) {
        open_serial(serialport);
    }

    if (gps_open(source.server, source.port, &gpsdata) != 0) {
        (void)fprintf(stderr,
                      "gpspipe: could not connect to gpsd %s:%s, %s(%d)\n",
                      source.server, source.port, gps_errstr(errno), errno);
        exit(EXIT_FAILURE);
    }

    if (source.device != NULL) {
        flags |= WATCH_DEVICE;
    }
    (void)gps_stream(&gpsdata, flags, source.device);

    if ((isatty(STDERR_FILENO) == 0) || daemonize) {
        vflag = 0;
    }

    for (;;) {
        int r = 0;
        struct timespec tv;

        tv.tv_sec = 0;
        tv.tv_nsec = 100000000;
        FD_ZERO(&fds);
        FD_SET(gpsdata.gps_fd, &fds);
        errno = 0;
        r = pselect(gpsdata.gps_fd+1, &fds, NULL, NULL, &tv, NULL);
        if (r >= 0 && exit_timer && time(NULL) >= exit_timer) {
            break;
        }
        if (r == -1 && errno != EINTR) {
            (void)fprintf(stderr, "gpspipe: select error %s(%d)\n",
                          strerror(errno), errno);
            exit(EXIT_FAILURE);
        } else if (r == 0) {
            continue;
        }

        if (vflag) {
            spinner(vflag, l++);
        }

        /* reading directly from the socket avoids decode overhead */
        errno = 0;
        r = (int)recv(gpsdata.gps_fd, buf, sizeof(buf), 0);
        if (r > 0) {
            int i = 0;
            int j = 0;
            for (i = 0; i < r; i++) {
                char c = buf[i];
                if (j < (int)(sizeof(serbuf) - 1)) {
                    serbuf[j++] = buf[i];
                }
                if (new_line && timestamp) {
                    char tmstr_u[40];            // time with "usec" resolution
                    struct timespec now;
                    struct tm tmp_now;
                    int written;

                    (void)clock_gettime(CLOCK_REALTIME, &now);
                    (void)gmtime_r((time_t *)&(now.tv_sec), &tmp_now);
                    (void)strftime(tmstr, sizeof(tmstr), format, &tmp_now);
                    new_line = false;

                    switch (option_u) {
                    case 2:
                        if (iso8601) {
                            // codacy does not like strlen()
                            written = strnlen(tmstr, sizeof(tmstr));
                            tmstr[written] = 'Z';
                            tmstr[written+1] = '\0';
                        }
                        (void)snprintf(tmstr_u, sizeof(tmstr_u),
                                       " %lld.%06ld",
                                       (long long)now.tv_sec,
                                       (long)now.tv_nsec/1000);
                        break;
                    case 1:
                        written = snprintf(tmstr_u, sizeof(tmstr_u),
                                           ".%06ld", (long)now.tv_nsec/1000);

                        if ((0 < written) && (40 > written) && iso8601) {
                            tmstr_u[written-1] = 'Z';
                            tmstr_u[written] = '\0';
                        }
                        break;
                    default:
                        *tmstr_u = '\0';
                    }

                    if (fprintf(fp, "%.24s%s: ", tmstr, tmstr_u) <= 0) {
                        (void)fprintf(stderr,
                                      "gpspipe: write error, %s(%d)\n",
                                      strerror(errno), errno);
                        exit(EXIT_FAILURE);
                    }
                }
                if (fputc(c, fp) == EOF) {
                    (void)fprintf(stderr, "gpspipe: write error, %s(%d)\n",
                                  strerror(errno), errno);
                    exit(EXIT_FAILURE);
                }

                if (c == '\n') {
                    if (serialport != NULL) {
                        if (write(fd_out, serbuf, (size_t) j) == -1) {
                            (void)fprintf(stderr,
                                          "gpspipe: serial port write error,"
                                          " %s(%d)\n",
                                          strerror(errno), errno);
                            exit(EXIT_FAILURE);
                        }
                        j = 0;
                    }

                    new_line = true;
                    /* flush after every good line */
                    if (fflush(fp)) {
                        (void)fprintf(stderr,
                                      "gpspipe: fflush error, %s(%d)\n",
                                      strerror(errno), errno);
                        exit(EXIT_FAILURE);
                    }
                    if (count > 0) {
                        if (0 >= --count) {
                            /* completed count */
                            exit(EXIT_SUCCESS);
                        }
                    }
                }
            }
        } else {
            if (r == -1) {
                if (errno == EAGAIN) {
                    continue;
                } else {
                    (void)fprintf(stderr, "gpspipe: read error %s(%d)\n",
                              strerror(errno), errno);
                }
                exit(EXIT_FAILURE);
            } else {
                exit(EXIT_SUCCESS);
            }
        }
    }

#ifdef __UNUSED__
    if (serialport != NULL) {
        /* Restore the old serial port settings. */
        if (tcsetattr(fd_out, TCSANOW, &oldtio) != 0) {
            (void)fprintf(stderr,
                          "gpsipe: error restoring serial port settings\n");
            exit(EXIT_FAILURE);
        }
    }
#endif /* __UNUSED__ */

    exit(EXIT_SUCCESS);
}

static void spinner(unsigned int v, unsigned int num)
{
    char *spin = "|/-\\";

    (void)fprintf(stderr, "\010%c", spin[(num / (1 << (v - 1))) % 4]);
    (void)fflush(stderr);
}
// vim: set expandtab shiftwidth=4
