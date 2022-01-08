/*
 * Watch a specified serial port for transitions that might be 1PPS.
 *
 * Each output line is the second and nanosecond parts of a timestamp
 * followed by the names of handshake signals then asserted.  Off
 * transitions may generate lines with no signals asserted.
 *
 * If you don't see output within a second, use cgps, xgps, or some other
 * equivalent tool to check that your device has satellite lock and is
 * getting fixes before giving up on the possibility of 1PPS.
 *
 * Also, check your cable. Cheap DB9 to DB9 cables such as those
 * issued with UPSes often carry TXD/RXD/GND only, omitting handshake
 * lines such as DCD.  Suspect this especially if the cable jacket
 * looks too skinny to hold more than three leads!
 *
 * This code requires only ANSI/POSIX. If it doesn't compile and run
 * on your Unix there is something very wrong with your Unix.
 *
 * This code by ESR, Copyright 2013, under BSD terms.
 * This file is Copyright 2013 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <errno.h>
#include <fcntl.h>                    // needed for open() and friends
#ifdef HAVE_GETOPT_LONG
   #include <getopt.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#if defined(HAVE_SYS_TIMEPPS_H)
    #include <sys/timepps.h>
#endif
#include <sys/types.h>
#include <time.h>

// include unistd.h here as it is missing on older pps-tools releases.
// 'close' is not defined otherwise.
#include <unistd.h>

#include "../include/compiler.h"     // for FALLTHROUGH
#include "../include/timespec.h"

time_t exit_timer = 0;               // for -x option

struct assoc {
    int mask;
    char *string;
};

/*
 * Possible pins for PPS: DCD, CTS, RI, DSR. Pinouts:
 *
 * DB9  DB25  Name      Full name
 * ---  ----  ----      --------------------
 *  3     2    TXD  --> Transmit Data
 *  2     3    RXD  <-- Receive Data
 *  7     4    RTS  --> Request To Send
 *  8     5    CTS  <-- Clear To Send
 *  6     6    DSR  <-- Data Set Ready
 *  4    20    DTR  --> Data Terminal Ready
 *  1     8    DCD  <-- Data Carrier Detect
 *  9    22    RI   <-- Ring Indicator
 *  5     7    GND      Signal ground
 *
 * Note that it only makes sense to wait on handshake lines
 * activated from the receive side (DCE->DTE) here; in this
 * context "DCE" is the GPS. {CD,RI,CTS,DSR} is the
 * entire set of these.
 */
static const struct assoc hlines[] = {
    {TIOCM_CD, "TIOCM_CD"},
    {TIOCM_RI, "TIOCM_RI"},
    {TIOCM_DSR, "TIOCM_DSR"},
    {TIOCM_CTS, "TIOCM_CTS"},
};

#if defined(HAVE_SYS_TIMEPPS_H)
// aka RFC2783

static const struct assoc caps[] = {
    {PPS_CAPTUREASSERT, "PPS_CAPTUREASSERT"},
    {PPS_CAPTURECLEAR, "PPS_CAPTURECLEAR"},
    {PPS_CAPTUREBOTH, "PPS_CAPTUREBOTH"},
    {PPS_OFFSETASSERT, "PPS_OFFSETASSERT"},
    {PPS_OFFSETCLEAR, "PPS_OFFSETCLEAR"},
    {PPS_CANWAIT, "PPS_CANWAIT"},
    {PPS_CANPOLL, "PPS_CANPOLL"},
    {PPS_ECHOASSERT, "PPS_ECHOASSERT"},
    {PPS_ECHOCLEAR, "PPS_ECHOCLEAR"},
    {PPS_TSFMT_TSPEC, "PPS_TSFMT_TSPEC"},
    {PPS_TSFMT_NTPFP, "PPS_TSFMT_NTPFP"},
};

static void do_kpps(pps_handle_t kpps_handle)
{
    int kpps_caps = 0;
    pps_params_t pp;
    const struct assoc *sp;

    // have kernel PPS handle. get RFC2783 features supported
    if (0 > time_pps_getcap(kpps_handle, &kpps_caps)) {
        (void)fprintf(stderr,
                      "ERROR: time_pps_getcap() failed: %s(%d)\n",
                      strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
    (void)fprintf(stderr, "INFO: kpps_caps 0x%02X\n", kpps_caps);

    for (sp = caps; sp < caps + sizeof(caps) / sizeof(caps[0]); sp++) {
        if (0 != (kpps_caps & sp->mask)) {
            (void)printf("  %s\n", sp->string);
        }
    }
    puts("");
    if (0 == (PPS_CANWAIT & kpps_caps)) {
        (void)fputs("ERROR: PPS_CANWAIT is missing.\n", stderr);
    }

    // construct the setparms structure
    memset((void *)&pp, 0, sizeof(pps_params_t));
    pp.api_version = PPS_API_VERS_1;    // version 1 protocol

    switch ((PPS_CAPTUREASSERT | PPS_CAPTURECLEAR) & kpps_caps) {
    case PPS_CAPTUREASSERT:
        (void)fputs("WARNING: missing PPS_CAPTURECLEAR, pulse may be offset\n",
                    stderr);
        pp.mode |= PPS_CAPTUREASSERT;
        break;
    case PPS_CAPTURECLEAR:
        (void)fputs("WARNING: missing PPS_CAPTUREASSERT, pulse may be offset\n",
                    stderr);
        pp.mode |= PPS_CAPTURECLEAR;
        break;
    case PPS_CAPTUREASSERT | PPS_CAPTURECLEAR:
        pp.mode |= PPS_CAPTUREASSERT | PPS_CAPTURECLEAR;
        break;
    default:
        (void)fputs("WARNING: missing PPS_CAPTUREASSERT and PPS_CAPTURECLEAR\n",
                    stderr);
        exit(EXIT_FAILURE);
    }

    if (0 > time_pps_setparams(kpps_handle, &pp)) {
        (void)fprintf(stderr,
                      "ERROR: time_pps_setparams(mode=0x%02X) failed: %s(%d)\n",
                      pp.mode, strerror(errno), errno);
        exit(EXIT_FAILURE);
    }

    (void)puts("\n# Assert seq   , Clear seq");

    for (;;) {
        pps_info_t pi;
        struct timespec kpps_tv ;
        char ts_str1[TIMESPEC_LEN], ts_str2[TIMESPEC_LEN];

        if (0 < exit_timer &&
            time(NULL) >= exit_timer) {
            break;
        }
        kpps_tv.tv_sec = 3;   // 3 second timeout
        kpps_tv.tv_nsec = 0;

        memset((void *)&pi, 0, sizeof(pi));    // paranoia
        if (0 > time_pps_fetch(kpps_handle, PPS_TSFMT_TSPEC, &pi, &kpps_tv)) {
            if (ETIMEDOUT == errno ||
                EINTR == errno) {
                // just a timeout
                (void)puts("WARNING: time_pps_fetch() timeout\n");
                continue;
            }

            (void)fprintf(stderr, "ERROR: time_pps_fetch() failed: %s(%d)\n",
                          strerror(errno), errno);
            exit(EXIT_FAILURE);
        }
        (void)printf(" %s %lu, %s %lu\n",
                     timespec_str(&pi.assert_timestamp,
                                  ts_str1, sizeof(ts_str1)),
                     (unsigned long)pi.assert_sequence,
                     timespec_str(&pi.clear_timestamp,
                                  ts_str2, sizeof(ts_str2)),
                     (unsigned long) pi.clear_sequence);
    }

    exit(EXIT_SUCCESS);
}
#endif


static void usage(void)
{
        (void)fprintf(stderr,
        "usage: ppscheck [OPTIONS] <device>\n\n"
#ifdef HAVE_GETOPT_LONG
        "  --help            Show this help, then exit.\n"
        "  --seconds SEC     Exit after SEC seconds delay.\n"
        "  --version         Show version, then exit.\n"
#endif
        "   -?               Show this help, then exit.\n"
        "   -h               Show this help, then exit.\n"
        "   -V               Show version, then exit.\n"
        "   -x SEC           Exit after SEC seconds delay.\n"
        "\n"
        "   <device>         Device to check (/dev/ttyS0, /dev/pps0, etc.).\n");
}

int main(int argc, char *argv[])
{
    int fd;
    int handshakes;
    bool is_tty = false;
    bool has_kpps = false;
    struct timespec ts;
    char ts_buf[TIMESPEC_LEN];
#if defined(HAVE_SYS_TIMEPPS_H)
    // aka RFC2783
    // int pps_fd = -1;
    pps_handle_t kpps_handle;
#endif  // HAVE_SYS_TIMEPPS_H
    const char *optstring = "?hVx:";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"help", no_argument, NULL, 'h'},
        {"seconds", required_argument, NULL, 'x'},
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

        switch(ch){
        case '?':
            FALLTHROUGH
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
        default:
            usage();
            exit(EXIT_FAILURE);
        case 'V':
            (void)printf("%s: %s\n", argv[0], REVISION);
            exit(EXIT_SUCCESS);
        case 'x':
            exit_timer = time(NULL) + strtol(optarg, 0, 0);
            break;
        }
    }

    if ((optind + 1) != argc ||
       '\0' == argv[optind][0]) {
        (void)fputs("ERROR: can't run with no device specified\n", stderr);
        usage();
        exit(EXIT_FAILURE);
    }

    // TIOCM* one need RD, KPPS needs WR
    fd = open(argv[optind], O_RDWR);

    if (-1 == fd) {
        (void)fprintf(stderr, "ERROR: open(%s) failed: %.80s(%d)\n",
                      argv[1], strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
    // check that it is a tty
    if (0 == ioctl(fd, TIOCMGET, &handshakes)) {
        is_tty = true;
    } else {
        (void)fprintf(stderr,
                      "INFO: ioctl(%s, TIOCMGET) failed: %.80s(%d)\n"
                      "INFO: %s does not appear to be a tty\n",
                      argv[1], strerror(errno), errno, argv[1]);
        is_tty = false;
    }
#if defined(HAVE_SYS_TIMEPPS_H)
    // aka RFC2783
    if (0 == time_pps_create(fd, &kpps_handle)) {
        has_kpps = true;
    } else {
        (void)fprintf(stderr,
                      "WARNING: time_pps_create(%s)) failed: %.80s(%d)\n"
                      "WARRING: %s does not appear to be a KPPS device\n",
                      argv[1], strerror(errno), errno, argv[1]);
        has_kpps = false;;
    }
#else
    (void)puts("WARNING: KPPS not compiled in.");
    has_kpps = false;;
#endif  // HAVE_SYS_TIMEPPS_H

    if (!is_tty &&
        !has_kpps) {
        (void)fprintf(stderr,
                      "ERROR: %s is not a tty and does not support KPPS.\n",
                      argv[1]);
        exit(EXIT_FAILURE);
    }

#if defined(HAVE_SYS_TIMEPPS_H)
    // aka RFC2783
    if (!is_tty &&
        has_kpps) {
        do_kpps(kpps_handle);
        // never returns
    }
#endif
    // else is_tty && !has_kpps

    (void)puts("\n# Seconds  nanoSecs   Signals");
    for (;;) {
        const struct assoc *sp;

        if (exit_timer &&
            time(NULL) >= exit_timer) {
            break;
        }

        if (0 != ioctl(fd, TIOCMIWAIT,
                       TIOCM_CD | TIOCM_DSR | TIOCM_RI | TIOCM_CTS)) {
            (void)fprintf(stderr,
                          "ERROR: ioctl(TIOCMIWAIT) failed: %.80s(%d)\n",
                          strerror(errno), errno);
            exit(EXIT_FAILURE);
        }

        (void)clock_gettime(CLOCK_REALTIME, &ts);
        if (0 != ioctl(fd, TIOCMGET, &handshakes)) {
            (void)fprintf(stderr,
                          "ERROR: ioctl(TIOCMGET) failed: %.80s(%d)\n",
                          strerror(errno), errno);
            exit(EXIT_FAILURE);
        }

        (void)fputs(timespec_str(&ts, ts_buf, sizeof(ts_buf)), stdout);
        for (sp = hlines;
             sp < hlines + sizeof(hlines) / sizeof(hlines[0]);
             sp++) {
            if (0 != (handshakes & sp->mask)) {
                (void)fprintf(stdout, "  %s", sp->string);
            }
        }
        (void)puts("");
    }

    exit(EXIT_SUCCESS);
}

// vim: set expandtab shiftwidth=4
