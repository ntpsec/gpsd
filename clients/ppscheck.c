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

#include <dirent.h>                   // for opendir()
#include <errno.h>
#include <fcntl.h>                    // needed for open() and friends
#ifdef HAVE_GETOPT_LONG
   #include <getopt.h>
#endif
#include <limits.h>          // for PATH_MAX
#ifdef __linux__
   #include <linux/tty.h>             // for N_PPS
#endif
#include <stdio.h>
#include <stdlib.h>                   // for atexit()
#include <string.h>                   // strlcpy(), etc.
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
#include "../include/os_compat.h"    // backup for strlcpy()
#include "../include/timespec.h"

time_t exit_timer = 0;               // for -x option
int device_fd = -1;                  // fd open device
int pps_fd = -1;                     // fd open pps device
#if defined(HAVE_SYS_TIMEPPS_H)
    // aka RFC2783
    pps_handle_t kpps_handle = -1;   // from time_pps_create()
#endif  // HAVE_SYS_TIMEPPS_H
int path_fd = -1;                    // fd for open("sys/X")
DIR *sys_dir = NULL;                 // fd for opendir("sys")
const char *sys_path = "/sys/devices/virtual/pps";

// atexit function
static void myexit(void)
{
    if (0 <= device_fd) {
        close(device_fd);
    }
    if (0 <= pps_fd) {
        close(pps_fd);
    }
    if (0 <= path_fd) {
        close(path_fd);
        path_fd = -1;
    }
    if (NULL != sys_dir) {
        closedir(sys_dir);
    }
#if defined(HAVE_SYS_TIMEPPS_H)
    if (0 <= kpps_handle) {
        time_pps_destroy(kpps_handle);
    }
#endif  // HAVE_SYS_TIMEPPS_H
}

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


/* cfg_pps()
 * shows KPPS caps
 * enable CAPTURES
 *
 * Return: void
 *         exits on error
 */
static void cfg_kpps(void)
{
    int kpps_caps = 0;
    pps_params_t pp;
    const struct assoc *sp;

    // have kernel PPS handle. get RFC2783 features supported
    if (0 > time_pps_getcap(kpps_handle, &kpps_caps)) {
        (void)printf("ERROR: time_pps_getcap() failed: %s(%d)\n",
                     strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
    (void)printf("INFO: kpps_caps 0x%02X\n", kpps_caps);

    for (sp = caps; sp < caps + sizeof(caps) / sizeof(caps[0]); sp++) {
        if (0 != (kpps_caps & sp->mask)) {
            (void)printf("  %s\n", sp->string);
        }
    }
    puts("");
    if (0 == (PPS_CANWAIT & kpps_caps)) {
        (void)puts("ERROR: PPS_CANWAIT is missing.");
    }

    // construct the setparms structure
    memset((void *)&pp, 0, sizeof(pps_params_t));
    pp.api_version = PPS_API_VERS_1;    // version 1 protocol

    switch ((PPS_CAPTUREASSERT | PPS_CAPTURECLEAR) & kpps_caps) {
    case PPS_CAPTUREASSERT:
        (void)puts("WARNING: missing PPS_CAPTURECLEAR, pulse may be offset");
        pp.mode |= PPS_CAPTUREASSERT;
        break;
    case PPS_CAPTURECLEAR:
        (void)puts("WARNING: missing PPS_CAPTUREASSERT, pulse may be offset");
        pp.mode |= PPS_CAPTURECLEAR;
        break;
    case PPS_CAPTUREASSERT | PPS_CAPTURECLEAR:
        pp.mode |= PPS_CAPTUREASSERT | PPS_CAPTURECLEAR;
        break;
    default:
        (void)puts("WARNING: missing PPS_CAPTUREASSERT and PPS_CAPTURECLEAR");
        exit(EXIT_FAILURE);
    }

    if (0 > time_pps_setparams(kpps_handle, &pp)) {
        (void)printf("ERROR: time_pps_setparams(mode=0x%02X) failed: %s(%d)",
                     pp.mode, strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
}

static void do_kpps(void)
{
    time_t last = {0};

    pps_seq_t clear_seq = -1;     // KPPS clear sequence
    pps_seq_t assert_seq = -1;    // KPPS assert sequence
    cfg_kpps();        // get caps, configure KPPS

    (void)puts("\n# Src   Seconds                 Signal    Sequence");

    for (;;) {
        pps_info_t pi;
        struct timespec kpps_tv;
        time_t now;
        char ts_str[TIMESPEC_LEN];

        kpps_tv.tv_sec = 3;   // 3 second timeout
        kpps_tv.tv_nsec = 0;

        memset((void *)&pi, 0, sizeof(pi));    // paranoia
        // wait for an event
        if (0 > time_pps_fetch(kpps_handle, PPS_TSFMT_TSPEC, &pi, &kpps_tv)) {
            if (ETIMEDOUT == errno ||
                EINTR == errno) {
                // just a timeout
                (void)puts("WARNING: time_pps_fetch() timeout\n");
                continue;
            }

            (void)printf("ERROR: time_pps_fetch() failed: %s(%d)\n",
                         strerror(errno), errno);
            exit(EXIT_FAILURE);
        }
        now = time(NULL);
        if (0 < exit_timer &&
            now >= exit_timer) {
            break;
        }
        // new second, output a newline.
        if (last != now) {
            (void)putchar('\n');
            last = now;
        }
        if (pi.assert_sequence != assert_seq) {
            (void)printf("  KPPS %s    assert  %lu\n",
                         timespec_str(&pi.assert_timestamp,
                                      ts_str, sizeof(ts_str)),
                         (unsigned long)pi.assert_sequence);
            assert_seq = pi.assert_sequence;
        }
        if (pi.clear_sequence != clear_seq) {
            (void)printf("  KPPS %s    clear   %lu\n",
                         timespec_str(&pi.clear_timestamp,
                                      ts_str, sizeof(ts_str)),
                         (unsigned long)pi.clear_sequence);
            clear_seq = pi.clear_sequence;
        }
    }

    exit(EXIT_SUCCESS);
}
#endif

/* list_pps() - list pps devices
 * linux specific, OK to just let it fail on other OS
 * /sys/devices/virtual/pps/pps?/
 */
static void list_pps(void)
{
    struct dirent *dp;

    sys_dir = opendir(sys_path);
    if (NULL == sys_dir) {
        (void)printf("ERROR: opendir(%s) failed: %.80s(%d)\n",
                     sys_path, strerror(errno), errno);
        return;
    }
    while (NULL != (dp = readdir(sys_dir))) {
        char name_path[PATH_MAX];
        char tty_path[PATH_MAX];
        ssize_t len;

        if ('.' == dp->d_name[0]) {
            continue;
        }
        (void)printf("INFO: %s  ", dp->d_name);
        (void)snprintf(name_path, sizeof(name_path), "%s/%s/path",
                       sys_path, dp->d_name);
        if (0 <= path_fd) {
            close(path_fd);
            path_fd = -1;
        }
        path_fd = open(name_path, O_RDONLY);
        if (-1 == path_fd) {
            (void)printf("\nERROR: open(%s) failed: %.80s(%d)\n",
                         name_path, strerror(errno), errno);
            continue;
        }
        len = read(path_fd, tty_path, sizeof(tty_path));
        if (-1 == len) {
            (void)printf("\nERROR: read(%s) failed: %.80s(%d)\n",
                         name_path, strerror(errno), errno);
            continue;
        }
        // tty_path ends with \n
        if (0 < len) {
            tty_path[len - 1] = '\0';
        }
        puts(tty_path);
    }
    (void)close(path_fd);
}

/* find_pps() - find pps device that matches a tty device.
 * very similar to list_pps()
 * linux specific, OK to just let it fail on other OS
 * /sys/devices/virtual/pps/pps?/
 *
 * return: pointer to static buffer of answer
 *         NULL on no match
 */
static const char *find_pps(const char *device)
{
    struct dirent *dp;
    static char match[PATH_MAX];

    sys_dir = opendir(sys_path);
    if (NULL == sys_dir) {
        (void)printf("ERROR: opendir(%s) failed: %.80s(%d)\n",
                     sys_path, strerror(errno), errno);
        return NULL;
    }
    while (NULL != (dp = readdir(sys_dir))) {
        char name_path[PATH_MAX];
        char tty_path[PATH_MAX];
        ssize_t len;

        if ('.' == dp->d_name[0]) {
            continue;
        }
        (void)snprintf(name_path, sizeof(name_path), "%s/%s/path",
                       sys_path, dp->d_name);
        if (0 <= path_fd) {
            close(path_fd);
            path_fd = -1;
        }
        path_fd = open(name_path, O_RDONLY);
        if (-1 == path_fd) {
            (void)printf("ERROR: open(%s) failed: %.80s(%d)",
                         name_path, strerror(errno), errno);
            continue;
        }
        len = read(path_fd, tty_path, sizeof(tty_path));
        if (-1 == len) {
            (void)printf("ERROR: read(%s) failed: %.80s(%d)",
                         name_path, strerror(errno), errno);
            continue;
        }
        // tty_path ends with \n
        if (0 < len) {
            tty_path[len - 1] = '\0';
        }
        if (0 == strncmp(device, tty_path, sizeof(tty_path))) {
            (void)strlcpy(match, dp->d_name, sizeof(match));
            return match;
        }
    }
    (void)close(path_fd);
    return NULL;
}

/* do_tty()
 * the main loop for wathing a tty, and optional companion kpps
 *
 * return: Never
 */
static void do_tty(void)
{
#if defined(HAVE_SYS_TIMEPPS_H)
    pps_seq_t clear_seq = -1;     // KPPS clear sequence
    pps_seq_t assert_seq = -1;    // KPPS assert sequence
#endif  // HAVE_SYS_TIMEPPS_H

    int handshakes;
    struct timespec ts;
    time_t last_sec = -1;

    (void)puts("\n# Src   Seconds                 Signals");
    for (;;) {
        const struct assoc *sp;
#if defined(HAVE_SYS_TIMEPPS_H)
        pps_info_t pi;
        struct timespec kpps_tv ;
#endif  // HAVE_SYS_TIMEPPS_H
        char ts_str[TIMESPEC_LEN];

        if (0 < exit_timer &&
            time(NULL) >= exit_timer) {
            break;
        }

        // use TIOCMIWAIT to wait for change
        // no way to set a timeout on this ioctl()
        if (0 != ioctl(device_fd, TIOCMIWAIT,
                       TIOCM_CD | TIOCM_DSR | TIOCM_RI | TIOCM_CTS)) {
            (void)printf("ERROR: ioctl(TIOCMIWAIT) failed: %.80s(%d)\n",
                         strerror(errno), errno);
            exit(EXIT_FAILURE);
        }

        (void)clock_gettime(CLOCK_REALTIME, &ts);  // quick, grab current time

        // figure out what changed
        // look into TIOCGICOUNT instead?
        if (0 != ioctl(device_fd, TIOCMGET, &handshakes)) {
            (void)printf("ERROR: ioctl(TIOCMGET) failed: %.80s(%d)\n",
                         strerror(errno), errno);
            exit(EXIT_FAILURE);
        }
        if (last_sec != ts.tv_sec) {
            // new second, new line
            (void)putchar('\n');
            last_sec = ts.tv_sec;
        }

#if defined(HAVE_SYS_TIMEPPS_H)
        if (0 <= kpps_handle) {
            kpps_tv.tv_sec = 0;   // non-blocking
            kpps_tv.tv_nsec = 0;
            bool good_pi;

            good_pi = true;
            memset((void *)&pi, 0, sizeof(pi));    // paranoia
            if (0 > time_pps_fetch(kpps_handle, PPS_TSFMT_TSPEC,
                                   &pi, &kpps_tv)) {
                (void)printf("ERROR: time_pps_fetch() failed: %s(%d)\n",
                             strerror(errno), errno);
                good_pi = false;
            }

            // print KPPS first, as its timestamp will be before
            // TIOCMIWAIT time
            if (good_pi) {
                if (pi.assert_sequence != assert_seq) {
                    (void)printf("  KPPS %s    assert  %lu\n",
                                 timespec_str(&pi.assert_timestamp,
                                              ts_str, sizeof(ts_str)),
                                 (unsigned long)pi.assert_sequence);
                    assert_seq = pi.assert_sequence;
                }
                if (pi.clear_sequence != clear_seq) {
                    (void)printf("  KPPS %s    clear   %lu\n",
                                 timespec_str(&pi.clear_timestamp,
                                              ts_str, sizeof(ts_str)),
                                 (unsigned long)pi.clear_sequence);
                    clear_seq = pi.clear_sequence;
                }
            }
        }
#endif   // HAVE_SYS_TIMEPPS_H
        (void)printf("  TTY  %s  ",
                     timespec_str(&ts, ts_str, sizeof(ts_str)));
        for (sp = hlines;
             sp < hlines + sizeof(hlines) / sizeof(hlines[0]);
             sp++) {
            if (0 != (handshakes & sp->mask)) {
                (void)printf("  %s", sp->string);
            }
        }
        (void)putchar('\n');
    }
    exit(EXIT_SUCCESS);
}


static void usage(void)
{
        (void)printf(
        "usage: ppscheck [OPTIONS] <device>\n\n"
#ifdef HAVE_GETOPT_LONG
        "  --help            Show this help, then exit.\n"
        "  --pps             List pps devices active.\n"
        "  --seconds SEC     Exit after SEC seconds delay.\n"
        "  --version         Show version, then exit.\n"
#endif
        "   -?               Show this help, then exit.\n"
        "   -h               Show this help, then exit.\n"
        "   -m               Find pps device that matches <device>\n"
        "   -p               List pps devices active.\n"
        "   -V               Show version, then exit.\n"
        "   -x SEC           Exit after SEC seconds delay.\n"
        "\n"
        "   <device>         Device to check (/dev/ttyS0, /dev/pps0, etc.).\n");
}

int main(int argc, char *argv[])
{
#ifdef __linux__
    int ldisc = 0;     // tty line discipline
#endif
    int handshakes;
    bool is_tty = false;
    bool has_kpps = false;
    bool find_kpps = false;
    const char *kpps_name = NULL;    // logical name of pps device (pps0, etc.).
    char kpps_path[PATH_MAX] = "";   // full path to devined kpps device
    char *device = NULL;          // pointer to <device> name
    char device_real[PATH_MAX];   // realname() of <device>
    const char *optstring = "?hmpVx:";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"help", no_argument, NULL, 'h'},
        {"match", no_argument, NULL, 'm'},
        {"pps", no_argument, NULL, 'p'},
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
        case 'm':
            find_kpps = true;
            break;
        case 'p':
            list_pps();
            exit(EXIT_SUCCESS);
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
        (void)puts("ERROR: can't run with no device specified");
        usage();
        exit(EXIT_FAILURE);
    }

    atexit(myexit);

    device = realpath(argv[optind], device_real);;
    if (NULL == device) {
        (void)printf("ERROR: realpath(%s) failed: %.80s(%d)\n",
                     argv[optind], strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
    if (0 != strncmp(device_real, argv[optind], sizeof(device_real))) {
        (void)printf("INFO: %s is a symlink to %s\n",
                     argv[optind], device);
    }

    // handle -p option
    if (find_kpps) {
        kpps_name = find_pps(device);
        if (NULL == kpps_name) {
            (void)printf("INFO: pps for %s not found\n", device);
        } else {
            (void)printf("INFO: %s uses %s\n", device, kpps_name);
        }
        exit(EXIT_SUCCESS);
    }

    // TIOCM* only needs RD, KPPS needs WR
    device_fd = open(device, O_RDWR);

    if (-1 == device_fd) {
        (void)printf("ERROR: open(%s) failed: %.80s(%d)\n",
                     argv[1], strerror(errno), errno);
        exit(EXIT_FAILURE);
    }
    // check that it is a tty
    if (0 == ioctl(device_fd, TIOCMGET, &handshakes)) {
        is_tty = true;
#ifdef __linux__
        /* check current line discipline
         * if (0 == ioctl(device_fd, TIOCGETD, &ldisc)) {
         * always returns ldisc == 0 */
        // set PPS line discipline
        ldisc = N_PPS;    // 18 - the PPS line discipline
        if (0 > ioctl(device_fd, TIOCSETD, &ldisc)) {
            (void)printf("ERROR: ioctl(%s, TIOCSETD, 18) failed: %.80s(%d)\n",
                         argv[1], strerror(errno), errno);
        }
        // try to find matching kpps device
        kpps_name = find_pps(device);
        if (NULL != kpps_name) {
            (void)snprintf(kpps_path, sizeof(kpps_path), "/dev/%s", kpps_name);
        }
#endif  // __linux__
    } else {
        (void)printf("INFO: ioctl(%s, TIOCMGET) failed: %.80s(%d)\n"
                     "INFO: %s does not appear to be a tty\n",
                     argv[1], strerror(errno), errno, argv[1]);
        is_tty = false;
    }

#if defined(HAVE_SYS_TIMEPPS_H)
    // aka RFC2783
    if (0 == time_pps_create(device_fd, &kpps_handle)) {
        has_kpps = true;
    } else {
        (void)printf("WARNING: time_pps_create(%s)) failed: %.80s(%d)\n"
                     "WARRING: %s does not appear to be a KPPS device\n",
                      device, strerror(errno), errno, device);
        if ('\0' != kpps_path[0]) {
            // try kpps_path
            pps_fd = open(kpps_path, O_RDWR);

            if (-1 == pps_fd) {
                (void)printf("WARNING: open(%s) failed: %.80s(%d)\n",
                             kpps_path, strerror(errno), errno);
            } else {
                (void)printf("INFO: matching %s opened\n", kpps_path);
                if (0 == time_pps_create(pps_fd, &kpps_handle)) {
                    has_kpps = true;
                } else {
                    (void)printf(
                        "WARNING: time_pps_create(%s)) failed: %.80s(%d)\n"
                        "WARRING: %s does not appear to be a KPPS device\n",
                        kpps_path, strerror(errno), errno, device);
                }
            }
        }
    }

    if (!is_tty &&
        has_kpps) {
        do_kpps();
        // never returns
    }
#else
    (void)puts("WARNING: KPPS not compiled in.");
#endif

    if (!is_tty &&
        !has_kpps) {
        (void)printf("ERROR: %s is not a tty and does not support KPPS.\n",
                     argv[1]);
        exit(EXIT_FAILURE);
    }
    // else is_tty && !has_kpps
    do_tty();
    // never returns

    // how could this happen?
    exit(EXIT_FAILURE);
}

// vim: set expandtab shiftwidth=4
