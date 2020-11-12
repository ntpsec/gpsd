/* ntpshmmon.c -- monitor the inner end of an ntpshmwrite.connection
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#ifdef HAVE_GETOPT_LONG
       #include <getopt.h>
#endif
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>           // for memset()
#include <unistd.h>

#include "../include/compiler.h"      // for FALLTHROUGH
#include "../include/gps.h"           // for safe_atof()
#include "../include/ntpshm.h"
#include "../include/timespec.h"
#include "../include/os_compat.h"

#define NTPSEGMENTS     256     /* NTPx for x any byte */

static struct shmTime *segments[NTPSEGMENTS + 1];

static void usage(void)
{
    (void)fprintf(stderr,
        "usage: ntpshmmon [OPTIONS]\n\n"
#ifdef HAVE_GETOPT_LONG
        "  --count COUNT       Exit after COUNT samples\n"
        "  --help              Print this help, then exit\n"
        "  --offset            Replace Seen@ with Offset\n"
        "  --rmshm             Remove SHMs and exit\n"
        "  --seconds SECONDS   Exit after SECONDS seconds\n"
        "  --verbose           Be verbose\n"
        "  --version           Show version, then exit\n"
#endif
        "  -?                  Print this help and exit.\n"
        "  -h                  Print this help and exit.\n"
        "  -n COUNT            Exit after COUNT samples\n"
        "  -o                  Replace Seen@ with Offset\n"
        "  -s                  Remove SHMs and exit\n"
        "  -t SECONDS          Exit after SECONDS seconds\n"
        "  -v                  Be verbose\n"
        "  -V                  Print version and exit.\n"
        );
    exit(EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    int i;
    bool killall = false;
    bool offset = false;            /* show offset, not seen */
    bool verbose = false;
    int nsamples = INT_MAX;
    time_t timeout = (time_t)0, starttime = time(NULL);
    /* a copy of all old segments */
    struct shm_stat_t   shm_stat_old[NTPSEGMENTS + 1];
    char *whoami;
    const char *optstring = "?hn:ost:vV";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"count", required_argument, NULL, 'n'},
        {"help", no_argument, NULL, 'h'},
        {"offset", no_argument, NULL, 'o'},
        {"rmshm", no_argument, NULL, 's'},
        {"seconds", required_argument, NULL, 't'},
        {"verbose", no_argument, NULL, 'v' },
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif

    /* strip path from program name */
    (whoami = strrchr(argv[0], '/')) ? ++whoami : (whoami = argv[0]);

    memset( shm_stat_old, 0 ,sizeof( shm_stat_old));

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
        default:
            // unknown option
            FALLTHROUGH
        case '?':
            FALLTHROUGH
        case 'h':
            usage();
            // never returns but shut up compiler warnings
            break;
        case 'n':
            nsamples = atoi(optarg);
            break;
        case 'o':
            offset = true;
            break;
        case 's':
            killall = true;
            break;
        case 't':
            timeout = (time_t)atoi(optarg);
            break;
        case 'v':
            verbose = true;
            break;
        case 'V':
            (void)fprintf(stderr, "%s: version %s (revision %s)\n",
                          whoami, VERSION, REVISION);
            exit(EXIT_SUCCESS);
        }
    }

    if (optind < argc) {
        (void)fprintf(stderr, "%s: Extra positional arguments: ", whoami);
        while (optind < argc) {
            (void)fprintf(stderr, "  %s", argv[optind++]);
        }
        (void)fprintf(stderr, "\n");
        exit(EXIT_FAILURE);
    }

    /* grab all segments, keep the non-null ones */
    for (i = 0; i < NTPSEGMENTS; i++) {
        segments[i] = shm_get(i, false, true);
        if (verbose && segments[i] != NULL)
            (void)fprintf(stderr, "unit %d opened\n", i);
    }

    if (killall) {
        struct shmTime **pp;

        for (pp = segments; pp < segments + NTPSEGMENTS; pp++)
            if (*pp != NULL)
                (void)shmdt((void *)(*pp));
        exit(EXIT_SUCCESS);
    }

    /*
     * We want line buffering even if stdout is going to a file.  This
     * is a (possibly futile) attempt to avoid writing an incomplete
     * line on interrupt.
     */
    setvbuf(stdout, NULL, _IOLBF, 0);

    (void)printf("%s: version %s\n", whoami, VERSION);
    if (offset) {
        (void)printf("#      Name     Offset            Clock"
                     "                 Real                 L Prc\n");
    } else {
        (void)printf("#      Name  Seen@                 Clock"
                     "                 Real                 L Prc\n");
    }

    do {
        /* the current segment */
        struct shm_stat_t       shm_stat;
        struct timespec delay;
        char ts_buf1[TIMESPEC_LEN];
        char ts_buf2[TIMESPEC_LEN];
        char ts_buf3[TIMESPEC_LEN];

        for (i = 0; i < NTPSEGMENTS; i++) {
            long long diff;  /* 32 bit long is too short for a timespec */
            enum segstat_t status = ntp_read(segments[i], &shm_stat, false);
            if (verbose)
                (void)fprintf(stderr, "unit %d status %d\n", i, status);
            switch(status) {
            case OK:
                /* ntpd can slew the clock at 120% real time
                 * so do not lock out slightly short cycles
                 * use 50% of cycle time as lock out limit.
                 * ignore that system time may jump. */
                diff = timespec_diff_ns(shm_stat.tvr, shm_stat_old[i].tvr);
                if ( 0 == diff) {
                    /* no change in tvr */
                    break;
                }
                diff = timespec_diff_ns(shm_stat.tvt, shm_stat_old[i].tvt);
                if ( 0 == diff) {
                    /* no change in tvt */
                    break;
                }
                /* time stamp it */
                clock_gettime(CLOCK_REALTIME, &shm_stat.tvc);
                if (offset) {
                    diff = timespec_diff_ns(shm_stat.tvr, shm_stat.tvt);
                    printf("sample %s %20.9f %s %s %d %3d\n",
                           ntp_name(i),
                           (double)diff * 1e-9,
                           timespec_str(&shm_stat.tvr, ts_buf1,
                                        sizeof(ts_buf1)),
                           timespec_str(&shm_stat.tvt, ts_buf2,
                                        sizeof(ts_buf2)),
                           shm_stat.leap, shm_stat.precision);
                } else {
                    printf("sample %s %s %s %s %d %3d\n",
                           ntp_name(i),
                           timespec_str(&shm_stat.tvc, ts_buf1,
                                        sizeof(ts_buf1)),
                           timespec_str(&shm_stat.tvr, ts_buf2,
                                        sizeof(ts_buf2)),
                           timespec_str(&shm_stat.tvt, ts_buf3,
                                        sizeof(ts_buf3)),
                           shm_stat.leap, shm_stat.precision);
                }
                --nsamples;
                /* save the new time stamp */
                shm_stat_old[i] = shm_stat; /* structure copy */

                break;
            case NO_SEGMENT:
                break;
            case NOT_READY:
                /* do nothing, data not ready, wait another cycle */
                break;
            case BAD_MODE:
                (void)fprintf(stderr,
                              "ntpshmmon: unknown mode %d on segment %s\n",
                              shm_stat.status, ntp_name(i));
                break;
            case CLASH:
                /* do nothing, data is corrupt, wait another cycle */
                break;
            default:
                (void)fprintf(stderr,
                              "ntpshmmon: unknown status %d on segment %s\n",
                              status, ntp_name(i));
                break;
            }
        }
        /* all segments now checked */

        /*
         * Even on a 1 Hz PPS, a sleep(1) may end up
         * being sleep(1.1) and missing a beat.  Since
         * we're ignoring duplicates via timestamp, polling
         * at fast intervals should not be a problem
         *
         * PPS is not always one pulse per second.
         * the Garmin GPS 18x-5Hz outputs 5 pulses per second.
         * That is a 200 millSec cycle, minimum 20 milliSec duration
         * we will wait 1 milliSec out of caution
         *
         * and, of course, nanosleep() may sleep a lot longer than we ask...
         */
        if ( timeout ) {
            /* do not read time unless it matters */
            if ( time(NULL) > (starttime + timeout ) ) {
                /* time to exit */
                break;
            }
        }

        /* wait 1,000 uSec */
        delay.tv_sec = 0;
        delay.tv_nsec = 1000000L;
        nanosleep(&delay, NULL);
    } while ( 0 < nsamples );

    exit(EXIT_SUCCESS);
}

/* end */
// vim: set expandtab shiftwidth=4
