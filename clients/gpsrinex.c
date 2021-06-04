/*
 * gpsrinex: read "RAW" messages from a gpsd and output a RINEX 3 obs file.
 *
 * gpsrinex will read live data from gpsd and create a file of RINEX 3
 * observations.  Currently this only works if the GPS is a u-blox
 * GPS and is sending UBX-RXM-RAWX messages.
 *
 * The u-blox must be configured for u-blox binary messages.  GLONASS,
 * GALILEO, and BEIDOU must be off.  Optionally SBAS on, but can be
 * flakey.
 *
 * Too much data for 9600!
 *
 * To configure a u-blox to output the proper data:
 *    # gpsctl -s 115200
 *    # sleep 2
 *    # ubxtool -d NMEA
 *    # ubxtool -e BINARY
 *    # ubxtool -d GLONASS
 *    # ubxtool -d BEIDOU
 *    # ubxtool -d GALILEO
 *    # ubxtool -d SBAS
 *    # ubxtool -e RAWX
 *
 * If you have a u-blox 9 then enable GLONASS as well.
 *
 * After collecting the default number of observations, gpsrinex will
 * create the RINEX .obs file and exit.  Upload this file to an
 * offline processing service to get cm accuracy.
 *
 * One service known to work with obsrinex output is [CSRS-PPP]:
 *  https://webapp.geod.nrcan.gc.ca/geod/tools-outils/ppp.php
 *
 * Examples:
 *    To collect 4 hours of samples as 30 second intervals:
 *        # gpsrinex -i 30 -n 480
 *
 * To generate RINEX 3 from a u-blox capture file:
 *     Grab 4 hours of raw live data:
 *         # gpspipe -x 14400 -R > 4h-raw.ubx
 *     Feed that data to gpsfake:
 *         # gpsfake -1 -P 3000 4h-raw.ubx
 *     In another window, convert that raw to RINEX 3:
 *         # gpsrinex -i 1 -n 1000000
 *
 * See also:
 *     [1] RINEX: The Receiver Independent Exchange Format, Version 3.04
 *     ftp://igs.org/pub/data/format/rinex304.pdf
 *
 *     [2] GPSTk, http://www.gpstk.org/
 *
 *     [3] Nischan, Thomas (2016):
 *     GFZRNX - RINEX GNSS Data Conversion and Manipulation Toolbox.
 *     GFZ  Data Services.  http://dx.doi.org/10.5880/GFZ.1.1.2016.002
 *
 *     [4] RTKLIB: An Open Source Program Package for GNSS Positioning
 *     http://www.rtklib.com/
 *
 * This file is Copyright 2018 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <assert.h>
#include <errno.h>
#include <libgen.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>    // for umask()
#include <sys/stat.h>     // for umask()
#include <time.h>
#include <unistd.h>

#ifdef HAVE_GETOPT_LONG
       #include <getopt.h>
#endif

#include "../include/compiler.h"
#include "../include/gps.h"
#include "../include/gpsdclient.h"
#include "../include/os_compat.h"
#include "../include/timespec.h"

static char *progname;
static struct fixsource_t source;
static double ecefx = 0.0;
static double ecefy = 0.0;
static double ecefz = 0.0;
static timespec_t start_time = {0};      /* report gen time, UTC */
static timespec_t first_mtime = {0};     /* GPS time, not UTC */
static timespec_t last_mtime = {0};      /* GPS time, not UTC */
static int leap_seconds = 0;             // set if non-zero

// strings for the RINEX file
static char agency[41] = "Unknown";
static char ant_num[21] = "0";
static char ant_type[21] = "UNKNOWN EXT     NONE";
static double ant_e = 0.0;
static double ant_h = 0.0;
static double ant_n = 0.0;
static char marker_name[61] = "XXXX";
static char marker_type[61] = "NON_PHYSICAL";
static char observer[21] = "Unknown";
static char rec_num[21] = "0";
static char rec_type[21] = "Unknown";
static char rec_vers[21] = "0";

/* total count of observations by u-blox gnssid [0-7]
 *  0 = GPS       RINEX G
 *  1 = SBAS      RINEX S
 *  2 = Galileo   RINEX E
 *  3 - BeiDou    RINEX C
 *  4 = IMES      not supported by RINEX
 *  5 = QZSS      RINEX J
 *  6 = GLONASS   RINEX R
 *  7 = IRNSS     RINEX I
 *
 * RINEX 3 observation codes [1]:
 * C1C  L1 C/A Pseudorange
 * C1P  L1 P Pseudorange
 * C1W  L1 Z-tracking Pseudorange
 * D1C  L1 C/A Doppler
 * L1C  L1 C/A Carrier Phase
 * L1P  L1 P Carrier Phase
 * L1W  L1 Z-tracking Carrier Phase
 * C2C  L2 C/A Pseudorange
 * C2P  L2 P Pseudorange
 * C2W  L2 Z-tracking Pseudorange
 * D2C  L2 C/A Doppler
 * L2C  L2 C/A Carrier phase
 * L2P  L1 P Carrier Phase
 * L2W  L2 Z-tracking Carrier Phase
 *
 * C2L  L2C (L), Pseudo Range, BeiDou
 * D2L  L2C (L), Doppler, BeiDou
 * L2L  L2C (L), Carrier Phase, BeiDou
 *
 * L5I  L5 I Pseudo Range
 * C5I  L5 I Carrier Phase
 * D5I  L5 I Doppler
 *
 * CSRS-PPP supports:
 * GPS:      C1C  L1C  C2C  L2C  C1W  L1W  C2W  L2W
 * GLONASS : C1C  L1C  C2C  L2C  C1P  L1P  C2P  L2P
 *
 */
typedef enum {C1C = 0, D1C, L1C,
              C2C, D2C, L2C,
              C2L, D2L, L2L,
              C5I, D5I, L5I,
              C7I, D7I, L7I,
              C7Q, D7Q, L7Q, CODEMAX} obs_codes;
/* structure to hold count of observations by gnssid:svid
 * MAXCHANNEL+1 is just a WAG of max size */
#define MAXCNT (MAXCHANNELS + 1)
static struct obs_cnt_t {
        unsigned char gnssid;
        unsigned char svid;     /* svid of 0 means unused slot */
        unsigned int obs_cnts[CODEMAX+1];    /* count of obscode */
} obs_cnt[MAXCNT] = {{0}};

static FILE * tmp_file;             /* file handle for temp file */
static int sample_count = 20;       /* number of measurement sets to get */
/* timespec_t between measurement sets */
static timespec_t sample_interval_ts = {30, 0};
/* milli-seconds between measurement sets */
static unsigned  sample_interval_ms = 30000;

#define DEBUG_QUIET 0
#define DEBUG_INFO 1
#define DEBUG_PROG 2
#define DEBUG_RAW 3
static int debug = DEBUG_INFO;               /* debug level */

static struct gps_data_t gpsdata;
static FILE *log_file;

/* convert a u-blox/gpsd gnssid to the RINEX 3 constellation code
 * see [1] Section 3.5
 */
static char gnssid2rinex(int gnssid)
{
    switch (gnssid) {
    case GNSSID_GPS:      /* 0 = GPS */
        return 'G';
    case GNSSID_SBAS:     /* 1 = SBAS */
        return 'S';
    case GNSSID_GAL:      /* 2 = Galileo */
        return 'E';
    case GNSSID_BD:       /* 3 = BeiDou */
        return 'C';
    case GNSSID_IMES:     /* 4 = IMES - unsupported */
        return 'X';
    case GNSSID_QZSS:     /* 5 = QZSS */
        return 'J';
    case GNSSID_GLO:      /* 6 = GLONASS */
        return 'R';
    case GNSSID_IRNSS:    /* 7 = IRNSS */
        return 'I';
    default:    /* Huh? */
        return 'x';
    }
}

/* obs_cnt_inc()
 *
 * increment an observation count
 */
static void obs_cnt_inc(unsigned char gnssid, unsigned char svid,
                        obs_codes obs_code)
{
    int i;

    if (CODEMAX <= obs_code) {
        /* should never happen... */
        fprintf(stderr, "ERROR: obs_code_inc() obs_code %d out of range\n",
                obs_code);
        exit(1);
    }
    /* yeah, slow and ugly, linear search. */
    for (i = 0; i < MAXCNT; i++) {
        if (0 == obs_cnt[i].svid) {
            /* end of list, not found, so add this gnssid:svid */
            obs_cnt[i].gnssid = gnssid;
            obs_cnt[i].svid = svid;
            obs_cnt[i].obs_cnts[obs_code] = 1;
            break;
        }
        if (obs_cnt[i].gnssid != gnssid) {
            continue;
        }
        if (obs_cnt[i].svid != svid) {
            continue;
        }
        /* found it, increment it */
        obs_cnt[i].obs_cnts[obs_code]++;
        if (99999 < obs_cnt[i].obs_cnts[obs_code]) {
            /* RINEX 3 max is 99999 */
            obs_cnt[i].obs_cnts[obs_code] = 99999;
        }
        break;
    }
    /* fell out because table full, item added, or item incremented */
    return;
}

/* compare two obs_cnt, for sorting by gnssid, and svid */
static int compare_obs_cnt(const void  *A, const void  *B)
{
    const struct obs_cnt_t *a = (const struct obs_cnt_t *)A;
    const struct obs_cnt_t *b = (const struct obs_cnt_t *)B;
    unsigned char a_gnssid = a->gnssid;
    unsigned char b_gnssid = b->gnssid;

    /* 0 = svid means unused, make those last */
    if (0 == a->svid) {
        a_gnssid = 255;
    }
    if (0 == b->svid) {
        b_gnssid = 255;
    }
    if (a_gnssid != b_gnssid) {
        return a_gnssid - b_gnssid;
    }
    /* put unused last */
    if (a->svid != b->svid) {
        return a->svid - b->svid;
    }
    /* two blank records */
    return 0;
}

/* return number of unique PRN in a gnssid from obs_cnt.
 * return all PRNs if 255 == gnssid */
static int obs_cnt_prns(unsigned char gnssid)
{
    int i;
    int prn_cnt = 0;

    for (i = 0; i < MAXCNT; i++) {
        if (0 == obs_cnt[i].svid) {
            /* end of list, done */
            break;
        }
        if ((255 != gnssid) && (gnssid != obs_cnt[i].gnssid)) {
            /* wrong gnssid */
            continue;
        }
        prn_cnt++;
    }
    /* fell out because table full, item added, or item incremented */
    return prn_cnt;
}

/* print_rinex_header()
 * Print a RINEX 3 header to the file "log_file".
 * Some of the data in the header is only known after processing all
 * the raw data.
 */
static void print_rinex_header(void)
{
    int i, j;
    char tmstr[40];              /* time: yyyymmdd hhmmss UTC */
    struct tm *report_time;
    struct tm *first_time;
    struct tm *last_time;
    struct tm tm_buf;            // temp buffer for gmtime_r()
    int prn_count[GNSSID_CNT] = {0};   /* count of PRN per gnssid */

    if (DEBUG_PROG <= debug) {
        (void)fprintf(stderr, "doing header\n");
    }

    report_time = gmtime_r(&(start_time.tv_sec), &tm_buf);
    (void)strftime(tmstr, sizeof(tmstr), "%Y%m%d %H%M%S UTC", report_time);

    (void)fprintf(log_file,
        "%9s%11s%-20s%-20s%-20s\n",
        "3.03", "", "OBSERVATION DATA", "M: Mixed", "RINEX VERSION / TYPE");
    (void)fprintf(log_file,
        "%-20s%-20s%-20s%-20s\n",
        "gpsrinex " VERSION, "", tmstr,
        "PGM / RUN BY / DATE");
    (void)fprintf(log_file, "%-60s%-20s\n",
         "Source: gpsd live data", "COMMENT");
    (void)fprintf(log_file, "%-60s%-20s\n", marker_name, "MARKER NAME");
    (void)fprintf(log_file, "%-60s%-20s\n", marker_type, "MARKER TYPE");
    (void)fprintf(log_file, "%-20s%-40s%-20s\n",
                  observer, agency, "OBSERVER / AGENCY");
    (void)fprintf(log_file, "%-20s%-20s%-20s%-20s\n",
                  rec_num, rec_type, rec_vers, "REC # / TYPE / VERS");
    (void)fprintf(log_file, "%-20s%-20s%-20s%-20s\n",
                  ant_num, ant_type, "" , "ANT # / TYPE");
    if (isfinite(ecefx) &&
        isfinite(ecefy) &&
        isfinite(ecefz)) {
        (void)fprintf(log_file, "%14.4f%14.4f%14.4f%18s%-20s\n",
            ecefx, ecefy, ecefz, "", "APPROX POSITION XYZ");
    } else if (DEBUG_INFO <= debug) {
        (void)fprintf(stderr, "INFO: missing ECEF\n");
    }

    (void)fprintf(log_file, "%14.4f%14.4f%14.4f%18s%-20s\n",
        ant_h, ant_e, ant_n, "", "ANTENNA: DELTA H/E/N");
    (void)fprintf(log_file, "%6d%6d%48s%-20s\n", 1, 1,
         "", "WAVELENGTH FACT L1/2");

    /* get PRN stats */
    qsort(obs_cnt, MAXCNT, sizeof(struct obs_cnt_t), compare_obs_cnt);
    for (i = 0; i < GNSSID_CNT; i++ ) {
        prn_count[i] = obs_cnt_prns(i);
    }
    /* CSRS-PPP needs C1C, L1C or C1C, L1C, D1C
     * CSRS-PPP refuses files with L1C first
     * convbin wants C1C, L1C, D1C
     * for some reason gfzrnx_lx wants C1C, D1C, L1C, not C1C, L1C, D1C */
    if (0 < prn_count[GNSSID_GPS]) {
        /* GPS, code G */
        (void)fprintf(log_file, "%c%5d%4s%4s%4s%4s%4s%4s%4s%4s%22s%-20s\n",
             gnssid2rinex(GNSSID_GPS), 6, "C1C", "L1C", "D1C", "C2C", "L2C",
             "D2C", "", "", "", "SYS / # / OBS TYPES");
    }
    if (0 < prn_count[GNSSID_SBAS]) {
        /* SBAS, L1 and L5 only, code S */
        (void)fprintf(log_file, "%c%5d%4s%4s%4s%4s%4s%4s%4s%4s%22s%-20s\n",
             gnssid2rinex(GNSSID_SBAS), 3, "C1C", "L1C", "D1C", "", "", "",
             "", "", "", "SYS / # / OBS TYPES");
    }
    if (0 < prn_count[GNSSID_GAL]) {
        /* Galileo, E1, E5 aand E6 only, code E  */
        (void)fprintf(log_file, "%c%5d%4s%4s%4s%4s%4s%4s%4s%4s%22s%-20s\n",
             gnssid2rinex(GNSSID_GAL), 6, "C1C", "L1C", "D1C", "C7Q",
             "L7Q", "D7Q", "", "", "", "SYS / # / OBS TYPES");
    }
    if (0 < prn_count[GNSSID_BD]) {
        /* BeiDou, BDS, code C */
        (void)fprintf(log_file, "%c%5d%4s%4s%4s%4s%4s%4s%4s%4s%22s%-20s\n",
             gnssid2rinex(GNSSID_BD), 6, "C1C", "L1C", "D1C", "C7I", "L7I",
             "D7I", "", "", "", "SYS / # / OBS TYPES");
    }
    if (0 < prn_count[GNSSID_QZSS]) {
        /* QZSS, code J */
        (void)fprintf(log_file, "%c%5d%4s%4s%4s%4s%4s%4s%4s%4s%22s%-20s\n",
             gnssid2rinex(GNSSID_QZSS), 6, "C1C", "L1C", "D1C", "C2L",
             "L2L", "D2L", "", "", "", "SYS / # / OBS TYPES");
    }
    if (0 < prn_count[GNSSID_GLO]) {
        /* GLONASS, R */
        (void)fprintf(log_file, "%c%5d%4s%4s%4s%4s%4s%4s%4s%4s%22s%-20s\n",
             gnssid2rinex(GNSSID_GLO), 6, "C1C", "L1C", "D1C", "C2C", "L2C",
             "D2C", "", "", "", "SYS / # / OBS TYPES");
    }
    // FIXME: Add IRNSS...

    (void)fprintf(log_file, "%6d%54s%-20s\n", obs_cnt_prns(255),
                  "", "# OF SATELLITES");

    /* get all the PRN / # OF OBS */
    for (i = 0; i < MAXCNT; i++) {
        int cnt = 0;                     /* number of obs for one sat */

        if (0 == obs_cnt[i].svid) {
            /* done */
            break;
        }
        for (j = 0; j < CODEMAX; j++) {
            cnt += obs_cnt[i].obs_cnts[j];
        }
        if (0 > cnt) {
            /* no counts for this sat */
            continue;
        }
        switch (obs_cnt[i].gnssid) {
        case GNSSID_GPS:
            /* GPS, code G */
            (void)fprintf(log_file,"   %c%02d%6u%6u%6u%6u%6u%6u%18s%-20s\n",
                          gnssid2rinex(obs_cnt[i].gnssid), obs_cnt[i].svid,
                          obs_cnt[i].obs_cnts[C1C],
                          obs_cnt[i].obs_cnts[L1C],
                          obs_cnt[i].obs_cnts[D1C],
                          obs_cnt[i].obs_cnts[C2C],
                          obs_cnt[i].obs_cnts[L2C],
                          obs_cnt[i].obs_cnts[D2C],
                          "", "PRN / # OF OBS");
            break;
        case GNSSID_SBAS:
            /* SBAS, L1C and L5C, code S */
            (void)fprintf(log_file,"   %c%02d%6u%6u%6u%6u%6u%6u%18s%-20s\n",
                          gnssid2rinex(obs_cnt[i].gnssid), obs_cnt[i].svid,
                          obs_cnt[i].obs_cnts[C1C],
                          obs_cnt[i].obs_cnts[L1C],
                          obs_cnt[i].obs_cnts[D1C],
                          obs_cnt[i].obs_cnts[C5I],
                          obs_cnt[i].obs_cnts[L5I],
                          obs_cnt[i].obs_cnts[D5I],
                          "", "PRN / # OF OBS");
            break;
        case GNSSID_GAL:
            /* Galileo, code E */
            (void)fprintf(log_file,"   %c%02d%6u%6u%6u%6u%6u%6u%18s%-20s\n",
                          gnssid2rinex(obs_cnt[i].gnssid), obs_cnt[i].svid,
                          obs_cnt[i].obs_cnts[C1C],
                          obs_cnt[i].obs_cnts[L1C],
                          obs_cnt[i].obs_cnts[D1C],
                          obs_cnt[i].obs_cnts[C7Q],
                          obs_cnt[i].obs_cnts[L7Q],
                          obs_cnt[i].obs_cnts[D7Q],
                          "", "PRN / # OF OBS");
            break;
        case GNSSID_BD:
            /* BeiDou, code C */
            (void)fprintf(log_file,"   %c%02d%6u%6u%6u%6u%6u%6u%18s%-20s\n",
                          gnssid2rinex(obs_cnt[i].gnssid), obs_cnt[i].svid,
                          obs_cnt[i].obs_cnts[C1C],
                          obs_cnt[i].obs_cnts[L1C],
                          obs_cnt[i].obs_cnts[D1C],
                          obs_cnt[i].obs_cnts[C7I],
                          obs_cnt[i].obs_cnts[L7I],
                          obs_cnt[i].obs_cnts[D7I],
                          "", "PRN / # OF OBS");
            break;
        case GNSSID_QZSS:
            /* QZSS, code J */
            (void)fprintf(log_file,"   %c%02d%6u%6u%6u%6u%6u%6u%18s%-20s\n",
                          gnssid2rinex(obs_cnt[i].gnssid), obs_cnt[i].svid,
                          obs_cnt[i].obs_cnts[C1C],
                          obs_cnt[i].obs_cnts[L1C],
                          obs_cnt[i].obs_cnts[D1C],
                          obs_cnt[i].obs_cnts[C2L],
                          obs_cnt[i].obs_cnts[L2L],
                          obs_cnt[i].obs_cnts[D2L],
                          "", "PRN / # OF OBS");
            break;
        case GNSSID_GLO:
            /* GLONASS, code R */
            (void)fprintf(log_file,"   %c%02d%6u%6u%6u%6u%6u%6u%18s%-20s\n",
                          gnssid2rinex(obs_cnt[i].gnssid), obs_cnt[i].svid,
                          obs_cnt[i].obs_cnts[C1C],
                          obs_cnt[i].obs_cnts[L1C],
                          obs_cnt[i].obs_cnts[D1C],
                          obs_cnt[i].obs_cnts[C2C],
                          obs_cnt[i].obs_cnts[L2C],
                          obs_cnt[i].obs_cnts[D2C],
                          "", "PRN / # OF OBS");
            break;
        // FIXME: Add GNSSID_IRNSS, L5A
        default:
            (void)fprintf(log_file,"   %c%02d%6u%6u%6u%6s%6s%24s%-20s\n",
                          gnssid2rinex(obs_cnt[i].gnssid), obs_cnt[i].svid,
                          obs_cnt[i].obs_cnts[C1C],
                          obs_cnt[i].obs_cnts[L1C],
                          obs_cnt[i].obs_cnts[D1C],
                          "", "",
                          "", "PRN / # OF OBS");
        }
    }

    (void)fprintf(log_file, "%-10s%50s%-20s\n",
                  "DBHZ", "", "SIGNAL STRENGTH UNIT");
    (void)fprintf(log_file, "%10.3f%50s%-20s\n",
                  (double)sample_interval_ms / 1000.0, "", "INTERVAL");

    /* GPS time not UTC */
    first_time = gmtime_r(&(first_mtime.tv_sec), &tm_buf);
    (void)fprintf(log_file, "%6d%6d%6d%6d%6d%5d.%07ld%8s%9s%-20s\n",
         first_time->tm_year + 1900,
         first_time->tm_mon + 1,
         first_time->tm_mday,
         first_time->tm_hour,
         first_time->tm_min,
         first_time->tm_sec,
         (long)(first_mtime.tv_nsec / 100),
         "GPS", "",
         "TIME OF FIRST OBS");

    /* GPS time not UTC */
    last_time = gmtime_r(&(last_mtime.tv_sec), &tm_buf);
    (void)fprintf(log_file, "%6d%6d%6d%6d%6d%5d.%07ld%8s%9s%-20s\n",
         last_time->tm_year + 1900,
         last_time->tm_mon + 1,
         last_time->tm_mday,
         last_time->tm_hour,
         last_time->tm_min,
         last_time->tm_sec,
         (long)(last_mtime.tv_nsec / 100),
         "GPS", "",
         "TIME OF LAST OBS");

    if (0 < prn_count[GNSSID_GPS]) {
        /* GPS, code G */
        (void)fprintf(log_file, "%-60s%-20s\n",
             "G L1C", "SYS / PHASE SHIFT");
        (void)fprintf(log_file, "%-60s%-20s\n",
             "G L2C", "SYS / PHASE SHIFT");
    }
    if (0 < prn_count[GNSSID_SBAS]) {
        /* SBAS, L1 and L5 only, code S */
        (void)fprintf(log_file, "%-60s%-20s\n",
             "S L1C", "SYS / PHASE SHIFT");
        (void)fprintf(log_file, "%-60s%-20s\n",
             "E L5Q", "SYS / PHASE SHIFT");
    }
    if (0 < prn_count[GNSSID_GAL]) {
        /* GALILEO, E1, E5 and E6, code E */
        (void)fprintf(log_file, "%-60s%-20s\n",
             "E L1C", "SYS / PHASE SHIFT");
        (void)fprintf(log_file, "%-60s%-20s\n",
             "E L7Q", "SYS / PHASE SHIFT");
    }
    if (0 < prn_count[GNSSID_BD]) {
        /* BeiDou, code C */
        (void)fprintf(log_file, "%-60s%-20s\n",
             "B L1C", "SYS / PHASE SHIFT");
        (void)fprintf(log_file, "%-60s%-20s\n",
             "B L7I", "SYS / PHASE SHIFT");
    }
    if (0 < prn_count[GNSSID_QZSS]) {
        /* QZSS, code J */
        (void)fprintf(log_file, "%-60s%-20s\n",
             "J L1C", "SYS / PHASE SHIFT");
        (void)fprintf(log_file, "%-60s%-20s\n",
             "J L2L", "SYS / PHASE SHIFT");
    }
    if (0 < prn_count[GNSSID_GLO]) {
        /* GLONASS, code R */
        (void)fprintf(log_file, "%-60s%-20s\n",
             "R L1C", "SYS / PHASE SHIFT");
        (void)fprintf(log_file, "%-60s%-20s\n",
             "R L2C", "SYS / PHASE SHIFT");
    }
    (void)fprintf(log_file, "%-60s%-20s\n",
         "", "END OF HEADER");
    if (DEBUG_PROG <= debug) {
        (void)fprintf(stderr,"done header\n");
    }
    return;
}

/* print_rinex_footer()
 * print a RINEX 3 footer to the file "log_file".
 * Except RINEX 3 has no footer.  So what this really does is
 * call the header function, then move the processed observations from
 * "tmp_file" to "log_file".
 */
static void print_rinex_footer(void)
{
    char buffer[4096];

    /* print the header */
    print_rinex_header();
    /* now replay the data in the tmp_file into the output */
    (void)fflush(tmp_file);
    rewind(tmp_file);
    while (true) {
        size_t count;

        count = fread(buffer, 1, sizeof(buffer), tmp_file);
        if (0 == count ) {
            // nothing read, or read error
            break;
        }
        (void)fwrite(buffer, 1, count, log_file);
    }
    (void)fclose(tmp_file);
    (void)fclose(log_file);
    (void)gps_close(&gpsdata);
}

/* compare two meas_t, for sorting by gnssid, svid, and sigid */
static int compare_meas(const void  *A, const void  *B)
{
    const struct meas_t *a = (const struct meas_t*)A;
    const struct meas_t *b = (const struct meas_t*)B;

    if (a->gnssid != b->gnssid) {
        return a->gnssid - b->gnssid;
    }
    if (a->svid != b->svid) {
        return a->svid - b->svid;
    }
    if (a->sigid != b->sigid) {
        return a->sigid - b->sigid;
    }
    /* two blank records */
    return 0;
}


/* convert an observation item and return it as a (F14,3,I1,I1)
 * in a static buffer */
static const char * fmt_obs(double val, unsigned char lli, unsigned char snr)
{
    static char buf[20];
    char lli_c;         /* set zero lli to blank */
    char snr_c;         /* set zero snr to blank */

    if (!isfinite(val)) {
        /* bad value, return 16 blanks */
        return "                ";
    }
    switch (lli) {
    case 0:
    default:
        lli_c = ' ';
        break;
    case 1:
        lli_c = '1';
        break;
    case 2:
        lli_c = '2';
        break;
    case 3:
        lli_c = '3';
        break;
    }
    if ((1 > snr) || (9 < snr)) {
        snr_c = ' ';
    } else {
        snr_c = 48 + snr;
    }
    (void)snprintf(buf, sizeof(buf), "%14.3f%c%1c", val, lli_c, snr_c);
    return buf;
}

/* one_sig() - print one signal
 *
 * one CxC s LxC DxC
 */
static void one_sig(struct meas_t *meas)
{
    unsigned char snr;
    unsigned gnssid = meas->gnssid;
    unsigned svid = meas->svid;
    unsigned sigid = meas->sigid;
    obs_codes cxx = C1C;
    obs_codes lxx = L1C;
    obs_codes dxx = D1C;

    if (DEBUG_PROG <= debug) {
        (void)fprintf(stderr, "INFO: one_sig() %c %u:%u:%u\n",
                      gnssid2rinex(gnssid),
                      gnssid, svid, sigid);
    }

    switch (sigid) {
    default:
        (void)fprintf(stderr, "ERROR: one_sig() gnmssid %u unknown sigid %u\n",
                      gnssid, sigid);
        FALLTHROUGH
    case 0:
        /* L1C */
        cxx = C1C;
        lxx = L1C;
        dxx = D1C;
        break;
    case 2:
        /* GLONASS L2 OF or BeiDou B2I D1 */
        if (GNSSID_BD == gnssid) {
            /* WAG */
            cxx = C7I;
            lxx = L7I;
            dxx = D7I;
        } else {
            cxx = C2C;
            lxx = L2C;
            dxx = D2C;
        }
        break;
    case 3:
        /* GPS L2 or BD B2I D2 */
        cxx = C2C;
        lxx = L2C;
        dxx = D2C;
        break;
    case 5:
        /* QZSS L2C (L) */
        cxx = C2L;
        lxx = L2L;
        dxx = D2L;
        break;
    case 6:
        /* Galileo E5 bQ */
        cxx = C7Q;
        lxx = L7Q;
        dxx = D7Q;
        break;
    }

    /* map snr to RINEX snr flag [1-9] */
    if (0 == meas->snr) {
        snr = 0;
    } else if (12 > meas->snr) {
        snr = 1;
    } else if (18 >= meas->snr) {
        snr = 2;
    } else if (23 >= meas->snr) {
        snr = 3;
    } else if (29 >= meas->snr) {
        snr = 4;
    } else if (35 >= meas->snr) {
        snr = 5;
    } else if (41 >= meas->snr) {
        snr = 6;
    } else if (47 >= meas->snr) {
        snr = 7;
    } else if (53 >= meas->snr) {
        snr = 8;
    } else {
        /* snr >= 54 */
        snr = 9;
    }

    /* check for slip
     * FIXME: use actual interval
     * locktime is in milliseconds
     * sample_interval_ms is milli seconds */
    if (meas->locktime < sample_interval_ms) {
        meas->lli |= 2;
    }

    if (0 != isfinite(meas->pseudorange)) {
        obs_cnt_inc(gnssid, svid, cxx);
    }

    if (0 != isfinite(meas->carrierphase)) {
        obs_cnt_inc(gnssid, svid, lxx);
    }

    if (0 != isfinite(meas->doppler)) {
        obs_cnt_inc(gnssid, svid, dxx);
    }

    (void)fputs(fmt_obs(meas->pseudorange, 0, 0), tmp_file);
    // putting snr here, with phase, is deprecated.
    // it should be an S observation.
    (void)fputs(fmt_obs(meas->carrierphase, meas->lli, snr), tmp_file);
    (void)fputs(fmt_obs(meas->doppler, 0, 0), tmp_file);
}


/* print_raw()
 * print one epoch of observations into "tmp_file"
 */
static void print_raw(struct gps_data_t *gpsdata)
{
    struct tm *now_time;
    struct tm tm_buf;            // temp buffer for gmtime_r()
    unsigned nrec = 0;
    unsigned nsat = 0;
    unsigned i;
    unsigned char last_gnssid = 0;
    unsigned char last_svid = 0;
    int need_nl = 0;
    int got_l1 = 0;
    timespec_t interval_ts;

    TS_SUB(&interval_ts, &gpsdata->raw.mtime, &last_mtime);
    if (!TS_GE(&interval_ts, &sample_interval_ts)) {
        /* not time yet */
        return;
    }

#ifdef __UNUSED
    fprintf(stderr, "sample: %ld %ld\n", (long)sample_interval_ts.tv_sec,
            (long)sample_interval_ts.tv_nsec);
    fprintf(stderr, "epoch: %ld %ld\n", (long)gpsdata->raw.mtime.tv_sec,
            (long)gpsdata->raw.mtime.tv_nsec);
#endif // __UNUSED

    // do modulo only for sample_interval of even seconds
    if (0 == sample_interval_ts.tv_nsec &&
        0 < sample_interval_ts.tv_sec) {
        time_t epoch_sec = gpsdata->raw.mtime.tv_sec;
        if (500000000 < gpsdata->raw.mtime.tv_nsec) {
             // round it up.  To match convbin.
             // does this break opus?
             epoch_sec++;
        }

        /* opus insists (time % interval) = 0 */
        if (0 != (epoch_sec % sample_interval_ts.tv_sec)) {
            return;
        }
    }

    /* RINEX 3 wants records in each epoch sorted by gnssid.
     * To look nice: sort by gnssid and svid
     * To work nice, sort by gnssid, svid and sigid.
     * Each sigid is one record in RAW, but all sigid is one
     * record in RINEX
     */

    /* go through list three times, first just to get a count for sort */
    for (i = 0; i < MAXCHANNELS; i++) {
        if (0 == gpsdata->raw.meas[i].svid) {
            /* bad svid, end of list */
            break;
        }
        nrec++;
    }

    if (0 == nrec) {
        /* nothing to do */
        return;
    }
    qsort(gpsdata->raw.meas, nrec, sizeof(gpsdata->raw.meas[0]),
          compare_meas);

    /* second just to get a count, needed for epoch header */
    for (i = 0; i < nrec; i++) {
        if (0 == gpsdata->raw.meas[i].svid) {
            /* bad svid */
            continue;
        }
        if (4 == gpsdata->raw.meas[i].gnssid) {
            /* skip IMES */
            continue;
        }
        if (GNSSID_CNT <= gpsdata->raw.meas[i].gnssid) {
            /* invalid gnssid */
            continue;
        }
        /* prevent separate sigid from double counting gnssid:svid */
        if ((last_gnssid == gpsdata->raw.meas[i].gnssid) &&
            (last_svid == gpsdata->raw.meas[i].svid)) {
            /* duplicate sat */
            continue;
        }
        last_gnssid = gpsdata->raw.meas[i].gnssid;
        last_svid = gpsdata->raw.meas[i].svid;
        nsat++;
    }
    if (0 == nsat) {
        /* nothing to do */
        return;
    }

    /* save time of last measurement, GPS time, not UTC */
    last_mtime = gpsdata->raw.mtime;     /* structure copy */
    if (0 == first_mtime.tv_sec) {
        /* save time of first measurement */
        first_mtime = last_mtime;     /* structure copy */
    }

    /* print epoch header line */
    now_time = gmtime_r(&(last_mtime.tv_sec), &tm_buf);
    (void)fprintf(tmp_file,"> %4d %02d %02d %02d %02d %02d.%07ld  0%3u\n",
         now_time->tm_year + 1900,
         now_time->tm_mon + 1,
         now_time->tm_mday,
         now_time->tm_hour,
         now_time->tm_min,
         now_time->tm_sec,
         (long)(last_mtime.tv_nsec / 100), nsat);

    last_gnssid = 0;
    last_svid = 0;
    need_nl = 0;
    got_l1 = 0;

    /* Print the observations, one gnssid:svid per line.
     * The fun is merging consecutive records (new sigid) of
     * same gnssid:svid */
    for (i = 0; i < nrec; i++) {
        char rinex_gnssid;
        unsigned char gnssid;
        unsigned char svid;
        unsigned char sigid;

        gnssid = gpsdata->raw.meas[i].gnssid;
        rinex_gnssid = gnssid2rinex(gnssid);
        svid = gpsdata->raw.meas[i].svid;
        sigid = gpsdata->raw.meas[i].sigid;

        if (DEBUG_RAW <= debug) {
            (void)fprintf(stderr,"record: %u:%u:%u %s\n",
                          gnssid, svid, sigid,
                          gpsdata->raw.meas[i].obs_code);
        }

        if (0 == gpsdata->raw.meas[i].svid) {
            /* should not happen... */
            continue;
        }

        /* line can be longer than 80 chars in RINEX 3 */
        if ((last_gnssid != gpsdata->raw.meas[i].gnssid) ||
            (last_svid != gpsdata->raw.meas[i].svid)) {

            if (0 != need_nl) {
                (void)fputs("\n", tmp_file);
            }
            got_l1 = 0;
            /* new record line gnssid:svid preamble  */
            (void)fprintf(tmp_file,"%c%02d", rinex_gnssid, svid);
        }

        last_gnssid = gpsdata->raw.meas[i].gnssid;
        last_svid = gpsdata->raw.meas[i].svid;

        /* L1x */
        switch (gpsdata->raw.meas[i].sigid) {
        case 0:
            /* L1 */
            one_sig(&gpsdata->raw.meas[i]);
            got_l1 = 1;
            break;
        case 2:
            /* GLONASS L2 OF or BD B2I D1 */
            if (0 == got_l1) {
                /* space to start of L2 */
                (void)fprintf(tmp_file, "%48s", "");
            }
            one_sig(&gpsdata->raw.meas[i]);
            break;
        case 3:
            /* GPS L2 or BD B2I D2 */
            if (0 == got_l1) {
                /* space to start of L2 */
                (void)fprintf(tmp_file, "%48s", "");
            }
            one_sig(&gpsdata->raw.meas[i]);
            break;
        case 5:
            /* QZSS L2C (L) */
            if (0 == got_l1) {
                /* space to start of L2 */
                (void)fprintf(tmp_file, "%48s", "");
            }
            one_sig(&gpsdata->raw.meas[i]);
            break;
        case 6:
            /* Galileo E5 bQ */
            if (0 == got_l1) {
                /* space to start of L2 */
                (void)fprintf(tmp_file, "%48s", "");
            }
            one_sig(&gpsdata->raw.meas[i]);
            break;
        default:
            (void)fprintf(stderr,
                          "ERROR: print_raw() gnssid %u unknown sigid %u\n",
                          gnssid, sigid);
            break;
        }

        need_nl = 1;
    }
    if (0 != need_nl) {
        (void)fputs("\n", tmp_file);
    }
    sample_count--;
}

static int sig_flag = 0;

static void quit_handler(int signum)
{
    // CWE-479: Signal Handler Use of a Non-reentrant Function
    // See: The C Standard, 7.14.1.1, paragraph 5 [ISO/IEC 9899:2011]
    // Can't log in a signal handler.  Can't even call exit().
    sig_flag = signum;
    return;
}

/* conditionally_log_fix()
 * take the new gpsdata and decide what to do with it.
 */
static void conditionally_log_fix(struct gps_data_t *gpsdata)
{
    if (0 == leap_seconds && 0 < gpsdata->leap_seconds) {
        // grab a static copy of the current leap second.
        leap_seconds = gpsdata->leap_seconds;
    }

    if (DEBUG_PROG <= debug) {
        /* The (long long unsigned) is for 32/64-bit compatibility */
        (void)fprintf(stderr, "mode %d set %llx leap %d\n",
                      gpsdata->fix.mode,
                      (long long unsigned)gpsdata->set,
                      leap_seconds);
    }
    if (0 == leap_seconds) {
        // Can't do anything until we know the current leap second
        return;
    }

    /* mostly we don't care if 2D or 3D fix, let the post processor
     * decide */

    if (MODE_2D < gpsdata->fix.mode) {
        /* got a good 3D fix */
        if (1.0 > ecefx &&
            isfinite(gpsdata->fix.ecef.x) &&
            isfinite(gpsdata->fix.ecef.y) &&
            isfinite(gpsdata->fix.ecef.z)) {
            /* save ecef for "APPROX POS" */
            ecefx = gpsdata->fix.ecef.x;
            ecefy = gpsdata->fix.ecef.y;
            ecefz = gpsdata->fix.ecef.z;

            if (DEBUG_PROG <= debug) {
                (void)fprintf(stderr,"got ECEF\n");
            }
        }
    }

    if (RAW_SET & gpsdata->set) {
        if (DEBUG_RAW <= debug) {
            (void)fprintf(stderr,"got RAW\n");
        }
        /* RINEX 3 prefers GPS time. Accepts GLO (UTC) time.
         * NRCan does not accept GLO time
         * Remove the leap second to get GPS from UTC.
         */
        gpsdata->raw.mtime.tv_sec += leap_seconds;
        print_raw(gpsdata);
    }
    return;
}

/* usage()
 * print usages, and exit
 */
static void usage(void)
{
    (void)fprintf(stderr,
          "Usage: %s [OPTIONS] [server[:port:[device]]]\n"
          "\n"
          "Mandatory arguments to long options are mandatory for "
          "short options too.\n"
          "     -D, --debug LVL            Set debug level, default 0\n"
          "     -f FILE, --fileout FILE    Output to filename\n"
          "                                default: gpsrinexYYYYDDDDHHMM.obs\n"
          "     -h, --help                 print this usage and exit\n"
          "     -i SEC, --interval SEC     Time between samples in seconds\n"
          "                                default: %0.3f\n"
          "     -n COUNT, --count COUNT    Number samples to collect\n"
          "                                default: %d\n"
          "     -V, --version              print version and exit\n"
          "\nThese strings get placed in the generated RINEX 3 obs file\n"
          "     --agency AGENCY           agency\n"
          "     --ant_e EASTING           antenna easting in meters\n"
          "     --ant_h HEIGHT            antenna height in meters\n"
          "     --ant_n NORTHING          antenna northing in meters\n"
          "     --ant_num NUM             antenna number\n"
          "     --ant_type TYPE           antenna type\n"
          "     --marker_name NAME        marker name\n"
          "     --marker_type TYPE        marker type\n"
          "     --observer OBSERVER       observer\n"
          "     --rec_num NUM             receiver number\n"
          "     --rec_type TYPE           receiver type\n"
          "     --rec_vers VERS           receiver vers\n"
          "\n"
          "defaults to '%s -n %d -i %0.3f localhost:2947'\n",
          progname, (double)sample_interval_ms / 1000.0, sample_count, progname,
          sample_count, (double)sample_interval_ms / 1000.0);
    exit(EXIT_FAILURE);
}

// defines for getopt_long()
#define AGENCY 301
#define ANT_E 302
#define ANT_H 303
#define ANT_N 304
#define ANT_NUM 305
#define ANT_TYPE 306
#define MARKER_NAME 307
#define MARKER_TYPE 308
#define OBSERVER 309
#define REC_NUM 310
#define REC_TYPE 311
#define REC_VERS 312

/*
 *
 * Main
 *
 */
int main(int argc, char **argv)
{
    char tmstr[40];              // time: YYYYDDDMMHH
    char tmp_fname[32];          // temp file name, for mkstemp
    int tmp_file_desc;           // temp file descriptor
    struct tm *report_time;
    struct tm tm_buf;            // temp buffer for gmtime_r()
    unsigned int flags = WATCH_ENABLE;
    char   *fname = NULL;
    int timeout = 10;
    double f;

    progname = argv[0];

    log_file = stdout;
    while (1) {
        int ch;
        const char *optstring = "?D:f:hi:n:V";

#ifdef HAVE_GETOPT_LONG
        int option_index = 0;
        static struct option long_options[] = {
            {"agency", required_argument, NULL, AGENCY},
            {"ant_num", required_argument, NULL, ANT_NUM},
            {"ant_type", required_argument, NULL, ANT_TYPE},
            {"ant_e", required_argument, NULL, ANT_E},
            {"ant_h", required_argument, NULL, ANT_H},
            {"ant_n", required_argument, NULL, ANT_N},
            {"count", required_argument, NULL, 'n' },
            {"debug", required_argument, NULL, 'D' },
            {"fileout", required_argument, NULL, 'f' },
            {"help", no_argument, NULL, 'h' },
            {"interval", required_argument, NULL, 'i' },
            {"marker_name", required_argument, NULL, MARKER_NAME},
            {"marker_type", required_argument, NULL, MARKER_TYPE},
            {"observer", required_argument, NULL, OBSERVER},
            {"rec_num", required_argument, NULL, REC_NUM},
            {"rec_type", required_argument, NULL, REC_TYPE},
            {"rec_vers", required_argument, NULL, REC_VERS},
            {"version", no_argument, NULL, 'V' },
            {NULL, 0, NULL, 0},
        };

        ch = getopt_long(argc, argv, optstring, long_options, &option_index);
#else
        ch = getopt(argc, argv, optstring);
#endif
        if (ch == -1) {
            break;
        }

        switch (ch) {
        case 'D':
            debug = atoi(optarg);
            gps_enable_debug(debug, log_file);
            break;
        case 'f':       /* Output file name. */
            fname = strdup(optarg);
            break;
        case 'i':               /* set sampling interval */
            f = safe_atof(optarg); // still in seconds
            if (3600.0 <= f) {
                (void)fprintf(stderr,
                              "WARNING: sample interval is an hour or more!\n");
            }
            sample_interval_ms = (unsigned)(1000 * f); // now in ms
            if (0 == sample_interval_ms) {
                // underflow
                sample_interval_ms = 1;
            }
            MSTOTS(&sample_interval_ts, sample_interval_ms);
            break;
        case 'n':
            sample_count = atoi(optarg);
            break;
        case 'V':
            (void)fprintf(stderr, "%s: version %s (revision %s)\n",
                          progname, VERSION, REVISION);
            exit(EXIT_SUCCESS);
        case AGENCY:
            strlcpy(agency, optarg, sizeof(agency));
            break;
        case ANT_E:
            ant_e = safe_atof(optarg);
            break;
        case ANT_H:
            ant_h = safe_atof(optarg);
            break;
        case ANT_N:
            ant_n = safe_atof(optarg);
            break;
        case ANT_NUM:
            strlcpy(ant_num, optarg, sizeof(ant_num));
            break;
        case ANT_TYPE:
            strlcpy(ant_type, optarg, sizeof(ant_type));
            break;
        case MARKER_NAME:
            strlcpy(marker_name, optarg, sizeof(marker_name));
            break;
        case MARKER_TYPE:
            strlcpy(marker_type, optarg, sizeof(marker_type));
            break;
        case OBSERVER:
            strlcpy(observer, optarg, sizeof(observer));
            break;
        case REC_NUM:
            strlcpy(rec_num, optarg, sizeof(rec_num));
            break;
        case REC_TYPE:
            strlcpy(rec_type, optarg, sizeof(rec_type));
            break;
        case REC_VERS:
            strlcpy(rec_vers, optarg, sizeof(rec_vers));
            break;
        case '?':
            FALLTHROUGH
        case 'h':
            FALLTHROUGH
        default:
            usage();
            /* NOTREACHED */
        }
    }

    /* init source defaults */
    source.server = (char *)"localhost";
    source.port = (char *)DEFAULT_GPSD_PORT;
    source.device = NULL;

    if (optind < argc) {
        /* in this case, switch to the method "socket" always */
        gpsd_source_spec(argv[optind], &source);
    }
    if (DEBUG_INFO <= debug) {
        char *device;
        if (NULL == source.device) {
            device = "Default";
        } else {
            device = source.device;
        }
        (void)fprintf(stderr, "INFO: server: %s port: %s  device: %s\n",
                      source.server, source.port, device);
    }

    /* save start time of report */
    (void)clock_gettime(CLOCK_REALTIME, &start_time);
    report_time = gmtime_r(&(start_time.tv_sec), &tm_buf);

    /* open the output file */
    if (NULL == fname) {
        (void)strftime(tmstr, sizeof(tmstr), "gpsrinex%Y%j%H%M%S.obs",
                       report_time);
        fname = tmstr;
    }
    log_file = fopen(fname, "w");
    if (log_file == NULL) {
        syslog(LOG_ERR, "ERROR: Failed to open %s: %s",
               fname, strerror(errno));
        exit(3);
    }

    /* clear the counts */
    memset(obs_cnt, 0, sizeof(obs_cnt));

    /* catch all interesting signals */
    (void)signal(SIGTERM, quit_handler);
    (void)signal(SIGQUIT, quit_handler);
    (void)signal(SIGINT, quit_handler);

    if (gps_open(source.server, source.port, &gpsdata) != 0) {
        (void)fprintf(stderr, "%s: no gpsd running or network error: %d, %s\n",
                      progname, errno, gps_errstr(errno));
        exit(EXIT_FAILURE);
    }

    if (source.device != NULL)
        flags |= WATCH_DEVICE;
    (void)gps_stream(&gpsdata, flags, source.device);

    // create temp file, coverity does not like tmpfile()
    // covarfity wants a umask
    (void)umask(0177);        // force rw-r--r--
    strlcpy(tmp_fname, "/tmp/gpsrinexXXXXXX", sizeof(tmp_fname));
    tmp_file_desc = mkstemp(tmp_fname);
    if (0 > tmp_file_desc) {
        (void)fprintf(stderr, "ERROR: mkstemp(%s) failed: %s\n",
                      tmp_fname, strerror(errno));
        exit(2);
    }
    tmp_file = fdopen(tmp_file_desc, "w+");
    if (NULL == tmp_file) {
        (void)fprintf(stderr, "ERROR: fdopen() failed: %s\n",
                      strerror(errno));
        exit(2);
    }

    for (;;) {
        if (0 != sig_flag) {
            break;
        }
        /* wait for gpsd */
        if (!gps_waiting(&gpsdata, timeout * 1000000)) {
            (void)fprintf(stderr, "gpsrinex: timeout\n");
            syslog(LOG_INFO, "timeout;");
            break;
        }
        if (0 != sig_flag) {
            break;
        }
        (void)gps_read(&gpsdata, NULL, 0);
        if (ERROR_SET & gpsdata.set) {
            fprintf(stderr, "gps_read() error '%s'\n", gpsdata.error);
            exit(6);
        }
        if (0 != sig_flag) {
            break;
        }
        conditionally_log_fix(&gpsdata);
        if (0 >= sample_count) {
            /* done */
            break;
        }
    }

    print_rinex_footer();

    // remove the temp file
    (void)unlink(tmp_fname);
    if (0 != sig_flag && SIGINT != sig_flag) {
        syslog(LOG_INFO, "exiting, signal %d received", sig_flag);
    }
    exit(EXIT_SUCCESS);
}

// vim: set expandtab shiftwidth=4
