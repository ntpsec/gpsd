/* gpssnmp - poll gpsd varaiables for SNMP.
 *
 * Copyright 2016 David Taylor <gpsd@david.taylor.name>
 *
 * Copyright 2018 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <errno.h>                   // for errno
#ifdef HAVE_GETOPT_LONG
   #include <getopt.h>
#endif
#include <math.h>                    // for isfinite(), fabs()
#include <stdio.h>
#include <stdlib.h>
#include <string.h>                  // for strlcpy()
#include <unistd.h>                  // for alarm()

#include "../include/compiler.h"     // for FALLTHROUGH
#include "../include/gps.h"
#include "../include/gpsdclient.h"   // for gpsd_source_spec()
#include "../include/os_compat.h"    // for strlcpy() if needed
#include "../include/timespec.h"     // for TS_SUB_D()

#define PROGNAME "gpssnmp"
static int debug = 0;                      // debug level
static struct gps_data_t gpsdata;          // last received gps_data_t
static struct gps_data_t gpsdata_ll;       // cached gps_data_t with LATLON_SET
static struct gps_data_t gpsdata_ver;      // cached gps_data_t with VERSION_SET
static int one = 1;                        // the one!
static double snr_avg = 0;
static FILE *logfd;

typedef enum {
    t_double,
    t_dummy,      // used for non-terminal OIDs.
    t_sbyte,
    t_schort,
    t_sinteger,
    t_slongint,
    t_string,
    t_time,
    t_ubyte,
    t_uinteger,
    t_ulongint,
    t_ushort}
gpsdata_type;

struct oid_mib_xlate {
    const char *oid;            // this OID
    const char *short_mib;      // short MIB for this
    gpsdata_type type;          // the type of the value
    void *pval;                 // pointer the value
    int64_t scale;              // scale factor to convert to int32_t
    int64_t min;                // minimum value of scaled value
    gps_mask_t need;            // the _SET this needs
    const char *desc;           // description, for usage()
};

/* Keep this list sorted, so it can be "walked".
 * Sorted "Numerically", not "alphabetically".
 * For now we only handle the first device, so table OIDs, end in .1
 */
static const struct oid_mib_xlate xlate[] = {
    // next three are "pirate" OIDs, deprecated
    {".1.3.6.1.2.1.25.1.31", NULL, t_sinteger, &gpsdata.satellites_visible,
     1, -9, SATELLITE_SET, ""},
    {".1.3.6.1.2.1.25.1.32", NULL, t_sinteger, &gpsdata.satellites_used,
     1, 0, SATELLITE_SET, ""},
    {".1.3.6.1.2.1.25.1.33", NULL, t_sinteger, &snr_avg, 1, 0, SATELLITE_SET,
     ""},
    // previous three are "pirate" OIDs, deprecated
    {".1.3.6.1.4.1.59054", "gpsd", t_dummy, NULL, 0, 0, ONLINE_SET,
     "Anchor for GPSD-MIB"},
    // start sky
    {".1.3.6.1.4.1.59054.11", "sky", t_dummy, NULL, 0, 0, ONLINE_SET,
     "Anchor for SKY"},
    // only handle one device, for now
    {".1.3.6.1.4.1.59054.11.1", "skyNumber", t_sinteger, &one, 1, -1,
     ONLINE_SET, "The number of devices in the skyTable"},
    {".1.3.6.1.4.1.59054.11.2.1.1.1", "skyIndex", t_sinteger, &one, 1, 0,
     ONLINE_SET, "skyTable Index"},
    {".1.3.6.1.4.1.59054.11.2.1.2.1", "skyPath", t_string, &gpsdata.dev.path,
     1, 0, SATELLITE_SET, "path for this device"},
    {".1.3.6.1.4.1.59054.11.2.1.3.1", "skynSat.1", t_sinteger,
     &gpsdata.satellites_visible, 1, -1, SATELLITE_SET,
     "Number of satellties seen"},
    {".1.3.6.1.4.1.59054.11.2.1.4.1", "skyuSat.1", t_sinteger,
     &gpsdata.satellites_used, 1, -1, SATELLITE_SET,
     "Number of satellties in use"},
    {".1.3.6.1.4.1.59054.11.2.1.5.1", "skySNRavg.1", t_double,
     &snr_avg, 100, 0, SATELLITE_SET,
     "Average SNR of all satellites in use."},
    {".1.3.6.1.4.1.59054.11.2.1.6.1", "skyGdop.1", t_double,
     &gpsdata.dop.gdop, 100, 0, DOP_SET,
     "gdop."},
    {".1.3.6.1.4.1.59054.11.2.1.7.1", "skyHdop.1", t_double,
     &gpsdata.dop.hdop, 100, 0, DOP_SET,
     "hdop."},
    {".1.3.6.1.4.1.59054.11.2.1.8.1", "skyPdop.1", t_double,
     &gpsdata.dop.pdop, 100, 0, DOP_SET,
     "pdop."},
    {".1.3.6.1.4.1.59054.11.2.1.9.1", "skyTdop.1", t_double,
     &gpsdata.dop.tdop, 100, 0, DOP_SET,
     "tdop."},
    {".1.3.6.1.4.1.59054.11.2.1.10.1", "skyVdop.1", t_double,
     &gpsdata.dop.vdop, 100, 0, DOP_SET,
     "vdop."},
    {".1.3.6.1.4.1.59054.11.2.1.11.1", "skyXdop.1", t_double,
     &gpsdata.dop.xdop, 100, 0, DOP_SET,
     "xdop."},
    {".1.3.6.1.4.1.59054.11.2.1.12.1", "skyYdop.1", t_double,
     &gpsdata.dop.ydop, 100, 0, DOP_SET,
     "ydop."},
    // end sky
    // start tpv
    {".1.3.6.1.4.1.59054.13", "tpv", t_dummy, NULL, 0, 0, ONLINE_SET,
     "Anchor for TPV"},
    {".1.3.6.1.4.1.59054.13.1", "tpvLeapSeconds", t_sinteger,
     &gpsdata.leap_seconds, 1, 1, TIME_SET,
     ""},
    // only handle one device, for now
    {".1.3.6.1.4.1.59054.13.2", "tpvNumber", t_sinteger,
     &one, 1, 0, ONLINE_SET,
     "The number of devices in the tpvTable"},
    {".1.3.6.1.4.1.59054.13.3.1.1.1", "tpvIndex", t_sinteger,
     &one, 1, 1, ONLINE_SET, "tpvTable Index"},
    {".1.3.6.1.4.1.59054.13.3.1.2.1", "tpvPath", t_string,
     &gpsdata.dev.path, 1, 1, MODE_SET, "path for this device"},
    {".1.3.6.1.4.1.59054.13.3.1.3.1", "tpvMode.1", t_sinteger,
     &gpsdata.fix.mode, 1, 0, MODE_SET, "Fix Mode"},
    {".1.3.6.1.4.1.59054.13.3.1.4.1", "tpvStatus.1", t_sinteger,
     &gpsdata.fix.status, 1, 0, STATUS_SET, "Fix Status"},
    // why 1e7?  Because SNMP chokes on INTEGERS > 32 bits.
    {".1.3.6.1.4.1.59054.13.3.1.5.1", "tpvLatitude.1", t_double,
     &gpsdata.fix.latitude, 10000000LL, -900000000LL, LATLON_SET,
     "Latitude in degrees."},
    {".1.3.6.1.4.1.59054.13.3.1.6.1", "tpvLongitude.1", t_double,
     &gpsdata.fix.longitude, 10000000LL, -18010000000LL, LATLON_SET,
     "Longitude in degrees."},
    {".1.3.6.1.4.1.59054.13.3.1.7.1", "tpvAltHAE.1", t_double,
     &gpsdata.fix.altHAE, 10000, LONG_MIN, ALTITUDE_SET,
     "Height above Ellipsoid, in meters."},
    {".1.3.6.1.4.1.59054.13.3.1.8.1", "tpvAltMSL.1", t_double,
     &gpsdata.fix.altMSL, 10000, LONG_MIN, ALTITUDE_SET,
     "Height above MSL, in meters."},
    {".1.3.6.1.4.1.59054.13.3.1.9.1", "tpvClimb.1", t_double,
     &gpsdata.fix.climb, 10000, LONG_MIN, CLIMB_SET,
     "CLimb rate in meters/second"},
    {".1.3.6.1.4.1.59054.13.3.1.10.1", "tpvTrack.1", t_double,
     &gpsdata.fix.track, 100000, -1, TRACK_SET,
     "True Track in degrees."},
    {".1.3.6.1.4.1.59054.13.3.1.11.1", "tpvSpeed.1", t_double,
     &gpsdata.fix.speed, 10000, -1, SPEED_SET,
     "Ground speed (2D) in meters/second."},
    {".1.3.6.1.4.1.59054.13.3.1.12.1", "tpvEpc.1", t_double,
     &gpsdata.fix.epc, 100000, -1, HERR_SET,
     "Estimated climb error in meters / second."},
    {".1.3.6.1.4.1.59054.13.3.1.13.1", "tpvEpd.1", t_double,
     &gpsdata.fix.epd, 100000, -1, HERR_SET,
     "Estimated track (direction) error in degrees."},
    {".1.3.6.1.4.1.59054.13.3.1.14.1", "tpvEph.1", t_double,
     &gpsdata.fix.eph, 100000, -1, HERR_SET,
     "Estimated horizontal (2D) error in meters."},
    {".1.3.6.1.4.1.59054.13.3.1.15.1", "tpvEps.1", t_double,
     &gpsdata.fix.eps, 100000, -1, HERR_SET,
     "Estimated speed (2d) error in meters / second."},
    {".1.3.6.1.4.1.59054.13.3.1.16.1", "tpvEpt.1", t_double,
     &gpsdata.fix.ept, 10000000, -1, HERR_SET,
     "Estimated time in seconds."},
    {".1.3.6.1.4.1.59054.13.3.1.17.1", "tpvEpv.1", t_double,
     &gpsdata.fix.epv, 100000, -1, VERR_SET,
     "Estimated vertical (altitude) error in meters."},
    {".1.3.6.1.4.1.59054.13.3.1.18.1", "tpvEpx.1", t_double,
     &gpsdata.fix.epx, 100000, -1, HERR_SET,
     "Estimated longitude error in meters."},
    {".1.3.6.1.4.1.59054.13.3.1.19.1", "tpvEpy.1", t_double,
     &gpsdata.fix.epy, 100000, -1, HERR_SET,
     "Estimated latitude error in meters."},
    {".1.3.6.1.4.1.59054.13.3.1.20.1", "tpvTime.1", t_time,
     &gpsdata.fix.time, 1, -1, TIME_SET,
     "UTC time of fix."},
    // end tpv
    // start version
    {".1.3.6.1.4.1.59054.14.1", "verRelease", t_string,
     &gpsdata.version.release, 1, 1, VERSION_SET,
     "Release number of gpsd."},
    {".1.3.6.1.4.1.59054.14.2", "verRevision", t_string,
     &gpsdata.version.rev, 1, 1, VERSION_SET,
     "Revision string of gpsd."},
    // end version
    {NULL, NULL, t_sinteger, NULL, 0, 0, ONLINE_SET, ""},
};

/* compare_oid() -- compare 2 oids, "numerically".
 *
 * even though they look alphanumeric, oids need to be compared
 * "numerically".
 *
 * so ".1.3.6.1.4.1.59054.13.3.1.9.1"
 * comes before ".1.3.6.1.4.1.59054.13.3.1.10.1"
 *
 * Return: 0 if oid1 == oid2
 *         negative  if oid1 < oid2
 *         positive  if oid1 > oid2
 *
 */
static long compare_oid(const char *oid1, const char *oid2)
{
    /* One could assume that in1 == in2, until the mismatch, but
     * someone add a leading zero and mess things up. */
    int in1 = 0;
    int in2 = 0;
    long part1, part2, ret = 0;

    while (1) {
        if ('.' != oid1[in1] ||
            '.' != oid2[in2]) {
            // legally, the only option is nul.
            ret = (long)oid1[in1] - (long)oid2[in2];
            break;
        }
        in1++;
        in2++;

        part1 = atol(&oid1[in1]);
        part2 = atol(&oid2[in2]);
        if (part1 != part2) {
            ret = part1 - part2;
            break;
        }
        // same, so far
        // scan to next period or nul
        while (1) {
            in1++;
            if ('\0' == oid1[in1] ||
                '.' == oid1[in1]) {
                break;
            }
        }
        while (1) {
            in2++;
            if ('\0' == oid2[in2] ||
                '.' == oid2[in2]) {
                break;
            }
        }
    }
    // debug:
    // (void)fprintf(logfd, PROGNAME ": %s (%d) %s (%d) %ld\n\n",
    //               oid1, in1, oid2, in2, ret);
    return ret;
}

/* get_line()
 *
 * get one line from stdin, remove the trailing \n, check for errors.
 * used for pass_persist mode
 *
 * Return: void
 */
static void get_line(char *inbuf, size_t inbuf_len)
{
    size_t i;
    // get a command from stdin
    char *s;

    inbuf_len--;

    errno = 0;
    // Don't hang forever in the syscall
    alarm(3);
    inbuf[0] = '\0';
    s = fgets(inbuf, inbuf_len, stdin);
    alarm(0);

    if (NULL == s ||
        '\0' == s[0]) {
        // read error
        fprintf(logfd, PROGNAME ": fgets()got NULL. %s(%d)\n",
                strerror(errno), errno);
        exit(0);
    }
    // remove the trailing \n, if any
    for (i = 0; ; i++) {
        if (inbuf_len <= i) {
            fprintf(logfd, PROGNAME ": string overrun\n");
            exit(0);
        }
        if ('\0' == s[i]) {
            fprintf(logfd, PROGNAME ": missing \\n\n");
            exit(0);
        }
        if ('\n' == s[i]) {
            // got the \n
            s[i] = '\0';
            break;
        }
    }

    fprintf(logfd, PROGNAME ": got s: %s\n", s);
    if ('\0' == s[0]) {
        // done
        puts("");
        exit(0);
    }
    if (0 != fflush(NULL)) {
        // flush error
        fprintf(logfd, PROGNAME ": fflush() error %d\n", errno);
        exit(1);
    }
}

/* put_line()
 *
 * send one line to stdout, add the trailing \n.
 * used for pass_persist mode
 *
 * Return: void
 */
static void put_line(const char *outbuf)
{
    // send it
    if (0 > puts(outbuf)) {
        // write error
        (void)fprintf(logfd, PROGNAME ": fputs() error %d\n", errno);
        exit(1);
    }
    if (1 <= debug) {
        // log it
        if (0 > fprintf(logfd, PROGNAME ": sent: %s\n", outbuf)) {
            // log failure
            (void)fprintf(stderr, "fprintf() error %d\n", errno);
            exit(1);
        }
    }
    // flush it
    if (0 != fflush(stdout)) {
        // flush error
        (void)fprintf(logfd, PROGNAME ": fflush() error %d\n", errno);
        exit(1);
    }
}

/* get_one() -- get gpsdata, until "need" satisfied
 *
 * Wait at most 10 seconds
 *
 * exits on read errors and time outs.
 *
 * Return: void
 *         exits on time out or error
 */
static void get_one(gps_mask_t need)
{
    struct timespec ts_start, ts_now;
    int status, i;
    double snr_total = 0;

    if (ONLINE_SET == need) {
        // nothing needed
        return;
    }

    if (LATLON_SET == need &&
        LATLON_SET == (LATLON_SET & gpsdata_ll.set)) {
        // use cached tpv data
        gpsdata = gpsdata_ll;
        return;
    }
    if (VERSION_SET == need &&
        '\0' != gpsdata_ver.version.release[0]) {
        // use cached version data
        gpsdata = gpsdata_ver;
        return;
    }

    // snmpd is impatient, it will not wait longer than 5 seconds.
    clock_gettime(CLOCK_REALTIME, &ts_start);

    // timout is in micro seconds.
    while (gps_waiting(&gpsdata, 2 * US_IN_SEC)) {

        // wait 3 seconds, tops.
        clock_gettime(CLOCK_REALTIME, &ts_now);
        // use fabs(), in case time went backwards...
        if (3 < fabs(TS_SUB_D(&ts_now, &ts_start))) {
            // FIXME:  Make this configurable.
            // timeout
            (void)fputs(PROGNAME ": ERROR: timeout", logfd);
            exit(1);
        }

        status = gps_read(&gpsdata, NULL, 0);
        if (-1 == status) {
            (void)fprintf(logfd, PROGNAME "gpssnmp: ERROR: read failed %d\n",
                          status);
            exit(1);
        }
        if (VERSION_SET == (VERSION_SET & gpsdata.set)) {
            /* VERSION_SET only come once after connect, so cache
             * that data when we get it. */
            // FIXME: do Similar for DEVICELIST_SET
            gpsdata_ver = gpsdata;
        } else if (LATLON_SET == (LATLON_SET & gpsdata.set)) {
            // cache lat/lon, good for persist mode
            gpsdata_ll = gpsdata;
        }
        if (need == (need & gpsdata.set)) {
            // got something
            break;
        }
    }
    if (SATELLITE_SET == (need & gpsdata.set)) {
        // compute a derived value: snr_avg
        for(i = 0; i <= gpsdata.satellites_used; i++) {
            if (0 < gpsdata.skyview[i].used &&
                1 <  gpsdata.skyview[i].ss) {
                // printf("i: %d, P:%d, ss: %f\n", i, gpsdata.skyview[i].PRN,
                //         gpsdata.skyview[i].ss);
                snr_total += gpsdata.skyview[i].ss;
            }
        }
        if (0 < gpsdata.satellites_used) {
            snr_avg = snr_total / gpsdata.satellites_used;
        }
    }
}

/* oid_lookup()
 *
 * get xlate table entry for oid
 * OR:
 * get next xlate table entry after noid
 * Then output OID, type and value to stdout.
 *
 * Return: NULL on not found
 *         pointer to xlate[x] if found
 */
static const struct oid_mib_xlate *oid_lookup(const char *oid,
                                              bool next)
{
    const struct oid_mib_xlate *pxlate;
    long long value;         // (long long) so we get 64 bits on 32-bit CPUs.
    char outbuf[512];

    for (pxlate = xlate; NULL != pxlate->oid; pxlate++) {
        int compare;

        if ('.' == oid[0]) {
            // numeric OID
            compare = compare_oid(pxlate->oid, oid);
        } else {
            // mib
            if (NULL == pxlate->short_mib) {
                // nothing to match against
                continue;
            }
            if (0 != strcmp(pxlate->short_mib, oid)) {
                // no match
                // no way to do getnext until we can convert MIB to OID.
                continue;
            }
            // get match
        }

        if (4 <= debug) {
            (void)fprintf(logfd,
                          PROGNAME ": Trying %s, next %d, compare %d\n",
                          pxlate->oid, next, compare);
            fflush(NULL);
        }
        if (0 > compare) {
            // not yet, keep going
            continue;
        }
        // else
        if (0 == compare) {
            // exact match
            if (next) {
                // need next
                continue;
            }
            // got get match
        } else {
            // (0 < compare), gone past
            if (!next) {
                // gone past exact match
                return NULL;
            }
            // got next
        }

        /* got match
         * The output here conforms to the requierments of the
         * "pass [-p priority] MIBOID PROG" option to snmpd.conf
         */

        if (4 <= debug) {
            (void)fprintf(logfd, PROGNAME ": match type %d need %s\n",
                          pxlate->type, gps_maskdump(pxlate->need));
        }
        get_one(pxlate->need);      // fill gpsdata with what we need

        switch (pxlate->type) {
        case t_dummy:
            // skip, go to next one
            continue;
        case t_double:
            // SNMP is too stupid to understand IEEE754, use scaled integers
            // SNMP chokes on INTEGER > 32 bits.
            if (isfinite(*(double *)pxlate->pval)) {
                value = (long long)(*(double *)pxlate->pval * pxlate->scale);
                if (pxlate->min <= value) {
                    put_line(pxlate->oid);
                    put_line("INTEGER");
                    snprintf(outbuf, sizeof(outbuf), "%lld", value);
                    put_line(outbuf);
                }
            } else {
                // skip, go to next one
                continue;
            }
            break;
        case t_sinteger:
            // not scaled
            value = *(int *)pxlate->pval;
            if (pxlate->min <= value) {
                put_line(pxlate->oid);
                put_line("INTEGER");
                snprintf(outbuf, sizeof(outbuf), "%lld", value);
                put_line(outbuf);
            }
            break;
        case t_string:
            // 255 seems to be max STRING length.
            put_line(pxlate->oid);
            put_line("STRING");
            snprintf(outbuf, sizeof(outbuf), "%.255s", (char *)pxlate->pval);
            put_line(outbuf);
            break;
        case t_time:
            // 255 seems to be max STRING length.
            put_line(pxlate->oid);
            put_line("STRING");
            put_line(timespec_to_iso8601(*(timespec_t *)pxlate->pval,
                                         outbuf, sizeof(outbuf)));
            break;
        default:
            (void)fprintf(logfd,
                          PROGNAME ": ERROR: internal error, OID %s\n\n",
                          oid);
            // continue or abort?
            continue;
        }
        break;
    }
    return pxlate;
}


/* print usage info, then exit.
 *
 * Never returns
 */
static void usage(char *prog_name) {
    const struct oid_mib_xlate *pxlate;

    // don't add  --persist until is works...
    (void)printf("usage: %s [OPTIONS] [server[:port[:device]]]\n\n\
Options include: \n\
  -?, -h, --help            = help message\n\
                              Use with -D 1 to show possible OIDs\n\
                              Use with -D 2 to show scale factors\n\
  -D, --debug LVL           = set debug level to LVL, default 0 \n\
  -g, --get OID             = get value for OID\n\
  -n, --next OID            = next OID value\n\
  -V, --version             = emit version and exit.\n\n\
Examples:\n\n\
to get the number of saltellites seen with the OID\n\
   $ gpssnmp -g .1.3.6.1.4.1.59054.11.2.1.3.1\n\
   .1.3.6.1.4.1.59054.11.2.1.3.1\n\
   INTEGER\n\
   15\n\n\
to get the number of saltellites seen with the MIB name\n\
   $ gpssnmp -g skynSat.1\n\
   .1.3.6.1.4.1.59054.11.2.1.3.1\n\
   INTEGER\n\
   15\n\n",
        prog_name);

    if (0 >= debug) {
        // done, exit
        exit(0);
    }
    puts("Supported OIDs and their short names:\n");
    for (pxlate = xlate; NULL != pxlate->oid; pxlate++) {
        if (NULL == pxlate->short_mib) {
            // skip deprecated OIDs
            continue;
        }
        printf("   %-15s %-50s\n", pxlate->short_mib, pxlate->oid);
        if (2 > debug) {
            continue;
        }
        if ('\0' != pxlate->desc[0]) {
            printf("     Desc: %s\n", pxlate->desc);
        }
        if (t_sinteger == pxlate->type ||
            t_double == pxlate->type) {
            printf("     Scale: %ld\n", pxlate->scale);
        }
    }
    puts("");
    exit(0);
}

int main (int argc, char **argv)
{
    bool persist = false;
    bool do_usage = false;
    int status;
    char oid[40] = "";       // requested OID
    struct fixsource_t source;
    const struct oid_mib_xlate *pxlate;
    char inbuf[512];
    bool get = false;        // doing a get?
    bool next = false;       // doing a next?

    const char *optstring = "?D:g:hn:pV";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"debug", required_argument, NULL, 'D'},
        {"get", required_argument, NULL, 'g'},
        {"help", no_argument, NULL, 'h'},
        {"next", required_argument, NULL, 'n'},
        {"persist", no_argument, NULL, 'p'},
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif  // HAVE_GETOPT_LONG

    logfd = stderr;

    /* Process the options.  Print help if requested. */
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
        case '?':
            FALLTHROUGH
        case 'h':
            do_usage = true;
            break;
        case 'D':
            debug = atoi(optarg);
            break;
        case 'g':
            strlcpy(oid, optarg, sizeof(oid));
            get = true;
            break;
        case 'n':
            strlcpy(oid, optarg, sizeof(oid));
            next = true;
            break;
        case 'p':
            persist = true;
            break;
        case 'V':
            (void)fprintf(logfd, PROGNAME ": %s (revision %s)\n",
                          VERSION, REVISION);
            exit(EXIT_SUCCESS);
        default:
            (void)fprintf(logfd,
                          PROGNAME ": ERROR: Unknown option %c\n\n", ch);
            do_usage = true;
            break;
        }
    }

    if (do_usage) {
        fprintf(logfd, PROGNAME ": usage\n");
        usage(argv[0]);    // never returns
    }

    if (persist) {
        // debug, log to file
        logfd = fopen("/tmp/gpssnmp.log", "a");
    } else {
        if (!get && !next) {
            (void)fprintf(logfd,
                          PROGNAME ": ERROR: Missing option -g or -n\n\n");
            usage(PROGNAME);
            exit(1);
        }

        if (get && next) {
            (void)fprintf(logfd,
                PROGNAME ": ERROR: Use either -g or -n, not both\n\n");
            usage(PROGNAME);
            exit(1);
        }
    }

    if (debug) {
        gps_enable_debug(debug, logfd);
    }

    // Grok the server, port, and device
    if (optind < argc) {
        gpsd_source_spec(argv[optind], &source);
    } else {
        gpsd_source_spec(NULL, &source);
    }

    // Open the stream to gpsd
    status = gps_open(source.server, source.port, &gpsdata);
    if (0 != status) {
        (void)fprintf(logfd, PROGNAME ": ERROR: connection failed: %d\n",
                      status);
        exit(1);
    }
    // we want JSON
    (void)gps_stream(&gpsdata, WATCH_ENABLE | WATCH_JSON, NULL);

    if (persist) {
        while (1) {
            get_line(inbuf, sizeof(inbuf));

            if (0 == strcmp("PING", inbuf)) {
                // send PONG
                put_line("PONG");
            } else if (0 == strcmp("get", inbuf)) {
                get_line(inbuf, sizeof(inbuf));

                pxlate = oid_lookup(inbuf, false);
                if (NULL == pxlate ||
                    NULL == pxlate->oid) {
                    // if requested OID is \n, getnext gpsd
                    put_line("NONE");
                }
            } else if (0 == strcmp("getnext", inbuf)) {
                get_line(inbuf, sizeof(inbuf));

                pxlate = oid_lookup(inbuf, true);
                if (NULL == pxlate ||
                    NULL == pxlate->oid) {
                    // if requested OID is \n, getnext gpsd
                    put_line("NONE");
                }
            } else if (0 == strcmp("set", inbuf)) {
                // read only
                // get OID, ignore it
                get_line(inbuf, sizeof(inbuf));

                // get value, ignore it
                get_line(inbuf, sizeof(inbuf));
                put_line("not-writable");
            } else {
                put_line("NONE");
            }
        }
    }

    // else, !persist
    pxlate = oid_lookup(oid, next);
    if (NULL == pxlate ||
        NULL == pxlate->oid) {
        // NONE is supposedly for persist mode only, but, why not?
        put_line("NONE");
        // fell of the end of the list...
        (void)fprintf(logfd, PROGNAME ": ERROR: Unknown OID %s\n\n",
                      oid);
        exit(1);
    }
    gps_close(&gpsdata);

    exit(0);
}
