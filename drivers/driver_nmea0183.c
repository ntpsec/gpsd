/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <ctype.h>       // for isdigit()
#include <float.h>       // for FLT_EVAL_METHOD
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#include "../include/gpsd.h"
#include "../include/strfuncs.h"

#include "../include/timespec.h"
/**************************************************************************
 *
 * Parser helpers begin here
 *
 **************************************************************************/

/* Allow avoiding long double intermediate values.
 *
 * On platforms with 0 != FLT_EVAL_METHOD intermediate values may be kept
 * as long doubles.  Some 32-bit OpenBSD and 32-bit Debian have
 * FLT_EVAL_METHOD == 2.  FreeBSD 13,0 has FLT_EVAL_METHOD == -1.  Various
 * cc options (-mfpmath=387, -mno-sse, etc.) can also change FLT_EVAL_METHOD
 * from 0.
 *
 * Although (long double) may in principle more accurate then (double), it
 * can cause slight differences that lead to regression failures.  In
 * other cases (long double) and (double) are the same, thus no effect.
 * Storing values in volatile variables forces the exact size requested.
 * Where the volatile declaration is unnessary (and absent), such extra
 * intermediate variables are normally optimized out.
 */

#if !defined(FLT_EVAL_METHOD) || 0 != FLT_EVAL_METHOD
#define FLT_VOLATILE volatile
#else
#define FLT_VOLATILE
#endif   // FLT_EVAL_METHOD

/* Common lat/lon decoding for do_lat_lon
 *
 * This version avoids the use of modf(), which can be slow and also suffers
 * from exactness problems.  The integer minutes are first extracted and
 * corrected for the improper degree scaling, using integer arithmetic.
 * Then the fractional minutes are added as a double, and the result is scaled
 * to degrees, using multiply which is faster than divide.
 *
 * Forcing the intermediate minutes value to a double is sufficient to
 * avoid regression problems with FLT_EVAL_METHOD>=2.
 */
static inline double decode_lat_or_lon(const char *field)
{
    long degrees, minutes;
    FLT_VOLATILE double full_minutes;
    char *cp;

    // Get integer "minutes"
    minutes = strtol(field, &cp, 10);
    // Must have decimal point
    if ('.' != *cp) {
        return NAN;
    }
    // Extract degrees (scaled by 100)
    degrees = minutes / 100;
    // Rescale degrees to normal factor of 60
    minutes -= degrees * (100 - 60);
    // Add fractional minutes
    full_minutes = minutes + safe_atof(cp);
    // Scale to degrees & return
    return full_minutes * (1.0 / 60.0);
}

/* process a pair of latitude/longitude fields starting at field index BEGIN
 * The input fields look like this:
 *     field[0]: 4404.1237962
 *     field[1]: N
 *     field[2]: 12118.8472460
 *     field[3]: W
 * input format of lat/lon is NMEA style  DDDMM.mmmmmmm
 * yes, 7 digits of precision past the decimal point from survey grade GPS
 *
 * Ignoring the complications ellipsoids add:
 *   1 minute latitude = 1853 m
 *   0.001 minute latitude = 1.853 m
 *   0.000001 minute latitude = 0.001853 m = 1.853 mm
 *   0.0000001 minute latitude = 0.0001853 m = 0.1853 mm
 *
 * return: 0 == OK, non zero is failure.
 */
static int do_lat_lon(char *field[], struct gps_fix_t *out)
{
    double lon;
    double lat;

    if ('\0' == field[0][0] ||
        '\0' == field[1][0] ||
        '\0' == field[2][0] ||
        '\0' == field[3][0]) {
        return 1;
    }

    lat = decode_lat_or_lon(field[0]);
    if ('S' == field[1][0])
        lat = -lat;

    lon = decode_lat_or_lon(field[2]);
    if ('W' == field[3][0])
        lon = -lon;

    if (0 == isfinite(lat) ||
        0 == isfinite(lon)) {
        return 2;
    }

    out->latitude = lat;
    out->longitude = lon;
    return 0;
}

/* process an FAA mode character
 * return status as in session->newdata.status
 */
static int faa_mode(char mode)
{
    int newstatus = STATUS_GPS;

    switch (mode) {
    case '\0':  // missing
        newstatus = STATUS_UNK;
        break;
    case 'A':   // Autonomous
    default:
        newstatus = STATUS_GPS;
        break;
    case 'C':   // Quectel unique: Caution
        newstatus = STATUS_UNK;
        break;
    case 'D':   // Differential
        newstatus = STATUS_DGPS;
        break;
    case 'E':   // Estimated dead reckoning
        newstatus = STATUS_DR;
        break;
    case 'F':   // Float RTK
        newstatus = STATUS_RTK_FLT;
        break;
    case 'N':   // Data Not Valid
        // already handled, for paranoia sake also here
        newstatus = STATUS_UNK;
        break;
    case 'P':   // Precise (NMEA 4+)
        newstatus = STATUS_DGPS;    // sort of DGPS
        break;
    case 'R':   // fixed RTK
        newstatus = STATUS_RTK_FIX;
        break;
    case 'M':   // manual input.  Same as simulated?  Or surveyed?
        FALLTHROUGH
    case 'S':   // simulator
        newstatus = STATUS_SIM;
        break;
    case 'U':   // Quectel unique: Unsafe
        newstatus = STATUS_UNK;
        break;
    }
    return newstatus;
}

/**************************************************************************
 *
 * Scary timestamp fudging begins here
 *
 * Four sentences, GGA and GLL and RMC and ZDA, contain timestamps.
 * GGA/GLL/RMC timestamps look like hhmmss.ss, with the trailing .ss,
 * or .sss, part optional.
 * RMC has a date field, in the format ddmmyy.  ZDA has separate fields
 * for day/month/year, with a 4-digit year.  This means that for RMC we
 * must supply a century and for GGA and GLL we must supply a century,
 * year, and day.  We get the missing data from a previous RMC or ZDA;
 * century in RMC is supplied from the daemon's context (initialized at
 * startup time) if there has been no previous ZDA.
 *
 **************************************************************************/

#define DD(s)   ((int)((s)[0]-'0')*10+(int)((s)[1]-'0'))

/* sentence supplied ddmmyy, but no century part
 *
 * return: 0 == OK,  greater than zero on failure
 */
static int merge_ddmmyy(char *ddmmyy, struct gps_device_t *session)
{
    int yy;
    int mon;
    int mday;
    int year;
    unsigned i;    // NetBSD complains about signed array index

    if (NULL == ddmmyy) {
        return 1;
    }
    for (i = 0; i < 6; i++) {
        // NetBSD 6 wants the cast
        if (0 == isdigit((int)ddmmyy[i])) {
            // catches NUL and non-digits
            // Telit HE910 can set year to "-1" (1999 - 2000)
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NMEA0183: merge_ddmmyy(%s), malformed date\n",  ddmmyy);
            return 2;
        }
    }
    // check for termination
    if ('\0' != ddmmyy[6]) {
        // missing NUL
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: merge_ddmmyy(%s), malformed date\n",  ddmmyy);
        return 3;
    }

    // should be no defects left to segfault DD()
    yy = DD(ddmmyy + 4);
    mon = DD(ddmmyy + 2);
    mday = DD(ddmmyy);

    // check for century wrap
    if (session->nmea.date.tm_year % 100 == 99 && yy == 0)
        gpsd_century_update(session, session->context->century + 100);
    year = (session->context->century + yy);

    /* 32 bit systems will break in 2038.
     * Telix fails on GPS rollover to 2099, which 32 bit system
     * can not handle.  So wrap at 2080.  That way 64 bit systems
     * work until 2080, and 2099 gets reported as 1999.
     * since GPS epoch started in 1980, allows for old NMEA to work.
     */
    if (2080 <= year) {
        year -= 100;
    }

    if ((1 > mon) ||
        (12 < mon)) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: merge_ddmmyy(%s), malformed month\n",  ddmmyy);
        return 4;
    } else if (1 > mday ||
               31 < mday) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: merge_ddmmyy(%s), malformed day\n",  ddmmyy);
        return 5;
    } else {
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: merge_ddmmyy(%s) sets year %d\n",
                 ddmmyy, year);
        session->nmea.date.tm_year = year - 1900;
        session->nmea.date.tm_mon = mon - 1;
        session->nmea.date.tm_mday = mday;
    }
    GPSD_LOG(LOG_RAW, &session->context->errout,
             "NMEA0183: merge_ddmmyy(%s) %d %d %d\n",
             ddmmyy,
             session->nmea.date.tm_mon,
             session->nmea.date.tm_mday,
             session->nmea.date.tm_year);
    return 0;
}

/* decode an hhmmss.ss string into struct tm data and nsecs
 *
 * return: 0 == OK,  otherwise failure
 */
static int decode_hhmmss(struct tm *date, long *nsec, char *hhmmss,
                         struct gps_device_t *session)
{
    int old_hour = date->tm_hour;
    int i;

    if (NULL == hhmmss) {
        return 1;
    }
    for (i = 0; i < 6; i++) {
        // NetBSD 6 wants the cast
        if (0 == isdigit((int)hhmmss[i])) {
            // catches NUL and non-digits
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NMEA0183: decode_hhmmss(%s), malformed time\n",  hhmmss);
            return 2;
        }
    }
    // don't check for termination, might have fractional seconds

    date->tm_hour = DD(hhmmss);
    if (date->tm_hour < old_hour) {  // midnight wrap
        date->tm_mday++;
    }
    date->tm_min = DD(hhmmss + 2);
    date->tm_sec = DD(hhmmss + 4);

    if ('.' == hhmmss[6] &&
        // NetBSD 6 wants the cast
        0 != isdigit((int)hhmmss[7])) {
        // codacy hates strlen()
        int sublen = strnlen(hhmmss + 7, 20);
        i = atoi(hhmmss + 7);
        *nsec = (long)i * (long)pow(10.0, 9 - sublen);
    } else {
        *nsec = 0;
    }

    return 0;
}

/* update from a UTC time
 *
 * return: 0 == OK,  greater than zero on failure
 */
static int merge_hhmmss(char *hhmmss, struct gps_device_t *session)
{
    int old_hour = session->nmea.date.tm_hour;
    int i;

    if (NULL == hhmmss) {
        return 1;
    }
    for (i = 0; i < 6; i++) {
        // NetBSD 6 wants the cast
        if (0 == isdigit((int)hhmmss[i])) {
            // catches NUL and non-digits
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NMEA0183: merge_hhmmss(%s), malformed time\n",  hhmmss);
            return 2;
        }
    }
    // don't check for termination, might have fractional seconds

    session->nmea.date.tm_hour = DD(hhmmss);
    if (session->nmea.date.tm_hour < old_hour) {  // midnight wrap
        session->nmea.date.tm_mday++;
    }
    session->nmea.date.tm_min = DD(hhmmss + 2);
    session->nmea.date.tm_sec = DD(hhmmss + 4);

    session->nmea.subseconds.tv_sec = 0;
    if ('.' == hhmmss[6] &&
        // NetBSD 6 wants the cast
        0 != isdigit((int)hhmmss[7])) {
        // codacy hates strlen()
        int sublen = strnlen(hhmmss + 7, 20);

        i = atoi(hhmmss + 7);
        session->nmea.subseconds.tv_nsec = (long)i *
                                           (long)pow(10.0, 9 - sublen);
    } else {
        session->nmea.subseconds.tv_nsec = 0;
    }

    return 0;
}

static void register_fractional_time(const char *tag, const char *fld,
                                     struct gps_device_t *session)
{

    if ('\0' != fld[0]) {
        char ts_buf[TIMESPEC_LEN];

        session->nmea.last_frac_time = session->nmea.this_frac_time;
        DTOTS(&session->nmea.this_frac_time, safe_atof(fld));
        session->nmea.latch_frac_time = true;
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: %s: registers fractional time %s\n",
                 tag,
                 timespec_str(&session->nmea.this_frac_time, ts_buf,
                              sizeof(ts_buf)));
    }
}

/**************************************************************************
 *
 * NMEA sentence handling begins here
 *
 **************************************************************************/

static gps_mask_t processACCURACY(int c UNUSED, char *field[],
                                  struct gps_device_t *session)
{
    /*
     * $GPACCURACY,961.2*04
     *
     * ACCURACY,x.x*hh<cr><lf>
     *
     * The only data field is "accuracy".
     * The MT3333 manual just says "The smaller the number is, the be better"
     */
    gps_mask_t mask = ONLINE_SET;

    if ('\0' == field[1][0]) {
        // no data
        return mask;
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: $GPACCURACY: %10s.\n", field[1]);
    return mask;
}

// BWC - Bearing and Distance to Waypoint - Great Circle
static gps_mask_t processBWC(int count, char *field[],
                             struct gps_device_t *session)
{
    /*
     * GPBWC,220516,5130.02,N,00046.34,W,213.8,T,218.0,M,0004.6,N,EGLM*11
     *
     * 1. UTC Time, hh is hours, mm is minutes, ss.ss is seconds
     * 2. Waypoint Latitude
     * 3. N = North, S = South
     * 4. Waypoint Longitude
     * 5. E = East, W = West
     * 6. Bearing, degrees True
     * 7. T = True
     * 8. Bearing, degrees Magnetic
     * 9. M = Magnetic
     * 10. Distance, Nautical Miles
     * 11. N = Nautical Miles
     * 12. Waypoint ID
     * 13. FAA mode indicator (NMEA 2.3 and later, optional)
     * 14. Checksum
     *
     * Parse this just to get the time, to help the cycle ender
     */
    gps_mask_t mask = ONLINE_SET;

    if ('\0' != field[1][0]) {
        if (0 == merge_hhmmss(field[1], session)) {
            if (0 == session->nmea.date.tm_year) {
                GPSD_LOG(LOG_WARN, &session->context->errout,
                         "NMEA0183: can't use BWC time until after ZDA or RMC"
                         " has supplied a year.\n");
            } else {
                mask = TIME_SET;
            }
        }
    }
    if (12 <= count) {
        // NMEA 2.3 and later
        session->newdata.status = faa_mode(field[12][0]);
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: BWC: hhmmss=%s faa mode %d\n",
             field[1], session->newdata.status);
    return mask;
}

/* process xxVTG
 *     $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K
 *     $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K,A
 *
 * where:
 *         1,2     054.7,T      True track made good (degrees)
 *         3,4     034.4,M      Magnetic track made good
 *         5,6     005.5,N      Ground speed, knots
 *         7,8     010.2,K      Ground speed, Kilometers per hour
 *         9       A            Mode Indicator (optional)
 *                                see faa_mode() for possible mode values
 *
 * see also:
 * https://gpsd.gitlab.io/gpsd/NMEA.html#_vtg_track_made_good_and_ground_speed
 */
static gps_mask_t processVTG(int count,
                             char *field[],
                             struct gps_device_t *session)
{
    gps_mask_t mask = ONLINE_SET;

    if( (field[1][0] == '\0') || (field[5][0] == '\0')){
        return mask;
    }

    // ignore empty/missing field, fix mode of last resort
    if ((9 < count) &&
        ('\0' != field[9][0])) {

        switch (field[9][0]) {
        case 'A':
            // Autonomous, 2D or 3D fix
            FALLTHROUGH
        case 'D':
            // Differential, 2D or 3D fix
            // MODE_SET here causes issues
            // mask |= MODE_SET;
            break;
        case 'E':
            // Estimated, DR only
            FALLTHROUGH
        case 'N':
            // Not Valid
            // MODE_SET here causes issues
            // mask |= MODE_SET;
            // nothing to use here, leave
            return mask;
        default:
            // Huh?
            break;
        }
    }

    // set true track
    session->newdata.track = safe_atof(field[1]);
    mask |= TRACK_SET;

    // set magnetic variation
    if ('\0' != field[3][0]) {  // ignore empty fields
        session->newdata.magnetic_track = safe_atof(field[3]);
        mask |= MAGNETIC_TRACK_SET;
    }

    session->newdata.speed = safe_atof(field[5]) * KNOTS_TO_MPS;
    mask |= SPEED_SET;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: VTG: course(T)=%.2f, course(M)=%.2f, speed=%.2f",
             session->newdata.track, session->newdata.magnetic_track,
             session->newdata.speed);
    return mask;
}

// Recommend Minimum Course Specific GPS/TRANSIT Data
static gps_mask_t processRMC(int count, char *field[],
                             struct gps_device_t *session)
{
    /*
     * RMC,225446.33,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E,A*68
     * 1     225446.33    Time of fix 22:54:46 UTC
     * 2     A            Status of Fix:
     *                     A = Autonomous, valid;
     *                     D = Differential, valid;
     *                     V = invalid
     * 3,4   4916.45,N    Latitude 49 deg. 16.45 min North
     * 5,6   12311.12,W   Longitude 123 deg. 11.12 min West
     * 7     000.5        Speed over ground, Knots
     * 8     054.7        Course Made Good, True north
     * 9     181194       Date of fix ddmmyy.  18 November 1994
     * 10,11 020.3,E      Magnetic variation 20.3 deg East
     * 12    A            FAA mode indicator (NMEA 2.3 and later)
     *                     see faa_mode() for possible mode values
     * 13    V            Nav Status (NMEA 4.1 and later)
     *                     A=autonomous,
     *                     D=differential,
     *                     E=Estimated,
     *                     M=Manual input mode
     *                     N=not valid,
     *                     S=Simulator,
     *                     V=Valid
     * *68        mandatory nmea_checksum
     *
     * SiRF chipsets don't return either Mode Indicator or magnetic variation.
     */
    gps_mask_t mask = ONLINE_SET;
    char status = field[2][0];
    int newstatus;

    switch (status) {
    default:
        // missing
        FALLTHROUGH
    case 'V':
        // Invalid
        session->newdata.mode = MODE_NO_FIX;
        if ('\0' == field[1][0] ||
            '\0' ==  field[9][0]) {
            /* No time available. That breaks cycle end detector
             * Force report to bypass cycle detector and get report out.
             * To handle Querks (Quectel) like this:
             *  $GPRMC,,V,,,,,,,,,,N*53
             */
            memset(&session->nmea.date, 0, sizeof(session->nmea.date));
            session->cycle_end_reliable = false;
            mask |= REPORT_IS | TIME_SET;
        }
        mask |= STATUS_SET | MODE_SET;
        break;
    case 'D':
        /* Differential Fix
         * STATUS_DGPS set below, after lat/lon check */
        FALLTHROUGH
    case 'A':
        // Valid Fix
        /*
         * The MTK3301, Royaltek RGM-3800, and possibly other
         * devices deliver bogus time values when the navigation
         * warning bit is set.
         */
        if ('\0' != field[1][0] &&
            9 < count &&
            '\0' !=  field[9][0]) {
            if (0 == merge_hhmmss(field[1], session) &&
                0 == merge_ddmmyy(field[9], session)) {
                // got a good data/time
                mask |= TIME_SET;
                register_fractional_time(field[0], field[1], session);
            }
        }
        // else, no point to the time only case, no regressions with that

        if (0 == do_lat_lon(&field[3], &session->newdata)) {
            if ('D' == status) {
                newstatus = STATUS_DGPS;
            } else {
                newstatus = STATUS_GPS;
            }
            mask |= LATLON_SET;
            if (MODE_2D >= session->lastfix.mode) {
                /* we have at least a 2D fix
                 * might cause blinking */
                session->newdata.mode = MODE_2D;
                mask |= MODE_SET;
            } else if (MODE_3D == session->lastfix.mode) {
                // keep the 3D, this may be cycle starter
                // might cause blinking
                session->newdata.mode = MODE_3D;
                mask |= MODE_SET;
            }
        } else {
            newstatus = STATUS_UNK;
            session->newdata.mode = MODE_NO_FIX;
            mask |= MODE_SET;
        }
        if ('\0' != field[7][0]) {
            session->newdata.speed = safe_atof(field[7]) * KNOTS_TO_MPS;
            mask |= SPEED_SET;
        }
        if ('\0' != field[8][0]) {
            session->newdata.track = safe_atof(field[8]);
            mask |= TRACK_SET;
        }

        // get magnetic variation
        if ('\0' != field[10][0] &&
            '\0' != field[11][0]) {
            session->newdata.magnetic_var = safe_atof(field[10]);

            switch (field[11][0]) {
            case 'E':
                // no change
                break;
            case 'W':
                session->newdata.magnetic_var = -session->newdata.magnetic_var;
                break;
            default:
                // huh?
                session->newdata.magnetic_var = NAN;
                break;
            }
            if (0 == isfinite(session->newdata.magnetic_var) ||
                0.09 >= fabs(session->newdata.magnetic_var)) {
                // some GPS set 0.0,E, or 0,w instead of blank
                session->newdata.magnetic_var = NAN;
            } else {
                mask |= MAGNETIC_TRACK_SET;
            }
        }

        if (12 <= count) {
            newstatus = faa_mode(field[12][0]);
            /* QUectel uses
             * S = Safe  (s/b Simulated)
             * C = Caution (not NMEA)
             * U = Unsafe (not NMEA)
             * V = Invalid
             */
        }

        /*
         * This copes with GPSes like the Magellan EC-10X that *only* emit
         * GPRMC. In this case we set mode and status here so the client
         * code that relies on them won't mistakenly believe it has never
         * received a fix.
         */
        if (3 < session->gpsdata.satellites_used) {
            // 4 sats used means 3D
            session->newdata.mode = MODE_3D;
            mask |= MODE_SET;
        } else if (0 != isfinite(session->gpsdata.fix.altHAE) ||
                   0 != isfinite(session->gpsdata.fix.altMSL)) {
            /* we probably have at least a 3D fix
             * this handles old GPS that do not report 3D */
            session->newdata.mode = MODE_3D;
            mask |= MODE_SET;
        }
        session->newdata.status = newstatus;
        mask |= STATUS_SET;
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: RMC: ddmmyy=%s hhmmss=%s lat=%.2f lon=%.2f "
             "speed=%.2f track=%.2f mode=%d var=%.1f status=%d\n",
             field[9], field[1],
             session->newdata.latitude,
             session->newdata.longitude,
             session->newdata.speed,
             session->newdata.track,
             session->newdata.mode,
             session->newdata.magnetic_var,
             session->newdata.status);
    return mask;
}

// Geographic position - Latitude, Longitude
static gps_mask_t processGLL(int count, char *field[],
                             struct gps_device_t *session)
{
    /* Introduced in NMEA 3.0.
     *
     * $GPGLL,4916.45,N,12311.12,W,225444,A,A*5C
     *
     * 1,2: 4916.46,N    Latitude 49 deg. 16.45 min. North
     * 3,4: 12311.12,W   Longitude 123 deg. 11.12 min. West
     * 5:   225444       Fix taken at 22:54:44 UTC
     * 6:   A            Data valid
     * 7:   A            Autonomous mode
     * 8:   *5C          Mandatory NMEA checksum
     *
     * 1,2 Latitude, N (North) or S (South)
     * 3,4 Longitude, E (East) or W (West)
     * 5 UTC of position
     * 6 A = Active, V = Invalid data
     * 7 Mode Indicator
     *    See faa_mode() for possible mode values.
     *
     * I found a note at <http://www.secoh.ru/windows/gps/nmfqexep.txt>
     * indicating that the Garmin 65 does not return time and status.
     * SiRF chipsets don't return the Mode Indicator.
     * This code copes gracefully with both quirks.
     *
     * Unless you care about the FAA indicator, this sentence supplies nothing
     * that GPRMC doesn't already.  But at least two (Garmin GPS 48 and
     * Magellan Triton 400) actually ship updates in GLL that aren't redundant.
     *
     */
    char *status = field[7];
    gps_mask_t mask = ONLINE_SET;

    if (field[5][0] != '\0') {
        if (0 == merge_hhmmss(field[5], session)) {
            register_fractional_time(field[0], field[5], session);
            if (0 == session->nmea.date.tm_year) {
                GPSD_LOG(LOG_WARN, &session->context->errout,
                         "NMEA0183: can't use GLL time until after ZDA or RMC"
                         " has supplied a year.\n");
            } else {
                mask = TIME_SET;
            }
        }
    }
    if ('\0' == field[6][0] ||
        'V' == field[6][0]) {
        // Invalid
        session->newdata.status = STATUS_UNK;
        session->newdata.mode = MODE_NO_FIX;
    } else if ('A' == field[6][0] &&
        (count < 8 || *status != 'N') &&
        0 == do_lat_lon(&field[1], &session->newdata)) {
        int newstatus;

        mask |= LATLON_SET;

        newstatus = STATUS_GPS;
        if (8 <= count) {
            newstatus = faa_mode(*status);
        }
        /*
         * This is a bit dodgy.  Technically we shouldn't set the mode
         * bit until we see GSA, or similar.  But it may be later in the
         * cycle, some devices like the FV-18 don't send it by default,
         * and elsewhere in the code we want to be able to test for the
         * presence of a valid fix with mode > MODE_NO_FIX.
         */
        if (0 != isfinite(session->gpsdata.fix.altHAE) ||
            0 != isfinite(session->gpsdata.fix.altMSL)) {
            session->newdata.mode = MODE_3D;
        } else if (3 < session->gpsdata.satellites_used) {
            // 4 sats used means 3D
            session->newdata.mode = MODE_3D;
        } else if (MODE_2D > session->gpsdata.fix.mode ||
                   (0 == isfinite(session->oldfix.altHAE) &&
                    0 == isfinite(session->oldfix.altMSL))) {
            session->newdata.mode = MODE_2D;
        }
        session->newdata.status = newstatus;
    } else {
        session->newdata.status = STATUS_UNK;
        session->newdata.mode = MODE_NO_FIX;
    }
    mask |= STATUS_SET | MODE_SET;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: GLL: hhmmss=%s lat=%.2f lon=%.2f mode=%d status=%d\n",
             field[5],
             session->newdata.latitude,
             session->newdata.longitude,
             session->newdata.mode,
             session->newdata.status);
    return mask;
}

// Geographic position - Latitude, Longitude, and more
static gps_mask_t processGNS(int count UNUSED, char *field[],
                             struct gps_device_t *session)
{
    /* Introduced in NMEA 4.0?
     *
     * This mostly duplicates RMC, except for the multi GNSS mode
     * indicator.
     *
     * Example.  Ignore the line break.
     * $GPGNS,224749.00,3333.4268304,N,11153.3538273,W,D,19,0.6,406.110,
     *        -26.294,6.0,0138,S,*6A
     *
     * 1:  224749.00     UTC HHMMSS.SS.  22:47:49.00
     * 2:  3333.4268304  Latitude DDMM.MMMMM. 33 deg. 33.4268304 min
     * 3:  N             Latitude North
     * 4:  12311.12      Longitude 111 deg. 53.3538273 min
     * 5:  W             Longitude West
     * 6:  D             FAA mode indicator
     *                     see faa_mode() for possible mode values
     *                     May be one to six characters.
     *                       Char 1 = GPS
     *                       Char 2 = GLONASS
     *                       Char 3 = Galileo
     *                       Char 4 = BDS
     *                       Char 5 = QZSS
     *                       Char 6 = NavIC (IRNSS)
     * 7:  19           Number of Satellites used in solution
     * 8:  0.6          HDOP
     * 9:  406110       MSL Altitude in meters
     * 10: -26.294      Geoid separation in meters
     * 11: 6.0          Age of differential corrections, in seconds
     * 12: 0138         Differential reference station ID
     * 13: S            NMEA 4.1+ Navigation status
     *                   S = Safe
     *                   C = Caution
     *                   U = Unsafe
     *                   V = Not valid for navigation
     * 8:   *6A          Mandatory NMEA checksum
     *
     */
    int newstatus;
    int satellites_used;
    gps_mask_t mask = ONLINE_SET;

    if ('\0' != field[1][0]) {
        if (0 == merge_hhmmss(field[1], session)) {
            register_fractional_time(field[0], field[1], session);
            if (0 == session->nmea.date.tm_year) {
                GPSD_LOG(LOG_WARN, &session->context->errout,
                         "NMEA0183: can't use GNS time until after ZDA or RMC"
                         " has supplied a year.\n");
            } else {
                mask = TIME_SET;
            }
        }
    }

    /* FAA mode: not valid, ignore
     * Yes, in 2019 a GLONASS only fix may be valid, but not worth
     * the confusion */
    if ('\0' == field[6][0] ||      // FAA mode: missing
        'N' == field[6][0]) {       // FAA mode: not valid
        session->newdata.mode = MODE_NO_FIX;
        mask |= MODE_SET;
        return mask;
    }
    /* navigation status, assume S=safe and C=caution are OK
     * can be missing on valid fix */
    if ('U' == field[13][0] ||      // Unsafe
        'V' == field[13][0]) {      // not valid
        return mask;
    }

    satellites_used = atoi(field[7]);

    if (0 == do_lat_lon(&field[2], &session->newdata)) {
        mask |= LATLON_SET;
        session->newdata.mode = MODE_2D;

        if ('\0' != field[9][0]) {
            // altitude is MSL
            session->newdata.altMSL = safe_atof(field[9]);
            if (0 != isfinite(session->newdata.altMSL)) {
                mask |= ALTITUDE_SET;
                if (3 < satellites_used) {
                    // more than 3 sats used means 3D
                    session->newdata.mode = MODE_3D;
                }
            }
            // only need geoid_sep if in 3D mode
            if ('\0' != field[10][0]) {
                session->newdata.geoid_sep = safe_atof(field[10]);
            }
            // Let gpsd_error_model() deal with geoid_sep and altHAE
        }
    } else {
        session->newdata.mode = MODE_NO_FIX;
        mask |= MODE_SET;
    }

    if ('\0' != field[8][0]) {
        session->gpsdata.dop.hdop = safe_atof(field[8]);
        mask |= DOP_SET;
    }

    // we ignore all but the leading mode indicator.
    newstatus = faa_mode(field[6][0]);

    session->newdata.status = newstatus;
    mask |= MODE_SET | STATUS_SET;

    // get DGPS stuff
    if ('\0' != field[11][0] &&
        '\0' != field[12][0]) {
        // both, or neither
        session->newdata.dgps_age = safe_atof(field[11]);
        session->newdata.dgps_station = atoi(field[12]);
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: GNS: hhmmss=%s lat=%.2f lon=%.2f mode=%d status=%d\n",
             field[1],
             session->newdata.latitude,
             session->newdata.longitude,
             session->newdata.mode,
             session->newdata.status);
    return mask;
}

// GNSS Range residuals
static gps_mask_t processGRS(int count UNUSED, char *field[],
                             struct gps_device_t *session)
{
    /* In NMEA 3.01
     *
     * Example:
     * $GPGRS,150119.000,1,-0.33,-2.59,3.03,-0.09,-2.98,7.12,-15.6,17.0,,,,*5A
     *
     * 1:  150119.000    UTC HHMMSS.SS
     * 2:  1             Mode: 0 == original, 1 == recomputed
     * 3:  -0.33         range residual in meters sat 1
     * 4:  -2.59         range residual sat 2
     * [...]
     * n:   *5A          Mandatory NMEA checksum
     *
     */
    int mode;
    gps_mask_t mask = ONLINE_SET;

    if ('\0' == field[1][0] ||
        0 != merge_hhmmss(field[1], session)) {
        // bad time
        return mask;
    }

    mode = atoi(field[2]);
    if (1 != mode &&
        2 != mode) {
        // bad mode
        return mask;
    }

    // FIXME: partial decode.  How to match sat numbers up with GSA?

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: %s: mode %d count %d\n",
             field[0], mode, count);
    return mask;
}

// Global Positioning System Fix Data
static gps_mask_t processGGA(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * GGA,123519,4807.038,N,01131.324,E,1,08,0.9,545.4,M,46.9,M, , *42
     * 1     123519       Fix taken at 12:35:19 UTC
     * 2,3   4807.038,N   Latitude 48 deg 07.038' N
     * 4,5   01131.324,E  Longitude 11 deg 31.324' E
     * 6     1            Fix quality:
     *                     0 = invalid,
     *                     1 = GPS,
     *                         u-blox may use 1 for Estimated
     *                     2 = DGPS,
     *                     3 = PPS (Precise Position Service),
     *                     4 = RTK (Real Time Kinematic) with fixed integers,
     *                     5 = Float RTK,
     *                     6 = Estimated,
     *                     7 = Manual,
     *                     8 = Simulator
     * 7     08           Number of satellites in use
     * 8     0.9          Horizontal dilution of position
     * 9,10  545.4,M      Altitude, Meters MSL
     * 11,12 46.9,M       Height of geoid (mean sea level) above WGS84
     *                    ellipsoid, in Meters
     * 13    33           time in seconds since last DGPS update
     *                    usually empty
     * 14    1023         DGPS station ID number (0000-1023)
     *                    usually empty
     *
     * Some GPS, like the SiRFstarV in NMEA mode, send both GPGSA and
     * GLGPSA with identical data.
     */
    gps_mask_t mask = ONLINE_SET;
    int newstatus;
    char last_last_gga_talker = session->nmea.last_gga_talker;
    int fix;              // a.k.a Quality flag
    int satellites_visible;
    session->nmea.last_gga_talker = field[0][1];

    if ('\0' == field[6][0]) {
        /* no data is no data, assume no fix
         * the test/daemon/myguide-3100.log shows lat/lon/alt but
         * no status, and related RMC shows no fix. */
        fix = -1;
    } else {
        fix = atoi(field[6]);
    }
    // Jackson Labs Micro JLT uses nonstadard fix flag, not handled
    switch (fix) {
    case 0:     // no fix
        newstatus = STATUS_UNK;
        if ('\0' == field[1][0]) {
            /* No time available. That breaks cycle end detector
             * Force report to bypass cycle detector and get report out.
             * To handle Querks (Quectel) like this:
             *  $GPGGA,,,,,,0,,,,,,,,*66
             */
            memset(&session->nmea.date, 0, sizeof(session->nmea.date));
            session->cycle_end_reliable = false;
            mask |= REPORT_IS | TIME_SET;
        }
        break;
    case 1:
        // could be 2D, 3D, GNSSDR
        newstatus = STATUS_GPS;
        break;
    case 2:     // differential
        newstatus = STATUS_DGPS;
        break;
    case 3:
        // GPS PPS, fix valid, could be 2D, 3D, GNSSDR
        newstatus = STATUS_PPS_FIX;
        break;
    case 4:     // RTK integer
        newstatus = STATUS_RTK_FIX;
        break;
    case 5:     // RTK float
        newstatus = STATUS_RTK_FLT;
        break;
    case 6:
        // dead reckoning, could be valid or invalid
        newstatus = STATUS_DR;
        break;
    case 7:
        // manual input, surveyed
        newstatus = STATUS_TIME;
        break;
    case 8:
        /* simulated mode
         * Garmin GPSMAP and Gecko sends an 8, but undocumented why */
        newstatus = STATUS_SIM;
        break;
    case -1:
        FALLTHROUGH
    default:
        newstatus = -1;
        break;
    }
    if (0 <= newstatus) {
        session->newdata.status = newstatus;
        mask = STATUS_SET;
    }
    /*
     * There are some receivers (the Trimble Placer 450 is an example) that
     * don't ship a GSA with mode 1 when they lose satellite lock. Instead
     * they just keep reporting GGA and GSA on subsequent cycles with the
     * timestamp not advancing and a bogus mode.
     *
     * On the assumption that GGA is only issued once per cycle we can
     * detect this here (it would be nicer to do it on GSA but GSA has
     * no timestamp).
     *
     * SiRFstarV breaks this assumption, sending GGA with different
     * talker IDs.
     */
    if ('\0' != last_last_gga_talker &&
        last_last_gga_talker != session->nmea.last_gga_talker) {
        // skip the time check
        session->nmea.latch_mode = 0;
    } else {
        session->nmea.latch_mode = strncmp(field[1],
                          session->nmea.last_gga_timestamp,
                          sizeof(session->nmea.last_gga_timestamp))==0;
    }

    if (session->nmea.latch_mode) {
        session->newdata.status = STATUS_UNK;
        session->newdata.mode = MODE_NO_FIX;
        mask |= MODE_SET | STATUS_SET;
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA0183: xxGGA: latch mode\n");
    } else {
        (void)strlcpy(session->nmea.last_gga_timestamp, field[1],
                      sizeof(session->nmea.last_gga_timestamp));
    }

    /* satellites_visible is used as an accumulator in xxGSV
     * so if we set it here we break xxGSV
     * Some GPS, like SiRFstarV NMEA, report per GNSS used
     * counts in GPGGA and GLGGA.
     * session->gpsdata.satellites_visible = atoi(field[7]);
     */
    satellites_visible = atoi(field[7]);

    if ('\0' == field[1][0]) {
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: GGA time missing.\n");
    } else if (0 == merge_hhmmss(field[1], session)) {
        register_fractional_time(field[0], field[1], session);
        if (0 == session->nmea.date.tm_year) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NMEA0183: can't use GGA time until after ZDA or RMC"
                     " has supplied a year.\n");
        } else {
            mask |= TIME_SET;
        }
    }

    if (0 == do_lat_lon(&field[2], &session->newdata)) {
        session->newdata.mode = MODE_2D;
        mask |= LATLON_SET;
        if ('\0' != field[11][0]) {
            session->newdata.geoid_sep = safe_atof(field[11]);
        } else {
            session->newdata.geoid_sep = wgs84_separation(
                session->newdata.latitude, session->newdata.longitude);
        }
        /*
         * SiRF chipsets up to version 2.2 report a null altitude field.
         * See <http://www.sirf.com/Downloads/Technical/apnt0033.pdf>.
         * If we see this, force mode to 2D at most.
         */
        if ('\0' != field[9][0]) {
            // altitude is MSL
            session->newdata.altMSL = safe_atof(field[9]);
            // Let gpsd_error_model() deal with altHAE
            mask |= ALTITUDE_SET;
            /*
             * This is a bit dodgy.  Technically we shouldn't set the mode
             * bit until we see GSA.  But it may be later in the cycle,
             * some devices like the FV-18 don't send it by default, and
             * elsewhere in the code we want to be able to test for the
             * presence of a valid fix with mode > MODE_NO_FIX.
             *
             * Use satellites_visible as double check on MODE_3D
             */
            if (4 <= satellites_visible) {
                session->newdata.mode = MODE_3D;
            }
        }
        if (3 > satellites_visible) {
            session->newdata.mode = MODE_NO_FIX;
        }
    } else {
        session->newdata.mode = MODE_NO_FIX;
    }
    mask |= MODE_SET;

    // BT-451, and others, send 99.99 for invalid DOPs
    if ('\0' != field[8][0]) {
        double hdop;
        hdop = safe_atof(field[8]);
        if (99.0 > hdop) {
            // why not to newdata?
            session->gpsdata.dop.hdop = hdop;
            mask |= DOP_SET;
        }
    }

    // get DGPS stuff
    if ('\0' != field[13][0] &&
        '\0' != field[14][0]) {
        // both, or neither
        double age;
        int station;

        age = safe_atof(field[13]);
        station = atoi(field[14]);
        if (0.09 < age ||
            0 < station) {
            // ignore both zeros
            session->newdata.dgps_age = age;
            session->newdata.dgps_station = station;
        }
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: GGA: hhmmss=%s lat=%.2f lon=%.2f altMSL=%.2f "
             "mode=%d status=%d\n",
             field[1],
             session->newdata.latitude,
             session->newdata.longitude,
             session->newdata.altMSL,
             session->newdata.mode,
             session->newdata.status);
    return mask;
}


// GST - GPS Pseudorange Noise Statistics
static gps_mask_t processGST(int count, char *field[],
                             struct gps_device_t *session)
{
    /*
     * GST,hhmmss.ss,x,x,x,x,x,x,x,*hh
     * 1 UTC time of associated GGA fix
     * 2 Total RMS standard deviation of ranges inputs to the nav solution
     * 3 Standard deviation (meters) of semi-major axis of error ellipse
     * 4 Standard deviation (meters) of semi-minor axis of error ellipse
     * 5 Orientation of semi-major axis of error ellipse (true north degrees)
     * 6 Standard deviation (meters) of latitude error
     * 7 Standard deviation (meters) of longitude error
     * 8 Standard deviation (meters) of altitude error
     * 9 Checksum
     */
    struct tm date;
    timespec_t ts;
    int ret;
    char ts_buf[TIMESPEC_LEN];
    gps_mask_t mask = ONLINE_SET;

    if (0 > count) {
      return mask;
    }

    // since it is NOT current time, do not register_fractional_time()
    // compute start of today
    if (0 < session->nmea.date.tm_year) {
        // Do not bother if no current year
        memset(&date, 0, sizeof(date));
        date.tm_year = session->nmea.date.tm_year;
        date.tm_mon = session->nmea.date.tm_mon;
        date.tm_mday = session->nmea.date.tm_mday;

        /* note this is not full UTC, just HHMMSS.ss
         * this is not the current time,
         * it references another GPA of the same stamp. So do not set
         * any time stamps with it */
        ret = decode_hhmmss(&date, &ts.tv_nsec, field[1], session);
    } else {
        ret = 1;
    }
    if (0 == ret) {
        // convert to timespec_t , tv_nsec already set
        session->gpsdata.gst.utctime.tv_sec = mkgmtime(&date);
        session->gpsdata.gst.utctime.tv_nsec = ts.tv_nsec;
    } else {
        // no idea of UTC time now
        session->gpsdata.gst.utctime.tv_sec = 0;
        session->gpsdata.gst.utctime.tv_nsec = 0;
    }
    session->gpsdata.gst.rms_deviation       = safe_atof(field[2]);
    session->gpsdata.gst.smajor_deviation    = safe_atof(field[3]);
    session->gpsdata.gst.sminor_deviation    = safe_atof(field[4]);
    session->gpsdata.gst.smajor_orientation  = safe_atof(field[5]);
    session->gpsdata.gst.lat_err_deviation   = safe_atof(field[6]);
    session->gpsdata.gst.lon_err_deviation   = safe_atof(field[7]);
    session->gpsdata.gst.alt_err_deviation   = safe_atof(field[8]);

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: GST: utc = %s, rms = %.2f, maj = %.2f, min = %.2f,"
             " ori = %.2f, lat = %.2f, lon = %.2f, alt = %.2f\n",
             timespec_str(&session->gpsdata.gst.utctime, ts_buf,
                          sizeof(ts_buf)),
             session->gpsdata.gst.rms_deviation,
             session->gpsdata.gst.smajor_deviation,
             session->gpsdata.gst.sminor_deviation,
             session->gpsdata.gst.smajor_orientation,
             session->gpsdata.gst.lat_err_deviation,
             session->gpsdata.gst.lon_err_deviation,
             session->gpsdata.gst.alt_err_deviation);

    mask = GST_SET | ONLINE_SET;
    return mask;
}

// convert NMEA sigid to ublox sigid
static unsigned char nmea_sigid_to_ubx(unsigned char nmea_sigid)
{
    unsigned char ubx_sigid = 0;

    // FIXME: need to know gnssid to guess sigid
    switch (nmea_sigid) {
    default:
        FALLTHROUGH
    case 0:
        // missing, assume GPS L1
        ubx_sigid = 0;
        break;
    case 1:
        // L1
        ubx_sigid = 0;
        break;
    case 2:
        // E5, could be 5 or 6.
        ubx_sigid = 5;
        break;
    case 3:
        // B2 or L2, could be 2 or 3.
        ubx_sigid = 2;
        break;
    case 5:
        // L2
        ubx_sigid = 4;
        break;
    case 6:
        // L2CL
        ubx_sigid = 3;
        break;
    case 7:
        // E1, could be 0 or 1.
        ubx_sigid = 0;
        break;
    }

    return ubx_sigid;
}

/* Deal with range-mapping attempts to use IDs 1-32 by Beidou, etc.
 *
 * See struct satellite_t in gps.h for ubx and nmea gnssid and svid mappings
 *
 * char *talker              -- NMEA talker string
 * int nmea_satnum           -- NMEA (All ver) satellite number (kinda the PRN)
 * int nmea_gnssid           -- NMEA 4.10 gnssid, if known, otherwise zero
 * unsigned char *ubx_gnssid -- returned u-blox gnssid
 * unsigned char *ubx_svid   -- returned u-blox gnssid
 *
 * Return the NMEA 2.x to 4.0 extended PRN
 */
static int nmeaid_to_prn(char *talker, int nmea_satnum,
                         int nmea_gnssid,
                         unsigned char *ubx_gnssid,
                         unsigned char *ubx_svid)
{
    /*
     * According to https://github.com/mvglasow/satstat/wiki/NMEA-IDs
     * and u-blox documentation.
     * NMEA IDs can be roughly divided into the following ranges:
     *
     *   1..32:  GPS
     *   33..64: Various SBAS systems (EGNOS, WAAS, SDCM, GAGAN, MSAS)
     *   65..96: GLONASS
     *   101..136: Quectel Querk, (not NMEA), seems to be Galileo
     *   152..158: Various SBAS systems (EGNOS, WAAS, SDCM, GAGAN, MSAS)
     *   173..182: IMES
     *   193..202: QZSS   (u-blox extended 4.10)
     *   201..264: BeiDou (not NMEA, not u-blox?) Quectel Querk.
     *   301..336: Galileo
     *   401..437: BeiDou
     *   null: GLONASS unused
     *   500-509: NavIC (IRNSS)  NOT STANDARD!
     *
     * The issue is what to do when GPSes from these different systems
     * fight for IDs in the  1-32 range, as in this pair of Beidou sentences
     *
     * $BDGSV,2,1,07,01,00,000,45,02,13,089,35,03,00,000,37,04,00,000,42*6E
     * $BDGSV,2,2,07,05,27,090,,13,19,016,,11,07,147,*5E
     *
     * Because the PRNs are only used for generating a satellite
     * chart, mistakes here aren't dangerous.  The code will record
     * and use multiple sats with the same ID in one skyview; in
     * effect, they're recorded by the order in which they occur
     * rather than by PRN.
     */
    int nmea2_prn = nmea_satnum;

    *ubx_gnssid = 0;   // default to ubx_gnssid is GPS
    *ubx_svid = 0;     // default to unnknown ubx_svid

    if (1 > nmea_satnum) {
        // uh, oh...
        nmea2_prn = 0;
    } else if (0 < nmea_gnssid) {
        // this switch handles case where nmea_gnssid is known
        switch (nmea_gnssid) {
        default:
            // x = IMES                Not defined by NMEA 4.10
            FALLTHROUGH
        case 0:
            // none given, ignore
            nmea2_prn = 0;
            break;
        case 1:
            if (33 > nmea_satnum) {
                // 1 = GPS       1-32
                *ubx_gnssid = 0;
                *ubx_svid = nmea_satnum;
            } else if (65 > nmea_satnum) {
                // 1 = SBAS      33-64
                *ubx_gnssid = 1;
                *ubx_svid = nmea_satnum + 87;
            } else if (137 > nmea_satnum) {
                // 3 = Galileo, 101-136, NOT NMEA.  Quectel Querk
                *ubx_gnssid = 3;
                *ubx_svid = nmea_satnum - 100;
            } else if (152 > nmea_satnum) {
                // Huh?
                *ubx_gnssid = 0;
                *ubx_svid = 0;
                nmea2_prn = 0;
            } else if (158 > nmea_satnum) {
                // 1 = SBAS      152-158
                *ubx_gnssid = 1;
                *ubx_svid = nmea_satnum;
            } else if (193 > nmea_satnum) {
                // Huh?
                *ubx_gnssid = 0;
                *ubx_svid = 0;
                nmea2_prn = 0;
            } else if (200 > nmea_satnum) {
                // 1 = QZSS      193-197
                // undocumented u-blox goes to 199
                *ubx_gnssid = 3;
                *ubx_svid = nmea_satnum - 192;
            } else if (265 > nmea_satnum) {
                // 3 = BeiDor, 201-264, NOT NMEA.  Quectel Querk
                *ubx_gnssid = 3;
                *ubx_svid = nmea_satnum - 200;
            } else {
                // Huh?
                *ubx_gnssid = 0;
                *ubx_svid = 0;
                nmea2_prn = 0;
            }
            break;
        case 2:
            //  2 = GLONASS   65-96, nul
            *ubx_gnssid = 6;
            if (64 > nmea_satnum) {
                // NMEA 1 - 64
                *ubx_svid = nmea_satnum;
            } else {
                // SiRF quirk 65-96
                // Jackson Labs Micro JLT quirk 65-96
                *ubx_svid = nmea_satnum - 64;
            }
            nmea2_prn = 64 + *ubx_svid;
            break;
        case 3:
            //  3 = Galileo   1-36
            *ubx_gnssid = 2;
            if (100 > nmea_satnum) {
                // NMEA
                *ubx_svid = nmea_satnum;
            } else if (100 < nmea_satnum &&
                       200 > nmea_satnum) {
                // Quectel Querk, NOT NMEA, 101 - 199
                *ubx_svid = nmea_satnum - 100;
            } else if (300 < nmea_satnum &&
                       400 > nmea_satnum) {
                // Jackson Labs quirk, NOT NMEA, 301 - 399
                *ubx_svid = nmea_satnum - 300;
            }
            nmea2_prn = 300 + *ubx_svid;    // 301 - 399
            break;
        case 4:
            //  4 - BeiDou    1-37
            *ubx_gnssid = 3;
            if (100 > nmea_satnum) {
                // NMEA 1 - 99
                *ubx_svid = nmea_satnum;
            } else if (200 < nmea_satnum &&
                       300 > nmea_satnum) {
                // Quectel Querk, NOT NMEA, 201 - 299
                *ubx_svid = nmea_satnum - 200;
            } else if (400 < nmea_satnum &&
                       500 > nmea_satnum) {
                // Jackson Labs quirk, NOT NMEA, 401 - 499
                *ubx_svid = nmea_satnum - 400;
            }
            // put it at 400+ where NMEA 4.11 wants it
            nmea2_prn = 400 + *ubx_svid;
            break;
        case 5:
            //  5 - QZSS, 1 - 10, NMEA 4.11
            *ubx_gnssid = 5;
            if (100 > nmea_satnum) {
                // NMEA 1 - 99
                *ubx_svid = nmea_satnum;
            } else {
                // Telit quirk, not NMEA 193 - 199
                *ubx_svid = nmea_satnum - 192;
            }

            // put it at 193 to 199 where NMEA 4.11 wants it
            // huh?  space for only 7?
            nmea2_prn = 192 + *ubx_svid;;
            break;
        case 6:
            //  6 - NavIC (IRNSS)    1-15
            *ubx_gnssid = 7;
            *ubx_svid = nmea_satnum;
            nmea2_prn = nmea_satnum + 500;  // This is wrong...
            break;
        }

    /* left with NMEA 2.x to NMEA 4.0 satnums
     * use talker ID to disambiguate */
    } else if (32 >= nmea_satnum) {
        *ubx_svid = nmea_satnum;
        switch (talker[0]) {
        case 'G':
            switch (talker[1]) {
            case 'A':
                // Galileo
                nmea2_prn = 300 + nmea_satnum;
                *ubx_gnssid = 2;
                break;
            case 'B':
                // map Beidou IDs 1..37 to 401..437
                *ubx_gnssid = 3;
                nmea2_prn = 400 + nmea_satnum;
                break;
            case 'I':
                // map NavIC (IRNSS) IDs 1..10 to 500 - 509, not NMEA
                *ubx_gnssid = 7;
                nmea2_prn = 500 + nmea_satnum;
                break;
            case 'L':
                // GLONASS GL doesn't seem to do this, better safe than sorry
                nmea2_prn = 64 + nmea_satnum;
                *ubx_gnssid = 6;
                break;
            case 'Q':
                // GQ, QZSS, 1 - 10
                nmea2_prn = 192 + nmea_satnum;
                *ubx_gnssid = 5;
                break;
            case 'N':
                // all of them, but only GPS is 0 < PRN < 33
                FALLTHROUGH
            case 'P':
                // GPS,SBAS,QZSS, but only GPS is 0 < PRN < 33
                FALLTHROUGH
            default:
                // WTF?
                break;
            }  // else ??
            break;
        case 'B':
            if ('D' == talker[1]) {
                // map Beidou IDs
                nmea2_prn = 400 + nmea_satnum;
                *ubx_gnssid = 3;
            }  // else ??
            break;
        case 'P':
            // Quectel EC25 & EC21 use PQxxx for BeiDou
            if ('Q' == talker[1]) {
                // map Beidou IDs
                nmea2_prn = 400 + nmea_satnum;
                *ubx_gnssid = 3;
            }  // else ??
            break;
        case 'Q':
            if ('Z' == talker[1]) {
                // QZSS
                nmea2_prn = 192 + nmea_satnum;
                *ubx_gnssid = 5;
            }  // else ?
            break;
        default:
            // huh?
            break;
        }
    } else if (64 >= nmea_satnum) {
        // NMEA-ID (33..64) to SBAS PRN 120-151.
        // SBAS
        *ubx_gnssid = 1;
        *ubx_svid = 87 + nmea_satnum;
    } else if (96 >= nmea_satnum) {
        // GLONASS 65..96
        *ubx_gnssid = 6;
        *ubx_svid = nmea_satnum - 64;
    } else if (120 > nmea_satnum) {
        // Huh?
        *ubx_gnssid = 0;
        *ubx_svid = 0;
        nmea2_prn = 0;
    } else if (158 >= nmea_satnum) {
        // SBAS 120..158
        *ubx_gnssid = 1;
        *ubx_svid = nmea_satnum;
    } else if (173 > nmea_satnum) {
        // Huh?
        *ubx_gnssid = 0;
        *ubx_svid = 0;
        nmea2_prn = 0;
    } else if (182 >= nmea_satnum) {
        // IMES 173..182
        *ubx_gnssid = 4;
        *ubx_svid = nmea_satnum - 172;
    } else if (193 > nmea_satnum) {
        // Huh?
        *ubx_gnssid = 0;
        *ubx_svid = 0;
        nmea2_prn = 0;
    } else if (197 >= nmea_satnum) {
        // QZSS 193..197
        // undocumented u-blox goes to 199
        *ubx_gnssid = 5;
        *ubx_svid = nmea_satnum - 192;
    } else if (201 > nmea_satnum) {
        // Huh?
        *ubx_gnssid = 0;
        *ubx_svid = 0;
        nmea2_prn = 0;
    } else if (237 >= nmea_satnum) {
        // BeiDou, non-standard, some SiRF put BeiDou 201-237
        // $GBGSV,2,2,05,209,07,033,*62
        *ubx_gnssid = 3;
        *ubx_svid = nmea_satnum - 200;
        nmea2_prn += 200;           // move up to 400 where NMEA 2.x wants it.
    } else if (301 > nmea_satnum) {
        // Huh?
        *ubx_gnssid = 0;
        *ubx_svid = 0;
        nmea2_prn = 0;
    } else if (356 >= nmea_satnum) {
        // Galileo 301..356
        *ubx_gnssid = 2;
        *ubx_svid = nmea_satnum - 300;
    } else if (401 > nmea_satnum) {
        // Huh?
        *ubx_gnssid = 0;
        *ubx_svid = 0;
        nmea2_prn = 0;
    } else if (437 >= nmea_satnum) {
        // BeiDou
        *ubx_gnssid = 3;
        *ubx_svid = nmea_satnum - 400;
    } else {
        // greater than 437 Huh?
        *ubx_gnssid = 0;
        *ubx_svid = 0;
        nmea2_prn = 0;
    }

    return nmea2_prn;
}

// GPS DOP and Active Satellites
static gps_mask_t processGSA(int count, char *field[],
                             struct gps_device_t *session)
{
#define GSA_TALKER      field[0][1]
    /*
     * eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
     * eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35
     * NMEA 4.10: $GNGSA,A,3,13,12,22,19,08,21,,,,,,,1.05,0.64,0.83,4*0B
     * 1    = Mode:
     *         M=Manual, forced to operate in 2D or 3D
     *         A=Automatic, 3D/2D
     * 2    = Mode:
     *         1=Fix not available,
     *         2=2D,
     *         3=3D
     * 3-14 = PRNs of satellites used in position fix (null for unused fields)
     * 15   = PDOP
     * 16   = HDOP
     * 17   = VDOP
     * 18   - NMEA 4.1+ GNSS System ID, u-blox extended
     *             1 = GPS L1C/A, L2CL, L2CM
     *             2 = GLONASS L1 OF, L2 OF
     *             3 = Galileo E1C, E1B, E5 bl, E5 bQ
     *             4 = BeiDou B1I D1, B1I D2, B2I D1, B2I D12
     *             5 = QZSS
     *             6 - NavID (IRNSS)
     *
     * Not all documentation specifies the number of PRN fields, it
     * may be variable.  Most doc that specifies says 12 PRNs.
     *
     * The Navior-24 CH-4701 outputs 30 fields, 24 PRNs!
     * GPGSA,A,3,27,23,13,07,25,,,,,,,,,,,,,,,,,,,,07.9,06.0,05.2
     *
     * The Skytraq S2525F8-BD-RTK output both GPGSA and BDGSA in the
     * same cycle:
     * $GPGSA,A,3,23,31,22,16,03,07,,,,,,,1.8,1.1,1.4*3E
     * $BDGSA,A,3,214,,,,,,,,,,,,1.8,1.1,1.4*18
     * These need to be combined like GPGSV and BDGSV
     *
     * Some GPS emit GNGSA.  So far we have not seen a GPS emit GNGSA
     * and then another flavor of xxGSA
     *
     * Some Skytraq will emit all GPS in one GNGSA, Then follow with
     * another GNGSA with the BeiDou birds.
     *
     * SEANEXX, SiRFstarIV, and others also do it twice in one cycle:
     * $GNGSA,A,3,31,26,21,,,,,,,,,,3.77,2.55,2.77*1A
     * $GNGSA,A,3,75,86,87,,,,,,,,,,3.77,2.55,2.77*1C
     * seems like the first is GNSS and the second GLONASS
     *
     * u-blox 9 outputs one per GNSS on each cycle.  Note the
     * extra last parameter which is NMEA gnssid:
     * $GNGSA,A,3,13,16,21,15,10,29,27,20,,,,,1.05,0.64,0.83,1*03
     * $GNGSA,A,3,82,66,81,,,,,,,,,,1.05,0.64,0.83,2*0C
     * $GNGSA,A,3,07,12,33,,,,,,,,,,1.05,0.64,0.83,3*0A
     * $GNGSA,A,3,13,12,22,19,08,21,,,,,,,1.05,0.64,0.83,4*0B
     * Also note the NMEA 4.0 GLONASS PRN (82) in an NMEA 4.1
     * sentence.
     */
    gps_mask_t mask = ONLINE_SET;
    char last_last_gsa_talker = session->nmea.last_gsa_talker;
    int nmea_gnssid = 0;
    int nmea_sigid = 0;
    int ubx_sigid = 0;

    /*
     * One chipset called the i.Trek M3 issues GPGSA lines that look like
     * this: "$GPGSA,A,1,,,,*32" when it has no fix.  This is broken
     * in at least two ways: it's got the wrong number of fields, and
     * it claims to be a valid sentence (A flag) when it isn't.
     * Alarmingly, it's possible this error may be generic to SiRFstarIII.
     */
    if (18 > count) {
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: xxGSA: malformed, setting ONLINE_SET only.\n");
    } else if (session->nmea.latch_mode) {
        // last GGA had a non-advancing timestamp; don't trust this GSA
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: xxGSA: non-advancing timestamp\n");
    } else {
        int i;
        session->newdata.mode = atoi(field[2]);
        /*
         * The first arm of this conditional ignores dead-reckoning
         * fixes from an Antaris chipset. which returns E in field 2
         * for a dead-reckoning estimate.  Fix by Andreas Stricker.
         */
        if ('E' != field[2][0]) {
            mask = MODE_SET;
        }

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA0183: xxGSA sets mode %d\n", session->newdata.mode);

        if (19 < count) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NMEA0183: xxGSA: count %d too long!\n", count);
        } else {
            double dop;
            // Just ignore the last fields of the Navior CH-4701
            if ('\0' != field[15][0]) {
                dop = safe_atof(field[15]);
                if (99.0 > dop) {
                    session->gpsdata.dop.pdop = dop;
                    mask |= DOP_SET;
                }
            }
            // BT-451, and others, send 99.99 for invalid DOPs
            if ('\0' != field[16][0]) {
                dop = safe_atof(field[16]);
                if (99.0 > dop) {
                    session->gpsdata.dop.hdop = dop;
                    mask |= DOP_SET;
                }
            }
            if ('\0' != field[17][0]) {
                dop = safe_atof(field[17]);
                if (99.0 > dop) {
                    session->gpsdata.dop.vdop = dop;
                    mask |= DOP_SET;
                }
            }
            if (19 == count &&
                '\0' != field[18][0]) {
                // get the NMEA 4.10 sigid
                nmea_sigid = atoi(field[18]);
                // FIXME: ubx_sigid not used yet
                ubx_sigid = nmea_sigid_to_ubx(nmea_sigid);
            }
        }
        /*
         * might have gone from GPGSA to GLGSA/BDGSA
         * or GNGSA to GNGSA
         * in which case accumulate
         */
        if ('\0' == session->nmea.last_gsa_talker ||
            (GSA_TALKER == session->nmea.last_gsa_talker &&
             'N' != GSA_TALKER) ) {
            session->gpsdata.satellites_used = 0;
            memset(session->nmea.sats_used, 0, sizeof(session->nmea.sats_used));
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: xxGSA: clear sats_used\n");
        }
        session->nmea.last_gsa_talker = GSA_TALKER;
        switch (session->nmea.last_gsa_talker) {
        case 'A':
            // GA Galileo
            nmea_gnssid = 3;
            session->nmea.seen_gagsa = true;
            break;
        case 'B':
            // GB BeiDou
            FALLTHROUGH
        case 'D':
            // BD BeiDou
            nmea_gnssid = 4;
            session->nmea.seen_bdgsa = true;
            break;
        case 'I':
            // GI IRNSS
            nmea_gnssid = 6;
            session->nmea.seen_gigsa = true;
            break;
        case 'L':
            // GL GLONASS
            nmea_gnssid = 2;
            session->nmea.seen_glgsa = true;
            break;
        case 'N':
            // GN GNSS
            session->nmea.seen_gngsa = true;
            break;
        case 'P':
            // GP GPS
            session->nmea.seen_gpgsa = true;
            break;
        case 'Q':
            // Quectel EC25 & EC21 use PQGSA for QZSS
            FALLTHROUGH
        case 'Z':        // QZ QZSS
            // NMEA 4.11 GQGSA for QZSS
            nmea_gnssid = 5;
            session->nmea.seen_qzgsa = true;
            break;
        }

        // the magic 6 here counts the tag, two mode fields, and DOP fields
        for (i = 0; i < count - 6; i++) {
            int prn;
            int nmea_satnum;
            unsigned char ubx_gnssid;   // UNUSED
            unsigned char ubx_svid;     // UNUSED

            // skip empty fields, otherwise empty becomes prn=200
            nmea_satnum = atoi(field[i + 3]);
            if (1 > nmea_satnum) {
                continue;
            }
            prn = nmeaid_to_prn(field[0], nmea_satnum, nmea_gnssid,
                                &ubx_gnssid, &ubx_svid);

#ifdef __UNUSED__
            // debug
            GPSD_LOG(LOG_SHOUT, &session->context->errout,
                     "NMEA0183: %s nmeaid_to_prn: nmea_gnssid "
                     "%d nmea_satnum %d "
                     "ubx_gnssid %d ubx_svid %d nmea2_prn %d\n",
                     field[0],
                     nmea_gnssid, nmea_satnum, ubx_gnssid, ubx_svid, prn);
            GPSD_LOG(LOG_SHOUT, &session->context->errout,
                     "NMEA0183: %s count %d\n", field[0], count);
#endif  //  __UNUSED__

            if (0 < prn) {
                // check first BEFORE over-writing memory
                if (MAXCHANNELS <= session->gpsdata.satellites_used) {
                    /* This should never happen as xxGSA is limited to 12,
                     * except for the Navior-24 CH-4701.
                     * But it could happen with multiple GSA per cycle */
                    break;
                }
                session->nmea.sats_used[session->gpsdata.satellites_used++] =
                    (unsigned short)prn;
            }
        }
        mask |= USED_IS;
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: xxGSA: mode=%d used=%d pdop=%.2f hdop=%.2f "
                 " vdop=%.2f "
                 "ubx_sigid %d\n",
                 session->newdata.mode,
                 session->gpsdata.satellites_used,
                 session->gpsdata.dop.pdop,
                 session->gpsdata.dop.hdop,
                 session->gpsdata.dop.vdop, ubx_sigid);
    }
    // assumes GLGSA or BDGSA, if present, is emitted directly after the GPGSA
    if ((session->nmea.seen_bdgsa ||
         session->nmea.seen_gagsa ||
         session->nmea.seen_gigsa ||
         session->nmea.seen_glgsa ||
         session->nmea.seen_gngsa ||
         session->nmea.seen_qzgsa) &&
         GSA_TALKER == 'P') {
        mask = ONLINE_SET;
    } else if ('N' != last_last_gsa_talker &&
               'N' == GSA_TALKER) {
        /* first of two GNGSA
         * if mode == 1 some GPS only output 1 GNGSA, so ship mode always */
        mask =  ONLINE_SET | MODE_SET;
    }

    // cast for 32/64 compatibility
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: xxGSA: mask %#llx\n", (long long unsigned)mask);
    return mask;
#undef GSA_TALKER
}

// xxGSV -  GPS Satellites in View
static gps_mask_t processGSV(int count, char *field[],
                             struct gps_device_t *session)
{
#define GSV_TALKER      field[0][1]
    /*
     * GSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
     *  1) 2           Number of sentences for full data
     *  2) 1           Sentence 1 of 2
     *  3) 08          Total number of satellites in view
     *  4) 01          Satellite PRN number
     *  5) 40          Elevation, degrees
     *  6) 083         Azimuth, degrees
     *  7) 46          Signal-to-noise ratio in decibels
     * <repeat for up to 4 satellites per sentence>
     *   )             NMEA 4.1 signalId
     *   )             checksum
     *
     * NMEA 4.1+:
     * $GAGSV,3,1,09,02,00,179,,04,09,321,,07,11,134,11,11,10,227,,7*7F
     * after the satellite block, before the checksum, new field:
     * 7           NMEA Signal ID
     *             1 = GPS L1C/A, BeiDou B1I D1, BeiDou B1I D2, GLONASS L1 OF
     *             2 = Galileo E5 bl, E5 bQ
     *             3 = BeiDou B2I D1, B2I D2
     *             5 = GPS L2 CM
     *             6 = GPS L2 CL
     *             7 = Galileo E1C, E1B
     *
     * Can occur with talker IDs:
     *   BD (Beidou),
     *   GA (Galileo),
     *   GB (Beidou),
     *   GI (IRNSS
     *   GL (GLONASS),
     *   GN (GLONASS, any combination GNSS),
     *   GP (GPS, SBAS, QZSS),
     *   GQ (QZSS).
     *   PQ (QZSS). Quectel Quirk
     *   QZ (QZSS).
     *
     * As of April 2019:
     *    no gpsd regressions have GNGSV
     *    every xxGSV cycle starts with GPGSV
     *    xxGSV cycles may be spread over several xxRMC cycles
     *
     * GL may be (incorrectly) used when GSVs are mixed containing
     * GLONASS, GN may be (incorrectly) used when GSVs contain GLONASS
     * only.  Usage is inconsistent.
     *
     * In the GLONASS version sat IDs run from 65-96 (NMEA0183
     * standardizes this). At least two GPSes, the BU-353 GLONASS and
     * the u-blox NEO-M8N, emit a GPGSV set followed by a GLGSV set.
     * We have also seen two GPSes, the Skytraq S2525F8-BD-RTK and a
     * SiRF-IV variant, that emit GPGSV followed by BDGSV. We need to
     * combine these.
     *
     * The following shows how the Skytraq S2525F8-BD-RTK output both
     * GPGSV and BDGSV in the same cycle:
     * $GPGSV,4,1,13,23,66,310,29,03,65,186,33,26,43,081,27,16,41,124,38*78
     * $GPGSV,4,2,13,51,37,160,38,04,37,066,25,09,34,291,07,22,26,156,37*77
     * $GPGSV,4,3,13,06,19,301,,31,17,052,20,193,11,307,,07,11,232,27*4F
     * $GPGSV,4,4,13,01,03,202,30*4A
     * $BDGSV,1,1,02,214,55,153,40,208,01,299,*67
     *
     * The driver automatically adapts to either case, but it takes until the
     * second cycle (usually 10 seconds from device connect) for it to
     * learn to expect BSDGV or GLGSV.
     *
     * Some GPS (Garmin 17N) spread the xxGSV over several cycles.  So
     * cycles, or cycle time, can not be used to determine start of
     * xxGSV cycle.
     *
     * NMEA 4.1 adds a signal-ID field just before the checksum. First
     * seen in May 2015 on a u-blox M8.  It can output 2 sets of GPGSV
     * in one cycle, one for L1C and the other for L2C.
     *
     * Skytraq PX1172RH_DS can output GPGSV, GLGSV, GAGSV and GBGSV all in
     * same epoch.  And each of those repeated for different signals
     * (L1C/L2C/etc.)
     */

    int n, fldnum;
    unsigned char  nmea_sigid = 0;
    int nmea_gnssid = 0;
    unsigned char  ubx_sigid = 0;

    if (3 >= count) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: malformed xxGSV - fieldcount %d <= 3\n",
                 count);
        gpsd_zero_satellites(&session->gpsdata);
        return ONLINE_SET;
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: x%cGSV: part %s of %s, last_gsv_talker '%#x' "
             " last_gsv_sigid %u\n",
             GSV_TALKER, field[2], field[1],
             session->nmea.last_gsv_talker,
             session->nmea.last_gsv_sigid);

    /*
     * This check used to be !=0, but we have loosen it a little to let by
     * NMEA 4.1 GSVs with an extra signal-ID field at the end.
     */
    switch (count % 4) {
    case 0:
        // normal, pre-NMEA 4.10
        break;
    case 1:
        // NMEA 4.10, get the signal ID
        nmea_sigid = atoi(field[count - 1]);
        ubx_sigid = nmea_sigid_to_ubx(nmea_sigid);
        break;
    default:
        // bad count
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: malformed GPGSV - fieldcount %d %% 4 != 0\n",
                 count);
        gpsd_zero_satellites(&session->gpsdata);
        return ONLINE_SET;
    }

    session->nmea.await = atoi(field[1]);
    if (1 > (session->nmea.part = atoi(field[2]))) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: malformed GPGSV - bad part\n");
        gpsd_zero_satellites(&session->gpsdata);
        return ONLINE_SET;
    }

    if (1 == session->nmea.part) {
        /*
         * might have gone from GPGSV to GLGSV/BDGSV/QZGSV,
         * in which case accumulate
         *
         * NMEA 4.1 might have gone from GPGVS,sigid=x to GPGSV,sigid=y
         *
         * session->nmea.last_gsv_talker is zero at cycle start
         */
        if ('\0' == session->nmea.last_gsv_talker ||
            ('P' == GSV_TALKER &&
             0 == ubx_sigid)) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "NMEA0183: x%cGSV: new part %d, last_gsv_talker '%#x', "
                     "zeroing\n",
                     GSV_TALKER,
                     session->nmea.part,
                     session->nmea.last_gsv_talker);
            gpsd_zero_satellites(&session->gpsdata);
        }
    }

    session->nmea.last_gsv_talker = GSV_TALKER;
    session->nmea.last_gsv_sigid = ubx_sigid;  // UNUSED
    switch (GSV_TALKER) {
    case 'A':        // GA Galileo
        nmea_gnssid = 3;
        session->nmea.seen_gagsv = true;
        break;
    case 'B':        // GB BeiDou
        FALLTHROUGH
    case 'D':        // BD BeiDou
        nmea_gnssid = 4;
        session->nmea.seen_bdgsv = true;
        break;
    case 'I':        // GI IRNSS
        nmea_gnssid = 6;
        session->nmea.seen_gigsv = true;
        break;
    case 'L':        // GL GLONASS
        nmea_gnssid = 2;
        session->nmea.seen_glgsv = true;
        break;
    case 'N':        // GN GNSS
        session->nmea.seen_gngsv = true;
        break;
    case 'P':        // GP GPS
        session->nmea.seen_gpgsv = true;
        break;
    case 'Q':        // GQ, and PQ (Quectel Querk) QZSS
        // Quectel EC25 & EC21 use PQGSA for QZSS
        FALLTHROUGH
    case 'Z':        // QZ QZSS
        nmea_gnssid = 5;
        session->nmea.seen_qzgsv = true;
        break;
    default:
        // uh, what?
        break;
    }

    for (fldnum = 4; fldnum < count / 4 * 4;) {
        struct satellite_t *sp;
        int nmea_svid;

        if (MAXCHANNELS <= session->gpsdata.satellites_visible) {
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "NMEA0183: xxGSV: internal error - too many "
                     "satellites [%d]!\n",
                     session->gpsdata.satellites_visible);
            gpsd_zero_satellites(&session->gpsdata);
            break;
        }
        sp = &session->gpsdata.skyview[session->gpsdata.satellites_visible];
        nmea_svid = atoi(field[fldnum++]);
        if (0 == nmea_svid) {
            // skip bogus fields
            continue;
        }
        // FIXME: this ignores possible NMEA 4.1 Signal ID hint
        sp->PRN = (short)nmeaid_to_prn(field[0], nmea_svid, nmea_gnssid,
                                       &sp->gnssid, &sp->svid);

        sp->elevation = (double)atoi(field[fldnum++]);
        sp->azimuth = (double)atoi(field[fldnum++]);
        sp->ss = (double)atoi(field[fldnum++]);
        sp->used = false;
        sp->sigid = ubx_sigid;

#ifdef __UNUSED__
        // debug
        GPSD_LOG(LOG_SHOUT, &session->context->errout,
                 "NMEA0183: %s nmeaid_to_prn: nmea_gnssid %d "
                 "nmea_satnum %d ubx_gnssid %d ubx_svid %d nmea2_prn %d "
                 "az %.1f el %.1f\n",
                 field[0], nmea_gnssid, nmea_svid, sp->gnssid, sp->svid,
                 sp->PRN, sp->elevation, sp->azimuth);
#endif  // __UNUSED__

        /* sadly NMEA 4.1 does not tell us which sigid (L1, L2) is
         * used.  So if the ss is zero, do not mark used */
        if (0 < sp->PRN &&
            0 < sp->ss) {
            for (n = 0; n < MAXCHANNELS; n++)
                if (session->nmea.sats_used[n] == (unsigned short)sp->PRN) {
                    sp->used = true;
                    break;
                }
        }
        /*
         * Incrementing this unconditionally falls afoul of chipsets like
         * the Motorola Oncore GT+ that emit empty fields at the end of the
         * last sentence in a GPGSV set if the number of satellites is not
         * a multiple of 4.
         */
        session->gpsdata.satellites_visible++;
    }

#ifdef __UNUSED
    // debug code
    GPSD_LOG(LOG_ERROR, &session->context->errout,
        "NMEA0183: x%cGSV: vis %d bdgsv %d gagsv %d gigsv %d glgsv %d "
        "gngsv %d qpgsv %dqzgsv %d\n",
        GSV_TALKER,
        session->gpsdata.satellites_visible,
        session->nmea.seen_bdgsv,
        session->nmea.seen_gagsv,
        session->nmea.seen_gigsv,
        session->nmea.seen_glgsv,
        session->nmea.seen_gngsv,
        session->nmea.seen_gpgsv,
        session->nmea.seen_qzgsv);
#endif

    /*
     * Alas, we can't sanity check field counts when there are multiple sat
     * pictures, because the visible member counts *all* satellites - you
     * get a bad result on the second and later SV spans.  Note, this code
     * assumes that if any of the special sat pics occur they come right
     * after a stock GPGSV one.
     *
     * FIXME: Add per-talker totals so we can do this check properly.
     */
    if (!(session->nmea.seen_bdgsv ||
          session->nmea.seen_gagsv ||
          session->nmea.seen_gigsv ||
          session->nmea.seen_glgsv ||
          session->nmea.seen_gngsv ||
          session->nmea.seen_qzgsv)) {
        if (session->nmea.part == session->nmea.await
                && atoi(field[3]) != session->gpsdata.satellites_visible) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NMEA0183: xxGSV field 3 value of %d != actual count %d\n",
                     atoi(field[3]), session->gpsdata.satellites_visible);
        }
    }

    // not valid data until we've seen a complete set of parts
    if (session->nmea.part < session->nmea.await) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA0183: xxGSV: Partial satellite data (%d of %d).\n",
                 session->nmea.part, session->nmea.await);
        session->nmea.gsx_more = true;
        return ONLINE_SET;
    }
    session->nmea.gsx_more = false;
    /*
     * This sanity check catches an odd behavior of SiRFstarII receivers.
     * When they can't see any satellites at all (like, inside a
     * building) they sometimes cough up a hairball in the form of a
     * GSV packet with all the azimuth entries 0 (but nonzero
     * elevations).  This behavior was observed under SiRF firmware
     * revision 231.000.000_A2.
     */
    for (n = 0; n < session->gpsdata.satellites_visible; n++) {
        if (0 != session->gpsdata.skyview[n].azimuth) {
            goto sane;
        }
    }
    GPSD_LOG(LOG_WARN, &session->context->errout,
             "NMEA0183: xxGSV: Satellite data no good (%d of %d).\n",
             session->nmea.part, session->nmea.await);
    gpsd_zero_satellites(&session->gpsdata);
    return ONLINE_SET;
  sane:
    session->gpsdata.skyview_time.tv_sec = 0;
    session->gpsdata.skyview_time.tv_nsec = 0;
    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: xxGSV: Satellite data OK (%d of %d).\n",
             session->nmea.part, session->nmea.await);

    // assumes GLGSV or BDGSV group, if present, is emitted after the GPGSV
    if ((session->nmea.seen_bdgsv ||
         session->nmea.seen_gagsv ||
         session->nmea.seen_gigsv ||
         session->nmea.seen_glgsv ||
         session->nmea.seen_gngsv ||
         session->nmea.seen_qzgsv)
        && GSV_TALKER == 'P')
        return ONLINE_SET;

#ifdef __UNUSED
    // debug code
    GPSD_LOG(LOG_ERROR, &session->context->errout,
        "NMEA0183: x%cGSV: set skyview_time %s frac_time %.2f\n", GSV_TALKER,
         timespec_str(&session->gpsdata.skyview_time, ts_buf, sizeof(ts_buf)),
        session->nmea.this_frac_time);
#endif

    return SATELLITE_SET;
#undef GSV_TALKER
}

/* Android GNSS super message
 * A stub.
 */
static gps_mask_t processPGLOR(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $PGLOR,0,FIX,....
     * 1    = sentence version (may not be present)
     * 2    = message subtype
     * ....
     *
     * subtypes:
     *  $PGLOR,[],AGC - ??
     *  $PGLOR,[],CPU - CPU Loading
     *  $PGLOR,[],FIN - Request completion status
     *  $PGLOR,0,FIX,seconds - Time To Fix
     *  $PGLOR,[],FTS - Factory Test Status
     *  $PGLOR,[],GFC - GeoFence Fix
     *  $PGLOR,[],GLO - ??
     *  $PGLOR,[],HLA - Value of HULA sensors
     *  $PGLOR,[],IMS - IMES messages
     *  $PGLOR,1,LSQ,hhmmss.ss  - Least squares GNSS fix
     *  $PGLOR,NET    - Report network information
     *  $PGLOR,[],NEW - Indicate new GPS request
     *  $PGLOR,[],PFM - Platform Status
     *  $PGLOR,[],PPS - Indicate PPS time corrections
     *  $PGLOR,5,PWR i - Power consumption report
     *                  Only have doc for 5, Quectel uses 4
     *  $PGLOR,[],RID - Version Information
     *  $PGLOR,2,SAT - GPS Satellite information
     *  $PGLOR,[],SIO - Serial I/O status report
     *  $PGLOR,[],SPA - Spectrum analyzer results
     *  $PGLOR,0,SPD  - Speed, Steps, etc.
     *  $PGLOR,SPL    - ??
     *  $PGLOR,[],SPS - ??
     *  $PGLOR,10,STA - GLL status
     *  $PGLOR,[],SVC - ??
     *  $PGLOR,[],SVD - SV Dopplers detected in the false alarm test.
     *  $PGLOR,[],SMx - Report GPS Summary Information
     *  $PGLOR,[],UNC - ??
     *
     * Are NET and SPL really so different?
     *
     */
    gps_mask_t mask = ONLINE_SET;
    int got_one = 0;

    switch (field[1][0]) {
    case '0':
        if (0 == strncmp("FIX", field[2], 3)) {
            got_one = 1;
            // field 3, time to first fix in seconds
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: PGLOR: FIX, TTFF %s\n",
                     field[3]);
        } else if (0 == strncmp("SPD", field[2], 3)) {
            got_one = 1;
            // field 4, ddmmy.ss UTC
            // field 5, hhmmss.ss UTC
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: PGLOR: SPD, %s %s UTC\n",
                     field[4], field[5]);
        }
        break;
    case '1':
        if (0 == strncmp("LSQ", field[2], 3)) {
            got_one = 1;
            // field 3, hhmmss.ss UTC, only field Quectel supplies
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: PGLOR: LSQ %s UTC\n",
                     field[3]);
        } else if ('0' == field[1][1] &&
                   0 == strncmp("STA", field[2], 3)) {
            // version 10
            got_one = 1;
            // field 3, hhmmss.ss UTC
            // field 7, Position uncertainty meters
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: PGLOR: STA, UTC %s PosUncer  %s\n",
                     field[3], field[7]);
        }
        break;
    }
    if (0 != got_one) {
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: PGLOR: seq %s type %s\n",
                 field[1], field[2]);
    }
    return mask;
}

/* smart watch sensors
 * A stub.
 */
static gps_mask_t processPRHS(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $PRHS ,type,....
     *   type = message type
     *
     * Yes: $PRHS[space],
     *
     * types:
     * $PRHS ,ACC,9.952756,0.37819514,1.3165021,20150305072428436*44
     * $PRHS ,COM,238.09642,16.275442,82.198425,20150305072428824*43
     * $PRHS ,GYR,0.0,0.0,0.0,20150305072428247*4D
     * $PRHS ,LAC,0.23899937,0.009213656,0.02143073,20150305072428437*46
     * $PRHS ,MAG,47.183502,-51.789,-2.7145,20150305072428614*41
     * $PRHS ,ORI,187.86511,-2.1546898,-82.405205,20150305072428614*53
     * $PRHS ,RMC,20150305072427985*55
     *
     */
    gps_mask_t mask = ONLINE_SET;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: PRHS: type %s\n",
             field[1]);
    return mask;
}

/* Garmin Estimated Position Error */
static gps_mask_t processPGRME(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $PGRME,15.0,M,45.0,M,25.0,M*22
     * 1    = horizontal error estimate
     * 2    = units
     * 3    = vertical error estimate
     * 4    = units
     * 5    = spherical error estimate
     * 6    = units
     * *
     * * Garmin won't say, but the general belief is that these are 50% CEP.
     * * We follow the advice at <http://gpsinformation.net/main/errors.htm>.
     * * If this assumption changes here, it should also change in garmin.c
     * * where we scale error estimates from Garmin binary packets, and
     * * in libgpsd_core.c where we generate $PGRME.
     */
    gps_mask_t mask = ONLINE_SET;

    if ('M' == field[2][0] &&
        'M' == field[4][0] &&
        'M' == field[6][0]) {
        session->newdata.epx = session->newdata.epy =
            safe_atof(field[1]) * (1 / sqrt(2))
                      * (GPSD_CONFIDENCE / CEP50_SIGMA);
        session->newdata.epv =
            safe_atof(field[3]) * (GPSD_CONFIDENCE / CEP50_SIGMA);
        session->newdata.sep =
            safe_atof(field[5]) * (GPSD_CONFIDENCE / CEP50_SIGMA);
        mask = HERR_SET | VERR_SET | PERR_IS;
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: PGRME: epx=%.2f epy=%.2f sep=%.2f\n",
             session->newdata.epx,
             session->newdata.epy,
             session->newdata.sep);
    return mask;
}

/* Garmin GPS Fix Data Sentence
 *
 * FIXME: seems to happen after cycle ender, so little happens...
 */
static gps_mask_t processPGRMF(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
 /*
  * $PGRMF,290,293895,160305,093802,13,5213.1439,N,02100.6511,E,A,2,0,226,2,1*11
  *
  * 1 = GPS week
  * 2 = GPS seconds
  * 3 = UTC Date ddmmyy
  * 4 = UTC time hhmmss
  * 5 = GPS leap seconds
  * 6 = Latitude ddmm.mmmm
  * 7 = N or S
  * 8 = Longitude dddmm.mmmm
  * 9 = E or W
  * 10 = Mode, M = Manual, A = Automatic
  * 11 = Fix type, 0 = No fix, 2 = 2D fix, 2 = 3D fix
  * 12 = Ground Speed, 0 to 1151 km/hr
  * 13 = Course over ground, 0 to 359 degrees true
  * 14 = pdop, 0 to 9
  * 15 = dop, 0 to 9
  */
    gps_mask_t mask = ONLINE_SET;
    timespec_t ts_tow = {0, 0};

    /* Some garmin fail due to GPS Week Roll Over
     * Ignore their UTC date/time, use their GPS week, GPS tow and
     * leap seconds to decide the correct time */
    if (isdigit((int)field[5][0])) {
        session->context->leap_seconds = atoi(field[5]);
        session->context->valid = LEAP_SECOND_VALID;
    }
    if (isdigit((int)field[1][0]) &&
        isdigit((int)field[2][0]) &&
        0 < session->context->leap_seconds) {
        // have GPS week, tow and leap second
        unsigned short week = atol(field[1]);
        ts_tow.tv_sec = atol(field[2]);
        ts_tow.tv_nsec = 0;
        session->newdata.time = gpsd_gpstime_resolv(session, week, ts_tow);
        mask |= TIME_SET;
        // (long long) cast for 32/64 bit compat
        GPSD_LOG(LOG_SPIN, &session->context->errout,
                 "NMEA0183: PGRMF gps time %lld\n",
                 (long long)session->newdata.time.tv_sec);
    } else if (0 == merge_hhmmss(field[4], session) &&
               0 == merge_ddmmyy(field[3], session)) {
        // fall back to UTC if we need and can
        // (long long) cast for 32/64 bit compat
        GPSD_LOG(LOG_SPIN, &session->context->errout,
                 "NMEA0183: PGRMF gps time %lld\n",
                 (long long)session->newdata.time.tv_sec);
        mask |= TIME_SET;
    }
    if ('A' != field[10][0]) {
        // Huh?
        return mask;
    }
    if (0 == do_lat_lon(&field[6], &session->newdata)) {
        mask |= LATLON_SET;
    }
    switch (field[11][0]) {
    default:
        // Huh?
        break;
    case '0':
        session->newdata.mode = MODE_NO_FIX;
        mask |= MODE_SET;
        break;
    case '1':
        session->newdata.mode = MODE_2D;
        mask |= MODE_SET;
        break;
    case '2':
        session->newdata.mode = MODE_3D;
        mask |= MODE_SET;
        break;
    }
    session->newdata.speed = safe_atof(field[12]) / MPS_TO_KPH;
    session->newdata.track = safe_atof(field[13]);
    mask |= SPEED_SET | TRACK_SET;
    if ('\0' != field[14][0]) {
        session->gpsdata.dop.pdop = safe_atof(field[14]);
        mask |= DOP_SET;
    }
    if ('\0' != field[15][0]) {
        session->gpsdata.dop.tdop = safe_atof(field[15]);
        mask |= DOP_SET;
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: PGRMF: pdop %.1f tdop %.1f \n",
             session->gpsdata.dop.pdop,
             session->gpsdata.dop.tdop);
    return mask;
}

/* Garmin Map Datum
 *
 * FIXME: seems to happen after cycle ender, so nothing happens...
 */
static gps_mask_t processPGRMM(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $PGRMM,NAD83*29
     * 1    = Map Datum
     */
    gps_mask_t mask = ONLINE_SET;

    if ('\0' != field[1][0]) {
        strlcpy(session->newdata.datum, field[1],
                sizeof(session->newdata.datum));
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: PGRMM: datum=%.40s\n",
             session->newdata.datum);
    return mask;
}

// Garmin Altitude Information
static gps_mask_t processPGRMZ(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $PGRMZ,246,f,3*1B
     * 1    = Altitude (probably MSL) in feet
     * 2    = f (feet)
     * 3    = Mode
     *         1 = No Fix
     *         2 = 2D Fix
     *         3 = 3D Fix
     *
     * From: Garmin Proprietary NMEA 0183 Sentences
     *       technical Specifications
     *       190-00684-00, Revision C December 2008
     */
    gps_mask_t mask = ONLINE_SET;

    // codacy does not like strlen()
    if ('f' == field[2][0] &&
        0 < strnlen(field[1], 20)) {
        // have a GPS altitude, must be 3D
        // seems to be altMSL.  regressions show this matches GPGGA MSL
        session->newdata.altMSL = atoi(field[1]) * FEET_TO_METERS;
        mask |= (ALTITUDE_SET);
    }
    switch (field[3][0]) {
    default:
        // Huh?
        break;
    case '1':
        session->newdata.mode = MODE_NO_FIX;
        mask |= MODE_SET;
        break;
    case '2':
        session->newdata.mode = MODE_2D;
        mask |= MODE_SET;
        break;
    case '3':
        session->newdata.mode = MODE_3D;
        mask |= MODE_SET;
        break;
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: PGRMZ: altMSL %.2f mode %d\n",
             session->newdata.altMSL,
             session->newdata.mode);
    return mask;
}

// Magellan Status
static gps_mask_t processPMGNST(int c UNUSED, char *field[],
                                struct gps_device_t *session)
{
    /*
     * $PMGNST,01.75,3,T,816,11.1,-00496,00*43
     * 1 = Firmware version number
     * 2 = Mode (1 = no fix, 2 = 2D fix, 3 = 3D fix)
     * 3 = T if we have a fix
     * 4 = battery percentage left in tenths of a percent
     * 5 = time left on the GPS battery in hours
     * 6 = numbers change (freq. compensation?)
     * 7 = PRN number receiving current focus
     */
    gps_mask_t mask = ONLINE_SET;
    int newmode = atoi(field[3]);

    if ('T' == field[4][0]) {
        switch(newmode) {
        default:
            session->newdata.mode = MODE_NO_FIX;
            break;
        case 2:
            session->newdata.mode = MODE_2D;
            break;
        case 3:
            session->newdata.mode = MODE_3D;
            break;
        }
    } else {
        // Can report 3D fix, but 'F' for no fix
        session->newdata.mode = MODE_NO_FIX;
    }
    mask |= MODE_SET;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: PMGNST: mode: %d\n",
             session->newdata.mode);
    return mask;
}

// SiRF Estimated Position Errors
static gps_mask_t processPSRFEPE(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $PSRFEPE,100542.000,A,0.7,6.82,10.69,0.0,180.0*24
     * 1    = UTC Time hhmmss.sss
     * 2    = Status.  A = Valid, V = Data not valid
     * 3    = HDOP
     * 4    = EHPE meters (Estimated Horizontal Position Error)
     * 5    = EVPE meters (Estimated Vertical Position Error)
     * 6    = EHVE meters (Estimated Speed Over Ground/Velocity Error)
     * 7    = EHE degrees (Estimated Heading Error)
     *
     * SiRF won't say if these are 1-sigma or what...
     */
    gps_mask_t mask = STATUS_SET;

    // get time/ valid or not
    if ('\0' != field[1][0]) {
        if (0 == merge_hhmmss(field[1], session)) {
            register_fractional_time(field[0], field[1], session);
            if (0 == session->nmea.date.tm_year) {
                GPSD_LOG(LOG_WARN, &session->context->errout,
                         "NMEA0183: can't use PSRFEPE time until after ZDA "
                         "or RMC has supplied a year.\n");
            } else {
                mask |= TIME_SET;
            }
        }
    }
    if ('A' != field[2][0]) {
        // Huh?
        return mask;
    }

    if ('\0' != field[3][0]) {
        /* This adds nothing, it just agrees with the gpsd calculation
         * from the skyview.  Which is a nice confirmation. */
        session->gpsdata.dop.hdop = safe_atof(field[3]);
        mask |= DOP_SET;
    }
    if ('\0' != field[4][0]) {
        // EHPE (Estimated Horizontal Position Error)
        session->newdata.eph = safe_atof(field[4]);
        mask |= HERR_SET;
    }

    if ('\0' != field[5][0]) {
        // Estimated Vertical Position Error (meters, 0.01 resolution)
        session->newdata.epv = safe_atof(field[5]);
        mask |= VERR_SET;
    }

    if ('\0' != field[6][0]) {
        // Estimated Horizontal Speed Error meters/sec
        session->newdata.eps = safe_atof(field[6]);
    }

    if ('\0' != field[7][0]) {
        // Estimated Heading Error degrees
        session->newdata.epd = safe_atof(field[7]);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: PSRFEPE: hdop=%.1f eph=%.1f epv=%.1f eps=%.1f "
             "epd=%.1f\n",
             session->gpsdata.dop.hdop,
             session->newdata.eph,
             session->newdata.epv,
             session->newdata.eps,
             session->newdata.epd);
    return mask;
}

/* NMEA Map Datum
 *
 * FIXME: seems to happen after cycle ender, so nothing happens...
 */
static gps_mask_t processDTM(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $GPDTM,W84,C*52
     * $GPDTM,xxx,x,xx.xxxx,x,xx.xxxx,x,,xxx*hh<CR><LF>
     * 1    = Local datum code (xxx):
     *          W84 – WGS84
     *          W72 – WGS72
     *          S85 – SGS85
     *          P90 – PE90
     *          999 – User defined
     *          IHO datum code
     * 2     = Local datum sub code (x)
     * 3     = Latitude offset in minutes (xx.xxxx)
     * 4     = Latitude offset mark (N: +, S: -) (x)
     * 5     = Longitude offset in minutes (xx.xxxx)
     * 6     = Longitude offset mark (E: +, W: -) (x)
     * 7     = Altitude offset in meters. Always null
     * 8     = Datum (xxx):
     *          W84 – WGS84
     *          W72 – WGS72
     *          S85 – SGS85
     *          P90 – PE90
     *          999 – User defined
     *          IHO datum code
     * 9    = checksum
     */
    int i;
    static struct
    {
        char *code;
        char *name;
    } codes[] = {
        {"W84", "WGS84"},
        {"W72", "WGS72"},
        {"S85", "SGS85"},
        {"P90", "PE90"},
        {"999", "User Defined"},
        {"", ""},
    };

    gps_mask_t mask = ONLINE_SET;

    if ('\0' == field[1][0]) {
        return mask;
    }

    for (i = 0; ; i++) {
        if ('\0' == codes[i].code[0]) {
            // not found
            strlcpy(session->newdata.datum, field[1],
                    sizeof(session->newdata.datum));
            break;
        }
        if (0 ==strcmp(codes[i].code, field[1])) {
            strlcpy(session->newdata.datum, codes[i].name,
                    sizeof(session->newdata.datum));
            break;
        }
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: xxDTM: datum=%.40s\n",
             session->newdata.datum);
    return mask;
}

// NMEA 3.0 Estimated Position Error
static gps_mask_t processGBS(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $GPGBS,082941.00,2.4,1.5,3.9,25,,-43.7,27.5*65
     *  1) UTC time of the fix associated with this sentence (hhmmss.ss)
     *  2) Expected error in latitude (meters)
     *  3) Expected error in longitude (meters)
     *  4) Expected error in altitude (meters)
     *  5) PRN of most likely failed satellite
     *  6) Probability of missed detection for most likely failed satellite
     *  7) Estimate of bias in meters on most likely failed satellite
     *  8) Standard deviation of bias estimate
     *  9) NMEA 4.1 GNSS ID
     * 10) NMEA 4.1 Signal ID
     *     Checksum
     *
     * Fields 2, 3 and 4 are one standard deviation.
     */
    gps_mask_t mask = ONLINE_SET;

    // register fractional time for end-of-cycle detection
    register_fractional_time(field[0], field[1], session);

    // check that we're associated with the current fix
    if (session->nmea.date.tm_hour == DD(field[1]) &&
        session->nmea.date.tm_min == DD(field[1] + 2) &&
        session->nmea.date.tm_sec == DD(field[1] + 4)) {
        session->newdata.epy = safe_atof(field[2]);
        session->newdata.epx = safe_atof(field[3]);
        session->newdata.epv = safe_atof(field[4]);
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: GBS: epx=%.2f epy=%.2f epv=%.2f\n",
                 session->newdata.epx,
                 session->newdata.epy,
                 session->newdata.epv);
        mask = HERR_SET | VERR_SET;
    } else {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA0183: second in $GPGBS error estimates doesn't match.\n");
    }
    return mask;
}

// Time & Date
static gps_mask_t processZDA(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    /*
     * $GPZDA,160012.71,11,03,2004,-1,00*7D
     * 1) UTC time (hours, minutes, seconds, may have fractional subsecond)
     * 2) Day, 01 to 31
     * 3) Month, 01 to 12
     * 4) Year (4 digits)
     * 5) Local zone description, 00 to +- 13 hours
     * 6) Local zone minutes description, apply same sign as local hours
     * 7) Checksum
     *
     * Note: some devices, like the u-blox ANTARIS 4h, are known to ship ZDAs
     * with some fields blank under poorly-understood circumstances (probably
     * when they don't have satellite lock yet).
     */
    gps_mask_t mask = ONLINE_SET;
    int year, mon, mday, century;
    char ts_buf[TIMESPEC_LEN];

    if ('\0' == field[1][0] ||
        '\0' == field[2][0] ||
        '\0' == field[3][0] ||
        '\0' == field[4][0]) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: ZDA fields are empty\n");
        return mask;
    }

    if (0 != merge_hhmmss(field[1], session)) {
        // bad time
        return mask;
    }

    /*
     * We didn't register fractional time here because we wanted to leave
     * ZDA out of end-of-cycle detection. Some devices sensibly emit it only
     * when they have a fix, so watching for it can make them look
     * like they have a variable fix reporting cycle.  But later thought
     * was to not throw out good data because it is inconvenient.
     */
    year = atoi(field[4]);
    mon = atoi(field[3]);
    mday = atoi(field[2]);
    century = year - year % 100;
    if (1900 > year  ||
        2200 < year) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: malformed ZDA year: %s\n",  field[4]);
    } else if (1 > mon ||
               12 < mon) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: malformed ZDA month: %s\n",  field[3]);
    } else if (1 > mday ||
               31 < mday) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: malformed ZDA day: %s\n",  field[2]);
    } else {
        gpsd_century_update(session, century);
        session->nmea.date.tm_year = year - 1900;
        session->nmea.date.tm_mon = mon - 1;
        session->nmea.date.tm_mday = mday;
        session->newdata.time = gpsd_utc_resolve(session);
        register_fractional_time(field[0], field[1], session);
        mask = TIME_SET;
    }
    GPSD_LOG(LOG_DATA, &session->context->errout,
         "NMEA0183: ZDA time %s\n",
          timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)));
    return mask;
}

static gps_mask_t processHDG(int c UNUSED, char *field[],
                             struct gps_device_t *session)
{
    /*
     *  $SDHDG,234.6,,,1.3,E*34
     *
     *  $--HDG,h.h,d.d,a,v.v,a*hh<CR><LF>
     *  Magnetic sensor heading, degrees
     *  Magnetic deviation, degrees E/W
     *  Magnetic variation, degrees, E/W
     *
     *  1. To obtain Magnetic Heading:
     *  Add Easterly deviation (E) to Magnetic Sensor Reading
     *  Subtract Westerly deviation (W) from Magnetic Sensor Reading
     *  2. To obtain True Heading:
     *  Add Easterly variation (E) to Magnetic Heading
     *  Subtract Westerly variation (W) from Magnetic Heading
     *  3. Variation and deviation fields shall be null fields if unknown.
     */

    gps_mask_t mask = ONLINE_SET;
    double sensor_heading;
    double magnetic_deviation;

    if ('\0' == field[1][0]) {
        // no data
        return mask;
    }
    sensor_heading = safe_atof(field[1]);
    if ((0.0 > sensor_heading) ||
        (360.0 < sensor_heading)) {
        // bad data */
        return mask;
    }
    magnetic_deviation = safe_atof(field[2]);
    if ((0.0 > magnetic_deviation) ||
        (360.0 < magnetic_deviation)) {
        // bad data
        return mask;
    }
    switch (field[2][0]) {
    case 'E':
        sensor_heading += magnetic_deviation;
        break;
    case 'W':
        sensor_heading += magnetic_deviation;
        break;
    default:
        // ignore
        break;
    }

    // good data
    session->newdata.magnetic_track = sensor_heading;
    mask |= MAGNETIC_TRACK_SET;

    // get magnetic variation
    if ('\0' != field[3][0] &&
        '\0' != field[4][0]) {
        session->newdata.magnetic_var = safe_atof(field[3]);

        switch (field[4][0]) {
        case 'E':
            // no change
            mask |= MAGNETIC_TRACK_SET;
            break;
        case 'W':
            session->newdata.magnetic_var = -session->newdata.magnetic_var;
            mask |= MAGNETIC_TRACK_SET;
            break;
        default:
            // huh?
            session->newdata.magnetic_var = NAN;
            break;
        }
    }


    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: $SDHDG heading %lf var %.1f\n",
             session->newdata.magnetic_track,
             session->newdata.magnetic_var);
    return mask;
}

static gps_mask_t processHDT(int c UNUSED, char *field[],
                             struct gps_device_t *session)
{
    /*
     * $HEHDT,341.8,T*21
     *
     * $xxHDT,x.x*hh<cr><lf>
     *
     * The only data field is true heading in degrees.
     * The following field is required to be 'T' indicating a true heading.
     * It is followed by a mandatory nmea_checksum.
     */
    gps_mask_t mask = ONLINE_SET;
    double heading;

    if ('\0' == field[1][0]) {
        // no data
        return mask;
    }
    heading = safe_atof(field[1]);
    if (0.0 > heading ||
        360.0 < heading) {
        // bad data
        return mask;
    }
    // True heading
    session->gpsdata.attitude.heading = heading;

    mask |= ATTITUDE_SET;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: $xxHDT heading %lf.\n",
             session->gpsdata.attitude.heading);
    return mask;
}

static gps_mask_t processDBT(int c UNUSED, char *field[],
                                struct gps_device_t *session)
{
    /*
     * $SDDBT,7.7,f,2.3,M,1.3,F*05
     * 1) Depth below sounder in feet
     * 2) Fixed value 'f' indicating feet
     * 3) Depth below sounder in meters
     * 4) Fixed value 'M' indicating meters
     * 5) Depth below sounder in fathoms
     * 6) Fixed value 'F' indicating fathoms
     * 7) Checksum.
     *
     * In real-world sensors, sometimes not all three conversions are reported.
     */
    gps_mask_t mask = ONLINE_SET;

    if ('\0' != field[3][0]) {
        session->newdata.depth = safe_atof(field[3]);
        mask |= (ALTITUDE_SET);
    } else if ('\0' != field[1][0]) {
        session->newdata.depth = safe_atof(field[1]) * FEET_TO_METERS;
        mask |= (ALTITUDE_SET);
    } else if ('\0' != field[5][0]) {
        session->newdata.depth = safe_atof(field[5]) * FATHOMS_TO_METERS;
        mask |= (ALTITUDE_SET);
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: mode %d, depth %lf.\n",
             session->newdata.mode,
             session->newdata.depth);
    return mask;
}

// $xxTHS -- True Heading and Status
static gps_mask_t processTHS(int c UNUSED, char *field[],
                             struct gps_device_t *session)
{
    /*
     * $GNTHS,121.15.A*1F<CR><LF>
     * 1  - Heading, degrees True
     * 2  - Mode indicator
     *      'A’ = Autonomous
     *      'E’ = Estimated (dead reckoning)
     *      'M’ = Manual input
     *      'S’ = Simulator
     *      'V’ = Data not valid
     * 3  - Checksum
     */
    gps_mask_t mask = ONLINE_SET;
    double heading;

    if ('\0' == field[1][0] ||
        '\0' == field[2][0]) {
        // no data
        return mask;
    }
    if ('V' == field[2][0]) {
        // invalid data
        // ignore A, E, M and S for now
        return mask;
    }
    heading = safe_atof(field[1]);
    if ((0.0 > heading) ||
        (360.0 < heading)) {
        // bad data
        return mask;
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: $xxTHS heading %lf mode %s\n",
             heading, field[2]);

    return mask;
}

// GPS Text message
static gps_mask_t processTXT(int count, char *field[],
                             struct gps_device_t *session)
{
    /*
     * $GNTXT,01,01,01,PGRM inv format*2A
     * 1                   Number of sentences for full data
     * 1                   Sentence 1 of 1
     * 01                  Message type
     *       00 - error
     *       01 - warning
     *       02 - notice
     *       07 - user
     * PGRM inv format     ASCII text
     *
     * Can occur with talker IDs:
     *   BD (Beidou),
     *   GA (Galileo),
     *   GB (Beidou),
     *   GI (IRNSS
     *   GL (GLONASS),
     *   GN (GLONASS, any combination GNSS),
     *   GP (GPS, SBAS, QZSS),
     *   GQ (QZSS).
     *   PQ (QZSS). Quectel Quirk
     *   QZ (QZSS).
     */
    gps_mask_t mask = ONLINE_SET;
    int msgType = 0;
    char *msgType_txt = "Unknown";

    if (5 != count) {
      return mask;
    }

    msgType = atoi(field[3]);

    switch ( msgType ) {
    case 0:
        msgType_txt = "Error";
        break;
    case 1:
        msgType_txt = "Warning";
        break;
    case 2:
        msgType_txt = "Notice";
        break;
    case 7:
        msgType_txt = "User";
        break;
    }

    // maximum text length unknown, guess 80
    GPSD_LOG(LOG_WARN, &session->context->errout,
             "NMEA0183: TXT: %.10s: %.80s\n",
             msgType_txt, field[4]);
    return mask;
}

static gps_mask_t processTNTHTM(int c UNUSED, char *field[],
                                struct gps_device_t *session)
{
    /*
     * Proprietary sentence for True North Technologies Magnetic Compass.
     * This may also apply to some Honeywell units since they may have been
     * designed by True North.

     $PTNTHTM,14223,N,169,N,-43,N,13641,2454*15

     HTM,x.x,a,x.x,a,x.x,a,x.x,x.x*hh<cr><lf>
     Fields in order:
     1. True heading (compass measurement + deviation + variation)
     2. magnetometer status character:
     C = magnetometer calibration alarm
     L = low alarm
     M = low warning
     N = normal
     O = high warning
     P = high alarm
     V = magnetometer voltage level alarm
     3. pitch angle
     4. pitch status character - see field 2 above
     5. roll angle
     6. roll status character - see field 2 above
     7. dip angle
     8. relative magnitude horizontal component of earth's magnetic field
     *hh          mandatory nmea_checksum

     By default, angles are reported as 26-bit integers: weirdly, the
     technical manual says either 0 to 65535 or -32768 to 32767 can
     occur as a range.
     */
    gps_mask_t mask = ONLINE_SET;

    // True heading
    session->gpsdata.attitude.heading = safe_atof(field[1]);
    session->gpsdata.attitude.mag_st = *field[2];
    session->gpsdata.attitude.pitch = safe_atof(field[3]);
    session->gpsdata.attitude.pitch_st = *field[4];
    session->gpsdata.attitude.roll = safe_atof(field[5]);
    session->gpsdata.attitude.roll_st = *field[6];
    session->gpsdata.attitude.dip = safe_atof(field[7]);
    session->gpsdata.attitude.mag_x = safe_atof(field[8]);
    mask |= (ATTITUDE_SET);

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: $PTNTHTM heading %lf (%c).\n",
             session->gpsdata.attitude.heading,
             session->gpsdata.attitude.mag_st);
    return mask;
}

static gps_mask_t processTNTA(int c UNUSED, char *field[],
                              struct gps_device_t *session)
{
    /*
     * Proprietary sentence for iSync GRClok/LNRClok.

     $PTNTA,20000102173852,1,T4,,,6,1,0*32

     1. Date/time in format year, month, day, hour, minute, second
     2. Oscillator quality 0:warming up, 1:freerun, 2:disciplined.
     3. Always T4. Format indicator.
     4. Interval ppsref-ppsout in [ns]. Blank if no ppsref.
     5. Fine phase comparator in approx. [ns]. Always close to -500 or
        +500 if not disciplined. Blank if no ppsref.
     6. iSync Status.  0:warming up or no light, 1:tracking set-up,
        2:track to PPSREF, 3:synch to PPSREF, 4:Free Run. Track OFF,
        5:FR. PPSREF unstable, 6:FR. No PPSREF, 7:FREEZE, 8:factory
        used, 9:searching Rb line
     7. GPS messages indicator. 0:do not take account, 1:take account,
        but no message, 2:take account, partially ok, 3:take account,
        totally ok.
     8. Transfer quality of date/time. 0:no, 1:manual, 2:GPS, older
        than x hours, 3:GPS, fresh.

     */
    gps_mask_t mask = ONLINE_SET;

    if (0 == strcmp(field[3], "T4")) {
        struct oscillator_t *osc = &session->gpsdata.osc;
        unsigned int quality = atoi(field[2]);
        unsigned int delta = atoi(field[4]);
        unsigned int fine = atoi(field[5]);
        unsigned int status = atoi(field[6]);
        char deltachar = field[4][0];

        osc->running = (0 < quality);
        osc->reference = (deltachar && (deltachar != '?'));
        if (osc->reference) {
            if (500 > delta) {
                osc->delta = fine;
            } else {
                osc->delta = ((delta < 500000000) ? delta : 1000000000 - delta);
            }
        } else {
            osc->delta = 0;
        }
        osc->disciplined = ((quality == 2) && (status == 3));
        mask |= OSCILLATOR_SET;

        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: PTNTA,T4: quality=%s, delta=%s, fine=%s,"
                 "status=%s\n",
                 field[2], field[4], field[5], field[6]);
    }
    return mask;
}

#ifdef OCEANSERVER_ENABLE
static gps_mask_t processOHPR(int c UNUSED, char *field[],
                              struct gps_device_t *session)
{
    /*
     * Proprietary sentence for OceanServer Magnetic Compass.

     OHPR,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x,x.x*hh<cr><lf>
     Fields in order:
     1. Azimuth
     2. Pitch Angle
     3. Roll Angle
     4. Sensor temp, degrees centigrade
     5. Depth (feet)
     6. Magnetic Vector Length
     7-9. 3 axis Magnetic Field readings x,y,z
     10. Acceleration Vector Length
     11-13. 3 axis Acceleration Readings x,y,z
     14. Reserved
     15-16. 2 axis Gyro Output, X,y
     17. Reserved
     18. Reserved
     *hh          mandatory nmea_checksum
     */
    gps_mask_t mask = ONLINE_SET;

    // True heading?
    session->gpsdata.attitude.heading = safe_atof(field[1]);
    session->gpsdata.attitude.pitch = safe_atof(field[2]);
    session->gpsdata.attitude.roll = safe_atof(field[3]);
    session->gpsdata.attitude.temp = safe_atof(field[4]);
    session->gpsdata.attitude.depth = safe_atof(field[5]) * FEET_TO_METERS;
    session->gpsdata.attitude.mag_len = safe_atof(field[6]);
    session->gpsdata.attitude.mag_x = safe_atof(field[7]);
    session->gpsdata.attitude.mag_y = safe_atof(field[8]);
    session->gpsdata.attitude.mag_z = safe_atof(field[9]);
    session->gpsdata.attitude.acc_len = safe_atof(field[10]);
    session->gpsdata.attitude.acc_x = safe_atof(field[11]);
    session->gpsdata.attitude.acc_y = safe_atof(field[12]);
    session->gpsdata.attitude.acc_z = safe_atof(field[13]);
    session->gpsdata.attitude.gyro_x = safe_atof(field[15]);
    session->gpsdata.attitude.gyro_y = safe_atof(field[16]);
    mask |= (ATTITUDE_SET);

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: Heading %lf.\n", session->gpsdata.attitude.heading);
    return mask;
}
#endif  // OCEANSERVER_ENABLE

/* Ashtech sentences take this format:
 * $PASHDR,type[,val[,val]]*CS
 * type is an alphabetic subsentence type
 *
 * Oxford Technical Solutions (OxTS) also uses the $PASHR sentence,
 * but with a very different sentence contents:
 * $PASHR,HHMMSS.SSS,HHH.HH,T,RRR.RR,PPP.PP,aaa.aa,r.rrr,p.ppp,h.hhh,Q1,Q2*CS
 *
 * so field 1 in ASHTECH is always alphabetic and numeric in OXTS
 *
 */
static gps_mask_t processPASHR(int c UNUSED, char *field[],
                               struct gps_device_t *session)
{
    gps_mask_t mask = ONLINE_SET;
    char ts_buf[TIMESPEC_LEN];

    if (0 == strcmp("ACK", field[1])) {
        // ACK
        GPSD_LOG(LOG_DATA, &session->context->errout, "NMEA0183: PASHR,ACK\n");
        return ONLINE_SET;
    } else if (0 == strcmp("MCA", field[1])) {
        // MCA, raw data
        GPSD_LOG(LOG_DATA, &session->context->errout, "NMEA0183: PASHR,MCA\n");
        return ONLINE_SET;
    } else if (0 == strcmp("NAK", field[1])) {
        // NAK
        GPSD_LOG(LOG_DATA, &session->context->errout, "NMEA0183: PASHR,NAK\n");
        return ONLINE_SET;
    } else if (0 == strcmp("PBN", field[1])) {
        // PBN, position data
        // FIXME: decode this for ECEF
        GPSD_LOG(LOG_DATA, &session->context->errout, "NMEA0183: PASHR,PBN\n");
        return ONLINE_SET;
    } else if (0 == strcmp("POS", field[1])) {  // 3D Position
        /* $PASHR,POS,
         *
         * 2: position type:
         *      0 = autonomous
         *      1 = position differentially corrected with RTCM code
         *      2 = position differentially corrected with CPD float solution
         *      3 = position is CPD fixed solution
         */
        mask |= MODE_SET | STATUS_SET | CLEAR_IS;
        if ('\0' == field[2][0]) {
            // empty first field means no 3D fix is available
            session->newdata.status = STATUS_UNK;
            session->newdata.mode = MODE_NO_FIX;
        } else {
            int satellites_used;

            // if we make it this far, we at least have a 3D fix
            session->newdata.mode = MODE_3D;
            if (1 <= atoi(field[2]))
                session->newdata.status = STATUS_DGPS;
            else
                session->newdata.status = STATUS_GPS;

            /* don't use as this breaks the GPGSV counter
             * session->gpsdata.satellites_used = atoi(field[3]);  */
            satellites_used = atoi(field[3]);
            if (0 == merge_hhmmss(field[4], session)) {
                register_fractional_time(field[0], field[4], session);
                mask |= TIME_SET;
            }
            if (0 == do_lat_lon(&field[5], &session->newdata)) {
                mask |= LATLON_SET;
                if ('\0' != field[9][0]) {
                    // altitude is already WGS 84
                    session->newdata.altHAE = safe_atof(field[9]);
                    mask |= ALTITUDE_SET;
                }
            }
            session->newdata.track = safe_atof(field[11]);
            session->newdata.speed = safe_atof(field[12]) / MPS_TO_KPH;
            session->newdata.climb = safe_atof(field[13]);
            if ('\0' != field[14][0]) {
                session->gpsdata.dop.pdop = safe_atof(field[14]);
                mask |= DOP_SET;
            }
            if ('\0' != field[15][0]) {
                session->gpsdata.dop.hdop = safe_atof(field[15]);
                mask |= DOP_SET;
            }
            if ('\0' != field[16][0]) {
                session->gpsdata.dop.vdop = safe_atof(field[16]);
                mask |= DOP_SET;
            }
            if ('\0' != field[17][0]) {
                session->gpsdata.dop.tdop = safe_atof(field[17]);
                mask |= DOP_SET;
            }
            mask |= (SPEED_SET | TRACK_SET | CLIMB_SET);
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: PASHR,POS: hhmmss=%s lat=%.2f lon=%.2f"
                     " altHAE=%.f"
                     " speed=%.2f track=%.2f climb=%.2f mode=%d status=%d"
                     " pdop=%.2f hdop=%.2f vdop=%.2f tdop=%.2f used=%d\n",
                     field[4], session->newdata.latitude,
                     session->newdata.longitude, session->newdata.altHAE,
                     session->newdata.speed, session->newdata.track,
                     session->newdata.climb, session->newdata.mode,
                     session->newdata.status, session->gpsdata.dop.pdop,
                     session->gpsdata.dop.hdop, session->gpsdata.dop.vdop,
                     session->gpsdata.dop.tdop, satellites_used);
        }
    } else if (0 == strcmp("RID", field[1])) {  // Receiver ID
        (void)snprintf(session->subtype, sizeof(session->subtype) - 1,
                       "%s ver %s", field[2], field[3]);
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: PASHR,RID: subtype=%s mask={}\n",
                 session->subtype);
        return mask;
    } else if (0 == strcmp("SAT", field[1])) {  // Satellite Status
        struct satellite_t *sp;
        int i, n = session->gpsdata.satellites_visible = atoi(field[2]);

        session->gpsdata.satellites_used = 0;
        for (i = 0, sp = session->gpsdata.skyview;
            sp < session->gpsdata.skyview + n; sp++, i++) {

            sp->PRN = (short)atoi(field[3 + i * 5 + 0]);
            sp->azimuth = (double)atoi(field[3 + i * 5 + 1]);
            sp->elevation = (double)atoi(field[3 + i * 5 + 2]);
            sp->ss = safe_atof(field[3 + i * 5 + 3]);
            sp->used = false;
            if ('U' == field[3 + i * 5 + 4][0]) {
                sp->used = true;
                session->gpsdata.satellites_used++;
            }
        }
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: PASHR,SAT: used=%d\n",
                 session->gpsdata.satellites_used);
        session->gpsdata.skyview_time.tv_sec = 0;
        session->gpsdata.skyview_time.tv_nsec = 0;
        mask |= SATELLITE_SET | USED_IS;

    } else if (0 == strcmp("T", field[3])) {   // Assume OxTS PASHR
        // FIXME: decode OxTS $PASHDR, time is wrong, breaks cycle order
        if (0 == merge_hhmmss(field[1], session)) {
            // register_fractional_time(field[0], field[1], session);
            // mask |= TIME_SET; confuses cycle order
        }
        // Assume true heading
        session->gpsdata.attitude.heading = safe_atof(field[2]);
        session->gpsdata.attitude.roll = safe_atof(field[4]);
        session->gpsdata.attitude.pitch = safe_atof(field[5]);
        // mask |= ATTITUDE_SET;  * confuses cycle order ??
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: PASHR (OxTS) time %s, heading %lf.\n",
                  timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)),
                  session->gpsdata.attitude.heading);
    }
    return mask;
}

static gps_mask_t processMWD(int c UNUSED, char *field[],
                              struct gps_device_t *session)
{
    /*
     * xxMWD - Wind direction and speed
     * $xxMWD,x.x,T,x.x,M,x.x,N,x.x,M*hh<cr><lf>
     * Fields in order:
     * 1. wind direction, 0 to 359, True
     * 2. T
     * 3. wind direction, 0 to 359, Magnetic
     * 4. M
     * 5. wind speed, knots
     * 6. N
     * 7. wind speed, meters/sec
     * 8. M
     * *hh          mandatory nmea_checksum
     */
    gps_mask_t mask = ONLINE_SET;

    session->newdata.wanglet = safe_atof(field[1]);
    session->newdata.wanglem = safe_atof(field[3]);
    session->newdata.wspeedt = safe_atof(field[7]);
    mask |= NAVDATA_SET;

    GPSD_LOG(LOG_DATA, &session->context->errout,
        "NMEA0183: xxMWD wanglet %.2f wanglem %.2f wspeedt %.2f\n",
        session->newdata.wanglet,
        session->newdata.wanglem,
        session->newdata.wspeedt);
    return mask;
}

static gps_mask_t processMWV(int c UNUSED, char *field[],
                              struct gps_device_t *session)
{
    /*
     * xxMWV - Wind speed and angle
     * $xxMWV,x.x,a,x.x,a,A*hh<cr><lf>
     * Fields in order:
     * 1. wind angle, 0 to 359, True
     * 2. R = Relative (apparent), T = Theoretical (calculated)
     *    Is T magnetic or true??
     * 3. wind speed
     * 4. wind speed units K/M/N/S
     * 6. A = Valid, V = invalid
     * *hh          mandatory nmea_checksum
     */
    gps_mask_t mask = ONLINE_SET;

    if (('R' == field[2][0]) &&
        ('N' == field[4][0]) &&
        ('A' == field[5][0])) {
        // relative, knots, and valid
        session->newdata.wangler = safe_atof(field[1]);
        session->newdata.wspeedr = safe_atof(field[3]) * KNOTS_TO_MPS;
        mask |= NAVDATA_SET;
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
        "NMEA0183: xxMWV wangler %.2f wspeedr %.2f\n",
        session->newdata.wangler,
        session->newdata.wspeedr);
    return mask;
}

static gps_mask_t processPQVERNO(int c UNUSED, char* field[],
                                 struct gps_device_t* session)
{
    /* Example request & response are provided courtesy of Quectel below.
     * This command is not publically documented, but Quectel support
     * provided this description via email. This has been tested on
     * Quectel version L70-M39, but all recent (2022) versions of Quectel
     * support this command is well.
     *
     * Request:
     * $PQVERNO,R*3F
     *
     * Response:
     * $PQVERNO,R,L96NR01A03S,2018/07/30,04:17*6B
     *
     * Description of the 6 fields are below.
     *
     * 1. $PQVERNO,              Query command
     * 2. R,                     Read
     * 3. L96NR01A03S,           Quectel firmware version number
     * 4. 2018/07/30,            Firmware build date
     * 5. 04:17*                 Firmware build time
     * 6. 6B                     Checksum
     */

    if (0 == strncmp(session->nmea.field[0], "PQVERNO", sizeof("PQVERNO")) &&
        '\0' != field[2][0]) {
        (void)snprintf(session->subtype1, sizeof(session->subtype1),
                       "%s,%s,%s",
                       field[2], field[3], field[4]);
    }

    return ONLINE_SET;
}

static gps_mask_t processPMTK001(int c UNUSED, char *field[],
                                 struct gps_device_t *session)
{
    int reason;

    // ACK / NACK
    reason = atoi(field[2]);
    if (4 == reason) {
        // ACK
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA0183: MTK ACK: %s\n", field[1]);
    } else {
        // NACK
        const char *mtk_reasons[] = {
            "Invalid",
            "Unsupported",
            "Valid but Failed",
            "Valid success",       // unused, see above
            "Unknown",             // gpsd only
        };
        if (0 > reason ||
            4 < reason) {
            // WTF?
            reason = 4;
        }
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: MTK NACK: %s, reason: %s\n",
                 field[1], mtk_reasons[reason]);
    }
    return ONLINE_SET;
}

static gps_mask_t processPMTK424(int c UNUSED, char *field[],
                                 struct gps_device_t *session)
{
    // PPS pulse width response
    /*
     * Response will look something like: $PMTK424,0,0,1,0,69*12
     * The pulse width is in field 5 (69 in this example).  This
     * sentence is poorly documented at:
     * http://www.trimble.com/embeddedsystems/condor-gps-module.aspx?dtID=documentation
     *
     * Packet Type: 324 PMTK_API_SET_OUTPUT_CTL
     * Packet meaning
     * Write the TSIP/antenna/PPS configuration data to the Flash memory.
     * DataField [Data0]:TSIP Packet[on/off]
     * 0 - Disable TSIP output (Default).
     * 1 - Enable TSIP output.
     * [Data1]:Antenna Detect[on/off]
     * 0 - Disable antenna detect function (Default).
     * 1 - Enable antenna detect function.
     * [Data2]:PPS on/off
     * 0 - Disable PPS function.
     * 1 - Enable PPS function (Default).
     * [Data3]:PPS output timing
     * 0 - Always output PPS (Default).
     * 1 - Only output PPS when GPS position is fixed.
     * [Data4]:PPS pulse width
     * 1~16367999: 61 ns~(61x 16367999) ns (Default = 69)
     *
     * The documentation does not give the units of the data field.
     * Andy Walls <andy@silverblocksystems.net> says:
     *
     * "The best I can figure using an oscilloscope, is that it is
     * in units of 16.368000 MHz clock cycles.  It may be
     * different for any other unit other than the Trimble
     * Condor. 69 cycles / 16368000 cycles/sec = 4.216 microseconds
     * [which is the pulse width I have observed]"
     *
     * Support for this theory comes from the fact that crystal
     * TXCOs with a 16.368MHZ period are commonly available from
     * multiple vendors. Furthermore, 61*69 = 4209, which is
     * close to the observed cycle time and suggests that the
     * documentation is trying to indicate 61ns units.
     *
     * He continues:
     *
     * "I chose [127875] because to divides 16368000 nicely and the
     * pulse width is close to 1/100th of a second.  Any number
     * the user wants to use would be fine.  127875 cycles /
     * 16368000 cycles/second = 1/128 seconds = 7.8125
     * milliseconds"
     */

    // too short?  Make it longer
    if (127875 > atoi(field[5])) {
        (void)nmea_send(session, "$PMTK324,0,0,1,0,127875");
    }
    return ONLINE_SET;
}

static gps_mask_t processPMTK705(int count, char *field[],
                                 struct gps_device_t *session)
{
    /* Trimble version:
     * $PMTK705,AXN_1.30,0000,20090609,*20<CR><LF>
     *
     * 0 PMTK705
     * 1 ReleaseStr - Firmware release name and version
     * 2 Build_ID   - Build ID
     * 3 Date code  - YYYYMMDD
     * 4 Checksum
     *
     * Quectel Querk.  L26.
     * $PMTK705,AXN_3.20_3333_13071501,0003,QUECTEL-L26,*1E<CR><LF>
     *
     * 0 PMTK705
     * 1 ReleaseStr - Firmware release name and version
     * 2 Build_ID   - Build ID
     * 3 Date code  - Product Model
     * 4 SDK Version (optional)
     * * Checksum
    */

    // set device subtype
    if (4 == count) {
        (void)snprintf(session->subtype, sizeof(session->subtype),
                       "%s,%s,%s",
                       field[1], field[2], field[3]);
    } else {
        // Once again Quectel goes their own way...
        (void)snprintf(session->subtype, sizeof(session->subtype),
                       "%s,%s,%s,%s",
                       field[1], field[2], field[3], field[4]);
    }

    if ('\0' == session->subtype1[0]) {
        /* Query for the Quectel firmware version.
         * Quectel GPS receivers containing an MTK chipset use
         * this command to return their FW version.
         * From
         * https://forums.quectel.com/t/determine-nmea-version-of-l76-l/3882/5
         * "$PQVERNO is an internal command and used to query Quectel FW
         * version. We haven’t added this internal command in GNSS
         * protocol spec."
         */
        (void)nmea_send(session, "$PQVERNO,R");
    }

    return ONLINE_SET;
}

//  Recommended Minimum 3D GNSS Data
static gps_mask_t processPSTI030(int count UNUSED, char *field[],
                                 struct gps_device_t *session)
{
    /*
     * $PSTI,030,hhmmss.sss,A,dddmm.mmmmmmm,a,dddmm.mmmmmmm,a,x.x,
            x.x,x.x,x.x,ddmmyy,a.x.x,x.x*hh<CR><LF>
     * 1     030          Sentence ID
     * 2     225446.334   Time of fix 22:54:46 UTC
     * 3     A            Status of Fix: A = Autonomous, valid;
     *                                 V = invalid
     * 4,5   4916.45,N    Latitude 49 deg. 16.45 min North
     * 6,7   12311.12,W   Longitude 123 deg. 11.12 min West
     * 8     103.323      Mean Sea Level meters
     * 9     0.00         East Velocity meters/sec
     * 10    0.00         North Velocity meters/sec
     * 11    0.00         Up Velocity meters/sec
     * 12    181194       Date of fix  18 November 1994
     * 13    A            FAA mode indicator
     *                        See faa_mode() for possible mode values.
     * 14    1.2          RTK Age
     * 15    4.2          RTK Ratio
     * 16    *68          mandatory nmea_checksum
     *
     * In private email, SkyTraq says F mode is 10x more accurate
     * than R mode.
     */
    gps_mask_t mask = ONLINE_SET;

    if (0 != strncmp(session->device_type->type_name, "Skytraq", 7)) {
        // this is skytraq, but not marked yet, so probe for Skytraq
        // Send MID 0x02, to get back MID 0x80
        (void)gpsd_write(session, "\xA0\xA1\x00\x02\x02\x01\x03\x0d\x0a",9);
    }

    if ('V' == field[3][0] ||
        'N' == field[13][0]) {
        // nav warning, or FAA not valid, ignore the rest of the data
        session->newdata.status = STATUS_UNK;
        session->newdata.mode = MODE_NO_FIX;
        mask |= MODE_SET | STATUS_SET;
    } else if ('A' == field[3][0]) {
        double east, north, climb, age, ratio;

        // data valid
        if ('\0' != field[2][0] &&
            '\0' != field[12][0]) {
            // good date and time
            if (0 == merge_hhmmss(field[2], session) &&
                0 == merge_ddmmyy(field[12], session)) {
                mask |= TIME_SET;
                register_fractional_time( "PSTI030", field[2], session);
            }
        }
        if (0 == do_lat_lon(&field[4], &session->newdata)) {
            session->newdata.mode = MODE_2D;
            mask |= LATLON_SET;
            if ('\0' != field[8][0]) {
                // altitude is MSL
                session->newdata.altMSL = safe_atof(field[8]);
                mask |= ALTITUDE_SET;
                session->newdata.mode = MODE_3D;
                // Let gpsd_error_model() deal with geoid_sep and altHAE
            }
            mask |= MODE_SET;
        }
        /* convert ENU to track
         * this has more precision than GPVTG, GPVTG comes earlier
         * in the cycle */
        east = safe_atof(field[9]);     // east velocity m/s
        north = safe_atof(field[10]);   // north velocity m/s
        climb = safe_atof(field[11]);   // up velocity m/s
        age = safe_atof(field[14]);
        ratio = safe_atof(field[15]);

        session->newdata.NED.velN = north;
        session->newdata.NED.velE = east;
        session->newdata.NED.velD = -climb;
        if (0.05 < (age + ratio)) {
            // don't report age == ratio == 0.0
            session->newdata.dgps_age = age;
            session->gpsdata.fix.base.ratio = ratio;
        }

        mask |= VNED_SET | STATUS_SET;

        session->newdata.status = faa_mode(field[13][0]);
        if (STATUS_RTK_FIX == session->newdata.status ||
            STATUS_RTK_FLT == session->newdata.status) {
            // RTK_FIX or RTK_FLT
            session->gpsdata.fix.base.status = session->newdata.status;
        }
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: PSTI,030: ddmmyy=%s hhmmss=%s lat=%.2f lon=%.2f "
             "status=%d, RTK(Age=%.1f Ratio=%.1f)\n",
             field[12], field[2],
             session->newdata.latitude,
             session->newdata.longitude,
             session->newdata.status,
             session->newdata.dgps_age,
             session->newdata.base.ratio);
    return mask;
}

/* Skytraq RTK Baseline, fixed base to rover or moving base
 * Same as $PSTI.035, except that is moving base to rover
 * PX1172RH
 */
static gps_mask_t processPSTI032(int count UNUSED, char *field[],
                                 struct gps_device_t *session)
{
    /*
     * $PSTI,032,041457.000,170316,A,R,0.603,‐0.837,‐0.089,1.036,144.22,,,,,*1B
     *
     * 2  UTC time,  hhmmss.sss
     * 3  UTC Date, ddmmyy
     * 4  Status, ‘V’ = Void ‘A’ = Active
     * 5  Mode indicator, 'O' = Float RTK, ‘F’ = Float RTK. ‘R’ = Fixed RTK
     * 6  East‐projection of baseline, meters
     * 7  North‐projection of baseline, meters
     * 8  Up‐projection of baseline, meters
     * 9  Baseline length, meters
     * 10 Baseline course 144.22, true degrees
     * 11 Reserved
     * 12 Reserved
     * 13 Reserved
     * 14 Reserved
     * 15 Reserved
     * 16 Checksum
     */
    gps_mask_t mask = ONLINE_SET;
    struct baseline_t *base = &session->gpsdata.fix.base;

    if ('A' != field[4][0]) {
        //  status not valid
        return mask;
    }

    // Status Valid
    if ('\0' != field[2][0] &&
        '\0' != field[3][0]) {
        // have date and time
        if (0 == merge_hhmmss(field[2], session) &&
            0 == merge_ddmmyy(field[3], session)) {
            // good date and time
            mask |= TIME_SET;
            register_fractional_time("PSTI032", field[2], session);
        }
    }

    if ('F' == field[5][0] ||
        'O' == field[5][0]) {
        // Floating point RTK
        // 'O' is undocuemented, private email says it is jsut a crappy 'F'.
        base->status = STATUS_RTK_FLT;
    } else if ('R' == field[5][0]) {
        // Fixed point RTK
        base->status = STATUS_RTK_FIX;
    } else {
        // WTF?
        return mask;
    }

    base->east = safe_atof(field[6]);
    base->north = safe_atof(field[7]);
    base->up = safe_atof(field[8]);
    base->length = safe_atof(field[9]);
    base->course = safe_atof(field[10]);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: PSTI,032: RTK Baseline mode %d E %.3f  N %.3f  U %.3f "
             "length %.3f course %.3f\n",
             base->status, base->east, base->north, base->up,
             base->length, base->course);
    return mask;
}

/* Skytraq  RTK RAW Measurement Monitoring Data
 */
static gps_mask_t processPSTI033(int count UNUSED, char *field[],
                                 struct gps_device_t *session)
{
    /*
     * $PSTI,033,hhmmss.sss,ddmmyy,x,R,x,G,x,x,,,C,x,x,,,E,x,x,,,R,x,x,,*hh
     * $PSTI,033,110431.000,150517,2,R,1,G,1,0,,,C,0,0,,,E,0,0,,,R,0,0,,*72
     *
     * 2  UTC time,  hhmmss.sss
     * 3  UTC Date, ddmmyy
     * 4  "2", version
     * 5  Receiver, R = Rover, B = Base
     * 6  total cycle‐slipped raw measurements
     * 7  "G", GPS
     * 8  cycle slipped L1
     * 9  cycle slipped L2
     * 10 reserved
     * 11 reserved
     * 12 "C", BDS
     * 12 cycle slipped B1
     * 14 cycle slipped B2
     * 15 reserved
     * 16 reserved
     * 17 "E", Galileo
     * 18 cycle slipped E1
     * 19 cycle slipped E5b
     * 20 reserved
     * 21 reserved
     * 22 "R", GLONASS
     * 23 cycle slipped G1
     * 24 cycle slipped G2
     * 25 reserved
     * 26 reserved
     * 27 Checksum
     */
    gps_mask_t mask = ONLINE_SET;
    char receiver;
    unsigned total, L1, L2, B1, B2, E1, E5b, G1, G2;

    if ('2' != field[4][0]) {
        //  we only understand version 2
        return mask;
    }
    if ('B' != field[5][0] &&
        'R' != field[5][0]) {
        //  Huh?  Rover or Base
        return mask;
    }
    receiver = field[5][0];

    if ('\0' != field[2][0] &&
        '\0' != field[3][0]) {
        // have date and time
        if (0 == merge_hhmmss(field[2], session) &&
            0 == merge_ddmmyy(field[3], session)) {
            // good date and time
            mask |= TIME_SET;
            register_fractional_time("PSTI033", field[2], session);
        }
    }
    total = atoi(field[6]);
    L1 = atoi(field[7]);
    L2 = atoi(field[8]);
    B1 = atoi(field[13]);
    B2 = atoi(field[14]);
    E1 = atoi(field[18]);
    E5b = atoi(field[19]);
    G1 = atoi(field[23]);
    G2 = atoi(field[24]);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: PSTI,033: RTK RAW receiver %c Slips: total %u L1 %u "
             "L2 %u B1 %u B2 %u E1 %u E5b %u G1 %u G2 %u\n",
             receiver, total, L1, L2, B1, B2, E1, E5b, G1, G2);
    return mask;
}

// Skytraq RTK Baseline, moving base to moving rover
// PX1172RH
static gps_mask_t processPSTI035(int count UNUSED, char *field[],
                                 struct gps_device_t *session)
{
    /*
     * $PSTI,035,041457.000,170316,A,R,0.603,‐0.837,‐0.089,1.036,144.22,,,,,*1B
     *
     * 2  UTC time,  hhmmss.sss
     * 3  UTC Date, ddmmyy
     * 4  Status, ‘V’ = Void ‘A’ = Active
     * 5  Mode indicator, ‘F’ = Float RTK. ‘R’ = FIxed RTK
     * 6  East‐projection of baseline, meters
     * 7  North‐projection of baseline, meters
     * 8  Up‐projection of baseline, meters
     * 9  Baseline length, meters
     * 10 Baseline course 144.22, true degrees
     * 11 Reserved
     * 12 Reserved
     * 13 Reserved
     * 14 Reserved
     * 15 Reserved
     * 16 Checksum
     */

    gps_mask_t mask = ONLINE_SET;
    struct baseline_t *base = &session->gpsdata.attitude.base;

    // RTK Baseline Data of Rover Moving Base Receiver
    if ('\0' != field[2][0] &&
        '\0' != field[3][0]) {
        // good date and time
        if (0 == merge_hhmmss(field[2], session) &&
            0 == merge_ddmmyy(field[3], session)) {
            mask |= TIME_SET;
            register_fractional_time( "PSTI035", field[2], session);
        }
    }
    if ('A' != field[4][0]) {
        // No valid data, except time, sort of
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA0183: PSTI,035: not valid\n");
        base->status = STATUS_UNK;
        return mask;
    }
    if ('F' == field[5][0]) {
        // Float RTX
        base->status = STATUS_RTK_FLT;
    } else if ('R' == field[5][0]) {
        // Fix RTX
        base->status = STATUS_RTK_FIX;
    } // else ??

    base->east = safe_atof(field[6]);
    base->north = safe_atof(field[7]);
    base->up = safe_atof(field[8]);
    base->length = safe_atof(field[9]);
    base->course = safe_atof(field[10]);
    mask |= ATTITUDE_SET;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: PSTI,035: RTK Baseline mode %d E %.3f  N %.3f  U %.3f "
             "length %.3f course %.3f\n",
             base->status, base->east, base->north, base->up,
             base->length, base->course);
    return mask;
}

// Skytraq PSTI,036 – Heading, Pitch and Roll
// PX1172RH
static gps_mask_t processPSTI036(int count UNUSED, char *field[],
                                 struct gps_device_t *session)
{
    /*
     * $PSTI,036,054314.000,030521,191.69,‐16.35,0.00,R*4D
     *
     * 2  UTC time,  hhmmss.sss
     * 3  UTC Date, ddmmyy
     * 4  Heading, 0 - 359.9, when mode == R, degrees
     * 5  Pitch, -90 - 90, when mode == R, degrees
     * 6  Roll, -90 - 90, when mode == R, degrees
     * 7  Mode
     *     'N’ = Data not valid
     *     'A’ = Autonomous mode
     *     'D’ = Differential mode
     *     'E’ = Estimated (dead reckoning) mode
     *     'M’ = Manual input mode
     *     'S’ = Simulator mode
     *     'F’ = Float RTK
     *     'R’ = Fix RTK
     * 8  Checksum
     */

    gps_mask_t mask = ONLINE_SET;
    int mode;

    if ('\0' != field[2][0] &&
        '\0' != field[3][0]) {
        // good date and time
        if (0 == merge_hhmmss(field[2], session) &&
            0 == merge_ddmmyy(field[3], session)) {
            mask |= TIME_SET;
            register_fractional_time("PSTI036", field[2], session);
        }
    }
    if ('\0' == field[7][0] ||
        'N' == field[7][0]) {
        // No valid data, except time, sort of
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA0183: PSTI,036: not valid\n");
        return mask;
    }
    // good attitude data to use
    session->gpsdata.attitude.mtime = gpsd_utc_resolve(session);
    session->gpsdata.attitude.heading = safe_atof(field[4]);
    session->gpsdata.attitude.pitch = safe_atof(field[5]);
    session->gpsdata.attitude.roll = safe_atof(field[6]);
    mode = faa_mode(field[7][0]);

    mask |= ATTITUDE_SET;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: PSTI,036: mode %d heading %.2f  pitch %.2f roll %.2f\n",
             mode,
             session->gpsdata.attitude.heading,
             session->gpsdata.attitude.pitch,
             session->gpsdata.attitude.roll);
    return mask;
}

/*
 * Skytraq undocumented debug sentences take this format:
 * $STI,type,val*CS
 * type is a 2 char subsentence type
 * Note: NO checksum
 */
static gps_mask_t processSTI(int count, char *field[],
                             struct gps_device_t *session)
{
    gps_mask_t mask = ONLINE_SET;

    if (0 != strncmp(session->device_type->type_name, "Skytraq", 7)) {
        // this is skytraq, but marked yet, so probe for Skytraq
        // Send MID 0x02, to get back MID 0x80
        (void)gpsd_write(session, "\xA0\xA1\x00\x02\x02\x01\x03\x0d\x0a",9);
    }

    if ( 0 == strcmp( field[1], "IC") ) {
        // $STI,IC,error=XX, this is always very bad, but undocumented
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA0183: Skytraq: $STI,%s,%s\n", field[1], field[2]);
        return mask;
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA0183: STI,%s: Unknown type, Count: %d\n", field[1], count);

    return mask;
}

/**************************************************************************
 *
 * Entry points begin here
 *
 **************************************************************************/

// parse an NMEA sentence, unpack it into a session structure
gps_mask_t nmea_parse(char *sentence, struct gps_device_t * session)
{
    typedef gps_mask_t(*nmea_decoder) (int count, char *f[],
                                       struct gps_device_t * session);
    static struct
    {
        char *name;
        char *name1;            // 2nd field to match, as is $PSTI,030
        int nf;                 // minimum number of fields required to parse
        bool cycle_continue;    // cycle continuer?
        nmea_decoder decoder;
    } nmea_phrase[NMEA_NUM] = {
        {"PGLOR", NULL, 2,  false, processPGLOR},  // Android something...
        {"PGRMB", NULL, 0,  false, NULL},     // ignore Garmin DGPS Beacon Info
        {"PGRMC", NULL, 0,  false, NULL},        // ignore Garmin Sensor Config
        {"PGRME", NULL, 7,  false, processPGRME},
        {"PGRMF", NULL, 15, false, processPGRMF},  // Garmin GPS Fix Data
        {"PGRMH", NULL, 0,  false, NULL},     // ignore Garmin Aviation Height
        {"PGRMI", NULL, 0,  false, NULL},          // ignore Garmin Sensor Init
        {"PGRMM", NULL, 2,  false, processPGRMM},  // Garmin Map Datum
        {"PGRMO", NULL, 0,  false, NULL},     // ignore Garmin Sentence Enable
        {"PGRMT", NULL, 0,  false, NULL},          // ignore Garmin Sensor Info
        {"PGRMV", NULL, 0,  false, NULL},     // ignore Garmin 3D Velocity Info
        {"PGRMZ", NULL, 4,  false, processPGRMZ},
            /*
             * Basic sentences must come after the PG* ones, otherwise
             * Garmins can get stuck in a loop that looks like this:
             *
             * 1. A Garmin GPS in NMEA mode is detected.
             *
             * 2. PGRMC is sent to reconfigure to Garmin binary mode.
             *    If successful, the GPS echoes the phrase.
             *
             * 3. nmea_parse() sees the echo as RMC because the talker
             *    ID is ignored, and fails to recognize the echo as
             *    PGRMC and ignore it.
             *
             * 4. The mode is changed back to NMEA, resulting in an
             *    infinite loop.
             */
        {"AAM", NULL, 0,  false, NULL},    // ignore Waypoint Arrival Alarm
        {"ACCURACY", NULL, 1,  true, processACCURACY},
        {"ALM", NULL, 0,  false, NULL},    // ignore GPS Almanac Data
        {"APB", NULL, 0,  false, NULL},    // ignore Autopilot Sentence B
        {"BOD", NULL, 0,  false, NULL},    // Bearing Origin to Destination
        // Bearing & Distance to Waypoint, Great Circle
        {"BWC", NULL, 12, false, processBWC},
        {"DBT", NULL, 7,  false, processDBT},
        {"DPT", NULL, 0,  false, NULL},       // ignore depth
        {"DTM", NULL, 2,  false, processDTM}, // datum
        {"GBS", NULL, 7,  false, processGBS},
        {"GGA", NULL, 13, false, processGGA},
        {"GLC", NULL, 0,  false, NULL},   // ignore Geographic Position, LoranC
        {"GLL", NULL, 7,  true, processGLL},
        {"GNS", NULL, 13, false, processGNS},
        {"GRS", NULL, 4,  false, processGRS},    // GNSS Range Residuals
        {"GSA", NULL, 18, false, processGSA},
        {"GST", NULL, 8,  false, processGST},
        {"GSV", NULL, 0,  false, processGSV},
        // ignore Heading, Deviation and Variation
        {"HDG", NULL, 0,  false, processHDG},
        {"HDT", NULL, 1,  false, processHDT},
        {"HWBIAS", NULL, 0, false, NULL},       // Unknown HuaWei sentence
        {"MLA", NULL, 0,  false, NULL},         // GLONASS Almana Data
        {"MSS", NULL, 0,  false, NULL},         // beacon receiver status
        {"MTW", NULL, 0,  false, NULL},         // ignore Water Temperature
        {"MWD", NULL, 0,  false, processMWD},   // Wind Direction and Speed
        {"MWV", NULL, 0,  false, processMWV},   // Wind Speed and Angle
#ifdef OCEANSERVER_ENABLE
        {"OHPR", NULL, 18, false, processOHPR},
#endif  // OCEANSERVER_ENABLE
        {"OSD", NULL, 0,  false, NULL},             // ignore Own Ship Data
        // general handler for Ashtech
        {"PASHR", NULL, 3, false, processPASHR},
        // Furuno proprietary
        {"PERDACK", NULL, 4, false, NULL},          // ACK
        {"PERDCRD", NULL, 15, false, NULL},         // NLOSMASK?
        {"PERDCRG", "DCR", 6, false, NULL},         // QZSS DC report
        {"PERDCRJ", "FREQ", 9, false, NULL},        // Jamming Status
        {"PERDCRP", NULL, 9, false, NULL},          // Position
        {"PERDCRQ", NULL, 11, false, NULL},         // Galileo SAR
        {"PERDCRW", "TPS1", 8, false, NULL},        // Time
        {"PERDCRW", "TPS2", 12, false, NULL},       // PPS
        {"PERDCRW", "TPS3", 11, false, NULL},       // Position Mode
        {"PERDCRW", "TPS4", 13, false, NULL},       // GCLK
        {"PERDMSG", NULL, 3, false, NULL},          // Message
        {"PERDSYS", "ANTSEL", 5, false, NULL},      // Antenna
        {"PERDSYS", "FIXSESSION", 5, false, NULL},  // Fix Session
        {"PERDSYS", "GPIO", 3, false, NULL},        // GPIO
        {"PERDSYS", "VERSION", 6, false, NULL},     // Version
        // Jackson Labs proprietary
        {"PJLTS", NULL, 11,  false, NULL},          // GPSDO status
        {"PJLTV", NULL, 4,  false, NULL},           // Time and 3D velocity
        {"PMGNST", NULL, 8, false, processPMGNST},  // Magellan Status
        // MediaTek proprietary
        {"PMTK001", NULL, 3, false, processPMTK001},  // ACK
        {"PMTK010", NULL, 2, false, NULL},          // System Message
        {"PMTK011", NULL, 2, false, NULL},          // Text Message
        {"PMTK424", NULL, 3, false, processPMTK424},
        {"PMTK705", NULL, 4, false, processPMTK705},
        // MediaTek/Trimble Satellite Channel Status
        {"PMTKCHN", NULL, 0, false, NULL},
        // Quectel proprietary
        {"PQVERNO", NULL, 5, false, processPQVERNO},  // Version
        // smart watch sensors, Yes: space!
        {"PRHS ", NULL, 2,  false, processPRHS},
        {"PRWIZCH", NULL, 0, false, NULL},          // Rockwell Channel Status
        {"PSRF140", NULL, 0, false, NULL},          // SiRF ephemeris
        {"PSRF150", NULL, 0, false, NULL},          // SiRF flow control
        {"PSRF151", NULL, 0, false, NULL},          // SiRF Power
        {"PSRF152", NULL, 0, false, NULL},          // SiRF ephemeris
        {"PSRF155", NULL, 0, false, NULL},          // SiRF proprietary
        {"PSRFEPE", NULL, 7, false, processPSRFEPE},  // SiRF Estimated Errors
        /*
         * Skytraq sentences take this format:
         * $PSTI,type[,val[,val]]*CS
         * type is a 2 or 3 digit subsentence type
         *
         * Note: these sentences can be at least 105 chars long.
         * That violates the NMEA 3.01 max of 82.
         */
        // 1 PPS Timing report ID
        {"PSTI", "000", 4, false, NULL},
        // Active Antenna Status Report
        {"PSTI", "001", 2, false, NULL},
        // GPIO 10 event-triggered time & position stamp.
        {"PSTI", "005", 2, false, NULL},
        //  Recommended Minimum 3D GNSS Data
        {"PSTI", "030", 16, false, processPSTI030},
        // RTK Baseline
        {"PSTI", "032", 16, false, processPSTI032},
        // RTK RAW Measurement Monitoring Data
        {"PSTI", "033", 27, false,  processPSTI033},
        // RTK Baseline Data of Rover Moving Base Receiver
        {"PSTI", "035", 8, false, processPSTI035},
        // Heading, Pitch and Roll Messages of vehicle
        {"PSTI", "036", 2, false, processPSTI036},
        // $PSTM ST Micro STA8088xx/STA8089xx/STA8090xx
        {"PSTM", NULL, 0, false, NULL},
        {"PTFTTXT", NULL, 0, false, NULL},        // unknown uptime
        {"PTKM", NULL, 0, false, NULL},           // Robertson RGC12 Gyro
        {"PTNLRHVR", NULL, 0, false, NULL},       // Trimble Software Version
        {"PTNLRPT", NULL, 0, false, NULL},        // Trimble Serial Port COnfig
        {"PTNLRSVR", NULL, 0, false, NULL},       // Trimble Firmware Version
        {"PTNLRZD", NULL, 0, false, NULL},        // Extended Time and Date
        {"PTNTA", NULL, 8, false, processTNTA},
        {"PTNTHTM", NULL, 9, false, processTNTHTM},
        {"PUBX", NULL, 0, false, NULL},         // u-blox and Antaris
        {"QSM", NULL, 3, false, NULL},          // QZSS DC Report
        {"RLM", NULL, 0, false, NULL},          // Return Link Message
        // ignore Recommended Minimum Navigation Info, waypoint
        {"RMB", NULL, 0,  false, NULL},         // Recommended Min Nav Info
        {"RMC", NULL, 8,  false, processRMC},
        {"ROT", NULL, 0,  false, NULL},         // ignore Rate of Turn
        {"RPM", NULL, 0,  false, NULL},         // ignore Revolutions
        {"RSA", NULL, 0,  false, NULL},         // Rudder Sensor Angle
        {"RTE", NULL, 0,  false, NULL},         // ignore Routes
        {"STI", NULL, 2,  false, processSTI},   // $STI  Skytraq
        {"THS", NULL, 0,  false, processTHS},   // True Heading and Status
        {"TXT", NULL, 5,  false, processTXT},
        {"VBW", NULL, 0,  false, NULL},         // Dual Ground/Water Speed
        {"VDO", NULL, 0,  false, NULL},         // Own Vessel's Information
        {"VDR", NULL, 0,  false, NULL},         // Set and Drift
        {"VHW", NULL, 0,  false, NULL},         // Water Speed and Heading
        {"VLW", NULL, 0,  false, NULL},         // Dual ground/water distance
        {"VTG", NULL, 5,  false, processVTG},
        {"XDR", NULL, 0,  false, NULL},         // $HCXDR, IMU?
        {"XTE", NULL, 0,  false, NULL},         // Cross-Track Error
        {"ZDA", NULL ,4,  false, processZDA},
        {NULL, NULL,  0,  false, NULL},         // no more
    };

    int count;
    gps_mask_t mask = 0;
    unsigned i, thistag = 0, lasttag;
    char *p, *e;
    volatile char *t;
    char ts_buf1[TIMESPEC_LEN];
    char ts_buf2[TIMESPEC_LEN];
    bool skytraq_sti = false;
    size_t mlen;

    /*
     * We've had reports that on the Garmin GPS-10 the device sometimes
     * (1:1000 or so) sends garbage packets that have a valid checksum
     * but are like 2 successive NMEA packets merged together in one
     * with some fields lost.  Usually these are much longer than the
     * legal limit for NMEA, so we can cope by just tossing out overlong
     * packets.  This may be a generic bug of all Garmin chipsets.
     */
    // codacy does not like strlen()
    mlen = strnlen(sentence, NMEA_MAX + 1);
    if (NMEA_MAX < mlen) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA0183: Overlong packet of %zd+ chars rejected.\n",
                 mlen);
        return ONLINE_SET;
    }

    // make an editable copy of the sentence
    (void)strlcpy((char *)session->nmea.fieldcopy, sentence,
                  sizeof(session->nmea.fieldcopy) - 1);
    // discard the checksum part
    for (p = (char *)session->nmea.fieldcopy;
         ('*' != *p) && (' ' <= *p);) {
        ++p;
    }
    if ('*' == *p) {
        *p++ = ',';             // otherwise we drop the last field
    }
#ifdef SKYTRAQ_ENABLE_UNUSED
    // $STI is special, no trailing *, or chacksum
    if (0 != strncmp( "STI,", sentence, 4)) {
        skytraq_sti = true;
        *p++ = ',';             // otherwise we drop the last field
    }
#endif
    *p = '\0';
    e = p;

    // split sentence copy on commas, filling the field array
    count = 0;
    t = p;                      // end of sentence
    p = (char *)session->nmea.fieldcopy + 1;  // beginning of tag, 'G' not '$'
    // while there is a search string and we haven't run off the buffer...
    while ((NULL != p) &&
           (p <= t)) {
        session->nmea.field[count] = p;      // we have a field. record it
        if (NULL != (p = strchr(p, ','))) {  // search for the next delimiter
            *p = '\0';                       // replace it with a NUL
            count++;                         // bump the counters and continue
            p++;
        }
    }

    // point remaining fields at empty string, just in case
    for (i = (unsigned int)count; i < NMEA_MAX_FLD; i++) {
        session->nmea.field[i] = e;
    }

    // sentences handlers will tell us when they have fractional time
    session->nmea.latch_frac_time = false;
    // GSA and GSV will set this if more in that series to come.
    session->nmea.gsx_more = false;

#ifdef __UNUSED
    // debug
    GPSD_LOG(0, &session->context->errout,
             "NMEA0183: got %s\n", session->nmea.field[0]);
#endif // __UNUSED

    // dispatch on field zero, the sentence tag
    for (i = 0; i < NMEA_NUM; ++i) {
        char *s = session->nmea.field[0];
        if (NULL == nmea_phrase[i].name) {
            mask = ONLINE_SET;
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: Unknown sentence type %s\n",
                     session->nmea.field[0]);
            break;
        }
        // strnlen() to shut up codacy
        if (3 == strnlen(nmea_phrase[i].name, 4) &&
            !skytraq_sti) {
            // $STI is special
            s += 2;             // skip talker ID
        }
        if (0 != strcmp(nmea_phrase[i].name, s)) {
            // no match
            continue;
        }
        if (NULL != nmea_phrase[i].name1 &&
            0 != strcmp(nmea_phrase[i].name1, session->nmea.field[1])) {
            // no match on field 2.  As in $PSTI,030,
            continue;
        }
        // got a match
        if (NULL == nmea_phrase[i].decoder) {
            // no decoder for this sentence
            mask = ONLINE_SET;
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: No decoder for sentence type %s\n",
                     session->nmea.field[0]);
            break;
        }
        if (count < nmea_phrase[i].nf) {
            // sentence to short
            mask = ONLINE_SET;
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA0183: Sentence %s too short\n",
                     session->nmea.field[0]);
            break;
        }
        mask = (nmea_phrase[i].decoder)(count, session->nmea.field,
                                        session);
        session->nmea.cycle_continue = nmea_phrase[i].cycle_continue;
        /*
         * Must force this to be nz, as we're going to rely on a zero
         * value to mean "no previous tag" later.
         */
        // FIXME: this fails on Skytrak, $PSTI,xx, many different xx
        thistag = i + 1;
        break;
    }

    // prevent overaccumulation of sat reports
    if (!str_starts_with(session->nmea.field[0] + 2, "GSV")) {
        session->nmea.last_gsv_talker = '\0';
    }
    if (!str_starts_with(session->nmea.field[0] + 2, "GSA")) {
        session->nmea.last_gsa_talker = '\0';
    }

    // timestamp recording for fixes happens here
    if (0 != (mask & TIME_SET)) {
        if (0 == session->nmea.date.tm_year &&
            0 == session->nmea.date.tm_mday) {
            // special case to time zero
            session->newdata.time = (timespec_t){0, 0};
        } else {
            session->newdata.time = gpsd_utc_resolve(session);
        }

        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "NMEA0183: %s newtime is %s = "
                 "%d-%02d-%02dT%02d:%02d:%02d.%03ldZ\n",
                 session->nmea.field[0],
                 timespec_str(&session->newdata.time, ts_buf1, sizeof(ts_buf1)),
                 1900 + session->nmea.date.tm_year,
                 session->nmea.date.tm_mon + 1,
                 session->nmea.date.tm_mday,
                 session->nmea.date.tm_hour,
                 session->nmea.date.tm_min,
                 session->nmea.date.tm_sec,
                 session->nmea.subseconds.tv_nsec / 1000000L);
        /*
         * If we have time and PPS is available, assume we have good time.
         * Because this is a generic driver we don't really have enough
         * information for a sharper test, so we'll leave it up to the
         * PPS code to do its own sanity filtering.
         */
        mask |= NTPTIME_IS;
    }

    /*
     * The end-of-cycle detector.  This code depends on just one
     * assumption: if a sentence with a timestamp occurs just before
     * start of cycle, then it is always good to trigger a report on
     * that sentence in the future.  For devices with a fixed cycle
     * this should work perfectly, locking in detection after one
     * cycle.  Most split-cycle devices (Garmin 48, for example) will
     * work fine.  Problems will only arise if a a sentence that
     * occurs just before timestamp increments also occurs in
     * mid-cycle, as in the Garmin eXplorist 210; those might jitter.
     */
    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA0183: %s time %s last %s latch %d cont %d\n",
             session->nmea.field[0],
             timespec_str(&session->nmea.this_frac_time, ts_buf1,
                          sizeof(ts_buf1)),
             timespec_str(&session->nmea.last_frac_time, ts_buf2,
                          sizeof(ts_buf2)),
             session->nmea.latch_frac_time,
             session->nmea.cycle_continue);
    lasttag = session->nmea.lasttag;
    if (session->nmea.gsx_more) {
        // more to come, so ignore for cycle ender
        // appears that GSA and GSV never start a cycle.
    } else if (session->nmea.latch_frac_time) {
        timespec_t ts_delta;
        TS_SUB(&ts_delta, &session->nmea.this_frac_time,
                          &session->nmea.last_frac_time);
        if (0.01 < fabs(TSTONS(&ts_delta))) {
            // time changed
            mask |= CLEAR_IS;
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "NMEA0183: %s starts a reporting cycle. lasttag %d\n",
                     session->nmea.field[0], lasttag);
            /*
             * Have we seen a previously timestamped NMEA tag?
             * If so, designate as end-of-cycle marker.
             * But not if there are continuation sentences;
             * those get sorted after the last timestamped sentence
             *
             */
            if (0 < lasttag &&
                false == (session->nmea.cycle_enders[lasttag]) &&
                !session->nmea.cycle_continue) {
                session->nmea.cycle_enders[lasttag] = true;
                // we might have a (somewhat) reliable end-of-cycle
                session->cycle_end_reliable = true;
                GPSD_LOG(LOG_PROG, &session->context->errout,
                         "NMEA0183: tagged %s as a cycle ender. %u\n",
                         nmea_phrase[lasttag - 1].name,
                         lasttag);
            }
        }
    } else {
        // ignore multiple sequential, like GSV, GSA
        // extend the cycle to an un-timestamped sentence?
        if (true == session->nmea.cycle_enders[lasttag]) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "NMEA0183: %s is just after a cycle ender. (%s)\n",
                     session->nmea.field[0],
                     gps_maskdump(mask));
            if (0 != (mask & ~ONLINE_SET)) {
                // new data... after cycle ender
                mask |= REPORT_IS;
            }
        }
        if (session->nmea.cycle_continue) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "NMEA0183: %s extends the reporting cycle.\n",
                     session->nmea.field[0]);
            // change ender
            session->nmea.cycle_enders[lasttag] = false;
            session->nmea.cycle_enders[thistag] = true;
            // have a cycle ender
            session->cycle_end_reliable = true;
        }
    }

    // here's where we check for end-of-cycle
    if ((session->nmea.latch_frac_time ||
         session->nmea.cycle_continue) &&
        (true == session->nmea.cycle_enders[thistag]) &&
        !session->nmea.gsx_more) {
        if (NULL == nmea_phrase[i].name1) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "NMEA0183: %s ends a reporting cycle.\n",
                     session->nmea.field[0]);
        } else {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "NMEA0183: %s,%s ends a reporting cycle.\n",
                     session->nmea.field[0],
                     session->nmea.field[1]);
        }
        mask |= REPORT_IS;
    }
    if (session->nmea.latch_frac_time) {
        session->nmea.lasttag = thistag;
    }

    /* don't downgrade mode if holding previous fix
     * usually because of xxRMC which does not report 2D/3D */
    if (MODE_SET == (mask & MODE_SET) &&
        MODE_3D == session->gpsdata.fix.mode &&
        MODE_NO_FIX != session->newdata.mode &&
        (0 != isfinite(session->lastfix.altHAE) ||
         0 != isfinite(session->oldfix.altHAE) ||
         0 != isfinite(session->lastfix.altMSL) ||
         0 != isfinite(session->oldfix.altMSL))) {
        session->newdata.mode = session->gpsdata.fix.mode;
    }
    return mask;
}


// add NMEA checksum to a possibly  terminated sentence
void nmea_add_checksum(char *sentence)
{
    unsigned char sum = '\0';
    char c, *p = sentence;

    if ('$' == *p ||
        '!' == *p) {
        p++;
    }
    while (('*' != (c = *p)) &&
           ('\0' != c)) {
        sum ^= c;
        p++;
    }
    *p++ = '*';
    (void)snprintf(p, 5, "%02X\r\n", (unsigned)sum);
}

// ship a command to the GPS, adding * and correct checksum
ssize_t nmea_write(struct gps_device_t *session, char *buf, size_t len UNUSED)
{
    (void)strlcpy(session->msgbuf, buf, sizeof(session->msgbuf));
    if ('$' == session->msgbuf[0]) {
        (void)strlcat(session->msgbuf, "*", sizeof(session->msgbuf));
        nmea_add_checksum(session->msgbuf);
    } else {
        (void)strlcat(session->msgbuf, "\r\n", sizeof(session->msgbuf));
    }
    // codacy hates strlen()
    session->msgbuflen = strnlen(session->msgbuf, sizeof(session->msgbuf));
    return gpsd_write(session, session->msgbuf, session->msgbuflen);
}

ssize_t nmea_send(struct gps_device_t * session, const char *fmt, ...)
{
    char buf[BUFSIZ];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf) - 5, fmt, ap);
    va_end(ap);
    // codacy hates strlen()
    return nmea_write(session, buf, strnlen(buf, sizeof(buf)));
}

// vim: set expandtab shiftwidth=4
