/* gpsutils.c -- code shared between low-level and high-level interfaces
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

/* The strptime prototype is not provided unless explicitly requested.
 * We also need to set the value high enough to signal inclusion of
 * newer features (like clock_gettime).  See the POSIX spec for more info:
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/V2_chap02.html#tag_15_02_01_02 */

#include "../include/gpsd_config.h"    // must be before all includes

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>  // for to have a pselect(2) prototype a la POSIX
#include <sys/time.h>    // for to have a pselect(2) prototype a la SuS
#include <time.h>

#include "../include/gps.h"
#include "../include/libgps.h"
#include "../include/os_compat.h"
#include "../include/timespec.h"

#ifdef USE_QT
#include <QDateTime>
#include <QStringList>
#endif

/*
 * Berkeley implementation of strtod(), inlined to avoid locale problems
 * with the decimal point and stripped down to an atof()-equivalent.
 */

/* Takes a decimal ASCII floating-point number, optionally
 * preceded by white space.  Must have form "SI.FE-X",
 * S may be ither of the signs may be "+", "-", or omitted.
 * I is the integer part of the mantissa,
 * F is the fractional part of the mantissa,
 * X is the exponent.
 * Either I or F may be omitted, or both.
 * The decimal point isn't necessary unless F is
 * present.  The "E" may actually be an "e".  E and X
 * may both be omitted (but not just one).
 *
 * returns NaN if:
 *    *string is zero length,
 *    the first non-white space is not negative sign ('-'), positive sign ('_')
 *    or a digit
 */
double safe_atof(const char *string)
{
    static int maxExponent = 511;   /* Largest possible base 10 exponent.  Any
                                     * exponent larger than this will already
                                     * produce underflow or overflow, so there's
                                     * no need to worry about additional digits.
                                     */
    /* Table giving binary powers of 10.  Entry is 10^2^i.
     * Used to convert decimal exponents into floating-point numbers. */
    static double powersOf10[] = {
        10.,
        100.,
        1.0e4,
        1.0e8,
        1.0e16,
        1.0e32,
        1.0e64,
        1.0e128,
        1.0e256
    };

    bool sign = false, expSign = false;
    double fraction, dblExp, *d;
    const char *p;
    int c;
    int exp = 0;                // Exponent read from "EX" field.
    int fracExp = 0;            /* Exponent that derives from the fractional
                                 * part.  Under normal circumstatnces, it is
                                 * the negative of the number of digits in F.
                                 * However, if I is very long, the last digits
                                 * of I get dropped (otherwise a long I with a
                                 * large negative exponent could cause an
                                 * unnecessary overflow on I alone).  In this
                                 * case, fracExp is incremented one for each
                                 * dropped digit. */
    int mantSize;               // Number of digits in mantissa.
    int decPt;                  /* Number of mantissa digits BEFORE decimal
                                 * point. */
    const char *pExp;           /* Temporarily holds location of exponent
                                 * in string. */

    /*
     * Strip off leading blanks and check for a sign.
     */

    p = string;
    while (isspace((int)*p)) {
        p += 1;
    }
    if (isdigit((int)*p)) {
        // ignore
    } else if ('-' == *p) {
        sign = true;
        p += 1;
    } else if ('+' == *p) {
        p += 1;
    } else if ('.' == *p) {
        // ignore
    } else {
        return NAN;
    }

    /*
     * Count the number of digits in the mantissa (including the decimal
     * point), and also locate the decimal point.
     */

    decPt = -1;
    for (mantSize = 0; ; mantSize += 1) {
        c = *p;
        if (!isdigit((int)c)) {
            if ((c != '.') || (decPt >= 0)) {
                break;
            }
            decPt = mantSize;
        }
        p += 1;
    }

    /*
     * Now suck up the digits in the mantissa.  Use two integers to
     * collect 9 digits each (this is faster than using floating-point).
     * If the mantissa has more than 18 digits, ignore the extras, since
     * they can't affect the value anyway.
     */

    pExp  = p;
    p -= mantSize;
    if (decPt < 0) {
        decPt = mantSize;
    } else {
        mantSize -= 1;                  // One of the digits was the point.
    }
    if (mantSize > 18) {
        fracExp = decPt - 18;
        mantSize = 18;
    } else {
        fracExp = decPt - mantSize;
    }
    if (mantSize == 0) {
        fraction = 0.0;
        // p = string;
        goto done;
    } else {
        int frac1, frac2;

        frac1 = 0;
        for ( ; mantSize > 9; mantSize -= 1) {
            c = *p;
            p += 1;
            if ('.' == c) {
                c = *p;
                p += 1;
            }
            frac1 = 10*frac1 + (c - '0');
        }
        frac2 = 0;
        for (; mantSize > 0; mantSize -= 1) {
            c = *p;
            p += 1;
            if ('.' == c) {
                c = *p;
                p += 1;
            }
            frac2 = 10*frac2 + (c - '0');
        }
        fraction = (1.0e9 * frac1) + frac2;
    }

    /*
     * Skim off the exponent.
     */

    p = pExp;
    if (('E' == *p) ||
        ('e' == *p)) {
        p += 1;
        if ('-' == *p) {
            expSign = true;
            p += 1;
        } else {
            if ('+' == *p) {
                p += 1;
            }
            expSign = false;
        }
        while (isdigit((int) *p)) {
            exp = exp * 10 + (*p - '0');
            p += 1;
        }
    }
    if (expSign) {
        exp = fracExp - exp;
    } else {
        exp = fracExp + exp;
    }

    /*
     * Generate a floating-point number that represents the exponent.
     * Do this by processing the exponent one bit at a time to combine
     * many powers of 2 of 10. Then combine the exponent with the
     * fraction.
     */

    if (0 > exp) {
        expSign = true;
        exp = -exp;
    } else {
        expSign = false;
    }
    if (exp > maxExponent) {
        exp = maxExponent;
        errno = ERANGE;
    }
    dblExp = 1.0;
    for (d = powersOf10; exp != 0; exp >>= 1, d += 1) {
        if (exp & 01) {
            dblExp *= *d;
        }
    }
    if (expSign) {
        fraction /= dblExp;
    } else {
        fraction *= dblExp;
    }

done:
    if (sign) {
        return -fraction;
    }
    return fraction;
}

#define MONTHSPERYEAR   12      // months per calendar year

// clear a baseline_t
static void gps_clear_base(struct baseline_t *base)
{
    base->status = STATUS_UNK;
    base->east = NAN;
    base->north = NAN;
    base->up = NAN;
    base->length = NAN;
    base->course = NAN;
    base->ratio = NAN;
}

// stuff a fix structure with recognizable out-of-band values
void gps_clear_fix(struct gps_fix_t *fixp)
{
    memset(fixp, 0, sizeof(struct gps_fix_t));
    fixp->altitude = NAN;        // DEPRECATED, undefined
    fixp->altHAE = NAN;
    fixp->altMSL = NAN;
    fixp->climb = NAN;
    fixp->depth = NAN;
    fixp->epc = NAN;
    fixp->epd = NAN;
    fixp->eph = NAN;
    fixp->eps = NAN;
    fixp->ept = NAN;
    fixp->epv = NAN;
    fixp->epx = NAN;
    fixp->epy = NAN;
    fixp->latitude = NAN;
    fixp->longitude = NAN;
    fixp->magnetic_track = NAN;
    fixp->magnetic_var = NAN;
    fixp->mode = MODE_NOT_SEEN;
    fixp->sep = NAN;
    fixp->speed = NAN;
    fixp->track = NAN;
    // clear ECEF too
    fixp->ecef.x = NAN;
    fixp->ecef.y = NAN;
    fixp->ecef.z = NAN;
    fixp->ecef.vx = NAN;
    fixp->ecef.vy = NAN;
    fixp->ecef.vz = NAN;
    fixp->ecef.pAcc = NAN;
    fixp->ecef.vAcc = NAN;
    fixp->NED.relPosN = NAN;
    fixp->NED.relPosE = NAN;
    fixp->NED.relPosD = NAN;
    fixp->NED.velN = NAN;
    fixp->NED.velE = NAN;
    fixp->NED.velD = NAN;
    fixp->geoid_sep = NAN;
    fixp->dgps_age = NAN;
    fixp->dgps_station = -1;
    fixp->wanglem = NAN;
    fixp->wangler = NAN;
    fixp->wanglet = NAN;
    fixp->wspeedr = NAN;
    fixp->wspeedt = NAN;
    gps_clear_base(&fixp->base);
}

// stuff an attitude structure with recognizable out-of-band values
void gps_clear_att(struct attitude_t *attp)
{
    memset(attp, 0, sizeof(struct attitude_t));
    attp->acc_len = NAN;
    attp->acc_x = NAN;
    attp->acc_y = NAN;
    attp->acc_z = NAN;
    attp->depth = NAN;
    attp->dip = NAN;
    attp->gyro_temp = NAN;
    attp->gyro_x = NAN;
    attp->gyro_y = NAN;
    attp->gyro_z = NAN;
    attp->heading = NAN;
    attp->mag_len = NAN;
    attp->mag_x = NAN;
    attp->mag_y = NAN;
    attp->mag_z = NAN;
    attp->pitch = NAN;
    attp->roll = NAN;
    attp->temp = NAN;
    attp->yaw = NAN;
    gps_clear_base(&attp->base);
}

void gps_clear_dop( struct dop_t *dop)
{
    dop->xdop = dop->ydop = dop->vdop = dop->tdop = dop->hdop = dop->pdop =
        dop->gdop = NAN;
}

// stuff a log structure with recognizable out-of-band values
void gps_clear_log(struct gps_log_t *logp)
{
    memset(logp, 0, sizeof(struct gps_log_t));
    logp->lon = NAN;
    logp->lat = NAN;
    logp->altHAE = NAN;
    logp->altMSL = NAN;
    logp->gSpeed = NAN;
    logp->heading = NAN;
    logp->tAcc = NAN;
    logp->hAcc = NAN;
    logp->vAcc = NAN;
    logp->sAcc = NAN;
    logp->headAcc = NAN;
    logp->velN = NAN;
    logp->velE = NAN;
    logp->velD = NAN;
    logp->pDOP = NAN;
    logp->distance = NAN;
    logp->totalDistance = NAN;
    logp->distanceStd = NAN;
    logp->fixType = -1;
}

/* merge new data (from) into current fix (to)
 * Being careful not to lose information */
void gps_merge_fix(struct gps_fix_t *to,
                   gps_mask_t transfer,
                   struct gps_fix_t *from)
{
    if ((NULL == to) ||
        (NULL == from)) {
        return;
    }
    if (0 != (transfer & TIME_SET)) {
        to->time = from->time;
    }
    if (0 != (transfer & LATLON_SET)) {
        to->latitude = from->latitude;
        to->longitude = from->longitude;
    }
    if (0 != (transfer & MODE_SET)) {
        // FIXME?  Maybe only upgrade mode, not downgrade it
        to->mode = from->mode;
    }
    /* Some messages only report mode, some mode and status, some only status.
     * Only upgrade status, not downgrade it */
    if (0 != (transfer & STATUS_SET)) {
        if (to->status < from->status) {
            to->status = from->status;
        }
    }
    if ((transfer & ALTITUDE_SET) != 0) {
        if (0 != isfinite(from->altHAE)) {
            to->altHAE = from->altHAE;
        }
        if (0 != isfinite(from->altMSL)) {
            to->altMSL = from->altMSL;
        }
        if (0 != isfinite(from->depth)) {
            to->depth = from->depth;
        }
    }
    if (0 != (transfer & TRACK_SET)) {
        to->track = from->track;
    }
    if (0 != (transfer & MAGNETIC_TRACK_SET)) {
        if (0 != isfinite(from->magnetic_track)) {
            to->magnetic_track = from->magnetic_track;
        }
        if (0 != isfinite(from->magnetic_var)) {
            to->magnetic_var = from->magnetic_var;
        }
    }
    if (0 != (transfer & SPEED_SET)) {
        to->speed = from->speed;
    }
    if (0 != (transfer & CLIMB_SET)) {
        to->climb = from->climb;
    }
    if (0 != (transfer & TIMERR_SET)) {
        to->ept = from->ept;
    }
    if (0 != isfinite(from->epx) &&
        0 != isfinite(from->epy)) {
        to->epx = from->epx;
        to->epy = from->epy;
    }
    if (0 != isfinite(from->epd)) {
        to->epd = from->epd;
    }
    if (0 != isfinite(from->eph)) {
        to->eph = from->eph;
    }
    if (0 != isfinite(from->eps)) {
        to->eps = from->eps;
    }
    // spherical error probability, not geoid separation
    if (0 != isfinite(from->sep)) {
        to->sep = from->sep;
    }
    // geoid separation, not spherical error probability
    if (0 != isfinite(from->geoid_sep)) {
        to->geoid_sep = from->geoid_sep;
    }
    if (0 != isfinite(from->epv)) {
        to->epv = from->epv;
    }
    if (0 != (transfer & SPEEDERR_SET)) {
        to->eps = from->eps;
    }
    if (0 != (transfer & ECEF_SET)) {
        to->ecef.x = from->ecef.x;
        to->ecef.y = from->ecef.y;
        to->ecef.z = from->ecef.z;
        to->ecef.pAcc = from->ecef.pAcc;
    }
    if (0 != (transfer & VECEF_SET)) {
        to->ecef.vx = from->ecef.vx;
        to->ecef.vy = from->ecef.vy;
        to->ecef.vz = from->ecef.vz;
        to->ecef.vAcc = from->ecef.vAcc;
    }
    if (0 != (transfer & NED_SET)) {
        to->NED.relPosN = from->NED.relPosN;
        to->NED.relPosE = from->NED.relPosE;
        to->NED.relPosD = from->NED.relPosD;
        if ((0 != isfinite(from->NED.relPosH)) &&
            (0 != isfinite(from->NED.relPosL))) {
            to->NED.relPosH = from->NED.relPosH;
            to->NED.relPosL = from->NED.relPosL;
        }
    }
    if (0 != (transfer & VNED_SET)) {
        to->NED.velN = from->NED.velN;
        to->NED.velE = from->NED.velE;
        to->NED.velD = from->NED.velD;
    }
    if ('\0' != from->datum[0]) {
        strlcpy(to->datum, from->datum, sizeof(to->datum));
    }
    if (0 != isfinite(from->dgps_age) &&
        0 <= from->dgps_station) {
        // both, or neither
        to->dgps_age = from->dgps_age;
        to->dgps_station = from->dgps_station;
    }

    // navdata stuff.  just wind angle and angle for now
    if (0 != (transfer & NAVDATA_SET)) {
        if (0 != isfinite(from->wanglem)) {
            to->wanglem = from->wanglem;
        }
        if (0 != isfinite(from->wangler)) {
            to->wangler = from->wangler;
        }
        if (0 != isfinite(from->wanglet)) {
            to->wanglet = from->wanglet;
        }
        if (0 != isfinite(from->wspeedr)) {
            to->wspeedr = from->wspeedr;
        }
        if (0 != isfinite(from->wspeedt)) {
            to->wspeedt = from->wspeedt;
        }
    }
}

/* mkgmtime(tm)
 * convert struct tm, as UTC, to seconds since Unix epoch
 * This differs from mktime() from libc.
 * mktime() takes struct tm as localtime.
 *
 * The inverse of gmtime(time_t)
 */
time_t mkgmtime(struct tm * t)
{
    int year;
    time_t result;
    static const int cumdays[MONTHSPERYEAR] =
        { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

    year = 1900 + t->tm_year + t->tm_mon / MONTHSPERYEAR;
    result = (year - 1970) * 365 + cumdays[t->tm_mon % MONTHSPERYEAR];
    result += (year - 1968) / 4;
    result -= (year - 1900) / 100;
    result += (year - 1600) / 400;
    if (0 == (year % 4) &&
        (0 != (year % 100) ||
         0 == (year % 400)) &&
        (2 > (t->tm_mon % MONTHSPERYEAR))) {
        result--;
    }
    result += t->tm_mday - 1;
    result *= 24;
    result += t->tm_hour;
    result *= 60;
    result += t->tm_min;
    result *= 60;
    result += t->tm_sec;
    /* this is UTC, no DST
     * if (t->tm_isdst == 1)
     * result -= 3600;
     */
    return (result);
}

// ISO8601 UTC to Unix timespec, no leapsecond correction.
timespec_t iso8601_to_timespec(const char *isotime)
{
    timespec_t ret;

#ifndef __clang_analyzer__
#ifndef USE_QT
    double usec = 0;
    struct tm tm;
    memset(&tm,0,sizeof(tm));

#ifdef HAVE_STRPTIME
    {
        char *dp = NULL;
        dp = strptime(isotime, "%Y-%m-%dT%H:%M:%S", &tm);
        if (NULL != dp &&
            '.' == *dp) {
            usec = strtod(dp, NULL);
        }
    }
#else
    /* Fallback for systems without strptime (i.e. Windows)
     * This is a simplistic conversion for iso8601 strings only,
     * rather than embedding a full copy of strptime() that handles
     * all formats */
    /* Thus avoiding needing to test for (broken) negative date/time
     * numbers in token reading - only need to check the upper range */
    bool failed = false;
    char *isotime_tokenizer = strdup(isotime);
    if (isotime_tokenizer) {
        char *tmpbuf;
        char *pch = strtok_r(isotime_tokenizer, "-T:", &tmpbuf);
        int token_number = 0;

        while (NULL != pch) {
            double sec;
            unsigned int tmp;

            token_number++;
            // Give up if encountered way too many tokens.
            if (10 < token_number) {
                failed = true;
                break;
            }
            switch (token_number) {
            case 1:  // Year token
                tmp = atoi(pch);
                if (9999 > tmp) {
                    tm.tm_year = tmp - 1900; // Adjust to tm year
                } else {
                    failed = true;
                }
                break;
            case 2: // Month token
                tmp = atoi(pch);
                if (13 > tmp) {
                    tm.tm_mon = tmp - 1;  // Month indexing starts from zero
                } else {
                    failed = true;
                }
                break;
            case 3: // Day token
                tmp = atoi(pch);
                if (32 > tmp) {
                    tm.tm_mday = tmp;
                } else {
                    failed = true;
                }
                break;
            case 4:  // Hour token
                tmp = atoi(pch);
                if (24 > tmp) {
                    tm.tm_hour = tmp;
                } else {
                    failed = true;
                }
                break;
            case 5:  // Minute token
                tmp = atoi(pch);
                if (60 > tmp) {
                    tm.tm_min = tmp;
                } else {
                    failed = true;
                }
                break;
            case 6:  // Seconds token
                sec = safe_atof(pch);
                // NB To handle timestamps with leap seconds
                if (0 == isfinite(sec) &&
                    0.0 <= sec &&
                    61.5 > sec) {
                    // Truncate to get integer value
                    tm.tm_sec = (unsigned int)sec;
                    // Get the fractional part (if any)
                    usec = sec - (unsigned int)sec;
                } else {
                    failed = true;
                }
                break;
            default:
                break;
            }
            pch = strtok_r(NULL, "-T:", &tmpbuf);
        }
        free(isotime_tokenizer);
        /* Split may result in more than 6 tokens if the TZ has any t's
         * in it.  So check that we've seen enough tokens rather than
         * an exact number */
        if (6 > token_number) {
              failed = true;
        }
    }
    if (failed) {
        memset(&tm,0,sizeof(tm));
    } else {
        /* When successful this normalizes tm so that tm_yday is set
         * and thus tm is valid for use with other functions */
        if ((time_t)-1 == mktime(&tm)) {
            // Failed mktime - so reset the timestamp
            memset(&tm,0,sizeof(tm));
        }
    }
#endif
    /*
     * It would be nice if we could say mktime(&tm) - timezone + usec instead,
     * but timezone is not available at all on some BSDs. Besides, when working
     * with historical dates the value of timezone after an ordinary tzset(3)
     * can be wrong; you have to do a redirect through the IANA historical
     * timezone database to get it right.
     */
    ret.tv_sec = mkgmtime(&tm);
    ret.tv_nsec = usec * 1e9;;
#else
    double usec = 0;

    QString t(isotime);
    QDateTime d = QDateTime::fromString(isotime, Qt::ISODate);
    QStringList sl = t.split(".");
    if (1 < sl.size()) {
        usec = sl[1].toInt() / pow(10., (double)sl[1].size());
    }
    ret.tv_sec = d.toTime_t();
    ret.tv_nsec = usec * 1e9;;
#endif
#endif  // __clang_analyzer__

#if 4 < SIZEOF_TIME_T
    if (253402300799LL < ret.tv_sec) {
        // enforce max "9999-12-31T23:59:59.999Z"
        ret.tv_sec = 253402300799LL;
    }
#endif
    return ret;
}

/* Convert POSIX timespec to ISO8601 UTC, put result in isotime.
 * no timezone adjustment
 * Return: pointer to isotime.
 * example: 2007-12-11T23:38:51.033Z */
char *timespec_to_iso8601(timespec_t fixtime, char isotime[], size_t len)
{
    struct tm when;
    char timestr[30];
    long fracsec;

    if (0 > fixtime.tv_sec) {
        // Allow 0 for testing of 1970-01-01T00:00:00.000Z
        strlcpy(isotime, "NaN", len);
        return isotime;
    }
    if (999499999 < fixtime.tv_nsec) {
        // round up
        fixtime.tv_sec++;
        fixtime.tv_nsec = 0;
    }

#if 4 < SIZEOF_TIME_T
    if (253402300799LL < fixtime.tv_sec) {
        // enforce max "9999-12-31T23:59:59.999Z"
        fixtime.tv_sec = 253402300799LL;
    }
#endif

#ifdef HAVE_GMTIME_R
    (void)gmtime_r(&fixtime.tv_sec, &when);
#else
    // Fallback to try with gmtime_s - primarily for Windows
    (void)gmtime_s(&when, &fixtime.tv_sec);
#endif

    /*
     * Do not mess casually with the number of decimal digits in the
     * format!  Most GPSes report over serial links at 0.01s or 0.001s
     * precision.  Round to 0.001s
     */
    fracsec = (fixtime.tv_nsec + 500000) / 1000000;

    (void)strftime(timestr, sizeof(timestr), "%Y-%m-%dT%H:%M:%S", &when);
    (void)snprintf(isotime, len, "%s.%03ldZ",timestr, fracsec);

    return isotime;
}

/* return time now as ISO8601, no timezone adjustment
 * example: 2007-12-11T23:38:51.033Z */
char *now_to_iso8601(char *tbuf, size_t tbuf_sz)
{
    timespec_t ts_now;

    (void)clock_gettime(CLOCK_REALTIME, &ts_now);
    return timespec_to_iso8601(ts_now, tbuf, tbuf_sz);
}

#define Deg2Rad(n)      ((n) * DEG_2_RAD)

/* Distance in meters between two points specified in degrees, optionally
 * with initial and final bearings. */
double earth_distance_and_bearings(double lat1, double lon1,
                                   double lat2, double lon2,
                                   double *ib, double *fb)
{
    /*
     * this is a translation of the javascript implementation of the
     * Vincenty distance formula by Chris Veness. See
     * http://www.movable-type.co.uk/scripts/latlong-vincenty.html
     */
    double a, b, f;             // WGS-84 ellipsoid params
    double L, L_P, U1, U2, s_U1, c_U1, s_U2, c_U2;
    double uSq, A, B, d_S, lambda;
    // cppcheck-suppress variableScope
    double s_L, c_L, s_A, C;
    double c_S, S, s_S, c_SqA, c_2SM;
    int i = 100;

    a = WGS84A;
    b = WGS84B;
    f = 1 / WGS84F;
    L = Deg2Rad(lon2 - lon1);
    U1 = atan((1 - f) * tan(Deg2Rad(lat1)));
    U2 = atan((1 - f) * tan(Deg2Rad(lat2)));
    s_U1 = sin(U1);
    c_U1 = cos(U1);
    s_U2 = sin(U2);
    c_U2 = cos(U2);
    lambda = L;

    do {
        s_L = sin(lambda);
        c_L = cos(lambda);
        s_S = sqrt((c_U2 * s_L) * (c_U2 * s_L) +
                   (c_U1 * s_U2 - s_U1 * c_U2 * c_L) *
                   (c_U1 * s_U2 - s_U1 * c_U2 * c_L));

        if (0 == s_S) {
            return 0;
        }

        c_S = s_U1 * s_U2 + c_U1 * c_U2 * c_L;
        S = atan2(s_S, c_S);
        s_A = c_U1 * c_U2 * s_L / s_S;
        c_SqA = 1 - s_A * s_A;
        c_2SM = c_S - 2 * s_U1 * s_U2 / c_SqA;

        if (0 == isfinite(c_2SM)) {
            c_2SM = 0;
        }

        C = f / 16 * c_SqA * (4 + f * (4 - 3 * c_SqA));
        L_P = lambda;
        lambda = L + (1 - C) * f * s_A *
            (S + C * s_S * (c_2SM + C * c_S * (2 * c_2SM * c_2SM - 1)));
    } while ((fabs(lambda - L_P) > 1.0e-12) &&
             (0 < --i));

    if (0 == i) {
        return NAN;             // formula failed to converge
    }

    uSq = c_SqA * ((a * a) - (b * b)) / (b * b);
    A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    d_S = B * s_S * (c_2SM + B / 4 *
                     (c_S * (-1 + 2 * c_2SM * c_2SM) - B / 6 * c_2SM *
                      (-3 + 4 * s_S * s_S) * (-3 + 4 * c_2SM * c_2SM)));

    if (NULL != ib) {
        *ib = atan2(c_U2 * sin(lambda),
                    c_U1 * s_U2 - s_U1 * c_U2 * cos(lambda));
    }
    if (NULL != fb) {
        *fb = atan2(c_U1 * sin(lambda),
                    c_U1 * s_U2 * cos(lambda) - s_U1 * c_U2);
    }

    return (WGS84B * A * (S - d_S));
}

// Distance in meters between two points specified in degrees.
double earth_distance(double lat1, double lon1, double lat2, double lon2)
{
    return earth_distance_and_bearings(lat1, lon1, lat2, lon2, NULL, NULL);
}

// Wait for data until timeout, ignoring signals.
bool nanowait(int fd, struct timespec *to)
{
    fd_set fdset;

    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);
    TS_NORM(to);         // just in case
    // sigmask is NULL, so equivalent to select()
    return pselect(fd + 1, &fdset, NULL, NULL, to, NULL) == 1;
}

/* Accept a datum code, return matching string
 *
 * There are a ton of these, only a few are here
 *
 */
void datum_code_string(int code, char *buffer, size_t len)
{
    const char *datum_str;

    switch (code) {
    case 0:
        datum_str = "WGS84";
        break;
    case 21:
        datum_str = "WGS84";
        break;
    case 178:
        datum_str = "Tokyo Mean";
        break;
    case 179:
        datum_str = "Tokyo-Japan";
        break;
    case 180:
        datum_str = "Tokyo-Korea";
        break;
    case 181:
        datum_str = "Tokyo-Okinawa";
        break;
    case 182:
        datum_str = "PZ90.11";
        break;
    case 999:
        datum_str = "User Defined";
        break;
    default:
        datum_str = NULL;
        break;
    }

    if (NULL == datum_str) {
        // Fake it
        snprintf(buffer, len, "%d", code);
    } else {
        strlcpy(buffer, datum_str, len);
    }
}

/* make up an NMEA 4.0 (extended) PRN based on gnssId:svId,
 * This does NOT match NMEA 4.10 and 4.11 where all PRN are 1-99,
 * except IMEA
 * Ref Appendix A from u-blox ZED-F9P Interface Description
 * and
 * Section 1.5.3  M10-FW500_InterfaceDescription_UBX-20053845.pdf
 *
 * Return PRN, or zero for error
 */
short ubx2_to_prn(int gnssId, int svId)
{
    short nmea_PRN;

    if (1 > svId) {
        // skip 0 svId
        return 0;
    }

    switch (gnssId) {
    case 0:
        // GPS, 1-32 maps to 1-32
        if (32 < svId) {
            // skip bad svId
            return 0;
        }
        nmea_PRN = svId;
        break;
    case 1:
        // SBAS, 120..151, 152..158 maps to 33..64, 152..158
        if (120 > svId) {
            // Huh?
            return 0;
        } else if (151 >= svId) {
            nmea_PRN = svId - 87;
        } else if (158 >= svId) {
            nmea_PRN = svId;
        } else {
            // Huh?
            return 0;
        }
        break;
    case 2:
        // Galileo, ubx gnssid:svid 1..36 ->  301-336
        // Galileo, ubx PRN         211..246 ->  301-336
        if (36 >= svId) {
            nmea_PRN = svId + 300;
        } else if (211 > svId) {
            // skip bad svId
            return 0;
        } else if (246 >= svId) {
            nmea_PRN = svId + 90;
        } else {
            // skip bad svId
            return 0;
        }
        break;
    case 3:
        // BeiDou, ubx gnssid:svid 1..37 -> to 401-437
        // BeiDou, ubx PRN         159..163,33..64 -> to 401-437 ??
        if (37 >= svId) {
            nmea_PRN = svId + 400;
        } else if (159 > svId) {
            // skip bad svId
            return 0;
        } else if (163 >= svId) {
            nmea_PRN = svId + 242;
        } else {
            // skip bad svId
            return 0;
        }
        break;
    case 4:
        // IMES, ubx gnssid:svid 1-10 -> to 173-182
        // IMES, ubx PRN         173-182 to 173-182
        if (10 >= svId) {
            nmea_PRN = svId + 172;
        } else if (173 > svId) {
            // skip bad svId
            return 0;
        } else if (182 >= svId) {
            nmea_PRN = svId;
        } else {
            // > 182, skip bad svId
            return 0;
        }
        break;
    case 5:
        // QZSS, ubx gnssid:svid 1-10 to 193-202
        // QZSS, ubx PRN         193-202 to 193-202
        if (10 >= svId) {
            nmea_PRN = svId + 192;
        } else if (193 > svId) {
            // skip bad svId
            return 0;
        } else if (202 >= svId) {
            nmea_PRN = svId;
        } else {
            // skip bad svId
            return 0;
        }
        break;
    case 6:
        // GLONASS, 1-32 maps to 65-96
        if (32 >= svId) {
            nmea_PRN = svId + 64;
        } else if (65 > svId) {
            // skip bad svId
            return 0;
        } else if (96 >= svId) {
            nmea_PRN = svId;
        } else {
            // skip bad svId, 255 == tracked, but unidentified, skip
            return 0;
        }
        break;
    case 7:
        // NavIC (IRNSS)
        FALLTHROUGH
    default:
        // Huh?
        return 0;
    }

    return nmea_PRN;
}

// vim: set expandtab shiftwidth=4
