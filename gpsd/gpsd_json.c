/****************************************************************************
NAME
   gpsd_json.c - move data between in-core and JSON structures

DESCRIPTION
   These are functions (used only by the daemon) to dump the contents
   of various core data structures in JSON.

   Most of these should be static.

Written by Eric S. Raymond, 2009
This file is Copyright by the GPSD project
SPDX-License-Identifier: BSD-2-clause

***************************************************************************/

#include "../include/gpsd_config.h"  // must be before all includes

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>       // for qsort()
#include <string.h>       // for strcat(), strlcpy()

#include "../include/gpsd.h"
#include "../include/bits.h"
#include "../include/strfuncs.h"
#include "../include/gps_json.h"
#include "../include/timespec.h"

// *INDENT-OFF*
#define JSON_BOOL(x)    ((x)?"true":"false")

/*
 * Manifest names for the gnss_type enum - must be kept synced with it.
 * Also, masks so we can tell what packet types correspond to each class.
 */
// the map of device class names
struct classmap_t {
    char        *name;
    int         typemask;
    int         packetmask;
};
#define CLASSMAP_NITEMS 5

struct classmap_t classmap[CLASSMAP_NITEMS] = {
    // name     typemask        packetmask
    {"ANY",     0,              0},
    {"GPS",     SEEN_GPS,       GPS_TYPEMASK},
    {"RTCM2",   SEEN_RTCM2,     PACKET_TYPEMASK(RTCM2_PACKET)},
    {"RTCM3",   SEEN_RTCM3,     PACKET_TYPEMASK(RTCM3_PACKET)},
    {"AIS",     SEEN_AIS,       PACKET_TYPEMASK(AIVDM_PACKET)},
};
// *INDENT-ON*

// prevent negative zero confusion.
// Dif arch will return 0.0, or -0.0.
static inline double fix_zero(double d, double p)
{
    // prevent -0.000
    if (fabs(d) < p) {
        return 0.0;
    }
    return d;
}

// escape double quotes and control characters inside a JSON string
char *json_stringify(char *to, size_t len, const char *from)
{
    const char *sp;
    char *tp;

    tp = to;
    /*
     * The limit is len-6 here because we need to be leave room for
     * each character to generate an up to 6-character Java-style
     * escape
     */
    for (sp = from; *sp != '\0' && ((tp - to) < ((int)len - 6)); sp++) {
        if (!isascii((unsigned char) *sp) ||
            iscntrl((unsigned char) *sp)) {
            *tp++ = '\\';
            switch (*sp) {
            case '\b':
                *tp++ = 'b';
                break;
            case '\f':
                *tp++ = 'f';
                break;
            case '\n':
                *tp++ = 'n';
                break;
            case '\r':
                *tp++ = 'r';
                break;
            case '\t':
                *tp++ = 't';
                break;
            default:
                /* ugh, we'd prefer a C-style escape here, but this is JSON
                 * http://www.ietf.org/rfc/rfc4627.txt
                 * section 2.5, escape is \uXXX
                 * don't forget the NUL in the output count! */
                (void)snprintf(tp, 6, "u%04x", 0x00ff & (unsigned int)*sp);
                tp += strlen(tp);
            }
        } else {
            if ('"' == *sp ||
                '\\' == *sp) {
                *tp++ = '\\';
            }
            *tp++ = *sp;
        }
    }
    *tp = '\0';

    return to;
}

void json_version_dump(char *reply, size_t replylen)
{
    (void)snprintf(reply, replylen,
                   "{\"class\":\"VERSION\",\"release\":\"%s\",\"rev\":\"%s\","
                   "\"proto_major\":%d,\"proto_minor\":%d}\r\n",
                   VERSION, REVISION,
                   GPSD_PROTO_VERSION_MAJOR, GPSD_PROTO_VERSION_MINOR);
}

static void json_log_dump(const struct gps_device_t *session,
                          char *reply, size_t replylen)
{
    char tbuf[JSON_DATE_MAX+1];
    const struct gps_log_t *logp = &session->gpsdata.log;

    if (0 >= session->gpsdata.log.then.tv_sec) {
        // no data...
        return;
    }
    (void)snprintf(reply, replylen,
                   "{\"class\":\"LOG\",\"time\":\"%s\",\"idx\":%lu",
                   timespec_to_iso8601(logp->then, tbuf, sizeof(tbuf)),
                   (unsigned long)logp->index_cnt);
    if (0 < logp->string[0]) {
        str_appendf(reply, replylen, ",\"string\":%s", logp->string);
    }

    if (STATUS_DGPS <= logp->status) {
        // to save rebuilding all the regressions, skip UNK and GPS
        str_appendf(reply, replylen, ",\"status\":%d", logp->status);
    }
    // Sometimes char is signed, sometimes unsigned, handle both
    if (10 >= (logp->fixType & 0xFF)) {
        str_appendf(reply, replylen, ",\"mode\":%d", logp->fixType);
    }

    if (0 != isfinite(logp->lat)  &&
        0 != isfinite(logp->lon)) {
        str_appendf(reply, replylen, ",\"lat\":%.9f,\"lon\":%.9f",
                    logp->lat, logp->lon);
    }
    if (0 != isfinite(logp->altHAE)) {
        str_appendf(reply, replylen, ",\"altHAE\":%.4f", logp->altHAE);
    }
    if (0 != isfinite(logp->altMSL)) {
        str_appendf(reply, replylen, ",\"altMSL\":%.4f", logp->altMSL);
    }
    if (0 != isfinite(logp->gSpeed)) {
        str_appendf(reply, replylen, ",\"gSpeed\":%.0f", logp->gSpeed);
    }
    if (0 != isfinite(logp->heading)) {
        str_appendf(reply, replylen, ",\"heading\":%.0f", logp->heading);
    }
    if (0 != isfinite(logp->tAcc)) {
        str_appendf(reply, replylen, ",\"tAcc\":%.0f", logp->tAcc);
    }
    if (0 != isfinite(logp->hAcc)) {
        str_appendf(reply, replylen, ",\"hAcc\":%.0f", logp->hAcc);
    }
    if (0 != isfinite(logp->vAcc)) {
        str_appendf(reply, replylen, ",\"tAcc\":%.0f", logp->vAcc);
    }
    if (0 != isfinite(logp->sAcc)) {
        str_appendf(reply, replylen, ",\"sAcc\":%.0f", logp->sAcc);
    }
    if (0 != isfinite(logp->headAcc)) {
        str_appendf(reply, replylen, ",\"headAcc\":%.0f", logp->headAcc);
    }
    if (0 != isfinite(logp->velN) &&
        0 != isfinite(logp->velE)) {
        // 2D fix needs velN and velE
        str_appendf(reply, replylen,
                    ",\"velN\":%.3f,\"velE\":%.3f",
                    logp->velN,
                    logp->velE);
        if (0 != isfinite(logp->velD)) {
            // 3D fix add velD
            str_appendf(reply, replylen, ",\"velD\":%.3f",
                        logp->velD);
        }
    }
    if (0 != isfinite(logp->pDOP)) {
        str_appendf(reply, replylen, ",\"pDOP\":%.1f", logp->pDOP);
    }
    if (0 != isfinite(logp->distance)) {
        str_appendf(reply, replylen, ",\"distance\":%.0f", logp->distance);
    }
    if (0 != isfinite(logp->totalDistance)) {
        str_appendf(reply, replylen, ",\"tDistance\":%.0f",
                    logp->totalDistance);
    }
    if (0 != isfinite(logp->distanceStd)) {
        str_appendf(reply, replylen, ",\"distStd\":%.1f",
                    logp->distanceStd);
    }

    (void)strlcat(reply, "}\r\n", replylen);
}

/* dump baseline_t data.
 * used by json_tpv_dump() and json_att_dump()
 *
 * Return: void
 */
static void json_base_dump(const struct baseline_t *base,
                           char *reply, size_t replylen)
{
    // FIXME:  split into a function, and also use in FIX.
    if (STATUS_UNK == base->status) {
        return;
    }
    str_appendf(reply, replylen, ",\"baseS\":%d", base->status);
    if (0 != isfinite(base->east)) {
        str_appendf(reply, replylen, ",\"baseE\":%.3f", base->east);
    }
    if (0 != isfinite(base->north)) {
        str_appendf(reply, replylen, ",\"baseN\":%.3f", base->north);
    }
    if (0 != isfinite(base->up)) {
        str_appendf(reply, replylen, ",\"baseU\":%.3f", base->up);
    }
    if (0 != isfinite(base->length)) {
        str_appendf(reply, replylen, ",\"baseL\":%.3f", base->length);
    }
    if (0 != isfinite(base->course)) {
        str_appendf(reply, replylen, ",\"baseC\":%.3f", base->course);
    }
}

void json_tpv_dump(const gps_mask_t changed, struct gps_device_t *session,
                   const struct gps_policy_t *policy,
                   char *reply, size_t replylen)
{
    struct gps_data_t *gpsdata = &session->gpsdata;

    (void)strlcpy(reply, "{\"class\":\"TPV\"", replylen);
    if (gpsdata->dev.path[0] != '\0')
        // Note: Assumes /dev paths are always plain ASCII
        str_appendf(reply, replylen, ",\"device\":\"%s\"", gpsdata->dev.path);
    if (STATUS_DGPS <= gpsdata->fix.status) {
        // to save rebuilding all the regressions, skip UNK and GPS
        str_appendf(reply, replylen, ",\"status\":%d", gpsdata->fix.status);
    }
    str_appendf(reply, replylen, ",\"mode\":%d", gpsdata->fix.mode);
    if (0 < gpsdata->fix.time.tv_sec) {
        char tbuf[JSON_DATE_MAX+1];
        str_appendf(reply, replylen,
                       ",\"time\":\"%s\"",
                       timespec_to_iso8601(gpsdata->fix.time,
                                      tbuf, sizeof(tbuf)));
    }
    if (LEAP_SECOND_VALID == (session->context->valid & LEAP_SECOND_VALID)) {
        str_appendf(reply, replylen, ",\"leapseconds\":%d",
                    session->context->leap_seconds);
    }
    if (0 < gpsdata->fix.time.tv_sec) {
        // do not output ept if no time.
        if (isfinite(gpsdata->fix.ept) != 0)
            str_appendf(reply, replylen, ",\"ept\":%.3f", gpsdata->fix.ept);
    }
    /*
     * Suppressing TPV fields that would be invalid because the fix
     * quality doesn't support them is nice for cutting down on the
     * volume of meaningless output, but the real reason to do it is
     * that we've observed that geodetic fix computation is unstable
     * in a way that tends to change low-order digits in invalid
     * fixes. Dumping these tends to cause cross-architecture failures
     * in the regression tests.  This effect has been seen on SiRF-II
     * chips, which are quite common.
     */
    if (MODE_2D <= gpsdata->fix.mode) {
        double altitude = NAN;

        if (0 != isfinite(gpsdata->fix.latitude)) {
            str_appendf(reply, replylen,
                           ",\"lat\":%.9f", gpsdata->fix.latitude);
        }
        if (0 != isfinite(gpsdata->fix.longitude)) {
            str_appendf(reply, replylen,
                           ",\"lon\":%.9f", gpsdata->fix.longitude);
        }
        if (0 != isfinite(gpsdata->fix.altHAE)) {
            altitude = gpsdata->fix.altHAE;
            str_appendf(reply, replylen,
                           ",\"altHAE\":%.4f", gpsdata->fix.altHAE);
        }
        if (0 != isfinite(gpsdata->fix.altMSL)) {
            altitude = gpsdata->fix.altMSL;
            str_appendf(reply, replylen,
                           ",\"altMSL\":%.4f", gpsdata->fix.altMSL);
        }
        if (0 != isfinite(altitude)) {
            // DEPRECATED, undefined
            str_appendf(reply, replylen,
                           ",\"alt\":%.4f", altitude);
        }

        if (0 != isfinite(gpsdata->fix.epx)) {
            str_appendf(reply, replylen, ",\"epx\":%.3f", gpsdata->fix.epx);
        }
        if (0 != isfinite(gpsdata->fix.epy)) {
            str_appendf(reply, replylen, ",\"epy\":%.3f", gpsdata->fix.epy);
        }
        if (0 != isfinite(gpsdata->fix.epv)) {
            str_appendf(reply, replylen, ",\"epv\":%.3f", gpsdata->fix.epv);
        }
        if (0 != isfinite(gpsdata->fix.track)) {
            str_appendf(reply, replylen, ",\"track\":%.4f", gpsdata->fix.track);
        }
        if (0 != isfinite(gpsdata->fix.magnetic_track)) {
                str_appendf(reply, replylen, ",\"magtrack\":%.4f",
                            gpsdata->fix.magnetic_track);
        }
        if (0 != isfinite(gpsdata->fix.magnetic_var)) {
                str_appendf(reply, replylen, ",\"magvar\":%.1f",
                            gpsdata->fix.magnetic_var);
        }
        if (0 != isfinite(gpsdata->fix.speed)) {
            str_appendf(reply, replylen, ",\"speed\":%.3f", gpsdata->fix.speed);
        }
        if (MODE_3D <= gpsdata->fix.mode &&
            0 != isfinite(gpsdata->fix.climb)) {
            str_appendf(reply, replylen, ",\"climb\":%.3f",
                        fix_zero(gpsdata->fix.climb, 0.0005));
        }
        if (0 != isfinite(gpsdata->fix.epd)) {
            str_appendf(reply, replylen, ",\"epd\":%.4f", gpsdata->fix.epd);
        }
        if (0 != isfinite(gpsdata->fix.eps)) {
            str_appendf(reply, replylen, ",\"eps\":%.2f", gpsdata->fix.eps);
        }
        if (MODE_3D <= gpsdata->fix.mode) {
            if (0 != isfinite(gpsdata->fix.epc)) {
                str_appendf(reply, replylen, ",\"epc\":%.2f", gpsdata->fix.epc);
            }
            // ECEF is in meters, so %.3f is millimeter resolution
            if (0 != isfinite(gpsdata->fix.ecef.x)) {
                str_appendf(reply, replylen, ",\"ecefx\":%.2f",
                            gpsdata->fix.ecef.x);
            }
            if (0 != isfinite(gpsdata->fix.ecef.y)) {
                str_appendf(reply, replylen, ",\"ecefy\":%.2f",
                            gpsdata->fix.ecef.y);
            }
            if (0 != isfinite(gpsdata->fix.ecef.z)) {
                str_appendf(reply, replylen, ",\"ecefz\":%.2f",
                            gpsdata->fix.ecef.z);
            }
            if (0 != isfinite(gpsdata->fix.ecef.vx)) {
                str_appendf(reply, replylen, ",\"ecefvx\":%.2f",
                            fix_zero(gpsdata->fix.ecef.vx, 0.005));
            }
            if (0 != isfinite(gpsdata->fix.ecef.vy)) {
                str_appendf(reply, replylen, ",\"ecefvy\":%.2f",
                            fix_zero(gpsdata->fix.ecef.vy, 0.005));
            }
            if (0 != isfinite(gpsdata->fix.ecef.vz)) {
                str_appendf(reply, replylen, ",\"ecefvz\":%.2f",
                            fix_zero(gpsdata->fix.ecef.vz, 0.005));
            }
            if (0 != isfinite(gpsdata->fix.ecef.pAcc)) {
                str_appendf(reply, replylen, ",\"ecefpAcc\":%.2f",
                            gpsdata->fix.ecef.pAcc);
            }
            if (0 != isfinite(gpsdata->fix.ecef.vAcc)) {
                str_appendf(reply, replylen, ",\"ecefvAcc\":%.2f",
                            gpsdata->fix.ecef.vAcc);
            }
            // NED is in meters, so %.3f is millimeter resolution
            if (0 != isfinite(gpsdata->fix.NED.relPosN) &&
                0 != isfinite(gpsdata->fix.NED.relPosE)) {
                // 2D fix needs relN and relE
                str_appendf(reply, replylen, ",\"relN\":%.3f,\"relE\":%.3f",
                            gpsdata->fix.NED.relPosN,
                            gpsdata->fix.NED.relPosE);
                if (0 != isfinite(gpsdata->fix.NED.relPosD)) {
                    // 3D fix add relD
                    str_appendf(reply, replylen, ",\"relD\":%.3f",
                                gpsdata->fix.NED.relPosD);
                }
                if (0 != isfinite(gpsdata->fix.NED.relPosH) &&
                    0 != isfinite(gpsdata->fix.NED.relPosL)) {
                    // 2D fix needs relN and relE
                    str_appendf(reply, replylen, ",\"relH\":%.3f,\"relL\":%.3f",
                                gpsdata->fix.NED.relPosH,
                                gpsdata->fix.NED.relPosL);
                }
            }
            if (0 != isfinite(gpsdata->fix.NED.velN) &&
                0 != isfinite(gpsdata->fix.NED.velE)) {
                // 2D fix needs velN and velE
                str_appendf(reply, replylen,
                            ",\"velN\":%.3f,\"velE\":%.3f",
                            fix_zero(gpsdata->fix.NED.velN, 0.0005),
                            fix_zero(gpsdata->fix.NED.velE, 0.0005));
                if (0 != isfinite(gpsdata->fix.NED.velD)) {
                    // 3D fix add velD
                    str_appendf(reply, replylen, ",\"velD\":%.3f",
                                fix_zero(gpsdata->fix.NED.velD, 0.0005));
                }
            }
            if (0 != isfinite(gpsdata->fix.geoid_sep))
                str_appendf(reply, replylen, ",\"geoidSep\":%.3f",
                            gpsdata->fix.geoid_sep);
        }
        if (policy->timing) {
            char rtime_str[TIMESPEC_LEN];
            char ts_buf[TIMESPEC_LEN];
            struct timespec rtime_tmp;

            (void)clock_gettime(CLOCK_REALTIME, &rtime_tmp);
            str_appendf(reply, replylen, ",\"rtime\":%s",
                        timespec_str(&rtime_tmp, rtime_str,
                                     sizeof(rtime_str)));
            if (session->pps_thread.ppsout_count) {
                char ts_str[TIMESPEC_LEN];
                struct timedelta_t timedelta;

                // Can't have (const)session and (volatile)pps_thread.
                pps_thread_ppsout(&session->pps_thread, &timedelta);
                str_appendf(reply, replylen, ",\"pps\":%s",
                            timespec_str(&timedelta.clock, ts_str,
                                         sizeof(ts_str)));
                // TODO: add PPS precision to JSON output
            }
            str_appendf(reply, replylen,
                        ",\"sor\":%s,\"chars\":%lu,\"sats\":%2d,"
                        "\"week\":%u,\"tow\":%lld.%03ld,\"rollovers\":%d",
                        timespec_str(&session->sor, ts_buf, sizeof(ts_buf)),
                        session->chars,
                        gpsdata->satellites_used,
                        session->context->gps_week,
                        (long long)session->context->gps_tow.tv_sec,
                        session->context->gps_tow.tv_nsec / 1000000L,
                        session->context->rollovers);
        }
        // at the end because it is new and microjson chokes on new items
        if (0 != isfinite(gpsdata->fix.eph)) {
            str_appendf(reply, replylen, ",\"eph\":%.3f", gpsdata->fix.eph);
        }
        if (0 != isfinite(gpsdata->fix.sep)) {
            str_appendf(reply, replylen, ",\"sep\":%.3f", gpsdata->fix.sep);
        }
        if ('\0' != gpsdata->fix.datum[0]) {
            str_appendf(reply, replylen, ",\"datum\":\"%.40s\"",
                        gpsdata->fix.datum);
        }
        if (0 != isfinite(gpsdata->fix.depth)) {
            str_appendf(reply, replylen,
                           ",\"depth\":%.3f", gpsdata->fix.depth);
        }
        // Skytraq $PSTI, and u-blox, can have Age but no Station
        if (0 != isfinite(gpsdata->fix.dgps_age)) {
            str_appendf(reply, replylen,
                           ",\"dgpsAge\":%.1f", gpsdata->fix.dgps_age);
        }
        if (0 <= gpsdata->fix.dgps_station) {
            str_appendf(reply, replylen,
                           ",\"dgpsSta\":%d", gpsdata->fix.dgps_station);
        }
        if (0 != isfinite(gpsdata->fix.base.ratio)) {
            // Skytraq reports ratiiio to .3f
            str_appendf(reply, replylen,
                           ",\"dgpsRatio\":%.3f", gpsdata->fix.base.ratio);
        }
    }
    if (ANT_OK < gpsdata->fix.ant_stat){
        str_appendf(reply, replylen, ",\"ant\":%d", gpsdata->fix.ant_stat);
    }
    if (0 < gpsdata->fix.jam){
        str_appendf(reply, replylen, ",\"jam\":%u", gpsdata->fix.jam);
    }
    if (0 != gpsdata->fix.clockbias) {
        str_appendf(reply, replylen, ",\"clockbias\":%lld",
                    (long long)gpsdata->fix.clockbias);
    }
    if (0 != gpsdata->fix.clockdrift) {
        str_appendf(reply, replylen, ",\"clockdrift\":%lld",
                    (long long)gpsdata->fix.clockdrift);
    }
    if (0 != (changed & NAVDATA_SET)) {
        if (0 != isfinite(gpsdata->fix.wanglem)){
            str_appendf(reply, replylen,
                        ",\"wanglem\":%.1f", gpsdata->fix.wanglem);
        }
        if (0 != isfinite(gpsdata->fix.wangler)){
            str_appendf(reply, replylen,
                        ",\"wangler\":%.1f", gpsdata->fix.wangler);
        }
        if (0 != isfinite(gpsdata->fix.wanglet)){
            str_appendf(reply, replylen,
                        ",\"wanglet\":%.1f", gpsdata->fix.wanglet);
        }
        if (0 != isfinite(gpsdata->fix.wspeedr)){
            str_appendf(reply, replylen,
                        ",\"wspeedr\":%.1f", gpsdata->fix.wspeedr);
        }
        if (0 != isfinite(gpsdata->fix.wspeedt)){
            str_appendf(reply, replylen,
                        ",\"wspeedt\":%.1f", gpsdata->fix.wspeedt);
        }
    }
    if (0 != isfinite(gpsdata->fix.temp)) {
        // Receiver Temp, in degrees C
        str_appendf(reply, replylen,
                       ",\"temp\":%.3f", gpsdata->fix.temp);
    }
    if (0 != isfinite(gpsdata->fix.wtemp)) {
        // Water Temp, in degrees C
        str_appendf(reply, replylen,
                       ",\"wtemp\":%.3f", gpsdata->fix.wtemp);
    }
    if (STATUS_UNK != gpsdata->fix.base.status) {
        json_base_dump(&gpsdata->fix.base, reply, replylen);
    }
    (void)strlcat(reply, "}\r\n", replylen);
}

void json_noise_dump(const struct gps_data_t *gpsdata,
                     char *reply, size_t replylen)
{
    size_t header_len;       // a guard to prevent sending empty messages

    (void)strlcpy(reply, "{\"class\":\"GST\"", replylen);
    if ('\0' != gpsdata->dev.path[0]) {
        str_appendf(reply, replylen, ",\"device\":\"%s\"", gpsdata->dev.path);
    }
    if (0 < gpsdata->gst.utctime.tv_sec) {
        char tbuf[JSON_DATE_MAX+1];
        str_appendf(reply, replylen,
                   ",\"time\":\"%s\"",
                   timespec_to_iso8601(gpsdata->gst.utctime,
                                       tbuf, sizeof(tbuf)));
    }
    header_len = strnlen(reply, replylen);

#define ADD_GST_FIELD(tag, field) do {                     \
    if (0 != isfinite(gpsdata->gst.field))              \
        str_appendf(reply, replylen, ",\"" tag "\":%.3f", gpsdata->gst.field); \
    } while(0)

    ADD_GST_FIELD("rms",    rms_deviation);
    ADD_GST_FIELD("major",  smajor_deviation);
    ADD_GST_FIELD("minor",  sminor_deviation);
    ADD_GST_FIELD("orient", smajor_orientation);
    ADD_GST_FIELD("lat",    lat_err_deviation);
    ADD_GST_FIELD("lon",    lon_err_deviation);
    ADD_GST_FIELD("alt",    alt_err_deviation);
    ADD_GST_FIELD("ve",     ve_err_deviation);
    ADD_GST_FIELD("vn",     vn_err_deviation);
    ADD_GST_FIELD("vu",     vu_err_deviation);

#undef ADD_GST_FIELD

    if (header_len == strnlen(reply, replylen)) {
        // empty message, slip it
        *reply = '\0';
    } else {
        (void)strlcat(reply, "}\r\n", replylen);
    }
}

void json_sky_dump(const struct gps_device_t *session,
                   char *reply, size_t replylen)
{
    int i, reported = 0, used = 0;
    const struct gps_data_t *datap = &session->gpsdata;
    size_t header_len;       // a guard to prevent sending empty messages

    (void)strlcpy(reply, "{\"class\":\"SKY\"", replylen);
    if ('\0' != datap->dev.path[0]) {
        str_appendf(reply, replylen, ",\"device\":\"%s\"", datap->dev.path);
    }
    if (0 < datap->skyview_time.tv_sec) {
        char tbuf[JSON_DATE_MAX+1];

        str_appendf(reply, replylen,
                       ",\"time\":\"%s\"",
                       timespec_to_iso8601(datap->skyview_time,
                                      tbuf, sizeof(tbuf)));
    }
    header_len = strnlen(reply, replylen);

    if (0 != isfinite(datap->dop.gdop)) {
        str_appendf(reply, replylen, ",\"gdop\":%.2f", datap->dop.gdop);
    }
    if (0 != isfinite(datap->dop.hdop)) {
        str_appendf(reply, replylen, ",\"hdop\":%.2f", datap->dop.hdop);
    }
    if (0 != isfinite(datap->dop.pdop)) {
        str_appendf(reply, replylen, ",\"pdop\":%.2f", datap->dop.pdop);
    }
    if (0 != isfinite(datap->dop.tdop)) {
        str_appendf(reply, replylen, ",\"tdop\":%.2f", datap->dop.tdop);
    }
    if (0 != isfinite(datap->dop.xdop)) {
        str_appendf(reply, replylen, ",\"xdop\":%.2f", datap->dop.xdop);
    }
    if (0 != isfinite(datap->dop.ydop)) {
        str_appendf(reply, replylen, ",\"ydop\":%.2f", datap->dop.ydop);
    }
    if (0 != isfinite(datap->dop.vdop)) {
        str_appendf(reply, replylen, ",\"vdop\":%.2f", datap->dop.vdop);
    }
    if (0 != (datap->set & SATELLITE_SET)) {
        // insurance against flaky drivers
        for (i = 0; i < datap->satellites_visible; i++)
            if (datap->skyview[i].PRN) {
                reported++;
                if (datap->skyview[i].used) {
                    used++;
                }
            }
        str_appendf(reply, replylen, ",\"nSat\":%d,\"uSat\":%d",
                    reported, used);
        if (reported) {
            (void)strlcat(reply, ",\"satellites\":[", replylen);
            for (i = 0; i < reported; i++) {
                if (0 == datap->skyview[i].PRN) {
                    // blank slot.
                    continue;
                }
                // Put PRN, gnssid, svid, sigid, freqid, at front
                str_appendf(reply, replylen, "{\"PRN\":%d",
                            datap->skyview[i].PRN);
                if (0 != datap->skyview[i].svid) {
                    str_appendf(reply, replylen,
                       ",\"gnssid\":%d,\"svid\":%d",
                       datap->skyview[i].gnssid,
                       datap->skyview[i].svid);
                }
                if (0 != datap->skyview[i].sigid) {
                    str_appendf(reply, replylen,
                       ",\"sigid\":%d", datap->skyview[i].sigid);
                }
                if (GNSSID_GLO == datap->skyview[i].gnssid &&
                    0 <= datap->skyview[i].freqid &&
                    16 >= datap->skyview[i].freqid) {
                    str_appendf(reply, replylen,
                       ",\"freqid\":%d", datap->skyview[i].freqid);
                }
                // now the rest in alphabetic order.
                if (0 != isfinite(datap->skyview[i].azimuth) &&
                    0 <= fabs(datap->skyview[i].azimuth) &&
                    360 > fabs(datap->skyview[i].azimuth)) {
                    str_appendf(reply, replylen, ",\"az\":%.1f",
                                datap->skyview[i].azimuth);
                }
                if (0 != isfinite(datap->skyview[i].elevation) &&
                    90 >= fabs(datap->skyview[i].elevation)) {
                    str_appendf(reply, replylen, ",\"el\":%.1f",
                                datap->skyview[i].elevation);
                }
                if (0 != isfinite(datap->skyview[i].pr)) {
                    str_appendf(reply, replylen, ",\"pr\":%.3f",
                                datap->skyview[i].pr);
                }
                if (0 != isfinite(datap->skyview[i].prRate)) {
                    str_appendf(reply, replylen, ",\"prRate\":%.1f",
                                datap->skyview[i].prRate);
                }
                if (0 != isfinite(datap->skyview[i].prRes)) {
                    str_appendf(reply, replylen, ",\"prRes\":%.1f",
                                datap->skyview[i].prRes);
                }
                if (0 <= datap->skyview[i].qualityInd) {
                    str_appendf(reply, replylen, ",\"qual\":%d",
                                datap->skyview[i].qualityInd);
                }
                if (0 != isfinite(datap->skyview[i].ss)) {
                    str_appendf(reply, replylen, ",\"ss\":%.1f",
                                datap->skyview[i].ss);
                }
                str_appendf(reply, replylen,
                   ",\"used\":%s",
                   datap->skyview[i].used ? "true" : "false");
                if (SAT_HEALTH_UNK != datap->skyview[i].health) {
                    str_appendf(reply, replylen,
                       ",\"health\":%d", datap->skyview[i].health);
                }
                (void)strlcat(reply, "},", replylen);
            }
            str_rstrip_char(reply, ',');
            (void)strlcat(reply, "]", replylen);
        }
    } else if (0 != session->nmea.gga_sats_used) {
        // no sat data, but we have number used from $_GGA, $__GNS, or $PASHR
        str_appendf(reply, replylen, ",\"uSat\":%u",
                    session->nmea.gga_sats_used);
    }
    if (header_len == strnlen(reply, replylen)) {
        // empty message, slip it
        *reply = '\0';
    } else {
        (void)strlcat(reply, "}\r\n", replylen);
    }
}

/* FiXME: should be done once in parse_uri_dest() or close by
 * strip "user@example.com:password@" from a uri.
 * better not be an @ after the host name.
 *
 * Needs proper tests.
 */
static const char *obfuscate_uri(const char *uri)
{
    static char buf[GPS_PATH_MAX];
    char *p;
    const char *at = NULL;
    const char *last_at = NULL;

    // Find the protocol separator
    const char* proto_end = strstr(uri, "://");
    if (!proto_end) {
        // none
        return uri;
    }

    at = proto_end + 3;
    last_at = NULL;
    while (at &&
           '\0' != *at) {
        at = strchr(at, '@');
        if (!at) {
            break;
        }
        last_at = at;
        at++;
    }
    if (!last_at) {
        return uri;   // No credentials,
    }

    // grab prefix
    p = stpncpy(buf, uri, 3 + proto_end - uri);
    // p = stpcpy(p, "XXXX:XXXX");  // just strip, not obfuscae.
    p = stpcpy(p, last_at + 1);

    return buf;
}

void json_device_dump(const struct gps_device_t *device,
                      char *reply, size_t replylen)
{
    struct classmap_t *cmp;
    char buf1[JSON_VAL_MAX * 2 + 1];

    (void)strlcpy(reply, "{\"class\":\"DEVICE\",\"path\":\"", replylen);
    (void)strlcat(reply, obfuscate_uri(device->gpsdata.dev.path), replylen);
    (void)strlcat(reply, "\"", replylen);
    if (NULL != device->device_type) {
        (void)strlcat(reply, ",\"driver\":\"", replylen);
        (void)strlcat(reply, device->device_type->type_name, replylen);
        (void)strlcat(reply, "\"", replylen);
    }
    if ('\0' != device->gpsdata.dev.sernum[0]) {
        (void)strlcat(reply, ",\"sernum\":\"", replylen);
        (void)strlcat(reply,
                      json_stringify(buf1, sizeof(buf1),
                                     device->gpsdata.dev.sernum),
                      replylen);
        (void)strlcat(reply, "\"", replylen);
    }
    if ('\0' != device->subtype[0]) {
        (void)strlcat(reply, ",\"subtype\":\"", replylen);
        (void)strlcat(reply,
                      json_stringify(buf1, sizeof(buf1), device->subtype),
                      replylen);
        (void)strlcat(reply, "\"", replylen);
    }
    if ('\0' != device->subtype1[0]) {
        (void)strlcat(reply, ",\"subtype1\":\"", replylen);
        (void)strlcat(reply,
                      json_stringify(buf1, sizeof(buf1), device->subtype1),
                      replylen);
        (void)strlcat(reply, "\"", replylen);
    }
    if (device->context->readonly) {
        (void)strlcat(reply, ",\"readonly\":\"true\"", replylen);
    }
    /*
     * There's an assumption here: Anything that we type SERVICE_SENSOR is
     * a serial device with the usual control parameters.
     */
    if (0 < device->gpsdata.online.tv_sec) {
        // odd, using online, not activated, time
        str_appendf(reply, replylen, ",\"activated\":\"%s\"",
                    timespec_to_iso8601(device->gpsdata.online,
                                        buf1, sizeof(buf1)));
        if (0 != device->observed) {
            int mask = 0;

            for (cmp = classmap; cmp < classmap + NITEMS(classmap); cmp++) {
                if (0 != (device->observed & cmp->packetmask)) {
                    mask |= cmp->typemask;
                }
            }
            if (0 != mask) {
                str_appendf(reply, replylen, ",\"flags\":%d", mask);
            }
        }
        if (SERVICE_SENSOR == device->servicetype) {
            int speed = 0;

            /* speed can be 0 if the device is not currently active,
             * or device is a file, pipe, /dev/pps, ttyACM, etc.
             * can be -1 if never configured. */
            if (0 < gpsd_serial_isatty(device) &&
                0 != (speed = gpsd_get_speed(device))) {

                str_appendf(reply, replylen,
                            ",\"native\":%d,\"bps\":%d,\"parity\":\"%c\","
                            "\"stopbits\":%u,\"cycle\":%lld.%02ld",
                            device->gpsdata.dev.driver_mode,
                            (int)speed,
                            device->gpsdata.dev.parity,
                            device->gpsdata.dev.stopbits,
                            (long long)device->gpsdata.dev.cycle.tv_sec,
                            device->gpsdata.dev.cycle.tv_nsec / 10000000);
            }
            if (NULL != device->device_type &&
                NULL != device->device_type->rate_switcher) {
                str_appendf(reply, replylen,
                               ",\"mincycle\":%lld.%02ld",
                               (long long)device->device_type->min_cycle.tv_sec,
                               device->device_type->min_cycle.tv_nsec /
                               10000000);
            }
        }
    }
    (void)strlcat(reply, "}\r\n", replylen);
}

void json_watch_dump(const struct gps_policy_t *ccp,
                     char *reply, size_t replylen)
{
    (void)snprintf(reply, replylen,
                   "{\"class\":\"WATCH\",\"enable\":%s,\"json\":%s,"
                   "\"nmea\":%s,\"raw\":%d,\"scaled\":%s,\"timing\":%s,"
                   "\"split24\":%s,\"pps\":%s",
                   ccp->watcher ? "true" : "false",
                   ccp->json ? "true" : "false",
                   ccp->nmea ? "true" : "false",
                   ccp->raw,
                   ccp->scaled ? "true" : "false",
                   ccp->timing ? "true" : "false",
                   ccp->split24 ? "true" : "false",
                   ccp->pps ? "true" : "false");
    // UNUSED: loglevel, remote
    if ('\0' != ccp->devpath[0]) {
        str_appendf(reply, replylen, ",\"device\":\"%s\"", ccp->devpath);
    }
    (void)strlcat(reply, "}\r\n", replylen);
}

// dump the hoppity skipity orbit_t
static void json_subframe_dump_orb(const orbit_t *orbit,
                                   const bool scaled UNUSED,
                                   char buf[], size_t buflen)
{
    str_appendf(buf, buflen, "\"sv\":%d", orbit->sv);

    if (0 <= orbit->AODC) {
        str_appendf(buf, buflen, ",\"AODC\":%d", orbit->AODC);
    }
    if (0 <= orbit->AODE) {
        str_appendf(buf, buflen, ",\"AODE\":%d", orbit->AODE);
    }
    if (0 != isfinite(orbit->af0)) {
        str_appendf(buf, buflen, ",\"af0\":%.12e", orbit->af0);
    }
    if (0 != isfinite(orbit->af1)) {
        str_appendf(buf, buflen, ",\"af1\":%.12e", orbit->af1);
    }
    if (0 != isfinite(orbit->af2)) {
        str_appendf(buf, buflen, ",\"af2\":%.12e", orbit->af2);
    }
    if (0 != isfinite(orbit->alpha0)) {
        str_appendf(buf, buflen, ",\"alpha0\":%.12e", orbit->alpha0);
    }
    if (0 != isfinite(orbit->alpha1)) {
        str_appendf(buf, buflen, ",\"alpha1\":%.12e", orbit->alpha1);
    }
    if (0 != isfinite(orbit->alpha2)) {
        str_appendf(buf, buflen, ",\"alpha2\":%.12e", orbit->alpha2);
    }
    if (0 != isfinite(orbit->alpha3)) {
        str_appendf(buf, buflen, ",\"alpha3\":%.12e", orbit->alpha3);
    }
    if (0 != isfinite(orbit->beta0)) {
        str_appendf(buf, buflen, ",\"beta0\":%.12e", orbit->beta0);
    }
    if (0 != isfinite(orbit->beta1)) {
        str_appendf(buf, buflen, ",\"beta1\":%.12e", orbit->beta1);
    }
    if (0 != isfinite(orbit->beta2)) {
        str_appendf(buf, buflen, ",\"beta2\":%.12e", orbit->beta2);
    }
    if (0 != isfinite(orbit->beta3)) {
        str_appendf(buf, buflen, ",\"beta3\":%.12e", orbit->beta3);
    }
    if (0 != isfinite(orbit->Cic)) {
        str_appendf(buf, buflen, ",\"Cic\":%.12e", orbit->Cic);
    }
    if (0 != isfinite(orbit->Cis)) {
        str_appendf(buf, buflen, ",\"Cis\":%.12e", orbit->Cis);
    }
    if (0 != isfinite(orbit->Crc)) {
        str_appendf(buf, buflen, ",\"Crc\":%.12e", orbit->Crc);
    }
    if (0 != isfinite(orbit->Crs)) {
        str_appendf(buf, buflen, ",\"Crs\":%.12e", orbit->Crs);
    }
    if (0 != isfinite(orbit->Cuc)) {
        str_appendf(buf, buflen, ",\"Cuc\":%.12e", orbit->Cuc);
    }
    if (0 != isfinite(orbit->Cus)) {
        str_appendf(buf, buflen, ",\"Cus\":%.12e", orbit->Cus);
    }
    if (0 != isfinite(orbit->deltai)) {
        str_appendf(buf, buflen, ",\"deltai\":%.12e", orbit->deltai);
    }
    if (0 != isfinite(orbit->deltan)) {
        str_appendf(buf, buflen, ",\"deltan\":%.12e", orbit->deltan);
    }
    if (0 <= orbit->E1BHS) {
        str_appendf(buf, buflen, ",\"E1BHS\":%d", orbit->E1BHS);
    }
    if (0 <= orbit->E5bHS) {
        str_appendf(buf, buflen, ",\"E5bHS\":%d", orbit->E5bHS);
    }
    if (0 != isfinite(orbit->eccentricity)) {
        str_appendf(buf, buflen, ",\"e\":%.12e", orbit->eccentricity);
    }
    if (0 != isfinite(orbit->IDOT)) {
        str_appendf(buf, buflen, ",\"IDOT\":%.16e", orbit->IDOT);
    }
    if (0 <= orbit->IODA) {
        str_appendf(buf, buflen, ",\"IODA\":%d", orbit->IODA);
    }
    if (0 <= orbit->IODC) {
        str_appendf(buf, buflen, ",\"IODC\":%d", orbit->IODC);
    }
    if (0 <= orbit->IODE) {
        str_appendf(buf, buflen, ",\"IODE\":%d", orbit->IODE);
    }
    if (0 != isfinite(orbit->i0)) {
        str_appendf(buf, buflen, ",\"i0\":%.16f", orbit->i0);
    }
    if (0 != isfinite(orbit->M0)) {
        str_appendf(buf, buflen, ",\"M0\":%.16f", orbit->M0);
    }
    if (0 != isfinite(orbit->Omega0)) {
        str_appendf(buf, buflen, ",\"Omega0\":%.16f", orbit->Omega0);
    }
    if (0 != isfinite(orbit->Omegad)) {
        str_appendf(buf, buflen, ",\"Omegad\":%.12e", orbit->Omegad);
    }
    if (0 != isfinite(orbit->omega)) {
        str_appendf(buf, buflen, ",\"omega\":%.16f", orbit->omega);
    }
    if (0 != isfinite(orbit->sqrtA) &&
        2600 < orbit->sqrtA) {
        // Sanity check: A must be greater than Earth radius
        str_appendf(buf, buflen, ",\"sqrtA\":%.12f", orbit->sqrtA);
    }
    if (0 <= orbit->SISAa) {
        str_appendf(buf, buflen, ",\"SISAa\":%d", orbit->SISAa);
    }
    if (0 <= orbit->SISAb) {
        str_appendf(buf, buflen, ",\"SISAb\":%d", orbit->SISAb);
    }
    if (0 <= orbit->svh) {
        str_appendf(buf, buflen, ",\"svh\":%d", orbit->svh);
    }
    if (0 != isfinite(orbit->TGD1)) {
        str_appendf(buf, buflen, ",\"TGD1\":%.1f", orbit->TGD1);
    }
    if (0 != isfinite(orbit->TGD2)) {
        str_appendf(buf, buflen, ",\"TGD2\":%.1f", orbit->TGD2);
    }
    if (0 <= orbit->toa) {
        str_appendf(buf, buflen, ",\"toa\":%ld", orbit->toa);
    }
    if (0 <= orbit->toc) {
        str_appendf(buf, buflen, ",\"toc\":%ld", orbit->toc);
    }
    if (0 <= orbit->toe) {
        str_appendf(buf, buflen, ",\"toe\":%ld", orbit->toe);
    } else if (0 <= orbit->toeLSB) {
        str_appendf(buf, buflen, ",\"toeLSB\":%ld", orbit->toeLSB);
    } else if (0 <= orbit->toeMSB) {
        str_appendf(buf, buflen, ",\"toeMSB\":%ld", orbit->toeMSB);
    }
    if (0 <= orbit->URAI) {
        str_appendf(buf, buflen, ",\"URAI\":%d", orbit->URAI);
    }
    if (0 <= orbit->WN) {
        str_appendf(buf, buflen, ",\"WN\":%d", orbit->WN);
    }
    (void)strlcat(buf, "}", buflen);
}

void json_subframe_dump(const struct gps_data_t *datap, const bool scaled,
                        char buf[], size_t buflen)
{
    const struct subframe_t *subframe = &datap->subframe;

    (void)snprintf(buf, buflen, "{\"class\":\"SUBFRAME\",\"device\":\"%s\","
                   "\"gnssId\":%u,\"tSV\":%u,\"frame\":%u",
                   datap->dev.path,
                   (unsigned int)subframe->gnssId,
                   (unsigned int)subframe->tSVID,
                   (unsigned int)subframe->subframe_num);

    if (0 <= subframe->WN) {
        str_appendf(buf, buflen, ",\"WN\":%d", subframe->WN);
    }

    if (0 <= subframe->TOW17) {
        // TOW17 is always scaled
        switch (subframe->gnssId){
        case GNSSID_GPS:
            FALLTHROUGH
        case GNSSID_SBAS:
            str_appendf(buf, buflen, ",\"TOW17\":%d", subframe->TOW17);
            break;
        case GNSSID_BD:
            str_appendf(buf, buflen, ",\"SOW\":%d", subframe->TOW17);
            break;
        case GNSSID_GAL:
            FALLTHROUGH
        default:
            str_appendf(buf, buflen, ",\"TOW\":%d", subframe->TOW17);
            break;
        }
    }

    if (SUBFRAME_ORBIT == subframe->is_almanac){
        str_appendf(buf, buflen, ",\"scaled\":true");
        switch (subframe->orbit.type) {
        case ORBIT_ALMANAC:
            (void)strlcat(buf, ",\"ALMANAC\":{", buflen);
            json_subframe_dump_orb(&subframe->orbit, scaled, buf, buflen);
            if (0 < subframe->orbit1.sv) {
                (void)strlcat(buf, ",\"ALMANAC1\":{", buflen);
                json_subframe_dump_orb(&subframe->orbit1, scaled, buf, buflen);
            }
            break;
        case ORBIT_EPHEMERIS:
            (void)strlcat(buf, ",\"EPHEMERIS\":{", buflen);
            json_subframe_dump_orb(&subframe->orbit, scaled, buf, buflen);
            break;
        default:
            // Huh?
            break;
        }
        (void)strlcat(buf, "}\r\n", buflen);
        return;
    }

    str_appendf(buf, buflen, ",\"scaled\":%s", JSON_BOOL(scaled));

    switch (subframe->subframe_num) {
    case 1:
        if (scaled) {
            // NASA uses RINEX 2 to report current ephemeris
            // RINEX 2, everything is %.12e, so we will too.
            str_appendf(buf, buflen,
                        ",\"EPHEM1\":{\"WN\":%u,\"IODC\":%u,\"L2\":%u,"
                        "\"ura\":%u,\"hlth\":%u,\"L2P\":%u,\"Tgd\":%.12e,"
                        "\"toc\":%lu,\"af2\":%.12e,\"af1\":%.12e,\"af0\":%.12e}",
                        (unsigned int)subframe->sub1.WN,
                        (unsigned int)subframe->sub1.IODC,
                        (unsigned int)subframe->sub1.l2,
                        subframe->sub1.ura,
                        subframe->sub1.hlth,
                        (unsigned int)subframe->sub1.l2p,
                        subframe->sub1.d_Tgd,
                        (unsigned long)subframe->sub1.l_toc,
                        subframe->sub1.d_af2,
                        subframe->sub1.d_af1,
                        subframe->sub1.d_af0);
        } else {
            str_appendf(buf, buflen,
                        ",\"EPHEM1\":{\"WN\":%u,\"IODC\":%u,\"L2\":%u,"
                        "\"ura\":%u,\"hlth\":%u,\"L2P\":%u,\"Tgd\":%d,"
                        "\"toc\":%u,\"af2\":%ld,\"af1\":%d,\"af0\":%d}",
                        (unsigned int)subframe->sub1.WN,
                        (unsigned int)subframe->sub1.IODC,
                        (unsigned int)subframe->sub1.l2,
                        subframe->sub1.ura,
                        subframe->sub1.hlth,
                        (unsigned int)subframe->sub1.l2p,
                        (int)subframe->sub1.Tgd,
                        (unsigned int)subframe->sub1.toc,
                        (long)subframe->sub1.af2,
                        (int)subframe->sub1.af1,
                        (int)subframe->sub1.af0);
        }
        break;
    case 2:
        if (scaled) {
            // NASA uses RINEX 2 to report current ephemeris
            // RINEX 2, everything is %.12e, so we will too.
            str_appendf(buf, buflen,
                        ",\"EPHEM2\":{\"IODE\":%u,\"Crs\":%.12e,"
                        "\"deltan\":%.12e,\"M0\":%.12e,\"Cuc\":%.12e,"
                        "\"e\":%.12e,\"Cus\":%.12e," "\"sqrtA\":%.12e,"
                        "\"toe\":%lu,\"FIT\":%u,\"AODO\":%u}",
                        (unsigned int)subframe->sub2.IODE,
                        subframe->sub2.d_Crs,
                        subframe->sub2.d_deltan,
                        subframe->sub2.d_M0,
                        subframe->sub2.d_Cuc,
                        subframe->sub2.d_eccentricity,
                        subframe->sub2.d_Cus,
                        subframe->sub2.d_sqrtA,
                        (unsigned long)subframe->sub2.l_toe,
                        (unsigned int)subframe->sub2.fit,
                        (unsigned int)subframe->sub2.u_AODO);
        } else {
            str_appendf(buf, buflen,
                        ",\"EPHEM2\":{\"IODE\":%u,\"Crs\":%d,\"deltan\":%d,"
                        "\"M0\":%ld,\"Cuc\":%d,\"e\":%ld,\"Cus\":%d,"
                        "\"sqrtA\":%lu,\"toe\":%lu,\"FIT\":%u,\"AODO\":%u}",
                        (unsigned int)subframe->sub2.IODE,
                        (int)subframe->sub2.Crs,
                        (int)subframe->sub2.deltan,
                        (long)subframe->sub2.M0,
                        (int)subframe->sub2.Cuc,
                        (long)subframe->sub2.e,
                        (int)subframe->sub2.Cus,
                        (unsigned long)subframe->sub2.sqrtA,
                        (unsigned long)subframe->sub2.toe,
                        (unsigned int)subframe->sub2.fit,
                        (unsigned int)subframe->sub2.AODO);
        }
        break;
    case 3:
        if (scaled) {
            // NASA uses RINEX 2 to report current ephemeris
            // RINEX 2, everything is %.12e, so we will too.
            str_appendf(buf, buflen,
                        ",\"EPHEM3\":{\"IODE\":%3u,\"IDOT\":%.12e,"
                        "\"Cic\":%.12e,\"Omega0\":%.12e,\"Cis\":%.12e,"
                        "\"i0\":%.12e,\"Crc\":%.12e,\"omega\":%.12e,"
                        "\"Omegad\":%.12e}",
                        (unsigned int)subframe->sub3.IODE,
                        subframe->sub3.d_IDOT,
                        subframe->sub3.d_Cic,
                        subframe->sub3.d_Omega0,
                        subframe->sub3.d_Cis,
                        subframe->sub3.d_i0,
                        subframe->sub3.d_Crc,
                        subframe->sub3.d_omega,
                        subframe->sub3.d_Omegad );
        } else {
            str_appendf(buf, buflen,
                        ",\"EPHEM3\":{\"IODE\":%u,\"IDOT\":%u,\"Cic\":%u,"
                        "\"Omega0\":%ld,\"Cis\":%d,\"i0\":%ld,\"Crc\":%d,"
                        "\"omega\":%ld,\"Omegad\":%ld}",
                        (unsigned int)subframe->sub3.IODE,
                        (unsigned int)subframe->sub3.IDOT,
                        (unsigned int)subframe->sub3.Cic,
                        (long int)subframe->sub3.Omega0,
                        (int)subframe->sub3.Cis,
                        (long int)subframe->sub3.i0,
                        (int)subframe->sub3.Crc,
                        (long int)subframe->sub3.omega,
                        (long int)subframe->sub3.Omegad);
        }
        break;
    case 4:
        FALLTHROUGH
    case 5:
        // pageid is unique to all of subframes 4 and 5, handle as one

        str_appendf(buf, buflen, ",\"dataid\":%u",
                    (unsigned int)subframe->pageid);
        if (subframe->is_almanac) {
            if (scaled) {
                // IS-GPS-240 uses 14 digits past decimal, so we do too
                str_appendf(buf, buflen,
                            ",\"ALMANAC\":{\"ID\":%d,\"Health\":%u,"
                            "\"e\":%.14e,\"toa\":%lu,"
                            "\"deltai\":%.14e,\"Omegad\":%.14e,\"sqrtA\":%.14e,"
                            "\"Omega0\":%.14e,\"omega\":%.14e,\"M0\":%.14e,"
                            "\"af0\":%.14e,\"af1\":%.14e}",
                            (int)subframe->sub5.almanac.sv,
                            (unsigned int)subframe->sub5.almanac.svh,
                            subframe->sub5.almanac.d_eccentricity,
                            subframe->sub5.almanac.l_toa,
                            subframe->sub5.almanac.d_deltai,
                            subframe->sub5.almanac.d_Omegad,
                            subframe->sub5.almanac.d_sqrtA,
                            subframe->sub5.almanac.d_Omega0,
                            subframe->sub5.almanac.d_omega,
                            subframe->sub5.almanac.d_M0,
                            subframe->sub5.almanac.d_af0,
                            subframe->sub5.almanac.d_af1);
            } else {
                str_appendf(buf, buflen,
                            ",\"ALMANAC\":{\"ID\":%d,\"Health\":%u,"
                            "\"e\":%u,\"toa\":%u,"
                            "\"deltai\":%d,\"Omegad\":%d,\"sqrtA\":%lu,"
                            "\"Omega0\":%ld,\"omega\":%ld,\"M0\":%ld,"
                            "\"af0\":%d,\"af1\":%d}",
                            (int)subframe->sub5.almanac.sv,
                            (unsigned int)subframe->sub5.almanac.svh,
                            (unsigned int)subframe->sub5.almanac.e,
                            (unsigned int)subframe->sub5.almanac.toa,
                            (int)subframe->sub5.almanac.deltai,
                            (int)subframe->sub5.almanac.Omegad,
                            (unsigned long)subframe->sub5.almanac.sqrtA,
                            (long)subframe->sub5.almanac.Omega0,
                            (long)subframe->sub5.almanac.omega,
                            (long)subframe->sub5.almanac.M0,
                            (int)subframe->sub5.almanac.af0,
                            (int)subframe->sub5.almanac.af1);
            }
        } else {
            int i;

            switch (subframe->pageid ) {
            case 51:    // subframe5, page 25
                str_appendf(buf, buflen,
                    ",\"HEALTH2\":{\"toa\":%lu,\"WNa\":%u",
                               subframe->sub5_25.l_toa,
                               (unsigned int)subframe->sub5_25.WNa);
                // 1-index loop to construct json
                for(i = 1 ; i <= 24; i++){
                    str_appendf(buf, buflen,
                                ",\"SVH%d\":%u",
                                i, subframe->sub5_25.sv[i]);
                }
                str_appendf(buf, buflen, "}");
                break;

            case 52:    // data ID 52, subframe 4, page 13, aka NMCT
                // decoding of ERD to SV is non trivial and not done yet
                str_appendf(buf, buflen,
                    ",\"NMCT\":{\"ai\":%u", subframe->sub4_13.ai);

                // 1-index loop to construct json, rather than giant snprintf
                // ERD for SV 32, and for transmitting SV, are never sent.
                for(i = 1 ; i < 32; i++){
                    // 0xe0 is sign extended 0x20, is -32, marked invalid
                    if (scaled) {
                        // JSON has no nan, use "null" instead
                        if (-32 >= subframe->sub4_13.ERD[i]) {
                            str_appendf(buf, buflen, ",\"ERD%d\":\"null\"", i);
                        } else {
                            str_appendf(buf, buflen, ",\"ERD%d\":%.3f", i,
                                        subframe->sub4_13.ERD[i] * 0.3);
                        }
                    } else {
                        str_appendf(buf, buflen, ",\"ERD%d\":%d", i,
                                    subframe->sub4_13.ERD[i]);
                    }
                }
                str_appendf(buf, buflen, "}");
                break;

            case 55:   // subframe 4, page 17, System Message
                /* JSON is UTF-8. double quote, backslash and
                 * control charactores (U+0000 through U+001F).must be
                 * escaped. */
                /* system message can be 24 bytes, JSON can escape all
                 * chars so up to 24*6 long. */

                {
                    char buf1[25 * 6];

                    (void)json_stringify(buf1, sizeof(buf1),
                                         subframe->sub4_17.str);
                    str_appendf(buf, buflen, ",\"system_message\":\"%.144s\"",
                                buf1);
                }
                break;

            case 56:   // subframe 4, page 18
                if (scaled) {
                    str_appendf(buf, buflen,
                        ",\"IONO\":{\"a0\":%.5g,\"a1\":%.5g,\"a2\":%.5g,"
                        "\"a3\":%.5g,\"b0\":%.5g,\"b1\":%.5g,\"b2\":%.5g,"
                        "\"b3\":%.5g,\"A1\":%.11e,\"A0\":%.11e,"
                        "\"tot\":%lu,\"WNt\":%u,\"ls\":%d,\"WNlsf\":%u,"
                        "\"DN\":%u,\"lsf\":%d}",
                        subframe->sub4_18.d_alpha0,
                        subframe->sub4_18.d_alpha1,
                        subframe->sub4_18.d_alpha2,
                        subframe->sub4_18.d_alpha3,
                        subframe->sub4_18.d_beta0,
                        subframe->sub4_18.d_beta1,
                        subframe->sub4_18.d_beta2,
                        subframe->sub4_18.d_beta3,
                        subframe->sub4_18.d_A1,
                        subframe->sub4_18.d_A0,
                        subframe->sub4_18.t_tot,
                        subframe->sub4_18.WNt,
                        subframe->sub4_18.leap,
                        subframe->sub4_18.WNlsf,
                        subframe->sub4_18.DN,
                        subframe->sub4_18.lsf);
                } else {
                    str_appendf(buf, buflen,
                        ",\"IONO\":{\"a0\":%d,\"a1\":%d,\"a2\":%d,"
                        "\"a3\":%d,\"b0\":%d,\"b1\":%d,\"b2\":%d,"
                        "\"b3\":%d,\"A1\":%ld,\"A0\":%ld,\"tot\":%u,"
                        "\"WNt\":%u,\"ls\":%d,\"WNlsf\":%u,\"DN\":%u,"
                        "\"lsf\":%d}",
                        (int)subframe->sub4_18.alpha0,
                        (int)subframe->sub4_18.alpha1,
                        (int)subframe->sub4_18.alpha2,
                        (int)subframe->sub4_18.alpha3,
                        (int)subframe->sub4_18.beta0,
                        (int)subframe->sub4_18.beta1,
                        (int)subframe->sub4_18.beta2,
                        (int)subframe->sub4_18.beta3,
                        (long)subframe->sub4_18.A1,
                        (long)subframe->sub4_18.A0,
                        subframe->sub4_18.tot,
                        subframe->sub4_18.WNt,
                        subframe->sub4_18.leap,
                        subframe->sub4_18.WNlsf,
                        subframe->sub4_18.DN,
                        subframe->sub4_18.lsf);
                }
                break;
            case 63:      // subframe 4, page 25
                str_appendf(buf, buflen, ",\"HEALTH\":{\"SV1\":%d",
                                (int)subframe->sub4_25.svf[1]);

                // 1-index loop to construct json, rather than giant snprintf
                for(i = 2 ; i <= 32; i++) {
                    str_appendf(buf, buflen, ",\"SV%d\":%u",
                                i, subframe->sub4_25.svf[i]);
                }
                for(i = 0 ; i < 8; i++) {
                    // 0-index
                    str_appendf(buf, buflen, ",\"SVH%d\":%u",
                                i + 25, subframe->sub4_25.svhx[i]);
                }
                str_appendf(buf, buflen, "}");
                break;
            default:
                break;
            }
        }
    }
    (void)strlcat(buf, "}\r\n", buflen);
}

// RAW dump - should be good enough to make a RINEX 3 file
void json_raw_dump(const struct gps_data_t *gpsdata,
                   char *reply, size_t replylen)
{
    int i;

    if (0 == gpsdata->raw.mtime.tv_sec) {
        // no data to dump
        return;
    }
    (void)strlcpy(reply, "{\"class\":\"RAW\"", replylen);
    if ('\0' != gpsdata->dev.path[0]) {
        str_appendf(reply, replylen, ",\"device\":\"%s\"", gpsdata->dev.path);
    }

    str_appendf(reply, replylen, ",\"time\":%lld,\"nsec\":%ld,\"rawdata\":[",
                (long long)gpsdata->raw.mtime.tv_sec,
                gpsdata->raw.mtime.tv_nsec);

    for (i = 0; i < MAXCHANNELS; i++) {
        if (0 == gpsdata->raw.meas[i].svid ||
            255 == gpsdata->raw.meas[i].svid) {
            // skip empty and GLONASS 255
            continue;
        }
        str_appendf(reply, replylen,
                    "{\"gnssid\":%u,\"svid\":%u,\"snr\":%u,"
                    "\"obs\":\"%s\",\"lli\":%1u,\"locktime\":%u",
                    gpsdata->raw.meas[i].gnssid, gpsdata->raw.meas[i].svid,
                    gpsdata->raw.meas[i].snr,
                    gpsdata->raw.meas[i].obs_code, gpsdata->raw.meas[i].lli,
                    gpsdata->raw.meas[i].locktime);
        if (0 < gpsdata->raw.meas[i].sigid) {
            str_appendf(reply, replylen, ",\"sigid\":%u",
                        gpsdata->raw.meas[i].sigid);
        }
        if (GNSSID_GLO == gpsdata->raw.meas[i].gnssid) {
            str_appendf(reply, replylen, ",\"freqid\":%u",
                        gpsdata->raw.meas[i].freqid);
        }

        if (0 != isfinite(gpsdata->raw.meas[i].pseudorange) &&
            1.0 < gpsdata->raw.meas[i].pseudorange) {
            str_appendf(reply, replylen, ",\"pseudorange\":%f",
                        gpsdata->raw.meas[i].pseudorange);

            if (0 != isfinite(gpsdata->raw.meas[i].carrierphase)) {
                str_appendf(reply, replylen, ",\"carrierphase\":%f",
                            gpsdata->raw.meas[i].carrierphase);
            }
        }
        if (0 != isfinite(gpsdata->raw.meas[i].doppler)) {
            str_appendf(reply, replylen, ",\"doppler\":%f",
                        gpsdata->raw.meas[i].doppler);
        }

        // L2 C/A pseudo range, RINEX C2C
        if (0 != isfinite(gpsdata->raw.meas[i].c2c) &&
            1.0 < gpsdata->raw.meas[i].c2c) {
            str_appendf(reply, replylen, ",\"c2c\":%f",
                        gpsdata->raw.meas[i].c2c);

            // L2 C/A carrier phase, RINEX L2C
            if (0 != isfinite(gpsdata->raw.meas[i].l2c)) {
                str_appendf(reply, replylen, ",\"l2c\":%f",
                            gpsdata->raw.meas[i].l2c);
            }
        }
        (void)strlcat(reply, "},", replylen);
    }
    str_rstrip_char(reply, ',');
    (void)strlcat(reply, "]}\r\n", replylen);
}

// compare two struct rtk_sat_t
static int rtk_sat_cmp(const void *a, const void *b)
{
    const struct rtk_sat_t *A = (const struct rtk_sat_t*)a;
    const struct rtk_sat_t *B = (const struct rtk_sat_t*)b;

    return A->ident - B->ident;
}

// dump the contents of a parsed RTCM104 message as JSON
void json_rtcm2_dump(struct rtcm2_t *rtcm,
                     const char *device,
                     char buf[], size_t buflen)
{
    char buf1[JSON_VAL_MAX * 2 + 1];
    unsigned int n;

    if (NULL == rtcm) {
        // shut up clang about possible NULL pointer
        return;
    }

    (void)snprintf(buf, buflen, "{\"class\":\"RTCM2\",");
    if (NULL != device &&
        '\0' != device[0]) {
        str_appendf(buf, buflen, "\"device\":\"%s\",", device);
    }
    str_appendf(buf, buflen,
                "\"type\":%u,\"station_id\":%u,\"zcount\":%0.1f,"
                "\"seqnum\":%u,\"length\":%u,\"station_health\":%u,",
                rtcm->type, rtcm->refstaid, rtcm->zcount, rtcm->seqnum,
                rtcm->length, rtcm->stathlth);

    switch (rtcm->type) {
    case 1:
        FALLTHROUGH
    case 9:
        (void)strlcat(buf, "\"satellites\":[", buflen);
        for (n = 0; n < rtcm->gps_ranges.nentries; n++) {
            const struct gps_rangesat_t *rsp = &rtcm->gps_ranges.sat[n];

            str_appendf(buf, buflen,
                           "{\"ident\":%u,\"udre\":%u,\"iod\":%u,"
                           "\"prc\":%0.3f,\"rrc\":%0.3f},",
                           rsp->ident,
                           rsp->udre, rsp->iod,
                           rsp->prc, rsp->rrc);
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 3:
        if (rtcm->ref_sta.valid) {
            str_appendf(buf, buflen,
                        "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,",
                        rtcm->ref_sta.x, rtcm->ref_sta.y, rtcm->ref_sta.z);
        }
        break;

    case 4:
        if (rtcm->reference.valid) {
            /*
             * Beware! Needs to stay synchronized with a JSON
             * enumeration map in the parser. This interpretation of
             * NAVSYSTEM_GALILEO is assumed from RTCM3, it's not
             * actually documented in RTCM 2.1 or 2.2.
             */
            static char *navsysnames[] = { "GPS", "GLONASS", "GALILEO" };
            str_appendf(buf, buflen,
                           "\"system\":\"%s\",\"sense\":%1d,"
                           "\"datum\":\"%s\",\"dx\":%.1f,\"dy\":%.1f,"
                           "\"dz\":%.1f,",
                           rtcm->reference.system >= NITEMS(navsysnames)
                           ? "UNKNOWN"
                           : navsysnames[rtcm->reference.system],
                           rtcm->reference.sense,
                           rtcm->reference.datum,
                           rtcm->reference.dx,
                           rtcm->reference.dy, rtcm->reference.dz);
        }
        break;

    case 5:
        (void)strlcat(buf, "\"satellites\":[", buflen);
        for (n = 0; n < rtcm->conhealth.nentries; n++) {
            const struct consat_t *csp = &rtcm->conhealth.sat[n];

            str_appendf(buf, buflen,
                           "{\"ident\":%u,\"iodl\":%s,\"health\":%1u,"
                           "\"snr\":%d,\"health_en\":%s,\"new_data\":%s,"
                           "\"los_warning\":%s,\"tou\":%u},",
                           csp->ident,
                           JSON_BOOL(csp->iodl),
                           (unsigned)csp->health,
                           csp->snr,
                           JSON_BOOL(csp->health_en),
                           JSON_BOOL(csp->new_data),
                           JSON_BOOL(csp->los_warning), csp->tou);
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 6:                     // NOP msg
        break;

    case 7:
        (void)strlcat(buf, "\"satellites\":[", buflen);
        for (n = 0; n < rtcm->almanac.nentries; n++) {
            const struct station_t *ssp = &rtcm->almanac.station[n];

            str_appendf(buf, buflen,
                        "{\"lat\":%.4f,\"lon\":%.4f,\"range\":%u,"
                        "\"frequency\":%.1f,\"health\":%u,"
                        "\"station_id\":%u,\"bitrate\":%u},",
                        ssp->latitude,
                        ssp->longitude,
                        ssp->range,
                        ssp->frequency,
                        ssp->health, ssp->station_id, ssp->bitrate);
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 13:
        str_appendf(buf, buflen,
                    "\"status\":%s,\"rangeflag\":%s,"
                    "\"lat\":%.2f,\"lon\":%.2f,\"range\":%u,",
                    JSON_BOOL(rtcm->xmitter.status),
                    JSON_BOOL(rtcm->xmitter.rangeflag),
                    rtcm->xmitter.lat,
                    rtcm->xmitter.lon,
                    rtcm->xmitter.range);
        break;

    case 14:
        str_appendf(buf, buflen,
                    "\"week\":%u,\"hour\":%u,\"leapsecs\":%u,",
                    rtcm->gpstime.week,
                    rtcm->gpstime.hour,
                    rtcm->gpstime.leapsecs);
        break;

    case 16:
        str_appendf(buf, buflen,
                    "\"message\":\"%s\"", json_stringify(buf1,
                    sizeof(buf1),
                    rtcm->message));
        break;

    case 18:
        str_appendf(buf, buflen, "\"tom\":%u,\"f\":%u,",
                    rtcm->rtk.tom, rtcm->rtk.f);
        (void)strlcat(buf, "\"satellites\":[", buflen);
        // sorted lists are nicer
        qsort((void *)rtcm->rtk.sat, rtcm->rtk.nentries,
              sizeof(rtcm->rtk.sat[0]), rtk_sat_cmp);
        for (n = 0; n < rtcm->rtk.nentries; n++) {
            str_appendf(buf, buflen,
                        "{\"ident\":%u,\"m\":%u,\"pc\":%u,\"g\":%u,\"dq\":%u,"
                        "\"clc\":%u,\"carrierphase\":%u},",
                        rtcm->rtk.sat[n].ident,
                        rtcm->rtk.sat[n].m,
                        rtcm->rtk.sat[n].pc,
                        rtcm->rtk.sat[n].g,
                        rtcm->rtk.sat[n].dq,
                        rtcm->rtk.sat[n].clc,
                        rtcm->rtk.sat[n].carrier_phase);
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 19:
        str_appendf(buf, buflen, "\"tom\":%u,\"f\":%u,\"sm\":%u,",
                    rtcm->rtk.tom, rtcm->rtk.f, rtcm->rtk.sm);
        (void)strlcat(buf, "\"satellites\":[", buflen);
        // sorted lists are nicer
        qsort((void *)rtcm->rtk.sat, rtcm->rtk.nentries,
              sizeof(rtcm->rtk.sat[0]), rtk_sat_cmp);
        for (n = 0; n < rtcm->rtk.nentries; n++) {
            str_appendf(buf, buflen,
                        "{\"ident\":%u,\"m\":%u,\"pc\":%u,\"g\":%u,\"dq\":%u,"
                        "\"me\":%u,\"pseudorange\":%u},",
                        rtcm->rtk.sat[n].ident,
                        rtcm->rtk.sat[n].m,
                        rtcm->rtk.sat[n].pc,
                        rtcm->rtk.sat[n].g,
                        rtcm->rtk.sat[n].dq,
                        rtcm->rtk.sat[n].me,
                        rtcm->rtk.sat[n].pseudorange);
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 20:
        str_appendf(buf, buflen, "\"tom\":%u,\"f\":%u,",
                    rtcm->rtk.tom, rtcm->rtk.f);
        break;

    case 21:
        str_appendf(buf, buflen, "\"tom\":%u,\"f\":%u,\"sm\":%u,",
                    rtcm->rtk.tom, rtcm->rtk.f, rtcm->rtk.sm);
        break;

    case 22:
        str_appendf(buf, buflen, "\"gs\":%u,", rtcm->ref_sta.gs);

        if (0 != isfinite(rtcm->ref_sta.dx) &&
            0 != isfinite(rtcm->ref_sta.dy) &&
            0 != isfinite(rtcm->ref_sta.dz)) {
            // L1 ECEF deltas
            str_appendf(buf, buflen,
                        "\"dx\":%.6f,\"dy\":%.6f,\"dz\":%.6f,",
                        rtcm->ref_sta.dx, rtcm->ref_sta.dy,
                        rtcm->ref_sta.dz);
        }
        if (0 != isfinite(rtcm->ref_sta.ah)) {
            // Antenna Height above reference point, cm
            str_appendf(buf, buflen, "\"ah\":%.6f,", rtcm->ref_sta.ah);
        }
        if (0 != isfinite(rtcm->ref_sta.dx2) &&
            0 != isfinite(rtcm->ref_sta.dy2) &&
            0 != isfinite(rtcm->ref_sta.dz2)) {
            // L2 ECEF deltas
            str_appendf(buf, buflen,
                           "\"dx2\":%.6f,\"dy2\":%.6f,\"dz2\":%.6f,",
                           rtcm->ref_sta.dx, rtcm->ref_sta.dy2,
                           rtcm->ref_sta.dz);
        }
        break;

    case 23:
        str_appendf(buf, buflen, "\"ar\":\"%d\",\"sid\":\"%u\",",
                    rtcm->ref_sta.ar,
                    rtcm->ref_sta.setup_id);
        if ('\0' != rtcm->ref_sta.ant_desc[0]) {
            str_appendf(buf, buflen, "\"ad\":\"%.32s\",",
                        rtcm->ref_sta.ant_desc);
        }
        if ('\0' != rtcm->ref_sta.ant_serial[0]) {
            str_appendf(buf, buflen, "\"as\":\"%.32s\",",
                        rtcm->ref_sta.ant_serial);
        }
        break;

    case 24:
        str_appendf(buf, buflen, "\"gs\":%u,", rtcm->ref_sta.gs);

        if (0 != isfinite(rtcm->ref_sta.x) &&
            0 != isfinite(rtcm->ref_sta.y) &&
            0 != isfinite(rtcm->ref_sta.z)) {
            // L1 ECEF
            str_appendf(buf, buflen,
                        "\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,",
                        rtcm->ref_sta.x, rtcm->ref_sta.y, rtcm->ref_sta.z);
        }
        if (0 != isfinite(rtcm->ref_sta.ah)) {
            // Antenna Height above reference point, cm
            str_appendf(buf, buflen, "\"ah\":%.4f,", rtcm->ref_sta.ah);
        }
        break;

    case 31:
        (void)strlcat(buf, "\"satellites\":[", buflen);
        for (n = 0; n < rtcm->glonass_ranges.nentries; n++) {
            const struct glonass_rangesat_t *rsp = &rtcm->glonass_ranges.sat[n];

            str_appendf(buf, buflen,
                        "{\"ident\":%u,\"udre\":%u,\"change\":%s,"
                        "\"tod\":%u,\"prc\":%0.3f,\"rrc\":%0.3f},",
                        rsp->ident,
                        rsp->udre,
                        JSON_BOOL(rsp->change),
                        rsp->tod,
                        rsp->prc, rsp->rrc);
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    default:
        (void)strlcat(buf, "\"data\":[", buflen);
        for (n = 0; n < rtcm->length; n++) {
            str_appendf(buf, buflen, "\"0x%08x\",", rtcm->words[n]);
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;
    }

    str_rstrip_char(buf, ',');
    (void)strlcat(buf, "}\r\n", buflen);
}

/* dump the contents of a parsed RTCM104v3 message into buf as JSON
 *
 * return: void
 */
void json_rtcm3_dump(const struct rtcm3_t *rtcm,
                     const char *device,
                     char buf[], size_t buflen)
{
    char buf1[JSON_VAL_MAX * 2 + 1];
    char buf2[JSON_VAL_MAX * 2 + 1];
    char *ptr;                       // utility pointer
    unsigned short i;
    unsigned int n;

    if (0 == rtcm->type ||
        0 == rtcm->length) {
        // runt, ignore
        return;
    }
    (void)snprintf(buf, buflen, "{\"class\":\"RTCM3\",");
    if (NULL != device &&
        '\0' != device[0]) {
        str_appendf(buf, buflen, "\"device\":\"%s\",", device);
    }
    str_appendf(buf, buflen, "\"type\":%u,", rtcm->type);
    str_appendf(buf, buflen, "\"length\":%u,", rtcm->length);

#define CODE(x) (unsigned int)(x)
#define INT(x) (unsigned int)(x)
    switch (rtcm->type) {
    case 1001:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"tow\":%d,\"sync\":\"%s\","
                    "\"smoothing\":\"%s\",\"interval\":\"%u\","
                    "\"satellites\":[",
                    rtcm->rtcmtypes.rtcm3_1001.header.station_id,
                    (int)rtcm->rtcmtypes.rtcm3_1001.header.tow,
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1001.header.sync),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1001.header.smoothing),
                    rtcm->rtcmtypes.rtcm3_1001.header.interval);
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1001.header.satcount; i++) {
#define R1001 rtcm->rtcmtypes.rtcm3_1001.rtk_data[i]
            str_appendf(buf, buflen,
                        "{\"ident\":%u,\"ind\":%u,\"prange\":%.2f,"
                        "\"delta\":%.4f,\"lockt\":%u},",
                        R1001.ident,
                        CODE(R1001.L1.indicator),
                        R1001.L1.pseudorange,
                        R1001.L1.rangediff,
                        INT(R1001.L1.locktime));
#undef R1001
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 1002:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"tow\":%d,\"sync\":\"%s\","
                    "\"smoothing\":\"%s\",\"interval\":\"%u\","
                    "\"satellites\":[",
                    rtcm->rtcmtypes.rtcm3_1002.header.station_id,
                    (int)rtcm->rtcmtypes.rtcm3_1002.header.tow,
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1002.header.sync),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1002.header.smoothing),
                    rtcm->rtcmtypes.rtcm3_1002.header.interval);
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1002.header.satcount; i++) {
#define R1002 rtcm->rtcmtypes.rtcm3_1002.rtk_data[i]
            str_appendf(buf, buflen,
                           "{\"ident\":%u,\"ind\":%u,\"prange\":%.2f,"
                           "\"delta\":%.4f,\"lockt\":%u,\"amb\":%u,"
                           "\"CNR\":%.2f},",
                           R1002.ident,
                           CODE(R1002.L1.indicator),
                           R1002.L1.pseudorange,
                           R1002.L1.rangediff,
                           INT(R1002.L1.locktime),
                           INT(R1002.L1.ambiguity),
                           R1002.L1.CNR);
#undef R1002
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 1003:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"tow\":%d,\"sync\":\"%s\","
                    "\"smoothing\":\"%s\",\"interval\":\"%u\","
                    "\"satellites\":[",
                    rtcm->rtcmtypes.rtcm3_1003.header.station_id,
                    (int)rtcm->rtcmtypes.rtcm3_1003.header.tow,
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1003.header.sync),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1003.header.smoothing),
                    rtcm->rtcmtypes.rtcm3_1003.header.interval);
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1003.header.satcount; i++) {
#define R1003 rtcm->rtcmtypes.rtcm3_1003.rtk_data[i]
            str_appendf(buf, buflen,
                        "{\"ident\":%u,"
                        "\"L1\":{\"ind\":%u,\"prange\":%.2f,"
                        "\"delta\":%.4f,\"lockt\":%u},"
                        "\"L2\":{\"ind\":%u,\"prange\":%.2f,"
                        "\"delta\":%.4f,\"lockt\":%u},"
                        "},",
                        R1003.ident,
                        CODE(R1003.L1.indicator),
                        R1003.L1.pseudorange,
                        R1003.L1.rangediff,
                        INT(R1003.L1.locktime),
                        CODE(R1003.L2.indicator),
                        R1003.L2.pseudorange,
                        R1003.L2.rangediff,
                        INT(R1003.L2.locktime));
#undef R1003
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 1004:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"tow\":%lu,\"sync\":\"%s\","
                    "\"smoothing\":\"%s\",\"interval\":\"%u\","
                    "\"satellites\":[",
                    rtcm->rtcmtypes.rtcm3_1004.header.station_id,
                    rtcm->rtcmtypes.rtcm3_1004.header.tow,
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1004.header.sync),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1004.header.smoothing),
                    rtcm->rtcmtypes.rtcm3_1004.header.interval);
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1004.header.satcount; i++) {
#define R1004 rtcm->rtcmtypes.rtcm3_1004.rtk_data[i]
            str_appendf(buf, buflen,
                        "{\"ident\":%u,"
                        "\"L1\":{\"ind\":%u,\"prange\":%0.2f,"
                        "\"delta\":%.4f,\"lockt\":%u,"
                        "\"amb\":%u,\"CNR\":%.2f},"
                        "\"L2\":{\"ind\":%u,\"prange\":%.2f,"
                        "\"delta\":%.4f,\"lockt\":%u,"
                        "\"CNR\":%.2f}"
                        "},",
                        R1004.ident,
                        CODE(R1004.L1.indicator),
                        R1004.L1.pseudorange,
                        R1004.L1.rangediff,
                        INT(R1004.L1.locktime),
                        INT(R1004.L1.ambiguity),
                        R1004.L1.CNR,
                        CODE(R1004.L2.indicator),
                        R1004.L2.pseudorange,
                        R1004.L2.rangediff,
                        INT(R1004.L2.locktime),
                        R1004.L2.CNR);
#undef R1004
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 1005:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"system\":[",
                    rtcm->rtcmtypes.rtcm3_1005.station_id);
        if (0 != (rtcm->rtcmtypes.rtcm3_1005.system & 0x04)) {
            (void)strlcat(buf, "\"GPS\",", buflen);
        }
        if (0 != (rtcm->rtcmtypes.rtcm3_1005.system & 0x02)) {
            (void)strlcat(buf, "\"GLONASS\",", buflen);
        }
        if (0 != (rtcm->rtcmtypes.rtcm3_1005.system & 0x01)) {
            (void)strlcat(buf, "\"GALILEO\",", buflen);
        }
        // FIXME: other systems now?
        str_rstrip_char(buf, ',');
        str_appendf(buf, buflen,
                    "],\"refstation\":%s,\"sro\":%s,"
                    "\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,",
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1005.reference_station),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1005.single_receiver),
                    rtcm->rtcmtypes.rtcm3_1005.ecef_x,
                    rtcm->rtcmtypes.rtcm3_1005.ecef_y,
                    rtcm->rtcmtypes.rtcm3_1005.ecef_z);
        break;

    case 1006:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"system\":[",
                    rtcm->rtcmtypes.rtcm3_1006.station_id);
        if (0 != (rtcm->rtcmtypes.rtcm3_1006.system & 0x04)) {
            (void)strlcat(buf, "\"GPS\",", buflen);
        }
        if (0 != (rtcm->rtcmtypes.rtcm3_1006.system & 0x02)) {
            (void)strlcat(buf, "\"GLONASS\",", buflen);
        }
        if (0 != (rtcm->rtcmtypes.rtcm3_1006.system & 0x01)) {
            (void)strlcat(buf, "\"GALILEO\",", buflen);
        }
        // FIXME: other systems now?
        str_rstrip_char(buf, ',');
        str_appendf(buf, buflen,
                    "],\"refstation\":%s,\"sro\":%s,"
                    "\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,"
                    "\"h\":%.4f,",
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1006.reference_station),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1006.single_receiver),
                    rtcm->rtcmtypes.rtcm3_1006.ecef_x,
                    rtcm->rtcmtypes.rtcm3_1006.ecef_y,
                    rtcm->rtcmtypes.rtcm3_1006.ecef_z,
                    rtcm->rtcmtypes.rtcm3_1006.height);
        break;

    case 1007:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"desc\":\"%s\",\"setup_id\":%u",
                    rtcm->rtcmtypes.rtcm3_1007.station_id,
                    rtcm->rtcmtypes.rtcm3_1007.descriptor,
                    rtcm->rtcmtypes.rtcm3_1007.setup_id);
        break;

    case 1008:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"desc\":\"%s\","
                    "\"setup_id\":%u,\"serial\":\"%s\"",
                    rtcm->rtcmtypes.rtcm3_1008.station_id,
                    rtcm->rtcmtypes.rtcm3_1008.descriptor,
                    INT(rtcm->rtcmtypes.rtcm3_1008.setup_id),
                    rtcm->rtcmtypes.rtcm3_1008.serial);
        break;

    case 1009:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"tow\":%lld,\"sync\":\"%s\","
                    "\"smoothing\":\"%s\",\"interval\":\"%u\","
                    "\"satcount\":\"%u\","
                    "\"satellites\":[",
                    rtcm->rtcmtypes.rtcm3_1009.header.station_id,
                    (long long)rtcm->rtcmtypes.rtcm3_1009.header.tow,
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1009.header.sync),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1009.header.smoothing),
                    rtcm->rtcmtypes.rtcm3_1009.header.interval,
                    rtcm->rtcmtypes.rtcm3_1009.header.satcount);
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1009.header.satcount; i++) {
#define R1009 rtcm->rtcmtypes.rtcm3_1009.rtk_data[i]
            str_appendf(buf, buflen,
                        "{\"ident\":%u,\"ind\":%u,\"channel\":%u,"
                        "\"prange\":%.2f,\"delta\":%.4f,\"lockt\":%u},",
                        R1009.ident,
                        CODE(R1009.L1.indicator),
                        R1009.L1.channel,
                        R1009.L1.pseudorange,
                        R1009.L1.rangediff,
                        INT(R1009.L1.locktime));
#undef R1009
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 1010:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"tow\":%d,\"sync\":\"%s\","
                    "\"smoothing\":\"%s\",\"interval\":\"%u\","
                    "\"satellites\":[",
                    rtcm->rtcmtypes.rtcm3_1010.header.station_id,
                    (int)rtcm->rtcmtypes.rtcm3_1010.header.tow,
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1010.header.sync),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1010.header.smoothing),
                    rtcm->rtcmtypes.rtcm3_1010.header.interval);
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1010.header.satcount; i++) {
#define R1010 rtcm->rtcmtypes.rtcm3_1010.rtk_data[i]
            str_appendf(buf, buflen,
                        "{\"ident\":%u,\"ind\":%u,\"channel\":%u,"
                        "\"prange\":%.2f,\"delta\":%.4f,\"lockt\":%u,"
                        "\"amb\":%u,\"CNR\":%.2f},",
                        R1010.ident,
                        CODE(R1010.L1.indicator),
                        R1010.L1.channel,
                        R1010.L1.pseudorange,
                        R1010.L1.rangediff,
                        INT(R1010.L1.locktime),
                        INT(R1010.L1.ambiguity),
                        R1010.L1.CNR);
#undef R1010
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 1011:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"tow\":%d,\"sync\":\"%s\","
                    "\"smoothing\":\"%s\",\"interval\":\"%u\","
                    "\"satellites\":[",
                    rtcm->rtcmtypes.rtcm3_1011.header.station_id,
                    (int)rtcm->rtcmtypes.rtcm3_1011.header.tow,
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1011.header.sync),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1011.header.smoothing),
                    rtcm->rtcmtypes.rtcm3_1011.header.interval);
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1011.header.satcount; i++) {
#define R1011 rtcm->rtcmtypes.rtcm3_1011.rtk_data[i]
            str_appendf(buf, buflen,
                        "{\"ident\":%u,\"channel\":%u,"
                        "\"L1\":{\"ind\":%u,"
                        "\"prange\":%.2f,\"delta\":%.4f,\"lockt\":%u},"
                        "\"L2:{\"ind\":%u,\"prange\":%.2f,"
                        "\"delta\":%.4f,\"lockt\":%u}"
                        "}",
                        R1011.ident,R1011.L1.channel,
                        CODE(R1011.L1.indicator),
                        R1011.L1.pseudorange,
                        R1011.L1.rangediff,
                        INT(R1011.L1.locktime),
                        CODE(R1011.L2.indicator),
                        R1011.L2.pseudorange,
                        R1011.L2.rangediff,
                        INT(R1011.L2.locktime));
#undef R1011
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 1012:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"tow\":%d,\"sync\":\"%s\","
                    "\"smoothing\":\"%s\",\"interval\":\"%u\","
                    "\"satellites\":[",
                    rtcm->rtcmtypes.rtcm3_1012.header.station_id,
                    (int)rtcm->rtcmtypes.rtcm3_1012.header.tow,
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1012.header.sync),
                    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1012.header.smoothing),
                    rtcm->rtcmtypes.rtcm3_1012.header.interval);
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1012.header.satcount; i++) {
#define R1012 rtcm->rtcmtypes.rtcm3_1012.rtk_data[i]
            str_appendf(buf, buflen,
                        "{\"ident\":%u,\"channel\":%u,"
                        "\"L1\":{\"ind\":%u,\"prange\":%.2f,"
                        "\"delta\":%.4f,\"lockt\":%u,\"amb\":%u,"
                        "\"CNR\":%.2f},"
                        "\"L2\":{\"ind\":%u,\"prange\":%.2f,"
                        "\"delta\":%.4f,\"lockt\":%u,"
                        "\"CNR\":%.2f}"
                        "},",
                        R1012.ident,
                        R1012.L1.channel,
                        CODE(R1012.L1.indicator),
                        R1012.L1.pseudorange,
                        R1012.L1.rangediff,
                        INT(R1012.L1.locktime),
                        INT(R1012.L1.ambiguity),
                        R1012.L1.CNR,
                        CODE(R1012.L2.indicator),
                        R1012.L2.pseudorange,
                        R1012.L2.rangediff,
                        INT(R1012.L2.locktime),
                        R1012.L2.CNR);
#undef R1012
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;

    case 1013:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"mjd\":%u,\"sec\":%u,"
                    "\"leapsecs\":%u",
                    rtcm->rtcmtypes.rtcm3_1013.station_id,
                    rtcm->rtcmtypes.rtcm3_1013.mjd,
                    rtcm->rtcmtypes.rtcm3_1013.sod,
                    INT(rtcm->rtcmtypes.rtcm3_1013.leapsecs));

        if (0 < rtcm->rtcmtypes.rtcm3_1013.ncount) {
	    (void)strlcat(buf, ",\"announcements\":[", buflen);

	    for (n = 0; n < (unsigned)rtcm->rtcmtypes.rtcm3_1013.ncount; n++) {
		str_appendf(buf, buflen,
			    "{\"id\":%u,\"sync\":\"%s\",\"interval\":%u},",
			    rtcm->rtcmtypes.rtcm3_1013.announcements[n].id,
			    JSON_BOOL(rtcm->rtcmtypes.rtcm3_1013.
				 announcements[n].sync),
			    rtcm->rtcmtypes.rtcm3_1013.
			    announcements[n].interval);
            }

	    str_rstrip_char(buf, ',');
	    (void)strlcat(buf, "]", buflen);
        }

        break;

    case 1014:
        str_appendf(buf, buflen,
                    "\"netid\":%u,\"subnetid\":%u,\"statcount\":%u,"
                    "\"master\":%u,\"aux\":%u,\"lat\":%f,\"lon\":%f,"
                    "\"alt\":%f,",
                    rtcm->rtcmtypes.rtcm3_1014.network_id,
                    rtcm->rtcmtypes.rtcm3_1014.subnetwork_id,
                    rtcm->rtcmtypes.rtcm3_1014.stationcount,
                    rtcm->rtcmtypes.rtcm3_1014.master_id,
                    rtcm->rtcmtypes.rtcm3_1014.aux_id,
                    rtcm->rtcmtypes.rtcm3_1014.d_lat,
                    rtcm->rtcmtypes.rtcm3_1014.d_lon,
                    rtcm->rtcmtypes.rtcm3_1014.d_alt);
        break;

    case 1015:    // GPS Ionospheric Correction Differences
        FALLTHROUGH
    case 1016:    // GPS Geometric Correction Differences
        FALLTHROUGH
    case 1017:    // GPS Combined Geometric & Iono Correction Differences
        // just the header for now
        str_appendf(buf, buflen,
                    "\"network_id\":%u,\"subnetwork_id\":%u,\"tow\":%lu,"
                    "\"multimesg\":%u,\"master_id\":%u,\"aux_id\":%u,"
                    "\"satcount\":%u,",
                    rtcm->rtcmtypes.rtcm3_1015.header.network_id,
                    rtcm->rtcmtypes.rtcm3_1015.header.subnetwork_id,
                    (long)rtcm->rtcmtypes.rtcm3_1015.header.tow,
                    rtcm->rtcmtypes.rtcm3_1015.header.multimesg,
                    rtcm->rtcmtypes.rtcm3_1015.header.master_id,
                    rtcm->rtcmtypes.rtcm3_1015.header.aux_id,
                    rtcm->rtcmtypes.rtcm3_1015.header.satcount);
        break;
    case 1021:
        str_appendf(buf, buflen,
                    "\"src_name\":\"%s\",\"tar_name\":\"%s\","
                    "\"sys_id\":%u, \"plate_number\":%u,"
                    "\"lat_origin\":%f,\"lon_origin\":%f,"
                    "\"lat_extension\":%f,\"lon_extension\":%f,"
                    "\"dX\":%.3f,\"dY\":%.3f,\"dZ\":%.3f,"
                    "\"rX\":%f,\"rY\":%f,\"rZ\":%f,\"dS\":%f,"
                    "\"add_as\":%.3f,\"add_bs\":%.3f,"
                    "\"add_at\":%.3f,\"add_bt\":%.3f,",
                    json_stringify(buf1, sizeof(buf1),
                                   rtcm->rtcmtypes.rtcm3_1021.src_name),
                    json_stringify(buf2, sizeof(buf2),
                                   rtcm->rtcmtypes.rtcm3_1021.tar_name),
                    rtcm->rtcmtypes.rtcm3_1021.sys_id_num,
                    rtcm->rtcmtypes.rtcm3_1021.plate_number,
                    rtcm->rtcmtypes.rtcm3_1021.lat_origin,
                    rtcm->rtcmtypes.rtcm3_1021.lon_origin,
                    rtcm->rtcmtypes.rtcm3_1021.lat_extension,
                    rtcm->rtcmtypes.rtcm3_1021.lon_extension,
                    rtcm->rtcmtypes.rtcm3_1021.x_trans,
                    rtcm->rtcmtypes.rtcm3_1021.y_trans,
                    rtcm->rtcmtypes.rtcm3_1021.z_trans,
                    rtcm->rtcmtypes.rtcm3_1021.x_rot,
                    rtcm->rtcmtypes.rtcm3_1021.y_rot,
                    rtcm->rtcmtypes.rtcm3_1021.z_rot,
                    rtcm->rtcmtypes.rtcm3_1021.ds,
                    rtcm->rtcmtypes.rtcm3_1021.add_as,
                    rtcm->rtcmtypes.rtcm3_1021.add_bs,
                    rtcm->rtcmtypes.rtcm3_1021.add_at,
                    rtcm->rtcmtypes.rtcm3_1021.add_bt
        );
        break;
    case 1023:
        str_appendf(buf, buflen,
                    "\"sys_id\":%u,"
                    "\"shift_h\":%u,\"shift_v\":%u,"
                    "\"lat_origin\":%f,\"lon_origin\":%f,"
                    "\"lat_extension\":%f,\"lon_extension\":%f,"
                    "\"lat_mean\":%.3f,\"lon_mean\":%.3f,\"hgt_mean\":%.2f,"
                    "\"mjd\":%u,\"residuals\":{",
                    rtcm->rtcmtypes.rtcm3_1023.sys_id_num,
                    rtcm->rtcmtypes.rtcm3_1023.shift_id_hori,
                    rtcm->rtcmtypes.rtcm3_1023.shift_id_vert,
                    rtcm->rtcmtypes.rtcm3_1023.lat_origin,
                    rtcm->rtcmtypes.rtcm3_1023.lon_origin,
                    rtcm->rtcmtypes.rtcm3_1023.lat_extension,
                    rtcm->rtcmtypes.rtcm3_1023.lon_extension,
                    rtcm->rtcmtypes.rtcm3_1023.lat_mean,
                    rtcm->rtcmtypes.rtcm3_1023.lon_mean,
                    rtcm->rtcmtypes.rtcm3_1023.hgt_mean,
                    rtcm->rtcmtypes.rtcm3_1023.mjd
        );
        for (i = 0; i < RTCM3_GRID_SIZE; i++) {
#define R1023 rtcm->rtcmtypes.rtcm3_1023.residuals[i]
            str_appendf(buf, buflen,
                        "\"lat_%02u\":%.5f,"
                        "\"lon_%02u\":%.5f,"
                        "\"hgt_%02u\":%.3f,",
                        i + 1, R1023.lat_res,
                        i + 1, R1023.lon_res,
                        i + 1, R1023.hgt_res);
#undef R1023
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "}", buflen);
            break;
    case 1025:
        switch (rtcm->rtcmtypes.rtcm3_1025.projection_type) {
        case PR_TM:
            ptr = "TM";
            break;
        case PR_TMS:
            ptr = "TMS";
            break;
        case PR_LCC1SP:
            ptr = "LCC1SP";
            break;
        case PR_LCC2SP:
            ptr = "LCC2SP";
            break;
        case PR_LCCW:
            ptr = "LCCW";
            break;
        case PR_CS:
            ptr = "CS";
            break;
        default:
            ptr = "UNKNOWN";
            break;
        }
        str_appendf(buf, buflen,
                    "\"sys_id\":%u,"
                    "\"lat_origin\":%.9f,\"lon_origin\":%.9f,"
                    "\"add_sno\":%.5f,"
                    "\"false_easting\":%.3f,\"false_northing\":%.3f,"
                    "\"projection_type\":\"%s\"",
                    rtcm->rtcmtypes.rtcm3_1025.sys_id_num,
                    rtcm->rtcmtypes.rtcm3_1025.lat_origin,
                    rtcm->rtcmtypes.rtcm3_1025.lon_origin,
                    rtcm->rtcmtypes.rtcm3_1025.add_sno,
                    rtcm->rtcmtypes.rtcm3_1025.false_east,
                    rtcm->rtcmtypes.rtcm3_1025.false_north, ptr);
        break;

    case 1029:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"mjd\":%u,\"sec\":%u,"
                    "\"len\":%zd,\"units\":%zd,\"msg\":\"%s\"",
                    rtcm->rtcmtypes.rtcm3_1029.station_id,
                    rtcm->rtcmtypes.rtcm3_1029.mjd,
                    rtcm->rtcmtypes.rtcm3_1029.sod,
                    rtcm->rtcmtypes.rtcm3_1029.len,
                    rtcm->rtcmtypes.rtcm3_1029.unicode_units,
                    json_stringify(buf1, sizeof(buf1),
                        (const char *)rtcm->rtcmtypes.rtcm3_1029.text));
        break;

    case 1033:
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"desc\":\"%s\","
                    "\"setup_id\":%u,\"serial\":\"%s\","
                    "\"receiver\":\"%s\",\"firmware\":\"%s\"",
                    rtcm->rtcmtypes.rtcm3_1033.station_id,
                    rtcm->rtcmtypes.rtcm3_1033.descriptor,
                    INT(rtcm->rtcmtypes.rtcm3_1033.setup_id),
                    rtcm->rtcmtypes.rtcm3_1033.serial,
                    rtcm->rtcmtypes.rtcm3_1033.receiver,
                    rtcm->rtcmtypes.rtcm3_1033.firmware);
        break;

    case 1071: // GPS MSM 1
        FALLTHROUGH
    case 1072: // GPS MSM 2
        FALLTHROUGH
    case 1073: // GPS MSM 3
        FALLTHROUGH
    case 1074: // GPS MSM 4
        FALLTHROUGH
    case 1075: // GPS MSM 5
        FALLTHROUGH
    case 1076: // GPS MSM 6
        FALLTHROUGH
    case 1077: // GPS MSM 7
        FALLTHROUGH
    case 1081: // GLO MSM 1
        FALLTHROUGH
    case 1082: // GLO MSM 2
        FALLTHROUGH
    case 1083: // GLO MSM 3
        FALLTHROUGH
    case 1084: // GLO MSM 4
        FALLTHROUGH
    case 1085: // GLO MSM 5
        FALLTHROUGH
    case 1086: // GLO MSM 6
        FALLTHROUGH
    case 1087: // GLO MSM 7
        FALLTHROUGH
    case 1091: // GAL MSM 1
        FALLTHROUGH
    case 1092: // GAL MSM 2
        FALLTHROUGH
    case 1093: // GAL MSM 3
        FALLTHROUGH
    case 1094: // GAL MSM 4
        FALLTHROUGH
    case 1095: // GAL MSM 5
        FALLTHROUGH
    case 1096: // GAL MSM 6
        FALLTHROUGH
    case 1097: // GAL MSM 7
        FALLTHROUGH
    case 1101: // SBAS MSM 1
        FALLTHROUGH
    case 1102: // SBAS MSM 2
        FALLTHROUGH
    case 1103: // SBAS MSM 3
        FALLTHROUGH
    case 1104: // SBAS MSM 4
        FALLTHROUGH
    case 1105: // SBAS MSM 5
        FALLTHROUGH
    case 1106: // SBAS MSM 6
        FALLTHROUGH
    case 1107: // SBAS MSM 7
        FALLTHROUGH
    case 1111: // QZSS MSM 1
        FALLTHROUGH
    case 1112: // QZSS MSM 2
        FALLTHROUGH
    case 1113: // QZSS MSM 3
        FALLTHROUGH
    case 1114: // QZSS MSM 4
        FALLTHROUGH
    case 1115: // QZSS MSM 5
        FALLTHROUGH
    case 1116: // QZSS MSM 6
        FALLTHROUGH
    case 1117: // QZSS MSM 7
        FALLTHROUGH
    case 1121: // BD MSM 1
        FALLTHROUGH
    case 1122: // BD MSM 2
        FALLTHROUGH
    case 1123: // BD MSM 3
        FALLTHROUGH
    case 1124: // BD MSM 4
        FALLTHROUGH
    case 1125: // BD MSM 5
        FALLTHROUGH
    case 1126: // BD MSM 6
        FALLTHROUGH
    case 1127: // BD MSM 7
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"gnssid\":%u,\"subtype\":\"MSM%d\","
                    "\"tow\":%lld,\"sync\":\"%u\",\"IODS\":%u,"
                    "\"steering\":%u,\"extclk\":%u,"
                    "\"smoothing\":%u,\"interval\":%u,"
                    "\"MaskSat\":%llu,\"MaskSig\":%u,\"MaskCell\":%llu,"
                    "\"NSat\":%u,\"NSig\":%u,\"NCell\":%u",
                    rtcm->rtcmtypes.rtcm3_msm.station_id,
                    // FIXME: make gnssid a string?
                    rtcm->rtcmtypes.rtcm3_msm.gnssid,
                    rtcm->rtcmtypes.rtcm3_msm.msm,
                    (long long)rtcm->rtcmtypes.rtcm3_msm.tow,
                    rtcm->rtcmtypes.rtcm3_msm.sync,
                    rtcm->rtcmtypes.rtcm3_msm.IODS,
                    rtcm->rtcmtypes.rtcm3_msm.steering,
                    rtcm->rtcmtypes.rtcm3_msm.ext_clk,
                    rtcm->rtcmtypes.rtcm3_msm.smoothing,
                    rtcm->rtcmtypes.rtcm3_msm.interval,
                    (unsigned long long)rtcm->rtcmtypes.rtcm3_msm.sat_mask,
                    rtcm->rtcmtypes.rtcm3_msm.sig_mask,
                    (unsigned long long)rtcm->rtcmtypes.rtcm3_msm.cell_mask,
                    rtcm->rtcmtypes.rtcm3_msm.n_sat,
                    rtcm->rtcmtypes.rtcm3_msm.n_sig,
                    rtcm->rtcmtypes.rtcm3_msm.n_cell);
        break;

    case 1230:
        // bias_indicator is undocumented...
        str_appendf(buf, buflen,
                    "\"station_id\":%u,\"ind\":\"%u\"",
                    rtcm->rtcmtypes.rtcm3_1230.station_id,
                    rtcm->rtcmtypes.rtcm3_1230.bias_indicator);
        // actual mask order is undocumented...
        if (1 & rtcm->rtcmtypes.rtcm3_1230.signals_mask) {
            str_appendf(buf, buflen, ",\"l1_ca\":%d",
                        rtcm->rtcmtypes.rtcm3_1230.l1_ca_bias);
        }
        if (2 & rtcm->rtcmtypes.rtcm3_1230.signals_mask) {
            str_appendf(buf, buflen, ",\"l1_p\":%d",
                        rtcm->rtcmtypes.rtcm3_1230.l1_p_bias);
        }
        if (4 & rtcm->rtcmtypes.rtcm3_1230.signals_mask) {
            str_appendf(buf, buflen, ",\"l2_ca\":%d",
                        rtcm->rtcmtypes.rtcm3_1230.l2_ca_bias);
        }
        if (8 & rtcm->rtcmtypes.rtcm3_1230.signals_mask) {
            str_appendf(buf, buflen, ",\"l2_p\":%d",
                        rtcm->rtcmtypes.rtcm3_1230.l2_p_bias);
        }
        break;
    case 4976:
        // IGS proprietary, SSR
        // TODO: this is just the header.
        str_appendf(buf, buflen,
                    "\"vers\":%u,\"num\":%u,\"epoch\":%u,\"update\":%u,"
                    "\"mmi\": %u,\"iod\": %u,\"provider\":%u,\"solution\": %u",
                    rtcm->rtcmtypes.rtcm3_4076.ssr_vers,
                    rtcm->rtcmtypes.rtcm3_4076.igs_num,
                    rtcm->rtcmtypes.rtcm3_4076.ssr_epoch,
                    rtcm->rtcmtypes.rtcm3_4076.ssr_update,
                    rtcm->rtcmtypes.rtcm3_4076.ssr_mmi,
                    rtcm->rtcmtypes.rtcm3_4076.ssr_iod,
                    rtcm->rtcmtypes.rtcm3_4076.ssr_provider,
                    rtcm->rtcmtypes.rtcm3_4076.ssr_solution);
        break;

    case 1018: // Reserved, alternative Ionospheric Correction Differences
        FALLTHROUGH
    case 1019: // GPS Ephemeris
        FALLTHROUGH
    case 1020: // GLO Ephemeris
        FALLTHROUGH

    default:
        (void)strlcat(buf, "\"data\":[", buflen);
        for (n = 0; n < rtcm->length; n++) {
            str_appendf(buf, buflen,
                        "\"0x%02x\",", (unsigned int)rtcm->rtcmtypes.data[n]);
        }
        str_rstrip_char(buf, ',');
        (void)strlcat(buf, "]", buflen);
        break;
    }

    str_rstrip_char(buf, ',');
    (void)strlcat(buf, "}\r\n", buflen);
#undef CODE
#undef INT
}

#if defined(AIVDM_ENABLE)
/* json_aivdm_dump() - output AIS messages as JSON
 *
 * AIS defined in ITU-R M1371-5 (2014)
 *
 * https://www.navcen.uscg.gov/ais-class-a-reports
 * https://www.navcen.uscg.gov/ais-class-b-reports
 *
 * return: void
 */
void json_aivdm_dump(const struct ais_t *ais,
                     const char *device, bool scaled,
                     char *buf, size_t buflen)
{
    char buf1[JSON_VAL_MAX * 2 + 1];
    char buf2[JSON_VAL_MAX * 2 + 1];
    char buf3[JSON_VAL_MAX * 2 + 1];
    char scratchbuf[MAX_PACKET_LENGTH * 2 + 1];
    int i;

    // "Navigation Status" in Type 1/2/3
    static char *nav_legends[] = {
        "Under way using engine",
        "At anchor",
        "Not under command",
        "Restricted maneuverability",
        "Constrained by her draught",
        "Moored",
        "Aground",
        "Engaged in fishing",
        "Under way sailing",
        "Reserved for HSC",
        "Reserved for WIG",
        "Power-driven vessel towing astern",
        "Power-driven vessel pushing ahead or towing alongside",
        "Reserved",
        "AIS-SART is active",
        "Not defined",
    };

    static char *epfd_legends[] = {
        "Undefined",
        "GPS",
        "GLONASS",
        "Combined GPS/GLONASS",
        "Loran-C",
        "Chayka",
        "Integrated navigation system",
        "Surveyed",
        "Galileo",
        "Reserved (9)",
        "Reserved (10)",
        "Reserved (11)",
        "Reserved (12)",
        "Reserved (13)",
        "Reserved (14)",
        "Internal GNSS",
    };

#define EPFD_DISPLAY(n) (((n) < (unsigned int)NITEMS(epfd_legends)) ? \
                         epfd_legends[n] : "INVALID EPFD")

    static char *ship_type_legends[100] = {
        "Not available",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Wing in ground (WIG) - all ships of this type",
        "Wing in ground (WIG) - Hazardous category A",
        "Wing in ground (WIG) - Hazardous category B",
        "Wing in ground (WIG) - Hazardous category C",
        "Wing in ground (WIG) - Hazardous category D",
        "Wing in ground (WIG) - Reserved for future use",
        "Wing in ground (WIG) - Reserved for future use",
        "Wing in ground (WIG) - Reserved for future use",
        "Wing in ground (WIG) - Reserved for future use",
        "Wing in ground (WIG) - Reserved for future use",
        "Fishing",
        "Towing",
        "Towing: length exceeds 200m or breadth exceeds 25m",
        "Dredging or underwater ops",
        "Diving ops",
        "Military ops",
        "Sailing",
        "Pleasure Craft",
        "Reserved",
        "Reserved",
        "High speed craft (HSC) - all ships of this type",
        "High speed craft (HSC) - Hazardous category A",
        "High speed craft (HSC) - Hazardous category B",
        "High speed craft (HSC) - Hazardous category C",
        "High speed craft (HSC) - Hazardous category D",
        "High speed craft (HSC) - Reserved for future use",
        "High speed craft (HSC) - Reserved for future use",
        "High speed craft (HSC) - Reserved for future use",
        "High speed craft (HSC) - Reserved for future use",
        "High speed craft (HSC) - No additional information",
        "Pilot Vessel",
        "Search and Rescue vessel",
        "Tug",
        "Port Tender",
        "Anti-pollution equipment",
        "Law Enforcement",
        "Spare - Local Vessel",
        "Spare - Local Vessel",
        "Medical Transport",
        "Ship according to RR Resolution No. 18",
        "Passenger - all ships of this type",
        "Passenger - Hazardous category A",
        "Passenger - Hazardous category B",
        "Passenger - Hazardous category C",
        "Passenger - Hazardous category D",
        "Passenger - Reserved for future use",
        "Passenger - Reserved for future use",
        "Passenger - Reserved for future use",
        "Passenger - Reserved for future use",
        "Passenger - No additional information",
        "Cargo - all ships of this type",
        "Cargo - Hazardous category A",
        "Cargo - Hazardous category B",
        "Cargo - Hazardous category C",
        "Cargo - Hazardous category D",
        "Cargo - Reserved for future use",
        "Cargo - Reserved for future use",
        "Cargo - Reserved for future use",
        "Cargo - Reserved for future use",
        "Cargo - No additional information",
        "Tanker - all ships of this type",
        "Tanker - Hazardous category A",
        "Tanker - Hazardous category B",
        "Tanker - Hazardous category C",
        "Tanker - Hazardous category D",
        "Tanker - Reserved for future use",
        "Tanker - Reserved for future use",
        "Tanker - Reserved for future use",
        "Tanker - Reserved for future use",
        "Tanker - No additional information",
        // 90
        "Other Type - all ships of this type",
        "Other Type - Hazardous category A",
        "Other Type - Hazardous category B",
        "Other Type - Hazardous category C",
        "Other Type - Hazardous category D",
        "Other Type - Reserved for future use",
        "Other Type - Reserved for future use",
        "Other Type - Reserved for future use",
        "Other Type - Reserved for future use",
        "Other Type - no additional information",
    };

#define SHIPTYPE_DISPLAY(n) (((n) < \
                             (unsigned int)NITEMS(ship_type_legends)) ? \
                             ship_type_legends[n] : "INVALID SHIP TYPE")

    static const char *station_type_legends[] = {
        "All types of mobiles",
        "Reserved for future use",
        "All types of Class B mobile stations",
        "SAR airborne mobile station",
        "Aid to Navigation station",
        "Class B shipborne mobile station",
        "Regional use and inland waterways",
        "Regional use and inland waterways",
        "Regional use and inland waterways",
        "Regional use and inland waterways",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
        "Reserved for future use",
    };

#define STATIONTYPE_DISPLAY(n) (((n) < (unsigned)NITEMS(station_type_legends)) ? \
                                station_type_legends[n] : "INVALID STATION TYPE")

    static const char *navaid_type_legends[] = {
        "Unspecified",
        "Reference point",
        "RACON",
        "Fixed offshore structure",
        "Spare, Reserved for future use.",
        "Light, without sectors",
        "Light, with sectors",
        "Leading Light Front",
        "Leading Light Rear",
        "Beacon, Cardinal N",
        "Beacon, Cardinal E",
        "Beacon, Cardinal S",
        "Beacon, Cardinal W",
        "Beacon, Port hand",
        "Beacon, Starboard hand",
        "Beacon, Preferred Channel port hand",
        "Beacon, Preferred Channel starboard hand",
        "Beacon, Isolated danger",
        "Beacon, Safe water",
        "Beacon, Special mark",
        "Cardinal Mark N",
        "Cardinal Mark E",
        "Cardinal Mark S",
        "Cardinal Mark W",
        "Port hand Mark",
        "Starboard hand Mark",
        "Preferred Channel Port hand",
        "Preferred Channel Starboard hand",
        "Isolated danger",
        "Safe Water",
        "Special Mark",
        "Light Vessel / LANBY / Rigs",
    };

#define NAVAIDTYPE_DISPLAY(n) (((n) < (unsigned)NITEMS(navaid_type_legends)) ? \
                               navaid_type_legends[n] : "INVALID NAVAID TYPE")

    // cppcheck-suppress variableScope
    static const char *signal_legends[] = {
        "N/A",
        "Serious emergency - stop or divert according to instructions.",
        "Vessels shall not proceed.",
        "Vessels may proceed. One way traffic.",
        "Vessels may proceed. Two way traffic.",
        "Vessels shall proceed on specific orders only.",
        "Vessels in main channel shall not proceed.",
        "Vessels in main channel shall proceed on specific orders only.",
        "Vessels in main channel shall proceed on specific orders only.",
        "I = \"in-bound\" only acceptable.",
        "O = \"out-bound\" only acceptable.",
        "F = both \"in- and out-bound\" acceptable.",
        "XI = Code will shift to \"I\" in due time.",
        "XO = Code will shift to \"O\" in due time.",
        "X = Vessels shall proceed only on direction.",
    };

#define SIGNAL_DISPLAY(n) (((n) < (unsigned)NITEMS(signal_legends)) ? \
                           signal_legends[n] : "INVALID SIGNAL TYPE")

    static const char *route_type[32] = {
        "Undefined (default)",
        "Mandatory",
        "Recommended",
        "Alternative",
        "Recommended route through ice",
        "Ship route plan",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Reserved for future use.",
        "Cancel route identified by message linkage",
    };

    // cppcheck-suppress variableScope
    static const char *idtypes[] = {
        "mmsi",
        "imo",
        "callsign",
        "other",
    };

    // cppcheck-suppress variableScope
    static const char *racon_status[] = {
        "No RACON installed",
        "RACON not monitored",
        "RACON operational",
        "RACON ERROR"
    };

    // cppcheck-suppress variableScope
    static const char *light_status[] = {
        "No light or no monitoring",
        "Light ON",
        "Light OFF",
        "Light ERROR"
    };

    // cppcheck-suppress variableScope
    static const char *rta_status[] = {
        "Operational",
        "Limited operation",
        "Out of order",
        "N/A",
    };

    // cppcheck-suppress variableScope
    const char *position_types[8] = {
        "Not available",
        "Port-side to",
        "Starboard-side to",
        "Mediterranean (end-on) mooring",
        "Mooring buoy",
        "Anchorage",
        "Reserved for future use",
        "Reserved for future use",
    };

    (void)snprintf(buf, buflen, "{\"class\":\"AIS\"");
    if (NULL != device &&
        '\0' != device[0]) {
        str_appendf(buf, buflen, ",\"device\":\"%s\"", device);
    }
    str_appendf(buf, buflen,
                ",\"type\":%u,\"repeat\":%u,\"mmsi\":%u,\"scaled\":%s",
                ais->type, ais->repeat, ais->mmsi, JSON_BOOL(scaled));
    switch (ais->type) {
    case 1:                     // Position Report
        FALLTHROUGH
    case 2:
        FALLTHROUGH
    case 3:
        if (scaled) {
            char turnlegend[20];
            char speedlegend[20];

            /*
             * Express turn as "n/a" if not available,
             * "fastleft"/"fastright" for fast turns.
             */
            if (AIS_TURN_NOT_AVAILABLE <= abs(ais->type1.turn)) {
                (void)strlcpy(turnlegend, "\"n/a\"", sizeof(turnlegend));
            } else if (AIS_TURN_HARD_LEFT == ais->type1.turn) {
                (void)strlcpy(turnlegend, "\"fastleft\"", sizeof(turnlegend));
            } else if (AIS_TURN_HARD_RIGHT == ais->type1.turn) {
                (void)strlcpy(turnlegend, "\"fastright\"", sizeof(turnlegend));
            } else {
                // range -708° to 708°
                double rot = ais->type1.turn / 4.733;

                rot *= rot;
                if (0 > ais->type1.turn) {
                    rot = -rot;
                }
                (void)snprintf(turnlegend, sizeof(turnlegend),
                    "\"%.2f\"", rot);
            }

            /*
             * Express speed as "n/a" if not available,
             * "fast" for fast movers.
             */
            if (AIS_SPEED_NOT_AVAILABLE == ais->type1.speed) {
                (void)strlcpy(speedlegend, "\"n/a\"", sizeof(speedlegend));
            } else if (AIS_SPEED_FAST_MOVER == ais->type1.speed) {
                (void)strlcpy(speedlegend, "\"fast\"", sizeof(speedlegend));
            } else {
                (void)snprintf(speedlegend, sizeof(speedlegend),
                               "%.1f", ais->type1.speed / 10.0);
            }

            str_appendf(buf, buflen,
                        ",\"status\":%u,\"status_text\":\"%s\","
                        "\"turn\":%s,\"speed\":%s,"
                        "\"accuracy\":%s,\"lon\":%.7f,\"lat\":%.7f,"
                        "\"course\":%.1f,\"heading\":%u,\"second\":%u,"
                        "\"maneuver\":%u,\"raim\":%s,\"radio\":%u}\r\n",
                        ais->type1.status,
                        nav_legends[ais->type1.status],
                        turnlegend,
                        speedlegend,
                        JSON_BOOL(ais->type1.accuracy),
                        ais->type1.lon / AIS_LATLON_DIV,
                        ais->type1.lat / AIS_LATLON_DIV,
                        ais->type1.course / 10.0,
                        ais->type1.heading,
                        ais->type1.second,
                        ais->type1.maneuver,
                        JSON_BOOL(ais->type1.raim), ais->type1.radio);
        } else {
            str_appendf(buf, buflen,
                        ",\"status\":%u,\"status_text\":\"%s\","
                        "\"turn\":%d,\"speed\":%u,"
                        "\"accuracy\":%s,\"lon\":%d,\"lat\":%d,"
                        "\"course\":%u,\"heading\":%u,\"second\":%u,"
                        "\"maneuver\":%u,\"raim\":%s,\"radio\":%u}\r\n",
                        ais->type1.status,
                        nav_legends[ais->type1.status],
                        ais->type1.turn,
                        ais->type1.speed,
                        JSON_BOOL(ais->type1.accuracy),
                        ais->type1.lon,
                        ais->type1.lat,
                        ais->type1.course,
                        ais->type1.heading,
                        ais->type1.second,
                        ais->type1.maneuver,
                        JSON_BOOL(ais->type1.raim), ais->type1.radio);
        }
        break;
    case 4:                     // Base Station Report
        FALLTHROUGH
    case 11:                    // UTC/Date Response
        // some fields have beem merged to an ISO8601 date
        if (scaled) {
            // The use of %u instead of %04u for the year is to allow
            // out-of-band year values.
            str_appendf(buf, buflen,
                        ",\"timestamp\":\"%04u-%02u-%02uT%02u:%02u:%02uZ\","
                        "\"accuracy\":%s,\"lon\":%.7f,\"lat\":%.7f,"
                        "\"epfd\":%u,\"epfd_text\":\"%s\","
                        "\"raim\":%s,\"radio\":%u}\r\n",
                        ais->type4.year,
                        ais->type4.month,
                        ais->type4.day,
                        ais->type4.hour,
                        ais->type4.minute,
                        ais->type4.second,
                        JSON_BOOL(ais->type4.accuracy),
                        ais->type4.lon / AIS_LATLON_DIV,
                        ais->type4.lat / AIS_LATLON_DIV,
                        ais->type4.epfd,
                        EPFD_DISPLAY(ais->type4.epfd),
                        JSON_BOOL(ais->type4.raim), ais->type4.radio);
        } else {
            str_appendf(buf, buflen,
                        ",\"timestamp\":\"%04u-%02u-%02uT%02u:%02u:%02uZ\","
                        "\"accuracy\":%s,\"lon\":%d,\"lat\":%d,"
                        "\"epfd\":%u,\"epfd_text\":\"%s\","
                        "\"raim\":%s,\"radio\":%u}\r\n",
                        ais->type4.year,
                        ais->type4.month,
                        ais->type4.day,
                        ais->type4.hour,
                        ais->type4.minute,
                        ais->type4.second,
                        JSON_BOOL(ais->type4.accuracy),
                        ais->type4.lon,
                        ais->type4.lat,
                        ais->type4.epfd,
                        EPFD_DISPLAY(ais->type4.epfd),
                        JSON_BOOL(ais->type4.raim), ais->type4.radio);
        }
        break;
    case 5:                     // Ship static and voyage related data
        // some fields have beem merged to an ISO8601 partial date
        if (scaled) {
            str_appendf(buf, buflen,
                        ",\"imo\":%u,\"ais_version\":%u,\"callsign\":\"%s\","
                        "\"shipname\":\"%s\","
                        "\"shiptype\":%u,\"shiptype_text\":\"%s\","
                        "\"to_bow\":%u,\"to_stern\":%u,\"to_port\":%u,"
                        "\"to_starboard\":%u,"
                        "\"epfd\":%u,\"epfd_text\":\"%s\","
                        "\"eta\":\"%02u-%02uT%02u:%02uZ\","
                        "\"draught\":%.1f,\"destination\":\"%s\","
                        "\"dte\":%u}\r\n",
                        ais->type5.imo,
                        ais->type5.ais_version,
                        json_stringify(buf1, sizeof(buf1),
                                       ais->type5.callsign),
                        json_stringify(buf2, sizeof(buf2),
                                       ais->type5.shipname),
                        ais->type5.shiptype,
                        SHIPTYPE_DISPLAY(ais->type5.shiptype),
                        ais->type5.to_bow, ais->type5.to_stern,
                        ais->type5.to_port, ais->type5.to_starboard,
                        ais->type5.epfd,
                        EPFD_DISPLAY(ais->type5.epfd),
                        ais->type5.month,
                        ais->type5.day,
                        ais->type5.hour, ais->type5.minute,
                        ais->type5.draught / 10.0,
                        json_stringify(buf3, sizeof(buf3),
                                       ais->type5.destination),
                        ais->type5.dte);
        } else {
            str_appendf(buf, buflen,
                        ",\"imo\":%u,\"ais_version\":%u,\"callsign\":\"%s\","
                        "\"shipname\":\"%s\","
                        "\"shiptype\":%u,\"shiptype_text\":\"%s\","
                        "\"to_bow\":%u,\"to_stern\":%u,\"to_port\":%u,"
                        "\"to_starboard\":%u,"
                        "\"epfd\":%u,\"epfd_text\":\"%s\","
                        "\"eta\":\"%02u-%02uT%02u:%02uZ\","
                        "\"draught\":%u,\"destination\":\"%s\","
                        "\"dte\":%u}\r\n",
                        ais->type5.imo,
                        ais->type5.ais_version,
                        json_stringify(buf1, sizeof(buf1),
                                       ais->type5.callsign),
                        json_stringify(buf2, sizeof(buf2),
                                       ais->type5.shipname),
                        ais->type5.shiptype,
                        SHIPTYPE_DISPLAY(ais->type5.shiptype),
                        ais->type5.to_bow,
                        ais->type5.to_stern,
                        ais->type5.to_port,
                        ais->type5.to_starboard,
                        ais->type5.epfd,
                        EPFD_DISPLAY(ais->type5.epfd),
                        ais->type5.month,
                        ais->type5.day,
                        ais->type5.hour,
                        ais->type5.minute,
                        ais->type5.draught,
                        json_stringify(buf3, sizeof(buf3),
                                       ais->type5.destination),
                        ais->type5.dte);
        }
        break;
    case 6:                     // Binary Message
        str_appendf(buf, buflen,
                    ",\"seqno\":%u,\"dest_mmsi\":%u,"
                    "\"retransmit\":%s,\"dac\":%u,\"fid\":%u",
                    ais->type6.seqno,
                    ais->type6.dest_mmsi,
                    JSON_BOOL(ais->type6.retransmit),
                    ais->type6.dac,
                    ais->type6.fid);
        if (!ais->type6.structured) {
            str_appendf(buf, buflen,
                        ",\"data\":\"%zd:%s\"}\r\n",
                        ais->type6.bitcount,
                        json_stringify(
                            buf1, sizeof(buf1),
                            gps_hexdump(scratchbuf,
                                     sizeof(scratchbuf),
                                     (const unsigned char *)ais->type6.bitdata,
                                     BITS_TO_BYTES(ais->type6.bitcount))));
            break;
        }
        if (200 == ais->type6.dac) {
            switch (ais->type6.fid) {
            case 21:
                str_appendf(buf, buflen,
                           ",\"country\":\"%s\",\"locode\":\"%s\","
                           "\"section\":\"%s\",\"terminal\":\"%s\","
                           "\"hectometre\":\"%s\",\"eta\":\"%u-%uT%u:%u\","
                           "\"tugs\":%u,\"airdraught\":%u}\r\n",
                           ais->type6.dac200fid21.country,
                           ais->type6.dac200fid21.locode,
                           ais->type6.dac200fid21.section,
                           ais->type6.dac200fid21.terminal,
                           ais->type6.dac200fid21.hectometre,
                           ais->type6.dac200fid21.month,
                           ais->type6.dac200fid21.day,
                           ais->type6.dac200fid21.hour,
                           ais->type6.dac200fid21.minute,
                           ais->type6.dac200fid21.tugs,
                           ais->type6.dac200fid21.airdraught);
                break;
            case 22:
                str_appendf(buf, buflen,
                            ",\"country\":\"%s\",\"locode\":\"%s\","
                            "\"section\":\"%s\","
                            "\"terminal\":\"%s\",\"hectometre\":\"%s\","
                            "\"eta\":\"%u-%uT%u:%u\","
                            "\"status\":%u,\"status_text\":\"%s\"}\r\n",
                            ais->type6.dac200fid22.country,
                            ais->type6.dac200fid22.locode,
                            ais->type6.dac200fid22.section,
                            ais->type6.dac200fid22.terminal,
                            ais->type6.dac200fid22.hectometre,
                            ais->type6.dac200fid22.month,
                            ais->type6.dac200fid22.day,
                            ais->type6.dac200fid22.hour,
                            ais->type6.dac200fid22.minute,
                            ais->type6.dac200fid22.status,
                            rta_status[ais->type6.dac200fid22.status]);
                break;
            case 55:
                str_appendf(buf, buflen,
                            ",\"crew\":%u,\"passengers\":%u,\"personnel\":"
                            "%u}\r\n",
                            ais->type6.dac200fid55.crew,
                            ais->type6.dac200fid55.passengers,
                            ais->type6.dac200fid55.personnel);
                break;
            }
        } else if (235 == ais->type6.dac ||
                   250 == ais->type6.dac) {
            switch (ais->type6.fid) {
            case 10:    // GLA - AtoN monitoring data
                str_appendf(buf, buflen,
                            ",\"off_pos\":%s,\"alarm\":%s,"
                            "\"stat_ext\":%u",
                            JSON_BOOL(ais->type6.dac235fid10.off_pos),
                            JSON_BOOL(ais->type6.dac235fid10.alarm),
                            ais->type6.dac235fid10.stat_ext);
                if (scaled &&
                    0 != ais->type6.dac235fid10.ana_int) {
                    str_appendf(buf, buflen,
                                ",\"ana_int\":%.2f",
                                ais->type6.dac235fid10.ana_int*0.05);
                } else {
                    str_appendf(buf, buflen,
                                ",\"ana_int\":%u",
                                ais->type6.dac235fid10.ana_int);
                }
                if (scaled &&
                    0 != ais->type6.dac235fid10.ana_ext1) {
                    str_appendf(buf, buflen,
                                ",\"ana_ext1\":%.2f",
                                ais->type6.dac235fid10.ana_ext1*0.05);
                } else {
                    str_appendf(buf, buflen,
                                ",\"ana_ext1\":%u",
                                ais->type6.dac235fid10.ana_ext1);
                }
                if (scaled &&
                    0 != ais->type6.dac235fid10.ana_ext2) {
                    str_appendf(buf, buflen,
                                ",\"ana_ext2\":%.2f",
                                ais->type6.dac235fid10.ana_ext2*0.05);
                } else {
                    str_appendf(buf, buflen,
                                ",\"ana_ext2\":%u",
                                ais->type6.dac235fid10.ana_ext2);
                }
                str_appendf(buf, buflen,
                            ",\"racon\":%u,"
                            "\"racon_text\":\"%s\","
                            "\"light\":%u,"
                            "\"light_text\":\"%s\"",
                            ais->type6.dac235fid10.racon,
                            racon_status[ais->type6.dac235fid10.racon],
                            ais->type6.dac235fid10.light,
                            light_status[ais->type6.dac235fid10.light]);
                (void)strlcat(buf, "}\r\n", buflen);
                break;
            }
        } else if (1 == ais->type6.dac) {
            char buf4[JSON_VAL_MAX * 2 + 1];

            switch (ais->type6.fid) {
            case 12:    // IMO236 -Dangerous cargo indication
                // some fields have beem merged to an ISO8601 partial date
                str_appendf(buf, buflen,
                            ",\"lastport\":\"%s\","
                            "\"departure\":\"%02u-%02uT%02u:%02uZ\","
                            "\"nextport\":\"%s\","
                            "\"eta\":\"%02u-%02uT%02u:%02uZ\","
                            "\"dangerous\":\"%s\",\"imdcat\":\"%s\","
                            "\"unid\":%u,\"amount\":%u,\"unit\":%u}\r\n",
                            json_stringify(buf1, sizeof(buf1),
                                           ais->type6.dac1fid12.lastport),
                            ais->type6.dac1fid12.lmonth,
                            ais->type6.dac1fid12.lday,
                            ais->type6.dac1fid12.lhour,
                            ais->type6.dac1fid12.lminute,
                            json_stringify(buf2, sizeof(buf2),
                                           ais->type6.dac1fid12.nextport),
                            ais->type6.dac1fid12.nmonth,
                            ais->type6.dac1fid12.nday,
                            ais->type6.dac1fid12.nhour,
                            ais->type6.dac1fid12.nminute,
                            json_stringify(buf3, sizeof(buf3),
                                           ais->type6.dac1fid12.dangerous),
                            json_stringify(buf4, sizeof(buf4),
                                           ais->type6.dac1fid12.imdcat),
                            ais->type6.dac1fid12.unid,
                            ais->type6.dac1fid12.amount,
                            ais->type6.dac1fid12.unit);
                break;
            case 15: // IMO236 - Extended Ship Static and Voyage Related Data
                str_appendf(buf, buflen,
                            ",\"airdraught\":%u}\r\n",
                            ais->type6.dac1fid15.airdraught);
                break;
            case 16:    // IMO236 - Number of persons on board
                str_appendf(buf, buflen,
                            ",\"persons\":%u}\r\n",
                            ais->type6.dac1fid16.persons);
                break;
            case 18:    // IMO289 - Clearance time to enter port
                str_appendf(buf, buflen,
                            ",\"linkage\":%u,"
                            "\"arrival\":\"%02u-%02uT%02u:%02uZ\","
                            "\"portname\":\"%s\",\"destination\":\"%s\"",
                            ais->type6.dac1fid18.linkage,
                            ais->type6.dac1fid18.month,
                            ais->type6.dac1fid18.day,
                            ais->type6.dac1fid18.hour,
                            ais->type6.dac1fid18.minute,
                            json_stringify(buf1, sizeof(buf1),
                                           ais->type6.dac1fid18.portname),
                            json_stringify(buf2, sizeof(buf2),
                                           ais->type6.dac1fid18.destination));
                if (scaled) {
                    str_appendf(buf, buflen,
                                ",\"lon\":%.6f,\"lat\":%.6f}\r\n",
                                ais->type6.dac1fid18.lon / AIS_LATLON3_DIV,
                                ais->type6.dac1fid18.lat / AIS_LATLON3_DIV);
                } else {
                    str_appendf(buf, buflen,
                                ",\"lon\":%d,\"lat\":%d}\r\n",
                                ais->type6.dac1fid18.lon,
                                ais->type6.dac1fid18.lat);
                }
                break;
            case 20:        // IMO289 - Berthing Data
                str_appendf(buf, buflen,
                            ",\"linkage\":%u,\"berth_length\":%u,"
                            "\"position\":%u,\"position_text\":\"%s\","
                            "\"arrival\":\"%u-%uT%u:%u\","
                            "\"availability\":%u,"
                            "\"agent\":%u,\"fuel\":%u,\"chandler\":%u,"
                            "\"stevedore\":%u,\"electrical\":%u,"
                            "\"water\":%u,\"customs\":%u,\"cartage\":%u,"
                            "\"crane\":%u,\"lift\":%u,\"medical\":%u,"
                            "\"navrepair\":%u,\"provisions\":%u,"
                            "\"shiprepair\":%u,\"surveyor\":%u,"
                            "\"steam\":%u,\"tugs\":%u,\"solidwaste\":%u,"
                            "\"liquidwaste\":%u,\"hazardouswaste\":%u,"
                            "\"ballast\":%u,\"additional\":%u,"
                            "\"regional1\":%u,\"regional2\":%u,"
                            "\"future1\":%u,\"future2\":%u,"
                            "\"berth_name\":\"%s\"",
                            ais->type6.dac1fid20.linkage,
                            ais->type6.dac1fid20.berth_length,
                            ais->type6.dac1fid20.position,
                            position_types[ais->type6.dac1fid20.position],
                            ais->type6.dac1fid20.month,
                            ais->type6.dac1fid20.day,
                            ais->type6.dac1fid20.hour,
                            ais->type6.dac1fid20.minute,
                            ais->type6.dac1fid20.availability,
                            ais->type6.dac1fid20.agent,
                            ais->type6.dac1fid20.fuel,
                            ais->type6.dac1fid20.chandler,
                            ais->type6.dac1fid20.stevedore,
                            ais->type6.dac1fid20.electrical,
                            ais->type6.dac1fid20.water,
                            ais->type6.dac1fid20.customs,
                            ais->type6.dac1fid20.cartage,
                            ais->type6.dac1fid20.crane,
                            ais->type6.dac1fid20.lift,
                            ais->type6.dac1fid20.medical,
                            ais->type6.dac1fid20.navrepair,
                            ais->type6.dac1fid20.provisions,
                            ais->type6.dac1fid20.shiprepair,
                            ais->type6.dac1fid20.surveyor,
                            ais->type6.dac1fid20.steam,
                            ais->type6.dac1fid20.tugs,
                            ais->type6.dac1fid20.solidwaste,
                            ais->type6.dac1fid20.liquidwaste,
                            ais->type6.dac1fid20.hazardouswaste,
                            ais->type6.dac1fid20.ballast,
                            ais->type6.dac1fid20.additional,
                            ais->type6.dac1fid20.regional1,
                            ais->type6.dac1fid20.regional2,
                            ais->type6.dac1fid20.future1,
                            ais->type6.dac1fid20.future2,
                            json_stringify(buf1, sizeof(buf1),
                                           ais->type6.dac1fid20.berth_name));
                if (scaled) {
                    str_appendf(buf, buflen,
                                ",\"berth_lon\":%.6f,"
                                "\"berth_lat\":%.6f,"
                                "\"berth_depth\":%.1f}\r\n",
                                ais->type6.dac1fid20.berth_lon /
                                    AIS_LATLON3_DIV,
                                ais->type6.dac1fid20.berth_lat /
                                    AIS_LATLON3_DIV,
                                ais->type6.dac1fid20.berth_depth * 0.1);
                } else {
                    str_appendf(buf, buflen,
                                ",\"berth_lon\":%d,"
                                "\"berth_lat\":%d,"
                                "\"berth_depth\":%u}\r\n",
                                ais->type6.dac1fid20.berth_lon,
                                ais->type6.dac1fid20.berth_lat,
                                ais->type6.dac1fid20.berth_depth);
                }
                break;
            case 23:    // IMO289 - Area notice - addressed
                break;
            case 25:    // IMO289 - Dangerous cargo indication
                str_appendf(buf, buflen,
                            ",\"unit\":%u,\"amount\":%u,\"cargos\":[",
                            ais->type6.dac1fid25.unit,
                            ais->type6.dac1fid25.amount);
                for (i = 0; i < (int)ais->type6.dac1fid25.ncargos; i++) {
                    str_appendf(buf, buflen,
                                "{\"code\":%u,\"subtype\":%u},",
                                ais->type6.dac1fid25.cargos[i].code,
                                ais->type6.dac1fid25.cargos[i].subtype);
                }
                str_rstrip_char(buf, ',');
                (void)strlcat(buf, "]}\r\n", buflen);
                break;
            case 28:    // IMO289 - Route info - addressed
                str_appendf(buf, buflen,
                            ",\"linkage\":%u,\"sender\":%u,"
                            "\"rtype\":%u,"
                            "\"rtype_text\":\"%s\","
                            "\"start\":\"%02u-%02uT%02u:%02uZ\","
                            "\"duration\":%u,\"waypoints\":[",
                            ais->type6.dac1fid28.linkage,
                            ais->type6.dac1fid28.sender,
                            ais->type6.dac1fid28.rtype,
                            route_type[ais->type6.dac1fid28.rtype],
                            ais->type6.dac1fid28.month,
                            ais->type6.dac1fid28.day,
                            ais->type6.dac1fid28.hour,
                            ais->type6.dac1fid28.minute,
                            ais->type6.dac1fid28.duration);
                for (i = 0; i < ais->type6.dac1fid28.waycount; i++) {
                    if (scaled) {
                        str_appendf(buf, buflen,
                            "{\"lon\":%.7f,\"lat\":%.7f},",
                            ais->type6.dac1fid28.waypoints[i].lon /
                                AIS_LATLON4_DIV,
                            ais->type6.dac1fid28.waypoints[i].lat /
                                AIS_LATLON4_DIV);
                    } else {
                        str_appendf(buf, buflen,
                            "{\"lon\":%d,\"lat\":%d},",
                            ais->type6.dac1fid28.waypoints[i].lon,
                            ais->type6.dac1fid28.waypoints[i].lat);
                    }
                }
                str_rstrip_char(buf, ',');
                (void)strlcat(buf, "]}\r\n", buflen);
                break;
            case 30:    // IMO289 - Text description - addressed
                str_appendf(buf, buflen,
                            ",\"linkage\":%u,\"text\":\"%s\"}\r\n",
                            ais->type6.dac1fid30.linkage,
                            json_stringify(buf1, sizeof(buf1),
                                           ais->type6.dac1fid30.text));
                break;
            case 14:    // IMO236 - Tidal Window
                FALLTHROUGH
            case 32:    // IMO289 - Tidal Window
              str_appendf(buf, buflen,
                          ",\"month\":%u,\"day\":%u,\"tidals\":[",
                          ais->type6.dac1fid32.month,
                          ais->type6.dac1fid32.day);
              for (i = 0; i < ais->type6.dac1fid32.ntidals; i++) {
                  const struct tidal_t *tp =  &ais->type6.dac1fid32.tidals[i];

                  if (scaled) {
                      str_appendf(buf, buflen,
                          "{\"lon\":%.6f,\"lat\":%.6f",
                          tp->lon / AIS_LATLON3_DIV,
                          tp->lat / AIS_LATLON3_DIV);
                  } else {
                      str_appendf(buf, buflen,
                          "{\"lon\":%d,\"lat\":%d",
                          tp->lon,
                          tp->lat);
                  }
                  str_appendf(buf, buflen,
                              ",\"from_hour\":%u,\"from_min\":%u,"
                              "\"to_hour\":%u,\"to_min\":%u,\"cdir\":%u",
                              tp->from_hour,
                              tp->from_min,
                              tp->to_hour,
                              tp->to_min,
                              tp->cdir);
                  if (scaled) {
                      str_appendf(buf, buflen, ",\"cspeed\":%.1f},",
                                  tp->cspeed / 10.0);
                  } else {
                      str_appendf(buf, buflen, ",\"cspeed\":%u},",
                                  tp->cspeed);
                  }
              }
              str_rstrip_char(buf, ',');
              (void)strlcat(buf, "]}\r\n", buflen);
              break;
            }
        }
        break;
    case 7:                     // Binary Acknowledge
        FALLTHROUGH
    case 13:                    // Safety Related Acknowledge
        str_appendf(buf, buflen,
                    ",\"mmsi1\":%u,\"mmsi2\":%u,\"mmsi3\":%u,"
                    "\"mmsi4\":%u}\r\n",
                    ais->type7.mmsi1,
                    ais->type7.mmsi2, ais->type7.mmsi3, ais->type7.mmsi4);
        break;
    case 8:                     // Binary Broadcast Message
        str_appendf(buf, buflen,
                    ",\"dac\":%u,\"fid\":%u",
                    ais->type8.dac, ais->type8.fid);
        if (!ais->type8.structured) {
            str_appendf(buf, buflen,
                        ",\"data\":\"%zd:%s\"}\r\n",
                        ais->type8.bitcount,
                        json_stringify(buf1, sizeof(buf1),
                           gps_hexdump(
                               scratchbuf, sizeof(scratchbuf),
                               (const unsigned char *)ais->type8.bitdata,
                               BITS_TO_BYTES(ais->type8.bitcount))));
            break;
        }
        if (1 == ais->type8.dac) {
            const char *trends[] = {
                "steady",
                "increasing",
                "decreasing",
                "N/A",
            };
            // WMO 306, Code table 4.201
            const char *preciptypes[] = {
                "reserved",
                "rain",
                "thunderstorm",
                "freezing rain",
                "mixed/ice",
                "snow",
                "reserved",
                "N/A",
            };
            const char *ice[] = {
                "no",
                "yes",
                "reserved",
                "N/A",
            };
            switch (ais->type8.fid) {
            case 11:        // IMO236 - Meteorological/Hydrological data
                // some fields have been merged to an ISO8601 partial date
                // layout is almost identical to FID=31 from IMO289
                if (scaled) {
                    str_appendf(buf, buflen,
                                ",\"lat\":%.6f,\"lon\":%.6f",
                                ais->type8.dac1fid11.lat / AIS_LATLON3_DIV,
                                ais->type8.dac1fid11.lon / AIS_LATLON3_DIV);
                } else {
                    str_appendf(buf, buflen,
                                ",\"lat\":%d,\"lon\":%d",
                                ais->type8.dac1fid11.lat,
                                ais->type8.dac1fid11.lon);
                }
                str_appendf(buf, buflen,
                            ",\"timestamp\":\"%02uT%02u:%02uZ\","
                            "\"wspeed\":%u,\"wgust\":%u,\"wdir\":%u,"
                            "\"wgustdir\":%u,\"humidity\":%u",
                            ais->type8.dac1fid11.day,
                            ais->type8.dac1fid11.hour,
                            ais->type8.dac1fid11.minute,
                            ais->type8.dac1fid11.wspeed,
                            ais->type8.dac1fid11.wgust,
                            ais->type8.dac1fid11.wdir,
                            ais->type8.dac1fid11.wgustdir,
                            ais->type8.dac1fid11.humidity);
                if (scaled) {
                    str_appendf(buf, buflen,
                                ",\"airtemp\":%.1f,\"dewpoint\":%.1f,"
                                "\"pressure\":%u,\"pressuretend\":\"%s\"",
                                ((signed int)ais->type8.dac1fid11.airtemp -
                                DAC1FID11_AIRTEMP_OFFSET) / DAC1FID11_AIRTEMP_DIV,
                                ((signed int)ais->type8.dac1fid11.dewpoint -
                                DAC1FID11_DEWPOINT_OFFSET) /
                                DAC1FID11_DEWPOINT_DIV,
                                ais->type8.dac1fid11.pressure -
                                DAC1FID11_PRESSURE_OFFSET,
                                trends[ais->type8.dac1fid11.pressuretend]);
                    str_appendf(buf, buflen,
                                ",\"visibility\":%.1f",
                                ais->type8.dac1fid11.visibility /
                                DAC1FID11_VISIBILITY_DIV);
                    str_appendf(buf, buflen,
                                ",\"waterlevel\":%.1f",
                                ((signed int)ais->type8.dac1fid11.waterlevel -
                                DAC1FID11_WATERLEVEL_OFFSET) /
                                DAC1FID11_WATERLEVEL_DIV);
                    str_appendf(buf, buflen,
                                ",\"leveltrend\":\"%s\","
                                "\"cspeed\":%.1f,\"cdir\":%u,"
                                "\"cspeed2\":%.1f,\"cdir2\":%u,"
                                "\"cdepth2\":%u,"
                                "\"cspeed3\":%.1f,\"cdir3\":%u,"
                                "\"cdepth3\":%u,"
                                "\"waveheight\":%.1f,\"waveperiod\":%u,"
                                "\"wavedir\":%u,"
                                "\"swellheight\":%.1f,\"swellperiod\":%u,"
                                "\"swelldir\":%u,"
                                "\"seastate\":%u,\"watertemp\":%.1f,"
                                "\"preciptype\":%u,"
                                "\"preciptype_text\":\"%s\","
                                "\"salinity\":%.1f,\"ice\":%u,"
                                "\"ice_text\":\"%s\"",
                                trends[ais->type8.dac1fid11.leveltrend],
                                ais->type8.dac1fid11.cspeed /
                                DAC1FID11_CSPEED_DIV,
                                ais->type8.dac1fid11.cdir,
                                ais->type8.dac1fid11.cspeed2 /
                                DAC1FID11_CSPEED_DIV,
                                ais->type8.dac1fid11.cdir2,
                                ais->type8.dac1fid11.cdepth2,
                                ais->type8.dac1fid11.cspeed3 /
                                DAC1FID11_CSPEED_DIV,
                                ais->type8.dac1fid11.cdir3,
                                ais->type8.dac1fid11.cdepth3,
                                ais->type8.dac1fid11.waveheight /
                                DAC1FID11_WAVEHEIGHT_DIV,
                                ais->type8.dac1fid11.waveperiod,
                                ais->type8.dac1fid11.wavedir,
                                ais->type8.dac1fid11.swellheight /
                                DAC1FID11_WAVEHEIGHT_DIV,
                                ais->type8.dac1fid11.swellperiod,
                                ais->type8.dac1fid11.swelldir,
                                ais->type8.dac1fid11.seastate,
                                ((signed int)ais->type8.dac1fid11.watertemp -
                                DAC1FID11_WATERTEMP_OFFSET) /
                                DAC1FID11_WATERTEMP_DIV,
                                ais->type8.dac1fid11.preciptype,
                                preciptypes[ais->type8.dac1fid11.preciptype],
                                ais->type8.dac1fid11.salinity /
                                DAC1FID11_SALINITY_DIV,
                                ais->type8.dac1fid11.ice,
                                ice[ais->type8.dac1fid11.ice]);
                } else {
                    str_appendf(buf, buflen,
                                ",\"airtemp\":%u,\"dewpoint\":%u,"
                                "\"pressure\":%u,\"pressuretend\":%u",
                                ais->type8.dac1fid11.airtemp,
                                ais->type8.dac1fid11.dewpoint,
                                ais->type8.dac1fid11.pressure,
                                ais->type8.dac1fid11.pressuretend);
                    str_appendf(buf, buflen,
                                   ",\"visibility\":%u",
                                   ais->type8.dac1fid11.visibility);
                    str_appendf(buf, buflen,
                                   ",\"waterlevel\":%d",
                                   ais->type8.dac1fid11.waterlevel);
                    str_appendf(buf, buflen,
                                ",\"leveltrend\":%u,"
                                "\"cspeed\":%u,\"cdir\":%u,"
                                "\"cspeed2\":%u,\"cdir2\":%u,"
                                "\"cdepth2\":%u,"
                                "\"cspeed3\":%u,\"cdir3\":%u,"
                                "\"cdepth3\":%u,"
                                "\"waveheight\":%u,\"waveperiod\":%u,"
                                "\"wavedir\":%u,"
                                "\"swellheight\":%u,\"swellperiod\":%u,"
                                "\"swelldir\":%u,"
                                "\"seastate\":%u,\"watertemp\":%u,"
                                "\"preciptype\":%u,"
                                "\"preciptype_text\":\"%s\","
                                "\"salinity\":%u,\"ice\":%u,"
                                "\"ice_text\":\"%s\"",
                                ais->type8.dac1fid11.leveltrend,
                                ais->type8.dac1fid11.cspeed,
                                ais->type8.dac1fid11.cdir,
                                ais->type8.dac1fid11.cspeed2,
                                ais->type8.dac1fid11.cdir2,
                                ais->type8.dac1fid11.cdepth2,
                                ais->type8.dac1fid11.cspeed3,
                                ais->type8.dac1fid11.cdir3,
                                ais->type8.dac1fid11.cdepth3,
                                ais->type8.dac1fid11.waveheight,
                                ais->type8.dac1fid11.waveperiod,
                                ais->type8.dac1fid11.wavedir,
                                ais->type8.dac1fid11.swellheight,
                                ais->type8.dac1fid11.swellperiod,
                                ais->type8.dac1fid11.swelldir,
                                ais->type8.dac1fid11.seastate,
                                ais->type8.dac1fid11.watertemp,
                                ais->type8.dac1fid11.preciptype,
                                preciptypes[ais->type8.dac1fid11.preciptype],
                                ais->type8.dac1fid11.salinity,
                                ais->type8.dac1fid11.ice,
                                ice[ais->type8.dac1fid11.ice]);
                }
                (void)strlcat(buf, "}\r\n", buflen);
                break;
            case 13:        // IMO236 - Fairway closed
                str_appendf(buf, buflen,
                            ",\"reason\":\"%s\",\"closefrom\":\"%s\","
                            "\"closeto\":\"%s\",\"radius\":%u,"
                            "\"extunit\":%u,"
                            "\"from\":\"%02u-%02uT%02u:%02u\","
                            "\"to\":\"%02u-%02uT%02u:%02u\"}\r\n",
                            json_stringify(buf1, sizeof(buf1),
                                           ais->type8.dac1fid13.reason),
                            json_stringify(buf2, sizeof(buf2),
                                           ais->type8.dac1fid13.closefrom),
                            json_stringify(buf3, sizeof(buf3),
                                           ais->type8.dac1fid13.closeto),
                            ais->type8.dac1fid13.radius,
                            ais->type8.dac1fid13.extunit,
                            ais->type8.dac1fid13.fmonth,
                            ais->type8.dac1fid13.fday,
                            ais->type8.dac1fid13.fhour,
                            ais->type8.dac1fid13.fminute,
                            ais->type8.dac1fid13.tmonth,
                            ais->type8.dac1fid13.tday,
                            ais->type8.dac1fid13.thour,
                            ais->type8.dac1fid13.tminute);
                break;
            case 15:        // IMO236 - Extended ship and voyage
                str_appendf(buf, buflen,
                            ",\"airdraught\":%u}\r\n",
                            ais->type8.dac1fid15.airdraught);
                break;
            case 16:    // IMO289 - Number of persons on board
                str_appendf(buf, buflen,
                            ",\"persons\":%u}\r\n",
                            ais->type6.dac1fid16.persons);
                break;
            case 17:        // IMO289 - VTS-generated/synthetic targets
                (void)strlcat(buf, ",\"targets\":[", buflen);
                for (i = 0; i < ais->type8.dac1fid17.ntargets; i++) {
                    str_appendf(buf, buflen,
                                "{\"idtype\":%u,\"idtype_text\":\"%s\"",
                                ais->type8.dac1fid17.targets[i].idtype,
                                idtypes[ais->type8.dac1fid17.targets[i].idtype]);
                    switch (ais->type8.dac1fid17.targets[i].idtype) {
                    case DAC1FID17_IDTYPE_MMSI:
                        str_appendf(buf, buflen,
                            ",\"%s\":\"%u\"",
                            idtypes[ais->type8.dac1fid17.targets[i].idtype],
                            ais->type8.dac1fid17.targets[i].id.mmsi);
                        break;
                    case DAC1FID17_IDTYPE_IMO:
                        str_appendf(buf, buflen,
                            ",\"%s\":\"%u\"",
                            idtypes[ais->type8.dac1fid17.targets[i].idtype],
                            ais->type8.dac1fid17.targets[i].id.imo);
                        break;
                    case DAC1FID17_IDTYPE_CALLSIGN:
                        str_appendf(buf, buflen,
                            ",\"%s\":\"%s\"",
                            idtypes[ais->type8.dac1fid17.targets[i].idtype],
                            json_stringify(buf1, sizeof(buf1),
                               ais->type8.dac1fid17.targets[i].id.callsign));
                        break;
                    default:
                        str_appendf(buf, buflen,
                            ",\"%s\":\"%s\"",
                            idtypes[ais->type8.dac1fid17.targets[i].idtype],
                            json_stringify(buf1, sizeof(buf1),
                                ais->type8.dac1fid17.targets[i].id.other));
                    }
                    if (scaled) {
                        str_appendf(buf, buflen,
                            ",\"lat\":%.6f,\"lon\":%.6f",
                            ais->type8.dac1fid17.targets[i].lat /
                                AIS_LATLON3_DIV,
                            ais->type8.dac1fid17.targets[i].lon /
                                AIS_LATLON3_DIV);
                    } else {
                        str_appendf(buf, buflen,
                            ",\"lat\":%d,\"lon\":%d",
                            ais->type8.dac1fid17.targets[i].lat,
                            ais->type8.dac1fid17.targets[i].lon);
                    }
                    str_appendf(buf, buflen,
                        ",\"course\":%u,\"second\":%u,\"speed\":%u},",
                        ais->type8.dac1fid17.targets[i].course,
                        ais->type8.dac1fid17.targets[i].second,
                        ais->type8.dac1fid17.targets[i].speed);
                }
                str_rstrip_char(buf, ',');
                (void)strlcat(buf, "]}\r\n", buflen);
                break;
            case 19:        // IMO289 - Marine Traffic Signal
                str_appendf(buf, buflen,
                            ",\"linkage\":%u,\"station\":\"%s\","
                            "\"lon\":%.6f,\"lat\":%.6f,\"status\":%u,"
                            "\"signal\":%u,\"signal_text\":\"%s\","
                            "\"hour\":%u,\"minute\":%u,"
                            "\"nextsignal\":%u"
                            "\"nextsignal_text\":\"%s\""
                            "}\r\n",
                            ais->type8.dac1fid19.linkage,
                            json_stringify(buf1, sizeof(buf1),
                                           ais->type8.dac1fid19.station),
                            ais->type8.dac1fid19.lon / AIS_LATLON3_DIV,
                            ais->type8.dac1fid19.lat / AIS_LATLON3_DIV,
                            ais->type8.dac1fid19.status,
                            ais->type8.dac1fid19.signal,
                            SIGNAL_DISPLAY(ais->type8.dac1fid19.signal),
                            ais->type8.dac1fid19.hour,
                            ais->type8.dac1fid19.minute,
                            ais->type8.dac1fid19.nextsignal,
                            SIGNAL_DISPLAY(ais->type8.dac1fid19.nextsignal));
                break;
            case 21:        // IMO289 - Weather obs. report from ship
                break;
            case 22:        // IMO289 - Area notice - broadcast
                break;
            case 24:  // IMO289 - Extended ship static & voyage-related data
                break;
            case 25:        // IMO289 - Dangerous Cargo Indication
                break;
            case 27:        // IMO289 - Route information - broadcast
                str_appendf(buf, buflen,
                            ",\"linkage\":%u,\"sender\":%u,"
                            "\"rtype\":%u,"
                            "\"rtype_text\":\"%s\","
                            "\"start\":\"%02u-%02uT%02u:%02uZ\","
                            "\"duration\":%u,\"waypoints\":[",
                            ais->type8.dac1fid27.linkage,
                            ais->type8.dac1fid27.sender,
                            ais->type8.dac1fid27.rtype,
                            route_type[ais->type8.dac1fid27.rtype],
                            ais->type8.dac1fid27.month,
                            ais->type8.dac1fid27.day,
                            ais->type8.dac1fid27.hour,
                            ais->type8.dac1fid27.minute,
                            ais->type8.dac1fid27.duration);
                for (i = 0; i < ais->type8.dac1fid27.waycount; i++) {
                    if (scaled) {
                        str_appendf(buf, buflen,
                            "{\"lon\":%.7f,\"lat\":%.7f},",
                            ais->type8.dac1fid27.waypoints[i].lon /
                                AIS_LATLON4_DIV,
                            ais->type8.dac1fid27.waypoints[i].lat /
                                AIS_LATLON4_DIV);
                    } else {
                        str_appendf(buf, buflen,
                            "{\"lon\":%d,\"lat\":%d},",
                            ais->type8.dac1fid27.waypoints[i].lon,
                            ais->type8.dac1fid27.waypoints[i].lat);
                    }
                }
                str_rstrip_char(buf, ',');
                (void)strlcat(buf, "]}\r\n", buflen);
                break;
            case 29:        // IMO289 - Text Description - broadcast
                str_appendf(buf, buflen,
                            ",\"linkage\":%u,\"text\":\"%s\"}\r\n",
                            ais->type8.dac1fid29.linkage,
                            json_stringify(buf1, sizeof(buf1),
                                           ais->type8.dac1fid29.text));
                break;
            case 31:        // IMO289 - Meteorological/Hydrological data
                // some fields have been merged to an ISO8601 partial date
                // layout is almost identical to FID=11 from IMO236
                if (scaled) {
                    str_appendf(buf, buflen,
                                ",\"lat\":%.6f,\"lon\":%.6f",
                                ais->type8.dac1fid31.lat / AIS_LATLON3_DIV,
                                ais->type8.dac1fid31.lon / AIS_LATLON3_DIV);
                } else {
                    str_appendf(buf, buflen,
                                ",\"lat\":%d,\"lon\":%d",
                                ais->type8.dac1fid31.lat,
                                ais->type8.dac1fid31.lon);
                }
                str_appendf(buf, buflen,
                            ",\"accuracy\":%s",
                            JSON_BOOL(ais->type8.dac1fid31.accuracy));
                str_appendf(buf, buflen,
                            ",\"timestamp\":\"%02uT%02u:%02uZ\","
                            "\"wspeed\":%u,\"wgust\":%u,\"wdir\":%u,"
                            "\"wgustdir\":%u,\"humidity\":%u",
                            ais->type8.dac1fid31.day,
                            ais->type8.dac1fid31.hour,
                            ais->type8.dac1fid31.minute,
                            ais->type8.dac1fid31.wspeed,
                            ais->type8.dac1fid31.wgust,
                            ais->type8.dac1fid31.wdir,
                            ais->type8.dac1fid31.wgustdir,
                            ais->type8.dac1fid31.humidity);
                if (scaled) {
                    str_appendf(buf, buflen,
                                ",\"airtemp\":%.1f,\"dewpoint\":%.1f,"
                                "\"pressure\":%u,\"pressuretend\":\"%s\","
                                "\"visgreater\":%s",
                                ais->type8.dac1fid31.airtemp /
                                DAC1FID31_AIRTEMP_DIV,
                                ais->type8.dac1fid31.dewpoint /
                                DAC1FID31_DEWPOINT_DIV,
                                ais->type8.dac1fid31.pressure -
                                DAC1FID31_PRESSURE_OFFSET,
                                trends[ais->type8.dac1fid31.pressuretend],
                                JSON_BOOL(ais->type8.dac1fid31.visgreater));
                    str_appendf(buf, buflen,
                                ",\"visibility\":%.1f",
                                ais->type8.dac1fid31.visibility /
                                DAC1FID31_VISIBILITY_DIV);
                    str_appendf(buf, buflen,
                                ",\"waterlevel\":%.1f",
                                ((unsigned int)ais->type8.dac1fid31.waterlevel -
                                DAC1FID31_WATERLEVEL_OFFSET) /
                                DAC1FID31_WATERLEVEL_DIV);
                    str_appendf(buf, buflen,
                                ",\"leveltrend\":\"%s\","
                                "\"cspeed\":%.1f,\"cdir\":%u,"
                                "\"cspeed2\":%.1f,\"cdir2\":%u,"
                                "\"cdepth2\":%u,"
                                "\"cspeed3\":%.1f,\"cdir3\":%u,"
                                "\"cdepth3\":%u,"
                                "\"waveheight\":%.1f,\"waveperiod\":%u,"
                                "\"wavedir\":%u,"
                                "\"swellheight\":%.1f,\"swellperiod\":%u,"
                                "\"swelldir\":%u,"
                                "\"seastate\":%u,\"watertemp\":%.1f,"
                                "\"preciptype\":\"%s\",\"salinity\":%.1f,"
                                "\"ice\":\"%s\"",
                                trends[ais->type8.dac1fid31.leveltrend],
                                ais->type8.dac1fid31.cspeed / DAC1FID31_CSPEED_DIV,
                                ais->type8.dac1fid31.cdir,
                                ais->type8.dac1fid31.cspeed2 / DAC1FID31_CSPEED_DIV,
                                ais->type8.dac1fid31.cdir2,
                                ais->type8.dac1fid31.cdepth2,
                                ais->type8.dac1fid31.cspeed3 / DAC1FID31_CSPEED_DIV,
                                ais->type8.dac1fid31.cdir3,
                                ais->type8.dac1fid31.cdepth3,
                                ais->type8.dac1fid31.waveheight / DAC1FID31_HEIGHT_DIV,
                                ais->type8.dac1fid31.waveperiod,
                                ais->type8.dac1fid31.wavedir,
                                ais->type8.dac1fid31.swellheight / DAC1FID31_HEIGHT_DIV,
                                ais->type8.dac1fid31.swellperiod,
                                ais->type8.dac1fid31.swelldir,
                                ais->type8.dac1fid31.seastate,
                                ais->type8.dac1fid31.watertemp / DAC1FID31_WATERTEMP_DIV,
                                preciptypes[ais->type8.dac1fid31.preciptype],
                                ais->type8.dac1fid31.salinity / DAC1FID31_SALINITY_DIV,
                                ice[ais->type8.dac1fid31.ice]);
                } else {
                    str_appendf(buf, buflen,
                                ",\"airtemp\":%d,\"dewpoint\":%d,"
                                "\"pressure\":%u,\"pressuretend\":%u,"
                                "\"visgreater\":%s",
                                ais->type8.dac1fid31.airtemp,
                                ais->type8.dac1fid31.dewpoint,
                                ais->type8.dac1fid31.pressure,
                                ais->type8.dac1fid31.pressuretend,
                                JSON_BOOL(ais->type8.dac1fid31.visgreater));
                    str_appendf(buf, buflen,
                                ",\"visibility\":%u",
                                ais->type8.dac1fid31.visibility);
                    str_appendf(buf, buflen,
                                   ",\"waterlevel\":%d",
                                   ais->type8.dac1fid31.waterlevel);
                    str_appendf(buf, buflen,
                                ",\"leveltrend\":%u,"
                                "\"cspeed\":%u,\"cdir\":%u,"
                                "\"cspeed2\":%u,\"cdir2\":%u,"
                                "\"cdepth2\":%u,"
                                "\"cspeed3\":%u,\"cdir3\":%u,"
                                "\"cdepth3\":%u,"
                                "\"waveheight\":%u,\"waveperiod\":%u,"
                                "\"wavedir\":%u,"
                                "\"swellheight\":%u,\"swellperiod\":%u,"
                                "\"swelldir\":%u,"
                                "\"seastate\":%u,\"watertemp\":%d,"
                                "\"preciptype\":%u,\"salinity\":%u,"
                                "\"ice\":%u",
                                ais->type8.dac1fid31.leveltrend,
                                ais->type8.dac1fid31.cspeed,
                                ais->type8.dac1fid31.cdir,
                                ais->type8.dac1fid31.cspeed2,
                                ais->type8.dac1fid31.cdir2,
                                ais->type8.dac1fid31.cdepth2,
                                ais->type8.dac1fid31.cspeed3,
                                ais->type8.dac1fid31.cdir3,
                                ais->type8.dac1fid31.cdepth3,
                                ais->type8.dac1fid31.waveheight,
                                ais->type8.dac1fid31.waveperiod,
                                ais->type8.dac1fid31.wavedir,
                                ais->type8.dac1fid31.swellheight,
                                ais->type8.dac1fid31.swellperiod,
                                ais->type8.dac1fid31.swelldir,
                                ais->type8.dac1fid31.seastate,
                                ais->type8.dac1fid31.watertemp,
                                ais->type8.dac1fid31.preciptype,
                                ais->type8.dac1fid31.salinity,
                                ais->type8.dac1fid31.ice);
                }
                (void)strlcat(buf, "}\r\n", buflen);
                break;
            }
        } else if (200 == ais->type8.dac) {
            struct {
                const unsigned int code;
                const unsigned int ais;
                const char *legend;
            } *cp, shiptypes[] = {
                /*
                 * The Inland AIS standard is not clear which numbers are
                 * supposed to be in the type slot.  The ranges are disjoint,
                 * so we'll match on both.
                 */
                {8000, 99, "Vessel, type unknown"},
                {8010, 79, "Motor freighter"},
                {8020, 89, "Motor tanker"},
                {8021, 80, "Motor tanker, liquid cargo, type N"},
                {8022, 80, "Motor tanker, liquid cargo, type C"},
                {8023, 89,
                 "Motor tanker, dry cargo as if liquid (e.g. cement)"},
                {8030, 79, "Container vessel"},
                {8040, 80, "Gas tanker"},
                {8050, 79, "Motor freighter, tug"},
                {8060, 89, "Motor tanker, tug"},
                {8070, 79, "Motor freighter with one or more ships alongside"},
                {8080, 89, "Motor freighter with tanker"},
                {8090, 79, "Motor freighter pushing one or more freighters"},
                {8100, 89, "Motor freighter pushing at least one tank-ship"},
                {8110, 79, "Tug, freighter"},
                {8120, 89, "Tug, tanker"},
                {8130, 31, "Tug freighter, coupled"},
                {8140, 31, "Tug, freighter/tanker, coupled"},
                {8150, 99, "Freightbarge"},
                {8160, 99, "Tankbarge"},
                {8161, 90, "Tankbarge, liquid cargo, type N"},
                {8162, 90, "Tankbarge, liquid cargo, type C"},
                {8163, 99, "Tankbarge, dry cargo as if liquid (e.g. cement)"},
                {8170, 99, "Freightbarge with containers"},
                {8180, 90, "Tankbarge, gas"},
                {8210, 79, "Pushtow, one cargo barge"},
                {8220, 79, "Pushtow, two cargo barges"},
                {8230, 79, "Pushtow, three cargo barges"},
                {8240, 79, "Pushtow, four cargo barges"},
                {8250, 79, "Pushtow, five cargo barges"},
                {8260, 79, "Pushtow, six cargo barges"},
                {8270, 79, "Pushtow, seven cargo barges"},
                {8280, 79, "Pushtow, eight cargo barges"},
                {8290, 79, "Pushtow, nine or more barges"},
                {8310, 80, "Pushtow, one tank/gas barge"},
                {8320, 80,
                 "Pushtow, two barges at least one tanker or gas barge"},
                {8330, 80,
                 "Pushtow, three barges at least one tanker or gas barge"},
                {8340, 80,
                 "Pushtow, four barges at least one tanker or gas barge"},
                {8350, 80,
                 "Pushtow, five barges at least one tanker or gas barge"},
                {8360, 80,
                 "Pushtow, six barges at least one tanker or gas barge"},
                {8370, 80,
                 "Pushtow, seven barges at least one tanker or gas barg"},
                {0, 0, "Illegal ship type value."},
            };
            const char *hazard_types[] = {
                "0 blue cones/lights",
                "1 blue cone/light",
                "2 blue cones/lights",
                "3 blue cones/lights",
                "4 B-Flag",
                "Unknown",
            };

#define HTYPE_DISPLAY(n) (((n) < (unsigned int)NITEMS(hazard_types)) ? \
                                  hazard_types[n] : "INVALID HAZARD TYPE")

            const char *lstatus_types[] = {
                "N/A (default)",
                "Unloaded",
                "Loaded",
            };

#define LSTATUS_DISPLAY(n) (((n) < (unsigned int)NITEMS(lstatus_types)) ? \
                                   lstatus_types[n] : "INVALID LOAD STATUS")

            const char *emma_types[] = {
                "Not Available",
                "Wind",
                "Rain",
                "Snow and ice",
                "Thunderstorm",
                "Fog",
                "Low temperature",
                "High temperature",
                "Flood",
                "Forest Fire",
            };

#define EMMA_TYPE_DISPLAY(n) (((n) < (unsigned int)NITEMS(emma_types)) ? \
                                     emma_types[n] : "INVALID EMMA TYPE")

            const char *emma_classes[] = {
                "Slight",
                "Medium",
                "Strong",
            };

#define EMMA_CLASS_DISPLAY(n) (((n) < (unsigned int)NITEMS(emma_classes)) ? \
                                      emma_classes[n] : "INVALID EMMA TYPE")

            const char *emma_winds[] = {
                "N/A",
                "North",
                "North East",
                "East",
                "South East",
                "South",
                "South West",
                "West",
                "North West",
            };

#define EMMA_WIND_DISPLAY(n) (((n) < (unsigned int)NITEMS(emma_winds)) ? \
                                  emma_winds[n] : "INVALID EMMA WIND DIRECTION")

            const char *direction_vocabulary[] = {
                "Unknown",
                "Upstream",
                "Downstream",
                "To left bank",
                "To right bank",
            };

#define DIRECTION_DISPLAY(n) (((n) < \
                              (unsigned int)NITEMS(direction_vocabulary)) ? \
                              direction_vocabulary[n] : "INVALID DIRECTION")

            const char *status_vocabulary[] = {
                "Unknown",
                "No light",
                "White",
                "Yellow",
                "Green",
                "Red",
                "White flashing",
                "Yellow flashing.",
            };

#define STATUS_DISPLAY(n) (((n) < (unsigned int)NITEMS(status_vocabulary)) ? \
                           status_vocabulary[n] : "INVALID STATUS")

            switch (ais->type8.fid) {
            case 10:        // Inland ship static and voyage-related data
                for (cp = shiptypes; cp < shiptypes + NITEMS(shiptypes); cp++) {
                    if (cp->code == ais->type8.dac200fid10.shiptype ||
                        cp->ais == ais->type8.dac200fid10.shiptype ||
                        0 == cp->code) {
                        break;
                    }
                }
                str_appendf(buf, buflen,
                            ",\"vin\":\"%s\",\"length\":%u,\"beam\":%u,"
                            "\"shiptype\":%u,\"shiptype_text\":\"%s\","
                            "\"hazard\":%u,\"hazard_text\":\"%s\","
                            "\"draught\":%u,"
                            "\"loaded\":%u,\"loaded_text\":\"%s\","
                            "\"speed_q\":%s,"
                            "\"course_q\":%s,"
                            "\"heading_q\":%s}\r\n",
                            json_stringify(buf1, sizeof(buf1),
                            ais->type8.dac200fid10.vin),
                            ais->type8.dac200fid10.length,
                            ais->type8.dac200fid10.beam,
                            ais->type8.dac200fid10.shiptype,
                            cp->legend,
                            ais->type8.dac200fid10.hazard,
                            HTYPE_DISPLAY(ais->type8.dac200fid10.hazard),
                            ais->type8.dac200fid10.draught,
                            ais->type8.dac200fid10.loaded,
                            LSTATUS_DISPLAY(ais->type8.dac200fid10.loaded),
                            JSON_BOOL(ais->type8.dac200fid10.speed_q),
                            JSON_BOOL(ais->type8.dac200fid10.course_q),
                            JSON_BOOL(ais->type8.dac200fid10.heading_q));
                break;
            case 23:    // EMMA warning
                if (!ais->type8.structured) {
                    break;
                }
                str_appendf(buf, buflen,
                            ",\"start\":\"%4u-%02u-%02uT%02u:%02u\","
                            "\"end\":\"%4u-%02u-%02uT%02u:%02u\"",
                            ais->type8.dac200fid23.start_year + 2000,
                            ais->type8.dac200fid23.start_month,
                            ais->type8.dac200fid23.start_hour,
                            ais->type8.dac200fid23.start_minute,
                            ais->type8.dac200fid23.start_day,
                            ais->type8.dac200fid23.end_year + 2000,
                            ais->type8.dac200fid23.end_month,
                            ais->type8.dac200fid23.end_day,
                            ais->type8.dac200fid23.end_hour,
                            ais->type8.dac200fid23.end_minute);
                if (scaled) {
                    str_appendf(buf, buflen,
                        ",\"start_lon\":%.7f,\"start_lat\":%.7f,"
                        "\"end_lon\":%.7f,\"end_lat\":%.7f",
                        ais->type8.dac200fid23.start_lon / AIS_LATLON_DIV,
                        ais->type8.dac200fid23.start_lat / AIS_LATLON_DIV,
                        ais->type8.dac200fid23.end_lon / AIS_LATLON_DIV,
                        ais->type8.dac200fid23.end_lat / AIS_LATLON_DIV);
                } else {
                    str_appendf(buf, buflen,
                        ",\"start_lon\":%d,\"start_lat\":%d,\"end_lon\":%d,"
                        "\"end_lat\":%d",
                        ais->type8.dac200fid23.start_lon,
                        ais->type8.dac200fid23.start_lat,
                        ais->type8.dac200fid23.end_lon,
                        ais->type8.dac200fid23.end_lat);
                }
                str_appendf(buf, buflen,
                    ",\"type\":%u,\"type_text\":\"%s\",\"min\":%d,"
                    "\"max\":%d,\"class\":%u,\"class_text\":\"%s\","
                    "\"wind\":%u,\"wind_text\":\"%s\"}\r\n",

                    ais->type8.dac200fid23.type,
                    EMMA_TYPE_DISPLAY(ais->type8.dac200fid23.type),
                    ais->type8.dac200fid23.min,
                    ais->type8.dac200fid23.max,
                    ais->type8.dac200fid23.intensity,
                    EMMA_CLASS_DISPLAY(ais->type8.dac200fid23.intensity),
                    ais->type8.dac200fid23.wind,
                    EMMA_WIND_DISPLAY(ais->type8.dac200fid23.wind));
                break;
            case 24:    // Inland AIS Water Levels
                str_appendf(buf, buflen, ",\"country\":\"%s\",\"gauges\":[",
                            ais->type8.dac200fid24.country);
                for (i = 0; i < ais->type8.dac200fid24.ngauges; i++) {
                    str_appendf(buf, buflen,
                        "{\"id\":%u,\"level\":%d},",
                        ais->type8.dac200fid24.gauges[i].id,
                        ais->type8.dac200fid24.gauges[i].level);
                }
                str_rstrip_char(buf, ',');
                (void)strlcat(buf, "]}\r\n", buflen);
                break;
            case 40:    // Inland AIS Signal Strength
                if (scaled) {
                    str_appendf(buf, buflen,
                        ",\"lon\":%.7f,\"lat\":%.7f",
                        ais->type8.dac200fid40.lon / AIS_LATLON_DIV,
                        ais->type8.dac200fid40.lat / AIS_LATLON_DIV);
                } else {
                    str_appendf(buf, buflen,
                        ",\"lon\":%d,\"lat\":%d",
                        ais->type8.dac200fid40.lon,
                        ais->type8.dac200fid40.lat);
                }
                str_appendf(buf, buflen,
                    ",\"form\":%u,\"facing\":%u,\"direction\":%u,"
                    "\"direction_text\":\"%s\",\"status\":%u,"
                    "\"status_text\":\"%s\"}\r\n",
                    ais->type8.dac200fid40.form,
                    ais->type8.dac200fid40.facing,
                    ais->type8.dac200fid40.direction,
                    DIRECTION_DISPLAY(ais->type8.dac200fid40.direction),
                    ais->type8.dac200fid40.status,
                    STATUS_DISPLAY(ais->type8.dac200fid40.status));
                break;
            }
        }
        break;
    case 9:                     // Standard SAR Aircraft Position Report
        if (scaled) {
            char altlegend[20];
            char speedlegend[20];

            /*
             * Express altitude as nan if not available,
             * "high" for above the reporting ceiling.
             */
            if (AIS_ALT_NOT_AVAILABLE == ais->type9.alt) {
                (void)strlcpy(altlegend, "\"nan\"", sizeof(altlegend));
            } else if (AIS_ALT_HIGH == ais->type9.alt) {
                (void)strlcpy(altlegend, "\"high\"", sizeof(altlegend));
            } else {
                (void)snprintf(altlegend, sizeof(altlegend),
                               "%u", ais->type9.alt);
            }

            /*
             * Express speed as nan if not available,
             * "high" for above the reporting ceiling.
             */
            if (AIS_SAR_SPEED_NOT_AVAILABLE == ais->type9.speed) {
                (void)strlcpy(speedlegend, "\"nan\"", sizeof(speedlegend));
            } else if (AIS_SAR_FAST_MOVER == ais->type9.speed) {
                (void)strlcpy(speedlegend, "\"fast\"", sizeof(speedlegend));
            } else {
                (void)snprintf(speedlegend, sizeof(speedlegend),
                               "%u", ais->type9.speed);
            }

            str_appendf(buf, buflen,
                        ",\"alt\":%s,\"speed\":%s,\"accuracy\":%s,"
                        "\"lon\":%.7f,\"lat\":%.7f,\"course\":%.1f,"
                        "\"second\":%u,\"regional\":%u,\"dte\":%u,"
                        "\"raim\":%s,\"radio\":%u}\r\n",
                        altlegend,
                        speedlegend,
                        JSON_BOOL(ais->type9.accuracy),
                        ais->type9.lon / AIS_LATLON_DIV,
                        ais->type9.lat / AIS_LATLON_DIV,
                        ais->type9.course / 10.0,
                        ais->type9.second,
                        ais->type9.regional,
                        ais->type9.dte,
                        JSON_BOOL(ais->type9.raim), ais->type9.radio);
        } else {
            str_appendf(buf, buflen,
                        ",\"alt\":%u,\"speed\":%u,\"accuracy\":%s,"
                        "\"lon\":%d,\"lat\":%d,\"course\":%u,"
                        "\"second\":%u,\"regional\":%u,\"dte\":%u,"
                        "\"raim\":%s,\"radio\":%u}\r\n",
                        ais->type9.alt,
                        ais->type9.speed,
                        JSON_BOOL(ais->type9.accuracy),
                        ais->type9.lon,
                        ais->type9.lat,
                        ais->type9.course,
                        ais->type9.second,
                        ais->type9.regional,
                        ais->type9.dte,
                        JSON_BOOL(ais->type9.raim), ais->type9.radio);
        }
        break;
    case 10:                    // UTC/Date Inquiry
        str_appendf(buf, buflen,
                    ",\"dest_mmsi\":%u}\r\n", ais->type10.dest_mmsi);
        break;
    case 12:                    // Safety Related Message
        str_appendf(buf, buflen,
                    ",\"seqno\":%u,\"dest_mmsi\":%u,\"retransmit\":%s,"
                    "\"text\":\"%s\"}\r\n",
                    ais->type12.seqno,
                    ais->type12.dest_mmsi,
                    JSON_BOOL(ais->type12.retransmit),
                    json_stringify(buf1, sizeof(buf1), ais->type12.text));
        break;
    case 14:                    // Safety Related Broadcast Message
        str_appendf(buf, buflen,
                    ",\"text\":\"%s\"}\r\n",
                    json_stringify(buf1, sizeof(buf1), ais->type14.text));
        break;
    case 15:                    // Interrogation
        str_appendf(buf, buflen,
                    ",\"mmsi1\":%u,\"type1_1\":%u,\"offset1_1\":%u,"
                    "\"type1_2\":%u,\"offset1_2\":%u,\"mmsi2\":%u,"
                    "\"type2_1\":%u,\"offset2_1\":%u}\r\n",
                    ais->type15.mmsi1,
                    ais->type15.type1_1,
                    ais->type15.offset1_1,
                    ais->type15.type1_2,
                    ais->type15.offset1_2,
                    ais->type15.mmsi2,
                    ais->type15.type2_1, ais->type15.offset2_1);
        break;
    case 16:
        str_appendf(buf, buflen,
                    ",\"mmsi1\":%u,\"offset1\":%u,\"increment1\":%u,"
                    "\"mmsi2\":%u,\"offset2\":%u,\"increment2\":%u}\r\n",
                    ais->type16.mmsi1,
                    ais->type16.offset1,
                    ais->type16.increment1,
                    ais->type16.mmsi2,
                    ais->type16.offset2, ais->type16.increment2);
        break;
    case 17:
        if (scaled) {
            str_appendf(buf, buflen,
                        ",\"lon\":%.1f,\"lat\":%.1f,\"data\":\"%zd:%s\""
                        "}\r\n",
                        ais->type17.lon / AIS_GNSS_LATLON_DIV,
                        ais->type17.lat / AIS_GNSS_LATLON_DIV,
                        ais->type17.bitcount,
                        gps_hexdump(scratchbuf, sizeof(scratchbuf),
                             (const unsigned char *)ais->type17.bitdata,
                             BITS_TO_BYTES(ais->type17.bitcount)));
        } else {
            str_appendf(buf, buflen,
                        ",\"lon\":%d,\"lat\":%d,\"data\":\"%zd:%s\"}\r\n",
                        ais->type17.lon,
                        ais->type17.lat,
                        ais->type17.bitcount,
                        gps_hexdump(scratchbuf, sizeof(scratchbuf),
                             (const unsigned char *)ais->type17.bitdata,
                             BITS_TO_BYTES(ais->type17.bitcount)));
        }
        break;
    case 18:
        if (scaled) {
            str_appendf(buf, buflen,
                        ",\"reserved\":%u,\"speed\":%.1f,\"accuracy\":%s,"
                        "\"lon\":%.7f,\"lat\":%.7f,\"course\":%.1f,"
                        "\"heading\":%u,\"second\":%u,\"regional\":%u,"
                        "\"cs\":%s,\"display\":%s,\"dsc\":%s,\"band\":%s,"
                        "\"msg22\":%s,\"raim\":%s,\"radio\":%u}\r\n",
                        ais->type18.reserved,
                        ais->type18.speed / 10.0,
                        JSON_BOOL(ais->type18.accuracy),
                        ais->type18.lon / AIS_LATLON_DIV,
                        ais->type18.lat / AIS_LATLON_DIV,
                        ais->type18.course / 10.0,
                        ais->type18.heading,
                        ais->type18.second,
                        ais->type18.regional,
                        JSON_BOOL(ais->type18.cs),
                        JSON_BOOL(ais->type18.display),
                        JSON_BOOL(ais->type18.dsc),
                        JSON_BOOL(ais->type18.band),
                        JSON_BOOL(ais->type18.msg22),
                        JSON_BOOL(ais->type18.raim), ais->type18.radio);
        } else {
            str_appendf(buf, buflen,
                        ",\"reserved\":%u,\"speed\":%u,\"accuracy\":%s,"
                        "\"lon\":%d,\"lat\":%d,\"course\":%u,"
                        "\"heading\":%u,\"second\":%u,\"regional\":%u,"
                        "\"cs\":%s,\"display\":%s,\"dsc\":%s,\"band\":%s,"
                        "\"msg22\":%s,\"raim\":%s,\"radio\":%u}\r\n",
                        ais->type18.reserved,
                        ais->type18.speed,
                        JSON_BOOL(ais->type18.accuracy),
                        ais->type18.lon,
                        ais->type18.lat,
                        ais->type18.course,
                        ais->type18.heading,
                        ais->type18.second,
                        ais->type18.regional,
                        JSON_BOOL(ais->type18.cs),
                        JSON_BOOL(ais->type18.display),
                        JSON_BOOL(ais->type18.dsc),
                        JSON_BOOL(ais->type18.band),
                        JSON_BOOL(ais->type18.msg22),
                        JSON_BOOL(ais->type18.raim), ais->type18.radio);
        }
        break;
    case 19:
        if (scaled) {
            str_appendf(buf, buflen,
                        ",\"reserved\":%u,\"speed\":%.1f,\"accuracy\":%s,"
                        "\"lon\":%.7f,\"lat\":%.7f,\"course\":%.1f,"
                        "\"heading\":%u,\"second\":%u,\"regional\":%u,"
                        "\"shipname\":\"%s\","
                        "\"shiptype\":%u,\"shiptype_text\":\"%s\","
                        "\"to_bow\":%u,\"to_stern\":%u,\"to_port\":%u,"
                        "\"to_starboard\":%u,"
                        "\"epfd\":%u,\"epfd_text\":\"%s\","
                        "\"raim\":%s,\"dte\":%u,\"assigned\":%s}\r\n",
                        ais->type19.reserved,
                        ais->type19.speed / 10.0,
                        JSON_BOOL(ais->type19.accuracy),
                        ais->type19.lon / AIS_LATLON_DIV,
                        ais->type19.lat / AIS_LATLON_DIV,
                        ais->type19.course / 10.0,
                        ais->type19.heading,
                        ais->type19.second,
                        ais->type19.regional,
                        json_stringify(buf1, sizeof(buf1),
                                       ais->type19.shipname),
                        ais->type19.shiptype,
                        SHIPTYPE_DISPLAY(ais->type19.shiptype),
                        ais->type19.to_bow,
                        ais->type19.to_stern,
                        ais->type19.to_port,
                        ais->type19.to_starboard,
                        ais->type19.epfd,
                        EPFD_DISPLAY(ais->type19.epfd),
                        JSON_BOOL(ais->type19.raim),
                        ais->type19.dte,
                        JSON_BOOL(ais->type19.assigned));
        } else {
            str_appendf(buf, buflen,
                        ",\"reserved\":%u,\"speed\":%u,\"accuracy\":%s,"
                        "\"lon\":%d,\"lat\":%d,\"course\":%u,"
                        "\"heading\":%u,\"second\":%u,\"regional\":%u,"
                        "\"shipname\":\"%s\","
                        "\"shiptype\":%u,\"shiptype_text\":\"%s\","
                        "\"to_bow\":%u,\"to_stern\":%u,\"to_port\":%u,"
                        "\"to_starboard\":%u,"
                        "\"epfd\":%u,\"epfd_text\":\"%s\","
                        "\"raim\":%s,\"dte\":%u,\"assigned\":%s}\r\n",
                        ais->type19.reserved,
                        ais->type19.speed,
                        JSON_BOOL(ais->type19.accuracy),
                        ais->type19.lon,
                        ais->type19.lat,
                        ais->type19.course,
                        ais->type19.heading,
                        ais->type19.second,
                        ais->type19.regional,
                        json_stringify(buf1, sizeof(buf1),
                                       ais->type19.shipname),
                        ais->type19.shiptype,
                        SHIPTYPE_DISPLAY(ais->type19.shiptype),
                        ais->type19.to_bow,
                        ais->type19.to_stern,
                        ais->type19.to_port,
                        ais->type19.to_starboard,
                        ais->type19.epfd,
                        EPFD_DISPLAY(ais->type19.epfd),
                        JSON_BOOL(ais->type19.raim),
                        ais->type19.dte,
                        JSON_BOOL(ais->type19.assigned));
        }
        break;
    case 20:                    // Data Link Management Message
        str_appendf(buf, buflen,
                    ",\"offset1\":%u,\"number1\":%u,"
                    "\"timeout1\":%u,\"increment1\":%u,"
                    "\"offset2\":%u,\"number2\":%u,"
                    "\"timeout2\":%u,\"increment2\":%u,"
                    "\"offset3\":%u,\"number3\":%u,"
                    "\"timeout3\":%u,\"increment3\":%u,"
                    "\"offset4\":%u,\"number4\":%u,"
                    "\"timeout4\":%u,\"increment4\":%u}\r\n",
                    ais->type20.offset1,
                    ais->type20.number1,
                    ais->type20.timeout1,
                    ais->type20.increment1,
                    ais->type20.offset2,
                    ais->type20.number2,
                    ais->type20.timeout2,
                    ais->type20.increment2,
                    ais->type20.offset3,
                    ais->type20.number3,
                    ais->type20.timeout3,
                    ais->type20.increment3,
                    ais->type20.offset4,
                    ais->type20.number4,
                    ais->type20.timeout4, ais->type20.increment4);
        break;
    case 21:                    // Aid to Navigation
        if (scaled) {
            str_appendf(buf, buflen,
                        ",\"aid_type\":%u,\"aid_type_text\":\"%s\","
                        "\"name\":\"%s\",\"lon\":%.7f,"
                        "\"lat\":%.7f,\"accuracy\":%s,\"to_bow\":%u,"
                        "\"to_stern\":%u,\"to_port\":%u,\"to_starboard\":%u,"
                        "\"epfd\":%u,\"epfd_text\":\"%s\","
                        "\"second\":%u,\"regional\":%u,"
                        "\"off_position\":%s,\"raim\":%s,"
                        "\"virtual_aid\":%s}\r\n",
                        ais->type21.aid_type,
                        NAVAIDTYPE_DISPLAY(ais->type21.aid_type),
                        json_stringify(buf1, sizeof(buf1),
                                       ais->type21.name),
                        ais->type21.lon / AIS_LATLON_DIV,
                        ais->type21.lat / AIS_LATLON_DIV,
                        JSON_BOOL(ais->type21.accuracy),
                        ais->type21.to_bow, ais->type21.to_stern,
                        ais->type21.to_port, ais->type21.to_starboard,
                        ais->type21.epfd,
                        EPFD_DISPLAY(ais->type21.epfd),
                        ais->type21.second,
                        ais->type21.regional,
                        JSON_BOOL(ais->type21.off_position),
                        JSON_BOOL(ais->type21.raim),
                        JSON_BOOL(ais->type21.virtual_aid));
        } else {
            str_appendf(buf, buflen,
                        ",\"aid_type\":%u,\"aid_type_text\":\"%s\","
                        "\"name\":\"%s\",\"accuracy\":%s,"
                        "\"lon\":%d,\"lat\":%d,\"to_bow\":%u,"
                        "\"to_stern\":%u,\"to_port\":%u,\"to_starboard\":%u,"
                        "\"epfd\":%u,\"epfd_text\":\"%s\","
                        "\"second\":%u,\"regional\":%u,"
                        "\"off_position\":%s,\"raim\":%s,"
                        "\"virtual_aid\":%s}\r\n",
                        ais->type21.aid_type,
                        NAVAIDTYPE_DISPLAY(ais->type21.aid_type),
                        json_stringify(buf1, sizeof(buf1),
                                       ais->type21.name),
                        JSON_BOOL(ais->type21.accuracy),
                        ais->type21.lon,
                        ais->type21.lat,
                        ais->type21.to_bow,
                        ais->type21.to_stern,
                        ais->type21.to_port,
                        ais->type21.to_starboard,
                        ais->type21.epfd,
                        EPFD_DISPLAY(ais->type21.epfd),
                        ais->type21.second,
                        ais->type21.regional,
                        JSON_BOOL(ais->type21.off_position),
                        JSON_BOOL(ais->type21.raim),
                        JSON_BOOL(ais->type21.virtual_aid));
        }
        break;
    case 22:                    // Channel Management
        str_appendf(buf, buflen,
                    ",\"channel_a\":%u,\"channel_b\":%u,"
                    "\"txrx\":%u,\"power\":%s",
                    ais->type22.channel_a,
                    ais->type22.channel_b,
                    ais->type22.txrx, JSON_BOOL(ais->type22.power));
        if (ais->type22.addressed) {
            str_appendf(buf, buflen,
                        ",\"dest1\":%u,\"dest2\":%u",
                        ais->type22.mmsi.dest1, ais->type22.mmsi.dest2);
        } else if (scaled) {
            str_appendf(buf, buflen,
                        ",\"ne_lon\":\"%f\",\"ne_lat\":\"%f\","
                        "\"sw_lon\":\"%f\",\"sw_lat\":\"%f\"",
                        ais->type22.area.ne_lon / AIS_CHANNEL_LATLON_DIV,
                        ais->type22.area.ne_lat / AIS_CHANNEL_LATLON_DIV,
                        ais->type22.area.sw_lon / AIS_CHANNEL_LATLON_DIV,
                        ais->type22.area.sw_lat /
                        AIS_CHANNEL_LATLON_DIV);
        } else {
            str_appendf(buf, buflen,
                        ",\"ne_lon\":%d,\"ne_lat\":%d,"
                        "\"sw_lon\":%d,\"sw_lat\":%d",
                        ais->type22.area.ne_lon,
                        ais->type22.area.ne_lat,
                        ais->type22.area.sw_lon, ais->type22.area.sw_lat);
        }
        str_appendf(buf, buflen,
                    ",\"addressed\":%s,\"band_a\":%s,"
                    "\"band_b\":%s,\"zonesize\":%u}\r\n",
                    JSON_BOOL(ais->type22.addressed),
                    JSON_BOOL(ais->type22.band_a),
                    JSON_BOOL(ais->type22.band_b), ais->type22.zonesize);
        break;
    case 23:                    // Group Assignment Command
        if (scaled) {
            str_appendf(buf, buflen,
                        ",\"ne_lon\":\"%f\",\"ne_lat\":\"%f\","
                        "\"sw_lon\":\"%f\",\"sw_lat\":\"%f\","
                        "\"stationtype\":%u,\"stationtype_text\":\"%s\","
                        "\"shiptype\":%u,\"shiptype_text\":\"%s\","
                        "\"interval\":%u,\"quiet\":%u}\r\n",
                        ais->type23.ne_lon / AIS_CHANNEL_LATLON_DIV,
                        ais->type23.ne_lat / AIS_CHANNEL_LATLON_DIV,
                        ais->type23.sw_lon / AIS_CHANNEL_LATLON_DIV,
                        ais->type23.sw_lat / AIS_CHANNEL_LATLON_DIV,
                        ais->type23.stationtype,
                        STATIONTYPE_DISPLAY(ais->type23.stationtype),
                        ais->type23.shiptype,
                        SHIPTYPE_DISPLAY(ais->type23.shiptype),
                        ais->type23.interval, ais->type23.quiet);
        } else {
            str_appendf(buf, buflen,
                        ",\"ne_lon\":%d,\"ne_lat\":%d,"
                        "\"sw_lon\":%d,\"sw_lat\":%d,"
                        "\"stationtype\":%u,\"stationtype_text\":\"%s\","
                        "\"shiptype\":%u,\"shiptype_text\":\"%s\","
                        "\"interval\":%u,\"quiet\":%u}\r\n",
                        ais->type23.ne_lon,
                        ais->type23.ne_lat,
                        ais->type23.sw_lon,
                        ais->type23.sw_lat,
                        ais->type23.stationtype,
                        STATIONTYPE_DISPLAY(ais->type23.stationtype),
                        ais->type23.shiptype,
                        SHIPTYPE_DISPLAY(ais->type23.shiptype),
                        ais->type23.interval, ais->type23.quiet);
        }
        break;
    case 24:                    // Class B CS Static Data Report
        if (both != ais->type24.part) {
            static char *partnames[] = {"AB", "A", "B"};
            str_appendf(buf, buflen,
                        ",\"part\":\"%s\"",
                        json_stringify(buf1, sizeof(buf1),
                                       partnames[ais->type24.part]));
        }
        if (part_b != ais->type24.part) {
            str_appendf(buf, buflen,
                        ",\"shipname\":\"%s\"",
                        json_stringify(buf1, sizeof(buf1),
                                   ais->type24.shipname));
        }
        if (part_a != ais->type24.part) {
            str_appendf(buf, buflen,
                        ",\"shiptype\":%u,\"shiptype_text\":\"%s\","
                        "\"vendorid\":\"%s\",\"model\":%u,\"serial\":%u,"
                        "\"callsign\":\"%s\"",
                        ais->type24.shiptype,
                        SHIPTYPE_DISPLAY(ais->type24.shiptype),
                        json_stringify(buf1, sizeof(buf1),
                                       ais->type24.vendorid),
                        ais->type24.model,
                        ais->type24.serial,
                        json_stringify(buf2, sizeof(buf2),
                                       ais->type24.callsign));
            if (AIS_AUXILIARY_MMSI(ais->mmsi)) {
                str_appendf(buf, buflen,
                            ",\"mothership_mmsi\":%u",
                            ais->type24.mothership_mmsi);
            } else {
                str_appendf(buf, buflen,
                            ",\"to_bow\":%u,\"to_stern\":%u,"
                            "\"to_port\":%u,\"to_starboard\":%u",
                            ais->type24.dim.to_bow,
                            ais->type24.dim.to_stern,
                            ais->type24.dim.to_port,
                            ais->type24.dim.to_starboard);
            }
        }
        (void)strlcat(buf, "}\r\n", buflen);
        break;
    case 25:                    // Binary Message, Single Slot
        str_appendf(buf, buflen,
                    ",\"addressed\":%s,\"structured\":%s,\"dest_mmsi\":%u,"
                    "\"app_id\":%u,\"data\":\"%zd:%s\"}\r\n",
                    JSON_BOOL(ais->type25.addressed),
                    JSON_BOOL(ais->type25.structured),
                    ais->type25.dest_mmsi,
                    ais->type25.app_id,
                    ais->type25.bitcount,
                    gps_hexdump(scratchbuf, sizeof(scratchbuf),
                        (const unsigned char *)ais->type25.bitdata,
                        BITS_TO_BYTES(ais->type25.bitcount)));
        break;
    case 26:                    // Binary Message, Multiple Slot
        str_appendf(buf, buflen,
                    ",\"addressed\":%s,\"structured\":%s,\"dest_mmsi\":%u,"
                    "\"app_id\":%u,\"data\":\"%zd:%s\",\"radio\":%u}\r\n",
                    JSON_BOOL(ais->type26.addressed),
                    JSON_BOOL(ais->type26.structured),
                    ais->type26.dest_mmsi,
                    ais->type26.app_id,
                    ais->type26.bitcount,
                    gps_hexdump(scratchbuf, sizeof(scratchbuf),
                        (const unsigned char *)ais->type26.bitdata,
                        BITS_TO_BYTES(ais->type26.bitcount)),
                    ais->type26.radio);
        break;
    case 27:                    // Long Range AIS Broadcast message
        if (scaled) {
            str_appendf(buf, buflen,
                        ",\"status\":%u,\"status_text\":\"%s\""
                        "\"accuracy\":%s,\"lon\":%.4f,\"lat\":%.4f,"
                        "\"speed\":%u,\"course\":%u,\"raim\":%s,"
                        "\"gnss\":%s}\r\n",
                        ais->type27.status,
                        nav_legends[ais->type27.status],
                        JSON_BOOL(ais->type27.accuracy),
                        ais->type27.lon / AIS_LONGRANGE_LATLON_DIV,
                        ais->type27.lat / AIS_LONGRANGE_LATLON_DIV,
                        ais->type27.speed,
                        ais->type27.course,
                        JSON_BOOL(ais->type27.raim),
                        JSON_BOOL(ais->type27.gnss));
        } else {
            str_appendf(buf, buflen,
                        ",\"status\":%u,"
                        "\"accuracy\":%s,\"lon\":%d,\"lat\":%d,"
                        "\"speed\":%u,\"course\":%u,\"raim\":%s,"
                        "\"gnss\":%s}\r\n",
                        ais->type27.status,
                        JSON_BOOL(ais->type27.accuracy),
                        ais->type27.lon,
                        ais->type27.lat,
                        ais->type27.speed,
                        ais->type27.course,
                        JSON_BOOL(ais->type27.raim),
                        JSON_BOOL(ais->type27.gnss));
        }
        break;
    default:
        (void)strlcat(buf, "}\r\n", buflen);
        break;
    }
}
#endif  // defined(AIVDM_ENABLE)

/* dump the contents of an attitude_t structure as JSON
 * maybe gpsdata.attitude (class ATT), maybe gpsdata.imu (class IMU)
 */
void json_att_dump(const struct gps_data_t *gpsdata,
                   char *reply, size_t replylen,
                   const struct attitude_t *att, const char *class)
{
    (void)snprintf(reply, replylen, "{\"class\":\"%s\",\"device\":\"%s\"",
                   class, gpsdata->dev.path);

    if (0 < att->mtime.tv_sec) {
        char tbuf[JSON_DATE_MAX+1];

        str_appendf(reply, replylen, ",\"time\":\"%s\"",
                    timespec_to_iso8601(att->mtime, tbuf, sizeof(tbuf)));
    }
    if ('\0' != att->msg[0]) {
        str_appendf(reply, replylen, ",\"msg\":\"%.15s\"", att->msg);
    }
    if (0 != att->timeTag) {
        // yeah, a tiny chance the timeTag really is zero.
        str_appendf(reply, replylen, ",\"timeTag\":%lu", att->timeTag);
    }
    if (0 != isfinite(att->heading)) {
        // Trimble outputs %.3f, so we do too.
        str_appendf(reply, replylen, ",\"heading\":%.3f", att->heading);
        if ('\0' != att->mag_st) {
            str_appendf(reply, replylen, ",\"mag_st\":\"%c\"", att->mag_st);
        }
    }
    if (0 != isfinite(att->mheading)) {
        str_appendf(reply, replylen, ",\"mheading\":%.3f", att->mheading);
    }
    if (0 != isfinite(att->pitch)) {
        // pypilot reports %.3f
        str_appendf(reply, replylen, ",\"pitch\":%.3f", att->pitch);
        if ('\0' != att->pitch_st) {
            str_appendf(reply, replylen, ",\"pitch_st\":\"%c\"",
                        att->pitch_st);
        }
    }
    if (0 != isfinite(att->yaw)) {
        str_appendf(reply, replylen, ",\"yaw\":%.2f", att->yaw);
        if ('\0' != att->yaw_st) {
            str_appendf(reply, replylen, ",\"yaw_st\":\"%c\"", att->yaw_st);
        }
    }
    if (0 != isfinite(att->roll)) {
        // pypilot reports %.3f
        str_appendf(reply, replylen, ",\"roll\":%.3f", att->roll);
        if ('\0' != att->roll_st) {
            str_appendf(reply, replylen, ",\"roll_st\":\"%c\"", att->roll_st);
        }
    }
    if (0 != isfinite(att->rot)) {
        str_appendf(reply, replylen, ",\"rot\":%.3f", att->rot);
    }

    if (0 != isfinite(att->dip)) {
        str_appendf(reply, replylen, ",\"dip\":%.3f", att->dip);
    }

    if (0 != isfinite(att->mag_len)) {
        str_appendf(reply, replylen, ",\"mag_len\":%.3f", att->mag_len);
    }
    if (0 != isfinite(att->mag_x)) {
        str_appendf(reply, replylen, ",\"mag_x\":%.5f", att->mag_x);
    }
    if (0 != isfinite(att->mag_y)) {
        str_appendf(reply, replylen, ",\"mag_y\":%.5f", att->mag_y);
    }
    if (0 != isfinite(att->mag_z)) {
        str_appendf(reply, replylen, ",\"mag_z\":%.5f", att->mag_z);
    }

    if (0 != isfinite(att->acc_len)) {
        str_appendf(reply, replylen, ",\"acc_len\":%.5f", att->acc_len);
    }
    if (0 != isfinite(att->acc_x)) {
        str_appendf(reply, replylen, ",\"acc_x\":%.5f", att->acc_x);
    }
    if (0 != isfinite(att->acc_y)) {
        str_appendf(reply, replylen, ",\"acc_y\":%.5f", att->acc_y);
    }
    if (0 != isfinite(att->acc_z)) {
        str_appendf(reply, replylen, ",\"acc_z\":%.5f", att->acc_z);
    }

    if (0 != isfinite(att->gyro_temp)) {
        str_appendf(reply, replylen, ",\"gyro_temp\":%.2f", att->gyro_temp);
    }
    if (0 != isfinite(att->gyro_x)) {
        str_appendf(reply, replylen, ",\"gyro_x\":%.5f", att->gyro_x);
    }
    if (0 != isfinite(att->gyro_y)) {
        str_appendf(reply, replylen, ",\"gyro_y\":%.5f", att->gyro_y);
    }
    if (0 != isfinite(att->gyro_z)) {
        str_appendf(reply, replylen, ",\"gyro_z\":%.5f", att->gyro_z);
    }

    if (0 != isfinite(att->temp)) {
        str_appendf(reply, replylen, ",\"temp\":%.3f", att->temp);
    }
    if (0 != isfinite(att->depth)) {
        str_appendf(reply, replylen, ",\"depth\":%.3f", att->depth);
    }

    if (STATUS_UNK != att->base.status) {
        json_base_dump(&att->base, reply, replylen);
    }

    (void)strlcat(reply, "}\r\n", replylen);
}

#ifdef OSCILLATOR_ENABLE
// dump the contents of an oscillator_t structure as JSON
void json_oscillator_dump(const struct gps_data_t *datap,
                          char *reply, size_t replylen)
{
    (void)snprintf(reply, replylen,
                   "{\"class\":\"OSC\",\"device\":\"%s\",\"running\":%s,"
                   "\"reference\":%s,\"disciplined\":%s,\"delta\":%d}\r\n",
                   datap->dev.path,
                   JSON_BOOL(datap->osc.running),
                   JSON_BOOL(datap->osc.reference),
                   JSON_BOOL(datap->osc.disciplined),
                   datap->osc.delta);
}
#endif  // OSCILLATOR_ENABLE

// report a session state in JSON
void json_data_report(const gps_mask_t changed,
                      struct gps_device_t *session,
                      const struct gps_policy_t *policy,
                      char *buf, size_t buflen)
{
    struct gps_data_t *datap = &session->gpsdata;
    size_t buf_len;
    buf[0] = '\0';

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "json_data_report(%s) changed %s\n",
             session->gpsdata.dev.path, gps_maskdump(changed));

    if (0 != (changed & REPORT_IS)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);

        json_tpv_dump(changed, session, policy,
                      buf + buf_len, buflen - buf_len);
        // attitude is synchronous to epoch, so report like TPV.
        if (0 != (changed & ATTITUDE_SET)) {
            buf_len = strnlen(buf, MAX_PACKET_LENGTH);
            json_att_dump(datap, buf + buf_len, buflen - buf_len,
                          &datap->attitude, "ATT");
        }
    }

    if (0 != (changed & GST_SET)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_noise_dump(datap, buf + buf_len, buflen - buf_len);
    }

    if (0 != (changed & (DOP_SET | SATELLITE_SET))) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_sky_dump(session, buf + buf_len, buflen - buf_len);
    }

    if (0 != (changed & SUBFRAME_SET)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_subframe_dump(datap, policy->scaled, buf + buf_len,
                           buflen - buf_len);
    }

    if (0 != (changed & RAW_IS)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_raw_dump(datap, buf + buf_len, buflen - buf_len);
    }

    if (0 != (changed & IMU_SET)) {
        int max_imu, cur_imu = 0;

        max_imu = sizeof(datap->imu) / sizeof(struct attitude_t);
        for (cur_imu = 0; cur_imu < max_imu; cur_imu++ ) {
            if ('\0' == datap->imu[cur_imu].msg[0]) {
                break;
            }
            buf_len = strnlen(buf, MAX_PACKET_LENGTH);
            json_att_dump(datap, buf + buf_len, buflen - buf_len,
                          &datap->imu[cur_imu], "IMU");
        }
    }

    if (0 != (changed & RTCM2_SET)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_rtcm2_dump(&datap->rtcm2, datap->dev.path,
                        buf + buf_len, buflen - buf_len);
    }

    if (0 != (changed & RTCM3_SET)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_rtcm3_dump(&datap->rtcm3, datap->dev.path,
                        buf + buf_len, buflen - buf_len);
    }

#ifdef AIVDM_ENABLE
    if (0 != (changed & AIS_SET)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_aivdm_dump(&datap->ais, datap->dev.path,
                        policy->scaled,
                        buf + buf_len, buflen - buf_len);
    }
#endif  // AIVDM_ENABLE

#ifdef OSCILLATOR_ENABLE
    if (0 != (changed & OSCILLATOR_SET)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_oscillator_dump(datap, buf + buf_len, buflen - buf_len);
    }
#endif // OSCILLATOR_ENABLE
    if (0 != (changed & LOG_SET)) {
        buf_len = strnlen(buf, MAX_PACKET_LENGTH);
        json_log_dump(session, buf + buf_len, buflen - buf_len);
    }
}

#undef JSON_BOOL

// vim: set expandtab shiftwidth=4
