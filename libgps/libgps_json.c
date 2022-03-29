/****************************************************************************

NAME
   libgps_json.c - deserialize gpsd data coming from the server

DESCRIPTION
   This module uses the generic JSON parser to get data from JSON
representations to libgps structures.

PERMISSIONS
   Written by Eric S. Raymond, 2009
   This file is Copyright 2009 by the GPSD project
   SPDX-License-Identifier: BSD-2-clause

***************************************************************************/

/* isfinite() needs _POSIX_C_SOURCE >= 200112L
 * isnan(+Inf) is false, isfinite(+Inf) is false
 * use isfinite() to make sure a float or double is valid
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "../include/gpsd.h"
#include "../include/strfuncs.h"
#ifdef SOCKET_EXPORT_ENABLE
#include "../include/gps_json.h"
#include "../include/timespec.h"

static int json_tpv_read(const char *buf, struct gps_data_t *gpsdata,
                         const char **endptr)
{
    int ret;

    const struct json_attr_t json_attrs_1[] = {
        // *INDENT-OFF*
        {"class",  t_check,   .dflt.check = "TPV"},
        {"device", t_string,  .addr.string = gpsdata->dev.path,
                                 .len = sizeof(gpsdata->dev.path)},
        {"time",   t_time,    .addr.ts = &gpsdata->fix.time,
                                 .dflt.ts = {0, 0}},
        {"leapseconds",   t_integer, .addr.integer = &gpsdata->leap_seconds,
                                 .dflt.integer = 0},
        {"ept",    t_real,    .addr.real = &gpsdata->fix.ept,
                                 .dflt.real = NAN},
        {"lon",    t_real,    .addr.real = &gpsdata->fix.longitude,
                                 .dflt.real = NAN},
        {"lat",    t_real,    .addr.real = &gpsdata->fix.latitude,
                                 .dflt.real = NAN},
        {"alt",    t_real,    .addr.real = &gpsdata->fix.altitude,
                                 .dflt.real = NAN}, // DEPRECATED, undefined
        {"altHAE",    t_real,  .addr.real = &gpsdata->fix.altHAE,
                                 .dflt.real = NAN},
        {"altMSL", t_real,    .addr.real = &gpsdata->fix.altMSL,
                                 .dflt.real = NAN},
        {"datum",  t_string,  .addr.string = gpsdata->fix.datum,
                                 .len = sizeof(gpsdata->fix.datum)},
        {"epc",    t_real,    .addr.real = &gpsdata->fix.epc,
                                 .dflt.real = NAN},
        {"epd",    t_real,    .addr.real = &gpsdata->fix.epd,
                                 .dflt.real = NAN},
        {"eph",    t_real,    .addr.real = &gpsdata->fix.eph,
                                 .dflt.real = NAN},
        {"eps",    t_real,    .addr.real = &gpsdata->fix.eps,
                                 .dflt.real = NAN},
        {"epx",    t_real,    .addr.real = &gpsdata->fix.epx,
                                 .dflt.real = NAN},
        {"epy",    t_real,    .addr.real = &gpsdata->fix.epy,
                                 .dflt.real = NAN},
        {"epv",    t_real,    .addr.real = &gpsdata->fix.epv,
                                 .dflt.real = NAN},
        {"track",  t_real,    .addr.real = &gpsdata->fix.track,
                                 .dflt.real = NAN},
        {"magtrack",  t_real,    .addr.real = &gpsdata->fix.magnetic_track,
                                 .dflt.real = NAN},
        {"magvar",  t_real,   .addr.real = &gpsdata->fix.magnetic_var,
                                 .dflt.real = NAN},
        {"speed",  t_real,    .addr.real = &gpsdata->fix.speed,
                                 .dflt.real = NAN},
        {"climb",  t_real,    .addr.real = &gpsdata->fix.climb,
                                 .dflt.real = NAN},
        {"ecefx",  t_real,    .addr.real = &gpsdata->fix.ecef.x,
                                 .dflt.real = NAN},
        {"ecefy",  t_real,    .addr.real = &gpsdata->fix.ecef.y,
                                 .dflt.real = NAN},
        {"ecefz",  t_real,    .addr.real = &gpsdata->fix.ecef.z,
                                 .dflt.real = NAN},
        {"ecefvx", t_real,    .addr.real = &gpsdata->fix.ecef.vx,
                                 .dflt.real = NAN},
        {"ecefvy", t_real,    .addr.real = &gpsdata->fix.ecef.vy,
                                 .dflt.real = NAN},
        {"ecefvz", t_real,    .addr.real = &gpsdata->fix.ecef.vz,
                                 .dflt.real = NAN},
        {"ecefpAcc", t_real,  .addr.real = &gpsdata->fix.ecef.pAcc,
                                 .dflt.real = NAN},
        {"ecefvAcc", t_real,  .addr.real = &gpsdata->fix.ecef.vAcc,
                                 .dflt.real = NAN},
        {"mode",   t_integer, .addr.integer = &gpsdata->fix.mode,
                                 .dflt.integer = MODE_NOT_SEEN},
        {"sep",    t_real,    .addr.real = &gpsdata->fix.sep,
                                 .dflt.real = NAN},
        {"status", t_integer, .addr.integer = &gpsdata->fix.status,
                                 .dflt.integer = STATUS_UNK},
        {"relN", t_real,  .addr.real = &gpsdata->fix.NED.relPosN,
                                 .dflt.real = NAN},
        {"relE", t_real,  .addr.real = &gpsdata->fix.NED.relPosE,
                                 .dflt.real = NAN},
        {"relD", t_real,  .addr.real = &gpsdata->fix.NED.relPosD,
                                 .dflt.real = NAN},
        {"relH", t_real,  .addr.real = &gpsdata->fix.NED.relPosH,
                                 .dflt.real = NAN},
        {"relL", t_real,  .addr.real = &gpsdata->fix.NED.relPosL,
                                 .dflt.real = NAN},
        {"velN", t_real,  .addr.real = &gpsdata->fix.NED.velN,
                                 .dflt.real = NAN},
        {"velE", t_real,  .addr.real = &gpsdata->fix.NED.velE,
                                 .dflt.real = NAN},
        {"velD", t_real,  .addr.real = &gpsdata->fix.NED.velD,
                                 .dflt.real = NAN},
        {"geoidSep", t_real,  .addr.real = &gpsdata->fix.geoid_sep,
                                 .dflt.real = NAN},
        {"depth", t_real,  .addr.real = &gpsdata->fix.depth,
                                 .dflt.real = NAN},
        {"dgpsAge", t_real, .addr.real = &gpsdata->fix.dgps_age,
                                 .dflt.real = NAN},
        {"dgpsSta", t_integer, .addr.integer = &gpsdata->fix.dgps_station,
                                 .dflt.integer = -1},
        {"dgpsRatio", t_real, .addr.real = &gpsdata->fix.base.ratio,
                                 .dflt.real = NAN},
        {"wanglem", t_real, .addr.real = &gpsdata->fix.wanglem,
                                 .dflt.real = NAN},
        {"wangler", t_real, .addr.real = &gpsdata->fix.wangler,
                                 .dflt.real = NAN},
        {"wanglet", t_real, .addr.real = &gpsdata->fix.wanglet,
                                 .dflt.real = NAN},
        {"wspeedr", t_real, .addr.real = &gpsdata->fix.wspeedr,
                                 .dflt.real = NAN},
        {"wspeedt", t_real, .addr.real = &gpsdata->fix.wspeedt,
                                 .dflt.real = NAN},
        {"baseS",     t_integer,   .addr.integer = &gpsdata->fix.base.status,
         .dflt.integer = STATUS_UNK},     // aka zero
        {"baseE",     t_real,      .addr.real = &gpsdata->fix.base.east,
         .dflt.real = NAN},
        {"baseN",     t_real,      .addr.real = &gpsdata->fix.base.north,
         .dflt.real = NAN},
        {"baseU",     t_real,      .addr.real = &gpsdata->fix.base.up,
         .dflt.real = NAN},
        {"baseL",     t_real,      .addr.real = &gpsdata->fix.base.length,
         .dflt.real = NAN},
        {"baseC",     t_real,      .addr.real = &gpsdata->fix.base.course,
         .dflt.real = NAN},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},

        {NULL},
        // *INDENT-ON*
    };

    ret = json_read_object(buf, json_attrs_1, endptr);
    return ret;
}

static int json_noise_read(const char *buf, struct gps_data_t *gpsdata,
                           const char **endptr)
{
    int ret;

    const struct json_attr_t json_attrs_1[] = {
        // *INDENT-OFF*
        {"class",  t_check,   .dflt.check = "GST"},
        {"device", t_string,  .addr.string = gpsdata->dev.path,
                                 .len = sizeof(gpsdata->dev.path)},
        {"time",   t_time,    .addr.ts = &gpsdata->gst.utctime,
                                 .dflt.ts = {0, 0}},
        {"rms",    t_real,    .addr.real = &gpsdata->gst.rms_deviation,
                                 .dflt.real = NAN},
        {"major",  t_real,    .addr.real = &gpsdata->gst.smajor_deviation,
                                 .dflt.real = NAN},
        {"minor",  t_real,    .addr.real = &gpsdata->gst.sminor_deviation,
                                 .dflt.real = NAN},
        {"orient", t_real,    .addr.real = &gpsdata->gst.smajor_orientation,
                                 .dflt.real = NAN},
        {"lat",    t_real,    .addr.real = &gpsdata->gst.lat_err_deviation,
                                 .dflt.real = NAN},
        {"lon",    t_real,    .addr.real = &gpsdata->gst.lon_err_deviation,
                                 .dflt.real = NAN},
        {"alt",    t_real,    .addr.real = &gpsdata->gst.alt_err_deviation,
                                 .dflt.real = NAN},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };

    ret = json_read_object(buf, json_attrs_1, endptr);

    return ret;
}

// decode a RAW messages into gpsdata.raw
static int json_raw_read(const char *buf, struct gps_data_t *gpsdata,
                         const char **endptr)
{
    int measurements;
    // initialized to shut up clang
    double mtime_s = 0.0, mtime_ns = 0.0;

    const struct json_attr_t json_attrs_meas[] = {
        // *INDENT-OFF*
        {"gnssid",       t_ubyte,    STRUCTOBJECT(struct meas_t, gnssid)},
        {"svid",         t_ubyte,    STRUCTOBJECT(struct meas_t, svid)},
        {"sigid",        t_ubyte,    STRUCTOBJECT(struct meas_t, sigid),
                                     .dflt.ubyte = 0},
        {"snr",          t_ubyte,    STRUCTOBJECT(struct meas_t, snr)},
        {"freqid",       t_ubyte,    STRUCTOBJECT(struct meas_t, freqid),
                                     .dflt.ubyte = 0},
        {"obs",          t_string,   STRUCTOBJECT(struct meas_t, obs_code),
                                .len = sizeof(gpsdata->raw.meas[0].obs_code)},
        {"lli",          t_ubyte,    STRUCTOBJECT(struct meas_t, lli),
                                     .dflt.ubyte = 0},
        {"locktime",     t_uinteger, STRUCTOBJECT(struct meas_t, locktime),
                                     .dflt.uinteger = 0},
        {"carrierphase", t_real,     STRUCTOBJECT(struct meas_t, carrierphase),
                                     .dflt.real = NAN},
        {"pseudorange",  t_real,     STRUCTOBJECT(struct meas_t, pseudorange),
                                     .dflt.real = NAN},
        {"doppler",      t_real,     STRUCTOBJECT(struct meas_t, doppler),
                                     .dflt.real = NAN},
        {"c2c",          t_real,     STRUCTOBJECT(struct meas_t, c2c),
                                    .dflt.real = NAN},
        {"l2c",          t_real,     STRUCTOBJECT(struct meas_t, l2c),
                                    .dflt.real = NAN},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        // *INDENT-ON*
        {NULL},
    };
    const struct json_attr_t json_attrs_raw[] = {
        // *INDENT-OFF*
        {"class",      t_check,   .dflt.check = "RAW"},
        {"device",     t_string,  .addr.string  = gpsdata->dev.path,
                                    .len = sizeof(gpsdata->dev.path)},
        {"time",       t_real,    .addr.real = &mtime_s,
                                 .dflt.real = NAN},
        {"nsec",       t_real,    .addr.real = &mtime_ns,
                                 .dflt.real = NAN},
        {"rawdata",    t_array,   STRUCTARRAY(gpsdata->raw.meas,
                                     json_attrs_meas, &measurements)},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };
    int status;

    memset(&gpsdata->raw, 0, sizeof(gpsdata->raw));

    status = json_read_object(buf, json_attrs_raw, endptr);
    if (status != 0)
        return status;
    if (0 == isfinite(mtime_s) || 0 == isfinite(mtime_ns))
        return status;
    gpsdata->raw.mtime.tv_sec = (time_t)mtime_s;
    gpsdata->raw.mtime.tv_nsec = (long)mtime_ns;

    return 0;
}

static int json_sky_read(const char *buf, struct gps_data_t *gpsdata,
                         const char **endptr)
{

    const struct json_attr_t json_attrs_satellites[] = {
        // *INDENT-OFF*
        {"PRN",    t_short,   STRUCTOBJECT(struct satellite_t, PRN)},
        {"el",     t_real,    STRUCTOBJECT(struct satellite_t, elevation),
                              .dflt.real = NAN},
        {"az",     t_real,    STRUCTOBJECT(struct satellite_t, azimuth),
                              .dflt.real = NAN},
        {"ss",     t_real,    STRUCTOBJECT(struct satellite_t, ss),
                              .dflt.real = NAN},
        {"used",   t_boolean, STRUCTOBJECT(struct satellite_t, used)},
        {"gnssid", t_ubyte,   STRUCTOBJECT(struct satellite_t, gnssid)},
        {"svid",   t_ubyte,   STRUCTOBJECT(struct satellite_t, svid)},
        {"sigid",  t_ubyte,   STRUCTOBJECT(struct satellite_t, sigid)},
        {"freqid", t_byte,    STRUCTOBJECT(struct satellite_t, freqid),
                              .dflt.byte = -1},
        {"health", t_ubyte,   STRUCTOBJECT(struct satellite_t, health),
                              .dflt.ubyte = SAT_HEALTH_UNK},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        // *INDENT-ON*
        {NULL},
    };

    int nSat = -1;  // Use nSat only to know if sats are in SKY

    const struct json_attr_t json_attrs_2[] = {
        // *INDENT-OFF*
        {"class",      t_check,   .dflt.check = "SKY"},
        {"device",     t_string,  .addr.string  = gpsdata->dev.path,
                                     .len = sizeof(gpsdata->dev.path)},
        {"time",       t_time,    .addr.ts = &gpsdata->skyview_time,
                                     .dflt.ts = {0, 0}},
        {"hdop",       t_real,    .addr.real    = &gpsdata->dop.hdop,
                                     .dflt.real = NAN},
        {"xdop",       t_real,    .addr.real    = &gpsdata->dop.xdop,
                                     .dflt.real = NAN},
        {"ydop",       t_real,    .addr.real    = &gpsdata->dop.ydop,
                                     .dflt.real = NAN},
        {"vdop",       t_real,    .addr.real    = &gpsdata->dop.vdop,
                                     .dflt.real = NAN},
        {"tdop",       t_real,    .addr.real    = &gpsdata->dop.tdop,
                                     .dflt.real = NAN},
        {"pdop",       t_real,    .addr.real    = &gpsdata->dop.pdop,
                                     .dflt.real = NAN},
        {"gdop",       t_real,    .addr.real    = &gpsdata->dop.gdop,
                                     .dflt.real = NAN},
        {"nSat",       t_integer, .addr.integer = &nSat,
                                     .dflt.integer = -1},
        {"satellites", t_array,
                                   STRUCTARRAY(gpsdata->skyview,
                                         json_attrs_satellites,
                                         &gpsdata->satellites_visible)},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };
    int status, i;

    memset(&gpsdata->skyview, 0, sizeof(gpsdata->skyview));

    status = json_read_object(buf, json_attrs_2, endptr);
    if (0 != status) {
        return status;
    }

    if (1 == isfinite(gpsdata->dop.hdop) ||
        1 == isfinite(gpsdata->dop.xdop) ||
        1 == isfinite(gpsdata->dop.ydop) ||
        1 == isfinite(gpsdata->dop.vdop) ||
        1 == isfinite(gpsdata->dop.tdop) ||
        1 == isfinite(gpsdata->dop.pdop) ||
        1 == isfinite(gpsdata->dop.gdop)) {
        // got at least one DOP
        gpsdata->set |= DOP_SET;
    }

    gpsdata->satellites_used = 0;
    gpsdata->satellites_visible = 0;

    if (-1 == nSat) {
        // no sats in the SKY, likely just dops.
        gpsdata->set &= ~SATELLITE_SET;
        return 0;
    }

    gpsdata->set |= SATELLITE_SET;
    // recalculate used and visible, do not use nSat, uSat
    for (i = 0; i < MAXCHANNELS; i++) {
        if (0 < gpsdata->skyview[i].PRN) {
            gpsdata->satellites_visible++;
        }
        if (gpsdata->skyview[i].used) {
            gpsdata->satellites_used++;
        }
    }

    return 0;
}

// decode class ATT, almost the same as IMU
static int json_att_read(const char *buf, struct gps_data_t *gpsdata,
                         const char **endptr)
{
    struct attitude_t *datap = &gpsdata->attitude;

    const struct json_attr_t json_attrs_1[] = {
        // *INDENT-OFF*
        {"class",     t_check,     .dflt.check = "ATT"},
        {"device",    t_string,    .addr.string = gpsdata->dev.path,
         .len = sizeof(gpsdata->dev.path)},
        {"acc_len",   t_real,      .addr.real = &datap->acc_len,
         .dflt.real = NAN},
        {"acc_x",     t_real,      .addr.real = &datap->acc_x,
         .dflt.real = NAN},
        {"acc_y",     t_real,      .addr.real = &datap->acc_y,
         .dflt.real = NAN},
        {"acc_z",     t_real,      .addr.real = &datap->acc_z,
         .dflt.real = NAN},
        {"baseS",     t_integer,   .addr.integer = &datap->base.status,
         .dflt.integer = STATUS_UNK},     // aka zero
        {"baseE",     t_real,      .addr.real = &datap->base.east,
         .dflt.real = NAN},
        {"baseN",     t_real,      .addr.real = &datap->base.north,
         .dflt.real = NAN},
        {"baseU",     t_real,      .addr.real = &datap->base.up,
         .dflt.real = NAN},
        {"baseL",     t_real,      .addr.real = &datap->base.length,
         .dflt.real = NAN},
        {"baseC",     t_real,      .addr.real = &datap->base.course,
         .dflt.real = NAN},
        {"dgpsRatio", t_real, .addr.real = &datap->base.ratio,
         .dflt.real = NAN},
        {"depth",     t_real,      .addr.real = &datap->depth,
         .dflt.real = NAN},
        {"dip",       t_real,      .addr.real = &datap->dip,
         .dflt.real = NAN},
        {"gyro_temp", t_real,      .addr.real = &datap->gyro_x,
         .dflt.real = NAN},
        {"gyro_x",    t_real,      .addr.real = &datap->gyro_x,
         .dflt.real = NAN},
        {"gyro_y",    t_real,      .addr.real = &datap->gyro_y,
         .dflt.real = NAN},
        {"gyro_z",    t_real,      .addr.real = &datap->gyro_z,
         .dflt.real = NAN},
        {"heading",   t_real,      .addr.real = &datap->heading,
         .dflt.real = NAN},
        {"mag_len",   t_real,      .addr.real = &datap->mag_len,
         .dflt.real = NAN},
        {"mag_st",    t_character, .addr.character = &datap->mag_st},
        {"mag_x",     t_real,      .addr.real = &datap->mag_x,
         .dflt.real = NAN},
        {"mag_y",     t_real,      .addr.real = &datap->mag_y,
         .dflt.real = NAN},
        {"mag_z",     t_real,      .addr.real = &datap->mag_z,
         .dflt.real = NAN},
        {"msg",       t_string,    .addr.string = datap->msg,
         .len = sizeof(datap->msg)},
        {"pitch_st",  t_character, .addr.character = &datap->pitch_st},
        {"pitch",     t_real,      .addr.real = &datap->pitch,
         .dflt.real = NAN},
        {"roll_st",   t_character, .addr.character = &datap->roll_st},
        {"roll",      t_real,      .addr.real = &datap->roll,
         .dflt.real = NAN},
        {"temp",      t_real,      .addr.real = &datap->temp,
         .dflt.real = NAN},
        {"time",      t_time,      .addr.ts = &datap->mtime, .dflt.ts = {0, 0}},
        {"timeTag",   t_ulongint,  .addr.ulongint = &datap->timeTag,
         .dflt.ulongint = 0},
        {"yaw_st",    t_character, .addr.character = &datap->yaw_st},
        {"yaw",       t_real,      .addr.real = &datap->yaw, .dflt.real = NAN},

        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };

    return json_read_object(buf, json_attrs_1, endptr);
}

// decode class IMU, almost the ame as ATT
static int json_imu_read(const char *buf, struct gps_data_t *gpsdata,
                         const char **endptr)
{
    // the client only uses the first slot.
    struct attitude_t *datap = &gpsdata->imu[0];

    const struct json_attr_t json_attrs_1[] = {
        // *INDENT-OFF*
        {"class",     t_check,     .dflt.check = "IMU"},
        {"device",    t_string,    .addr.string = gpsdata->dev.path,
         .len = sizeof(gpsdata->dev.path)},
        {"acc_len",   t_real,      .addr.real = &datap->acc_len,
         .dflt.real = NAN},
        {"acc_x",     t_real,      .addr.real = &datap->acc_x,
         .dflt.real = NAN},
        {"acc_y",     t_real,      .addr.real = &datap->acc_y,
         .dflt.real = NAN},
        {"acc_z",     t_real,      .addr.real = &datap->acc_z,
         .dflt.real = NAN},
        {"depth",     t_real,      .addr.real = &datap->depth,
         .dflt.real = NAN},
        {"dip",       t_real,      .addr.real = &datap->dip,
         .dflt.real = NAN},
        {"gyro_temp", t_real,      .addr.real = &datap->gyro_temp,
         .dflt.real = NAN},
        {"gyro_x",    t_real,      .addr.real = &datap->gyro_x,
         .dflt.real = NAN},
        {"gyro_y",    t_real,      .addr.real = &datap->gyro_y,
         .dflt.real = NAN},
        {"gyro_z",    t_real,      .addr.real = &datap->gyro_z,
         .dflt.real = NAN},
        {"heading",   t_real,      .addr.real = &datap->heading,
         .dflt.real = NAN},
        {"mag_len",   t_real,      .addr.real = &datap->mag_len,
         .dflt.real = NAN},
        {"mag_st",    t_character, .addr.character = &datap->mag_st},
        {"mag_x",     t_real,      .addr.real = &datap->mag_x,
         .dflt.real = NAN},
        {"mag_y",     t_real,      .addr.real = &datap->mag_y,
         .dflt.real = NAN},
        {"mag_z",     t_real,      .addr.real = &datap->mag_z,
         .dflt.real = NAN},
        {"msg",       t_string,    .addr.string = datap->msg,
         .len = sizeof(datap->msg)},
        {"pitch_st",  t_character, .addr.character = &datap->pitch_st},
        {"pitch",     t_real,      .addr.real = &datap->pitch,
         .dflt.real = NAN},
        {"roll_st",   t_character, .addr.character = &datap->roll_st},
        {"roll",      t_real,      .addr.real = &datap->roll,
         .dflt.real = NAN},
        {"temp",      t_real,      .addr.real = &datap->temp,
         .dflt.real = NAN},
        {"time",      t_time,      .addr.ts = &datap->mtime, .dflt.ts = {0, 0}},
        {"timeTag",   t_ulongint,  .addr.ulongint = &datap->timeTag,
         .dflt.ulongint = 0},
        {"yaw_st",    t_character, .addr.character = &datap->yaw_st},
        {"yaw",       t_real,      .addr.real = &datap->yaw, .dflt.real = NAN},

        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };

    return json_read_object(buf, json_attrs_1, endptr);
}

static int json_devicelist_read(const char *buf, struct gps_data_t *gpsdata,
                                const char **endptr)
{
    const struct json_attr_t json_attrs_subdevices[] = {
        // *INDENT-OFF*
        {"class",      t_check,      .dflt.check = "DEVICE"},
        {"path",       t_string,     STRUCTOBJECT(struct devconfig_t, path),
         .len = sizeof(gpsdata->devices.list[0].path)},
        {"activated",  t_time,
         STRUCTOBJECT(struct devconfig_t, activated)},
        {"activated",  t_real,
         STRUCTOBJECT(struct devconfig_t, activated)},
        {"flags",      t_integer,    STRUCTOBJECT(struct devconfig_t, flags)},
        {"driver",     t_string,     STRUCTOBJECT(struct devconfig_t, driver),
         .len = sizeof(gpsdata->devices.list[0].driver)},
        {"hexdata",    t_string,     STRUCTOBJECT(struct devconfig_t, hexdata),
         .len = sizeof(gpsdata->devices.list[0].hexdata)},
        {"subtype",    t_string,     STRUCTOBJECT(struct devconfig_t, subtype),
                            .len = sizeof(gpsdata->devices.list[0].subtype)},
        {"subtype1",   t_string,     STRUCTOBJECT(struct devconfig_t, subtype1),
                            .len = sizeof(gpsdata->devices.list[0].subtype1)},
        {"native",     t_integer, STRUCTOBJECT(struct devconfig_t, driver_mode),
                                        .dflt.integer = -1},
        {"bps",        t_uinteger,   STRUCTOBJECT(struct devconfig_t, baudrate),
                                        .dflt.uinteger = DEVDEFAULT_BPS},
        {"parity",     t_character,  STRUCTOBJECT(struct devconfig_t, parity),
                                        .dflt.character = DEVDEFAULT_PARITY},
        {"stopbits",   t_uinteger,   STRUCTOBJECT(struct devconfig_t, stopbits),
                                        .dflt.integer = DEVDEFAULT_STOPBITS},
        {"cycle",      t_timespec,   STRUCTOBJECT(struct devconfig_t, cycle),
                                        .dflt.ts = {0,0}},
        {"mincycle",   t_timespec,   STRUCTOBJECT(struct devconfig_t, mincycle),
                                        .dflt.ts = {0,0}},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };
    const struct json_attr_t json_attrs_devices[] = {
        {"class", t_check,.dflt.check = "DEVICES"},
        {"devices", t_array, STRUCTARRAY(gpsdata->devices.list,
                                         json_attrs_subdevices,
                                         &gpsdata->devices.ndevices)},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
    };
    int status;

    memset(&gpsdata->devices, 0, sizeof(gpsdata->devices));
    status = json_read_object(buf, json_attrs_devices, endptr);
    if (status != 0) {
        return status;
    }

    (void)clock_gettime(CLOCK_REALTIME, &gpsdata->devices.time);
    return 0;
}

static int json_version_read(const char *buf, struct gps_data_t *gpsdata,
                             const char **endptr)
{
    const struct json_attr_t json_attrs_version[] = {
        // *INDENT-OFF*
        {"class",     t_check,   .dflt.check = "VERSION"},
        {"release",   t_string,  .addr.string  = gpsdata->version.release,
                                    .len = sizeof(gpsdata->version.release)},
        {"rev",       t_string,  .addr.string  = gpsdata->version.rev,
                                    .len = sizeof(gpsdata->version.rev)},
        {"proto_major", t_integer,
         .addr.integer = &gpsdata->version.proto_major},
        {"proto_minor", t_integer,
         .addr.integer = &gpsdata->version.proto_minor},
        {"remote",    t_string,  .addr.string  = gpsdata->version.remote,
                                    .len = sizeof(gpsdata->version.remote)},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };
    int status;

    memset(&gpsdata->version, 0, sizeof(gpsdata->version));
    status = json_read_object(buf, json_attrs_version, endptr);

    return status;
}

static int json_error_read(const char *buf, struct gps_data_t *gpsdata,
                           const char **endptr)
{
    const struct json_attr_t json_attrs_error[] = {
        // *INDENT-OFF*
        {"class",     t_check,   .dflt.check = "ERROR"},
        {"message",   t_string,  .addr.string  = gpsdata->error,
                                    .len = sizeof(gpsdata->error)},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };
    int status;

    memset(&gpsdata->error, 0, sizeof(gpsdata->error));
    status = json_read_object(buf, json_attrs_error, endptr);
    if (status != 0)
        return status;

    return status;
}

int json_toff_read(const char *buf, struct gps_data_t *gpsdata,
                           const char **endptr)
{
    int real_sec = 0, real_nsec = 0, clock_sec = 0, clock_nsec = 0;
    const struct json_attr_t json_attrs_toff[] = {
        // *INDENT-OFF*
        {"class",     t_check,   .dflt.check = "TOFF"},
        {"device",    t_string,  .addr.string = gpsdata->dev.path,
                                 .len = sizeof(gpsdata->dev.path)},
        {"real_sec",  t_integer, .addr.integer = &real_sec,
                                 .dflt.integer = 0},
        {"real_nsec", t_integer, .addr.integer = &real_nsec,
                                 .dflt.integer = 0},
        {"clock_sec", t_integer, .addr.integer = &clock_sec,
                                 .dflt.integer = 0},
        {"clock_nsec",t_integer, .addr.integer = &clock_nsec,
                                 .dflt.integer = 0},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };
    int status;

    memset(&gpsdata->toff, 0, sizeof(gpsdata->toff));
    status = json_read_object(buf, json_attrs_toff, endptr);
    gpsdata->toff.real.tv_sec = (time_t)real_sec;
    gpsdata->toff.real.tv_nsec = (long)real_nsec;
    gpsdata->toff.clock.tv_sec = (time_t)clock_sec;
    gpsdata->toff.clock.tv_nsec = (long)clock_nsec;
    if (status != 0)
        return status;

    return status;
}

int json_pps_read(const char *buf, struct gps_data_t *gpsdata,
                  const char **endptr)
{
    int real_sec = 0, real_nsec = 0, clock_sec = 0, clock_nsec = 0, precision=0;
    int qErr = 0;

    const struct json_attr_t json_attrs_pps[] = {
        // *INDENT-OFF*
        {"class",     t_check,   .dflt.check = "PPS"},
        {"device",    t_string,  .addr.string = gpsdata->dev.path,
                                 .len = sizeof(gpsdata->dev.path)},
        {"real_sec",  t_integer, .addr.integer = &real_sec,
                                 .dflt.integer = 0},
        {"real_nsec", t_integer, .addr.integer = &real_nsec,
                                 .dflt.integer = 0},
        {"clock_sec", t_integer, .addr.integer = &clock_sec,
                                 .dflt.integer = 0},
        {"clock_nsec",t_integer, .addr.integer = &clock_nsec,
                                 .dflt.integer = 0},
        {"precision", t_integer, .addr.integer = &precision,
                                 .dflt.integer = 0},
        {"qErr", t_integer, .addr.integer = &qErr,
                                 .dflt.integer = 0},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };
    int status;

    memset(&gpsdata->pps, 0, sizeof(gpsdata->pps));
    status = json_read_object(buf, json_attrs_pps, endptr);

    // This is good until GPS are more than nanosec accurate
    gpsdata->pps.real.tv_sec = (time_t)real_sec;
    gpsdata->pps.real.tv_nsec = (long)real_nsec;
    gpsdata->pps.clock.tv_sec = (time_t)clock_sec;
    gpsdata->pps.clock.tv_nsec = (long)clock_nsec;
    // hope qErr fits in int
    gpsdata->qErr = (long)qErr;

    // FIXME: precision is currently parsed but discarded
    return status;
}

int json_oscillator_read(const char *buf, struct gps_data_t *gpsdata,
                         const char **endptr)
{
    bool running = false, reference = false, disciplined = false;
    int delta = 0;
    const struct json_attr_t json_attrs_osc[] = {
        // *INDENT-OFF*
        {"class",       t_check,   .dflt.check = "OSC"},
        {"device",      t_string,  .addr.string = gpsdata->dev.path,
                                   .len = sizeof(gpsdata->dev.path)},
        {"running",     t_boolean, .addr.boolean = &running,
                                   .dflt.boolean = false},
        {"reference",   t_boolean, .addr.boolean = &reference,
                                   .dflt.boolean = false},
        {"disciplined", t_boolean, .addr.boolean = &disciplined,
                                   .dflt.boolean = false},
        {"delta",       t_integer, .addr.integer = &delta,
                                   .dflt.integer = 0},
        // ignore unknown keys, for cross-version compatibility
        {"", t_ignore},
        {NULL},
        // *INDENT-ON*
    };
    int status;

    memset(&gpsdata->osc, 0, sizeof(gpsdata->osc));
    status = json_read_object(buf, json_attrs_osc, endptr);

    gpsdata->osc.running = running;
    gpsdata->osc.reference = reference;
    gpsdata->osc.disciplined = disciplined;
    gpsdata->osc.delta = delta;

    return status;
}

// Test for JSON read status values that should be treated as a go-ahead
// for further processing.  JSON_BADATTR - to allow JSON attributes unknown
// to this version of the library, for forward compatibility, is an obvious
// thing to go here.
#define PASS(n) (((n) == 0) || ((n) == JSON_ERR_BADATTR))
#define FILTER(n) ((n) == JSON_ERR_BADATTR ? 0 : n)

// the only entry point - unpack a JSON object into gpsdata_t substructures
int libgps_json_unpack(const char *buf,
                       struct gps_data_t *gpsdata, const char **end)
{
    int status;
    char *classtag = strstr(buf, "\"class\":");

    if (NULL == classtag) {
        return -1;
    }

    if (str_starts_with(classtag, "\"class\":\"TPV\"")) {
        status = json_tpv_read(buf, gpsdata, end);
        gpsdata->set = STATUS_SET;
        if (0 != gpsdata->fix.time.tv_sec) {
            gpsdata->set |= TIME_SET;
        }
        if (0 != isfinite(gpsdata->fix.ept)) {
            gpsdata->set |= TIMERR_SET;
        }
        if (0 != isfinite(gpsdata->fix.longitude)) {
            gpsdata->set |= LATLON_SET;
        }
        if (0 != isfinite(gpsdata->fix.altitude) ||
            0 != isfinite(gpsdata->fix.altHAE) ||
            0 != isfinite(gpsdata->fix.depth) ||
            0 != isfinite(gpsdata->fix.altMSL)) {
            gpsdata->set |= ALTITUDE_SET;
        }
        if (0 != isfinite(gpsdata->fix.epx) &&
            0 != isfinite(gpsdata->fix.epy)) {
            gpsdata->set |= HERR_SET;
        }
        if (0 != isfinite(gpsdata->fix.epv)) {
            gpsdata->set |= VERR_SET;
        }
        if (0 != isfinite(gpsdata->fix.track)) {
            gpsdata->set |= TRACK_SET;
        }
        if (0 != isfinite(gpsdata->fix.magnetic_track) ||
            0 != isfinite(gpsdata->fix.magnetic_var)) {
            gpsdata->set |= MAGNETIC_TRACK_SET;
        }
        if (0 != isfinite(gpsdata->fix.speed)) {
            gpsdata->set |= SPEED_SET;
        }
        if (0 != isfinite(gpsdata->fix.climb)) {
            gpsdata->set |= CLIMB_SET;
        }
        if (0 != isfinite(gpsdata->fix.epd)) {
            gpsdata->set |= TRACKERR_SET;
        }
        if (0 != isfinite(gpsdata->fix.eps)) {
            gpsdata->set |= SPEEDERR_SET;
        }
        if (0 != isfinite(gpsdata->fix.epc)) {
            gpsdata->set |= CLIMBERR_SET;
        }
        if (MODE_NOT_SEEN != gpsdata->fix.mode) {
            gpsdata->set |= MODE_SET;
        }
        if (0 != isfinite(gpsdata->fix.wanglem) ||
            0 != isfinite(gpsdata->fix.wangler) ||
            0 != isfinite(gpsdata->fix.wanglet) ||
            0 != isfinite(gpsdata->fix.wspeedr) ||
            0 != isfinite(gpsdata->fix.wspeedt)) {
            gpsdata->set |= NAVDATA_SET;
        }
        if (0 != isfinite(gpsdata->fix.NED.relPosN) ||
            0 != isfinite(gpsdata->fix.NED.relPosE) ||
            0 != isfinite(gpsdata->fix.NED.relPosD) ||
            0 != isfinite(gpsdata->fix.NED.relPosH) ||
            0 != isfinite(gpsdata->fix.NED.relPosL) ||
            0 != isfinite(gpsdata->fix.NED.velN) ||
            0 != isfinite(gpsdata->fix.NED.velE) ||
            0 != isfinite(gpsdata->fix.NED.velD)) {
            gpsdata->set |= NED_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"GST\"")) {
        status = json_noise_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= GST_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"SKY\"")) {
        status = json_sky_read(buf, gpsdata, end);
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"ATT\"")) {
        status = json_att_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set |= ATTITUDE_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"IMU\"")) {
        status = json_imu_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set |= IMU_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"DEVICES\"")) {
        status = json_devicelist_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= DEVICELIST_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"DEVICE\"")) {
        status = json_device_read(buf, &gpsdata->dev, end);
        if (PASS(status))
            gpsdata->set |= DEVICE_SET;
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"WATCH\"")) {
        status = json_watch_read(buf, &gpsdata->policy, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= POLICY_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"VERSION\"")) {
        status = json_version_read(buf, gpsdata, end);
        if (status ==  0) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= VERSION_SET;
        }
        return FILTER(status);
    }
#ifdef RTCM104V2_ENABLE
    if (str_starts_with(classtag, "\"class\":\"RTCM2\"")) {
        status = json_rtcm2_read(buf,
                                 gpsdata->dev.path, sizeof(gpsdata->dev.path),
                                 &gpsdata->rtcm2, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= RTCM2_SET;
        }
        return FILTER(status);
    }
#endif  // RTCM104V2_ENABLE
#ifdef RTCM104V3_ENABLE
    if (str_starts_with(classtag, "\"class\":\"RTCM3\"")) {
        status = json_rtcm3_read(buf,
                                 gpsdata->dev.path, sizeof(gpsdata->dev.path),
                                 &gpsdata->rtcm3, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= RTCM3_SET;
        }
        return FILTER(status);
    }
#endif  // RTCM104V3_ENABLE
#ifdef AIVDM_ENABLE
    if (str_starts_with(classtag, "\"class\":\"AIS\"")) {
        status = json_ais_read(buf,
                               gpsdata->dev.path, sizeof(gpsdata->dev.path),
                               &gpsdata->ais, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= AIS_SET;
        }
        return FILTER(status);
    }
#endif  // AIVDM_ENABLE
    if (str_starts_with(classtag, "\"class\":\"ERROR\"")) {
        status = json_error_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= ERROR_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"TOFF\"")) {
        status = json_pps_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= TOFF_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"PPS\"")) {
        status = json_pps_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= PPS_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"OSC\"")) {
        status = json_oscillator_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= OSCILLATOR_SET;
        }
        return FILTER(status);
    }
    if (str_starts_with(classtag, "\"class\":\"RAW\"")) {
        status = json_raw_read(buf, gpsdata, end);
        if (PASS(status)) {
            gpsdata->set &= ~UNION_SET;
            gpsdata->set |= RAW_SET;
        }
        return FILTER(status);
    }
    // else, unkwown class type
    return -1;
}


#endif  // SOCKET_EXPORT_ENABLE

// vim: set expandtab shiftwidth=4
