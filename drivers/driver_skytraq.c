/*
 * This is the gpsd driver for Skytraq GPSes operating in binary mode.
 *
 * SkyTraq is Big Endian
 *
 * This file is Copyright 2016 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>       // for strlcpy()
#include <strings.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/bits.h"
#include "../include/strfuncs.h"
#if defined(SKYTRAQ_ENABLE)
#include "../include/timespec.h"

#define HI(n)           ((n) >> 8)
#define LO(n)           ((n) & 0xff)

/*
 * No ACK/NAK?  Just retry after 6 seconds
 */
#define SKY_RETRY_TIME  6
// Phoenix has 230 channels
#define SKY_CHANNELS    230      // max channels allowed in format

#ifdef __UNUSED
// Poll Software Version MID 2
static unsigned char versionprobe[] = {
    0xa0, 0xa1, 0x00, 0x02,
    0x02,               // MID 2
    0x01,               // System
    0x00, 0x0d, 0x0a
};
#endif  // __UNUSED

/* place checksum into msg, write to device
 * Return: number of bytes written
 *         negative on error
 */
static ssize_t sky_write(struct gps_device_t *session, char *msg,
                         size_t data_len)
{
    uint8_t chk;
    uint16_t len;
    ssize_t i;
    bool ok;
    unsigned type = (unsigned)msg[4];
    uint8_t buf[BUFSIZ];
    uint8_t outbuf[BUFSIZ];

    // do not write if -b (readonly) option set
    // "passive" handled earlier
    if (session->context->readonly) {
        return data_len;
    }

    if (sizeof(buf) <= data_len) {
        // uh, oh;
        return -1;
    }
    // make a copy, so we can edit it
    memcpy(buf, msg, data_len);

    // max length is undocumented, largest I could find is 261
    len = (buf[2] << 8) | buf[3];
    // limit to 512 to pacify coverity
    if (512 < len) {
        len = 512;
    }
    if ((size_t)(len + 7) != data_len) {
        // uh, oh;
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "Skytraq: Length error: len %u data_len %zu buf %s\n",
                 len, data_len,
                 gpsd_hexdump((char *)outbuf, sizeof(outbuf),
                              (char *)buf, data_len));
        return -2;
    }

    // calculate Checksum
    chk = 0;
    // coverity_submit[tainted_data]
    for (i = 0; i < len; i++) {
        chk ^= buf[4 + i];
    }

    // enter checksum after payload
    buf[len + 4] = chk;
    len += 7;

    GPSD_LOG(LOG_IO, &session->context->errout,
             "Skytraq: Writing control MID %02x: %s\n", type,
             gpsd_hexdump((char *)outbuf, sizeof(outbuf), (char *)buf, len));
    ok = gpsd_write(session, (const char *)buf, len) == len;

    return ok;
}

/* stub for mode changer, someday
 * need it to make driver sticky
 */
static void sky_mode(struct gps_device_t *session UNUSED, int mode)
{
    if (MODE_BINARY == mode) {
    } else {
        // MODE_NMEA
    }
    return;
}

/*
 * Convert PRN to gnssid and svid
 */
static void PRN2_gnssId_svId(short PRN, uint8_t *gnssId, uint8_t *svId)
{
    // fit into gnssid:svid
    if (0 == PRN) {
        // skip 0 PRN
        *gnssId = 0;
        *svId = 0;
    } else if ((1 <= PRN) &&
               (32 >= PRN)) {
        // GPS
        *gnssId = 0;
        *svId = PRN;
    } else if ((65 <= PRN) &&
               (96 >= PRN)) {
        // GLONASS
        *gnssId = 6;
        *svId = PRN - 64;
    } else if ((120 <= PRN) &&
               (158 >= PRN)) {
        // SBAS
        *gnssId = 1;
        *svId = PRN;
    } else if ((201 <= PRN) &&
               (239 >= PRN)) {
        // BeiDou
        *gnssId = 3;
        *svId = PRN - 200;
    } else if ((240 <= PRN) &&
               (254 >= PRN)) {
        // IRNSS
        *gnssId = 20;
        *svId = PRN - 240;
    } else {
        // huh?
        *gnssId = 0;
        *svId = 0;
    }
    return;
}

/*
 decode MID 0x62 -- super packet
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_62(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned sid;
    unsigned u[23];
    int i;

    if (3 > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x62: bad len %zu\n", len);
        return 0;
    }

    sid = getub(buf, 1);
    switch (sid) {
    case  0x80:
        // SBAS status
        for (i = 0; i < 6; i++) {
            u[i] = getub(buf, i + 2);
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x62/80: enable %u ranging %u URA mask %u "
                 "correction %u chans %u subsystems %u \n",
                 u[0], u[1], u[2], u[3], u[4], u[5]);
        break;
    case  0x81:
        // QXSS status
        u[0] = getub(buf, 2);
        u[1] = getub(buf, 3);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x62/81: enable %u chans %u\n",
                 u[0], u[1]);
        break;
    case  0x82:
        // SBAS advanced status
        for (i = 0; i < 22; i++) {
            u[i] = getub(buf, i + 2);
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x62/82: enable %u ranging %u URA %u corr %u "
                 "chans %u mask x%02x WAAS %u %u %u %u "
                 "EGNOS %u %u %u %u MSAS %u %u %u %u "
                 "GAGAN %u %u %u %u\n",
                 u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], u[8],
                 u[9], u[10], u[11], u[12], u[13], u[14], u[15], u[16], u[17],
                 u[18], u[19], u[20], u[21]);
        break;
    default:
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x62: SID x%02x len %zu\n", sid, len);
    }
    return 0;
}

/*
 * decode MID 0x63 -- super packet
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_63(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned sid;

    if (3 > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x63: bad len %zu\n", len);
        return 0;
    }

    sid = getub(buf, 1);

    // FIXME: decode them!
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0x63: SID %u\n", sid);
    return 0;
}

/*
 * decode MID 0x64 -- super packet
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_64(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned sid;
    unsigned u[13];
    int i;
    int s[3];

    if (3 > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x64: bad len %zu\n", len);
        return 0;
    }

    sid = getub(buf, 1);
    switch (sid) {
    case  0x80:
        // GNSS Boot status
        u[0] = getub(buf, 2);
        u[1] = getub(buf, 3);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/80: enable %u type %u\n",
                 u[0], u[1]);
        break;
    case  0x81:
        // Extended NMEA Message Interval
        for (i = 0; i < 12; i++) {
            u[i] = getub(buf, i + 2);
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/81: GGA %u GSA %u GSV %u GLL %u RMC %u "
                 "VTG %u ZDA %u GNS %u GBS %u GRS %u DTM %u GST %u\n",
                 u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], u[8],
                 u[9], u[10], u[11]);
        break;
    case  0x83:
        // Interference Detection Status
        u[0] = getub(buf, 2);
        u[1] = getub(buf, 3);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/83: enable %u status %u\n",
                 u[0], u[1]);
        break;
    case  0x85:
        // GPS PARAMETER SEARCH ENGINE NUMBER
        u[0] = getub(buf, 2);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/85: search engine number %u\n", u[0]);
        break;
    case  0x88:
        // Position/Fix navigation mask
        u[0] = getub(buf, 2);
        u[1] = getub(buf, 3);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/88: 1st %u subsequent %u\n",
                 u[0], u[1]);
        break;
    case  0x8a:
        // GPS UTC Reference time
        u[0] = getub(buf, 2);
        u[1] = getbeu16(buf, 3);
        u[2] = getub(buf, 5);
        u[3] = getub(buf, 6);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/8a: enable %u year %u month %u day %u\n",
                 u[0], u[1], u[2], u[3]);
        break;
    case  0x8b:
        // GNSS Nav mode
        u[0] = getub(buf, 2);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/8b: mode %u\n", u[0]);
        break;
    case  0x8c:
        // GNSS Constellation type for nav solution
        u[0] = getbeu16(buf, 2);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/8c: Nav Type x%02x\n", u[0]);
        break;
    case  0x8e:
        // GPS time
        u[0] = getbeu32(buf, 2);    // TOW ms
        u[1] = getbeu32(buf, 6);    // TOW ns
        u[2] = getbeu16(buf, 10);   // GPS week
        s[0] = getsb(buf, 12);      // default leap s
        s[1] = getsb(buf, 13);      // current leap s
        u[3] = getub(buf, 14);      // valid
        // FIXME: save GPS week and leap s
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/8a: TOW %u %u week %u leap %d %d valid x%x\n",
                 u[0], u[1], u[2], s[0], s[1], u[3]);
        break;
    case  0x92:
        // GLONASS Time corrections
        s[0] = getbes32(buf, 2);    // tau c
        s[1] = getbes32(buf, 6);    // tau gps
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/92: tau c %d tau GPS %d\n",
                 s[0], s[1]);
        break;
    case  0xfe:
        // Version extension string
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64/fe: >%.32s<\n", &buf[2]);
        break;
    default:
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x64: SID x%02x len %zu\n", sid, len);
    }
    return 0;
}

/*
 * decode MID 0x65 -- super packet
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_65(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned sid;
    unsigned u[13];

    if (3 > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x65: bad len %zu\n", len);
        return 0;
    }

    sid = getub(buf, 1);
    switch (sid) {
    case  0x80:
        // 1PPS Pulse width
        u[0] = getbeu32(buf, 2);    // pulse width miicro seconds
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x65/80: width %u\n", u[0]);
        break;
    case  0x81:
        // PPS2 frequency
        u[0] = getbeu32(buf, 2);    // freq of PPS2 Hz
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x65/81: PPS2 Hz %u\n", u[0]);
        break;
    default:
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x65: SID x%02x len %zu\n", sid, len);
    }
    return 0;
}

/*
 * decode MID 0x6A -- super packet
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_6A(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned sid;
    unsigned u[13];
    double d[5];

    if (3 > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x6A: bad len %zu\n", len);
        return 0;
    }

    sid = getub(buf, 1);
    switch (sid) {
    case  0x83:
        // RTK mode and operational functioN
        u[0] = getub(buf, 2);                // RTK mode
        u[1] = getub(buf, 3);                // RTK function
        u[2] = getbeu32(buf, 4);             // saved survey length
        u[3] = getbeu32(buf, 8);             // standard deviation
        d[0] = getled64((char *)buf, 12);    // latitude
        d[1] = getled64((char *)buf, 20);    // longitude
        d[3] = getlef32((char *)buf, 28);    // altitude (HAE or MSL?)
        u[4] = getub(buf, 32);               // runtime function
        u[5] = getbeu32(buf, 33);            // run-time survey length
        d[4] = getlef32((char *)buf, 37);    // baseline length constant
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x6A/83: mode %u func %u len %u sdev %u lat %.8f "
                 "lon %.8f alt %.4f func %u len %u len %.4f\n",
                 u[0], u[1], u[2], u[3], d[0], d[1], d[3], u[4], u[5], d[4]);
        break;
    case  0x85:
        // RTK slave base serial port baud ratE
        u[0] = getub(buf, 2);        // rate code
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x6A/85: rate %u\n", u[0]);
        break;
    case  0x88:
        // RTK kinematic base serial port baud ratE
        u[0] = getub(buf, 2);        // rate code
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x6A/88: rate %u\n", u[0]);
        break;
    default:
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x6A: SID x%02x len %zu\n", sid, len);
    }
    return 0;
}

/*
 * decode MID 0x7A -- super packet
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_7A(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned sid, ssid;
    unsigned u[16];
    double d[2];
    int i;

    if (3 > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x7A: bad len %zu\n", len);
        return 0;
    }

    sid = getub(buf, 1);
    ssid = getub(buf, 2);
    switch ((sid << 8) | ssid) {
    case 0x0e80:
        // Moving base software version
        for (i = 0; i < 13; i++) {
            u[i] = getub(buf, i + 3);
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x7A/0E/80: type %u "
                  "kver %u.%u.%u over %u.%u.%u rev %02u.%02u.%02u\n",
                 u[0], u[2], u[3], u[4], u[6], u[7], u[8], u[10],
                 u[11], u[12]);
        break;
    case 0x0e81:
        // Moving base software CRC
        u[0] = getub(buf, 3);
        u[1] = getbeu16(buf, 4);

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq  0x7A/0E/801: type %u crc %u\n", u[0], u[1]);
        break;
    case 0x0e82:
        // Moving base pos update rate
        u[0] = getub(buf, 3);

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq  0x7A/0E/802: rate %u\n", u[0]);
        break;
    case 0x0e83:
        // Moving base heading and pitch offsets
        d[0] = getbeu32(buf, 3);    // heading
        d[1] = getbeu32(buf, 7);    // pitch

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq  0x7A/0E/803: heading %f pitch %f\n", d[0], d[1]);
        break;
    default:
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq 0x7A: SID x%02x/%02x len %zu\n", sid, ssid, len);
    }
    return 0;
}

/*
 * decode MID 0x80, Software Version
 *
 * 10 bytes
 *
 * Present in: Venus 6
 *             Venus 8
 *             Phoenix
 */
static gps_mask_t sky_msg_80(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned kver_x;  // kernel version
    unsigned kver_y;  // kernel version
    unsigned kver_z;  // kernel version
    unsigned over_x;  // ODM version
    unsigned over_y;  // ODM version
    unsigned over_z;  // ODM version
    unsigned rev_yy;  // revision
    unsigned rev_mm;  // revision
    unsigned rev_dd;  // revision

    if (14 != len) {
        return 0;
    }

    kver_x  = getbeu16(buf, 2);
    kver_y  = getub(buf, 4);
    kver_z  = getub(buf, 5);
    over_x  = getbeu16(buf, 6);
    over_y  = getub(buf, 8);
    over_z  = getub(buf, 9);
    rev_yy  = getbeu16(buf, 10);
    rev_mm  = getub(buf, 12);
    rev_dd  = getub(buf, 13);

    (void)snprintf(session->subtype, sizeof(session->subtype) - 1,
                   "kver %u.%u.%u over %u.%u.%u rev %02u.%02u.%02u",
                   kver_x, kver_y, kver_z,
                   over_x, over_y, over_z,
                   rev_yy, rev_mm, rev_dd);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0x80: %s\n",
             session->subtype);
    return 0;
}

/*
 * decode MID 0x81 - Software CRC
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_81(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned type, crc;

    if (4 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x81: bad len %zu\n", len);
        return 0;
    }

    type = getub(buf, 1);
    crc  = getbeu16(buf, 2);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0x81: type %u crc %u\n", type, crc);
    return 0;
}

/*
 * decode MID 0x86 - Position Update Rate
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_86(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned rate;

    if (2 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x86: bad len %zu\n", len);
        return 0;
    }

    rate = getub(buf, 1);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0x86: rate %u\n", rate);
    return 0;
}

/*
 * decode MID 0x89 - Binary measurement data output status
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_89(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned i;
    uint8_t u[7];

    if (8 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x89: bad len %zu\n", len);
        return 0;
    }

    for (i = 0; i < 7; i++) {
        u[i] = getub(buf, i + 1);
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0x89: rate %u Meas %u raw %u CH_status %u "
             "RCV_statas %u subf %u eraw %u\n",
             u[0], u[1], u[2], u[3], u[4], u[5], u[6]);

    return 0;
}

/*
 * decode MID 0x8A - Binary rtcm data output status
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_8A(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned i;
    uint8_t u[15];

    if (16 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x8A: bad len %zu\n", len);
        return 0;
    }

    for (i = 0; i < 15; i++) {
        u[i] = getub(buf, i + 1);
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0x8A: enable %u MSM %u 1005 %u 107x %u 108x %u "
             "109x %u 110x %u 111x %u 112x %u 1019 %u 1020 %u "
             "1042 %u 1046 %u type %u version %u\n",
             u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7],
             u[8], u[9], u[10], u[11], u[12], u[13], u[14]);

    return 0;
}

/*
 * decode MID 0x8B - Base position
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_8B(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    uint8_t u[5];
    double d[3];

    if (35 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x8B: bad len %zu\n", len);
        return 0;
    }

    u[0] = getub(buf, 1);
    u[1] = getbeu32(buf, 2);
    u[2] = getbeu32(buf, 6);
    d[0] = getbed64((const char *)buf, 10);
    d[1] = getbed64((const char *)buf, 18);
    d[2] = getbef32((const char *)buf, 26);
    u[3] = getub(buf, 30);
    u[4] = getbeu32(buf, 31);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0x8B: saved mode %u saved length %u stddev %u "
             "lat %.9f lon %.9f HAE %0.4f run mode %u survey len %u\n",
             u[0], u[1], u[2], d[0], d[1], d[2], u[3], u[4]);

    return 0;
}

/*
 * decode MID 0x93 - NMEA Talker ID
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_93(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned mode;

    if (2 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0x93: bad len %zu\n", len);
        return 0;
    }

    mode = getub(buf, 1);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0x93: mode %u\n", mode);
    return 0;
}

/*
 * decode MID 0xAE - GNSS Datum
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_AE(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned datum;

    if (3 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0xAE: bad len %zu\n", len);
        return 0;
    }

    datum  = getbeu16(buf, 1);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0xAE: datum %u\n", datum);
    return 0;
}

/*
 * decode MID 0xaf - DOP mask
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_AF(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned mode, pdop, hdop, gdop;

    if (8 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0xAF: bad len %zu\n", len);
        return 0;
    }

    mode = getub(buf, 1);
    pdop = getbeu16(buf, 2);
    hdop = getbeu16(buf, 4);
    gdop = getbeu16(buf, 6);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0xAF: Masks: mode %u pdop %u hdop %u gdop %u\n",
             mode, pdop, hdop, gdop);
    return 0;
}

/*
 * decode MID 0xb0 Elevation and DOP mask
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_B0(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned select, elevation, cnr;

    if (4 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0xB0: bad len %zu\n", len);
        return 0;
    }

    select = getub(buf, 1);
    elevation = getub(buf, 2);
    cnr = getub(buf, 3);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0xB0: select %u el %u cnr %u\n",
             select, elevation, cnr);
    return 0;
}

/*
 * decode MID 0xb4 - Position Pinning Status
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_B4(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned status, pspeed, pcnt, uspeed, ucnt, udist;

    if (12 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0xB4: bad len %zu\n", len);
        return 0;
    }

    status = getub(buf, 1);
    pspeed = getbeu16(buf, 2);
    pcnt = getbeu16(buf, 4);
    uspeed = getbeu16(buf, 6);
    ucnt = getbeu16(buf, 8);
    udist = getbeu16(buf, 10);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0xB4: status %u pspeed %u pcnt %u uspeed %u "
             "ucnt %u udist %u\n",
             status, pspeed, pcnt, uspeed, ucnt, udist);
    return 0;
}

/*
 * decode MID 0xB9 - Power Mode Status
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_B9(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned mode;

    if (2 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0xB9: bad len %zu\n", len);
        return 0;
    }

    mode = getub(buf, 1);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0xB9: mode %u\n", mode);
    return 0;
}

/*
 * decode MID 0xBB - 1PPS Cable Delay
 *
 * Present in Phoenix
 */
static gps_mask_t sky_msg_BB(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    int delay;

    if (5 != len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "Skytraq 0xBB: bad len %zu\n", len);
        return 0;
    }

    delay = getbeu32(buf, 1);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq 0xBB: delay %d\n", delay);
    return 0;
}

/*
 * decode MID 0xDC, Measurement Time
 *
 * 10 bytes
 */
static gps_mask_t sky_msg_DC(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned iod;   // Issue of data 0 - 255
    unsigned wn;    // week number 0 - 65535
    unsigned tow;   // receiver tow 0 - 604799999 in mS
    unsigned mp;    // measurement period 1 - 1000 ms
    char ts_buf[TIMESPEC_LEN];
    timespec_t ts_tow;

    if (10 != len) {
        return 0;
    }

    iod = (unsigned)getub(buf, 1);
    wn = getbeu16(buf, 2);
    tow = getbeu32(buf, 4);
    mp = getbeu16(buf, 8);
    MSTOTS(&ts_tow, tow);

    // should this be newdata.skyview_time?
    session->gpsdata.skyview_time = gpsd_gpstime_resolv(session, wn, ts_tow);

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "Skytraq 0xDC: iod %u wn %u tow %u mp %u t%s\n",
             iod, wn, tow, mp,
             timespec_str(&session->gpsdata.skyview_time, ts_buf,
                          sizeof(ts_buf)));
    return 0;
}

/*
 * decode MID 0xDD, Raw Measurements
 *
 */
static gps_mask_t sky_msg_DD(struct gps_device_t *session,
                             unsigned char *buf, size_t len UNUSED)
{
    unsigned iod;   // Issue of data 0 - 255
    unsigned nmeas; // number of measurements
    unsigned i;     // generic loop variable

    iod = (unsigned)getub(buf, 1);
    nmeas = (unsigned)getub(buf, 2);

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "Skytraq 0xDD: iod=%u, nmeas=%u\n",
             iod, nmeas);

    // check IOD?
    session->gpsdata.raw.mtime = session->gpsdata.skyview_time;

    /* zero the measurement data
     * so we can tell which meas never got set */
    memset(session->gpsdata.raw.meas, 0, sizeof(session->gpsdata.raw.meas));

    for (i = 0; i < nmeas; i++) {
        const char *obs_code;
        int off = 3 + (23 * i);

        uint8_t PRN = getub(buf, off + 0);
        // carrier-to-noise density ratio dB-Hz
        uint8_t cno = getub(buf, off + 1);
        // pseudorange in meters
        double prMes = getbed64((const char *)buf, off + 2);
        // carrier phase in cycles
        double cpMes = getbed64((const char *)buf, off + 10);
        // doppler in Hz, positive towards sat
        double doMes = getbef32((const char *)buf, off + 18);

        /* tracking stat
         * bit 0 - prMes valid
         * bit 1 - doppler valid
         * bit 2 - cpMes valid
         * bit 3 - cp slip
         * bit 4 - Coherent integration time?
         */
        uint8_t trkStat = getub(buf, off + 22);
        uint8_t gnssId = 0;
        uint8_t svId = 0;
        PRN2_gnssId_svId(PRN, &gnssId, &svId);

        session->gpsdata.raw.meas[i].gnssid = gnssId;
        switch (gnssId) {
        case 0:       // GPS
            FALLTHROUGH
        case 5:       // QZSS
            FALLTHROUGH
        case 20:      // IRNSS, just guessing here
            obs_code = "L1C";       // u-blox calls this L1C/A ?
            break;
        case 1:       // SBAS
            svId -= 100;            // adjust for RINEX 3 svid
            obs_code = "L1C";       // u-blox calls this L1C/A
            break;
        case 2:       // GALILEO
            obs_code = "L1B";       // u-blox calls this E1OS
            break;
        case 3:       // BeiDou
            obs_code = "L2I";       // u-blox calls this B1I
            break;
        default:      // huh?
            FALLTHROUGH
        case 4:       // IMES.  really?
            obs_code = "";       // u-blox calls this L1
            break;
        case 6:       // GLONASS
            obs_code = "L1C";       // u-blox calls this L1OF
            break;
        }
        (void)strlcpy(session->gpsdata.raw.meas[i].obs_code, obs_code,
                      sizeof(session->gpsdata.raw.meas[i].obs_code));

        session->gpsdata.raw.meas[i].svid = svId;
        session->gpsdata.raw.meas[i].snr = cno;
        session->gpsdata.raw.meas[i].satstat = trkStat;
        if (trkStat & 1) {
            // prMes valid
            session->gpsdata.raw.meas[i].pseudorange = prMes;
        } else {
            session->gpsdata.raw.meas[i].pseudorange = NAN;
        }
        if (trkStat & 2) {
            // doppler valid
            session->gpsdata.raw.meas[i].doppler = doMes;
        } else {
            session->gpsdata.raw.meas[i].doppler = NAN;
        }
        if (trkStat & 4) {
            // cpMes valid
            session->gpsdata.raw.meas[i].carrierphase = cpMes;
        } else {
            session->gpsdata.raw.meas[i].carrierphase = NAN;
        }
        session->gpsdata.raw.meas[i].codephase = NAN;
        session->gpsdata.raw.meas[i].deltarange = NAN;
        // skytraq does not report locktime, so assume max
        session->gpsdata.raw.meas[i].locktime = LOCKMAX;
        if (trkStat & 8) {
            // possible slip
            session->gpsdata.raw.meas[i].lli = 2;
        }
        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "PRN %u (%u:%u) prMes %f cpMes %f doMes %f\n"
                 "cno %u  rtkStat %u\n", PRN,
                 gnssId, svId, prMes, cpMes, doMes, cno, trkStat);

    }

    // return RAW_IS;  // WIP
    return 0;
}

/*
 * decode MID 0xDE, SV and channel status
 *
 * max payload: 3 + (Num_sats * 10) = 483 bytes
 */
static gps_mask_t sky_msg_DE(struct gps_device_t *session,
                             unsigned char *buf, size_t len UNUSED)
{
    int st, nsv;
    unsigned i;
    unsigned iod;   // Issue of data 0 - 255
    unsigned nsvs;  // number of SVs in this packet

    iod = (unsigned)getub(buf, 1);
    nsvs = (unsigned)getub(buf, 2);
    // too many sats?
    if (SKY_CHANNELS < nsvs) {
        return 0;
    }

    gpsd_zero_satellites(&session->gpsdata);
    for (i = st = nsv =  0; i < nsvs; i++) {
        int off = 3 + (10 * i);  // offset into buffer of start of this sat
        bool good;               // do we have a good record ?
        unsigned short sv_stat;
        unsigned short chan_stat;
        unsigned short ura;
        short PRN;
        uint8_t gnssId = 0;
        uint8_t svId = 0;

        PRN = (short)getub(buf, off + 1);
        // fit into gnssid:svid
        if (0 == PRN) {
            // skip 0 PRN
            continue;
        }
        PRN2_gnssId_svId(PRN, &gnssId, &svId);

        session->gpsdata.skyview[st].gnssid = gnssId;
        session->gpsdata.skyview[st].svid = svId;
        session->gpsdata.skyview[st].PRN = PRN;

        sv_stat = (unsigned short)getub(buf, off + 2);
        ura = (unsigned short)getub(buf, off + 3);
        session->gpsdata.skyview[st].ss = (double)getub(buf, off + 4);
        session->gpsdata.skyview[st].elevation =
            (double)getbes16(buf, off + 5);
        session->gpsdata.skyview[st].azimuth =
            (double)getbes16(buf, off + 7);
        chan_stat = (unsigned short)getub(buf, off + 9);

        session->gpsdata.skyview[st].used = (bool)(chan_stat & 0x30);
        good = session->gpsdata.skyview[st].PRN != 0 &&
            session->gpsdata.skyview[st].azimuth != 0 &&
            session->gpsdata.skyview[st].elevation != 0;

        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "Skytraq PRN=%2d El=%4.0f Az=%5.0f ss=%3.2f stat=%02x,%02x "
                 "ura=%d %c\n",
                 session->gpsdata.skyview[st].PRN,
                 session->gpsdata.skyview[st].elevation,
                 session->gpsdata.skyview[st].azimuth,
                 session->gpsdata.skyview[st].ss,
                 chan_stat, sv_stat, ura,
                 good ? '*' : ' ');

        if (good) {
            st += 1;
            if (session->gpsdata.skyview[st].used) {
                nsv++;
            }
        }
    }

    session->gpsdata.satellites_visible = st;
    session->gpsdata.satellites_used = nsv;

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "Skytraq 0xDE: nsvs=%u visible=%u iod=%u\n", nsvs,
             session->gpsdata.satellites_visible, iod);
    return SATELLITE_SET | USED_IS;
}

/*
 * decode MID 0xDF, Nav status (PVT)
 *
 * 81 bytes
 */
static gps_mask_t sky_msg_DF(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    unsigned short navstat;
    unsigned iod;           // Issue of data 0 - 255
    unsigned wn;            // week number 0 - 65535
    double f_tow;           // receiver tow Sec
    double clock_bias;
    double clock_drift;
    gps_mask_t mask = 0;
    timespec_t ts_tow;
    char ts_buf[TIMESPEC_LEN];

    if (81 != len) {
        return 0;
    }

    iod = (unsigned)getub(buf, 1);

    // fix status is byte 2
    navstat = (unsigned short)getub(buf, 2);
    session->newdata.status = STATUS_UNK;
    session->newdata.mode = MODE_NO_FIX;
    switch (navstat) {
    case 1:
        // fix prediction, ignore
        break;
    case 2:
        session->newdata.status = STATUS_GPS;
        session->newdata.mode = MODE_2D;
        break;
    case 3:
        session->newdata.status = STATUS_GPS;
        session->newdata.mode = MODE_3D;
        break;
    case 4:
        session->newdata.status = STATUS_DGPS;
        session->newdata.mode = MODE_3D;
        break;
    default:
        break;
    }

    wn = getbeu16(buf, 3);
    f_tow = getbed64((const char *)buf, 5);
    DTOTS(&ts_tow, f_tow);

    // position/velocity is bytes 13-48, meters and m/s
    session->newdata.ecef.x = (double)getbed64((const char *)buf, 13),
    session->newdata.ecef.y = (double)getbed64((const char *)buf, 21),
    session->newdata.ecef.z = (double)getbed64((const char *)buf, 29),
    session->newdata.ecef.vx = (double)getbef32((const char *)buf, 37),
    session->newdata.ecef.vy = (double)getbef32((const char *)buf, 41),
    session->newdata.ecef.vz = (double)getbef32((const char *)buf, 45);
    mask |= ECEF_SET | VECEF_SET;

    clock_bias = getbed64((const char *)buf, 49);
    clock_drift = getbes32(buf, 57);

    session->gpsdata.dop.gdop = getbef32((const char *)buf, 61);
    session->gpsdata.dop.pdop = getbef32((const char *)buf, 65);
    session->gpsdata.dop.hdop = getbef32((const char *)buf, 69);
    session->gpsdata.dop.vdop = getbef32((const char *)buf, 73);
    session->gpsdata.dop.tdop = getbef32((const char *)buf, 77);
    mask |= DOP_SET;

    session->newdata.time = gpsd_gpstime_resolv(session, wn, ts_tow );

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "Skytraq 0xDF: iod=%u, stat=%u, wn=%u, tow=%f, t=%s "
             "cb: %f, cd: %f "
             "gdop: %.2f, pdop: %.2f, hdop: %.2f, vdop: %.2f, tdop: %.2f\n",
             iod, navstat, wn, f_tow,
             timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)),
             clock_bias, clock_drift,
             session->gpsdata.dop.gdop,
             session->gpsdata.dop.pdop,
             session->gpsdata.dop.hdop,
             session->gpsdata.dop.vdop,
             session->gpsdata.dop.tdop);

    mask |= TIME_SET | STATUS_SET | MODE_SET | CLEAR_IS | REPORT_IS;
    return mask;
}

/*
 * decode MID 0xE0, GPS Subframe data
 *
 * len 33 bytes
 *
 */
static gps_mask_t sky_msg_E0(struct gps_device_t *session,
                             unsigned char *buf, size_t len UNUSED)
{
    int i;
    unsigned prn;   // GPS sat PRN
    unsigned subf;  // subframe 1-5
    // the words are preprocessed, not raw, just the 24bits of data
    uint32_t words[10];  // subframe 1-5

    if (33 != len) {
        return 0;
    }

    prn = (unsigned)getub(buf, 1);
    subf = (unsigned)getub(buf, 2);
    for (i = 0; i < 10; i++) {
        words[i] = (uint32_t)getbeu24(buf, 3 + (i * 3));
    }

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "Skytraq 0xE0: prn=%u, subf=%u\n",
             prn, subf);

    // could be SBAS?
    return gpsd_interpret_subframe(session, GNSSID_GPS, prn, words);
}

/*
 * pretend to decode MID 0xE2, Beiduo D1 Subframe data
 *
 * from Beidou Standard BDS-SIS-ICD-2.0
 * D1, with the data rate of 50 bps, is broadcasted by the MEO/IGSO satellites
 *
 * len 31 bytes
 *
 */
static gps_mask_t sky_msg_E2(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    int i;
    unsigned prn;   // BeidouPS sat PRN 206-214
    unsigned subf;  // subframe 1-5
    // the words are preprocessed, not raw, just the 28 bytes of data
    uint8_t bytes[28];  // raw data

    if (31 != len) {
        return 0;
    }

    prn = (unsigned)getub(buf, 1);
    subf = (unsigned)getub(buf, 2);
    for (i = 0; i < 28; i++) {
        bytes[i] = getub(buf, 3 + i);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq Beidou D1 subframe PRN %d Subframe %d "
             "length %zd byte:%s\n",
             prn, subf, len,
             gpsd_hexdump(session->msgbuf, sizeof(session->msgbuf),
                          (char *)bytes, 28));

    return ONLINE_SET;
}

/*
 * pretend to decode MID 0xE3, Beiduo D2 Subframe data
 *
 * from Beidou Standard BDS-SIS-ICD-2.0
 * D2, with the data rate of 500 bps, is broadcasted by the GEO satellites.
 *
 * len 31 bytes
 *
 */
static gps_mask_t sky_msg_E3(struct gps_device_t *session,
                             unsigned char *buf, size_t len)
{
    int i;
    unsigned prn;   // BeidouPS sat PRN 201-205
    unsigned subf;  // subframe 1-5
    // the words are preprocessed, not raw, just the 28 bytes of data
    uint8_t bytes[28];  // raw data

    if (31 != len) {
        return 0;
    }

    prn = (unsigned)getub(buf, 1);
    subf = (unsigned)getub(buf, 2);
    for (i = 0; i < 28; i++) {
        bytes[i] = getub(buf, 3 + i);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "Skytraq Beidou D2 subframe PRN %d Subframe %d "
             "length %zd byte:%s\n",
             prn, subf, len,
             gpsd_hexdump(session->msgbuf, sizeof(session->msgbuf),
                          (char *)bytes, 28));

    return ONLINE_SET;
}


static gps_mask_t sky_parse(struct gps_device_t * session, unsigned char *buf,
                            size_t len)
{
    gps_mask_t mask = 0;

    if (0 == len) {
        return mask;
    }

    buf += 4;   // skip the leaders and length
    len -= 7;   // don't count the leaders, length, csum and terminators
    // session->driver.sirf.lastid = buf[0];

    // check the checksum??

    // could change if the set of messages we enable does
    // session->cycle_end_reliable = true;

    switch (buf[0]) {
    case 0x62:
        mask = sky_msg_62(session, buf, len);
        break;
    case 0x63:
        mask = sky_msg_63(session, buf, len);
        break;
    case 0x64:
        mask = sky_msg_64(session, buf, len);
        break;
    case 0x65:
        mask = sky_msg_65(session, buf, len);
        break;
    case 0x6A:
        mask = sky_msg_6A(session, buf, len);
        break;
    case 0x7A:
        mask = sky_msg_7A(session, buf, len);
        break;
    case 0x80:
        // 128
        mask = sky_msg_80(session, buf, len);
        break;
    case 0x81:
        // Software CRC
        mask = sky_msg_81(session, buf, len);
        break;
    case 0x83:
        // 131 - ACK
        if (2 == len) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "Skytraq 0x83: ACK MID %#02x\n", buf[1]);
        } else if (3 == len) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "Skytraq 0x83: ACK MID %#02x/%02x\n", buf[1], buf[2]);
        } else if (4 <= len) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "Skytraq 0x83: ACK MID %#02x/%02x/%02x\n",
                     buf[1], buf[2], buf[3]);
        } else {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "Skytraq 0x83: ACK\n");
        }
        break;
    case 0x84:
        // 132 - NACK
        if (2 == len) {
            GPSD_LOG(LOG_INF, &session->context->errout,
                     "Skytraq 0x84: NACK MID %#02x\n", buf[1]);
        } else if (3 == len) {
            GPSD_LOG(LOG_INF, &session->context->errout,
                     "Skytraq 0x84: NACK MID %#02x/%02x\n", buf[1], buf[2]);
        } else if (4 <= len) {
            GPSD_LOG(LOG_INF, &session->context->errout,
                     "Skytraq 0x84: NACK MID %#02x/%02x/x%02x\n",
                     buf[1], buf[2], buf[3]);
        } else {
            GPSD_LOG(LOG_INF, &session->context->errout,
                     "Skytraq 0x84: NACK\n");
        }
        break;
    case 0x86:
        // 134 Position Update Rate
        mask = sky_msg_86(session, buf, len);
        break;
    case 0x89:
        mask = sky_msg_89(session, buf, len);
        break;
    case 0x8A:
        mask = sky_msg_8A(session, buf, len);
        break;
    case 0x8B:
        mask = sky_msg_8B(session, buf, len);
        break;
    case 0x93:
        // NMEA TALKER id
        mask = sky_msg_93(session, buf, len);
        break;

    case 0xae:
        // GNSS Datum
        mask = sky_msg_AE(session, buf, len);
        break;

    case 0xaf:
        // DOP Mask
        mask = sky_msg_AF(session, buf, len);
        break;

    case 0xb0:
        // Elevation and CNR mask
        mask = sky_msg_B0(session, buf, len);
        break;

    case 0xb4:
        // Positoin Pinning Status
        mask = sky_msg_B4(session, buf, len);
        break;

    case 0xb9:
        mask = sky_msg_B9(session, buf, len);
        break;

    case 0xbb:
        mask = sky_msg_BB(session, buf, len);
        break;

    case 0xDC:
        // 220
        mask = sky_msg_DC(session, buf, len);
        break;

    case 0xDD:
        // 221
        mask = sky_msg_DD(session, buf, len);
        break;

    case 0xDE:
        // 222
        mask = sky_msg_DE(session, buf, len);
        break;

    case 0xDF:
        // 223 - Nave status (PVT)
        mask = sky_msg_DF(session, buf, len);
        break;

    case 0xE0:
        // 224
        mask = sky_msg_E0(session, buf, len);
        break;

    case 0xE2:
        // 226 - Beidou2 D1 Subframe data
        mask = sky_msg_E2(session, buf, len);
        break;

    case 0xE3:
        // 227 - Beidou2 D2 Subframe data
        mask = sky_msg_E3(session, buf, len);
        break;

    case 0x67:   // sub-id messages
        FALLTHROUGH
    case 0x6F:   // sub-id messages
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq Unknown MID x%02x SID x%02x length %zd\n",
                 buf[0], buf[1], len);
        break;
    default:
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "Skytraq Unknown MID %#02x length %zd\n",
                 buf[0], len);
        break;
    }
    return mask;
}

static gps_mask_t skybin_parse_input(struct gps_device_t *session)
{
    /*
     * Use this hook to step, slowly, through the init messages.
     * By sending only one for each three received we try
     * to avoid overrunning the receiver input buffer.
     */

    if (UINT_MAX != session->cfg_stage) {
        session->cfg_step++;
    }

    if (UINT_MAX != session->cfg_stage &&
        3 <= session->cfg_step) {
        // more init to do

        session->cfg_stage++;
        session->cfg_step = 0;

        // Note: the checksums in the Skytaq doc are sometimes wrong...

        // drivers/driver_nmea0183.c send 0x04 to get MID 0x80 on detect

        // FIXME: make a table
        switch (session->cfg_stage) {
        case 1:
            // Send MID 0x3, to get back MID 0x81 Software CRC
            (void)sky_write(session, "\xA0\xA1\x00\x02\x03\x00\x03\x0d\x0a",
                            9);
            break;
        case 2:
            // Send MID 0x10, to get back MID 0x86 (Position Update Rate)
            (void)sky_write(session, "\xA0\xA1\x00\x01\x10\x10\x0d\x0a", 8);
            break;
        case 3:
            // Send MID 0x15, to get back MID 0xB9 (Power Mode Status)
            (void)sky_write(session, "\xA0\xA1\x00\x01\x15\x15\x0d\x0a", 8);
            break;
        case 4:
            // Send MID 0x1f, to get back MID 0x89 Measurement data statuS
            (void)sky_write(session, "\xA0\xA1\x00\x01\x1f\x1f\x0d\x0a", 8);
            break;
        case 5:
            // Send MID 0x21, to get back MID 0x8a  RTCM Data output status
            (void)sky_write(session, "\xA0\xA1\x00\x01\x21\x21\x0d\x0a", 8);
            break;
        case 6:
            // Send MID 0x23, to get back MID 0x8B (Base Position)
            (void)sky_write(session, "\xA0\xA1\x00\x01\x23\x23\x0d\x0a", 8);
            break;
        case 7:
            // Send MID 0x2d, to get back MID 0xAE (GNSS Datum)
            (void)sky_write(session, "\xA0\xA1\x00\x01\x2d\x2d\x0d\x0a", 8);
            break;
        case 8:
            // Send MID 0x2E, to get back MID 0xAF (DOP Mask)
            (void)sky_write(session, "\xA0\xA1\x00\x01\x2e\x2e\x0d\x0a", 8);
            break;
        case 9:
            // Send MID 0x2F, to get back MID 0x80 Elevation and SNR mask
            (void)sky_write(session, "\xA0\xA1\x00\x01\x2f\x2f\x0d\x0a", 8);
            break;
        case 10:
            // Send MID 0x3a, to get back MID 0xb4 Position Pinning
            (void)sky_write(session, "\xA0\xA1\x00\x01\x3a\x3a\x0d\x0a", 8);
            break;
        case 11:
            // Send MID 0x44, to get back MID 0xc2 1PPS timing
            // Timing mode versions only
            (void)sky_write(session, "\xA0\xA1\x00\x01\x44\x44\x0d\x0a", 8);
            break;
        case 12:
            // Send MID 0x46, to get back MID 0xbb 1PPS delay
            (void)sky_write(session, "\xA0\xA1\x00\x01\x46\x46\x0d\x0a", 8);
            break;
        case 13:
            // Send MID 0x4f, to get back MID 0x93 NMEA talker ID
            (void)sky_write(session, "\xA0\xA1\x00\x01\x4f\x4f\x0d\x0a", 8);
            break;
        case 14:
            // Send MID 0x56, to get back MID 0xc3 1PPS Output Mode
            // Timing mode versions only
            (void)sky_write(session, "\xA0\xA1\x00\x01\x56\x56\x0d\x0a", 8);
            break;
        case 15:
            // Send MID 0x62/02, to get back MID 0x62/80 SBAS status
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x62\x02\x60\x0d\x0a", 9);
            break;
        case 16:
            // Send MID 0x62/04, to get back MID 0x62/81 QZSS status
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x62\x04\x66\x0d\x0a", 9);
            break;
        case 17:
            // Send MID 0x62/06, to get back MID 0x62/82 SBAS Advanced status
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x62\x06\x64\x0d\x0a", 9);
            break;
        case 18:
            // Send MID 0x63/02, to get back MID 0x62/80 SAEE Status
            // not on PX1172RH_DS
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x63\x02\x61\x0d\x0a", 9);
            break;
        case 19:
            // Send MID 0x64/01, to get back MID 0x64/80
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x01\x65\x0d\x0a", 9);
            break;
        case 20:
            // Send MID 0x64/03, to get back MID 0x64/81
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x03\x67\x0d\x0a", 9);
            break;
        case 21:
            // Send MID 0x64/07, to get back MID 0x64/83
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x07\x63\x0d\x0a", 9);
            break;
        case 22:
            // Send MID 0x64/0b, to get back MID 0x64/85
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x0b\x6f\x0d\x0a", 9);
            break;
        case 23:
            // Send MID 0x64/12, to get back MID 0x64/88
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x12\x76\x0d\x0a", 9);
            break;
        case 24:
            // Send MID 0x64/16, to get back MID 0x64/8a
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x16\x72\x0d\x0a", 9);
            break;
        case 25:
            // Send MID 0x64/18, to get back MID 0x64/8b
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x18\x7c\x0d\x0a", 9);
            break;
        case 26:
            // Send MID 0x64/1a, to get back MID 0x64/8c
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x1a\x7e\x0d\x0a", 9);
            break;
        case 27:
            // Send MID 0x64/20, to get back MID 0x64/8e
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x20\x44\x0d\x0a", 9);
            break;
        case 28:
            // Send MID 0x64/22, to get back MID 0x64/8f
            // not on PX1172RH_DS
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x22\x46\x0d\x0a", 9);
            break;
        case 29:
            // Send MID 0x64/28, to get back MID 0x64/92
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x28\x4c\x0d\x0a", 9);
            break;
        case 30:
            // Send MID 0x64/30, to get back MID 0x64/98
            // not on PX1172RH_DS
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x30\x54\x0d\x0a", 9);
            break;
        case 31:
            // Send MID 0x64/31, to get back MID 0x64/
            // not on PX1172RH_DS
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x31\x55\x0d\x0a", 9);
            break;
        case 32:
            // Send MID 0x64/7d, to get back MID 0x64/fe
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x7d\x19\x0d\x0a", 9);
            break;
        case 33:
            // Send MID 0x64/35, to get back MID 0x64/99
            // not on PX1172RH_DS
            (void)sky_write(session,
                            "\xA0\xA1\x00\x03\x64\x35\x01\x50\x0d\x0a", 10);
            break;
        case 34:
            // Send MID 0x64/36, to get back MID 0x64/9a
            // not on PX1172RH_DS
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x64\x36\x52\x0d\x0a", 9);
            break;
        case 35:
            // Send MID 0x64/3c, to get back MID 0x64/99
            // not on PX1172RH_DS
            (void)sky_write(session,
                            "\xA0\xA1\x00\x04\x64\x3c\x47\x47\x19\x0d\x0a",
                            11);
            break;
        case 36:
            // Send MID 0x65/02, to get back MID 0x64/80
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x65\x02\x67\x0d\x0a", 9);
            break;
        case 37:
            // Send MID 0x65/04, to get back MID 0x64/8f
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x65\x04\x61\x0d\x0a", 9);
            break;
        case 38:
            // Send MID 0x6a/02, to get back MID 0x6a/83
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x6a\x02\x68\x0d\x0a", 9);
            break;
        case 39:
            // Send MID 0x6a/07, to get back MID 0x6a/83
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x6a\x07\x6d\x0d\x0a", 9);
            break;
        case 40:
            // Send MID 0x6a/0d, to get back MID 0x6a/85
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x6a\x0d\x67\x0d\x0a", 9);
            break;
        case 41:
            // Send MID 0x6a/14, to get back MID 0x6a/86
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x6a\x14\xfd\x0d\x0a", 9);
            break;
        case 42:
            // Send MID 0x6a/16, to get back MID 0x6a/89
            // not on PX1172RH_DS ?
            (void)sky_write(session,
                            "\xA0\xA1\x00\x02\x6a\x16\x7c\x0d\x0a", 9);
            break;
        case 43:
            // Send MID 0x7a/0e/01, to get back MID 0x7a/0e/80
            // not on PX1172RH_DS ?
            (void)sky_write(session,
                            "\xA0\xA1\x00\x03\x7a\x0e\x01\x75\x0d\x0a", 10);
            break;
        case 44:
            // Send MID 0x7a/0e/02, to get back MID 0x7a/0e/81
            // not on PX1172RH_DS ?
            (void)sky_write(session,
                            "\xA0\xA1\x00\x03\x7a\x0e\x02\x76\x0d\x0a", 10);
            break;
        case 45:
            // Send MID 0x7a/0e/03, to get back MID 0x7a/0e/82
            // not on PX1172RH_DS ?
            (void)sky_write(session,
                            "\xA0\xA1\x00\x03\x7a\x0e\x03\x77\x0d\x0a", 10);
            break;
        case 46:
            // Send MID 0x7a/0e/05, to get back MID 0x7a/0e/83
            // not on PX1172RH_DS ?
            (void)sky_write(session,
                            "\xA0\xA1\x00\x03\x7a\x0e\x05\x71\x0d\x0a", 10);
            break;
        default:
            // Done
            session->cfg_stage = UINT_MAX;
            break;
        }
    }

    if (SKY_PACKET == session->lexer.type) {
        return  sky_parse(session, session->lexer.outbuffer,
                        session->lexer.outbuflen);
    }
    if (NMEA_PACKET == session->lexer.type) {
        return nmea_parse((char *)session->lexer.outbuffer, session);
    }
    // should not get here...

    return 0;
}

// this is everything we export
// *INDENT-OFF*
const struct gps_type_t driver_skytraq =
{
    .channels       = SKY_CHANNELS,          // consumer-grade GPS
    .control_send   = sky_write,             // how to send a control string
    .event_hook     = NULL,                  // lifetime event handler
    .flags          = DRIVER_STICKY,         // remember this
    .get_packet     = generic_get,           // be prepared for Skytraq or NMEA
    .init_query     = NULL,                  // non-perturbing initial qury
    .mode_switcher  = sky_mode,              // Mode switcher
    .packet_type    = SKY_PACKET,            // associated lexer packet type
    .parse_packet   = skybin_parse_input,    // parse message packets
    .probe_detect   = NULL,                  // no probe
    .rtcm_writer    = gpsd_write,            // send RTCM data straight
    .trigger        = NULL,                  // no trigger
    .type_name      = "Skytraq",             // full name of type
};
// *INDENT-ON*
#endif  // defined SKYTRAQ_ENABLE) && defined(BINARY_ENABLE)

// vim: set expandtab shiftwidth=4
