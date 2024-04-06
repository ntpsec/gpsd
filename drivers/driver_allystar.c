/*
 * skeletal ALLYSTAR driver
 *
 * ALLYSTAR binary is a sloppy copy of UBX Binary.
 * The message structure is the same:
 *  header, ID, length, payload, checksum
 *
 * Header changed feom the UBX 0xb5 0x62 to 0xf1 0xd9
 * IDs are similar
 * The length, and content, of some payloads are change.
 * Checksum algorithm is the same.
 *
 * This file is Copyright Gary E  Miller
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>                // for abs()
#include <string.h>

#include "../include/gpsd.h"
#include "../include/bits.h"

// one byte class
typedef enum {
    ALLY_NAV = 0x01,     // Navigation
    ALLY_RXM = 0x02,     // Receiver Manager Messages
    ALLY_ACK = 0x05,     // (Not) Acknowledges for cfg messages
    ALLY_CFG = 0x06,     // Configuration requests
    ALLY_MON = 0x0a,     // System monitoring
    ALLY_AID = 0x0b,     // AGPS (Deprecated)
    ALLY_NMEA = 0xF0,    // NMEA, for CFG-MSG
} ally_classes_t;

#define MSGID(cls_, id_) (((cls_)<<8)|(id_))

typedef enum {
    ACK_ACK         = MSGID(ALLY_ACK, 0x01),
    ACK_NAK         = MSGID(ALLY_ACK, 0x00),
    MON_VER         = MSGID(ALLY_MON, 0x04),
    NAV_POSECEF     = MSGID(ALLY_NAV, 0x01),
    NAV_POSLLH      = MSGID(ALLY_NAV, 0x02),
    NAV_DOP         = MSGID(ALLY_NAV, 0x04),
    NAV_TIME        = MSGID(ALLY_NAV, 0x05),
    NAV_CLOCK       = MSGID(ALLY_NAV, 0x22),
    NAV_SVINFO      = MSGID(ALLY_NAV, 0x30),
} ally_msgs_t;

// ACK-* ids
static struct vlist_t vack_ids[] = {
    {ACK_ACK, "ACK-ACK"},
    {ACK_NAK, "ACK-NAK"},
    {0, NULL},
};

// NAV-TIME flags
static struct flist_t vtime_flags[] = {
    {1, 1, "week"},
    {2, 2, "second"},
    {4, 4, "leapsecond"},
    {0, 0, NULL},
};

// NAV-TIME navSys
static struct vlist_t vtime_navsys[] = {
    {0, "GPS"},
    {1, "BDS"},
    {0, NULL},
};

/* send a ALLYSTAR message.
 * calcualte checksums, etc.
 */
bool ally_write(struct gps_device_t * session,
                unsigned int msg_class, unsigned int msg_id,
                const unsigned char *msg, size_t payload_len)
{
    unsigned char CK_A, CK_B;
    ssize_t count;
    size_t i;
    bool ok;

    // do not write if -b (readonly) option set
    // "passive" handled earlier
    if (session->context->readonly) {
        return true;
    }

    session->msgbuf[0] = 0xf1;
    session->msgbuf[1] = 0xd9;

    CK_A = CK_B = 0;
    session->msgbuf[2] = msg_class;
    session->msgbuf[3] = msg_id;
    session->msgbuf[4] = payload_len & 0xff;
    session->msgbuf[5] = (payload_len >> 8) & 0xff;

    if ((sizeof(session->msgbuf) - 8) <= payload_len) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "=> GPS: ALLYL class: %02x, id: %02x, len: %zd TOO LONG!\n",
                 msg_class, msg_id, payload_len);
    }
    if (NULL != msg &&
        0 < payload_len) {
        (void)memcpy(&session->msgbuf[6], msg, payload_len);
    }

    // calculate CRC
    for (i = 2; i < (6 + payload_len); i++) {
        CK_A += session->msgbuf[i];
        CK_B += CK_A;
    }

    session->msgbuf[6 + payload_len] = CK_A;
    session->msgbuf[7 + payload_len] = CK_B;
    session->msgbuflen = payload_len + 8;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "=> GPS: ALLY: class: %02x, id: %02x, len: %zd, crc: %02x%02x\n",
             msg_class, msg_id, payload_len,
             CK_A, CK_B);
    count = gpsd_write(session, session->msgbuf, session->msgbuflen);
    ok = (count == (ssize_t) session->msgbuflen);
    return ok;
}

// ACK-ACK, ACK-NAK
static gps_mask_t msg_ack(struct gps_device_t *session,
                          unsigned char *buf, size_t payload_len)
{
    unsigned msgid = getbes16(buf, 2);

    if (2 > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: %s-: runt payload len %zd",
                 val2str(msgid, vack_ids), payload_len);
        return 0;
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: %s: class: %02x, id: %02x\n",
             val2str(msgid, vack_ids),
             buf[2], buf[3]);
    return 0;
}

// MON-*

/**
 * Receiver/Software Version
 * MON-VER
 * kinda sorta like early UBX-MON-VER if you squint real hard.
 *
 * buf points to payload.
 * payload_len is length of payload.
 *
 */
static gps_mask_t msg_mon_ver(struct gps_device_t *session,
                              unsigned char *buf,
                              size_t payload_len)
{
    if (32 > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: MON-VER: runt payload len %zd", payload_len);
        return 0;
    }

    // save SW and HW Version as subtype
    (void)snprintf(session->subtype, sizeof(session->subtype),
                   "SW %.16s,HW %.16s",
                   (char *)buf,
                   (char *)(buf + 16));

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: MON-VER: %s\n",
             session->subtype);

    return 0;
}

/* msg_mon() -- handle CLASS-MON
 */
static gps_mask_t msg_mon(struct gps_device_t *session,
                          unsigned char *buf, size_t payload_len)
{
    unsigned msgid = getbes16(buf, 2);
    gps_mask_t mask = 0;

    switch (msgid) {
    case MON_VER:
        mask = msg_mon_ver(session, &buf[4], payload_len);
        break;
    default:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: MON- %02x payload_len %zd\n",
                 msgid & 0xff, payload_len);
        break;
    }
    return mask;
}

// NAV-*

/**
 * Clock Solution NAV-CLOCK
 * Seems to match UBX-NAV-CLOCK
 *
 */
static gps_mask_t msg_nav_clock(struct gps_device_t *session,
                                unsigned char *buf, size_t data_len)
{
    unsigned long tAcc, fAcc;

    if (20 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV-CLOCK: runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    session->gpsdata.fix.clockbias = getles32(buf, 4);
    session->gpsdata.fix.clockdrift = getles32(buf, 8);
    tAcc = getleu32(buf, 12);
    fAcc = getleu32(buf, 16);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-CLOCK: iTOW=%lld clkB %ld clkD %ld tAcc %lu fAcc %lu\n",
             (long long)session->driver.ubx.iTOW,
             session->gpsdata.fix.clockbias,
             session->gpsdata.fix.clockdrift,
             tAcc, fAcc);
    return 0;
}

/**
 * NAV-DOP, Dilution of precision message
 * Seems to match UBX-NAV-DOP
 *
 */
static gps_mask_t msg_nav_dop(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len)
{
    unsigned u;
    gps_mask_t mask = 0;

    if (18 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV-DOP: runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);  // in milli seconds
    /*
     * We make a deliberate choice not to clear DOPs from the
     * last skyview here, but rather to treat this as a supplement
     * to our calculations from the visibility matrix, trusting
     * the firmware algorithms over ours.
     */
    u = getleu16(buf, 4);
    if (9999 > u) {
        session->gpsdata.dop.gdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 6);
    if (9999 > u) {
        session->gpsdata.dop.pdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 8);
    if (9999 > u) {
        session->gpsdata.dop.tdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 10);
    if (9999 > u) {
        session->gpsdata.dop.vdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 12);
    if (9999 > u) {
        session->gpsdata.dop.hdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    // Northing DOP
    u = getleu16(buf, 14);
    if (9999 > u) {
        session->gpsdata.dop.ydop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    // Easting DOP
    u = getleu16(buf, 16);
    if (9999 > u) {
        session->gpsdata.dop.xdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-DOP: gdop=%.2f pdop=%.2f "
             "hdop=%.2f vdop=%.2f tdop=%.2f ydop=%.2f xdop=%.2f\n",
             session->gpsdata.dop.gdop,
             session->gpsdata.dop.hdop,
             session->gpsdata.dop.vdop,
             session->gpsdata.dop.pdop, session->gpsdata.dop.tdop,
             session->gpsdata.dop.ydop, session->gpsdata.dop.xdop);
    return mask;
}

/*
 * Navigation Position ECEF message -- NAV-POSECEF
 * Looks same as UBX-NAV-POSECEF
 *
 * This message does not bother to tell us if it is valid.
 */
static gps_mask_t msg_nav_posecef(struct gps_device_t *session,
                                 unsigned char *buf, size_t data_len)
{
    gps_mask_t mask = ECEF_SET;

    if (20 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV-POSECEF: runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    // all in cm
    session->newdata.ecef.x = getles32(buf, 4) / 100.0;
    session->newdata.ecef.y = getles32(buf, 8) / 100.0;
    session->newdata.ecef.z = getles32(buf, 12) / 100.0;
    session->newdata.ecef.pAcc = getleu32(buf, 16) / 100.0;

    // (long long) cast for 32-bit compat
    GPSD_LOG(LOG_PROG, &session->context->errout,
        "ALLY: NAV-POSECEF: iTOW=%lld ECEF x=%.2f y=%.2f z=%.2f pAcc=%.2f\n",
        (long long)session->driver.ubx.iTOW,
        session->newdata.ecef.x,
        session->newdata.ecef.y,
        session->newdata.ecef.z,
        session->newdata.ecef.pAcc);
    return mask;
}

 /**
 * Geodetic position solution message
 * NAV-POSLLH, Class 1, ID 2
 * Seems same as UBX-MON-POSLLH
 *
 * This message does not bother to tell us if it is valid.
 * No mode, so limited usefulness
 */
static gps_mask_t msg_nav_posllh(struct gps_device_t *session,
                                 unsigned char *buf,
                                 size_t data_len UNUSED)
{
    gps_mask_t mask = 0;

    if (28 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV-POSLLH: runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    session->newdata.longitude = 1e-7 * getles32(buf, 4);
    session->newdata.latitude = 1e-7 * getles32(buf, 8);
    // altitude WGS84
    session->newdata.altHAE = 1e-3 * getles32(buf, 12);
    // altitude MSL
    session->newdata.altMSL = 1e-3 * getles32(buf, 16);
    // Let gpsd_error_model() deal with geoid_sep

    // Horizontal accuracy estimate in mm, unknown type
    session->newdata.eph = getleu32(buf, 20) * 1e-3;
    // Vertical accuracy estimate in mm, unknown type
    session->newdata.epv = getleu32(buf, 24) * 1e-3;

    GPSD_LOG(LOG_PROG, &session->context->errout,
        "ALLY: NAV-POSLLH: iTOW=%lld lat=%.3f lon=%.3f altHAE=%.3f "
        "eph %.3f epv %.3f\n",
        (long long)session->driver.ubx.iTOW,
        session->newdata.latitude,
        session->newdata.longitude,
        session->newdata.altHAE,
        session->newdata.eph,
        session->newdata.epv);

    mask = ONLINE_SET | HERR_SET | VERR_SET | LATLON_SET | ALTITUDE_SET;
    return mask;
}

/**
 * GPS Satellite Info -- NAV-SVINFO
 * Sort of like UBX-NAV-SVINFO
 *   UBX is (8 + 12 * numCh)
 *   ALLYSTAR if (8 + 24 *numCh)
 *
 */
static gps_mask_t msg_nav_svinfo(struct gps_device_t *session,
                               unsigned char *buf, size_t data_len)
{
    // char buf2[80];
    unsigned i, nsv, st;
    long long nchan;
    timespec_t ts_tow;

    if (8 > data_len) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "ALLY: NAV-SVINFO runt datalen %zd\n", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->driver.ubx.iTOW);
    session->gpsdata.skyview_time =
        gpsd_gpstime_resolv(session, session->context->gps_week, ts_tow);

    nchan = getleu32(buf, 4);  // 32 bits, seriously??
    if (nchan > MAXCHANNELS) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV SVINFO: runt >%d reported visible",
                 MAXCHANNELS);
        return 0;
    }

    gpsd_zero_satellites(&session->gpsdata);
    nsv = 0;
    for (i = st = 0; i < nchan; i++) {
        unsigned off = 24 * i;
        unsigned nmea_PRN;
        // v 2.3.1, 8 bit chand and 8-bit svid became 16-bit svid.
        // no doc on svid numbers...
        unsigned chan = getu2(buf, off + 8);
        unsigned ally_svid = getub(buf, off + 9);
        // no doc on flags
        unsigned flags = getub(buf, off + 10);
        // no doc on quality
        unsigned quality = getub(buf, off + 11);
        unsigned cno = getub(buf, off + 12);
        // bool used = (bool)(flags & 0x01);
        int el = getsb(buf, off + 13);
        int az = getles16(buf, off + 14);
        long long prRes = getles32(buf, off + 16);  // pseudorange residue, cm
        // pr Rate, m/s
        session->gpsdata.skyview[st].prRate = getlef32((const char*)buf,
                                                        off + 20);
        // pseudorange, m
        session->gpsdata.skyview[st].pr = getled64((const char*)buf, off + 24);

        // nmea_PRN = ubx_to_prn(ubx_PRN,
                              // &session->gpsdata.skyview[st].gnssid,
                              // &session->gpsdata.skyview[st].svid);

        // if (1 > nmea_PRN) {
        //     // skip bad PRN
        //     GPSD_LOG(LOG_PROG, &session->context->errout,
        //              "ALLY: NAV-SVINFO bad NMEA PRN %d\n", nmea_PRN);
        //     continue;
        // }
        nmea_PRN = ally_svid;      // conversion undocumented.
        session->gpsdata.skyview[st].PRN = nmea_PRN;

        session->gpsdata.skyview[st].ss = (double)cno;
        if (90 >= abs(el)) {
            session->gpsdata.skyview[st].elevation = (double)el;
        }
        if (359 > az &&
            0 <= az) {
            session->gpsdata.skyview[st].azimuth = (double)az;
        }
        session->gpsdata.skyview[st].prRes = prRes / 100.0;
        // session->gpsdata.skyview[st].qualityInd = quality;
        // session->gpsdata.skyview[st].used = used;
        // if (0x10 == (0x10 & flags)) {
        //    session->gpsdata.skyview[st].health = SAT_HEALTH_BAD;
        // } else {
        //    session->gpsdata.skyview[st].health = SAT_HEALTH_OK;
        // }

        // sbas_in_use is not same as used
        // if (used) {
        //     // not really 'used', just integrity data from there
        //     nsv++;
        //     session->gpsdata.skyview[st].used = true;
        // }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "ALLY: NAV-SVINFO chnd %llu  ally_svid %d gnssid %d, svid %d "
                 "nmea_PRN %d flags x%x az %.0f el %.0f cno %.0f prRes %.2f "
                 "quality x%x, prRate %f pr %.4f\n",
                 chan, ally_svid,
                 session->gpsdata.skyview[st].gnssid,
                 session->gpsdata.skyview[st].svid, nmea_PRN, flags,
                 session->gpsdata.skyview[st].azimuth,
                 session->gpsdata.skyview[st].elevation,
                 session->gpsdata.skyview[st].ss,
                 session->gpsdata.skyview[st].prRes,
                 quality,
                 session->gpsdata.skyview[st].prRate,
                 session->gpsdata.skyview[st].pr);
        // flags and quality undocumented.
        // GPSD_LOG(LOG_IO, &session->context->errout,
        //          "UBX: NAV-SVINFO: flags:%s quality:%s\n",
        //          flags2str(flags, fsvinfo_flags, buf2, sizeof(buf2)),
        //          val2str(quality, vquality));

        st++;
    }

    session->gpsdata.satellites_visible = (int)st;
    session->gpsdata.satellites_used = (int)nsv;
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-SVINFO: visible=%d used=%d\n",
             session->gpsdata.satellites_visible,
             session->gpsdata.satellites_used);
    // GPSD_LOG(LOG_IO, &session->context->errout,
    //          "ALLY: NAV-SVINFO: chipGen %s\n",
    //          val2str(globalFlags & 7, vglobalFlags));
    return SATELLITE_SET | USED_IS;
}

/**
 * GPS Leap Seconds - NAV-TIME
 * sorta like UBX-NAV-TIMEGPS
 */
static gps_mask_t msg_nav_time(struct gps_device_t *session,
                               unsigned char *buf, size_t data_len)
{
    char buf2[80];
    unsigned navSys;        // which constellation
    unsigned flags;         // Validity Flags
    unsigned FracTow;       // fractional TOW, ns
    gps_mask_t mask = 0;
    char ts_buf[TIMESPEC_LEN];

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV-TIME: runt payload len %zd", data_len);
        return 0;
    }

    navSys = getub(buf, 0);
    flags = getub(buf, 1);
    FracTow = getleu32(buf, 2);
    session->driver.ubx.iTOW = getleu64(buf, 4);   // refTow, ms

    // Valid leap seconds ?
    if (4 == (flags &4)) {
        session->context->leap_seconds = (int)getub(buf, 10);
        session->context->valid |= LEAP_SECOND_VALID;
    }

    // Valid GPS time of week and week number
    if (3 == (flags & 3)) {
        unsigned week;
        double timeErr;      // Time Accuracy Estimate in ns
        timespec_t ts_tow;

        week = getleu16(buf, 8);
        MSTOTS(&ts_tow, session->driver.ubx.iTOW);
        ts_tow.tv_nsec += FracTow;
        TS_NORM(&ts_tow);
        session->newdata.time = gpsd_gpstime_resolv(session, week, ts_tow);

        // timeErr in ns, unknown type (1 sigma, 50%, etc.)
        timeErr = (double)getleu32(buf, 12);
        session->newdata.ept = timeErr / 1e9;
        mask |= (TIME_SET | NTPTIME_IS);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-TIME: time=%s flags x%x\n",
             timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)),
             flags);
    GPSD_LOG(LOG_IO, &session->context->errout,
             "ALLY: NAV-TIME: navSys %s flags:%s\n",
	     val2str(navSys, vtime_navsys),
	     flags2str(flags, vtime_flags, buf2, sizeof(buf2)));
    return mask;
}

/* msg_nav() -- handle CLASS-NAV
 */
static gps_mask_t msg_nav(struct gps_device_t *session,
                          unsigned char *buf, size_t payload_len)
{
    unsigned msgid = getbes16(buf, 2);
    gps_mask_t mask = 0;

    switch (msgid) {
    case NAV_CLOCK:
        mask = msg_nav_clock(session, &buf[4], payload_len);
        break;
    case NAV_DOP:
        mask = msg_nav_dop(session, &buf[4], payload_len);
        break;
    case NAV_POSECEF:
        mask = msg_nav_posecef(session, &buf[4], payload_len);
        break;
    case NAV_POSLLH:
        mask = msg_nav_posllh(session, &buf[4], payload_len);
        break;
    case NAV_SVINFO:
        mask = msg_nav_svinfo(session, &buf[4], payload_len);
        break;
    case NAV_TIME:
        mask = msg_nav_time(session, &buf[4], payload_len);
        break;
    default:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV- %02x payload_len %zd\n",
                 msgid & 0xff, payload_len);
        break;
    }
    return mask;
}

static gps_mask_t ally_parse(struct gps_device_t * session, unsigned char *buf,
                            size_t len)
{
    unsigned payload_len;
    unsigned  msg_class;
    unsigned  msg_id;
    gps_mask_t mask = 0;

    /* the packet at least 8 bytes / Min packet is 8 ==  header (2),
    *  Message ID (2), length (2) and checksum (2).  The packetizer should
    *  already guarantee, This is to protect against malicious fuzzing. */
    if (8 > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: runt message len %zu\n", len);
        return 0;
    }

    // session->cycle_end_reliable = true; // Not yet.
    // for now, use driver.ubx.
    session->driver.ubx.iTOW = -1;        // set by decoder

    // extract message id and length
    msg_class = getub(buf, 2);
    msg_id = getub(buf, 3);
    payload_len = getles16(buf, 4);

    if ((len - 8) != payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: len (%zu) does not match payload (%u) + 8\n",
                 len, payload_len);
        return 0;
    }

    /* FIXME: make each case just call one function.
     / then this switch can be turned into a table. */
    switch (msg_class) {
    case ALLY_ACK:
        mask = msg_ack(session, buf, payload_len);
        break;
    case ALLY_AID:
        // Deprecated
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: AID- %02x length %zd/%u)\n",
                 msg_id, len, payload_len);
        break;
    case ALLY_CFG:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: CFG- %02x length %zd/%u)\n",
                 msg_id, len, payload_len);
        break;
    case ALLY_MON:
        mask = msg_mon(session, buf, payload_len);
        break;
    case ALLY_NAV:
        mask = msg_nav(session, buf, payload_len);
        break;
    case ALLY_RXM:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: RXM- %02x length %zd/%u)\n",
                 msg_id, len, payload_len);
        break;
    default:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: unknown packet id x%02x %02x length %zd/%u)\n",
                 msg_class, msg_id, len, payload_len);
        break;
    }
    return mask;
}

static gps_mask_t parse_input(struct gps_device_t *session)
{
    if (ALLYSTAR_PACKET == session->lexer.type) {
        return ally_parse(session, session->lexer.outbuffer,
                          session->lexer.outbuflen);
    }
    // a comment, JSON, or NMEA 0183
    return generic_parse_input(session);
}

// not used by gpsd, it's for gpsctl and friends
static ssize_t control_send(struct gps_device_t *session, char *msg,
                            size_t data_len)
{
    return ally_write(session, (unsigned int)msg[0], (unsigned int)msg[1],
                      (unsigned char *)msg + 2,
                      (size_t)(data_len - 2)) ? ((ssize_t) (data_len + 7)) : -1;
}

// ally_mode(), stub for NMEA/BINARY changer.
static void ally_mode(struct gps_device_t *session, int mode UNUSED)
{
    unsigned char msg[4] = {0};

    // turn on rate one NMEA
    msg[0] = ALLY_NMEA;     // class, NMEA
    msg[2] = 0x01;          // rate, one
    msg[3] = 0x07;          // ZDA
    (void)ally_write(session, ALLY_CFG, 0x01, msg, 3);

    // turn on rate one NAV-DOP
    msg[0] = ALLY_NAV;      // class, NAV
    msg[2] = 0x01;          // rate, one
    msg[3] = 0x01;          // DOP
    (void)ally_write(session, ALLY_CFG, 0x01, msg, 3);

    // turn on rate one NAV-POSLLH
    msg[3] = 0x02;          // POSLLH
    (void)ally_write(session, ALLY_CFG, 0x01, msg, 3);

    // turn on rate one NAV-TIME
    msg[3] = 0x05;          // TIME
    (void)ally_write(session, ALLY_CFG, 0x01, msg, 3);

    // turn on rate one NAV-CLOCK
    msg[3] = 0x22;          // CLOCK
    (void)ally_write(session, ALLY_CFG, 0x01, msg, 3);
}

static void event_hook(struct gps_device_t *session, event_t event)
{
    if (session->context->readonly) {
        return;
    }
    if (event == EVENT_IDENTIFIED) {
        GPSD_LOG(LOG_PROG, &session->context->errout, "ALLY: identified\n");

        // no longer set UBX-CFG-SBAS here, u-blox 9 and 10 do not have it

        if (session->context->passive) {
            /* passive mode, do no autoconfig
             * but we really want MON-VER. */
            (void)ally_write(session, ALLY_MON, 0x04, NULL, 0);
        } else if (O_OPTIMIZE == session->mode) {
            ally_mode(session, MODE_BINARY);
        } else {
            //* Turn off NMEA output, turn on UBX on this port.
            ally_mode(session, MODE_NMEA);
        }
    } else if (event == EVENT_DEACTIVATE) {
        /* There used to be a hotstart/reset here.
         * That caused u-blox USB to re-enumerate.
         * Sometimes to a new device name.
         * Bad.  Don't do that anymore...
         */
    }
}

static void init_query(struct gps_device_t *session)
{
    // MON-VER: query for version information
    (void)ally_write(session, ALLY_MON, 0x04, NULL, 0);
}

// this is everything we export
// *INDENT-OFF*
const struct gps_type_t driver_allystar =
{
    .type_name      = "ALLYSTAR",               // full name of type
    .packet_type    = ALLYSTAR_PACKET,          // lexer packet type
    .flags          = DRIVER_STICKY,            // remember this
    .trigger        = NULL,                     // recognize the type
    .channels       = 240,                      //  a guess
    .probe_detect   = NULL,                     // no probe
    .get_packet     = packet_get1,              // use generic one
    .parse_packet   = parse_input,              // parse message packets
    .rtcm_writer    = gpsd_write,               // send RTCM data straight
    .init_query     = init_query,               // non-perturbing query
    .event_hook     = event_hook,               // lifetime event handler
    .speed_switcher = NULL,                     // we can change baud rates
    .mode_switcher  = NULL,                     // there is a mode switcher
    .rate_switcher  = NULL,                     // change sample rate
    .min_cycle.tv_sec  = 1,                     // default
    .min_cycle.tv_nsec = 0,                     // default
    .control_send   = control_send,             // how to send a control string
    .time_offset    = NULL,                     // no NTP fudge factor
};
// *INDENT-ON*

// vim: set expandtab shiftwidth=4
