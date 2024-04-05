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
    NAV_DOP         = MSGID(ALLY_NAV, 0x01),
    NAV_POSLLH      = MSGID(ALLY_NAV, 0x02),
    NAV_TIME        = MSGID(ALLY_NAV, 0x05),
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
    case NAV_DOP:
        mask = msg_nav_dop(session, &buf[4], payload_len);
        break;
    case NAV_POSLLH:
        mask = msg_nav_posllh(session, &buf[4], payload_len);
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
