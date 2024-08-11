/*
 * Driver for the "CASIC" protocol used by the
 * 杭州中科微电子有限公司 / Zhongkewei /
 * icofchina.com AT6558 family(?) of chips.
 *
 * Documentation is the original "CASIC 多模卫星导航接收机协议规范",
 * and the "CASIC Multimode Satellite Navigation Receiver Protocol
 * specification" in English, which seems to be a machine translation,
 * both V4.2.0.3 2020-01-06.  Using both documents is helpful because
 * the translation loses all the images and much of the formatting.  A
 * Google translation of the oroginal was also helpful to understand
 * some things that were lost in the official translation, but beware
 * of errors.
 *
 * This chip is often used in inexpensive modules calling themselves
 * "ATGM336H", made by Zhongkewei and many other vendors.  It's also
 * used in the AI Thinker GP-01/GP-02 modules.  There are many boards
 * being sold under those names on Aliexpress.  The chip is also used
 * on boards by some of the slightly higher-level OEMs like Beitian
 * and Quescan, I think.
 *
 * At startup, mine says:
 *
 * $GPTXT,01,01,02,IC=AT6558F-5N-32-1C580900*06
 * $GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D
 * $GPTXT,01,01,02,TB=2020-03-26,13:25:12*4B
 * $GPTXT,01,01,02,MO=GB*77
 *
 * Mine is configured to emit both NMEA and CASIC messages. It also
 * responds to the UBX probe, at least partly.  That's a completely
 * undocumented feature.
 *
 * This code will probably work with the AT331C/AT332D modules.
 *
 * CASIC binary is another not quite clone of UBX Binary.
 * The message header fields are reordered:
 *  header, length, ID, payload, checksum
 *
 * Header changed feom the UBX 0xb5 0x62 to 0xba 0xce
 * Classes, IDs, and payloads are sometimes same/similar
 * Checksum algorithm is a simple 32-bit checksum of
 * everything after the header.
 *
 * This code was copied from driver_allystar.c, most of it was
 * deleted, then it was lightly altered to support the CASIC protocol.
 *
 * This file is Copyright John Hood
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
    CASIC_NAV = 0x01,     // Navigation
    CASIC_TIM = 0x02,     // Timing message: time pulse, time mark
    CASIC_RXM = 0x03,     // Receiver Manager Messages
    CASIC_ACK = 0x05,     // (Not) Acknowledges for cfg messages
    CASIC_CFG = 0x06,     // Configuration requests
    CASIC_MSG = 0x08,     // Satellite information
    CASIC_MON = 0x0a,     // System monitoring
    CASIC_AID = 0x0b,     // AGPS
} casic_classes_t;

#define MSGID(cls_, id_) (((cls_)<<8)|(id_))

typedef enum {
    NAV_STATUS      = MSGID(CASIC_NAV, 0x00),
    NAV_DOP         = MSGID(CASIC_NAV, 0x01),
    NAV_SOL         = MSGID(CASIC_NAV, 0x02),
    NAV_PV          = MSGID(CASIC_NAV, 0x03),
    NAV_TIMEUTC     = MSGID(CASIC_NAV, 0x10),
    NAV_CLOCK       = MSGID(CASIC_NAV, 0x11),
    NAV_GPSINFO     = MSGID(CASIC_NAV, 0x20),
    NAV_BDSINFO     = MSGID(CASIC_NAV, 0x21),
    NAV_GLNINFO     = MSGID(CASIC_NAV, 0x22),

    TIM_TP          = MSGID(CASIC_TIM, 0x00),

    RXM_MEASX       = MSGID(CASIC_RXM, 0x10),
    RXM_SVPOS       = MSGID(CASIC_RXM, 0x11),

    ACK_NAK         = MSGID(CASIC_ACK, 0x00),
    ACK_ACK         = MSGID(CASIC_ACK, 0x01),

    CFG_PRT         = MSGID(CASIC_CFG, 0x00),
    CFG_MSG         = MSGID(CASIC_CFG, 0x01),
    CFG_RST         = MSGID(CASIC_CFG, 0x02),
    CFG_TP          = MSGID(CASIC_CFG, 0x03),
    CFG_RATE        = MSGID(CASIC_CFG, 0x04),
    CFG_CFG         = MSGID(CASIC_CFG, 0x05),
    CFG_TMODE       = MSGID(CASIC_CFG, 0x06),
    CFG_NAVX        = MSGID(CASIC_CFG, 0x07),
    CFG_GROUP       = MSGID(CASIC_CFG, 0x08),

    MSG_BDSUTC      = MSGID(CASIC_MSG, 0x00),
    MSG_BDSION      = MSGID(CASIC_MSG, 0x01),
    MSG_BDSEPH      = MSGID(CASIC_MSG, 0x02),
    MSG_GPSUTC      = MSGID(CASIC_MSG, 0x05),
    MSG_GPSION      = MSGID(CASIC_MSG, 0x06),
    MSG_GPSEPH      = MSGID(CASIC_MSG, 0x07),
    MSG_GLNEPH      = MSGID(CASIC_MSG, 0x08),

    MON_VER         = MSGID(CASIC_MON, 0x04),
    MON_HW          = MSGID(CASIC_MON, 0x09),

    AID_INI         = MSGID(CASIC_MON, 0x01),
    AID_HUI         = MSGID(CASIC_MON, 0x03),
} casic_msgs_t;

// 2 bytes payload length, 2 bytes leader, 2 bytes ID
#define CASIC_PREFIX_LEN 6

static struct vlist_t vclass[] = {
    {0x01, "NAV"},
    {0x02, "TIM"},
    {0x03, "RXM"},
    {0x05, "ACK"},
    {0x06, "CFG"},
    {0x08, "MSG"},
    {0x0a, "MON"},
    {0x0b, "AID"},
    {0, NULL},
};

/* send a CASIC message.
 * calculate checksums, etc.
 */
bool casic_write(struct gps_device_t * session,
                unsigned int msg_class, unsigned int msg_id,
                const unsigned char *msg, size_t payload_len)
{
    ssize_t count;
    bool ok;
    unsigned checksum;

    // do not write if -b (readonly) option set
    // "passive" handled earlier
    if (session->context->readonly) {
        return true;
    }

    if ((sizeof(session->msgbuf) - 10) <= payload_len ||
        2048 < payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "=> GPS: CASIC class: %02x, id: %02x, len: %zd TOO LONG!\n",
                 msg_class, msg_id, payload_len);
    }
    if (0 != (payload_len % 4)) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "=> GPS: CASIC class: %02x, id: %02x, len: %zd UN ALIGNED!\n",
                 msg_class, msg_id, payload_len);
    }

    session->msgbuf[0] = 0xba;
    session->msgbuf[1] = 0xce;

    session->msgbuf[2] = payload_len & 0xff;
    session->msgbuf[3] = (payload_len >> 8) & 0xff;
    session->msgbuf[4] = msg_class;
    session->msgbuf[5] = msg_id;

    if (NULL != msg &&
        0 < payload_len) {
        (void)memcpy(&session->msgbuf[CASIC_PREFIX_LEN], msg, payload_len);
    }

    checksum = casic_checksum((unsigned char *)session->msgbuf + 2,
                              payload_len + 4);
    putle32(session->msgbuf, payload_len + 6, checksum);

    session->msgbuflen = payload_len + 10;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "=> GPS: CASIC: class: %02x, id: %02x, len: %zd, csum: %04x\n",
             msg_class, msg_id, payload_len,
             checksum);
    count = gpsd_write(session, session->msgbuf, session->msgbuflen);
    ok = (count == (ssize_t) session->msgbuflen);
    return ok;
}

// ACK-*

// ACK-ACK
static gps_mask_t msg_ack_ack(struct gps_device_t *session,
                              unsigned char *buf, size_t payload_len UNUSED)
{
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: ACK-ACK: class: %02x(%s), id: %02x\n",
              buf[0], val2str(buf[0], vclass), buf[1]);
    return 0;
}

// ACK-NAK
static gps_mask_t msg_ack_nak(struct gps_device_t *session,
                              unsigned char *buf, size_t payload_len UNUSED)
{
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: ACK-NAK: class: %02x(%s), id: %02x\n",
              buf[0], val2str(buf[0], vclass), buf[1]);
    return 0;
}

// CFG-*

/**
 * Port configuration
 * CFG-PRT
 *
 * buf points to payload.
 * payload_len is length of payload.
 *
 */
static gps_mask_t msg_cfg_prt(struct gps_device_t *session,
                              unsigned char *buf,
                              size_t payload_len UNUSED)
{
    unsigned portID = getub(buf, 0);
    unsigned protoMask = getub(buf, 1);
    unsigned mode = getleu16(buf, 2);
    unsigned long long baudRate = getleu32(buf, 4);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: CFG-PRT: portID %d protoMask %02x mode %04x "
             " baudRate %llu\n",
             portID, protoMask, mode, baudRate);

    return 0;
}

// MON-*

/**
 * Receiver/Software Version
 * MON-VER
 *
 * buf points to payload.
 * payload_len is length of payload.
 *
 */
static gps_mask_t msg_mon_ver(struct gps_device_t *session,
                              unsigned char *buf,
                              size_t payload_len UNUSED)
{
    // save SW and HW Version as subtype
    (void)snprintf(session->subtype, sizeof(session->subtype),
                   "SW %.32s,HW %.32s",
                   (char *)buf,
                   (char *)(buf + 32));

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: MON-VER: %s\n",
             session->subtype);

    return 0;
}

/* msg_decode() -- dispatch all message types to proper decoder
 */
static gps_mask_t msg_decode(struct gps_device_t *session,
                             unsigned char *buf, size_t payload_len)
{
    unsigned msgid = getbes16(buf, 4);
    gps_mask_t mask = 0;
    size_t needed_len;
    const char *msg_name;
    gps_mask_t (* p_decode)(struct gps_device_t *, unsigned char *, size_t);

    switch (msgid) {
    case ACK_NAK:
        needed_len = 4;
        msg_name = "ACK-NAK";
        p_decode = msg_ack_nak;
        break;
    case ACK_ACK:
        needed_len = 4;
        msg_name = "ACK-ACK";
        p_decode = msg_ack_ack;
        break;
    case CFG_PRT:
        msg_name ="CFG-PRT";
        needed_len = 8;
        p_decode = msg_cfg_prt;
        break;
    case MON_VER:
        msg_name ="MON-VER";
        needed_len = 64;
        p_decode = msg_mon_ver;
        break;
    default:
        msg_name ="UNK-UNK";
        needed_len = 0;
        p_decode = NULL;
        break;
    }
    if (needed_len > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: %s: runt payload len %zd need %zd\n",
                 msg_name, payload_len, needed_len);
        return 0;
    }
    if (NULL == p_decode) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: Unsupported/unknown %s(%02x)-%02x payload_len %zd\n",
                 val2str((msgid >> 8) & 0xff, vclass),
                 (msgid >> 8) & 0xff,
                 msgid & 0xff, payload_len);
        return 0;
    }
    mask = p_decode(session, &buf[CASIC_PREFIX_LEN], payload_len);
    return mask;
}

static gps_mask_t casic_parse(struct gps_device_t * session,
                              unsigned char *buf, size_t len)
{
    size_t payload_len;
    gps_mask_t mask = 0;
    int class, id;

    /* Minimum packet size is 10: header (2), length (2), Message ID
    *  (2), payload (0), and checksum (4).  The packetizer should
    *  already guarantee this to protect against malicious fuzzing. */
    if (10 > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: runt message len %zu\n", len);
        return 0;
    }

    // extract payload length, check against actual length
    payload_len = getles16(buf, 2);

    if ((len - 10) != payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: len (%zu) does not match payload (%zu) + 10\n",
                 len, payload_len);
        return 0;
    }

    class = buf[4];
    id = buf[5];

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: %s(%02x)-%02x\n",
             val2str(class, vclass), class, id);

    mask = msg_decode(session, buf, payload_len);

    return mask;
}

static gps_mask_t parse_input(struct gps_device_t *session)
{
    if (CASIC_PACKET == session->lexer.type) {
        return casic_parse(session, session->lexer.outbuffer,
                           session->lexer.outbuflen);
    }
    // a comment, JSON, or NMEA 0183
    return generic_parse_input(session);
}

// not used by gpsd, it's for gpsctl and friends
static ssize_t control_send(struct gps_device_t *session, char *msg,
                            size_t data_len)
{
    return casic_write(session, (unsigned int)msg[0], (unsigned int)msg[1],
                      (unsigned char *)msg + 2,
                      (size_t)(data_len - 2)) ? ((ssize_t)(data_len + 7)) : -1;
}


static void event_hook(struct gps_device_t *session, event_t event)
{
    if (session->context->readonly) {
        return;
    }
    if (EVENT_IDENTIFIED == event) {
        return;
    }
    // We would like MON-VER but it at least sometimes doesn't work.
    (void)casic_write(session, CASIC_MON, 0x04, NULL, 0);
    // Port configuration seems to work.
    (void)casic_write(session, CASIC_CFG, 0x00, NULL, 0);
}

static void init_query(struct gps_device_t *session)
{
    // MON-VER: query for version information
    (void)casic_write(session, CASIC_MON, 0x04, NULL, 0);
}

// this is everything we export
// *INDENT-OFF*
const struct gps_type_t driver_casic =
{
    .type_name      = "CASIC",                  // full name of type
    .packet_type    = CASIC_PACKET,             // lexer packet type
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
