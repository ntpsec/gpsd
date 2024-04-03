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
} ally_classes_t;

#define MSGID(cls_, id_) (((cls_)<<8)|(id_))

typedef enum {
    ACK_ACK         = MSGID(ALLY_ACK, 0x01),
    ACK_NAK         = MSGID(ALLY_ACK, 0x00),
} ally_msgs_t;

// ACK-* ids
static struct vlist_t vack_ids[] = {
    {ACK_ACK, "ACK-ACK"},
    {ACK_NAK, "ACK-NAK"},
    {0, NULL},
};

// ACK-ACK, ACK-NAK
static gps_mask_t msg_ack(struct gps_device_t *session,
                              unsigned char *buf, size_t data_len)
{
    unsigned msgid = getbes16(buf, 2);

    if (2 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: %s-: runt payload len %zd",
                 val2str(msgid, vack_ids), data_len);
        return 0;
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: %s: class: %02x, id: %02x\n",
             val2str(msgid, vack_ids),
             buf[2], buf[3]);
    return 0;
}

static gps_mask_t ally_parse(struct gps_device_t * session, unsigned char *buf,
                            size_t len)
{
    unsigned data_len;
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
    data_len = getles16(buf, 4);

    if ((len - 8) != data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: len (%zu) does not match payload (%u) + 8\n",
                 len, data_len);
        return 0;
    }

    /* FIXME: make each case just call one function.
     / then this switch can be turned into a table. */
    switch (msg_class) {
    case ALLY_ACK:
        mask = msg_ack(session, buf, data_len);
        break;
    case ALLY_AID:
        // Deprecated
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: AID- %02x length %zd/%u)\n",
                 msg_id, len, data_len);
        break;
    case ALLY_CFG:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: CFG- %02x length %zd/%u)\n",
                 msg_id, len, data_len);
        break;
    case ALLY_MON:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: MON- %02x length %zd/%u)\n",
                 msg_id, len, data_len);
        break;
    case ALLY_NAV:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV- %02x length %zd/%u)\n",
                 msg_id, len, data_len);
        break;
    case ALLY_RXM:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: RXM- %02x length %zd/%u)\n",
                 msg_id, len, data_len);
        break;
    default:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: unknown packet id x%02x %02x length %zd/%u)\n",
                 msg_class, msg_id, len, data_len);
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
    .init_query     = NULL,                     // non-perturbing query
    .event_hook     = NULL,                     // lifetime event handler
    .speed_switcher = NULL,                     // we can change baud rates
    .mode_switcher  = NULL,                     // there is a mode switcher
    .rate_switcher  = NULL,                     // change sample rate
    .min_cycle.tv_sec  = 1,                     // default
    .min_cycle.tv_nsec = 0,                     // default
    .control_send   = NULL,                     // how to send a control string
    .time_offset    = NULL,                     // no NTP fudge factor
};
// *INDENT-ON*

// vim: set expandtab shiftwidth=4
