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

// prefix is 0xf1 0xd9, [class] [id]
#define ALLY_PREFIX_LEN 4

// one byte class
typedef enum {
    ALLY_NAV = 0x01,     // Navigation
    ALLY_RXM = 0x02,     // Receiver Manager Messages
    ALLY_ACK = 0x05,     // (Not) Acknowledges for cfg messages
    ALLY_CFG = 0x06,     // Configuration requests
    ALLY_MON = 0x0a,     // System monitoring
    ALLY_AID = 0x0b,     // AGPS (Deprecated)
} ally_classes_t;

static gps_mask_t ally_parse(struct gps_device_t * session, unsigned char *buf,
                            size_t len)
{
    unsigned data_len;
    unsigned  msg_class;
    unsigned  msg_id;
    gps_mask_t mask = 0;

    // the packet at least contains a head long enough for an empty message
    if (ALLY_PREFIX_LEN > len) {
        return 0;
    }

    session->cycle_end_reliable = true;
    // for now, use driver.ubx.
    session->driver.ubx.iTOW = -1;        // set by decoder

    // extract message id and length
    msg_class = getub(buf, 2);
    msg_id = getub(buf, 3);
    data_len = getles16(buf, 4);

    /* FIXME: make each case just call one function.
     / then this switch can be turned into a table. */
    switch (msg_class) {
    case ALLY_ACK:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: ACK- %02x length %zd/%u)\n",
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
