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
 * Quectel's "L76K GNSS Protocol Specification" V1.1, 2021-12-16, covers
 * the same protocol for one module in the family.  It is public, and
 * has worked examples, but documents no NAV messages.
 *
 * The two documents number their sections differently.  A bare section
 * or table number below is from the L76K spec, one from the CASIC spec
 * is written "CASIC section 2.7.4".
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
#include "../include/driver_casic.h"
#include "../include/strfuncs.h"

static struct vlist_t vclass[] = {
    {CASIC_NAV, "NAV"},
    {CASIC_TIM, "TIM"},
    {CASIC_RXM, "RXM"},
    {CASIC_ACK, "ACK"},
    {CASIC_CFG, "CFG"},
    {CASIC_MSG, "MSG"},
    {CASIC_MON, "MON"},
    {CASIC_AID, "AID"},
    {0, NULL},
};

/* send a CASIC message.
 * calculate checksums, etc.
 *
 * Return: True -- read-only, or sent OK
 *         False -- send failed
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

    if ((sizeof(session->msgbuf) - CASIC_OVERHEAD) <= payload_len ||
        2048 < payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "=> GPS: CASIC class: %02x, id: %02x, len: %zd TOO LONG!\n",
                 msg_class, msg_id, payload_len);
        return false;
    }
    if (0 != (payload_len % 4)) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "=> GPS: CASIC class: %02x, id: %02x, len: %zd UN ALIGNED!\n",
                 msg_class, msg_id, payload_len);
        return false;
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

    // the checksum covers everything after the leader, except itself
    checksum = casic_checksum((unsigned char *)session->msgbuf + 2,
                              payload_len + CASIC_PREFIX_LEN - 2);
    putle32(session->msgbuf, payload_len + CASIC_PREFIX_LEN, checksum);

    session->msgbuflen = payload_len + CASIC_OVERHEAD;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "=> GPS: CASIC: class: %02x, id: %02x, len: %zd, csum: %04x\n",
             msg_class, msg_id, payload_len,
             checksum);
    count = gpsd_write(session, session->msgbuf, session->msgbuflen);
    ok = (count == (ssize_t) session->msgbuflen);
    return ok;
}

/* Send a message named by its joined class/ID.  casic_write() keeps
 * the halves apart for control_send(), which gets them from a client.
 */
static bool casic_send(struct gps_device_t *session, casic_msgs_t msgid,
                       const unsigned char *msg, size_t payload_len)
{
    return casic_write(session, CASIC_CLS_OF(msgid), CASIC_ID_OF(msgid),
                       msg, payload_len);
}

/* ProtoMask of the port gpsd has open, false if it cannot be pinned to
 * one.  Best source is what we last commanded, since a Set addresses
 * the current port; failing that, the poll's answers narrowed to
 * ports taking binary input at our speed.
 */
static bool casic_our_proto_mask(const struct gps_device_t *session,
                                 unsigned char *maskp)
{
    unsigned char mask = 0;
    bool have = false;
    unsigned i;

    if (session->driver.casic.last_commanded_valid) {
        *maskp = session->driver.casic.last_commanded;
        return true;
    }

    for (i = 0; i < CASIC_MAX_PORTS; i++) {
        if (!session->driver.casic.port[i].valid ||
            0 == (session->driver.casic.port[i].proto_mask &
                  CASIC_PROTO_IN_BIN) ||
            (unsigned)gpsd_get_speed(session) !=
                session->driver.casic.port[i].baud) {
            continue;
        }
        if (have &&
            mask != session->driver.casic.port[i].proto_mask) {
            // two ports fit this link and disagree, so neither is usable
            return false;
        }
        mask = session->driver.casic.port[i].proto_mask;
        have = true;
    }
    if (have) {
        *maskp = mask;
    }
    return have;
}

/* Encode framing into a CFG-PRT "Mode" field, CASIC_MODE_* in
 * driver_casic.h.
 *
 * gpsd has no word size getter; gpsd_set_speed() pairs two stop bits
 * with seven data bits, and gpsd.c only calls a speed switcher when
 * wordsize == 9 - stopbits.  So stopbits decides both fields.
 */
static unsigned short casic_mode_bits(char parity, int stopbits)
{
    unsigned mode = (2 == stopbits) ?
                        (CASIC_MODE_DATA_7 | CASIC_MODE_STOP_2) :
                        (CASIC_MODE_DATA_8 | CASIC_MODE_STOP_1);

    switch (parity) {
    case 'O':
        FALLTHROUGH
    case 'o':
        mode |= CASIC_MODE_PAR_ODD;
        break;
    case 'E':
        FALLTHROUGH
    case 'e':
        mode |= CASIC_MODE_PAR_EVEN;
        break;
    case 'N':
        FALLTHROUGH
    case 'n':
        FALLTHROUGH
    default:
        mode |= CASIC_MODE_PAR_NONE;
        break;
    }

    return (unsigned short)mode;
}

/* The line speed to write into a CFG-PRT Set, 0 if there is none.  A
 * Set always commits a baud rate, and gpsd_get_speed() answers a
 * fabricated 9600 on sources that have no line speed.
 */
static unsigned int casic_our_baud(const struct gps_device_t *session)
{
    switch (session->sourcetype) {
    case SOURCE_RS232:
        FALLTHROUGH
    case SOURCE_USB:
        FALLTHROUGH
    case SOURCE_BLUETOOTH:
        return (unsigned int)gpsd_get_speed(session);
    default:
        return 0;
    }
}

/* Build and send a CFG-PRT Set, section 3.2.2.1.  Overwrites the whole
 * port config, CFG-PRT has no delta form.
 */
static bool casic_cfg_prt(struct gps_device_t *session,
                          unsigned int baud_rate, unsigned char proto_mask,
                          char parity, int stopbits)
{
    unsigned char payload[8];
    unsigned short mode;
    bool ok;

    if (1 != stopbits &&
        2 != stopbits) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: CFG-PRT: %d stop bits not supported\n", stopbits);
        return false;
    }
    if (0 == baud_rate) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: CFG-PRT: no line speed for this source\n");
        return false;
    }
    mode = casic_mode_bits(parity, stopbits);

    putbyte(payload, 0, 0xff);           // portID: current port
    putbyte(payload, 1, proto_mask);
    putle16(payload, 2, mode);
    putle32(payload, 4, baud_rate);

    ok = casic_send(session, CASIC_CFG_PRT, payload, sizeof(payload));
    if (ok) {
        /* ?DEVICE setting mode and speed calls mode_switcher() then
         * set_serial(); without this the second undoes the first.
         */
        session->driver.casic.last_commanded = proto_mask;
        session->driver.casic.last_commanded_valid = true;
    }
    return ok;
}

/* True when gpsd may not write to the receiver, -b.  Logs, because
 * callers discard a switcher's false return.  Not --passive, which
 * stops only the autoconfiguration in event_hook().
 */
static bool casic_readonly(const struct gps_device_t *session,
                           const char *what)
{
    if (!session->context->readonly) {
        return false;
    }
    GPSD_LOG(LOG_WARN, &session->context->errout,
             "CASIC: readonly, not changing %s\n", what);
    return true;
}

// Map a line speed to its $PCAS01 <CMD>, CASIC_PCAS01_NONE if it has none.
static casic_pcas01_t casic_pcas01_cmd(speed_t speed)
{
    switch (speed) {
    case 4800:   return CASIC_PCAS01_4800;
    case 9600:   return CASIC_PCAS01_9600;
    case 19200:  return CASIC_PCAS01_19200;
    case 38400:  return CASIC_PCAS01_38400;
    case 57600:  return CASIC_PCAS01_57600;
    case 115200: return CASIC_PCAS01_115200;
    default:     return CASIC_PCAS01_NONE;
    }
}

/* speed_switcher.  $PCAS01 and CFG-PRT both change the UART baud rate,
 * and set_serial() flips gpsd's own speed right after sending, so only
 * one can go per call, and returning true is what commits the flip.
 * Refuse unless the receiver is known to be listening for the one sent.
 */
static bool casic_speed(struct gps_device_t *session, speed_t speed,
                        char parity, int stopbits)
{
    unsigned char proto_mask;
    casic_pcas01_t cmd;

    if (casic_readonly(session, "speed")) {
        return false;
    }
    if (0 == casic_our_baud(session)) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: source has no line speed, not changing speed\n");
        return false;
    }
    if (!casic_our_proto_mask(session, &proto_mask)) {
        // event_hook()'s CFG-PRT poll fills this in
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: port protocols unknown, not changing speed\n");
        return false;
    }

    /* $PCAS01 carries a baud enum only, so any framing but 8N1 needs
     * CFG-PRT, as does a port that does not take text input.
     */
    if (('N' != parity &&
         'n' != parity) ||
        1 != stopbits ||
        0 == (proto_mask & CASIC_PROTO_IN_TXT)) {
        if (0 == (proto_mask & CASIC_PROTO_IN_BIN)) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "CASIC: port takes neither $PCAS01 nor binary input, "
                     "not changing speed\n");
            return false;
        }
        return casic_cfg_prt(session, (unsigned int)speed, proto_mask,
                             parity, stopbits);
    }

    cmd = casic_pcas01_cmd(speed);
    if (CASIC_PCAS01_NONE == cmd) {
        return false;
    }
    return (0 < nmea_send(session, "$PCAS01,%d", (int)cmd));
}

/* Legal CFG-RATE Interval and $PCAS02 values, in ms.  Sections 2.3.2
 * and 3.2.2.4 Table 13 list these three, it is not a range.
 */
static const unsigned casic_intervals[] = {200, 500, 1000};

// how far off a request may be and still count as one of the above
#define CASIC_RATE_SLOP_MS 10

/* Map a requested cycle time to a legal CFG-RATE Interval, 0 if the
 * receiver cannot produce it.  Does not round: gpsd.c records the
 * requested cycle whenever rate_switcher() returns true.
 */
static unsigned casic_rate_ms(double cycletime)
{
    int64_t ms = (int64_t)(cycletime * MS_IN_SEC);
    size_t i;

    for (i = 0; i < ROWS(casic_intervals); i++) {
        if (CASIC_RATE_SLOP_MS >= llabs(ms - (int64_t)casic_intervals[i])) {
            return casic_intervals[i];
        }
    }
    return 0;
}

/* rate_switcher.  Unlike speed, rate is safe to send both ways:
 * neither CFG-RATE nor $PCAS02 touches the baud rate, so whichever
 * the receiver is listening for wins and the other is ignored.
 */
static bool casic_rate(struct gps_device_t *session, double cycletime)
{
    unsigned interval;
    unsigned baud = casic_our_baud(session);
    unsigned char payload[CASIC_LEN_RATE];
    bool ok_bin, ok_txt;

    if (casic_readonly(session, "rate")) {
        return false;
    }

    interval = casic_rate_ms(cycletime);
    if (0 == interval) {
        char legal[64] = "";
        size_t i;

        for (i = 0; i < ROWS(casic_intervals); i++) {
            str_appendf(legal, sizeof(legal), " %u", casic_intervals[i]);
        }
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: cycle %.3f s not supported, only:%s ms\n",
                 cycletime, legal);
        return false;
    }

    if (1000 > interval &&
        0 != baud &&
        115200 > baud) {
        // Section 2.3.2: NMEA output must be cut to one sentence, and
        // baud raised to 115200, before a sub-second interval keeps
        // up.  This driver does not do that for you.  Nothing to say
        // on a source with no line speed to outrun.
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: rate %u ms at %u bps may overrun the link\n",
                 interval, baud);
    }

    putle16(payload, 0, interval);
    putle16(payload, 2, 0);              // Res
    ok_bin = casic_send(session, CASIC_CFG_RATE, payload, sizeof(payload));
    ok_txt = (0 < nmea_send(session, "$PCAS02,%u", interval));

    return ok_bin || ok_txt;
}

/* mode_switcher.  CFG-PRT ProtoMask is the only protocol select here,
 * $PCAS has no equivalent.  Being non-NULL also makes the core treat
 * NMEA from this device as ours (dependent_nmea in libgpsd_core.c), as
 * for driver_ubx.c.
 */
static void casic_mode(struct gps_device_t *session, int mode)
{
    unsigned char proto_mask;
    unsigned int baud = casic_our_baud(session);

    if (casic_readonly(session, "mode")) {
        return;
    }
    if (0 == baud) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: source has no line speed, cannot set mode\n");
        return;
    }

    // keep our port's input bits, assume both if unknown.  Output only.
    if (!casic_our_proto_mask(session, &proto_mask)) {
        proto_mask = CASIC_PROTO_IN_BOTH;
    }
    proto_mask &= CASIC_PROTO_IN_BOTH;

    if (MODE_BINARY == mode) {
        proto_mask |= CASIC_PROTO_OUT_BIN;
    } else {
        proto_mask |= CASIC_PROTO_OUT_TXT;
    }

    (void)casic_cfg_prt(session, baud, proto_mask,
                        gpsd_get_parity(session),
                        gpsd_get_stopbits(session));

    /* Read the ports back.  Safe to poll at once only because a mode
     * change leaves the baud alone; casic_speed() must not do this.
     */
    (void)casic_send(session, CASIC_CFG_PRT, NULL, 0);
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

    if (CASIC_CFG_PRT == CASIC_MSGID(buf[0], buf[1])) {
        /* The Set did not take, so drop the ProtoMask casic_cfg_prt()
         * recorded on the way out.
         */
        session->driver.casic.last_commanded_valid = false;
    }
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

    /* An empty-payload poll answers for every port, so keep the replies
     * apart; casic_our_proto_mask() works out which one is ours.
     */
    if (CASIC_MAX_PORTS <= portID) {
        // 0xff is the "current port" sentinel, and we index by portID
        return 0;
    }
    session->driver.casic.port[portID].proto_mask = (unsigned char)protoMask;
    session->driver.casic.port[portID].baud = (unsigned int)baudRate;
    session->driver.casic.port[portID].valid = true;

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

// NAV-*

/**
 * Positioning precision factor
 * NAV-DOP
 *
 * buf points to payload.
 * payload_len is length of payload.
 *
 */
static gps_mask_t msg_nav_dop(struct gps_device_t *session,
                              unsigned char *buf,
                              size_t payload_len)
{
    if (CASIC_LEN_DOP > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
        "CASIC: NAV-DOP: runt payload len %zd", payload_len);
        return 0;
    }

    session->gpsdata.dop.pdop = getlef32((char *)buf, 4);    // Location DOP
    session->gpsdata.dop.hdop = getlef32((char *)buf, 8);    // Horizontal DOP
    session->gpsdata.dop.vdop = getlef32((char *)buf, 12);   // Vertical DOP
    session->gpsdata.dop.ydop = getlef32((char *)buf, 16);   // Northbound DOP
    session->gpsdata.dop.xdop = getlef32((char *)buf, 20);   // Eastbound DOP
    session->gpsdata.dop.tdop = getlef32((char *)buf, 24);   // Time DOP

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: NAV-DOP: pdop=%.2f hdop=%.2f vdop=%.2f tdop=%.2f "
             "ydop=%.2f xdop=%.2f\n",
             session->gpsdata.dop.pdop,
             session->gpsdata.dop.hdop,
             session->gpsdata.dop.vdop,
             session->gpsdata.dop.tdop,
             session->gpsdata.dop.ydop,
             session->gpsdata.dop.xdop);

    return DOP_SET;
}

static struct vlist_t vfix_valid[] = {
    {CASIC_FIX_NONE, "NONE"},
    {CASIC_FIX_EXTERNAL, "EXTERNAL"},
    {CASIC_FIX_ESTIMATE, "ESTIMATE"},
    {CASIC_FIX_HOLD, "HOLD"},
    {CASIC_FIX_PROJECTED, "PROJECTED"},
    {CASIC_FIX_FAST, "FAST"},
    {CASIC_FIX_2D, "2D"},
    {CASIC_FIX_3D, "3D"},
    {CASIC_FIX_GNSSDR, "GNSSDR"},
    {0, NULL},
};

/* NAV-PV's accuracy fields are variances, CASIC section 2.7.4, so
 * take the square root.  Shipped unscaled, one sigma, as driver_ubx.c
 * does with u-blox hAcc/vAcc.
 *
 * Zero is a "not computed" sentinel, not a perfect measurement.
 * gpsd_error_model() only substitutes its own estimate for a
 * non-finite value, so a zero would reach the client as a claim of no
 * error at all.  The negated test also passes a payload NAN through.
 */
static double casic_sigma(double variance)
{
    if (!(0 < variance)) {
        return NAN;
    }
    return sqrt(variance);
}

/**
 * Position and velocity
 * NAV-PV
 *
 * buf points to payload.
 * payload_len is length of payload.
 *
 * 80 bytes, CASIC section 2.7.4:
 *
 *    0 U4  runTime      ms since power-on
 *    4 U1  posValid     CASIC_FIX_*
 *    5 U1  velValid     CASIC_FIX_*
 *    6 U1  system       constellations contributing
 *    7 U1  numSV        satellites in the fix
 *    8 U1  numSVGPS
 *    9 U1  numSVBDS
 *   10 U1  numSVGLN
 *   12 R4  pDop
 *   16 R8  lon                      degrees
 *   24 R8  lat                      degrees
 *   32 R4  height                   m, HAE
 *   36 R4  sepGeoid                 m
 *   40 R4  hAcc                     m^2, variance
 *   44 R4  vAcc                     m^2, variance
 *   48 R4  velN                     m/s
 *   52 R4  velE                     m/s
 *   56 R4  velU                     m/s, up
 *   60 R4  speed3D                  m/s
 *   64 R4  speed2D                  m/s
 *   68 R4  heading                  degrees
 *   72 R4  sAcc                     (m/s)^2, variance
 *   76 R4  cAcc                     deg^2, variance
 */
static gps_mask_t msg_nav_pv(struct gps_device_t *session,
                             unsigned char *buf, size_t payload_len)
{
    gps_mask_t mask = 0;
    casic_fix_t posValid, velValid;
    unsigned numSV;
    // dimensions each solution has: 0 none, 2 horizontal, 3 with height
    unsigned pos_dims, vel_dims;
    char where[96] = "";

    if (CASIC_LEN_PV > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: NAV-PV: runt payload len %zd\n", payload_len);
        return 0;
    }
    posValid = (casic_fix_t)getub(buf, 4);
    velValid = (casic_fix_t)getub(buf, 5);
    numSV = getub(buf, 7);

    // no default: below, so -Wswitch checks.  Out of range stays no-fix.
    pos_dims = 0;
    session->newdata.mode = MODE_NO_FIX;
    session->newdata.status = STATUS_UNK;

    switch (posValid) {
    case CASIC_FIX_2D:
        pos_dims = 2;
        session->newdata.mode = MODE_2D;
        session->newdata.status = STATUS_GPS;
        break;
    case CASIC_FIX_3D:
        pos_dims = 3;
        session->newdata.mode = MODE_3D;
        session->newdata.status = STATUS_GPS;
        break;
    case CASIC_FIX_GNSSDR:
        pos_dims = 3;
        session->newdata.mode = MODE_3D;
        session->newdata.status = STATUS_GNSSDR;
        break;
    case CASIC_FIX_NONE:
        FALLTHROUGH
    case CASIC_FIX_EXTERNAL:
        FALLTHROUGH
    case CASIC_FIX_ESTIMATE:
        FALLTHROUGH
    case CASIC_FIX_HOLD:
        FALLTHROUGH
    case CASIC_FIX_PROJECTED:
        FALLTHROUGH
    case CASIC_FIX_FAST:
        // no position gpsd can report
        break;
    }
    mask |= MODE_SET | STATUS_SET;

    if (0 != pos_dims) {
        session->newdata.latitude = getled64((char *)buf, 24);
        session->newdata.longitude = getled64((char *)buf, 16);
        session->newdata.geoid_sep = getlef32((char *)buf, 36);
        session->newdata.eph = casic_sigma(getlef32((char *)buf, 40));
        mask |= LATLON_SET | HERR_SET;

        if (3 == pos_dims) {
            session->newdata.altHAE = getlef32((char *)buf, 32);
            session->newdata.epv = casic_sigma(getlef32((char *)buf, 44));
            /* eph and epv are sigmas on orthogonal axes, from the
             * same message, so hypot() gives the 3D sigma.  Left
             * unset, gpsd_error_model() fills sep from
             * pdop * P_UERE_NO_DGPS, a 95% figure on a generic 19 m
             * UERE, several times larger than the receiver's own.
             */
            session->newdata.sep = hypot(session->newdata.eph,
                                         session->newdata.epv);
            mask |= ALTITUDE_SET | VERR_SET;
        }
    }

    // no default: here either.  Out of range gives no velocity.
    vel_dims = 0;
    switch (velValid) {
    case CASIC_FIX_2D:
        // ground speed and track only: no vertical component
        vel_dims = 2;
        break;
    case CASIC_FIX_3D:
        FALLTHROUGH
    case CASIC_FIX_GNSSDR:
        vel_dims = 3;
        break;
    case CASIC_FIX_NONE:
        FALLTHROUGH
    case CASIC_FIX_EXTERNAL:
        FALLTHROUGH
    case CASIC_FIX_ESTIMATE:
        FALLTHROUGH
    case CASIC_FIX_HOLD:
        FALLTHROUGH
    case CASIC_FIX_PROJECTED:
        FALLTHROUGH
    case CASIC_FIX_FAST:
        // no velocity gpsd can report
        break;
    }

    if (0 != vel_dims) {
        double cAcc;

        session->newdata.NED.velN = getlef32((char *)buf, 48);
        session->newdata.NED.velE = getlef32((char *)buf, 52);
        session->newdata.speed = getlef32((char *)buf, 64);
        session->newdata.track = getlef32((char *)buf, 68);
        session->newdata.eps = casic_sigma(getlef32((char *)buf, 72));
        mask |= SPEED_SET | TRACK_SET | SPEEDERR_SET;

        if (3 == vel_dims) {
            // offset 56 is velU, up.  NED.velD is positive down.
            session->newdata.NED.velD = -getlef32((char *)buf, 56);
        }
        // gps_merge_fix() copies the whole NED block under VNED_SET,
        // so withholding it on 2D would drop velN and velE too.
        mask |= VNED_SET;

        // the cAcc sentinel is not a measurement, and nothing
        // downstream would filter it out
        cAcc = getlef32((char *)buf, 76);
        if (CASIC_CACC_UNKNOWN > cAcc) {
            session->newdata.epd = casic_sigma(cAcc);
            mask |= TRACKERR_SET;
        }
    }

    /* The NAV-*INFO messages count for themselves, and land later in
     * the epoch.  This stands when they never come.
     */
    session->gpsdata.satellites_used = (int)numSV;
    mask |= USED_IS;

    // no solution means no numbers, only NANs worth no line width
    if (0 != pos_dims) {
        str_appendf(where, sizeof(where), " lat %.7f lon %.7f",
                    session->newdata.latitude, session->newdata.longitude);
    }
    if (3 == pos_dims) {
        str_appendf(where, sizeof(where), " altHAE %.2f",
                    session->newdata.altHAE);
    }
    if (0 != vel_dims) {
        str_appendf(where, sizeof(where), " track %.2f speed %.2f",
                    session->newdata.track, session->newdata.speed);
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: NAV-PV: posValid %u(%s) velValid %u(%s) numSV %u%s\n",
             (unsigned)posValid, val2str(posValid, vfix_valid),
             (unsigned)velValid, val2str(velValid, vfix_valid), numSV, where);
    return mask;
}

/**
 * UTC time
 * NAV-TIMEUTC
 *
 * buf points to payload.
 * payload_len is length of payload.
 *
 * 24 bytes, CASIC section 2.7.5:
 *
 *    0 U4  runTime      ms since power-on
 *    4 R4  tAcc         s^2, but the scaling column reads "1/c2",
 *                       unresolvable, so not decoded into ept
 *    8 R4  msErr        rounding residual, ms
 *   12 U2  ms           milliseconds of the second
 *   14 U2  year
 *   16 U1  month
 *   17 U1  day
 *   18 U1  hour
 *   19 U1  minute
 *   20 U1  second
 *   21 U1  valid        CASIC_TIME_* bits
 *   22 U1  timeSrc      CASIC_TIMESRC_*
 *   23 U1  dateValid    CASIC_DATE_*
 */
static struct vlist_t vtimesrc[] = {
    {CASIC_TIMESRC_GPS, "GPS"},
    {CASIC_TIMESRC_BDS, "BDS"},
    {CASIC_TIMESRC_GLO, "GLO"},
    {0, NULL},
};

static struct vlist_t vdate_valid[] = {
    {CASIC_DATE_NONE, "NONE"},
    {CASIC_DATE_EXTERNAL, "EXTERNAL"},
    {CASIC_DATE_SAT, "SAT"},
    {CASIC_DATE_SATS, "SATS"},
    {0, NULL},
};

static gps_mask_t msg_nav_timeutc(struct gps_device_t *session,
                                  unsigned char *buf, size_t payload_len)
{
    struct tm date = {0};
    unsigned ms, valid;
    casic_timesrc_t timeSrc;
    casic_date_t dateValid;

    if (CASIC_LEN_TIMEUTC > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: NAV-TIMEUTC: runt payload len %zd\n", payload_len);
        return 0;
    }
    valid = getub(buf, 21);
    timeSrc = (casic_timesrc_t)getub(buf, 22);
    dateValid = (casic_date_t)getub(buf, 23);

    ms = getleu16(buf, 12);
    date.tm_year = (int)getleu16(buf, 14) - 1900;
    date.tm_mon = (int)getub(buf, 16) - 1;
    date.tm_mday = (int)getub(buf, 17);
    date.tm_hour = (int)getub(buf, 18);
    date.tm_min = (int)getub(buf, 19);
    date.tm_sec = (int)getub(buf, 20);
    date.tm_isdst = 0;
    date.tm_wday = 0;
    date.tm_yday = 0;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: NAV-TIMEUTC: valid %02x timeSrc %u(%s) dateValid %u(%s) "
             "%04d-%02d-%02d %02d:%02d:%02d.%03u\n",
             valid, (unsigned)timeSrc, val2str(timeSrc, vtimesrc),
             (unsigned)dateValid, val2str(dateValid, vdate_valid),
             date.tm_year + 1900, date.tm_mon + 1, date.tm_mday,
             date.tm_hour, date.tm_min, date.tm_sec, ms);

    /* No complete UTC time, or no date, expected while cold.  The
     * candidate and the bits that rejected it are in the line above.
     */
    if (CASIC_TIME_UTC != (valid & CASIC_TIME_UTC) ||
        CASIC_DATE_NONE == dateValid) {
        return 0;
    }

    session->newdata.time.tv_sec = mkgmtime(&date);
    session->newdata.time.tv_nsec = (long)ms * 1000000L;
    TS_NORM(&session->newdata.time);

    /* GOODTIME_IS: the valid bits say the leap second and date are
     * known, so timehint.c may ship this to ntpd before a fix.
     */
    return TIME_SET | NTPTIME_IS | GOODTIME_IS;
}

/* True while driver_nmea0183.c is part way through a $xxGSV burst: it
 * zeroes the skyview at part 1 and indexes the rest off
 * satellites_visible, so a CASIC sky landing mid-burst would truncate
 * it.  event_hook() turns one protocol off, --passive does not.
 */
static bool casic_nmea_owns_sky(const struct gps_device_t *session)
{
    return '\0' != session->nmea.last_gsv_talker;
}

/**
 * Satellite information, one constellation each
 * NAV-GPSINFO, NAV-BDSINFO, NAV-GLNINFO
 *
 * All three share one layout, CASIC sections 2.7.7 to 2.7.9:
 *
 *   header, 8 bytes:  U4 runTime, U1 numViewSv, U1 numFixSv,
 *                     U1 system (0 GPS, 1 BDS, 2 GLONASS), U1 res
 *   per satellite, 12 bytes:
 *     U1 chn (0xff when not tracked), U1 svid, U1 flags, U1 quality,
 *     U1 cno (dB-Hz), I1 elev (deg), I2 azim (deg), R4 prRes (m)
 *
 * flags and quality read as one U2, CASIC section 2.7.7 Remark [2].
 */
static gps_mask_t casic_svinfo(struct gps_device_t *session,
                               unsigned char *buf, size_t payload_len,
                               casic_msgs_t msgid, gnssid_t gnssid)
{
    unsigned numViewSv, numFixSv, used_here, i;
    int st;

    if (CASIC_LEN_SVINFO > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: NAV-SVINFO(%04x): runt payload len %zd\n",
                 (unsigned)msgid, payload_len);
        return 0;
    }
    if (!session->driver.casic.sky_valid) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CASIC: NAV-SVINFO(%04x): NMEA owns the sky, skipped\n",
                 msgid);
        return 0;
    }
    numViewSv = getub(buf, 4);
    numFixSv = getub(buf, 5);

    if ((CASIC_LEN_SVINFO + (CASIC_LEN_SVINFO_SV * numViewSv)) > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: NAV-SVINFO(%04x): runt payload %zd for %u sats\n",
                 (unsigned)msgid, payload_len, numViewSv);
        return 0;
    }

    /* No epoch handling, casic_parse() cleared the sky.  Just append to
     * what the other constellations already put there.
     */
    st = session->gpsdata.satellites_visible;
    // st is 0..MAXCHANNELS, so MAXCHANNELS - st cannot go negative
    if (numViewSv > (unsigned)(MAXCHANNELS - st)) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: NAV-SVINFO(%04x): %u sats will not fit %d\n",
                 (unsigned)msgid, numViewSv, MAXCHANNELS - st);
        return 0;
    }
    if (0 == st) {
        /* First NAV-*INFO of the epoch.  Its own used flags replace
         * NAV-PV's numSV, the rest of them add to it.
         */
        session->gpsdata.satellites_used = 0;
    }
    used_here = 0;
    for (i = 0; i < numViewSv; i++) {
        unsigned char *r = &buf[CASIC_LEN_SVINFO + (CASIC_LEN_SVINFO_SV * i)];
        unsigned svid = getub(r, 1);
        unsigned status = getleu16(r, 2);
        unsigned cno = getub(r, 4);
        int elev = getsb(r, 5);
        int azim = getles16(r, 6);
        bool used = (0 != (status & CASIC_SV_USED));

        session->gpsdata.skyview[st].gnssid = gnssid;
        session->gpsdata.skyview[st].svid = (unsigned char)svid;
        /* CASIC svid is u-blox gnssid:svid numbering: this receiver's
         * own $GxGSV names the same PRNs ubx2_to_prn() computes, for
         * all three constellations -- NAV-BDSINFO svid 27 comes back as
         * PRN 427.  tests/test_casic.c pins a captured NAV-*INFO frame
         * against the $GxGSV for the same epoch.
         */
        session->gpsdata.skyview[st].PRN = ubx2_to_prn((int)gnssid, (int)svid);
        session->gpsdata.skyview[st].ss = (double)cno;
        if (90 >= abs(elev)) {
            session->gpsdata.skyview[st].elevation = (double)elev;
        }
        if (0 <= azim &&
            360 > azim) {
            session->gpsdata.skyview[st].azimuth = (double)azim;
        }
        session->gpsdata.skyview[st].prRes = getlef32((char *)r, 8);
        session->gpsdata.skyview[st].used = used;
        if (used) {
            used_here++;
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "CASIC: NAV-SVINFO(%04x): gnssid %u svid %u PRN %d "
                 "cno %u el %d az %d used %d status %04x\n",
                 (unsigned)msgid, gnssid, svid,
                 session->gpsdata.skyview[st].PRN,
                 cno, elev, azim, (int)used, status);
        st++;
    }
    session->gpsdata.satellites_visible = st;
    session->gpsdata.satellites_used += (int)used_here;

    /* numFixSv is the receiver's own count of satellites in the fix.
     * A mismatch means flags B0 is not "used in fix" after all.
     */
    if (used_here != numFixSv) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: NAV-SVINFO(%04x): used %u != numFixSv %u\n",
                 (unsigned)msgid, used_here, numFixSv);
    }

    /* No SATELLITE_SET here: per constellation it would put three SKYs
     * on the wire per epoch, two part built.  casic_parse() raises it
     * once, at the epoch's end.  SKY still goes out on DOP_SET alone,
     * as it does for driver_ubx.c.
     */
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: NAV-SVINFO(%04x): visible %u used %d\n",
             (unsigned)msgid, session->gpsdata.satellites_visible,
             session->gpsdata.satellites_used);
    return 0;
}

// NAV-GPSINFO
static gps_mask_t msg_nav_gpsinfo(struct gps_device_t *session,
                                  unsigned char *buf, size_t payload_len)
{
    return casic_svinfo(session, buf, payload_len, CASIC_NAV_GPSINFO,
                        GNSSID_GPS);
}

// NAV-BDSINFO
static gps_mask_t msg_nav_bdsinfo(struct gps_device_t *session,
                                  unsigned char *buf, size_t payload_len)
{
    return casic_svinfo(session, buf, payload_len, CASIC_NAV_BDSINFO,
                        GNSSID_BD);
}

// NAV-GLNINFO
static gps_mask_t msg_nav_glninfo(struct gps_device_t *session,
                                  unsigned char *buf, size_t payload_len)
{
    return casic_svinfo(session, buf, payload_len, CASIC_NAV_GLNINFO,
                        GNSSID_GLO);
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
    case CASIC_ACK_NAK:
        needed_len = CASIC_LEN_ACK;
        msg_name = "ACK-NAK";
        p_decode = msg_ack_nak;
        break;
    case CASIC_ACK_ACK:
        needed_len = CASIC_LEN_ACK;
        msg_name = "ACK-ACK";
        p_decode = msg_ack_ack;
        break;
    case CASIC_CFG_PRT:
        msg_name ="CFG-PRT";
        needed_len = CASIC_LEN_PRT;
        p_decode = msg_cfg_prt;
        break;
    case CASIC_MON_VER:
        msg_name ="MON-VER";
        needed_len = CASIC_LEN_VER;
        p_decode = msg_mon_ver;
        break;
    case CASIC_NAV_DOP:
        msg_name ="NAV-DOP";
        needed_len = CASIC_LEN_DOP;
        p_decode = msg_nav_dop;
        break;
    case CASIC_NAV_PV:
        msg_name = "NAV-PV";
        needed_len = CASIC_LEN_PV;
        p_decode = msg_nav_pv;
        break;
    case CASIC_NAV_TIMEUTC:
        msg_name = "NAV-TIMEUTC";
        needed_len = CASIC_LEN_TIMEUTC;
        p_decode = msg_nav_timeutc;
        break;
    case CASIC_NAV_GPSINFO:
        msg_name = "NAV-GPSINFO";
        needed_len = CASIC_LEN_SVINFO;
        p_decode = msg_nav_gpsinfo;
        break;
    case CASIC_NAV_BDSINFO:
        msg_name = "NAV-BDSINFO";
        needed_len = CASIC_LEN_SVINFO;
        p_decode = msg_nav_bdsinfo;
        break;
    case CASIC_NAV_GLNINFO:
        msg_name = "NAV-GLNINFO";
        needed_len = CASIC_LEN_SVINFO;
        p_decode = msg_nav_glninfo;
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

/* How far the run time word may move and still be the same epoch, in
 * ms.  As for UBX iTOW, an exact compare would turn jitter into a
 * spurious boundary.  The fastest interval this part has is 200 ms.
 */
#define CASIC_TOW_SLOP_MS       10

static gps_mask_t casic_parse(struct gps_device_t * session,
                              unsigned char *buf, size_t len)
{
    size_t payload_len;
    gps_mask_t mask = 0;
    int64_t tow = -1;
    int class, id;
    bool new_epoch = false;

    /* Minimum packet size is the overhead alone: header (2), length
    *  (2), Message ID (2), payload (0), and checksum (4).  The
    *  packetizer should already guarantee this to protect against
    *  malicious fuzzing. */
    if (CASIC_OVERHEAD > len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: runt message len %zu\n", len);
        return 0;
    }

    // extract payload length, check against actual length
    // unsigned: a signed length would sign extend into a huge size_t
    payload_len = getleu16(buf, 2);

    if ((len - CASIC_OVERHEAD) != payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "CASIC: len (%zu) does not match payload (%zu) + %d\n",
                 len, payload_len, CASIC_OVERHEAD);
        return 0;
    }

    class = buf[4];
    id = buf[5];

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "CASIC: %s(%02x)-%02x\n",
             val2str(class, vclass), class, id);

    /* Not until the ender is known: claiming it sooner turns off
     * gpsd.c's report-on-every-change fallback while nothing yet
     * raises REPORT_IS, losing the first epoch outright.
     */
    session->cycle_end_reliable = (0 != session->driver.casic.end_msgid);

    /* Before dispatch, not after: the satellite messages append to a
     * sky that must be cleared at the boundary.  Only decoded messages
     * take part, nothing establishes that an undecoded one's leading
     * word is a comparable run time.
     */
    switch (CASIC_MSGID(class, id)) {
    case CASIC_NAV_DOP:
        FALLTHROUGH
    case CASIC_NAV_PV:
        FALLTHROUGH
    case CASIC_NAV_TIMEUTC:
        FALLTHROUGH
    case CASIC_NAV_GPSINFO:
        FALLTHROUGH
    case CASIC_NAV_BDSINFO:
        FALLTHROUGH
    case CASIC_NAV_GLNINFO:
        if (4 <= payload_len) {
            tow = (int64_t)getleu32(&buf[CASIC_PREFIX_LEN], 0);
        }
        break;
    default:
        break;
    }

    if (0 <= tow &&
        CASIC_TOW_SLOP_MS < llabs(tow - session->driver.casic.last_tow)) {
        // New epoch.  Whatever arrived last in the old one ends it.
        if (0 != session->driver.casic.last_msgid &&
            session->driver.casic.end_msgid !=
                session->driver.casic.last_msgid) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "CASIC: cycle ender is now %04x, was %04x\n",
                     session->driver.casic.last_msgid,
                     session->driver.casic.end_msgid);
            session->driver.casic.end_msgid =
                session->driver.casic.last_msgid;
        }
        new_epoch = true;
        session->driver.casic.last_tow = tow;
        // leave the skyview alone for an epoch NMEA is mid-burst in
        session->driver.casic.sky_valid = !casic_nmea_owns_sky(session);
        if (session->driver.casic.sky_valid) {
            // zeroes satellites_visible, but not satellites_used
            gpsd_zero_satellites(&session->gpsdata);
            session->gpsdata.satellites_used = 0;
        }
        mask |= CLEAR_IS;
    }

    /* Carry NAV-PV's sAcc through the rest of the epoch: gpsd_poll()
     * clears newdata on every packet, and gpsd_error_model() replaces
     * eps with its own WAG whenever newdata.eps is unset, so it would
     * be gone by the cycle ender.  epd needs none of this, that WAG
     * tests gpsdata.fix.epd itself.  Skipped at a boundary, where
     * gpsdata.fix still holds the epoch the CLEAR_IS above just ended.
     * Before msg_decode(), so a fresh NAV-PV wins.
     */
    if (!new_epoch &&
        0 != isfinite(session->gpsdata.fix.eps)) {
        session->newdata.eps = session->gpsdata.fix.eps;
        mask |= SPEEDERR_SET;
    }

    mask |= msg_decode(session, buf, payload_len);

    /* gpsd holds changes pending until REPORT_IS, so raising it once
     * per epoch coalesces a cycle into one TPV and one SKY.
     */
    if (0 <= tow) {
        unsigned msgid = (unsigned)CASIC_MSGID(class, id);

        if (session->driver.casic.end_msgid == msgid) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "CASIC: cycle end %04x tow %lld, %d satellites\n",
                     msgid, (long long)tow,
                     session->gpsdata.satellites_visible);
            mask |= REPORT_IS;

            if (session->driver.casic.sky_valid &&
                0 < session->gpsdata.satellites_visible) {
                /* The epoch is over, so every NAV-*INFO in it has
                 * arrived and the sky is whole.  Its time is the one
                 * NAV-TIMEUTC gave: still in newdata if that was this
                 * very packet, already merged into gpsdata.fix if it
                 * came earlier in the epoch, and zero if the receiver
                 * had no UTC to give.
                 */
                session->gpsdata.skyview_time =
                    (0 != (mask & TIME_SET)) ? session->newdata.time :
                                               session->gpsdata.fix.time;
                mask |= SATELLITE_SET | USED_IS;
            }
        }
        session->driver.casic.last_msgid = msgid;
    }

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
    if (EVENT_IDENTIFIED != event) {
        return;
    }
    GPSD_LOG(LOG_PROG, &session->context->errout, "CASIC: identified\n");
    // We would like MON-VER but it at least sometimes doesn't work.
    (void)casic_send(session, CASIC_MON_VER, NULL, 0);
    // Port configuration seems to work.
    (void)casic_send(session, CASIC_CFG_PRT, NULL, 0);

    if (session->context->passive) {
        return;
    }
    /* These parts ship emitting both at once, which leaves
     * driver_nmea0183.c's cycle logic and casic_parse()'s both live.
     */
    if (O_OPTIMIZE == session->mode) {
        casic_mode(session, MODE_BINARY);
    } else {
        casic_mode(session, MODE_NMEA);
    }
}

static void init_query(struct gps_device_t *session)
{
    // MON-VER: query for version information
    (void)casic_send(session, CASIC_MON_VER, NULL, 0);
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
    .speed_switcher = casic_speed,              // CFG-PRT or $PCAS01
    .mode_switcher  = casic_mode,               // CFG-PRT ProtoMask
    .rate_switcher  = casic_rate,               // CFG-RATE and $PCAS02
    /* 200 ms is the fastest interval.  gpsd only calls rate_switcher()
     * above min_cycle, and casic_rate_ms() allows 10 ms of slop.
     */
    .min_cycle.tv_sec  = 0,
    .min_cycle.tv_nsec = 200000000,             // 200 ms, 5 Hz
    .control_send   = control_send,             // how to send a control string
    .time_offset    = NULL,                     // no NTP fudge factor
};
// *INDENT-ON*

// vim: set expandtab shiftwidth=4
