/*
 * Defines for driver_casic.c
 *
 * Section numbers are Quectel's "L76K GNSS Protocol Specification"
 * V1.1; a number written "CASIC section 2.7.4" is from the "CASIC
 * Multimode Satellite Navigation Receiver Protocol specification"
 * V4.2.0.3.
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#ifndef _GPSD_CASIC_H_
#define _GPSD_CASIC_H_

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

#define CASIC_MSGID(cls_, id_) (((cls_)<<8)|(id_))
// ...and back again, for the places that want the halves apart
#define CASIC_CLS_OF(msgid_)    (((msgid_) >> 8) & 0xff)
#define CASIC_ID_OF(msgid_)     ((msgid_) & 0xff)

typedef enum {
    CASIC_NAV_STATUS      = CASIC_MSGID(CASIC_NAV, 0x00),
    CASIC_NAV_DOP         = CASIC_MSGID(CASIC_NAV, 0x01),
    CASIC_NAV_SOL         = CASIC_MSGID(CASIC_NAV, 0x02),
    CASIC_NAV_PV          = CASIC_MSGID(CASIC_NAV, 0x03),
    CASIC_NAV_TIMEUTC     = CASIC_MSGID(CASIC_NAV, 0x10),
    CASIC_NAV_CLOCK       = CASIC_MSGID(CASIC_NAV, 0x11),
    CASIC_NAV_GPSINFO     = CASIC_MSGID(CASIC_NAV, 0x20),
    CASIC_NAV_BDSINFO     = CASIC_MSGID(CASIC_NAV, 0x21),
    CASIC_NAV_GLNINFO     = CASIC_MSGID(CASIC_NAV, 0x22),

    CASIC_TIM_TP          = CASIC_MSGID(CASIC_TIM, 0x00),

    CASIC_RXM_MEASX       = CASIC_MSGID(CASIC_RXM, 0x10),
    CASIC_RXM_SVPOS       = CASIC_MSGID(CASIC_RXM, 0x11),

    CASIC_ACK_NAK         = CASIC_MSGID(CASIC_ACK, 0x00),
    CASIC_ACK_ACK         = CASIC_MSGID(CASIC_ACK, 0x01),

    CASIC_CFG_PRT         = CASIC_MSGID(CASIC_CFG, 0x00),
    CASIC_CFG_MSG         = CASIC_MSGID(CASIC_CFG, 0x01),
    CASIC_CFG_RST         = CASIC_MSGID(CASIC_CFG, 0x02),
    CASIC_CFG_TP          = CASIC_MSGID(CASIC_CFG, 0x03),
    CASIC_CFG_RATE        = CASIC_MSGID(CASIC_CFG, 0x04),
    CASIC_CFG_CFG         = CASIC_MSGID(CASIC_CFG, 0x05),
    CASIC_CFG_TMODE       = CASIC_MSGID(CASIC_CFG, 0x06),
    CASIC_CFG_NAVX        = CASIC_MSGID(CASIC_CFG, 0x07),
    CASIC_CFG_GROUP       = CASIC_MSGID(CASIC_CFG, 0x08),

    CASIC_MSG_BDSUTC      = CASIC_MSGID(CASIC_MSG, 0x00),
    CASIC_MSG_BDSION      = CASIC_MSGID(CASIC_MSG, 0x01),
    CASIC_MSG_BDSEPH      = CASIC_MSGID(CASIC_MSG, 0x02),
    CASIC_MSG_GPSUTC      = CASIC_MSGID(CASIC_MSG, 0x05),
    CASIC_MSG_GPSION      = CASIC_MSGID(CASIC_MSG, 0x06),
    CASIC_MSG_GPSEPH      = CASIC_MSGID(CASIC_MSG, 0x07),
    CASIC_MSG_GLNEPH      = CASIC_MSGID(CASIC_MSG, 0x08),

    CASIC_MON_VER         = CASIC_MSGID(CASIC_MON, 0x04),
    CASIC_MON_HW          = CASIC_MSGID(CASIC_MON, 0x09),

    CASIC_AID_INI         = CASIC_MSGID(CASIC_MON, 0x01),
    CASIC_AID_HUI         = CASIC_MSGID(CASIC_MON, 0x03),
} casic_msgs_t;

// 2 bytes leader, 2 bytes payload length, 2 bytes ID
#define CASIC_PREFIX_LEN 6
// 4 bytes checksum, after the payload
#define CASIC_CKSUM_LEN 4
// everything in a message that is not payload
#define CASIC_OVERHEAD  (CASIC_PREFIX_LEN + CASIC_CKSUM_LEN)

/* Payload length each decoder needs, in bytes.  msg_decode() drops a
 * shorter message; tests/test_casic.c checks its vectors against these.
 */
#define CASIC_LEN_ACK           4
#define CASIC_LEN_PRT           8
#define CASIC_LEN_VER           64
#define CASIC_LEN_DOP           28
#define CASIC_LEN_PV            80
#define CASIC_LEN_TIMEUTC       24
#define CASIC_LEN_RATE          4
// NAV-*INFO is a header plus one record per satellite
#define CASIC_LEN_SVINFO        8
#define CASIC_LEN_SVINFO_SV     12

/* CFG-PRT ProtoMask bits, section 3.2.2.1 Table 9.  Ports report
 * undocumented bits too, the worked example answers 0xff for UART0.
 * Accept whatever comes back, only ever send bits from this set.
 */
#define CASIC_PROTO_IN_BIN      0x01    // accept CASIC binary input
#define CASIC_PROTO_IN_TXT      0x02    // accept NMEA/$PCAS input
#define CASIC_PROTO_OUT_BIN     0x10    // emit CASIC binary
#define CASIC_PROTO_OUT_TXT     0x20    // emit NMEA
#define CASIC_PROTO_IN_BOTH     (CASIC_PROTO_IN_BIN | CASIC_PROTO_IN_TXT)

/* CFG-PRT Mode field, section 3.2.2.1 Table 10.  Bits 7:6 are the data
 * bits, 11:9 the parity, 13:12 the stop bits.  8N1 is 0x08c0, as in the
 * worked example.
 */
#define CASIC_MODE_DATA_7       (2U << 6)
#define CASIC_MODE_DATA_8       (3U << 6)
#define CASIC_MODE_PAR_EVEN     (0U << 9)
#define CASIC_MODE_PAR_ODD      (1U << 9)
#define CASIC_MODE_PAR_NONE     (4U << 9)
#define CASIC_MODE_STOP_1       (0U << 12)
#define CASIC_MODE_STOP_2       (2U << 12)

// $PCAS01 <CMD> is a baud rate enum, section 2.3.1, not a bps value.
typedef enum {
    CASIC_PCAS01_NONE    = -1,        // no $PCAS01 for this speed
    CASIC_PCAS01_4800    = 0,
    CASIC_PCAS01_9600    = 1,
    CASIC_PCAS01_19200   = 2,
    CASIC_PCAS01_38400   = 3,
    CASIC_PCAS01_57600   = 4,
    CASIC_PCAS01_115200  = 5,
} casic_pcas01_t;

/* NAV-PV posValid and velValid, CASIC sections 2.7.3 Remarks [1] and
 * [2].  Only the last three are a solution gpsd can report.
 */
typedef enum {
    CASIC_FIX_NONE      = 0,    // nothing yet
    CASIC_FIX_EXTERNAL  = 1,
    CASIC_FIX_ESTIMATE  = 2,
    CASIC_FIX_HOLD      = 3,    // last solution held over
    CASIC_FIX_PROJECTED = 4,
    CASIC_FIX_FAST      = 5,    // fast-mode solution
    CASIC_FIX_2D        = 6,
    CASIC_FIX_3D        = 7,
    CASIC_FIX_GNSSDR    = 8,    // GNSS + dead reckoning
} casic_fix_t;

/* NAV-PV cAcc reads exactly this when the receiver has no heading
 * accuracy: a sentinel, not a measurement.  deg^2, and no real error
 * comes near it.
 */
#define CASIC_CACC_UNKNOWN      1.0e6

// NAV-TIMEUTC timeSrc, CASIC section 2.7.5
typedef enum {
    CASIC_TIMESRC_GPS   = 0,
    CASIC_TIMESRC_BDS   = 1,
    CASIC_TIMESRC_GLO   = 2,
} casic_timesrc_t;

// NAV-TIMEUTC dateValid, CASIC section 2.7.5 Remark [3]
typedef enum {
    CASIC_DATE_NONE     = 0,
    CASIC_DATE_EXTERNAL = 1,
    CASIC_DATE_SAT      = 2,    // from one satellite
    CASIC_DATE_SATS     = 3,    // agreed by several satellites
} casic_date_t;

// NAV-TIMEUTC valid is a bitmask, CASIC section 2.7.5 Remark [1].
#define CASIC_TIME_TOW          0x01    // B0, UTC time of week valid
#define CASIC_TIME_WEEK         0x02    // B1, UTC week valid
#define CASIC_TIME_LEAP         0x04    // B2, leap-second correction valid
#define CASIC_TIME_UTC          (CASIC_TIME_TOW | CASIC_TIME_WEEK | \
                                 CASIC_TIME_LEAP)

// NAV-*INFO flags B0: this satellite is used in the solution
#define CASIC_SV_USED           0x0001

#endif  // _GPSD_CASIC_H_
