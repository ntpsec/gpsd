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
    ALLY_NMEA = 0xf0,    // NMEA, for CFG-MSG
    ALLY_RTCM = 0xf8,    // RTCM, for CFG-MSG

} ally_classes_t;

#define MSGID(cls_, id_) (((cls_)<<8)|(id_))

typedef enum {
    ACK_ACK         = MSGID(ALLY_ACK, 0x01),
    ACK_NAK         = MSGID(ALLY_ACK, 0x00),
    CFG_PRT         = MSGID(ALLY_CFG, 0x00),
    CFG_MSG         = MSGID(ALLY_CFG, 0x01),
    CFG_PPS         = MSGID(ALLY_CFG, 0x07),
    CFG_CFG         = MSGID(ALLY_CFG, 0x09),
    CFG_DOP         = MSGID(ALLY_CFG, 0x0a),
    CFG_ELEV        = MSGID(ALLY_CFG, 0x0b),
    CFG_NAVSAT      = MSGID(ALLY_CFG, 0x0c),
    CFG_HEIGHT      = MSGID(ALLY_CFG, 0x0d),
    CFG_SBAS        = MSGID(ALLY_CFG, 0x0e),
    CFG_SPDHOLD     = MSGID(ALLY_CFG, 0x0f),
    CFG_EPHSAVE     = MSGID(ALLY_CFG, 0x10),
    CFG_NUMSV       = MSGID(ALLY_CFG, 0x11),
    CFG_SURVEY      = MSGID(ALLY_CFG, 0x12),  // HD9200/HD9400 only
    CFG_FIXEDLLA    = MSGID(ALLY_CFG, 0x13),  // HD9200/HD9400 only
    CFG_FIXEDECEF   = MSGID(ALLY_CFG, 0x14),  // HD9200/HD9400 only
    CFG_ANTIJAM     = MSGID(ALLY_CFG, 0x15),
    CFG_BDGEO       = MSGID(ALLY_CFG, 0x16),
    CFG_CARRSMOOTH  = MSGID(ALLY_CFG, 0x17),
    CFG_GEOFENCE    = MSGID(ALLY_CFG, 0x18),
    CFG_SIMPLERST   = MSGID(ALLY_CFG, 0x40),
    CFG_SLEEP       = MSGID(ALLY_CFG, 0x41),
    CFG_PWRCTL      = MSGID(ALLY_CFG, 0x42),
    CFG_NMEAVER     = MSGID(ALLY_CFG, 0x43),
    CFG_PWRCTL2     = MSGID(ALLY_CFG, 0x44),
    MON_VER         = MSGID(ALLY_MON, 0x04),
    MON_INFO        = MSGID(ALLY_MON, 0x05),
    MON_TRKCHAN     = MSGID(ALLY_MON, 0x08),
    MON_RCVCLK      = MSGID(ALLY_MON, 0x09),
    MON_CWI         = MSGID(ALLY_MON, 0x0a),
    NAV_POSECEF     = MSGID(ALLY_NAV, 0x01),
    NAV_POSLLH      = MSGID(ALLY_NAV, 0x02),
    NAV_DOP         = MSGID(ALLY_NAV, 0x04),
    NAV_TIME        = MSGID(ALLY_NAV, 0x05),
    NAV_VELECEF     = MSGID(ALLY_NAV, 0x11),
    NAV_VELNED      = MSGID(ALLY_NAV, 0x12),
    NAV_TIMEUTC     = MSGID(ALLY_NAV, 0x21),
    NAV_CLOCK       = MSGID(ALLY_NAV, 0x22),
    NAV_CLOCK2      = MSGID(ALLY_NAV, 0x23),
    NAV_PVERR       = MSGID(ALLY_NAV, 0x26),
    NAV_SVINFO      = MSGID(ALLY_NAV, 0x30),
    NAV_SVSTATE     = MSGID(ALLY_NAV, 0x32),
    NAV_AUTO        = MSGID(ALLY_NAV, 0xc0),
    NAV_PVT         = MSGID(ALLY_NAV, 0xc1),
    /* NMEA_* for CFG-MSG, srouce:
     * https://docs.datagnss.com/rtk-board/#9download-the-latest-firmware
     */
    NMEA_GGA        = MSGID(ALLY_NMEA, 0x00),
    NMEA_GLL        = MSGID(ALLY_NMEA, 0x01),
    NMEA_GSA        = MSGID(ALLY_NMEA, 0x02),
    NMEA_GRS        = MSGID(ALLY_NMEA, 0x03),
    NMEA_GSV        = MSGID(ALLY_NMEA, 0x04),
    NMEA_RMC        = MSGID(ALLY_NMEA, 0x05),
    NMEA_VTG        = MSGID(ALLY_NMEA, 0x06),
    NMEA_ZDA        = MSGID(ALLY_NMEA, 0x07),
    NMEA_TXT        = MSGID(ALLY_NMEA, 0x20),
    /* RTCM_* for CFG-MSG, srouce:
     * https://docs.datagnss.com/rtk-board/#9download-the-latest-firmware
     */
    RTCM_1005       = MSGID(ALLY_RTCM, 0x05),
    RTCM_1019       = MSGID(ALLY_RTCM, 0x12),
    RTCM_1020       = MSGID(ALLY_RTCM, 0x14),
    RTCM_1077       = MSGID(ALLY_RTCM, 0x4d),
    RTCM_1087       = MSGID(ALLY_RTCM, 0x57),
    RTCM_1097       = MSGID(ALLY_RTCM, 0x06),
    RTCM_1117       = MSGID(ALLY_RTCM, 0x75),
    RTCM_1127       = MSGID(ALLY_RTCM, 0x7f),
    // RXM-*
    RXM_DUMPRAW     = MSGID(ALLY_RXM, 0x01),   // -DUM and -DUMPRAW the same??
    RXM_DUM         = MSGID(ALLY_RXM, 0x01),
    RXM_GALSAR      = MSGID(ALLY_RXM, 0x02),
} ally_msgs_t;

// 2 bytes leader, 2 bytes ID, 2 bytes payload length
#define ALLY_PREFIX_LEN 6

// ACK-* ids
static struct vlist_t vack_ids[] = {
    {ACK_ACK, "ACK-ACK"},
    {ACK_NAK, "ACK-NAK"},
    {0, NULL},
};

//CFG-ANTIJAM statsys_mask
static struct flist_t fsatsys[] = {
    {0x02, 0x02, "GPS"},
    {0x04, 0x04, "QZSS"},
    {0x08, 0x08, "SBAS"},
    {0x10, 0x10, "GAL"},
    {0x20, 0x20, "BDS"},
    {0x40, 0x40, "GLO"},
    {0, 0, NULL},
};

// CFG-GEOFENCE  cfg_flag
static struct vlist_t vcfg_flags[] = {
    {0, "None"},
    {1, "68%"},
    {2, "95%"},
    {3, "99.7%"},
    {4, "99.99%"},
    {5, "99.99999%"},
    {0, NULL},
};

/* CFG-NAVSAT enableMask
 * NAV-CLOCK2 sysmask */
static struct flist_t fenableMask[] = {
    {1, 1, "GPS L1"},
    {2, 2, "GLO G1"},
    {4, 4, "BDS B1"},
    {8, 8, "QZSS L1S"},
    {0x10, 0x10, "GAL E1"},
    {0x20, 0x20, "QZSS L1"},
    {0x40, 0x40, "SBAS L1"},
    {0x80, 0x80, "NAVIC L5"},
    {0x100, 0x100, "GPS L1C"},
    {0x200, 0x800, "GPS L5"},
    {0x400, 0x400, "GPS L2C"},
    {0x800, 0x40000, "x800"},
    {0x1000, 0x40000, "x1000"},
    {0x2000, 0x2000, "GLO G2"},
    {0x4000, 0x4000, "BDS B1C"},
    {0x8000, 0x8000, "BDS B2A"},
    {0x10000, 0x10000, "BDS B3I"},
    {0x20000, 0x20000, "BDS B5"},
    {0x40000, 0x40000, "BDS B2"},
    {0x80000, 0x80000, "0x80000"},
    {0x100000, 0x100000, "GAL E5A"},
    {0x200000, 0x200000, "GAL E5B"},
    {0x400000, 0x400000, "GAL E6"},
    {0x800000, 0x800000, "0x800000"},
    {0x1000000, 0x1000000, "QZSS L6"},
    {0x2000000, 0x2000000, "QZSS L1C"},
    {0x4000000, 0x4000000, "QZSS L5"},
    {0x8000000, 0x8000000, "QZSS L2C"},
    {0, 0, NULL},
};

// CFG-NMEAVER NMEA versions
static struct vlist_t vversions[] = {
    {0, "None"},
    {1, "V3.01"},
    {2, "V4.00"},
    {3, "V4.10"},
    {0, NULL},
};

// CFG-PRT, portID
static struct vlist_t vportID[] = {
    {0, "UART0"},
    {1, "UART1"},
    {0, NULL},
};

// NAV-AUTO fixstate
static struct vlist_t vfixstate[] = {
    {0, "None"},
    {1, "Aided"},         // ??
    {2, "Clock Bias"},    // ??
    {3, "2D"},
    {4, "3D"},
    {5, "DGNSS"},
    {6, "RTTK Flt"},
    {7, "RTTK Fix"},
    {0, NULL},
};

// NAV-PVT fixType fixstate
static struct vlist_t vfixType[] = {
    {0, "None"},
    {1, "DR"},
    {2, "2D"},
    {3, "3D"},
    {4, "Surveyed"},
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
// What does 15 or 21 mean??
static struct vlist_t vtime_navsys[] = {
    {0, "GPS"},
    {1, "BDS"},
    {2, "GLO"},
    {3, "GAL"},
    {0, NULL},
};

// NAV-TIMEUTC utcStandard
static struct vlist_t vtimeutc_utc[] = {
    {0x00, "NA"},
    {0x10, "NTSC"},
    {0x20, "USNO"},
    {0x30, "3"},
    {0x40, "EUL"},
    {0x50, "SU"},
    {0x60, "INDIA"},
    {0x0, NULL},
};

// NAV-TIMEUTC ValidFlag
static struct flist_t vtimeutc_valid[] = {
    {1, 1, "Valid tow"},
    {2, 2, "Valid week"},
    {4, 4, "Valid UTC"},    // Valid Leapsecond
    {0, 0, NULL},
};

// NAV-SVSTATE health
static struct vlist_t vsvstate_hlth[] = {
    {0, "Unk"},
    {1, "Healthy"},
    {2, "Unhealthy"},
    {0, NULL},
};

// NAV-SVSTATE eph_cource, alm_source
static struct vlist_t vsvstate_src[] = {
    {0, "NA"},
    {1, "GNSS"},
    {2, "Aided"},
    {0, NULL},
};

// NAV-SVSTATE visibility
static struct vlist_t vsvstate_viz[] = {
    {0, "Unk"},
    {1, "Below Hrz"},
    {2, "Above Hrz"},
    {3, "Above mask"},
    {0, NULL},
};

/* send a ALLYSTAR message.
 * calculate checksums, etc.
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
        (void)memcpy(&session->msgbuf[ALLY_PREFIX_LEN], msg, payload_len);
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
                          unsigned char *buf, size_t payload_len UNUSED)
{
    unsigned msgid = getbes16(buf, 2);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: %s: class: %02x, id: %02x\n",
             val2str(msgid, vack_ids),
             buf[2], buf[3]);
    return 0;
}

// CFG-*

/**
 * CFG-ANTIJAM - Constellations in useantijam settings
 *
 */
static gps_mask_t msg_cfg_antijam(struct gps_device_t *session,
                                  unsigned char *buf,
                                  size_t payload_len UNUSED)
{
    char buf2[80];
    unsigned satsys_mask = getleu16(buf, 0);
    unsigned threshold = getub(buf, 2);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: CFG-ANTIJAM: satsys_mask x%x(%s) threshold %u\n",
             satsys_mask,
             flags2str(satsys_mask, fsatsys, buf2, sizeof(buf2)),
             threshold);
    return 0;
}

/**
 * CFG_CARRSMOOTH
 *
 */
static gps_mask_t msg_cfg_carrsmooth(struct gps_device_t *session,
                                     unsigned char *buf,
                                     size_t payload_len UNUSED)
{
    int windows_value = getsb(buf, 0);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: CFG-CARRSMOOTH: windows_value %d\n",
             windows_value);
    return 0;
}

// CFG_FIXEDLLA    alt is altHAE, per datagnss.

/**
 * CFG-GEOFENCE -- genfences
 *
 */
static gps_mask_t msg_cfg_geofence(struct gps_device_t *session,
                                   unsigned char *buf, size_t payload_len)
{
    unsigned idx;
    unsigned llr_num = getub(buf, 0);
    unsigned cfg_flag = getub(buf, 1);
    unsigned gpio_enable = getub(buf, 2);
    unsigned polarity = getub(buf, 13);
    unsigned gpionum = getub(buf, 14);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: CFG-GEOFENCE: llr_num %u cfg_flag %u(%s) gpio_enable %u "
             "polarity %u gpionum %u\n",
             llr_num, cfg_flag,
             val2str(cfg_flag, vcfg_flags),
             gpio_enable, polarity, gpionum);

    if ((8 + (12 * llr_num)) != payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: CFG-GEOFENCE odd payload len %zu\n",
                 payload_len);
    }

    for (idx = 8; idx < payload_len; idx += 12) {
        double lon = getles32(buf, idx) / 1e7;
        double lat = getles32(buf, idx + 4) / 1e7;
        double radius = getleu16(buf, idx + 8) / 100.0;
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "ALLY: CFG-GEOFENCE: lat %.4f lon %.4f rad %.2f\n",
                 lat, lon, radius);
    }
    return 0;
}

/**
 * CFG-NAVSAT - Constellations in use
 *
 */
static gps_mask_t msg_cfg_navsat(struct gps_device_t *session,
                                 unsigned char *buf, size_t payload_len UNUSED)
{
    char buf2[200];
    unsigned long long enableMask = getleu32(buf, 0);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: CFG-NAVSAT: enableMask x%llx(%s)\n",
             enableMask,
             flags2str(enableMask, fenableMask, buf2, sizeof(buf2)));
    return 0;
}

/**
 * CFG-NMEAVER - NMEA version
 *
 */
static gps_mask_t msg_cfg_nmeaver(struct gps_device_t *session,
                                  unsigned char *buf,
                                  size_t payload_len UNUSED)
{
    unsigned version = getub(buf, 0);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: CFG-NMEAVER: version %u(%s)\n",
             version,
             val2str(version, vversions));
    return 0;
}

/**
 * CFG-PPS -- PPS settings
 *
 * payload 5 on early chips (Cynosure I)
 *         12 on later chops
 *
 */
static gps_mask_t msg_cfg_pps(struct gps_device_t *session,
                                  unsigned char *buf, size_t payload_len)
{
    if (5 == payload_len) {
        unsigned long long length = getleu32(buf, 0);
        unsigned polarity  = getub(buf, 4);

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "ALLY: CFG-PPS: length %llu polarity %u\n",
                 length, polarity);
    } else if (15 == payload_len) {
        unsigned long long period = getleu32(buf, 0);
        long long offset = getleu32(buf, 4);
        unsigned long long dutyCycle = getleu32(buf, 8);
        unsigned polarity  = getub(buf, 12);
        unsigned gpio  = getub(buf, 13);
        unsigned sync  = getub(buf, 14);

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "ALLY: CFG-PPS: period %llu offset %lld duty %llu "
                 "polarity %u gpio %u sync %u\n",
                 period, offset, dutyCycle,  polarity, gpio, sync);
    } else {
        // doc mentions payload 12, but no details.
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: CFG-PPS: odd payload len %zd\n",
                 payload_len);
    }

    return 0;
}

/**
 * CFG-PRT -- Coomm port settings
 *
 * Sadly, no way to tell what port is the current port!
 *
 */
static gps_mask_t msg_cfg_prt(struct gps_device_t *session,
                              unsigned char *buf, size_t payload_len UNUSED)
{
    unsigned portID  = getub(buf, 0);
    unsigned long long baudrate = getleu32(buf, 4);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: CFG-PRT: portID %u(%s) baudrate %llu\n",
             portID, val2str(portID, vportID), baudrate);

    return 0;
}

/**
 * CFG-SBAS - SBAS in use
 *
 */
static gps_mask_t msg_cfg_sbas(struct gps_device_t *session,
                               unsigned char *buf, size_t payload_len UNUSED)
{
    unsigned idx;

    for (idx = 0; idx < payload_len; idx += 2) {
        unsigned PRN = getub(buf, idx);
        unsigned enabled = getub(buf, idx + 1);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "ALLY: CFG-SBAS: PRN %u enabled: %u\n",
                 PRN, enabled);
    }
    return 0;
}

/* msg_cfg() -- handle CLASS-CFG
 */
static gps_mask_t msg_cfg(struct gps_device_t *session,
                          unsigned char *buf, size_t payload_len)
{
    unsigned msgid = getbes16(buf, 2);
    gps_mask_t mask = 0;
    size_t needed_len;
    const char *msg_name;
    gps_mask_t (* p_decode)(struct gps_device_t *, unsigned char *, size_t);

    switch (msgid) {
    case CFG_ANTIJAM:
        needed_len = 3;
        msg_name = "CFG-ANTIJAM";
        p_decode = msg_cfg_antijam;
        break;
    case CFG_CARRSMOOTH:
        msg_name = "CFG-CARRSMOOTH";
        needed_len = 1;
        p_decode = msg_cfg_carrsmooth;
        break;
    case CFG_GEOFENCE:
        msg_name = "CFG-GEOFENCE";
        needed_len = 8;
        p_decode = msg_cfg_geofence;
        break;
    case CFG_NAVSAT:
        needed_len = 4;
        msg_name = "CFG-NAVSAT";
        p_decode = msg_cfg_navsat;
        break;
    case CFG_NMEAVER:
        msg_name = "CFG-NMEAVER";
        needed_len = 1;
        p_decode = msg_cfg_nmeaver;
        break;
    case CFG_PPS:
        msg_name = "CFG-PPS";
        needed_len = 5;
        p_decode = msg_cfg_pps;
        break;
    case CFG_PRT:
        msg_name = "CFG-PRT";
        needed_len = 2;
        p_decode = msg_cfg_prt;
        break;
    case CFG_SBAS:
        msg_name = "CFG-SBAS";
        needed_len = 0;
        p_decode = msg_cfg_sbas;
        break;
    default:
        msg_name = "CFG-x";
        needed_len = 0;
        p_decode = NULL;
        break;
    }
    if (needed_len > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: %s: runt payload len %zd\n", msg_name, payload_len);
        return 0;
    }
    if (NULL == p_decode) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: Unknown CFG-%02x payload_len %zd\n",
                 msgid & 0xff, payload_len);
        return 0;
    }
    mask = p_decode(session, &buf[ALLY_PREFIX_LEN], payload_len);
    return mask;
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
                              size_t payload_len UNUSED)
{
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

// NAV-*

/**
 * NAV-AUTO -- GNSS info
 *
 */
static gps_mask_t msg_nav_auto(struct gps_device_t *session,
                                unsigned char *buf, size_t payload_len UNUSED)
{
    gps_mask_t mask = 0;
    unsigned mode = MODE_NOT_SEEN;
    unsigned status = STATUS_UNK;
    struct tm unpacked_date = {0};

    unsigned fixstate = getub(buf, 0);
    unsigned year = getleu16(buf, 1);
    unsigned month = getub(buf, 3);
    unsigned day = getub(buf, 4);
    unsigned hour = getub(buf, 5);
    unsigned min = getub(buf, 6);
    unsigned sec = getub(buf, 7);
    long long lon = getles32(buf, 8);
    long long lat = getles32(buf, 12);
    long long altHAE = getles32(buf, 16);
    // no place for speed_3d....
    unsigned speed_3d = getleu16(buf, 20);
    int heading = getles16(buf, 22);
    unsigned pDOP = getles16(buf, 24);
    unsigned hDOP = getles16(buf, 26);
    unsigned vDOP = getles16(buf, 28);
    /* We don't use satInUse, satInView.  ALLYSTAR counts two signals from
     * one sat, as one sat */
    unsigned satInUse = getub(buf, 30);
    unsigned satInView = getub(buf, 31);

    switch (fixstate) {
    case 0:    // No fix
        mode |= MODE_NO_FIX;
        status = STATUS_UNK;
        mask |= STATUS_SET | MODE_SET;
        break;
    case 1:    // Aided fix ??
        FALLTHROUGH
    case 2:    // Clock bias fix ??
        // assume these are like surveyed-in
        mode = MODE_3D;
        status = STATUS_TIME;
        mask |= STATUS_SET | MODE_SET;
        break;
    case 3:         // 2D fix
        mode = MODE_2D;
        status = STATUS_GPS;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    case 4:         // 3D fix
        mode = MODE_3D;
        status = STATUS_GPS;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    case 5:         // DGNSS
        mode = MODE_3D;
        status = STATUS_DGPS;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    case 6:         // RTK Float
        mode = MODE_3D;
        status = STATUS_RTK_FLT;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    case 7:         // RTK Fix
        mode = MODE_3D;
        status = STATUS_RTK_FIX;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    default:
        // huh?
        break;
    }
    session->newdata.mode = mode;
    session->newdata.status = status;

    // We don't know if time, or leapseconds, is valid.
    unpacked_date.tm_year = getleu16(buf, 4) - 1900;
    unpacked_date.tm_mon = getub(buf, 6) - 1;
    unpacked_date.tm_mday = getub(buf, 7);
    unpacked_date.tm_hour = getub(buf, 8);
    unpacked_date.tm_min = getub(buf, 9);
    unpacked_date.tm_sec = getub(buf, 10);
    session->newdata.time.tv_sec = mkgmtime(&unpacked_date);
    // If rate highter than 1Hz, we have no idea of sub-seconds...
    session->newdata.time.tv_nsec = 0;
    TS_NORM(&session->newdata.time);
    if (3 <= fixstate) {
        // not sure baout states 1 and 2
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }

    session->newdata.longitude = lon / 1e7;
    session->newdata.latitude = lat / 1e7;
    session->newdata.altHAE = altHAE / 1e3;

    session->newdata.track = heading / 100.0;
    mask |= TRACK_SET;

    if (9999 > pDOP) {
        session->gpsdata.dop.pdop = pDOP / 100.0;
        mask |= DOP_SET;
    }
    if (9999 > hDOP) {
        session->gpsdata.dop.hdop = hDOP / 100.0;
        mask |= DOP_SET;
    }
    if (9999 > vDOP) {
        session->gpsdata.dop.vdop = vDOP / 100.0;
        mask |= DOP_SET;
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-AUTO: time %lld fixstate %u %u/%u/%u "
             "%u:%02u:%02u lon %.7f lat %.7f altHAE %.3f "
             "speed_3d %u, heading %.2f pDOP %.2f hDOP %.2f vDOP %.2f "
             "satInUse %u satInView %u\n",
             (long long)session->newdata.time.tv_sec,
             fixstate,
             year, month, day, hour, min, sec,
             session->newdata.longitude,
             session->newdata.latitude,
             session->newdata.altHAE,
             speed_3d,
             session->newdata.track,
             session->gpsdata.dop.pdop,
             session->gpsdata.dop.hdop,
             session->gpsdata.dop.vdop,
             satInUse, satInView);
     GPSD_LOG(LOG_IO, &session->context->errout,
              "UBX: NAV-SUTO: fixstate %u(%s) mode %u(%s) status %u(%s)\n",
             fixstate, val2str(fixstate, vfixstate),
             session->newdata.mode,
             val2str(session->newdata.mode, vmode_str),
             session->newdata.status,
             val2str(session->newdata.status, vstatus_str));
    return mask;
}

/**
 * Clock Solution NAV-CLOCK
 * Seems to match UBX-NAV-CLOCK
 *
 */
static gps_mask_t msg_nav_clock(struct gps_device_t *session,
                                unsigned char *buf, size_t payload_len UNUSED)
{
    unsigned long tAcc = getleu32(buf, 12);
    unsigned long fAcc = getleu32(buf, 16);

    session->driver.ubx.iTOW = getleu32(buf, 0);
    session->gpsdata.fix.clockbias = getles32(buf, 4);
    session->gpsdata.fix.clockdrift = getles32(buf, 8);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-CLOCK: iTOW=%lld clkB %ld clkD %ld tAcc %lu fAcc %lu\n",
             (long long)session->driver.ubx.iTOW,
             session->gpsdata.fix.clockbias,
             session->gpsdata.fix.clockdrift,
             tAcc, fAcc);
    return 0;
}

/**
 * NAV-CLOCK2 Sat Clock Solution
 * Untested, unsupported on TAU1202, SW 3.018.a3f23db
 *
 */
static gps_mask_t msg_nav_clock2(struct gps_device_t *session,
                                 unsigned char *buf, size_t payload_len)
{
    char buf2[80];
    unsigned u;

    unsigned long long numClk = getleu32(buf, 4);
    session->driver.ubx.iTOW = getleu32(buf, 0);

    for (u = 8; u < payload_len; u += 12) {
        unsigned long long sysmask = getleu32(buf, u);
        long long clkB = getles32(buf, u + 4);              // ns
        unsigned long long tAcc = getleu16(buf, u + 8);     // ns

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "ALLY: CFG-CLOCK2: sysmask x%llx(%s) clkB %lld yAcc %llu\n",
                 sysmask,
                 flags2str(sysmask, fenableMask, buf2, sizeof(buf2)),
                 clkB, tAcc);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-CLOCK2: iTOW=%lld numClk %lld\n",
             (long long)session->driver.ubx.iTOW,
             numClk);
    return 0;
}

/**
 * NAV-DOP, Dilution of precision message
 * Seems to match UBX-NAV-DOP
 *
 */
static gps_mask_t msg_nav_dop(struct gps_device_t *session,
                              unsigned char *buf, size_t payload_len UNUSED)
{
    unsigned u;
    gps_mask_t mask = 0;

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
                                 unsigned char *buf, size_t payload_len UNUSED)
{
    gps_mask_t mask = ECEF_SET;

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

/*
 * Geodetic position solution message
 * NAV-POSLLH, Class 1, ID 2
 * Seems same as UBX-MON-POSLLH
 *
 * This message does not bother to tell us if it is valid.
 * No mode, so limited usefulness
 */
static gps_mask_t msg_nav_posllh(struct gps_device_t *session,
                                 unsigned char *buf,
                                 size_t payload_len UNUSED)
{
    gps_mask_t mask = 0;

    session->driver.ubx.iTOW = getleu32(buf, 0);
    session->newdata.longitude = getles32(buf, 4) / 1e7;
    session->newdata.latitude = getles32(buf, 8) / 1e7;
    // altitude WGS84
    session->newdata.altHAE = getles32(buf, 12) / 1e3;
    // altitude MSL
    session->newdata.altMSL = getles32(buf, 16) / 1e3;
    // Let gpsd_error_model() deal with geoid_sep

    // Horizontal accuracy estimate in mm, unknown type
    session->newdata.eph = getleu32(buf, 20) / 1e3;
    // Vertical accuracy estimate in mm, unknown type
    session->newdata.epv = getleu32(buf, 24) / 1e3;

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

/*
 * Positioning velocity error estimation
 * NAV-PVERR, Class 1, ID x26
 * Untested, unsupported on TAU1202, SW 3.018.a3f23db
 * UBX has no equivalent
 * Most of the data in $GPGST, but missing the ellipsoid data.
 */
static gps_mask_t msg_nav_pverr(struct gps_device_t *session,
                                unsigned char *buf,
                                size_t payload_len UNUSED)
{
    gps_mask_t mask = 0;

    // Probably NOT fix time
    session->driver.ubx.iTOW = getleu32(buf, 0);
    // latitude, millimeters
    session->gpsdata.gst.lat_err_deviation = getles32(buf, 4)/ 1000.0;
    // longitude, millimeters
    session->gpsdata.gst.lon_err_deviation = getles32(buf, 4)/ 1000.0;
    // altitude, millimeters
    session->gpsdata.gst.alt_err_deviation = getles32(buf, 4)/ 1000.0;
    // velocity east, millimeters / second
    session->gpsdata.gst.ve_err_deviation  = getles32(buf, 4)/ 1000.0;
    // velocity north, millimeters / second
    session->gpsdata.gst.vn_err_deviation = getles32(buf, 4)/ 1000.0;
    // velocity up, millimeters / second
    session->gpsdata.gst.vu_err_deviation = getles32(buf, 4)/ 1000.0;

    GPSD_LOG(LOG_PROG, &session->context->errout,
        "ALLY: NAV-PVERR: iTOW %lld stdlat %.3f stdlon %.3f stdalt %.3f "
        "stdve %.3f stdvn %.3f stdvu %.3f\n",
        (long long)session->driver.ubx.iTOW,
        session->gpsdata.gst.lat_err_deviation,
        session->gpsdata.gst.lon_err_deviation,
        session->gpsdata.gst.alt_err_deviation,
        session->gpsdata.gst.ve_err_deviation,
        session->gpsdata.gst.vn_err_deviation,
        session->gpsdata.gst.vu_err_deviation);

    mask = GST_SET | ONLINE_SET;
    return mask;
}

/**
 * NAV-PVT
 * Untested, unsupported on TAU1202, SW 3.018.a3f23db
 *
 */
static gps_mask_t msg_nav_pvt(struct gps_device_t *session,
                              unsigned char *buf, size_t payload_len UNUSED)
{
    gps_mask_t mask = 0;
    unsigned mode = MODE_NOT_SEEN;
    unsigned status = STATUS_UNK;
    struct tm unpacked_date = {0};
    timespec_t ts_tow;

    unsigned year = getleu16(buf, 4);
    unsigned month = getub(buf, 6);
    unsigned day = getub(buf, 7);
    unsigned hour = getub(buf, 8);
    unsigned min = getub(buf, 9);
    unsigned sec = getub(buf, 10);
    unsigned valid = getub(buf, 10);      // undocumented!
    unsigned long long tAcc = getleu32(buf, 12);
    long long nano = getles32(buf, 16);
    unsigned fixType = getub(buf, 20);
    // 21, 22 reserved
    /* We don't use numSV.  ALLYSTAR counts two signals from
     * one sat, as one sat */
    unsigned numSV = getub(buf, 23);
    long long lon = getles32(buf, 24);
    long long lat = getles32(buf, 28);
    long long altHAE = getles32(buf, 32);
    long long hMSL = getles32(buf, 36);
    unsigned long long hAcc = getleu32(buf, 40);
    unsigned long long vAcc = getleu32(buf, 44);
    long long Vel_N = getles32(buf, 48);
    long long Vel_E = getles32(buf, 52);
    long long Vel_D = getles32(buf, 56);
    long long gSpeed = getles32(buf, 60);        // signed??
    unsigned long long headMot = getles32(buf, 64);
    long long sAcc = getleu32(buf, 68);
    long long headAcc = getleu32(buf, 72);
    int pDOP = getles16(buf, 76);
    // Different than headMot ??
    unsigned long long headVeh = getles32(buf, 84); 

    session->driver.ubx.iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->driver.ubx.iTOW);

    switch (fixType) {
    case 0:    // No fix
        mode |= MODE_NO_FIX;
        status = STATUS_UNK;
        mask |= STATUS_SET | MODE_SET;
        break;
    case 1:    // Aided fix ??
        FALLTHROUGH
    case 2:    // Clock bias fix ??
        // assume these are like surveyed-in
        mode = MODE_3D;
        status = STATUS_TIME;
        mask |= STATUS_SET | MODE_SET;
        break;
    case 3:         // 2D fix
        mode = MODE_2D;
        status = STATUS_GPS;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    case 4:         // 3D fix
        mode = MODE_3D;
        status = STATUS_GPS;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    case 5:         // DGNSS
        mode = MODE_3D;
        status = STATUS_DGPS;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    case 6:         // RTK Float
        mode = MODE_3D;
        status = STATUS_RTK_FLT;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    case 7:         // RTK Fix
        mode = MODE_3D;
        status = STATUS_RTK_FIX;
        mask |= MODE_SET | STATUS_SET | LATLON_SET;
        break;
    default:
        // huh?
        break;
    }
    session->newdata.mode = mode;
    session->newdata.status = status;

    unpacked_date.tm_year = year;
    unpacked_date.tm_mon = month;
    unpacked_date.tm_mday = day;
    unpacked_date.tm_hour = hour;
    unpacked_date.tm_min = min;
    unpacked_date.tm_sec = sec;
    // FIXME:  add in fractional iTOW
    session->newdata.time.tv_sec = mkgmtime(&unpacked_date);
    session->newdata.time.tv_nsec = nano;
    TS_NORM(&session->newdata.time);
    if (0 < fixType) {
        // accept DR
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }

    session->newdata.longitude = lon / 1e7;
    session->newdata.latitude = lat / 1e7;
    session->newdata.altHAE = altHAE / 1e3;
    session->newdata.altMSL = hMSL / 1e3;
    session->newdata.speed = gSpeed / 1e3;

    session->newdata.track = headMot / 100.0;
    mask |= TRACK_SET;

    if (9999 > pDOP) {
        session->gpsdata.dop.pdop = pDOP / 100.0;
        mask |= DOP_SET;
    }
    session->newdata.NED.velN = Vel_N / 1000.0;
    session->newdata.NED.velE = Vel_E / 1000.0;
    session->newdata.NED.velD = Vel_D / 1000.0;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-PVT: time %lld%.09ld %u/%u/%u %u:%02u:%02u valid x%x "
             "tAcc %llu fixType %u  numSv %u lon %.7f lat %.7f "
             "altHAE %.3f altMSL %.3f hAcc %llu vAcc %llu "
             "speed %.3f, headMot %.2f sAcc ^%.55f headAcc %.5f pDOP %.2f "
             "headVeh %.2f\n",
             (long long)session->newdata.time.tv_sec,
             session->newdata.time.tv_nsec,
             year, month, day, hour, min, sec,
             valid, tAcc,
             fixType, numSV,
             session->newdata.longitude,
             session->newdata.latitude,
             session->newdata.altHAE,
             session->newdata.altMSL,
             hAcc, vAcc,
             session->newdata.speed,
             session->newdata.track,
             sAcc / 1e5, headAcc / 1e5,
             session->gpsdata.dop.pdop,
             headVeh / 1e5);
     GPSD_LOG(LOG_IO, &session->context->errout,
              "UBX: NAV-PVT fixType %u(%s) mode %u(%s) status %u(%s)\n",
             fixType, val2str(fixType, vfixType),
             session->newdata.mode,
             val2str(session->newdata.mode, vmode_str),
             session->newdata.status,
             val2str(session->newdata.status, vstatus_str));
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
                               unsigned char *buf, size_t payload_len UNUSED)
{
    // char buf2[80];
    unsigned i, nsv, st;
    long long nchan;
    timespec_t ts_tow;

    session->driver.ubx.iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->driver.ubx.iTOW);
    session->gpsdata.skyview_time =
        gpsd_gpstime_resolv(session, session->context->gps_week, ts_tow);

    nchan = getleu32(buf, 4);  // 32 bits, seriously??
    if (nchan > MAXCHANNELS) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: NAV SVINFO: runt >%d reported visible\n",
                 MAXCHANNELS);
        return 0;
    }

    // FIXME: check payload_len
    gpsd_zero_satellites(&session->gpsdata);
    nsv = 0;
    for (i = st = 0; i < nchan; i++) {
        // No info on sigid???
        unsigned off = 24 * i;
        unsigned nmea_PRN;
        // v 2.3.1, 8 bit chand and 8-bit svid became 16-bit svid.
        // no doc on svid numbers... same as NMEA??
        unsigned ally_svid = getleu16(buf, off + 8);
        // no doc on flags
        unsigned flags = getub(buf, off + 10);
        // no doc on quality
        unsigned quality = getub(buf, off + 11);
        unsigned cno = getub(buf, off + 12);
        int el = getsb(buf, off + 13);
        int az = getles16(buf, off + 14);
        long long prRes = getles32(buf, off + 16);  // pseudorange residue, cm
        // pr Rate, m/s
        session->gpsdata.skyview[st].prRate = getlef32((const char*)buf,
                                                        off + 20);
        // pseudorange, m
        session->gpsdata.skyview[st].pr = getled64((const char*)buf, off + 24);

        nmea_PRN = ally_svid;
        // nmea_PRN = ubx_to_prn(ubx_PRN,
                              // &session->gpsdata.skyview[st].gnssid,
                              // &session->gpsdata.skyview[st].svid);

        // if (1 > nmea_PRN) {
        //     // skip bad PRN
        //     GPSD_LOG(LOG_PROG, &session->context->errout,
        //              "ALLY: NAV-SVINFO bad NMEA PRN %d\n", nmea_PRN);
        //     continue;
        // }
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
        // No health data, no used data.
        // flags and quality undocumented.
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "ALLY: NAV-SVINFO ally_svid %u  gnssid %d, svid %d "
                 "nmea_PRN %d flags x%x az %.0f el %.0f cno %.0f prRes %.2f "
                 "quality x%x, prRate %f pr %.4f\n",
                 ally_svid,
                 session->gpsdata.skyview[st].gnssid,
                 session->gpsdata.skyview[st].svid, nmea_PRN, flags,
                 session->gpsdata.skyview[st].azimuth,
                 session->gpsdata.skyview[st].elevation,
                 session->gpsdata.skyview[st].ss,
                 session->gpsdata.skyview[st].prRes,
                 quality,
                 session->gpsdata.skyview[st].prRate,
                 session->gpsdata.skyview[st].pr);
        st++;
    }

    // session->gpsdata.satellites_visible = (int)st;
    // session->gpsdata.satellites_used = (int)nsv;
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-SVINFO: visible=%d used=%d\n",
             st, nsv);
    // not ready for prime time, no sigid!!
    // return SATELLITE_SET | USED_IS;
    return 0;
}

/**
 * NAV-SVSTATE
 */
static gps_mask_t msg_nav_svstate(struct gps_device_t *session,
                                  unsigned char *buf, size_t payload_len)
{
    unsigned idx;
    gps_mask_t mask = 0;
    // char ts_buf[TIMESPEC_LEN];

    unsigned numSV = getub(buf, 4);                // 32 bits?!?!
    session->driver.ubx.iTOW = getleu32(buf, 0);   // iTow, ms
    // 5 reserved

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-SVSTATE: iTOW %lld numSV %u payload_len %lld\n",
             (long long)session->driver.ubx.iTOW, numSV,
             (long long) payload_len);
    for (idx = 8; idx < payload_len; idx += 4) {
        unsigned svid = getleu16(buf, idx);         // NMEA PRN
        unsigned eph_state = getub(buf, idx + 2);   // ephemeris state
        unsigned alm_state = getub(buf, idx + 3);   // ephemeris state

        GPSD_LOG(LOG_INFO, &session->context->errout,
                 "ALLY: NAV-SVSTATE: svid %4u "
                 "eph x%x(use %d viz %u(%s) hlth %u(%s) "
                 "alm x%x(use %d asrc %u(%s) esrc %u(%s))\n",
                 svid, eph_state, (eph_state >> 4) & 0x0f,
                 (eph_state >> 2) & 0x03,
                 val2str((eph_state >> 2) & 0x03, vsvstate_viz),
                 eph_state & 0x03,
                 val2str(eph_state & 0x03, vsvstate_hlth),
                 alm_state,
                 (alm_state >> 4) & 0x0f,
                 (alm_state >> 2) & 0x03,
                 val2str((eph_state >> 2) & 0x03, vsvstate_src),
                 alm_state & 0x03,
                 val2str(eph_state & 0x03, vsvstate_src));
    }
    return mask;
}

/**
 * GPS Leap Seconds - NAV-TIME
 * sorta like UBX-NAV-TIMEGPS
 */
static gps_mask_t msg_nav_time(struct gps_device_t *session,
                               unsigned char *buf, size_t payload_len UNUSED)
{
    char buf2[80];
    gps_mask_t mask = 0;
    char ts_buf[TIMESPEC_LEN];

    // TAU1202 returns 21 ???
    unsigned navSys = getub(buf, 0);               // which constellation
    unsigned flag = getub(buf, 1);                 // Validity flags
    int FracTow = getles16(buf, 2);                // fractional TOW, ns
    unsigned week = getleu16(buf, 8);
    int leapSec = getsb(buf, 10);
    // timeErr in ns, unknown type (1 sigma, 50%, etc.)
    double timeErr = (double)getleu32(buf, 12);

    session->driver.ubx.iTOW = getleu32(buf, 4);   // refTow, ms

    // Valid leap seconds ?
    if (4 == (flag & 4)) {
        session->context->leap_seconds = leapSec;
        session->context->valid |= LEAP_SECOND_VALID;
    }

    // Valid GPS time of week and week number
    if (3 == (flag & 3)) {
        timespec_t ts_tow;

        MSTOTS(&ts_tow, session->driver.ubx.iTOW);
        ts_tow.tv_nsec += FracTow;
        TS_NORM(&ts_tow);
        session->newdata.time = gpsd_gpstime_resolv(session, week, ts_tow);

        session->newdata.ept = timeErr / 1e9;
        mask |= (TIME_SET | NTPTIME_IS);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-TIME: time=%s navSys %u flag x%x FracTow %d "
             "refTow %lld week %u leapSec %d timeErr %g\n",
             timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)),
             navSys, flag, FracTow,
             (long long)session->driver.ubx.iTOW, week,
             leapSec, timeErr);
    GPSD_LOG(LOG_IO, &session->context->errout,
             "ALLY: NAV-TIME: navSys %s flag:%s\n",
	     val2str(navSys, vtime_navsys),
	     flags2str(flag, vtime_flags, buf2, sizeof(buf2)));
    return mask;
}

/**
 * NAV-TIMEUTC
 */
static gps_mask_t msg_nav_timeutc(struct gps_device_t *session,
                                  unsigned char *buf, size_t payload_len UNUSED)
{
    char buf2[80];
    gps_mask_t mask = 0;
    // char ts_buf[TIMESPEC_LEN];

    // tAcc in ns, unknown type (1 sigma, 50%, etc.)
    unsigned long long tAcc = getleu32(buf, 4);
    long long nano = getles32(buf, 8);        // fractional TOW, ns
    unsigned year = getleu16(buf, 12);
    unsigned month = getub(buf, 14);
    unsigned day = getub(buf, 15);
    unsigned hour = getub(buf, 16);
    unsigned min = getub(buf, 17);
    unsigned sec = getub(buf, 18);
    unsigned ValidFlag = getub(buf, 19);
    session->driver.ubx.iTOW = getleu32(buf, 0);   // iTow, ms

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-TIMEUTC: iTOW %lld tAcc %llu nano %lld "
             "time %u/%u/%u %u:%02u:%02u ValidFlag x%x\n",
             (long long)session->driver.ubx.iTOW, tAcc, nano,
             year, month, day, hour, min, sec, ValidFlag);
             // timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)),
    GPSD_LOG(LOG_IO, &session->context->errout,
             "ALLY: NAV-TIMEUTC: ValidFlag:x%x(%s) utc %s\n",
             ValidFlag,
             flags2str(ValidFlag, vtimeutc_valid, buf2, sizeof(buf2)),
             val2str(ValidFlag, vtimeutc_utc));
    return mask;
}

/**
 * NAV-VELECEF
 * Untested, unsupported on TAU1202, SW 3.018.a3f23db
 */
static gps_mask_t msg_nav_velecef(struct gps_device_t *session,
                                  unsigned char *buf, size_t payload_len UNUSED)
{
    gps_mask_t mask = VECEF_SET;

    session->driver.ubx.iTOW = getleu32(buf, 0);              // iTow, ms
    session->newdata.ecef.vx = getles32(buf, 4) / 100.0;      // cm/s
    session->newdata.ecef.vy = getles32(buf, 8) / 100.0;      // cm/s
    session->newdata.ecef.vz = getles32(buf, 12) / 100.0;     // cm/s
    // sAcc (vAcc) in ns, unknown type (1 sigma, 50%, etc.)
    session->newdata.ecef.vAcc = getleu32(buf, 16) / 100.0;   // cm/s

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-VELECEF: iTOW %lld vECEF %.2f %.2f %.2f sAcc %.2f\n",
             (long long)session->driver.ubx.iTOW,
             session->newdata.ecef.vx,
             session->newdata.ecef.vy,
             session->newdata.ecef.vz,
             session->newdata.ecef.vAcc);
    return mask;
}

/**
 * NAV-VELNED
 * Untested, unsupported on TAU1202, SW 3.018.a3f23db
 */
static gps_mask_t msg_nav_velned(struct gps_device_t *session,
                                 unsigned char *buf, size_t payload_len UNUSED)
{
    gps_mask_t mask = VNED_SET;

    session->driver.ubx.iTOW = getleu32(buf, 0);      // iTow, ms
    session->newdata.NED.velN = getles32(buf, 4) / 100.0;
    session->newdata.NED.velE = getles32(buf, 8) / 100.0;
    session->newdata.NED.velD = getles32(buf, 12) / 100.0;

    // sAcc (vAcc) in ns, unknown type (1 sigma, 50%, etc.)
    session->newdata.ecef.vAcc = getleu32(buf, 28) / 100.0;   // cm/s

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "ALLY: NAV-VELNED: iTOW %lld vNED %.2f %.2f %.2f sAcc %.2f\n",
             (long long)session->driver.ubx.iTOW,
             session->newdata.ecef.vx,
             session->newdata.ecef.vy,
             session->newdata.ecef.vz,
             session->newdata.ecef.vAcc);
    return mask;
}

/* msg_nav() -- handle CLASS-NAV and CLASS-MON
 */
static gps_mask_t msg_nav(struct gps_device_t *session,
                          unsigned char *buf, size_t payload_len)
{
    unsigned msgid = getbes16(buf, 2);
    gps_mask_t mask = 0;
    size_t needed_len;
    const char *msg_name;
    gps_mask_t (* p_decode)(struct gps_device_t *, unsigned char *, size_t);

    switch (msgid) {
    case MON_VER:
        msg_name ="MON-VER";
        needed_len = 32;
        p_decode = msg_mon_ver;
        break;
    case NAV_AUTO:
        msg_name ="NAV-AUTO";
        needed_len = 32;
        p_decode = msg_nav_auto;
        break;
    case NAV_CLOCK:
        msg_name ="NAV-CLOCK";
        needed_len = 20;
        p_decode = msg_nav_clock;
        break;
    case NAV_CLOCK2:
        msg_name ="NAV-CLOCK2";
        needed_len = 8;
        p_decode = msg_nav_clock2;
        break;
    case NAV_DOP:
        msg_name ="NAV-DOP";
        needed_len = 18;
        p_decode = msg_nav_dop;
        break;
    case NAV_POSECEF:
        msg_name ="NAV-POSECEF";
        needed_len = 20;
        p_decode = msg_nav_posecef;
        break;
    case NAV_POSLLH:
        msg_name ="NAV-POSLLH";
        needed_len = 28;
        p_decode = msg_nav_posllh;
        break;
    case NAV_PVERR:
        msg_name ="NAV-PVERR";
        needed_len = 28;
        p_decode = msg_nav_pverr;
        break;
    case NAV_PVT:
        msg_name ="NAV-PVT";
        needed_len = 88;
        p_decode = msg_nav_pvt;
        break;
    case NAV_SVINFO:
        msg_name ="NAV-SVINFO";
        needed_len = 8;
        p_decode = msg_nav_svinfo;
        break;
    case NAV_SVSTATE:
        msg_name ="NAV-SVSTATE";
        needed_len = 8;
        p_decode = msg_nav_svstate;
        break;
    case NAV_TIME:
        msg_name ="NAV-TIME";
        needed_len = 16;
        p_decode = msg_nav_time;
        break;
    case NAV_TIMEUTC:
        msg_name ="NAV-TIMEUTC";
        needed_len = 20;
        p_decode = msg_nav_timeutc;
        break;
    case NAV_VELECEF:
        msg_name ="NAV-VELECEF";
        needed_len = 20;
        p_decode = msg_nav_velecef;
        break;
    case NAV_VELNED:
        msg_name ="NAV-VELNED";
        needed_len = 36;
        p_decode = msg_nav_velned;
        break;
    default:
        msg_name ="NAV-unk";
        needed_len = 0;
        p_decode = NULL;
        break;
    }
    if (needed_len > payload_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: %s: runt payload len %zd need %zd\n",
                 msg_name, payload_len, needed_len);
        return 0;
    }
    if (NULL == p_decode) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: Unknown NAV-%02x payload_len %zd\n",
                 msgid & 0xff, payload_len);
        return 0;
    }
    mask = p_decode(session, &buf[ALLY_PREFIX_LEN], payload_len);
    return mask;
}

static gps_mask_t ally_parse(struct gps_device_t * session, unsigned char *buf,
                            size_t len)
{
    unsigned payload_len;
    unsigned  msg_class;
    unsigned  msg_id;
    unsigned msgid = getbes16(buf, 2);
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
        if (2 > payload_len) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "ALLY: %s-: runt payload len %zd\n",
                     val2str(msgid, vack_ids), payload_len);
            break;
        }
        mask = msg_ack(session, buf, payload_len);
        break;
    case ALLY_AID:
        // Deprecated
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: AID- %02x length %zd/%u)\n",
                 msg_id, len, payload_len);
        break;
    case ALLY_CFG:
        mask = msg_cfg(session, buf, payload_len);
        break;
    case ALLY_MON:
        FALLTHROUGH
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

    /* handle the init queue.  Some parts get cranky when they
     * get too many configuration changes at once.
     */
    if (!session->context->readonly &&
        0 <= session->queue &&
        100 > session->queue) {
        unsigned char msg[4] = {0};

        GPSD_LOG(LOG_DATA, &session->context->errout,
                 "ALLY: queue %d\n", session->queue);


        switch (session->queue) {
        case 10:
            // start at 10 so the initial configuration can finish first.
            // Poll CFG-ANTIJAM
            (void)ally_write(session, ALLY_CFG, 0x15, NULL, 0);
            break;
        case 14:
            // Poll CFG_CARRSMOOTH
            (void)ally_write(session, ALLY_CFG, 0x17, NULL, 0);
            break;
        case 18:
            // Poll CFG_GEOFENCE
            (void)ally_write(session, ALLY_CFG, 0x18, NULL, 0);
            break;
        case 22:
            // Poll CFG-NAVSAT
            (void)ally_write(session, ALLY_CFG, 0x0c, NULL, 0);
            break;
        case 26:
            // Poll CFG-NMEAVER
            (void)ally_write(session, ALLY_CFG, 0x43, NULL, 0);
            break;
        case 30:
            // Poll CFG-PRT
            (void)ally_write(session, ALLY_CFG, 0x00, NULL, 0);
            break;
        case 34:
            // Poll CFG-PRT(UART1)
            msg[0] = 0;
            (void)ally_write(session, ALLY_CFG, 0x00, msg, 1);
            break;
        case 38:
            // Poll CFG-PRT(UART2)
            msg[0] = 1;
            (void)ally_write(session, ALLY_CFG, 0x00, msg, 1);
            break;
        case 40:
            // Poll CFG-SBAS
            (void)ally_write(session, ALLY_CFG, 0x0e, NULL, 0);
            break;
        case 44:
            // turn on rate one NAV-PVT
            // prolly no need for NAV-AUTO and NAV-POLL
            putbe16(msg, 0, NAV_PVT);
            msg[2] = 0x01;          // rate, one
            (void)ally_write(session, ALLY_CFG, 0x01, msg, 3);
            break;
        default:
            break;
        }
        session->queue++;
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
    unsigned u;

    // TAU1202 seems OK with being blasted with CFG-MSG's.
    static ally_msgs_t all_nav[] = {
        // prolly no need for NAV-AUTO and NAV-POLL
        NAV_AUTO,
        NAV_CLOCK,
        // NAV_CLOCK2,           // unsuported TAU1202
        NAV_DOP,
        NAV_POSECEF,
        NAV_POSLLH,
        // NAV_PVERR,            // unsuported TAU1202
        // NAV_PVT,              // unsuported TAU1202
        NAV_SVINFO,
        NAV_SVSTATE,
        NAV_TIME,
        NAV_TIMEUTC,
        // NAV_VELECEF,          // unsuported TAU1202
        // NAV_VELNED,           // unsuported TAU1202
    };

    // turn on all binary NAV- messages we know of
    for (u = 0; u < ROWS(all_nav); u++) {
        // turn on rate one
        putbe16(msg, 0, all_nav[u]);
        msg[2] = 0x01;          // rate, one
        (void)ally_write(session, ALLY_CFG, 0x01, msg, 3);
    }
}

/* speed()
 * WIP: implement speed changing.
 * Luckily ALLYSTAR is only 8N1.
 * Unluckily, no way to know what port we are on...
 */
static bool speed(struct gps_device_t *session, speed_t speed,
                  char parity UNUSED, int stopbits UNUSED)
{
    unsigned char msg[8] = {0};

    memset(msg, '\0', sizeof(msg));

    GPSD_LOG(LOG_WARN, &session->context->errout,
             "ALLY: baudrate %u\n", speed);

    if (0 == speed) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "ALLY: invalid baudrate %u\n", speed);
        return 0;
    }
    // poll CFG-PRT for UART0
    putbe16(msg, 0, CFG_PRT);
    msg[2] = 0x00;          // WAG: UART0
    putbe32(msg, 4, speed);
    (void)ally_write(session, ALLY_CFG, 0x01, msg, 8);
    return true;
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
            // restart init queue
            session->queue = 0;
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
    .speed_switcher = speed,                    // we can change baud rates
    .mode_switcher  = NULL,                     // there is a mode switcher
    .rate_switcher  = NULL,                     // change sample rate
    .min_cycle.tv_sec  = 1,                     // default
    .min_cycle.tv_nsec = 0,                     // default
    .control_send   = control_send,             // how to send a control string
    .time_offset    = NULL,                     // no NTP fudge factor
};
// *INDENT-ON*

// vim: set expandtab shiftwidth=4
