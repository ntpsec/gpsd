/*
 * UBX driver.  For u-blox binary, also includes Antaris4 binary
 * Reference manuals are at
 * http://www.u-blox.com/en/download/documents-a-resources/u-blox-6-gps-modules-resources.html
 *
 * updated for u-blox 8
 * http://www.ublox.com/images/downloads/Product_Docs/u-bloxM8_ReceiverDescriptionProtocolSpec_%28UBX-13003221%29_Public.pdf
 *
 * Week counters are not limited to 10 bits. It's unknown what
 * the firmware is doing to disambiguate them, if anything; it might just
 * be adding a fixed offset based on a hidden epoch value, in which case
 * unhappy things will occur on the next rollover.
 *
 * For the Antaris 4, the default leap-second offset (before getting one from
 * the sats, one presumes) is 0sec; for the u-blox 6 it's 15sec.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include "../include/compiler.h"   // for FALLTHROUGH
#include "../include/gpsd.h"
#if defined(UBLOX_ENABLE) && defined(BINARY_ENABLE)
#include "../include/driver_ubx.h"

#include "../include/bits.h"       // For UINT2INT()
#include "../include/timespec.h"

/*
 * Some high-precision messages provide data where the main part is a
 * signed 32-bit integer (same as the standard-precision versions),
 * and there's an 8-bit signed field providing an addend scaled to
 * 1/100th of the main value.  This macro provides a fetch for such
 * values, scaled to match the extension (i.e., 100X the main-value scale).
 * Since the fields are nonconsective, the offsets are provided separately.
 * The result is a signed 64-bit integer.
 *
 * The second macro incorporates scaling the result by a specified double.
 */
#define getles32x100s8(buf, off, offx) \
    ((int64_t)(getles32((buf), (off)) * 100LL + getsb((buf), (offx))))
#define getles32x100s8d(buf, off, offx, scale) \
    (getles32x100s8((buf), (off), (offx)) * (double)(scale))

/*
 * A ubx packet looks like this:
 * leader: 0xb5 0x62
 * message class: 1 byte
 * message type: 1 byte
 * length of payload: 2 bytes
 * payload: variable length
 * checksum: 2 bytes
 *
 * see also the FV25 and UBX documents on reference.html
 */
#define UBX_PREFIX_LEN          6
#define UBX_CLASS_OFFSET        2
#define UBX_TYPE_OFFSET         3

/* because we hates magic numbers forever */
#define USART1_ID               1
#define USART2_ID               2
#define USB_ID                  3
#define UBX_PROTOCOL_MASK       0x01
#define NMEA_PROTOCOL_MASK      0x02
#define RTCM_PROTOCOL_MASK      0x04
#define RTCM3_PROTOCOL_MASK     0x20    // protVer 20+
#define UBX_CFG_LEN             20
#define outProtoMask            14

static gps_mask_t ubx_parse(struct gps_device_t *session, unsigned char *buf,
                            size_t len);
static gps_mask_t ubx_msg_log_batch(struct gps_device_t *session,
                                    unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_log_info(struct gps_device_t *session,
                                   unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_log_retrievepos(struct gps_device_t *session,
                                          unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_log_retrieveposextra(struct gps_device_t *session,
                                               unsigned char *buf,
                                               size_t data_len);
static gps_mask_t ubx_msg_log_retrievestring(struct gps_device_t *session,
                                             unsigned char *buf,
                                             size_t data_len);
static gps_mask_t ubx_msg_nav_dop(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_nav_eoe(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_inf(struct gps_device_t *session, unsigned char *buf,
                              size_t data_len);
static gps_mask_t ubx_msg_nav_posecef(struct gps_device_t *session,
                                      unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_nav_pvt(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_mon_ver(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_nav_sat(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_nav_sol(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_nav_svinfo(struct gps_device_t *session,
                                     unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_nav_timegps(struct gps_device_t *session,
                                      unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_nav_velecef(struct gps_device_t *session,
                                      unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_nav_sbas(struct gps_device_t *session,
                                   unsigned char *buf, size_t data_len);
static gps_mask_t ubx_msg_tim_tp(struct gps_device_t *session,
                                 unsigned char *buf, size_t data_len);
static void ubx_mode(struct gps_device_t *session, int mode);

typedef struct {
    const char *fw_string;
    const float protver;
} fw_protver_map_entry_t;

/* based on u-blox document no. GPS.G7-SW-12001-B1 (15 June 2018) */
/* capture decimal parts of protVer info even when session->protver currently
 * is integer (which _might_ change in the future, so avoid having to revisit
 * the info at that time).
 * This list is substantially incomplete and over specific. */
static const fw_protver_map_entry_t fw_protver_map[] = {
    {"2.10", 8.10},           // antaris 4, version 8 is a guess
    {"2.11", 8.11},           // antaris 4, version 8 is a guess
    {"3.04", 9.00},           // antaris 4, version 9 is a guess
    {"4.00", 10.00},          // antaris 4, and u-blox 5
    {"4.01", 10.01},          // antaris 4, and u-blox 5
    {"5.00", 11.00},          // u-blox 5 and antaris 4
    {"6.00", 12.00},          // u-blox 5 and 6
    {"6.02", 12.02},          // u-blox 5 and 6
    {"7.01", 13.01},          // u-blox 7
    {"7.03", 13.03},          // u-blox 7
    {"1.00", 14.00},          // u-blox 6 w/ GLONASS, and 7
    // protVer >14 should carry explicit protVer in MON-VER extension
    {NULL, 0.0},
};

/*
 * Model  Fw          Protver
 * M10    SPG 5.00    34.00
 */

/* Convert a ubx PRN to an NMEA 4.0 (extended) PRN and ubx gnssid, svid
 *
 * return 0 on fail
 */
static short ubx_to_prn(int ubx_PRN, unsigned char *gnssId,
                        unsigned char *svId)
{
    *gnssId = 0;
    *svId = 0;

    // IRNSS??
    if (1 > ubx_PRN) {
        // skip 0 PRN
        return 0;
    } else if (32 >= ubx_PRN) {
        // GPS 1..32 -> 1..32
        *gnssId = 0;
        *svId = ubx_PRN;
    } else if (64 >= ubx_PRN) {
        // BeiDou, 159..163,33..64 -> 1..5,6..37
        *gnssId = 3;
        *svId = ubx_PRN - 27;
    } else if (96 >= ubx_PRN) {
        // GLONASS 65..96 -> 1..32
        *gnssId = 6;
        *svId = ubx_PRN - 64;
    } else if (120 > ubx_PRN) {
        // Huh?
        return 0;
    } else if (158 >= ubx_PRN) {
        // SBAS 120..158 -> 120..158
        *gnssId = 1;
        *svId = ubx_PRN;
    } else if (163 >= ubx_PRN) {
        // BeiDou, 159..163 -> 1..5
        *gnssId = 3;
        *svId = ubx_PRN - 158;
    } else if (173 > ubx_PRN) {
        // Huh?
        return 0;
    } else if (182 >= ubx_PRN) {
        // IMES 173..182 -> 1..5, in u-blox 8, bot u-blox 9
        *gnssId = 4;
        *svId = ubx_PRN - 172;
    } else if (193 > ubx_PRN) {
        // Huh?
        return 0;
    } else if (199 >= ubx_PRN) {
        // QZSS 193..197 -> 1..5
        // ZED-F9T also see 198 and 199
        *gnssId = 5;
        *svId = ubx_PRN - 192;
    } else if (211 > ubx_PRN) {
        // Huh?
        return 0;
    } else if (246 >= ubx_PRN) {
        // Galileo 211..246 -> 1..36
        *gnssId = 2;
        *svId = ubx_PRN - 210;
    } else {
        // greater than 246, GLONASS (255), unused, or other unknown
        return 0;
    }
    return ubx2_to_prn(*gnssId, *svId);
}

// UBX-CFG-RATE
// Deprecated in u-blox 10
static void ubx_msg_cfg_rate(struct gps_device_t *session, unsigned char *buf,
                             size_t data_len)
{
    uint16_t measRate, navRate, timeRef;

    if (6 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-CFG-RATE message, runt payload len %zd", data_len);
        return;
    }

    measRate = getleu16(buf, 0);  // Measurement rate (ms)
    navRate = getleu16(buf, 2);   // Navigation rate (cycles)
    timeRef = getleu16(buf, 4);   // Time system, e.g. UTC, GPS, ...

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-CFG-RATE: measRate %ums, navRate %u cycle(s), "
             "timeRef %u\n",
             (unsigned)measRate, (unsigned)navRate,
             (unsigned)timeRef);

    // Update our notion of what the device's measurement rate is
    MSTOTS(&session->gpsdata.dev.cycle, measRate);

    return;
}

/* UBX-ESF-ALG
 *
 * UBX-ESF-ALG, and UBX-ESF-INS are synchronous to the GNSS epoch.
 * They need to be combined and reported together with the rest of
 * the epoch.
 */
static gps_mask_t
ubx_msg_esf_alg(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned version, flags, error, reserved1;
    unsigned long yaw;
    int pitch, roll;
    static gps_mask_t mask = 0;

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-ESF-ALG message, runt payload len %zd", data_len);
        return mask;
    }

    // UBX-ESF-ALG is aligned with the GNSS epoch.
    session->driver.ubx.iTOW = getleu32(buf, 0);

    version = getub(buf, 4);
    flags = getub(buf, 5);
    error = getub(buf, 6);
    reserved1 = getub(buf, 7);
    yaw = getleu32(buf, 8);
    pitch = getles16(buf, 12);
    roll = getles16(buf, 14);

    if (0 == (2 & error)) {
        // no yawAlgError
        session->gpsdata.attitude.yaw = 0.01 * yaw;
        mask |= ATTITUDE_SET;
    }
    if (0 == (5 & error)) {
        // no tiltAlgError or angleError
        session->gpsdata.attitude.roll = 0.01 * roll;
        session->gpsdata.attitude.pitch = 0.01 * pitch;
        mask |= ATTITUDE_SET;
    }

    if (0 != mask) {
        timespec_t ts_tow;
        // got good data, set the measurement time
        MSTOTS(&ts_tow, session->driver.ubx.iTOW);
        session->gpsdata.attitude.mtime =
            gpsd_gpstime_resolv(session, session->context->gps_week, ts_tow);
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-ESF-ALG: iTOW %lld version %u flags x%x error x%x"
             " reserved1 x%x yaw %ld pitch %u roll %u\n",
            (long long)session->driver.ubx.iTOW, version, flags, error,
            reserved1, yaw, pitch, roll);

    return mask;
}

/* UBX-ESF-INS
 *
 * protVer 19 and up.  ADR and UDR only
 *
 * UBX-ESF-ALG, and UBX-ESF-INS are synchronous to the GNSS epoch.
 * They need to be combined and reported together with the rest of
 * the epoch.
 */
static gps_mask_t
ubx_msg_esf_ins(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned long long bitfield0, reserved1;
    long xAngRate, yAngRate, zAngRate;
    long xAccel, yAccel, zAccel;
    static gps_mask_t mask = 0;

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-ESF-INS message, runt payload len %zd", data_len);
        return mask;
    }

    bitfield0 = getleu32(buf, 0);
    reserved1 = getleu32(buf, 4);
    // UBX-ESF-INS is aligned with the GNSS epoch.
    session->driver.ubx.iTOW = getleu32(buf, 8);
    xAngRate = getles32(buf, 12);
    yAngRate = getles32(buf, 16);
    zAngRate = getles32(buf, 20);
    xAccel = getles32(buf, 24);
    yAccel = getles32(buf, 28);
    zAccel = getles32(buf, 32);

    if (0x100 == (0x100 & bitfield0)) {
        // xAngRateValid
        session->gpsdata.attitude.gyro_x = 0.001 * xAngRate;  // deg/s
        mask |= ATTITUDE_SET;
    }
    if (0x200 == (0x200 & bitfield0)) {
        // yAngRateValid
        session->gpsdata.attitude.gyro_x = 0.001 * yAngRate;  // deg/s
        mask |= ATTITUDE_SET;
    }
    if (0x400 == (0x400 & bitfield0)) {
        // zAngRateValid
        session->gpsdata.attitude.gyro_x = 0.001 * zAngRate;  // deg/s
        mask |= ATTITUDE_SET;
    }
    if (0x800 == (0x800 & bitfield0)) {
        // xAccelValid
        session->gpsdata.attitude.acc_x = 0.01 * xAccel;  // m/s^2
        mask |= ATTITUDE_SET;
    }
    if (0x1000 == (0x1000 & bitfield0)) {
        // yAccelValid
        session->gpsdata.attitude.acc_y = 0.01 * yAccel;  // m/s^2
        mask |= ATTITUDE_SET;
    }
    if (0x2000 == (0x2000 & bitfield0)) {
        // zAccelValid
        session->gpsdata.attitude.acc_z = 0.01 * zAccel;  // m/s^2
        mask |= ATTITUDE_SET;
    }

    if (0 != mask) {
        timespec_t ts_tow;
        // got good data, set the measurement time
        MSTOTS(&ts_tow, session->driver.ubx.iTOW);
        session->gpsdata.attitude.mtime =
            gpsd_gpstime_resolv(session, session->context->gps_week, ts_tow);
    }
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-ESF-INS: bitfield0 %llu, reserved1 %llu iTOW %lld"
             " xAngRate %ld yAngRate %ld zAngRate %ld"
             " xAccel %ld yAccel %ld zAccel %ld\n",
            bitfield0, reserved1,
            (long long)session->driver.ubx.iTOW,
            xAngRate, yAngRate, zAngRate, xAccel, yAccel, zAccel);

    return mask;
}

/* UBX-ESF-MEAS
 *
 * protVer 15 and up.  ADR only
 * protVer 19 and up.  ADR and UDR only
 *
 * asynchronous to the GNSS epoch, and at a higher rate.
 * Needs to be reported immediately.
 *
 */
static gps_mask_t
ubx_msg_esf_meas(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    unsigned flags, id, numMeas, expected_len;
    gps_mask_t mask = 0;
    unsigned i;
    // where to store the IMU data.
    struct attitude_t *datap = &session->gpsdata.imu[0];

    if (8 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-ESF-MEAS message, runt payload len %zd", data_len);
        return mask;
    }
    // do not acumulate IMU data
    gps_clear_att(datap);
    (void)strlcpy(datap->msg, "UBX-ESF-MEAS", sizeof(datap->msg));

    datap->timeTag = getleu32(buf, 0);
    flags = getleu16(buf, 4);
    numMeas = (flags >> 11) & 0x01f;
    id = getleu16(buf, 6);
    expected_len = 8 + (4 * numMeas);
    if (0x08 & flags) {
        expected_len += 4;
    }
    if (expected_len != data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-ESF-MEAS: bad length.  Got %zd, expected %u",
                 data_len, expected_len);
        return 0;
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-ESF-MEAS: timeTag %lu flags x%x (numMeas %u) id %u\n",
             datap->timeTag, flags, numMeas, id);

    for (i = 0; i < numMeas; i++) {
        unsigned long data, dataField;
        long dataF;
        unsigned char dataType;

        data = getleu32(buf, 8 + (i * 4));
        dataType = (unsigned char)(data >> 24) & 0x3f;
        dataField = data & BITMASK(24);
        switch (dataType) {
        case 5:            // gyro z angular rate, deg/s^2
            dataF = UINT2INT(dataField, 24);
            datap->gyro_z = dataF / 4096.0;
            mask |= IMU_SET;
            break;
        case 12:           // gyro temp, deg C
            dataF = UINT2INT(dataField, 24);
            datap->gyro_temp = dataF / 100.0;
            mask |= IMU_SET;
            break;
        case 13:           // gyro y angular rate, deg/s^2
            dataF = UINT2INT(dataField, 24);
            datap->gyro_y = dataF / 4096.0;
            mask |= IMU_SET;
            break;
        case 14:           // gyro x angular rate, deg/s^2
            dataF = UINT2INT(dataField, 24);
            datap->gyro_x = dataF / 4096.0;
            mask |= IMU_SET;
            break;
        case 16:            // accel x, m/s^2
            dataF = UINT2INT(dataField, 24);
            datap->acc_x = dataF / 1024.0;
            mask |= IMU_SET;
            break;
        case 17:           // accel y, m/s^2
            dataF = UINT2INT(dataField, 24);
            datap->acc_y = dataF / 1024.0;
            mask |= IMU_SET;
            break;
        case 18:           // accel z, m/s^2
            dataF = UINT2INT(dataField, 24);
            datap->acc_z = dataF / 1024.0;
            mask |= IMU_SET;
            break;
        // case 6:            // front-left wheel ticks
        // case 7:            // front-right wheel ticks
        // case 8:            // rear-left wheel ticks
        // case 9:            // rear-right wheel ticks
        // case 10:           // speed tick
        // case 11:           // speed, m/s
        default:
            // ignore all else
            dataF = dataField;
            break;
        }

        GPSD_LOG(LOG_PROG + 1, &session->context->errout,
                 "UBX-ESF-MEAS: dataType %2u dataField %9ld\n",
                 dataType, dataF);
    }

    return mask;
}

/* UBX-ESF-RAW
 *
 * protVer 15 and up.  ADR only
 * protVer 19 and up.  ADR and UDR only
 *
 * asynchronous to the GNSS epoch, and a a higher rate.
 * Needs to be reported immediately.
 *
 */
static gps_mask_t
ubx_msg_esf_raw(struct gps_device_t *session, unsigned char *buf,
                             size_t data_len)
{
    unsigned long reserved1, last_sTtag = 0;
    unsigned i;
    uint16_t blocks;
    gps_mask_t mask = 0;
    struct attitude_t *datap = NULL;
    int max_imu, cur_imu = -1;
    max_imu = sizeof(session->gpsdata.imu) / sizeof(struct attitude_t);

    if (4 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-ESF-RAW message, runt payload len %zd", data_len);
        return mask;
    }

    reserved1 = getleu32(buf, 0);  // reserved1
    if (0 != ((data_len - 4) % 8)) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-ESF-RAW message, weird payload len %zd", data_len);
        return mask;
    }
    blocks = (data_len - 4) / 8;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-ESF-RAW: reserved1 x%lx, blocks %u\n",
             reserved1, blocks);

    // loop over all blocks, use the next imu[] when time changes.
    for (i = 0; i < blocks; i++) {
        unsigned long data, dataField, sTtag;
        long dataF;
        unsigned char dataType;

        sTtag = getleu32(buf, 8 + (i * 8));
        if ((-1 == cur_imu) ||
            (last_sTtag != sTtag)) {
            cur_imu++;
            if (max_imu <= cur_imu) {
                GPSD_LOG(LOG_WARN, &session->context->errout,
                         "UBX-ESF-RAW message, too many imu max %d block %u\n",
                         max_imu, i);
                break;
            }
            last_sTtag = sTtag;
            datap = &session->gpsdata.imu[cur_imu];
            // do not acumulate IMU data
            gps_clear_att(datap);
            (void)strlcpy(datap->msg, "UBX-ESF-RAW", sizeof(datap->msg));
        }
        if (NULL == datap) {
            // paranoia
            continue;
        }

        data = getleu32(buf, 4 + (i * 8));
        dataType = (unsigned char)(data >> 24) & 0x3f;
        dataField = data & BITMASK(24);
        datap->timeTag = sTtag;
        switch (dataType) {
        case 5:            // gyro z angular rate, deg/s^2
            dataF = UINT2INT(dataField, 24);
            datap->gyro_z = dataF / 4096.0;
            mask |= IMU_SET;
            break;
        case 12:           // gyro temp, deg C
            dataF = UINT2INT(dataField, 24);
            datap->gyro_temp = dataF / 100.0;
            mask |= IMU_SET;
            break;
        case 13:           // gyro y angular rate, deg/s^2
            dataF = UINT2INT(dataField, 24);
            datap->gyro_y = dataF / 4096.0;
            mask |= IMU_SET;
            break;
        case 14:           // gyro x angular rate, deg/s^2
            dataF = UINT2INT(dataField, 24);
            datap->gyro_x = dataF / 4096.0;
            mask |= IMU_SET;
            break;
        case 16:            // accel x, m/s^2
            dataF = UINT2INT(dataField, 24);
            datap->acc_x = dataF / 1024.0;
            mask |= IMU_SET;
            break;
        case 17:           // accel y, m/s^2
            dataF = UINT2INT(dataField, 24);
            datap->acc_y = dataF / 1024.0;
            mask |= IMU_SET;
            break;
        case 18:           // accel z, m/s^2
            dataF = UINT2INT(dataField, 24);
            datap->acc_z = dataF / 1024.0;
            mask |= IMU_SET;
            break;
        // case 6:            // front-left wheel ticks
        // case 7:            // front-right wheel ticks
        // case 8:            // rear-left wheel ticks
        // case 9:            // rear-right wheel ticks
        // case 10:           // speed tick
        // case 11:           // speed, m/s
        default:
            // ignore all else
            dataF = dataField;
            break;
        }

        GPSD_LOG(LOG_PROG + 1, &session->context->errout,
                 "UBX-ESF-RAW: dataType %2u dataField %9ld sTtag %lu\n",
                 dataType, dataF, datap->timeTag);
    }
    return mask;
}

// UBX-ESF-STATUS
static gps_mask_t
ubx_msg_esf_status(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    unsigned version, fusionMode, numSens, expected_len;
    static gps_mask_t mask = 0;

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-ESF-STATUS message, runt payload len %zd", data_len);
        return mask;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    version = getub(buf, 4);
    fusionMode = getub(buf, 12);
    numSens = getub(buf, 15);
    expected_len = 16 + (4 * numSens);

    if (expected_len != data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-ESF-STATUS: bad length.  Expected %u got %zd",
                 expected_len, data_len);
        return mask;
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-ESF-STATUS: iTOW %lld version %u fusionMode %u numSens %u\n",
            (long long)session->driver.ubx.iTOW, version, fusionMode, numSens);

    return mask;
}

/**
 * HNR Attitude solution
 * UBX-HNR-ATT Class x28, ID 1
 *
 * Not before u-blox 8, protVer 19.2 and up.
 * only on ADR, and UDR
 */
static gps_mask_t
ubx_msg_hnr_att(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    uint8_t version;;
    int64_t iTOW;
    timespec_t ts_tow;
    gps_mask_t mask = 0;

    if (32 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-HNR-ATT message, runt payload len %zd", data_len);
        return 0;
    }

    if (19 > session->driver.ubx.protver) {
        // this GPS is at least protver 19.2
        session->driver.ubx.protver = 19;
    }
    // don't set session->driver.ubx.iTOW, HNR is off-cycle
    iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, iTOW);
    session->gpsdata.attitude.mtime =
        gpsd_gpstime_resolv(session, session->context->gps_week, ts_tow);

    version  = (unsigned int)getub(buf, 4);

    session->gpsdata.attitude.roll = 1e-5 * getles32(buf, 8);
    session->gpsdata.attitude.pitch = 1e-5 * getles32(buf, 12);
    // seems to be true heading
    session->gpsdata.attitude.heading = 1e-5 * getles32(buf, 16);
    mask |= ATTITUDE_SET;

    GPSD_LOG(LOG_PROG, &session->context->errout,
         "HNR-ATT: iTOW %lld version %u roll %.5f pitch %.5f heading %.5f\n",
         (long long)iTOW,
         version,
         session->gpsdata.attitude.roll,
         session->gpsdata.attitude.pitch,
         session->gpsdata.attitude.heading);

    return mask;
}

/**
 * HNR Vehicle dynamics information
 * UBX-HNR-INS Class x28, ID 2
 *
 * Not before u-blox 8, protVer 19.1 and up.
 * only on ADR, and UDR
 */
static gps_mask_t
ubx_msg_hnr_ins(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    uint8_t version;;
    uint32_t bitfield0;
    gps_mask_t mask = 0;
    int64_t iTOW;

    if (36 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-HNR-INS message, runt payload len %zd", data_len);
        return 0;
    }

    if (19 > session->driver.ubx.protver) {
        // this GPS is at least protver 19.1
        session->driver.ubx.protver = 19;
    }
    version  = (unsigned int)getub(buf, 0);

    bitfield0 = getleu32(buf, 0);
    // don't set session->driver.ubx.iTOW, HNR is off-cycle
    iTOW = getleu32(buf, 8);

    if (0x100 == (0x100 & bitfield0)) {
        // xAngRateValid
        session->gpsdata.attitude.gyro_x = 0.001 * getles32(buf, 12);  // deg/s
        mask |= ATTITUDE_SET;
    }
    if (0x200 == (0x200 & bitfield0)) {
        // yAngRateValid
        session->gpsdata.attitude.gyro_x = 0.001 * getles32(buf, 16);  // deg/s
        mask |= ATTITUDE_SET;
    }
    if (0x400 == (0x400 & bitfield0)) {
        // zAngRateValid
        session->gpsdata.attitude.gyro_x = 0.001 * getles32(buf, 20);  // deg/s
        mask |= ATTITUDE_SET;
    }
    if (0x800 == (0x800 & bitfield0)) {
        // xAccelValid
        session->gpsdata.attitude.acc_x = 0.01 * getles32(buf, 24);  // m/s^2
        mask |= ATTITUDE_SET;
    }
    if (0x1000 == (0x1000 & bitfield0)) {
        // yAccelValid
        session->gpsdata.attitude.acc_y = 0.01 * getles32(buf, 28);  // m/s^2
        mask |= ATTITUDE_SET;
    }
    if (0x2000 == (0x2000 & bitfield0)) {
        // zAccelValid
        session->gpsdata.attitude.acc_z = 0.01 * getles32(buf, 32);  // m/s^2
        mask |= ATTITUDE_SET;
    }

    if (0 != mask) {
        timespec_t ts_tow;
        // got good data, set the measurement time
        MSTOTS(&ts_tow, iTOW);
        session->gpsdata.attitude.mtime =
            gpsd_gpstime_resolv(session, session->context->gps_week, ts_tow);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
         "HNR-INS: iTOW %lld version %u bitfield0 x%x "
         "gyro_x %.3f gyro_y %.3f gyro_z %.3f "
         "acc_x %.3f acc_y %.3f acc_z %.3f\n",
         (long long)iTOW,
         version, bitfield0,
         session->gpsdata.attitude.gyro_x,
         session->gpsdata.attitude.gyro_y,
         session->gpsdata.attitude.gyro_z,
         session->gpsdata.attitude.acc_x,
         session->gpsdata.attitude.acc_y,
         session->gpsdata.attitude.acc_z);

    return mask;
}

/**
 * High rate output of PVT solution
 * UBX-HNR-PVT Class x28, ID 2
 *
 * Not before u-blox 8, protVer 19 and up.
 * only on ADR, and UDR
 */
static gps_mask_t
ubx_msg_hnr_pvt(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    char ts_buf[TIMESPEC_LEN];
    gps_mask_t mask = 0;
    int64_t iTOW;
    int *mode = &session->newdata.mode;
    int *status = &session->newdata.status;
    struct tm unpacked_date;
    uint8_t flags;
    uint8_t gpsFix;
    uint8_t valid;

    if (72 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-HNR-PVT message, runt payload len %zd", data_len);
        return 0;
    }

    if (19 > session->driver.ubx.protver) {
        // this GPS is at least protver 19
        session->driver.ubx.protver = 19;
    }
    // don't set session->driver.ubx.iTOW, HNR is off-cycle
    iTOW = getleu32(buf, 0);
    // valid same as UBX-NAV-PVT valid
    valid = (unsigned int)getub(buf, 11);
    // gpsFix same as UBX-NAV-PVT fixType
    gpsFix = (unsigned char)getub(buf, 16);
    // flags NOT same as UBX-NAV-PVT flags
    flags = (unsigned int)getub(buf, 17);

    switch (gpsFix) {
    case UBX_MODE_TMONLY:
        // 5 - Surveyed-in, so a precise 3D.
        *mode = MODE_3D;
        *status = STATUS_TIME;
        mask |= STATUS_SET | MODE_SET;
        break;

    case UBX_MODE_3D:
        // 3
        FALLTHROUGH
    case UBX_MODE_GPSDR:
        // 4
        if (*mode != MODE_3D) {
            *mode = MODE_3D;
            mask |= MODE_SET;
        }
        if (UBX_NAV_PVT_FLAG_DGPS == (flags & UBX_NAV_PVT_FLAG_DGPS)) {
            *status = STATUS_DGPS;
            mask |= STATUS_SET;
        } else {
            *status = STATUS_GPS;
            mask |= STATUS_SET;
        }
        mask |=   LATLON_SET;
        break;

    case UBX_MODE_2D:
        // 2
        FALLTHROUGH
    case UBX_MODE_DR:           // consider this too as 2D
        // 1
        if (MODE_2D != *mode) {
            *mode = MODE_2D;
            mask |= MODE_SET;
        };
        if (STATUS_GPS != *status) {
            // FIXME: Set DR status if it is DR
            *status = STATUS_GPS;
            mask |= STATUS_SET;
        }
        mask |= LATLON_SET | SPEED_SET;
        break;

    case UBX_MODE_NOFIX:
        // 0
        FALLTHROUGH
    default:
        // huh?
        if (*mode != MODE_NO_FIX) {
            *mode = MODE_NO_FIX;
            mask |= MODE_SET;
        };
        if (*status != STATUS_UNK) {
            *status = STATUS_UNK;
            mask |= STATUS_SET;
        }
        break;
    }

    if (UBX_NAV_PVT_VALID_DATE_TIME == (valid & UBX_NAV_PVT_VALID_DATE_TIME)) {
        unpacked_date.tm_year = (uint16_t)getleu16(buf, 4) - 1900;
        unpacked_date.tm_mon = (uint8_t)getub(buf, 6) - 1;
        unpacked_date.tm_mday = (uint8_t)getub(buf, 7);
        unpacked_date.tm_hour = (uint8_t)getub(buf, 8);
        unpacked_date.tm_min = (uint8_t)getub(buf, 9);
        unpacked_date.tm_sec = (uint8_t)getub(buf, 10);
        unpacked_date.tm_isdst = 0;
        unpacked_date.tm_wday = 0;
        unpacked_date.tm_yday = 0;
        session->newdata.time.tv_sec = mkgmtime(&unpacked_date);
        // field 9, nano, can be negative! So normalize
        session->newdata.time.tv_nsec = getles32(buf, 12);
        TS_NORM(&session->newdata.time);
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }

    session->newdata.longitude = 1e-7 * getles32(buf, 20);
    session->newdata.latitude = 1e-7 * getles32(buf, 24);
    // altitude WGS84
    session->newdata.altHAE = (double)1e-3 * getles32(buf, 28);
    // altitude MSL, double to prevent promotion to (long double)
    session->newdata.altMSL = (double)1e-3 * getles32(buf, 32);
    // Let gpsd_error_model() deal with geoid_sep

    // gSpeed (2D)
    session->newdata.speed = 1e-3 * (int32_t)getles32(buf, 36);
    // offset 40,  Speed (3D) do what with it?
    // u-blox calls this headMot (Heading of motion 2-D)
    session->newdata.track = 1e-5 * (int32_t)getles32(buf, 44);
    // offset 48, headVeh (Heading of Vehicle 2-D)
    mask |= LATLON_SET | ALTITUDE_SET | SPEED_SET | TRACK_SET;

    /* u-blox does not document the basis for the following "accuracy"
     * estimates.  Maybe CEP(50), one sigma, two sigma, CEP(99), etc. */

    // Horizontal Accuracy estimate, in mm
    session->newdata.eph = (double)(getles32(buf, 52) / 1000.0);
    // Vertical Accuracy estimate, in mm
    session->newdata.epv = (double)(getles32(buf, 56) / 1000.0);
    // Speed Accuracy estimate, in mm/s
    session->newdata.eps = (double)(getles32(buf, 60) / 1000.0);
    // headAcc (Heading Accuracy)
    session->newdata.epd = (double)getles32(buf, 64) * 1e-5;
    /* let gpsd_error_model() do the rest */

    // 4 final bytes reserved

    mask |= HERR_SET | SPEEDERR_SET | VERR_SET;
    // HNR-PVT interleaves with the normal cycle, so cycle end is a mess
    mask |= REPORT_IS;

    GPSD_LOG(LOG_PROG, &session->context->errout,
         "HNR-PVT: iTOW %lld flags %02x time %s lat %.2f lon %.2f altHAE %.2f "
         "track %.2f speed %.2f climb %.2f mode %d status %d used %d\n",
         (long long)iTOW, flags,
         timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)),
         session->newdata.latitude,
         session->newdata.longitude,
         session->newdata.altHAE,
         session->newdata.track,
         session->newdata.speed,
         session->newdata.climb,
         session->newdata.mode,
         session->newdata.status,
         session->gpsdata.satellites_used);

    return mask;
}

/**
 * Receiver/Software Version
 * UBX-MON-VER
 *
 * sadly more info than fits in session->swtype for now.
 * so squish the data hard.
 */
static gps_mask_t ubx_msg_mon_ver(struct gps_device_t *session,
                                  unsigned char *buf,
                                  size_t data_len)
{
    int n = 0;                           // extended info counter
    int num_ext = (data_len - 40) / 30;  // number of extensions
    char obuf[128];                      // temp version string buffer
    char *cptr;

    if (40 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-MON-VER message, runt payload len %zd", data_len);
        return 0;
    }

    // save SW and HW Version as subtype
    (void)snprintf(obuf, sizeof(obuf),
                   "SW %.30s,HW %.10s",
                   (char *)buf,
                   (char *)(buf + 30));

    // save what we can
    (void)strlcpy(session->subtype, obuf, sizeof(session->subtype));

    obuf[0] = '\0';
    // extract Extended info strings.
    for (n = 0; n < num_ext; n++) {
        int start_of_str = 40 + (30 * n);

        if (0 < n) {
            // commas between elements
            (void)strlcat(obuf, ",", sizeof(obuf));
        }
        (void)strlcat(obuf, (char *)&buf[start_of_str], sizeof(obuf));
    }

    // save what we can in subtype1
    (void)strlcpy(session->subtype1, obuf, sizeof(session->subtype1));

    // find PROTVER literal, followed by single separator character
    cptr = strstr(obuf, "PROTVER=");     // protVer 18 and above
    if (NULL == cptr) {
        cptr = strstr(obuf, "PROTVER "); // protVer 17 and below
    }
    if (NULL != cptr) {
        int protver = atoi(cptr + 8);
        if (7 < protver) {
            /* protver 8, u-blox Antaris, is the oldest we know, but never
             * used explicitly.  protver 15, u-blox 8, is oldest seen. */
            session->driver.ubx.protver = protver;
        }
    }

    /* MON-VER did not contain PROTVER in any extension field (typical for
     * protVer < 15), so use mapping table to try to derive protVer from
     * firmware revision number carried in swVersion field */
    if (0 == session->driver.ubx.protver) {
        for (n = 0; NULL != fw_protver_map[n].fw_string; n++) {
            // skip "SW " prefix in session->subtype
            cptr = strstr(session->subtype + 3, fw_protver_map[n].fw_string);
            // use only when swVersion field starts with fw_string
            if (cptr == (session->subtype + 3)) {
                session->driver.ubx.protver =
                    (unsigned char)fw_protver_map[n].protver;
                break;
            }
        }
        if (0 == session->driver.ubx.protver) {
            // Still not found, old chip.  Set to one so we know we tried.
            session->driver.ubx.protver = 1;
        }
    }

    // output SW and HW Version at LOG_INF
    GPSD_LOG(LOG_INF, &session->context->errout,
             "UBX-MON-VER: %s %s PROTVER %u\n",
             session->subtype, session->subtype1,
             session->driver.ubx.protver);
    return 0;
}

/* UBX-MON-TXBUF
 * Present in u-blox 5+ through at least protVer 23.01
 * Supported but deprecated in M9P protVer 27.11
 * Supported but deprecated in M9N protVer 32.00 */
static gps_mask_t
ubx_msg_mon_txbuf(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned int tUsage, tPeakusage;
    unsigned char errors, limit, reserved1;
    int i;

    if (28 != data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-MON-TXBUF message, runt payload len %zd\n", data_len);
        return 0;
    }

    errors = limit = getub(buf, 26);

    for (i = 0; i < 6; i++) {
        unsigned int pending = getleu16(buf, i * 2);
        unsigned int usage =  getub(buf, 12 + i);
        unsigned int peakUsage = getub(buf, 18 + i);

        GPSD_LOG(LOG_INF, &session->context->errout,
                 "TXBUF: target %d, limit %u, pending %4u bytes, "
                 "usage %3u%%, peakUsage %3d%%\n",
                 i, limit & 1, pending, usage, peakUsage);
        limit = limit >> 1;
    }
    tUsage = getub(buf, 24);
    tPeakusage = getub(buf, 25);
    reserved1 = getub(buf, 27);

    GPSD_LOG(LOG_INF, &session->context->errout,
             "TXBUF: tUsage %3u%%, tPeakusage %3u%%, errors 0x%02x, "
             "reserved1 0x%02x\n",
             tUsage, tPeakusage, errors, reserved1);

    if ((errors & 0x40) == 0x40 || (errors & 0x80) == 0x80) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
             "TXBUF: alloc %u, mem %u\n",
             errors >> 7, (errors >> 6) & 1);
    }
    return 0;
}

/* UBX-MON-RXBUF
 * Present in u-blox 5+ through at least protVer 23.01
 * Supported but deprecated in M9P protVer 27.11
 * Supported but deprecated in M9N protVer 32.00 */
static gps_mask_t
ubx_msg_mon_rxbuf(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    int i;

    if (24 != data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-MON-RXBUF message, runt payload len %zd\n", data_len);
        return 0;
    }

    for (i = 0; i < 6; i++) {
        unsigned int pending = getleu16(buf, i * 2);
        unsigned int usage =  getub(buf, 12 + i);
        unsigned int peakUsage = getub(buf, 18 + i);

        GPSD_LOG(LOG_INF, &session->context->errout,
                 "RXBUF: target %d, pending %4u bytes, "
                 "usage %3u%%, peakUsage %3d%%\n",
                 i, pending, usage, peakUsage);
    }
    return 0;
}

/**
 * UBX-LOG-BATCH entry only part of UBX protocol
 * Used for GPS standalone operation (internal batch retrieval)
 */
static gps_mask_t
ubx_msg_log_batch(struct gps_device_t *session, unsigned char *buf UNUSED,
                  size_t data_len)
{
    struct tm unpacked_date = {0};
    unsigned char contentValid, timeValid, flags, psmState;
    bool gnssFixOK, diffSoln;
    char ts_buf[TIMESPEC_LEN];
    gps_mask_t mask = 0;

    gps_clear_log(&session->gpsdata.log);
    /* u-blox 8 100 bytes payload */
    if (100 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-LOG-BATCH: runt len %zd", data_len);
        return 0;
    }
    timeValid = getub(buf, 15);
    if (3 != (timeValid & 3)) {
        // No time, pointless...
        return 0;
    }

    unpacked_date.tm_year = getleu16(buf, 8) - 1900;
    unpacked_date.tm_mon = getub(buf, 10) - 1;
    unpacked_date.tm_mday = getub(buf, 11);
    unpacked_date.tm_hour = getub(buf, 12);
    unpacked_date.tm_min = getub(buf, 13);
    unpacked_date.tm_sec = getub(buf, 14);

    contentValid = getub(buf, 1);
    session->gpsdata.log.index_cnt = getleu16(buf, 2);

    session->gpsdata.log.then.tv_sec = mkgmtime(&unpacked_date);
    session->gpsdata.log.then.tv_nsec = getles32(buf, 20);
    TS_NORM(&session->gpsdata.log.then);

    session->gpsdata.log.fixType = getub(buf, 24);
    flags = getub(   buf, 25);
    gnssFixOK = flags & 1;
    diffSoln = flags & 2;
    psmState = ((flags >> 2) & 7);

    // flags2 undocumented
    // flags2 = getub(   buf, 26);

    if ((gnssFixOK &&
         2 <= session->gpsdata.log.fixType)) {
        // good 2D fix
        session->gpsdata.log.lon = 1.0e-7 * getles32(buf, 28);
        session->gpsdata.log.lat = 1.0e-7 * getles32(buf, 32);
        session->gpsdata.log.gSpeed = 1.0e-3 * getles32(buf, 64);
        // seems to be true heading
        session->gpsdata.log.heading = 1.0e-5 * getles32(buf, 68);
        if (diffSoln) {
            session->gpsdata.log.status = STATUS_DGPS;
        } else {
            session->gpsdata.log.status = STATUS_GPS;
        }
        if (3 <= session->gpsdata.log.fixType) {
            // good 3D fix
            session->gpsdata.log.altHAE = 1.0e-3 * getles32(buf, 36);
        }
    }
    session->gpsdata.log.hAcc = 1.0e-3 * getleu32(buf, 44);

    GPSD_LOG(LOG_INF, &session->context->errout,
            "UBX-LOG-BATCH: time=%s index_cnt=%u fixType=%u lon=%.7f lat=%.7f"
             " gSpeed=%.3f heading=%.5f altHae=%.3f psmState=%u hAcc=%.3f\n",
             timespec_str(&session->gpsdata.log.then, ts_buf, sizeof(ts_buf)),
             session->gpsdata.log.index_cnt, session->gpsdata.log.fixType,
             session->gpsdata.log.lon, session->gpsdata.log.lat,
             session->gpsdata.log.gSpeed, session->gpsdata.log.heading,
             session->gpsdata.log.altHAE, psmState,
             session->gpsdata.log.hAcc);


    if (1 == (contentValid & 1)) {
        // extraPVT valid
        //  iTOW = getleu32(buf, 4);
        session->gpsdata.log.tAcc = (double)getleu32(buf, 16);
        session->gpsdata.log.numSV = getub(buf, 27);
        session->gpsdata.log.altMSL = 1.0e-3 * getles32(buf, 40);
        session->gpsdata.log.vAcc = 1.0e-3 * getleu32(buf, 48);
        session->gpsdata.log.velN = 1.0e-3 * getles32(buf, 52);
        session->gpsdata.log.velE = 1.0e-3 * getles32(buf, 56);
        session->gpsdata.log.velD = 1.0e-3 * getles32(buf, 60);
        session->gpsdata.log.sAcc = 1.0e-3 * getleu32(buf, 72);
        session->gpsdata.log.headAcc = 1.0e-5 * getleu32(buf, 76);
        session->gpsdata.log.pDOP = 1.0e-2 * getleu32(buf, 80);
        GPSD_LOG(LOG_INF, &session->context->errout,
                "UBX-LOG-BATCH extraPVT: time=%s index_cnt=%d"
                 " tAcc=%.2f numSV=%d altMSL=%.3f hAcc=%.2f vAcc=%.3f"
                 " velN=%.3f velE=%.3f velD=%.3f sAcc=%.3f headAcc=%.5f"
                 " pDOP=%.5f\n",
                 timespec_str(&session->gpsdata.log.then, ts_buf,
                              sizeof(ts_buf)),
                 session->gpsdata.log.index_cnt,
                 session->gpsdata.log.tAcc, session->gpsdata.log.numSV,
                 session->gpsdata.log.altMSL, session->gpsdata.log.hAcc,
                 session->gpsdata.log.vAcc, session->gpsdata.log.velN,
                 session->gpsdata.log.velE, session->gpsdata.log.velD,
                 session->gpsdata.log.sAcc, session->gpsdata.log.headAcc,
                 session->gpsdata.log.pDOP);
    }

    if (2 == (contentValid & 2)) {
        session->gpsdata.log.distance = getleu32(buf, 84);
        session->gpsdata.log.totalDistance = getleu32(buf, 88);
        session->gpsdata.log.distanceStd = getleu32(buf, 92);
        GPSD_LOG(LOG_INF, &session->context->errout,
                 "UBX-LOG-BATCH extraOdo: time=%s index_cnt=%d distance=%.0f"
                 " totalDistance=%.0f distanceStd=%.0f\n",
                 timespec_str(&session->gpsdata.log.then, ts_buf,
                              sizeof(ts_buf)),
                 session->gpsdata.log.index_cnt, session->gpsdata.log.distance,
                 session->gpsdata.log.totalDistance,
                 session->gpsdata.log.distanceStd);
    }

    mask |= LOG_SET;
    return mask;
}

/**
 * UBX-LOG-INFO info of log status
 * u-blox 7,8,9.  protVer 14 to 29
 * WIP: Initial decode, log only.
 *
 */
static gps_mask_t
ubx_msg_log_info(struct gps_device_t *session, unsigned char *buf UNUSED,
                 size_t data_len)
{
    struct tm oldest_date = {0}, newest_date = {0};
    timespec_t oldest = {0, 0};
    timespec_t newest = {0, 0};
    unsigned char version, status;
    unsigned long filestoreCapacity;
    unsigned long currentMaxLogSize;
    unsigned long currentLogSize;
    unsigned long entryCount;
    char ts_buf[TIMESPEC_LEN];
    char ts_buf1[TIMESPEC_LEN];
    gps_mask_t mask = 0;

    gps_clear_log(&session->gpsdata.log);
    // u-blox 7/8/9 48 bytes payload
    if (48 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-LOG-INFO: runt len %zd", data_len);
        return 0;
    }
    // u-blox 7/8/9 version 1
    version = getub(buf, 0);
    filestoreCapacity = getleu32(buf, 4);
    currentMaxLogSize = getleu32(buf, 16);
    currentLogSize = getleu32(buf, 20);
    entryCount = getleu32(buf, 24);
    status = getub(buf, 44);

    oldest_date.tm_year = getleu16(buf, 28);
    if (0 != oldest_date.tm_year) {
        oldest_date.tm_year -= 1900;
        oldest_date.tm_mon = getub(buf, 30) - 1;
        oldest_date.tm_mday = getub(buf, 31);
        oldest_date.tm_hour = getub(buf, 32);
        oldest_date.tm_min = getub(buf, 33);
        oldest_date.tm_sec = getub(buf, 34);
        oldest.tv_sec = mkgmtime(&oldest_date);
        oldest.tv_nsec = 0;
        TS_NORM(&oldest);
    }

    newest_date.tm_year = getleu16(buf, 36);
    if (0 != newest_date.tm_year) {
        newest_date.tm_year -= 1900;
        newest_date.tm_mon = getub(buf, 38) - 1;
        newest_date.tm_mday = getub(buf, 39);
        newest_date.tm_hour = getub(buf, 40);
        newest_date.tm_min = getub(buf, 41);
        newest_date.tm_sec = getub(buf, 42);
        newest.tv_sec = mkgmtime(&newest_date);
        newest.tv_nsec = 0;
        TS_NORM(&newest);
    }

    GPSD_LOG(LOG_INF, &session->context->errout,
             "UBX-LOG-INFO: version=%u status=x%x Cap=%lu MaxSize=%lu "
             "Size=%lu cnt=%lu oldest=%s newest=%s\n",
             version, status,
             filestoreCapacity,
             currentMaxLogSize,
             currentLogSize,
             entryCount,
             timespec_str(&oldest, ts_buf, sizeof(ts_buf)),
             timespec_str(&newest, ts_buf1, sizeof(ts_buf1)));

    // mask |= LOG_SET;
    return mask;
}

/*
 * UBX-LOG-RETRIEVEPOS (Indexed PVT entry)
 * Used for GPS standalone operation and host saved logs
 * u-blox 7,8,9.  protVer 14 to 29
 */
static gps_mask_t
ubx_msg_log_retrievepos(struct gps_device_t *session, unsigned char *buf UNUSED,
                        size_t data_len)
{
    struct tm unpacked_date = {0};
    unsigned char fixType;
    gps_mask_t mask = 0;

    gps_clear_log(&session->gpsdata.log);
    /* u-blox 40 bytes payload */
    if (40 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-LOG-RETRIEVEPOS: runt len %zd", data_len);
        return 0;
    }
    unpacked_date.tm_year = getleu16(buf, 30);
    if (1900 > unpacked_date.tm_year) {
        // useless, no date
        return 0;
    }
    unpacked_date.tm_year -= 1900;
    unpacked_date.tm_mon = getub(buf, 32) - 1;
    unpacked_date.tm_mday = getub(buf, 33);
    unpacked_date.tm_hour = getub(buf, 34);
    unpacked_date.tm_min = getub(buf, 35);
    unpacked_date.tm_sec = getub(buf, 36);
    session->gpsdata.log.then.tv_sec = mkgmtime(&unpacked_date);

    session->gpsdata.log.index_cnt = getleu32(buf, 0);
    session->gpsdata.log.lon = getleu32(buf, 4) * 1.0e-7;
    session->gpsdata.log.lat = getleu32(buf, 8) * 1.0e-7;
    session->gpsdata.log.altMSL = getleu32(buf, 12) * 1.0e-3;
    // hAcc CEP() unspecified...
    session->gpsdata.log.hAcc = getleu32(buf, 16) * 1.0e-3;
    session->gpsdata.log.gSpeed = getleu32(buf, 20) * 1.0e-3;
    // seems to be true heading
    session->gpsdata.log.heading = getleu32(buf, 24) * 1.0e-5;
    fixType = getub(buf, 29);
    session->gpsdata.log.numSV = getub(buf, 38);

    switch (fixType) {
    case 1:
        // doc is unclear: 2D or 3D?
        session->gpsdata.log.fixType = MODE_3D;
        session->gpsdata.log.status = STATUS_DR;
        break;
    case 2:
        session->gpsdata.log.fixType = MODE_2D;
        session->gpsdata.log.status = STATUS_GPS;
        break;
    case 3:
        session->gpsdata.log.fixType = MODE_3D;
        session->gpsdata.log.status = STATUS_GPS;
        break;
    case 4:
        // doc is unclear: 2D or 3D?
        session->gpsdata.log.fixType = MODE_3D;
        session->gpsdata.log.status = STATUS_GNSSDR;
        break;

    case 0:
        FALLTHROUGH
    default:
        // huh?
        session->gpsdata.log.fixType = MODE_NO_FIX;
        session->gpsdata.log.status = STATUS_UNK;
        break;
    }

    // (long long) because of time_t
    GPSD_LOG(LOG_INF, &session->context->errout,
             "UBX-LOG-RETRIEVEPOS: time=%lld entryIndex=%d"
             " lon=%.7f lat=%.7f altMSL=%.3f hAcc=%.3f"
             " gspeed=%.3f heading=%.5f fixType=%d numSV=%d\n",
             (long long)session->gpsdata.log.then.tv_sec,
             session->gpsdata.log.index_cnt, session->gpsdata.log.lon,
             session->gpsdata.log.lat, session->gpsdata.log.altMSL,
             session->gpsdata.log.hAcc, session->gpsdata.log.gSpeed,
             session->gpsdata.log.heading, session->gpsdata.log.fixType,
             session->gpsdata.log.numSV);


    mask |= LOG_SET;
    return mask;
}

/*
 * UBX-LOG-RETRIEVEPOSEXTRA (Indexed Odometry entry)
 * Used for GPS standalone operation and host saved logs
 * u-blox 7,8,9.  protVer 14 to 29
 */
static gps_mask_t
ubx_msg_log_retrieveposextra(struct gps_device_t *session,
                             unsigned char *buf UNUSED, size_t data_len)
{
    struct tm unpacked_date = {0};
    gps_mask_t mask = 0;

    gps_clear_log(&session->gpsdata.log);
    /* u-blox 32 bytes payload */
    if (32 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-LOG-RETRIEVEPOSEXTRA: runt len %zd", data_len);
        return 0;
    }

    unpacked_date.tm_year = getleu16(buf, 6);
    if (1900 > unpacked_date.tm_year) {
        // useless, no date
        return 0;
    }
    unpacked_date.tm_year -= 1900;
    unpacked_date.tm_mon = getub(buf, 8) - 1;
    unpacked_date.tm_mday = getub(buf, 9);
    unpacked_date.tm_hour = getub(buf, 10);
    unpacked_date.tm_min = getub(buf, 11);
    unpacked_date.tm_sec = getub(buf, 12);

    session->gpsdata.log.then.tv_sec = mkgmtime(&unpacked_date);
    session->gpsdata.log.index_cnt = getleu32(buf, 0);
    // distance units undocumented!  Assume meters, as in UBX-LOG-BATCH
    session->gpsdata.log.distance = (double)getleu32(buf, 16);

    // (long long) because of time_t
    GPSD_LOG(LOG_INF, &session->context->errout,
             "UBX-LOG-RETRIEVEPOSEXTRA:"
             " time=%lld entryindex=%u distance=%.0f\n",
             (long long)session->gpsdata.log.then.tv_sec,
             session->gpsdata.log.index_cnt, session->gpsdata.log.distance);

    mask |= LOG_SET;
    return mask;
}

/*
 * UBX-LOG-RETRIEVESTRING
 * Used for GPS standalone operation and host saved logs
 * u-blox 7,8,9.  protVer 14 to 29
 */
static gps_mask_t
ubx_msg_log_retrievestring(struct gps_device_t *session,
                           unsigned char *buf UNUSED, size_t data_len)
{
    struct tm unpacked_date = {0};
    unsigned int byteCount;
    gps_mask_t mask = 0;

    gps_clear_log(&session->gpsdata.log);
    /* u-blox 16+ bytes payload */
    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-LOG-RETRIEVESTRING: runt len %zd", data_len);
        return 0;
    }

    unpacked_date.tm_year = getleu16(buf, 6);
    if (1900 > unpacked_date.tm_year) {
        // useless, no date
        return 0;
    }
    unpacked_date.tm_year -= 1900;
    unpacked_date.tm_mon = getub(buf, 8) - 1;
    unpacked_date.tm_mday = getub(buf, 9);
    unpacked_date.tm_hour = getub(buf, 10);
    unpacked_date.tm_min = getub(buf, 11);
    unpacked_date.tm_sec = getub(buf, 12);

    session->gpsdata.log.then.tv_sec = mkgmtime(&unpacked_date);
    session->gpsdata.log.index_cnt = getleu32(buf, 0);
    byteCount = getleu16(buf, 14);

    // string could be 0 to 256 bytes, plus NUL
    (void)strlcpy(session->gpsdata.log.string, (const char*)&buf[16],
                  sizeof(session->gpsdata.log.string));
    // (long long) because of time_t
    GPSD_LOG(LOG_INF, &session->context->errout,
             "UBX-LOG-RETRIEVESTRING:"
             " time=%lld entryindex=%u byteCount=%u string=%s\n",
             (long long)session->gpsdata.log.then.tv_sec,
             session->gpsdata.log.index_cnt,
             byteCount, session->gpsdata.log.string);

    mask |= LOG_SET;
    return mask;
}

/*
 * UBX-NAV-HPPOSECEF - High Precision Position Solution in ECEF
 *
 * Present in u-blox 8 and above, protVwer 20.00 and up.
 * Only with High Precision firmware.
 */
static gps_mask_t
ubx_msg_nav_hpposecef(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    gps_mask_t mask = ECEF_SET;
    int version;

    if (28 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-HPPOSECEF message, runt payload len %zd", data_len);
        return 0;
    }

    version = getub(buf, 0);
    session->driver.ubx.iTOW = getleu32(buf, 4);
    session->newdata.ecef.x = getles32x100s8d(buf, 8, 20, 1e-4);
    session->newdata.ecef.y = getles32x100s8d(buf, 12, 21, 1e-4);
    session->newdata.ecef.z = getles32x100s8d(buf, 16, 22, 1e-4);

    session->newdata.ecef.pAcc = getleu32(buf, 24) / (double)10000.0;
    /* (long long) cast for 32-bit compat */
    GPSD_LOG(LOG_PROG, &session->context->errout,
        "UBX-NAV-HPPOSECEF: version %d iTOW=%lld ECEF x=%.4f y=%.4f z=%.4f "
        "pAcc=%.4f\n",
        version,
        (long long)session->driver.ubx.iTOW,
        session->newdata.ecef.x,
        session->newdata.ecef.y,
        session->newdata.ecef.z,
        session->newdata.ecef.pAcc);
    return mask;
}

 /**
 * High Precision Geodetic Position Solution
 * UBX-NAV-HPPOSLLH, Class 1, ID x14
 *
 * No mode, so limited usefulness.
 *
 * Present in u-blox 8 and above, protVwer 20.00 and up.
 * Only with High Precision firmware.
 */
static gps_mask_t
ubx_msg_nav_hpposllh(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    int version;
    gps_mask_t mask = 0;

    if (36 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-HPPOSLLH message, runt payload len %zd", data_len);
        return mask;
    }

    mask = ONLINE_SET | HERR_SET | VERR_SET | LATLON_SET | ALTITUDE_SET;

    version = getub(buf, 0);
    session->driver.ubx.iTOW = getles32(buf, 4);
    session->newdata.longitude = getles32x100s8d(buf, 8, 24, 1e-9);
    session->newdata.latitude = getles32x100s8d(buf, 12, 25, 1e-9);
    /* altitude WGS84 */
    session->newdata.altHAE = getles32x100s8d(buf, 16, 26, 1e-5);
    /* altitude MSL */
    session->newdata.altMSL = getles32x100s8d(buf, 20, 27, 1e-5);
    /* Let gpsd_error_model() deal with geoid_sep */

    /* Horizontal accuracy estimate in .1 mm, unknown est type */
    session->newdata.eph = getleu32(buf, 28) * (double)1e-4;
    /* Vertical accuracy estimate in .1 mm, unknown est type */
    session->newdata.epv = getleu32(buf, 32) * (double)1e-4;

    GPSD_LOG(LOG_PROG, &session->context->errout,
        "UBX-NAV-HPPOSLLH: version %d iTOW=%lld lat=%.4f lon=%.4f "
        "altHAE=%.4f\n",
        version,
        (long long)session->driver.ubx.iTOW,
        session->newdata.latitude,
        session->newdata.longitude,
        session->newdata.altHAE);
    return mask;
}

/*
 * Navigation Position ECEF message
 *
 * This message does not bother to tell us if it is valid.
 */
static gps_mask_t
ubx_msg_nav_posecef(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    gps_mask_t mask = ECEF_SET;

    if (20 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-POSECEF message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    /* all in cm */
    session->newdata.ecef.x = getles32(buf, 4) * 1e-2;
    session->newdata.ecef.y = getles32(buf, 8) * 1e-2;
    session->newdata.ecef.z = getles32(buf, 12) * 1e-2;
    session->newdata.ecef.pAcc = getleu32(buf, 16) * 1e-2;

    /* (long long) cast for 32-bit compat */
    GPSD_LOG(LOG_PROG, &session->context->errout,
        "UBX-NAV-POSECEF: iTOW=%lld ECEF x=%.2f y=%.2f z=%.2f pAcc=%.2f\n",
        (long long)session->driver.ubx.iTOW,
        session->newdata.ecef.x,
        session->newdata.ecef.y,
        session->newdata.ecef.z,
        session->newdata.ecef.pAcc);
    return mask;
}

/**
 * Navigation Position Velocity Time solution message
 * UBX-NAV-PVT Class 1, ID 7
 *
 * Not in u-blox 5 or 6, present in u-blox 7
 * u-blox 6 w/ GLONASS, protver 14 have NAV-PVT
 */
static gps_mask_t
ubx_msg_nav_pvt(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    uint8_t valid;
    uint8_t flags;
    uint8_t fixType;
    struct tm unpacked_date;
    int *status = &session->newdata.status;
    int *mode = &session->newdata.mode;
    gps_mask_t mask = 0;
    char ts_buf[TIMESPEC_LEN];

    /* u-blox 6 and 7 are 84 bytes, u-blox 8 and 9 are 92 bytes  */
    if (84 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-PVT message, runt payload len %zd", data_len);
        return 0;
    }

    if (14 > session->driver.ubx.protver) {
        /* this GPS is at least protver 14 */
        session->driver.ubx.protver = 14;
    }
    session->driver.ubx.iTOW = getleu32(buf, 0);
    valid = (unsigned int)getub(buf, 11);
    fixType = (unsigned char)getub(buf, 20);
    flags = (unsigned int)getub(buf, 21);

    switch (fixType) {
    case UBX_MODE_TMONLY:
        // 5 - Surveyed-in, so a precise 3D.
        *mode = MODE_3D;
        *status = STATUS_TIME;
        mask |= STATUS_SET | MODE_SET;
        break;

    case UBX_MODE_3D:
        // 3
        FALLTHROUGH
    case UBX_MODE_GPSDR:
        // 4
        if (*mode != MODE_3D) {
            *mode = MODE_3D;
            mask |= MODE_SET;
        }
        if ((flags & UBX_NAV_PVT_FLAG_DGPS) == UBX_NAV_PVT_FLAG_DGPS) {
            *status = STATUS_DGPS;
            mask |= STATUS_SET;
        } else {
            *status = STATUS_GPS;
            mask |= STATUS_SET;
        }
        mask |=   LATLON_SET;
        break;

    case UBX_MODE_2D:
        // 2
        FALLTHROUGH
    case UBX_MODE_DR:           /* consider this too as 2D */
        // 1
        if (MODE_2D != *mode) {
            *mode = MODE_2D;
            mask |= MODE_SET;
        };
        if (STATUS_GPS != *status) {
            // FIXME: Set DR if this is DR
            *status = STATUS_GPS;
            mask |= STATUS_SET;
        }
        mask |= LATLON_SET | SPEED_SET;
        break;

    case UBX_MODE_NOFIX:
        // 0
        FALLTHROUGH
    default:
        // huh?
        if (*mode != MODE_NO_FIX) {
            *mode = MODE_NO_FIX;
            mask |= MODE_SET;
        };
        if (*status != STATUS_UNK) {
            *status = STATUS_UNK;
            mask |= STATUS_SET;
        }
        break;
    }

    if ((valid & UBX_NAV_PVT_VALID_DATE_TIME) == UBX_NAV_PVT_VALID_DATE_TIME) {
        unpacked_date.tm_year = (uint16_t)getleu16(buf, 4) - 1900;
        unpacked_date.tm_mon = (uint8_t)getub(buf, 6) - 1;
        unpacked_date.tm_mday = (uint8_t)getub(buf, 7);
        unpacked_date.tm_hour = (uint8_t)getub(buf, 8);
        unpacked_date.tm_min = (uint8_t)getub(buf, 9);
        unpacked_date.tm_sec = (uint8_t)getub(buf, 10);
        unpacked_date.tm_isdst = 0;
        unpacked_date.tm_wday = 0;
        unpacked_date.tm_yday = 0;
        session->newdata.time.tv_sec = mkgmtime(&unpacked_date);
        /* field 16, nano, can be negative! So normalize */
        session->newdata.time.tv_nsec = getles32(buf, 16);
        TS_NORM(&session->newdata.time);
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }

    session->newdata.longitude = 1e-7 * getles32(buf, 24);
    session->newdata.latitude = 1e-7 * getles32(buf, 28);
    /* altitude WGS84 */
    session->newdata.altHAE = 1e-3 * getles32(buf, 32);
    /* altitude MSL */
    session->newdata.altMSL = 1e-3 * getles32(buf, 36);
    /* Let gpsd_error_model() deal with geoid_sep */

    session->newdata.speed = 1e-3 * (int32_t)getles32(buf, 60);
    /* u-blox calls this Heading of motion (2-D) */
    session->newdata.track = 1e-5 * (int32_t)getles32(buf, 64);
    mask |= LATLON_SET | ALTITUDE_SET | SPEED_SET | TRACK_SET;

    /* u-blox does not document the basis for the following "accuracy"
     * estimates.  Maybe CEP(50), one sigma, two sigma, CEP(99), etc. */

    /* Horizontal Accuracy estimate, in mm */
    session->newdata.eph = (double)(getles32(buf, 40) / 1000.0);
    /* Vertical Accuracy estimate, in mm */
    session->newdata.epv = (double)(getles32(buf, 44) / 1000.0);
    /* Speed Accuracy estimate, in mm/s */
    session->newdata.eps = (double)(getles32(buf, 68) / 1000.0);
    /* let gpsd_error_model() do the rest */

    mask |= HERR_SET | SPEEDERR_SET | VERR_SET;
    // if cycle ender worked, could get rid of this REPORT_IS.
    // mask |= REPORT_IS;

    GPSD_LOG(LOG_PROG, &session->context->errout,
         "NAV-PVT: flags=%02x time=%s lat=%.2f lon=%.2f altHAE=%.2f "
         "track=%.2f speed=%.2f climb=%.2f mode=%d status=%d used=%d\n",
         flags,
         timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)),
         session->newdata.latitude,
         session->newdata.longitude,
         session->newdata.altHAE,
         session->newdata.track,
         session->newdata.speed,
         session->newdata.climb,
         session->newdata.mode,
         session->newdata.status,
         session->gpsdata.satellites_used);
    if (92 <= data_len) {
        // u-blox 8 and 9 extended
        double magDec = NAN;
        double magAcc = NAN;
#ifdef __UNUSED
        if (flags & UBX_NAV_PVT_FLAG_HDG_OK) {
            /* u-blox calls this Heading of vehicle (2-D)
             * why is it different than earlier track? */
            session->newdata.track = (double)(getles32(buf, 84) * 1e-5);
        }
#endif  // __UNUSED
        if (valid & UBX_NAV_PVT_VALID_MAG) {
            magDec = (double)(getles16(buf, 88) * 1e-2);
            magAcc = (double)(getleu16(buf, 90) * 1e-2);
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
             "  headVeh %.5f magDec %.2f magAcc %.2f\n",
             session->newdata.track, magDec, magAcc);
    }
    return mask;
}


 /**
 * High Precision Relative Positioning Information in NED frame
 * UBX-NAV-RELPOSNED, Class 1, ID x3c
 * HP GNSS only, protver 20+
 */
static gps_mask_t
ubx_msg_nav_relposned(struct gps_device_t *session, unsigned char *buf,
                      size_t data_len)
{
    int version;
    unsigned flags;
    double accN = NAN, accE = NAN, accD = NAN, accL = NAN, accH = NAN;;
    gps_mask_t mask = 0;

    if (40 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-RELPOSNED:0 message, runt payload len %zd",
                 data_len);
        return mask;
    }
    version = getub(buf, 0);
    /* WTF?  u-blox did not make this sentence upward compatible
     * 40 bytes in Version 0, protVer 20 to 27
     * 64 bytes in Version 1, protVer 27.11+ */

    session->newdata.dgps_station = getleu16(buf, 2);          // 0 to 4095
    session->driver.ubx.iTOW = getleu32(buf, 4);
    if (1 > version) {
        // version 0
        flags = getleu32(buf, 36);
        if (1 != (1 & flags)) {
            // not gnssFixOK
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "UBX-NAV-RELPOSNED:0 no fix");
            return mask;
        }
        if (4 & flags) {
            // rePosValid
            session->newdata.NED.relPosN = getles32x100s8d(buf, 8, 20, 1e-4);
            session->newdata.NED.relPosE = getles32x100s8d(buf, 12, 21, 1e-4);
            session->newdata.NED.relPosD = getles32x100s8d(buf, 16, 22, 1e-4);

            accN = 1e-4 * getles32(buf, 24);
            accE = 1e-4 * getles32(buf, 28);
            accD = 1e-4 * getles32(buf, 32);
            mask |= NED_SET;
        }
    } else {
        // assume version 1
        if (64 > data_len) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "UBX-NAV-RELPOSNED:1 message, runt payload len %zd",
                     data_len);
            return mask;
        }
        flags = getleu32(buf, 60);
        if (1 != (1 & flags)) {
            // not gnssFixOK
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "UBX-NAV-RELPOSNED:1 no fix");
            return mask;
        }
        if (4 & flags) {
            // rePosValid
            session->newdata.NED.relPosN = getles32x100s8d(buf, 8, 32, 1e-4);
            session->newdata.NED.relPosE = getles32x100s8d(buf, 12, 33, 1e-4);
            session->newdata.NED.relPosD = getles32x100s8d(buf, 16, 34, 1e-4);
            session->newdata.NED.relPosL = getles32x100s8d(buf, 20, 35, 1e-4);

            accN = 1e-4 * getles32(buf, 36);
            accE = 1e-4 * getles32(buf, 40);
            accD = 1e-4 * getles32(buf, 44);
            accL = 1e-4 * getles32(buf, 48);
            accH = 1e-4 * getles32(buf, 52);
            if (0x100 & flags) {
                // relPosHeadingValid
                session->newdata.NED.relPosH = 1e-5 * getles32(buf, 24);
            }
            mask |= NED_SET;
        }
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
        "UBX-NAV-RELPOSNED: version %d iTOW=%lld refStationId %u flags x%x\n"
        "UBX-NAV-RELPOSNED: relPos N=%.4f E=%.4f D=%.4f\n"
        "UBX-NAV-RELPOSNED: acc N=%.4f E=%.4f D=%.4f L=%.4f H=%.4f\n",
        version,
        (long long)session->driver.ubx.iTOW,
        session->newdata.dgps_station,
        flags,
        session->newdata.NED.relPosN,
        session->newdata.NED.relPosE,
        session->newdata.NED.relPosD,
        accN, accE, accD, accL, accH);

    if (5 != (flags & 5)) {
        /* gnssFixOK or relPosValid are false, no fix */
        return 0;
    }
    return mask;
}

/**
 * Navigation solution message: UBX-NAV-SOL
 *
 * UBX-NAV-SOL, present in Antaris, up to 23,01
 * deprecated in u-blox 6, gone in u-blox 9.
 * Use UBX-NAV-PVT instead
 *
 * UBX-NAV-SOL has ECEF and VECEF, so no need for UBX-NAV-POSECEF and
 * UBX-NAV-VELECEF
 */
static gps_mask_t ubx_msg_nav_sol(struct gps_device_t *session,
                                  unsigned char *buf, size_t data_len)
{
    unsigned flags, pdop;
    unsigned char navmode;
    gps_mask_t mask;
    char ts_buf[TIMESPEC_LEN];

    if (52 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-SOL message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    flags = (unsigned int)getub(buf, 11);
    mask = 0;
#define DATE_VALID      (UBX_SOL_VALID_WEEK | UBX_SOL_VALID_TIME)
    if ((flags & DATE_VALID) == DATE_VALID) {
        unsigned short week;
        timespec_t ts_tow;

        MSTOTS(&ts_tow, session->driver.ubx.iTOW);
        ts_tow.tv_nsec += (long)getles32(buf, 4);
        TS_NORM(&ts_tow);
        week = (unsigned short)getles16(buf, 8);
        session->newdata.time = gpsd_gpstime_resolv(session, week, ts_tow);
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }
#undef DATE_VALID

    session->newdata.ecef.x = getles32(buf, 12) / 100.0;
    session->newdata.ecef.y = getles32(buf, 16) / 100.0;
    session->newdata.ecef.z = getles32(buf, 20) / 100.0;
    session->newdata.ecef.pAcc = getleu32(buf, 24) / 100.0;
    session->newdata.ecef.vx = getles32(buf, 28) / 100.0;
    session->newdata.ecef.vy = getles32(buf, 32) / 100.0;
    session->newdata.ecef.vz = getles32(buf, 36) / 100.0;
    session->newdata.ecef.vAcc = getleu32(buf, 40) / 100.0;
    mask |= ECEF_SET | VECEF_SET;

    session->newdata.eps = (double)(getles32(buf, 40) / 100.0);
    mask |= SPEEDERR_SET;

    pdop = getleu16(buf, 44);
    if (9999 > pdop) {
        session->gpsdata.dop.pdop = (double)(pdop / 100.0);
        mask |= DOP_SET;
    }
    session->gpsdata.satellites_used = (int)getub(buf, 47);

    navmode = (unsigned char)getub(buf, 10);
    switch (navmode) {
    case UBX_MODE_TMONLY:
        // Surveyed-in, better not have moved
        session->newdata.mode = MODE_3D;
        session->newdata.status = STATUS_TIME;
        break;
    case UBX_MODE_3D:
        session->newdata.mode = MODE_3D;
        session->newdata.status = STATUS_GPS;
        break;
    case UBX_MODE_2D:
        session->newdata.mode = MODE_2D;
        session->newdata.status = STATUS_GPS;
        break;
    case UBX_MODE_DR:           // consider this too as 2D
        session->newdata.mode = MODE_2D;
        session->newdata.status = STATUS_DR;
        break;
    case UBX_MODE_GPSDR:        // DR-aided GPS is valid 3D
        session->newdata.mode = MODE_3D;
        session->newdata.status = STATUS_GNSSDR;
        break;
    default:
        session->newdata.mode = MODE_NO_FIX;
        session->newdata.status = STATUS_UNK;
        break;
    }

    if (0 != (flags & UBX_SOL_FLAG_DGPS))
        session->newdata.status = STATUS_DGPS;

    mask |= MODE_SET | STATUS_SET;
    // older u-blox, cycle ender may be iffy
    // so err o nthe side of over-reporting TPV
    mask |= REPORT_IS;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-NAV-SOL: time=%s ecef x:%.2f y:%.2f z:%.2f track=%.2f "
             "speed=%.2f climb=%.2f mode=%d status=%d used=%d\n",
             timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)),
             session->newdata.ecef.x,
             session->newdata.ecef.y,
             session->newdata.ecef.z,
             session->newdata.track,
             session->newdata.speed,
             session->newdata.climb,
             session->newdata.mode,
             session->newdata.status,
             session->gpsdata.satellites_used);
    return mask;
}


/**
 * Receiver navigation status
 * UBX-NAV-STATUS Class 1, ID 3
 *
 * Present in Antaris to 9-series
 */
static gps_mask_t
ubx_msg_nav_status(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    uint8_t gpsFix;
    uint8_t flags;
    uint8_t fixStat;
    uint8_t flags2;
    uint32_t ttff;
    uint32_t msss;
    int *status = &session->newdata.status;
    int *mode = &session->newdata.mode;
    gps_mask_t mask = 0;

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-STATUS message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    gpsFix = getub(buf, 4);
    flags = getub(buf, 5);
    fixStat = getub(buf, 6);
    flags2 = getub(buf, 7);
    ttff = getleu32(buf, 8);
    msss = getleu32(buf, 12);

    // FIXME: how does this compare with other places ubx sets mode/status?
    if (0 == (1 & flags)) {
        // gpsFix not OK
        *mode = MODE_NO_FIX;
        *status = STATUS_UNK;
    } else {
        switch (gpsFix) {
        case UBX_MODE_TMONLY:
            // 5 - Surveyed-in, so a precise 3D.
            *mode = MODE_3D;
            *status = STATUS_TIME;
            break;

        case UBX_MODE_3D:
            // 3
            FALLTHROUGH
        case UBX_MODE_GPSDR:
            // 4
            *mode = MODE_3D;
            if (2 == (2 & fixStat)) {
                *status = STATUS_DGPS;
            } else {
                // FIXME:  Set DR if this is DR
                *status = STATUS_GPS;
            }
            break;

        case UBX_MODE_2D:
            // 2
            FALLTHROUGH
        case UBX_MODE_DR:           // consider this too as 2D
            // 1
            *mode = MODE_2D;
            if (2 == (2 & fixStat)) {
                *status = STATUS_DGPS;
            } else {
                // FIXME:  Set DR if this is DR
                *status = STATUS_GPS;
            }
            break;

        case UBX_MODE_NOFIX:
            // 0
            FALLTHROUGH
        default:
            // > 5
            *mode = MODE_NO_FIX;
            *status = STATUS_UNK;
            break;
        }
    }
    mask |= STATUS_SET | MODE_SET;

    GPSD_LOG(LOG_PROG, &session->context->errout,
         "NAV-STATUS: iTOW=%lld gpsFix=%u flags=%02x fixStat=%02x flags2=%02x "
         "ttff=%llu msss=%llu mode=%u status=%u\n",
         (long long)session->driver.ubx.iTOW,
         gpsFix,
         flags,
         fixStat,
         flags2,
         (long long unsigned)ttff,
         (long long unsigned)msss,
         session->newdata.mode,
         session->newdata.status);
    return mask;
}

/**
 * Navigation time to leap second: UBX-NAV-TIMELS
 *
 * Sets leap_notify if leap second is < 23 hours away.
 * Not in u-blox 5
 */
static gps_mask_t
ubx_msg_nav_timels(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    int version;
    unsigned int flags;
    int valid_curr_ls;
    int valid_time_to_ls_event;

#define UBX_TIMELS_VALID_CURR_LS 0x01
#define UBX_TIMELS_VALID_TIME_LS_EVT 0x01

    if (24 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-TIMELS: unexpected length %zd, expecting 24\n",
                 data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    version = getsb(buf, 4);
    // Only version 0 is defined up to ub-blox 9
    flags = (unsigned int)getub(buf, 23);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-NAV-TIMELS: flags 0x%x message version %d\n",
             flags, version);
    valid_curr_ls = flags & UBX_TIMELS_VALID_CURR_LS;
    valid_time_to_ls_event = flags & UBX_TIMELS_VALID_TIME_LS_EVT;
    if (valid_curr_ls) {
        unsigned int src_of_curr_ls = getub(buf,8);
        int curr_ls = getsb(buf,9);
        char *src = "Unknown";
        static char *srcOfCurrLs[] = {
            "firmware",
            "GPS GLONASS difference",
            "GPS",
            "SBAS",
            "BeiDou",
            "Galileo",
            "Aided data",
            "Configured"
        };

        if (src_of_curr_ls < (sizeof(srcOfCurrLs) / sizeof(srcOfCurrLs[0])))
            src = srcOfCurrLs[src_of_curr_ls];

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-NAV-TIMELS: source_of_current_leapsecond=%u:%s "
                 "curr_ls=%d\n",
                 src_of_curr_ls, src,curr_ls);
        session->context->leap_seconds = curr_ls;
        session->context->valid |= LEAP_SECOND_VALID;
    } /* Valid current leap second */

    if (valid_time_to_ls_event) {
        char *src = "Unknown";
        unsigned int src_of_ls_change;
        unsigned short dateOfLSGpsWn, dateOfLSGpsDn;
        int lsChange = getsb(buf, 11);
        int timeToLsEvent = getles32(buf, 12);
        static char *srcOfLsChange[] = {
            "No Source",
            "Undefined",
            "GPS",
            "SBAS",
            "BeiDou",
            "Galileo",
            "GLONASS",
        };

        src_of_ls_change = getub(buf,10);
        if (src_of_ls_change <
            (sizeof(srcOfLsChange) / sizeof(srcOfLsChange[0]))) {
            src = srcOfLsChange[src_of_ls_change];
        }

        dateOfLSGpsWn = getles16(buf,16);
        dateOfLSGpsDn = getles16(buf,18);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-NAV-TIMELS: source_of_leapsecond_change %u:%s "
                 "leapSecondChage %d timeToLsEvent %d\n",
                 src_of_ls_change,src,lsChange,timeToLsEvent);

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-NAV-TIMELS: dateOfLSGpsWn=%d dateOfLSGpsDn=%d\n",
                 dateOfLSGpsWn,dateOfLSGpsDn);
        if ((0 != lsChange) && (0 < timeToLsEvent) &&
            ((60 * 60 * 23) > timeToLsEvent)) {
            if (1 == lsChange) {
                session->context->leap_notify = LEAP_ADDSECOND;
                GPSD_LOG(LOG_INF, &session->context->errout,
                         "UBX-NAV-TIMELS: Positive leap second today\n");
            } else if (-1 == lsChange) {
                session->context->leap_notify = LEAP_DELSECOND;
                GPSD_LOG(LOG_INF, &session->context->errout,
                         "UBX-NAV-TIMELS: Negative leap second today\n");
            }
        } else {
            session->context->leap_notify = LEAP_NOWARNING;
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "UBX-NAV-TIMELS: leap_notify %d, none today\n",
                     session->context->leap_notify);
        }
    }
    return 0;
}

 /**
 * Geodetic position solution message
 * UBX-NAV-POSLLH, Class 1, ID 2
 *
 * This message does not bother to tell us if it is valid.
 * No mode, so limited usefulness
 */
static gps_mask_t
ubx_msg_nav_posllh(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len UNUSED)
{
    gps_mask_t mask = 0;

    if (28 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-POSLLH message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    session->newdata.longitude = 1e-7 * getles32(buf, 4);
    session->newdata.latitude = 1e-7 * getles32(buf, 8);
    /* altitude WGS84 */
    session->newdata.altHAE = 1e-3 * getles32(buf, 12);
    /* altitude MSL */
    session->newdata.altMSL = 1e-3 * getles32(buf, 16);
    /* Let gpsd_error_model() deal with geoid_sep */

    /* Horizontal accuracy estimate in mm, unknown type */
    session->newdata.eph = getleu32(buf, 20) * 1e-3;
    /* Vertical accuracy estimate in mm, unknown type */
    session->newdata.epv = getleu32(buf, 24) * 1e-3;

    GPSD_LOG(LOG_PROG, &session->context->errout,
        "UBX-NAV-POSLLH: iTOW=%lld lat=%.3f lon=%.3f altHAE=%.3f "
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
 * Clock Solution
 *
 * Present in u-blox 7
 */
static gps_mask_t
ubx_msg_nav_clock(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    long clkB, clkD;
    unsigned long tAcc, fAcc;

    if (20 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-CLOCK message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    clkB = getles32(buf, 4);
    clkD = getles32(buf, 8);
    tAcc = getleu32(buf, 12);
    fAcc = getleu32(buf, 16);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NAV-CLOCK: iTOW=%lld clkB %ld clkD %ld tAcc %lu fAcc %lu\n",
             (long long)session->driver.ubx.iTOW, clkB, clkD, tAcc, fAcc);
    return 0;
}

/**
 * DGPS Data Used for NAV
 *
 * May be good cycle ender
 *
 * Present in u-blox 7
 */
static gps_mask_t ubx_msg_nav_dgps(struct gps_device_t *session,
                                   unsigned char *buf, size_t data_len)
{
    long age;

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-DGPS message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    age = getleu32(buf, 4);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NAV-DGPS: iTOW=%lld age %ld\n",
             (long long)session->driver.ubx.iTOW, age);
    return 0;
}

/**
 * Dilution of precision message
 */
static gps_mask_t
ubx_msg_nav_dop(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned u;
    gps_mask_t mask = 0;

    if (18 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-DOP message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
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
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NAV-DOP: gdop=%.2f pdop=%.2f "
             "hdop=%.2f vdop=%.2f tdop=%.2f mask={DOP}\n",
             session->gpsdata.dop.gdop,
             session->gpsdata.dop.hdop,
             session->gpsdata.dop.vdop,
             session->gpsdata.dop.pdop, session->gpsdata.dop.tdop);
    return mask;
}

/**
 * Position error ellipse parameters
 * protVer 19.1 and up
 * Not in u-blox 5, 6 or 7
 * Present in some u-blox 8, 9 and 10 (ADR, HPS)
 */
static gps_mask_t
ubx_msg_nav_eell(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned version;
    unsigned errEllipseOrient;
    unsigned long errEllipseMajor, errEllipseMinor;

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-EELL message, runt payload len %zd", data_len);
        return 0;
    }

    if (18 > session->driver.ubx.protver) {
        /* this GPS is at least protver 18 */
        session->driver.ubx.protver = 18;
    }
    session->driver.ubx.iTOW = getleu32(buf, 0);
    version = getub(buf, 4);
    errEllipseOrient = getleu16(buf, 6);
    errEllipseMajor = getleu32(buf, 8);
    errEllipseMinor = getleu32(buf, 12);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-NAV-EELL: iTOW %lld version %u errEllipseOrient %u "
             "errEllipseMajor %lu errEllipseMinor %lu\n",
             (long long)session->driver.ubx.iTOW, version, errEllipseOrient,
             errEllipseMajor, errEllipseMinor);
    return 0;
}

/**
 * End of Epoch
 * Not in u-blox 5, 6 or 7
 * Present in u-blox 8 and 9
 */
static gps_mask_t
ubx_msg_nav_eoe(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    if (4 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-EOE message, runt payload len %zd", data_len);
        return 0;
    }

    if (18 > session->driver.ubx.protver) {
        /* this GPS is at least protver 18 */
        session->driver.ubx.protver = 18;
    }
    session->driver.ubx.iTOW = getleu32(buf, 0);
    GPSD_LOG(LOG_PROG, &session->context->errout, "NAV-EOE: iTOW=%lld\n",
             (long long)session->driver.ubx.iTOW);
    // nothing to report, but the iTOW for cycle ender is good
    return 0;
}

/**
 * GPS Leap Seconds - UBX-NAV-TIMEGPS
 */
static gps_mask_t
ubx_msg_nav_timegps(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    uint8_t valid;         /* Validity Flags */
    gps_mask_t mask = 0;
    char ts_buf[TIMESPEC_LEN];

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-TIMEGPS message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    valid = getub(buf, 11);
    // Valid leap seconds ?
    if ((valid & UBX_TIMEGPS_VALID_LEAP_SECOND) ==
        UBX_TIMEGPS_VALID_LEAP_SECOND) {
        session->context->leap_seconds = (int)getub(buf, 10);
        session->context->valid |= LEAP_SECOND_VALID;
    }
    // Valid GPS time of week and week number
#define VALID_TIME (UBX_TIMEGPS_VALID_TIME | UBX_TIMEGPS_VALID_WEEK)
    if ((valid & VALID_TIME) == VALID_TIME) {
#undef VALID_TIME
        uint16_t week;
        double tAcc;      /* Time Accuracy Estimate in ns */
        timespec_t ts_tow;

        week = getles16(buf, 8);
        MSTOTS(&ts_tow, session->driver.ubx.iTOW);
        ts_tow.tv_nsec += (long)getles32(buf, 4);
        TS_NORM(&ts_tow);
        session->newdata.time = gpsd_gpstime_resolv(session, week, ts_tow);

        tAcc = (double)getleu32(buf, 12);     // tAcc in ns
        session->newdata.ept = tAcc * 1e-9;
        mask |= (TIME_SET | NTPTIME_IS);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "TIMEGPS: time=%s mask={TIME}\n",
             timespec_str(&session->newdata.time, ts_buf, sizeof(ts_buf)));
    return mask;
}

/**
 * UBX-NAV-TIMEUTC
 */
static gps_mask_t
ubx_msg_nav_timeutc(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    uint8_t valid;         // Validity Flags
    gps_mask_t mask = 0;

    if (20 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-TIMEUTC message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    valid = getub(buf, 19);
    if (4 == (4 & valid)) {
        // UTC is valid
        struct tm date = {0};
        // mask |= (TIME_SET | NTPTIME_IS);
        uint32_t tAcc = getleu32(buf, 4);          // tAcc in ns
        // nano can be negative, so this is not normalized UTC.
        int32_t nano = getles32(buf, 8);           // fract sec in ns
        date.tm_year = getleu16(buf, 12) - 1900;   // year, 1999..2099
        date.tm_mon = getub(buf, 14) - 1;          // month 1..12
        date.tm_mday = getub(buf, 15);             // day 1..31
        date.tm_hour = getub(buf, 16);             // hour 0..23
        date.tm_min = getub(buf, 17);              // min 0..59
        date.tm_sec = getub(buf, 18);              // sec 0..60
        session->newdata.time.tv_sec = mkgmtime(&date);
        session->newdata.time.tv_nsec = nano;
        // nano, can be negative! So normalize
        TS_NORM(&session->newdata.time);
        // other timestamped messages lack nano, so time will jump around...
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-NAV-TIMEUTC: iTOW=%lld valid=%02x %04d-%02d-%02d "
                 "%02d:%02d:%02d.%09d tAcc=%llu time %lld.%09lld\n",
                 (long long)session->driver.ubx.iTOW,
                 valid, date.tm_year + 1900, date.tm_mon + 1, date.tm_mday,
                 date.tm_hour, date.tm_min, date.tm_sec, nano,
                 (long long unsigned)tAcc,
                 (long long)session->newdata.time.tv_sec,
                 (long long)session->newdata.time.tv_nsec);
    } else {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "TIMEUTC: iTOW=%lld valid=%02x\n",
                 (long long)session->driver.ubx.iTOW,
                 valid);
    }
    return mask;
}

/**
 * GPS Satellite Info -- new style UBX-NAV-SAT
 * Not in u-blox 5
 * Present in u-blox 8,  protocol version 15+
 */
static gps_mask_t
ubx_msg_nav_sat(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned int i, nchan, nsv, st, ver;
    timespec_t ts_tow;

    if (8 > data_len) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-NAV-SAT runt datalen %zd\n", data_len);
        return 0;
    }

    if (15 > session->driver.ubx.protver) {
        /* this GPS is at least protver 15 */
        session->driver.ubx.protver = 15;
    }
    session->driver.ubx.iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->driver.ubx.iTOW);
    session->gpsdata.skyview_time =
        gpsd_gpstime_resolv(session, session->context->gps_week, ts_tow);

    ver = (unsigned int)getub(buf, 4);
    if (1 != ver) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NAV-SAT message unknown version %d", ver);
        return 0;
    }
    nchan = (unsigned int)getub(buf, 5);
    if (nchan > MAXCHANNELS) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-SAT message, runt >%d reported visible",
                 MAXCHANNELS);
        return 0;
    }
    /* two "unused" bytes at buf[6:7] */

    gpsd_zero_satellites(&session->gpsdata);
    nsv = 0;
    for (i = st = 0; i < nchan; i++) {
        unsigned int off = 8 + 12 * i;
        short nmea_PRN = 0;
        unsigned char gnssId = getub(buf, off + 0);
        short svId = (short)getub(buf, off + 1);
        unsigned char cno = getub(buf, off + 2);
        /* health data in flags. */
        uint32_t flags = getleu32(buf, off + 8);
        bool used = (bool)(flags  & 0x08);
        int tmp;
        /* Notice NO sigid! */

        nmea_PRN = ubx2_to_prn(gnssId, svId);

#ifdef __UNUSED
        // debug
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NAV-SAT gnssid %d, svid %d nmea_PRN %d\n",
                 gnssId, svId, nmea_PRN);
#endif  // __UNUSED

        session->gpsdata.skyview[st].gnssid = gnssId;
        session->gpsdata.skyview[st].svid = svId;
        session->gpsdata.skyview[st].PRN = nmea_PRN;

        session->gpsdata.skyview[st].ss = (double)cno;
        tmp = getsb(buf, off + 3);
        if (90 >= abs(tmp)) {
            session->gpsdata.skyview[st].elevation = (double)tmp;
        }
        tmp = getles16(buf, off + 4);
        if (359 > tmp && 0 <= tmp) {
            session->gpsdata.skyview[st].azimuth = (double)tmp;
        }
        session->gpsdata.skyview[st].used = used;
        /* by some coincidence, our health flags matches u-blox's */
        session->gpsdata.skyview[st].health = (flags >> 4) & 3;
        /* sbas_in_use is not same as used */
        if (used) {
            nsv++;
            session->gpsdata.skyview[st].used = true;
        }
        st++;
    }

    session->gpsdata.satellites_visible = (int)st;
    session->gpsdata.satellites_used = (int)nsv;
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "SAT: visible=%d used=%d mask={SATELLITE|USED}\n",
             session->gpsdata.satellites_visible,
             session->gpsdata.satellites_used);
    return SATELLITE_SET | USED_IS;
}

/**
 * GPS Satellite Info -- deprecated - UBX-NAV-SVINFO
 * Not in u-blox 9 or 10, use UBX-NAV-SAT instead
 */
static gps_mask_t
ubx_msg_nav_svinfo(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    unsigned int i, nchan, nsv, st;
    timespec_t ts_tow;

    if (8 > data_len) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-NAV-SVINFO runt datalen %zd\n", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->driver.ubx.iTOW);
    session->gpsdata.skyview_time =
        gpsd_gpstime_resolv(session, session->context->gps_week, ts_tow);

    nchan = (unsigned int)getub(buf, 4);
    if (nchan > MAXCHANNELS) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV SVINFO message, runt >%d reported visible",
                 MAXCHANNELS);
        return 0;
    }
    gpsd_zero_satellites(&session->gpsdata);
    nsv = 0;
    for (i = st = 0; i < nchan; i++) {
        unsigned int off = 8 + 12 * i;
        short nmea_PRN;
        short ubx_PRN = (short)getub(buf, off + 1);
        unsigned char snr = getub(buf, off + 4);
        bool used = (bool)(getub(buf, off + 2) & 0x01);
        unsigned char flags = getub(buf, off + 12) & 3;
        int tmp;

        nmea_PRN = ubx_to_prn(ubx_PRN,
                              &session->gpsdata.skyview[st].gnssid,
                              &session->gpsdata.skyview[st].svid);

#ifdef __UNUSED
        // debug
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NAV-SVINFO ubx_prn %d gnssid %d, svid %d nmea_PRN %d\n",
                 ubx_PRN,
                 session->gpsdata.skyview[st].gnssid,
                 session->gpsdata.skyview[st].svid, nmea_PRN);
#endif  // __UNUSED
        if (1 > nmea_PRN) {
            // skip bad PRN
            continue;
        }
        session->gpsdata.skyview[st].PRN = nmea_PRN;

        session->gpsdata.skyview[st].ss = (double)snr;
        tmp = getsb(buf, off + 5);
        if (90 >= abs(tmp)) {
            session->gpsdata.skyview[st].elevation = (double)tmp;
        }
        tmp = (double)getles16(buf, off + 6);
        if (359 > tmp && 0 <= tmp) {
            session->gpsdata.skyview[st].azimuth = (double)tmp;
        }
        session->gpsdata.skyview[st].used = used;
        if (0x10 & flags) {
           session->gpsdata.skyview[st].health = SAT_HEALTH_BAD;
        } else {
           session->gpsdata.skyview[st].health = SAT_HEALTH_OK;
        }

        /* sbas_in_use is not same as used */
        if (used) {
            /* not really 'used', just integrity data from there */
            nsv++;
            session->gpsdata.skyview[st].used = true;
        }
        st++;
    }

    session->gpsdata.satellites_visible = (int)st;
    session->gpsdata.satellites_used = (int)nsv;
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "SVINFO: visible=%d used=%d mask={SATELLITE|USED}\n",
             session->gpsdata.satellites_visible,
             session->gpsdata.satellites_used);
    return SATELLITE_SET | USED_IS;
}

/*
 * Velocity Position ECEF message, UBX-NAV-VELECEF
 */
static gps_mask_t
ubx_msg_nav_velecef(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    gps_mask_t mask = VECEF_SET;

    if (20 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-VELECEF message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    session->newdata.ecef.vx = getles32(buf, 4) / 100.0;
    session->newdata.ecef.vy = getles32(buf, 8) / 100.0;
    session->newdata.ecef.vz = getles32(buf, 12) / 100.0;
    session->newdata.ecef.vAcc = getleu32(buf, 16) / 100.0;
    GPSD_LOG(LOG_PROG, &session->context->errout,
        "UBX-NAV-VELECEF: iTOW=%lld ECEF vx=%.2f vy=%.2f vz=%.2f vAcc=%.2f\n",
        (long long)session->driver.ubx.iTOW,
        session->newdata.ecef.vx,
        session->newdata.ecef.vy,
        session->newdata.ecef.vz,
        session->newdata.ecef.vAcc);
    return mask;
}

/*
 * Velocity NED message, UBX-NAV-VELNED
 * protocol versions 15+
 */
static gps_mask_t
ubx_msg_nav_velned(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    gps_mask_t mask = VNED_SET;

    if (36 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-VELNED message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    session->newdata.NED.velN = getles32(buf, 4) / 100.0;
    session->newdata.NED.velE = getles32(buf, 8) / 100.0;
    session->newdata.NED.velD = getles32(buf, 12) / 100.0;
    /* ignore speed for now */
    GPSD_LOG(LOG_PROG, &session->context->errout,
        "UBX-NAV-VELNED: iTOW=%lld NED velN=%.2f velE=%.2f velD=%.2f\n",
         (long long)session->driver.ubx.iTOW,
        session->newdata.NED.velN,
        session->newdata.NED.velE,
        session->newdata.NED.velD);
    return mask;
}

/*
 * SBAS Info UBX-NAV-SBAS
 * in u-blox 4_
 * in NEO-M9N
 * Not in some u-blox 9
 * Decode looks good, but data only goes to log.
 */
static gps_mask_t
ubx_msg_nav_sbas(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    unsigned i, cnt;
    unsigned ubx_PRN;
    short nmea_PRN;
    unsigned char gnssid = 0;
    unsigned char svid = 0;

    if (12 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-SBAS message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    ubx_PRN = getub(buf, 4);
    cnt = getub(buf, 8);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-NAV-SBAS iTOW %lu geo %u mode %u sys %u service x%x "
             "cnt %u\n",
             (unsigned long)session->driver.ubx.iTOW,
             ubx_PRN, (unsigned)getub(buf, 5),
             (unsigned)getub(buf, 6), (unsigned)getub(buf, 7),
             cnt);

    if (MAXCHANNELS < cnt) {
        // too many sats for us, pacify coverity
        cnt = MAXCHANNELS;
    }
    if (data_len < (12 + (12 * cnt))) {
        // length check, pacify coverity
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-NAV-SBAS message, bad message length %zd", data_len);
    }
    for (i = 0; i < cnt; i++) {
        int off = 12 + (12 * i);
        unsigned svID = getub(buf, off);
        unsigned flags = getub(buf, off + 1);
        // User Differential Range Error (udre)
        unsigned udre = getub(buf, off + 2);
        int svSys = getsb(buf, off + 3);
        unsigned svService = getub(buf, off + 4);
        int prc = getles16(buf, off + 6);
        int ic = getles16(buf, off + 10);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-NAV-SBAS SV%3u flags x%02x udre %u svSys %2d "
                 "svService x%x prc %d ic %d\n",
                 svID, flags, udre, svSys, svService, prc, ic);
    }
    /* really 'in_use' depends on the sats info, EGNOS is still
     * in test.  In WAAS areas one might also check for the type of
     * corrections indicated
     */

    nmea_PRN = ubx_to_prn(ubx_PRN, &gnssid, &svid);
#ifdef __UNUSED
    // debug
    GPSD_LOG(LOG_ERROR, &session->context->errout,
             "UBX-NAV-SBAS ubx_prn %d gnssid %d, svid %d nmea_PRN %d\n",
             ubx_PRN, gnssid, svid, nmea_PRN);
#endif  // __UNUSED
    session->driver.ubx.sbas_in_use = nmea_PRN;
    return 0;
}

/*
 * Multi-GNSS Raw measurement Data -- UBX-RXM-RAWX
 * Not in u-blox 5, 6 or 7
 * u-blox 9, message version 0 (but no version byte!)
 * u-blox 9, message version 1
 */
static gps_mask_t ubx_msg_rxm_rawx(struct gps_device_t *session,
                                   const unsigned char *buf,
                                   size_t data_len)
{
    double rcvTow;
    uint16_t week;
    int8_t leapS;
    uint8_t numMeas;
    uint8_t recStat;
    uint8_t version;
    int i;
    const char * obs_code;
    timespec_t ts_tow;

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-RXM-RAWX message, runt payload len %zd", data_len);
        return 0;
    }

    /* Note: this is "approximately" GPS TOW, this is not iTOW */
    rcvTow = getled64((const char *)buf, 0);   /* time of week in seconds */
    week = getleu16(buf, 8);
    leapS = getsb(buf, 10);
    numMeas = getub(buf, 11);
    recStat = getub(buf, 12);
    /* byte 13 is version on u-blox 9, reserved on u-blox 8
     * how is that supposed to work?? */
    version = getub(buf, 13);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-RXM-RAWX: rcvTow %f week %u leapS %d numMeas %u recStat %d"
             " version %u\n",
             rcvTow, week, leapS, numMeas, recStat, version);

    if (recStat & 1) {
        /* Valid leap seconds */
        session->context->leap_seconds = leapS;
        session->context->valid |= LEAP_SECOND_VALID;
    }
    /* convert GPS weeks and "approximately" GPS TOW to UTC */
    DTOTS(&ts_tow, rcvTow);
    // Do not set newdata.time.  set gpsdata.raw.mtime
    session->gpsdata.raw.mtime = gpsd_gpstime_resolv(session, week, ts_tow);

    /* zero the measurement data */
    /* so we can tell which meas never got set */
    memset(session->gpsdata.raw.meas, 0, sizeof(session->gpsdata.raw.meas));

    if (numMeas > MAXCHANNELS) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-RXM-RAWX message, too many measurements (%u)",
                 numMeas);
        return 0;
    }
    for (i = 0; i < numMeas; i++) {
        int off = 32 * i;
        /* pseudorange in meters */
        double prMes = getled64((const char *)buf, off + 16);
        /* carrier phase in cycles */
        double cpMes = getled64((const char *)buf, off + 24);
        /* doppler in Hz, positive towards sat */
        double doMes = getlef32((const char *)buf, off + 32);
        uint8_t gnssId = getub(buf, off + 36);
        uint8_t svId = getub(buf, off + 37);
        // reserved in u-blox 8, sigId in u-blox 9 (version 1)
        uint8_t sigId = getub(buf, off + 38);
        /* GLONASS frequency slot */
        uint8_t freqId = getub(buf, off + 39);
        /* carrier phase locktime in ms, max 64500ms */
        uint16_t locktime = getleu16(buf, off + 40);
        /* carrier-to-noise density ratio dB-Hz */
        uint8_t cno = getub(buf, off + 42);
        uint8_t prStdev = getub(buf, off + 43) & 0x0f;
        uint8_t cpStdev = getub(buf, off + 44) & 0x0f;
        uint8_t doStdev = getub(buf, off + 45) & 0x0f;
        /* tracking stat
         * bit 0 - prMes valid
         * bit 1 - cpMes valid
         * bit 2 - halfCycle valid
         * bit 3 - halfCycle subtracted from phase
         */
        uint8_t trkStat = getub(buf, off + 46);
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "%u:%u:%u freqId %u prMes %f cpMes %f doMes %f locktime %u\n"
                 "cno %u prStdev %u cpStdev %u doStdev %u rtkStat %u\n",
                 gnssId, svId, sigId, freqId, prMes, cpMes, doMes, locktime,
                 cno, prStdev, cpStdev, doStdev, trkStat);

        session->gpsdata.raw.meas[i].gnssid = gnssId;
        session->gpsdata.raw.meas[i].sigid = sigId;

        /* some of these are GUESSES as the u-blox codes do not
         * match RINEX codes */
        switch (gnssId) {
        case 0:       /* GPS */
            switch (sigId) {
            default:
                /* let PPP figure it out */
                FALLTHROUGH
            case 0:       /* L1C/A */
                obs_code = "L1C";
                break;
            case 3:       /* L2 CL */
                obs_code = "L2C";
                break;
            case 4:       /* L2 CM */
                obs_code = "L2X";
                break;
            }
            break;
        case 1:       /* SBAS */
            /* sigId added on protVer 27, and SBAS gone in protVer 27
             * so must be L1C/A */
            svId -= 100;            /* adjust for RINEX 3 svid */

            obs_code = "L1C";       /* u-blox calls this L1C/A */
            /* SBAS can do L5I, but the code? */
            switch (sigId) {
            default:
                /* let PPP figure it out */
                break;
            case 0:       /* L1C/A */
                obs_code = "L1C";
                break;
            }
            break;
        case 2:       /* GALILEO */
            switch (sigId) {
            default:
                /* let PPP figure it out */
                FALLTHROUGH
            case 0:       /*  */
                obs_code = "L1C";       /* u-blox calls this E1OS or E1C */
                break;
            case 1:       /*  */
                obs_code = "L1B";       /* u-blox calls this E1B */
                break;
            case 5:       /*  */
                obs_code = "L7I";       /* u-blox calls this E5bl */
                break;
            case 6:       /*  */
                obs_code = "L7Q";       /* u-blox calls this E5bQ */
                break;
            }
            break;
        case 3:       /* BeiDou */
            switch (sigId) {
            default:
                /* let PPP figure it out */
                FALLTHROUGH
            case 0:       /*  */
                obs_code = "L2Q";       /* u-blox calls this B1I D1 */
                break;
            case 1:       /*  */
                obs_code = "L2I";       /* u-blox calls this B1I D2 */
                break;
            case 2:       /*  */
                obs_code = "L7Q";       /* u-blox calls this B2I D1 */
                break;
            case 3:       /*  */
                obs_code = "L7I";       /* u-blox calls this B2I D2 */
                break;
            }
            break;
        default:      /* huh? */
        case 4:       /* IMES.  really? */
            obs_code = "";       /* u-blox calls this L1 */
            break;
        case 5:       /* QZSS */
            switch (sigId) {
            default:
                /* let PPP figure it out */
                FALLTHROUGH
            case 0:       /*  */
                obs_code = "L1C";       /* u-blox calls this L1C/A */
                break;
            case 4:       /*  */
                obs_code = "L2S";       /* u-blox calls this L2CM */
                break;
            case 5:       /*  */
                obs_code = "L2L";       /* u-blox calls this L2CL*/
                break;
            }
            break;
        case 6:       /* GLONASS */
            switch (sigId) {
            default:
                /* let PPP figure it out */
                FALLTHROUGH
            case 0:       /*  */
                obs_code = "L1C";       /* u-blox calls this L1OF */
                break;
            case 2:       /*  */
                obs_code = "L2C";       /* u-blox calls this L2OF */
                break;
            }
            break;
        }
        (void)strlcpy(session->gpsdata.raw.meas[i].obs_code, obs_code,
                      sizeof(session->gpsdata.raw.meas[i].obs_code));

        session->gpsdata.raw.meas[i].svid = svId;
        session->gpsdata.raw.meas[i].freqid = freqId;
        session->gpsdata.raw.meas[i].snr = cno;
        session->gpsdata.raw.meas[i].satstat = trkStat;
        if (trkStat & 1) {
            /* prMes valid */
            session->gpsdata.raw.meas[i].pseudorange = prMes;
        } else {
            session->gpsdata.raw.meas[i].pseudorange = NAN;
        }
        if ((trkStat & 2) && (5 >= cpStdev)) {
            /* cpMes valid, RTKLIB uses 5 < cpStdev */
            session->gpsdata.raw.meas[i].carrierphase = cpMes;
        } else {
            session->gpsdata.raw.meas[i].carrierphase = NAN;
        }
        session->gpsdata.raw.meas[i].doppler = doMes;
        session->gpsdata.raw.meas[i].codephase = NAN;
        session->gpsdata.raw.meas[i].deltarange = NAN;
        session->gpsdata.raw.meas[i].locktime = locktime;
        if (0 == locktime) {
            /* possible slip */
            session->gpsdata.raw.meas[i].lli = 2;
        }
    }

    return RAW_IS;
}

/*
 * Raw Subframes - UBX-RXM-SFRB
 * In u-blox 7, only in raw firmware option
 * Not in u-blox 8 or 9
 */
static gps_mask_t ubx_msg_rxm_sfrb(struct gps_device_t *session,
                                   unsigned char *buf, size_t data_len)
{
    unsigned int i, chan, svid;
    uint32_t words[10];

    if (42 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-RXM-SFRB message, runt payload len %zd", data_len);
        return 0;
    }

    chan = (unsigned int)getub(buf, 0);
    svid = (unsigned int)getub(buf, 1);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-RXM-SFRB: %u %u\n", chan, svid);

    /* UBX does all the parity checking, but still bad data gets through */
    for (i = 0; i < 10; i++) {
        // bits 24 to 31 undefined, remove them.
        words[i] = (uint32_t)getleu32(buf, 4 * i + 2) & 0x00ffffff;
    }

    // probably GPS, could be SBAS
    return gpsd_interpret_subframe(session, GNSSID_GPS, svid, words);
}

/*
 * Raw Subframes - UBX-RXM-SFRBX
 * in u-blox 8, protver 17 and up, time sync firmware only
 * in u-blox F9P abd HPG only
 * not present  before u-blox8
 */
static gps_mask_t ubx_msg_rxm_sfrbx(struct gps_device_t *session,
                                    unsigned char *buf, size_t data_len)
{
    unsigned i;
    uint8_t gnssId, svId, freqId, numWords, chn, version;
    uint32_t words[17];
    char *chn_s;

    if (8 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-RXM-SFRBX message, runt payload len %zd", data_len);
        return 0;
    }

    numWords = getub(buf, 4);
    if (data_len != (size_t)(8 + (4 * numWords)) ||
        16 < numWords) {
        // test numwords directly to shut up Coverity
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-RXM-SFRBX message, wrong payload len %zd, numwords %u "
                 "s/b %u",
                 data_len, 8 + (4 * numWords), numWords);
        return 0;
    }

    gnssId = getub(buf, 0);
    svId = getub(buf, 1);
    freqId = getub(buf, 2);
    version = getub(buf, 6);
    chn = getub(buf, 5);
    if (1 < version) {
        // receiver channel in version 2 and up.
        // valid range 0 to 13?
        chn_s = "chn";
    } else {
        chn_s = "reserved";
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX-RXM-SFRBX: version %u gnssId %u %s %u svId %u "
             "freqId %u words %u\n",
             version, gnssId, chn_s, chn, svId, freqId, numWords);

    if (0 == version) {
        // unknown ersion
        return 0;
    }

    memset(words, 0, sizeof(words));
    for (i = 0; i < numWords; i++) {
        // grab the words, don't mangle them
        words[i] = (uint32_t)getleu32(buf, 4 * i + 8);
    }

    // do we need freqId or chn?
    return gpsd_interpret_subframe_raw(session, gnssId, svId, words, numWords);
}

/**
 * SV Status Info
 *
 * May be good cycle ender
 *
 * Present in u-blox 7
 */
static gps_mask_t
ubx_msg_rxm_svsi(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    unsigned numVis, numSV;

    if (8 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-RXM-SVSI message, runt payload len %zd", data_len);
        return 0;
    }

    session->driver.ubx.iTOW = getleu32(buf, 0);
    session->context->gps_week = getleu16(buf, 4);
    numVis = getub(buf, 6);
    numSV = getub(buf, 7);
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NAV-CLOCK: iTOW=%lld week %d numVis %u numSV %u\n",
             (long long)session->driver.ubx.iTOW,
            session->context->gps_week, numVis, numSV);
    return 0;
}

/* UBX-INF-* */
static gps_mask_t
ubx_msg_inf(struct gps_device_t *session, unsigned char *buf, size_t data_len)
{
    unsigned short msgid;
    static char txtbuf[MAX_PACKET_LENGTH];

    /* No minimum payload length */

    msgid = (unsigned short)((buf[2] << 8) | buf[3]);
    if (data_len > MAX_PACKET_LENGTH - 1)
        data_len = MAX_PACKET_LENGTH - 1;

    (void)strlcpy(txtbuf, (char *)buf + UBX_PREFIX_LEN, sizeof(txtbuf));
    txtbuf[data_len] = '\0';
    switch (msgid) {
    case UBX_INF_DEBUG:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-INF-DEBUG: %s\n",
                 txtbuf);
        break;
    case UBX_INF_TEST:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-INF-TEST: %s\n",
                 txtbuf);
        break;
    case UBX_INF_NOTICE:
        GPSD_LOG(LOG_INF, &session->context->errout, "UBX-INF-NOTICE: %s\n",
                 txtbuf);
        break;
    case UBX_INF_WARNING:
        GPSD_LOG(LOG_WARN, &session->context->errout, "UBX-INF-WARNING: %s\n",
                 txtbuf);
        break;
    case UBX_INF_ERROR:
        GPSD_LOG(LOG_WARN, &session->context->errout, "UBX-INF-ERROR: %s\n",
                 txtbuf);
        break;
    default:
        break;
    }
    return 0;
}

/**
 * Survey-in data - UBX-TIM-SVIN
 * Time Sync products only
 */
static gps_mask_t
ubx_msg_tim_svin(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    gps_mask_t mask = ONLINE_SET;
    uint32_t dur;
    int32_t meanX;
    int32_t meanY;
    int32_t meanZ;
    uint32_t meanV;
    uint32_t obs;
    uint8_t valid;
    uint8_t active;

    if (28 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-TIM-SVIN message, runt payload len %zd", data_len);
        return 0;
    }

    dur = getleu32(buf, 0);
    meanX = getles32(buf, 4);
    meanY = getles32(buf, 8);
    meanZ = getles32(buf, 12);
    meanV = getleu32(buf, 16);
    obs = getleu32(buf, 20);
    valid = getub(buf, 24);
    active = getub(buf, 25);
    // two reserved bytes

    /* casts for 32 bit compatibility */
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "TIM-SVIN: dur=%lu meanX=%ld meanY=%ld meanZ=%ld meanV=%lu "
             "obs=%lu valid=%u active=%u\n",
             (unsigned long)dur, (long)meanX, (long)meanY, (long)meanZ,
             (long)meanV, (unsigned long)obs, valid, active);
    return mask;
}

/**
 * Time Pulse Timedata - UBX-TIM-TP
 */
static gps_mask_t
ubx_msg_tim_tp(struct gps_device_t *session, unsigned char *buf,
               size_t data_len)
{
    gps_mask_t mask = ONLINE_SET;
    uint32_t towMS;
    uint32_t towSubMS;
    int32_t qErr;
    uint16_t week;
    uint8_t flags;
    uint8_t refInfo;
    timespec_t ts_tow;

    if (16 > data_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX-TIM-TP message, runt payload len %zd", data_len);
        return 0;
    }

    towMS = getleu32(buf, 0);
    // towSubMS always seems zero, which will match the PPS
    towSubMS = getleu32(buf, 4);
    qErr = getles32(buf, 8);
    week = getleu16(buf, 12);
    flags = buf[14];
    refInfo = buf[15];

    /* are we UTC, and towSubMs is zero? */
    if (3 == (flags & 0x03) &&
        0 == towSubMS) {

        // leap already added!?!?
        int saved_leap = session->context->leap_seconds;
        // remove it!
        session->context->leap_seconds = 0;

        /* good, save qErr and qErr_time */
        session->gpsdata.qErr = qErr;
        MSTOTS(&ts_tow, towMS);
        session->gpsdata.qErr_time = gpsd_gpstime_resolv(session, week, ts_tow);

        // restore leap
        session->context->leap_seconds = saved_leap;

#ifdef __UNUSED
        {
         struct gps_device_t *ppsonly;
         // FIXME!! should be up a layer so other drivers can use it
         // FIXME!! this qErr can only apply to one PPS!
         /* propagate this in-band-time to all PPS-only devices */
         for (ppsonly = devices; ppsonly < devices + MAX_DEVICES; ppsonly++)
             if (SOURCE_PPS == ppsonly->sourcetype) {
                 pps_thread_qErrin(&ppsonly->pps_thread, qErr,
                                   session->gpsdata.qErr_time);
             }
        }
#endif  // __UNUSED

    }

    /* cast for 32 bit compatibility */
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "TIM-TP: towMS %lu, towSubMS %lu, qErr %ld week %u "
             "flags %#x, refInfo %#x\n",
             (unsigned long)towMS, (unsigned long)towSubMS, (long)qErr,
              week, flags, refInfo);
    return mask;
}

gps_mask_t ubx_parse(struct gps_device_t * session, unsigned char *buf,
                     size_t len)
{
    size_t data_len;
    unsigned short msgid;
    gps_mask_t mask = 0;

    // the packet at least contains a head long enough for an empty message
    if (UBX_PREFIX_LEN > len) {
        return 0;
    }

    session->cycle_end_reliable = true;
    session->driver.ubx.iTOW = -1;        // set by decoder

    // extract message id and length
    msgid = (buf[2] << 8) | buf[3];
    data_len = (size_t) getles16(buf, 4);

    switch (msgid) {
    case UBX_ACK_ACK:
        if (2 <= data_len) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "UBX-ACK-ACK, class: %02x, id: %02x\n",
                     buf[UBX_PREFIX_LEN],
                     buf[UBX_PREFIX_LEN + 1]);
        }
        break;
    case UBX_ACK_NAK:
        if (2 <= data_len) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "UBX-ACK-NAK, class: %02x, id: %02x\n",
                     buf[UBX_PREFIX_LEN],
                     buf[UBX_PREFIX_LEN + 1]);
        }
        break;

    case UBX_CFG_NAV5:
        // deprecated in u-blox 10
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-CFG-NAV5\n");
        break;
    case UBX_CFG_NAVX5:
        // deprecated in u-blox 10
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-CFG-NAVX5\n");
        break;
    case UBX_CFG_PRT:
        // deprecated in u-blox 10
        if (session->driver.ubx.port_id != buf[UBX_PREFIX_LEN + 0] ) {
            session->driver.ubx.port_id = buf[UBX_PREFIX_LEN + 0];
            GPSD_LOG(LOG_INF, &session->context->errout,
                     "UBX-CFG-PRT: port %d\n", session->driver.ubx.port_id);
        }
        break;
    case UBX_CFG_RATE:
        // deprecated in u-blox 10
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-CFG-RATE\n");
        ubx_msg_cfg_rate(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_ESF_ALG:
        mask = ubx_msg_esf_alg(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_ESF_INS:
        mask = ubx_msg_esf_ins(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_ESF_MEAS:
        mask = ubx_msg_esf_meas(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_ESF_RAW:
        mask = ubx_msg_esf_raw(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_ESF_STATUS:
        mask = ubx_msg_esf_status(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_HNR_ATT:
        mask = ubx_msg_hnr_att(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_HNR_INS:
        mask = ubx_msg_hnr_ins(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_HNR_PVT:
        mask = ubx_msg_hnr_pvt(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_INF_DEBUG:
        FALLTHROUGH
    case UBX_INF_ERROR:
        FALLTHROUGH
    case UBX_INF_NOTICE:
        FALLTHROUGH
    case UBX_INF_TEST:
        FALLTHROUGH
    case UBX_INF_USER:
        FALLTHROUGH
    case UBX_INF_WARNING:
        mask = ubx_msg_inf(session, buf, data_len);
        break;

    case UBX_LOG_BATCH:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-LOG-BATCH\n");
        mask = ubx_msg_log_batch(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_LOG_INFO:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-LOG-INFO\n");
        mask = ubx_msg_log_info(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_LOG_RETRIEVEPOS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-LOG-RETRIEVEPOS\n");
        mask = ubx_msg_log_retrievepos(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_LOG_RETRIEVEPOSEXTRA:
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-LOG-RETRIEVEPOSEXTRA\n");
        mask = ubx_msg_log_retrieveposextra(session, &buf[UBX_PREFIX_LEN],
                                            data_len);
        break;
    case UBX_LOG_RETRIEVESTRING:
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX-LOG-RETRIEVESTRING\n");
        mask = ubx_msg_log_retrievestring(session, &buf[UBX_PREFIX_LEN],
                                          data_len);
        break;

    case UBX_MON_BATCH:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-BATCH\n");
        break;
    case UBX_MON_EXCEPT:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-EXCEPT\n");
        break;
    case UBX_MON_GNSS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-GNSS\n");
        break;
    case UBX_MON_HW:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-HW\n");
        break;
    case UBX_MON_HW2:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-HW2\n");
        break;
    case UBX_MON_HW3:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-HW3\n");
        break;
    case UBX_MON_IO:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-IO\n");
        break;
    case UBX_MON_IPC:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-IPC\n");
        break;
    case UBX_MON_MSGPP:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-MSGPP\n");
        break;
    case UBX_MON_PATCH:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-PATCH\n");
        break;
    case UBX_MON_RF:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-RF\n");
        break;
    case UBX_MON_RXBUF:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-RXBUF\n");
        ubx_msg_mon_rxbuf(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_MON_RXR:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-RXR\n");
        break;
    case UBX_MON_SCHED:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-SCHED\n");
        break;
    case UBX_MON_SMGR:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-SMGR\n");
        break;
    case UBX_MON_SPAN:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-SPAN\n");
        break;
    case UBX_MON_TXBUF:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-TXBUF\n");
        ubx_msg_mon_txbuf(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_MON_USB:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-USB\n");
        break;
    case UBX_MON_VER:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MON-VER\n");
        mask = ubx_msg_mon_ver(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_NAV_AOPSTATUS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-AOPSTATUS\n");
        break;
    case UBX_NAV_ATT:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-ATT\n");
        break;
    case UBX_NAV_CLOCK:
        mask = ubx_msg_nav_clock(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_DGPS:
        mask = ubx_msg_nav_dgps(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_DOP:
        // DOP seems to be the last NAV sent in a cycle, unless NAV-EOE
        mask = ubx_msg_nav_dop(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_EELL:
        mask = ubx_msg_nav_eell(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_EKFSTATUS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-EKFSTATUS\n");
        break;
    case UBX_NAV_EOE:
        mask = ubx_msg_nav_eoe(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_GEOFENCE:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-GEOFENCE\n");
        break;
    case UBX_NAV_HPPOSECEF:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-HPPOSECEF\n");
        mask = ubx_msg_nav_hpposecef(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_HPPOSLLH:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-HPPOSLLH\n");
        mask = ubx_msg_nav_hpposllh(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_ODO:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-ODO\n");
        break;
    case UBX_NAV_ORB:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-ORB\n");
        break;
    case UBX_NAV_POSECEF:
        mask = ubx_msg_nav_posecef(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_POSLLH:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-POSLLH\n");
        mask = ubx_msg_nav_posllh(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_POSUTM:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-POSUTM\n");
        break;
    case UBX_NAV_PVT:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-PVT\n");
        mask = ubx_msg_nav_pvt(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_RELPOSNED:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-RELPOSNED\n");
        mask = ubx_msg_nav_relposned(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_RESETODO:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-RESETODO\n");
        break;
    case UBX_NAV_SAT:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-SAT\n");
        mask = ubx_msg_nav_sat(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SBAS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-SBAS\n");
        mask = ubx_msg_nav_sbas(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SIG:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-SIG\n");
        break;
    case UBX_NAV_SOL:
        /* UBX-NAV-SOL deprecated in u-blox 6, gone in u-blox 9 and 10.
         * Use UBX-NAV-PVT instead */
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-SOL\n");
        mask = ubx_msg_nav_sol(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_STATUS:
        mask = ubx_msg_nav_status(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SVIN:
        mask = ubx_msg_tim_svin(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SVINFO:
        // UBX-NAV-SVINFO deprecated, use UBX-NAV-SAT instead
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-SVINFO\n");
        mask = ubx_msg_nav_svinfo(session, &buf[UBX_PREFIX_LEN], data_len);

        /* this is a hack to move some initialization until after we
         * get some u-blox message so we know the GPS is alive */
        if ('\0' == session->subtype[0]) {
            // one time only
            (void)strlcpy(session->subtype, "Unknown", 8);
            // request SW and HW Versions
            (void)ubx_write(session, UBX_CLASS_MON, 0x04, NULL, 0);
        }

        break;
    case UBX_NAV_TIMEBDS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-TIMEBDS\n");
        break;
    case UBX_NAV_TIMEGAL:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-TIMEGAL\n");
        break;
    case UBX_NAV_TIMEGLO:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-TIMEGLO\n");
        break;
    case UBX_NAV_TIMEGPS:
        mask = ubx_msg_nav_timegps(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_TIMELS:
        mask = ubx_msg_nav_timels(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_TIMEQZSS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-TIMEQZSS\n");
        break;
    case UBX_NAV_TIMEUTC:
        mask = ubx_msg_nav_timeutc(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_VELECEF:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-VELECEF\n");
        mask = ubx_msg_nav_velecef(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_VELNED:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-NAV-VELNED\n");
        mask = ubx_msg_nav_velned(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_MGA_ACK:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MGA-ACK\n");
        break;
    case UBX_MGA_DBD:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-MGA-DBD\n");
        break;

    case UBX_RXM_ALM:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-ALM\n");
        break;
    case UBX_RXM_EPH:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-EPH\n");
        break;
    case UBX_RXM_IMES:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-IMES\n");
        break;
    case UBX_RXM_MEASX:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-MEASX\n");
        break;
    case UBX_RXM_PMREQ:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-PMREQ\n");
        break;
    case UBX_RXM_POSREQ:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-POSREQ\n");
        break;
    case UBX_RXM_RAW:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-RAW\n");
        break;
    case UBX_RXM_RAWX:
        mask = ubx_msg_rxm_rawx(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_RLM:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-RLM\n");
        break;
    case UBX_RXM_RTCM:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-RXM-RTCM\n");
        break;
    case UBX_RXM_SFRB:
        mask = ubx_msg_rxm_sfrb(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_SFRBX:
        mask = ubx_msg_rxm_sfrbx(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_SVSI:
        // Gone in u-blox 10, use UBX-NAV-ORB instead
        mask = ubx_msg_rxm_svsi(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    // undocumented
    // case UBX_SEC_SESSID:
    //     GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-SEC-SESSID\n");
    //     break;
    case UBX_SEC_SIGN:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX_SEC_SIGN\n");
        break;
    case UBX_SEC_UNIQID:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX_SEC_UNIQID\n");
        break;

    case UBX_TIM_DOSC:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-DOSC\n");
        break;
    case UBX_TIM_FCHG:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-FCHG\n");
        break;
    case UBX_TIM_HOC:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-HOC\n");
        break;
    case UBX_TIM_SMEAS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-SMEAS\n");
        break;
    case UBX_TIM_SVIN:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-SVIN\n");
        break;
    case UBX_TIM_TM:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-TM\n");
        break;
    case UBX_TIM_TM2:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-TM2\n");
        break;
    case UBX_TIM_TP:
        mask = ubx_msg_tim_tp(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_TIM_TOS:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-TOS\n");
        break;
    case UBX_TIM_VCOCAL:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-VCOCAL\n");
        break;
    case UBX_TIM_VRFY:
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX-TIM-VRFY\n");
        break;

    default:
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "UBX: unknown packet id x%04hx (length %zd)\n",
                 msgid, len);
    }
#ifdef __UNUSED
    // debug
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX: msgid x%04x end x%04x last x%04x iTOW %lld last %lld\n",
             msgid,
             session->driver.ubx.end_msgid,
             session->driver.ubx.last_msgid,
             (long long)session->driver.ubx.iTOW,
             (long long)session->driver.ubx.last_iTOW);
#endif

    // iTOW drives the cycle start/end detection
    // iTOW is in ms, can go forward or backward
    if (-1 < session->driver.ubx.iTOW) {
        int64_t iTOW_diff;

        // this sentence has a (maybe good) time
        // end of cycle ?
        if (session->driver.ubx.end_msgid == msgid) {
            // got known cycle ender.  Assume end of cycle, report it
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "UBX: cycle end x%04x iTOW %lld\n",
                     msgid, (long long)session->driver.ubx.iTOW);
            mask |= REPORT_IS;
        }

        // start of cycle?  Start can equal end if only one message per epoch
        // u-blox iTOW can have ms jitter in the same epoch!
        iTOW_diff = session->driver.ubx.last_iTOW - session->driver.ubx.iTOW;
        if (10 < llabs(iTOW_diff)) {
            // time changed more than 10 ms (100 Hz), cycle start

            if (session->driver.ubx.end_msgid !=
                session->driver.ubx.last_msgid) {
                // new cycle ender
                GPSD_LOG(LOG_PROG, &session->context->errout,
                         "UBX: new ender x%04x was x%04x iTOW %lld was %lld\n",
                         session->driver.ubx.last_msgid,
                         session->driver.ubx.end_msgid,
                         (long long)session->driver.ubx.iTOW,
                         (long long)session->driver.ubx.last_iTOW);
                session->driver.ubx.end_msgid = session->driver.ubx.last_msgid;
            }
            session->driver.ubx.last_iTOW = session->driver.ubx.iTOW;
            mask |= CLEAR_IS;;
        }

        session->driver.ubx.last_msgid = msgid;
        // FIXME: last_time never used...
        session->driver.ubx.last_time = session->newdata.time;
    } else {
        // no time
        /* debug
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "UBX: No time, msgid %x\n", msgid);
         */
    }

    // Did protver change?
    if (session->driver.ubx.last_protver != session->driver.ubx.protver) {
        /* Assumption: we just did init, but did not have
         * protver then, so init is not complete.  Finish now.
         * unless user requested passive mode */
        if (session->mode == O_OPTIMIZE &&
            !session->context->passive) {
            ubx_mode(session, MODE_BINARY);
        }
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "UBX: new PROTVER %u was %u\n",
                 session->driver.ubx.protver,
                 session->driver.ubx.last_protver);
        session->driver.ubx.last_protver = session->driver.ubx.protver;
    }

    return mask | ONLINE_SET;
}

static gps_mask_t parse_input(struct gps_device_t *session)
{
    if (UBX_PACKET == session->lexer.type) {
        return ubx_parse(session, session->lexer.outbuffer,
                         session->lexer.outbuflen);
    }
    return generic_parse_input(session);
}

bool ubx_write(struct gps_device_t * session,
               unsigned int msg_class, unsigned int msg_id,
               const unsigned char *msg, size_t data_len)
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

    session->msgbuf[0] = 0xb5;
    session->msgbuf[1] = 0x62;

    CK_A = CK_B = 0;
    session->msgbuf[2] = msg_class;
    session->msgbuf[3] = msg_id;
    session->msgbuf[4] = data_len & 0xff;
    session->msgbuf[5] = (data_len >> 8) & 0xff;

    assert(msg != NULL || data_len == 0);
    if (msg != NULL)
        (void)memcpy(&session->msgbuf[6], msg, data_len);

    // calculate CRC
    for (i = 2; i < 6; i++) {
        CK_A += session->msgbuf[i];
        CK_B += CK_A;
    }
    if (NULL != msg)
        for (i = 0; i < data_len; i++) {
            CK_A += msg[i];
            CK_B += CK_A;
        }

    session->msgbuf[6 + data_len] = CK_A;
    session->msgbuf[7 + data_len] = CK_B;
    session->msgbuflen = data_len + 8;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "=> GPS: UBX class: %02x, id: %02x, len: %zd, crc: %02x%02x\n",
             msg_class, msg_id, data_len,
             CK_A, CK_B);
    count = gpsd_write(session, session->msgbuf, session->msgbuflen);
    ok = (count == (ssize_t) session->msgbuflen);
    return (ok);
}

// not used by gpsd, it's for gpsctl and friends
static ssize_t ubx_control_send(struct gps_device_t *session, char *msg,
                                size_t data_len)
{
    return ubx_write(session, (unsigned int)msg[0], (unsigned int)msg[1],
                     (unsigned char *)msg + 2,
                     (size_t)(data_len - 2)) ? ((ssize_t) (data_len + 7)) : -1;
}

static void ubx_init_query(struct gps_device_t *session)
{
    // UBX-MON-VER: query for version information
    (void)ubx_write(session, UBX_CLASS_MON, 0x04, NULL, 0);
}

static void ubx_event_hook(struct gps_device_t *session, event_t event)
{
    if (session->context->readonly ||
        session->context->passive) {
        return;
    }
    if (event == event_identified) {
        GPSD_LOG(LOG_PROG, &session->context->errout, "UBX identified\n");

        // no longer set UBX-CFG-SBAS here, u-blox 9 and 10 do not have it

        /*
         * Turn off NMEA output, turn on UBX on this port.
         */
        if (session->context->passive) {
            // passive mode, do no autoconfig
        } else if (session->mode == O_OPTIMIZE) {
            ubx_mode(session, MODE_BINARY);
        } else {
            ubx_mode(session, MODE_NMEA);
        }
    } else if (event == event_deactivate) {
        /* There used to be a hotstart/reset here.
         * That caused u-blox USB to re-enumerate.
         * Sometimes to a new device name.
         * Bad.  Don't do that anymore...
         */
    }
}

// generate and send a configuration block
static gps_mask_t ubx_cfg_prt(struct gps_device_t *session, speed_t speed,
                              const char parity,
                              const int stopbits, const int mode)
{
    unsigned long usart_mode = 0;
    unsigned char buf[UBX_CFG_LEN];
    unsigned long i;

    memset(buf, '\0', UBX_CFG_LEN);

    /*
     * When this is called from gpsd, the initial probe for UBX should
     * have picked up the device's port number from the CFG_PRT response.
     */
    // FIXME!  Bad test, port_id == 0 is valid too.  DDC (I2X) = port 0
    if (session->driver.ubx.port_id != 0) {
        buf[0] = session->driver.ubx.port_id;
    }
    /*
     * This default can be hit if we haven't sent a CFG_PRT query yet,
     * which can happen in gpsmon because it doesn't autoprobe.
     *
     * What we'd like to do here is dispatch to USART1_ID or
     * USB_ID intelligently based on whether this is a USB or RS232
     * source.  Unfortunately the GR601-W screws that up by being
     * a USB device with port_id 1.  So we bite the bullet and
     * default to port 1.
     *
     * Without further logic, this means gpsmon wouldn't be able to
     * change the speed on the EVK 6H's USB port.  But! To pick off
     * the EVK 6H on Linux as a special case, we notice that its
     * USB device name is /dev/ttyACMx - it presents as a USB modem.
     *
     * This logic will fail on any USB u-blox device that presents
     * as an ordinary USB serial device (/dev/ttyUSB*) and actually
     * has port ID 3 the way it "ought" to.
     */
    else if (strstr(session->gpsdata.dev.path, "/ttyACM") != NULL) {
        /* using the built in USB port */
        // FIXME!!  USB port has no speed!
        // FIXME!!  maybe we know the portid already?
        session->driver.ubx.port_id = buf[0] = USB_ID;
    } else {
        // A guess.  Could be UART2, or SPI, or DDC port
        session->driver.ubx.port_id = buf[0] = USART1_ID;
    }

    putle32(buf, 8, speed);

    /*
     * u-blox tech support explains the default contents of the mode
     * field as follows:
     *
     * D0 08 00 00     mode (LSB first)
     *
     * re-ordering bytes: 000008D0
     * dividing into fields: 000000000000000000 00 100 0 11 0 1 0000
     * nStopbits = 00 = 1
     * parity = 100 = none
     * charLen = 11 = 8-bit
     * reserved1 = 1
     *
     * The protocol reference further gives the following subfield values:
     * 01 = 1.5 stop bits (?)
     * 10 = 2 stopbits
     * 000 = even parity
     * 001 = odd parity
     * 10x = no parity
     * 10 = 7 bits
     *
     * Some UBX reference code amplifies this with:
     *
     *   prtcfg.mode = (1<<4) | // compatibility with ANTARIS 4
     *                 (1<<7) | // charLen = 11 = 8 bit
     *                 (1<<6) | // charLen = 11 = 8 bit
     *                 (1<<11); // parity = 10x = none
     */
    usart_mode |= (1<<4);       // reserved1 Antaris 4 compatibility bit
    usart_mode |= (1<<7);       // high bit of charLen

    // u-blox 5+ binary only supports 8N1
    switch (parity) {
    case (int)'E':
    case 2:
        usart_mode |= (1<<7);           // 7E
        break;
    case (int)'O':
    case 1:
        usart_mode |= (1<<9) | (1<<7);  // 7O
        break;
    case (int)'N':
    case 0:
    default:
        usart_mode |= (1<<11) | (3<<6); // 8N
        break;
    }

    if (2 == stopbits) {
        usart_mode |= (1<<13);
    }

    putle32(buf, 4, usart_mode);

    // enable all input protocols by default
    // RTCM3 is protver 20+
    buf[12] = NMEA_PROTOCOL_MASK | UBX_PROTOCOL_MASK | RTCM_PROTOCOL_MASK |
              RTCM3_PROTOCOL_MASK;

    /* enable all input protocols by default
     * no u-blox has RTCM2 out
     * RTCM3 is protver 20+ */
    buf[outProtoMask] = NMEA_PROTOCOL_MASK | UBX_PROTOCOL_MASK |
                        RTCM3_PROTOCOL_MASK;
    // FIXME: use VALGET if protver  24+
    (void)ubx_write(session, UBX_CLASS_CFG, 0x00, buf, sizeof(buf));

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX ubx_cfg_prt mode %d port %d PROTVER %d\n", mode, buf[0],
             session->driver.ubx.protver);

    /* selectively enable output protocols */
    if (mode == MODE_NMEA) {
        /*
         * We have to club the GR601-W over the head to make it stop emitting
         * UBX after we've told it to start.  But do not mung the
         * protocol out mask, that breaks things.
         */

        unsigned char msg[3];
        /* nmea to turn on at rate one (multiplier on measurement rate)
         * u-blox 8 default: RMC, VTG, GGA, GSA GSV, GLL
         * who wanted GST? */
        const unsigned char nmea_on[] = {
            0x00,          // msg id  = GGA
            // 0x01,          /* msg id  = GLL, only need RMC */
            0x02,          // msg id  = GSA
            0x03,          // msg id  = GSV
            0x04,          // msg id  = RMC
            0x05,          // msg id  = VTG
            0x07,          // msg id  = GST, GNSS pseudorange error statistics
            0x08,          // msg id  = ZDA, for UTC year
            0x09,          // msg id  = GBS, for RAIM errors
        };

        const unsigned char ubx_nav_off[] = {
            0x01,          // msg id = NAV-POSECEF
            0x04,          // msg id = UBX-NAV-DOP
            0x06,          // msg id = NAV-SOL, deprecated in 6, gone in 9
            0x07,          // msg id = NAV-PVT, in u-blox 6 and on
            0x11,          // msg id = NAV-VELECEF
            0x20,          // msg id = UBX-NAV-TIMEGPS
            // 0x26;       // msg id  = UBX-NAV-TIMELS, allow as low rate
            0x30,          // msg id = NAV-SVINFO, in 4 to 8, not 9
            0x32,          // msg id = NAV-SBAS, in u-blox 4 to 8, not all 9
            0x35,          // msg id = NAV-SAT, in u-blox 8 and 9
            0x61,          // msg id = NAV-EOE
        };

        // enable NMEA first, in case we over-run receiver input buffer.

        // turn on rate one NMEA
        msg[0] = 0xf0;          /* class, NMEA */
        msg[2] = 0x01;          /* rate, one */
        for (i = 0; i < sizeof(nmea_on); i++) {
            msg[1] = nmea_on[i];          // msg id to turn on
            (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);
        }

        // Now turn off UBX-NAV, one at a time.
        msg[0] = 0x01;          // class, UBX-NAV
        msg[2] = 0x00;          // rate off
        for (i = 0; i < sizeof(ubx_nav_off); i++) {
            msg[1] = ubx_nav_off[i];          // msg id to turn on
            (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);
        }

    } else {    // MODE_BINARY
        // nmea to turn off
        const unsigned char nmea_off[] = {
            0x00,          // msg id  = GGA
            0x01,          // msg id  = GLL
            0x02,          // msg id  = GSA
            0x03,          // msg id  = GSV
            0x04,          // msg id  = RMC
            0x05,          // msg id  = VTG
            0x07,          // msg id  = GST
            0x08,          // msg id  = ZDA
            0x09,          // msg id  = GBS
        };

        const unsigned char ubx_nav_on[] = {
            0x04,          // msg id = UBX-NAV-DOP
            // UBX-NAV-TIMEGPS is a great cycle ender, NAV-EOE better
            0x20,          // msg id = UBX-NAV-TIMEGPS
            // 0x26,           // msg id  = UBX-NAV-TIMELS, low rate, skip here
            /* NAV-SBAS errors guranteed by FAA within 6 seconds!
             * in NEO-M8N, but not most other 9-series.
             * Do not set NAV-SBAS as the gpsd decode does not go to JSON,
             * so the data is wasted. */
            // 0x32,          // msg id = NAV-SBAS, in u-blox 4 to 8, not 9
        };

        /* UBX-NAV-SOL deprecated in u-blox 6, gone in u-blox 9.
         * Use UBX-NAV-PVT after u-blox 7 (protver 15+)
         * u-blox 6 w/ GLONASS, protver 14 have NAV-PVT
         * UBX-NAV-SOL has same data from NAV-POSECEF and NAV-VELECEF.
         * Need NAV-SOL for fix type and fix flags.
         * skip NAV-POSLLH as we compute lat/lon/alt/geoid from ECEF.
         *
         * UBX-NAV-SVINFO deprecated in u-blox 8, gone in u-blox 9.
         * Use UBX-NAV-SAT after u-blox 7
         *
         * UBX-NAV-EOE makes a good cycle ender */

        // UBX for protver < 15
        const unsigned char ubx_14_nav_on[] = {
            0x06,              // msg id = NAV-SOL
            0x30,              // msg id = NAV-SVINFO
        };

        // UBX for protver >= 15
        const unsigned char ubx_15_nav_on[] = {
            // Need NAV-POSECEF, NAV-VELECEF and NAV-PVT to replace NAV-SOL
            0x01,              // msg id = NAV-POSECEF
            0x07,              // msg id = NAV-PVT
            0x11,              // msg id = NAV-VELECEF
            0x35,              // msg id = NAV-SAT
            0x61,              // msg id = NAV-EOE, first in protver 18
        };

        /*
         * Just enabling the UBX protocol for output is not enough to
         * actually get UBX output; the sentence mix is initially empty.
         * Fix that...
         */

        /* FIXME: possibly sending too many messages without waiting
         * for u-blox ACK, over running its input buffer.
         *
         * For example, the UBX-MON-VER may fail here, but works in other
         * contexts.
         *
         * Need UBX-MON-VER for protver.  Need protver to properly configure
         * the message set.
         */
        unsigned char msg[3] = {0, 0, 0};
        // request SW and HW Versions, prolly already requested at detection
        // ask again
        (void)ubx_write(session, UBX_CLASS_MON, 0x04, msg, 0);

        // turn on common UBX-NAV
        msg[0] = 0x01;          // class, UBX-NAV
        msg[2] = 0x01;          // rate, one
        for (i = 0; i < sizeof(ubx_nav_on); i++) {
            msg[1] = ubx_nav_on[i];          // msg id to turn on
            (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);
        }

        /* if protver unknown, turn on everything.  Which may be too
         * much for slower serial port speeds.  Hope that we know protver
         * later and can fix things then. */
        if (15 > session->driver.ubx.protver) {
            /* protver 14 or less, or unknown version,
             * turn on pre-15 UBX-NAV */
            msg[0] = 0x01;          // class, UBX-NAV
            msg[2] = 0x01;          // rate, one
            for (i = 0; i < sizeof(ubx_14_nav_on); i++) {
                msg[1] = ubx_14_nav_on[i];          // msg id to turn on
                (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);
            }
            if (0 != session->driver.ubx.protver) {
                // protver 14 or less, known version only.
                // turn off 15 and above UBX-NAV
                msg[0] = 0x01;          // class, UBX-NAV
                msg[2] = 0x00;          // rate, off
                for (i = 0; i < sizeof(ubx_15_nav_on); i++) {
                    msg[1] = ubx_15_nav_on[i];          // msg id to turn off
                    (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);
                }
            }
        }

        if (15 <= session->driver.ubx.protver ||
            0 == session->driver.ubx.protver) {
            // protver 15 or more, or unknown version, turn on 15+ UBX-NAV
            msg[0] = 0x01;          // class, UBX-NAV
            msg[2] = 0x01;          // rate, one
            for (i = 0; i < sizeof(ubx_15_nav_on); i++) {
                msg[1] = ubx_15_nav_on[i];          // msg id to turn on
                (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);
            }
            if (15 <= session->driver.ubx.protver) {
                // protver 15 or more, turn off 14 and below UBX-NAV
                msg[0] = 0x01;          // class, UBX-NAV
                msg[2] = 0x00;          // rate, off
                for (i = 0; i < sizeof(ubx_14_nav_on); i++) {
                    msg[1] = ubx_14_nav_on[i];          // msg id to turn off
                    (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);
                }
            }
        }

        msg[0] = 0x01;          // class
        msg[1] = 0x26;          // msg id  = UBX-NAV-TIMELS
        msg[2] = 0xff;          // about every 4 minutes if nav rate is 1Hz
        (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);

        // turn off common NMEA
        msg[0] = 0xf0;          // class, NMEA
        msg[2] = 0x00;          // rate, off
        for (i = 0; i < sizeof(nmea_off); i++) {
            msg[1] = nmea_off[i];          // msg id to turn off
            (void)ubx_write(session, UBX_CLASS_CFG, 0x01, msg, 3);
        }
    }
    return 0;
}

static void ubx_mode(struct gps_device_t *session, int mode)
{
    ubx_cfg_prt(session,
                gpsd_get_speed(session),
                gpsd_get_parity(session),
                gpsd_get_stopbits(session),
                mode);
}

static bool ubx_speed(struct gps_device_t *session,
                      speed_t speed, char parity, int stopbits)
{
    ubx_cfg_prt(session,
                speed,
                parity,
                stopbits,
                (session->lexer.type == UBX_PACKET) ? MODE_BINARY : MODE_NMEA);
    return true;
}

/* change the sample rate of the GPS */
static bool ubx_rate(struct gps_device_t *session, double cycletime)
{
    /* Minimum measurement cycle time currently known from documentation
     * for fastest devices, here in milli seconds. Maintained in
     * struct gps_type_t driver_ubx.
     */
    const int64_t min_cycle = TSTOMS(&session->device_type->min_cycle);
    // cycletime in milli seconds
    int64_t measRate = (int64_t)(cycletime * MS_IN_SEC);
    /* Message to be sent to device. */
    unsigned char msg[6] = {
        0x00, 0x00,     /* U2: Measurement rate (ms), will be set below */
        0x01, 0x00,     /* U2: Navigation rate (cycles), set to 1 */
        0x00, 0x00,     /* U2: Alignment to reference time: 0 = UTC */
    };

    // check max
    if (65535 < measRate) {
        measRate = 65535;   // milli seconds
    } else if (min_cycle > measRate) {
        /* Clamp cycle time to lowest bound given in documentation.
         * protVer >= 24 has 25 ms min.
         * protVer < 24 has min of 50ms or more.
         */
        measRate = min_cycle;
    }
    // we now know measRate fits in a U2

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "UBX rate change, measRate %lld millisecs\n",
             (long long) measRate);
    msg[0] = (unsigned char)(measRate & 0xff);
    msg[1] = (unsigned char)(measRate >> 8);

    // UBX-CFG-RATE deprecated in u-blox 10
    return ubx_write(session, UBX_CLASS_CFG, 0x08, msg, 6); // CFG-RATE
}

// This is everything we export
// *INDENT-OFF*
const struct gps_type_t driver_ubx = {
    .type_name         = "u-blox",       // Full name of type
    .packet_type       = UBX_PACKET,     // associated lexer packet type
    .flags             = DRIVER_STICKY,  // remember this
    .trigger           = NULL,
    // Number of satellite channels supported by the device
    // ZED-F0P supports 60
    .channels          = 60,
    .probe_detect      = NULL,           // Startup-time device detector
    // Packet getter (using default routine)
    .get_packet        = generic_get,
    .parse_packet      = parse_input,    // Parse message packets
    // RTCM handler (using default routine)
    .rtcm_writer       = gpsd_write,
    .init_query        = ubx_init_query, // non-perturbing initial query
    .event_hook        = ubx_event_hook, // Fire on various lifetime events
    .speed_switcher    = ubx_speed,      // Speed (baudrate) switch
    .mode_switcher     = ubx_mode,       // Mode switcher
    .rate_switcher     = ubx_rate,       // Message delivery rate switcher
    /* Minimum measurement cycle time currently known from documentation
     * for fastest devices.
     */
    .min_cycle.tv_sec  = 0,
    .min_cycle.tv_nsec = 25000000,          // Maximum 40Hz sample rate
    .control_send      = ubx_control_send,  // how to send a control string
    .time_offset       = NULL,              // no method for NTP fudge factor
};
// *INDENT-ON*
#endif // defined(UBLOX_ENABLE) && defined(BINARY_ENABLE)

// vim: set expandtab shiftwidth=4
