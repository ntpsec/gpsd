/*****************************************************************************

This is a decoder for RTCM-104 3.x, a serial protocol used for
broadcasting pseudorange corrections from differential-GPS reference
stations.  The applicable specification is RTCM 10403.1: RTCM Paper
177-2006-SC104-STD.  This obsolesces the earlier RTCM-104 2.x
specifications. The specification document is proprietary; ordering
instructions are accessible from <http://www.rtcm.org/>
under "Publications".

Unike the RTCM 2.x protocol, RTCM3.x does not use the strange
sliding-bit-window IS-GPS-200 protocol as a transport layer, but is a
self-contained byte-oriented packet protocol.  Packet recognition is
handled in the GPSD packet-getter state machine; this code is
concerned with unpacking the packets into well-behaved C structures,
coping with odd field lengths and fields that may overlap byte
boundaries.  These report structures live in gps.h.

Note that the unpacking this module does is probably useful only for
RTCM reporting and diagnostic tools.  It is not necessary when
passing RTCM corrections to a GPS, which normally should just be
passed an entire correction packet for processing by their internal
firmware.

Decodes of the following types have been verified: 1004, 1005, 1006,
1008, 1012, 1013, 1029. There is good reason to believe the 1007 code
is correct, as it's identical to 1008 up to where it ends.

The 1033 decode was arrived at by looking at an rtcminspect dump and noting
that it carries an information superset of the 1008.  There are additional
Receiver and Firmware fields we're not certain to decode without access
to an RTCM3 standard at revision 4 or later, but the guess in the code
has been observed to correctly analyze a message with a nonempty Receiver
field.

This file is Copyright 2010 by the GPSD project
SPDX-License-Identifier: BSD-2-clause

*****************************************************************************/

#include "../include/gpsd_config.h"  // must be before all includes

#include <string.h>

#include "../include/gpsd.h"
#include "../include/bits.h"

#ifdef RTCM104V3_ENABLE

// scaling constants for RTCM3 real number types
#define GPS_PSEUDORANGE_RESOLUTION      0.02    // DF011
#define PSEUDORANGE_DIFF_RESOLUTION     0.0005  // DF012, DF042
#define CARRIER_NOISE_RATIO_UNITS       0.25    // DF015, DF045, DF050
#define ANTENNA_POSITION_RESOLUTION     0.0001  // DF025, DF026, DF027
#define GLONASS_PSEUDORANGE_RESOLUTION  0.02    // DF041
#define ANTENNA_DEGREE_RESOLUTION       25e-6   // DF062
#define GPS_EPOCH_TIME_RESOLUTION       0.1     // DF065
// DF069, DF070, DF192, DF193, DF194, DF195
#define PHASE_CORRECTION_RESOLUTION     0.5
// DF156, DF157, DF158, DF166, DF167, DF168, DF169, DF196, DF197
#define TRANSLATION_MM_RESOLUTION       0.001
#define VALIDITY_RESOLUTION             2.0     // DF152, DF153, DF154, DF155
#define SCALE_PPM_RESOLUTION            1e-5    // DF162
#define ROTATION_ARCSEC_RESOLUTION      2e-5    // DF159, DF160, DF161
// DF171, DF172, DF176, DF177, DF178, DF179, DF183, DF184, DF185, DF186
#define PROJ_ORIGIN_RESOLUTION          11e-9
#define DEG_ARCSEC_RESOLUTION           3600
#define CM_RESOLUTION                   0.01    // DF198
#define RES_ARCSEC_RESOLUTION           3e-5    // DF199, DF200

// Other magic values
#define GPS_INVALID_PSEUDORANGE         0x80000 // DF012, DF018
#define GLONASS_INVALID_RANGEINCR       0x2000  // DF047
#define GLONASS_CHANNEL_BASE            7       // DF040

// Large case statements make GNU indent very confused
// *INDENT-OFF*

/* good source on message types:
 * https://software.rtcm-ntrip.org/export/HEAD/ntrip/trunk/BNC/src/bnchelp.html
 * Also look in the BNC source
 * and look at the tklib source: http://www.rtklib.com/
 */

#define ugrab(width)    (bitcount += width, ubits((unsigned char *)buf, \
                         bitcount - width, width, false))
#define sgrab(width)    (bitcount += width, sbits((signed char *)buf,  \
                         bitcount - width, width, false))

/* decode 1015/1016/1017 header
 * they share a common header
 * TODO: rtklib has C code for these.
 *
 * Return: false if decoded
 *         true if runt
 */
static bool rtcm3_101567(const struct gps_context_t *context,
                         struct rtcm3_t *rtcm, char *buf)
{
    int bitcount = 36;  // 8 preamble, 6 zero, 10 length, 12 type

    if (22 > rtcm->length) {
        // need 76 bits, 9.5 bytes
        rtcm->length = 0;          // set to zero to prevent JSON decode
        GPSD_LOG(LOG_WARN, &context->errout,
                 "RTCM3: rtcm3_101567_msm() type %d runt length %d ",
                 rtcm->type, rtcm->length);
        return true;
    }

    // 1015, 1016, and 1017 all use the 1015 struct
    rtcm->rtcmtypes.rtcm3_1015.header.network_id = (unsigned)ugrab(12);
    rtcm->rtcmtypes.rtcm3_1015.header.subnetwork_id = (unsigned )ugrab(4);
    rtcm->rtcmtypes.rtcm3_1015.header.tow = (time_t)ugrab(23);
    rtcm->rtcmtypes.rtcm3_1015.header.multimesg = (bool)ugrab(1);
    rtcm->rtcmtypes.rtcm3_1015.header.master_id = (unsigned)ugrab(12);
    rtcm->rtcmtypes.rtcm3_1015.header.aux_id = (unsigned)ugrab(12);
    rtcm->rtcmtypes.rtcm3_1015.header.satcount = (unsigned char)ugrab(4);

    // (long long)tow for 32 bit machines.
    GPSD_LOG(LOG_PROG, &context->errout, "RTCM3: rtcm3_10567(%u) "
             "network_id %u subnetwork_id %u tow %lld multimesg %u "
             "master_id %u aux_id %u satcount %u",
             rtcm->type,
             rtcm->rtcmtypes.rtcm3_1015.header.network_id,
             rtcm->rtcmtypes.rtcm3_1015.header.subnetwork_id,
             (long long)rtcm->rtcmtypes.rtcm3_1015.header.tow,
             rtcm->rtcmtypes.rtcm3_1015.header.multimesg,
             rtcm->rtcmtypes.rtcm3_1015.header.master_id,
             rtcm->rtcmtypes.rtcm3_1015.header.aux_id,
             rtcm->rtcmtypes.rtcm3_1015.header.satcount);
    return false;
}

/* decode MSM header
 * MSM1 to MSM7 share a common header
 * TODO: rtklib has C code for these.
 *
 * Return: false if decoded
 *         true if runt, or error
 */
static bool rtcm3_decode_msm(const struct gps_context_t *context,
                             struct rtcm3_t *rtcm, char *buf)
{
    int bitcount = 36;  // 8 preamble, 6 zero, 10 length, 12 type
    unsigned n_sig = 0, n_sat = 0, n_cell = 0;
    uint64_t sat_mask;
    uint32_t sig_mask;
    unsigned i;

    if (22 > rtcm->length) {
        // need 169 bits, 21.125 bytes
        rtcm->length = 0;          // set to zero to prevent JSON decode
        GPSD_LOG(LOG_WARN, &context->errout,
                 "RTCM3: rtcm3_decode_msm() type %d runt length %d ",
                 rtcm->type, rtcm->length);
        return true;
    }

    rtcm->rtcmtypes.rtcm3_msm.station_id = ugrab(12);
    rtcm->rtcmtypes.rtcm3_msm.tow = (time_t)ugrab(30);
    rtcm->rtcmtypes.rtcm3_msm.sync = ugrab(1);
    rtcm->rtcmtypes.rtcm3_msm.IODS = ugrab(3);
    bitcount += 7;             // skip 7 reserved bits, DF001
    rtcm->rtcmtypes.rtcm3_msm.steering = ugrab(2);
    rtcm->rtcmtypes.rtcm3_msm.ext_clk = ugrab(2);
    rtcm->rtcmtypes.rtcm3_msm.smoothing = ugrab(1);
    rtcm->rtcmtypes.rtcm3_msm.interval = ugrab(3);
    // FIXME: rtcm->rtcmtypes.rtcm3_msm.sat_mask = ugrab(64);
    // ugrab(56) is max, can't do 64, so stack it
    rtcm->rtcmtypes.rtcm3_msm.sat_mask = ugrab(32) << 32;
    rtcm->rtcmtypes.rtcm3_msm.sat_mask |= ugrab(32);
    rtcm->rtcmtypes.rtcm3_msm.sig_mask = ugrab(32);

    // count satellites
    sat_mask = rtcm->rtcmtypes.rtcm3_msm.sat_mask;
    while (sat_mask) {
        n_sat += sat_mask & 1;
        sat_mask >>= 1;
    }
    // count signals
    sig_mask = rtcm->rtcmtypes.rtcm3_msm.sig_mask;
    while (sig_mask) {
        n_sig += sig_mask & 1;
        sig_mask >>= 1;
    }
    // determine cells
    n_cell = n_sat * n_sig;
    rtcm->rtcmtypes.rtcm3_msm.n_sat = n_sat;
    rtcm->rtcmtypes.rtcm3_msm.n_sig = n_sig;
    rtcm->rtcmtypes.rtcm3_msm.n_cell = n_cell;

    if (0 == n_sat ||
        64 < n_cell) {
        GPSD_LOG(LOG_WARN, &context->errout,
                 "RTCM3: rtcm3_decode_msm(%u) interval %u  sat_mask x%llx "
                 "sig_mask x%x invalid n_cell %u\n",
                 rtcm->type,
                 rtcm->rtcmtypes.rtcm3_msm.interval,
                 (unsigned long long)rtcm->rtcmtypes.rtcm3_msm.sat_mask,
                 rtcm->rtcmtypes.rtcm3_msm.sig_mask,
                 n_cell);
        return false;
    }

    // cell_mask is variable length!  ugrab() width max is 56
    if (56 >= n_cell) {
        rtcm->rtcmtypes.rtcm3_msm.cell_mask = ugrab(n_cell);
    } else {
        // 57 to 64, breaks ugrab(), workaround it...
        rtcm->rtcmtypes.rtcm3_msm.cell_mask = ugrab(56);
        rtcm->rtcmtypes.rtcm3_msm.cell_mask <<= n_cell - 56;
        rtcm->rtcmtypes.rtcm3_msm.cell_mask |= ugrab(n_cell - 56);
    }

    // Decode Satellite Data

    // Decode DF397 (MSM 4-7)
    if (4 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        5 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        6 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_sat; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sat[i].rr_ms = ugrab(8);
        }
    }

    // Decode Extended Info (MSM 5+7)
    if (5 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_sat; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sat[i].ext_info = ugrab(4);
        }
    }

    // Decode DF398 (MSM 1-7)
    for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_sat; i++) {
        rtcm->rtcmtypes.rtcm3_msm.sat[i].rr_m1 = ugrab(10);
    };

    // Decode DF399 (MSM 5+7)
    if (5 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_sat; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sat[i].rates_rphr = ugrab(14);
        }
    }

    // Decode Signal Data

    // Decode DF400 (MSM 1,3,4,5) resp. DF405 (MSM 6+7)
    if (1 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        3 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        4 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        5 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].pseudo_r = sgrab(15);
        }
    } else if (6 == rtcm->rtcmtypes.rtcm3_msm.msm ||
               7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].pseudo_r = sgrab(20);
        }
    }

    // Decode DF401 (MSM 2,3,4,5) resp. DF406 (MSM 6+7)
    if (2 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        3 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        4 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        5 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].phase_r = sgrab(22);
        }
    } else if (6 == rtcm->rtcmtypes.rtcm3_msm.msm ||
               7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].phase_r = sgrab(24);
        }
    }

    // Decode DF402 (MSM 2,3,4,5) resp. DF407 (MSM 6+7)
    if (2 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        3 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        4 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        5 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].lti = ugrab(4);
        }
    } else if (6 == rtcm->rtcmtypes.rtcm3_msm.msm ||
               7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].lti = ugrab(10);
        }
    }

    // Decode DF420 (MSM 2-7)
    if (2 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        3 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        4 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        5 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        6 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].half_amb = ugrab(1);
        }
    }

    // Decode DF403 (MSM 4+5) resp. DF408 (MSM 6+7)
    if (4 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        5 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].cnr = ugrab(6);
        }
    } else if (6 == rtcm->rtcmtypes.rtcm3_msm.msm ||
               7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].cnr = ugrab(10);
        }
    }

    // Decode DF404 (MSM 5+7)
    if (5 == rtcm->rtcmtypes.rtcm3_msm.msm ||
        7 == rtcm->rtcmtypes.rtcm3_msm.msm) {
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_msm.n_cell; i++) {
            rtcm->rtcmtypes.rtcm3_msm.sig[i].cnr = sgrab(15);
        }
    }

    // (long long)tow for 32 bit machines.
    GPSD_LOG(LOG_PROG, &context->errout, "RTCM3: rtcm3_decode_msm(%u) "
             "gnssid %u MSM%u id %u tow %lld sync %u IODS %u "
             "steering %u ext_clk %u smoothing %u interval %u "
             "sat_mask x%llx sig_mask x%lx cell_mask %llx\n",
             rtcm->type,
             rtcm->rtcmtypes.rtcm3_msm.gnssid,
             rtcm->rtcmtypes.rtcm3_msm.msm,
             rtcm->rtcmtypes.rtcm3_msm.station_id,
             (long long)rtcm->rtcmtypes.rtcm3_msm.tow,
             rtcm->rtcmtypes.rtcm3_msm.sync,
             rtcm->rtcmtypes.rtcm3_msm.IODS,
             rtcm->rtcmtypes.rtcm3_msm.steering,
             rtcm->rtcmtypes.rtcm3_msm.ext_clk,
             rtcm->rtcmtypes.rtcm3_msm.smoothing,
             rtcm->rtcmtypes.rtcm3_msm.interval,
             (long long unsigned)rtcm->rtcmtypes.rtcm3_msm.sat_mask,
             (long unsigned)rtcm->rtcmtypes.rtcm3_msm.sig_mask,
             (long long unsigned)rtcm->rtcmtypes.rtcm3_msm.cell_mask);
    return false;
}

/* break out the raw bits into the scaled report-structure fields
 *
 * Return: void
 */
void rtcm3_unpack(const struct gps_context_t *context,
                  struct rtcm3_t *rtcm, char *buf)
{
    unsigned n, n2, n3, n4;
    int bitcount = 0;
    unsigned i;
    signed long temp;
    bool unknown = true;              // we don't know how to decode
    const char *unknown_name = NULL;  // no decode, but maybe we know the name
    unsigned preamble, mbz;           // preamble 0xd3, and must be zero

#define GPS_PSEUDORANGE(fld, len) \
    {temp = (unsigned long)ugrab(len);          \
    if (temp == GPS_INVALID_PSEUDORANGE) {      \
        fld.pseudorange = 0;                    \
    } else {                                    \
        fld.pseudorange = temp * GPS_PSEUDORANGE_RESOLUTION;} \
    }
#define RANGEDIFF(fld, len) \
    temp = (long)sgrab(len);                    \
    if (temp == GPS_INVALID_PSEUDORANGE) {      \
        fld.rangediff = 0;                      \
    } else {                                    \
        fld.rangediff = temp * PSEUDORANGE_DIFF_RESOLUTION; \
    }

    memset(rtcm, 0, sizeof(struct rtcm3_t));

    // check preamble and zero bits
    preamble = ugrab(8);
    mbz = ugrab(6);
    if (0xD3 != preamble ||
        0 != mbz) {
        GPSD_LOG(LOG_WARN, &context->errout,
                 "RTCM3: invalid preamble x%2x or mbz x%x\n",
                 preamble, mbz);
    }

    rtcm->length = (unsigned)ugrab(10);
    if (2 > rtcm->length) {
        // ignore zero payload messages, they do not evan have type
        // need 2 bytes just to read 10 bit type.
        return;
    }
    rtcm->type = (unsigned)ugrab(12);

    GPSD_LOG(LOG_RAW, &context->errout,
             "RTCM3: type %d payload length %d bitcount %d\n",
             rtcm->type, rtcm->length, bitcount);

    // RTCM3 message type numbers start at 1001
    switch (rtcm->type) {
    case 1001:
        // GPS Basic RTK, L1 Only
        rtcm->rtcmtypes.rtcm3_1001.header.station_id = (unsigned)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1001.header.tow = (time_t)ugrab(30);
        rtcm->rtcmtypes.rtcm3_1001.header.sync = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1001.header.satcount = (unsigned short)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1001.header.smoothing = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1001.header.interval = (unsigned short)ugrab(3);
#define R1001 rtcm->rtcmtypes.rtcm3_1001.rtk_data[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1001.header.satcount; i++) {
            R1001.ident = (unsigned short)ugrab(6);
            R1001.L1.indicator = (unsigned char)ugrab(1);
            GPS_PSEUDORANGE(R1001.L1, 24);
            RANGEDIFF(R1001.L1, 20);
            R1001.L1.locktime = (unsigned char)sgrab(7);
        }
#undef R1001
        unknown = false;
        break;

    case 1002:
        // GPS Extended RTK, L1 Only
        rtcm->rtcmtypes.rtcm3_1002.header.station_id = (unsigned)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1002.header.tow = (time_t)ugrab(30);
        rtcm->rtcmtypes.rtcm3_1002.header.sync = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1002.header.satcount = (unsigned short)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1002.header.smoothing = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1002.header.interval = (unsigned short)ugrab(3);
#define R1002 rtcm->rtcmtypes.rtcm3_1002.rtk_data[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1002.header.satcount; i++) {
            R1002.ident = (unsigned short)ugrab(6);
            R1002.L1.indicator = (unsigned char)ugrab(1);
            GPS_PSEUDORANGE(R1002.L1, 24);
            RANGEDIFF(R1002.L1, 20);
            R1002.L1.locktime = (unsigned char)sgrab(7);
            R1002.L1.ambiguity = (unsigned char)ugrab(8);
            R1002.L1.CNR = (ugrab(8)) * CARRIER_NOISE_RATIO_UNITS;
        }
#undef R1002
        unknown = false;
        break;

    case 1003:
        // GPS Basic RTK, L1 & L2
        rtcm->rtcmtypes.rtcm3_1003.header.station_id = (unsigned)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1003.header.tow = (time_t)ugrab(30);
        rtcm->rtcmtypes.rtcm3_1003.header.sync = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1003.header.satcount = (unsigned short)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1003.header.smoothing = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1003.header.interval = (unsigned short)ugrab(3);
#define R1003 rtcm->rtcmtypes.rtcm3_1003.rtk_data[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1003.header.satcount; i++) {
            R1003.ident = (unsigned short)ugrab(6);
            R1003.L1.indicator = (unsigned char)ugrab(1);
            GPS_PSEUDORANGE(R1003.L1, 24);
            RANGEDIFF(R1003.L1, 20);
            R1003.L1.locktime = (unsigned char)sgrab(7);
            R1003.L2.indicator = (unsigned char)ugrab(2);
            GPS_PSEUDORANGE(R1003.L2, 24);
            temp = (long)sgrab(20);
            if (temp == GPS_INVALID_PSEUDORANGE) {
                R1003.L2.rangediff = 0;
            } else {
                R1003.L2.rangediff = temp * PSEUDORANGE_DIFF_RESOLUTION;
            }
            R1003.L2.locktime = (unsigned char)sgrab(7);
        }
#undef R1003
        unknown = false;
        break;

    case 1004:
        // GPS Extended RTK, L1 & L2
        rtcm->rtcmtypes.rtcm3_1004.header.station_id = (unsigned)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1004.header.tow = (time_t)ugrab(30);
        rtcm->rtcmtypes.rtcm3_1004.header.sync = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1004.header.satcount = (unsigned short)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1004.header.smoothing = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1004.header.interval = (unsigned short)ugrab(3);
#define R1004 rtcm->rtcmtypes.rtcm3_1004.rtk_data[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1004.header.satcount; i++) {
            R1004.ident = (unsigned short)ugrab(6);
            R1004.L1.indicator = (bool)ugrab(1);
            GPS_PSEUDORANGE(R1004.L1, 24);
            RANGEDIFF(R1004.L1, 20);
            R1004.L1.locktime = (unsigned char)sgrab(7);
            R1004.L1.ambiguity = (unsigned char)ugrab(8);
            R1004.L1.CNR = ugrab(8) * CARRIER_NOISE_RATIO_UNITS;
            R1004.L2.indicator = (unsigned char)ugrab(2);
            GPS_PSEUDORANGE(R1004.L2, 14);
            RANGEDIFF(R1004.L2, 20);
            R1004.L2.locktime = (unsigned char)sgrab(7);
            R1004.L2.CNR = ugrab(8) * CARRIER_NOISE_RATIO_UNITS;
        }
#undef R1004
        unknown = false;
        break;

    case 1005:
        /* Stationary Antenna Reference Point, No Height Information
         * 19 bytes */
#define R1005 rtcm->rtcmtypes.rtcm3_1005
        R1005.station_id = (unsigned short)ugrab(12);
        ugrab(6);               // reserved
        R1005.system = ugrab(3);
        R1005.reference_station = (bool)ugrab(1);
        R1005.ecef_x = sgrab(38) * ANTENNA_POSITION_RESOLUTION;
        R1005.single_receiver = ugrab(1);
        ugrab(1);
        R1005.ecef_y = sgrab(38) * ANTENNA_POSITION_RESOLUTION;
        ugrab(2);
        R1005.ecef_z = sgrab(38) * ANTENNA_POSITION_RESOLUTION;
#undef R1005
        unknown = false;
        break;

    case 1006:
        /* Stationary Antenna Reference Point, with Height Information
         * 21 bytes */
#define R1006 rtcm->rtcmtypes.rtcm3_1006
        R1006.station_id = (unsigned short)ugrab(12);
        (void)ugrab(6);         // reserved
        R1006.system = ugrab(3);
        R1006.reference_station = (bool)ugrab(1);
        R1006.ecef_x = sgrab(38) * ANTENNA_POSITION_RESOLUTION;
        R1006.single_receiver = ugrab(1);
        ugrab(1);
        R1006.ecef_y = sgrab(38) * ANTENNA_POSITION_RESOLUTION;
        ugrab(2);
        R1006.ecef_z = sgrab(38) * ANTENNA_POSITION_RESOLUTION;
        R1006.height = ugrab(16) * ANTENNA_POSITION_RESOLUTION;
#undef R1006
        unknown = false;
        break;

    case 1007:
        /* Antenna Description
         * 5 to 36 bytes */
        rtcm->rtcmtypes.rtcm3_1007.station_id = (unsigned short)ugrab(12);
        n = (unsigned long)ugrab(8);
        (void)memcpy(rtcm->rtcmtypes.rtcm3_1007.descriptor, buf + 7, n);
        rtcm->rtcmtypes.rtcm3_1007.descriptor[n] = '\0';
        bitcount += 8 * n;
        rtcm->rtcmtypes.rtcm3_1007.setup_id = ugrab(8);
        unknown = false;
        break;

    case 1008:
        /* Antenna Description & Serial Number
         * 6 to 68 bytes */
        rtcm->rtcmtypes.rtcm3_1008.station_id = (unsigned short)ugrab(12);
        n = (unsigned long)ugrab(8);
        (void)memcpy(rtcm->rtcmtypes.rtcm3_1008.descriptor, buf + 7, n);
        rtcm->rtcmtypes.rtcm3_1008.descriptor[n] = '\0';
        bitcount += 8 * n;
        rtcm->rtcmtypes.rtcm3_1008.setup_id = ugrab(8);
        n2 = (unsigned long)ugrab(8);
        (void)memcpy(rtcm->rtcmtypes.rtcm3_1008.serial, buf + 9 + n, n2);
        rtcm->rtcmtypes.rtcm3_1008.serial[n2] = '\0';
        // bitcount += 8 * n2;
        unknown = false;
        break;

    case 1009:
        // GLONASS Basic RTK, L1 Only
        rtcm->rtcmtypes.rtcm3_1009.header.station_id =
            (unsigned short)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1009.header.tow = (time_t)ugrab(27);
        rtcm->rtcmtypes.rtcm3_1009.header.sync = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1009.header.satcount = (unsigned short)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1009.header.smoothing = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1009.header.interval = (unsigned short)ugrab(3);
#define R1009 rtcm->rtcmtypes.rtcm3_1009.rtk_data[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1009.header.satcount; i++) {
            R1009.ident = (unsigned short)ugrab(6);
            R1009.L1.indicator = (bool)ugrab(1);
            R1009.L1.channel = (short)ugrab(5) - GLONASS_CHANNEL_BASE;
            R1009.L1.pseudorange = ugrab(25) * GLONASS_PSEUDORANGE_RESOLUTION;
            RANGEDIFF(R1009.L1, 20);
            R1009.L1.locktime = (unsigned char)sgrab(7);
        }
#undef R1009
        unknown = false;
        break;

    case 1010:
        // GLONASS Extended RTK, L1 Only
        rtcm->rtcmtypes.rtcm3_1010.header.station_id =
            (unsigned short)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1010.header.tow = (time_t)ugrab(27);
        rtcm->rtcmtypes.rtcm3_1010.header.sync = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1010.header.satcount = (unsigned short)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1010.header.smoothing = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1010.header.interval = (unsigned short)ugrab(3);
#define R1010 rtcm->rtcmtypes.rtcm3_1010.rtk_data[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1010.header.satcount; i++) {
            R1010.ident = (unsigned short)ugrab(6);
            R1010.L1.indicator = (bool)ugrab(1);
            R1010.L1.channel = (short)ugrab(5) - GLONASS_CHANNEL_BASE;
            R1010.L1.pseudorange = ugrab(25) * GLONASS_PSEUDORANGE_RESOLUTION;
            RANGEDIFF(R1010.L1, 20);
            R1010.L1.locktime = (unsigned char)sgrab(7);
            R1010.L1.ambiguity = (unsigned char)ugrab(7);
            R1010.L1.CNR = ugrab(8) * CARRIER_NOISE_RATIO_UNITS;
        }
#undef R1010
        unknown = false;
        break;

    case 1011:
        // GLONASS Basic RTK, L1 & L2
        rtcm->rtcmtypes.rtcm3_1011.header.station_id =
            (unsigned short)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1011.header.tow = (time_t)ugrab(27);
        rtcm->rtcmtypes.rtcm3_1011.header.sync = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1011.header.satcount = (unsigned short)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1011.header.smoothing = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1011.header.interval = (unsigned short)ugrab(3);
#define R1011 rtcm->rtcmtypes.rtcm3_1011.rtk_data[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1011.header.satcount; i++) {
            R1011.ident = (unsigned short)ugrab(6);
            R1011.L1.indicator = (bool)ugrab(1);
            R1011.L1.channel = (short)ugrab(5) - GLONASS_CHANNEL_BASE;
            R1011.L1.pseudorange = ugrab(25) * GLONASS_PSEUDORANGE_RESOLUTION;
            RANGEDIFF(R1011.L1, 20);
            R1011.L1.locktime = (unsigned char)sgrab(7);
            R1011.L1.ambiguity = (unsigned char)ugrab(7);
            R1011.L1.CNR = ugrab(8) * CARRIER_NOISE_RATIO_UNITS;
            R1011.L2.indicator = (bool)ugrab(1);
            R1011.L2.channel = (short)ugrab(5) - GLONASS_CHANNEL_BASE;
            R1011.L2.pseudorange = ugrab(25) * GLONASS_PSEUDORANGE_RESOLUTION;
            RANGEDIFF(R1011.L2, 20);
            R1011.L2.locktime = (unsigned char)sgrab(7);
            R1011.L2.ambiguity = (unsigned char)ugrab(7);
            R1011.L2.CNR = ugrab(8) * CARRIER_NOISE_RATIO_UNITS;
        }
#undef R1011
        unknown = false;
        break;

    case 1012:
        // GLONASS Extended RTK, L1 & L2
        rtcm->rtcmtypes.rtcm3_1012.header.station_id =
            (unsigned short)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1012.header.tow = (time_t)ugrab(27);
        rtcm->rtcmtypes.rtcm3_1012.header.sync = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1012.header.satcount = (unsigned short)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1012.header.smoothing = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1012.header.interval = (unsigned short)ugrab(3);
#define R1012 rtcm->rtcmtypes.rtcm3_1012.rtk_data[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1012.header.satcount; i++) {
            unsigned rangeincr;

            R1012.ident = (unsigned short)ugrab(6);
            R1012.L1.indicator = (bool)ugrab(1);
            R1012.L1.channel = (short)ugrab(5) - GLONASS_CHANNEL_BASE;
            R1012.L1.pseudorange = ugrab(25) * GLONASS_PSEUDORANGE_RESOLUTION;
            RANGEDIFF(R1012.L1, 20);
            R1012.L1.locktime = (unsigned char)ugrab(7);
            R1012.L1.ambiguity = (unsigned char)ugrab(7);
            R1012.L1.CNR = (unsigned char)ugrab(8) * CARRIER_NOISE_RATIO_UNITS;
            R1012.L2.indicator = (bool)ugrab(2);
            rangeincr = ugrab(14);
            if (rangeincr == GLONASS_INVALID_RANGEINCR) {
                R1012.L2.pseudorange = 0;
            } else {
                R1012.L2.pseudorange = (rangeincr *
                                        GLONASS_PSEUDORANGE_RESOLUTION);
            }
            RANGEDIFF(R1012.L2, 20);
            R1012.L2.locktime = (unsigned char)sgrab(7);
            R1012.L2.CNR = (unsigned char)ugrab(8) * CARRIER_NOISE_RATIO_UNITS;
        }
#undef R1012
        unknown = false;
        break;

    case 1013:
        // System Parameters
        rtcm->rtcmtypes.rtcm3_1013.station_id = (unsigned short)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1013.mjd = (unsigned short)ugrab(16);
        rtcm->rtcmtypes.rtcm3_1013.sod = (unsigned short)ugrab(17);
        rtcm->rtcmtypes.rtcm3_1013.ncount = (unsigned long)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1013.leapsecs = (unsigned char)ugrab(8);
#define R1013 rtcm->rtcmtypes.rtcm3_1013.announcements[i]
        for (i = 0; i < rtcm->rtcmtypes.rtcm3_1013.ncount; i++) {
            R1013.id = (unsigned short)ugrab(12);
            R1013.sync = (bool)ugrab(1);
            R1013.interval = (unsigned short)ugrab(16);
        }
#undef R1013
        unknown = false;
        break;

    case 1014:
        /* Network Auxiliary Station Data
         * coordinate difference between one Aux station and the master station
         */
        rtcm->rtcmtypes.rtcm3_1014.network_id = (int)ugrab(8);
        rtcm->rtcmtypes.rtcm3_1014.subnetwork_id = (int)ugrab(4);
        rtcm->rtcmtypes.rtcm3_1014.stationcount = (char)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1014.master_id = (int)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1014.aux_id = (int)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1014.d_lat =
            (unsigned short)ugrab(20) * ANTENNA_DEGREE_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1014.d_lon =
            (unsigned short)ugrab(21) * ANTENNA_DEGREE_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1014.d_alt = (unsigned short)ugrab(23) / 1000;
        unknown = false;
        break;

    case 1015:
        /* RTCM 3.1
         * GPS Ionospheric Correction Differences for all satellites
         * between the master station and one auxiliary station
         * 9 bytes minimum
         */
        unknown = rtcm3_101567(context, rtcm, buf);
        unknown_name = "GPS Ionospheric Correction Differences";
        break;

    case 1016:
        /* RTCM 3.1
         * GPS Geometric Correction Differences for all satellites between
         * the master station and one auxiliary station.
         * 9 bytes minimum
         */
        unknown = rtcm3_101567(context, rtcm, buf);
        unknown_name = "GPS Geometric Correction Differences";
        break;

    case 1017:
        /* RTCM 3.1
         * GPS Combined Geometric and Ionospheric Correction Differences
         * for all satellites between one Aux station and the master station
         * (same content as both types 1015 and 1016 together, but less size)
         * 9 bytes minimum
         */
        unknown = rtcm3_101567(context, rtcm, buf);
        unknown_name = "GPS Combined Geometric and Ionospheric "
                       "Correction Differences";
        break;

    case 1018:
        /* RTCM 3.1
         * Reserved for alternative Ionospheric Correction Difference Message
         */
        unknown_name = "Reserved for alternative Ionospheric Correction "
                       "Differences";
        break;

    case 1019:
        /* RTCM 3.1 - 1020
         * GPS Ephemeris
         * 62 bytes
         */
        // TODO: rtklib has C code for this one.
        unknown_name = "GPS Ephemeris";
        break;

    case 1020:
        /* RTCM 3.1 - 1020
         * GLONASS Ephemeris
         * 45 bytes
         */
        // TODO: rtklib has C code for this one.
        unknown_name = "GLO Ephemeris";
        break;

    case 1021:
        /* RTCM 3.1
         * Helmert / Abridged Molodenski Transformation parameters
         */
        /* unknown_name = "Helmert / Abridged Molodenski Transformation "
                       "parameters";*/
        // Set Source-Name
        n = (unsigned)ugrab(5);
        if ((sizeof(rtcm->rtcmtypes.rtcm3_1021.src_name) -1) <= n) {
            // paranoia
            n = sizeof(rtcm->rtcmtypes.rtcm3_1021.src_name) - 1;
        }
        for (i = 0; i < n; i++) {
            rtcm->rtcmtypes.rtcm3_1021.src_name[i] = (char)ugrab(8);
        }
        rtcm->rtcmtypes.rtcm3_1021.src_name[n] = '\0';
        // Set Target-Name
        n2 = (unsigned)ugrab(5);
        if ((sizeof(rtcm->rtcmtypes.rtcm3_1021.tar_name) - 1) <= n2) {
            // paranoia
            n2 = sizeof(rtcm->rtcmtypes.rtcm3_1021.tar_name) - 1;
        }
        for (i = 0; i < n2; i++) {
            rtcm->rtcmtypes.rtcm3_1021.tar_name[i] = (char)ugrab(8);
        }
        rtcm->rtcmtypes.rtcm3_1021.tar_name[n2] = '\0';
        rtcm->rtcmtypes.rtcm3_1021.sys_id_num = (unsigned)ugrab(8);
#define R1021 rtcm->rtcmtypes.rtcm3_1021.ut_tr_msg_id[i]
        for (i = 0; i < RTCM3_DF148_SIZE; i++) {
            R1021 = (bool)ugrab(1);
        }
#undef R1021
        rtcm->rtcmtypes.rtcm3_1021.plate_number = (unsigned)ugrab(5);
        rtcm->rtcmtypes.rtcm3_1021.computation_id = (unsigned)ugrab(4);
        rtcm->rtcmtypes.rtcm3_1021.height_id = (unsigned)ugrab(2);
        rtcm->rtcmtypes.rtcm3_1021.lat_origin = sgrab(19) *
            VALIDITY_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.lon_origin = sgrab(20) *
            VALIDITY_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.lat_extension = sgrab(14) *
            VALIDITY_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.lon_extension = sgrab(14) *
            VALIDITY_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.x_trans = sgrab(23) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.y_trans = sgrab(23) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.z_trans = sgrab(23) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.x_rot = sgrab(32) *
            ROTATION_ARCSEC_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.y_rot = sgrab(32) *
            ROTATION_ARCSEC_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.z_rot = sgrab(32) *
            ROTATION_ARCSEC_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.ds = sgrab(25) * SCALE_PPM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.add_as = sgrab(24) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.add_bs = sgrab(25) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.add_at = sgrab(24) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.add_bt = sgrab(25) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1021.quality_hori = (unsigned)ugrab(3);
        rtcm->rtcmtypes.rtcm3_1021.quality_vert = (unsigned)ugrab(3);

        unknown = false;
        break;

    case 1022:
        /* RTCM 3.1
         * Molodenski-Badekas transformation parameters
         */
        unknown_name = "Molodenski-Badekas transformation parameters";
        break;

    case 1023:
        /* RTCM 3.1
         * Residuals Ellipsoidal Grid Representation
         */
        // unknown_name = "Residuals Ellipsoidal Grid Representation";
        rtcm->rtcmtypes.rtcm3_1023.sys_id_num = (unsigned)ugrab(8);
        rtcm->rtcmtypes.rtcm3_1023.shift_id_hori = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1023.shift_id_vert = (bool)ugrab(1);
        rtcm->rtcmtypes.rtcm3_1023.lat_origin = sgrab(21) *
            PHASE_CORRECTION_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1023.lon_origin = sgrab(22) *
            PHASE_CORRECTION_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1023.lat_extension = (unsigned)ugrab(12) *
            PHASE_CORRECTION_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1023.lon_extension = (unsigned)ugrab(12) *
            PHASE_CORRECTION_RESOLUTION / DEG_ARCSEC_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1023.lat_mean = sgrab(8) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1023.lon_mean = sgrab(8) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1023.hgt_mean = sgrab(15) * CM_RESOLUTION;
#define R1023 rtcm->rtcmtypes.rtcm3_1023.residuals[i]
        for (i = 0; i < RTCM3_GRID_SIZE; i++) {
            R1023.lat_res = sgrab(9) * RES_ARCSEC_RESOLUTION;
            R1023.lon_res = sgrab(9) * RES_ARCSEC_RESOLUTION;
            R1023.hgt_res = sgrab(9) * TRANSLATION_MM_RESOLUTION;
        }
#undef R1023
        rtcm->rtcmtypes.rtcm3_1023.interp_meth_id_hori = (unsigned)ugrab(2);
        rtcm->rtcmtypes.rtcm3_1023.interp_meth_id_vert = (unsigned)ugrab(2);
        rtcm->rtcmtypes.rtcm3_1023.grd_qual_id_hori = (unsigned)ugrab(3);
        rtcm->rtcmtypes.rtcm3_1023.grd_qual_id_vert = (unsigned)ugrab(3);
        rtcm->rtcmtypes.rtcm3_1023.mjd = (unsigned short)ugrab(16);
        unknown = false;
        break;

    case 1024:
        /* RTCM 3.1
         * Residuals Plane Grid Representation
         */
        unknown_name = "Residuals Plane Grid Representation";
        break;

    case 1025:
        /* RTCM 3.1
         * Projection Parameters, Projection Types other than LCC2SP
         */
        /* unknown_name = "Projection Parameters, Projection Types other "
                       "than LCC2SP"; */
        rtcm->rtcmtypes.rtcm3_1025.sys_id_num = (unsigned short)ugrab(8);
        rtcm->rtcmtypes.rtcm3_1025.projection_type = (unsigned short)ugrab(6);
        rtcm->rtcmtypes.rtcm3_1025.lat_origin = sgrab(34) *
            PROJ_ORIGIN_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1025.lon_origin = sgrab(35) *
            PROJ_ORIGIN_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1025.add_sno = (unsigned)ugrab(30) *
            SCALE_PPM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1025.false_east = (unsigned)ugrab(36) *
            TRANSLATION_MM_RESOLUTION;
        rtcm->rtcmtypes.rtcm3_1025.false_north = ugrab(35) *
            TRANSLATION_MM_RESOLUTION;
        unknown = false;
        break;

    case 1026:
        /* RTCM 3.1
         * Projection Parameters, Projection Type LCC2SP
         * (Lambert Conic Conformal)
         */
        unknown_name = "Projection Parameters, Projection Type LCC2SP";
        break;

    case 1027:
        /* RTCM 3.1
         * Projection Parameters, Projection Type OM (Oblique Mercator)
         */
        unknown_name = "Projection Parameters, Projection Type OM";
        break;

    case 1028:
        /* RTCM 3.1
         * Reserved for global to plate fixed transformation
         */
        unknown_name = "Reserved, Global to Plate Transformation";
        break;

    case 1029:
        /* Text in UTF8 format
         * 9 bytes minimum
         * (max. 127 multibyte characters and max. 255 bytes)
         */
        rtcm->rtcmtypes.rtcm3_1029.station_id = (unsigned short)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1029.mjd = (unsigned short)ugrab(16);
        rtcm->rtcmtypes.rtcm3_1029.sod = (unsigned short)ugrab(17);
        rtcm->rtcmtypes.rtcm3_1029.len = (unsigned long)ugrab(7);
        rtcm->rtcmtypes.rtcm3_1029.unicode_units = (size_t)ugrab(8);
        (void)memcpy(rtcm->rtcmtypes.rtcm3_1029.text,
                     buf + 12, rtcm->rtcmtypes.rtcm3_1029.unicode_units);
        unknown = false;
        break;

    case 1030:
        /* RTCM 3.1
         * GPS Network RTK Residual Message
         */
        unknown_name = "GPS Network RTK Residual";
        break;

    case 1031:
        /* RTCM 3.1
         * GLONASS Network RTK Residual Message
         */
        unknown_name = "GLONASS Network RTK Residual";
        break;

    case 1032:
        /* RTCM 3.1
         * Physical Reference Station Position message
         */
        unknown_name = "Physical Reference Station Position";
        break;

    case 1033:                  // see note in header
        /* Receiver and Antenna Descriptor
         * Type1033 is a combined Message Types 1007 and 1008
         * and hence contains antenna descriptor and serial number
         * as well as receiver descriptor and serial number.
         */
        // TODO: rtklib has C code for this one.
        rtcm->rtcmtypes.rtcm3_1033.station_id = (unsigned short)ugrab(12);
        n = (unsigned long)ugrab(8);
        (void)memcpy(rtcm->rtcmtypes.rtcm3_1033.descriptor, buf + 7, n);
        rtcm->rtcmtypes.rtcm3_1033.descriptor[n] = '\0';
        bitcount += 8 * n;
        rtcm->rtcmtypes.rtcm3_1033.setup_id = ugrab(8);
        n2 = (unsigned long)ugrab(8);
        (void)memcpy(rtcm->rtcmtypes.rtcm3_1033.serial, buf + 9 + n, n2);
        rtcm->rtcmtypes.rtcm3_1033.serial[n2] = '\0';
        bitcount += 8 * n2;
        n3 = (unsigned long)ugrab(8);
        (void)memcpy(rtcm->rtcmtypes.rtcm3_1033.receiver, buf + 10+n+n2, n3);
        rtcm->rtcmtypes.rtcm3_1033.receiver[n3] = '\0';
        bitcount += 8 * n3;
        n4 = (unsigned long)ugrab(8);
        (void)memcpy(rtcm->rtcmtypes.rtcm3_1033.firmware, buf + 11+n+n2+n3, n3);
        rtcm->rtcmtypes.rtcm3_1033.firmware[n4] = '\0';
        // bitcount += 8 * n4;
        // TODO: next is receiver serial number
        unknown = false;
        break;

    case 1034:
        /* RTCM 3.2
         * GPS Network FKP Gradient Message
         */
        unknown_name = "GPS Network FKP Gradient";
        break;

    case 1035:
        /* RTCM 3.2
         * GLONASS Network FKP Gradient Message
         */
        unknown_name = "GLO Network FKP Gradient";
        break;

    case 1037:
        /* RTCM 3.2
         * GLONASS Ionospheric Correction Differences
         */
        unknown_name = "GLO Ionospheric Correction Differences";
        break;

    case 1038:
        /* RTCM 3.2
         * GLONASS Geometric Correction Differences
         */
        unknown_name = "GLO Geometric Correction Differences";
        break;

    case 1039:
        /* RTCM 3.2
         * GLONASS Combined Geometric and Ionospheric Correction Differences
         */
        unknown_name = "GLONASS Combined Geometric and Ionospheric "
                       "Correction Differences";
        break;

    case 1042:
        /* RTCM 3.x - 1043
         * BeiDou Ephemeris
         * length ?
         */
        unknown_name = "BD Ephemeris";
        break;

    case 1043:
        /* RTCM 3.x - 1043
         * SBAS Ephemeris
         * length 29
         */
        unknown_name = "SBAS Ephemeris";
        break;

    case 1044:
        /* RTCM 3.x - 1044
         * QZSS ephemeris
         * length 61
         */
        // TODO: rtklib has C code for this one.
        unknown_name = "QZSS Ephemeris";
        break;

    case 1045:
        /* RTCM 3.2 - 1045
         * Galileo F/NAV Ephemeris Data
         * 64 bytes
         */
        // TODO: rtklib has C code for this one.
        unknown_name = "GAL F/NAV Ephemeris Data";
        break;

    case 1046:
        /* RTCM 3.x - 1046
         * Galileo I/NAV Ephemeris Data
         * length 63
         */
        // TODO: rtklib has C code for this one.
        unknown_name = "GAL I/NAV Ephemeris Data";
        break;

    case 1057:
        /* RTCM 3.2
         * SSR GPS Orbit Correction
         */
        unknown_name = "SSR GPS Orbit Correction";
        break;

    case 1058:
        /* RTCM 3.2
         * SSR GPS Clock Correction
         */
        unknown_name = "SSR GPS Clock Correction";
        break;

    case 1059:
        /* RTCM 3.2
         * SSR GPS Code Bias
         */
        unknown_name = "SSR GPS Code Bias";
        break;

    case 1060:
        /* RTCM 3.2
         * SSR GPS Combined Orbit and Clock Correction
         */
        unknown_name = "SSR GPS Combined Orbit and Clock Correction";
        break;

    case 1061:
        /* RTCM 3.2
         * SSR GPS URA
         */
        unknown_name = "SSR GPS URA";
        break;

    case 1062:
        /* RTCM 3.2
         * SSR GPS High Rate Clock Correction
         */
        unknown_name = "SSR GPS High Rate Clock Correction";
        break;

    case 1063:
        /* RTCM 3.2
         * SSR GLO Orbit Correction
         */
        unknown_name = "SSR GLO Orbit Correction";
        break;

    case 1064:
        /* RTCM 3.2
         * SSR GLO Clock Correction
         */
        unknown_name = "SSR GLO Clock Correction";
        break;

    case 1065:
        /* RTCM 3.2
         * SSR GLO Code Correction
         */
        unknown_name = "SSR GLO ode Correction";
        break;

    case 1066:
        /* RTCM 3.2
         * SSR GLO Combined Orbit and Clock Correction
         */
        unknown_name = "SSR GLO Combined Orbit and Clock Correction";
        break;

    case 1067:
        /* RTCM 3.2
         * SSR GLO URA
         */
        unknown_name = "SSR GLO URA";
        break;

    case 1068:
        /* RTCM 3.2
         * SSR GPS High Rate Clock Correction
         */
        unknown_name = "SSR GLO High Rate Clock Correction";
        break;

    case 1070:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1071:
        /* RTCM 3.2
         * GPS Multi Signal Message 1
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GPS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 1;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GPS MSM 1";
        break;

    case 1072:
        /* RTCM 3.2
         * GPS Multi Signal Message 2
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GPS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 2;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GPS MSM 2";
        break;

    case 1073:
        /* RTCM 3.2
         * GPS Multi Signal Message 3
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GPS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 3;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GPS MSM 3";
        break;

    case 1074:
        /* RTCM 3.2
         * GPS Multi Signal Message 4
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GPS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 4;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GPS MSM 4";
        break;

    case 1075:
        /* RTCM 3.2
         * GPS Multi Signal Message 5
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GPS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 5;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GPS MSM 5";
        break;

    case 1076:
        /* RTCM 3.2
         * GPS Multi Signal Message 6
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GPS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 6;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GPS MSM 6";
        break;

    case 1077:
        /* RTCM 3.2 - 1077
         * GPS Multi Signal Message 7
         * Full GPS pseudo-ranges, carrier phases, Doppler and
         * signal strength (high resolution)
         * min length 438
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GPS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 7;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GPS MSM7";
        break;

    case 1078:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1079:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1080:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1081:
        /* RTCM 3.2
         * GLONASS Multi Signal Message 1
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GLO;
        rtcm->rtcmtypes.rtcm3_msm.msm = 1;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GLO MSM 1";
        break;

    case 1082:
        /* RTCM 3.2
         * GLONASS Multi Signal Message 2
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GLO;
        rtcm->rtcmtypes.rtcm3_msm.msm = 2;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GLO MSM 2";
        break;

    case 1083:
        /* RTCM 3.2
         * GLONASS Multi Signal Message 4
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GLO;
        rtcm->rtcmtypes.rtcm3_msm.msm = 3;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GLO MSM 3";
        break;

    case 1084:
        /* RTCM 3.2
         * GLONASS Multi Signal Message 4
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GLO;
        rtcm->rtcmtypes.rtcm3_msm.msm = 4;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GLO MSM 4";
        break;

    case 1085:
        /* RTCM 3.2
         * GLONASS Multi Signal Message 5
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GLO;
        rtcm->rtcmtypes.rtcm3_msm.msm = 5;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GLO MSM 5";
        break;

    case 1086:
        /* RTCM 3.2
         * GLONASS Multi Signal Message 6
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GLO;
        rtcm->rtcmtypes.rtcm3_msm.msm = 6;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GLO MSM 6";
        break;

    case 1087:
        /* RTCM 3.2 - 1087
         * GLONASS Multi Signal Message 7
         * Full GLONASS pseudo-ranges, carrier phases, Doppler and
         * signal strength (high resolution)
         * length 417 or 427
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GLO;
        rtcm->rtcmtypes.rtcm3_msm.msm = 7;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GLO MSM 7";
        break;

    case 1088:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1089:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1090:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1091:
        /* RTCM 3.2
         * Galileo Multi Signal Message 1
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GAL;
        rtcm->rtcmtypes.rtcm3_msm.msm = 1;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GAL MSM 1";
        break;

    case 1092:
        /* RTCM 3.2
         * Galileo Multi Signal Message 2
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GAL;
        rtcm->rtcmtypes.rtcm3_msm.msm = 2;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GAL MSM 2";
        break;

    case 1093:
        /* RTCM 3.2
         * Galileo Multi Signal Message 3
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GAL;
        rtcm->rtcmtypes.rtcm3_msm.msm = 3;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GAL MSM 3";
        break;

    case 1094:
        /* RTCM 3.2
         * Galileo Multi Signal Message 4
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GAL;
        rtcm->rtcmtypes.rtcm3_msm.msm = 4;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GAL MSM 4";
        break;

    case 1095:
        /* RTCM 3.2
         * Galileo Multi Signal Message 5
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GAL;
        rtcm->rtcmtypes.rtcm3_msm.msm = 5;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GAL MSM 5";
        break;

    case 1096:
        /* RTCM 3.2
         * Galileo Multi Signal Message 6
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GAL;
        rtcm->rtcmtypes.rtcm3_msm.msm = 6;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GAL MSM 6";
        break;

    case 1097:
        /* RTCM 3.2 - 1097
         * Galileo Multi Signal Message 7
         * Full Galileo pseudo-ranges, carrier phases, Doppler and
         * signal strength (high resolution)
         * length 96
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_GAL;
        rtcm->rtcmtypes.rtcm3_msm.msm = 7;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "GAL MSM 7";
        break;

    case 1098:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1099:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1100:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1101:
        /* RTCM 3.3
         * SBAS Multi Signal Message 1
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_SBAS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 1;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "SBAS MSM 1";
        break;

    case 1102:
        /* RTCM 3.3
         * SBAS Multi Signal Message 2
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_SBAS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 2;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "SBAS MSM 2";
        break;

    case 1103:
        /* RTCM 3.3
         * SBAS Multi Signal Message 3
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_SBAS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 3;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "SBAS MSM 3";
        break;

    case 1104:
        /* RTCM 3.3
         * SBAS Multi Signal Message 4
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_SBAS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 4;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "SBAS MSM 4";
        break;

    case 1105:
        /* RTCM 3.3
         * SBAS Multi Signal Message 5
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_SBAS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 5;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "SBAS MSM 5";
        break;

    case 1106:
        /* RTCM 3.3
         * SBAS Multi Signal Message 6
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_SBAS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 6;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "SBAS MSM 6";
        break;

    case 1107:
        /* RTCM 3.3 - 1107
         * 'Multiple Signal Message
         * Full SBAS pseudo-ranges, carrier phases, Doppler and
         * signal strength (high resolution)
         * length 96
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_SBAS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 7;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "SBAS MSM 7";
        break;

    case 1108:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1109:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1110:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1111:
        /* RTCM 3.3
         * QZSS Multi Signal Message 1
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_QZSS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 1;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "QZSS MSM 1";
        break;

    case 1112:
        /* RTCM 3.3
         * QZSS Multi Signal Message 2
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_QZSS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 2;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "QZSS MSM 2";
        break;

    case 1113:
        /* RTCM 3.3
         * QZSS Multi Signal Message 3
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_QZSS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 3;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "QZSS MSM 3";
        break;

    case 1114:
        /* RTCM 3.3
         * QZSS Multi Signal Message 4
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_QZSS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 4;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "QZSS MSM 4";
        break;

    case 1115:
        /* RTCM 3.3
         * QZSS Multi Signal Message 5
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_QZSS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 5;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "QZSS MSM 5";
        break;

    case 1116:
        /* RTCM 3.3
         * QZSS Multi Signal Message 6
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_QZSS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 6;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "QZSS MSM 6";
        break;

    case 1117:
        /* RTCM 3.3
         * QZSS Multi Signal Message 7
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_QZSS;
        rtcm->rtcmtypes.rtcm3_msm.msm = 7;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "QZSS MSM 7";
        break;

    case 1118:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1119:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1120:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1121:
        /* RTCM 3.2 A.1
         * BD Multi Signal Message 1
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_BD;
        rtcm->rtcmtypes.rtcm3_msm.msm = 1;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "BD MSM 1";
        break;

    case 1122:
        /* RTCM 3.2 A.1
         * BD Multi Signal Message 2
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_BD;
        rtcm->rtcmtypes.rtcm3_msm.msm = 2;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "BD MSM 2";
        break;

    case 1123:
        /* RTCM 3.2 A.1
         * BD Multi Signal Message 3
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_BD;
        rtcm->rtcmtypes.rtcm3_msm.msm = 3;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "BD MSM 3";
        break;

    case 1124:
        /* RTCM 3.2 A.1
         * BD Multi Signal Message 4
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_BD;
        rtcm->rtcmtypes.rtcm3_msm.msm = 4;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "BD MSM 4";
        break;

    case 1125:
        /* RTCM 3.2 A.1
         * BeiDou Multi Signal Message 5
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_BD;
        rtcm->rtcmtypes.rtcm3_msm.msm = 5;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "BD MSM 5";
        break;

    case 1126:
        /* RTCM 3.2 A.1
         * BeiDou Multi Signal Message 6
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_BD;
        rtcm->rtcmtypes.rtcm3_msm.msm = 6;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "BD MSM 6";
        break;

    case 1127:
        /* RTCM 3.2 A.1
         * BeiDou Multi Signal Message 7
         */
        rtcm->rtcmtypes.rtcm3_msm.gnssid = GNSSID_BD;
        rtcm->rtcmtypes.rtcm3_msm.msm = 7;
        unknown = rtcm3_decode_msm(context, rtcm, buf);
        unknown_name = "BD MSM 7";
        break;

    case 1128:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1229:
        /* RTCM 3.x
         * Reserved for MSM
         */
        unknown_name = "Reserved for MSM";
        break;

    case 1230:
        /* RTCM 3.2
         * GLONASS L1 and L2, C/A and P, Code-Phase Biases.
         */
        unknown_name = "GLO L1 and L2 Code-Phase Biases";
        unknown = false;
        rtcm->rtcmtypes.rtcm3_1230.station_id = (unsigned short)ugrab(12);
        rtcm->rtcmtypes.rtcm3_1230.bias_indicator = (unsigned char)ugrab(1);
        (void)ugrab(1);         // reserved
        rtcm->rtcmtypes.rtcm3_1230.signals_mask = (unsigned char)ugrab(3);
        // actual mask order is undocumented...
        if (1 & rtcm->rtcmtypes.rtcm3_1230.signals_mask) {
            rtcm->rtcmtypes.rtcm3_1230.l1_ca_bias = ugrab(16);
        }
        if (2 & rtcm->rtcmtypes.rtcm3_1230.signals_mask) {
            rtcm->rtcmtypes.rtcm3_1230.l1_p_bias = ugrab(16);
        }
        if (4 & rtcm->rtcmtypes.rtcm3_1230.signals_mask) {
            rtcm->rtcmtypes.rtcm3_1230.l2_ca_bias = ugrab(16);
        }
        if (8 & rtcm->rtcmtypes.rtcm3_1230.signals_mask) {
            rtcm->rtcmtypes.rtcm3_1230.l2_p_bias = ugrab(16);
        }
        break;

    // Message Types 4001 — 4060 Are Reserved

    case 4062:
        /* RTCM 3.3
         * Geely Proprietary
         */
        unknown_name = "Geely Proprietary";
        break;

    case 4063:
        /* RTCM 3.3
         * CHC Navigation (CHCNAV) Proprietary
         */
        unknown_name = "CHC Navigation (CHCNAV) Proprietary";
        break;

    case 4064:
        /* RTCM 3.3
         * NTLab Proprietary
         */
        unknown_name = "NTLab Proprietary";
        break;

    case 4065:
        /* RTCM 3.3
         * Allystar Technology (Shenzhen) Co. Ltd. Proprietary
         */
        unknown_name = "Allystar Technology (Shenzhen) Co. Ltd. Proprietary";
        break;

    case 4066:
        /* RTCM 3.3
         * Lantmateriet Proprietary
         */
        unknown_name = "Lantmateriet Proprietary";
        break;

    case 4067:
        /* RTCM 3.x
         * China Transport telecommunications & Information Center Proprietary
         */
        unknown_name = "China Transport telecommunications & Information "
                       "Center Proprietary";
        break;

    case 4068:
        /* RTCM 3.3
         * Qianxun Location Networks Co. Ltd Proprietary
         */
        unknown_name = "Qianxun Location Networks Co. Ltd Proprietary";
        break;

    case 4069:
        /* RTCM 3.3
         * VERIPOS Ltd Proprietary
         */
        unknown_name = "VERIPOS Ltd Proprietary";
        break;

    case 4070:
        /* RTCM 3.3
         * Wuhan MengXin Technology
         */
        unknown_name = "Wuhan MengXin Technology Proprietary";
        break;

    case 4071:
        /* RTCM 3.3
         * Wuhan Navigation and LBS
         */
        unknown_name = "Wuhan Navigation and LBS Proprietary";
        break;

    case 4072:
        /* RTCM 3.x
         * u-blox Proprietary
         * Mitsubishi Electric Corp Proprietary
         * 4072.0 Reference station PVT (u-blox proprietary)
         * 4072.1 Additional reference station information (u-blox proprietary)
         */
        unknown_name = "u-blox Proprietary";
        break;

    case 4073:
        /* RTCM 3.x
         * Unicore Communications Proprietary
         */
        unknown_name = "Alberding GmbH Proprietary";
        break;

    case 4075:
        /* RTCM 3.x
         * Alberding GmbH Proprietary
         */
        unknown_name = "Alberding GmbH Proprietary";
        break;

    case 4076:
        /* RTCM 3.x
         * International GNSS Service Proprietary
         */
        unknown_name = "International GNSS Service Proprietary";
        break;

    case 4077:
        /* RTCM 3.x
         * Hemisphere GNSS Proprietary
         */
        unknown_name = "Hemisphere GNSS Proprietary";
        break;

    case 4078:
        /* RTCM 3.x
         * ComNav Technology Proprietary
         */
        unknown_name = "ComNav Technology Proprietary";
        break;

    case 4079:
        /* RTCM 3.x
         * SubCarrier Systems Corp Proprietary
         */
        unknown_name = "SubCarrier Systems Corp Proprietary";
        break;

    case 4080:
        /* RTCM 3.x
         * NavCom Technology, Inc.
         */
        unknown_name = "NavCom Technology, Inc.";
        break;

    case 4081:
        /* RTCM 3.x
         * Seoul National Universtiry GNSS Lab Proprietary
         */
        unknown_name = "Seoul National Universtiry GNSS Lab Proprietery";
        break;

    case 4082:
        /* RTCM 3.x
         * Cooperative Research Centre for Spatial Information Proprietary
         */
        unknown_name = "Cooperative Research Centre for Spatial Information "
                       "Proprietary";
        break;

    case 4083:
        /* RTCM 3.x
         * German Aerospace Center Proprietary
         */
        unknown_name = "German Aerospace Center Proprietary";
        break;

    case 4084:
        /* RTCM 3.x
         * Geodetics Inc Proprietary
         */
        unknown_name = "Geodetics Inc Proprietary";
        break;

    case 4085:
        /* RTCM 3.x
         * European GNSS Supervisory Authority Proprietary
         */
        unknown_name = "European GNSS Supervisory Authority Proprietary";
        break;

    case 4086:
        /* RTCM 3.x
         * InPosition GmbH Proprietary
         */
        unknown_name = "InPosition GmbH Proprietary";
        break;

    case 4087:
        /* RTCM 3.x
         * Fugro Proprietary
         */
        unknown_name = "Fugro Proprietary";
        break;

    case 4088:
        /* RTCM 3.x
         * IfEN GmbH Proprietary
         */
        unknown_name = "IfEN GmbH Proprietary";
        break;

    case 4089:
        /* RTCM 3.x
         * Septentrio Satellite Navigation Proprietary
         */
        unknown_name = "Septentrio Satellite Navigation Proprietary";
        break;

    case 4090:
        /* RTCM 3.x
         * Geo++ Proprietary
         */
        unknown_name = "Geo++ Proprietary";
        break;

    case 4091:
        /* RTCM 3.x
         * Topcon Positioning Systems Proprietary
         */
        unknown_name = "Topcon Positioning Systems Proprietary";
        break;

    case 4092:
        /* RTCM 3.x
         * Leica Geosystems Proprietary
         */
        unknown_name = "Leica Geosystems Proprietary";
        break;

    case 4093:
        /* RTCM 3.x
         * NovAtel Proprietary
         */
        unknown_name = "NovAtel Pr.orietary";
        break;

    case 4094:
        /* RTCM 3.x
         * Trimble Proprietary
         */
        unknown_name = "Trimble Proprietary";
        break;

    case 4095:
        /* RTCM 3.x
         * Ashtech/Magellan Proprietary
         */
        unknown_name = "Ashtech/Magellan Proprietary";
        break;

    default:
        break;
    }
#undef RANGEDIFF
#undef GPS_PSEUDORANGE
#undef sgrab
#undef ugrab
    if (unknown) {
        /*
         * Leader bytes, message length, and checksum won't be copied.
         * The first 12 bits of the copied payload will be the type field.
         */
        memcpy(rtcm->rtcmtypes.data, buf + 3, rtcm->length);
        if (NULL == unknown_name) {
            GPSD_LOG(LOG_PROG, &context->errout,
                     "RTCM3: unknown type %d, length %d\n",
                     rtcm->type, rtcm->length);
        } else {
            GPSD_LOG(LOG_PROG, &context->errout,
                     "RTCM3: %s (type %d), length %d\n",
                     unknown_name, rtcm->type, rtcm->length);
        }
    }
}

// *INDENT-ON*

#endif  // RTCM104V3_ENABLE

// vim: set expandtab shiftwidth=4
