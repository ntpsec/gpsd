/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#ifdef HAVE_GETOPT_LONG
       #include <getopt.h>   // for getopt_long()
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>

#include "../include/gpsd.h"         // for MAX_PACKET_LENGTH, etc
#include "../include/bits.h"
#include "../include/gps_json.h"
#include "../include/strfuncs.h"

static int verbose = 0;
static bool scaled = true;
static bool json = true;
static bool pseudonmea = false;
static bool spartn = false;
static bool split24 = false;
static bool minlength = false;
static unsigned int ntypes = 0;
static unsigned int typelist[32];
static struct gps_context_t context;

/**************************************************************************
 *
 * Generic machinery
 *
 **************************************************************************/

#ifdef AIVDM_ENABLE
static const char *raw_hexdump(char *scbuf, size_t scbuflen, int structured,
                               const unsigned char *binbuf,
                               size_t binbuflen)
{
    if (!structured) {
        return gps_hexdump(scbuf, scbuflen, binbuf, binbuflen);
    }

// Data parsed as structured doesn't have correct raw data
#ifndef SQUELCH_ENABLE
    size_t len =
        (size_t) ((binbuflen >
                   MAX_PACKET_LENGTH) ? MAX_PACKET_LENGTH : binbuflen) * 2;
    if (len > scbuflen - 1) {
        len = scbuflen - 1;
    }

    memset(scbuf, 'x', len);
    scbuf[len] = '\0';
#else  // SQUELCH defined
    scbuf[0] = '\0';
#endif  // SQUELCH_ENABLE
    return scbuf;
}

static void aivdm_csv_dump(struct ais_t *ais, char *buf, size_t buflen)
{
    char scratchbuf[MAX_PACKET_LENGTH*2+1];
    bool imo = false;

    (void)snprintf(buf, buflen, "%u|%u|%09u|", ais->type, ais->repeat,
                   ais->mmsi);
    switch (ais->type) {
    case 1:                     // Position Report
    case 2:
    case 3:
        str_appendf(buf, buflen,
                    "%u|%d|%u|%u|%d|%d|%u|%u|%u|0x%x|%u|0x%x",
                    ais->type1.status,
                    ais->type1.turn,
                    ais->type1.speed,
                    (unsigned int) ais->type1.accuracy,
                    ais->type1.lon,
                    ais->type1.lat,
                    ais->type1.course,
                    ais->type1.heading,
                    ais->type1.second,
                    ais->type1.maneuver,
                    (unsigned int) ais->type1.raim, ais->type1.radio);
        break;
    case 4:                     // Base Station Report
    case 11:                    // UTC/Date Response
        str_appendf(buf, buflen,
                    "%04u-%02u-%02uT%02u:%02u:%02uZ|%u|%d|%d|%u|%u|0x%x",
                    ais->type4.year,
                    ais->type4.month,
                    ais->type4.day,
                    ais->type4.hour,
                    ais->type4.minute,
                    ais->type4.second,
                    (unsigned int) ais->type4.accuracy,
                    ais->type4.lon,
                    ais->type4.lat,
                    ais->type4.epfd,
                    (unsigned int) ais->type4.raim, ais->type4.radio);
        break;
    case 5:                     // Ship static and voyage related data
        str_appendf(buf, buflen,
                    "%u|%u|%s|%s|%u|%u|%u|%u|%u|%u|%02u-%02uT%02u:%02uZ"
                    "|%u|%s|%u",
                    ais->type5.imo,
                    ais->type5.ais_version,
                    ais->type5.callsign,
                    ais->type5.shipname,
                    ais->type5.shiptype,
                    ais->type5.to_bow,
                    ais->type5.to_stern,
                    ais->type5.to_port,
                    ais->type5.to_starboard,
                    ais->type5.epfd,
                    ais->type5.month,
                    ais->type5.day,
                    ais->type5.hour,
                    ais->type5.minute,
                    ais->type5.draught,
                    ais->type5.destination, ais->type5.dte);
        break;
    case 6:                     // Binary Message
        str_appendf(buf, buflen,
                    "%u|%u|%u|%u|%u",
                    ais->type6.seqno,
                    ais->type6.dest_mmsi,
                    (unsigned int) ais->type6.retransmit,
                    ais->type6.dac,
                    ais->type6.fid);
        switch(ais->type6.dac) {
        case 235:                       // UK
        case 250:                       // Rep. Of Ireland
            switch(ais->type6.fid) {
            case 10:            // GLA - AtoN monitoring
                str_appendf(buf, buflen,
                           "|%u|%u|%u|%u|%u|%u|%u|%u",
                           ais->type6.dac235fid10.ana_int,
                           ais->type6.dac235fid10.ana_ext1,
                           ais->type6.dac235fid10.ana_ext2,
                           ais->type6.dac235fid10.racon,
                           ais->type6.dac235fid10.light,
                           (unsigned int)ais->type6.dac235fid10.alarm,
                           ais->type6.dac235fid10.stat_ext,
                           (unsigned int)ais->type6.dac235fid10.off_pos);
                imo = true;
                break;
            }
            break;
        }
        if (!imo) {
            str_appendf(buf, buflen,
                        "|%zd:%s",
                        ais->type6.bitcount,
                        raw_hexdump(scratchbuf, sizeof(scratchbuf),
                                    ais->type6.structured,
                                    (const unsigned char *)ais->type6.bitdata,
                                    BITS_TO_BYTES(ais->type6.bitcount)));
        }
        break;
    case 7:                     // Binary Acknowledge
    case 13:                    // Safety Related Acknowledge
        str_appendf(buf, buflen,
                       "%u|%u|%u|%u",
                       ais->type7.mmsi1,
                       ais->type7.mmsi2, ais->type7.mmsi3, ais->type7.mmsi4);
        break;
    case 8:                     // Binary Broadcast Message
        str_appendf(buf, buflen, "%u|%u", ais->type8.dac, ais->type8.fid);
        switch(ais->type8.dac) {
        case 1:                 // International
            switch(ais->type8.fid) {
            case 11:            // IMO236 - Met/Hydro message
                str_appendf(buf, buflen,
                            "|%d|%d|%02uT%02u:%02uZ|%u|%u|%u|%u|%u|%u|%u"
                            "|%u|%u|%u|%d|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u"
                            "|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u",
                            ais->type8.dac1fid11.lon,
                            ais->type8.dac1fid11.lat,
                            ais->type8.dac1fid11.day,
                            ais->type8.dac1fid11.hour,
                            ais->type8.dac1fid11.minute,
                            ais->type8.dac1fid11.wspeed,
                            ais->type8.dac1fid11.wgust,
                            ais->type8.dac1fid11.wdir,
                            ais->type8.dac1fid11.wgustdir,
                            ais->type8.dac1fid11.airtemp,
                            ais->type8.dac1fid11.humidity,
                            ais->type8.dac1fid11.dewpoint,
                            ais->type8.dac1fid11.pressure,
                            ais->type8.dac1fid11.pressuretend,
                            ais->type8.dac1fid11.visibility,
                            ais->type8.dac1fid11.waterlevel,
                            ais->type8.dac1fid11.leveltrend,
                            ais->type8.dac1fid11.cspeed,
                            ais->type8.dac1fid11.cdir,
                            ais->type8.dac1fid11.cspeed2,
                            ais->type8.dac1fid11.cdir2,
                            ais->type8.dac1fid11.cdepth2,
                            ais->type8.dac1fid11.cspeed3,
                            ais->type8.dac1fid11.cdir3,
                            ais->type8.dac1fid11.cdepth3,
                            ais->type8.dac1fid11.waveheight,
                            ais->type8.dac1fid11.waveperiod,
                            ais->type8.dac1fid11.wavedir,
                            ais->type8.dac1fid11.swellheight,
                            ais->type8.dac1fid11.swellperiod,
                            ais->type8.dac1fid11.swelldir,
                            ais->type8.dac1fid11.seastate,
                            ais->type8.dac1fid11.watertemp,
                            ais->type8.dac1fid11.preciptype,
                            ais->type8.dac1fid11.salinity,
                            ais->type8.dac1fid11.ice);
                imo = true;
                break;
            case 31:            // IMO289 - Met/Hydro message
                str_appendf(buf, buflen,
                            "|%d|%d|%02uT%02u:%02uZ|%u|%u|%u|%u|%d|%u|%d|%u"
                            "|%u|%u|%d|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u"
                            "|%u|%u|%u|%u|%d|%u|%u|%u",
                            ais->type8.dac1fid31.lon,
                            ais->type8.dac1fid31.lat,
                            ais->type8.dac1fid31.day,
                            ais->type8.dac1fid31.hour,
                            ais->type8.dac1fid31.minute,
                            ais->type8.dac1fid31.wspeed,
                            ais->type8.dac1fid31.wgust,
                            ais->type8.dac1fid31.wdir,
                            ais->type8.dac1fid31.wgustdir,
                            ais->type8.dac1fid31.airtemp,
                            ais->type8.dac1fid31.humidity,
                            ais->type8.dac1fid31.dewpoint,
                            ais->type8.dac1fid31.pressure,
                            ais->type8.dac1fid31.pressuretend,
                            ais->type8.dac1fid31.visibility,
                            ais->type8.dac1fid31.waterlevel,
                            ais->type8.dac1fid31.leveltrend,
                            ais->type8.dac1fid31.cspeed,
                            ais->type8.dac1fid31.cdir,
                            ais->type8.dac1fid31.cspeed2,
                            ais->type8.dac1fid31.cdir2,
                            ais->type8.dac1fid31.cdepth2,
                            ais->type8.dac1fid31.cspeed3,
                            ais->type8.dac1fid31.cdir3,
                            ais->type8.dac1fid31.cdepth3,
                            ais->type8.dac1fid31.waveheight,
                            ais->type8.dac1fid31.waveperiod,
                            ais->type8.dac1fid31.wavedir,
                            ais->type8.dac1fid31.swellheight,
                            ais->type8.dac1fid31.swellperiod,
                            ais->type8.dac1fid31.swelldir,
                            ais->type8.dac1fid31.seastate,
                            ais->type8.dac1fid31.watertemp,
                            ais->type8.dac1fid31.preciptype,
                            ais->type8.dac1fid31.salinity,
                            ais->type8.dac1fid31.ice);
                imo = true;
                break;
            }
            break;
        }
        if (!imo) {
            str_appendf(buf, buflen,
                       "|%zd:%s",
                       ais->type8.bitcount,
                       raw_hexdump(scratchbuf, sizeof(scratchbuf),
                                   ais->type8.structured,
                                   (const unsigned char *)ais->type8.bitdata,
                                   BITS_TO_BYTES(ais->type8.bitcount)));
        }
        break;
    case 9:
        str_appendf(buf, buflen,
                    "%u|%u|%u|%d|%d|%u|%u|0x%x|%u|%u|0x%x",
                    ais->type9.alt,
                    ais->type9.speed,
                    (unsigned int) ais->type9.accuracy,
                    ais->type9.lon,
                    ais->type9.lat,
                    ais->type9.course,
                    ais->type9.second,
                    ais->type9.regional,
                    ais->type9.dte,
                    (unsigned int) ais->type9.raim, ais->type9.radio);
        break;
    case 10:                    // UTC/Date Inquiry
        str_appendf(buf, buflen, "%u", ais->type10.dest_mmsi);
        break;
    case 12:                    // Safety Related Message
        str_appendf(buf, buflen,
                    "%u|%u|%u|%s",
                    ais->type12.seqno,
                    ais->type12.dest_mmsi,
                    (unsigned int) ais->type12.retransmit, ais->type12.text);
        break;
    case 14:                    // Safety Related Broadcast Message
        str_appendf(buf, buflen, "%s", ais->type14.text);
        break;
    case 15:
        str_appendf(buf, buflen,
                    "%u|%u|%u|%u|%u|%u|%u|%u",
                    ais->type15.mmsi1,
                    ais->type15.type1_1,
                    ais->type15.offset1_1,
                    ais->type15.type1_2,
                    ais->type15.offset1_2,
                    ais->type15.mmsi2,
                    ais->type15.type2_1, ais->type15.offset2_1);
        break;
    case 16:
        str_appendf(buf, buflen,
                    "%u|%u|%u|%u|%u|%u",
                    ais->type16.mmsi1,
                    ais->type16.offset1,
                    ais->type16.increment1,
                    ais->type16.mmsi2,
                    ais->type16.offset2, ais->type16.increment2);
        break;
    case 17:
        str_appendf(buf, buflen,
                    "%d|%d|%zd:%s",
                    ais->type17.lon,
                    ais->type17.lat,
                    ais->type17.bitcount,
                    gps_hexdump(scratchbuf, sizeof(scratchbuf),
                                (const unsigned char *)ais->type17.bitdata,
                                BITS_TO_BYTES(ais->type17.bitcount)));
        break;
    case 18:
        str_appendf(buf, buflen,
                    "%u|%u|%u|%d|%d|%u|%u|%u|0x%x|%u|%u|%u|%u|%u|%u|0x%x",
                    ais->type18.reserved,
                    ais->type18.speed,
                    (unsigned int) ais->type18.accuracy,
                    ais->type18.lon,
                    ais->type18.lat,
                    ais->type18.course,
                    ais->type18.heading,
                    ais->type18.second,
                    ais->type18.regional,
                    (unsigned int) ais->type18.cs,
                    (unsigned int) ais->type18.display,
                    (unsigned int) ais->type18.dsc,
                    (unsigned int) ais->type18.band,
                    (unsigned int) ais->type18.msg22,
                    (unsigned int) ais->type18.raim, ais->type18.radio);
        break;
    case 19:
        str_appendf(buf, buflen,
                    "%u|%u|%u|%d|%d|%u|%u|%u|0x%x|%s|%u|%u|%u|%u|%u|%u|%u|%u|%u",
                    ais->type19.reserved,
                    ais->type19.speed,
                    (unsigned int) ais->type19.accuracy,
                    ais->type19.lon,
                    ais->type19.lat,
                    ais->type19.course,
                    ais->type19.heading,
                    ais->type19.second,
                    ais->type19.regional,
                    ais->type19.shipname,
                    ais->type19.shiptype,
                    ais->type19.to_bow,
                    ais->type19.to_stern,
                    ais->type19.to_port,
                    ais->type19.to_starboard,
                    ais->type19.epfd,
                    (unsigned int) ais->type19.raim,
                    ais->type19.dte, (unsigned int) ais->type19.assigned);
        break;
    case 20:                    // Data Link Management Message
        str_appendf(buf, buflen,
                    "%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u|%u",
                    ais->type20.offset1,
                    ais->type20.number1,
                    ais->type20.timeout1,
                    ais->type20.increment1,
                    ais->type20.offset2,
                    ais->type20.number2,
                    ais->type20.timeout2,
                    ais->type20.increment2,
                    ais->type20.offset3,
                    ais->type20.number3,
                    ais->type20.timeout3,
                    ais->type20.increment3,
                    ais->type20.offset4,
                    ais->type20.number4,
                    ais->type20.timeout4, ais->type20.increment4);
        break;
    case 21:                    // Aid to Navigation
        str_appendf(buf, buflen,
                    "%u|%s|%u|%d|%d|%u|%u|%u|%u|%u|%u|%u|0x%x|%u|%u",
                    ais->type21.aid_type,
                    ais->type21.name,
                    (unsigned int) ais->type21.accuracy,
                    ais->type21.lon,
                    ais->type21.lat,
                    ais->type21.to_bow,
                    ais->type21.to_stern,
                    ais->type21.to_port,
                    ais->type21.to_starboard,
                    ais->type21.epfd,
                    ais->type21.second,
                    ais->type21.regional,
                    (unsigned int) ais->type21.off_position,
                    (unsigned int) ais->type21.raim,
                    (unsigned int) ais->type21.virtual_aid);
        break;
    case 22:                    // Channel Management
        if (ais->type22.addressed) {
            str_appendf(buf, buflen,
                        "%u|%u|%u|%u|%u|%u|%u|%u|%u|%u",
                        ais->type22.channel_a,
                        ais->type22.channel_b,
                        ais->type22.txrx,
                        (unsigned int) ais->type22.power,
                        ais->type22.mmsi.dest1,
                        ais->type22.mmsi.dest2,
                        (unsigned int) ais->type22.addressed,
                        (unsigned int) ais->type22.band_a,
                        (unsigned int) ais->type22.band_b,
                        ais->type22.zonesize);
        } else {
            str_appendf(buf, buflen,
                        "%u|%u|%u|%u|%d|%d|%d|%d|%u|%u|%u|%u",
                        ais->type22.channel_a,
                        ais->type22.channel_b,
                        ais->type22.txrx,
                        (unsigned int) ais->type22.power,
                        ais->type22.area.ne_lon,
                        ais->type22.area.ne_lat,
                        ais->type22.area.sw_lon,
                        ais->type22.area.sw_lat,
                        (unsigned int)ais->type22.addressed,
                        (unsigned int)ais->type22.band_a,
                        (unsigned int)ais->type22.band_b,
                        ais->type22.zonesize);
        }
        break;
    case 23:                    // Group Management Command
        str_appendf(buf, buflen,
                    "%d|%d|%d|%d|%u|%u|%u|%u|%u",
                    ais->type23.ne_lon,
                    ais->type23.ne_lat,
                    ais->type23.sw_lon,
                    ais->type23.sw_lat,
                    ais->type23.stationtype,
                    ais->type23.shiptype,
                    ais->type23.txrx,
                    ais->type23.interval, ais->type23.quiet);
        break;
    case 24:                    // Class B CS Static Data Report
        str_appendf(buf, buflen, "%s|", ais->type24.shipname);
        str_appendf(buf, buflen, "%u|", ais->type24.shiptype);
        str_appendf(buf, buflen, "%s|", ais->type24.vendorid);
        str_appendf(buf, buflen, "%u|", ais->type24.model);
        str_appendf(buf, buflen, "%u|", ais->type24.serial);
        str_appendf(buf, buflen, "%s|", ais->type24.callsign);
        if (AIS_AUXILIARY_MMSI(ais->mmsi)) {
            str_appendf(buf, buflen, "%u", ais->type24.mothership_mmsi);
        } else {
            str_appendf(buf, buflen,
                        "%u|%u|%u|%u",
                        ais->type24.dim.to_bow,
                        ais->type24.dim.to_stern,
                        ais->type24.dim.to_port,
                        ais->type24.dim.to_starboard);
        }
        break;
    case 25:                    // Binary Message, Single Slot
        str_appendf(buf, buflen,
                    "%u|%u|%u|%u|%zd:%s",
                    (unsigned int) ais->type25.addressed,
                    (unsigned int) ais->type25.structured,
                    ais->type25.dest_mmsi,
                    ais->type25.app_id,
                    ais->type25.bitcount,
                    gps_hexdump(scratchbuf, sizeof(scratchbuf),
                                (const unsigned char *)ais->type25.bitdata,
                                BITS_TO_BYTES(ais->type25.bitcount)));
        break;
    case 26:                    // Binary Message, Multiple Slot
        str_appendf(buf, buflen,
                    "%u|%u|%u|%u|%zd:%s:%u",
                    (unsigned int) ais->type26.addressed,
                    (unsigned int) ais->type26.structured,
                    ais->type26.dest_mmsi,
                    ais->type26.app_id,
                    ais->type26.bitcount,
                    gps_hexdump(scratchbuf, sizeof(scratchbuf),
                                (const unsigned char *)ais->type26.bitdata,
                                BITS_TO_BYTES(ais->type26.bitcount)),
                    ais->type26.radio);
        break;
    case 27:                    // Long Range AIS Broadcast message
        str_appendf(buf, buflen,
                    "%u|%u|%d|%d|%u|%u|%u|%u",
                    ais->type27.status,
                    (unsigned int)ais->type27.accuracy,
                    ais->type27.lon,
                    ais->type27.lat,
                    ais->type27.speed,
                    ais->type27.course,
                    (unsigned int)ais->type27.raim,
                    (unsigned int)ais->type27.gnss);
        break;
    default:
        str_appendf(buf, buflen, "unknown AIVDM message content.");
        break;
    }
    (void)strlcat(buf, "\r\n", buflen);
}
#endif

/* say whether a given message should be visible
 *
 * Return: False if this message should be filtered.
 */
static bool filter(gps_mask_t changed, struct gps_device_t *session)
{
    unsigned int i, t;

    if (0 == ntypes) {
        return true;
    }

    if (0 != (changed & AIS_SET)) {
        t = session->gpsdata.ais.type;
    } else if (0 != (changed & RTCM2_SET)) {
        t = session->gpsdata.rtcm2.type;
    } else if (0 != (changed & RTCM3_SET)) {
        t = session->gpsdata.rtcm3.type;
    } else {
        return true;
    }
    for (i = 0; i < ntypes; i++) {
        if (t == typelist[i]) {
            return true;
        }
    }
    return false;
}

// report pseudo-NMEA in appropriate circumstances
// FIXME: duplicated in gpsd/gpsd.c
static void pseudonmea_report(gps_mask_t changed, struct gps_device_t *device)
{
    if (GPS_PACKET_TYPE(device->lexer.type)
        && !TEXTUAL_PACKET_TYPE(device->lexer.type)) {
        char buf[MAX_PACKET_LENGTH * 3 + 2];

        if (0 != (changed & REPORT_IS)) {
            nmea_tpv_dump(device, buf, sizeof(buf));
            (void)fputs(buf, stdout);
        }

        if (0 != (changed & (SATELLITE_SET|USED_IS))) {
            nmea_sky_dump(device, buf, sizeof(buf));
            (void)fputs(buf, stdout);
        }

        if (0 != (changed & SUBFRAME_SET)) {
            nmea_subframe_dump(device, buf, sizeof(buf));
            (void)fputs(buf, stdout);
        }
#ifdef AIVDM_ENABLE
        if (0 != (changed & AIS_SET)) {
            nmea_ais_dump(device, buf, sizeof(buf));
            (void)fputs(buf, stdout);
        }
#endif  // AIVDM_ENABLE
    }
}

/* decode(): decode sensor data from fpin to dump format on fpout
 *
 * Return: void
 */
static void decode(FILE *fpin, FILE *fpout)
{
    struct gps_device_t session;
    struct gps_policy_t policy;
    size_t minima[PACKET_TYPES + 1];
    char buf[GPS_JSON_RESPONSE_MAX * 4];
    size_t i;

    //This looks like a good idea, but it breaks regression tests
    //(void)strlcpy(session.gpsdata.dev.path, "stdin",
    //              sizeof(session.gpsdata.dev.path));
    memset(&policy, '\0', sizeof(policy));
    policy.json = json;
    policy.scaled = scaled;
    policy.nmea = pseudonmea;

    gpsd_time_init(&context, time(NULL));
    context.readonly = true;
    gpsd_init(&session, &context, NULL);
    gpsd_clear(&session);
    if (spartn) {
        // enable SPARTN
        session.lexer.type_mask &=  ~PACKET_TYPEMASK(SPARTN_PACKET);
    }

    session.gpsdata.gps_fd = fileno(fpin);
    session.gpsdata.dev.baudrate = 38400;     // hack to enable subframes
    (void)strlcpy(session.gpsdata.dev.path,
                  "stdin",
                  sizeof(session.gpsdata.dev.path));
    for (i = 0; i < (sizeof(minima) / sizeof(minima[0])); i++) {
        minima[i] = MAX_PACKET_LENGTH + 1;
    }

    for (;;) {
        gps_mask_t changed = gpsd_poll(&session);

        if (ERROR_SET == changed ||
            NODATA_IS == changed) {
            break;
        }
        if (COMMENT_PACKET == session.lexer.type) {
            gpsd_set_century(&session);
        }
        if (1 <= verbose &&
            TEXTUAL_PACKET_TYPE(session.lexer.type)) {
            (void)fputs((char *)session.lexer.outbuffer, fpout);
        }
        if (session.lexer.outbuflen < minima[session.lexer.type+1]) {
            minima[session.lexer.type+1] = session.lexer.outbuflen;
        }
#ifdef __UNUSED__  // debug
        (void)fprintf(stderr, "decode(): json %d mask %s\n",
                      json, gps_maskdump(session.gpsdata.set));
        (void)fprintf(stderr, "decode(): json %d changed %s\n",
                      json, gps_maskdump(changed));
#endif
        // mask should match what's in gpsd/gpsd.c report_data()
        if (0 != (changed & (ATTITUDE_SET | LATLON_SET | MODE_SET))) {
            changed |= REPORT_IS;
        }
        if (0 == (changed & (AIS_SET | ATTITUDE_SET | GST_SET | DOP_SET |
                             IMU_SET | RAW_IS |  REPORT_IS| RTCM2_SET |
                             RTCM3_SET | SATELLITE_SET | SUBFRAME_SET))) {
            continue;
        }
        if (!filter(changed, &session)) {
            continue;
        }
        if (json) {
            if (0 != (changed & PASSTHROUGH_IS)) {
                (void)fputs((char *)session.lexer.outbuffer, fpout);
                (void)fputs("\n", fpout);
            } else {
                if (0 != (changed & AIS_SET)) {
                    if (24 == session.gpsdata.ais.type &&
                        both != session.gpsdata.ais.type24.part &&
                        !split24) {
                        continue;
                    }
                }
                json_data_report(changed, &session, &policy,
                                 buf, sizeof(buf));
                (void)fputs(buf, fpout);
            }
#ifdef AIVDM_ENABLE
        } else if (AIVDM_PACKET == session.lexer.type) {
            if (0 != (changed & AIS_SET)) {
                if (24 == session.gpsdata.ais.type &&
                    both != session.gpsdata.ais.type24.part &&
                    !split24) {
                    continue;
                }
                aivdm_csv_dump(&session.gpsdata.ais, buf, sizeof(buf));
                (void)fputs(buf, fpout);
            }
#endif  // AIVDM_ENABLE
        }
        if (policy.nmea) {
            pseudonmea_report(changed, &session);
        }
    }

    if (minlength) {
        for (i = 0; i < (sizeof(minima) / sizeof(minima[0])); i++) {
            // dump all minima, ignoring comments
            if (1 != i &&
                MAX_PACKET_LENGTH >= minima[i]) {
                const struct gps_type_t **dp;
                char *np = "Unknown";

                for (dp = gpsd_drivers; *dp; dp++) {
                    if ((int)(i - 1) == (*dp)->packet_type) {
                        np = (*dp)->type_name;
                        break;
                    }
                }
                printf("%s (%lu): %lu\n", np, i - 1, minima[i]);
            }
        }
    }
}

// JSON format on fpin to JSON on fpout - idempotency test
static void encode(FILE *fpin, FILE *fpout)
{
    char inbuf[BUFSIZ];
    struct gps_policy_t policy;
    struct gps_device_t session;
    int lineno = 0;

    memset(&policy, '\0', sizeof(policy));
    memset(&session, '\0', sizeof(session));
    session.context = &context;
    context.errout.debug = LOG_SHOUT;
    context.errout.label = "gpsdecode";
    (void)strlcpy(session.gpsdata.dev.path,
                  "stdin",
                  sizeof(session.gpsdata.dev.path));
    policy.json = true;
    policy.nmea = pseudonmea;
    /* Parsing is always made in unscaled mode,
     * this policy applies to the dumping */
    policy.scaled = scaled;

    while (NULL != fgets(inbuf, (int)sizeof(inbuf), fpin)) {
        int status;

        ++lineno;
        if ('#' == inbuf[0]) {
            continue;
        }
        status = libgps_json_unpack(inbuf, &session.gpsdata, NULL);
        if (0 != status) {
            (void)fprintf(stderr,
                          "gpsdecode: dying with status %d (%s) on line %d\n",
                          status, json_error_string(status), lineno);
            exit(EXIT_FAILURE);
        }
        json_data_report(session.gpsdata.set, &session, &policy,
                         inbuf, sizeof(inbuf));
        (void)fputs(inbuf, fpout);
    }
}

/* usage()
 * print usages, and exit
 */
static void usage(void)
{
    (void)fprintf(stderr,
          "Usage: gpsdecode [OPTIONS]\n"
          "\n"
#ifdef HAVE_GETOPT_LONG
          "  --ais              AIS dump format with an ASCII pipe separator.\n"
          "  --debug DEBUG      Set debug level.\n"
          "  --decode           Decode [default]\n"
          "  --encode           Encode.  JSON decode/encode\n"
          "  --help             Show this help, then exit\n"
          "  --json             JSON.\n"
          "  --minlength        Minimum length, no JSON.\n"
          "  --nmea             pseudo NMEA\n"
          "  --spartn           SPARTN enable.\n"
          "  --split24          split24\n"
          "  --types TYPES      Types\n"
          "  --unscaled         Unscaled\n"
          "  --verbose          Verbose.\n"
          "  --version          Print version, then exit\n"
#endif
          "  -?                 Show this help, then exit\n"
          "  -c                 AIS dump format with an ASCII pipe separator.\n"
          "  -D DEBUG           Set debug level.\n"
          "  -d                 Decode [default]\n"
          "  -e                 Encode.  JSON decode/encode\n"
          "  -h                 Show this help, then exit\n"
          "  -j                 JSON.\n"
          "  -m                 Minimum length, no JSON\n"
          "  -n                 pseudo NMEA\n"
          "  -s                 split24 \n"
          "  -t TYPES           Types, comma separated.\n"
          "  -u                 Unscaled\n"
          "  -V                 Print version and exit.\n"
          "  -v                 Verbose.\n"
          "  -z                 SPARTN enable.\n"
          "\n");
    exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
    const char *optstring = "?cdehjmnst:uvzVD:";
    enum { doencode, dodecode } mode = dodecode;
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"debug", required_argument, NULL, 'D'},
        {"decode", no_argument, NULL, 'd'},
        {"encode", no_argument, NULL, 'e'},
        {"help", no_argument, NULL, 'h'},
        {"json", no_argument, NULL, 'j'},
        {"minlength", no_argument, NULL, 'm'},
        {"nmea", no_argument, NULL, 'n'},
        {"nojson", no_argument, NULL, 'c'},
        {"spartn", no_argument, NULL, 'z'},
        {"split24", no_argument, NULL, 's'},
        {"types", required_argument, NULL, 't'},
        {"unscaled", no_argument, NULL, 'u' },
        {"verbose", no_argument, NULL, 'v' },
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif

    gps_context_init(&context, "gpsdecode");

    while (1) {
        int ch;
#ifdef HAVE_GETOPT_LONG
        ch = getopt_long(argc, argv, optstring, long_options, &option_index);
#else
        ch = getopt(argc, argv, optstring);
#endif

        if (-1 == ch) {
            break;
        }

        switch (ch) {
        case 'c':
            json = false;
            break;

        case 'd':
            mode = dodecode;
            break;

        case 'D':
            context.errout.debug = verbose = atoi(optarg);
            json_enable_debug(verbose - 2, stderr);
            break;

        case 'e':
            mode = doencode;
            break;

        case 'j':
            json = true;
            break;

        case 'm':
            minlength = true;
            json = false;
            break;

        case 'n':
            pseudonmea = true;
            break;

        case 's':
            split24 = true;
            break;

        case 't':
            typelist[ntypes++] = (unsigned int)atoi(strtok(optarg, ","));
            for(;;) {
                char *next = strtok(NULL, ",");
                if (NULL == next) {
                    break;
                }
                typelist[ntypes++] = (unsigned int)atoi(next);
            }
            break;

        case 'u':
            scaled = false;
            break;

        case 'v':
            verbose = 1;
            break;

        case 'z':
            // Dangerous!
            spartn = true;
            break;

        case 'V':
            (void)fprintf(stderr, "gpsdecode revision " VERSION
                          ", revision " REVISION "\n");
            exit(EXIT_SUCCESS);

        case '?':
            FALLTHROUGH
        case 'h':
            FALLTHROUGH
        default:
            usage();
        }
    }
    // argc -= optind;
    // argv += optind;

    if (2 < verbose) {
        int cnt;

        (void)fprintf(stderr, "gpsdecode:INFO: version " VERSION
                      ", revision " REVISION "\n"
                      "gpsdecode:INFO: Command line: ");
        for (cnt = 0; cnt < argc; cnt++) {
            (void)fprintf(stderr, " %s", argv[cnt]);
        }
        (void)fprintf(stderr, "\n");
    }
    if (mode == doencode) {
        encode(stdin, stdout);
    } else {
        decode(stdin, stdout);
    }
    exit(EXIT_SUCCESS);
}

// vim: set expandtab shiftwidth=4
