/*
 * NMEA2000 over CAN.
 *
 * NMEA2000 is proprietary and the doc is not public.
 * Much of this code is reverse engineered or built from
 * the sparse doc some vendors provide on their interpretation
 * of the specification.
 *
 * Here is one good source of reverse engineered info:
 *     https://github.com/canboat/canboat
 *     https://canboat.github.io/canboat/canboat.html
 *
 * Message contents can be had from canboat/analyzer:
 *     analyzer -explain
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#if defined(NMEA2000_ENABLE)

#include <ctype.h>
#include <errno.h>                  // for strerror(errno), errno)
#include <fcntl.h>
#include <linux/can.h>              // for  struct can_frame
#include <linux/can/error.h>        // for CAN_ERR_BUSOFF, etc.
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/libgps.h"
#include "../include/driver_nmea2000.h"
#include "../include/bits.h"
#include "../include/timespec.h"


#define LOG_FILE 1
#define NMEA2000_NETS 4
/* NMEA 2000 source addr (SA) is a byte,
 * but 254 is "request for address claim"
 * and 255 is broadcast address.  So 254 is the number of addresses possible,
 * and 253 the highest addresses number. */
#define NMEA2000_ADDRS 254
#define CAN_NAMELEN 32
#define MIN(a,b) ((a < b) ? a : b)

#define NMEA2000_DEBUG_AIS 0

static struct gps_device_t *nmea2000_units[NMEA2000_NETS][NMEA2000_ADDRS];
static char can_interface_name[NMEA2000_NETS][CAN_NAMELEN + 1];

typedef struct PGN {
    unsigned int pgn;
    char  fast;
    char  type;
    gps_mask_t (* func)(struct gps_device_t *session);
    const char *name;
} PGN;

#if LOG_FILE
FILE *logFile = NULL;
#endif  // of if LOG_FILE

// WTF???
extern bool __attribute__ ((weak)) gpsd_add_device(const char *device_name,
                                                   bool flag_nowait);

/* Industry ids
 * https://canboat.github.io/canboat/canboat.html#lookup-INDUSTRY_CODE
 */
static const struct vlist_t indus_ids[] = {
    {0, "Global"},
    {1, "Highway"},
    {2, "Agriculture"},
    {3, "Construction"},
    {4, "Marine Industry"},
    {5, "Industrial"},
    {0, NULL}
};

/* Manfacturer ids
 * https://canboat.github.io/canboat/canboat.html#lookup-MANUFACTURER_CODE
 */
static const struct vlist_t mfg_ids[] = {
    {69, "ARKS Enterprises, Inc."},
    {78, "FW Murphy/Enovation Controls"},
    {80, "Twin Disc"},
    {85, "Kohler Power Systems"},
    {88, "Hemisphere GPS Inc"},
    {116, "BEP Marine"},
    {135, "Airmar"},
    {137, "Maretron"},
    {140, "Lowrance"},
    {144, "Mercury Marine"},
    {147, "Nautibus Electronic GmbH"},
    {148, "Blue Water Data"},
    {154, "Westerbeke"},
    {157, "ISSPRO Inc"},
    {161, "Offshore Systems (UK) Ltd."},
    {163, "Evinrude/BRP"},
    {165, "CPAC Systems AB"},
    {168, "Xantrex Technology Inc."},
    {169, "Marlin Technologies, Inc."},
    {172, "Yanmar Marine"},
    {174, "Volvo Penta"},
    {175, "Honda Marine"},
    {176, "Carling Technologies Inc. (Moritz Aerospace)"},
    {185, "Beede Instruments"},
    {192, "Floscan Instrument Co. Inc."},
    {193, "Nobletec"},
    {198, "Mystic Valley Communications"},
    {199, "Actia"},
    {200, "Honda Marine"},
    {201, "Disenos Y Technologia"},
    {211, "Digital Switching Systems"},
    {215, "Xintex/Atena"},
    {224, "EMMI NETWORK S.L."},
    {225, "Honda Marine"},
    {228, "ZF"},
    {229, "Garmin"},
    {233, "Yacht Monitoring Solutions"},
    {235, "Sailormade Marine Telemetry/Tetra Technology LTD"},
    {243, "Eride"},
    {250, "Honda Marine"},
    {257, "Honda Motor Company LTD"},
    {272, "Groco"},
    {273, "Actisense"},
    {274, "Amphenol LTW Technology"},
    {275, "Navico"},
    {283, "Hamilton Jet"},
    {285, "Sea Recovery"},
    {286, "Coelmo SRL Italy"},
    {295, "BEP Marine"},
    {304, "Empir Bus"},
    {305, "NovAtel"},
    {306, "Sleipner Motor AS"},
    {307, "MBW Technologies"},
    {311, "Fischer Panda"},
    {315, "ICOM"},
    {328, "Qwerty"},
    {329, "Dief"},
    {341, "Boening Automationstechnologie GmbH & Co. KG"},
    {345, "Korean Maritime University"},
    {351, "Thrane and Thrane"},
    {355, "Mastervolt"},
    {356, "Fischer Panda Generators"},
    {358, "Victron Energy"},
    {370, "Rolls Royce Marine"},
    {373, "Electronic Design"},
    {374, "Northern Lights"},
    {378, "Glendinning"},
    {381, "B & G"},
    {384, "Rose Point Navigation Systems"},
    {385, "Johnson Outdoors Marine Electronics Inc Geonav"},
    {394, "Capi 2"},
    {396, "Beyond Measure"},
    {400, "Livorsi Marine"},
    {404, "ComNav"},
    {409, "Chetco"},
    {419, "Fusion Electronics"},
    {421, "Standard Horizon"},
    {422, "True Heading AB"},
    {426, "Egersund Marine Electronics AS"},
    {427, "em-trak Marine Electronics"},
    {431, "Tohatsu Co, JP"},
    {437, "Digital Yacht"},
    {438, "Comar Systems Limited"},
    {440, "Cummins"},
    {443, "VDO (aka Continental-Corporation)"},
    {451, "Parker Hannifin aka Village Marine Tech"},
    {459, "Alltek Marine Electronics Corp"},
    {460, "SAN GIORGIO S.E.I.N"},
    {466, "Veethree Electronics & Marine"},
    {467, "Humminbird Marine Electronics"},
    {470, "SI-TEX Marine Electronics"},
    {471, "Sea Cross Marine AB"},
    {475, "GME aka Standard Communications Pty LTD"},
    {476, "Humminbird Marine Electronics"},
    {478, "Ocean Sat BV"},
    {481, "Chetco Digitial Instruments"},
    {493, "Watcheye"},
    {499, "Lcj Capteurs"},
    {502, "Attwood Marine"},
    {503, "Naviop S.R.L."},
    {504, "Vesper Marine Ltd"},
    {510, "Marinesoft Co. LTD"},
    {513, "Simarine"},
    {517, "NoLand Engineering"},
    {518, "Transas USA"},
    {529, "National Instruments Korea"},
    {530, "National Marine Electronics Association"},
    {532, "Onwa Marine"},
    {540, "Webasto"},
    {571, "Marinecraft (South Korea)"},
    {573, "McMurdo Group aka Orolia LTD"},
    {578, "Advansea"},
    {579, "KVH"},
    {580, "San Jose Technology"},
    {583, "Yacht Control"},
    {586, "Suzuki Motor Corporation"},
    {591, "US Coast Guard"},
    {595, "Ship Module aka Customware"},
    {600, "Aquatic AV"},
    {605, "Aventics GmbH"},
    {606, "Intellian"},
    {612, "SamwonIT"},
    {614, "Arlt Tecnologies"},
    {637, "Bavaria Yacts"},
    {641, "Diverse Yacht Services"},
    {644, "Wema U.S.A dba KUS"},
    {645, "Garmin"},
    {658, "Shenzhen Jiuzhou Himunication"},
    {688, "Rockford Corp"},
    {699, "Harman International"},
    {704, "JL Audio"},
    {708, "Lars Thrane"},
    {715, "Autonnic"},
    {717, "Yacht Devices"},
    {734, "REAP Systems"},
    {735, "Au Electronics Group"},
    {739, "LxNav"},
    {741, "Littelfuse, Inc (formerly Carling Technologies)"},
    {743, "DaeMyung"},
    {744, "Woosung"},
    {748, "ISOTTA IFRA srl"},
    {773, "Clarion US"},
    {776, "HMI Systems"},
    {777, "Ocean Signal"},
    {778, "Seekeeper"},
    {781, "Poly Planar"},
    {785, "Fischer Panda DE"},
    {795, "Broyda Industries"},
    {796, "Canadian Automotive"},
    {797, "Tides Marine"},
    {798, "Lumishore"},
    {799, "Still Water Designs and Audio"},
    {802, "BJ Technologies (Beneteau)"},
    {803, "Gill Sensors"},
    {811, "Blue Water Desalination"},
    {815, "FLIR"},
    {824, "Undheim Systems"},
    {826, "Lewmar Inc"},
    {838, "TeamSurv"},
    {844, "Fell Marine"},
    {847, "Oceanvolt"},
    {862, "Prospec"},
    {868, "Data Panel Corp"},
    {890, "L3 Technologies"},
    {894, "Rhodan Marine Systems"},
    {896, "Nexfour Solutions"},
    {905, "ASA Electronics"},
    {909, "Marines Co (South Korea)"},
    {911, "Nautic-on"},
    {917, "Sentinel"},
    {929, "JL Marine ystems"},
    {930, "Ecotronix"},
    {944, "Zontisa Marine"},
    {951, "EXOR International"},
    {962, "Timbolier Industries"},
    {963, "TJC Micro"},
    {968, "Cox Powertrain"},
    {969, "Blue Seas"},
    {981, "Kobelt Manufacturing Co. Ltd"},
    {992, "Blue Ocean IOT"},
    {997, "Xenta Systems"},
    {1004, "Ultraflex SpA"},
    {1008, "Lintest SmartBoat"},
    {1011, "Soundmax"},
    {1020, "Team Italia Marine (Onyx Marine Automation s.r.l)"},
    {1021, "Entratech"},
    {1022, "ITC Inc."},
    {1029, "The Marine Guardian LLC"},
    {1047, "Sonic Corporation"},
    {1051, "ProNav"},
    {1053, "Vetus Maxwell INC."},
    {1056, "Lithium Pros"},
    {1059, "Boatrax"},
    {1062, "Marol Co ltd"},
    {1065, "CALYPSO Instruments"},
    {1066, "Spot Zero Water"},
    {1069, "Lithionics Battery LLC"},
    {1070, "Quick-teck Electronics Ltd"},
    {1075, "Uniden America"},
    {1083, "Nauticoncept"},
    {1084, "Shadow-Caster LED lighting LLC"},
    {1085, "Wet Sounds, LLC"},
    {1088, "E-T-A Circuit Breakers"},
    {1092, "Scheiber"},
    {1100, "Smart Yachts International Limited"},
    {1109, "Dockmate"},
    {1114, "Bobs Machine"},
    {1118, "L3Harris ASV"},
    {1119, "Balmar LLC"},
    {1120, "Elettromedia spa"},
    {1127, "Electromaax"},
    {1140, "Across Oceans Systems Ltd."},
    {1145, "Kiwi Yachting"},
    {1150, "BSB Artificial Intelligence GmbH"},
    {1151, "Orca Technologoes AS"},
    {1154, "TBS Electronics BV"},
    {1158, "Technoton Electroics"},
    {1160, "MG Energy Systems B.V."},
    {1169, "Sea Macine Robotics Inc."},
    {1171, "Vista Manufacturing"},
    {1183, "Zipwake"},
    {1186, "Sailmon BV"},
    {1192, "Airmoniq Pro Kft"},
    {1194, "Sierra Marine"},
    {1200, "Xinuo Information Technology (Xiamen)"},
    {1218, "Septentrio"},
    {1233, "NKE Marine Elecronics"},
    {1238, "SuperTrack Aps"},
    {1239, "Honda Electronics Co., LTD"},
    {1245, "Raritan Engineering Company, Inc"},
    {1249, "Integrated Power Solutions AG"},
    {1260, "Interactive Technologies, Inc."},
    {1283, "LTG-Tech"},
    {1299, "Energy Solutions (UK) LTD."},
    {1300, "WATT Fuel Cell Corp"},
    {1302, "Pro Mainer"},
    {1305, "Dragonfly Energy"},
    {1306, "Koden Electronics Co., Ltd"},
    {1311, "Humphree AB"},
    {1316, "Hinkley Yachts"},
    {1317, "Global Marine Management GmbH (GMM)"},
    {1320, "Triskel Marine Ltd"},
    {1330, "Warwick Control Technologies"},
    {1331, "Dolphin Charger"},
    {1337, "Barnacle Systems Inc"},
    {1348, "Radian IoT, Inc."},
    {1353, "Ocean LED Marine Ltd"},
    {1359, "BluNav"},
    {1361, "OVA (Nantong Saiyang Electronics Co., Ltd)"},
    {1368, "RAD Propulsion"},
    {1369, "Electric Yacht"},
    {1372, "Elco Motor Yachts"},
    {1384, "Tecnoseal Foundry S.r.l"},
    {1385, "Pro Charging Systems, LLC"},
    {1389, "EVEX Co., LTD"},
    {1398, "Gobius Sensor Technology AB"},
    {1403, "Arco Marine"},
    {1408, "Lenco Marine Inc."},
    {1413, "Naocontrol S.L."},
    {1417, "Revatek"},
    {1438, "Aeolionics"},
    {1439, "PredictWind Ltd"},
    {1440, "Egis Mobile Electric"},
    {1445, "Starboard Yacht Group"},
    {1446, "Roswell Marine"},
    {1451, "ePropulsion (Guangdong ePropulsion Technology Ltd.)"},
    {1452, "Micro-Air LLC"},
    {1453, "Vital Battery"},
    {1458, "Ride Controller LLC"},
    {1460, "Tocaro Blue"},
    {1461, "Vanquish Yachts"},
    {1471, "FT Technologies"},
    {1478, "Alps Alpine Co., Ltd."},
    {1481, "E-Force Marine"},
    {1482, "CMC Marine"},
    {1483, "Nanjing Sandemarine Information Technology Co., Ltd."},
    {1850, "Teleflex Marine (SeaStar Solutions)"},
    {1851, "Raymarine"},
    {1852, "Navionics"},
    {1853, "Japan Radio Co"},
    {1854, "Northstar Technologies"},
    {1855, "Furuno"},
    {1856, "Trimble"},
    {1857, "Simrad"},
    {1858, "Litton"},
    {1859, "Kvasar AB"},
    {1860, "MMP"},
    {1861, "Vector Cantech"},
    {1862, "Yamaha Marine"},
    {1863, "Faria Instrument"},
    {0, NULL}
};

#define SHIFT32 0x100000000l

static int scale_int(int32_t var, const int64_t factor)
{
    int64_t ret = (var * factor) >> 32;

    return (int)ret;
}

static void print_data(struct gps_device_t *session)
{
    unsigned char *buffer = session->lexer.outbuffer;
    size_t len = session->lexer.outbuflen;
    const PGN *pgn = (const PGN *)session->driver.nmea2000.workpgn;
    size_t l1;
    int l2 = 0;
    int idx = 0;
    char bu[128];

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn %6d SA %u len %zu %s\n",
             pgn->pgn, session->driver.nmea2000.source_addr, len, pgn->name);

    if (LOG_IO > libgps_debuglevel) {
        return;
    }

    for (l1 = 0; l1 < len; l1++) {
        if (0 == (l1 % 20) &&
            0 != l1) {
            GPSD_LOG(LOG_IO, &session->context->errout,
                     "NMEA2000: got data: %s\n", bu);
            idx = 0;
            bu[0] = '\0';
        }
        // FIXME: check buffer overrun
        l2 = sprintf(&bu[idx], "x%02x ", (unsigned)buffer[l1]);
        idx += l2;
    }
    GPSD_LOG(LOG_IO, &session->context->errout,
             "NMEA2000: got data: %s\n", bu);
}

static gps_mask_t get_mode(struct gps_device_t *session)
{
    if (1 & session->driver.nmea2000.mode_valid) {
        session->newdata.mode = session->driver.nmea2000.mode;
    } else {
        session->newdata.mode = MODE_NOT_SEEN;
    }

    if (2 & session->driver.nmea2000.mode_valid) {
        return MODE_SET | USED_IS;
    } else {
        return MODE_SET;
    }
}


static int decode_ais_header(struct gps_context_t *context,
                             unsigned char *bu,
                             size_t len,
                             struct ais_t *ais,
                             unsigned int mask)
{
    if (4 < len) {
        ais->type   = (unsigned)( bu[0]       & 0x3f);
        ais->repeat = (unsigned)((bu[0] >> 6) & 0x03);
        ais->mmsi   = (unsigned) getleu32(bu, 1);
        ais->mmsi  &= mask;
        GPSD_LOG(LOG_INF, &context->errout,
                 "NMEA2000 AIS  message type %u, MMSI %09u:\n",
                 ais->type, ais->mmsi);
        return 1;
    }
    //  else
    ais->type   =  0;
    ais->repeat =  0;
    ais->mmsi   =  0;
    GPSD_LOG(LOG_ERROR, &context->errout,
             "NMEA2000 AIS  message type %u, too short message.\n",
             ais->type);
    return 0;
}


static void decode_ais_channel_info(unsigned char *bu,
                                    size_t len,
                                    unsigned int offset,
                                    struct gps_device_t *session)
{
    unsigned int pos = offset / 8;
    unsigned int bpos = offset % 8;
    uint16_t x;

    if (pos >= (unsigned int)len) {
        session->driver.aivdm.ais_channel = 'A';
        return;
    }
    x = getleu16(bu, pos);
    x = (uint16_t)((x >> bpos) & 0x1f);
    switch (x) {
    case 1:
        FALLTHROUGH
    case 3:
        session->driver.aivdm.ais_channel = 'B';
        break;
    default:
        session->driver.aivdm.ais_channel = 'A';
        break;
    }
    return;
}


static int ais_turn_rate(int rate)
{
    if (0 > rate) {
        return -ais_turn_rate(-rate);
    }
    return (int)(4.733 * sqrt(rate * RAD_2_DEG * .0001 * 60.0));
}


static double ais_direction(unsigned int val, double scale)
{
    if ((0xffff == val) &&
        (1.0 == scale)) {
        return 511.0;
    }
    return val * RAD_2_DEG * 0.0001 * scale;
}


/*
 *   PGN 59392: ISO Acknowledgment
 */
static gps_mask_t hnd_059392(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 60928: ISO Address Claim
 */
static gps_mask_t hnd_060928(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 126208: NMEA Command/Request/Acknowledge
 */
static gps_mask_t hnd_126208(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 126464: ISO Transmit/Receive PGN List
 */
static gps_mask_t hnd_126464(struct gps_device_t *session UNUSED)
{
    return 0;
}

/*
 *   PGN 126720: Maretron proprietary, used by Garmin, etc.:
 */
static gps_mask_t hnd_126720(struct gps_device_t *session UNUSED)
{
    unsigned char *bu = session->lexer.outbuffer;

    unsigned word0 = getleu16(bu, 0);
    unsigned mfg = word0 & 0x07ff;           // 11 bits Manufacturer Code
    // 2 bits reserved
    unsigned indus = (word0 >> 13) & 0x07;   // 3 bits Industry Code
    unsigned prop_id = bu[2];                // 8 bits Proprietary ID
    unsigned cmd = bu[3];                    // 8 bits Command

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 126720 mfg %u indus %u prop %u cmd %u (%s/%s)\n",
             mfg, indus, prop_id, cmd,
             val2str(mfg, mfg_ids),
             val2str(indus, indus_ids));
    return 0;
}

static const struct vlist_t time_sources[] = {
    {0, "GPS"},
    {1, "GLONASS"},
    {2, "Radio Station"},
    {3, "Local Cesium clock"},
    {4, "Local Rubidium clock"},
    {5, "Local Crystal clock"},
    {0, NULL}
};

/*
 * PGN: 126992 / 00370020 / 1F010 - 8 - System Time
 *
 *  Field #1: SID
 *                  Bits: 8
 *                  Signed: false
 *  Field #2: Source
 *                  Bits: 4
 *                  Type: Lookup table
 *                  Signed: false
 *                  Lookup: 0=GPS
 *                  Lookup: 1=GLONASS
 *                  Lookup: 2=Radio Station
 *                  Lookup: 3=Local Cesium clock
 *                  Lookup: 4=Local Rubidium clock
 *                  Lookup: 5=Local Crystal clock
 *  Field #3: Reserved - Reserved
 *                  Bits: 4
 *                  Type: Binary data
 *                  Signed: false
 *  Field #4: Date - Days since January 1, 1970
 *                  Bits: 16
 *                  Units: days
 *                  Type: Date
 *                  Resolution: 1
 *                  Signed: false
 *  Field #5: Time - Seconds since midnight
 *                  Bits: 32
 *                  Units: s
 *                  Type: Time
 *                  Resolution: 0.0001
 *                  Signed: false
 *
 */
static gps_mask_t hnd_126992(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;

    unsigned sid = bu[0];
    unsigned source = bu[1] & 0x0f;
    uint64_t usecs = getleu32(bu, 4) * 100UL;   // time of day in us

    USTOTS(&session->newdata.time, usecs);
    session->newdata.time.tv_sec += (time_t)(getleu16(bu, 2) * 24 * 60 * 60);

    // casts for 32 bit time_t
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 126992 sid %u source %u (%s) time %lld %09ld\n",
             sid, source, val2str(source, time_sources),
             (long long)session->newdata.time.tv_sec,
             (long)session->newdata.time.tv_nsec);

    return TIME_SET | get_mode(session);
}


/*
 *   PGN 126996: ISO Product Information
 */
static gps_mask_t hnd_126996(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 127245: NAV Rudder
 */
static gps_mask_t hnd_127245(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 127250: NAV Vessel Heading
 */
static gps_mask_t hnd_127250(struct gps_device_t *session UNUSED)
{
    unsigned char *bu = session->lexer.outbuffer;
    int aux;

    session->gpsdata.attitude.heading = getleu16(bu, 1) * RAD_2_DEG * 0.0001;
//  printf("ATT 0:%8.3f\n",session->gpsdata.attitude.heading);
    aux = getles16(bu, 3);
    if (0x07fff != aux) {
        session->gpsdata.attitude.heading += aux * RAD_2_DEG * 0.0001;
    }
//  printf("ATT 1:%8.3f %6x\n",session->gpsdata.attitude.heading, aux);
    aux = getles16(bu, 5);
    if (0x07fff != aux) {
        session->gpsdata.attitude.heading += aux * RAD_2_DEG * 0.0001;
    }
//  printf("ATT 2:%8.3f %6x\n",session->gpsdata.attitude.heading, aux);

    return ONLINE_SET | ATTITUDE_SET;
}


/*
 *   PGN 127258: GNSS Magnetic Variation
 *
 *   1 Sequence ID
 *   2 Variation Source
 *   3 Reserved Bits
 *   4 Age of Service (Date)
 *   5 Variation
 *   6 Reserved B
 */
static gps_mask_t hnd_127258(struct gps_device_t *session UNUSED)
{
    // FIXME?  Get magnetic variation
    return 0;
}


static const struct vlist_t dc_types[] = {
    {0, "Battery"},
    {1, "Alternator"},
    {2, "Convertor"},
    {3, "Solar cell"},
    {4, "Wind generator"},
    {0, NULL},
};

/*
 *   PGN 127506: PWR DC Detailed Status
 */
static gps_mask_t hnd_127506(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;

    unsigned sid = bu[0];
    unsigned instance = bu[1];
    unsigned dc_type = bu[2];
    unsigned charge = bu[3];
    unsigned health = bu[4];
    unsigned timer = getles16(bu, 5);
    unsigned ripple = getles16(bu, 7);
    unsigned cap = getles16(bu, 9);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 127506 sid %u instance %u DC type %u (%s) "
             "charge %u health %u time %u ripple %u cap %u\n",
             sid, instance, dc_type, val2str(dc_type, dc_types),
             charge, health, timer, ripple, cap);
    return 0;
}


/*
 *   PGN 127508: PWR Battery Status
 */
static gps_mask_t hnd_127508(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 127513: PWR Battery Configuration Status
 */
static gps_mask_t hnd_127513(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 128259: NAV Speed
 */
static gps_mask_t hnd_128259(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 128267: NAV Water Depth
 */
static gps_mask_t hnd_128267(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;

    unsigned sid = bu[0];
    double offset= getleu16(bu, 5) / 1000.0;
    unsigned range = bu[7];
    session->gpsdata.attitude.depth = getleu32(bu, 1) / 100.0 ;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 128267 sid %u depth %.2f offset %.3f range %u\n",
             sid, session->gpsdata.attitude.depth, offset, range);
    return ONLINE_SET | ATTITUDE_SET;
}


/*
 *   PGN 128275: NAV Distance Log
 */
static gps_mask_t hnd_128275(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 129283: NAV Cross Track Error
 */
static gps_mask_t hnd_129283(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 129284: NAV Navigation Data
 */
static gps_mask_t hnd_129284(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 129285: NAV Navigation - Route/WP Information
 */
static gps_mask_t hnd_129285(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 129025: GNSS Position Rapid Update
 */
static gps_mask_t hnd_129025(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;

    session->newdata.latitude = getles32(bu, 0) * 1e-7;
    session->newdata.longitude = getles32(bu, 4) * 1e-7;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 129025 lat %.4f lon %.4f\n",
             session->newdata.latitude,
             session->newdata.longitude);
    return LATLON_SET | get_mode(session);
}


static const struct vlist_t cog_refs[] = {
    {0, "True"},
    {1, "Magnetic"},
    {2, "Error"},
    {0, NULL},
};

/*
 *   PGN 129026: GNSS COG and SOG Rapid Update
 */
static gps_mask_t hnd_129026(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;

    unsigned cog_ref = bu[1] & 0x03;
    session->driver.nmea2000.sid[0] = bu[0];

    session->newdata.track = getleu16(bu, 2) * 1e-4 * RAD_2_DEG;
    session->newdata.speed = getleu16(bu, 4) * 1e-2;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 129026 sid %u ref %u (%s) COG %.3f SOG %.3f\n",
             session->driver.nmea2000.sid[0], cog_ref,
             val2str(cog_ref, cog_refs),
             session->newdata.track, session->newdata.speed);
    return SPEED_SET | TRACK_SET | get_mode(session);
}

static const struct vlist_t integritys[] = {
    {0, "No integrity checking"},
    {1, "Safe"},
    {2, "Caution"},
    {3, "Unsafe"},
    {0, NULL}
};

static const struct vlist_t gns_methods[] = {
    {0, "no GNSS"},
    {1, "GNSS fix"},
    {2, "DGNSS fix"},
    {3, "Precise GNSS"},
    {4, "RTK Fixed Integer"},
    {5, "RTK float"},
    {6, "Estimated (DR) mode"},
    {7, "Manual Input"},
    {8, "Simulate mode"},
    {0, NULL}
};

static const struct vlist_t gnss_types[] = {
    {0, "GPS"},
    {1, "GLONASS"},
    {2, "GPS+GLONASS"},
    {3, "GPS+SBAS/WAAS"},
    {4, "GPS+SBAS/WAAS+GLONASS"},
    {5, "Chayka"},
    {6, "integrated"},
    {7, "surveyed"},
    {8, "Galileo"},
    {0, NULL}
};

/*
 * PGN: 129029 / 00374005 / 1F805 - 51 - GNSS Position Data
 *
 *      The last 3 fields repeat until the data is exhausted.
 *
 *   Field #1: SID
 *                   Bits: 8
 *                   Signed: false
 *   Field #2: Date - Days since January 1, 1970
 *                   Bits: 16
 *                   Units: days
 *                   Type: Date
 *                   Resolution: 1
 *                   Signed: false
 *   Field #3: Time - Seconds since midnight
 *                   Bits: 32
 *                   Units: s
 *                   Type: Time
 *                   Resolution: 0.0001
 *                   Signed: false
 *   Field #4: Latitude
 *                   Bits: 64
 *                   Units: deg
 *                   Type: Latitude
 *                   Resolution: 0.0000000000000001
 *                   Signed: true
 *   Field #5: Longitude
 *                   Bits: 64
 *                   Units: deg
 *                   Type: Longitude
 *                   Resolution: 0.0000000000000001
 *                   Signed: true
 *   Field #6: Altitude - Altitude referenced to WGS-84
 *                   Bits: 64
 *                   Units: m
 *                   Resolution: 1e-06
 *                   Signed: true
 *   Field #7: GNSS type
 *                   Bits: 4
 *                   Type: Lookup table
 *                   Signed: false
 *                   Lookup: 0=GPS
 *                   Lookup: 1=GLONASS
 *                   Lookup: 2=GPS+GLONASS
 *                   Lookup: 3=GPS+SBAS/WAAS
 *                   Lookup: 4=GPS+SBAS/WAAS+GLONASS
 *                   Lookup: 5=Chayka
 *                   Lookup: 6=integrated
 *                   Lookup: 7=surveyed
 *                   Lookup: 8=Galileo
 *   Field #8: Method
 *                   Bits: 4
 *                   Type: Lookup table
 *                   Signed: false
 *                   Lookup: 0=no GNSS
 *                   Lookup: 1=GNSS fix
 *                   Lookup: 2=DGNSS fix
 *                   Lookup: 3=Precise GNSS
 *                   Lookup: 4=RTK Fixed Integer
 *                   Lookup: 5=RTK float
 *                   Lookup: 6=Estimated (DR) mode
 *                   Lookup: 7=Manual Input
 *                   Lookup: 8=Simulate mode
 *   Field #9: Integrity
 *                   Bits: 2
 *                   Type: Lookup table
 *                   Signed: false
 *                   Lookup: 0=No integrity checking
 *                   Lookup: 1=Safe
 *                   Lookup: 2=Caution
 *   Field #10: Reserved - Reserved
 *                   Bits: 6
 *                   Type: Binary data
 *                   Signed: false
 *   Field #11: Number of SVs - Number of satellites used in solution
 *                   Bits: 8
 *                   Signed: false
 *   Field #12: HDOP - Horizontal dilution of precision
 *                   Bits: 16
 *                   Resolution: 0.01
 *                   Signed: true
 *   Field #13: PDOP - Probable dilution of precision
 *                   Bits: 16
 *                   Resolution: 0.01
 *                   Signed: true
 *   Field #14: Geoidal Separation - Geoidal Separation
 *                   Bits: 32
 *                   Units: m
 *                   Resolution: 0.01
 *                   Signed: true
 *   Field #15: Reference Stations - Number of reference stations
 *                   Bits: 8
 *                   Signed: false
 *   Field #16: Reference Station Type
 *                   Bits: 4
 *                   Type: Lookup table
 *                   Signed: false
 *                   Lookup: 0=GPS
 *                   Lookup: 1=GLONASS
 *                   Lookup: 2=GPS+GLONASS
 *                   Lookup: 3=GPS+SBAS/WAAS
 *                   Lookup: 4=GPS+SBAS/WAAS+GLONASS
 *                   Lookup: 5=Chayka
 *                   Lookup: 6=integrated
 *                   Lookup: 7=surveyed
 *                   Lookup: 8=Galileo
 *   Field #17: Reference Station ID
 *                   Bits: 12
 *                   Units:
 *                   Signed: false
 *   Field #18: Age of DGNSS Corrections
 *                   Bits: 16
 *                   Units: s
 *                   Resolution: 0.01
 *                   Signed: false
 *
 */
static gps_mask_t hnd_129029(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    gps_mask_t mask = 0;
    uint64_t usecs;                           // time of day in us
    unsigned gns_method = (bu[31] >> 4) & 0x0f;
    unsigned gnss_type = bu[31] & 0x0f;
    unsigned integrity = bu[32] & 0x03;
    unsigned refs = bu[40];

    session->driver.nmea2000.sid[3]  = bu[0];

    // field 3 is time of day in 0.1 ms
    usecs = getleu32(bu, 3) * (uint64_t)100;
    USTOTS(&session->newdata.time, usecs);
    // add in the date from field 2
    session->newdata.time.tv_sec += (time_t)(getleu16(bu,1) * 24 * 60 * 60);
    mask |= TIME_SET;

    session->newdata.latitude = getles64(bu, 7) * 1e-16;
    session->newdata.longitude = getles64(bu, 15) * 1e-16;
    mask |= LATLON_SET;

    session->newdata.altHAE = getles64(bu, 23) * 1e-6;
    mask |= ALTITUDE_SET;

    switch (gns_method) {
    case 0:
        session->newdata.status = STATUS_UNK;
        break;
    case 1:
        session->newdata.status = STATUS_GPS;
        break;
    case 2:
        session->newdata.status = STATUS_DGPS;
        break;
    case 3:
        session->newdata.status = STATUS_PPS_FIX;
        break;
    case 4:
        session->newdata.status = STATUS_RTK_FIX;
        break;
    case 5:
        session->newdata.status = STATUS_RTK_FLT;
        break;
    case 6:
        session->newdata.status = STATUS_DR;
        break;
    case 7:
        session->newdata.status = STATUS_TIME;
        break;
    case 8:
        session->newdata.status = STATUS_SIM;
        break;
    default:
        session->newdata.status = STATUS_UNK;
        break;
    }
    mask |= STATUS_SET;

    session->newdata.geoid_sep = getles32(bu, 38) / 100.0;

    session->gpsdata.satellites_used = (int)bu[33];

    session->gpsdata.dop.hdop = getleu16(bu, 34) * 1e-2;
    session->gpsdata.dop.pdop = getleu16(bu, 36) * 1e-2;
    mask |= DOP_SET;

    session->newdata.geoid_sep = getles32(bu, 38) / 100;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 129029 SA %u sid %u lat %.2f lon %.2f HAE %.2f "
             "type %u(%s) method %u(%s) integrity %u(%s) "
             "hdop:%5.2f pdop:%5.2f sep %.2f refs %u\n",
             session->driver.nmea2000.source_addr,
             session->driver.nmea2000.sid[3],
             session->newdata.latitude,
             session->newdata.longitude,
             session->newdata.altHAE,
             gnss_type, val2str(gnss_type, gnss_types),
             gns_method, val2str(gns_method, gns_methods),
             integrity, val2str(integrity, integritys),
             session->gpsdata.dop.hdop,
             session->gpsdata.dop.pdop,
             session->newdata.geoid_sep, refs);
    return mask | get_mode(session);
}


/*
 *   PGN 129038: AIS Class A Position Report
 */
static gps_mask_t hnd_129038(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0xffffffffU)) {
        ais->type1.lon = (int)scale_int(getles32(bu, 5),
                                        (int64_t)(SHIFT32 *.06L));
        ais->type1.lat = (int)scale_int(getles32(bu, 9),
                                        (int64_t)(SHIFT32 *.06L));
        ais->type1.accuracy  = (bool)         ((bu[13] >> 0) & 0x01);
        ais->type1.raim      = (bool)         ((bu[13] >> 1) & 0x01);
        ais->type1.second    = (unsigned int) ((bu[13] >> 2) & 0x3f);
        ais->type1.course = (unsigned int)ais_direction(
                                       (unsigned int)getleu16(bu, 14), 10.0);
        ais->type1.speed = (unsigned int)(getleu16(bu, 16) *
                                          MPS_TO_KNOTS * 0.01 / 0.1);
        ais->type1.radio     = (unsigned int) (getleu32(bu, 18) & 0x7ffff);
        ais->type1.heading =
            (unsigned int)ais_direction((unsigned int)getleu16(bu, 21), 1.0);
        ais->type1.turn = ais_turn_rate((int)getles16(bu, 23));
        ais->type1.status    = (unsigned int) ((bu[25] >> 0) & 0x0f);
        ais->type1.maneuver  = 0;  // Not transmitted ????
        decode_ais_channel_info(bu, len, 163, session);

        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


/*
 *   PGN 129039: AIS Class B Position Report
 */
static gps_mask_t hnd_129039(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0xffffffffU)) {
        ais->type18.lon = (int)scale_int(getles32(bu, 5),
                                         (int64_t)(SHIFT32 *.06L));
        ais->type18.lat = (int)scale_int(getles32(bu, 9),
                                         (int64_t)(SHIFT32 *.06L));
        ais->type18.accuracy = (bool)         ((bu[13] >> 0) & 0x01);
        ais->type18.raim     = (bool)         ((bu[13] >> 1) & 0x01);
        ais->type18.second   = (unsigned int) ((bu[13] >> 2) & 0x3f);
        ais->type18.course =
            (unsigned int)ais_direction((unsigned int) getleu16(bu, 14), 10.0);
        ais->type18.speed = (unsigned int)(getleu16(bu, 16) *
                                           MPS_TO_KNOTS * 0.01 / 0.1);
        ais->type18.radio    = (unsigned int) (getleu32(bu, 18) & 0x7ffff);
        ais->type18.heading =
            (unsigned int)ais_direction((unsigned int) getleu16(bu, 21), 1.0);
        ais->type18.reserved = 0;
        ais->type18.regional = (unsigned int) ((bu[24] >> 0) & 0x03);
        ais->type18.cs       = (bool)         ((bu[24] >> 2) & 0x01);
        ais->type18.display  = (bool)         ((bu[24] >> 3) & 0x01);
        ais->type18.dsc      = (bool)         ((bu[24] >> 4) & 0x01);
        ais->type18.band     = (bool)         ((bu[24] >> 5) & 0x01);
        ais->type18.msg22    = (bool)         ((bu[24] >> 6) & 0x01);
        ais->type18.assigned = (bool)         ((bu[24] >> 7) & 0x01);
        decode_ais_channel_info(bu, len, 163, session);

        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


/*
 *   PGN 129040: AIS Class B Extended Position Report
 *
 *  No test case for this message at the moment
 */
static gps_mask_t hnd_129040(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0xffffffffU)) {
        uint16_t length, beam, to_bow, to_starboard;

        ais->type19.lon = (int)scale_int(getles32(bu, 5),
                                         (int64_t)(SHIFT32 *.06L));
        ais->type19.lat = (int)scale_int(getles32(bu, 9),
                                         (int64_t)(SHIFT32 *.06L));
        ais->type19.accuracy     = (bool)         ((bu[13] >> 0) & 0x01);
        ais->type19.raim         = (bool)         ((bu[13] >> 1) & 0x01);
        ais->type19.second       = (unsigned int) ((bu[13] >> 2) & 0x3f);
        ais->type19.course =
            (unsigned int)ais_direction((unsigned int)getleu16(bu, 14), 10.0);
        ais->type19.speed =
            (unsigned int)(getleu16(bu, 16) * MPS_TO_KNOTS * 0.01 / 0.1);
        ais->type19.reserved     = (unsigned int) ((bu[18] >> 0) & 0xff);
        ais->type19.regional     = (unsigned int) ((bu[19] >> 0) & 0x0f);
        ais->type19.shiptype     = (unsigned int) ((bu[20] >> 0) & 0xff);
        ais->type19.heading =
           (unsigned int)  ais_direction((unsigned int) getleu16(bu, 21), 1.0);
        length                   =                 getleu16(bu, 24);
        beam                     =                 getleu16(bu, 26);
        to_starboard             =                 getleu16(bu, 28);
        to_bow                   =                 getleu16(bu, 30);
        if ((0xffff == length) ||
            (0xffff == to_bow)) {
            length       = 0;
            to_bow       = 0;
        }
        if ((0xffff == beam) ||
            (0xffff == to_starboard)) {
            beam         = 0;
            to_starboard = 0;
        }
        ais->type19.to_bow       = (unsigned int) (to_bow / 10);
        ais->type19.to_stern     = (unsigned int) ((length-to_bow) / 10);
        ais->type19.to_port      = (unsigned int) ((beam-to_starboard) / 10);
        ais->type19.to_starboard = (unsigned int) (to_starboard / 10);
        ais->type19.epfd         = (unsigned int) ((bu[23] >> 4) & 0x0f);
        ais->type19.dte          = (unsigned int) ((bu[52] >> 0) & 0x01);
        ais->type19.assigned     = (bool)         ((bu[52] >> 1) & 0x01);
        strlcpy(ais->type19.shipname, (char *)&bu[32],
                sizeof(ais->type19.shipname));
        decode_ais_channel_info(bu, len, 422, session);

        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


static const int mode_tab[] = {MODE_NO_FIX, MODE_2D, MODE_3D, MODE_NO_FIX,
                               MODE_NO_FIX, MODE_NO_FIX, MODE_NO_FIX,
                               MODE_NO_FIX};

static const struct vlist_t gnss_modes[] = {
    {0, "1D"},
    {1, "2D"},
    {2, "3D"},
    {3, "Auto"},
    {0, NULL},
};

/*
 *   PGN 129539: GNSS DOPs
 */
static gps_mask_t hnd_129539(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    gps_mask_t mask = 0;
    unsigned int req_mode;
    unsigned int act_mode;

    session->driver.nmea2000.sid[1]  = bu[0];

    session->driver.nmea2000.mode_valid |= 1;

    req_mode = (unsigned int)((bu[1] >> 0) & 0x07);
    act_mode = (unsigned int)((bu[1] >> 3) & 0x07);

    /* This is a workaround for some GARMIN plotter,
     * actual mode auto makes no sense for me! */
    if ((3 == act_mode) &&
        (3 != req_mode)) {
        act_mode = req_mode;
    }

    session->driver.nmea2000.mode    = mode_tab[act_mode];

    session->gpsdata.dop.hdop        = getleu16(bu, 2) * 1e-2;
    session->gpsdata.dop.vdop        = getleu16(bu, 4) * 1e-2;
    session->gpsdata.dop.tdop        = getleu16(bu, 6) * 1e-2;
    mask                            |= DOP_SET;

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 129539 SA %u sid %u req %u(%s) act %u(%s) "
             "hdop %5.2f vdop %5.2f tdop %5.2f\n",
             session->driver.nmea2000.source_addr,
             session->driver.nmea2000.sid[1],
             req_mode, val2str(req_mode, gnss_modes),
             act_mode, val2str(act_mode, gnss_modes),
             session->gpsdata.dop.hdop,
             session->gpsdata.dop.vdop,
             session->gpsdata.dop.tdop);

    return mask | get_mode(session);
}

static const struct vlist_t range_modes[] = {
    {0, "Range residuals used"},
    {1, "Range residuals calculated"},
    {0, NULL}
};

static const struct vlist_t svts[] = {
    {0, "Not tracked"},
    {1, "Tracked"},
    {2, "Used"},
    {3, "Not tracked+Diff"},
    {4, "Tracked+Diff"},
    {5, "Used+Diff"},
    {0, NULL}
};

/*
 *   PGN 129540: GNSS Satellites in View
 */
static gps_mask_t hnd_129540(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    unsigned l1;
    size_t expected_len;
    unsigned range_mode = bu[1] & 0x03;

    session->driver.nmea2000.sid[2]           = bu[0];
    session->gpsdata.satellites_visible       = (int)bu[2];
    if (MAXCHANNELS <= session->gpsdata.satellites_visible) {
        // Handle a CVE for overrunning skyview[]
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA2000: pgn 129540 SA %u Too many sats %d\n",
                 session->driver.nmea2000.source_addr,
                 session->gpsdata.satellites_visible);
        session->gpsdata.satellites_visible = MAXCHANNELS;
    }
    expected_len = 3 + (12 * session->gpsdata.satellites_visible);
    if (len != expected_len) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA2000: pgn 129540 SA %u wrong length %zu s/b %zu\n",
                 session->driver.nmea2000.source_addr,
                 len, expected_len);
        return 0;
    }

    memset(session->gpsdata.skyview, '\0', sizeof(session->gpsdata.skyview));
    for (l1 = 0; l1 < session->gpsdata.satellites_visible; l1++) {
        int offset = 3 + (12 * l1);
        double elev  = getles16(bu, offset + 1) * 1e-4 * RAD_2_DEG;
        double azi   = getleu16(bu, offset + 3) * 1e-4 * RAD_2_DEG;
        double snr   = getles16(bu, offset + 5) * 1e-2;

        unsigned svt = bu[offset + 11] & 0x0f;

        session->gpsdata.skyview[l1].elevation  = elev;
        session->gpsdata.skyview[l1].azimuth    = azi;
        session->gpsdata.skyview[l1].ss         = snr;
        session->gpsdata.skyview[l1].PRN        = (int16_t)bu[offset];
        session->gpsdata.skyview[l1].used = false;
        if ((2 == svt) ||
            (5 == svt)) {
            session->gpsdata.skyview[l1].used = true;
        }
        GPSD_LOG(LOG_IO, &session->context->errout,
                 "NMEA2000: pgn 129540 PRN %d svt %u(%s)\n",
                 session->gpsdata.skyview[l1].PRN,
                 svt, val2str(svt, svts));
    }
    session->driver.nmea2000.mode_valid |= 2;
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: pgn 129540 SA %u sid %u mode %u(%s) seen %u\n",
             session->driver.nmea2000.source_addr,
             session->driver.nmea2000.sid[2],
             range_mode, val2str(range_mode, range_modes),
             session->gpsdata.satellites_visible);
    return SATELLITE_SET | USED_IS;
}


/*
 *   PGN 129793: AIS UTC and Date Report
 */
static gps_mask_t hnd_129793(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0xffffffffU)) {
        uint32_t  time;
        uint32_t  date;
        time_t    date1;
        struct tm date2;

        ais->type4.lon = (int)scale_int(getles32(bu, 5),
                                        (int64_t)(SHIFT32 *.06L));
        ais->type4.lat = (int)scale_int(getles32(bu, 9),
                                        (int64_t)(SHIFT32 *.06L));
        ais->type4.accuracy     = (bool)         ((bu[13] >> 0) & 0x01);
        ais->type4.raim         = (bool)         ((bu[13] >> 1) & 0x01);

        time = getleu32(bu, 14);
        if (0xffffffff != time) {
            time                = time / 10000;
            ais->type4.second   = time % 60; time = time / 60;
            ais->type4.minute   = time % 60; time = time / 60;
            ais->type4.hour     = time % 24;
        } else {
            ais->type4.second   = AIS_SECOND_NOT_AVAILABLE;
            ais->type4.minute   = AIS_MINUTE_NOT_AVAILABLE;
            ais->type4.hour     = AIS_HOUR_NOT_AVAILABLE;
        }

        ais->type4.radio        = (unsigned int) (getleu32(bu, 18) & 0x7ffff);

        date = getleu16(bu, 21);
        if (0xffff != date) {
            date1 = (time_t)date * (24L *60L *60L);
            (void) gmtime_r(&date1, &date2);
            ais->type4.year     = (unsigned int) (date2.tm_year + 1900);
            ais->type4.month    = (unsigned int) (date2.tm_mon + 1);
            ais->type4.day      = (unsigned int) (date2.tm_mday);
        } else {
            ais->type4.day      = AIS_DAY_NOT_AVAILABLE;
            ais->type4.month    = AIS_MONTH_NOT_AVAILABLE;
            ais->type4.year     = AIS_YEAR_NOT_AVAILABLE;
        }

        ais->type4.epfd         = (unsigned int) ((bu[23] >> 4) & 0x0f);

        decode_ais_channel_info(bu, len, 163, session);

        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


/*
 *   PGN 129794: AIS Class A Static and Voyage Related Data
 */
static gps_mask_t hnd_129794(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0xffffffffU)) {
        uint16_t  length, beam, to_bow, to_starboard, date;
        int       l;
        uint32_t  time;
        time_t    date1;
        struct tm date2;
        int       cpy_stop;

        ais->type5.ais_version   = (unsigned int) ((bu[73] >> 0) & 0x03);
        ais->type5.imo           = (unsigned int)  getleu32(bu,  5);
        if (0xffffffffU == ais->type5.imo) {
            ais->type5.imo       = 0;
        }
        ais->type5.shiptype      = (unsigned int) ((bu[36] >> 0) & 0xff);
        length                   =                 getleu16(bu, 37);
        beam                     =                 getleu16(bu, 39);
        to_starboard             =                 getleu16(bu, 41);
        to_bow                   =                 getleu16(bu, 43);
        if ((0xffff == length) ||
            (0xffff == to_bow)) {
            length       = 0;
            to_bow       = 0;
        }
        if ((0xffff == beam) ||
            (0xffff == to_starboard)) {
            beam         = 0;
            to_starboard = 0;
        }
        ais->type5.to_bow        = (unsigned int) (to_bow/10);
        ais->type5.to_stern      = (unsigned int) ((length-to_bow) / 10);
        ais->type5.to_port       = (unsigned int) ((beam-to_starboard) / 10);
        ais->type5.to_starboard  = (unsigned int) (to_starboard / 10);
        ais->type5.epfd          = (unsigned int) ((bu[73] >> 2) & 0x0f);
        date                     =                 getleu16(bu, 45);
        time                     =                 getleu32(bu, 47);
        date1                    = (time_t)       (date * 24 * 60 * 60);
        (void) gmtime_r(&date1, &date2);
        ais->type5.month         = (unsigned int) (date2.tm_mon + 1);
        ais->type5.day           = (unsigned int) (date2.tm_mday);
        ais->type5.minute        = (unsigned int) (time/(10000 * 60));
        ais->type5.hour          = (unsigned int) (ais->type5.minute / 60);
        ais->type5.minute =
            (unsigned int)(ais->type5.minute-(ais->type5.hour * 60));

        ais->type5.draught       = (unsigned int) (getleu16(bu, 51) / 10);
        ais->type5.dte           = (unsigned int) ((bu[73] >> 6) & 0x01);

        for (l = 0, cpy_stop = 0; l < 7; l++) {
            char next;

            next = (char) bu[9+l];
            if ((' ' > next) ||
                (0x7e < next)) {
                cpy_stop = 1;
            }
            if (0 == cpy_stop) {
                ais->type5.callsign[l] = next;
            } else {
                ais->type5.callsign[l] = 0;
            }
        }
        ais->type5.callsign[7]   = (char) 0;

        for (l = 0, cpy_stop = 0; l < AIS_SHIPNAME_MAXLEN; l++) {
            char next;

            next = (char) bu[16+l];
            if ((next < ' ') ||
                (next > 0x7e)) {
                cpy_stop = 1;
            }
            if (cpy_stop == 0) {
                ais->type5.shipname[l] = next;
            } else {
                ais->type5.shipname[l] = 0;
            }
        }
        ais->type5.shipname[AIS_SHIPNAME_MAXLEN] = (char) 0;

        for (l = 0, cpy_stop = 0; l < 20; l++) {
            char next;

            next = (char) bu[53+l];
            if ((next < ' ') ||
                (next > 0x7e)) {
                cpy_stop = 1;
            }
            if (cpy_stop == 0) {
                ais->type5.destination[l] = next;
            } else {
                ais->type5.destination[l] = 0;
            }
        }
        ais->type5.destination[20] = (char) 0;
#if NMEA2000_DEBUG_AIS
        printf("AIS: MMSI:  %09u\n",
               ais->mmsi);
        printf("AIS: name:  %-20.20s i:%8u c:%-8.8s b:%6u s:%6u p:%6u"
               "s:%6u dr:%4.1f\n",
               ais->type5.shipname,
               ais->type5.imo,
               ais->type5.callsign,
               ais->type5.to_bow,
               ais->type5.to_stern,
               ais->type5.to_port,
               ais->type5.to_starboard,
               ais->type5.draught / 10.0);
        printf("AIS: arrival:%-20.20s at %02u-%02u-%04d %02u:%0u\n",
               ais->type5.destination,
               ais->type5.day,
               ais->type5.month,
               date2.tm_year + 1900,
               ais->type5.hour,
               ais->type5.minute);
#endif  // end of #if NMEA2000_DEBUG_AIS
        decode_ais_channel_info(bu, len, 592, session);
        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


/*
 *   PGN 129798: AIS SAR Aircraft Position Report
 *
 * No test case for this message at the moment
 */
static gps_mask_t hnd_129798(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0xffffffffU)) {
        ais->type9.lon = (int)scale_int(getles32(bu, 5),
                                        (int64_t)(SHIFT32 *.06L));
        ais->type9.lat = (int)scale_int(getles32(bu, 9),
                                        (int64_t)(SHIFT32 *.06L));
        ais->type9.accuracy  = (bool)         ((bu[13] >> 0) & 0x01);
        ais->type9.raim      = (bool)         ((bu[13] >> 1) & 0x01);
        ais->type9.second    = (unsigned int) ((bu[13] >> 2) & 0x3f);
        ais->type9.course =
            (unsigned int)ais_direction((unsigned int)getleu16(bu, 14), 10.0);
        ais->type9.speed =
            (unsigned int)(getleu16(bu, 16) * MPS_TO_KNOTS * 0.01 / 0.1);
        ais->type9.radio     = (unsigned int) (getleu32(bu, 18) & 0x7ffff);
        ais->type9.alt       = (unsigned int) (getleu64(bu, 21)/1000000);
        ais->type9.regional  = (unsigned int) ((bu[29] >> 0) & 0xff);
        ais->type9.dte       = (unsigned int) ((bu[30] >> 0) & 0x01);
//      ais->type9.spare     = (bu[30] >> 1) & 0x7f;
        ais->type9.assigned  = 0;  // Not transmitted ????
        decode_ais_channel_info(bu, len, 163, session);

        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


/*
 *   PGN 129802: AIS Safety Related Broadcast Message
 *
 * No test case for this message at the moment
 */
static gps_mask_t hnd_129802(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0x3fffffff)) {
//      ais->type14.channel = (bu[ 5] >> 0) & 0x1f;
        strlcpy(ais->type14.text, (char *)&bu[6], sizeof(ais->type14.text));
        decode_ais_channel_info(bu, len, 40, session);

        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


/*
 *   PGN 129809: AIS Class B CS Static Data Report, Part A
 */
static gps_mask_t hnd_129809(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0xffffffffU)) {
        int index   = session->driver.aivdm.context[0].type24_queue.index;
        struct ais_type24a_t *saveptr =
            &session->driver.aivdm.context[0].type24_queue.ships[index];

        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA2000: AIS message 24A from %09u stashed.\n",
                 ais->mmsi);

        strlcpy(ais->type24.shipname, (char *)&bu[5],
                sizeof(ais->type24.shipname));
        strlcpy(saveptr->shipname, (char *)&bu[5], sizeof(saveptr->shipname));

        saveptr->mmsi = ais->mmsi;

        index += 1;
        index %= MAX_TYPE24_INTERLEAVE;
        session->driver.aivdm.context[0].type24_queue.index = index;

        decode_ais_channel_info(bu, len, 200, session);

        ais->type24.part = part_a;
        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


/*
 *   PGN 129810: AIS Class B CS Static Data Report, Part B
 */
static gps_mask_t hnd_129810(struct gps_device_t *session)
{
    unsigned char *bu = session->lexer.outbuffer;
    size_t len  = session->lexer.outbuflen;
    struct ais_t *ais =  &session->gpsdata.ais;

    if (0 != decode_ais_header(session->context, bu, len, ais, 0xffffffffU)) {
        int i;

        ais->type24.shiptype = (unsigned int) ((bu[ 5] >> 0) & 0xff);

        strlcpy(ais->type24.vendorid, (char *)&bu[6],
                sizeof(ais->type24.vendorid));
        strlcpy(ais->type24.callsign, (char *)&bu[13],
                sizeof(ais->type24.callsign));

        ais->type24.model = 0;
        ais->type24.serial = 0;

        if (AIS_AUXILIARY_MMSI(ais->mmsi)) {
            ais->type24.mothership_mmsi   = (unsigned int)getleu32(bu, 28);
        } else {
            uint16_t length, beam, to_bow, to_starboard;

            length                        =                 getleu16(bu, 20);
            beam                          =                 getleu16(bu, 22);
            to_starboard                  =                 getleu16(bu, 24);
            to_bow                        =                 getleu16(bu, 26);
            if ((length == 0xffff) || (to_bow       == 0xffff)) {
                length       = 0;
                to_bow       = 0;
            }
            if ((beam   == 0xffff) || (to_starboard == 0xffff)) {
                beam         = 0;
                to_starboard = 0;
            }
            ais->type24.dim.to_bow   = (unsigned int) (to_bow/10);
            ais->type24.dim.to_stern = (unsigned int) ((length-to_bow)/10);
            ais->type24.dim.to_port  = (unsigned int) ((beam-to_starboard)/10);
            ais->type24.dim.to_starboard  = (unsigned int) (to_starboard/10);
        }

        for (i = 0; i < MAX_TYPE24_INTERLEAVE; i++) {
            if (session->driver.aivdm.context[0].type24_queue.ships[i].mmsi ==
                ais->mmsi) {
                strlcpy(ais->type24.shipname,
                       (char *)session->driver.aivdm.context[0].type24_queue.ships[i].shipname,
                       sizeof(ais->type24.shipname));

                GPSD_LOG(LOG_PROG, &session->context->errout,
                         "NMEA2000: AIS 24B from %09u matches a 24A.\n",
                         ais->mmsi);
                // prevent false match if a 24B is repeated
                session->driver.aivdm.context[0].type24_queue.ships[i].mmsi = 0;
#if NMEA2000_DEBUG_AIS
                printf("AIS: MMSI:  %09u\n", ais->mmsi);
                printf("AIS: name:  %-20.20s v:%-8.8s c:%-8.8s b:%6u "
                       "s:%6u p:%6u s:%6u\n",
                       ais->type24.shipname,
                       ais->type24.vendorid,
                       ais->type24.callsign,
                       ais->type24.dim.to_bow,
                       ais->type24.dim.to_stern,
                       ais->type24.dim.to_port,
                       ais->type24.dim.to_starboard);
#endif  // of #if NMEA2000_DEBUG_AIS

                decode_ais_channel_info(bu, len, 264, session);
                ais->type24.part = both;
                return ONLINE_SET | AIS_SET;
            }
        }
#if NMEA2000_DEBUG_AIS
        printf("AIS: MMSI  :  %09u\n", ais->mmsi);
        printf("AIS: vendor:  %-8.8s c:%-8.8s b:%6u s:%6u p:%6u s:%6u\n",
               ais->type24.vendorid,
               ais->type24.callsign,
               ais->type24.dim.to_bow,
               ais->type24.dim.to_stern,
               ais->type24.dim.to_port,
               ais->type24.dim.to_starboard);
#endif  // of #if NMEA2000_DEBUG_AIS
        decode_ais_channel_info(bu, len, 264, session);
        ais->type24.part = part_b;
        return ONLINE_SET | AIS_SET;
    }
    return 0;
}


/*
 *   PGN 130306: NAV Wind Data
 */
static gps_mask_t hnd_130306(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 130310: NAV Water Temp., Outside Air Temp., Atmospheric Pressure
 */
static gps_mask_t hnd_130310(struct gps_device_t *session UNUSED)
{
    return 0;
}


/*
 *   PGN 130311: NAV Environmental Parameters
 */
static gps_mask_t hnd_130311(struct gps_device_t *session UNUSED)
{
    return 0;
}


/* keep list sorted!
 * Thanks to: https://www.isobus.net/isobus/pGNAndSPN/index
 */
static const PGN pgnlst[] = {
                             { 51968, 0, 0, NULL, "Process Data Message"},
                             { 58880, 0, 0, NULL, "Virtual Terminal-to-Node"},
                             { 59136, 0, 0, NULL, "Node-to-Virtual Terminal"},
                             { 59392, 0, 0, hnd_059392, "ISO Acknowledgment"},
                             { 59904, 0, 0, NULL, "Request"},
                             { 60160, 0, 0, NULL,
                               "Transport Protocol - Data Transfer"},
                             { 60416, 0, 0, NULL,
                               "Transport Protocol - Connection Mgmt"},
                             { 60928, 0, 0, NULL, "Address Claimed"},
                             { 61444, 0, 0, NULL,
                               "Electronic Engine Controller 1"},
                             { 65037, 0, 0, NULL, "Working Set Master "},
                             { 65039, 0, 0, NULL, "Language command"},
                             { 65040, 0, 0, NULL,
                              "Auxiliary valve 0 estimated flow"},
                             { 65041, 0, 0, NULL,
                              "Auxiliary valve 1 estimated flow"},
                             { 65042, 0, 0, NULL,
                              "Auxiliary valve 2 estimated flow"},
                             { 65043, 0, 0, NULL,
                              "Auxiliary valve 3 estimated flow"},
                             { 65089, 0, 0, NULL, "Lighting command"},
                             { 65093, 0, 0, NULL,
                              "Primary or Rear Hitch Status"},
                             { 65094, 0, 0, NULL,
                              "Secondary or Front Hitch Status"},
                             { 65096, 0, 0, NULL,
                              "Wheel-based Speed and Distance"},
                             { 65097, 0, 0, NULL,
                              "Ground-based Speed and Distance"},
                             { 60928, 0, 0, hnd_060928, "ISO Address Claim"},
                             { 65226, 0, 0, NULL,
                              "Active Diagnostic Trouble Codes"},
                             {126208, 0, 0, hnd_126208,
                              "NMEA Command/Request/Acknowledge"},
                             {126464, 1, 0, hnd_126464,
                              "ISO Transmit/Receive PGN List"},
                             {126720, 1, 0, hnd_126720,
                              "Maretron proprietary"},
                             {126992, 0, 0, hnd_126992, "GNSS System Time"},
                             {126996, 1, 0, hnd_126996,
                              "ISO Product Information"},
                             {127245, 0, 4, hnd_127245, "NAV Rudder"},
                             {127250, 0, 4, hnd_127250, "NAV Vessel Heading"},
                             {127258, 0, 0, hnd_127258,
                             "GNSS Magnetic Variation"},
                             {127258, 0, 0, hnd_127258, "NAV Vessel Heading"},
                             {127506, 1, 3, hnd_127506,
                             "PWR DC Detailed Status"},
                             {127508, 1, 3, hnd_127508, "PWR Battery Status"},
                             {127513, 1, 3, hnd_127513,
                              "PWR Battery Configuration Status"},
                             {128259, 0, 4, hnd_128259, "NAV Speed"},
                             {128267, 0, 4, hnd_128267, "NAV Water Depth"},
                             {128275, 1, 4, hnd_128275, "NAV Distance Log"},
                             {129025, 0, 1, hnd_129025,
                              "GNSS Position Rapid Update"},
                             {129026, 0, 1, hnd_129026,
                              "GNSS COG and SOG Rapid Update"},
                             {129029, 1, 1, hnd_129029,
                              "GNSS Position Data"},
                             {129038, 1, 2, hnd_129038,
                              "AIS Class A Position Report"},
                             {129039, 1, 2, hnd_129039,
                              "AIS Class B Position Report"},
                             {129040, 1, 2, hnd_129040,
                              "AIS Class B Extended Position Report"},
                             {129283, 0, 0, hnd_129283,
                              "NAV Cross Track Error"},
                             {129284, 1, 0, hnd_129284, "NAV Navigation Data"},
                             {129285, 1, 0, hnd_129285,
                              "NAV Navigation - Route/WP Information"},
                             {129539, 0, 1, hnd_129539, "GNSS DOPs"},
                             {129540, 1, 1, hnd_129540,
                              "GNSS Satellites in View"},
                             {129793, 1, 2, hnd_129793,
                              "AIS  UTC and Date report"},
                             {129794, 1, 2, hnd_129794,
                              "AIS Class A Static and Voyage Related Data"},
                             {129798, 1, 2, hnd_129798,
                              "AIS  SAR Aircraft Position Report"},
                             {129802, 1, 2, hnd_129802,
                              "AIS  Safety Related Broadcast Message"},
                             {129809, 1, 2, hnd_129809,
                              "AIS Class B CS Static Data Report, Part A"},
                             {129810, 1, 2, hnd_129810,
                              "AIS Class B CS Static Data Report, Part B"},
                             {130306, 0, 4, hnd_130306, "NAV Wind Data"},
                             {130310, 0, 4, hnd_130310,
                              "NAV Water Temp., Outside Air Temp., "
                              "Atmospheric Pressure"},
                             {130311, 0, 4, hnd_130311,
                              "NAV Environmental Parameters"},
                             {0     , 0, 0, NULL, "**error**"},
};


static const PGN *search_pgnlist(unsigned int pgn)
{
    int l1 = 0;

    // since list is sorted, we can early out
    for (l1 = 0; pgn > pgnlst[l1].pgn; l1++) {
        if (0 == pgnlst[l1].pgn) {
            break;
        }
    }
    if (pgnlst[l1].pgn == pgn) {
        return &pgnlst[l1];
    }
    return NULL;
}

static void find_pgn(struct can_frame *frame, struct gps_device_t *session)
{
    unsigned can_net;
    unsigned source_prio;
    unsigned daddr;
    unsigned source_pgn;
    unsigned source_addr;

    session->driver.nmea2000.workpgn = NULL;
    can_net = session->driver.nmea2000.can_net;

    GPSD_LOG(LOG_RAW, &session->context->errout,
             "NMEA2000 find_pgn() can_id x%x can_net %u\n",
             frame->can_id, can_net);

    if (NMEA2000_NETS <= can_net) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 find_pgn: Invalid can network %u.\n", can_net);
        return;
    }

    if (frame->can_id & CAN_ERR_FLAG) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 CAN_ERR_FLAG set x%x.\n", frame->can_id);
        return;
    }

    if (!(frame->can_id & CAN_EFF_FLAG)) {
        // we got RTR or 2.0A CAN frame, not used.  SHould have been filtered.
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA2000 CAN_EFF_FLAG not set x%x.\n", frame->can_id);
        return;
    }
#if LOG_FILE
    if (NULL != logFile) {
        struct timespec  msgTime;

        clock_gettime(CLOCK_REALTIME, &msgTime);
        (void)fprintf(logFile,
                      "(%010lld.%06ld) can0 %08x#",
                      (long long)msgTime.tv_sec,
                      msgTime.tv_nsec / 1000,
                      frame->can_id & 0x1ffffff);
        if (0 < (frame->can_dlc & 0x0f)) {
            int l1;
            for (l1 = 0; l1 < (frame->can_dlc & 0x0f); l1++) {
                (void)fprintf(logFile, "%02x", frame->data[l1]);
            }
        }
        (void)fprintf(logFile, "\n");
    }
#endif  // of if LOG_FILE
    source_addr = frame->can_id & 0x0ff;
    if (NMEA2000_ADDRS <= source_addr) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "NMEA2000 ignoring SA %u.\n", source_addr);
        return;
    }
    session->driver.nmea2000.can_msgcnt += 1;
    source_pgn = (frame->can_id >> 8) & 0x1ffff;
    source_prio = (frame->can_id >> 26) & 0x7;

    if (240 > ((source_pgn & 0x0ff00) >> 8)) {
        daddr  = source_pgn & 0x000ff;
        source_pgn  = source_pgn & 0x1ff00;
    } else {
        daddr = 0xff;
    }
    GPSD_LOG(LOG_DATA, &session->context->errout,
             "NMEA2000: source_prio %u SA %u daddr %u\n",
             source_prio, source_addr, daddr);

    if (!session->driver.nmea2000.source_addr) {
        unsigned int l1, l2;

        for (l1 = 0; l1 < NMEA2000_NETS; l1++) {
            for (l2 = 0; l2 < NMEA2000_ADDRS; l2++) {
                if (session == nmea2000_units[l1][l2]) {
                    session->driver.nmea2000.source_addr = l2;
                    session->driver.nmea2000.sa_valid = true;
                    session->driver.nmea2000.can_net = l1;
                    can_net = l1;
                }
            }
        }

        session->driver.nmea2000.source_addr = source_addr;
        session->driver.nmea2000.sa_valid = true;
        nmea2000_units[can_net][source_addr] = session;
    }

    if (source_addr == session->driver.nmea2000.source_addr) {
        // current source_addr.  Current net???
        const PGN *work = search_pgnlist(source_pgn);

        if (NULL == work) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NMEA2000: pgn %u SA %u pgn not found\n",
                     source_pgn, source_addr);
        } else if (0 == work->fast) {
            // not FAST, one packet is one complete message

            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA2000: pgn %6d:%s \n", work->pgn, work->name);
            session->driver.nmea2000.workpgn = (const void *)work;
            session->lexer.outbuflen =  frame->can_dlc & 0x0f;  // max 15
            memcpy(session->lexer.outbuffer, frame->data,
                   session->lexer.outbuflen);
        } else if (0 == (frame->data[0] & 0x1f)) {
            // FAST, first packet of multi packet message

            // max frame data 223
            session->driver.nmea2000.fast_packet_len = frame->data[1];
            session->driver.nmea2000.idx = frame->data[0];
            GPSD_LOG(LOG_IO, &session->context->errout,
                     "NMEA2000: Set idx %u SA %u flen %2x %6d\n",
                     frame->data[0],
                     session->driver.nmea2000.source_addr,
                     frame->data[1],
                     source_pgn);
            session->lexer.inbuflen = 6;
            session->driver.nmea2000.idx += 1;
            memcpy(session->lexer.inbuffer, &frame->data[2], 6);
            GPSD_LOG(LOG_DATA, &session->context->errout,
                     "NMEA2000: pgn %6d:%s \n", work->pgn, work->name);
        } else if (frame->data[0] == session->driver.nmea2000.idx) {
            /* FAST, the expected next packet of multi packet message.
             * we assume FAST packets come in sequence order.
             * Not always true.
             * See: https://canboat.github.io/canboat/canboat.html
             * Secton: packet framing. */
            unsigned l2;

            // FIXME: check inbuflen and fast_packet_len
            l2 = session->driver.nmea2000.fast_packet_len -
                 session->lexer.inbuflen;
            if (223 < l2) {
                // WTF??
                l2 = 0;
            } else if (7 < l2) {
                // max 7 per packet
                l2 = 7;
            }
            // take up to 7 bytes, of 8.  1st byte is idx.
            memcpy(&session->lexer.inbuffer[session->lexer.inbuflen],
                   &frame->data[1], l2);
            session->lexer.inbuflen += l2;

            if (session->lexer.inbuflen ==
                session->driver.nmea2000.fast_packet_len) {
                // Got a complete message
                GPSD_LOG(LOG_IO, &session->context->errout,
                         "NMEA2000: Fast done  idx %2x/%2x SA %u "
                         "flen %2x %6d\n",
                         session->driver.nmea2000.idx,
                         frame->data[0],
                         session->driver.nmea2000.source_addr,
                         (unsigned)session->driver.nmea2000.fast_packet_len,
                         source_pgn);
                session->driver.nmea2000.workpgn = (const void *)work;
                session->lexer.outbuflen =
                    session->driver.nmea2000.fast_packet_len;
                memcpy(session->lexer.outbuffer, session->lexer.inbuffer,
                       session->lexer.outbuflen);
                session->driver.nmea2000.fast_packet_len = 0;
            } else {
                // More to come.
                session->driver.nmea2000.idx += 1;
            }
        } else {
            /* error? or packets out of order?
             * reset FAST expected?? */
            GPSD_LOG(LOG_WARN, &session->context->errout,
                 "NMEA2000: Fast error idx%2x/%2x SA %u flen %2x %6d\n",
                 session->driver.nmea2000.idx,
                 frame->data[0],
                 session->driver.nmea2000.source_addr,
                 (unsigned)session->driver.nmea2000.fast_packet_len,
                 source_pgn);
        }
    } else if (NULL == nmea2000_units[can_net][source_addr]) {
        // unknown net/SA, add it as a new device.
        char buffer[GPS_PATH_MAX];

        (void)snprintf(buffer, sizeof(buffer), "nmea2000://%s:%u",
                       can_interface_name[can_net],
                       source_addr);
        if (NULL != gpsd_add_device) {
            if (gpsd_add_device(buffer, true)) {
                GPSD_LOG(LOG_INF, &session->context->errout,
                         "NMEA2000: gpsd_add_device(%s)\n", buffer);
            } else {
                GPSD_LOG(LOG_ERROR, &session->context->errout,
                         "NMEA2000: gpsd_add_device(%s) failed\n",
                         buffer);
            }
        }
    } // else, known net/SA that is not this net/SA.  Ignore it.
}


static ssize_t nmea2000_get(struct gps_device_t *session)
{
    struct can_frame frame;
    ssize_t          status;

    errno = 0;
    session->lexer.outbuflen = 0;
    // FIXME: read() into a struct is not guaranteed in C
    // sizeof(frame) === 16
    status = read(session->gpsdata.gps_fd, &frame, sizeof(frame));
    if ((ssize_t)sizeof(frame) == status) {
        session->lexer.type = NMEA2000_PACKET;
        find_pgn(&frame, session);

        return frame.can_dlc & 0x0f;
    }
    if (-1 == status &&
        EAGAIN == errno &&
        LOG_PROG > session->context->errout.debug) {
        /* nothing to read, try again later
         * do not log at low log levels */
        return 0;
    }
    // long cast for 32-bit.
    GPSD_LOG(LOG_WARN, &session->context->errout,
             "NMEA2000 nmea2000_get() status %ld %s(%d) \n",
             (long)status, strerror(errno), errno);
    return 0;
}

static gps_mask_t nmea2000_parse_input(struct gps_device_t *session)
{
    gps_mask_t mask = 0;
    const PGN *work;
    char buf[128];

    GPSD_LOG(LOG_RAW, &session->context->errout,
             "NMEA2000 nmea2000_parse_input(%s)\n",
             gps_hexdump(buf, sizeof(buf),
                         session->lexer.outbuffer,
                         session->lexer.outbuflen));
    work = (const PGN *)session->driver.nmea2000.workpgn;

    if (NULL != work) {
        print_data(session);
        if (NULL != work->func) {
            mask = (work->func)(session);
        } // else, no decode available.
        session->driver.nmea2000.workpgn = NULL;
    }
    session->lexer.outbuflen = 0;

    return mask;
}


int nmea2000_open(struct gps_device_t *session)
{
    char interface_name[GPS_PATH_MAX];
    socket_t sock;
    int status;
    int source_addr = -1;
    int can_net = -1;
    unsigned int l;
    struct ifreq ifr;
    struct sockaddr_can addr;
    char *sa_ptr = NULL;
    can_err_mask_t err_mask;
    int rcvbuf_size = 1000000;  // requested receiver buffer size
    int curr_rcvbuf_size;
    socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);
    struct can_filter can_filter;
    size_t interface_name_len;

    // FIXME: if this was a live socket, then we left orphan fd.
    INVALIDATE_SOCKET(session->gpsdata.gps_fd);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000 nmea2000_open(%s)\n",
             session->gpsdata.dev.path);

    session->driver.nmea2000.can_net = 0;

    // skip the leading "nmea2000://"
    (void)strlcpy(interface_name, session->gpsdata.dev.path + 11,
                  sizeof(interface_name));

    interface_name_len = strnlen(interface_name, sizeof(interface_name));
    for (l = 0; l < interface_name_len; l++) {
        if (':' == interface_name[l]) {
            sa_ptr = &interface_name[l + 1];
            interface_name[l] = 0;
            continue;
        }
        if (NULL != sa_ptr) {
            if (0 == isdigit(interface_name[l])) {
                GPSD_LOG(LOG_ERROR, &session->context->errout,
                         "NMEA2000 open: Invalid char x%x in source addr.\n",
                         interface_name[l]);
                return -1;
            }
        }
    }

    if (NULL != sa_ptr) {
        source_addr = atoi(sa_ptr);
        if ((0 > source_addr) ||
            (NMEA2000_ADDRS <= source_addr)) {
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "NMEA2000 open: SA %u out of range.\n",
                     source_addr);
            return -1;
        }
        for (l = 0; l < NMEA2000_NETS; l++) {
            if (0 == strncmp(can_interface_name[l],
                             interface_name,
                             MIN(sizeof(interface_name),
                                 sizeof(can_interface_name[l])))) {
                can_net = l;
                break;
            }
        }
        if (0 > can_net) {
            for (l = 0; l < NMEA2000_NETS; l++) {
                if (0 == can_interface_name[l][0]) {
                    can_net = l;
                    break;
                }
            }
        }
        if (0 > can_net) {
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "NMEA2000 open: CAN device not open: %s .\n",
                     interface_name);
            return -1;
        }
    } else {
        for (l = 0; l < NMEA2000_NETS; l++) {
            if (0 == strncmp(can_interface_name[l],
                             interface_name,
                             MIN(sizeof(interface_name),
                                 sizeof(can_interface_name[l])))) {
                GPSD_LOG(LOG_ERROR, &session->context->errout,
                         "NMEA2000 open: CAN device duplicate open: %s .\n",
                         interface_name);
                return -1;
            }
        }
        for (l = 0; l < NMEA2000_NETS; l++) {
            if (0 == can_interface_name[l][0]) {
                can_net = l;
                break;
            }
        }
        if (0 > can_net) {
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "NMEA2000 open: Too many CAN networks open.\n");
            return -1;
        }
    }

    // Create the socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (BAD_SOCKET(sock)) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 open: socket(PF_CAN) %s(%d).\n",
                 strerror(errno), errno);
        return -1;
    }

    status = fcntl(sock, F_SETFL, O_NONBLOCK);
    if (0 != status) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 open: fcntl(O_NONBLOCK) %s(%d).\n",
                 strerror(errno), errno);
        close(sock);
        return -1;
    }

    // turn on CANBUS error reporting
    err_mask = CAN_ERR_ACK | CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_LOSTARB |
               CAN_ERR_PROT | CAN_ERR_RESTARTED | CAN_ERR_TRX |
               CAN_ERR_TX_TIMEOUT;

    status = setsockopt(sock, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                        &err_mask, sizeof(err_mask));
    if (0 != status) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 open: setsockopt() %s(%d)\n",
                 strerror(errno), errno);
    }

    /* enbiggen the receiver buffer size
     * try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
    if (0 > setsockopt(sock, SOL_SOCKET, SO_RCVBUFFORCE,
                       &rcvbuf_size, sizeof(rcvbuf_size))) {
            GPSD_LOG(LOG_ERROR, &session->context->errout,
                     "NMEA2000 open:SO_RCVBUFFORCE failed try RCVBUF. "
                     "%s(%d)\n",
                     strerror(errno), errno);
            if (0 > setsockopt(sock, SOL_SOCKET, SO_RCVBUF,
                               &rcvbuf_size, sizeof(rcvbuf_size))) {
                GPSD_LOG(LOG_ERROR, &session->context->errout,
                         "NMEA2000 open:setsockopt(SO_RCVBUF) %s(%d).\n",
                         strerror(errno), errno);
            }
    }
    if (0 > getsockopt(sock, SOL_SOCKET, SO_RCVBUF,
                   &curr_rcvbuf_size, &curr_rcvbuf_size_len)) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 open:getsockopt(SO_RCVBUF) %s(%d)\n",
                 strerror(errno), errno);
    } else {
        GPSD_LOG(LOG_NOTICE, &session->context->errout,
                 "NMEA2000 open:getsockopt(SO_RCVBUF) =  %d\n",
                 curr_rcvbuf_size);
    }

    // Locate the interface you wish to use
    (void)strlcpy(ifr.ifr_name, interface_name, sizeof(ifr.ifr_name));
    // ifr.ifr_ifindex gets filled with that device's index
    status = ioctl(sock, SIOCGIFINDEX, &ifr);

    if (0 != status) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 open: can not find CAN device.\n");
        close(sock);
        return -1;
    }

    // Select that CAN interface, and bind the socket to it.
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    status = bind(sock, (struct sockaddr*)&addr, sizeof(addr) );
    if (0 != status) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 open: bind failed.\n");
        close(sock);
        return -1;
    }

    gpsd_switch_driver(session, "NMEA2000");
    session->gpsdata.gps_fd = sock;
    session->sourcetype = SOURCE_CAN;
    session->servicetype = SERVICE_SENSOR;
    session->driver.nmea2000.can_net = can_net;

    (void)strlcpy(can_interface_name[can_net],
                  interface_name, sizeof(can_interface_name[0]));

    if (NULL == sa_ptr) {
        //  Only include EFF CAN frames
        can_filter.can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG;
        can_filter.can_id = CAN_EFF_FLAG;

        session->driver.nmea2000.sa_valid = false;
        // no source addr, yet.
        memset(nmea2000_units[can_net], 0, sizeof(nmea2000_units[can_net]));
    } else {
        // Only include EFF CAN frames with the specific source address
        can_filter.can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | 0xff;
        can_filter.can_id = CAN_EFF_FLAG | source_addr;

        nmea2000_units[can_net][source_addr] = session;
        session->driver.nmea2000.source_addr = source_addr;
        session->driver.nmea2000.sa_valid = true;
    }

    status = setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER,
                        &can_filter, sizeof(can_filter));
    if (0 != status) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NMEA2000 open:setsockopt(CAN_RAW_FILTER) %s(%d).\n",
                 strerror(errno), errno);
        close(sock);
        session->gpsdata.gps_fd = -1;
        return -1;
    }

    // how do we know the speed???
    session->gpsdata.dev.parity = 'N';
    session->gpsdata.dev.baudrate = 250000;
    session->gpsdata.dev.stopbits = 0;
    return session->gpsdata.gps_fd;
}

void nmea2000_close(struct gps_device_t *session)
{
    if (BAD_SOCKET(session->gpsdata.gps_fd)) {
        return;
    }

    // cast for 32-bit ints.
    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NMEA2000: close(%ld) in nmea2000_close(%s)\n",
             (long)session->gpsdata.gps_fd, session->gpsdata.dev.path);
    (void)close(session->gpsdata.gps_fd);
    INVALIDATE_SOCKET(session->gpsdata.gps_fd);

    if (session->driver.nmea2000.sa_valid) {
        unsigned int l1, l2;

        for (l1 = 0; l1 < NMEA2000_NETS; l1++) {
            for (l2 = 0; l2 < NMEA2000_ADDRS; l2++) {
                if (session == nmea2000_units[l1][l2]) {
                    memset(&session->driver.nmea2000, 0,
                           sizeof(session->driver.nmea2000));
                    nmea2000_units[l1][l2] = NULL;
                }
            }
        }
    }
}

// *INDENT-OFF*
const struct gps_type_t driver_nmea2000 = {
    .type_name      = "NMEA2000",       // full name of type
    .packet_type    = NMEA2000_PACKET,  // associated lexer packet type
    .flags          = DRIVER_STICKY,    // remember this
    .trigger        = NULL,             // detect their main sentence
    .channels       = 12,               // not an actual GPS at all
    .probe_detect   = NULL,
    .get_packet     = nmea2000_get,     // how to get a packet
    .parse_packet   = nmea2000_parse_input,     // how to interpret a packet
    .rtcm_writer    = NULL,             // Don't send RTCM to this
    .init_query     = NULL,             // non-perturbing query
    .event_hook     = NULL,
    .speed_switcher = NULL,             // no speed switcher
    .mode_switcher  = NULL,             // no mode switcher
    .rate_switcher  = NULL,             // no rate switcher
    .min_cycle.tv_sec  = 1,             // not relevant, no rate switch
    .min_cycle.tv_nsec = 0,             // not relevant, no rate switch
    .control_send   = NULL,             // how to send control strings
    .time_offset     = NULL,
};
// *INDENT-ON*

// end

#else   // of  defined(NMEA2000_ENABLE)
/* dummy variable to some old linkers do not complain about empty
 * object file */
int nmea2000_dummy = 1;
#endif  // of  defined(NMEA2000_ENABLE)

// vim: set expandtab shiftwidth=4
