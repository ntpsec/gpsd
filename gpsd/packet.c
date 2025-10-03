/****************************************************************************

NAME:
   packet.c -- a packet-sniffing engine for reading from GPS devices

DESCRIPTION:

Initial conditions of the problem:

1. We have a file descriptor open for (possibly non-blocking) read. The device
   on the other end is sending packets at us.

2. It may require more than one read to gather a packet.  Reads may span packet
   boundaries.

3. There may be leading garbage before the first packet.  After the first
   start-of-packet, the input should be well-formed.

The problem: how do we recognize which kind of packet we're getting?

No need to handle Garmin USB binary, we know that type by the fact we're
connected to the Garmin kernel driver.  But we need to be able to tell the
others apart and distinguish them from baud barf.

PERMISSIONS
   This file is Copyright 2010 by the GPSD project
   SPDX-License-Identifier: BSD-2-clause

***************************************************************************/

#include "../include/gpsd_config.h"  // must be before all includes

#include <arpa/inet.h>          // for htons()
#include <ctype.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>             // for strtol()
#include <string.h>
#include <sys/time.h>           // for struct timeval
#include <sys/types.h>
#include <unistd.h>

#include "../include/bits.h"
#include "../include/driver_greis.h"
#include "../include/gpsd.h"
#include "../include/crc24q.h"
#include "../include/strfuncs.h"

/*
 * The packet-recognition state machine.  This takes an incoming byte stream
 * and tries to segment it into packets.  There are four types of packets:
 *
 * 1) Comments. These begin with # and end with \r\n.
 *
 * 2) NMEA lines.  These begin with $, and with \r\n, and have a checksum.
 *    Except $PASHR packets have no checksum!
 *
 * 3) Checksummed binary packets.  These begin with some fixed leader
 *    character(s), have a length embedded in them, and end with a
 *    checksum (and possibly some fixed trailing bytes).
 *
 * 4) ISGPS packets. The input may be a bitstream containing IS-GPS-200
 *    packets.  Each includes a fixed leader byte, a length, and check bits.
 *    In this case, it is not guaranteed that packet starts begin on byte
 *    boundaries; the recognizer has to run a separate state machine against
 *    each byte just to achieve synchronization lock with the bitstream.
 *
 * 5) Un-checksummed binary packets.  Like case 3, but without
 *    a checksum it's much easier to get a false match from garbage.
 *    The packet recognizer gives checksummed types higher priority.
 *
 * Adding support for a new GPS protocol typically requires adding state
 * transitions to support whatever binary packet structure it has.  The
 * goal is for the lexer to be able to cope with arbitrarily mixed packet
 * types on the input stream.  This is a requirement because (1) sometimes
 * gpsd wants to switch a device that supports both NMEA and a binary
 * packet protocol to the latter for more detailed reporting, and (b) in
 * the presence of device hotplugging, the type of GPS report coming
 * in is subject to change at any time.
 *
 * Caller should consume a packet when it sees one of the *_RECOGNIZED
 * states.  It's good practice to follow the _RECOGNIZED transition
 * with one that recognizes a leader of the same packet type rather
 * than dropping back to ground state -- this for example will prevent
 * the state machine from hopping between recognizing TSIP and
 * EverMore packets that both start with a DLE.
 *
 * Error handling is brutally simple; any time we see an unexpected
 * character, go to GROUND_STATE and reset the machine (except that a
 * $ in an NMEA payload only resets back to NMEA_DOLLAR state).  Because
 * another good packet will usually be along in less than a second
 * repeating the same data, Boyer-Moore-like attempts to do parallel
 * recognition beyond the headers would make no sense in this
 * application, they'd just add complexity.
 *
 * The NMEA portion of the state machine allows the following talker IDs:
 *      $BD -- Beidou
 *      $EC -- Electronic Chart Display & Information System (ECDIS)
 *      $GA -- Galileo
 *      $GB -- Beidou
 *      $GL -- GLONASS, according to IEIC 61162-1
 *      $GN -- Mixed GPS and GLONASS data, according to IEIC 61162-1
 *      $GP -- Global Positioning System.
 *      $GY -- Unicore Gyro
 *      $HC -- Heading/compass (Airmar PB200).
 *      $II -- Integrated Instrumentation (Raytheon's SeaTalk system).
 *      $IN -- Integrated Navigation (Garmin uses this).
 *      $P  -- Vendor-specific sentence
 *      $QZ -- QZSS GPS augmentation system
 *      $SD -- Depth Sounder
 *      $SN -- Unicore Sensor data
 *      $ST -- $STI, Skytraq Debug Output
 *      $TI -- Turn indicator (Airmar PB200).
 *      $WI -- Weather instrument (Airmar PB200, Radio Ocean ROWIND,
 *                                 Vaisala WXT520).
 *      $YX -- Transducer (used by some Airmar equipment including PB100)
 *
 *      !AB -- NMEA 4.0 Base AIS station
 *      !AD -- MMEA 4.0 Dependent AIS Base Station
 *      !AI -- Mobile AIS station
 *      !AN -- NMEA 4.0 Aid to Navigation AIS station
 *      !AR -- NMEA 4.0 AIS Receiving Station
 *      !AX -- NMEA 4.0 Repeater AIS station
 *      !AS -- NMEA 4.0 Limited Base Station
 *      !AT -- NMEA 4.0 AIS Transmitting Station
 *      !BS -- Base AIS station (deprecated in NMEA 4.0)
 *      !SA -- NMEA 4.0 Physical Shore AIS Station
 */

enum
{
#include "../include/packet_states.h"
};

static char *state_table[] = {
#include "../include/packet_names.h"
};

#define SOH     (unsigned char)0x01
#define DLE     (unsigned char)0x10
#define STX     (unsigned char)0x02
#define ETX     (unsigned char)0x03
#define MICRO   (unsigned char)0xb5

#if defined(TSIP_ENABLE)
// Maximum length a TSIP packet can be
#define TSIP_MAX_PACKET 255
#endif

unsigned casic_checksum(unsigned char *buf, size_t len)
{
    // CASIC uses a 32 bit little-endian checksum
    // All payloads sizes are 4-byte aligned
    size_t idx;
    unsigned crc_computed = 0;

    for (idx = 0; idx < len; idx += 4) {
        crc_computed += getleu32(buf, idx);
    }
    return crc_computed;
}

#ifdef ONCORE_ENABLE
static size_t oncore_payload_cksum_length(unsigned char id1, unsigned char id2)
{
    size_t l;

    /* For the packet sniffer to not terminate the message due to
     * payload data looking like a trailer, the known payload lengths
     * including the checksum are given.  Return -1 for unknown IDs.
     */

#define ONCTYPE(id2,id3) ((((unsigned int)id2) << 8) | (id3))

    switch (ONCTYPE(id1,id2)) {
    // A...
    case ONCTYPE('A','a'):
        // time of day
        l = 10;
        break;
    case ONCTYPE('A','b'):
        // GMT offset
        l = 10;
        break;
    case ONCTYPE('A','c'):
        // date
        l = 11;
        break;
    case ONCTYPE('A','d'):
        // latitude
        l = 11;
        break;
    case ONCTYPE('A','e'):
        // longitude
        l = 11;
        break;
    case ONCTYPE('A','f'):
        // height
        l = 15;
        break;
    case ONCTYPE('A','g'):
        // satellite mask angle
        l = 8;
        break;
    // Command "Ao" gives "Ap" response (select datum)
    case ONCTYPE('A','p'):
        // set user datum / select datum
        l = 25;
        break;
    case ONCTYPE('A','q'):
        // atmospheric correction mode
        l = 8;
        break;
    case ONCTYPE('A','s'):
        // position-hold position
        l = 20;
        break;
    case ONCTYPE('A','t'):
        // position-hold mode
        l = 8;
        break;
    case ONCTYPE('A','u'):
        // altitude hold height
        l = 12;
        break;
    case ONCTYPE('A','v'):
        // altitude hold mode
        l = 8;
        break;
    case ONCTYPE('A','w'):
        // time mode
        l = 8;
        break;
    case ONCTYPE('A','y'):
        // 1PPS offset
        l = 11;
        break;
    case ONCTYPE('A','z'):
        // 1PPS cable delay
        l = 11;
        break;
    case ONCTYPE('A','N'):
        // velocity filter
        l = 8;
        break;
    case ONCTYPE('A','O'):
        // RTCM report mode
        l = 8;
        break;
    case ONCTYPE('A','P'):
        // pulse mode
        l = 8;
        break;

    // B...
    case ONCTYPE('B','b'):
        // visible satellites status
        l = 92;
        break;
    case ONCTYPE('B','j'):
        // leap seconds pending
        l = 8;
        break;
    case ONCTYPE('B','o'):
        // UTC offset status
        l = 8;
        break;

    // C...
    case ONCTYPE('C','b'):
        // almanac output ("Be" response)
        l = 33;
        break;
    case ONCTYPE('C','c'):
        // ephemeris data input ("Bf")
        l = 80;
        break;
    case ONCTYPE('C','f'):
        // set-to-defaults
        l = 7;
        break;
    // Command "Ci" (switch to NMEA, GT versions only) has no response
    case ONCTYPE('C','h'):
        // almanac input ("Cb" response)
        l = 9;
        break;
    case ONCTYPE('C','j'):
        // receiver ID
        l = 294;
        break;
    case ONCTYPE('C','k'):
        // pseudorng correction inp. ("Ce")
        l = 7;
        break;

    // E...
    case ONCTYPE('E','a'):
        // position/status/data
        l = 76;
        break;
    case ONCTYPE('E','n'):
        // time RAIM setup and status
        l = 69;
        break;
    case ONCTYPE('E','q'):
        // ASCII position
        l = 96;
        break;

    // F...
    case ONCTYPE('F','a'):
        // self-test
        l = 9;
        break;

    // S...
    case ONCTYPE('S','z'):
        // system power-on failure
        l = 8;
        break;

    default:
        return 0;
    }

    return l - 6;               // Subtract header and trailer.
}
#endif  // ONCORE_ENABLE

#ifdef GREIS_ENABLE

// Convert hex char to binary form. Requires that c be a hex char.
static unsigned long greis_hex2bin(char c)
{
    if (('a' <= c) &&
        ('f' >= c)) {
        c = c + 10 - 'a';
    } else if (('A' <= c) &&
               ('F' >= c)) {
        c = c + 10 - 'A';
    } else if (('0' <= c) &&
               ('9' >= c)) {
        c -= '0';
    }
    // FIXME: No error handling?

    return c;
}

#endif  // GREIS_ENABLE

/* nmea_checksum(*errout, buf) -- check NMEA checksum for message in buffer.
 * Also handles !AI checksums.
 *
 * Return: True = Checksum good
 *         False -- checksum bad
 */
static bool nmea_checksum(const struct gpsd_errout_t *errout,
                          const char *buf, const char *endp)
{
    bool checksum_ok = true;
    const char *end;
    unsigned int n, csum = 0;
    char csum_s[3] = { '0', '0', '0' };

    /* These have no checksum:
     *  GPS-320FW emits $PLCS
     *  MTK-3301 emits $POLYN
     *  Skytraq S2525F8-BD-RTK emits $STI
     *  Telit SL869 emits $GPTXT
     *  Ashtech (old!) $PASHR,MCA and $PASHR,PBN with no checksum
     * All undocumented,  Let them fail, except $STI.
     */
    if (str_starts_with(buf, "$STI,")) {
        // Let this one go...
        return true;
    }

    /* Some messages, like !AIVMD, !AIVMO, can have "stuff" after the
     * checksum.  Some messages can have "*" in the body of a message.
     * At least one GPS (the Firefly 1a) emits \r\r\n at the end.
     *
     * So scan backwards until we find the *.  Use the 2 chars to the
     * right as the checksum.
     */
    for (end = endp - 1; buf < end; end--) {
        if ('*' == *end) {
            break;
        }
    }

    if ('*' != *end) {
        // no asterisk found
        return false;
    }

    /* Verify checksum is 2 hex digits.  Irnoge trailing stuff.
     * Magellan EC-10X has lower case hex in checksum. It is rare.  */
    if (!isxdigit((unsigned char) end[1]) ||
        !isxdigit((unsigned char) end[2])) {
        return false;
    }

    // compute the checksum
    for (n = 1; buf + n < end; n++) {
        csum ^= buf[n];
    }
    (void)snprintf(csum_s, sizeof(csum_s), "%02X", csum);
    checksum_ok = (csum_s[0] == toupper((int)end[1]) &&
                   csum_s[1] == toupper((int)end[2]));
    if (!checksum_ok) {
        GPSD_LOG(LOG_WARN, errout,
                 "bad checksum in NMEA packet; got %c%c expected %s.\n",
                 end[1], end[2], csum_s);
    }

    return checksum_ok;
}

// push back the last character grabbed, setting a specified state
static bool character_pushback(struct gps_lexer_t *lexer,
                               unsigned int newstate)
{
    --lexer->inbufptr;
    --lexer->char_counter;
    lexer->state = newstate;
    if (lexer->errout.debug >= LOG_RAW2) {
        unsigned char c = *lexer->inbufptr;

        GPSD_LOG(LOG_RAW, &lexer->errout,
                 "%08ld: character '%c' [%02x]  pushed back, state set to %s\n",
                 lexer->char_counter,
                 (isprint((int)c) ? c : '.'), c,
                 state_table[lexer->state]);
    }

    return false;
}

// shift the input buffer to discard one character and reread data
static void character_discard(struct gps_lexer_t *lexer)
{
    memmove(lexer->inbuffer, lexer->inbuffer + 1, (size_t)-- lexer->inbuflen);
    lexer->inbufptr = lexer->inbuffer;
    if (lexer->errout.debug >= LOG_RAW1) {
        char scratchbuf[MAX_PACKET_LENGTH*4+1];

        GPSD_LOG(LOG_RAW1, &lexer->errout,
                 "Character discarded, buffer %zu chars = %s\n",
                 lexer->inbuflen,
                 gpsd_packetdump(scratchbuf, sizeof(scratchbuf),
                                 lexer->inbuffer, lexer->inbuflen));
    }
}

/* get 0-origin big-endian words relative to start of packet buffer
 * used for Zodiac */
#define getzuword(i) (unsigned)(lexer->inbuffer[2 * (i)] | \
                                (lexer->inbuffer[2 * (i) + 1] << 8))
#define getzword(i) (short)(lexer->inbuffer[2 * (i)] | \
                           (lexer->inbuffer[2 * (i) + 1] << 8))

static bool nextstate(struct gps_lexer_t *lexer, unsigned char c)
{
    static int n = 0;
    enum isgpsstat_t isgpsstat;
#ifdef SUPERSTAR2_ENABLE
    static unsigned char ctmp;
#endif  // SUPERSTAR2_ENABLE

    n++;
    switch (lexer->state) {
    case GROUND_STATE:
        n = 0;
#ifdef STASH_ENABLE
        lexer->stashbuflen = 0;
#endif
        switch (c) {
#ifdef SUPERSTAR2_ENABLE
        case SOH:          // 0x01
            lexer->state = SUPERSTAR2_LEADER;
            break;
#endif // SUPERSTAR2_ENABLE
#ifdef NAVCOM_ENABLE
        case STX:          // 0x02
            lexer->state = NAVCOM_LEADER_1;
            break;
#endif  // NAVCOM_ENABLE
#if defined(TSIP_ENABLE) || defined(EVERMORE_ENABLE) || defined(GARMIN_ENABLE)
        case DLE:          // 0x10
            lexer->state = DLE_LEADER;
            break;
#endif  // TSIP_ENABLE || EVERMORE_ENABLE || GARMIN_ENABLE
        case '!':
            lexer->state = AIS_BANG;
            break;
        case '#':
            lexer->state = COMMENT_BODY;
            break;
        case '$':
            lexer->state = NMEA_DOLLAR;
            break;
#if defined(TNT_ENABLE) || defined(GARMINTXT_ENABLE) || defined(ONCORE_ENABLE)
        case '@':
            if (ISGPS_MESSAGE == rtcm2_decode(lexer, c)) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = AT1_LEADER;
            break;
#endif // TNT_ENABLE, GARMINTXT_ENABLE, ONCORE_ENABLE
#ifdef ITRAX_ENABLE
        case '<':
            lexer->state = ITALK_LEADER_1;
            break;
#endif  // ITRAX_ENABLE
#ifdef TRIPMATE_ENABLE
        case 'A':
            if (ISGPS_MESSAGE == rtcm2_decode(lexer, c)) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = ASTRAL_1;
            break;
#endif  // TRIPMATE_ENABLE
#ifdef EARTHMATE_ENABLE
        case 'E':
            if (ISGPS_MESSAGE == rtcm2_decode(lexer, c)) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = EARTHA_1;
            break;
#endif  // EARTHMATE_ENABLE
#ifdef GEOSTAR_ENABLE
        case 'P':
            lexer->state = GEOSTAR_LEADER_1;
            break;
#endif  // GEOSTAR_ENABLE
#ifdef GREIS_ENABLE
        case 'R':
            lexer->state = GREIS_REPLY_1;
            break;
#endif  // GREIS_ENABLE
        case '{':
            return character_pushback(lexer, JSON_LEADER);
#ifdef GREIS_ENABLE
        // Tilda, Not the only possibility, but a distinctive cycle starter.
        case '~':
            lexer->state = GREIS_ID_1;
            break;
#endif  // GREIS_ENABLE
#if defined(SIRF_ENABLE) || defined(SKYTRAQ_ENABLE)
        case 0xa0:     // latin1 non breaking space
            lexer->state = SIRF_LEADER_1;
            break;
#endif  // SIRF_ENABLE || SKYTRAQ_ENABLE
        case MICRO:      // latin1 micro, 0xb5
            lexer->state = UBX_LEADER_1;
            break;
        case 0xba:      // latin1 MASCULINE ORDINAL INDICATOR
            // got 1st, of 2, bytes of leader
            lexer->state = CASIC_LEADER_1;
            break;
        case 0xD3:      // latin1 capital O acute
            lexer->state = RTCM3_LEADER_1;
            break;
        case 0xf1:      // latin1 small letter N with tilde
            // got 1st, of 2, bytes of leader
            lexer->state = ALLY_LEADER_1;
            break;
#ifdef ZODIAC_ENABLE
        case 0xff:      // lattin1 small y with diaeresis
            lexer->state = ZODIAC_LEADER_1;
            break;
#endif  // ZODIAC_ENABLE
        default:
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
            }
            break;
        }
        break;
    case COMMENT_BODY:
        if ('\n' == c) {
            lexer->state = COMMENT_RECOGNIZED;
        } else if ('\r' == c ||
                   '\t' == c) {
            // allow tabs and CR in comments
        } else if (!isprint(c)) {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NMEA_DOLLAR:
        // We have the leading $
        switch (c) {
        case 'A':
            /* $A (SiRF Ack), $AI (Mobile Class A or B AIS Station?), or
             * $AP (autopliot) */
            lexer->state = NMEA_LEAD_A;
            break;
        case 'B':           // $BD
            lexer->state = BEIDOU_LEAD_1;
            break;
        case 'E':           // $E, ECDIS
            // codacy thinks this is impossible
            lexer->state = ECDIS_LEAD_1;
            break;
        case 'G':           // $GP, $GN, $GY, etc.
            lexer->state = NMEA_PUB_LEAD;
            break;
        case 'H':           // $H, Heading/compass. gyro
            lexer->state = HEADCOMP_LEAD_1;
            break;
        case 'I':           // $I, Seatalk
            lexer->state = SEATALK_LEAD_1;
            break;
        case 'P':           // $P, vendor sentence
            lexer->state = NMEA_VENDOR_LEAD;
            break;
        case 'Q':          // $QZ
            lexer->state = QZSS_LEAD_1;
            break;
        case 'S':          // $S
            // $SD, $SN, $ST
            lexer->state = SOUNDER_LEAD_1;
            break;
        case 'T':           // $T, Turn indicator
            lexer->state = TURN_LEAD_1;
            break;
        case 'W':           // $W, Weather instrument
            lexer->state = WEATHER_LEAD_1;
            break;
        case 'Y':           // $Y
            lexer->state = TRANSDUCER_LEAD_1;
            break;
        default:
            (void)character_pushback(lexer, GROUND_STATE);
            break;
        }
        break;
    case NMEA_PUB_LEAD:
        /*
         * $GP == GPS, $GL = GLONASS only, $GN = mixed GPS and GLONASS,
         * according to NMEA (IEIC 61162-1) DRAFT 02/06/2009.
         * We have a log from China with a BeiDou device using $GB
         * rather than $BD.
         *
         * Unicore uses the non-standard $GY for IMU data.
         */
        if ('A' == c ||      // $GA, Galileo only
            'B' == c ||      // $GB, BeiDou only
            'L' == c ||      // $GL, GLONASS only
            'N' == c ||      // $GN, mixed
            'P' == c ||      // $GP, GPS
            'Y' == c) {      // $GY, Gyro  (IMU)
            lexer->state = NMEA_LEADER_END;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NMEA_VENDOR_LEAD:
        if ('A' == c) {            // $PA
            lexer->state = NMEA_PASHR_A;
        } else if (isalpha(c)) {
            lexer->state = NMEA_LEADER_END;
        } else {
            (void) character_pushback(lexer, GROUND_STATE);
        }
        break;
    /*
     * Without the following six states (NMEA_PASH_*, NMEA_BINARY_*)
     * DLE in a $PASHR can fool the  sniffer into thinking it sees a
     * TSIP packet.  Hilarity ensues.
     */
    case NMEA_PASHR_A:
        if ('S' == c) {        // $PAS
            lexer->state = NMEA_PASHR_S;
        } else if (isalpha(c)) {
            lexer->state = NMEA_LEADER_END;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NMEA_PASHR_S:
        if ('H' == c) {        // $PASH
            lexer->state = NMEA_PASHR_H;
        } else if (isalpha(c)) {
            lexer->state = NMEA_LEADER_END;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NMEA_PASHR_H:
        if ('R' == c) {         // $PASHR
            lexer->state = NMEA_BINARY_BODY;
        } else if (isalpha(c)) {
            lexer->state = NMEA_LEADER_END;
        } else {
            (void) character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NMEA_BINARY_BODY:
        if ('\r' == c) {
            lexer->state = NMEA_BINARY_CR;
        }
        break;
    case NMEA_BINARY_CR:
        if (c == '\n') {
            lexer->state = NMEA_BINARY_NL;
        } else {
            lexer->state = NMEA_BINARY_BODY;
        }
        break;
    case NMEA_BINARY_NL:
        if ('$' == c) {
            (void)character_pushback(lexer, NMEA_RECOGNIZED);
        } else {
            lexer->state = NMEA_BINARY_BODY;
        }
        break;
    // end PASHR, TSIP mitigation

    // start of AIS states
    case AIS_BANG:
        if ('A' == c) {          // !A
            lexer->state = AIS_LEAD_1;
        } else if ('B' == c) {   // !B
            lexer->state = AIS_LEAD_ALT1;
        } else if ('S' == c) {   // !S
            lexer->state = AIS_LEAD_ALT3;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_LEAD_1:
        if (NULL != strchr("BDINRSTX", c)) {
            // !AB, !AD, !AI, !AN, !AR, !AS, !AT, !AX
            lexer->state = AIS_LEAD_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_LEAD_2:
        if (isalpha(c)) {
            lexer->state = AIS_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_LEAD_ALT1:
        if ('S' == c) {
            // !BS
            lexer->state = AIS_LEAD_ALT2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_LEAD_ALT2:
        if (isalpha(c)) {
            lexer->state = AIS_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_LEAD_ALT3:
        if ('A' == c) {
            // !SA
            lexer->state = AIS_LEAD_ALT4;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_LEAD_ALT4:
        if (isalpha(c)) {
            lexer->state = AIS_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_LEADER_END:
        // We stay here grabbing the body of the message, until \r\n
        if ('\r' == c) {
            lexer->state = AIS_CR;
        } else if ('\n' == c) {
            /* not strictly correct (missing \r), but helps with
             * interpreting logfiles. */
            lexer->state = AIS_RECOGNIZED;
        } else if (!isprint(c)) {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_CR:
        if ('\n' == c) {
            lexer->state = AIS_RECOGNIZED;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    // end of AIS states

#if defined(TNT_ENABLE) || defined(GARMINTXT_ENABLE) || defined(ONCORE_ENABLE)
    case AT1_LEADER:
        switch (c) {
#ifdef ONCORE_ENABLE
        case '@':
            lexer->state = ONCORE_AT2;
            break;
#endif  // ONCORE_ENABLE
#ifdef TNT_ENABLE
        case '*':
            /*
             * TNT has similar structure to NMEA packet, '*' before
             * optional checksum ends the packet. Since '*' cannot be
             * received from GARMIN working in TEXT mode, use this
             * difference to tell that this is not GARMIN TEXT packet,
             * could be TNT.
             */
            lexer->state = NMEA_LEADER_END;
            break;
#endif  // TNT_ENABLE
#if defined(GARMINTXT_ENABLE)
        case '\r':
            /* stay in this state, next character should be '\n'
             * in the theory we can stop search here and don't wait for '\n' */
            lexer->state = AT1_LEADER;
            break;
        case '\n':
            // end of packet found
            lexer->state = GTXT_RECOGNIZED;
            break;
#endif  // GARMINTXT_ENABLE
        default:
            if (!isprint(c)) {
                return character_pushback(lexer, GROUND_STATE);
            }
        }
        break;
#endif  // TNT_ENABLE || GARMINTXT_ENABLE || ONCORE_ENABLE
    case NMEA_LEADER_END:
        // We stay here grabbing the body of the message
        if ('\r' == c) {
            lexer->state = NMEA_CR;
        } else if ('\n' == c) {
            /* not strictly correct (missing \r), but helps with
             * interpreting logfiles. */
            lexer->state = NMEA_RECOGNIZED;
        } else if ('$' == c) {
#ifdef STASH_ENABLE
            (void)character_pushback(lexer, STASH_RECOGNIZED);
#else
            (void)character_pushback(lexer, GROUND_STATE);
#endif
        } else if (!isprint(c)) {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NMEA_CR:
        if ('\n' == c) {
            lexer->state = NMEA_RECOGNIZED;
        } else if ('\r' == c) {
            /*
             * There's a GPS called a Jackson Labs Firefly-1a that emits \r\r\n
             * at the end of each sentence.  Don't be confused by this.
             */
            lexer->state = NMEA_CR;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case AIS_RECOGNIZED:
        // AIS and NMEA often mixed treat them similar to start
        FALLTHROUGH
    case NMEA_RECOGNIZED:
        if ('#' == c) {
            lexer->state = COMMENT_BODY;
        } else if ('$' == c) {
            // codacy thinks this state is impossible
            lexer->state = NMEA_DOLLAR;
        } else if ('!' == c) {
            lexer->state = AIS_BANG;
        } else if (MICRO == c) {   // latin1 micro, 0xb5
            // LEA-5H can/will output NMEA/UBX back to back
            // codacy says this state impossible?
            lexer->state = UBX_LEADER_1;
        } else if ('{' == c) {
            // codacy says this state impossible?
            return character_pushback(lexer, JSON_LEADER);
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case SEATALK_LEAD_1:
        if ('I' == c ||
            'N' == c) {         // $II or $IN are accepted
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case WEATHER_LEAD_1:
        if ('I' == c) {         // $WI, Weather instrument leader accepted
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case HEADCOMP_LEAD_1:
        if ('C' == c ||         // $HC, Heading/compass leader accepted
            'E' == c) {         // $HE, Gyro, north seeking
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case TURN_LEAD_1:
        if ('I' == c) {           // $TI, Turn indicator leader accepted
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ECDIS_LEAD_1:
        if ('C' == c) {           // $EC, ECDIS leader accepted
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case SOUNDER_LEAD_1:
        if ('D' == c ||              // $SD, Depth-sounder
            'N' == c ||              // $SN, to $SNRSTAT
            'T' == c) {              // $ST, to $STI
            // leader accepted
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case TRANSDUCER_LEAD_1:
        if ('X' == c) {           // $YX, Transducer leader accepted
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case BEIDOU_LEAD_1:
        if ('D' == c) {           // $BD, Beidou leader accepted
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case QZSS_LEAD_1:
        if ('Z' == c) {           // $QZ, QZSS leader accepted
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#ifdef TRIPMATE_ENABLE
    case ASTRAL_1:
        if ('S' == c) {       // AS
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = ASTRAL_2;
        } else
            (void)character_pushback(lexer, GROUND_STATE);
        break;
    case ASTRAL_2:
        if ('T' == c) {        // AST
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = ASTRAL_3;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ASTRAL_3:
        if ('R' == c) {      // ASTR
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = ASTRAL_5;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ASTRAL_4:
        if ('A' == c) {    // ASTRA
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = ASTRAL_2;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ASTRAL_5:
        if ('L' == c) {       // ASTRAL
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = NMEA_RECOGNIZED;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // TRIPMATE_ENABLE
#ifdef EARTHMATE_ENABLE
    case EARTHA_1:
        if ('A' == c) {     // EA
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = EARTHA_2;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case EARTHA_2:
        if ('R' == c) {     // EAR
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = EARTHA_3;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case EARTHA_3:
        if ('T' == c) {       // EART
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = EARTHA_4;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case EARTHA_4:
        if ('H' == c) {        // EARTH
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = EARTHA_5;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case EARTHA_5:
        if ('A' == c) {     // EARTHA
            if (ISGPS_SYNC == (isgpsstat = rtcm2_decode(lexer, c))) {
                lexer->state = RTCM2_SYNC_STATE;
                break;
            } else if (ISGPS_MESSAGE == isgpsstat) {
                lexer->state = RTCM2_RECOGNIZED;
                break;
            }
            lexer->state = NMEA_RECOGNIZED;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // EARTHMATE_ENABLE
    case NMEA_LEAD_A:
        if ('c' == c) {                // $Ac
            lexer->state = SIRF_ACK_LEAD_2;
        } else if ('I' == c) {         // $AI, Mobile Class A or B AIS Station
            lexer->state = AIS_LEAD_2;
        } else if ('P' == c) {         // $AP, auto pilot
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case SIRF_ACK_LEAD_2:
        if ('k' == c) {                // $Ack
            lexer->state = NMEA_LEADER_END;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#if defined(SIRF_ENABLE) || defined(SKYTRAQ_ENABLE)
    case SIRF_LEADER_1:
# ifdef SKYTRAQ_ENABLE
        // Skytraq leads with 0xA0,0xA1
        if (0xa1 == c) {
            lexer->state = SKY_LEADER_2;
            break;
        }
# endif // SKYTRAQ_ENABLE
# ifdef SIRF_ENABLE
        // SIRF leads with 0xA0,0xA2
        if (0xa2 == c) {
            lexer->state = SIRF_LEADER_2;
            break;
        }
# endif // SIRF_ENABLE
        return character_pushback(lexer, GROUND_STATE);
        break;
#endif  // SIRF_ENABLE || SKYTRAQ_ENABLE
#ifdef SIRF_ENABLE
    case SIRF_LEADER_2:
        // first part of length
        lexer->length = (size_t) (c << 8);
        lexer->state = SIRF_LENGTH_1;
        break;
    case SIRF_LENGTH_1:
        // second part of length
        lexer->length += c + 2;
        if (lexer->length <= MAX_PACKET_LENGTH) {
            lexer->state = SIRF_PAYLOAD;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case SIRF_PAYLOAD:
        if (0 == --lexer->length) {
            lexer->state = SIRF_DELIVERED;
        }
        break;
    case SIRF_DELIVERED:
        if (0xb0 == c) {     // latin1 degree sign
            lexer->state = SIRF_TRAILER_1;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case SIRF_TRAILER_1:
        if (0xb3 == c) {     // latin1 superscript 3
            lexer->state = SIRF_RECOGNIZED;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case SIRF_RECOGNIZED:
        if (0xa0 == c) {     // latin1 no break space
            lexer->state = SIRF_LEADER_1;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // SIRF_ENABLE
#ifdef SKYTRAQ_ENABLE
    case SKY_LEADER_2:
        // MSB of length is first
        lexer->length = (size_t)(c << 8);
        lexer->state = SKY_LENGTH_1;
        break;
    case SKY_LENGTH_1:
        // Skytraq length can be any 16 bit number, except 0
        lexer->length += c;
        if (0 == lexer->length) {
            return character_pushback(lexer, GROUND_STATE);
        }
        if (MAX_PACKET_LENGTH < lexer->length) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = SKY_PAYLOAD;
        break;
    case SKY_PAYLOAD:
        if (0 == --lexer->length) {
            lexer->state = SKY_DELIVERED;
        }
        break;
    case SKY_DELIVERED:
        {
            char scratchbuf[MAX_PACKET_LENGTH * 4 + 1];
            unsigned char csum = 0;

            GPSD_LOG(LOG_RAW, &lexer->errout,
                     "Skytraq = %s\n",
                     gpsd_packetdump(scratchbuf,  sizeof(scratchbuf),
                         lexer->inbuffer,
                         lexer->inbufptr - (unsigned char *)lexer->inbuffer));
            for (n = 4;
                 (unsigned char *)(lexer->inbuffer + n) < lexer->inbufptr - 1;
                 n++) {
                csum ^= lexer->inbuffer[n];
            }
            if (csum != c) {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "Skytraq bad checksum 0x%hhx, expecting 0x%x\n",
                         csum, c);
                lexer->state = GROUND_STATE;
                break;
            }
        }
        lexer->state = SKY_CSUM;
        break;
    case SKY_CSUM:
        if ('\r' != c) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = SKY_TRAILER_1;
        break;
    case SKY_TRAILER_1:
        if ('\n' != c) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = SKY_RECOGNIZED;
        break;
    case SKY_RECOGNIZED:
        if (0xa0 != c) {     // non break space
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = SIRF_LEADER_1;
        break;
#endif  // SKYTRAQ
#ifdef SUPERSTAR2_ENABLE
    case SUPERSTAR2_LEADER:
        ctmp = c;          // seems a dodgy way to keep state...
        lexer->state = SUPERSTAR2_ID1;
        break;
    case SUPERSTAR2_ID1:
        if ((ctmp ^ 0xff) == c) {
            lexer->state = SUPERSTAR2_ID2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case SUPERSTAR2_ID2:
        lexer->length = (size_t) c;  // how many data bytes follow this byte
        if (lexer->length) {
            lexer->state = SUPERSTAR2_PAYLOAD;
        } else {
            lexer->state = SUPERSTAR2_CKSUM1;   // no data, jump to checksum
        }
        break;
    case SUPERSTAR2_PAYLOAD:
        if (0 == --lexer->length) {
            // Done with payload
            lexer->state = SUPERSTAR2_CKSUM1;
        }
        break;
    case SUPERSTAR2_CKSUM1:
        lexer->state = SUPERSTAR2_CKSUM2;
        break;
    case SUPERSTAR2_CKSUM2:
        // checksum not checked here?
        lexer->state = SUPERSTAR2_RECOGNIZED;
        break;
    case SUPERSTAR2_RECOGNIZED:
        if (SOH == c) {
            lexer->state = SUPERSTAR2_LEADER;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // SUPERSTAR2_ENABLE
#ifdef ONCORE_ENABLE
    case ONCORE_AT2:
        if (isupper(c)) {
            lexer->length = (size_t)c;
            lexer->state = ONCORE_ID1;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ONCORE_ID1:
        if (!isalpha(c)) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->length =
            oncore_payload_cksum_length((unsigned char)lexer->length, c);
        if (0 != lexer->length) {
            lexer->state = ONCORE_PAYLOAD;
        }
        // else?
        break;
    case ONCORE_PAYLOAD:
        if (0 == --lexer->length) {
            lexer->state = ONCORE_CHECKSUM;
        }
        break;
    case ONCORE_CHECKSUM:
        if ('\r' != c) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = ONCORE_CR;
        break;
    case ONCORE_CR:
        if ('\n' == c) {
            lexer->state = ONCORE_RECOGNIZED;
        } else {
            lexer->state = ONCORE_PAYLOAD;
        }
        break;
    case ONCORE_RECOGNIZED:
        if ('@' == c) {
            lexer->state = AT1_LEADER;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // ONCORE_ENABLE
#if defined(TSIP_ENABLE) || defined(EVERMORE_ENABLE) || defined(GARMIN_ENABLE)
    case DLE_LEADER:
#ifdef EVERMORE_ENABLE
        if (STX == c) {        // DLE (0x10), STX (0x01)
            lexer->state = EVERMORE_LEADER_2;
            break;
        }
#endif  // EVERMORE_ENABLE
#if defined(TSIP_ENABLE) || defined(GARMIN_ENABLE) || defined(NAVCOM_ENABLE)
        // garmin is special case of TSIP
        // check last because there's no checksum
#if defined(TSIP_ENABLE)
        if (0x13 <= c) {       // DLE (0x10), DC3
            lexer->length = TSIP_MAX_PACKET;
            lexer->state = TSIP_PAYLOAD;
            break;
        }
#endif  // TSIP_ENABLE
        if (DLE == c) {        // DLE (0x10), DLE (0x10)
            lexer->state = GROUND_STATE;
            break;
        }
        // give up
        lexer->state = GROUND_STATE;
        break;
#endif  // TSIP_ENABLE
#ifdef NAVCOM_ENABLE
    case NAVCOM_LEADER_1:
        if (0x99 == c) {        // latin1 TM (0x99)
            lexer->state = NAVCOM_LEADER_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NAVCOM_LEADER_2:
        if ('f' == c) {        // (TM), f
            lexer->state = NAVCOM_LEADER_3;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NAVCOM_LEADER_3:
        lexer->state = NAVCOM_ID;
        break;
    case NAVCOM_ID:
        lexer->length = (size_t)c - 4;
        lexer->state = NAVCOM_LENGTH_1;
        break;
    case NAVCOM_LENGTH_1:
        lexer->length += (c << 8);
        lexer->state = NAVCOM_LENGTH_2;
        break;
    case NAVCOM_LENGTH_2:
        if (0 == --lexer->length) {
            lexer->state = NAVCOM_PAYLOAD;
        }
        break;
    case NAVCOM_PAYLOAD:
        {
            unsigned char csum = lexer->inbuffer[3];
            for (n = 4;
                 (unsigned char *)(lexer->inbuffer + n) < lexer->inbufptr - 1;
                 n++)
                csum ^= lexer->inbuffer[n];
            if (csum != c) {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "Navcom packet type 0x%hhx bad checksum 0x%hhx, "
                         "expecting 0x%x\n",
                         lexer->inbuffer[3], csum, c);
                lexer->state = GROUND_STATE;
                break;
            }
        }
        lexer->state = NAVCOM_CSUM;
        break;
    case NAVCOM_CSUM:
        if (ETX == c) {     // ETX (0x03)
            lexer->state = NAVCOM_RECOGNIZED;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case NAVCOM_RECOGNIZED:
        if (STX == c) {     // STX (0x02)
            lexer->state = NAVCOM_LEADER_1;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // NAVCOM_ENABLE
#endif  // TSIP_ENABLE || EVERMORE_ENABLE || GARMIN_ENABLE
    case RTCM3_LEADER_1:
        // high 6 bits must be zero, low 2 bits are MSB of a 10-bit length
        if (0 == (c & 0xFC)) {
            lexer->length = (size_t)c << 8;
            lexer->state = RTCM3_LEADER_2;
        } else {
            GPSD_LOG(LOG_IO, &lexer->errout,
                     "RTCM3 must be zero bits aren't: %u\n", c & 0xFC);
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case RTCM3_LEADER_2:
        // third byte is the low 8 bits of the RTCM3 packet length
        lexer->length |= c;
        lexer->length += 3;     // to get the three checksum bytes
        lexer->state = RTCM3_PAYLOAD;
        break;
    case RTCM3_PAYLOAD:
        if (0 == --lexer->length) {
            lexer->state = RTCM3_RECOGNIZED;
        }
        break;
#ifdef ZODIAC_ENABLE
    case ZODIAC_EXPECTED:
        FALLTHROUGH
    case ZODIAC_RECOGNIZED:
        if (0xff == c) {         // y with diaeresis
            lexer->state = ZODIAC_LEADER_1;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ZODIAC_LEADER_1:
        if (0x81 == c) {         // latin1 non-printing
            lexer->state = ZODIAC_LEADER_2;
        } else {
            (void)character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ZODIAC_LEADER_2:
        lexer->state = ZODIAC_ID_1;
        break;
    case ZODIAC_ID_1:
        lexer->state = ZODIAC_ID_2;
        break;
    case ZODIAC_ID_2:
        lexer->length = (size_t)c;
        lexer->state = ZODIAC_LENGTH_1;
        break;
    case ZODIAC_LENGTH_1:
        lexer->length += (c << 8);
        lexer->state = ZODIAC_LENGTH_2;
        break;
    case ZODIAC_LENGTH_2:
        lexer->state = ZODIAC_FLAGS_1;
        break;
    case ZODIAC_FLAGS_1:
        lexer->state = ZODIAC_FLAGS_2;
        break;
    case ZODIAC_FLAGS_2:
        lexer->state = ZODIAC_HSUM_1;
        break;
    case ZODIAC_HSUM_1:
        {
            short sum = getzword(0) + getzword(1) + getzword(2) + getzword(3);
            sum *= -1;
            if (sum != getzword(4)) {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "Zodiac Header checksum 0x%x expecting 0x%x\n",
                         sum, getzword(4));
                lexer->state = GROUND_STATE;
                break;
            }
        }
        GPSD_LOG(LOG_RAW1, &lexer->errout,
                 "Zodiac header id=%u len=%u flags=%x\n",
                 getzuword(1), getzuword(2), getzuword(3));
        if (0 == lexer->length) {
            lexer->state = ZODIAC_RECOGNIZED;
            break;
        }
        lexer->length *= 2;     // word count to byte count
        lexer->length += 2;     // checksum
        // 10 bytes is the length of the Zodiac header
        // no idea what Zodiac max length really is
        if ((MAX_PACKET_LENGTH - 10) >= lexer->length) {
            lexer->state = ZODIAC_PAYLOAD;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ZODIAC_PAYLOAD:
        if (0 == --lexer->length) {
            lexer->state = ZODIAC_RECOGNIZED;
        }
        break;
#endif  // ZODIAC_ENABLE
    case UBX_LEADER_1:
        if ('b' == c) {      // micro, b
            lexer->state = UBX_LEADER_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case UBX_LEADER_2:
        lexer->state = UBX_CLASS_ID;
        break;
    case UBX_CLASS_ID:
        lexer->state = UBX_MESSAGE_ID;
        break;
    case UBX_MESSAGE_ID:
        lexer->length = (size_t)c;
        lexer->state = UBX_LENGTH_1;
        break;
    case UBX_LENGTH_1:
        lexer->length += (c << 8);
        if (0 == lexer->length) {
            // no payload
            lexer->state = UBX_CHECKSUM_A;
        } else if (MAX_PACKET_LENGTH >= lexer->length) {
            // normal size payload
            lexer->state = UBX_LENGTH_2;
        } else {
            // bad length
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case UBX_LENGTH_2:
        lexer->state = UBX_PAYLOAD;
        break;
    case UBX_PAYLOAD:
        if (0 == --lexer->length) {
            lexer->state = UBX_CHECKSUM_A;
        }
        // else stay in payload state
        break;
    case UBX_CHECKSUM_A:
        lexer->state = UBX_RECOGNIZED;
        break;
    case UBX_RECOGNIZED:
        if (MICRO == c) {       // latin1 micro (0xb5)
            lexer->state = UBX_LEADER_1;
        } else if ('$' == c) {  // LEA-5H can/will output NMEA/UBX back to back
            lexer->state = NMEA_DOLLAR;
        } else if ('{' == c) {
            // codacy thinks this can never happen
            return character_pushback(lexer, JSON_LEADER);
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;

    // start ALLYSTAR
    case ALLY_LEADER_1:
        if (0xd9 == c) {      // latin capital letter U with grave
            // got 2nd, of 2, bytes of leader
            lexer->state = ALLY_LEADER_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ALLY_LEADER_2:
        // got 1 byte Class ID
        lexer->state = ALLY_CLASS_ID;
        break;
    case ALLY_CLASS_ID:
        // got 1 byte Message ID
        lexer->state = ALLY_MESSAGE_ID;
        break;
    case ALLY_MESSAGE_ID:
        // got 1st, of 2, bytes of length
        lexer->length = (size_t)c;
        lexer->state = ALLY_LENGTH_1;
        break;
    case ALLY_LENGTH_1:
        // got 2nd, of 2, bytes of length
        lexer->length += (c << 8);
        if (MAX_PACKET_LENGTH <= lexer->length) {
            // bad length
            return character_pushback(lexer, GROUND_STATE);
        }  // else

        /* no payload or normal size payload.
         * no idea the real max. */
        lexer->state = ALLY_PAYLOAD;
        break;
    case ALLY_PAYLOAD:
        if (0 == lexer->length) {
            // got 1st, of 2, bytes of checksum
            lexer->state = ALLY_CHECKSUM_A;
        } // else stay in payload state

        if (0 < lexer->length) {
            // Pacify Coverity 498037 about underflow
            lexer->length--;
        }  // else something bad happened...
        break;
    case ALLY_CHECKSUM_A:
        // got 2nd, of 2, bytes of checksum
        lexer->state = ALLY_RECOGNIZED;
        break;
    case ALLY_RECOGNIZED:
        if (0xf1 == c) {   // latin1 small letter N with tilde
            lexer->state = ALLY_LEADER_1;
        } else if ('$' == c) {  // LEA-5H can/will output NMEA/ALL back to back
            lexer->state = NMEA_DOLLAR;
        } else if ('{' == c) {
            // codacy thinks this can never happen
            return character_pushback(lexer, JSON_LEADER);
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    // send ALLYSTAR

    // start CASIC
    case CASIC_LEADER_1:
      if (0xce == c) { // LATIN CAPITAL LETTER I WITH CIRCUMFLEX
            // got 2nd, of 2, bytes of leader
            lexer->state = CASIC_LEADER_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case CASIC_LEADER_2:
        // got 1st, of 2, bytes of length
        lexer->length = (size_t)c;
        lexer->state = CASIC_LENGTH_1;
        break;
    case CASIC_LENGTH_1:
        /* got 2nd, of 2, bytes of length
         * Validate the length field, the driver and code at
         * CASIC_RECOGNIZED require this.
         * Max length seems to be RXM-SVPOS 1536 (packet total 1554)
         * Their doc says payload "<2k bytes"
         */
        lexer->length += (c << 8);
        if (2048 <= lexer->length ||
            0 != (lexer->length % 4)) {
            // bad length
            return character_pushback(lexer, GROUND_STATE);
        }  // else

        /* no payload or normal size payload. */
        lexer->state = CASIC_LENGTH_2;
        break;
    case CASIC_LENGTH_2:
        lexer->state = CASIC_CLASS_ID;
        break;
    case CASIC_CLASS_ID:
        lexer->state = CASIC_MESSAGE_ID;
        break;
    case CASIC_MESSAGE_ID:
        // We're at the first byte of payload, or the first byte of
        // checksum.  Go directly to CASIC_PAYLOAD.
        lexer->state = CASIC_PAYLOAD;
        FALLTHROUGH
    case CASIC_PAYLOAD:
        if (0 == lexer->length) {
            // got 1st, of 4, bytes of checksum
            lexer->state = CASIC_CHECKSUM_A;
        } else if (2048 <= lexer->length) {
            /* RXM-SVPOS seems to have the longest length: 1552, use 2048
             * Their doc says payload "<2k bytes"
             * how could this happen?
             * stay in payload state */
            lexer->length = 0;
        } else {
            // more to go, stay in payload state, Coverity 498037
            lexer->length--;
        }
        break;
    case CASIC_CHECKSUM_A:
        // got 2nd, of 4, bytes of checksum
        lexer->state = CASIC_CHECKSUM_B;
        break;
    case CASIC_CHECKSUM_B:
        // got 3rd, of 4, bytes of checksum
        lexer->state = CASIC_CHECKSUM_C;
        break;
    case CASIC_CHECKSUM_C:
        // got 4th, of 4, bytes of checksum
        lexer->state = CASIC_RECOGNIZED;
        break;
    case CASIC_RECOGNIZED:
        if (0xba == c) {   // latin1 MASCULINE ORDINAL INDICATOR
            lexer->state = CASIC_LEADER_1;
        } else if ('$' == c) {
            // CASIC can/will output NMEA/CASIC back to back
            lexer->state = NMEA_DOLLAR;
        } else if ('{' == c) {
            // JSON
            return character_pushback(lexer, JSON_LEADER);
        }
        // Unknown..
        return character_pushback(lexer, GROUND_STATE);
        break;
    // end CASIC
#ifdef EVERMORE_ENABLE
    case EVERMORE_LEADER_1:
        if (STX == c) {        // DLE, STX
            lexer->state = EVERMORE_LEADER_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case EVERMORE_LEADER_2:
        lexer->length = (size_t)c;
        if (DLE == c) {
            lexer->state = EVERMORE_PAYLOAD_DLE;
        } else {
            lexer->state = EVERMORE_PAYLOAD;
        }
        break;
    case EVERMORE_PAYLOAD:
        if (DLE == c) {
            // Evermore doubles DLE's
            lexer->state = EVERMORE_PAYLOAD_DLE;
        } else if (0 == --lexer->length) {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case EVERMORE_PAYLOAD_DLE:
        switch (c) {
        case DLE:
            lexer->state = EVERMORE_PAYLOAD;
            break;
        case ETX:
            lexer->state = EVERMORE_RECOGNIZED;
            break;
        default:
            lexer->state = GROUND_STATE;
        }
        break;
    case EVERMORE_RECOGNIZED:
        if (DLE == c) {
            lexer->state = EVERMORE_LEADER_1;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // EVERMORE_ENABLE
#ifdef ITRAX_ENABLE
    case ITALK_LEADER_1:
        if ('!' == c) {      // <!
            lexer->state = ITALK_LEADER_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ITALK_LEADER_2:
        lexer->length = (size_t)(lexer->inbuffer[6] & 0xff);
        lexer->state = ITALK_LENGTH;
        break;
    case ITALK_LENGTH:
        lexer->length += 1;     // fix number of words in payload
        lexer->length *= 2;     // convert to number of bytes
        lexer->length += 3;     // add trailer length
        lexer->state = ITALK_PAYLOAD;
        break;
    case ITALK_PAYLOAD:
        // lookahead for "<!" because sometimes packets are short but valid
        if (('>' == c) &&
            ('<' == lexer->inbufptr[0]) &&
            ('!' == lexer->inbufptr[1])) {
            lexer->state = ITALK_RECOGNIZED;
            GPSD_LOG(LOG_PROG, &lexer->errout,
                     "ITALK: trying to process runt packet\n");
        } else if (0 == --lexer->length) {
            lexer->state = ITALK_DELIVERED;
        }
        break;
    case ITALK_DELIVERED:
        if ('>' == c) {
            lexer->state = ITALK_RECOGNIZED;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case ITALK_RECOGNIZED:
        if ('<' == c) {
            lexer->state = ITALK_LEADER_1;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // ITRAX_ENABLE
#ifdef GEOSTAR_ENABLE
    case GEOSTAR_LEADER_1:
        if ('S' == c) {     // PS
            lexer->state = GEOSTAR_LEADER_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case GEOSTAR_LEADER_2:
        if ('G' == c) {     // PSG
            lexer->state = GEOSTAR_LEADER_3;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case GEOSTAR_LEADER_3:
        if ('G' == c) {     // PSGG
            lexer->state = GEOSTAR_LEADER_4;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case GEOSTAR_LEADER_4:
        lexer->state = GEOSTAR_MESSAGE_ID_1;
        break;
    case GEOSTAR_MESSAGE_ID_1:
        lexer->state = GEOSTAR_MESSAGE_ID_2;
        break;
    case GEOSTAR_MESSAGE_ID_2:
        lexer->length = (size_t)c * 4;
        lexer->state = GEOSTAR_LENGTH_1;
        break;
    case GEOSTAR_LENGTH_1:
        lexer->length += (c << 8) * 4;
        if (MAX_PACKET_LENGTH >= lexer->length) {
            lexer->state = GEOSTAR_LENGTH_2;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case GEOSTAR_LENGTH_2:
        lexer->state = GEOSTAR_PAYLOAD;
        break;
    case GEOSTAR_PAYLOAD:
        if (0 == --lexer->length) {
            lexer->state = GEOSTAR_CHECKSUM_A;
        }
        // else stay in payload state
        break;
    case GEOSTAR_CHECKSUM_A:
        lexer->state = GEOSTAR_CHECKSUM_B;
        break;
    case GEOSTAR_CHECKSUM_B:
        lexer->state = GEOSTAR_CHECKSUM_C;
        break;
    case GEOSTAR_CHECKSUM_C:
        lexer->state = GEOSTAR_RECOGNIZED;
        break;
    case GEOSTAR_RECOGNIZED:
        if ('P' == c) {      // P
            lexer->state = GEOSTAR_LEADER_1;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // GEOSTAR_ENABLE
#ifdef GREIS_ENABLE
    case GREIS_EXPECTED:
        FALLTHROUGH
    case GREIS_RECOGNIZED:
        if (!isascii(c)) {
            return character_pushback(lexer, GROUND_STATE);
        }
        if ('#' == c) {
            // Probably a comment used by the testsuite
            lexer->state = COMMENT_BODY;
        } else if ('\n' == c ||
                   '\r' == c) {
            // Arbitrary CR/LF allowed here, so continue to expect GREIS
            lexer->state = GREIS_EXPECTED;
            character_discard(lexer);
        } else {
            lexer->state = GREIS_ID_1;
        }
        break;
    case GREIS_REPLY_1:
        if ('E' != c) {       // RE
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = GREIS_REPLY_2;
        break;
    case GREIS_ID_1:
        if (!isascii(c)) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = GREIS_ID_2;
        break;
    case GREIS_REPLY_2:
        FALLTHROUGH
    case GREIS_ID_2:
        if (!isxdigit(c)) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->length = greis_hex2bin(c) << 8;
        lexer->state = GREIS_LENGTH_1;
        break;
    case GREIS_LENGTH_1:
        if (!isxdigit(c)) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->length += greis_hex2bin(c) << 4;
        lexer->state = GREIS_LENGTH_2;
        break;
    case GREIS_LENGTH_2:
        if (!isxdigit(c)) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->length += greis_hex2bin(c);
        lexer->state = GREIS_PAYLOAD;
        break;
    case GREIS_PAYLOAD:
        if (0 == --lexer->length) {
            lexer->state = GREIS_RECOGNIZED;
        }
        // else stay in payload state
        break;
#endif  // GREIS_ENABLE
#ifdef TSIP_ENABLE
    case TSIP_LEADER:
        // unused case. see TSIP_RECOGNIZED
        if (0x13 <= c) {       // DC3
            lexer->length = TSIP_MAX_PACKET;
            lexer->state = TSIP_PAYLOAD;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case TSIP_PAYLOAD:
        if (DLE == c) {
            lexer->state = TSIP_DLE;
        }
        if (0 == --lexer->length ) {
            // uh, oh, packet too long, probably was never TSIP
            // note lexer->length is unsigned
            lexer->state = GROUND_STATE;
        }
        break;
    case TSIP_DLE:
        switch (c) {
        case ETX:
            lexer->state = TSIP_RECOGNIZED;
            break;
        case DLE:
            lexer->length = TSIP_MAX_PACKET;
            lexer->state = TSIP_PAYLOAD;
            break;
        default:
            lexer->state = GROUND_STATE;
            break;
        }
        break;
    case TSIP_RECOGNIZED:
        if (DLE == c) {
            /*
             * Don't go to TSIP_LEADER state -- TSIP packets aren't
             * checksummed, so false positives are easy.  We might be
             * looking at another DLE-stuffed protocol like EverMore
             * or Garmin streaming binary.
             */
            lexer->state = DLE_LEADER;
        } else {
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
#endif  // TSIP_ENABLE
    case RTCM2_SYNC_STATE:
        FALLTHROUGH
    case RTCM2_SKIP_STATE:
        if (ISGPS_MESSAGE == (isgpsstat = rtcm2_decode(lexer, c))) {
            lexer->state = RTCM2_RECOGNIZED;
        } else if (ISGPS_NO_SYNC == isgpsstat) {
            lexer->state = GROUND_STATE;
        }
        break;

    case RTCM2_RECOGNIZED:
        if ('#' == c) {
            /*
             * There's a remote possibility this could fire when # =
             * 0x23 is legitimate in-stream RTCM2 data.  No help for
             * it, the test framework needs this case so it can inject
             * # EOF and we'll miss a packet.
             */
            return character_pushback(lexer, GROUND_STATE);
        }
        if (ISGPS_SYNC == rtcm2_decode(lexer, c)) {
            lexer->state = RTCM2_SYNC_STATE;
        } else {
            lexer->state = GROUND_STATE;
        }
        break;
    case JSON_LEADER:
        switch (c) {
        case '{':
            FALLTHROUGH
        case '[':
            lexer->json_depth++;
            break;
        case '}':
            FALLTHROUGH
        case ']':
            if (0 == --lexer->json_depth) {
                lexer->state = JSON_RECOGNIZED;
            }
            break;
        case ',':
            break;
        case '"':
            lexer->state = JSON_STRINGLITERAL;
            lexer->json_after = JSON_END_ATTRIBUTE;
            break;
        default:
            if (isspace(c)) {
                break;
            }
            GPSD_LOG(LOG_RAW1, &lexer->errout,
                     "%08ld: missing attribute start after header\n",
                     lexer->char_counter);
            lexer->state = GROUND_STATE;
        }
        break;
    case JSON_STRINGLITERAL:
        if ('\\' == c) {
            lexer->state = JSON_STRING_SOLIDUS;
        } else if ('"' == c) {
            lexer->state = lexer->json_after;
        }
        break;
    case JSON_STRING_SOLIDUS:
        lexer->state = JSON_STRINGLITERAL;
        break;
    case JSON_END_ATTRIBUTE:
        if (isspace(c)) {
            break;
        }
        if (':' == c) {
            lexer->state = JSON_EXPECT_VALUE;
        } else {
            // saw something other than value start after colon
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case JSON_EXPECT_VALUE:
        if (isspace(c)) {
            break;
        }
        switch (c) {
        case '"':
            lexer->state = JSON_STRINGLITERAL;
            lexer->json_after = JSON_END_VALUE;
            break;
        case '{':
            FALLTHROUGH
        case '[':
            return character_pushback(lexer, JSON_LEADER);
            break;
        case '-':
            FALLTHROUGH
        case '0':
            FALLTHROUGH
        case '1':
            FALLTHROUGH
        case '2':
            FALLTHROUGH
        case '3':
            FALLTHROUGH
        case '4':
            FALLTHROUGH
        case '5':
            FALLTHROUGH
        case '6':
            FALLTHROUGH
        case '7':
            FALLTHROUGH
        case '8':
            FALLTHROUGH
        case '9':
            lexer->state = JSON_NUMBER;
            break;
        case 'f':
            FALLTHROUGH
        case 'n':
            FALLTHROUGH
        case 't':
            /*
             * This is a bit more permissive than strictly necessary, as
             * GPSD JSON does not include the null token.  Still, it's
             * futureproofing.
             */
            lexer->state = JSON_SPECIAL;
            break;
        default:
            // couldn't recognize start of value literal
            return character_pushback(lexer, GROUND_STATE);
        }
        break;
    case JSON_NUMBER:
        /*
         * Will recognize some ill-formed numeric literals.
         * Should be OK as we're already three stages deep inside
         * JSON recognition; odds that we'll actually see an
         * ill-formed literal are quite low. and the worst
         * possible result if it happens is our JSON parser will
         * quietly chuck out the object.
         */
        if (NULL == strchr("1234567890.eE+-", c)) {
            return character_pushback(lexer, JSON_END_VALUE);
        }
        break;
    case JSON_SPECIAL:
        if (NULL == strchr("truefalsnil", c)) {
            return character_pushback(lexer, JSON_END_VALUE);
        }
        break;
    case JSON_END_VALUE:
        if (isspace(c)) {
            break;
        }
        if ('}' == c ||
            ']' == c) {
            return character_pushback(lexer, JSON_LEADER);
        }
        if (',' != c) {
            // trailing garbage after JSON value
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = JSON_LEADER;
        break;
#ifdef STASH_ENABLE
    case STASH_RECOGNIZED:
        if ('$' != c) {
            return character_pushback(lexer, GROUND_STATE);
        }
        lexer->state = NMEA_DOLLAR;
        break;
#endif  // STASH_ENABLE
    }

    return true;        // no pushback
}

// packet grab succeeded, move to output buffer
static void packet_accept(struct gps_lexer_t *lexer, int packet_type)
{
    size_t packetlen = lexer->inbufptr - lexer->inbuffer;

    if (sizeof(lexer->outbuffer) > packetlen) {
        char scratchbuf[MAX_PACKET_LENGTH * 4 + 1];

        memcpy(lexer->outbuffer, lexer->inbuffer, packetlen);
        lexer->outbuflen = packetlen;
        lexer->outbuffer[packetlen] = '\0';
        lexer->type = packet_type;
        GPSD_LOG(LOG_RAW1, &lexer->errout,
                 "Packet type %d accepted %zu = %s\n",
                 packet_type, packetlen,
                 gpsd_packetdump(scratchbuf,  sizeof(scratchbuf),
                                 lexer->outbuffer,
                                 lexer->outbuflen));
    } else {
        GPSD_LOG(LOG_ERROR, &lexer->errout,
                 "Rejected too long packet type %d len %zu\n",
                 packet_type, packetlen);
    }
}

// shift the input buffer to discard all data up to current input pointer
static void packet_discard(struct gps_lexer_t *lexer)
{
    size_t discard = lexer->inbufptr - lexer->inbuffer;
    size_t remaining = lexer->inbuflen - discard;
    char scratchbuf[MAX_PACKET_LENGTH * 4 + 1];

    if (sizeof(lexer->inbuffer) < discard) {
        // Huh?
        GPSD_LOG(LOG_WARN, &lexer->errout,
                 "packet_discard() of %zu??\n", discard);
        lexer->inbufptr = lexer->inbuffer;
        lexer->inbuflen = 0;
        return;
    }  // else

    lexer->inbufptr = memmove(lexer->inbuffer, lexer->inbufptr, remaining);
    lexer->inbuflen = remaining;

    GPSD_LOG(LOG_RAW1, &lexer->errout,
             "packet_discard() of %zu, chars remaining is %zu = %s\n",
             discard, remaining,
             gpsd_packetdump(scratchbuf, sizeof(scratchbuf),
                             lexer->inbuffer, lexer->inbuflen));
}

#ifdef STASH_ENABLE
// See test/daemon/isync.log for why the stash is needed.

// stash the input buffer up to current input pointer
static void packet_stash(struct gps_lexer_t *lexer)
{
    size_t stashlen = lexer->inbufptr - lexer->inbuffer;
    char scratchbuf[MAX_PACKET_LENGTH * 4 + 1];

    memcpy(lexer->stashbuffer, lexer->inbuffer, stashlen);
    lexer->stashbuflen = stashlen;

    GPSD_LOG(LOG_RAW1, &lexer->errout,
             "Packet stash of %zu = %s\n",
             stashlen,
             gpsd_packetdump(scratchbuf, sizeof(scratchbuf),
                             lexer->stashbuffer,
                             lexer->stashbuflen));
}

// return stash to start of input buffer
static void packet_unstash(struct gps_lexer_t *lexer)
{
    size_t available = sizeof(lexer->inbuffer) - lexer->inbuflen;
    size_t stashlen = lexer->stashbuflen;

    if (stashlen <= available) {
        char scratchbuf[MAX_PACKET_LENGTH * 4 + 1];

        memmove(lexer->inbuffer + stashlen, lexer->inbuffer, lexer->inbuflen);
        memcpy(lexer->inbuffer, lexer->stashbuffer, stashlen);
        lexer->inbuflen += stashlen;
        lexer->stashbuflen = 0;

        GPSD_LOG(LOG_RAW1, &lexer->errout,
                 "Packet unstash of %zu, reconstructed is %zu = %s\n",
                 stashlen, lexer->inbuflen,
                 gpsd_packetdump(scratchbuf, sizeof(scratchbuf),
                                 lexer->inbuffer, lexer->inbuflen));
    } else {
        GPSD_LOG(LOG_ERROR, &lexer->errout,
                 "Rejected too long unstash of %zu\n", stashlen);
        lexer->stashbuflen = 0;
    }
}
#endif  // STASH_ENABLE

// entry points begin here

// reset lexer structure
void lexer_init(struct gps_lexer_t *lexer, struct gpsd_errout_t *errout)
{
    memset(lexer, 0, sizeof(struct gps_lexer_t));
    /* let memset() do all the zeros
     *
     *  lexer->char_counter = 0;
     *  lexer->retry_counter = 0;
     *  lexer->json_depth = 0;
     *  lexer->start_time.tv_sec = 0;
     *  lexer->start_time.tv_nsec = 0;
     */
    // set start_time to help out autobaud.
    (void)clock_gettime(CLOCK_REALTIME, &lexer->start_time);
    packet_reset(lexer);
    lexer->errout = *errout;    // srtucture copy
}

// grab one packet from inbufptr
// move it to outbuffer[0], set outbuflen, and add a NUL.
// adjust pointers and lengths, then return
void packet_parse(struct gps_lexer_t *lexer)
{

    lexer->outbuflen = 0;
    while (0 < packet_buffered_input(lexer)) {
        unsigned char c = *lexer->inbufptr++;
        unsigned int oldstate = lexer->state;
        unsigned inbuflen;      // bytes in inbuffer for message
        unsigned idx;           // index into inbuffer
        unsigned crc_computed;  // the CRC/checksum we computed
        unsigned crc_expected;  // the CRC/checksum the message claims to have
        enum {PASS, ACCEPT} acc_dis;
        int packet_type;        // gpsd packet type
        unsigned pkt_id;        // native type or ID the message thinks it is
        unsigned data_len;      // What the message says the data length is.
        unsigned char ck_a, ck_b;  // for ubx check bytes
#ifdef STASH_ENABLE
        bool unstash = false;
#endif  //  STASH_ENABLE

        if (!nextstate(lexer, c)) {
            continue;
        }
        GPSD_LOG(LOG_RAW2, &lexer->errout,
                 "%08ld: character '%c' [%02x], %s -> %s\n",
                 lexer->char_counter, (isprint(c) ? c : '.'), c,
                 state_table[oldstate], state_table[lexer->state]);
        lexer->char_counter++;
        inbuflen = lexer->inbufptr - lexer->inbuffer;
        acc_dis = PASS;

        /* check if we have a _RECOGNISED state, if so, perform final
         * checks on the packet, before decoding.
         * Cases alpha sorted to be easy to find. */
        switch (lexer->state) {
        case AIS_RECOGNIZED:
            acc_dis = ACCEPT;
            if (!nmea_checksum(&lexer->errout,
                               (const char *)lexer->inbuffer,
                               (const char *)lexer->inbufptr)) {
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
                break;
            }
            packet_type = AIVDM_PACKET;
            break;

        case ALLY_RECOGNIZED:
            // ALLYSTAR use a TCP like checksum, 8-bit Fletcher Algorithm
            ck_a = (unsigned char)0;
            ck_b = (unsigned char)0;
            // payload length
            data_len = getleu16(lexer->inbuffer, 4);

            GPSD_LOG(LOG_IO, &lexer->errout, "ALLY: buflen %d. paylen %u\n",
                     inbuflen, data_len);
            if (inbuflen < (data_len + 8)) {
                GPSD_LOG(LOG_INFO, &lexer->errout,
                         "ALLY: bad length %d/%u\n",
                         inbuflen, data_len);
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
                acc_dis = ACCEPT;
                break;
            }
            // from class ID (byte 2), msg ID, length,  to end of payload
            for (idx = 0; idx < (data_len + 4); idx++) {
                ck_a += lexer->inbuffer[idx + 2];
                ck_b += ck_a;
            }
            if (ck_a == lexer->inbuffer[data_len + 6] &&
                ck_b == lexer->inbuffer[data_len + 7]) {
                packet_type = ALLYSTAR_PACKET;
            } else {
                char scratchbuf[200];

                GPSD_LOG(LOG_WARN, &lexer->errout,
                         "ALLY: bad checksum 0x%02hhx%02hhx length %d/%u"
                         ", %s\n",
                         ck_a,
                         ck_b,
                         inbuflen, data_len,
                         gps_hexdump(scratchbuf, sizeof(scratchbuf),
                                     lexer->inbuffer, lexer->inbuflen));
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
            }
            acc_dis = ACCEPT;
            break;

        case CASIC_RECOGNIZED:
            /* Payload length.  This field has already been partially
             * validated in nextstate().  */
            data_len = getleu16(lexer->inbuffer, 2);
            if (inbuflen < (data_len + 10)) {
                GPSD_LOG(LOG_INFO, &lexer->errout,
                         "CASIC: bad length %d/%u\n",
                         inbuflen, data_len);
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
                acc_dis = ACCEPT;
                break;
            }
            crc_computed = casic_checksum(lexer->inbuffer + 2, data_len + 4);
            crc_expected = getleu32(lexer->inbuffer, data_len + 6);
            if (crc_computed == crc_expected) {
                packet_type = CASIC_PACKET;
            } else {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "CASIC checksum 0x%04x over length %d,"
                         " expecting 0x%04x (type 0x%02hhx%02hhx)\n",
                         crc_computed,
                         data_len + 4,
                         crc_expected,
                         lexer->inbuffer[4], lexer->inbuffer[5]);
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
            }
            acc_dis = ACCEPT;
            break;

        case COMMENT_RECOGNIZED:
            packet_type = COMMENT_PACKET;
            acc_dis = ACCEPT;
            lexer->state = GROUND_STATE;
            break;

#ifdef EVERMORE_ENABLE
        case EVERMORE_RECOGNIZED:
            // Evermore uses DLE stuffing, what a PITA.
            // Assume failure.
            packet_type = BAD_PACKET;
            acc_dis = ACCEPT;
            lexer->state = GROUND_STATE;

            do {
                // the do{} is only done once, just so we can break

                // check for leader
                idx = 0;
                if (DLE != lexer->inbuffer[idx++] ||
                    STX != lexer->inbuffer[idx++]) {
                    // should not happen
                    break;
                }

                // get one byte length, if length is 0x10, two DLE are sent.
                data_len = lexer->inbuffer[idx++];
                if (DLE == data_len &&
                    DLE != lexer->inbuffer[idx++]) {
                    // should not happen
                    break;
                }
                if (8 > data_len) {
                    /* should not happen, need 1 byte of data for message ID
                     * shortest message is 8 bytes of data_len */
                    break;
                }

                data_len -= 2;
                crc_computed = 0;
                for (; data_len > 0; data_len--) {
                    crc_computed += lexer->inbuffer[idx];
                    if (DLE == lexer->inbuffer[idx++] &&
                        DLE != lexer->inbuffer[idx++]) {
                        // should not happen, DLE not doubled.
                        break;
                    }
                }
                // get one byte checksum
                crc_expected = lexer->inbuffer[idx++];
                if (DLE == crc_expected &&
                    DLE != lexer->inbuffer[idx++]) {
                    // should not happen, DLE not doubled.
                    break;
                }
                // get two byte trailer
                if (DLE != lexer->inbuffer[idx++] ||
                    ETX != lexer->inbuffer[idx]) {
                    // we used to say n++ here, but scan-build complains
                    // bad trailer
                    break;
                }
                crc_computed &= 0xff;
                if (crc_computed != crc_expected) {
                    GPSD_LOG(LOG_PROG, &lexer->errout,
                             "EverMore checksum failed: %02x != %02x\n",
                             crc_computed, crc_expected);
                    break;
                }
                packet_type = EVERMORE_PACKET;
                lexer->state = EVERMORE_RECOGNIZED;
                break;     // redundant
            } while (0);
            break;
#endif  // EVERMORE_ENABLE

#ifdef GEOSTAR_ENABLE
        case GEOSTAR_RECOGNIZED:
            // GeoStar uses a XOR 32bit checksum
            acc_dis = ACCEPT;
            crc_computed = 0;

            // Calculate checksum
            for (idx = 0; idx < inbuflen; idx += 4) {
                crc_computed ^= getleu32(lexer->inbuffer, idx);
            }

            if (0 != crc_computed) {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "GeoStar checksum failed 0x%x over length %d\n",
                         crc_computed, inbuflen);
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
                break;
            }
            packet_type = GEOSTAR_PACKET;
            break;
#endif  // GEOSTAR_ENABLE

#ifdef GREIS_ENABLE
        case GREIS_RECOGNIZED:
            acc_dis = ACCEPT;

            if ('R' == lexer->inbuffer[0] &&
                'E' == lexer->inbuffer[1]) {
                // Replies don't have checksum
                GPSD_LOG(LOG_IO, &lexer->errout,
                         "Accept GREIS reply packet len %d\n", inbuflen);
                packet_type = GREIS_PACKET;
                break;
            }
            if ('E' == lexer->inbuffer[0] &&
                'R' == lexer->inbuffer[1]) {
                // Error messages don't have checksum
                GPSD_LOG(LOG_IO, &lexer->errout,
                         "Accept GREIS error packet len %d\n", inbuflen);
                packet_type = GREIS_PACKET;
                break;
            }
            // 8-bit checksum
            crc_computed = greis_checksum(lexer->inbuffer, inbuflen);

            if (0 != crc_computed) {
                /*
                 * Print hex instead of raw characters, since they might be
                 * unprintable. If \0, it will even mess up the log output.
                 */
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "REJECT GREIS len %d."
                         " Bad checksum %#02x, expecting 0."
                         " Packet type in hex: 0x%02x%02x",
                         inbuflen, crc_computed,
                         lexer->inbuffer[0],
                         lexer->inbuffer[1]);
                packet_type = BAD_PACKET;
                // got this far, fair to expect we will get more GREIS
                lexer->state = GREIS_EXPECTED;
                break;
            }
            GPSD_LOG(LOG_IO, &lexer->errout,
                     "Accept GREIS packet type '%c%c' len %d\n",
                     lexer->inbuffer[0], lexer->inbuffer[1], inbuflen);
            packet_type = GREIS_PACKET;
            break;
#endif  // GREIS_ENABLE

        case GROUND_STATE:
            character_discard(lexer);
            break;

#ifdef GARMINTXT_ENABLE
        case GTXT_RECOGNIZED:
            // As of June 2023, we have no regression of GARMINTXT.
            if (57 <= inbuflen) {
                packet_accept(lexer, GARMINTXT_PACKET);
                packet_discard(lexer);
                lexer->state = GROUND_STATE;
            } else {
                packet_accept(lexer, BAD_PACKET);
                lexer->state = GROUND_STATE;
            }
            break;
#endif

#ifdef ITRAX_ENABLE
#define getib(j) ((uint8_t)lexer->inbuffer[(j)])
#define getiw(i) ((uint16_t)(((uint16_t)getib((i) + 1) << 8) | \
                             (uint16_t)getib((i))))

        case ITALK_RECOGNIZED:
            // number of words
            data_len = lexer->inbuffer[6] & 0xff;

            // expected checksum
            crc_expected = getiw(7 + 2 * data_len);

            crc_computed = 0;
            for (idx = 0; idx < data_len; idx++) {
                uint16_t tmpw = getiw(7 + 2 * idx);
                uint32_t tmpdw  = (crc_computed + 1) * (tmpw + idx);
                crc_computed ^= (tmpdw & 0xffff) ^ ((tmpdw >> 16) & 0xffff);
            }
            if (0 == data_len ||
                crc_computed == crc_expected) {
                packet_type = ITALK_PACKET;
            } else {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "ITALK: checksum failed - "
                         "type 0x%02x expected 0x%04x got 0x%04x\n",
                         lexer->inbuffer[4], crc_expected, crc_computed);
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
            }
            acc_dis = ACCEPT;
#undef getiw
#undef getib
            break;
#endif  // ITRAX_ENABLE

        case JSON_RECOGNIZED:
            if (11 <= inbuflen) {
                // {"class": }
                packet_type = JSON_PACKET;
            } else {
                packet_type = BAD_PACKET;
            }
            lexer->state = GROUND_STATE;
            acc_dis = ACCEPT;
            break;

#ifdef NAVCOM_ENABLE
        case NAVCOM_RECOGNIZED:
            // By the time we got here we know checksum is OK
            packet_type = NAVCOM_PACKET;
            acc_dis = ACCEPT;
            break;
#endif  // NAVCOM_ENABLE

        case NMEA_RECOGNIZED:
            if (nmea_checksum(&lexer->errout,
                               (const char *)lexer->inbuffer,
                               (const char *)lexer->inbufptr)) {
                packet_type = NMEA_PACKET;
#ifdef STASH_ENABLE
                unstash = true;
#endif  // STASH_ENABLE
            } else {
                lexer->state = GROUND_STATE;
                packet_type = BAD_PACKET;
            }
            acc_dis = ACCEPT;
            break;

#ifdef ONCORE_ENABLE
        case ONCORE_RECOGNIZED:
            acc_dis = ACCEPT;
            crc_computed = 0;
            for (idx = 2; idx < inbuflen - 2; idx++) {
                crc_computed ^= lexer->inbuffer[idx];
            }

            if (0 != crc_computed) {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "REJECT OnCore packet @@%c%c len %d\n",
                         lexer->inbuffer[2], lexer->inbuffer[3], inbuflen);
                lexer->state = GROUND_STATE;
                packet_type = BAD_PACKET;
                break;
            }
            GPSD_LOG(LOG_IO, &lexer->errout,
                     "Accept OnCore packet @@%c%c len %d\n",
                     lexer->inbuffer[2], lexer->inbuffer[3], inbuflen);
            packet_type = ONCORE_PACKET;
            break;
#endif  // ONCORE_ENABLE

        case RTCM2_RECOGNIZED:
            /*
             * RTCM packets don't have checksums.  The six bits of parity
             * per word and the preamble better be good enough.
             */
            packet_type = RTCM2_PACKET;
            acc_dis = ACCEPT;
            break;

        case RTCM3_RECOGNIZED:
            // RTCM3 message header not always at inbuffer[0]
            for (idx = 0; idx < inbuflen; idx++) {
                if (0xd3 == lexer->inbuffer[idx]) {
                    break;
                }
            }
            if (2048 < idx) {
                idx = 0;        // can't happen, but pacify fuzzer.
            }
            // we assume xd3 must be in there!
            // yes, the top 6 bits should be zero, total 10 bits of length
            data_len = (lexer->inbuffer[idx + 1] << 8) |
                       lexer->inbuffer[idx + 2];
            data_len &= 0x03ff;   // truncate below 1024, so pacify fuzzer
            if (LOG_IO <= lexer->errout.debug) {
                char outbuf[BUFSIZ];
                // 12 bits of message type
                pkt_id = (lexer->inbuffer[idx + 3] << 4) |
                         (lexer->inbuffer[idx + 4] >> 4);

                // print the inbuffer packet, +3 to peek ahead. (maybe)
                GPSD_LOG(LOG_IO, &lexer->errout,
                         "RTCM3 data_len %u type %u idx %u inbufflen %u "
                         " buf %s\n",
                         data_len, pkt_id, idx, inbuflen,
                         gps_hexdump(outbuf, sizeof(outbuf),
                                     &lexer->inbuffer[idx], data_len + 6 + 3));
            }

            // The CRC includes the preamble, and data.
            if (crc24q_check(&lexer->inbuffer[idx], data_len + 6)) {
                packet_type = RTCM3_PACKET;
            } else {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "RTCM3 data crc failure, "
                         "%0x against %02x %02x %02x\n",
                         crc24q_hash(&lexer->inbuffer[idx], data_len + 3),
                         lexer->inbufptr[idx + data_len + 1],
                         lexer->inbufptr[idx + data_len + 2],
                         lexer->inbufptr[idx + data_len + 3]);
                packet_type = BAD_PACKET;
            }
            acc_dis = ACCEPT;
            lexer->state = GROUND_STATE;
            break;

#ifdef SIRF_ENABLE
        case SIRF_RECOGNIZED:
            {
                unsigned char *trailer;
                trailer = lexer->inbufptr - 4;

                crc_expected = (trailer[0] << 8) | trailer[1];
                crc_computed = 0;

                for (idx = 4; idx < (inbuflen - 4); idx++) {
                    crc_computed += lexer->inbuffer[idx];
                }
                crc_computed &= 0x7fff;
                if (crc_expected == crc_computed) {
                    packet_type = SIRF_PACKET;
                    acc_dis = ACCEPT;
                } else {
                    packet_type = BAD_PACKET;
                    acc_dis = ACCEPT;
                    lexer->state = GROUND_STATE;
                }
            }
            break;
#endif  // SIRF_ENABLE

#ifdef SKYTRAQ_ENABLE
        case SKY_RECOGNIZED:
            packet_type = SKY_PACKET;
            acc_dis = ACCEPT;
            break;
#endif  // SKYTRAQ_ENABLE

#ifdef STASH_ENABLE
        case STASH_RECOGNIZED:
            packet_stash(lexer);
            packet_discard(lexer);
            break;
#endif  // STASH_ENABLE

#ifdef SUPERSTAR2_ENABLE
        case SUPERSTAR2_RECOGNIZED:

            crc_computed = 0;
            lexer->length = 4 + (size_t)lexer->inbuffer[3] + 2;
            if (261 < lexer->length) {
                // can't happen, pacify coverity by checking anyway.
                lexer->length = 261;
            }
            for (idx = 0; idx < lexer->length - 2; idx++) {
                crc_computed += lexer->inbuffer[idx];
            }
            crc_expected = getleu16(lexer->inbuffer, lexer->length - 2);
            GPSD_LOG(LOG_IO, &lexer->errout,
                     "SuperStarII pkt dump: type %u len %zu\n",
                     lexer->inbuffer[1], lexer->length);
            if (crc_expected != crc_computed) {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "REJECT SuperStarII packet type 0x%02x"
                         "%zd bad checksum 0x%04x, expecting 0x%04x\n",
                         lexer->inbuffer[1], lexer->length,
                         crc_computed, crc_expected);
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
            } else {
                packet_type = SUPERSTAR2_PACKET;
            }
            acc_dis = ACCEPT;
            break;
#endif  // SUPERSTAR2_ENABLE

#if defined(TSIP_ENABLE) || defined(GARMIN_ENABLE)
        case TSIP_RECOGNIZED:
            /* Could be Garmin, or TSIP.  Both are DLE stuffed.
             *
             * Garmin: DLE, ID, Length, data..., checksum, DLE, ETX
             * TSIP: DLE, ID, data..., DLE, ETX
             *
             * Note: TSIP has no length, or checksum.  Shame!
             * So we check for Garmin length and checksum, if they
             * fail, we check for TSIP ID's, maybe their matching lengths.
             */

            // Assume bad
            packet_type = BAD_PACKET;
            lexer->state = GROUND_STATE;
            acc_dis = ACCEPT;

            do {
                int dlecnt;

                // don't count stuffed DLEs in the length
                dlecnt = 0;
                for (idx = 0; idx < inbuflen; idx++) {
                    if (DLE == lexer->inbuffer[idx]) {
                        dlecnt++;
                    }
                }
                if (dlecnt > 2) {
                    dlecnt -= 2;
                    dlecnt /= 2;
                    GPSD_LOG(LOG_RAW1, &lexer->errout,
                             "Unstuffed %d DLEs\n", dlecnt);
                    inbuflen -= dlecnt;
                }

                if (5 > inbuflen) {
                    // Message has no data.  Can't be GARMIN or TSIP.
                    break;
                }
#ifdef GARMIN_ENABLE
                do {
#ifdef TSIP_ENABLE
                    // last packet was TSIP, shortcut garmin
                    if (TSIP_PACKET == lexer->type) {
                        break;
                    }
#endif  // TSIP_ENABLE
                    // We know DLE == lexer->inbuffer[0]
                    idx = 1;

                    // Garmin promises ID's 3 (ETX) and 16 (DLE) are never used
                    pkt_id = lexer->inbuffer[idx++];  // packet ID, byte 1.

                    // Get data length from packet.
                    data_len = lexer->inbuffer[idx++];
                    crc_computed = data_len + pkt_id;
                    if (DLE == data_len &&
                        DLE != lexer->inbuffer[idx++]) {
                        // Bad DLE stuffing
                        break;
                    }
                    // Compute checksum.
                    data_len++;
                    for (; data_len > 0; data_len--) {
                        crc_computed += lexer->inbuffer[idx];
                        if (DLE == lexer->inbuffer[idx++] &&
                            DLE != lexer->inbuffer[idx++]) {
                            // Bad DLE stuffing
                            break;
                        }
                    }

                    crc_computed &= 0xff;
                    if (0 != crc_computed) {
                        GPSD_LOG(LOG_PROG, &lexer->errout,
                                 "Garmin checksum failed: %02x!=0\n",
                                 crc_computed);
                        break;
                    }

                    // Check for trailer where expected
                    if (DLE != lexer->inbuffer[idx++] ||
                        ETX != lexer->inbuffer[idx]) {
                        // we used to say idx++ here, but scan-build complains
                        break;
                    }

                    // A good packet!
                    packet_type = GARMIN_PACKET;
                    break;    // redundant...
                } while (0);

                if (GARMIN_PACKET == packet_type) {
                    break;
                }
                GPSD_LOG(LOG_RAW1, &lexer->errout, "Not a Garmin packet\n");
                // Could be TSIP, but line noise can look like TSIP.

#endif  // GARMIN_ENABLE
#ifdef TSIP_ENABLE
                do {
                    /* Since TSIP has no length, or checksum,
                     * check for some common TSIP packet types:
                     * 0x13, TSIP Parsing Error Notification
                     * 0x1c, Hardware/Software Version Information
                     * 0x38, Request SV system data
                     * 0x40, Almanac
                     * 0x41, GPS time, data length 10
                     * 0x42, Single Precision Fix XYZ, data length 16
                     * 0x43, Velocity Fix XYZ, ECEF, data length 20
                     * 0x45, Software Version Information, data length 10
                     * 0x46, Health of Receiver, data length 2
                     * 0x47, Signal Level all Sats Tracked, data length 1+5*numSV
                     * 0x48, GPS System Messages, data length 22
                     * 0x49, Almanac Health Page, data length 32
                     * 0x4a, Single Precision Fix LLA, data length 20
                     * 0x4b, Machine Code Status, data length 3
                     * 0x4c, Operating Parameters Report, data length 17
                     * 0x4d, Oscillator Offset
                     * 0x4e, Response to set GPS time
                     * 0x54, One Satellite Bias, data length 12
                     * 0x55, I/O Options, data length 4
                     * 0x56, Velocity Fix ENU, data length 20
                     * 0x57, Last Computed Fix Report, data length 8
                     * 0x58, Satellite System Data
                     * 0x58-05, UTC
                     * 0x59, Satellite Health
                     * 0x5a, Raw Measurements
                     * 0x5b, Satellite Ephemeris Status, data length 16
                     * 0x5c, Satellite Tracking Status, data length 24
                     * 0x5d, Satellite Tracking Stat, multi-gnss, data length 26
                     * 0x5e, Additional Fix Status Report
                     * 0x5f, Severe Failure Notification
                     * 0x5F-01-0B: Reset Error Codes
                     * 0x5F-02: Ascii text message
                     * 0x6c, Satellite Selection List, data length 18+numSV
                     * 0x6d, All-In-View Satellites, data length 17+numSV
                     * 0x6f, Synced Measurement Packet
                     * 0x72, PV filter parameters
                     * 0x74, Altitude filter parameters
                     * 0x78, Max DGPS correction age
                     * 0x7b, NMEA message schedule
                     * 0x82, Differential Position Fix Mode, data length 1
                     * 0x83, Double Precision Fix XYZ, data length 36
                     * 0x84, Double Precision Fix LLA, data length 36
                     * 0x85, DGPS Correction status
                     * 0x8f, Superpackets
                     * 0x8f-01,
                     * 0x8f-02,
                     * 0x8f-03, port configuration
                     * 0x8f-14, datum
                     * 0x8f-15, datum
                     * 0x8f-17, Single Precision UTM
                     * 0x8f-18, Double Precision UTM
                     * 0x8f-20, LLA & ENU
                     * 0x8f-26, SEEPROM write status
                     * 0x8f-40, TAIP Configuration
                     * 0x8f-42, Stored Production Parameters
                     * 0x90-XX, Version/Config (TSIPv1)
                     * 0xa1-00, Timing Info (TSIPv1)
                     * 0xa1-01, Frequency Info (TSIPv1)
                     * 0xa1-02, Position Info (TSIPv1)
                     * 0xbb, GPS Navigation Configuration
                     * 0xbc, Receiver Port Configuration
                     *
                     * <DLE>[pkt id] [data] <DLE><ETX>
                     *
                     * The best description is in [TSIP], the Trimble Standard
                     * Interface Protocol manual; unless otherwise specified
                     * that is where these type/length notifications are from.
                     *
                     * Note that not all Trimble chips conform perfectly to this
                     * specification, nor does it cover every packet type we
                     * may see on the wire.
                     */
                    pkt_id = lexer->inbuffer[1];    // packet ID
                    // *INDENT-OFF*
                    // FIXME: combine this if, and the next ones?
                    if (!((0x13 == pkt_id) ||
                          (0x1c == pkt_id) ||
                          (0x38 == pkt_id) ||
                          ((0x41 <= pkt_id) && (0x4c >= pkt_id)) ||
                          ((0x54 <= pkt_id) && (0x57 >= pkt_id)) ||
                          ((0x5a <= pkt_id) && (0x5f >= pkt_id)) ||
                          (0x6c == pkt_id) ||
                          (0x6d == pkt_id) ||
                          (0x82 <= pkt_id &&
                           0x84 >= pkt_id) ||
                          (0x8f <= pkt_id &&
                           0x93 >= pkt_id) ||
                          (0xbb == pkt_id) ||
                          (0xbc == pkt_id) ||
                          ((0xa1 <= pkt_id &&
                           0xa3 >= pkt_id)))) {
                        GPSD_LOG(LOG_PROG, &lexer->errout,
                                 "Packet ID 0x%02x out of range for TSIP\n",
                                 pkt_id);
                        break;
                    }
                    // *INDENT-ON*
#define TSIP_ID_AND_LENGTH(id, len)     ((id == pkt_id) && \
                                         (len == (inbuflen - 4)))

                    if (0x13 == pkt_id) {
                        ;  // pass
                        /*
                         * Not in [TSIP],  Accutime Gold only. Variable length.
                         */
                    } else if ((0x1c == pkt_id) &&
                               (11 <= inbuflen)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x41, 10)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x42, 16)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x43, 20)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x45, 10)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x46, 2)) {
                        ;  // pass
                    } else if ((0x47 == pkt_id) &&
                               (0 == (inbuflen % 5))) {
                        /*
                         * 0x47 data length 1+5*numSV, packetlen is 5+5*numSV
                         * FIXME, should be a proper length calculation
                         */
                         ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x48, 22)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x49, 32)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x4a, 20)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x4b, 3)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x4c, 17)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x54, 12)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x55, 4)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x56, 20)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x57, 8)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x5a, 25)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x5b, 16)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x5c, 24)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x5d, 26)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x5e, 2)) {
                        ;  // pass
                     /*
                     * Not in [TSIP]. the TSIP driver doesn't use type 0x5f.
                     * but we test for it so as to avoid setting packet not_tsip
                     */
                    } else if (TSIP_ID_AND_LENGTH(0x5f, 66)) {
                        /*
                         * 0x6c data length 18+numSV, total packetlen is 22+numSV
                         * numSV up to 224
                         */
                        ;  // pass
                    } else if ((0x6c == pkt_id) &&
                               ((22 <= inbuflen) &&
                                (246 >= inbuflen))) {
                        /*
                         * 0x6d data length 17+numSV, total packetlen is 21+numSV
                         * numSV up to 32
                         */
                        ;  // pass
                    } else if ((0x6d == pkt_id) &&
                               ((21 <= inbuflen) &&
                                (53 >= inbuflen))) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x82, 1)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x83, 36)) {
                        ;  // pass
                    } else if (TSIP_ID_AND_LENGTH(0x84, 36)) {
                        // pass
                    } else if (0x8f <= pkt_id &&
                               0x93 >= pkt_id) {
                        // pass, TSIP super packets, variable length
                        // pass, TSIPv1 version/config/info super packet
                    } else if (0xa0 <= pkt_id &&
                               0xa3 >= pkt_id) {
                        // PASS, TSIPv1
                        // FIXME: check for sub packet id 0 to 2
                    /*
                     * This is according to [TSIP].
                     */
                    } else if (TSIP_ID_AND_LENGTH(0xbb, 40)) {
                        ;  // pass
                    /*
                     * The Accutime Gold ships a version of this packet with a
                     * 43-byte payload.  We only use the first 21 bytes, and
                     * parts after byte 27 are padding.
                     */
                    } else if (TSIP_ID_AND_LENGTH(0xbb, 43)) {
                        ;  // pass
                    } else {
                        ;  // pass
                        GPSD_LOG(LOG_PROG, &lexer->errout,
                                 "TSIP REJECT pkt_id = %#02x, inbuflen= %d\n",
                                 pkt_id, inbuflen);
                        break;
                    }
#undef TSIP_ID_AND_LENGTH
                    // Debug
                    GPSD_LOG(LOG_RAW, &lexer->errout,
                             "TSIP pkt_id = %#02x, inbuflen= %d\n",
                             pkt_id, inbuflen);
                    packet_type = TSIP_PACKET;
                    lexer->state = TSIP_RECOGNIZED;
                    break;     // redundant
                } while (0);

                if (BAD_PACKET == packet_type) {
                    GPSD_LOG(LOG_RAW1, &lexer->errout, "Not a TSIP packet\n");
                    acc_dis = ACCEPT;
                    lexer->state = GROUND_STATE;
                }
                break;   // redundant
#endif  // TSIP_ENABLE
            } while (0);
            break;
#endif  // TSIP_ENABLE || GARMIN_ENABLE

        case UBX_RECOGNIZED:
            // UBX use a TCP like checksum
            ck_a = (unsigned char)0;
            ck_b = (unsigned char)0;

            GPSD_LOG(LOG_IO, &lexer->errout, "UBX: len %d\n", inbuflen);
            for (idx = 2; idx < (inbuflen - 2); idx++) {
                ck_a += lexer->inbuffer[idx];
                ck_b += ck_a;
            }
            if (ck_a == lexer->inbuffer[inbuflen - 2] &&
                ck_b == lexer->inbuffer[inbuflen - 1]) {
                packet_type = UBX_PACKET;
            } else {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "UBX checksum 0x%02hhx%02hhx over length %d,"
                         " expecting 0x%02hhx%02hhx (type 0x%02hhx%02hhx)\n",
                         ck_a,
                         ck_b,
                         inbuflen,
                         lexer->inbuffer[inbuflen - 2],
                         lexer->inbuffer[inbuflen - 1],
                         lexer->inbuffer[2], lexer->inbuffer[3]);
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
            }
            acc_dis = ACCEPT;
            break;

#ifdef ZODIAC_ENABLE
        case ZODIAC_RECOGNIZED:
            // be paranoid, look ahead for a good checksum
            data_len = getzuword(2);
            if (253 < data_len) {
                // pacify coverity, 253 seems to be max length
                data_len = 253;
            }
            crc_computed = 0;
            for (idx = 0; idx < data_len; idx++) {
                crc_computed += getzword(5 + idx);
            }
            crc_expected = getzword(5 + data_len);
            crc_computed += crc_expected;
            crc_computed &= 0x0ff;
            if (0 == data_len ||
                0 == crc_computed) {
                packet_type = ZODIAC_PACKET;
            } else {
                GPSD_LOG(LOG_PROG, &lexer->errout,
                         "Zodiac data checksum 0x%x over length %u, "
                         "expecting 0x%x\n",
                         crc_expected, data_len, getzword(5 + data_len));
                packet_type = BAD_PACKET;
                lexer->state = GROUND_STATE;
            }
            acc_dis = ACCEPT;
            break;
#endif  // ZODIAC_ENABLE

        }
        if (ACCEPT == acc_dis) {
            packet_accept(lexer, packet_type);
            packet_discard(lexer);
#ifdef STASH_ENABLE
            if (unstash &&
                0 != lexer->stashbuflen) {
                packet_unstash(lexer);
            }
#endif  // STASH_ENABLE
            break;
        }
    }                           // while
}

/* packet_get() -- deprecated 2023, use packet_get1() instead
 * exposed in Python FFI.
 */
ssize_t packet_get(int fd, struct gps_lexer_t *lexer)
{
    struct gps_device_t session = {{0}};
    ssize_t retval;
    ssize_t inbufptrcnt = lexer->inbufptr - lexer->inbuffer;

    session.gpsdata.gps_fd = fd;
    session.lexer = *lexer;   // structure copy

    // fix inbufptr
    session.lexer.inbufptr = session.lexer.inbuffer + inbufptrcnt;

    retval = packet_get1(&session);

    *lexer = session.lexer;   // structure copy

    // fix inbufptr
    inbufptrcnt = session.lexer.inbufptr - session.lexer.inbuffer;
    lexer->inbufptr = lexer->inbuffer + inbufptrcnt;

    return retval;
}


/* packet_get1_chunked() - grab an http/1.1 chunked packet;
 *
 * Handle http/1.1 chunking as a layer above the packet layer.
 * so far only NTRIP v2 uses it.  Perversely the chunks do
 * not seem to align with received packets.
 *
 * A pointless feature in http/1.1
 *
 * return: greater than zero: length
 *         > 0  == got a packet.
 *         0 == EOF or no full packet
 *        -1 == I/O error
 */
static ssize_t packet_get1_chunked(struct gps_device_t *session)
{
    ssize_t recvd;
    char scratchbuf[MAX_PACKET_LENGTH * 4 + 1];
    // (int) to pacify Codacy
    int fd = (int)session->gpsdata.gps_fd;
    struct gps_lexer_t *lexer = &session->lexer;
    size_t idx = 0;                 // index into inbuffer.
    unsigned char *tmp_bufptr;      // pointer to head in tmp_buffer
    ssize_t taken;
    // make tmp_buffer large, to simplify overflow prevention
    unsigned char tmp_buffer[sizeof(lexer->inbuffer) * 2];

    GPSD_LOG(LOG_PROG, &lexer->errout,
             "PACKET: packet_get1_chunked(fd %d) enter inbuflen %zu "
             "offset %zd remaining %d\n",
             fd, lexer->inbuflen, lexer->inbufptr - lexer->inbuffer,
             lexer->chunk_remaining);

    if (sizeof(lexer->inbuffer) < lexer->inbuflen) {
        GPSD_LOG(LOG_ERROR, &lexer->errout,
                 "PACKET: packet_get1_chunked(fd %d) start inbuflen %zu "
                 "< 0 !!!\n",
                 fd, lexer->inbuflen);
        return -1;  // unrecoverable error
    }

    errno = 0;
    recvd = 0;
    if (2048 > lexer->inbuflen) {
        /* Do not bother to read if we already have enough for longest
         * RTCM3 message.  Longest RTCM3 message is 1023 plus header
         * and chunk overhead.
         * O_NONBLOCK set, so this should not block.
         * Best not to block on an unresponsive NTRIP server.
         * They tend to be bursty.  Like 18kb, then nothing for many
         * seconds. */
        recvd = read(fd, lexer->inbuffer + lexer->inbuflen,
                     sizeof(lexer->inbuffer) - lexer->inbuflen);
    } else {
        GPSD_LOG(LOG_SHOUT, &lexer->errout,
                 "PACKET: packet_get1_chunked(fd %d) got enough inbuflen %zu "
                 "offset %zd\n",
                 fd, lexer->inbuflen, lexer->inbufptr - lexer->inbuffer);
    }

    if (0 == recvd &&
        0 == lexer->inbuflen) {
        /* When reading from a TCP socket, and no bytes ready, read()
         * returns 0 and sets errno to 11 (Resource temporarily unavailable).
         */
        if (EAGAIN != errno) {
            GPSD_LOG(LOG_WARN, &lexer->errout,
                     "PACKET: packet_get1_chunked(fd %d) recvd %zd %s(%d)\n",
                     fd, recvd, strerror(errno), errno);
            return -1;   // unrecoverable error.
        } // else
        GPSD_LOG(LOG_RAW2, &lexer->errout,
                 "PACKET: packet_get1_chunked(fd %d)  no bytes ready\n",
                 fd);
        return 1;  // pretend we got something, to keep connection open
    } // else

    if (0 > recvd) {
        if (EAGAIN == errno ||
            EINTR == errno) {
            GPSD_LOG(LOG_RAW2, &lexer->errout, "PACKET: no bytes ready\n");
            recvd = 0;
            // No new bytes, maybe already have enough bytes for a message
        } else {
            GPSD_LOG(LOG_WARN, &lexer->errout,
                     "PACKET: packet_get1_chunked(fd %d) errno: %s(%d)\n",
                     fd, strerror(errno), errno);
            return -1;   // unrecoverable error.
        }
    }  // else

    // Got some data.
    lexer->inbuflen += recvd;

    GPSD_LOG(LOG_IO, &lexer->errout,
             "PACKET: packet_get1_chunked(fd %d) recvd %zd inbuflen %zd "
             "mid remaining %d >%.100s<\n",
             fd, recvd, lexer->inbuflen, lexer->chunk_remaining,
             gps_hexdump(scratchbuf, sizeof(scratchbuf),
                         lexer->inbufptr, lexer->inbuflen));

    /* Preversly, the buffer may be just 4 bytes.
     * Some servers send just 0x36340d0a in one packet
     * smallest valid chunk: "0\r\n\r\n"
     * Give up for now */
    if (5 >= lexer->inbuflen) {
        GPSD_LOG(LOG_IO, &lexer->errout,
                 "PACKET: packet_get1_chunked(fd %d) < 5 remaining %d\n",
                 fd, lexer->chunk_remaining);
        return 0;       // got nothing.
    }
    /* The length in the header may be up to 1023, plus message
     * overhead.
     * Here we may have N remaining at the head of inbuffer, followed
     * by part, or all, of the next bit to unchunk.*/
    if (0 > lexer->chunk_remaining) {
        GPSD_LOG(LOG_ERROR, &lexer->errout,
                 "PACKET: packet_get1_chunkedfd %d) remaining %d < 0 !!!\n",
                 fd, lexer->chunk_remaining);
        return -1;   // unrecoverable error.
    }
    if (lexer->inbuflen > (long unsigned)lexer->chunk_remaining) {
        // need unchunking
        size_t tmp_buflen = lexer->inbuflen - lexer->chunk_remaining;
        int chunk_num;            // many chunks in one buffer,

        /* Make a copy of the unchunked part of inbuffer.
         * Then unchunk it back into inbuffer. */
        lexer->inbufptr = &lexer->inbuffer[lexer->chunk_remaining];
        memmove(tmp_buffer, lexer->inbufptr, tmp_buflen);

        // get ready to copy chunks back into the inbuffer
        tmp_bufptr = tmp_buffer;
        lexer->inbuflen = lexer->chunk_remaining;

        for (chunk_num = 0; ; chunk_num++) {
            size_t needed = 0;
            int chunk_size = 0;        // given chunk size
            long chunk_size_l = 0;     // given chunk size, as a long
            unsigned char *endptr;     // for strtol()

            // get the hexadecimal chunk size.
            chunk_size_l = strtol((char *)tmp_bufptr, (char **)&endptr, 16);
            if (0 > chunk_size_l ||
                10000 < chunk_size_l) {
                // don't let chunk be negative, or too large.
                GPSD_LOG(LOG_ERROR, &lexer->errout,
                         "PACKET: packet_get1_chunkedfd %d) invalid  "
                         "chunk_size %ld!!!\n",
                         fd, chunk_size_l);
                return -1;   // unrecoverable error.
            }
            chunk_size = chunk_size_l;  // We already tested it fits.

            GPSD_LOG(LOG_IO, &lexer->errout,
                     "PACKET: packet_get1_chunkedfd %d) doing chunk %d  "
                     "size %d inbuflen %zu >%.200s<\n",
                     fd, chunk_num, chunk_size, lexer->inbuflen,
                     gps_hexdump(scratchbuf, sizeof(scratchbuf),
                                 tmp_bufptr, tmp_buflen));

            idx = endptr - tmp_bufptr;

            // check for valid hex ending
            // need better test for overrun
            if (';' != *endptr &&
                '\r' != *endptr) {
                // invalid ending.  valid endings are ':' or '\r\n' (0d0a).
                GPSD_LOG(LOG_WARN, &lexer->errout,
                         "PACKET: NTRIP: packet_get1_chunked(fd %d) "
                         "invalid ending idx %zu (x%x).\n",
                         fd, idx, *endptr);
                // unrecoverable?
                break;   // assume we need more input??
            }
            idx++;   // skip past the ":" or "\r"

            // move past '\n' line ending
            for (; tmp_bufptr[idx] < tmp_buffer[sizeof(tmp_buffer) - 1];
                 idx++) {
                if ('\n' == tmp_bufptr[idx]) {
                    break;
                }
            }
            if ('\n' != tmp_bufptr[idx]) {
                // Invalid ending.  The only valid ending is '\n'.
                GPSD_LOG(LOG_SHOUT, &lexer->errout,
                         "PACKET: NTRIP: packet_get1_chunked(fd %d) "
                         "invalid ending 2, idx %zu x%02x\n",
                         fd, idx, tmp_bufptr[idx]);
                break;   // assume we need more input.
            }
            idx++;    // move past the trailing '\n'

            /* to unchunk we need chunk size + 2 more for \r\n + 2 more
             * for the tailing \r\n */
            needed = chunk_size + 2 + idx;
            GPSD_LOG(LOG_IO, &lexer->errout,
                     "PACKET: NTRIP: packet_get1_chunked(fd  %d) size %d "
                     "idx %zu buflen %zu needed %zu %s\n",
                     fd, chunk_size, idx, tmp_buflen, needed,
                     gps_hexdump(scratchbuf, sizeof(scratchbuf),
                                 tmp_bufptr, 10));
            if (needed > tmp_buflen) {
                /* Don't have enough yet.  Annoyingly, centipede can send
                 * the chunk count line, but not the chunked data yet!!
                 * Like this: "64\r\n".
                 * Save the fragment back into inbuffer
                 */
                memcpy(lexer->inbufptr, tmp_bufptr, tmp_buflen);
                lexer->inbuflen += tmp_buflen;
                GPSD_LOG(LOG_IO, &lexer->errout,
                         "PACKET: NTRIP: packet_get1_chunked(fd %d) "
                         "chunk %d not full needed %zd tmp_buflen %zu\n",
                         fd, chunk_num, needed, tmp_buflen);
                break;
            }
            lexer->chunk_remaining += chunk_size;
            /* enough data in inbuffer, starting at tmp_bufptr[idx]
             * move past that chunk header. */
            tmp_bufptr += idx;
            tmp_buflen -= idx;
            GPSD_LOG(LOG_IO, &lexer->errout,
                     "PACKET: NTRIP: packet_get1_chunked(fd %d) got "
                     "chunk %d >%s<\n",
                     fd, chunk_num,
                     gps_hexdump(scratchbuf, sizeof(scratchbuf),
                                 lexer->inbufptr, chunk_size));

            // save the chunk into inbuffer
            memcpy(lexer->inbufptr, tmp_bufptr, chunk_size);
            lexer->inbuflen += chunk_size;
            lexer->inbufptr += chunk_size;

            // skip past the chunk trailer (\r\n)
            tmp_bufptr += chunk_size + 2;
            tmp_buflen -= chunk_size + 2;
            if (0 == tmp_buflen) {
                break;
            }
            // smallest valid chunk: "0\r\n\r\n"
            if (5 >= tmp_buflen) {
                // left overs!, put back into inbuffer later.
                GPSD_LOG(LOG_IO, &lexer->errout,
                         "PACKET: NTRIP: packet_get1_chunked(fd %d) "
                         "left over %zu inbuflen %zu\n",
                         fd, tmp_buflen, lexer->inbuflen);
                break;
            }
        }
    } // else, inbuf already unchunked

    if (0 == lexer->inbuflen) {
        GPSD_LOG(LOG_IO, &lexer->errout,
                 "PACKET: NTRIP: packet_get1_chunked(fd %d) got nothing,\n",
                  fd);
        return 1;   // not right, close enough
    }
    // data starts at lexer->inbuffer[0]
    if (0 > lexer->chunk_remaining) {
        GPSD_LOG(LOG_ERROR, &lexer->errout,
                 "PACKET: packet_get1_chunkedfd %d) remaining %d < 0 !!!\n",
                 fd, lexer->chunk_remaining);
        return -1;  // unrecoverable error
    }
    GPSD_LOG(LOG_IO, &lexer->errout,
             "PACKET: packet_get1_chunked(fd %d) inbuflen %zu remaining %d "
             "unchunked %.200s\n",
              fd, lexer->inbuflen, lexer->chunk_remaining,
              gps_hexdump(scratchbuf, sizeof(scratchbuf),
                          lexer->inbuffer, lexer->inbuflen));

    // now get one message
    // RTCM3 message header not always at inbufptr[0]
    for (idx = 0; idx < lexer->inbuflen; idx++) {
        if (0xd3 == lexer->inbuffer[idx] &&
            0 == (0xfc & lexer->inbuffer[idx +1])) {
            // Looking for 0xd3, followed by 6 zeros.
            break;
        }
    }
    if (0xd3 != lexer->inbuffer[idx]) {
        // start of RTCM3 not found.
        GPSD_LOG(LOG_IO, &lexer->errout,
                 "PACKET: packet_get1_chunked(fd %d) RTCM3 start not "
                 "found, idx %zu, %.200s\n",
                  fd, idx,
                  gps_hexdump(scratchbuf, sizeof(scratchbuf),
                              lexer->inbuffer, lexer->inbuflen));
        return 1;   // not right, close enough
    }
    /* Prolly should check for start of next RTCM3, to ensure we
     * have all of the first one.
     * packet_parse() works much better is the start (0xd3) of
     * messages is at inbuffer[0]
     */
    memmove(lexer->inbuffer, &lexer->inbuffer[idx], lexer->inbuflen - idx);
    lexer->inbufptr = lexer->inbuffer;
    lexer->inbuflen -= idx;
    lexer->chunk_remaining -= idx;
    if (sizeof(lexer->inbuffer) < lexer->inbuflen) {
        GPSD_LOG(LOG_ERROR, &lexer->errout,
                 "PACKET: packet_get1_chunked(fd %d) mid inbuflen %zu !!!  "
                 "idx %zu \n",
                 fd, lexer->inbuflen, idx);
        return -1;  // unrecoverable error
    }
    if (0 > lexer->chunk_remaining) {
        GPSD_LOG(LOG_ERROR, &lexer->errout,
                 "PACKET: packet_get1_chunked(fd %d) idx %zu remaining %d "
                 "< 0 !!!\n",
                 fd, idx, lexer->chunk_remaining);
        return -1;  // unrecoverable error
    }
    lexer->outbuflen = 0;

    GPSD_LOG(LOG_IO, &lexer->errout,
             "PACKET: NTRIP: packet_get1_chunked(fd %d) to packet_parse() "
              "inbuflen %zu idx %zu outbuflen %zu remaining %d pbu %zu "
              ">%.200s<\n",
              fd, lexer->inbuflen, idx, lexer->outbuflen,
              lexer->chunk_remaining, packet_buffered_input(lexer),
              gps_hexdump(scratchbuf, sizeof(scratchbuf),
                          lexer->inbufptr, lexer->inbuflen));
    taken = lexer->inbuflen;
    packet_parse(lexer);
    taken -= lexer->inbuflen;
    lexer->chunk_remaining -= taken;

    GPSD_LOG(LOG_IO, &lexer->errout,
             "PACKET: packet_get1_chunked(fd %d) fm packet_parse() taken %zd "
             "inbuflen %zd outbuflen %zd remaining %d >%.200s<\n",
             fd, taken, lexer->inbuflen, lexer->outbuflen,
             lexer->chunk_remaining,
             gps_hexdump(scratchbuf, sizeof(scratchbuf),
                         lexer->outbuffer, lexer->outbuflen));

    if (sizeof(lexer->inbuffer) < lexer->inbuflen) {
        GPSD_LOG(LOG_ERROR, &lexer->errout,
                 "PACKET: packet_get1_chunked(fd %d) start inbuflen %zu "
                 "< 0 !!!\n",
                 fd, lexer->inbuflen);
        return -1;  // unrecoverable error
    }
    return (ssize_t)lexer->outbuflen;
}

/* grab a packet;
 * return: greater than zero: length
 *         > 0  == got a packet.
 *         0 == EOF or no full packet
 *        -1 == I/O error
 */
ssize_t packet_get1(struct gps_device_t *session)
{
    /* recvd, and watned  big enough to hold size_t and ssize_t
     * to Pacify Coveruty 498038, and avoid signed/unsigned math */
    long long recvd;
    long long wanted;
    char scratchbuf[MAX_PACKET_LENGTH * 4 + 1];
    int fd = session->gpsdata.gps_fd;
    struct gps_lexer_t *lexer = &session->lexer;

    if (true == lexer->chunked) {
        // De-chunking too complicated to do unlike below.
        return packet_get1_chunked(session);
    }

    errno = 0;
    wanted = sizeof(lexer->inbuffer) - lexer->inbuflen;
    if (0 >= wanted) {
        // should never happen, pacify Coverity 498039
        return -1;
    }
    /* O_NONBLOCK set, so this should not block.
     * Best not to block on an unresponsive GNSS receiver */
    recvd = read(fd, lexer->inbuffer + lexer->inbuflen, wanted);

    if (-1 >= recvd) {
        if (EAGAIN == errno ||
            EINTR == errno) {
            GPSD_LOG(LOG_RAW2, &lexer->errout, "PACKET: no bytes ready\n");
            recvd = 0;
            // fall through, input buffer may be nonempty
        } else {
            GPSD_LOG(LOG_WARN, &lexer->errout,
                     "PACKET: packet_get1(fd %d) errno: %s(%d)\n",
                     fd, strerror(errno), errno);
            return -1;
        }
    } else {
        GPSD_LOG(LOG_RAW1, &lexer->errout,
                 "PACKET: Read %lld chars to buffer[%zd] (total %lld): %s\n",
                 recvd, lexer->inbuflen, lexer->inbuflen + recvd,
                 gpsd_packetdump(scratchbuf, sizeof(scratchbuf),
                                 lexer->inbufptr, (size_t) recvd));
        lexer->inbuflen += recvd;
    }
    GPSD_LOG(LOG_SPIN, &lexer->errout,
             "PACKET: packet_get1(fd %d) recvd %lld %s(%d)\n",
             fd, recvd, strerror(errno), errno);
    /*
     * Bail out, indicating no more input, only if we just received
     * nothing from the device and there is nothing waiting in the
     * packet input buffer.
     */
    if (0 >= recvd &&
        0 >= packet_buffered_input(lexer)) {
        GPSD_LOG(LOG_IO, &lexer->errout,
                 "PACKET: packet_get1(fd %d) recvd %lld\n",
                 fd, recvd);
        return recvd;
    }

    // Otherwise, consume from the packet input buffer
    // coverity[tainted_data]
    packet_parse(lexer);

    // if input buffer is full, discard
    if (sizeof(lexer->inbuffer) <= (lexer->inbuflen)) {
        // coverity[tainted_data]
        packet_discard(lexer);
        lexer->state = GROUND_STATE;
        GPSD_LOG(LOG_WARN, &lexer->errout,
                 "PACKET: packet_get1() inbuffer overflow.\n");
    }

    /*
     * If we gathered a packet, return its length; it will have been
     * consumed out of the input buffer and moved to the output
     * buffer.  We don't care whether the read() returned 0 or -1 and
     * gathered packet data was all buffered or whether it was partly
     * just physically read.
     *
     * Note: this choice greatly simplifies life for callers of
     * packet_get1(), but means that they cannot tell when a nonzero
     * return means there was a successful physical read.  They will
     * thus credit a data source that drops out with being alive
     * slightly longer than it actually was.  This is unlikely to
     * matter as long as any policy timeouts are large compared to
     * the time required to consume the greatest possible amount
     * of buffered input, but if you hack this code you need to
     * be aware of the issue. It might also slightly affect
     * performance profiling.
     */
    if (0 < lexer->outbuflen) {
        GPSD_LOG(LOG_IO, &lexer->errout,
                 "PACKET: packet_get1(fd %d) outbuflen %zd\n",
                 fd, lexer->outbuflen);
        return (ssize_t)lexer->outbuflen;
    }
    /*
     * Otherwise recvd is the size of whatever packet fragment we got.
     * It can still be 0 or -1 at this point even if buffer data
     * was consumed.
     */
    GPSD_LOG(LOG_IO, &lexer->errout,
             "PACKET: packet_get1(fd %d) recvd %lld\n",
             fd, recvd);
    return recvd;
}

// return the packet machine to the ground state
void packet_reset(struct gps_lexer_t *lexer)
{
    lexer->type = BAD_PACKET;
    lexer->state = GROUND_STATE;
    lexer->inbuflen = 0;
    lexer->inbufptr = lexer->inbuffer;
    isgps_init(lexer);
#ifdef STASH_ENABLE
    lexer->stashbuflen = 0;
#endif  // STASH_ENABLE
}


#ifdef __UNUSED__
// push back the last packet grabbed
void packet_pushback(struct gps_lexer_t *lexer)
{
    if (MAX_PACKET_LENGTH > (lexer->outbuflen + lexer->inbuflen)) {
        memmove(lexer->inbuffer + lexer->outbuflen,
                lexer->inbuffer, lexer->inbuflen);
        memmove(lexer->inbuffer, lexer->outbuffer, lexer->outbuflen);
        lexer->inbuflen += lexer->outbuflen;
        lexer->inbufptr += lexer->outbuflen;
        lexer->outbuflen = 0;
    }
}
#endif  // __UNUSED

// vim: set expandtab shiftwidth=4
