'''
class ubx
'''

# This file is Copyright by the GPSD project
# SPDX-License-Identifier: BSD-2-clause
#
# This code runs compatibly under Python 2 and 3.x for x >= 2.
# Preserve this property!

from __future__ import absolute_import, print_function, division

import binascii      # for binascii.hexlify()
import string        # for string.printable
import struct        # for pack()
import sys
import time

try:
    import gps
except ImportError:
    # PEP8 says local imports last
    sys.stderr.write("gps/ubx: failed to import gps, check PYTHONPATH\n")
    sys.exit(2)


def erd_s(erd):
    """convert 6 bits of subframe 4, page 13 (NMCT) ERD to string"""
    if erd & 0x20:
        if erd == 0x20:
            return "n/a"
        # else
        erd = 1 - erd
    # else
    return "%3s" % erd


def uint2int(u, bit):
    """Convert unsigned "bit" wide integer to signed integer"""
    if u & (1 << (bit - 1)):
        u -= (1 << bit)
    return u


# unpack_XXX() - unpack types from "word"
#
# word is a very long unsigned integer made from the message.
#
# I'd like to use pypy module bitstring or bitarray, but
# people complain when non stock python modules are used here.

def unpack_s10g(word, pos):
    """Grab GLONASS signed 10 bits from word

GLONASS uses the sign bit instead of the two's complement,
for instance, -42 is encoded as 1000101010.

See 'ICD_GLONASS_5.1_(2008)_en.pdf' Section 4.4
"""

    val = (word >> pos) & 0x03ff
    if 0x0200 & val:
        # sign bit on, mask out sign bit and negate
        val = -(val & ~0x0200)
    return val


def unpack_s11(word, pos):
    """Grab a signed 11 bits from offset pos of word"""

    ubytes = bytearray(2)
    ubytes[0] = (word >> pos) & 0xff
    ubytes[1] = (word >> (pos + 8)) & 0x07
    if 0x04 & ubytes[1]:
        # extend the sign
        ubytes[1] |= 0xf8
    u = struct.unpack_from('<h', ubytes, 0)
    return u[0]


def unpack_s11s(word):
    """Grab the weird split signed 11 bits from word"""

    newword = (word >> 22) & 0xff
    newword <<= 3
    newword |= (word >> 8) & 0x07
    return unpack_s11(newword, 0)


def unpack_s11g(word, pos):
    """Grab GLONASS signed 11 bits from word

GLONASS uses the sign bit instead of the two's complement,
for instance, -42 is encoded as 1000101010.

See 'ICD_GLONASS_5.1_(2008)_en.pdf' Section 4.4
"""

    val = (word >> pos) & 0x07ff
    if 0x0400 & val:
        # sign bit on, makks out sign bit and negate
        val = -(val & ~0x0400)
    return val


def unpack_s14(word, pos):
    """Grab a signed 14 bits from offset pos of word"""

    ubytes = bytearray(2)
    ubytes[0] = (word >> pos) & 0xff
    ubytes[1] = (word >> (pos + 8)) & 0x3f
    if 0x20 & ubytes[1]:
        # extend the sign
        ubytes[1] |= 0xc0
    u = struct.unpack_from('<h', ubytes, 0)
    return u[0]


def unpack_s16(word, pos):
    """Grab a signed two bytes from offset pos of word"""

    ubytes = bytearray(2)
    ubytes[0] = (word >> pos) & 0xff
    ubytes[1] = (word >> (pos + 8)) & 0xff
    u = struct.unpack_from('<h', ubytes, 0)
    return u[0]


def unpack_u16(word, pos):
    """Grab a unsigned two bytes from offset pos of word"""

    ubytes = bytearray(2)
    ubytes[0] = (word >> pos) & 0xff
    ubytes[1] = (word >> (pos + 8)) & 0xff
    u = struct.unpack_from('<H', ubytes, 0)
    return u[0]


def unpack_u17(word, pos):
    """Grab an unsigned 17 bits from offset pos of word"""

    return unpack_u24(word, pos) & 0x01ffff


def unpack_s22(word, pos):
    """Grab a signed 22 bits from offset pos of word"""

    ubytes = bytearray(4)
    ubytes[0] = (word >> pos) & 0xff
    ubytes[1] = (word >> (pos + 8)) & 0xff
    ubytes[2] = (word >> (pos + 16)) & 0x3f
    ubytes[3] = 0
    if 0x20 & ubytes[2]:
        # extend the sign
        ubytes[2] |= 0xc0
        ubytes[3] = 0xff

    u = struct.unpack_from('<l', ubytes, 0)
    return u[0]


def unpack_s24(word, pos):
    """Grab a signed 24 bits from offset pos of word"""

    ubytes = bytearray(4)
    ubytes[0] = (word >> pos) & 0xff
    ubytes[1] = (word >> (pos + 8)) & 0xff
    ubytes[2] = (word >> (pos + 16)) & 0xff
    ubytes[3] = 0
    if 0x80 & ubytes[2]:
        # extend the sign
        ubytes[3] = 0xff

    u = struct.unpack_from('<l', ubytes, 0)
    return u[0]


def unpack_u24(word, pos):
    """Grab an unsigned 24 bits from offset pos of word"""

    ubytes = bytearray(4)
    ubytes[0] = (word >> pos) & 0xff
    ubytes[1] = (word >> (pos + 8)) & 0xff
    ubytes[2] = (word >> (pos + 16)) & 0xff
    ubytes[3] = 0

    u = struct.unpack_from('<L', ubytes, 0)
    return u[0]


def unpack_s32s(word, word1):
    """Grab an signed 32 bits from weird split word, word1"""

    ubytes = bytearray(4)
    ubytes[0] = (word >> 6) & 0xff
    ubytes[1] = (word >> 14) & 0xff
    ubytes[2] = (word >> 22) & 0xff
    ubytes[3] = (word1 >> 6) & 0xff

    u = struct.unpack_from('<l', ubytes, 0)
    return u[0]


def unpack_u32s(word, word1):
    """Grab an unsigned 32 bits from weird split word, word1"""

    ubytes = bytearray(4)
    ubytes[0] = (word >> 6) & 0xff
    ubytes[1] = (word >> 14) & 0xff
    ubytes[2] = (word >> 22) & 0xff
    ubytes[3] = (word1 >> 6) & 0xff

    u = struct.unpack_from('<L', ubytes, 0)
    return u[0]


def unpack_s8(word, pos):
    """Grab a signed byte from offset pos of word"""

    ubytes = bytearray(1)
    ubytes[0] = (word >> pos) & 0xff
    u = struct.unpack_from('<b', ubytes, 0)
    return u[0]


def unpack_u8(word, pos):
    """Grab an unsigned byte from offset pos of word"""

    ubytes = bytearray(1)
    ubytes[0] = (word >> pos) & 0xff
    u = struct.unpack_from('<B', ubytes, 0)
    return u[0]


def pack_s16(number):
    """Convert a number to 2 bytes (little endian signed)"""
    return struct.pack('<h', number)


def pack_u16(number):
    """Convert a number to 2 bytes (little endian unsigned)"""
    return struct.pack('<h', number)


def pack_s32(number):
    """Convert a number to 4 bytes (little endian signed)"""
    return struct.pack('<i', number)


def pack_u32(number):
    """Convert a number to 4 bytes (little endian unsigned)"""
    return struct.pack('<I', number)


def flag_s(flag, descs):
    """Decode single bit flags using descs, return a string.
Ignores unknown bits."""

    s = ''
    for key, value in sorted(descs.items()):
        if key == (key & flag):
            s += value
            s += ' '

    return s.strip()


def flagm_s(flag, descs):
    """Decode complex (multi-bit) flag using descs, return a string.
Ignores unknown bits."""

    s = ''
    for val, mask, string in descs:
        if val == (flag & mask):
            s += string + ' '

    return s.strip()


def index_s(index, descs, nf="Unk"):
    """Decode flag using descs, return a string.  Otherwise Unk"""

    if index in descs:
        s = descs[index]
    else:
        s = nf

    return s


def timestamp(timestamp_opts):
    """Print current time as a timestamp.

UNIX epoch for timestamp_opts == 1, UTC when timestamp_opts == 2
"""

    now = time.time()
    if 1 == timestamp_opts:
        print("%.4f" % now)
    elif 2 == timestamp_opts:
        print("%.4f %s" % (now, time.asctime(time.gmtime(now))))


class ubx(object):
    """Class to hold u-blox stuff."""

    # expected statement identifier.
    expect_statement_identifier = False
    io_handle = None
    # when a statement identifier is received, it is stored here
    last_statement_identifier = None
    read_only = False
    protver = 10.0
    timestamp = None
    verbosity = gps.VERB_NONE

    def __init__(self):
        pass

    # allowable speeds
    speeds = (921600, 460800, 230400, 153600, 115200, 57600, 38400, 19200,
              9600, 4800)

    # UBX Satellite Numbering
    gnss_id = {0: 'GPS',
               1: 'SBAS',
               2: 'Galileo',
               3: 'BeiDou',
               4: 'IMES',
               5: 'QZSS',
               6: 'GLONASS',
               7: "IRNSS",     # formerly NavIC
               }

    # UBX Satellite/Dignal Numbering
    gnss_sig_id = {0x0000: 'GPS L1 C/A',
                   0x0003: 'GPS L2 CL',
                   0x0004: 'GPS L2 CM',
                   0x0006: 'GPS L5 I',
                   0x0007: 'GPS L5 Q',
                   0x0100: 'SBAS L1 C/A',
                   0x0200: 'GAL E1 C',
                   0x0201: 'GAL E1 B',
                   0x0203: 'GAL E5 aI',
                   0x0204: 'GAL E5 aQ',
                   0x0205: 'GAL E5 bI',
                   0x0206: 'GAL E5 bQ',
                   0x0300: 'BDS B1I D1',
                   0x0301: 'BDS B1I D2',
                   0x0302: 'BDS B2I D1',
                   0x0303: 'BDS B2I D2',
                   0x0305: 'BDS B1 Cp',
                   0x0306: 'BDS B1 Cd',
                   0x0307: 'BDS B2 ap',
                   0x0308: 'BDS B2 ad',
                   0x0508: 'QZSS L1 C/A',
                   0x0501: 'QZSS L1 S',
                   0x0504: 'QZSS L2 CM',
                   0x0505: 'QZSS L2 CL',
                   0x0508: 'QZSS L5 I',
                   0x0509: 'QZSS L5 Q',
                   0x0600: 'GLO L1 OF',
                   0x0602: 'GLO L2 OF',
                   0x0700: 'NavIc L5 A',
                   }

    # B-CNAV2 HS (Health Status)
    hs_vals = {0: 'Healthy',
               1: 'Unealthy',
               }

    # Names for portID values in UBX-CFG-PRT, UBX-MON-IO, etc.
    port_ids = {0: 'DDC',  # The license free name for i2c used in the spec
                1: 'UART1',
                2: 'UART2',
                3: 'USB',
                4: 'SPI',
                }
    port_id_map = dict([(v, k) for (k, v) in port_ids.items()])
    port_id_map['UART'] = port_id_map['UART1']  # Accept synonym
    port_ids[5] = 'Reserved'  # Don't include this in port_id_map

    # Names for portID values in UBX-CFG-COMMS
    # the doc does not match what is seen
    port_ids1 = {0: 'DDC',           # I2C on F9T
                 0x001: 'UART1',     # as documented on ZED-M9
                 0x003: 'USB',       # as documented on ZED-M9
                 0x004: 'SPI',       # as documented on ZED-M9
                 0x100: 'UART1',     # seen on ZED-M9, documented
                 0x101: 'UNKa',      # seen on ZED-M9T, undocumented
                 0x102: 'UART2',     # as documented on ZED-M9
                 0x200: 'UNKb',      # seen on ZED-M9T, undocumented
                 0x201: 'UART2',
                 0x300: 'USB',       # seen on ZED-M9, undocumented
                 0x400: 'SPI',
                 }
    # u-blox 9 cfg items as a 5-tuple
    # 1 - Name
    # 2 - key id
    # 3 - value type
    # 4 - scale
    # 5 - Unit
    # 6 - Description
    #
    # Sort this list by group (bits 16 to 23) and ID in group (bits 0 to 11)
    cfgs = (
        # CFG--
        ("CFG", 0x1FFFFFFF, "", 0, "",
         "get all CFG"),

        # CFG-ANA-
        ("CFG-ANA", 0x1023FFFF, "", 0, "",
         "get all CFG-ANA"),
        ("CFG-ANA-USE_ANA", 0x10230001, "L", 1, "",
         "Use AssistNow Autonomous"),
        ("CFG-ANA-ORBMAXERR", 0x30230002, "U2", 1, "m",
         "Maximum acceptable (modeled) orbit error"),

        # CFG-BATCH-
        ("CFG-BATCH", 0x1026FFFF, "", 0, "",
         "get all CFG-BATCH"),
        ("CFG-BATCH-ENABLE", 0x10260013, "L", 1, "",
         "Enable the feature. Needs further configuration."),
        ("CFG-BATCH-PIOENABLE", 0x10260014, "L", 1, "",
         "Enable PIO notification when buffer fill level exceeds WARNTHRS."),
        ("CFG-BATCH-MAXENTRIES", 0x30260015, "U2", 1, "",
         "Size of buffer in number of epochs to store."),
        ("CFG-BATCH-WARNTHRS", 0x30260016, "U2", 1, "",
         "Fill level to trigger PIO notification, number of epochs stored."),
        ("CFG-BATCH-PIOACTIVELOW", 0x10260018, "L", 1, "",
         "Drive CFG-BATCH-PIOID low when buffer fill level reaches WARNTHRS"),
        ("CFG-BATCH-PIOID", 0x20260019, "U1", 1, "",
         "PIO that is used for buffer fill level notification"),
        ("CFG-BATCH-EXTRAPVT", 0x1026001a, "L", 1, "",
         "Include additional PVT information in UBX-LOG-BATCH messages"),
        ("CFG-BATCH-EXTRAODO", 0x1026001b, "L", 1, "",
         "Include additional ODO information in UBX-LOG-BATCH messages"),

        # CFG-BDS--
        ("CFG-BDS", 0x1034FFFF, "", 0, "",
         "get all CFG-BDS"),
        ("CFG-BDS-D1D2_NAVDATA", 0x20340009, "E1", 1, "",
         "BDS D1 D2 navdata"),
        ("CFG-BDS-USE_GEO_PRN", 0x10340014, "L", 1, "",
         "Use BDS geostationary sats (PRN 1-5, 59-63)"),

        # CFG-CFB--
        ("CFG-CFB", 0x100cFFFF, "", 0, "",
         "get all CFG-CFB"),
        ("CFG-CFB-ENABLE", 0x100c0001, "L", 1, "",
         "Enable Confidence Bound estimation"),
        ("CFG-CFB-WINDOW_SIZE", 0x400c0007, "I4", 1, "",
         "Average Window Size for CFB"),

        # CFG-GAL-
        ("CFG-GAL", 0x1035FFFF, "", 0, "",
         "get all CFG-GAL"),
        ("CFG-GAL-USE_OSNMA", 0x10350005, "L", 1, "",
         "Use GAL Nav Data"),
        ("CFG-GAL-OSNMA_MINTAGLENGTH", 0x20350007, "U1", 1, "",
         "Use GAL Nav Data min length"),
        ("CFG-GAL-OSNMA_TIMESYNC", 0x10350009, "L", 1, "",
         "Use GAL OSNMA time sync"),
        ("CFG-GAL-OSNMA_INAVPRIM", 0x10350010, "L", 1, "",
         "Use GAL I/NAV as primary source"),

        # CFG-GEOFENCE-
        ("CFG-GEOFENCE", 0x2024FFFF, "", 0, "",
         "get all CFG-GEOFENCE"),
        ("CFG-GEOFENCE-CONFLVL", 0x20240011, "E1", 1, "",
         "Required confidence level for state evaluation"),
        ("CFG-GEOFENCE-USE_PIO", 0x10240012, "L", 1, "",
         "Use PIO combined fence state output"),
        ("CFG-GEOFENCE-PINPOL", 0x20240013, "E1", 1, "",
         "PIO pin polarity"),
        ("CFG-GEOFENCE-PIN", 0x20240014, "U1", 1, "",
         "PIO pin number"),
        ("CFG-GEOFENCE-USE_FENCE1", 0x10240020, "L", 1, "",
         "Use first geofence"),
        ("CFG-GEOFENCE-FENCE1_LAT", 0x40240021, "I4", 1e-7, "deg",
         "Latitude of the first geofence circle center"),
        ("CFG-GEOFENCE-FENCE1_LON", 0x40240022, "I4", 1e-7, "deg",
         "Longitude of the first geofence circle center"),
        ("CFG-GEOFENCE-FENCE1_RAD", 0x40240023, "U4", 0.01, "m",
         "Radius of the first geofence circle"),
        ("CFG-GEOFENCE-USE_FENCE2", 0x10240030, "L", 1, "",
         "Use second geofence"),
        ("CFG-GEOFENCE-FENCE2_LAT", 0x40240031, "I4", 1e-7, "deg",
         "Latitude of the second geofence circle center"),
        ("CFG-GEOFENCE-FENCE2_LON", 0x40240032, "I4", 1e-7, "deg",
         "Longitude of the second geofence circle center"),
        ("CFG-GEOFENCE-FENCE2_RAD", 0x40240033, "U4", 0.01, "m",
         "Radius of the second geofence circle"),
        ("CFG-GEOFENCE-USE_FENCE3", 0x10240040, "L", 1, "",
         "Use third geofence"),
        ("CFG-GEOFENCE-FENCE3_LAT", 0x40240041, "I4", 1e-7, "deg",
         "Latitude of the third geofence circle center"),
        ("CFG-GEOFENCE-FENCE3_LON", 0x40240042, "I4", 1e-7, "deg",
         "Longitude of the third geofence circle center"),
        ("CFG-GEOFENCE-FENCE3_RAD", 0x40240043, "U4", 0.01, "m",
         "Radius of the third geofence circle"),
        ("CFG-GEOFENCE-USE_FENCE4", 0x10240050, "L", 1, "",
         "Use fourth geofence"),
        ("CFG-GEOFENCE-FENCE4_LAT", 0x40240051, "I4", 1e-7, "deg",
         "Latitude of the fourth geofence circle center"),
        ("CFG-GEOFENCE-FENCE4_LON", 0x40240052, "I4", 1e-7, "deg",
         "Longitude of the fourth geofence circle center"),
        ("CFG-GEOFENCE-FENCE4_RAD", 0x40240053, "U4", 0.01, "m",
         "Radius of the fourth geofence circle"),

        # CFG-HW
        ("CFG-HW", 0x10a3FFFF, "", 0, "",
         "get all CFG-HW"),
        ("CFG-HW-ANT_CFG_VOLTCTRL", 0x10a3002e, "L", 1, "",
         "Active antenna voltage control flag"),
        ("CFG-HW-ANT_CFG_SHORTDET", 0x10a3002f, "L", 1, "",
         "Short antenna detection flag"),
        ("CFG-HW-ANT_CFG_SHORTDET_POL", 0x10a30030, "L", 1, "",
         "Short antenna detection polarity"),
        ("CFG-HW-ANT_CFG_OPENDET", 0x10a30031, "L", 1, "",
         "Open antenna detection flag"),
        ("CFG-HW-ANT_CFG_OPENDET_POL", 0x10a30032, "L", 1, "",
         "Open antenna detection polarity"),
        ("CFG-HW-ANT_CFG_PWRDOWN", 0x10a30033, "L", 1, "",
         "Power down antenna flag"),
        ("CFG-HW-ANT_CFG_PWRDOWN_POL", 0x10a30034, "L", 1, "",
         "Power down antenna logic polarity"),
        ("CFG-HW-ANT_CFG_RECOVER", 0x10a30035, "L", 1, "",
         "Automatic recovery from short state flag"),
        ("CFG-HW-ANT_SUP_SWITCH_PIN", 0x20a30036, "U1", 1, "",
         "ANT1 PIO number"),
        ("CFG-HW-ANT_SUP_SHORT_PIN", 0x20a30037, "U1", 1, "",
         "ANT0 PIO number"),
        ("CFG-HW-ANT_SUP_OPEN_PIN", 0x20a30038, "U1", 1, "",
         "ANT2 PIO number"),
        ("CFG-HW-ANT_ON_SHORT_US", 0x30a3003c, "U2", 1, "",
         "ANT2 on->short timeout"),
        # M10S, protVer 34.00
        ("CFG-HW-ANT_SUP_ENGINE", 0x20a30054, "E1", 1, "",
         "Antenna supervisor engine selection"),
        # M10S, protVer 34.00
        ("CFG-HW-ANT_SUP_SHORT_THR", 0x20a30055, "U1", 1, "mV",
         "Antenna supervisor MADC engine short detection threshold"),
        # M10S, protVer 34.00
        ("CFG-HW-ANT_SUP_OPEN_THR", 0x20a30056, "U1", 1, "mV",
         "Antenna supervisor MADC engine open detection threshold"),
        # M10S, protVer 34.00
        ("CFG-HW-RF_LNA_MODE", 0x20a30057, "E1", 1, "",
         "Mode for internal LNA"),
        ("CFG-HW-SENS_WOM_MODE", 0x20a30063, "E1", 1, "",
         "Select Wake-On-Motion mode"),
        ("CFG-HW-SENS_WOM_THLD", 0x20a30064, "U1", 1, "",
         "Select Wake-On-Motion threshold"),
        # X20, protVer 50.10
        ("CFG-HW-RF1_LNA_MODE_LOWGAIN", 0x10a3006a, "L", 1, "",
         "Low Gain Mode for internal LNA RF1"),
        # X20, protVer 50.10
        ("CFG-HW-RF2_LNA_MODE_LOWGAIN", 0x10a3006b, "L", 1, "",
         "Low Gain Mode for internal LNA RF2"),
        # X20, protVer 50.10
        ("CFG-HW-RF3_LNA_MODE_LOWGAIN", 0x10a3006c, "L", 1, "",
         "Low Gain Mode for internal LNA RF3"),

        # CFG-I2C
        ("CFG-I2C", 0x2051ffff, "", 0, "",
         "get all CFG-I2C"),
        ("CFG-I2C-ADDRESS", 0x20510001, "U1", 1, "",
         "I2C slave address of the receiver"),
        ("CFG-I2C-EXTENDEDTIMEOUT", 0x10510002, "L", 1, "",
         "Flag to disable timeouting the interface after 1.5 s"),
        ("CFG-I2C-ENABLED", 0x10510003, "L", 1, "",
         "Flag to indicate if the I2C interface should be enabled"),

        # M10S, protVer 34.00
        ("CFG-I2C-REMAP", 0x10510004, "L", 1, "",
         "I2C remapping"),

        # CFG-I2CINPROT
        ("CFG-I2CINPROT", 0x1071ffff, "", 0, "",
         "get all CFG-I2CINPROT"),
        ("CFG-I2CINPROT-UBX", 0x10710001, "L", 1, "",
         "Flag to indicate if UBX should be an input on I2C"),
        ("CFG-I2CINPROT-NMEA", 0x10710002, "L", 1, "",
         "Flag to indicate if NMEA should be an input on I2C"),
        ("CFG-I2CINPROT-RTCM2X", 0x10710003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input on I2C"),
        ("CFG-I2CINPROT-RTCM3X", 0x10710004, "L", 1, "",
         "Flag to indicate if RTCM3X should be input on I2C"),
        # X20, protVer 50.10
        ("CFG-I2CINPROT-SPARTN", 0x10710005, "L", 1, "",
         "Flag to indicate if SPARTN should be input on I2C"),

        # CFG-I2COUTPROT
        ("CFG-I2COUTPROT", 0x1072ffff, "", 0, "",
         "get all CFG-I2COUTPROT"),
        ("CFG-I2COUTPROT-UBX", 0x10720001, "L", 1, "",
         "Flag to indicate if UBX should be an output on I2C"),
        ("CFG-I2COUTPROT-NMEA", 0x10720002, "L", 1, "",
         "Flag to indicate if NMEA should be an output on I2C"),
        ("CFG-I2COUTPROT-RTCM3X", 0x10720004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output on I2C"),

        # CFG-INFMSG-
        ("CFG-INFMSG", 0x2092ffff, "", 0, "",
         "get all CFG-INFMSG"),
        ("CFG-INFMSG-UBX_I2C", 0x20920001, "X1", 1, "",
         "Information message enable flags for UBX protocol on I2C"),
        ("CFG-INFMSG-UBX_UART1", 0x20920002, "X1", 1, "",
         "Information message enable flags for UBX protocol on UART1"),
        ("CFG-INFMSG-UBX_UART2", 0x20920003, "X1", 1, "",
         "Information message enable flags for UBX protocol on UART2"),
        ("CFG-INFMSG-UBX_USB", 0x20920004, "X1", 1, "",
         "Information message enable flags for UBX protocol on USB"),
        ("CFG-INFMSG-UBX_SPI", 0x20920005, "X1", 1, "",
         "Information message enable flags for UBX protocol on SPI"),
        ("CFG-INFMSG-NMEA_I2C", 0x20920006, "X1", 1, "",
         "Information message enable flags for NMEA protocol on I2C"),
        ("CFG-INFMSG-NMEA_UART1", 0x20920007, "X1", 1, "",
         "Information message enable flags for NMEA protocol on UART1"),
        ("CFG-INFMSG-NMEA_UART2", 0x20920008, "X1", 1, "",
         "Information message enable flags for NMEA protocol on UART2"),
        ("CFG-INFMSG-NMEA_USB", 0x20920009, "X1", 1, "",
         "Information message enable flags for NMEA protocol on USB"),
        ("CFG-INFMSG-NMEA_SPI", 0x2092000a, "X1", 1, "",
         "Information message enable flags for NMEA protocol on SPI"),

        # CFG-ITFM-
        ("CFG-ITFM", 0x2041ffff, "", 0, "",
         "get all CFG-ITFM"),
        ("CFG-ITFM-BBTHRESHOLD", 0x20410001, "U1", 1, "",
         "Broadband jamming detection threshold"),
        ("CFG-ITFM-CWTHRESHOLD", 0x20410002, "U1", 1, "",
         "CW jamming detection threshold"),
        ("CFG-ITFM-ENABLE", 0x1041000d, "L", 1, "",
         "Enable interference detection"),
        ("CFG-ITFM-ANTSETTING", 0x20410010, "E1", 1, "",
         "Antenna setting"),
        ("CFG-ITFM-ENABLE_AUX", 0x10410013, "L", 1, "",
         "Set to true to scan auxiliary bands"),

        # CFG-LOGFILTER-
        ("CFG-LOGFILTER", 0x10deffff, "", 0, "",
         "get all CFG-LOGFILTER"),
        # removed in protVer 35
        ("CFG-LOGFILTER-RECORD_ENA", 0x10de0002, "L", 1, "",
         "Recording enabled"),
        # removed in protVer 35
        ("CFG-LOGFILTER-ONCE_PER_WAKE_UP_ENA", 0x10de0003, "L", 1, "",
         "Once per wakeup"),
        # removed in protVer 35
        ("CFG-LOGFILTER-APPLY_ALL_FILTERS", 0x10de0004, "L", 1, "",
         "Apply all filter settings"),
        # removed in protVer 35
        ("CFG-LOGFILTER-MIN_INTERVAL", 0x30de0005, "U2", 1, "s",
         "Minimum time interval between logged positions"),
        # removed in protVer 35
        ("CFG-LOGFILTER-TIME_THRS", 0x30de0006, "U2", 1, "s",
         "Time threshold"),
        # removed in protVer 35
        ("CFG-LOGFILTER-SPEED_THRS", 0x30de0007, "U2", 1, "m/s",
         "Speed threshold"),
        # removed in protVer 35
        ("CFG-LOGFILTER-POSITION_THRS", 0x40de0008, "U4", 1, "m",
         "Position threshold"),

        # CFG-MOT-
        ("CFG-MOT", 0x3025ffff, "", 0, "",
         "get all CFG-MOT"),
        ("CFG-MOT-GNSSSPEED_THRS", 0x20250038, "U1", 0.01, "m/s",
         "GNSS speed threshold below which platform is considered "
         "as stationary"),
        ("CFG-MOT-GNSSDIST_THRS", 0x3025003b, "U2", 1, "",
         "Distance above which GNSS-based stationary motion is exit"),
        ("CFG-MOT-IMU_FILT_WINDOW", 0x30250016, "U2", 1, "ms",
         "Averaging window for IMU measurement"),

        # CFG-MSGOUT-
        ("CFG-MSGOUT", 0x2091ffff, "", 0, "",
         "get all CFG-MSGOUT"),

        # CFG-MSGOUT-NMEA
        ("CFG-MSGOUT-NMEA_ID_DTM_I2C", 0x209100a6, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_DTM_UART1", 0x209100a7, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_DTM_UART2", 0x209100a8, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_DTM_USB", 0x209100a9, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_DTM_SPI", 0x209100aa, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_RMC_I2C", 0x209100ab, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_RMC_UART1", 0x209100ac, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_RMC_UART2", 0x209100ad, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_RMC_USB", 0x209100ae, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_RMC_SPI", 0x209100af, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port SPI"),

        ("CFG-MSGOUT-NMEA_ID_VTG_I2C", 0x209100b0, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_VTG_UART1", 0x209100b1, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_VTG_UART2", 0x209100b2, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_VTG_USB", 0x209100b3, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_VTG_SPI", 0x209100b4, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GNS_I2C", 0x209100b5, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GNS_UART1", 0x209100b6, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GNS_UART2", 0x209100b7, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GNS_USB", 0x209100b8, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GNS_SPI", 0x209100b9, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GGA_I2C", 0x209100ba, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GGA_UART1", 0x209100bb, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GGA_UART2", 0x209100bc, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GGA_USB", 0x209100bd, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GGA_SPI", 0x209100be, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GSA_I2C", 0x209100bf, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GSA_UART1", 0x209100c0, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GSA_UART2", 0x209100c1, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GSA_USB", 0x209100c2, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GSA_SPI", 0x209100c3, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GSV_I2C", 0x209100c4, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GSV_UART1", 0x209100c5, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GSV_UART2", 0x209100c6, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port UART"),
        ("CFG-MSGOUT-NMEA_ID_GSV_USB", 0x209100c7, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GSV_SPI", 0x209100c8, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GLL_I2C", 0x209100c9, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GLL_UART1", 0x209100ca, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GLL_UART2", 0x209100cb, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GLL_USB", 0x209100cc, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GLL_SPI", 0x209100cd, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GRS_I2C", 0x209100ce, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GRS_UART1", 0x209100cf, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port UART1"),

        ("CFG-MSGOUT-NMEA_ID_GRS_UART2", 0x209100d0, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GRS_USB", 0x209100d1, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GRS_SPI", 0x209100d2, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GST_I2C", 0x209100d3, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GST_UART1", 0x209100d4, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GST_UART2", 0x209100d5, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GST_USB", 0x209100d6, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GST_SPI", 0x209100d7, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_I2C", 0x209100d8, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_UART1", 0x209100d9, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_UART2", 0x209100da, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_USB", 0x209100db, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_SPI", 0x209100dc, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GBS_I2C", 0x209100dd, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GBS_UART1", 0x209100de, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GBS_UART2", 0x209100df, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port UART2"),

        ("CFG-MSGOUT-NMEA_ID_GBS_USB", 0x209100e0, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GBS_SPI", 0x209100e1, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_THS_I2C", 0x209100e2, "U1", 1, "",
         "Output rate of the NMEA-GX-THS message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_THS_UART1", 0x209100e3, "U1", 1, "",
         "Output rate of the NMEA-GX-THS message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_THS_UART2", 0x209100e4, "U1", 1, "",
         "Output rate of the NMEA-GX-THS message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_THS_USB", 0x209100e5, "U1", 1, "",
         "Output rate of the NMEA-GX-THS message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_THS_SPI", 0x209100e6, "U1", 1, "",
         "Output rate of the NMEA-GX-THS message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_VLW_I2C", 0x209100e7, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_VLW_UART1", 0x209100e8, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_VLW_UART2", 0x209100e9, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_VLW_USB", 0x209100ea, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_VLW_SPI", 0x209100eb, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port SPI"),

        ("CFG-MSGOUT-NMEA_ID_RLM_I2C", 0x20910400, "U1", 1, "",
         "Output rate of the NMEA-GX-RLM message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_RLM_UART1", 0x20910401, "U1", 1, "",
         "Output rate of the NMEA-GX-RLM message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_RLM_UART2", 0x20910402, "U1", 1, "",
         "Output rate of the NMEA-GX-RLM message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_RLM_USB", 0x20910403, "U1", 1, "",
         "Output rate of the NMEA-GX-RLM message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_RLM_SPI", 0x20910404, "U1", 1, "",
         "Output rate of the NMEA-GX-RLM message on port SPI"),

        ("CFG-MSGOUT-NMEA_NAV2_ID_RMC_I2C", 0x20910652, "U1", 1, "",
         "Output rate of the NMEA-NAV2-RMC message on port I2C"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_RMC_UART1", 0x20910653, "U1", 1, "",
         "Output rate of the NMEA-NAV2X-RMC message on port UART1"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_RMC_UART2", 0x20910654, "U1", 1, "",
         "Output rate of the NMEA-NAV2-RMC message on port UART2"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_RMC_USB", 0x20910655, "U1", 1, "",
         "Output rate of the NMEA-NAV2-RMC message on port USB"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_RMC_SPI", 0x20910656, "U1", 1, "",
         "Output rate of the NMEA-NAV2-RMC message on port SPI"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_VTG_I2C", 0x20910657, "U1", 1, "",
         "Output rate of the NMEA-NAV2-VTG message on port I2C"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_VTG_UART1", 0x20910658, "U1", 1, "",
         "Output rate of the NMEA-NAV2-VTG message on port UART1"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_VTG_UART2", 0x20910659, "U1", 1, "",
         "Output rate of the NMEA-NAV2-VTG message on port UART2"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_VTG_USB", 0x2091065a, "U1", 1, "",
         "Output rate of the NMEA-NAV2-VTG message on port USB"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_VTG_SPI", 0x2091065b, "U1", 1, "",
         "Output rate of the NMEA-NAV2-VTG message on port SPI"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GNS_I2C", 0x2091065c, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GNS message on port I2C"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GNS_UART1", 0x2091065d, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GNS message on port UART1"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GNS_UART2", 0x2091065e, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GNS message on port UART2"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GNS_USB", 0x2091065f, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GNS message on port USB"),

        ("CFG-MSGOUT-NMEA_NAV2_ID_GNS_SPI", 0x20910660, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GNS message on port SPI"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GGA_I2C", 0x20910661, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GGA message on port I2C"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GGA_UART1", 0x20910662, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GGA message on port UART1"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GGA_UART2", 0x20910663, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GGA message on port UART2"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GGA_USB", 0x20910664, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GGA message on port USB"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GGA_SPI", 0x20910665, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GGA message on port SPI"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GSA_I2C", 0x20910666, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GSA message on port I2C"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GSA_UART1", 0x20910667, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GSA message on port UART1"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GSA_UART2", 0x20910668, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GSA message on port UART2"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GSA_USB", 0x20910669, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GSA message on port USB"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GSA_SPI", 0x2091066a, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GSA message on port SPI"),

        ("CFG-MSGOUT-NMEA_NAV2_ID_GLL_I2C", 0x20910670, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GLL message on port I2C"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GLL_UART1", 0x20910671, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GLL message on port UART1"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GLL_UART2", 0x20910672, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GLL message on port UART2"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GLL_USB", 0x20910673, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GLL message on port USB"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_GLL_SPI", 0x20910674, "U1", 1, "",
         "Output rate of the NMEA-NAV2-GX-GLL message on port SPI"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_ZDA_I2C", 0x2091067f, "U1", 1, "",
         "Output rate of the NMEA-NAV2-ZDA message on port I2C"),

        ("CFG-MSGOUT-NMEA_NAV2_ID_ZDA_UART1", 0x20910680, "U1", 1, "",
         "Output rate of the NMEA-NAV2-ZDA message on port UART1"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_ZDA_UART2", 0x20910681, "U1", 1, "",
         "Output rate of the NMEA-NAV2-ZDA message on port UART2"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_ZDA_USB", 0x20910682, "U1", 1, "",
         "Output rate of the NMEA-NAV2-ZDA message on port USB"),
        ("CFG-MSGOUT-NMEA_NAV2_ID_ZDA_SPI", 0x20910683, "U1", 1, "",
         "Output rate of the NMEA-NAV2-ZDA message on port SPI"),

        ("CFG-MSGOUT-NMEA_ID_UTC_I2C", 0x209106cf, "U1", 1, "",
         "Output rate of the NMEA-GX-UTC message on port I2C"),

        ("CFG-MSGOUT-NMEA_ID_UTC_UART1", 0x209106d0, "U1", 1, "",
         "Output rate of the NMEA-GX-UTC message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_UTC_UART2", 0x209106d1, "U1", 1, "",
         "Output rate of the NMEA-GX-UTC message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_UTC_USB", 0x209106d2, "U1", 1, "",
         "Output rate of the NMEA-GX-UTC message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_UTC_SPI", 0x209106d3, "U1", 1, "",
         "Output rate of the NMEA-GX-UTC message on port SPI"),

        # CFG-MSGOUT-PUBX
        ("CFG-MSGOUT-PUBX_ID_POLYP_I2C", 0x209100ec, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port I2C"),
        ("CFG-MSGOUT-PUBX_ID_POLYP_UART1", 0x209100ed, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port UART1"),
        ("CFG-MSGOUT-PUBX_ID_POLYP_UART2", 0x209100ee, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port UART2"),
        ("CFG-MSGOUT-PUBX_ID_POLYP_USB", 0x209100ef, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port USB"),

        ("CFG-MSGOUT-PUBX_ID_POLYP_SPI", 0x209100f0, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port SPI"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_I2C", 0x209100f1, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port I2C"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_UART1", 0x209100f2, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port UART1"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_UART2", 0x209100f3, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port UART2"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_USB", 0x209100f4, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port USB"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_SPI", 0x209100f5, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port SPI"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_I2C", 0x209100f6, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port I2C"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_UART1", 0x209100f7, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port UART1"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_UART2", 0x209100f8, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port UART2"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_USB", 0x209100f9, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port USB"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_SPI", 0x209100fa, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port SPI"),

        # CFG-MSGOUT-RTCM_3X
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_I2C", 0x209102bd, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_UART1", 0x209102be, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_UART2", 0x209102bf, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port UART2"),

        ("CFG-MSGOUT-RTCM_3X_TYPE1005_USB", 0x209102c0, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_SPI", 0x209102c1, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_I2C", 0x209102cc, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port I2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_UART1", 0x209102cd, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_UART2", 0x209102ce, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_USB", 0x209102cf, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port USB"),

        ("CFG-MSGOUT-RTCM_3X_TYPE1077_SPI", 0x209102d0, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_I2C", 0x209102d1, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_UART1", 0x209102d2, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_UART2", 0x209102d3, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_USB", 0x209102d4, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_SPI", 0x209102d5, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_I2C", 0x209102d6, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_UART1", 0x209102d7, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_UART2", 0x209102d8, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_USB", 0x209102d9, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_SPI", 0x209102da, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port SPI"),

        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_I2C", 0x209102fe, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 0 on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART1", 0x209102ff, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 0 message on port UART1"),

        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART2", 0x20910300, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 0 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB", 0x20910301, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 0 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_SPI", 0x20910302, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 0 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_I2C", 0x20910303, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_UART1", 0x20910304, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_UART2", 0x20910305, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_USB", 0x20910306, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_SPI", 0x20910307, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port SPI"),

        ("CFG-MSGOUT-RTCM_3X_TYPE1097_I2C", 0x20910318, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_UART1", 0x20910319, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_UART2", 0x2091031a, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_USB", 0x2091031b, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_SPI", 0x2091031c, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port SPI"),

        ("CFG-MSGOUT-RTCM_3X_TYPE1074_I2C", 0x2091035e, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1074_UART1", 0x2091035f, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port UART1"),

        ("CFG-MSGOUT-RTCM_3X_TYPE1074_UART2", 0x20910360, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1074_USB", 0x20910361, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1074_SPI", 0x20910362, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_I2C", 0x20910363, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_UART1", 0x20910364, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_UART2", 0x20910365, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_USB", 0x20910366, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_SPI", 0x20910367, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_I2C", 0x20910368, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_UART1", 0x20910369, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_UART2", 0x2091036a, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_USB", 0x2091036b, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_SPI", 0x2091036c, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_I2C", 0x2091036d, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_UART1", 0x2091036e, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_UART2", 0x2091036f, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port UART2"),

        ("CFG-MSGOUT-RTCM_3X_TYPE1124_USB", 0x20910370, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_SPI", 0x20910371, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port SPI"),

        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_I2C", 0x20910381, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 1 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_UART1", 0x20910382, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 1 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_UART2", 0x20910383, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 1 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB", 0x20910384, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 1 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_SPI", 0x20910385, "U1", 1, "",
         "Output rate of RTCM-3X-TYPE4072, sub-type 1 message on  port SPI"),

        # CFG-MSGOUT-UBX_AID
        ("CFG-MSGOUT-UBX_AID_INI_I2C", 0x209100fb, "U1", 1, "",
         "Output rate of the UBX-AID-INI message on port I2C"),
        ("CFG-MSGOUT-UBX_AID_INI_UART1", 0x209100fc, "U1", 1, "",
         "Output rate of the UBX-AID-INI message on port UART1"),
        ("CFG-MSGOUT-UBX_AID_INI_UART2", 0x209100fd, "U1", 1, "",
         "Output rate of the UBX-AID-INI message on port UART2"),
        ("CFG-MSGOUT-UBX_AID_INI_USB", 0x209100fe, "U1", 1, "",
         "Output rate of the UBX-AID-INI message on port USB"),
        ("CFG-MSGOUT-UBX_AID_INI_SPI", 0x209100ff, "U1", 1, "",
         "Output rate of the UBX-AID-INI message on port SPI"),

        ("CFG-MSGOUT-UBX_AID_EPH_I2C", 0x20910164, "U1", 1, "",
         "Output rate of the UBX-AID-EPH message on port I2C"),
        ("CFG-MSGOUT-UBX_AID_EPH_UART1", 0x20910165, "U1", 1, "",
         "Output rate of the UBX-AID-EPH message on port UART1"),
        ("CFG-MSGOUT-UBX_AID_EPH_UART2", 0x20910166, "U1", 1, "",
         "Output rate of the UBX-AID-EPH message on port UART2"),
        ("CFG-MSGOUT-UBX_AID_EPH_USB", 0x20910167, "U1", 1, "",
         "Output rate of the UBX-AID-EPH message on port USB"),
        ("CFG-MSGOUT-UBX_AID_EPH_SPI", 0x20910168, "U1", 1, "",
         "Output rate of the UBX-AID-EPH message on port SPI"),
        ("CFG-MSGOUT-UBX_AID_ALM_I2C", 0x2091016e, "U1", 1, "",
         "Output rate of the UBX-AID-ALM message on port I2C"),
        ("CFG-MSGOUT-UBX_AID_ALM_UART1", 0x2091016f, "U1", 1, "",
         "Output rate of the UBX-AID-ALM message on port UART1"),

        ("CFG-MSGOUT-UBX_AID_ALM_UART2", 0x20910170, "U1", 1, "",
         "Output rate of the UBX-AID-ALM message on port UART2"),
        ("CFG-MSGOUT-UBX_AID_ALM_USB", 0x20910171, "U1", 1, "",
         "Output rate of the UBX-AID-ALM message on port USB"),
        ("CFG-MSGOUT-UBX_AID_ALM_SPI", 0x20910172, "U1", 1, "",
         "Output rate of the UBX-AID-ALM message on port SPI"),

        ("CFG-MSGOUT-UBX_AID_AOP_I2C", 0x2091026d, "U1", 1, "",
         "Output rate of the UBX-AID-AOP message on port I2C"),
        ("CFG-MSGOUT-UBX_AID_AOP_UART1", 0x2091026e, "U1", 1, "",
         "Output rate of the UBX-AID-AOP message on port UART1"),
        ("CFG-MSGOUT-UBX_AID_AOP_UART2", 0x2091026f, "U1", 1, "",
         "Output rate of the UBX-AID-AOP message on port UART2"),
        ("CFG-MSGOUT-UBX_AID_AOP_USB", 0x20910270, "U1", 1, "",
         "Output rate of the UBX-AID-AOP message on port USB"),
        ("CFG-MSGOUT-UBX_AID_AOP_SPI", 0x20910271, "U1", 1, "",
         "Output rate of the UBX-AID-AOP message on port SPI"),

        # CFG-MSGOUT-UBX_ESF
        ("CFG-MSGOUT-UBX_ESF_STATUS_I2C", 0x20910105, "U1", 1, "",
         "Output rate of the UBX-ESF-STATUS message on port I2C"),
        ("CFG-MSGOUT-UBX_ESF_STATUS_UART1", 0x20910106, "U1", 1, "",
         "Output rate of the UBX-ESF-STATUS message on port UART1"),
        ("CFG-MSGOUT-UBX_ESF_STATUS_UART2", 0x20910107, "U1", 1, "",
         "Output rate of the UBX-ESF-STATUS message on port UART2"),
        ("CFG-MSGOUT-UBX_ESF_STATUS_USB", 0x20910108, "U1", 1, "",
         "Output rate of the UBX-ESF-STATUS message on port USB"),
        ("CFG-MSGOUT-UBX_ESF_STATUS_SPI", 0x20910109, "U1", 1, "",
         "Output rate of the UBX-ESF-STATUS message on port SPI"),

        ("CFG-MSGOUT-UBX_ESF_ALG_I2C", 0x2091010f, "U1", 1, "",
         "Output rate of the UBX-ESF-ALG message on port I2C"),

        ("CFG-MSGOUT-UBX_ESF_ALG_UART1", 0x20910110, "U1", 1, "",
         "Output rate of the UBX_ESF_ALG message on port UART1"),
        ("CFG-MSGOUT-UBX_ESF_ALG_UART2", 0x20910111, "U1", 1, "",
         "Output rate of the UBX_ESF_ALG message on port UART2"),
        ("CFG-MSGOUT-UBX_ESF_ALG_USB", 0x20910112, "U1", 1, "",
         "Output rate of the UBX_ESF_ALG message on port USB"),
        ("CFG-MSGOUT-UBX_ESF_ALG_SPI", 0x20910113, "U1", 1, "",
         "Output rate of the UBX_ESF_ALG message on port SPI"),
        ("CFG-MSGOUT-UBX_ESF_INS_I2C", 0x20910114, "U1", 1, "",
         "Output rate of the UBX-ESF-INS message on port I2C"),
        ("CFG-MSGOUT-UBX_ESF_INS_UART1", 0x20910115, "U1", 1, "",
         "Output rate of the UBX_ESF_INS message on port UART1"),
        ("CFG-MSGOUT-UBX_ESF_INS_UART2", 0x20910116, "U1", 1, "",
         "Output rate of the UBX_ESF_INS message on port UART2"),
        ("CFG-MSGOUT-UBX_ESF_INS_USB", 0x20910117, "U1", 1, "",
         "Output rate of the UBX_ESF_INS message on port USB"),
        ("CFG-MSGOUT-UBX_ESF_INS_SPI", 0x20910118, "U1", 1, "",
         "Output rate of the UBX_ESF_INS message on port SPI"),

        ("CFG-MSGOUT-UBX_ESF_INS_I2C", 0x20910114, "U1", 1, "",
         "Output rate of the UBX-ESF-INS message on port I2C"),
        ("CFG-MSGOUT-UBX_ESF_INS_UART1", 0x20910115, "U1", 1, "",
         "Output rate of the UBX_ESF_INS message on port UART1"),
        ("CFG-MSGOUT-UBX_ESF_INS_UART2", 0x20910116, "U1", 1, "",
         "Output rate of the UBX_ESF_INS message on port UART2"),
        ("CFG-MSGOUT-UBX_ESF_INS_USB", 0x20910117, "U1", 1, "",
         "Output rate of the UBX_ESF_INS message on port USB"),
        ("CFG-MSGOUT-UBX_ESF_INS_SPI", 0x20910118, "U1", 1, "",
         "Output rate of the UBX_ESF_INS message on port SPI"),

        ("CFG-MSGOUT-UBX_ESF_MEAS_I2C", 0x20910277, "U1", 1, "",
         "Output rate of the UBX-ESF-MEAS message on port I2C"),
        ("CFG-MSGOUT-UBX_ESF_MEAS_UART1", 0x20910278, "U1", 1, "",
         "Output rate of the UBX-ESF-MEAS message on port UART1"),
        ("CFG-MSGOUT-UBX_ESF_MEAS_UART2", 0x20910279, "U1", 1, "",
         "Output rate of the UBX-ESF-MEAS message on port UART2"),
        ("CFG-MSGOUT-UBX_ESF_MEAS_USB", 0x2091027a, "U1", 1, "",
         "Output rate of the UBX-ESF-MEAS message on port USB"),
        ("CFG-MSGOUT-UBX_ESF_MEAS_SPI", 0x2091027b, "U1", 1, "",
         "Output rate of the UBX-ESF_MEAS message on port SPI"),

        ("CFG-MSGOUT-UBX_ESF_RAW_I2C", 0x2091029f, "U1", 1, "",
         "Output rate of the UBX-ESF-RAW message on port I2C"),

        ("CFG-MSGOUT-UBX_ESF_RAW_UART1", 0x209102a0, "U1", 1, "",
         "Output rate of the UBX-ESF-RAW message on port UART1"),
        ("CFG-MSGOUT-UBX_ESF_RAW_UART2", 0x209102a1, "U1", 1, "",
         "Output rate of the UBX-ESF-RAW message on port UART2"),
        ("CFG-MSGOUT-UBX_ESF_RAW_USB", 0x209102a2, "U1", 1, "",
         "Output rate of the UBX-ESF-RAW message on port USB"),
        ("CFG-MSGOUT-UBX_ESF_RAW_SPI", 0x209102a3, "U1", 1, "",
         "Output rate of the UBX-ESF_RAW message on port SPI"),

        ("CFG-MSGOUT-UBX_ESF_CAL_I2C", 0x209106ac, "U1", 1, "",
         "Output rate of the UBX-ESF-CAL message on port I2C"),
        ("CFG-MSGOUT-UBX_ESF_CAL_UART1", 0x209106ad, "U1", 1, "",
         "Output rate of the UBX_ESF_CAL message on port UART1"),
        ("CFG-MSGOUT-UBX_ESF_CAL_UART2", 0x209106ae, "U1", 1, "",
         "Output rate of the UBX_ESF_CAL message on port UART2"),
        ("CFG-MSGOUT-UBX_ESF_CAL_USB", 0x209106af, "U1", 1, "",
         "Output rate of the UBX_ESF_CAL message on port USB"),
        ("CFG-MSGOUT-UBX_ESF_CAL_SPI", 0x209106b0, "U1", 1, "",
         "Output rate of the UBX_ESF_CAL message on port SPI"),

        # CFG-MSGOUT-UBX_HNR
        ("CFG-MSGOUT-UBX_HNR_PVT_I2C", 0x2091028b, "U1", 1, "",
         "Output rate of the UBX-HNR_PVT message on port I2C"),
        ("CFG-MSGOUT-UBX_HNR_PVT_UART1", 0x2091028c, "U1", 1, "",
         "Output rate of the UBX-HNR_PVT message on port UART1"),
        ("CFG-MSGOUT-UBX_HNR_PVT_UART2", 0x2091028d, "U1", 1, "",
         "Output rate of the UBX-HNR_PVT message on port UART2"),
        ("CFG-MSGOUT-UBX_HNR_PVT_USB", 0x2091028e, "U1", 1, "",
         "Output rate of the UBX-HNR_PVT message on port USB"),
        ("CFG-MSGOUT-UBX_HNR_PVT_SPI", 0x2091028f, "U1", 1, "",
         "Output rate of the UBX-HNR_PVT message on port SPI"),

        ("CFG-MSGOUT-UBX_HNR_INS_I2C", 0x20910372, "U1", 1, "",
         "Output rate of the UBX-HNR_INS message on port I2C"),
        ("CFG-MSGOUT-UBX_HNR_INS_UART1", 0x20910373, "U1", 1, "",
         "Output rate of the UBX-HNR_INS message on port UART1"),
        ("CFG-MSGOUT-UBX_HNR_INS_UART2", 0x20910374, "U1", 1, "",
         "Output rate of the UBX-HNR_INS message on port UART2"),
        ("CFG-MSGOUT-UBX_HNR_INS_USB", 0x20910375, "U1", 1, "",
         "Output rate of the UBX-HNR_INS message on port USB"),
        ("CFG-MSGOUT-UBX_HNR_INS_SPI", 0x20910376, "U1", 1, "",
         "Output rate of the UBX-HNR_INS message on port SPI"),
        ("CFG-MSGOUT-UBX_HNR_ATT_I2C", 0x20910377, "U1", 1, "",
         "Output rate of the UBX-HNR_ATT message on port I2C"),
        ("CFG-MSGOUT-UBX_HNR_ATT_UART1", 0x20910378, "U1", 1, "",
         "Output rate of the UBX-HNR_ATT message on port UART1"),
        ("CFG-MSGOUT-UBX_HNR_ATT_UART2", 0x20910379, "U1", 1, "",
         "Output rate of the UBX-HNR_ATT message on port UART2"),
        ("CFG-MSGOUT-UBX_HNR_ATT_USB", 0x2091037a, "U1", 1, "",
         "Output rate of the UBX-HNR_ATT message on port USB"),
        ("CFG-MSGOUT-UBX_HNR_ATT_SPI", 0x2091037b, "U1", 1, "",
         "Output rate of the UBX-HNR_ATT message on port SPI"),

        # CFG-MSGOUT-UBX_LOG
        ("CFG-MSGOUT-UBX_LOG_INFO_I2C", 0x20910259, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port I2C"),
        ("CFG-MSGOUT-UBX_LOG_INFO_UART1", 0x2091025a, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port UART1"),
        ("CFG-MSGOUT-UBX_LOG_INFO_UART2", 0x2091025b, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port UART2"),
        ("CFG-MSGOUT-UBX_LOG_INFO_USB", 0x2091025c, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port USB"),
        ("CFG-MSGOUT-UBX_LOG_INFO_SPI", 0x2091025d, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_RLM_I2C", 0x2091025e, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_RLM_UART1", 0x2091025f, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port UART1"),

        ("CFG-MSGOUT-UBX_RXM_RLM_UART2", 0x20910260, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_RLM_USB", 0x20910261, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_RLM_SPI", 0x20910262, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_I2C", 0x20910268, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_UART1", 0x20910269, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_UART2", 0x2091026a, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_USB", 0x2091026b, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_SPI", 0x2091026c, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_USB", 0x2091026b, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port USB"),

        ("CFG-MSGOUT-UBX_RXM_RAWX_I2C", 0x209102a4, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_UART1", 0x209102a5, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_UART2", 0x209102a6, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_USB", 0x209102a7, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_SPI", 0x209102a8, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port SPI"),

        # CFG-MSGOUT-UBX_MON
        ("CFG-MSGOUT-UBX_MON_SYS_I2C", 0x2091069d, "U1", 1, "",
         "Output rate of the UBX-MON-SYS message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_SYS_UART1", 0x2091069e, "U1", 1, "",
         "Output rate of the UBX-MON-SYS message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_SYS_UART2", 0x2091069f, "U1", 1, "",
         "Output rate of the UBX-MON-SYS message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_SYS_USB", 0x209106a0, "U1", 1, "",
         "Output rate of the UBX-MON-SYS message on port USB"),
        ("CFG-MSGOUT-UBX_MON_SYS_SPI", 0x209106a1, "U1", 1, "",
         "Output rate of the UBX-MON-SYS message on port SPI"),

        ("CFG-MSGOUT-UBX_MON_RXR_I2C", 0x20910187, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_RXR_UART1", 0x20910188, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_RXR_UART2", 0x20910189, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_RXR_USB", 0x2091018a, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port USB"),
        ("CFG-MSGOUT-UBX_MON_RXR_SPI", 0x2091018b, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port SPI"),

        ("CFG-MSGOUT-UBX_MON_MSGPP_I2C", 0x20910196, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_UART1", 0x20910197, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_UART2", 0x20910198, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_USB", 0x20910199, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port USB"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_SPI", 0x2091019a, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_I2C", 0x2091019b, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_UART1", 0x2091019c, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_UART2", 0x2091019d, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_USB", 0x2091019e, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port USB"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_SPI", 0x2091019f, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port SPI"),

        ("CFG-MSGOUT-UBX_MON_RXBUF_I2C", 0x209101a0, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_UART1", 0x209101a1, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_UART2", 0x209101a2, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_USB", 0x209101a3, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port USB"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_SPI", 0x209101a4, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_IO_I2C", 0x209101a5, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_IO_UART1", 0x209101a6, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_IO_UART2", 0x209101a7, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_IO_USB", 0x209101a8, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port USB"),
        ("CFG-MSGOUT-UBX_MON_IO_SPI", 0x209101a9, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port SPI"),

        ("CFG-MSGOUT-UBX_MON_HW_I2C", 0x209101b4, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_HW_UART1", 0x209101b5, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_HW_UART2", 0x209101b6, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_HW_USB", 0x209101b7, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port USB"),
        ("CFG-MSGOUT-UBX_MON_HW_SPI", 0x209101b8, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_HW2_I2C", 0x209101b9, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_HW2_UART1", 0x209101ba, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_HW2_UART2", 0x209101bb, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_HW2_USB", 0x209101bc, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port USB"),
        ("CFG-MSGOUT-UBX_MON_HW2_SPI", 0x209101bd, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port SPI"),

        ("CFG-MSGOUT-UBX_MON_PMP_I2C", 0x20910322, "U1", 1, "",
         "Output rate of the UBX-MON-PMP message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_PMP_UART1", 0x20910323, "U1", 1, "",
         "Output rate of the UBX-MON-PMP message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_PMP_UART2", 0x20910324, "U1", 1, "",
         "Output rate of the UBX-MON-PMP message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_PMP_USB", 0x20910325, "U1", 1, "",
         "Output rate of the UBX-MON-PMP message on port USB"),
        ("CFG-MSGOUT-UBX_MON_PMP_SPI", 0x20910326, "U1", 1, "",
         "Output rate of the UBX-MON-PMP message on port SPI"),

        ("CFG-MSGOUT-UBX_MON_COMMS_I2C", 0x2091034f, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port I2C"),

        ("CFG-MSGOUT-UBX_MON_COMMS_UART1", 0x20910350, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_COMMS_UART2", 0x20910351, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_COMMS_USB", 0x20910352, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port USB"),
        ("CFG-MSGOUT-UBX_MON_COMMS_SPI", 0x20910353, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_HW3_I2C", 0x20910354, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_HW3_UART1", 0x20910355, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_HW3_UART2", 0x20910356, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_HW3_USB", 0x20910357, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port USB"),
        ("CFG-MSGOUT-UBX_MON_HW3_SPI", 0x20910358, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_RF_I2C", 0x20910359, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_RF_UART1", 0x2091035a, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_RF_UART2", 0x2091035b, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_RF_USB", 0x2091035c, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port USB"),
        ("CFG-MSGOUT-UBX_MON_RF_SPI", 0x2091035d, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port SPI"),

        ("CFG-MSGOUT-UBX_MON_SPAN_I2C", 0x2091038b, "U1", 1, "",
         "Output rate of the UBX-MON-SPAN message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_SPAN_UART1", 0x2091038c, "U1", 1, "",
         "Output rate of the UBX-MON-SPAN message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_SPAN_UART2", 0x2091038d, "U1", 1, "",
         "Output rate of the UBX-MON-SPAN message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_SPAN_USB", 0x2091038e, "U1", 1, "",
         "Output rate of the UBX-MON-SPAN message on port USB"),
        ("CFG-MSGOUT-UBX_MON_SPAN_SPI", 0x2091038f, "U1", 1, "",
         "Output rate of the UBX-MON-SPAN message on port SPI"),

        ("CFG-MSGOUT-UBX_MON_INST_I2C", 0x209103cb, "U1", 1, "",
         "Output rate of the UBX-MON-INST message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_INST_UART1", 0x209103cc, "U1", 1, "",
         "Output rate of the UBX-MON-INST message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_INST_UART2", 0x209103cd, "U1", 1, "",
         "Output rate of the UBX-MON-INST message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_INST_USB", 0x209103ce, "U1", 1, "",
         "Output rate of the UBX-MON-INST message on port USB"),
        ("CFG-MSGOUT-UBX_MON_INST_SPI", 0x209103cf, "U1", 1, "",
         "Output rate of the UBX-MON-INST message on port SPI"),

        # CFG-MSGOUT-UBX_NAV
        ("CFG-MSGOUT-UBX_NAV_SOL_I2C", 0x20910001, "U1", 1, "",
         "Output rate of the UBX-NAV-SOL message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SOL_UART1", 0x20910002, "U1", 1, "",
         "Output rate of the UBX-NAV-SOL message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SOL_UART2", 0x20910003, "U1", 1, "",
         "Output rate of the UBX-NAV-SOL message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SOL_USB", 0x20910004, "U1", 1, "",
         "Output rate of the UBX-NAV-SOL message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SOL_SPI", 0x20910005, "U1", 1, "",
         "Output rate of the UBX-NAV-SOL message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_PVT_I2C", 0x20910006, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_PVT_UART1", 0x20910007, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_PVT_UART2", 0x20910008, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_PVT_USB", 0x20910009, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_PVT_SPI", 0x2091000a, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_SVINFO_I2C", 0x2091000b, "U1", 1, "",
         "Output rate of the UBX-NAV-SVINFO message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SVINFO_UART1", 0x2091000c, "U1", 1, "",
         "Output rate of the UBX-NAV-SVINFO message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SVINFO_UART2", 0x2091000d, "U1", 1, "",
         "Output rate of the UBX-NAV-SVINFO message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SVINFO_USB", 0x2091000e, "U1", 1, "",
         "Output rate of the UBX-NAV-SVINFO message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SVINFO_SPI", 0x2091000f, "U1", 1, "",
         "Output rate of the UBX-NAV-SVINFO message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_ORB_I2C", 0x20910010, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_ORB_UART1", 0x20910011, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_ORB_UART2", 0x20910012, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_ORB_USB", 0x20910013, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_ORB_SPI", 0x20910014, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_SAT_I2C", 0x20910015, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SAT_UART1", 0x20910016, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SAT_UART2", 0x20910017, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SAT_USB", 0x20910018, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SAT_SPI", 0x20910019, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port SPI"),
        # M10S, protVer 34.00
        ("CFG-MSGOUT-UBX_NAV_STATUS_I2C", 0x2091001a, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_UART1", 0x2091001b, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_UART2", 0x2091001c, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_USB", 0x2091001d, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_SPI", 0x2091001e, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_ATT_I2C", 0x2091001f, "U1", 1, "",
         "Output rate of the UBX-NAV-ATT message on port I2C"),

        ("CFG-MSGOUT-UBX_NAV_ATT_UART1", 0x20910020, "U1", 1, "",
         "Output rate of the UBX-NAV-ATT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_ATT_UART2", 0x20910021, "U1", 1, "",
         "Output rate of the UBX-NAV-ATT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_ATT_USB", 0x20910022, "U1", 1, "",
         "Output rate of the UBX-NAV-ATT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_ATT_SPI", 0x20910023, "U1", 1, "",
         "Output rate of the UBX-NAV-ATT message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_POSECEF_I2C", 0x20910024, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_UART1", 0x20910025, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_UART2", 0x20910026, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_USB", 0x20910027, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_SPI", 0x20910028, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_I2C", 0x20910029, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_UART1", 0x2091002a, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_UART2", 0x2091002b, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_USB", 0x2091002c, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_SPI", 0x2091002d, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_I2C", 0x2091002e, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_UART1", 0x2091002f, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port UART1"),

        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_UART2", 0x20910030, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_USB", 0x20910031, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_SPI", 0x20910032, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_I2C", 0x20910033, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1", 0x20910034, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART2", 0x20910035, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB", 0x20910036, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_SPI", 0x20910037, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_DOP_I2C", 0x20910038, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_DOP_UART1", 0x20910039, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_DOP_UART2", 0x2091003a, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_DOP_USB", 0x2091003b, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_DOP_SPI", 0x2091003c, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_I2C", 0x2091003d, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_UART1", 0x2091003e, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_UART2", 0x2091003f, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port UART2"),

        ("CFG-MSGOUT-UBX_NAV_VELECEF_USB", 0x20910040, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_SPI", 0x20910041, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_I2C", 0x20910042, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_UART1", 0x20910043, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_UART2", 0x20910044, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_USB", 0x20910045, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_SPI", 0x20910046, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_I2C", 0x20910047, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1", 0x20910048, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_UART2", 0x20910049, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_USB", 0x2091004a, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_SPI", 0x2091004b, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_I2C", 0x2091004c, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_UART1", 0x2091004d, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_UART2", 0x2091004e, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_USB", 0x2091004f, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_SPI", 0x20910050, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_I2C", 0x20910051, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_UART1", 0x20910052, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_UART2", 0x20910053, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_USB", 0x20910054, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_SPI", 0x20910055, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_I2C", 0x20910056, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_UART1", 0x20910057, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_UART2", 0x20910058, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_USB", 0x20910059, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_SPI", 0x2091005a, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_I2C", 0x2091005b, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEUTC message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1", 0x2091005c, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEUTC message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_UART2", 0x2091005d, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEUTC message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_USB", 0x2091005e, "U1", 1, "",
         "Output rate of the UBX-NAV- TIMEUTC message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_SPI", 0x2091005f, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEUTC message on port S"),

        ("CFG-MSGOUT-UBX_NAV_TIMELS_I2C", 0x20910060, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_UART1", 0x20910061, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_UART2", 0x20910062, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_USB", 0x20910063, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_SPI", 0x20910064, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_CLOCK_I2C", 0x20910065, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_CLOCK_UART1", 0x20910066, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_CLOCK_UART2", 0x20910067, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_CLOCK_USB", 0x20910068, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_CLOCK_SPI", 0x20910069, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_I2C", 0x2091006a, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_UART1", 0x2091006b, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_UART2", 0x2091006c, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_USB", 0x2091006d, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_SPI", 0x2091006e, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_DGPS_I2C", 0x20910074, "U1", 1, "",
         "Output rate of the UBX-NAV-DGPS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_DGPS_UART1", 0x20910075, "U1", 1, "",
         "Output rate of the UBX_NAV-DGPS port UART1"),
        ("CFG-MSGOUT-UBX_NAV_DGPS_UART2", 0x20910076, "U1", 1, "",
         "Output rate of the UBX_NAV-DGPS port UART2"),
        ("CFG-MSGOUT-UBX_NAV_DGPS_USB", 0x20910077, "U1", 1, "",
         "Output rate of the UBX_NAV-DGPS port USB"),
        ("CFG-MSGOUT-UBX_NAV_DGPS_SPI", 0x20910078, "U1", 1, "",
         "Output rate of the UBX_NAV-DGPS port SPI"),

        ("CFG-MSGOUT-UBX_NAV_AOPSTATUS_I2C", 0x20910079, "U1", 1, "",
         "Output rate of the UBX-MON-AOPSTATUS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_AOPSTATUS_UART1", 0x2091007a, "U1", 1, "",
         "Output rate of the UBX-MON-AOPSTATUS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_AOPSTATUS_UART2", 0x2091007b, "U1", 1, "",
         "Output rate of the UBX-MON-AOPSTATUS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_AOPSTATUS_USB", 0x2091007c, "U1", 1, "",
         "Output rate of the UBX-MON-AOPSTATUS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_AOPSTATUS_SPI", 0x2091007d, "U1", 1, "",
         "Output rate of the UBX-MON-AOPSTATUS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_ODO_I2C", 0x2091007e, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_ODO_UART1", 0x2091007f, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port UART1"),

        ("CFG-MSGOUT-UBX_NAV_ODO_UART2", 0x20910080, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_ODO_USB", 0x20910081, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_ODO_SPI", 0x20910082, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_COV_I2C", 0x20910083, "U1", 1, "",
         "Output rate of the UBX-NAV-COV message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_COV_UART1", 0x20910084, "U1", 1, "",
         "Output rate of the UBX-NAV-COV message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_COV_UART2", 0x20910085, "U1", 1, "",
         "Output rate of the UBX-NAV-COV message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_COV_USB", 0x20910086, "U1", 1, "",
         "Output rate of the UBX-NAV-COV message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_COV_SPI", 0x20910087, "U1", 1, "",
         "Output rate of the UBX-NAV-COV message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_I2C", 0x20910088, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_UART1", 0x20910089, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_UART2", 0x2091008a, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_USB", 0x2091008b, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_SPI", 0x2091008c, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_I2C", 0x2091008d, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1", 0x2091008e, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_UART2", 0x2091008f, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port UART2"),

        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_USB", 0x20910090, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_SPI", 0x20910091, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_I2C", 0x209100a1, "U1", 1, "",
         "Output rate of the UBX-NAV-GEOFENCE message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_UART1", 0x209100a2, "U1", 1, "",
         "Output rate of the UBX-NAV-GEOFENCE message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_UART2", 0x209100a3, "U1", 1, "",
         "Output rate of the UBX-NAV-GEOFENCE message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_USB", 0x209100a4, "U1", 1, "",
         "Output rate of the UBX-NAV- GEOFENCE message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_SPI", 0x209100a5, "U1", 1, "",
         "Output rate of the UBX-NAV-GEOFENCE message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_EOE_I2C", 0x2091015f, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port I2C"),

        ("CFG-MSGOUT-UBX_NAV_EOE_UART1", 0x20910160, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_EOE_UART2", 0x20910161, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_EOE_USB", 0x20910162, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_EOE_SPI", 0x20910163, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_EELL_I2C", 0x20910313, "U1", 1, "",
         "Output rate of the UBX-NAV-EELL message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_EELL_UART1", 0x20910314, "U1", 1, "",
         "Output rate of the UBX-NAV-EELL message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_EELL_UART2", 0x20910315, "U1", 1, "",
         "Output rate of the UBX-NAV-EELL message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_EELL_USB", 0x20910316, "U1", 1, "",
         "Output rate of the UBX-NAV-EELL message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_EELL_SPI", 0x20910317, "U1", 1, "",
         "Output rate of the UBX-NAV-EELL message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_SLAS_I2C", 0x20910336, "U1", 1, "",
         "Output rate of the UBX-NAV-SLAS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SLAS_UART1", 0x20910337, "U1", 1, "",
         "Output rate of the UBX-NAV-SLAS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SLAS_UART2", 0x20910338, "U1", 1, "",
         "Output rate of the UBX-NAV-SLAS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SLAS_USB", 0x20910339, "U1", 1, "",
         "Output rate of the UBX-NAV-SLAS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SLAS_SPI", 0x2091033a, "U1", 1, "",
         "Output rate of the UBX-NAV-SLAS message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_SIG_I2C", 0x20910345, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SIG_UART1", 0x20910346, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SIG_UART2", 0x20910347, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SIG_USB", 0x20910348, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SIG_SPI", 0x20910349, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_TIMEQZSS_I2C", 0x20910386, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEQZSS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEQZSS_UART1", 0x20910387, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEQZSS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEQZSS_UART2", 0x20910388, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEQZSS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEQZSS_USB", 0x20910389, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEQZSS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEQZSS_SPI", 0x2091038a, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEQZSS message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_TIMETRUSTED_I2C", 0x209103a8, "U1", 1, "",
         "Output rate of the UBX-NAV_TIMETRUSTED message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMETRUSTED_UART1", 0x209103a9, "U1", 1, "",
         "Output rate of the UBX-NAV_TIMETRUSTED message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMETRUSTED_UART2", 0x209103aa, "U1", 1, "",
         "Output rate of the UBX-NAV_TIMETRUSTED message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMETRUSTED_USB", 0x209103ab, "U1", 1, "",
         "Output rate of the UBX-NAV_TIMETRUSTED message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMETRUSTED_SPI", 0x209103ac, "U1", 1, "",
         "Output rate of the UBX-NAV_TIMETRUSTED message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_PL_I2C", 0x20910415, "U1", 1, "",
         "Output rate of the UBX-NAV-PL message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_PL_UART1", 0x20910416, "U1", 1, "",
         "Output rate of the UBX-NAV-PL message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_PL_UART2", 0x20910417, "U1", 1, "",
         "Output rate of the UBX-NAV-PL message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_PL_USB", 0x20910418, "U1", 1, "",
         "Output rate of the UBX-NAV-PL message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_PL_SPI", 0x20910419, "U1", 1, "",
         "Output rate of the UBX-NAV-PL message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_NMI_I2C", 0x20910590, "U1", 1, "",
         "Output rate of the UBX-NAV-NMI message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_NMI_UART1", 0x20910591, "U1", 1, "",
         "Output rate of the UBX-NAV-NMI message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_NMI_UART2", 0x20910592, "U1", 1, "",
         "Output rate of the UBX-NAV-NMI message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_NMI_USB", 0x20910593, "U1", 1, "",
         "Output rate of the UBX-NAV-NMI message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_NMI_SPI", 0x20910594, "U1", 1, "",
         "Output rate of the UBX-NAV-NMI message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_PVAT_I2C", 0x2091062a, "U1", 1, "",
         "Output rate of the UBX-NAV-PVAT message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_PVAT_UART1", 0x2091062b, "U1", 1, "",
         "Output rate of the UBX-NAV-PVAT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_PVAT_UART2", 0x2091062c, "U1", 1, "",
         "Output rate of the UBX-NAV-PVAT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_PVAT_USB", 0x2091062d, "U1", 1, "",
         "Output rate of the UBX-NAV-PVAT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_PVAT_SPI", 0x2091062e, "U1", 1, "",
         "Output rate of the UBX-NAV-PVAT message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV_TIMENAVIC_I2C", 0x209106a2, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMENAVIC message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMENAVIC_UART1", 0x209106a3, "U1", 1, "",
         "Output rate of the UBX_NAV-TIMENAVIC port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMENAVIC_UART2", 0x209106a4, "U1", 1, "",
         "Output rate of the UBX_NAV-TIMENAVIC port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMENAVIC_USB", 0x209106a5, "U1", 1, "",
         "Output rate of the UBX_NAV-TIMENAVIC port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMENAVIC_SPI", 0x209106a6, "U1", 1, "",
         "Output rate of the UBX_NAV-TIMENAVIC port SPI"),

        ("CFG-MSGOUT-UBX_NAV_CFB_I2C", 0x209106d4, "U1", 1, "",
         "Output rate of the UBX-NAV-CFB message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_CFB_UART1", 0x209106d5, "U1", 1, "",
         "Output rate of the UBX_NAV-CFB port UART1"),
        ("CFG-MSGOUT-UBX_NAV_CFB_UART2", 0x209106d6, "U1", 1, "",
         "Output rate of the UBX_NAV-CFB port UART2"),
        ("CFG-MSGOUT-UBX_NAV_CFB_USB", 0x209106d7, "U1", 1, "",
         "Output rate of the UBX_NAV-CFB port USB"),
        ("CFG-MSGOUT-UBX_NAV_CFB_SPI", 0x209106d8, "U1", 1, "",
         "Output rate of the UBX_NAV-CFB port SPI"),

        # CFG-MSGOUT-UBX_NAV2
        ("CFG-MSGOUT-UBX_NAV2_CLOCK_I2C", 0x20910430, "U1", 1, "",
         "Output rate of the UBX-NAV2-CLOCK message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_CLOCK_UART1", 0x20910431, "U1", 1, "",
         "Output rate of the UBX-NAV2-CLOCK message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_CLOCK_UART2", 0x20910432, "U1", 1, "",
         "Output rate of the UBX-NAV2-CLOCK message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_CLOCK_USB", 0x20910433, "U1", 1, "",
         "Output rate of the UBX-NAV2-CLOCK message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_CLOCK_SPI", 0x20910434, "U1", 1, "",
         "Output rate of the UBX-NAV2-CLOCK message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_COV_I2C", 0x20910435, "U1", 1, "",
         "Output rate of the UBX-NAV2-COV message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_COV_UART1", 0x20910436, "U1", 1, "",
         "Output rate of the UBX-NAV2-COV message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_COV_UART2", 0x20910437, "U1", 1, "",
         "Output rate of the UBX-NAV2-COV message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_COV_USB", 0x20910438, "U1", 1, "",
         "Output rate of the UBX-NAV2-COV message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_COV_SPI", 0x20910439, "U1", 1, "",
         "Output rate of the UBX-NAV2-COV message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_DOP_I2C", 0x20910465, "U1", 1, "",
         "Output rate of the UBX-NAV2-DOP message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_DOP_UART1", 0x20910466, "U1", 1, "",
         "Output rate of the UBX-NAV2-DOP message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_DOP_UART2", 0x20910467, "U1", 1, "",
         "Output rate of the UBX-NAV2-DOP message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_DOP_USB", 0x20910468, "U1", 1, "",
         "Output rate of the UBX-NAV2-DOP message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_DOP_SPI", 0x20910469, "U1", 1, "",
         "Output rate of the UBX-NAV2-DOP message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_EELL_I2C", 0x20910470, "U1", 1, "",
         "Output rate of the UBX-NAV2-EELL message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_EELL_UART1", 0x20910471, "U1", 1, "",
         "Output rate of the UBX-NAV2-EELL message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_EELL_UART2", 0x20910472, "U1", 1, "",
         "Output rate of the UBX-NAV2-EELL message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_EELL_USB", 0x20910473, "U1", 1, "",
         "Output rate of the UBX-NAV2-EELL message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_EELL_SPI", 0x20910474, "U1", 1, "",
         "Output rate of the UBX-NAV2-EELL message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_ODO_I2C", 0x20910475, "U1", 1, "",
         "Output rate of the UBX-NAV2-ODO message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_ODO_UART1", 0x20910476, "U1", 1, "",
         "Output rate of the UBX-NAV2-ODO message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_ODO_UART2", 0x20910477, "U1", 1, "",
         "Output rate of the UBX-NAV2-ODO message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_ODO_USB", 0x20910478, "U1", 1, "",
         "Output rate of the UBX-NAV2-ODO message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_ODO_SPI", 0x20910479, "U1", 1, "",
         "Output rate of the UBX-NAV2-ODO message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_POSECEF_I2C", 0x20910480, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSECEF message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_POSECEF_UART1", 0x20910481, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSECEF message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_POSECEF_UART2", 0x20910482, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSECEF message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_POSECEF_USB", 0x20910483, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSECEF message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_POSECEF_SPI", 0x20910484, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSECEF message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_POSLLH_I2C", 0x20910485, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSLLH message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_POSLLH_UART1", 0x20910486, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSLLH message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_POSLLH_UART2", 0x20910487, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSLLH message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_POSLLH_USB", 0x20910488, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSLLH message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_POSLLH_SPI", 0x20910489, "U1", 1, "",
         "Output rate of the UBX-NAV2-POSLLH message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_PVT_I2C", 0x20910490, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVT message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_PVT_UART1", 0x20910491, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_PVT_UART2", 0x20910492, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_PVT_USB", 0x20910493, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_PVT_SPI", 0x20910494, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVT message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_SAT_I2C", 0x20910495, "U1", 1, "",
         "Output rate of the UBX-NAV2-SAT message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_SAT_UART1", 0x20910496, "U1", 1, "",
         "Output rate of the UBX-NAV2-SAT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_SAT_UART2", 0x20910497, "U1", 1, "",
         "Output rate of the UBX-NAV2-SAT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_SAT_USB", 0x20910498, "U1", 1, "",
         "Output rate of the UBX-NAV2-SAT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_SAT_SPI", 0x20910499, "U1", 1, "",
         "Output rate of the UBX-NAV2-SAT message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_SBAS_I2C", 0x20910500, "U1", 1, "",
         "Output rate of the UBX-NAV2-SBAS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_SBAS_UART1", 0x20910501, "U1", 1, "",
         "Output rate of the UBX-NAV2-SBAS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_SBAS_UART2", 0x20910502, "U1", 1, "",
         "Output rate of the UBX-NAV2-SBAS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_SBAS_USB", 0x20910503, "U1", 1, "",
         "Output rate of the UBX-NAV2-SBAS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_SBAS_SPI", 0x20910504, "U1", 1, "",
         "Output rate of the UBX-NAV2-SBAS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_SIG_I2C", 0x20910505, "U1", 1, "",
         "Output rate of the UBX-NAV2-SIG message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_SIG_UART1", 0x20910506, "U1", 1, "",
         "Output rate of the UBX-NAV2-SIG message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_SIG_UART2", 0x20910507, "U1", 1, "",
         "Output rate of the UBX-NAV2-SIG message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_SIG_USB", 0x20910508, "U1", 1, "",
         "Output rate of the UBX-NAV2-SIG message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_SIG_SPI", 0x20910509, "U1", 1, "",
         "Output rate of the UBX-NAV2-SIG message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_SLAS_I2C", 0x20910510, "U1", 1, "",
         "Output rate of the UBX-NAV2-SLAS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_SLAS_UART1", 0x20910511, "U1", 1, "",
         "Output rate of the UBX-NAV2-SLAS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_SLAS_UART2", 0x20910512, "U1", 1, "",
         "Output rate of the UBX-NAV2-SLAS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_SLAS_USB", 0x20910513, "U1", 1, "",
         "Output rate of the UBX-NAV2-SLAS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_SLAS_SPI", 0x20910514, "U1", 1, "",
         "Output rate of the UBX-NAV2-SLAS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_STATUS_I2C", 0x20910515, "U1", 1, "",
         "Output rate of the UBX-NAV2-STATUS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_STATUS_UART1", 0x20910516, "U1", 1, "",
         "Output rate of the UBX-NAV2-STATUS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_STATUS_UART2", 0x20910517, "U1", 1, "",
         "Output rate of the UBX-NAV2-STATUS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_STATUS_USB", 0x20910518, "U1", 1, "",
         "Output rate of the UBX-NAV2-STATUS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_STATUS_SPI", 0x20910519, "U1", 1, "",
         "Output rate of the UBX-NAV2-STATUS message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_SVIN_I2C", 0x20910520, "U1", 1, "",
         "Output rate of the UBX-NAV2-SVIN message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_SVIN_UART1", 0x20910521, "U1", 1, "",
         "Output rate of the UBX-NAV2-SVIN message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_SVIN_UART2", 0x20910522, "U1", 1, "",
         "Output rate of the UBX-NAV2-SVIN message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_SVIN_USB", 0x20910523, "U1", 1, "",
         "Output rate of the UBX-NAV2-SVIN message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_SVIN_SPI", 0x20910524, "U1", 1, "",
         "Output rate of the UBX-NAV2-SVIN message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEBDS_I2C", 0x20910525, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEBDS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEBDS_UART1", 0x20910526, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEBDS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEBDS_UART2", 0x20910527, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEBDS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEBDS_USB", 0x20910528, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEBDS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEBDS_SPI", 0x20910529, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEBDS message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_TIMEGAL_I2C", 0x20910530, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGAL message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGAL_UART1", 0x20910531, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGAL message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGAL_UART2", 0x20910532, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGAL message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGAL_USB", 0x20910533, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGAL message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGAL_SPI", 0x20910534, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGAL message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGLO_I2C", 0x20910535, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGLO message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGLO_UART1", 0x20910536, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGLO message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGLO_UART2", 0x20910537, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGLO message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGLO_USB", 0x20910538, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGLO message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGLO_SPI", 0x20910539, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGLO message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_TIMEGPS_I2C", 0x20910540, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGPS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGPS_UART1", 0x20910541, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGPS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGPS_UART2", 0x20910542, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGPS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGPS_USB", 0x20910543, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGPS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEGPS_SPI", 0x20910544, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEGPS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_TIMELS_I2C", 0x20910545, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMELS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_TIMELS_UART1", 0x20910546, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMELS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_TIMELS_UART2", 0x20910547, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMELS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_TIMELS_USB", 0x20910548, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMELS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_TIMELS_SPI", 0x20910549, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMELS message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_TIMEUTC_I2C", 0x20910550, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEUTC message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEUTC_UART1", 0x20910551, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEUTC message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEUTC_UART2", 0x20910552, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEUTC message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEUTC_USB", 0x20910553, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEUTC message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEUTC_SPI", 0x20910554, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEUTC message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_VELECEF_I2C", 0x20910555, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELECEF message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_VELECEF_UART1", 0x20910556, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELECEF message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_VELECEF_UART2", 0x20910557, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELECEF message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_VELECEF_USB", 0x20910558, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELECEF message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_VELECEF_SPI", 0x20910559, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELECEF message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_VELNED_I2C", 0x20910560, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELNED message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_VELNED_UART1", 0x20910561, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELNED message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_VELNED_UART2", 0x20910562, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELNED message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_VELNED_USB", 0x20910563, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELNED message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_VELNED_SPI", 0x20910564, "U1", 1, "",
         "Output rate of the UBX-NAV2-VELNED message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV2_EOE_I2C", 0x20910565, "U1", 1, "",
         "Output rate of the UBX-NAV2-EOE message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_EOE_UART1", 0x20910566, "U1", 1, "",
         "Output rate of the UBX-NAV2-EOE message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_EOE_UART2", 0x20910567, "U1", 1, "",
         "Output rate of the UBX-NAV2-EOE message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_EOE_USB", 0x20910568, "U1", 1, "",
         "Output rate of the UBX-NAV2-EOE message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_EOE_SPI", 0x20910569, "U1", 1, "",
         "Output rate of the UBX-NAV2-EOE message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_TIMEQZSS_I2C", 0x20910575, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEQZSS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEQZSS_UART1", 0x20910576, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEQZSS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEQZSS_UART2", 0x20910577, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEQZSS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEQZSS_USB", 0x20910578, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEQZSS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_TIMEQZSS_SPI", 0x20910579, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMEQZSS message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_PVAT_I2C", 0x2091062f, "U1", 1, "",
         "Output rate of the UBX-NA2-PVAT message on port I2C"),

        ("CFG-MSGOUT-UBX_NAV2_PVAT_UART1", 0x20910630, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVAT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_PVAT_UART2", 0x20910631, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVAT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_PVAT_USB", 0x20910632, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVAT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_PVAT_SPI", 0x20910633, "U1", 1, "",
         "Output rate of the UBX-NAV2-PVAT message on port SPI"),

        ("CFG-MSGOUT-UBX_NAV2_TIMENAVIC_I2C", 0x209106a7, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMENAVIC message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV2_TIMENAVIC_UART1", 0x209106a8, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMENAVIC message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV2_TIMENAVIC_UART2", 0x209106a9, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMENAVIC message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV2_TIMENAVIC_USB", 0x209106aa, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMENAVIC message on port USB"),
        ("CFG-MSGOUT-UBX_NAV2_TIMENAVIC_SPI", 0x209106ab, "U1", 1, "",
         "Output rate of the UBX-NAV2-TIMENAVIC message on port SPI"),

        # CFG-MSGOUT-UBX_RXM
        ("CFG-MSGOUT-UBX_RXM_SVSI_I2C", 0x20910150, "U1", 1, "",
         "Output rate of the UBX-RXM-SVSI message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_SVSI_UART1", 0x20910151, "U1", 1, "",
         "Output rate of the UBX-RXM-SVSI message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_SVSI_UART2", 0x20910152, "U1", 1, "",
         "Output rate of the UBX-RXM-SVSI message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_SVSI_USB", 0x20910153, "U1", 1, "",
         "Output rate of the UBX-RXM-SVSI message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_SVSI_SPI", 0x20910154, "U1", 1, "",
         "Output rate of the UBX-RXM-SVSI message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_IMES_I2C", 0x2091015a, "U1", 1, "",
         "Output rate of the UBX-RXM-IMES message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_IMES_UART1", 0x2091015b, "U1", 1, "",
         "Output rate of the UBX-RXM-IMES message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_IMES_UART2", 0x2091015c, "U1", 1, "",
         "Output rate of the UBX-RXM-IMES message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_IMES_USB", 0x2091015d, "U1", 1, "",
         "Output rate of the UBX-RXM-IMES message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_IMES_SPI", 0x2091015e, "U1", 1, "",
         "Output rate of the UBX-RXM-IMES message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_EPH_I2C", 0x20910169, "U1", 1, "",
         "Output rate of the UBX-RXM-EPH message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_EPH_UART1", 0x2091016a, "U1", 1, "",
         "Output rate of the UBX-RXM-EPH message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_EPH_UART2", 0x2091016b, "U1", 1, "",
         "Output rate of the UBX-RXM-EPH message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_EPH_USB", 0x2091016c, "U1", 1, "",
         "Output rate of the UBX-RXM-EPH message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_EPH_SPI", 0x2091016d, "U1", 1, "",
         "Output rate of the UBX-RXM-EPH message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_ALM_I2C", 0x20910173, "U1", 1, "",
         "Output rate of the UBX-RXM-ALM message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_ALM_UART1", 0x20910174, "U1", 1, "",
         "Output rate of the UBX-RXM-ALM message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_ALM_UART2", 0x20910175, "U1", 1, "",
         "Output rate of the UBX-RXM-ALM message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_ALM_USB", 0x20910176, "U1", 1, "",
         "Output rate of the UBX-RXM-ALM message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_ALM_SPI", 0x20910177, "U1", 1, "",
         "Output rate of the UBX-RXM-ALM message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_MEASX_I2C", 0x20910204, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_MEASX_UART1", 0x20910205, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_MEASX_UART2", 0x20910206, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_MEASX_USB", 0x20910207, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_MEASX_SPI", 0x20910208, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_SFRBX_I2C", 0x20910231, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_UART1", 0x20910232, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_UART2", 0x20910233, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_USB", 0x20910234, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_SPI", 0x20910235, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_PMP_I2C", 0x2091031d, "U1", 1, "",
         "Output rate of the UBX_RXM_PMP message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_PMP_UART1", 0x2091031e, "U1", 1, "",
         "Output rate of the UBX_RXM_PMP message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_PMP_UART2", 0x2091031f, "U1", 1, "",
         "Output rate of the UBX_RXM_PMP message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_PMP_USB", 0x20910320, "U1", 1, "",
         "Output rate of the UBX_RXM_PMP message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_PMP_SPI", 0x20910321, "U1", 1, "",
         "Output rate of the UBX_RXM_PMP message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_QZSSL6_UART1", 0x2091033b, "U1", 1, "",
         "output rate of the UBX-RXM-QZSSL6 message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_QZSSL6_UART2", 0x2091033c, "U1", 1, "",
         "output rate of the UBX-RXM-QZSSL6 message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_QZSSL6_USB", 0x2091033d, "U1", 1, "",
         "output rate of the UBX-RXM-QZSSL6 message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_QZSSL6_SPI", 0x2091033e, "U1", 1, "",
         "output rate of the UBX-RXM-QZSSL6 message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_QZSSL6_I2C", 0x2091033f, "U1", 1, "",
         "output rate of the UBX-RXM-QZSSL6 message on port I2C"),

        ("CFG-MSGOUT-UBX_RXM_MEASX2_I2C", 0x209103c1, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX2 message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_MEASX2_UART1", 0x209103c2, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX2 message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_MEASX2_UART2", 0x209103c3, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX2 message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_MEASX2_USB", 0x209103c4, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX2 message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_MEASX2_SPI", 0x209103c5, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX2 message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_SPARTN_I2C", 0x20910605, "U1", 1, "",
         "Output rate of the UBX-RXM-SPARTN message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_SPARTN_UART1", 0x20910606, "U1", 1, "",
         "Output rate of the UBX-RXM-SPARTN message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_SPARTN_UART2", 0x20910607, "U1", 1, "",
         "Output rate of the UBX-RXM-SPARTN message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_SPARTN_USB", 0x20910608, "U1", 1, "",
         "Output rate of the UBX-RXM-SPARTN message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_SPARTN_SPI", 0x20910609, "U1", 1, "",
         "Output rate of the UBX-RXM-SPARTN message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_TM_I2C", 0x20910610, "U1", 1, "",
         "Output rate of the UBX-RXM-TM message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_TM_UART1", 0x20910611, "U1", 1, "",
         "Output rate of the UBX-RXM-TM message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_TM_UART2", 0x20910612, "U1", 1, "",
         "Output rate of the UBX-RXM-TM message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_TM_USB", 0x20910613, "U1", 1, "",
         "Output rate of the UBX-RXM-TM message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_TM_SPI", 0x20910614, "U1", 1, "",
         "Output rate of the UBX-RXM-TM message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_MEASD12_I2C", 0x20910639, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASD12 message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_MEASD12_UART1", 0x2091063a, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASD12 message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_MEASD12_UART2", 0x2091063b, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASD12 message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_MEASD12_USB", 0x2091063c, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASD12 message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_MEASD12_SPI", 0x2091063d, "U1", 1, "", 
        "Output rate of the UBX-RXM-MEASD12 message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_MEASC12_I2C", 0x2091063e, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASC12 message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_MEASC12_UART1", 0x2091063f, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASC12 message on port UART1"),

        ("CFG-MSGOUT-UBX_RXM_MEASC12_UART2", 0x20910640, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASC12 message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_MEASC12_USB", 0x20910641, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASC12 message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_MEASC12_SPI", 0x20910642, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASC12 message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_MEAS20_I2C", 0x20910643, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS20 message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_MEAS20_UART1", 0x20910644, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS20 message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_MEAS20_UART2", 0x20910645, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS20 message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_MEAS20_USB", 0x20910646, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS20 message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_MEAS20_SPI", 0x20910647, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS20 message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_MEAS50_I2C", 0x20910648, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS50 message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_MEAS50_UART1", 0x20910649, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS50 message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_MEAS50_UART2", 0x2091064a, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS50 message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_MEAS50_USB", 0x2091064b, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS50 message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_MEAS50_SPI", 0x2091064c, "U1", 1, "",
         "Output rate of the UBX-RXM-MEAS50 message on port SPI"),

        ("CFG-MSGOUT-UBX_RXM_COR_I2C", 0x209106b6, "U1", 1, "",
         "Output rate of the UBX-RXM-COR message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_COR_UART1", 0x209106b7, "U1", 1, "",
         "Output rate of the UBX-RXM-COR message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_COR_UART2", 0x209106b8, "U1", 1, "",
         "Output rate of the UBX-RXM-COR message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_COR_USB", 0x209106b9, "U1", 1, "",
         "Output rate of the UBX-RXM-COR message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_COR_SPI", 0x209106ba, "U1", 1, "",
         "Output rate of the UBX-RXM-COR message on port SPI"),

        # CFG-MSGOUT-UBX_SEC
        ("CFG-MSGOUT-UBX_SEC_SIG_I2C", 0x20910634, "U1", 1, "",
         "Output rate of the UBX-SEC-SIG message on port I2C"),
        ("CFG-MSGOUT-UBX_SEC_SIG_UART1", 0x20910635, "U1", 1, "",
         "Output rate of the UBX-SEC-SIG message on port UART1"),
        ("CFG-MSGOUT-UBX_SEC_SIG_UART2", 0x20910636, "U1", 1, "",
         "Output rate of the UBX-SEC-SIG message on port UART2"),
        ("CFG-MSGOUT-UBX_SEC_SIG_USB", 0x20910637, "U1", 1, "",
         "Output rate of the UBX-SEC-SIG message on port USB"),
        ("CFG-MSGOUT-UBX_SEC_SIG_SPI", 0x20910638, "U1", 1, "",
         "Output rate of the UBX-SEC-SIG message on port SPI"),

        ("CFG-MSGOUT-UBX_SEC_SIGLOG_I2C", 0x20910689, "U1", 1, "",
         "Output rate of the UBX-SEC-SIGLOG message on port I2C"),
        ("CFG-MSGOUT-UBX_SEC_SIGLOG_UART1", 0x2091068a, "U1", 1, "",
         "Output rate of the UBX-SEC-SIGLOG message on port UART1"),
        ("CFG-MSGOUT-UBX_SEC_SIGLOG_UART2", 0x2091068b, "U1", 1, "",
         "Output rate of the UBX-SEC-SIGLOG message on port UART2"),
        ("CFG-MSGOUT-UBX_SEC_SIGLOG_USB", 0x2091068c, "U1", 1, "",
         "Output rate of the UBX-SEC-SIGLOG message on port USB"),
        ("CFG-MSGOUT-UBX_SEC_SIGLOG_SPI", 0x2091068d, "U1", 1, "",
         "Output rate of the UBX-SEC-SIGLOG message on port SPI"),

        ("CFG-MSGOUT-UBX_SEC_OSNMA_I2C", 0x209106ca, "U1", 1, "",
         "Output rate of the UBX-SEC-OSNMA message on port I2C"),
        ("CFG-MSGOUT-UBX_SEC_OSNMA_UART1", 0x209106cb, "U1", 1, "",
         "Output rate of the UBX-SEC-OSNMA message on port UART1"),
        ("CFG-MSGOUT-UBX_SEC_OSNMA_UART2", 0x209106cc, "U1", 1, "",
         "Output rate of the UBX-SEC-OSNMA message on port UART2"),
        ("CFG-MSGOUT-UBX_SEC_OSNMA_USB", 0x209106cd, "U1", 1, "",
         "Output rate of the UBX-SEC-OSNMA message on port USB"),
        ("CFG-MSGOUT-UBX_SEC_OSNMA_SPI", 0x209106ce, "U1", 1, "",
         "Output rate of the UBX-SEC-OSNMA message on port SPI"),

        # CFG-MSGOUT-UBX_TIM
        ("CFG-MSGOUT-UBX_TIM_VRFY_I2C", 0x20910092, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port I2C"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_UART1", 0x20910093, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port UART1"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_UART2", 0x20910094, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port UART2"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_USB", 0x20910095, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port USB"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_SPI", 0x20910096, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port SPI"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_I2C", 0x20910097, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port I2C"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_UART1", 0x20910098, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port UART1"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_UART2", 0x20910099, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port UART2"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_USB", 0x2091009a, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port USB"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_SPI", 0x2091009b, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port SPI"),

        ("CFG-MSGOUT-UBX_TIM_TM2_I2C", 0x20910178, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port I2C"),
        ("CFG-MSGOUT-UBX_TIM_TM2_UART1", 0x20910179, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port UART1"),
        ("CFG-MSGOUT-UBX_TIM_TM2_UART2", 0x2091017a, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port UART2"),
        ("CFG-MSGOUT-UBX_TIM_TM2_USB", 0x2091017b, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port USB"),
        ("CFG-MSGOUT-UBX_TIM_TM2_SPI", 0x2091017c, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port SPI"),
        ("CFG-MSGOUT-UBX_TIM_TP_I2C", 0x2091017d, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port I2C"),
        ("CFG-MSGOUT-UBX_TIM_TP_UART1", 0x2091017e, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port UART1"),
        ("CFG-MSGOUT-UBX_TIM_TP_UART2", 0x2091017f, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port UART2"),

        ("CFG-MSGOUT-UBX_TIM_TP_USB", 0x20910180, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port USB"),
        ("CFG-MSGOUT-UBX_TIM_TP_SPI", 0x20910181, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port SPI"),

        # CFG-NAV2-
        ("CFG-NAV2", 0x1017ffff, "", 0, "",
         "get all CFG-NAV2"),
        ("CFG-NAV2-OUT_ENABLED", 0x10170001, "L", 1, "",
         "Enable NAV2 output"),
        ("CFG-NAV2-SBAS_USE_INTEGRITY", 0x10170002, "L", 1, "",
         "Use SBAS integrity in NAV2 output"),
        ("CFG-NAV2-NAVSPG_ONLY_AUTHDATA", 0x10170003, "L", 1, "",
         "ONly authenticated data in NAV2 output"),

        # CFG-NAVHPG-
        ("CFG-NAVHPG", 0x2014ffff, "", 0, "",
         "get all CFG-NAVHPG"),
        ("CFG-NAVHPG-DGNSSMODE", 0x20140011, "E1", 1, "",
         "Differential corrections mode"),
        # ??
        # ("CFG-NAVHPG-INIFIX3D", 0x20140013, "L", 1, "",
        #  "Initial fix must be 3D"),
        # ("CFG-NAVHPG-FIXMODE", 0x20140011, "E1", 1, "",
        #  "Fix Mode"),
        # ("CFG-NAVHPG-WKNROLLOVER", 0x30140017, "U2", 1, "",
        #  "GPS Week Follover Number"),
        # ("CFG-NAVHPG-UTCSTANDARD", 0x2014001c, "E1", 1, "",
        #  "UTC Standard used"),
        # ("CFG-NAVHPG-DYNMODEL", 0x20140021, "E1", 1, "",
        #  "Dynamic Model"),
        # ("CFG-NAVHPG-ACKAIDING", 0x10140025, "L", 1, "",
        #  "ACK Assist Messages"),
        # ("CFG-NAVHPG-USE_USRDAT", 0x10140061, "L", 1, "",
        #  "Use user datum"),
        # ("CFG-NAVHPG-CORR_CONV_OSR", 0x30140062, "X2", 1, "",
        #  "Convert OSR corrections from one signal to another"),
        # ("CFG-NAVHPG-USRDAT_FLAT", 0x50140063, "R8", 1, "",
        #  "User datam flattening"),
        # ("CFG-NAVHPG-USRDAT_DX", 0x40140064, "R4", 1, "",
        #  "User datam DX"),
        # ("CFG-NAVHPG-USRDAT_DY", 0x40140065, "R4", 1, "",
        #  "User datam DY"),
        # ("CFG-NAVHPG-USRDAT_DZ", 0x40140066, "R4", 1, "",
        #  "User datam DZ"),
        # ??

        # CFG-NAVMASK-
        ("CFG-NAVMASK", 0x1018ffff, "", 0, "",
         "get all CFG-NAVMASK"),
        ("CFG-NAVMASK-EL_MASK_000_020", 0x50180001, "X8", 1, "",
         "Elevation mask for 0 <= Az < 20"),
        ("CFG-NAVMASK-EL_MASK_020_040", 0x50180002, "X8", 1, "",
         "Elevation mask for 20 <= Az < 40"),
        ("CFG-NAVMASK-EL_MASK_040_060", 0x50180003, "X8", 1, "",
         "Elevation mask for 40 <= Az < 60"),
        ("CFG-NAVMASK-EL_MASK_060_080", 0x50180004, "X8", 1, "",
         "Elevation mask for 40 <= Az < 60"),
        ("CFG-NAVMASK-EL_MASK_080_100", 0x50180005, "X8", 1, "",
         "Elevation mask for 80 <= Az < 100"),
        ("CFG-NAVMASK-EL_MASK_100_120", 0x50180006, "X8", 1, "",
         "Elevation mask for 100 <= Az < 120"),
        ("CFG-NAVMASK-EL_MASK_120_140", 0x50180007, "X8", 1, "",
         "Elevation mask for 120 <= Az < 140"),
        ("CFG-NAVMASK-EL_MASK_140_160", 0x50180008, "X8", 1, "",
         "Elevation mask for 140 <= Az < 160"),
        ("CFG-NAVMASK-EL_MASK_160_180", 0x50180009, "X8", 1, "",
         "Elevation mask for 160 <= Az < 180"),
        ("CFG-NAVMASK-EL_MASK_180_200", 0x5018000a, "X8", 1, "",
         "Elevation mask for 180 <= Az < 200"),
        ("CFG-NAVMASK-EL_MASK_200_220", 0x5018000b, "X8", 1, "",
         "Elevation mask for 200 <= Az < 220"),
        ("CFG-NAVMASK-EL_MASK_220_240", 0x5018000c, "X8", 1, "",
         "Elevation mask for 220 <= Az < 240"),
        ("CFG-NAVMASK-EL_MASK_240_260", 0x5018000d, "X8", 1, "",
         "Elevation mask for 240 <= Az < 260"),
        ("CFG-NAVMASK-EL_MASK_260_280", 0x5018000e, "X8", 1, "",
         "Elevation mask for 260 <= Az < 280"),
        ("CFG-NAVMASK-EL_MASK_280_300", 0x5018000f, "X8", 1, "",
         "Elevation mask for 280 <= Az < 300"),
        ("CFG-NAVMASK-EL_MASK_300_320", 0x50180010, "X8", 1, "",
         "Elevation mask for 300 <= Az < 320"),
        ("CFG-NAVMASK-EL_MASK_320_340", 0x50180011, "X8", 1, "",
         "Elevation mask for 320 <= Az < 340"),
        ("CFG-NAVMASK-EL_MASK_340_360", 0x50180012, "X8", 1, "",
         "Elevation mask for 340 <= Az < 660"),
        ("CFG-NAVMASK-SV_MASK_GPS", 0x50180013, "X8", 1, "",
         "Sat mask for GPS"),
        ("CFG-NAVMASK-SV_MASK_GAL", 0x50180014, "X8", 1, "",
         "Sat mask for Galileo"),
        ("CFG-NAVMASK-SV_MASK_GLO", 0x50180015, "X8", 1, "",
         "Sat mask for GLONASS"),
        ("CFG-NAVMASK-SV_MASK_BDS", 0x50180016, "X8", 1, "",
         "Sat mask for BeiDou"),
        ("CFG-NAVMASK-SV_MASK_QZSS", 0x50180017, "X8", 1, "",
         "Sat mask for QZSS"),
        ("CFG-NAVMASK-SV_MASK_NAVIC", 0x50180018, "X8", 1, "",
         "Sat mask for NaVIC"),

        # CFG-NAVSPG-
        ("CFG-NAVSPG", 0x2011ffff, "", 0, "",
         "get all CFG-NAVSPG"),
        ("CFG-NAVSPG-FIXMODE", 0x20110011, "E1", 1, "",
         "Position fix mode"),
        ("CFG-NAVSPG-INIFIX3D", 0x10110013, "L", 1, "",
         "Initial fix must be a 3d fix"),
        ("CFG-NAVSPG-WKNROLLOVER", 0x30110017, "U2", 1, "",
         "GPS week rollover number"),
        ("CFG-NAVSPG-USE_PPP", 0x10110019, "L", 1, "",
         "Use Precise Point Positioning"),
        ("CFG-NAVSPG-UTCSTANDARD", 0x2011001c, "E1", 1, "",
         "UTC standard to be used"),
        ("CFG-NAVSPG-DYNMODEL", 0x20110021, "E1", 1, "",
         "Dynamic platform model"),
        ("CFG-NAVSPG-ACKAIDING", 0x10110025, "L", 1, "",
         "Acknowledge assistance input messages"),
        ("CFG-NAVSPG-USE_USRDAT", 0x10110061, "L", 1, "",
         "Use user geodetic datum"),
        ("CFG-NAVSPG-USRDAT_MAJA", 0x50110062, "R8", 1, "m",
         "Geodetic datum semi-major axis"),
        ("CFG-NAVSPG-USRDAT_FLAT", 0x50110063, "R8", 1, "",
         "Geodetic datum 1.0 / flattening"),
        ("CFG-NAVSPG-USRDAT_DX", 0x40110064, "R4", 1, "m",
         "Geodetic datum X axis shift at the origin"),
        ("CFG-NAVSPG-USRDAT_DY", 0x40110065, "R4", 1, "m",
         "Geodetic datum Y axis shift at the origin"),
        ("CFG-NAVSPG-USRDAT_DZ", 0x40110066, "R4", 1, "m",
         "Geodetic datum Z axis shift at the origin"),
        ("CFG-NAVSPG-USRDAT_ROTX", 0x40110067, "R4", 1, "arcsec",
         "Geodetic datum rotation about the X axis"),
        ("CFG-NAVSPG-USRDAT_ROTY", 0x40110068, "R4", 1, "arcsec",
         "Geodetic datum rotation about the Y axis ()"),
        ("CFG-NAVSPG-USRDAT_ROTZ", 0x40110069, "R4", 1, "arcsec",
         "Geodetic datum rotation about the Z axis"),
        ("CFG-NAVSPG-USRDAT_SCALE", 0x4011006a, "R4", 1, "ppm",
         "Geodetic datum scale factor"),
        ("CFG-NAVSPG-INFIL_MINSVS", 0x201100a1, "U1", 1, "",
         "Minimum number of satellites for navigation"),
        ("CFG-NAVSPG-INFIL_MAXSVS", 0x201100a2, "U1", 1, "",
         "Maximum number of satellites for navigation"),
        ("CFG-NAVSPG-INFIL_MINCNO", 0x201100a3, "U1", 1, "dBHz",
         "Minimum satellite signal level for navigation"),
        ("CFG-NAVSPG-INFIL_MINELEV", 0x201100a4, "I1", 1, "deg",
         "Minimum elevation for a GNSS satellite to be used in navigation"),
        ("CFG-NAVSPG-INFIL_NCNOTHRS", 0x201100aa, "U1", 1, "",
         "Number of satellites required to have C/N0 above "
         "CFG-NAVSPG-INFIL_CNOTHRS for a fix to be attempted"),
        ("CFG-NAVSPG-INFIL_CNOTHRS", 0x201100ab, "U1", 1, "",
         "C/N0 threshold for deciding whether to attempt a fix"),
        ("CFG-NAVSPG-OUTFIL_PDOP", 0x301100b1, "U2", 0.1, "",
         "Output filter position DOP mask (threshold)"),
        ("CFG-NAVSPG-OUTFIL_TDOP", 0x301100b2, "U2", 0.11, "",
         "Output filter time DOP mask (threshold)"),
        ("CFG-NAVSPG-OUTFIL_PACC", 0x301100b3, "U2", 1, "m",
         "Output filter position accuracy mask (threshold)"),
        ("CFG-NAVSPG-OUTFIL_TACC", 0x301100b4, "U2", 1, "m",
         "Output filter time accuracy mask (threshold)"),
        ("CFG-NAVSPG-OUTFIL_FACC", 0x301100b5, "U2", 0.01, "m/s",
         "Output filter frequency accuracy mask (threshold)"),

        ("CFG-NAVSPG-CONSTR_ALT", 0x401100c1, "I4", 0.01, "m",
         "Fixed altitude (mean sea level) for 2D fix mode"),
        ("CFG-NAVSPG-CONSTR_ALTVAR", 0x401100c2, "U4", 0.0001, "M^2",
         "Fixed altitude variance for 2D mode"),
        ("CFG-NAVSPG-CONSTR_PROPLIMIT", 0x401100c3, "U4", 1, "",
         "Limit for propagating the state after NOFIX"),
        ("CFG-NAVSPG-CONSTR_DGNSSTO", 0x201100c4, "U1", 1, "s",
         "DGNSS timeout"),
        ("CFG-NAVSPG-CONSTR_DGNSSTO_SCALE", 0x201100c5, "U1", 1, "s",
         "DGNSS timeout scale"),
        ("CFG-NAVSPG-SIGATTCOMP", 0x201100d6, "E1", 1, "",
         "Permanently attenuated signal compensation mode"),
        ("CFG-NAVSPG-PL_ENA", 0x101100d7, "L", 1, "",
         "Protection level, if enabled"),
        ("CFG-NAVSPG-ONLY_AUTHDATA", 0x101100dd, "L", 1, "",
         "Protection level, if enabled"),
        ("CFG-NAVSPG-MAX_TIMETRUSTED_ACC", 0x301100de, "U2", 1, "",
         "Maximum trusted time accuracy"),

        # CFG-NMEA-
        ("CFG-NMEA", 0x2093ffff, "", 0, "",
         "get all CFG-NMEA"),
        ("CFG-NMEA-PROTVER", 0x20930001, "E1", 1, "",
         "NMEA protocol version"),
        ("CFG-NMEA-MAXSVS", 0x20930002, "E1", 1, "",
         "Maximum number of SVs to report per Talker ID"),
        ("CFG-NMEA-COMPAT", 0x10930003, "L", 1, "",
         "Enable compatibility mode"),
        ("CFG-NMEA-CONSIDER", 0x10930004, "L", 1, "",
         "Enable considering mode"),
        ("CFG-NMEA-LIMIT82", 0x10930005, "L", 1, "",
         "Enable strict limit to 82 characters maximum NMEA message length"),
        ("CFG-NMEA-HIGHPREC", 0x10930006, "L", 1, "",
         "Enable high precision mode"),
        ("CFG-NMEA-SVNUMBERING", 0x20930007, "E1", 1, "",
         "Display configuration for SVs that have no value defined in NMEA"),
        ("CFG-NMEA-FILT_GPS", 0x10930011, "L", 1, "",
         "Disable reporting of GPS satellites"),
        ("CFG-NMEA-FILT_SBAS", 0x10930012, "L", 1, "",
         "Disable reporting of SBAS satellites"),
        ("CFG-NMEA-FILT_GAL", 0x10930013, "L", 1, "",
         "Disable reporting of GALILEO satellites"),
        ("CFG-NMEA-FILT_QZSS", 0x10930015, "L", 1, "",
         "Disable reporting of QZSS satellites"),
        ("CFG-NMEA-FILT_GLO", 0x10930016, "L", 1, "",
         "Disable reporting of GLONASS satellites"),
        ("CFG-NMEA-FILT_BDS", 0x10930017, "L", 1, "",
         "Disable reporting of BeiDou satellites"),
        ("CFG-NMEA-FILT_NAVIC", 0x10930018, "L", 1, "",
         "Disable reporting of NavIC satellites"),
        ("CFG-NMEA-OUT_INVFIX", 0x10930021, "L", 1, "",
         "Enable position output for failed or invalid fixes"),
        ("CFG-NMEA-OUT_MSKFIX", 0x10930022, "L", 1, "",
         "Enable position output for invalid fixes"),
        ("CFG-NMEA-OUT_INVTIME", 0x10930023, "L", 1, "",
         "Enable time output for invalid times"),
        ("CFG-NMEA-OUT_INVDATE", 0x10930024, "L", 1, "",
         "Enable date output for invalid dates"),
        ("CFG-NMEA-OUT_ONLYGPS", 0x10930025, "L", 1, "",
         "Restrict output to GPS satellites only"),
        ("CFG-NMEA-OUT_FROZENCOG", 0x10930026, "L", 1, "",
         "Enable course over ground output even if it is frozen"),
        ("CFG-NMEA-MAINTALKERID", 0x20930031, "E1", 1, "",
         "Main Talker ID"),
        ("CFG-NMEA-GSVTALKERID", 0x20930032, "E1", 1, "",
         "Talker ID for GSV NMEA messages"),
        ("CFG-NMEA-BDSTALKERID", 0x30930033, "U2", 1, "",
         "BeiDou Talker ID"),

        # CFG-ODO-
        ("CFG-ODO", 0x1022ffff, "", 0, "",
         "get all CFG-ODO"),
        ("CFG-ODO-USE_ODO", 0x10220001, "L", 1, "",
         "Use odometer"),
        ("CFG-ODO-USE_COG", 0x10220002, "L", 1, "",
         "Use low-speed course over ground filter"),
        ("CFG-ODO-OUTLPVEL", 0x10220003, "L", 1, "",
         "Output low-pass filtered velocity"),
        ("CFG-ODO-OUTLPCOG", 0x10220004, "L", 1, "",
         "Output low-pass filtered course over ground (heading)"),
        ("CFG-ODO-PROFILE", 0x20220005, "E1", 1, "",
         "Odometer profile configuration"),
        ("CFG-ODO-COGMAXSPEED", 0x20220021, "U1", 1, "m/s",
         "Upper speed limit for low-speed course over ground filter"),
        ("CFG-ODO-COGMAXPOSACC", 0x20220022, "U1", 1, "",
         "Maximum acceptable position accuracy for computing low-speed  "
         "filtered course over ground"),
        ("CFG-ODO-VELLPGAIN", 0x20220031, "U1", 1, "",
         "Velocity low-pass filter level"),
        ("CFG-ODO-COGLPGAIN", 0x20220032, "U1", 1, "",
         "Course over ground low-pass filter level (at speed < 8 m/s)"),

        # CFG-PM-
        ("CFG-PM", 0x20d0ffff, "", 0, "",
         "get all CFG-PM, receiver power management"),
        ("CFG-PM-OPERATEMODE", 0x20d00001, "E1", 1, "",
         "set PSMOO or PSMCT mode"),
        ("CFG-PM-POSUPDATEPERIOD", 0x40d00002, "U4", 1, "",
         "Position update period for PSMOO."),
        ("CFG-PM-ACQPERIOD", 0x40d00003, "U4", 1, "s",
         "Acquisition period if receiver fails to achieve a position fix"),
        ("CFG-PM-GRIDOFFSET", 0x40d00004, "U4", 1, "s",
         "Position update period grid offset relative to GPS start of week."),
        ("CFG-PM-ONTIME", 0x30d00005, "U2", 1, "s",
         "Time to stay in Tracking state."),
        ("CFG-PM-MINACQTIME", 0x20d00006, "U1", 1, "s",
         "Minimum time to spend in Acquisition state"),
        ("CFG-PM-MAXACQTIME", 0x20d00007, "U1", 1, "s",
         "Maximum time to spend in Acquisition state"),
        ("CFG-PM-DONOTENTEROFF", 0x10d00008, "L", 1, "",
         "Behavior when failure to achieve a position during update period."),
        ("CFG-PM-WAITTIMEFIX", 0x10d00009, "L", 1, "",
         "wait for time fix"),
        ("CFG-PM-UPDATEEPH", 0x10d0000a, "L", 1, "",
         "Update ephemeris regularly."),
        ("CFG-PM-EXTINTSEL", 0x20d0000b, "E1", 1, "",
         "EXTINT pin select."),
        ("CFG-PM-EXTINTWAKE", 0x10d0000c, "L", 1, "",
         "EXTINT pin control (Wake)"),
        ("CFG-PM-EXTINTBACKUP", 0x10d0000d, "L", 1, "",
         "EXTINT pin control (Backup)"),
        ("CFG-PM-EXTINTINACTIVE", 0x10d0000e, "L", 1, "",
         "EXTINT pin control (Inactive)"),
        ("CFG-PM-UPDATEEPH", 0x10d0000d, "U4", 0.001, "s",
         "Inactivity time out on EXTINT pin if enabled"),
        ("CFG-PM-EXTINTINACTIVITY", 0x40d0000f, "U4", 1, "s",
         "Inactivity time out on EXTINT pin if enabled"),
        ("CFG-PM-LIMITPEAKCURR", 0x10d00010, "L", 1, "",
         "Limit peak current"),

        # CFG-PMP--
        ("CFG-PMP", 0x10b1FFFF, "", 0, "",
         "get all CFG-PMP"),

        # CFG-QZSS-
        ("CFG-QZSS", 0x3037ffff, "", 0, "s",
         "get all CFG-QZSS"),
        ("CFG-QZSS-USE_SLAS_DGNSS", 0x10370005, "L", 0.001, "",
         "Apply QZSS SLAS DGNSS corrections"),
        ("CFG-QZSS-USE_SLAS_TESTMODE", 0x10370006, "L", 0.001, "",
         "Use QZSS SLAS data when it is in test mode"),
        ("CFG-QZSS-USE_SLAS_RAIM_UNCORR", 0x10370007, "L", 0.001, "",
         "Raim out measurements that are not corrected by QZSS SLAS"),
        ("CFG-QZSS-SLAS_MAX_BASELINE", 0x30370008, "U2", 1, "km",
         "Max baseline to closest GMS"),
        ("CFG-QZSS-L6_SVIDA", 0x20370020, "I1", 1, "",
         "QZSS L6 SV Id to be decoded by channel A"),
        ("CFG-QZSS-L6_SVIDB", 0x20370030, "I1", 1, "",
         "QZSS L6 SV Id to be decoded by channel B"),
        ("CFG-QZSS-L6_MSGA", 0x20370050, "E1", 1, "",
         "QZSS L6 messages to be decoded by channel A"),
        ("CFG-QZSS-L6_MSGB", 0x20370060, "E1", 1, "",
         "QZSS L6 messages to be decoded by channel B"),
        ("CFG-QZSS-L6_RSDECODER", 0x20370080, "E1", 1, "",
         "QZSS L6 message Reed-Solomon decoder mode"),

        # CFG-RATE-
        ("CFG-RATE", 0x3021ffff, "", 0, "s",
         "get all CFG-RATE"),
        ("CFG-RATE-MEAS", 0x30210001, "U2", 0.001, "s",
         "Nominal time between GNSS measurements"),
        ("CFG-RATE-NAV", 0x30210002, "U2", 1, "",
         "Ratio of number of measurements to number of navigation solutions"),
        ("CFG-RATE-TIMEREF", 0x20210003, "E1", 1, "",
         "Time system to which measurements are aligned"),
        ("CFG-RATE-NAV_PRIO", 0x20210004, "U1", 1, "",
         "Time system to whichpirority nav measurements are output"),

        # CFG-RINV-
        ("CFG-RINV", 0x10c7ffff, "", 0, "",
         "get all CFG-RINV"),
        ("CFG-RINV-DUMP", 0x10c70001, "L", 1, "",
         "Dump data at startup"),
        ("CFG-RINV-BINARY", 0x10c70002, "L", 1, "",
         "Data is binary"),
        ("CFG-RINV-DATA_SIZE", 0x20c70003, "U1", 1, "",
         "Size of data"),
        ("CFG-RINV-CHUNK0", 0x50c70004, "X8", 1, "",
         "Data bytes 1-8 (LSB)"),
        ("CFG-RINV-CHUNK1", 0x50c70005, "X8", 1, "",
         "Data bytes 9-16"),
        ("CFG-RINV-CHUNK2", 0x50c70006, "X8", 1, "",
         "Data bytes 17-24"),
        ("CFG-RINV-CHUNK3", 0x50c70007, "X8", 1, "",
         "Data bytes 25-30 (MSB)"),

        # CFG-RTCM-
        ("CFG-RTCM", 0x1009ffff, "", 0, "",
         "get all CFG-RTCM"),
        ("CFG-RTCM-DF003_OUT", 0x30090001, "U2", 1, "",
         "RTCM DF003 reference station ID (output)"),
        ("CFG-RTCM-DF003_IN", 0x30090008, "U2", 1, "",
         "RTCM DF003 reference station ID (input)"),
        ("CFG-RTCM-DF003_IN_FILTER", 0x20090009, "E1", 1, "",
         "RTCM input filter configuration based on DF003 value"),

        # CFG-SBAS-
        ("CFG-SBAS", 0x1036ffff, "", 0, "",
         "get all CFG-SBAS"),
        ("CFG-SBAS-USE_TESTMODE", 0x10360002, "L", 1, "",
         "Use SBAS data when it is in test mode"),
        ("CFG-SBAS-USE_RANGING", 0x10360003, "L", 1, "",
         "Use SBAS GEOs as a ranging source (for navigation)"),
        ("CFG-SBAS-USE_DIFFCORR", 0x10360004, "L", 1, "",
         "Use SBAS differential corrections"),
        ("CFG-SBAS-USE_INTEGRITY", 0x10360005, "L", 1, "",
         "Use SBAS integrity information"),
        ("CFG-SBAS-PRNSCANMASK", 0x50360006, "X8", 1, "",
         "SBAS PRN search configuration"),
        ("CFG-SBAS-USE_IONOONLY", 0x10360007, "L", 1, "",
         "Use only SBAS ionosphere correctiopn"),
        ("CFG-SBAS-ACCEPT_NOT_IN_PRNMASK", 0x30360008, "X2", 1, "",
         "Accept SBAS not in PRN mask"),

        # CFG-SEC-
        # M10S, protVer 34.00
        # F9T, protVer 29.26
        ("CFG-SEC", 0x10f6ffff, "", 0, "",
         "get all CFG-SEC"),
        ("CFG-SEC-CFG_LOCK", 0x10f60009, "L", 1, "",
         "Configuration lockdown"),
        ("CFG-SEC-CFG_LOCK_UNLOCKGRP1", 0x30f6000a, "U2", 1, "",
         "Configuration lockdown exempted group 1"),
        ("CFG-SEC-CFG_LOCK_UNLOCKGRP2", 0x30f6000b, "U2", 1, "",
         "Configuration lockdown exempted group 2"),
        ("CFG-SEC-JAMDET_SENSITIVITY_HI", 0x10f60051, "L", 1, "",
         "HIgh sensitivity jam detection"),
        ("CFG-SEC-SPOOFDET_SIM_SIG_DIS", 0x10f6005d, "L", 1, "",
         "Disable spoof detection"),

        # CFG-SFCORE-
        # F9 DR products, protver 33.20
        ("CFG-SFCORE", 0x1008ffff, "", 0, "",
         "get all CFG-SFCORE"),
        ("CFG-SFCORE-USE_SF", 0x10080001, "L", 1, "",
         "Use ADR/UDR sensor fusion"),
        ("CFG-SFCORE-IMU2CRP_LA_X", 0x30080002, "I2", 1, "cm",
         "X coordinate of IMU-to-CRP "),
        ("CFG-SFCORE-IMU2CRP_LA_Y", 0x30080003, "I2", 1, "cm",
         "Y coordinate of IMU-to-CRP "),
        ("CFG-SFCORE-IMU2CRP_LA_Z", 0x30080004, "I2", 1, "cm",
         "Z coordinate of IMU-to-CRP "),
        ("CFG-SFCORE-HNR_RATE", 0x2008001a, "U1", 1, "",
         "Rate of navigation solution output"),

        # CFG-SFIMU-
        # F9 DR products, protver 33.20
        ("CFG-SFIMU", 0x1006ffff, "", 0, "",
         "get all CFG-SFIMU"),
        ("CFG-SFIMU-GYRO_TC_UPDATE_PERIOD", 0x30060007, "U2", 1, "s",
         "Update period for gyro bias table"),
        ("CFG-SFIMU-GYRO_RMSTHDL", 0x20060008, "U1", 2e-8, "deg/s",
         "Gyro sensor RMS threshold"),
        ("CFG-SFIMU-GYRO_FREQUENCY", 0x20060009, "U1", 1, "Hz",
         "Gyro sampling grequency"),
        ("CFG-SFIMU-GYRO_LATENCY", 0x3006000a, "U2", 1, "ms",
         "Gyro latency"),
        ("CFG-SFIMU-GYRO_ACCURACY", 0x3006000b, "U2", 1e-3, "deg/s",
         "Gyro Accuracy"),
        ("CFG-SFIMU-ACCEL_RMSTHDL", 0x20060015, "U1", 2e-6, "m/s^2",
         "Accel sensor RMS threshold"),
        ("CFG-SFIMU-ACCEL_FREQUENCY", 0x20060016, "U1", 1, "Hz",
         "Accel sampling grequency"),
        ("CFG-SFIMU-ACCEL_LATENCY", 0x30060017, "U2", 1, "ms",
         "Accel latency"),
        ("CFG-SFIMU-ACCEL_ACCURACY", 0x30060018, "U2", 1e-4, "m/s^2",
         "Accel Accuracy"),
        ("CFG-SFIMU-IMU_EN", 0x1006001d, "L", 1, "", "IMU enabled"),
        ("CFG-SFIMU-IMU_I2C_SCL_PIO", 0x2006001e, "U1", 1, "",
         "SCL PIO of IMU"),
        ("CFG-SFIMU-IMU_I2C_SDA_PIO", 0x2006001f, "U1", 1, "",
         "SCL PIO of IMU"),
        ("CFG-SFIMU-IMU2ANT_LA_X", 0x30060020, "I2", 1, "cm",
         "X of  IMU-to-ANT"),
        ("CFG-SFIMU-IMU2ANT_LA_Y", 0x30060021, "I2", 1, "cm",
         "Y of  IMU-to-ANT"),
        ("CFG-SFIMU-IMU2ANT_LA_Z", 0x30060022, "I2", 1, "cm",
         "Z of  IMU-to-ANT"),
        ("CFG-SFIMU-AUTO_MNTALG_ENA", 0x10060027, "L", 1, "",
         "Auto IMU algnment enable"),
        ("CFG-SFIMU-IMU_MNTALG_YAW", 0x4006002d, "U4", 1e-2, "deg",
         "IMU yaw angle"),
        ("CFG-SFIMU-IMU_MNTALG_PITCH", 0x3006002e, "I2", 1e-2, "deg",
         "IMU pitch angle"),
        ("CFG-SFIMU-IMU_MNTALG_ROLL", 0x3006002f, "I2", 1e-2, "deg",
         "IMU roll angle"),
        ("CFG-SFIMU-IMU_MNTALG_TOLERANCE", 0x20060030, "E1", 1, "",
         "User-defined IMU mount alignment angles tolerance level"),

        # CFG-SFODO-
        # F9 DR products, protver 33.20
        ("CFG-SFODO", 0x1007ffff, "", 0, "",
         "get all CFG-SFODO"),
        ("CFG-SFODO-COMBINE_TICKS", 0x10070001, "L", 1, "",
         "Use combined rear WT instead of the single tick."),
        ("CFG-SFODO-USE_SPEED", 0x10070003, "L", 1, "",
         "Use speed measurements"),
        ("CFG-SFODO-DIS_AUTOCOUNTMAX", 0x10070004, "L", 1, "",
         "Disable automatic estimation of WT ctr"),
        ("CFG-SFODO-DIS_AUTODIRPINPOL", 0x10070005, "L", 1, "",
         "Disable automatic WT direction pin polarity detection"),
        ("CFG-SFODO-DIS_AUTOSPEED", 0x10070006, "L", 1, "",
         "Disable automatic receiver reconfig speed instead WT data"),
        ("CFG-SFODO-FACTOR", 0x40070007, "U4", 1e-6, "",
         "WT scale factor"),
        ("CFG-SFODO-QUANT_ERROR", 0x40070008, "U4", 1e-6, "",
         "WT quantization"),
        ("CFG-SFODO-COUNT_MAX", 0x40070009, "U4", 1, "",
         "WT ctr max value"),
        ("CFG-SFODO-LATENCY", 0x3007000a, "U2", 1, "ms",
         "WT data latency"),
        ("CFG-SFODO-FREQUENCY", 0x2007000b, "U1", 1, "Hz",
         "nominal WT data freq (0=not set)"),
        ("CFG-SFODO-CNT_BOTH_EDGES", 0x1007000d, "L", 1, "",
         "Count both rising and falling edges on WT signal"),
        ("CFG-SFODO-SPEED_BAND", 0x3007000e, "U2", 1, "cm/s",
         "Speed sensor dead band (0 = not set)"),
        ("CFG-SFODO-USE_WT_PIN", 0x1007000f, "L", 1, "",
         "Flag indicating that WT signal is connected."),
        ("CFG-SFODO-DIR_PINPOL", 0x10070010, "L", 1, "",
         "WT direction pin polarity"),
        ("CFG-SFODO-DIS_AUTOSW", 0x10070011, "L", 1, "",
         "Disa auto use of WT or speed data received over SW interface"),
        ("CFG-SFODO-IMU2VRP_LA_X", 0x30070012, "I2", 1, "cm",
         "X coordinate of IMU-to-VRP lever-arm in the installation frame"),
        ("CFG-SFODO-IMU2VRP_LA_Y", 0x30070013, "I2", 1, "cm",
         "Y coordinate of IMU-to-VRP lever-arm in the installation frame"),
        ("CFG-SFODO-IMU2VRP_LA_Z", 0x30070014, "I2", 1, "cm",
         "X coordinate of IMU-to-VRP lever-arm in the installation frame"),
        ("CFG-SFODO-DIS_DIR_INFO", 0x1007001c, "L", 1, "",
         "Do not use directional information"),

        # CFG-PMP-
        ("CFG-PMP", 0x100bffff, "", 0, "",
         "get all CFG-PMP"),
        ("CFG-PMP-CENTER_FREQUENCY", 0x40b10011, "U4", 1, "",
         "Center frequency"),
        ("CFG-PMP-SEARCH_WINDOW", 0x30b10012, "U2", 1, "",
         "Search window"),
        ("CFG-PMP-DATA_RATE", 0x30b10013, "E2", 1, "",
         "Data rate"),
        ("CFG-PMP-USE_DESCRAMBLER", 0x10b10014, "L", 1, "",
         "Use descrambler"),
        ("CFG-PMP-DESCRAMBLER_INIT", 0x30b10015, "U2", 1, "",
         "Descrambler initialization"),
        ("CFG-PMP-USE_SERVICE_ID", 0x10b10016, "L", 1, "",
         "Use service ID"),
        ("CFG-PMP-SERVICE_ID", 0x30b10017, "U2", 1, "",
         "Service identifier"),
        ("CFG-PMP-USE_PRESCRAMBLING", 0x10b10019, "L", 1, "",
         "Use prescrambling"),
        ("CFG-PMP-UNIQUE_WORD", 0x50b1001a, "U8", 1, "",
         "Unique word"),

        # CFG-SIGNAL-
        # sort by group and id
        ("CFG-SIGNAL", 0x1031ffff, "", 0, "",
         "get all CFG-SIGNAL"),
        ("CFG-SIGNAL-GPS_L1CA_ENA", 0x10310001, "L", 1, "",
         "GPS L1C/5 enable"),
        ("CFG-SIGNAL-GPS_L2C_ENA", 0x10310003, "L", 1, "",
         "GPS L25 enable"),
        ("CFG-SIGNAL-GPS_L5_ENA", 0x10310004, "L", 1, "",
         "GPS L5 enable"),
        ("CFG-SIGNAL-SBAS_L1CA_ENA", 0x10310005, "L", 1, "",
         "SBAS L1C/A enable"),
        ("CFG-SIGNAL-GAL_E1_ENA", 0x10310007, "L", 1, "",
         "GAL E1 enable"),
        ("CFG-SIGNAL-GAL_E5A_ENA", 0x10310009, "L", 1, "",
         "GAL E5A enable"),
        ("CFG-SIGNAL-GAL_E5B_ENA", 0x1031000a, "L", 1, "",
         "GAL E5b enable"),
        ("CFG-SIGNAL-GAL_E6_ENA", 0x1031000b, "L", 1, "",
         "GAL E6 enable"),
        ("CFG-SIGNAL-BDS_B1_ENA", 0x1031000d, "L", 1, "",
         "BeiDou B1I enable"),
        ("CFG-SIGNAL-BDS_B2_ENA", 0x1031000e, "L", 1, "",
         "BeiDou B2I enable"),
        ("CFG-SIGNAL-BDS_B1C_ENA", 0x1031000f, "L", 1, "",
         "BeiDou B1C enable"),
        ("CFG-SIGNAL-BDS_B3_ENA", 0x10310010, "L", 1, "",
         "BDS B3 enable"),
        ("CFG-SIGNAL-QZSS_L1CA_ENA", 0x10310012, "L", 1, "",
         "QZSS L1C/A enable"),
        ("CFG-SIGNAL-QZSS_L1S_ENA", 0x10310014, "L", 1, "",
         "QZSS L1S enable"),
        ("CFG-SIGNAL-QZSS_L2C_ENA", 0x10310015, "L", 1, "",
         "QZSS L2C enable"),
        ("CFG-SIGNAL-QZSS_L5_ENA", 0x10310017, "L", 1, "",
         "QZSS_L5 enable"),
        ("CFG-SIGNAL-GLO_L1_ENA", 0x10310018, "L", 1, "",
         "GLONASS L1 enable"),
        ("CFG-SIGNAL-GLO_L2_ENA", 0x1031001a, "L", 1, "",
         "GLONASS L2 enable"),
        ("CFG-SIGNAL-NAVIC_L5_ENA", 0x1031001d, "L", 1, "",
         "NavIC L5 enable"),
        ("CFG-SIGNAL-GPS_ENA", 0x1031001f, "L", 1, "",
         "GPS enable"),
        ("CFG-SIGNAL-SBAS_ENA", 0x10310020, "L", 1, "",
         "SBAS enable"),
        ("CFG-SIGNAL-GAL_ENA", 0x10310021, "L", 1, "",
         "Galileo enable"),
        ("CFG-SIGNAL-BDS_ENA", 0x10310022, "L", 1, "",
         "BeiDou Enable"),
        ("CFG-SIGNAL-QZSS_ENA", 0x10310024, "L", 1, "",
         "QZSS enable"),
        ("CFG-SIGNAL-GLO_ENA", 0x10310025, "L", 1, "",
         "GLONASS enable"),
        ("CFG-SIGNAL-NAVIC_ENA", 0x10310026, "L", 1, "",
         "NavIC enable"),
        ("CFG-SIGNAL-BDS_B2A_ENA", 0x10310028, "L", 1, "",
         "BDS B2A enable"),
        ("CFG-SIGNAL-IGNORE_BAND_OTP", 0x10310029, "L", 1, "",
         "Enable all bands disregarding OTP band file. "
         "For production  testing."),
        ("CFG-SIGNAL-QZSS_L1CB_ENA", 0x10310039, "L", 1, "",
         "QZSS L1C/B"),
        ("CFG-SIGNAL-PLAN", 0x2031003a, "E1", 1, "",
         "SIGNAL PLAN"),

        # CFG-SPARTN-
        ("CFG-SPARTN", 0x10a7ffff, "", 0, "",
         "get all CFG-SPARTN"),
        ("CFG-SPARTN-USE_SOURCE", 0x20a70001, "E1", 1, "",
         "Selector for source SPARTN stream"),

        # CFG-SPI-
        ("CFG-SPI", 0x1064ffff, "", 0, "",
         "get all CFG-SPI"),
        ("CFG-SPI-MAXFF", 0x20640001, "U1", 1, "",
         "Number of bytes containing 0xFF to receive before "
         "switching off reception."),
        ("CFG-SPI-CPOLARITY", 0x10640002, "L", 1, "",
         "Clock polarity select"),
        ("CFG-SPI-CPHASE", 0x10640003, "L", 1, "",
         "Clock phase select"),
        ("CFG-SPI-EXTENDEDTIMEOUT", 0x10640005, "L", 1, "",
         "Flag to disable timeouting the interface after 1.5s"),
        ("CFG-SPI-ENABLED", 0x10640006, "L", 1, "",
         "Flag to indicate if the SPI interface should be enabled"),

        # CFG-SPIINPROT-
        ("CFG-SPIINPROT", 0x1079ffff, "", 0, "",
         "get all CFG-SPIINPROT"),
        ("CFG-SPIINPROT-UBX", 0x10790001, "L", 1, "",
         "Flag to indicate if UBX should be an input protocol on SPI"),
        ("CFG-SPIINPROT-NMEA", 0x10790002, "L", 1, "",
         "Flag to indicate if NMEA should be an input protocol on SPI"),
        ("CFG-SPIINPROT-RTCM2X", 0x10790003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input protocol on SPI"),
        ("CFG-SPIINPROT-RTCM3X", 0x10790004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an input protocol on SPI"),
        ("CFG-SPIINPROT-SPARTN", 0x10790005, "L", 1, "",
         "Flag to indicate if SPARTN should be an input protocol on SPI"),

        # CFG-SPIOUTPROT-
        ("CFG-SPIOUTPROT", 0x107affff, "", 0, "",
         "get all CFG-SPIOUTPROT"),
        ("CFG-SPIOUTPROT-UBX", 0x107a0001, "L", 1, "",
         "Flag to indicate if UBX should be an output protocol on SPI"),
        ("CFG-SPIOUTPROT-NMEA", 0x107a0002, "L", 1, "",
         "Flag to indicate if NMEA should be an output protocol on SPI"),
        ("CFG-SPIOUTPROT-RTCM3X", 0x107a0004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output protocol on SPI"),

        # CFG-TMODE-
        ("CFG-TMODE", 0x2003ffff, "", 0, "",
         "get all CFG-TMODE"),
        ("CFG-TMODE-MODE", 0x20030001, "E1", 1, "",
         "Receiver mode"),
        ("CFG-TMODE-POS_TYPE", 0x20030002, "E1", 1, "",
         "Determines whether the ARP position is given in ECEF or "
         "LAT/LON/HEIGHT?"),
        ("CFG-TMODE-ECEF_X", 0x40030003, "I4", 1, "cm",
         "ECEF X coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_Y", 0x40030004, "I4", 1, "cm",
         "ECEF Y coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_Z", 0x40030005, "I4", 1, "cm",
         "ECEF Z coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_X_HP", 0x20030006, "I1", 0.1, "mm",
         "High-precision ECEF X coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_Y_HP", 0x20030007, "I1", 0.1, "mm",
         "High-precision ECEF Y coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_Z_HP", 0x20030008, "I1", 0.1, "mm",
         "High-precision ECEF Z coordinate of the ARP position."),
        ("CFG-TMODE-LAT", 0x40030009, "I4", 1e-7, "deg",
         "Latitude of the ARP position."),
        ("CFG-TMODE-LON", 0x4003000a, "I4", 1e-7, "deg",
         "Longitude of the ARP position."),
        ("CFG-TMODE-HEIGHT", 0x4003000b, "I4", 1, "cm",
         "Height of the ARP position."),
        ("CFG-TMODE-LAT_HP", 0x2003000c, "I1", 1e-9, "deg",
         "High-precision latitude of the ARP position"),
        ("CFG-TMODE-LON_HP", 0x2003000d, "I1", 1e-9, "deg",
         "High-precision longitude of the ARP position."),
        ("CFG-TMODE-HEIGHT_HP", 0x2003000e, "I1", 0.1, "mm",
         "High-precision height of the ARP position."),
        ("CFG-TMODE-FIXED_POS_ACC", 0x4003000f, "U4", 0.1, "mm",
         "Fixed position 3D accuracy"),
        ("CFG-TMODE-SVIN_MIN_DUR", 0x40030010, "U4", 1, "s",
         "Survey-in minimum duration"),
        ("CFG-TMODE-SVIN_ACC_LIMIT", 0x40030011, "U4", 0.1, "mm",
         "Survey-in position accuracy limit"),

        # CFG-TP-
        # group 5
        ("CFG-TP", 0x3005ffff, "", 0, "",
         "get all CFG-TP"),
        ("CFG-TP-ANT_CABLEDELAY", 0x30050001, "I2", 0.000000001, "s",
         "Antenna cable delay"),
        ("CFG-TP-PERIOD_TP1", 0x40050002, "U4", 0.000001, "s",
         "Time pulse period (TP1)"),
        ("CFG-TP-PERIOD_LOCK_TP1", 0x40050003, "U4", 0.000001, "s",
         "Time pulse period when locked to GNSS time (TP1)"),
        ("CFG-TP-LEN_TP1", 0x40050004, "U4", 0.000001, "s",
         "Time pulse length (TP1)"),
        ("CFG-TP-LEN_LOCK_TP1", 0x40050005, "U4", 0.000001, "s",
         "Time pulse length when locked to GNSS time (TP1)"),
        ("CFG-TP-USER_DELAY_TP1", 0x40050006, "I4", 0.000000001, "s",
         "User configurable time pulse delay (TP1)"),
        ("CFG-TP-TP1_ENA", 0x10050007, "L", 1, "",
         "Enable the first timepulse"),
        ("CFG-TP-SYNC_GNSS_TP1", 0x10050008, "L", 1, "",
         "Sync time pulse to GNSS time or local clock (TP1)"),
        ("CFG-TP-USE_LOCKED_TP1", 0x10050009, "L", 1, "",
         "Use locked parameters when possible (TP1)"),
        ("CFG-TP-ALIGN_TO_TOW_TP1", 0x1005000a, "L", 1, "",
         "Align time pulse to top of second (TP1)"),
        ("CFG-TP-POL_TP1", 0x1005000b, "L", 1, "",
         "Set time pulse polarity (TP1)"),
        ("CFG-TP-TIMEGRID_TP1", 0x2005000c, "E1", 1, "",
         "Time grid to use (TP1)"),
        ("CFG-TP-PERIOD_TP2", 0x4005000d, "U4", 0.000001, "s",
         "Time pulse period (TP2)"),
        ("CFG-TP-PERIOD_LOCK_TP2", 0x4005000e, "U4", 0.000001, "s",
         "Time pulse period when locked to GNSS time (TP2)"),
        ("CFG-TP-LEN_TP2", 0x4005000f, "U4", 0.000001, "s",
         "Time pulse length (TP2)"),
        ("CFG-TP-LEN_LOCK_TP2", 0x40050010, "U4", 0.000001, "s",
         "Time pulse length when locked to GNSS time (TP2)"),
        ("CFG-TP-USER_DELAY_TP2", 0x40050011, "I4", 0.000000001, "s",
         "User configurable time pulse delay (TP2)"),
        ("CFG-TP-TP2_ENA", 0x10050012, "L", 1, "",
         "Enable the second timepulse"),
        ("CFG-TP-SYNC_GNSS_TP2", 0x10050013, "L", 1, "",
         "Sync time pulse to GNSS time or local clock (TP2)"),
        ("CFG-TP-USE_LOCKED_TP2", 0x10050014, "L", 1, "",
         "Use locked parameters when possible (TP2)"),
        ("CFG-TP-ALIGN_TO_TOW_TP2", 0x10050015, "L", 1, "",
         "Align time pulse to top of second (TP2)"),
        ("CFG-TP-POL_TP2", 0x10050016, "L", 1, "",
         "Set time pulse polarity (TP2)"),
        ("CFG-TP-TIMEGRID_TP2", 0x20050017, "E1", 1, "",
         "Time grid to use (TP2)"),
        ("CFG-TP-PULSE_DEF", 0x20050023, "E1", 1, "",
         "Determines whether the time pulse is interpreted as frequency "
         "or period?"),
        ("CFG-TP-FREQ_TP1", 0x40050024, "U4", 1, "Hz",
         "Time pulse frequency (TP1)"),
        ("CFG-TP-FREQ_LOCK_TP1", 0x40050025, "U4", 1, "Hz",
         "Time pulse frequency when locked to GNSS time (TP1)"),
        ("CFG-TP-FREQ_TP2", 0x40050026, "U4", 1, "Hz",
         "Time pulse frequency (TP2)"),
        ("CFG-TP-FREQ_LOCK_TP2", 0x40050027, "U4", 1, "Hz",
         "Time pulse frequency when locked to GNSS time (TP2)"),
        ("CFG-TP-DUTY_TP1", 0x5005002a, "R8", 1, "%",
         "Time pulse duty cycle (TP1)"),
        ("CFG-TP-DUTY_LOCK_TP1", 0x5005002b, "R8", 1, "%",
         "Time pulse duty cycle when locked to GNSS time (TP1)"),
        ("CFG-TP-DUTY_TP2", 0x5005002c, "R8", 1, "%",
         "Time pulse duty cycle (TP2)"),
        ("CFG-TP-DUTY_LOCK_TP2", 0x5005002d, "R8", 1, "%",
         "Time pulse duty cycle when locked to GNSS time (TP2)"),
        ("CFG-TP-PULSE_LENGTH_DEF", 0x20050030, "E1", 1, "",
         "Time pulse length is length[us] or pulse ratio[%]"),
        ("CFG-TP-DRSTR_TP1", 0x20050035, "E1", 1, "",
         "Drive strength TP1"),
        ("CFG-TP-DRSTR_TP2", 0x20050036, "E1", 1, "",
         "Drive strength TP2"),
        ("CFG-TP-MSG_ALWAYS", 0x10050037, "L", 1, "",
         "TP message behavior"),

        # CFG-TXREADY-
        # group a2
        ("CFG-TXREADY", 0x10a2ffff, "", 0, "",
         "get all CFG-TXREADY"),
        ("CFG-TXREADY-ENABLED", 0x10a20001, "L", 1, "",
         "Flag to indicate if tx ready pin mechanism should be enabled"),
        ("CFG-TXREADY-POLARITY", 0x10a20002, "L", 1, "",
         "Polarity of the tx ready pin:false:high-active, true:low-active"),
        ("CFG-TXREADY-PIN", 0x20a20003, "U1", 1, "",
         "Pin number to use for the tx ready functionality"),
        ("CFG-TXREADY-THRESHOLD", 0x30a20004, "U2", 1, "",
         "Amount of data ready on interface before triggering tx ready pin"),
        ("CFG-TXREADY-INTERFACE", 0x20a20005, "E1", 1, "",
         "Interface where the tx ready feature should be linked to"),

        # CFG-UART1-
        # group 52
        ("CFG-UART1", 0x4052ffff, "", 0, "",
         "get all CFG-UART1"),
        ("CFG-UART1-BAUDRATE", 0x40520001, "U4", 1, "",
         "The baud rate that should be configured on the UART1"),
        ("CFG-UART1-STOPBITS", 0x20520002, "E1", 1, "",
         "Number of stopbits that should be used on UART1"),
        ("CFG-UART1-DATABITS", 0x20520003, "E1", 1, "",
         "Number of databits that should be used on UART1"),
        ("CFG-UART1-PARITY", 0x20520004, "E1", 1, "",
         "Parity mode that should be used on UART1"),
        ("CFG-UART1-ENABLED", 0x10520005, "L", 1, "",
         "Flag to indicate if the UART1 should be enabled"),

        # CFG-UART1INPROT
        ("CFG-UART1INPROT", 0x1073ffff, "", 0, "",
         "get all CFG-UART1INPROT"),
        ("CFG-UART1INPROT-UBX", 0x10730001, "L", 1, "",
         "Flag to indicate if UBX should be an input protocol on UART1"),
        ("CFG-UART1INPROT-NMEA", 0x10730002, "L", 1, "",
         "Flag to indicate if NMEA should be an input protocol on UART1"),
        ("CFG-UART1INPROT-RTCM2X", 0x10730003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input protocol on UART1"),
        ("CFG-UART1INPROT-RTCM3X", 0x10730004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an input protocol on UART1"),
        ("CFG-UART1INPROT-SPARTN", 0x10730005, "L", 1, "",
         "Flag to indicate if SPARTN should be an input protocol on UART1"),

        # CFG-UART1OUTPROT
        ("CFG-UART1OUTPROT", 0x1074ffff, "", 0, "",
         "get all CFG-UART1OUTPROT"),
        ("CFG-UART1OUTPROT-UBX", 0x10740001, "L", 1, "",
         "Flag to indicate if UBX should be an output protocol on UART1"),
        ("CFG-UART1OUTPROT-NMEA", 0x10740002, "L", 1, "",
         "Flag to indicate if NMEA should be an output protocol on UART1"),
        ("CFG-UART1OUTPROT-RTCM3X", 0x10740004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output protocol on UART1"),

        # CFG-UART2-
        ("CFG-UART2", 0x4053FFFF, "", 0, "",
         "get all CFG-UART2"),
        ("CFG-UART2-BAUDRATE", 0x40530001, "U4", 1, "",
         "The baud rate that should be configured on the UART2"),
        ("CFG-UART2-STOPBITS", 0x20530002, "E1", 1, "",
         "Number of stopbits that should be used on UART2"),
        ("CFG-UART2-DATABITS", 0x20530003, "E1", 1, "",
         "Number of databits that should be used on UART2"),
        ("CFG-UART2-PARITY", 0x20530004, "E1", 1, "",
         "Parity mode that should be used on UART2"),
        ("CFG-UART2-ENABLED", 0x10530005, "L", 1, "",
         "Flag to indicate if the UART2 should be enabled"),
        ("CFG-UART2-REMAP", 0x10530006, "L", 1, "",
         "UART2 Remapping"),

        # CFG-UART2INPROT
        ("CFG-UART2INPROT", 0x1075ffff, "", 0, "",
         "get all CFG-UART2INPROT"),
        ("CFG-UART2INPROT-UBX", 0x10750001, "L", 1, "",
         "Flag to indicate if UBX should be an input protocol on UART2"),
        ("CFG-UART2INPROT-NMEA", 0x10750002, "L", 1, "",
         "Flag to indicate if NMEA should be an input protocol on UART2"),
        ("CFG-UART2INPROT-RTCM2X", 0x10750003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input protocol on UART2"),
        ("CFG-UART2INPROT-RTCM3X", 0x10750004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an input protocol on UART2"),
        ("CFG-UART2INPROT-SPARTN", 0x10750005, "L", 1, "",
         "Flag to indicate if SPARTN should be an input protocol on UART2"),

        # CFG-UART2OUTPROT
        ("CFG-UART2OUTPROT", 0x1076ffff, "", 0, "",
         "get all CFG-UART2OUTPROT"),
        ("CFG-UART2OUTPROT-UBX", 0x10760001, "L", 1, "",
         "Flag to indicate if UBX should be an output protocol on UART2"),
        ("CFG-UART2OUTPROT-NMEA", 0x10760002, "L", 1, "",
         "Flag to indicate if NMEA should be an output protocol on UART2"),
        ("CFG-UART2OUTPROT-RTCM3X", 0x10760004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output protocol on UART2"),

        # CFG-UNITTEST--
        ("CFG-UNITTEST", 0x10feFFFF, "", 0, "",
         "get all CFG-UNITTEST"),

        ("CFG-UNITTEST-FLAG1", 0x10fe0f11, "L", 1, "",
         "Single bit flag"),

        ("CFG-UNITTEST-UNSIGNED1", 0x20fe0f22, "U1", 1, "",
         "Unsigned one byte value"),
        ("CFG-UNITTEST-UNSIGNED2", 0x30fe0f23, "U2", 1, "",
         "Unsigned two byte value"),
        ("CFG-UNITTEST-UNSIGNED4", 0x40fe0f24, "U4", 1, "",
         "Unsigned four byte value"),
        ("CFG-UNITTEST-UNSIGNED8", 0x50fe0f25, "U8", 1, "",
         "Unsigned eight byte value"),

        ("CFG-UNITTEST-ENUM1", 0x20fe0f42, "E1", 1, "",
         "One byte enumeration"),
        ("CFG-UNITTEST-ENUM2", 0x30fe0f43, "E2", 1, "",
         "Two bytes enumeration"),
        ("CFG-UNITTEST-ENUM4", 0x40fe0f44, "E4", 1, "",
         "Four bytes enumeration"),

        ("CFG-UNITTEST-SIGNED1", 0x20fe0f32, "U1", 1, "",
         "Signed one byte value"),
        ("CFG-UNITTEST-SIGNED2", 0x30fe0f33, "I2", 1, "",
         "Signed two byte value"),
        ("CFG-UNITTEST-SIGNED4", 0x40fe0f34, "I4", 1, "",
         "Signed four byte value"),
        ("CFG-UNITTEST-SIGNED8", 0x50fe0f35, "I8", 1, "",
         "Signed eight byte value"),
        ("CFG-UNITTEST-FLOAT4", 0x40fe0f36, "R4", 1, "",
         "Four bytes float"),
        ("CFG-UNITTEST-FLOAT8", 0x50fe0f37, "R8", 1, "",
         "Eight bytes float"),

        ("CFG-UNITTEST-BITFIELD1", 0x20fe0f52, "X1", 1, "",
         "One-byte bit field"),
        ("CFG-UNITTEST-BITFIELD2", 0x30fe0f53, "X2", 1, "",
         "Two-byte bit field"),
        ("CFG-UNITTEST-BITFIELD4", 0x40fe0f54, "X4", 1, "",
         "Four-byte bit field"),
        ("CFG-UNITTEST-BITFIELD8", 0x50fe0f55, "X8", 1, "",
         "Eight-byte bit field"),

        ("CFG-UNITTEST-U1_PUB", 0x20fe0f61, "U1", 1, "",
         "Public config item"),
        ("CFG-UNITTEST-E1_PUB", 0x20fe0f65, "E1", 1, "",
         "Public enumeration"),
        ("CFG-UNITTEST-X1_PUB", 0x20fe0f69, "X1", 1, "",
         "Public bit field"),
        ("CFG-UNITTEST-R4_SCALE1", 0x40fe0f6d, "R4", 1, "",
         "Four bytes float with scale 2^-31"),
        ("CFG-UNITTEST-R8_SCALE1", 0x50fe0f6e, "R8", 1, "",
         "Eight bytes float with scale 2^-31"),
        ("CFG-UNITTEST-R4_SCALE2", 0x40fe0f6f, "R4", 1, "",
         "Four bytes float with scale 0.5"),

        ("CFG-UNITTEST-R8_SCALE2", 0x50fe0f70, "R8", 1, "",
         "Eight bytes float with scale 0.5"),
        ("CFG-UNITTEST-R4_SCALE3", 0x40fe0f71, "R4", 1, "",
         "Four bytes float with scale 1/3"),
        ("CFG-UNITTEST-R8_SCALE3", 0x50fe0f72, "R8", 1, "",
         "Eight bytes float with scale 1/3"),

        # CFG-USB-
        ("CFG-USB", 0x1065ffff, "", 0, "",
         "get all CFG-USB"),
        ("CFG-USB-ENABLED", 0x10650001, "L", 1, "",
         "Flag to indicate if the USB interface should be enabled"),
        ("CFG-USB-SELFPOW", 0x10650002, "L", 1, "",
         "Self-Powered device"),
        ("CFG-USB-VENDOR_ID", 0x3065000a, "U2", 1, "",
         "Vendor ID"),
        ("CFG-USB-PRODUCT_ID", 0x3065000b, "U2", 1, "",
         "Product ID"),
        ("CFG-USB-POWER", 0x3065000c, "U2", 1, "mA",
         "Power consumption"),
        ("CFG-USB-VENDOR_STR0", 0x5065000d, "X8", 1, "",
         "Vendor string characters 0-7"),
        ("CFG-USB-VENDOR_STR1", 0x5065000e, "X8", 1, "",
         "Vendor string characters 8-15"),
        ("CFG-USB-VENDOR_STR2", 0x5065000f, "X8", 1, "",
         "Vendor string characters 16-23"),
        ("CFG-USB-VENDOR_STR3", 0x50650010, "X8", 1, "",
         "Vendor string characters 24-31"),
        ("CFG-USB-PRODUCT_STR0", 0x50650011, "X8", 1, "",
         "Product string characters 0-7"),
        ("CFG-USB-PRODUCT_STR1", 0x50650012, "X8", 1, "",
         "Product string characters 8-15"),
        ("CFG-USB-PRODUCT_STR2", 0x50650013, "X8", 1, "",
         "Product string characters 16-23"),
        ("CFG-USB-PRODUCT_STR3", 0x50650014, "X8", 1, "",
         "Product string characters 24-31"),
        ("CFG-USB-SERIAL_NO_STR0", 0x50650015, "X8", 1, "",
         "Serial number string characters 0-7"),
        ("CFG-USB-SERIAL_NO_STR1", 0x50650016, "X8", 1, "",
         "Serial number string characters 8-15"),
        ("CFG-USB-SERIAL_NO_STR2", 0x50650017, "X8", 1, "",
         "Serial number string characters 16-23"),
        ("CFG-USB-SERIAL_NO_STR3", 0x50650018, "X8", 1, "",
         "Serial number string characters 24-31"),

        # CFG-USB-INPROT
        ("CFG-USBINPROT", 0x1077ffff, "", 0, "",
         "get all CFG-USBINPROT"),
        ("CFG-USBINPROT-UBX", 0x10770001, "L", 1, "",
         "Flag to indicate if UBX should be an input protocol on USB"),
        ("CFG-USBINPROT-NMEA", 0x10770002, "L", 1, "",
         "Flag to indicate if NMEA should be an input protocol on USB"),
        ("CFG-USBINPROT-RTCM2X", 0x10770003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input protocol on USB"),
        ("CFG-USBINPROT-RTCM3X", 0x10770004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an input protocol on USB"),
        ("CFG-USBINPROT-SPARTN", 0x10770005, "L", 1, "",
         "Flag to indicate if SPARTN should be an input protocol on USB"),

        # CFG-USB-OUTPROT
        ("CFG-USBOUTPROT", 0x1078ffff, "", 0, "",
         "get all CFG-USBOUTPROT"),
        ("CFG-USBOUTPROT-UBX", 0x10780001, "L", 1, "",
         "Flag to indicate if UBX should be an output protocol on USB"),
        ("CFG-USBOUTPROT-NMEA", 0x10780002, "L", 1, "",
         "Flag to indicate if NMEA should be an output protocol on USB"),
        ("CFG-USBOUTPROT-RTCM3X", 0x10780004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output protocol on USB"),

        # undocumented
        # CFG-4
        ("CFG-4", 0x1004ffff, "", 0, "",
         "get all CFG-50"),
        # F9T
        ("CFG-4-1", 0x10040001, "L", 1, "",
         "Unknown"),
        ("CFG-4-2", 0x10040002, "L", 1, "",
         "Unknown"),
        ("CFG-4-3", 0x10040003, "L", 1, "",
         "Unknown"),
        ("CFG-4-4", 0x10040004, "L", 1, "",
         "Unknown"),
        ("CFG-4-5", 0x10040005, "L", 1, "",
         "Unknown"),

        # CFG-50
        ("CFG-50", 0x1032ffff, "", 0, "",
         "get all CFG-50"),
        # F9T
        ("CFG-50-1", 0x10320001, "L", 1, "",
         "Ignore GPS L5 heatth status"),
        ("CFG-50-2", 0x10320002, "L", 1, "",
         "Unknown"),
        ("CFG-50-3", 0x10320003, "L", 1, "",
         "Unknown"),
        ("CFG-50-4", 0x10320004, "L", 1, "",
         "Unknown"),
       )

    def item_to_type(self, item):
        """Return (size, pack format, i/i/f) for item"""

        # conversion of known types from known key
        cfg_types = {"E1": (1, "<B", "u"),
                     "E2": (2, "<H", "u"),
                     "E4": (4, "<L", "u"),
                     "I1": (1, "<b", "i"),
                     "I2": (2, "<h", "i"),
                     "I4": (4, "<l", "i"),
                     "I8": (8, "<q", "i"),
                     "L": (1, "<B", "u"),
                     "R4": (4, "<f", "f"),
                     "R8": (8, "<d", "f"),
                     "U1": (1, "<B", "u"),
                     "U2": (2, "<H", "u"),
                     "U4": (4, "<L", "u"),
                     "U8": (8, "<Q", "u"),
                     "X1": (1, "<B", "u"),
                     "X2": (2, "<H", "u"),
                     "X4": (4, "<L", "u"),
                     "X8": (8, "<Q", "u"),
                     }
        # guess of known types from unknown key
        key_map = {0: (1, "<B", "u"),       # illegal
                   1: (1, "<B", "u"),       # one bit
                   2: (1, "<B", "u"),       # one byte
                   3: (2, "<H", "u"),       # two byte
                   4: (4, "<L", "u"),       # four byte
                   5: (8, "<B", "u"),       # eight byte
                   6: (1, "<B", "u"),       # illegal
                   7: (1, "<B", "u"),       # illegal
                   }

        key = item[1]
        val_type = item[2]
        if val_type in cfg_types:
            cfg_type = cfg_types[val_type]
        else:
            # unknown? get length correct
            key_size = (key >> 28) & 0x07
            cfg_type = key_map[key_size]

        return cfg_type

    cfg_by_key_group = {0x03: "TMODE",
                        0x05: "TP",
                        0x06: "SFIMU",
                        0x07: "SFODO",
                        0x08: "SFCORE",
                        0x09: "RTCM",
                        0x0c: "CFB",
                        0x11: "NAVSPG",
                        0x14: "NAVHPG",
                        0x17: "NAV2",
                        0x18: "NAVMASK",
                        0x21: "RATE",
                        0x22: "ODO",
                        0x23: "ANA",
                        0x24: "GEOFENCE",
                        0x25: "MOT",
                        0x26: "BATCH",
                        0x31: "SIGNAL",
                        0x34: "BDS",
                        0x35: "GAL",
                        0x36: "SBAS",
                        0x37: "QZSS",
                        0x41: "ITFM",
                        0x51: "I2C",
                        0x52: "UART1",
                        0x53: "UART2",
                        0x54: "GAL",
                        0x64: "SPI",
                        0x65: "USB",
                        0x71: "I2CINPROT",
                        0x72: "I2COUTPROT",
                        0x73: "UART1INPROT",
                        0x74: "UART1OUTPROT",
                        0x75: "UART2INPROT",
                        0x76: "UART2OUTPROT",
                        0x77: "USBOUTPROT",
                        0x78: "USBOUTPROT",
                        0x79: "SPIINPROT",
                        0x7a: "SPIOUTPROT",
                        0x91: "MSGOUT",
                        0x92: "INFMSG",
                        0x93: "NMEA",
                        0xa2: "TXREADY",
                        0xa3: "HW",
                        0xa7: "SPARTN",
                        0xb1: "PMP",
                        0xc7: "RINV",
                        0xd0: "PM",
                        0xde: "LOGFILTER",
                        0xf6: "SEC",
                        0xfe: "UNITTEST",
                        }

    cfg_by_key_kmap = {0: "Z0",
                       1: "L",
                       2: "U1",
                       3: "U2",
                       4: "U4",
                       5: "U8",
                       6: "Z6",
                       7: "Z7",
                       }

    def cfg_by_key(self, key):
        """Find a config item by key"""

        for item in self.cfgs:
            if item[1] == key:
                return item

        # not found, build a fake item, guess on decode
        group = (key >> 16) & 0xff
        group_name = index_s(group, self.cfg_by_key_group)
        if "Unk" == group_name:
            group_name = str(group)

        # ID within group is 12 bits
        name = "CFG-%s-%u" % (group_name, key & 0xfff)
        size = (key >> 28) & 0x07
        item = (name, key, self.cfg_by_key_kmap[size], 1, "Unk", "Unknown")

        return item

    def cfg_by_name(self, name):
        """Find a config item by name"""

        for item in self.cfgs:
            if item[0] == name:
                return item

        return None

    # keep up to date with include/gps.h sigids
    # these are UBX sigids, not NMEA sigids.
    id_map = {
        0: {"name": "GPS",
            "sig": {0: "L1C/A", 3: "L2CL", 4: "L2CM", 5: "L5I", 7: "L5Q"}},
        1: {"name": "SBAS",
            # Only L1C/A in use July 2026
            "sig": {0: "L1C/A", 3: "L2CL", 4: "L2CM"}},
        2: {"name": "Galileo",
            "sig": {0: "E1C", 1: "E1B", 3: "E5aI", 4: "E5aQ", 5: "E5 bl",
                    6: "E5 bQ", 8: "E6B", 9: "E6C", 10: "E6A"}},
        3: {"name": "BeiDou",
            "sig": {0: "B1I D1", 1: "B1I D2", 2: "B2I D1", 3: "B2I D2",
                    5: "B1 Cp",  6: "B1 Cd", 7: "B2 ap", 8: "B2 ad",
                    10: "B3I D2"}},
        4: {"name": "IMES",
            "sig": {0: "L1C/A", 3: "L2 CL", 4: "L2 CM"}},
        5: {"name": "QZSS",
            "sig": {0: "L1C/A", 1: "L1 S", 4: "L2 CM", 5: "L2 CL",
                    8: "L5 I", 9: "L5 Q", 12: "L1 C/B"}},
        6: {"name": "GLONASS",
            "sig": {0: "L1 OF", 2: "L2 OF"}},
        # formerly NavIC
        7: {"name": "IRNSS",
            "sig": {0: "L5A"}},
    }

    def gnss_s(self, gnssId, svId, sigId):
        """Verbose decode of gnssId, svId and sigId"""

        s = ''

        if gnssId in self.id_map:
            if "name" not in self.id_map[gnssId]:
                s = "%d:%d:%d" % (gnssId, svId, sigId)
            elif sigId not in self.id_map[gnssId]["sig"]:
                s = ("%s:%d:%d" %
                     (self.id_map[gnssId]["name"], svId, sigId))
            else:
                s = ("%s:%d:%s" %
                     (self.id_map[gnssId]["name"], svId,
                      self.id_map[gnssId]["sig"][sigId]))
        else:
            s = "%d:%d:%d" % (gnssId, svId, sigId)

        return s

    def ack_ack(self, buf):
        """UBX-ACK-ACK decode"""

        # NOTE: Not all messages to u-blox GPS are ACKed...

        u = struct.unpack_from('<BB', buf, 0)
        return ' ACK to %s' % self.class_id_s(u[0], u[1])

    def ack_nak(self, buf):
        """UBX-ACK-NAK decode"""

        # NOTE: Not all messages to u-blox GPS are ACKed...

        u = struct.unpack_from('<BB', buf, 0)
        return ' NAK to %s' % self.class_id_s(u[0], u[1])

    # UBX-ACK-
    ack_ids = {0: {'str': 'NAK', 'dec': ack_nak, 'minlen': 2,
                   'name': 'UBX-ACK-NAK'},
               1: {'str': 'ACK', 'dec': ack_ack, 'minlen': 2,
                   'name': 'UBX-ACK-ACK'}}

    # UBX-AID-
    # All UBX-AID- removed in protVer 32.  u-blox 9, and 10
    def aid_alm(self, buf):
        """UBX-AID-ALM decode, GPS Aiding Almanac Data

Deprecated in protVer 23.00
Removed in protVer 32.00 and up
"""
        m_len = len(buf)

        if 1 == m_len:
            return "  Poll request svid %u" % buf[0]

        if 8 > m_len:
            return "  Bad Length %s" % m_len

        u = struct.unpack_from('<LL', buf, 0)
        s = ' svid %u week %u ' % u

        if 0 != u[1] and 8 < m_len <= 40:
            u = struct.unpack_from('<LLLLLLLL', buf, 8)
            s += ('\n dwrd %08x %08x %08x %08x'
                  '\n      %08x %08x %08x %08x' % u)

        return s

    def aid_alp(self, buf):
        """UBX-AID-ALP decode, AlmanacPlus

Deprecated in protVer 23.00
Removed in M10 (protVer 34.00 and up)
"""

        # u-blox 6, protVer 6 to 7

        m_len = len(buf)
        if 1 == m_len(buf):
            # different meaning for in and out
            u = struct.unpack_from('<B', buf, 0)
            return '  dummy/ack  %u' % u

        if 24 == m_len(buf):
            # different meaning for in and out
            u = struct.unpack_from('LLlHHLBBH', buf, 0)
            return ('  predTow %u predDur %u age %d predWno %u almWno %u\n'
                    '  res1 %u svs %u res23 %u %u' % u)

        # else
        s = '  alpData len %u' % u

        # FIXME: partial decode

        return s

    def aid_alpsrv(self, buf):
        """UBX-AID-ALPSRV decode, AlmanacPlus

Deprecated in protVer 23.00
Removed in M10 (protVer 34.00 and up)
"""

        # u-blox 6, protVer 6 to 7

        u = struct.unpack_from('<BBHHH', buf, 0)
        s = ' idSize %u type %u ofx %u size %u fileId %u' % u

        # FIXME: partial decode

        return s

    def aid_aop(self, buf):
        """UBX-AID-AOP decode, AssistNow Autonomous data

Deprecated in protVer 23.00
Removed in M10 (protVer 34.00 and up)
"""
        m_len = len(buf)

        if 1 == m_len:
            return "  Poll request svid %u" % buf[0]

        # length 2 is undocumented...
        if 2 > m_len:
            return "  Bad Length %s" % m_len

        u = struct.unpack_from('<BB', buf, 0)
        s = ' gnssid %u svid %u' % u
        # FIXME: dump the rest...

        return s

    def aid_data(self, buf):
        """UBX-AID-DATA decode, Poll all GPS Initial Aiding Data

Deprecated in protVer 23.00
Removed in M10 (protVer 34.00 and up)
"""

        # u-blox 6
        # If this poll is received, the messages AID-INI, AID-HUI,
        # AID-EPH and AID-ALM are sent.
        return "  Poll all GPS Initial Aiding Data"

    def aid_eph(self, buf):
        """UBX-AID-EPH decode, GPS Aiding Ephemeris Data

Deprecated in protVer 23.00
Removed in M10 (protVer 34.00 and up)
"""
        m_len = len(buf)

        if 1 == m_len:
            return "  Poll request svid %u" % buf[0]

        if 8 > m_len:
            return "  Bad Length %s" % m_len

        u = struct.unpack_from('<LL', buf, 0)
        s = ' svid %u how x%x ' % u

        if 0 != u[1] and 8 < m_len <= 104:
            u = struct.unpack_from('<LLLLLLLLLLLLLLLLLLLLLLLL', buf, 8)
            s += ('\n sf1d %08x %08x %08x %08x'
                  '\n      %08x %08x %08x %08x'
                  '\n sf2d %08x %08x %08x %08x'
                  '\n      %08x %08x %08x %08x'
                  '\n sf3d %08x %08x %08x %08x'
                  '\n      %08x %08x %08x %08x' % u)

        return s

    def aid_hui(self, buf):
        """UBX-AID-HUI decode, GPS Heatlh, UTC, Ionosphere

Deprecated in protVer 23.00
Removed in M10 (protVer 34.00 and up)
"""

        u = struct.unpack_from('<LddlhhhhhhffffffffL', buf, 0)
        s = (' health x%x utcA0 %e utcA1 %e utcTOW %d'
             '\n utcWNT %d utcLS %d utcWNF %d utcDN %d utcLSF %d utcSpare %d'
             '\n klobA0 %e klobA1 %e klobA2 %e'
             '\n klobA3 %e klobB0 %e klobB1 %e'
             '\n klobB2 %e klobB3 %e flags x%x' % u)

        return s

    def aid_ini(self, buf):
        """UBX-AID-INI decode, Aiding position, time, frequency, clock drift

Deprecated in protVer 23.00
Removed in M10 (protVer 34.00 and up)
"""

        u = struct.unpack_from('<lllLHHLlLLlLL', buf, 0)
        s = (' ecefXOrLat %d ecefYOrLon %d ecefZOrAlt %d posAcc %u'
             '\n tmCfg x%x wnoOrDate %u towOrTime %u towNs %d'
             '\n tAccMs %u tAccNs %u clkDOrFreq %d clkDAccOrFreqAcc %u'
             '\n flags x%x' % u)

        return s

    def aid_req(self, buf):
        """UBX-AID-REQ decode, Sends a poll for all GPS Aiding Data

Deprecated in protVer 23.00
Removed in M10 (protVer 34.00 and up)
"""

        return "  poll (AID-DATA) for all GPS Aiding Data"

    # All UBX-AID messages are deprecated, gone in M10 (protVer 34)
    # use UBX-MGA messages instead
    aid_ids = {
               # u-blox 6
               0x00: {'str': 'REQ', 'dec': aid_req, 'minlen': 0,
                      'name': 'UBX-AID-REQ', 'depver': 23.0},
               0x01: {'str': 'INI', 'dec': aid_ini, 'minlen': 48,
                      'name': 'UBX-AID-INI', 'depver': 23.0},
               0x02: {'str': 'HUI', 'dec': aid_hui, 'minlen': 72,
                      'name': 'UBX-AID-HUI', 'depver': 23.0},
               # u-blox 6
               0x10: {'str': 'DATA', 'dec': aid_data, 'minlen': 0,
                      'name': 'UBX-AID-DATA', 'depver': 23.0},
               0x30: {'str': 'ALM', 'dec': aid_alm, 'minlen': 1,
                      'name': 'UBX-AID-ALM', 'depver': 23.0},
               0x31: {'str': 'EPH', 'dec': aid_eph, 'minlen': 1,
                      'name': 'UBX-AID-EPH', 'depver': 23.0},
               # u-blox 6
               0x32: {'str': 'ALPSRV', 'dec': aid_alpsrv, 'minlen': 8,
                      'name': 'UBX-AID-ALPSRV', 'depver': 23.01},
               0x33: {'str': 'AOP', 'dec': aid_aop, 'minlen': 1,
                      'name': 'UBX-AID-AOP', 'depver': 23.0},
               0x50: {'str': 'ALP', 'dec': aid_alp, 'minlen': 1,
                      'name': 'UBX-AID-ALP', 'depver': 23.0},
               }

    cfg_ant_pins = {
        1: 'svcs',
        2: 'scd',
        4: 'ocd',
        8: 'pdwnOnSCD',
        0x10: 'recovery',
        }

    def cfg_ant(self, buf):
        """UBX-CFG-ANT decode

Deprecated in protVer 23.01
"""

        u = struct.unpack_from('<HH', buf, 0)
        s = ' flags x%x pins x%x ' % u
        s += ('pinSwitch %u pinSCD %u pinOCD %u reconfig %u' %
              (u[1] & 0x1f, (u[1] >> 5) & 0x1f, (u[1] >> 10) & 0x1f,
               u[1] >> 15))
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n      flags (%s)' % flag_s(u[0], self.cfg_ant_pins))

        return s

    cfg_batch_flags = {
        1: 'enable',
        4: 'extraPvt',
        8: 'extraOdo',
        0x20: 'pioEnable',
        0x40: 'pioActiveLow',
        }

    def cfg_batch(self, buf):
        """UBX-CFG-BATCH decode

Deprecated in protVer 32.00
"""

        u = struct.unpack_from('<BBHHBB', buf, 0)
        s = ("  version %u flags x%x bufsize %u notifThrs %u\n"
             "  pioId %u reserved1 %u" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n      flags (%s)' % flag_s(u[1], self.cfg_batch_flags))

        return s

    cfg_cfg_mask = {
        0x1: 'ioPort',
        0x2: 'msgConf',
        0x4: 'infMsg',
        0x8: 'navConf',
        0x10: 'rxmConf',
        0x100: 'senConf',        # not on M8030, sfdrConf in u-blox 5
        0x200: 'rinvConf',
        0x400: 'antConf',
        0x800: 'logConf',        # not in u-blox 5
        0x1000: 'ftsConf',       # protVer 16+
        }

    cfg_cfg_dev = {
        0x1: 'devBBR',
        0x2: 'devFlash',
        0x4: 'devEEPROM',             # only protVer less then 14.00
        0x10: 'devSpiFlash',          # only protVer less than 14.00
        }

    def cfg_cfg(self, buf):
        """UBX-CFG-CFG decode

"not completely  backwards-compatible."

Deprecated in protVer 28.00
"""
        m_len = len(buf)

        if 12 == m_len:
            u = struct.unpack_from('<LLL', buf, 0)
        elif 13 == m_len:
            u = struct.unpack_from('<LLLB', buf, 0)
        else:
            return "  Bad Length %s" % m_len

        s = ('  clearMask: %#x (%s)\n' %
             (u[0], flag_s(u[0], self.cfg_cfg_mask)))
        s += ('  saveMask: %#x (%s)\n' %
              (u[1], flag_s(u[1], self.cfg_cfg_mask)))
        s += ('  loadMask: %#x (%s)\n' %
              (u[2], flag_s(u[2], self.cfg_cfg_mask)))

        if 13 <= m_len:
            s += ('  deviceMask: %#x (%s)\n' %
                  (u[3], flag_s(u[3], self.cfg_cfg_dev)))

        return s

    def cfg_dat(self, buf):
        """UBX-CFG-DAT decode, Standard Datum configuration

Deprecated in protVer 23.01
"""

        # u-blox 5 to 9, protVer 4.00 to 29
        m_len = len(buf)

        if 2 == m_len:
            # standard datum
            u = struct.unpack_from('<H', buf, 0)
            s = "  datumNum %u" % u

        elif 44 > m_len:
            s = "  Bad Length %d" % m_len

        elif 44 == m_len:
            # set user defined datum
            u = struct.unpack_from('<ddfffffff', buf, 0)
            s = ("  majA %.1f flat %.1f dX %.1f dY %.1f dZ %.1f\n"
                 "  rotX %.1f rotY %.1f rotZ %.1f scale %.1f" % u)

        elif 52 > m_len:
            s = "  Bad Length %d" % m_len

        elif 52 <= m_len:
            # get user defined datum
            u = struct.unpack_from('<HBBBBBBddfffffff', buf, 0)
            s = ("  datumNum %u datumNam %u %u %u %u %u %u\n"
                 "  majA %.1f flat %.1f dX %.1f dY %.1f dZ %.1f\n"
                 "  rotX %.1f rotY %.1f rotZ %.1f scale %.1f" % u)
            if gps.VERB_DECODE <= self.verbosity:
                s += ('\n   datumName (%s)' %
                      gps.polystr(buf[2:8]).rstrip('\0'))

        else:
            s = "I'm confused..."

        return s

    cfg_dgnss_mode = {
        2: "RTK Float",
        3: "RTK Fixed",
        }

    def cfg_dgnss(self, buf):
        """UBX-CFG-DGNSS decode, DGNSS configuration

Deprecated in protVer 32.00
"""

        u = struct.unpack_from('<BBBB', buf, 0)
        s = (" dgnssMode %u (%s) reserved1 %u %u %u" %
             (u[0], index_s(u[0], self.cfg_dgnss_mode), u[1], u[2], u[3]))
        return s

    cfg_dosc_controlIf = {
        0: "Custom host DAC",
        1: "Microchip MCP4726",
        2: "TI DAC8571",
        13: "12-bit host DAC",
        14: "14-bit host DAC",
        15: "16-bit host DAC",
        }

    cfg_dosc_isCal = {
        0: "Uncalibrated",
        1: "Calibrated",
        }

    cfg_dosc_oscId = {
        0: "Internal",
        1: "External",
        }

    def cfg_dosc(self, buf):
        """UBX-CFG-DOSC decode, Disciplined oscillator configuration"""

        # at least protver 16, time firmware only
        if 16 > self.protver:
            self.protver = 16

        u = struct.unpack_from('<BBH', buf, 0)
        if 0 != u[0]:
            return "  Unknown version %u" % u[0]
        if 2 > u[1]:
            return "  Bad numOsc %u" % u[1]

        s = "  version %u numOsc %u reserved1 x%x\n" % u

        m_len = len(buf)
        if 4 != (m_len - u[1] * 32):
            return "  Bad length %u s/b %u" % (m_len, 4 + 32 * u[1])

        for i in range(u[1]):
            u = struct.unpack_from('<BBHLlLLHHLBBH', buf, 4 + i * 32)
            s += "   %d: " % i
            s += ("oscId %u reserved2 x%x flags x%x freq %u\n"
                  "      phaseOffset %d withTemp %u withAge %u timeToTemp %u\n"
                  "      reserved3 x%x gainVco %u gainUncertainty %u\n"
                  "      reserved4 x%02x%04x\n" % u)
            if gps.VERB_DECODE <= self.verbosity:
                s += ("        oscId (%s) flags (%s, %s)\n"
                      "        freq %.1f Hz wtihTemp %.3f ppb  "
                      "withAge %.3f ppb/y\n"
                      "        gainVco %.4f ppb/r gainUncertainty %3f\n" %
                      (index_s(u[0], self.cfg_dosc_oscId),
                       index_s((u[2] >> 1) & 0x0f, self.cfg_dosc_controlIf),
                       index_s(u[2] & 1, self.cfg_dosc_isCal),
                       u[3] / 4.0, u[5] / 256.0, u[6] / 256.0,
                       u[9] / 65536.0, u[10] / 256.0))

        return s[0:-1]     # remove trailing \n

    def cfg_dynseed(self, buf):
        """UBX-CFG-DYNSEED decode,
Programming the dynamic seed for host interface signature"""

        # u-blox 8 only, protVer 18 to 23
        u = struct.unpack_from('<BBHLL', buf, 0)
        s = " version %u reserved1 %u %u seedHi %u seedLo %u" % u
        return s

    def cfg_esfa(self, buf):
        """UBX-CFG-ESFA decode, Accelerometer sensor configuration
protVer 19 and up, UDR only"""

        # at least protver 19
        if 19 > self.protver:
            self.protver = 19

        u = struct.unpack_from('<BLLBBBHHL', buf, 0)
        s = (' version %u reserved1 x%x %x %x accelRmsThdl %u frequency %u\n'
             '  latency %u accuracy %u reserved2 x%x' % u)
        return s

    cfg_esfalg_bitfield = {
        0x1: 'doAutoMntAlg',
        }

    def cfg_esfalg(self, buf):
        """UBX-CFG-ESFALG decode, IMU-mount misalignment configuration

protVer 15.01 and up, ADR and UDR only"""

        # at least protver 15
        if 15 > self.protver:
            self.protver = 15

        u = struct.unpack_from('<LLhh', buf, 0)
        s = ' bitfield x%x aw %u pitch %d roll %d' % u
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    bitfield (%s) version %u" %
                  (flag_s((u[0] >> 8) & 1, self.cfg_esfalg_bitfield),
                   u[0] & 0x0ff))
        return s

    def cfg_esfg(self, buf):
        """UBX-CFG-ESFG decode, Gyro sensor configuration

protVer 19 and up, UDR only"""

        # u-blox 8, protver 19 and up
        if 19 > self.protver:
            self.protver = 19

        u = struct.unpack_from('<BLHBHBBHHL', buf, 0)
        s = (' version %u reserved1 x%x %x %x tcTableSaveRate %u\n'
             '  gyroRmsThdl %u frequency %u latency %u accuracy %u\n'
             '  reserved2 x%x' % u)
        return s

    cfg_esfwt_flags1 = {
        0x1: 'combineTicks',
        0x10: 'useWtSpeed',
        0x20: 'dirPinPol',
        0x40: 'useWtPin',
        }

    cfg_esfwt_flags2 = {
        0x1: 'autoWtCountMaxOff',
        0x2: 'autoDirPinPolOff',
        0x4: 'autoSoftwareWtOff',
        0x8: 'autoUseWtSpeedOff',
        }

    def cfg_esfwt(self, buf):
        """UBX-CFG-ESFWT decode, Wheel tick configuration
protVer 15.01 and up, ADR only"""

        # at least protver 15
        if 15 > self.protver:
            self.protver = 15

        u = struct.unpack_from('<BBBBLLLHBBHLLH', buf, 0)
        s = (' version %u flags1 x%x flags2 x%x reserved1 x%x wtFactor %u\n'
             '  wtQuantError %u wtCountMax %u wtLatency %u wtFrequency %u\n'
             '  flags3 x%x speedDeadBand %ureserved2 x%x %x %x' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    flags1 (%s) flags2 (%s)" %
                  (flag_s(u[1], self.cfg_esfwt_flags1),
                   flag_s(u[2], self.cfg_esfwt_flags2)))
        return s

    cfg_esrc_extInt = {
        0: "EXTINT0",
        1: "EXTINT1",
        }

    cfg_esrc_polarity = {
        0: "rising",
        1: "falling",
        }

    cfg_esrc_sourceType = {
        0: "None",
        1: "Frequency",
        2: "Time",
        3: "External",
        }

    cfg_esrc_gnssUtc = {
        0: "GNSS",
        1: "UTC",
        }

    def cfg_esrc(self, buf):
        """UBX-CFG-ESRC decode, External synchronization source
        configuration"""

        # at least protver 16, time firmware only
        if 16 > self.protver:
            self.protver = 16

        u = struct.unpack_from('<BBH', buf, 0)
        if 0 != u[0]:
            return "  Unknown version %u" % u[0]
        if 2 > u[1]:
            return "  Bad numSources %u" % u[1]

        s = "  version %u numSources %u reserved1 x%x\n" % u

        m_len = len(buf)
        if 4 != (m_len - u[1] * 36):
            return "  Bad ilength %u s/b %u" % (m_len, 4 + 36 * u[1])

        for i in range(u[1]):
            u = struct.unpack_from('<BBHLLLLHHlLL', buf, 4 + 36 * i)
            s += "    %d: " % i
            s += ("extInt %u sourceType %u flags x%x freq %u\n"
                  "       reserved2 x%x withTemp %u withAge %u timeToTemp %u\n"
                  "       maxDevLifeTime %u offset %d offsetUncertainty %u "
                  "jitter %u\n" % u)

            if gps.VERB_DECODE <= self.verbosity:
                s += ("         extInt (%s) sourceType (%s) flags (%s,%s)\n"
                      "         freq %.1f Hz\n" %
                      (index_s(u[0], self.cfg_esrc_extInt),
                       index_s(u[1], self.cfg_esrc_sourceType),
                       index_s(u[2] & 1, self.cfg_esrc_polarity),
                       index_s((u[2] >> 1) & 1, self.cfg_esrc_gnssUtc),
                       u[3] / 4.0))

                if 1 == u[1]:
                    s += ("         withTemp %.1f ppb withAge %.1f ppb/y\n"
                          "         timeToTemp %u s maxDevLifeTime %u "
                          "ppg/y\n" %
                          (u[5] / 256.0, u[6] / 256.0, u[7], u[8]))
                if 2 == u[1]:
                    s += ("         offset %d ns offsetUncertainty %u ns "
                          "jitter %u ns/s\n" %
                          u[9:11])

        return s[0:-1]     # remove trailing \n

    def cfg_fixseed(self, buf):
        """UBX-CFG-FIXSEED decode,
Programming the fixed seed for host interface signature"""

        # u-blox 8 only, protVer 18 to 23
        u = struct.unpack_from('<BBHLL', buf, 0)
        s = " version %u length %u reserved1 %u seedHi %u seedLo %u" % u
        # FIXME, partial decode
        return s

    cfg_fxn_flags = {
        2: "sleep Float",
        8: "absAlign",
        0x10: "onOff",
        }

    def cfg_fxn(self, buf):
        """UBX-CFG-FXN decode, FXN FixNOW configuration"""

        # Antaris 4, u-blox 5, protVer 6.00 to 6.02
        u = struct.unpack_from('<LLLLLLLLL', buf, 0)
        s = ("  flags x%x tReacq %u tAcq %u tReacqOff %u\n"
             "  tAcqOff %u tOn %u tOff %u res %u baseTow %u\n" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    flags (%s)" %
                  (flag_s(u[0], self.cfg_fxn_flags)))
        return s

    def cfg_geofence(self, buf):
        """UBX-CFG-GEOFENCE decode, Geofencing configuration

Deprecated in protVer 23.01
"""

        # not in M10, protVer 34 and up

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = (" version %u numFences %u confLvl %u reserved1 %u\n"
             " pioEnabled %u pinPolarity %u pin %u reserved2 %u" % u)
        for i in range(0, u[1]):
            u = struct.unpack_from('<llL', buf, 8 + (i * 12))
            s = "\n   lat %d lon %d radius %d" % u
        return s

    # signals defined in protver 27+
    # signals used in protver 15+
    # top byte used, but not defined
    cfg_gnss_sig = {
        0: {0x010000: "L1C/A",    # GPS
            0x100000: "L2C",
            0x200000: "L5"},
        1: {0x010000: "L1C/A"},   # SBAS
        2: {0x010000: "E1",       # Galileo
            0x100000: "E5a",
            0x200000: "E5b"},
        3: {0x010000: "B1I",      # BeiDou
            0x100000: "B2I",
            0x800000: "B2A"},
        4: {0x010000: "L1"},      # IMES
        5: {0x010000: "L1C/A",    # QZSS
            0x040000: "L1S",
            0x100000: "L2C",
            0x200000: "L5"},
        6: {0x010000: "L1",       # GLONASS
            0x100000: "L2"},
        }

    def cfg_gnss(self, buf):
        """UBX-CFG-GNSS decode, GNSS system configuration

Present in protVer 15 and up
Deprecated in protVer 23.01
"""

        u = struct.unpack_from('<BBBB', buf, 0)
        s = " msgVer %u  numTrkChHw %u numTrkChUse %u numConfigBlocks %u" % u

        for i in range(0, u[3]):
            u = struct.unpack_from('<BBBBL', buf, 4 + (i * 8))
            sat = u[0]
            s += ("\n  gnssId %u TrkCh %2u maxTrCh %2u reserved %u "
                  "Flags x%08x\n" % u)

            if 7 > sat:
                s += ("   %s %s " %
                      (index_s(sat, self.gnss_id),
                       flag_s(u[4], self.cfg_gnss_sig[sat])))
            else:
                s += "Unk "

            if u[4] & 0x01:
                s += 'enabled'

        return s

    def cfg_hnr(self, buf):
        """UBX-CFG-HNR decode, High Navigation Rate Settings"""

        u = struct.unpack_from('<BBBb', buf, 0)
        s = " highNavRate %u reserved %u %u %u" % u
        return s

    cfg_inf_protid = {
        0: "UBX",
        1: "NMEA",
        # labels of following values based on u-blox document
        # no. GPS.G3-X-03002-D (Antaris protocol specification)
        2: "RTCM",      # cannot be used to output INF messages
        3: "RAW",       # used by u-center
        12: "User 0",   # used by u-center
        13: "User 1",   # used by u-center
        14: "User 2",   # used by u-center
        15: "User 3",   # used by u-center
        }

    def cfg_inf(self, buf):
        """UBX-CFG-INF decode, Poll configuration for one protocol

Deprecated in protVer 23.01
"""

        m_len = len(buf)
        if 1 == m_len:
            return ("  Poll request: %s" %
                    index_s(buf[0], self.cfg_inf_protid))

        if 8 > m_len:
            return "  Bad Length %d" % m_len

        # very old u-blox have 8 octets, e.g. Antaris 4 w/ protVer 11
        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = (" protocolId %u reserved1 %u %u %u\n"
             " infMsgMask %u %u %u %u" % u)

        # newer u-blox have 10 octets, e.g. u-blox 6 w/ protVer 13
        if 9 < m_len:
            u = struct.unpack_from('<BB', buf[8:], 0)
            s += (" %u %u" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n   protocolId (%s)' %
                  index_s(buf[0], self.cfg_inf_protid))

        return s

    cfg_itfm_config = {
        0x80000000: "enable",
        }

    cfg_itfm_config2 = {
        0x4000: "enable2",
        }

    cfg_itfm_ant = {
        1: "passive",
        2: "active",
        }

    def cfg_itfm(self, buf):
        """UBX-CFG-ITFM decode, Jamming/Interference Monitor configuration

Deprecated in protVer 23.01
"""

        u = struct.unpack_from('<LL', buf, 0)
        s = " config x%x config2 x%x" % u
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   config (%s) bbThreshold %u cwThreshold %u "
                  "algorithmBits x%x"
                  "\n   config2 (%s) generalBits x%x antSetting (%s)" %
                  (flag_s(buf[0], self.cfg_itfm_config),
                   u[0] & 0x0f, (u[0] >> 4) & 0x1f, (u[0] >> 9) & 0x1fffff,
                   flag_s(buf[1], self.cfg_itfm_config2),
                   u[1] & 0x0fff,
                   index_s((u[1] >> 12) & 3, self.cfg_itfm_config2)))
        return s

    cfg_logfilter_flags = {
        1: "recordEnabled",
        2: "psmOncePerWakupEnabled",
        4: "applyAllFilterSettings",
        }

    def cfg_logfilter(self, buf):
        """UBX-CFG-LOGFILTER decode, Data Logger Configuration

Deprecated in protVer 23.01
"""

        # u-blox 7+, protVer 14+
        u = struct.unpack_from('<BBHHHL', buf, 0)
        s = (" version %u flags x%x minInterval %u timeThreshold %u\n"
             " speedThreshold %u positionThreshold %u" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   flags (%s)" %
                  (flag_s(buf[1], self.cfg_logfilter_flags)))

        return s

    utc_std = {
        0: "Auto",
        1: "CRL",
        2: "NIST",
        3: "USNO",
        4: "BIPM",
        5: "EU",
        6: "SU",
        7: "NTSC",
        8: "NPLI",     # India
        }

    cfg_nav5_dyn = {
        0: "Portable",
        2: "Stationary",
        3: "Pedestrian",
        4: "Automotive",
        5: "Sea",
        6: "Airborne with <1g Acceleration",
        7: "Airborne with <2g Acceleration",
        8: "Airborne with <4g Acceleration",
        }

    cfg_nav5_fix = {
        1: "2D only",
        2: "3D only",
        3: "Auto 2D/3D",
        }

    cfg_nav5_mask = {
        1: "dyn",
        2: "minEl",
        4: "posFixMode",
        8: "drLim",
        0x10: "posMask",
        0x20: "timeMask",
        0x40: "staticHoldMask",
        0x80: "dgpsMask",
        0x100: "cnoThreshold",
        0x400: "utc",
        }

    def cfg_msg(self, buf):
        """UBX-CFG-MSG decode

Deprecated in protVer 23.01
"""
        m_len = len(buf)
        if 2 == m_len:
            u = struct.unpack_from('<BB', buf, 0)
            return '  Rate request %s' % self.class_id_s(u[0], u[1])

        if 3 == m_len:
            u = struct.unpack_from('<BBB', buf, 0)
            return ('  Rate set %s Rate %d' %
                    (self.class_id_s(u[0], u[1]), u[2]))

        if 8 != m_len:
            return "  Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = (' %s Rates %u %u %u %u %u %u' %
             (self.class_id_s(u[0], u[1]), u[2], u[3], u[4], u[5], u[6], u[7]))
        return s

    cfg_nmea_filter = {
        1: "posFilt",
        2: "mskPosFilt",
        4: "timeFilt",
        8: "dateFilt",
        0x10: "gpsOnlyFilter",
        0x20: "trackFilt",
        }

    cfg_nmea_ver = {
        0x21: "2.1",
        0x23: "2.3",
        0x40: "4.0",
        0x41: "4.10",
        0x4b: "4.11",
        }

    cfg_nmea_flags = {
        1: "compat",
        2: "consider",
        4: "limit82",
        8: "highPrec",
        }

    cfg_nmea_svn = {
        0: "Strict",
        1: "Extended",
        }

    cfg_nmea_mtid = {
        0: "Default",
        1: "GP",
        2: "GL",
        3: "GN",
        4: "GA",
        5: "GB",
        6: "GQ",
        }

    cfg_nmea_gtid = {
        0: "GNSS Specific",
        1: "Main",
        }

    cfg_nmea_gnssfilt = {
        1: "gps",
        2: "sbas",
        4: "galileo",
        0x10: "qzss",
        0x20: "glonass",
        0x40: "beidou",
        }

    def cfg_nav5(self, buf):
        """UBX-CFG-NAV5 nav Engine Settings

Deprecated in protVer 23.01
"""

        u = struct.unpack_from('<HBBlLbBHHHHbbbbHHbBL', buf, 0)
        s = (' mask %#x dynModel %u fixmode %d fixedAlt %d FixedAltVar %u\n'
             ' minElev %d drLimit %u pDop %u tDop %u pAcc %u tAcc %u\n'
             ' staticHoldThresh %u dgpsTimeOut %u cnoThreshNumSVs %u\n'
             ' cnoThresh %u res %u staticHoldMaxDist %u utcStandard %u\n'
             ' reserved x%x %x' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   dynModel (%s) fixMode (%s) utcStandard (%s)"
                  "\n   mask (%s)" %
                  (index_s(u[1], self.cfg_nav5_dyn),
                   index_s(u[2], self.cfg_nav5_fix),
                   index_s(u[17] >> 4, self.utc_std),
                   flag_s(u[0] >> 4, self.cfg_nav5_mask)))
        return s

    cfg_navx5_mask1 = {
        4: "minMax",
        8: "minCno",
        0x40: "initial3dfix",
        0x200: "wknRoll",
        0x400: "ackAid",
        0x2000: "ppp",
        0x4000: "aop",
        }

    cfg_navx5_mask2 = {
        0x40: "adr",
        0x80: "sigAttenComp",
        }

    cfg_navx5_aop = {
        1: "useAOP",
        }

    def cfg_navx5(self, buf):
        """UBX-CFG-NAVX5 decode, Navigation Engine Expert Settings

Deprecated in protVer 23.01
"""

        # deprecated protver 23+
        # length == 20 case seems broken?
        m_len = len(buf)

        u = struct.unpack_from('<HHLHBBBBBHBH', buf, 0)
        s = (" version %u mask1 x%x mask2 x%x reserved1 %u minSVs %u "
             "maxSVs %u minCNO %u\n"
             " reserved2 %u iniFix3D %u reserved3 %u  ackAiding %u "
             "wknRollover %u" % u)

        # length == 40 in protver 27
        if 40 <= m_len:
            u1 = struct.unpack_from('<BBHHBBHHLHBB', buf, 20)
            s += ("\n sigAttenCompMode %u reserved456 %u %u %u usePPP %u "
                  "aopCfg %u reserved7 %u"
                  "\n aopOrbMaxErr %u reserved89 %u %u %u useAdr %u" % u1)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   mask1 (%s)"
                  "\n   mask2 (%s) aopCfg (%s)" %
                  (flag_s(u[1], self.cfg_navx5_mask1),
                   flag_s(u[2], self.cfg_navx5_mask2),
                   flag_s(u[5], self.cfg_navx5_aop)))

        return s

    def cfg_nmea(self, buf):
        """UBX-CFG-NMEA decode, NMEA protocol configuration

Deprecated in protVer 23.01
"""

        # old u-blox have 4 octets, e.g. u-blox 6 w/ protVer < 14
        # less old u-blox have 12 octets, e.g. u-blox 7 w/ protVer == 14
        # more recent u-blox have 20 octets, e.g. u-blox 8 w/ protVer > 14
        # deprecated in most recent u-blox, e.g. u-blox 9 w/ protVer > 23.01

        u = struct.unpack_from('<BBBB', buf, 0)
        s = (" filter x%x nmeaVersion x%x numSv %u flags x%x " % u)

        if 11 < len(buf):
            u1 = struct.unpack_from('<LBBBB', buf[4:], 0)
            s += ("gnssToFilter x%x\n svNumbering %u"
                  " mainTalkerId %u gsvTalkerId %u version %u" % u1)
            u += u1

        if 19 < len(buf):
            u2 = struct.unpack_from('<BBBBBBBB', buf[12:], 0)
            s += ("\n bdsTalkerId %u %u reserved1 %u %u %u %u %u %u" % u2)
            u += u2

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n  filter (%s) NMEA Version (%s) numSV (%s) flags (%s)" %
                  (flag_s(u[0], self.cfg_nmea_filter),
                   index_s(u[1], self.cfg_nmea_ver),
                   u[2] if 0 != u[2] else "Unlimited",
                   flag_s(u[3], self.cfg_nmea_flags)))

            if 11 < len(buf):
                s += ("\n  gnssToFilter (%s) svNumbering (%s) "
                      "mainTalkerId (%s)"
                      "\n  gsvTalkerId (%s)" %
                      (flag_s(u[4], self.cfg_nmea_gnssfilt),
                       index_s(u[5], self.cfg_nmea_svn),
                       index_s(u[6], self.cfg_nmea_mtid),
                       index_s(u[7], self.cfg_nmea_gtid)))
        return s

    cfg_nvs_mask = {
        0x20000: 'alm',
        0x20000000: 'aop',
        }

    def cfg_nvs(self, buf):
        """UBX-CFG-NVS decode, Clear,
Save and Load non-volatile storage data"""

        # u-blox 6, protVer 7.03
        u = struct.unpack_from('<LLLB', buf, 0)
        s = ('  clearMask: %#x (%s)\n' %
             (u[0], flag_s(u[0], self.cfg_nvs_mask)))
        s += ('  saveMask: %#x (%s)\n' %
              (u[1], flag_s(u[1], self.cfg_nvs_mask)))
        s += ('  loadMask: %#x (%s)\n' %
              (u[2], flag_s(u[2], self.cfg_nvs_mask)))
        s += ('  deviceMask: %#x (%s)\n' %
              (u[3], flag_s(u[3], self.cfg_cfg_dev)))

        return s

    cfg_odo_flags = {
        1: "useODO",
        2: "useCOG",
        4: "useLPVel",
        8: "useLPCog",
        }

    cfg_odo_profile = {
        0: "Running",
        1: "Cycling",
        2: "Swimming",
        3: "Car",
        4: "Custom",
        }

    def cfg_odo(self, buf):
        """UBX-CFG-ODO decode, Odometer, Low-speed COG Engine Settings

Deprecated in protVer 23.01
"""

        u = struct.unpack_from('<BBBBBBBBBBBBBBBBBBBB', buf, 0)
        s = (" version %u reserved1 %u %u %u flags x%x odoCfg x%x\n"
             " reserved2 %u %u %u %u %u %u\n"
             " cagMaxSpeed %u cogMaxPosAcc %u reserved3 %u %u\n"
             " velLpGain %u cogLpGain %u reserved4 %u %u" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   flags (%s) odoCfg (%s)" %
                  (flag_s(u[4], self.cfg_odo_flags),
                   index_s(u[5], self.cfg_odo_profile)))
        return s

    cfg_pm_flags = {
        0x20: "extintWake",
        0x40: "extintBackup",
        0x80: "extintInactive",
        0x400: "waitTimeFix",
        0x800: "updateRTC",
        0x1000: "updateEPH",
        0x10000: "doNotEnterOff",
        }

    cfg_pm_limitPeakCurr = {
        0: "disabled",
        1: "enabled",
        2: "reserved",
        3: "reserved3",
        }

    def cfg_pm(self, buf):
        """UBX-CFG-PM decode, Poswer Management Configuration"""

        # u-blox 5, protVer 6.00 to 6.02

        u = struct.unpack_from('<BBBBLLLLHH', buf, 0)
        s = (" version %u res1 %u res2 %u res3 %u flags x%x updatePeriod %u\n"
             " searchPeriod %u gridOffset %u onTime %u minAcqTime %u\n" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n  flags (%s) extintSel (EXTINT%u) '
                  'limitPeakCurr (%s)' %
                  (flag_s(u[4], self.cfg_pm_flags),
                   (u[4] >> 4) & 1,
                   index_s((u[4] >> 8) & 3, self.cfg_pm_limitPeakCurr)))
        return s

    cfg_pm2_mode = {
        0: "ON/OFF operation (PSMOO)",
        1: "Cyclic tracking operation (PSMCT)",
        2: "reserved",
        3: "reserves3",
        }

    cfg_pm2_optTarget = {
        0: "performance",
        1: "power save",
        }

    def cfg_pm2(self, buf):
        """UBX-CFG-PM2 decode, Extended Power Mode Configuration

Deprecated in protVer 23.01
"""

        # three versions, two lengths
        # "version" 1 is 44 bytes
        # "version" 2 is 48 bytes,protver <= 22
        # "version" 2 is 48 bytes,protver >= 23

        m_len = len(buf)

        # 48 bytes protver 18+

        u = struct.unpack_from('<BBBBLLLLHHLLLLL', buf, 0)
        s = (" version %u reserved1 %u maxStartupStateDur %u reserved2 %u\n"
             " flags x%x updatePeriod %u searchPeriod %u\n"
             " gridOffset %u ontime %u minAcqTime %u\n"
             " reserved3 %u %u %u %u %u" % u)
        if 48 <= m_len:
            u1 = struct.unpack_from('<L', buf, 44)
            s += "\n extintInactivityMs %u" % u1

        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n  flags (%s) extintSel (EXTINT%u)'
                  '\n  limitPeakCurr (%s)'
                  '\n  optTarget (%s) mode (%s)' %
                  (flag_s(u[4], self.cfg_pm_flags),
                   (u[4] >> 4) & 1,
                   index_s((u[4] >> 8) & 3, self.cfg_pm_limitPeakCurr),
                   index_s((u[4] >> 1) & 3, self.cfg_pm2_optTarget,
                           nf="reserved"),
                   index_s((u[4] >> 17) & 3, self.cfg_pm2_mode)))

        return s

    cfg_pms_values = {0: "Full power",
                      1: "Balanced",
                      2: "Interval",
                      3: "Aggressive with 1Hz",
                      4: "Aggressive with 2Hz",
                      5: "Aggressive with 4Hz",
                      0xff: "Invalid"
                      }

    def cfg_pms(self, buf):
        """UBX-CFG-PMS decode, Power Mode Setup

Deprecated in protVer 32.00
"""

        u = struct.unpack_from('<BBHHBB', buf, 0)
        s = (' version %u powerSetupValue %u'
             ' period %u onTime %#x reserved1 %u %u' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n  powerSetupValue (%s)' %
                  index_s(u[1], self.cfg_pms_values))

        return s

    cfg_prt_flags = {
        0x2: 'extendedTxTimeout',
        }

    cfg_prt_proto = {
        0x1: 'UBX',
        0x2: 'NMEA',
        0x4: 'RTCM2',    # not in u-blox 5
        0x20: 'RTCM3',   # protVer 20+
        }

    def cfg_prt(self, buf):
        """UBX-CFG-PRT decode, Port Configuration

Deprecated in protVer 23.01
Still present in 42.02
"""

        m_len = len(buf)

        portid = buf[0]
        idstr = '%u (%s)' % (portid, self.port_ids.get(portid, '?'))

        if 1 == m_len:
            return "  Poll request PortID %s" % idstr

        # Note that this message can contain multiple 20-byte submessages, but
        # only in the send direction, which we don't currently do.
        if 20 > m_len:
            return "  Bad Length %s" % m_len

        u = struct.unpack_from('<BBHLLHHHH', buf, 0)

        s = [' PortID %s reserved1 %u txReady %#x' % (idstr, u[1], u[2])]
        s.append({1: '  mode %#x baudRate %u',
                  2: '  mode %#x baudRate %u',
                  3: '  reserved2 [%u %u]',
                  4: '  mode %#x reserved2 %u',
                  0: '  mode %#x reserved2 %u',
                  }.get(portid, '  ???: %u,%u') % tuple(u[3:5]))
        s.append('  inProtoMask %#x outProtoMask %#x' % tuple(u[5:7]))
        s.append({1: '  flags %#x reserved2 %u',
                  2: '  flags %#x reserved2 %u',
                  3: '  reserved3 %u reserved4 %u',
                  4: '  flags %#x reserved3 %u',
                  0: '  flags %#x reserved3 %u',
                  }.get(portid, '  ??? %u,%u') % tuple(u[7:]))

        if 0 == portid:
            s.append('    slaveAddr %#x' % (u[3] >> 1 & 0x7F))

        s.append('    inProtoMask (%s)\n'
                 '    outProtoMask (%s)' %
                 (flag_s(u[5], self.cfg_prt_proto),
                  flag_s(u[6], self.cfg_prt_proto)))

        if portid in set([1, 2, 4, 0]):
            s.append('    flags (%s)' % flag_s(u[7], self.cfg_prt_flags))

        return '\n'.join(s)

    cfg_pwr_state = {
        0x52554E20: "GNSS running",
        0x53544F50: "GNSS stopped",
        0x42434B50: "Software Backup",
        }

    def cfg_pwr(self, buf):
        """UBX-CFG-PWR decode, Put receiver in a defined power state

Deprecated in protVer 17.00
"""

        u = struct.unpack_from('<BBBBL', buf, 0)
        s = (" version %u reserved %u %u %u state %u" %
             (u + (index_s(u[0], self.cfg_pwr_state),)))
        return s

    cfg_rate_system = {
        0: "UTC",
        1: "GPS",
        2: "GLONASS",
        3: "BeiDou",
        4: "Galileo",
        }

    def cfg_rate(self, buf):
        """UBX-CFG-RATE decode, Navigation/Measurement Rate Settings

Deprecated in protVer 32.00
"""

        u = struct.unpack_from('<HHH', buf, 0)
        s = (" measRate %u navRate %u timeRef %u (%s)" %
             (u + (index_s(u[2], self.cfg_rate_system),)))
        return s

    cfg_rinv_flags = {
        1: "dump",
        2: "binary",
        }

    def cfg_rinv(self, buf):
        """UBX-CFG-RINV decode, Contents of Remote Inventory

Deprecated in protVer 23.01
"""

        # u-blox 5, protVer 6.00 to 6.02
        m_len = len(buf)

        u = struct.unpack_from('<B', buf, 0)
        s = (" flags x%x (%s) data:" %
             (u + (flag_s(u[0], self.cfg_rinv_flags),)))
        for i in range(0, m_len - 1):
            if 0 == (i % 8):
                s += "\n   "
            u = struct.unpack_from('<B', buf, i + 1)
            s += " %3u" % u

        return s

    cfg_rst_navBbr = {
        0: "Hot Start",
        1: "Warm Start",
        0xffff: "Cold Start",
        }

    cfg_rst_navBbr1 = {
        1: "eph",
        2: "alm",
        4: "health",
        8: "klob",
        0x10: "pos",
        0x20: "clkd",
        0x40: "osc",
        0x80: "utc",
        0x100: "rtc",
        0x800: "sfdr",
        0x1000: "vmon",
        0x2000: "tct",
        0x8000: "aop",
        }

    cfg_rst_resetMode = {
        0: "Hardware reset (watchdog)",
        1: "Software reset",
        2: "Software reset (GNSS only)",
        4: "Hardware reset, after shutdown",
        8: "Controlled GNSS stop",
        9: "Controlled GNSS start",
        0x0a: "Hardware reset (PWSEQ)",
        }

    def cfg_rst(self, buf):
        """UBX-CFG-RST decode, Reset Receiver/Clear Backup Data Structures

protVer 15 and up
"""

        u = struct.unpack_from('<HBB', buf, 0)
        s = ' navBbrmask x%x resetMode %u reserved %u' % u

        if gps.VERB_DECODE <= self.verbosity:
            # fun, two different ways to decode...
            s1 = index_s(u[0], self.cfg_rst_navBbr, nf="")
            if not s1:
                s1 = flag_s(u[0], self.cfg_rst_navBbr1)

            s += ("\n   resetMode (%s)"
                  "\n   navBbrMask (%s)" %
                  (index_s(u[1], self.cfg_rst_resetMode),
                   s1))

        return s

    cfg_rxm_lpMode = {
        0: "Continuous Mode",
        1: "Power Save Mode",
        4: "Continuous Mode",
        }

    def cfg_rxm(self, buf):
        """UBX-CFG-RXM decode, Navigation/Measurement

Deprecated in protVer 32.00
"""

        u = struct.unpack_from('<BB', buf, 0)
        s = (" reserved1 %u lpMode %u (%s)" %
             (u + (index_s(u[1], self.cfg_rxm_lpMode),)))
        return s

    cfg_sbas_mode = {
        1: "enabled",
        2: "test",
        }

    cfg_sbas_usage = {
        1: "range",
        2: "diffCorr",
        3: "integrity",
        }

    cfg_sbas_scanmode1 = {
        1: "PRN120",
        2: "PRN121",
        4: "PRN122",
        8: "PRN123",
        0x10: "PRN124",
        0x20: "PRN125",
        0x40: "PRN126",
        0x80: "PRN127",
        0x100: "PRN128",
        0x200: "PRN129",
        0x400: "PRN130",
        0x800: "PRN131",
        0x1000: "PRN132",
        0x2000: "PRN133",
        0x4000: "PRN134",
        0x8000: "PRN135",
        0x10000: "PRN136",
        0x20000: "PRN137",
        0x40000: "PRN138",
        0x80000: "PRN139",
        0x100000: "PRN140",
        0x200000: "PRN141",
        0x400000: "PRN142",
        0x800000: "PRN143",
        0x1000000: "PRN144",
        0x2000000: "PRN145",
        0x4000000: "PRN146",
        0x8000000: "PRN147",
        0x10000000: "PRN148",
        0x20000000: "PRN149",
        0x40000000: "PRN150",
        0x80000000: "PRN151",
        }

    cfg_sbas_scanmode2 = {
        1: "PRN152",
        2: "PRN153",
        4: "PRN154",
        8: "PRN155",
        0x10: "PRN156",
        0x20: "PRN157",
        0x40: "PRN158",
        }

    def cfg_sbas(self, buf):
        """UBX-CFG-SBAS decode, SBAS Configuration

Deprecated in protVer 32.00
"""

        u = struct.unpack_from('<BBBBL', buf, 0)
        s = (" mode x%x usage x%x maxSBAS %u scanMode2 x%x"
             " scanMode1: x%x" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   mode (%s) usage (%s) scanmode2 (%s)"
                  "\n   scanmode1 (%s)" %
                  (flag_s(u[0], self.cfg_sbas_mode),
                   flag_s(u[1], self.cfg_sbas_usage),
                   flag_s(u[3], self.cfg_sbas_scanmode2),
                   flag_s(u[4], self.cfg_sbas_scanmode1)))

        return s

    # UBX-CFG-SENIF, protVer 19 and up, ADR and UDR only

    cfg_slas_mode = {
        1: "enabled",
        2: "test",
        4: "raim",
        }

    def cfg_slas(self, buf):
        """UBX-CFG-SLAS decode, SLAS configuration"""

        # protVer 19.2 (ADR, UDR only)

        u = struct.unpack_from('<BBH', buf, 0)
        s = " mode %u reserved1 %u %u" % u

        if gps.VERB_DECODE <= self.verbosity:
            s += "\n   mode (%s)" % (flag_s(u[0], self.cfg_slas_mode))

        return s

    cfg_smgr_messageCfg = {
        1: "measInternal",
        2: "measGNSS",
        4: "measEXTINT0",
        8: "measEXTINT1",
        }

    def cfg_smgr(self, buf):
        """UBX-CFG-SMGR decode, Synchronization manager configuration"""

        u = struct.unpack_from('<BBHHBBHHHHL', buf, 0)
        s = (" version %u minGNSSFix %u maxFreqChangeRate %u "
             "maxPhaseCorrR %u\n"
             " reserved1 %u %u freqTolerance %u timeTolerance %u "
             "messageCfg x%x\n"
             " maxSlewRate %u flags x%x" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   messageCfg (%s)" %
                  (flag_s(u[7], self.cfg_smgr_messageCfg)))

        return s

    cfg_tmode_timeMode = {
        0: "Disabled",
        1: "Survey In",
        2: "Fixed Mode",
        }

    def cfg_tmode(self, buf):
        """UBX-CFG-TMODE decode, Time Mode Settings

Deprecated in protVer 13.00
"""

        # u-blox 5, protVer 5.00 to 6.02 (timing feature only)

        u = struct.unpack_from('<BBHlllLLL', buf, 0)
        s = ('  timeMode %u fixedPosX %d fixedPosY %d fixedPosZ %d\n'
             '  fixedPosVar %u svinMinDur %u svinAccLimit %u' % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    timeMode (%s)" %
                  (index_s(u[0], self.cfg_tmode_timeMode)))

        return s

    cfg_tmode2_flags = {
        1: "lla",
        2: "altInv",
        }

    def cfg_tmode2(self, buf):
        """UBX-CFG-TMODE2 decode, Time Mode Settings 2"""

        # u-blox 6, 7 and 8 (timing only)
        # not in u-blox 9
        # protver 13+

        u = struct.unpack_from('<BBHlllLLL', buf, 0)
        s = ('  timeMode %u reserved1 %u usage %#x\n'
             '  ecefXOrLat %d ecefYOrLon %d ecefZOrAlt %d\n'
             '  fixedPosAcc %u svinMinDur %u svinAccLimit %u' % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    timeMode (%s) flags (%s)" %
                  (index_s(u[0], self.cfg_tmode_timeMode),
                   flag_s(u[2], self.cfg_tmode2_flags)))

        return s

    cfg_tmode3_flags = {
        0x100: "lla",
        }

    def cfg_tmode3(self, buf):
        """UBX-CFG-TMODE3 decode, Time Mode Settings 3"""

        # in M8 HP only, protver 20 to 23
        # Not in u-blox 7-, HP only
        # undocumented, but present in ZED-F9T
        # documented in ZED-F9P, protver 27
        # deprecated in 23.01
        u = struct.unpack_from('<BBHlllbbbBLLLLL', buf, 0)
        s = ('  version %u reserved1 %u flags x%x\n'
             '  ecefXOrLat %d ecefYOrLon %d ecefZOrAlt %d\n'
             '  ecefXOrLatHP %d ecefYOrLonHP %d ecefZOrAltHP %d\n'
             '  reserved2 %u fixedPosAcc %u svinMinDur %u svinAccLimit %u\n'
             '  reserved3 %u %u' % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    flags (%s %s)" %
                  (index_s(u[2], self.cfg_tmode_timeMode),
                   flag_s(u[2] & 0x100, self.cfg_tmode3_flags)))

        return s

    # status + 1
    cfg_tp_status = {
        0: "negative",
        1: "off",
        2: "positive",
        }

    cfg_tp_timeRef = {
        0: "UTC",
        1: "GPS",
        2: "Local",
        }

    cfg_tp_flags = {
        0: "synced only",
        1: "always",
        }

    def cfg_tp(self, buf):
        """UBX-CFG-TP decode, Time Pulse Settings"""

        # protVer 4.00 to 6.02

        u = struct.unpack_from('<LLbBBBhhl', buf, 0)
        s = ('  interval %u length %u status %d timeRef %u flags x%x res x%x\n'
             '  antennaCableDelay %u rfGroupDelay %d userDelay %d' % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   status (%s) timeRef (%s) flags (%s)" %
                  (index_s(u[2] + 1, self.cfg_tp_status),
                   index_s(u[3], self.cfg_tp_timeRef),
                   index_s(u[4], self.cfg_tp_flags)))

        return s

    cfg_tp5_flags = {
        1: "Active",
        2: "lockGnssFreq",
        4: "lockedOtherSet",
        8: "isFreq",
        0x10: "isLength",
        0x20: "alignToTow",
        0x40: "RisingEdge",
        }

    cfg_tp5_grid = {
        0: 'UTC',
        1: 'GPS',
        2: 'Glonass',
        3: 'BeiDou',
        4: 'Galileo',
        }

    def cfg_tp5(self, buf):
        """UBX-CFG-TP5 decode, Time Pulse Parameters

Deprecated in protVer 27.00
"""

        m_len = len(buf)

        if 1 == m_len:
            return "  Poll request tpIdx %d" % buf[0]

        if 32 > m_len:
            return "  Bad Length %s" % m_len

        u = struct.unpack_from('<BBHhhLLLLlL', buf, 0)
        s = ("  tpIdx %u version %u reserved1 %u\n"
             "  antCableDelay %d rfGroupDelay %d freqPeriod %u "
             "freqPeriodLock %u\n"
             "  pulseLenRatio %u pulseLenRatioLock %u userConfigDelay %d\n"
             "  flags x%x" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    flags (%s)"
                  "\n    gridToGps (%s) syncMode %d" %
                  (flag_s(u[10] & 0x7f, self.cfg_tp5_flags),
                   index_s((u[10] >> 7) & 0x0f, self.cfg_tp5_grid),
                   (u[10] >> 11) & 0x03))

        return s

    cfg_usb_flags = {1: "reEnum "}

    cfg_usb_powerMode = {0: "self-powered",
                         2: "bus-powered",
                         }

    def cfg_usb(self, buf):
        """UBX-CFG-USB decode, USB Configuration

Only for models with built in USB.
Deprecated in protVer 23.01
"""

        u = struct.unpack_from('<HHHHHH', buf, 0)
        s = ('  vendorID %#x productID %#x reserved1 %u reserved2 %u\n'
             '  powerConsumption %u mA flags %#x' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    flags (%s%s)" %
                  (flag_s(u[5] & 1, self.cfg_usb_flags),
                   index_s(u[5] & 2, self.cfg_usb_powerMode)))

        s += ('\n  vendorString %s\n'
              '  productString %s\n'
              '  serialNumber %s' %
              (gps.polystr(buf[12:43]).rstrip('\0'),
               gps.polystr(buf[44:75]).rstrip('\0'),
               gps.polystr(buf[76:107]).rstrip('\0')))
        return s

    cfg_valdel_layers = {
        1: 'ram',
        2: 'bbr',
        4: 'flash',
        }

    cfg_valget_layers = {
        0: 'ram',
        1: 'bbr',
        2: 'flash',
        7: 'default',
        }

    cfg_valxxx_size = {
        1: "one bit",
        2: "one byte",
        3: "two bytes",
        4: "four bytes",
        5: "eight bytes",
        }

    cfg_valxxx_trans = {
        0: "Transactionless",
        1: "(Re)start Transaction",
        2: "Continue Transaction",
        3: "Apply and end Transaction",
        }

    def cfg_valdel(self, buf):
        """"UBX-CFG-VALDEL decode, Delete configuration items"""
        m_len = len(buf)

        # this is a poll options, so does not set min protver

        u = struct.unpack_from('<BBBB', buf, 0)
        s = ' version %u layer %#x transaction %#x reserved %u\n' % u
        s += ('  layers (%s) transaction (%s)' %
              (flag_s(u[1], self.cfg_valdel_layers),
               index_s(u[2], self.cfg_valxxx_trans)))

        m_len -= 4
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<L', buf, 4 + i * 4)
            item = self.cfg_by_key(u[0])
            s += ('\n    item: %s/%#x' % (item[0], u[0]))
            m_len -= 4
            i += 1
        return s

    def cfg_valget(self, buf):
        """"UBX-CFG-VALGET decode, Get configuration items"""
        m_len = len(buf)

        # version zero is a poll
        # version one is the response

        u = struct.unpack_from('<BBH', buf, 0)
        s = ' version %u layer %u position %u\n' % u
        s += '  layers (%s)' % index_s(u[1], self.cfg_valget_layers)
        m_len -= 4

        i = 0

        if 0 == u[0]:
            # this is a poll option, so does not set min protver
            while 0 < m_len:
                u = struct.unpack_from('<L', buf, 4 + i * 4)
                item = self.cfg_by_key(u[0])
                s += ('\n    item %s/%#x' % (item[0], u[0]))
                if gps.VERB_DECODE <= self.verbosity:
                    size = (u[0] >> 28) & 0x07
                    s += ('\n      size %s(%u) type %s scale %f'
                          ' \n      %s' %
                          (index_s(size, self.cfg_valxxx_size),
                           size, item[2], item[3], item[5]))
                m_len -= 4
                i += 1
        else:
            # answer to poll
            # we are at least protver 27
            if 27 > self.protver:
                self.protver = 27

            # sort of duplicated in cfg_valset()
            i += 4
            count = 0
            while 4 < m_len:
                count += 1
                u = struct.unpack_from('<L', buf, i)
                m_len -= 4
                i += 4
                item = self.cfg_by_key(u[0])
                cfg_type = self.item_to_type(item)
                size = cfg_type[0]
                if size > m_len:
                    s += "\nWARNING: not enough bytes!"
                    break

                frmat = cfg_type[1]
                # flavor = cfg_type[2]  UNUSED
                v = struct.unpack_from(frmat, buf, i)
                s += '\n    item %s/%#x val %s' % (item[0], u[0], v[0])
                if gps.VERB_DECODE <= self.verbosity:
                    rsize = (u[0] >> 28) & 0x07
                    s += ('\n      size %s(%u) type %s scale %f'
                          ' \n      %s' %
                          (index_s(rsize, self.cfg_valxxx_size),
                           rsize, item[2], item[3], item[5]))
                m_len -= size
                i += size

            if gps.VERB_DECODE <= self.verbosity:
                s += '\n   count %d' % (count)

            if 0 < m_len:
                s += "\nWARNING: %d extra bytes!" % m_len

        return s

    def cfg_valset(self, buf):
        """"UBX-CFG-VALSET decode, Set configuration items"""
        m_len = len(buf)

        # this is a poll option, so does not set min protver

        u = struct.unpack_from('<BBBB', buf, 0)
        s = ' version %u layer %#x transaction %#x reserved %u\n' % u
        s += ('  layers (%s) transaction (%s)' %
              (flag_s(u[1], self.cfg_valdel_layers),
               index_s(u[2], self.cfg_valxxx_trans)))

        # sort of duplicated in cfg_valget()
        m_len -= 4
        i = 4
        while 4 < m_len:
            u = struct.unpack_from('<L', buf, i)
            m_len -= 4
            i += 4
            item = self.cfg_by_key(u[0])
            cfg_type = self.item_to_type(item)

            size = cfg_type[0]
            # FIXME!  should check for enough bytes to unpack from
            frmat = cfg_type[1]
            # flavor = cfg_type[2]  UNUSED
            v = struct.unpack_from(frmat, buf, i)
            s += ('\n    item %s/%#x val %s' % (item[0], u[0], v[0]))
            m_len -= size
            i += size

        if 0 < m_len:
            s += "\nWARNING: %d extra bytes!" % m_len

        return s

    # Broadcom calls this BRM-STP-
    cfg_ids = {
        # in u-blox 5+
        # Broadcom calls this BRM-STP-PORT
        0x00: {'str': 'PRT', 'dec': cfg_prt, 'minlen': 1,
               'name': 'UBX-CFG-PRT', 'depver': 23.01},
        # in u-blox 5+
        # Broadcom calls this BRM-STP-MESSAGE
        0x01: {'str': 'MSG', 'dec': cfg_msg, 'minlen': 2,
               'name': 'UBX-CFG-MSG', 'depver': 23.01},
        # in u-blox 5+
        0x02: {'str': 'INF', 'dec': cfg_inf, 'minlen': 1,
               'name': 'UBX-CFG-INF', 'depver': 23.01},
        # in u-blox 5+
        # Broadcom calls this BRM-STP-RESET
        0x04: {'str': 'RST', 'dec': cfg_rst, 'minlen': 4,
               'name': 'UBX-CFG-RST'},
        # in u-blox 5 to 9
        0x06: {'str': 'DAT', 'dec': cfg_dat, 'minlen': 2,
               'name': 'UBX-CFG-DAT', 'depver': 23.01},
        # u-blox 5, 6.  Not in u-blox 7+
        0x07: {'str': 'TP', 'dec': cfg_tp, 'minlen': 20,
               'name': 'UBX-CFG-TP'},
        # in u-blox 5+
        0x08: {'str': 'RATE', 'dec': cfg_rate, 'minlen': 6,
               'name': 'UBX-CFG-RATE', 'depver': 23.01},
        # in u-blox 5+
        # Broadcom calls this BRM-STP-CONFIG
        0x09: {'str': 'CFG', 'dec': cfg_cfg, 'minlen': 12,
               'name': 'UBX-CFG-CFG', 'depver': 28.0},
        # Antaris 4, deprecated in u-blox 5/6, gone in 7
        0x0e: {'str': 'FXM', 'dec': cfg_fxn, 'minlen': 36,
               'name': 'UBX-CFG-FXM'},
        # u-blox 5, 6, 7, 8
        0x11: {'str': 'RXM', 'dec': cfg_rxm, 'minlen': 2,
               'name': 'UBX-CFG-RXM'},
        # in u-blox 6, SFDR only.  Not in 5- or 7+
        0x12: {'str': 'EKF', 'minlen': 16, 'name': 'UBX-CFG-EKF'},
        # in u-blox 5+
        0x13: {'str': 'ANT', 'dec': cfg_ant, 'minlen': 4,
               'name': 'UBX-CFG-ANT', 'depver': 23.01},
        # in u-blox 5+
        0x16: {'str': 'SBAS', 'dec': cfg_sbas, 'minlen': 8,
               'name': 'UBX-CFG-SBAS'},
        # in u-blox 5+
        # Broadcom calls this BRM-STP-NMEA
        0x17: {'str': 'NMEA', 'dec': cfg_nmea, 'minlen': 4,
               'name': 'UBX-CFG-NMEA', 'depver': 23.01},
        # in u-blox 6+, Not in u-blox 5-
        0x1b: {'str': 'USB', 'dec': cfg_usb, 'minlen': 108,
               'name': 'UBX-CFG-USB', 'depver': 23.01},
        # u-blox 5 and 6. Timing  only.  Not in u-blox 7+
        0x1d: {'str': 'TMODE', 'dec': cfg_tmode, 'minlen': 28,
               'name': 'UBX-CFG-TMODE', 'depver': 13.0},
        # in u-blox 8+. Not in u-blox 7-
        0x1e: {'str': 'ODO', 'dec': cfg_odo, 'minlen': 20,
               'name': 'UBX-CFG-ODO', 'depver': 23.01},
        # in u-blox 6, not in u-blox 7+
        0x22: {'str': 'NVS', 'dec': cfg_nvs, 'minlen': 13,
               'name': 'UBX-CFG-NVS'},
        # in u-blox 5+
        0x23: {'str': 'NAVX5', 'dec': cfg_navx5, 'minlen': 20,
               'name': 'UBX-CFG-NAVX5', 'depver': 23.01},
        # in u-blox 6+.  Not in 5-
        # Broadcom calls this BRM-STP-CONFIG2
        0x24: {'str': 'NAV5', 'dec': cfg_nav5, 'minlen': 36,
               'name': 'UBX-CFG-NAV5', 'depver': 23.01},
        # in u-blox 6. SFDR only. Not in u-blox 5- or 7+
        0x29: {'str': 'ESFGWT', 'minlen': 44, 'name': 'UBX-CFG-ESFGWT'},
        # in u-blox 6+, Not u-blox 5-
        0x31: {'str': 'TP5', 'dec': cfg_tp5, 'minlen': 1,
               'name': 'UBX-CFG-TP5', 'depver': 27.0},
        # In u-blox 5, 6. Not in u-blox 7+
        0x32: {'str': 'PM', 'dec': cfg_pm, 'minlen': 24,
               'name': 'UBX-CFG-PM'},
        # in u-blox 5+
        0x34: {'str': 'RINV', 'dec': cfg_rinv, 'minlen': 1,
               'name': 'UBX-CFG-RINV', 'depver': 23.01},
        # in u-blox 6+.  Not in u-blox 5-
        0x39: {'str': 'ITFM', 'dec': cfg_itfm, 'minlen': 8,
               'name': 'UBX-CFG-ITFM', 'depver': 23.01},
        # in u-blox 6+.  Not in u-blox 5-
        0x3b: {'str': 'PM2', 'dec': cfg_pm2, 'minlen': 44,
               'name': 'UBX-CFG-PM2', 'depver': 23.01},
        # in u-blox 6 and 7.  Not in u-blox 5- or 8+
        0x3d: {'str': 'TMODE2', 'dec': cfg_tmode2, 'minlen': 28,
               'name': 'UBX-CFG-TMODE2'},
        # in u-blox 7+  Not in u-blox 6-
        # Broadcom calls this BRM-STP-ME_SETTINGS
        0x3e: {'str': 'GNSS', 'dec': cfg_gnss, 'minlen': 4,
               'name': 'UBX-CFG-GNSS', 'depver': 23.01},
        # u-blox F9 MDR/TIM, set only, partially undocumented.
        0x41: {'str': 'OTP', 'minlen': 12, 'name': 'UBX-CFG-OTP'},
        # in u-blox 7+  Not in u-blox 6-
        0x47: {'str': 'LOGFILTER', 'dec': cfg_logfilter, 'minlen': 12,
               'name': 'UBX-CFG-LOGFILTER', 'depver': 23.01},
        # protVer 19 and up, UDR only
        0x4c: {'str': 'ESFA', 'dec': cfg_esfa, 'minlen': 20,
               'name': 'UBX-CFG-ESFA'},
        # protVer 19 and up, UDR only
        0x4d: {'str': 'ESFG', 'dec': cfg_esfg, 'minlen': 20,
               'name': 'UBX-CFG-ESFG'},
        # Not in u-blox 7-, FTS only
        0x53: {'str': 'TXSLOT', 'minlen': 2, 'name': 'UBX-CFG-TXSLOT'},
        # protVer 15.01 and up, ADR and UDR only
        0x56: {'str': 'ESFALG', 'dec': cfg_esfalg,  'minlen': 12,
               'name': 'UBX-CFG-ESFALG'},
        # Not in u-blox 7-
        0x57: {'str': 'PWR', 'dec': cfg_pwr, 'minlen': 8,
               'name': 'UBX-CFG-PWR', 'depver': 17.0},
        # Not before u-blox 8, protVer 15, ADR and UDR only
        0x5c: {'str': 'HNR', 'dec': cfg_hnr, 'minlen': 4,
               'name': 'UBX-CFG-HNR'},
        # Not in u-blox 7-, protVer 16 and up,  TFS only
        0x60: {'str': 'ESRC', 'dec': cfg_esrc, 'minlen': 4,
               'name': 'UBX-CFG-ESRC'},
        # Not in u-blox 8-
        0x61: {'str': 'DOSC', 'dec': cfg_dosc, 'minlen': 4,
               'name': 'UBX-CFG-DOSC'},
        # Not in u-blox 8-
        0x62: {'str': 'SMGR', 'dec': cfg_smgr, 'minlen': 20,
               'name': 'UBX-CFG-SMGR'},
        # protVer 15.01 and up, ADR and UDR only
        0x64: {'str': 'SPT', 'minlen': 12, 'name': 'UBX-CFG-SPT'},
        # Not in u-blox 8-
        0x69: {'str': 'GEOFENCE', 'dec': cfg_geofence, 'minlen': 8,
               'name': 'UBX-CFG-GEOFENCE', 'depver': 23.01},
        # Not in u-blox 8-
        0x70: {'str': 'DGNSS', 'dec': cfg_dgnss, 'minlen': 4,
               'name': 'UBX-CFG-DGNSS'},
        # Not in u-blox 7-, HP only
        # undocumented, but present in ZED-F9T
        0x71: {'str': 'TMODE3', 'dec': cfg_tmode3, 'minlen': 40,
               'name': 'UBX-CFG-TMODE3'},
        # protVer 15.01 and up, ADR only
        0x82: {'str': 'ESFWT', 'dec': cfg_esfwt, 'minlen': 32,
               'name': 'UBX-CFG-ESFWT'},
        # Not in u-blox 7-
        0x84: {'str': 'FIXSEED', 'dec': cfg_fixseed, 'minlen': 12,
               'name': 'UBX-CFG-FIXSEED'},
        # Not in u-blox 7-
        0x85: {'str': 'DYNSEED', 'dec': cfg_dynseed, 'minlen': 12,
               'name': 'UBX-CFG-DYNSEED'},
        # Not in u-blox 8-
        # Broadcom calls this BRM-STP-PWR_MODE
        0x86: {'str': 'PMS', 'dec': cfg_pms, 'minlen': 8,
               'name': 'UBX-CFG-PMS'},
        # protVer 19 and up, ADR and UDR only
        0x88: {'str': 'SENIF', 'minlen': 6, 'name': 'UBX-CFG-SENIF'},
        # in u-blox 9
        0x8a: {'str': 'VALSET', 'dec': cfg_valset, 'minlen': 4,
               'name': 'UBX-CFG-VALSET'},
        # in u-blox 9
        0x8b: {'str': 'VALGET', 'dec': cfg_valget, 'minlen': 4,
               'name': 'UBX-CFG-VALGET'},
        # in u-blox 9
        0x8c: {'str': 'VALDEL', 'dec': cfg_valdel, 'minlen': 4,
               'name': 'UBX-CFG-VALDEL'},
        # u-blox 8
        0x8d: {'str': 'SLAS', 'dec': cfg_slas, 'minlen': 4,
               'name': 'UBX-CFG-SLAS'},
        # only in u-blox 8
        0x93: {'str': 'BATCH', 'dec': cfg_batch, 'minlen': 8,
               'name': 'UBX-CFG-BATCH'},
        # F20 , protVer 50
        0xa4: {'str': 'OTP', 'minlen': 12, 'name': 'UBX-CFG-OTP'},
        # Broadcom calls this BRM-STP-L1L5bias
        # 0xb0:
        }

    esf_raw_type = {
        0: "none",
        1: "reserved",
        2: "reserved",
        3: "reserved",
        4: "reserved",
        5: "gyro z",
        6: "Front Left ticks",
        7: "Front Right ticks",
        8: "Rear Left ticks",
        9: "Rear Right ticks",
        10: "speed tick",
        11: "speed",
        12: "gyro temp",
        13: "gyro y",
        14: "gyro x",
        # no 15 ?
        16: "accel x",
        17: "accel y",
        18: "accel z",
        # 98 ?
        }

    esf_alg_error = {
        1: 'tiltAlgError',
        2: 'yawAlgError',
        4: 'angleError',
    }

    esf_alg_status = {
        0: 'user-defined/fixed angles',
        1: 'roll/pitch angles alignment is ongoing',
        2: 'roll/pitch/yaw angles alignment is ongoing',
        3: 'coarse alignment used',
        4: 'fine alignment used',
    }

    def esf_alg(self, buf):
        """UBX-ESF-ALG decode, IMU alignment information"""

        # at least protver 19
        if 19 > self.protver:
            self.protver = 19

        u = struct.unpack_from('<LBBBBLhh', buf, 0)
        s = (' iTOW %u version %u flags x%x error x%x reserved1 x%x\n'
             '   yaw %u pitch %d roll %d' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   flags (%s) status (%s)"
                  "\n   error (%s)" %
                  ('autoMntAlgOn' if (u[2] & 1) else '',
                   index_s((u[2] >> 1) & 7, self.esf_alg_status),
                   flag_s(u[3], self.esf_alg_error)))
        return s

    esf_status_bitfield0 = {
        0x10: 'xAngRateValid',
        0x20: 'yAngRateValid',
        0x40: 'zAngRateValid',
        0x80: 'xAccelValid',
        0x100: 'yAccelValid',
        0x200: 'zAccelValid',
    }

    def esf_ins(self, buf):
        """UBX-ESF-INS decode, Vehicle dynamics information"""

        # at least protver 19
        if 19 > self.protver:
            self.protver = 19

        u = struct.unpack_from('<LLLllllll', buf, 0)
        s = (' bitfield0 x%x reserved1 x%x iTOW %u\n'
             '   xAngRate %d yAngRate %d zAngRate %d\n'
             '   xAccel %d yAccel %d zAccel %d' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   bitfield0 (version %u %s)" %
                  (u[0] & 0xff,
                   flag_s(u[0] & 0x0ff00, self.esf_status_bitfield0)))
        return s

    esf_meas_flags = {
        1: 'timeMarkSent-on-Ext0',
        2: 'timeMarkSent-on-Ext1',
        4: 'timeMarkEdge-falling',
        8: 'calibTtagValid',
        0x10: 'Unk',
        0x20: 'Unk',
        }

    def esf_meas(self, buf):
        """UBX-ESF-MEAS decode, External sensor fusion measurements"""

        # at least protver 15
        if 15 > self.protver:
            self.protver = 15

        m_len = len(buf)
        blocks = int((m_len - 8) / 4)
        if ((blocks * 4) + 8) != m_len:
            s = ("ERROR: invalid m_len %d blocks %f" %
                 (m_len, (m_len - 8) / 4))
            return s

        u = struct.unpack_from('<LHH', buf, 0)
        s = ' timetag %u flags x%x id %u' % u
        numMeas = (u[1] >> 11) & 0x1f
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   flags (%s) numMeas %u" %
                  (flag_s(u[1] & 0x7ff, self.esf_meas_flags), numMeas))
        if (u[1] & 0x08) and (0 < blocks):
            # calibTtagValid
            blocks -= 1

        if numMeas != blocks:
            s += "\nERROR:  numMeas != blocks!!"
            return s

        n = 0
        s1 = ''
        while n < blocks:
            u1 = struct.unpack_from('<L', buf, 8 + (4 * n))
            data_type = (u1[0] >> 24) & 0x03f
            data = u1[0] & 0x0ffffff
            if data_type in [5, 11, 12, 13, 14, 16, 17, 18]:
                # 24 signed data
                data = uint2int(data, 24)
            if gps.VERB_DECODE <= self.verbosity:
                s1 = ' (%s)' % index_s(data_type, self.esf_raw_type)
            s += ('\n     dataType %3u%s dataField %7d' %
                  (data_type, s1, data))
            n += 1
        if u[1] & 0x08:
            # calibTtagValid
            u1 = struct.unpack_from('<L', buf, 8 + (4 * numMeas))
            s += '\n   calibTtag %u' % u[0]
        return s

    def esf_raw(self, buf):
        """UBX-ESF-RAW decode, raw sensor information"""

        # at least protver 15
        if 15 > self.protver:
            self.protver = 15

        m_len = len(buf)
        blocks = int((m_len - 4) / 8)
        if ((blocks * 8) + 4) != m_len:
            s = ("ERROR: invalid m_len %d blocks %f" %
                 (m_len, (m_len - 4) / 8))
            return s

        u = struct.unpack_from('<L', buf, 0)
        s = ' reserved1 x%x blocks %u' % (u[0], blocks)
        n = 0
        s1 = ''
        while n < blocks:
            u = struct.unpack_from('<LL', buf, 4 + (8 * n))
            data_type = (u[0] >> 24) & 0x0ff
            data = u[0] & 0x0ffffff
            if data_type in [5, 11, 12, 13, 14, 16, 17, 18]:
                # 24 bit signed data
                data = uint2int(data, 24)
            if gps.VERB_DECODE <= self.verbosity:
                s1 = " (%s)" % index_s(data_type, self.esf_raw_type)
            s += ('\n   data_type %3u%s data %8d sTtag %u' %
                  (data_type, s1, data, u[1]))
            n += 1
        return s

    esf_status_fusionMode = {
        0: 'Initialization mode',
        1: 'Fusion mode',
        2: 'Susended fusion mode',
        3: 'Disabled fusion mode',
        }

    def esf_status(self, buf):
        """UBX-ESF-STATUS decode, raw sensor status"""

        # at least protver 15
        if 15 > self.protver:
            self.protver = 15

        m_len = len(buf)
        blocks = int((m_len - 16) / 4)
        if ((blocks * 4) + 16) != m_len:
            s = ("ERROR: invalid m_len %d blocks %f" %
                 (m_len, (m_len - 16) / 4))
            return s

        u = struct.unpack_from('<LBBBBBBBBBHB', buf, 0)
        s = (' iTOW %u version %u reserved1 %x %x %x %x %x %x %x \n'
             '   fusionMode %u reserved2 %x numSens %u ' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n     fusionMode (%s)" %
                  index_s(u[10], self.esf_status_fusionMode))
        n = 0
        while n < blocks:
            u = struct.unpack_from('<BBBB', buf, 16 + (4 * n))
            s += '\n   sensStatus1 %x sensStatus2 %x freq %u faults %u' % u
            if gps.VERB_DECODE <= self.verbosity:
                s += ("\n      type (%s) used %s ready %s" %
                      (index_s(u[0] & 0x3f, self.esf_raw_type),
                       'Yes' if (u[0] & 0x40) else 'No',
                       'Yes' if (u[0] & 0x80) else 'No'))
            n += 1
        return s

    # UBX-ESF-
    # only with ADR or UDR products
    esf_ids = {0x02: {'str': 'MEAS', 'dec': esf_meas, 'minlen': 8,
                      'name': "UBX-ESF-MEAS"},
               0x03: {'str': 'RAW', 'dec': esf_raw, 'minlen': 4,
                      'name': "UBX-ESF-RAW"},
               0x04: {'str': 'CAL', 'minlen': 12,
                      'name': "UBX-ESF-CAL"},
               0x10: {'str': 'STATUS', 'dec': esf_status, 'minlen': 16,
                      'name': "UBX-ESF-STATUS"},
               0x13: {'str': 'RESETALG', 'minlen': 0,
                      'name': "UBX-ESF-RESETALG"},
               0x14: {'str': 'ALG', 'dec': esf_alg, 'minlen': 16,
                      'name': "UBX-ESF-ALG"},
               0x15: {'str': 'INS', 'dec': esf_ins, 'minlen': 16,
                      'name': "UBX-ESF-INS"},
               }

    # UBX-HNR-

    def hnr_att(self, buf):
        """UBX-HNR-ATT decode, HNR Attitude solution

only with ADR or UDR products, protVer 19 and up
removed in protVer 35
"""

        # Not before protVet 19.2
        # 32 bytes long in protver 19.2

        u = struct.unpack_from('<LBBBBlllLLL', buf, 0)
        s = ('  iTOW %u version %u reserved1 x%x %x %x\n'
             '  roll %d pitch %d heading %d\n'
             '  accRoll %u accPitch %u accHeading %u' % u)
        return s

    hnr_ins_bitfield0 = {
        0x100: 'xAngRateValid',
        0x200: 'yAngRateValid',
        0x400: 'zAngRateValid',
        0x800: 'xAccelValid',
        0x1000: 'yAccelValid',
        0x2000: 'zAccelValid',
        }

    def hnr_ins(self, buf):
        """UBX-HNR-INS decode, HNR Vehicle dynamics information

only with ADR or UDR products, protVer 19 and up
removed in protVer 35
"""

        # Not before protVet 19.1
        # 36 bytes long in protver 19.1

        u = struct.unpack_from('<LLLlllLLL', buf, 0)
        s = ('  bitfield0 x%x reserved1 x%x iTOW %u\n'
             '  xAngRate %d yAngRate %d zAngRate %d\n'
             '  xAccel %d zAccel %d zAccel %d' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    version %u"
                  "\n    bitfield0 (%s)" %
                  (u[0] & 0x0ff,
                   flag_s(u[0] & 0xffffff00, self.hnr_ins_bitfield0)))
        return s

    # diff from nav_pvt_flags
    hnr_pvt_flags = {
        1: "GpsFixOK",
        2: "diffSoln",
        4: "WKNSET",
        8: "TOWSET",
        0x10: "headVehValid",
        }

    def hnr_pvt(self, buf):
        """UBX-HNR-PVT decode, High rate output of PVT solution

removed in protVer 35
"""
        m_len = len(buf)

        # Not before protVet 19
        # 72 bytes long in protver 19.

        u = struct.unpack_from('<LHBBBBBBLBBHllllllllLLLLL', buf, 0)
        s = ('  iTOW %u time %u/%u/%u %02u:%02u:%02u valid x%x\n'
             '  nano %d gpsFix %u flags x%x reserved1 x%x\n'
             '  lon %d lat %d height %d hMSL %d\n'
             '  gSpeed %d speed %d headMot %d headVeh %d\n'
             '  hAcc %u vAcc %u sAcc %u headAcc %u reserved2 x%x' % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    valid (%s)"
                  "\n    gpsFix (%s)"
                  "\n    flags (%s)" %
                  (flag_s(u[7], self.nav_pvt_valid),
                   index_s(u[9], self.nav_pvt_fixType),
                   flag_s(u[10], self.hnr_pvt_flags)))
        return s

    # ADR, UDR only, protVer 19 and up
    hnr_ids = {0x00: {'str': 'PVT', 'dec': hnr_pvt, 'minlen': 72,
                      'name': "UBX-HNR-PVT"},
               0x01: {'str': 'ATT', 'dec': hnr_att, 'minlen': 32,
                      'name': "UBX-HNR-ATT"},
               0x02: {'str': 'INS', 'dec': hnr_ins, 'minlen': 36,
                      'name': "UBX-HNR-INS"},
               }

    def inf_debug(self, buf):
        """UBX-INF-DEBUG decode"""
        return ' Debug: ' + gps.polystr(buf)

    def inf_error(self, buf):
        """UBX-INF-ERROR decode"""
        return ' Error: ' + gps.polystr(buf)

    def inf_notice(self, buf):
        """UBX-INF-NOTICE decode"""
        return ' Notice: ' + gps.polystr(buf)

    def inf_test(self, buf):
        """UBX-INF-TET decode"""
        return ' Test: ' + gps.polystr(buf)

    def inf_warning(self, buf):
        """UBX-INF-WARNING decode"""
        return ' Warning: ' + gps.polystr(buf)

    inf_ids = {0x0: {'str': 'ERROR', 'dec': inf_error, 'minlen': 0,
                     'name': 'UBX-INF-ERROR'},
               0x1: {'str': 'WARNING', 'dec': inf_warning, 'minlen': 0,
                     'name': 'UBX-INF-WARNING'},
               0x2: {'str': 'NOTICE', 'dec': inf_notice, 'minlen': 0,
                     'name': 'UBX-INF-NOTICE'},
               0x3: {'str': 'TEST', 'dec': inf_test, 'minlen': 0,
                     'name': 'UBX-INF-TEST'},
               0x4: {'str': 'DEBUG', 'dec': inf_debug, 'minlen': 0,
                     'name': 'UBX-INF-DEBUG'},
               }

    log_batch_contentValid = {
        1: 'extraPvt',
        2: 'extraOdo',
        }

    log_batch_valid = {
        1: 'validDate',
        2: 'validTime',
        }

    log_batch_fixType = {
        0: 'No Fix',
        2: '2D Fix',
        3: '3D Fix',
        4: 'GNSS + DR',   # only UBX-LOG-RETRIEVEPOS
        }

    log_batch_flags = {
        1: 'gnssFixOK',
        2: 'diffSoln',
        }

    log_batch_psmState = {
        0: 'Not Active',
        4: 'Enabled',
        8: 'Acquisition',
        9: 'Tracking',
        0x10: 'Power Optimized Tracking',
        0x11: 'Inactive',
        }

    def log_batch(self, buf):
        """UBX-LOG-BATCH decode
Oddly this is polled with UBX-LOG-RETRIEVEBATCH
"""

        # u-blox 8, protVer 23+
        # in NEO-M9N, protVer 32.00
        u = struct.unpack_from('<BBHLHBBBBBBLlBBBBllllLLlllllLLHHLLLL', buf, 0)
        s = ("  version %u contentValid x%x msgCnt %u iTow %u\n"
             "  year %u month %u day %u hour %u min %u sec %u valid x%x\n"
             "  tAcc %u fracSec %d fixType %u flags x%x flags2 x%x numSV %u\n"
             "  lon %d lat %d height %d hMSL %d\n"
             "  hAcc %u vAcc %u\n"
             "  vel N %u E %u D %u gSpeed %d headMot %d sAcc %u headAcc %u\n"
             "  pdep %u reserved1 x%x distance %u totalDistance %u\n"
             "  distanceStd %u reserved2 x%x" % u)
        if gps.VERB_DECODE <= self.verbosity:
            # flags2 undocumented
            s += ("\n      contentValid (%s) valid (%s)"
                  "\n      fixType (%s)"
                  "\n      flags (%s) psmState (%s)" %
                  (flag_s(u[1], self.log_batch_contentValid),
                   flag_s(u[10], self.log_batch_valid),
                   index_s(u[13], self.log_batch_fixType),
                   flag_s(u[14] & 3, self.log_batch_flags),
                   index_s(u[14] & 0x1c, self.log_batch_psmState)))

        return s

    log_create_logCfg = {
        1: 'circular',
        }

    log_create_logSize = {
        0: 'Maximum safe',
        1: 'Minimum safe',
        2: 'user defined',
        }

    def log_create(self, buf):
        """UBX-LOG-CREATE decode"""

        # u-blox 7+, protVer 14+
        u = struct.unpack_from('<BBBBL', buf, 0)
        s = ("  version %u logCfg x%x reserved1 x%x logSize %u\n"
             "  userDefinedSize %u" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n      logCfg (%s) logSize (%s)" %
                  (flag_s(u[1], self.log_create_logCfg),
                   index_s(u[3], self.log_create_logSize)))

        return s

    def log_erase(self, buf):
        """UBX-LOG-ERASE decode"""

        # u-blox 7+, protVer 14+
        return "  Erase Logged Data"

    log_findtime_type = {
        0: 'request',
        1: 'response',
        }

    def log_findtime(self, buf):
        """UBX-LOG-FINDTIME decode"""

        # u-blox 7+, protVer 14+
        m_len = len(buf)

        # length 8 (version 1, type 1) is response.
        # WTF; length 10 (version 0, type 0) is request
        # length 12 (version 0, type 0) is request
        if 8 == m_len:
            # response
            u = struct.unpack_from('<BBHL', buf, 0)
            s = "  version %u type %u reserved1 x%x entryNumber %u" % u
            if 1 != u[0] or 1 != u[1]:
                s += "\nWARNING: Unknown version, type (%u, %u)" % (u[0], u[1])
        elif 10 == m_len:
            # request
            u = struct.unpack_from('<BBHBBBBBB', buf, 0)
            s = ("  version %u type %u\n"
                 "  year %u month %u day %u hour %u min %u sec %u "
                 "reserved2 x%x" % u)
            if 0 != u[0] or 0 != u[1]:
                s += "\nWARNING: Unknown version, type (%u, %u)" % (u[0], u[1])
        elif 12 == m_len:
            # request
            u = struct.unpack_from('<BBHHBBBBBB', buf, 0)
            s = ("  version %u type %u reserved1 x%x\n"
                 "  year %u month %u day %u hour %u min %u sec %u "
                 "reserved2 x%x" % u)
            if 0 != u[0] or 0 != u[1]:
                s += "\nWARNING: Unknown version, type (%u, %u)" % (u[0], u[1])
        else:
            return "  Bad Length %s" % m_len

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n      type (%s)" %
                  (index_s(u[1], self.log_findtime_type)))

        return s

    log_info_status = {
        8: 'recording',
        0x10: 'inactive',
        0x20: 'circular',
        }

    def log_info(self, buf):
        """UBX-LOG-INFO decode

removed in protVer 35
"""

        # u-blox 7+, protVer 14+
        u = struct.unpack_from('<BBHLLLLLLHBBBBBBHBBBBBBBBH', buf, 0)
        s = ("  version %u reserved1 x%x x%x filestoreCapacity %u "
             "reserved2 x%x x%x\n"
             "  currentMaxLogSize %u currentLogSize %u entryCount %u\n"
             "  oldestYear %u oldestMonth %u oldestDay %u \n"
             "  oldestHour %u oldestMin %u oldestSec %u reserved3 x%x\n"
             "  newestYear %u newestMonth %u newestDay %u \n"
             "  newestHour %u newestMin %u newestSec %u reserved4 x%x\n"
             "  status x%x reserved5 x%x x%x" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n      status (%s)" %
                  (flag_s(u[23], self.log_info_status)))

        return s

    log_retrievebatch_flags = {
        1: 'sendMonFirst',
        }

    def log_retrieve(self, buf):
        """UBX-LOG-RETRIEVE decode"""

        # u-blox 7+, protVer 14+
        u = struct.unpack_from('<LLBBH', buf, 0)
        s = "  startNumber %u entryCount %u version %u reserved1 x%x %x" % u

        return s

    def log_retrievebatch(self, buf):
        """UBX-LOG-RETRIEVEBATCH decode
Oddly this is the poll for UBX-LOG-BATCH
"""

        # u-blox 8, protVer 23.01 to 23.99
        # not in u-blox 7 or 9
        u = struct.unpack_from('<BBH', buf, 0)
        s = ("  version %u flags x%x reserved1 x%x" % u)
        if gps.VERB_DECODE <= self.verbosity:
            # flags2 undocumented
            s += ("\n      flags (%s)" %
                  flag_s(u[1], self.log_retrievebatch_flags))

        return s

    def log_retrievepos(self, buf):
        """UBX-LOG-RETRIEVEPOS decode"""

        # u-blox 7+, protVer 14+
        u = struct.unpack_from('<LlllLLLBBHBBBBBBBB', buf, 0)
        s = ("  entryIndex %u lon %d lat %d hMSL %d hAcc %u\n"
             "  gSpeed %u heading %u version %u fixType %u\n"
             "  year %u month %u day %u hour %u min %u sec %u\n"
             "  reserved1 x%x numSV %u reserved2 x%x" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n      fixType (%s)" %
                  (index_s(u[8], self.log_batch_fixType)))

        return s

    def log_retrieveposextra(self, buf):
        """UBX-LOG-RETRIEVEPOSEXTRA decode"""

        # u-blox 8+, protVer 15+
        u = struct.unpack_from('<LBBHBBBBBBHLLLL', buf, 0)
        s = ("  entryIndex %u version %u reserved1 x%x\n"
             "  year %u month %u day %u hour %u minute %u seconds %u\n"
             "  reserved2 x%x %x distance %u reserved3 x%x %x %x" % u)

        return s

    def log_retrievestring(self, buf):
        """UBX-LOG-RETRIEVESTRING decode"""

        # u-blox 7+, protVer 14+
        u = struct.unpack_from('<LBBHBBBBBBH', buf, 0)
        s = ("  entryIndex %u version %u reserved2 x%x\n"
             "  year %u month %u day %u hour %u min %u sec %u\n"
             "  reserved2 x%x byteCount %u\n"
             "  bytes \"" % u)
        if 0 < u[10]:
            for c in buf[16:16 + u[10]]:
                if c < 127 and chr(c) in string.printable:
                    s += chr(c)
                else:
                    s += "\\x%02x" % c
            s += '"'

        return s

    def log_string(self, buf):
        """UBX-LOG-STRING decode"""

        # u-blox 7+, protVer 14+
        m_len = len(buf)

        if 256 < m_len:
            return "  Bad Length %s" % m_len

        s = "  bytes "
        if 0 < m_len:
            s += gps.polystr(binascii.hexlify(buf))

        return s

    # UBX-LOG-
    log_ids = {
               0x03: {'str': 'ERASE', 'dec': log_erase, 'minlen': 0,
                      'name': "UBX-LOG-ERASE"},
               0x04: {'str': 'STRING', 'dec': log_string, 'minlen': 0,
                      'name': "UBX-LOG-STRING"},
               0x07: {'str': 'CREATE', 'dec': log_create, 'minlen': 8,
                      'name': "UBX-LOG-CREATE"},
               0x08: {'str': 'INFO', 'dec': log_info, 'minlen': 48,
                      'name': "UBX-LOG-INFO"},
               0x09: {'str': 'RETRIEVE', 'dec': log_retrieve, 'minlen': 12,
                      'name': "UBX-LOG-RETRIEVE"},
               0x0b: {'str': 'RETRIEVEPOS', 'dec': log_retrievepos,
                      'minlen': 40,
                      'name': "UBX-LOG-RETRIEVEPOS"},
               0x0d: {'str': 'RETRIEVESTRING', 'dec': log_retrievestring,
                      'minlen': 16,
                      'name': "UBX-LOG-RETRIEVESTRING"},
               0x0e: {'str': 'FINDTIME', 'dec': log_findtime, 'minlen': 8,
                      'name': "UBX-LOG-FINDTIME"},
               0x0f: {'str': 'RETRIEVEPOSEXTRA', 'dec': log_retrieveposextra,
                      'minlen': 32,
                      'name': "UBX-LOG-RETRIEVEPOSEXTRA"},
               0x10: {'str': 'RETRIEVEBATCH', 'dec': log_retrievebatch,
                      'minlen': 2,
                      'name': "UBX-LOG-RETRIEVEBATCH"},
               0x11: {'str': 'BATCH', 'dec': log_batch, 'minlen': 100,
                      'name': "UBX-LOG-BATCH"},
               }

    # UBX-MGA-
    mga_ack_type = {0: 'NACK',
                    1: 'ACK',
                    }

    mga_ack_infoCode = {0: 'OK',
                        1: 'Missing time',
                        2: 'Unsupported version',
                        3: 'Wrong size',
                        4: 'Storage failure',
                        5: 'Not ready',
                        5: 'Unknown Message',
                        }

    def mga_ack(self, buf):
        """UBX-MGA-ACK decode, Multiple GNSS acknowledge

u-blox 8, protVer 15 and up
"""

        u = struct.unpack_from('<BBBBL', buf, 0)
        s = (' type %u version %u infoCode %u msgId %u'
             ' msgPayloadStart x%s' % u)
        # plus some anonymous data...
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n    type (%s)' %
                  (index_s(u[0], self.mga_ack_type),
                   index_s(u[2], self.mga_ack_infoCode)))

        return s

    def mga_ano(self, buf):
        """UBX-MGA-ANO- decode, Multiple GNSS AssistNow Offline assistance

u-blox 8, protVer 15 and up
"""

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = (' type %u version %u svId %u gnssId %u'
             ' year %u month %u day %u reserved1 x%s' % u)
        # plus some anonymous data...
        if gps.VERB_DECODE <= self.verbosity:
            s += '\n    gnssId (%s)' % (index_s(u[3], self.gnss_id))

        return s

    def mga_dbd(self, buf):
        """UBX-MGA-DBD- decode, Navigation Database Dump Entry"""
        # max 172

        u = struct.unpack_from('<LLL', buf, 0)
        s = ' reserved1 %u %u %u' % u
        # plus some anonymous data...

        return s

    def mga_sf(self, buf):
        """UBX-MGA-SF- decode, Sensor Fusion data"""
        # max 96

        u = struct.unpack_from('<BB', buf, 0)
        s = ' type %u version %u' % u
        if 0 == u[0]:
            # MGA-SF-INI
            u = struct.unpack_from('<BBH', buf, 2)
            s += ' nValA %u nValB %u age %u reserved ...' % u
        elif 16 == u[0]:
            # MGA-SF-INI2
            s += ' reserved ...'
        else:
            s += '\n unknown type'

        return s

    # Braodcam calls this BRM-AST-
    mga_ids = {0x00: {'str': 'GPS', 'minlen': 16, 'name': "UBX-MGA-GPS"},
               0x02: {'str': 'GAL', 'minlen': 12, 'name': "UBX-MGA-GAL"},
               0x03: {'str': 'BDS', 'minlen': 16, 'name': "UBX-MGA-BDS"},
               0x05: {'str': 'QZSS', 'minlen': 12, 'name': "UBX-MGA-QZSS"},
               0x06: {'str': 'GLO', 'minlen': 20, 'name': "UBX-MGA-GLO"},
               0x10: {'str': 'SF', 'dec': mga_sf, 'minlen': 96,
                      'name': "UBX-MGA-SF"},
               # Braodcam calls this BRM-AST-LTO
               0x20: {'str': 'ANO', 'dec': mga_ano, 'minlen': 76,
                      'name': "UBX-MGA-ANO"},
               0x21: {'str': 'FLASH', 'minlen': 2, 'name': "UBX-MGA-FLASH"},
               # Braodcam calls this BRM-AST-REF_LOCATION
               # Braodcam calls this BRM-AST-REF_TIME_UTC
               0x40: {'str': 'INI', 'minlen': 12, 'name': "UBX-MGA-INI"},
               # Braodcam calls this BRM-AST-ACK
               0x60: {'str': 'ACK', 'dec': mga_ack,  'minlen': 8,
                      'name': "UBX-MGA-ACK"},
               # Braodcam calls this BRM-AST-NVMEM
               0x80: {'str': 'DBD', 'dec': mga_dbd, 'minlen': 12,
                      'name': "UBX-MGA-DBD"},
               }

    def mon_batch(self, buf):
        """UBX-MON-BATCH decode, Data batching buffer status"""

        # in u-blox 8 only, not in 7 or 9.
        # protVer 23.01, gone in 24
        u = struct.unpack_from('<BBBBHHHH', buf, 0)
        s = ("   version %u reserved1 %u %u %u fillLevel %u\n"
             "   dropsAll %u dropsSinceMon %u nextMsgCnt %u" % u)

        return s

    mon_comms_prot = {
        0: "UBX",
        1: "NMEA",
        2: "RTCM2",
        5: "RTCM3",
        6: "SPARTN",
        255: "None",
        }

    def mon_comms(self, buf):
        """UBX-MON-COMMS decode, Comm port information"""

        # at least protver 27
        if 27 > self.protver:
            self.protver = 27

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = ('version %u nPorts %u txErrors x%x reserved1 %u\n'
             'protIds %#x/%x/%x/%x' % u)
        s += (' (%s/%s/%s/%s)\n' %
              (index_s(u[4], self.mon_comms_prot),
               index_s(u[5], self.mon_comms_prot),
               index_s(u[6], self.mon_comms_prot),
               index_s(u[7], self.mon_comms_prot)))

        if gps.VERB_DECODE <= self.verbosity:
            errors = {
               0x40: "mem",
               0x80: "alloc",
               }

            s += "     txErrors (%s)\n" % (flag_s(u[2], errors))

        for i in range(0, u[1]):
            u = struct.unpack_from('<HHLBBHLBBHHHHHLLL', buf, (8 + (i * 40)))
            name = "%#x (%s)" % (u[0], index_s(u[0], self.port_ids1))
            if 0 < i:
                s += "\n"
            s += '  Port: %s\n' % name
            s += ('   txPending %u txBytes %u txUsage %u txPeakUsage %u\n'
                  '   rxPending %u rxBytes %u rxUsage %u rxPeakUsage %u\n'
                  '   overrunErrs %u msgs %u/%u/%u/%u reserved %x %x '
                  'skipped %u'
                  % u[1:])
        return s

    mon_gnss_supported_bits = {
        1: "GPS",
        2: "Glonass",
        4: "Beidou",
        8: "Galileo",
        }

    def mon_gnss(self, buf):
        """UBX-MON-GNSS decode, Information message major GNSS selection

Ignores newer constellations like NavIC on 9-series and later.
"""
        m_len = len(buf)

        if 8 > m_len:
            return "  Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = ('   version %u supported %#x defaultGnss %#x enabled %#x\n'
             '   simultaneous %u reserved1 %u %u %u' % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n     supported (%s)'
                  '\n     defaultGnss (%s)'
                  '\n     enabled (%s)' %
                  (flag_s(u[1], self.mon_gnss_supported_bits),
                   flag_s(u[2], self.mon_gnss_supported_bits),
                   flag_s(u[3], self.mon_gnss_supported_bits)))

        return s

    mon_hw_flags = {
        1: "rtcCalib",
        2: "safeBoot",
        }

    mon_hw_aPower = {
        0: "Off",
        1: "On",
        }

    jammingState = {
        0: "Unk",
        1: "OK",
        2: "Warning",
        3: "Critical",
        }

    def mon_hw(self, buf):
        """UBX-MON-HW decode, Hardware Status
Present from Antaris (4) to M10
68 bytes in 6-series
60 bytes in 8-series and 9-series
56 bytes in protVer 34 (10-series)
Deprecated in protVer 29.00 ( M9), use MON-H# and MON-RF
Deprecated. and undocumented, on M10, use MON-H# and MON-RF
"""
        m_len = len(buf)

        # the common part to 60 and 68 byte
        u = struct.unpack_from('<LLLLHHBBBBL', buf, 0)
        s = ('  pinSel x%x pinBank x%x pinDir x%x pinVal x%x noisePerMS %u\n'
             '  agcCnt %u aStatus %u aPower %u flags x%x reserved1 %u\n'
             '  usedMask x%x\n' % u)
        aStatus = u[6]
        aPower = u[7]
        flags = u[8]
        jammingState = (u[6] >> 2) & 0x03    # 8-series +
        xtalAbsent = (u[6] >> 4) & 1         # 9-series +
        jamInd = 0

        # VP
        # 17 bytes on protVer 15+
        # VP, 25 bytes on u-blox 6
        if 60 == m_len:
            u = struct.unpack_from('<BBBBBBBBBBBBBBBBBBBBLLL', buf, 28)
            s += ('  VP %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n'
                  '  jamInd %u reserved2 %u %u pinIrq x%x pullH x%x '
                  'pullL x%x' % u)
            jamInd = u[17]
        elif 68 == m_len:
            u = struct.unpack_from('<BBBBBBBBBBBBBBBBBBBBBBBBBBBBLLL', buf, 28)
            s += ('  VP %x %x %x %x %x %x %x %x %x %x %x %x %x\n'
                  '  VP %x %x %x %x %x %x %x %x %x %x %x %x\n'
                  '  jamInd %u reserved2 %u %u pinIrq x%x pullH x%x '
                  'pullL x%x' % u)
            jamInd = u[25]
        else:
            s += '   invalid message length %d\n' % mlen

        # flags:
        # 5 only has rtcCalib
        # 6 adds safeBoot and jammingState
        # protVer 18 (9+) adds xtalAbsent

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    aStatus (%s) aPower (%s) flags (%s) "
                  "jammingState (%s)\n"
                  "    jamInd %u xtalAbsent (%s)" %
                  (index_s(aStatus, self.mon_rf_antstat),
                   index_s(aPower, self.mon_hw_aPower),
                   index_s(flags, self.mon_hw_flags),
                   index_s(jammingState, self.jammingState),
                   jamInd, 'Yes' if xtalAbsent else 'No'))
        return s

    mon_hw2_cfgSource = {
        102: "flash image",
        111: "OTP",
        112: "config pins",
        114: "ROM",
        }

    def mon_hw2(self, buf):
        """UBX-MON-HW2 decode, Extended Hardware Status

Deprecated in protVer 29.00
"""

        u = struct.unpack_from('<bBbBBBBBLLLLL', buf, 0)
        s = ('   ofsI %d magI %u ofsQ %d magQ %u cfgSource %u\n'
             '   reserved0 %u %u %u lowLevCfg %d reserved1 %u %u\n'
             '   postStatus %u reserved2 %u' % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n     cfgSource (%s)' %
                  (index_s(u[4], self.mon_hw2_cfgSource)))

        return s

    mon_hw3_flags = {
        1: "rtcCalib",
        2: "safeBoot",
        4: "xtgalAbsent",
        }

    mon_hw3_pio = {
        0: "peripheral",
        1: "PIO",
        }

    mon_hw3_bank = {
        0: "A",
        1: "B",
        2: "C",
        3: "D",
        4: "E",
        5: "F",
        6: "G",
        7: "H",
        }

    mon_hw3_dir = {
        0: "In",
        1: "Out",
        }

    mon_hw3_value = {
        0: "Lo",
        1: "Hi",
        }

    mon_hw3_mask = {
        0x20: "VPM",
        0x40: "IRQ",
        0x100: "PUp",
        0x200: "PDn",
        }

    def mon_hw3(self, buf):
        """UBX-MON-HW3 decode, HW I/O pin information"""

        # at least protver 27
        if 27 > self.protver:
            self.protver = 27

        u = struct.unpack_from('<BBB', buf, 0)
        s = '   version %u nPins %u flags x%x' % u
        nPins = u[1]
        substr = buf[3:12]
        substr = substr.split(gps.polybytes('\0'))[0]
        s += ' hwVersion %s\n' % gps.polystr(substr)
        u1 = struct.unpack_from('<BBBBBBBBB', buf, 13)
        s += '   reserved1 %02x %02x %02x %02x %02x %02x %02x %02x %02x' % u1

        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n     flags (%s)' %
                  (index_s(u[2], self.mon_hw3_flags)))

        for i in range(22, 22 + (nPins * 6), 6):
            u = struct.unpack_from('<HHBB', buf, i)
            s += ('\n   pinId %4u pinMask %#5x VP %3u reserved2 %u' % u)
            if gps.VERB_DECODE <= self.verbosity:
                s += ('\n     pinMask (%s %s %s %s %s)' %
                      (index_s(u[1] & 1, self.mon_hw3_pio),
                       index_s((u[1] >> 1) & 7, self.mon_hw3_bank),
                       index_s((u[1] >> 4) & 1, self.mon_hw3_dir),
                       index_s((u[1] >> 5) & 1, self.mon_hw3_value),
                       flag_s(u[1] & 0xffc0, self.mon_hw3_mask)))

        return s

    def mon_io(self, buf):
        """UBX-MON-IO decode, I/O Subsystem Status

Deprecated in protVer 29.00
"""
        m_len = len(buf)

        s = ''
        for i in range(0, int(m_len / 20)):
            if 0 < i:
                s += "\n"

            u = struct.unpack_from('<LLHHHHL', buf, i * 20)
            s += ('  Port: %u (%s)\n' % (i, index_s(i, self.port_ids)))
            s += ('   rxBytes %u txBytes %u parityErrs %u framingErrs %u\n'
                  '   overrunErrs %u breakCond %u reserved %u' % u)
        return s

    def mon_msgpp(self, buf):
        """UBX-MON-MSGPP decode, Message Parse and Process Status

Deprecated in protVer 29.00
"""

        s = ''
        for i in range(1, 7):
            u = struct.unpack_from('<HHHHHHHH', buf, 0)
            s += " msg%u    %u %u %u %u %u %u %u %u\n" % ((i,) + u)

        u = struct.unpack_from('<LLLLLL', buf, 0)
        s += " skipped %u %u %u %u %u %u" % u
        return s

    mon_patch_act = {
        1: "Active"
        }
    mon_patch_loc = {
        0: "eFuse",
        2: "ROM",
        4: "BBR",
        6: "File System",
        }

    def mon_patch(self, buf):
        """UBX-MON-PATCH decode, Output information about installed patches."""

        # first seen in protver 15
        # len 0 == Poll requestm handled earlier

        u = struct.unpack_from('<HH', buf, 0)
        s = "  version %u nEntries %u" % u

        for i in range(0, u[1]):
            u = struct.unpack_from('<LLLL', buf, 4 + (i * 16))
            s += ("\n   patchInfo x%x comparatorNumber %u patchAddress %u "
                  "patchData %u" % u)
            if gps.VERB_DECODE <= self.verbosity:
                s += ('\n    patchInfo (%s, %s)' %
                      (flag_s(u[0] & 0x01, self.mon_patch_act),
                       index_s(u[0] & 0x06, self.mon_patch_loc)))

        return s

    mon_post_flags = (
        (0, 1, "active"),
        (1, 1, "inactive"),
    )

    def mon_post(self, buf):
        """UBX-MON-POST decode, Power On Self Test information

present F9T, protVer 29.
"""

        # len 0 == Poll requestm handled earlier

        u = struct.unpack_from('<BBHLL', buf, 0)
        s = ("  version %u flags x%x reserved1 x%x postState %d "
             "reserved2 x%x" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n    safeBoot (%s)' %
                  (flagm_s(u[1], self.mon_post_flags)))

        return s

    mon_rf_antstat = {
        0: "Init",
        1: "Unk",
        2: "OK",
        3: "Short",
        4: "Open",
        }

    mon_rf_antpwr = {
        0: "Off",
        1: "On",
        2: "Unk",
        }

    mon_rf_blockId = {
        0: "L1",
        1: "L2 or L5",  # varies by firmware and config
        }

    def mon_rf(self, buf):
        """UBX-MON-RF decode, RF Information"""

        # first seen in protver 27
        # at least protver 27
        if 27 > self.protver:
            self.protver = 27

        u = struct.unpack_from('<BBH', buf, 0)
        s = ' version %u nBlocks %u reserved1 x%x\n' % u

        if 0 == [1]:
            # avoid divide by zero
            return "  nBlocks is zero"

        m_len = len(buf)

        blockSize = (m_len - 4) / u[1]
        if 4 != (m_len - blockSize * u[1]):
            return ("  Bad length %u s/b %u, nBlocks %u" %
                    (m_len, 4 + blockSize * u[1], u[1]))

        # protVer 33.21 blockSize is 24
        # protVer 33.30 blockSize is 20 (compact format)
        # u-blox-F9-HPS-1.30_InterfaceDescription_UBX-22010984.pdf
        # protVer 33.40 blockSize is 24
        if 20 == blockSize:
            compact = True
        elif 24 == blockSize:
            compact = False
        else:
            return ("  Bad length %u s/b %u, nBlocks %u blockSize %f" %
                    (m_len, 4 + 24 * u[1], u[1], blockSize))

        for i in range(0, u[1]):
            s += "   %u: " % i
            if compact is False:
                # common 24 byte block
                u = struct.unpack_from('<BBBBLLHHBbBbBBBB', buf,
                                       4 + (24 * i))
                s += ("blockId %u flags x%x antStatus %u antPower %u "
                      "postStatus %u reserved1 x%x\n"
                      "      noisePerMS %u agcCnt %u jamInd %u ofsI %d "
                      "magI %u ofsQ %d magQ %u\n"
                      "      reserved2 %u %u %u\n" % u)
                if gps.VERB_DECODE <= self.verbosity:
                    # jammingState deprecated.  Use UBX-SEC-SIG (v2)
                    s += ("       blockId (%s) jammingState (%s) "
                          "antStatus (%s) antPower (%s)\n"
                          "       agc %.1f%%\n" %
                          (index_s(u[0], self.mon_rf_blockId),
                           index_s(u[1] & 0x03, self.jammingState),
                           index_s(u[2], self.mon_rf_antstat),
                           index_s(u[3], self.mon_rf_antpwr),
                           u[7] / 8181))
            else:  # True == compact
                # rare 20 byte block
                # msglen 20 does not have flags, jammingState
                u = struct.unpack_from('<BBBBLLHHbBbB', buf, 4 + (20 * i))
                s += ("blockId %u antStatus %u antPower %u cwSuppression %u "
                      "postStatus %u \n"
                      "      reserved1 x%x noisePerMS %u agcCnt %u\n"
                      "      ofsI %d magI %u ofsQ %d magQ %u\n" % u)

                if gps.VERB_DECODE <= self.verbosity:
                    s += ("       blockId (%s) antStatus (%s) "
                          "antPower (%s)\n"
                          "       agc %.1f%%\n" %
                          (index_s(u[0], self.mon_rf_blockId),
                           index_s(u[1], self.mon_rf_antstat),
                           index_s(u[2], self.mon_rf_antpwr),
                           u[8] / 81.91))

        return s[0:-1]    # remove trailing \n

    def mon_rxbuf(self, buf):
        """UBX-MON-RXBUF decode, Receiver Buffer Status

Deprecated in protVer 29.00
"""

        rxbuf_name = {
           1: " pending   ",
           2: " usage     ",
           3: " peakUsage ",
           }

        s = ''
        u = struct.unpack_from('<HHHHHH', buf, 0)
        s += rxbuf_name[1] + "%u %u %u %u %u %u\n" % u

        for i in range(2, 4):
            u = struct.unpack_from('<BBBBBB', buf, 6 * i)
            s += rxbuf_name[i] + "%u %u %u %u %u %u\n" % u
        return s

    mon_rxr_flags = {
        1: "Awake",
        }

    def mon_rxr(self, buf):
        """UBX-MON-RXBUF decode, Receiver Status Information"""

        # No way to poll

        u = struct.unpack_from('<B', buf, 0)
        s = "  flags: x%x" % u
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n  flags (%s)" %
                  index_s(u[0], self.mon_rxr_flags))

        return s

    mon_smgr_OscState = {
        0: "autonomous operation",
        1: "calibration ongoing",
        2: "oscillator steered by host",
        3: "idle state",
        }

    mon_smgr_Osc = {
        0x10: "OscCalib",
        0x20: "OscDisc",
        }

    mon_smgr_discSrc = {
        0: "internal oscillator",
        1: "GNSS",
        2: "EXTINT0",
        3: "EXTINT1",
        4: "internal oscillator measured by host",
        5: "external oscillator measured by host",
        }

    def mon_smgr(self, buf):
        """UBX-MON-SMGR decode, Synchronization manager status"""

        u = struct.unpack_from('<BBBBLHHBBBB', buf, 0)
        s = (" version %u reserved1 %u %u %u iTOW %u intOsc x%x extOsc x%x\n"
             " discOsc %u gnss x%x extInt0 x%x extInt1 x%x" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n  intOsc (%s) intOscState (%s)"
                  "\n  extOsc(%s) extoscState (%s)"
                  "\n  flags (%s)" %
                  (flag_s(u[3], self.mon_smgr_Osc),
                   index_s(u[3] & 0x0f, self.mon_smgr_OscState),
                   flag_s(u[4], self.mon_smgr_Osc),
                   index_s(u[4] & 0x0f, self.mon_smgr_OscState),
                   index_s(u[5], self.mon_smgr_discSrc)))
        return s

    def mon_span(self, buf):
        """UBX-MON-SPAN decode, Signal characteristics

protVer 27.00 and up
Present in F9, M9, M10S
"""

        # first seen in protver 27
        # at least protver 27
        if 27 > self.protver:
            self.protver = 27

        u = struct.unpack_from('<BBH', buf, 0)
        s = "  version %u numRfBlocks %u reserved0 %u" % u
        for i in range(0, u[1]):
            u = struct.unpack_from('<LLLBHB', buf, 260 + i * 272)
            s += "\n   span %u res %u center %u pga %u res x%x %x" % u
            center = u[2]
            span = u[0]

            if gps.VERB_DECODE <= self.verbosity:
                indent = "\n       "
            else:
                indent = "\n     "

            if gps.VERB_INFO <= self.verbosity:
                # -v 3, one f, data, per line
                for j in range(0, 256):
                    f = center + span * ((j - 128) / 256)
                    s += "\n     %u, %u" % (f, buf[(4 + j) + (i * 272)])
            else:
                for j in range(0, 256, 16):
                    # grab 256 bytes, ib lines of 16
                    u = struct.unpack_from('<BBBBBBBBBBBBBBBB', buf,
                                           (4 + j) + (i * 272))
                    if gps.VERB_DECODE <= self.verbosity:
                        f = center + span * ((j - 128) / 256)
                        s += "\n     f(%u) %u" % (j, f)
                    s += indent
                    s += ("%3u %3u %3u %3u %3u %3u %3u %3u "
                          "%3u %3u %3u %3u %3u %3u %3u %3u" % u)

        return s

    mon_sys_bootType = {
        0: "Unknown",
        1: "Cold Start",
        2: "Watchdog",
        3: "Hardware Reset",
        4: "Hardware backup",
        5: "Software backup",
        6: "Software reset",
        7: "VIO fail",
        8: "VDD_X fail",
        9: "VDD_RF fail",
        10: "V_CORE_HIGH fail",
        }

    def mon_sys(self, buf):
        """UBX-MON-SYS decode, Signal characteristics

protVer 27.00 and up
In:
    ZED-F9O protVer 27.50
Not in:
    NEO-M9N, protVer 32.00
    MAX-M10S, protVer 34.00
"""

        # first seen in protver 27
        # at least protver 27
        if 27 > self.protver:
            self.protver = 27

        u = struct.unpack_from('<BBBBBBBBLHHHbBL', buf, 0)
        if 1 != u[0]:
            s += "\n    WARNING: unknown version %u" % u[9]
            return s

        s = ("  msgVer %u bootType %u cpuLoad %u cpuLoadMax %u memUsage %u "
             "memUsageMax %u\n"
             "  ioUsage %u ioUsageMax %u runTime %u noticeCount %u "
             "warnCount %u\n"
             "  errCount %u tempValue %u reserved0 x%x %x" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    bootType (%s)" %
                  index_s(u[1], self.mon_sys_bootType))
        return s

    def mon_txbuf(self, buf):
        """UBX-MON-TXBUF decode, Transmitter Buffer Status

Deprecated in protVer 29.00
"""

        txbuf_name = {
           1: " pending   ",
           2: " usage     ",
           3: " peakUsage ",
           }

        errors = {
           0x40: "mem",
           0x80: "alloc",
           }

        s = ''
        u = struct.unpack_from('<HHHHHH', buf, 0)
        s += txbuf_name[1] + "%u %u %u %u %u %u\n" % u

        for i in range(2, 4):
            u = struct.unpack_from('<BBBBBB', buf, 6 * i)
            s += txbuf_name[i] + "%u %u %u %u %u %u\n" % u

        u = struct.unpack_from('<BBBB', buf, 24)
        s += " tUsage %u tPeakUsage %u errors x%x reserved1 %u" % u
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n  errors (%s) limit (%u)" %
                  (flag_s(u[2], errors), u[2] & 0x3f))

        return s

    def mon_ver(self, buf):
        """UBX-MON-VER decode, Poll Receiver/Software Version"""

        # min len = 40 in u-blox 5/6
        # min len = 70 in u-blox 9
        m_len = len(buf)

        substr = buf.split(gps.polybytes('\0'))[0]
        substr1 = buf[30:39]
        substr1 = substr1.split(gps.polybytes('\0'))[0]
        s = ("  swVersion %s\n"
             "  hwVersion %s" %
             (gps.polystr(substr), gps.polystr(substr1)))

        # extensions??
        protver = None
        num_ext = int((m_len - 40) / 30)
        for i in range(0, num_ext):
            loc = 40 + (i * 30)
            substr = buf[loc:]
            substr = substr.split(gps.polybytes('\0'))[0]
            polystr = gps.polystr(substr)
            s += '\n  extension %s' % polystr
            # is protver? delimiter may be space or equal
            if "PROTVER" == polystr[:7]:
                protver = "%.2f" % float(polystr[8:])

        opts_protver = "%.2f" % self.protver
        if protver and protver != opts_protver:
            s += ('\nWARNING:  protVer is %s, should be %s.'
                  '  Hint: use option "-P %s"' %
                  (opts_protver, protver, protver))
            # prolly too late to fix, try anyway.
            self.protver = float(protver)

        return s

    mon_ids = {0x02: {'str': 'IO', 'dec': mon_io, 'minlen': 20,
                      'name': 'UBX-MON-IO', 'depver': 29.0},
               0x04: {'str': 'VER', 'dec': mon_ver, 'minlen': 40,
                      'name': 'UBX-MON-VER'},
               0x06: {'str': 'MSGPP', 'dec': mon_msgpp, 'minlen': 120,
                      'name': 'UBX-MON-MSGPP', 'depver': 29.0},
               0x07: {'str': 'RXBUF', 'dec': mon_rxbuf, 'minlen': 24,
                      'name': 'UBX-MON-RXBUF', 'depver': 29.0},
               0x08: {'str': 'TXBUF', 'dec': mon_txbuf, 'minlen': 28,
                      'name': 'UBX-MON-TXBUF', 'depver': 29.0},
               0x09: {'str': 'HW', 'dec': mon_hw, 'minlen': 60,
                      'name': 'UBX-MON-HW'},
               0x0b: {'str': 'HW2', 'dec': mon_hw2, 'minlen': 28,
                      'name': 'UBX-MON-HW2', 'depver': 29.0},
               0x21: {'str': 'RXR', 'dec': mon_rxr, 'minlen': 1,
                      'name': 'UBX-MON-RXR'},
               0x27: {'str': 'PATCH', 'dec': mon_patch, 'minlen': 4,
                      'name': 'UBX-MON-PATCH'},
               0x28: {'str': 'GNSS', 'dec': mon_gnss, 'minlen': 8,
                      'name': 'UBX-MON-GNSS'},
               0x2e: {'str': 'SMGR', 'dec': mon_smgr, 'minlen': 16,
                      'name': 'UBX-MON-SMGR'},
               # protVer 19 and up, ADR and UDR only
               0x2f: {'str': 'SPT', 'minlen': 4, 'name': 'UBX-MON-SPT'},
               0x31: {'str': 'SPAN', 'dec': mon_span, 'minlen': 4,
                      'name': 'UBX-MON-SPAN'},
               0x32: {'str': 'BATCH', 'dec': mon_batch, 'minlen': 12,
                      'name': 'UBX-MON-BATCH'},
               0x36: {'str': 'COMMS', 'dec': mon_comms, 'minlen': 8,
                      'name': 'UBX-MON-COMMS'},
               0x37: {'str': 'HW3', 'dec': mon_hw3, 'minlen': 22,
                      'name': 'UBX-MON-HW3'},
               0x38: {'str': 'RF', 'dec': mon_rf, 'minlen': 4,
                      'name': 'UBX-MON-RF'},
               0x39: {'str': 'SYS', 'dec': mon_sys, 'minlen': 24,
                      'name': 'UBX-MON-SYS'},
               # protVer 29, F9T
               # protVer 50, X20
               0x3b: {'str': 'POST', 'dec': mon_post, 'minlen': 12,
                      'name': 'UBX-MON-POST'},
               }

    def nav_aopstatus(self, buf):
        """UBX-NAV-AOPSTATUS decode, AssistNow Autonomous Status"""

        u = struct.unpack_from('<LBBLLH', buf, 0)
        s = '  iTOW %u aopCfg %u status %u reserved1 %u %u %u' % u
        if gps.VERB_DECODE <= self.verbosity:
            aopCfg = {1: "useAOP"}
            s += "\n   aopCfg (%s)" % flag_s(u[1], aopCfg)

        return s

    def nav_att(self, buf):
        """UBX-NAV-ATT decode, Attitude Solution"""

        u = struct.unpack_from('<LBHBlllLLL', buf, 0)
        s = ("  iTOW %u version %u reserved1 %u %u\n"
             "  roll %d pitch %d heading %d\n"
             "  accRoll %d accPitch %d accHeading %d" % u)

        return s

    def nav_clock(self, buf):
        """UBX-NAV-CLOCK decode, Clock Solution"""

        u = struct.unpack_from('<LllLL', buf, 0)
        return '  iTOW %u clkB %d clkD %d tAcc %u fAcc %u' % u

    nav_dgps_status = {
        0: "None",
        1: "PR+PRR correction",
        }

    def nav_cov(self, buf):
        """UBX-NAV-COV decode, Covariance matrices

protVer 34 and up
"""

        u = struct.unpack_from('<LBBBLLBffffffffffff', buf, 0)
        return ('  iTOW %u version %u posCovValid %u velCovValid %u '
                'reserved0 %u %u %u\n'
                ' posCovNN %f posCovNE  %f posCovND %f\n'
                ' posCovEE %f posCovED  %f posCovDD %f\n'
                ' velCovNN %f velCovNE  %f velCovND %f\n'
                ' velCovEE %f velCovED  %f velCovDD %f\n' % u)

    def nav_dgps(self, buf):
        """UBX-NAV-DGPS decode, DGPS Data used for NAV"""

        # not present in protver 27+
        u = struct.unpack_from('<LlhhBBH', buf, 0)
        s = (' iTOW %u age %d baseID %d basehealth %d numCh %u\n'
             ' status x%x reserved1 %u' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n  status (%s)" %
                  index_s(u[5], self.nav_dgps_status))

        for i in range(0, u[4]):
            u = struct.unpack_from('<BbHff', buf, 16 + i * 12)
            s += ('\n  svid %3u flags x%2x ageC %u prc %f prcc %f' % u)
            if gps.VERB_DECODE <= self.verbosity:
                s += ("\n   channel %u dgps %u" %
                      (u[1] & 0x0f, (u[1] >> 4) & 1))

        return s

    def nav_dop(self, buf):
        """UBX-NAV-DOP decode, Dilution of Precision"""

        u = struct.unpack_from('<Lhhhhhhh', buf, 0)
        s = ('  iTOW %u gDOP %u pDOP %u tDOP %u vDOP %u\n'
             '  hDOP %u nDOP %u eDOP %u' % u)
        return s

    def nav_eell(self, buf):
        """UBX-NAV-EOE decode, Position error ellipse parameters

UBX-NAV-EELL, protVer 19.1 and up, ADR, MDR and HPS only
"""

        u = struct.unpack_from('<LBBHLL', buf, 0)
        if 0 != u[1]:
            return "  Unknown version %u, s/b 0" % u[1]

        return (' iTOW %u version %u reserved0 %u\n'
                '  errEllipseOrient %u errEllipseMajor %u errEllipseMinor %u' %
                u)

    def nav_eoe(self, buf):
        """UBX-NAV-EOE decode, End Of Epoch"""

        u = struct.unpack_from('<L', buf, 0)
        return ' iTOW %u' % u

    nav_geofence_state = {
        1: "Inside",
        2: "Outside",
        }

    nav_geofence_status = {
        0: "n/a",
        1: "Active",
        }

    def nav_geofence(self, buf):
        """UBX-NAV-GEOFENCE decode, Geofencing status"""

        u = struct.unpack_from('<LBBBB', buf, 0)
        s = '  iTOW:%u version %u status %u numFences %u combState %u' % u
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    status (%s) combState (%s)" %
                  (index_s(u[2], self.nav_geofence_status),
                   index_s(u[4], self.nav_geofence_state)))

        for i in range(0, u[3]):
            u = struct.unpack_from('<BB', buf, 8 + (i * 2))
            s += '\n  state %u reserved1 %u' % u
            if gps.VERB_DECODE <= self.verbosity:
                s += ("\n    state (%s)" %
                      (index_s(u[0], self.nav_geofence_state)))

        return s

    def nav_hpposecef(self, buf):
        """UBX-NAV-HPPOSECEF decode, High Precision Position Solution ECEF"""

        u = struct.unpack_from('<BBBBLlllbbbbL', buf, 0)
        if 0 != u[0]:
            return "  Unknown version %u, s/b 0" % u[0]

        s = ('  version %u reserved1 %u %u %u iTOW %u\n'
             '  ecef: X %d Y %d Z %d\n'
             '  ecefHP: X %d Y %d Z %d\n'
             '  reserved2 %u pAcc %u' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n    HPecef X %.4f m Y %.4f m Z %.4f m' %
                  (((u[5] * 100) + u[8]) * 10e-5,
                   ((u[6] * 100) + u[9]) * 10e-5,
                   ((u[7] * 100) + u[10]) * 10e-5))

        return s

    def nav_hpposllh(self, buf):
        """UBX-NAV-HPPOSLLH decode, HP Geodetic Position Solution LLH"""

        u = struct.unpack_from('<BBBBLllllbbbbLL', buf, 0)
        if 0 != u[0]:
            return "  Unknown version %u, s/b 0" % u[0]

        s = ('  version %u reserved1 %u %u %u iTOW %u\n'
             '  lon %d lat %d height %d hMSL %d\n'
             '  lonHp %d latHp %d heightHp %d hMSLHp %d\n'
             '  hAcc %u vAcc %u' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n    HPlon %.9f HPlat %.9f'
                  '\n    HPaltHAE %.4f HPaltMSL %.4f' %
                  (((u[5] * 100) + u[9]) * 10e-10,
                   ((u[6] * 100) + u[10]) * 10e-10,
                   ((u[7] * 100) + u[11]) * 10e-6,
                   ((u[8] * 100) + u[12]) * 10e-6))
        return s

    def nav_odo(self, buf):
        """UBX-NAV-ODO decode, Odometer Solution"""

        u = struct.unpack_from('<BBBBLLLL', buf, 0)
        return ("  version %u reserved1 %u %u %u iTOW %u\n"
                "  distance %u totalDistance %u distanceStd %u" % u)

    nav_orb_almUsability = {
            0: "Unusable",
            30: "> 30 days",
            31: "Unknown",
        }

    nav_orb_ephUsability = {
            0: "Unusable",
            30: "> 450 mins",
            31: "Unknown",
        }

    nav_orb_ephSource = {
            0: "not available",
            1: "GNSS transmission",
            2: "external aiding",
        }

    nav_orb_type = {
            0: "not available",
            1: "Assist now offline data",
            2: "Assist now autonomous data",
        }

    def nav_orb(self, buf):
        """UBX-NAV-ORB decode, GNSS Orbit Database Info"""

        u = struct.unpack_from('<LBBH', buf, 0)
        s = "  iTOW %u version %u numSv %u reserved1 %u" % u

        for i in range(0, u[2]):
            u = struct.unpack_from('<BBBBBB', buf, 8 + (i * 6))
            s += ("\n   gnssId %u svId %3u svFlag x%02x eph x%02x alm x%02x "
                  "otherOrb x%x" % u)
            if gps.VERB_DECODE <= self.verbosity:
                eph = int(u[3] & 0x1f)
                s1 = index_s(eph, self.nav_orb_ephUsability, nf="")
                if not s1:
                    s1 = "%d to %d mins" % ((eph - 1) * 15, eph * 15)

                alm = int(u[4] & 0x1f)
                s2 = index_s(alm, self.nav_orb_almUsability, nf="")
                if not s2:
                    s2 = "%d to %d days" % (alm - 1, alm)

                other = int(u[5] & 0x1f)
                s3 = index_s(other, self.nav_orb_almUsability, nf="")
                if not s3:
                    s3 = "%d to %d days" % (other - 1, other)

                s += ("\n    (%s:%u) health (%s) visibility (%s)"
                      "\n     ephUsability (%s) ephSource (%s)"
                      "\n     almUsability (%s) almSource (%s)"
                      "\n     anoAopUsability (%s) type (%s)" %
                      (index_s(u[0], self.gnss_id), u[1],
                       index_s(u[2] & 3, self.health),
                       index_s((u[2] >> 2) & 3, self.visibility),
                       s1,
                       index_s(u[3] >> 5, self.nav_orb_ephSource, nf="other"),
                       s2,
                       index_s(u[4] >> 5, self.nav_orb_ephSource, nf="other"),
                       s3,
                       index_s(u[5] >> 5, self.nav_orb_type, nf="other")))

        return s

    def nav_pl(self, buf):
        """UBX-NAV-PL decode, Protection Level Info

Partial decode."""

        u = struct.unpack_from('<B', buf, 0)
        if 1 != u[0]:
            return "  Unknown version %u" % u[0]

        u = struct.unpack_from('<BBbBBBBBBBBBBLLL', buf, 0)
        s = ("  version %u tmirCoeff %u tmirExp %u  plPosValid %u "
             "posPosFrame %u\n"
             "  posVelValid %u posVelFrame %u\n"
             "  plTimeValid %u plPosInvalidityReason %u plVel %u\n"
             "  plVelInvalidityReason %u plTimeInvalidtyReason %u "
             "reserved0 %u\n"
             "  iTow  %u plPos1 %u plPos2 %u" % u)
        return s

    def nav_posecef(self, buf):
        """UBX-NAV-POSECEF decode, Position Solution in ECEF"""

        # protVer 4+
        u = struct.unpack_from('<LlllL', buf, 0)
        return '  iTOW %u ecefX %d Y %d Z %d pAcc %u' % u

    def nav_posllh(self, buf):
        """UBX-NAV-POSLLH decode, Geodetic Position Solution"""

        u = struct.unpack_from('<LllllLL', buf, 0)
        return ('  iTOW %u lon %d lat %d height %d\n'
                '  hMSL %d hAcc %u vAcc %u' % u)

    # NAV-PVT and NAV-PVAT
    nav_pvt_valid = {
        1: "validDate",
        2: "ValidTime",
        4: "fullyResolved",
        8: "validMag",      # protver 27
        }

    # NAV-PVT and NAV-PVAT
    # u-blox TIME ONLY is same as Surveyed
    nav_pvt_fixType = {
        0: 'None',
        1: 'Dead Reckoning',
        2: '2D',
        3: '3D',
        4: 'GPS+DR',
        5: 'Surveyed',
        }

    # NAV-PVT and NAV-PVAT
    nav_pvt_flags = {
        (1, 1, "gnssFixOK,"),
        (2, 2, "diffSoln,"),
        (8, 8, "vehRollValid,"),
        (0x10, 0x10, "vehPitchValid,"),
        (0x20, 0x20, "vehHeadingValid,"),
        (0, 0xc0, "No Carrier Phase solution,"),    # not before protVer 20
        (0x40, 0xc0, "Carrier Phase float,"),
        (0x80, 0xc0, "Carrier Phase fixed,"),
        (0xc0, 0xc0, "Carrier Phase Unk,"),
        }

    # NAV-PVT and NAV-PVAT
    nav_pvt_flags2 = {
        0x20: "confirmedAvai",
        0x40: "confirmedDate",
        0x80: "confirmedTime",
        }

    def nav_pvat(self, buf):
        """UBX-NAV-PVAT decode, Nav Pos Att Velocity Time Solution

Combines UBX-NAV-PVT, UBX-HNR-ATT, UBX-NAV-EELL, and NAV-TIMEUTC

Present in:
   protver 30  (ADR/DBD/HPS/LAP/MDR 9-series firmware)

Not present in:
   u-blox 5, 6, 7 or 8
"""
        m_len = len(buf)

        # FIXME: partial.  Needs checking!
        u = struct.unpack_from('<LBB'
                               'HBBBBBBH'
                               'LlBBBB'
                               'llll'
                               'LLllllL'
                               'llll'
                               'HHH'
                               'hH'
                               'HLLLL',
                               buf, 0)
        if 0 != u[1]:
            return "  Unknown version %u s/b 0" % u[1]

        s = ('  iTOW %u version %u valid x%x\n'
             '  ymd %u/%02u/%02u hms %02u:%02u:%02u '
             'reserved0 x%x reserved1 x%x\n'
             '  tAcc %u nano %d fixType %u flags x%x flags2 x%x numSV %u\n'
             '  lon %d lat %d altHAE %d altMSL %d\n'
             '  hAcc %u vAAcc %u velNED %d %d %d gSpeed %d sAcc %u\n'
             '  vehRoll %d vehPitch %d vehHeading %d motHeading %d\n'
             '  accRoll %u accPitch %u accHeading %u\n'
             '  magDec %d magAcc %u\n'
             '  errEllipseOrient %u errEllipseMajor %u errEllipseMinor %u\n'
             '  reserved2 x%x reserved3 x%x' %
             u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    valid (%s)"
                  "\n    fixType (%s)"
                  "\n    flags (%s)"
                  "\n    flags2 (%s)" %
                  (flag_s(u[2], self.nav_pvt_valid),
                   index_s(u[13], self.nav_pvt_fixType),
                   flagm_s(u[14], self.nav_pvt_flags),
                   flag_s(u[15], self.nav_pvt_flags2)))
        return s

    nav_pvt_flags3 = {
        0x01: "invalidLlh",
        0x20: "authTime",
        0x40: "nmaFixStatus",
        }

    nav_pvt_psm = {
        0: "Not Active",
        1: "Enabled",
        2: "Acquisition",
        3: "Tracking",
        4: "Power Optimized Tracking",
        5: "Inactive",
        }

    # in UBX-NAV-STATUS since Antaris 4
    carrSoln = {
        0: "None",
        1: "Floating",
        2: "Fixed",
        }

    def nav_pvt(self, buf):
        """UBX-NAV-PVT decode, Navigation Position Velocity Time Solution"""
        m_len = len(buf)

        # 84 bytes long in protver 14.
        # 92 bytes long in protver 15 and later.

        # flags2 is protver 27
        # flags3 is protver 29
        u = struct.unpack_from('<LHBBBBBB'
                               'LlBBB'
                               'Blll'
                               'lLL'
                               'lllll'
                               'LLHHL', buf, 0)
        s = ('  iTOW %u time %u/%02u/%02u %02u:%02u:%02u valid x%x\n'
             '  tAcc %u nano %d fixType %u flags x%x flags2 x%x\n'
             '  numSV %u lon %d lat %d height %d\n'
             '  hMSL %d hAcc %u vAcc %u\n'
             '  velNED %d %d %d gSpeed %d headMot %d\n'
             '  sAcc %u headAcc %u pDOP %u flags3 x%x reserved0 x%x' % u)

        if 92 <= m_len:
            # version 15
            u1 = struct.unpack_from('<lhH', buf, 84)
            s += ('\n  headVeh %d magDec %d magAcc %u' % u1)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    valid (%s)"
                  "\n    fixType (%s)"
                  "\n    flags (%s)"
                  "\n    flags2 (%s)"
                  "\n    psmState (%s)"
                  "\n    carrSoln (%s)"
                  "\n    flags3 (%s) lastCorrectionAge %u" %
                  (flag_s(u[7], self.nav_pvt_valid),
                   index_s(u[10], self.nav_pvt_fixType),
                   flagm_s(u[11], self.nav_pvt_flags),
                   flag_s(u[12], self.nav_pvt_flags2),
                   index_s((u[11] >> 2) & 0x0f, self.nav_pvt_psm),
                   index_s((u[11] >> 6) & 0x03, self.carrSoln),
                   flag_s(u[28], self.nav_pvt_flags3),
                   (u[28] >> 1) & 0x0f))
        return s

    nav_relposned_flags = {
        1: "gnssFixOK",
        2: "diffSoln",
        4: "relPosValid",
        0x20: "isMoving",             # protVer 20.3+
        0x40: "refPosMiss",           # protVer 20.3+
        0x80: "refObsMiss",           # protVer 20.3+
        0x100: "relPosHeadingValid",  # protVer 27.11+
        0x200: "relPosNormalized",    # protVer 27.11+
        }

    def nav_relposned(self, buf):
        """UBX-NAV-RELPOSNED decode
Relative Positioning Information in NED frame.
protVer 20+ is 40 bytes
protVer 27.11+ is 64 bytes, and things reordered, so not upward compatible
High Precision GNSS products only."""

        m_len = len(buf)

        # common part
        u = struct.unpack_from('<BBHLlll', buf, 0)
        s = ('  version %u reserved1 %u refStationId %u iTOW %u\n'
             '  relPosN %d relPosE %d relPosD %d\n' % u)

        if (1 == u[0] and 64 <= m_len):
            # valid version 1 packet, newer u-blox 9
            u1 = struct.unpack_from('<llLbbbbLLLLLLL', buf, 20)
            s += ('  relLength %d relHeading %d reserved2 %u\n'
                  '  relPosHPN %d relPosHPE %d relPosHPD %d '
                  'relPosHPLength %d\n'
                  '  accN %u accE %u accD %u accLength %u accHeading %u\n'
                  '  reserved3 %u flags x%x' % u1)
            flags = u1[13]
        elif (0 == u[0] and 40 <= m_len):
            # valid version 0 packet, u-blox 8, and some u-blox 9
            u1 = struct.unpack_from('<bbbbLLLL', buf, 20)
            s += ('  relPosHPN %d relPosHPE %d relPosHPD %d reserved2 %u\n'
                  '  accN %u accE %u accD %u flags x%x' % u1)
            flags = u1[7]
        else:
            # WTF?
            return "  Bad Length %d version %u combination" % (m_len, u[0])

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    flags (%s)\n"
                  "    carrSoln (%s)" %
                  (flag_s(flags, self.nav_relposned_flags),
                   index_s((flags >> 3) & 0x03, self.carrSoln)))
        return s

    def nav_resetodo(self, buf):
        """UBX-NAV-RESETODO decode, reset odometer"""

        m_len = len(buf)
        if 0 == m_len:
            s = " reset request"
        else:
            s = " unexpected data"
        return s

    qualityInd = {
        0: "None",
        1: "Searching",
        2: "Acquired",
        3: "Detected",
        4: "Code and time locked",
        5: "Code, carrier and time locked",
        6: "Code, carrier and time locked",
        7: "Code, carrier and time locked",
        }

    health = {
        0: "Unknown",
        1: "Healthy",
        2: "Unhealthy",
        }

    visibility = {
        1: "below horizon",
        2: "above horizon",
        3: "above elevation mask",
        }

    nav_sat_orbit = {
        0: "None",
        1: "Ephemeris",
        2: "Almanac",
        3: "AssistNow Offline",
        4: "AssistNow Autonomous",
        5: "Other",
        6: "Other",
        7: "Other",
        }

    nav_sat_flags = {
        8: "svUsed",
        0x40: "diffCorr",
        0x80: "smoothed",
        0x800: "ephAvail",
        0x1000: "almAvail",
        0x2000: "anoAvail",
        0x4000: "aopAvail",
        0x10000: "sbasCorrUsed",
        0x20000: "rtcmCorrUsed",
        0x40000: "slasCorrUsed",
        0x80000: "spartnCorrUsed",
        0x100000: "prCorrUsed",
        0x200000: "crCorrUsed",
        0x400000: "doCorrUsed",
        0x800000: "classCorrUsed",
        0x1000000: "lppCorrUsed",
        0x2000000: "hasCorrUsed",
        }

    def nav_sat(self, buf):
        """UBX-NAV-SAT decode

Not in u-blox 5
Present in u-blox 8,  protocol version 15+
Present in protVer 32
"""

        u = struct.unpack_from('<LBBH', buf, 0)
        s = '  iTOW %u version %u numSvs %u reserved1 x%x' % u

        for i in range(0, u[2]):
            u = struct.unpack_from('<BBBbhhL', buf, 8 + (i * 12))
            s += ('\n   gnssId %u svid %3u cno %2u elev %3d azim %3d prRes %6d'
                  ' flags x%x' % u)
            if gps.VERB_DECODE <= self.verbosity:
                s += ("\n     (%s)"
                      "\n     flags(%s)"
                      "\n     qualityInd x%x (%s) health (%s)"
                      "\n     orbitSource (%s)" %
                      (index_s(u[0], self.gnss_id),
                       flag_s(u[6], self.nav_sat_flags),
                       u[6] & 7, index_s(u[6] & 7, self.qualityInd),
                       index_s((u[6] >> 4) & 3, self.health),
                       index_s((u[6] >> 8) & 7, self.nav_sat_orbit)))

        return s

    nav_sbas_mode = {
        0: "Disabled",
        1: "Enabled Integrity",
        2: "Enabled Testmode",
        }

    # sometimes U1 or I1, 255 or -1 == Unknown
    nav_sbas_sys = {
        0: "WAAS",
        1: "EGNOS",
        2: "MSAS",
        3: "GAGAN",
        4: "SDCM",      # per ICAO Annex 10, v1, Table B-27
        16: "GPS",
        }

    nav_sbas_service = {
        1: "Ranging,",
        2: "Corrections,",
        4: "Integrity,",
        8: "Testmode,",
        0x10: "Bad,",       # F10T, protVer 42
        }

    nav_sbas_statusFlags = {
        0: "SBAS integrity unknown",
        1: "SBAS integrity NA",
        2: "SBAS integrity used",
        }

    def nav_sbas(self, buf):
        """UBX-NAV-SBAS decode, SBAS Status Data"""

        # present in protver 10+ (Antaris4 to ZOE-M8B
        # undocumented, but present in protver 27+
        # undocumented, but present in protver 32, NEO-M9N

        u = struct.unpack_from('<LBBbBBBH', buf, 0)
        s = (" iTOW %d geo %u mode x%x sys %d service x%x cnt %u "
             "statusFlags x%x\n"
             " reserved0 x%x" % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    mode (%s) sys (%s)"
                  "\n    service (%s) statusFlags (%s)" %
                  (index_s(u[2], self.nav_sbas_mode),
                   index_s(u[3], self.nav_sbas_sys),
                   flag_s(u[4], self.nav_sbas_service),
                   index_s(u[5], self.nav_sbas_statusFlags)))

        for i in range(0, u[5]):
            u = struct.unpack_from('<BBBBBBhHh', buf, 12 + (i * 12))
            s += ("\n  svid %3d flags x%04x udre x%02x svSys %3d svService %2d"
                  " reserved2 %u"
                  "\n   prc %3d reserved3 %u ic %3d" % u)
            if gps.VERB_DECODE <= self.verbosity:
                # where are flags and udre defined??
                s += ("\n   svSys (%s) svService (%s)" %
                      (index_s(u[3], self.nav_sbas_sys),
                       flag_s(u[4], self.nav_sbas_service)))

        return s

    nav_sig_corrSource = {
        0: "None",
        1: "SBAS",
        2: "BeiDou",
        3: "RTCM2",
        4: "RTCM3 OSR",
        5: "RTCM3 SSR",
        6: "QZSS SLAS",
        7: "SPARTN",
        8: "CLAS",        # Dupe?
        9: "CLAS",        # Dupe?
        10: "LPP OSR",
        11: "LPP SSR",
        12: "GAL HAS",
        }

    nav_sig_ionoModel = {
        0: "None",
        1: "Klobuchar over GPS",
        2: "SBAS",
        3: "Klobuchar over BeiDou",
        8: "Dual Frequency obs",
        }

    nav_sig_sigFlags = {
        (0, 3, "health unkown,"),
        (1, 3, "healthy,"),
        (2, 3, "unhealthy,"),
        (3, 3, "bad flag,"),
        (4, 4, "prSmoothed"),
        (8, 8, "prUsed"),
        (0x10, 0x10, "crUsed"),
        (0x20, 0x20, "doUsed"),
        (0x40, 0x40, "prCorrUsed"),
        (0x80, 0x80, "crCorrUsed"),
        (0x100, 0x100, "doCorrUsed"),
        (0x200, 0x200, "authStatus"),
        }

    def nav_sig(self, buf):
        """decode UBX-NAV-SIG decode, Signal Information

Present in 9 and 10, protVer 29 and up
"""

        u = struct.unpack_from('<LBBH', buf, 0)
        s = '  iTOW %u version %u numSigs %u reserved1 %u' % u

        for i in range(0, u[2]):
            u = struct.unpack_from('<BBBBhBBBBHL', buf, 8 + (i * 16))
            s += ('\n   gnssId %u svId %u sigId %u freqId %u prRes %d cno %u '
                  'qualityInd %u\n'
                  '    corrSource %u ionoModel %u sigFlags x%x reserved2 %u' %
                  u)

            if gps.VERB_DECODE <= self.verbosity:
                s += ("\n      (%s) corrSource (%s)"
                      "\n      qualityInd (%s)"
                      "\n      ionoModel (%s)"
                      "\n      sigFlags (%s)" %
                      (self.gnss_s(u[0], u[1], u[2]),
                       index_s(u[7], self.nav_sig_corrSource),
                       index_s(u[6], self.qualityInd),
                       index_s(u[8], self.nav_sig_ionoModel),
                       flagm_s(u[9], self.nav_sig_sigFlags)))
        return s

    nav_slas_flags = {
        1: "gmsAvailable",
        2: "qzssSvAvailable",
        4: "testMode",
        }

    def nav_slas(self, buf):
        """UBX-NAV-SLAS decode, QZSS L1S SLAS Status Data"""

        u = struct.unpack_from('<LBBBBllBBBB', buf, 0)
        s = ('  iTOW %u version %u reserved1 %u %u %u'
             '  gmsLon %d gmsLon %d gmsCode %u qzssSvId %u'
             '  serviceFlags x%x cnt %d' % u)

        for i in range(0, u[8]):
            u = struct.unpack_from('<BBLh', buf, 20 + (i * 8))
            s += '\n   gnssId %u svId %u reserved23 %u prc %d ' % u

            if gps.VERB_DECODE <= self.verbosity:
                s += ("\n     flags (%s)" %
                      (flag_s(u[3], self.nav_slas_flags)))
        return s

    nav_sol_flags = {
        1: "GPSfixOK",
        2: "DiffSoln",
        4: "WKNSET",
        8: "TOWSET",
        }

    def nav_sol(self, buf):
        """UBX-NAV-SOL decode, Navigation Solution Information

deprecated by u-blox
removed in protVer 32 (u-blox 9 and 10)
Use UBX-NAV-PVT instead
"""

        u = struct.unpack_from('<LlhBBlllLlllLHBBL', buf, 0)
        s = ('  iTOW %u fTOW %d week %d gpsFix %u flags x%x\n'
             '  ECEF X %d Y %d Z %d pAcc %u\n'
             '  VECEF X %d Y %d Z %d sAcc %u\n'
             '  pDOP %u reserved1 %u numSV %u reserved2 %u' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   gpsfix (%s)"
                  "\n   flags (%s)" %
                  (index_s(u[3], self.nav_pvt_fixType),
                   flag_s(u[4], self.nav_sol_flags)))

        return s

    nav_status_fixStat = {
        1: "diffCorr",
        2: "carrSolnValid",
        }

    nav_status_mapMatching = {
        0: "None",
        0x40: "Too old",
        0x80: "Valid10",
        0xC0: "Valid11",
        }

    nav_status_psmState = {
        0: "Acquisition",
        1: "Tracking",
        2: "Power Optimized Tracking",
        3: "Inactive",
        }

    nav_status_spoofDetState = {
        0: "Deactivated",
        1: "None indicated",
        2: "Indicated",
        3: "Multiple Indicated",
        }

    def nav_status(self, buf):
        """UBX-NAV-STATUS decode"""

        u = struct.unpack_from('<LBBBBLL', buf, 0)
        s = ('  iTOW %d gpsFix %d flags %#x fixStat %#x flags2 %#x\n'
             '  ttff %d, msss %d' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   gpsfix (%s)"
                  "\n   flags (%s)"
                  "\n   fixStat (%s) mapMatching (%s)"
                  "\n   flags2 (psmState %s spoofDetState %s carrSoln %s)" %
                  (index_s(u[1], self.nav_pvt_fixType),
                   flag_s(u[2], self.nav_sol_flags),
                   flag_s(0x3f & u[3], self.nav_status_fixStat),
                   index_s(0xc0 & u[3], self.nav_status_mapMatching),
                   index_s(3 & u[4], self.nav_status_psmState),
                   index_s(3 & (u[4] >> 3), self.nav_status_spoofDetState),
                   index_s(3 & (u[4] >> 6), self.carrSoln)))
        return s

    def nav_svin(self, buf):
        """UBX-NAV-SVIN decode, Survey-in data

Removed in protVer 32.00
"""

        # in M8 HPG only
        u = struct.unpack_from('<BBBBLLlllbbbBLLBB', buf, 0)
        return ('  version %u reserved1[%u %u %u] iTOW %u dur %u\n'
                '  meanX %d meanY %d meanZ %d\n'
                '  meanXHP %d meanYHP %d meanZHP %d reserved2 %u meanAcc %u\n'
                '  obs %u valid %u active %u' % u)

    nav_svinfo_gflags = {
        0: 'Antaris, Antaris 4',
        1: 'u-blox 5',
        2: 'u-blox 6',
        3: 'u-blox 7',
        4: 'u-blox 8/M8',
        }

    nav_svinfo_flags = {
        1: 'svUsed',
        2: 'diffCorr',
        4: 'orbitAvail',
        8: 'orbitEph',
        0x10: 'unhealthy',
        0x20: 'orbitAlm',
        0x40: 'orbitAop',
        0x80: 'smoothed',
        }

    nav_svinfo_quality = {
        0: 'no signal',
        1: 'searching signal',
        2: 'signal acquired',
        3: 'signal detected but unusable',
        4: 'code locked, time synced',
        5: 'code and carrier locked, time synced',
        6: 'code and carrier locked, time synced',
        7: 'code and carrier locked, time synced',
        }

    def nav_svinfo(self, buf):
        """UBX-NAV-SVINFO decode

removed from u-blox 10 (protVer 34 and up)
Use UBX-NAV-SAT or UBX-NAV-SIG instead
Present in M8 Timing and FTS only
"""
        m_len = len(buf)

        u = struct.unpack_from('<LbbH', buf, 0)
        s = ' iTOW:%d numCh %d globalFlags %d reserved1 x%x' % u
        if gps.VERB_DECODE <= self.verbosity:
            s += '\n  globalFlags (%s)' % index_s(u[2], self.nav_svinfo_gflags)

        m_len -= 8
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<BBBBBbhl', buf, 8 + i * 12)
            s += ('\n  chn %3d svid %3d flags %#0.2x quality %u cno %2d'
                  ' elev %3d azim %3d prRes %6d' % u)
            if gps.VERB_DECODE <= self.verbosity:
                s += ('\n   flags (%s)'
                      '\n   quality (%s)' %
                      (flag_s(u[2], self.nav_svinfo_flags),
                       index_s(u[3], self.nav_svinfo_quality)))
            m_len -= 12
            i += 1

        return s

    nav_time_valid = {
        1: "towValid",
        2: "weekValid",
        4: "leapValid",
        }

    def nav_timebds(self, buf):
        """UBX-NAV-TIMEBDS decode"""

        u = struct.unpack_from('<LLlhbBL', buf, 0)
        s = ("  iTOW %d SOW %d fSOW %d week %d leapS %d\n"
             "  Valid %#x tAcc %d" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   valid (%s)" %
                  (flag_s(u[5], self.nav_time_valid)))
        return s

    def nav_timegal(self, buf):
        """UBX-NAV-TIMEGAL decode"""

        u = struct.unpack_from('<LLlhbBL', buf, 0)
        s = ("  iTOW %d galTOW %d fGalTow %d galWno %d leapS %d\n"
             "  Valid x%x, tAcc %d" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   valid (%s)" %
                  (flag_s(u[5], self.nav_time_valid)))
        return s

    def nav_timeglo(self, buf):
        """UBX-NAV-TIMEGLO decode"""

        u = struct.unpack_from('<LLlhbBL', buf, 0)
        s = ("  iTOW %d TOD %d fTOD %d Nt %d  N4 %d\n"
             "  Valid x%x tAcc %d" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   valid (%s)" %
                  (flag_s(u[5], self.nav_time_valid)))
        return s

    def nav_timegps(self, buf):
        """UBX-NAV-TIMEGPS decode"""

        u = struct.unpack_from('<LlhbBL', buf, 0)
        s = "  iTOW %u fTOW %u week %d leapS %d valid x%x tAcc %d" % u

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   valid (%s)" %
                  (flag_s(u[4], self.nav_time_valid)))
        return s

    nav_timels_srcOfCurrLs = {
        0: "Default",
        1: "GPS/GLONASS derived",
        2: "GPS",
        3: "SBAS",
        4: "BeiDou",
        5: "Galileo",
        6: "Aided data",
        7: "Configured",
        8: "IRNSS",        # aka NavIC
        255: "Unknown",
        }

    nav_timels_srcOfLsChange = {
        0: "None",
        2: "GPS",
        3: "SBAS",
        4: "BeiDou",
        5: "Galileo",
        6: "GLONASS",
        7: "IRNSS",        # aka NavIC
        }

    nav_timels_valid = {
        1: "validCurrLs",
        2: "validTimeToLsEvent",
        }

    def nav_timels(self, buf):
        """UBX-NAV-TIMELS decode, Leap second event information"""

        u = struct.unpack_from('<LBBBBBbBblHHBBBB', buf, 0)
        s = ('  iTOW %u version %u reserved2 %u %u %u srcOfCurrLs %u\n'
             '  currLs %d srcOfLsChange %u lsChange %d timeToLsEvent %d\n'
             '  dateOfLsGpsWn %u dateOfLsGpsDn %u reserved2 %u %u %u\n'
             '  valid x%x' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   srcOfCurrLs (%s) srcOfLsChange (%s)"
                  "\n   valid (%s)" %
                  (index_s(u[5], self.nav_timels_srcOfCurrLs),
                   index_s(u[7], self.nav_timels_srcOfLsChange),
                   flag_s(u[15], self.nav_timels_valid)))
        return s

    nav_timenavic_valid = {
        1: "NavicTowValid",
        2: "NavicWnoValid",
        4: "leapSValid",
        }

    def nav_timenavic(self, buf):
        """UBX-NAV-TIMENAVIC decode"""

        u = struct.unpack_from('<LLlhbBL', buf, 0)
        s = ("  iTOW %u NavicTow %u fNavicTow %d navicWn %d leapS %d\n"
             "  Valid %#x tAcc %u" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   valid (%s)" %
                  (flag_s(u[5], self.nav_timenavic_valid)))
        return s

    nav_timeqzss_valid = {
        1: "qzssTowValid",
        2: "qzssWnoValid",
        4: "leapSValid",
        }

    def nav_timeqzss(self, buf):
        """UBX-NAV-TIMEQZSS decode, QZSS time solution

protVer 34 and up
"""

        u = struct.unpack_from('<LLlhbBL', buf, 0)
        s = ("  iTOW %u qzssTow %u fQzssTow %d qzssWno %d leapS %d\n"
             "  valid x%x tAcc %d" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   valid (%s)" %
                  (flag_s(u[5], self.nav_timeqzss_valid)))
        return s

    nav_timetrusted_refSys = {
        0: "none",
        1: "GPS",
        2: "GST",
        3: "BDT",
        15: "IRNSS",     # NavIC
        }

    nav_timetrusted_valid = {
        1: "trustedTimeValid",
        2: "deltaTimeValid",
        }

    def nav_timetrusted(self, buf):
        """UBX-NAV-TIMETRUSTED decode, external trusted time info"""

        u = struct.unpack_from('<BBBBLHHLLLLlll', buf, 0)
        if 1 != u[0]:
            return "  Unknown version %u" % u[0]

        s = ("  version %u refSys %u valid x%x reserved0 %u iTOW %d\n"
             "  iniWno %u propWno %u iniTow %u propTow %u\n"
             "  iniTacc %d propTacc %u deltaS %d deltaMs %d "
             "reserved1 x%x" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   refSys (%s)"
                  "\n   valid (%s)" %
                  (index_s(u[2], self.nav_timetrusted_refSys),
                   flag_s(u[3], self.nav_timetrusted_valid)))
        return s

    nav_timeutc_valid = {
        1: "validTOW",
        2: "validWKN",
        4: "validUTC",
        8: "authStatus",
        }

    def nav_timeutc(self, buf):
        """UBX-NAV-TIMEUTC decode"""

        u = struct.unpack_from('<LLlHbbbbbB', buf, 0)
        s = ("  iTOW %u tAcc %u nano %d Time  %4u/%02u/%02u %02u:%02u:%02u\n"
             "  valid x%x" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n   valid (%s) utcStandard (%s)" %
                  (flag_s(u[9], self.nav_timeutc_valid),
                   index_s(u[9] >> 4, self.utc_std)))
        return s

    def nav_velecef(self, buf):
        """UBX-NAV-VELECEF decode"""

        # protVer 4+
        u = struct.unpack_from('<LlllL', buf, 0)
        return '  iTOW %d ecef: VX %d VY %d VZ %d vAcc:%u' % u

    def nav_velned(self, buf):
        """UBX-NAV-VELNED decode."""

        # protVer 15+
        u = struct.unpack_from('<LlllLLlLL', buf, 0)
        return ('  iTOW %u vel: N %d E %d D %d speed %u\n'
                '  gspeed %u heading %d sAcc %u cAcc %u' % u)

    # Broadcom calls this BRM-PVT-
    nav_ids = {0x01: {'str': 'POSECEF', 'dec': nav_posecef, 'minlen': 20,
                      'name': 'UBX-NAV-POSECEF'},
               0x02: {'str': 'POSLLH', 'dec': nav_posllh, 'minlen': 20,
                      'name': 'UBX-NAV-POSLLH'},
               0x03: {'str': 'STATUS', 'dec': nav_status, 'minlen': 16,
                      'name': 'UBX-NAV-STATUS'},
               # Broadcom calls this BRM-PVT-DOP
               0x04: {'str': 'DOP', 'dec': nav_dop, 'minlen': 18,
                      'name': 'UBX-NAV-DOP'},
               0x05: {'str': 'ATT', 'dec': nav_att, 'minlen': 32,
                      'name': 'UBX-NAV-ATT'},
               0x06: {'str': 'SOL', 'dec': nav_sol, 'minlen': 52,
                      'name': 'UBX-NAV-SOL'},
               # Broadcom calls this BRM-PVT-PVT
               0x07: {'str': 'PVT', 'dec': nav_pvt, 'minlen': 84,
                      'name': 'UBX-NAV-PVT'},
               # Broadcom calls this BRM-PVT-ACC_DIST
               0x09: {'str': 'ODO', 'dec': nav_odo, 'minlen': 20,
                      'name': 'UBX-NAV-ODO'},
               # Broadcom calls this BRM-PVT-RESET_ACC_DIST
               0x10: {'str': 'RESETODO', 'dec': nav_resetodo, 'minlen': 0,
                      'name': 'UBX-NAV-RESETODO'},
               0x11: {'str': 'VELECEF', 'dec': nav_velecef, 'minlen': 20,
                      'name': 'UBX-NAV-VELECEF'},
               0x12: {'str': 'VELNED', 'dec': nav_velned, 'minlen': 36,
                      'name': 'UBX-NAV-VELNED'},
               0x13: {'str': 'HPPOSECEF', 'dec': nav_hpposecef, 'minlen': 28,
                      'name': 'UBX-NAV-HPPOSECEF'},
               0x14: {'str': 'HPPOSLLH', 'dec': nav_hpposllh, 'minlen': 36,
                      'name': 'UBX-NAV-HPPOSLLH'},
               0x17: {'str': 'PVAT', 'dec': nav_pvat, 'minlen': 116,
                      'name': 'UBX-NAV-PVAT'},
               0x20: {'str': 'TIMEGPS', 'dec': nav_timegps, 'minlen': 16,
                      'name': 'UBX-NAV-TIMEGPS'},
               0x21: {'str': 'TIMEUTC', 'dec': nav_timeutc, 'minlen': 20,
                      'name': 'UBX-NAV-TIMEUTC'},
               0x22: {'str': 'CLOCK', 'dec': nav_clock, 'minlen': 20,
                      'name': 'UBX-NAV-CLOCK'},
               0x23: {'str': 'TIMEGLO', 'dec': nav_timeglo, 'minlen': 20,
                      'name': 'UBX-NAV-TIMEGLO'},
               0x24: {'str': 'TIMEBDS', 'dec': nav_timebds, 'minlen': 20,
                      'name': 'UBX-NAV-TIMEBDS'},
               0x25: {'str': 'TIMEGAL', 'dec': nav_timegal, 'minlen': 20,
                      'name': 'UBX-NAV-TIMEGAL'},
               0x26: {'str': 'TIMELS', 'dec': nav_timels, 'minlen': 24,
                      'name': 'UBX-NAV-TIMELS'},
               0x27: {'str': 'TIMEQZSS', 'dec': nav_timeqzss, 'minlen': 20,
                      'name': 'UBX-NAV-TIMEQZSS'},
               0x30: {'str': 'SVINFO', 'dec': nav_svinfo, 'minlen': 8,
                      'name': 'UBX-NAV-SVINFO'},
               0x31: {'str': 'DGPS', 'dec': nav_dgps, 'minlen': 16,
                      'name': 'UBX-NAV-DGPS'},
               0x32: {'str': 'SBAS', 'dec': nav_sbas, 'minlen': 12,
                      'name': 'UBX-NAV-SBAS'},
               0x34: {'str': 'ORB', 'dec': nav_orb, 'minlen': 8,
                      'name': 'UBX-NAV-ORB'},
               # Broadcom calls this BRM-PVT-SAT
               0x35: {'str': 'SAT', 'dec': nav_sat, 'minlen': 8,
                      'name': 'UBX-NAV-SAT'},
               0x36: {'str': 'COV', 'dec': nav_cov, 'minlen': 64,
                      'name': 'UBX-NAV-COV'},
               0x39: {'str': 'GEOFENCE', 'dec': nav_geofence, 'minlen': 8,
                      'name': 'UBX-NAV-GEOFENCE'},
               0x3B: {'str': 'SVIN', 'dec': nav_svin, 'minlen': 40,
                      'name': 'UBX-NAV-SVIN'},
               # M8P length = 40, M9P length = 64
               0x3C: {'str': 'RELPOSNED', 'dec': nav_relposned, 'minlen': 40,
                      'name': 'UBX-NAV-RELPOSNED'},
               # protVer 19.1 and up, ADR, MDR, HPS only
               0x3d: {'str': 'EELL', 'dec': nav_eell, 'minlen': 16,
                      'name': 'UBX-NAV-EELL'},
               # deprecated in u-blox 6, SFDR only
               0x40: {'str': 'EKFSTATUS', 'minlen': 36,
                      'name': 'UBX-NAV-EKFSTATUS'},
               0x42: {'str': 'SLAS', 'dec': nav_slas, 'minlen': 20,
                      'name': 'UBX-NAV-SLAS'},
               0x43: {'str': 'SIG', 'dec': nav_sig, 'minlen': 8,
                      'name': 'UBX-NAV-SIG'},
               0x60: {'str': 'AOPSTATUS', 'dec': nav_aopstatus, 'minlen': 16,
                      'name': 'UBX-NAV-AOPSTATUS'},
               # Broadcom calls this BRM-PVT-EOE
               0x61: {'str': 'EOE', 'dec': nav_eoe, 'minlen': 4,
                      'name': 'UBX-NAV-EOE'},
               0x62: {'str': 'PL', 'dec': nav_pl, 'minlen': 52,
                      'name': 'UBX-NAV-PL'},
               0x63: {'str': 'TIMENAVIC', 'dec': nav_timenavic, 'minlen': 20,
                      'name': 'UBX-NAV-TIMENAVIC'},
               # protVer 29, F10T
               # protVer 50, X20
               0x64: {'str': 'TIMETRUSTED', 'dec': nav_timetrusted,
                      'minlen': 40, 'name': 'UBX-NAV-TIMETRUSTED'},
               }

    def nav2_clock(self, buf):
        """UBX-NAV2-CLOCK decode, Clock Solution

Duplicate of UBX-NAV-CLOCK
"""

        return nav_clock(self, buf)

    # F9T
    nav2_ids = {0x01: {'str': 'POSECEF', 'minlen': 20,
                       'name': 'UBX-NAV2-POSECEF'},
                0x02: {'str': 'POSLLH',  'minlen': 20,
                       'name': 'UBX-NAV2-POSLLH'},
                0x03: {'str': 'STATUS',  'minlen': 16,
                       'name': 'UBX-NAV2-STATUS'},
                0x04: {'str': 'DOP', 'minlen': 18,
                       'name': 'UBX-NAV2-DOP'},
                0x07: {'str': 'PVT',  'minlen': 92,
                       'name': 'UBX-NAV2-PVT'},
                0x11: {'str': 'VELECEF',  'minlen': 20,
                       'name': 'UBX-NAV2-VELECEF'},
                0x12: {'str': 'VELNED', 'minlen': 36,
                       'name': 'UBX-NAV2-VELNED'},
                0x20: {'str': 'TIMEGPS', 'minlen': 16,
                       'name': 'UBX-NAV2-TIMEGPS'},
                0x21: {'str': 'TIMEUTC',  'minlen': 20,
                       'name': 'UBX-NAV2-TIMEUTC'},
                0x22: {'str': 'CLOCK', 'dec': nav2_clock, 'minlen': 20,
                       'name': "UBX-NAV2-CLOCK"},
                0x24: {'str': 'TIMEBDS', 'minlen': 20,
                       'name': 'UBX-NAV2-TIMEBDS'},
                0x25: {'str': 'TIMEGAL', 'minlen': 20,
                       'name': 'UBX-NAV2-TIMEGAL'},
                0x26: {'str': 'TIMELS', 'minlen': 24,
                       'name': 'UBX-NAV2-TIMELS'},
                0x27: {'str': 'TIMEQZSS',  'minlen': 20,
                       'name': 'UBX-NAV2-QZSS'},
                0x32: {'str': 'SBAS',  'minlen': 12,
                       'name': 'UBX-NAV2-SBAS'},
                0x35: {'str': 'SAT',  'minlen': 8,
                       'name': 'UBX-NAV2-SAT'},
                0x36: {'str': 'COV', 'minlen': 64,
                       'name': 'UBX-NAV2-COV'},
                0x42: {'str': 'SLAS', 'minlen': 20,
                       'name': 'UBX-NAV2-SLAS'},
                0x43: {'str': 'SIG', 'minlen': 8,
                       'name': 'UBX-NAV2-SIG'},
                0x61: {'str': 'EOE', 'minlen': 4,
                       'name': 'UBX-NAV2-EOE'},
                0x63: {'str': 'TIMENAVIC', 'minlen': 20,
                       'name': 'UBX-NAV2-TIMENAVIC'},
                }

    # used for RTCM3 rate config
    rtcm_ids = {5: {'str': '1005'},
                0x4a: {'str': '1074'},
                0x4d: {'str': '1077'},
                0x54: {'str': '1084'},
                0x57: {'str': '1087'},
                0x61: {'str': '1097'},
                0x7c: {'str': '1124'},
                0x7f: {'str': '1127'},
                0xe6: {'str': '1230'},
                0xfd: {'str': '4072-1'},
                0xfe: {'str': '4072-0'},
                }

    # used for NMEA rate config
    nmea_ids = {0: {'str': 'GGA'},
                1: {'str': 'GLL'},
                2: {'str': 'GSA'},
                3: {'str': 'GSV'},
                4: {'str': 'RMC'},
                5: {'str': 'VTG'},
                6: {'str': 'GRS'},
                7: {'str': 'GST'},
                8: {'str': 'ZDA'},
                9: {'str': 'GBS'},
                0x0a: {'str': 'DTM'},
                0x0d: {'str': 'GNS'},
                0x0f: {'str': 'VLW'},
                0x40: {'str': 'GPQ'},
                0x41: {'str': 'TXT'},
                0x42: {'str': 'GNQ'},
                0x43: {'str': 'GLQ'},
                0x44: {'str': 'GBQ'},
                0x45: {'str': 'GAQ'},
                }

    def rxm_imes(self, buf):
        """UBX-RXM-IMES decode, Indoor Messaging System Information

Removed in protVer 32.00
"""

        # not supported in M1
        u = struct.unpack_from('<BBH', buf, 0)
        s = ' numTx %u version %u reserved1 %u' % u

        for i in range(0, u[0]):
            u = struct.unpack_from('<BBHBBHlLLLllLLL', buf, 4 + (i * 44))
            s += ('\n  reserved %u txId %u reserved3 %u %u cno %u reserved4 %u'
                  '\n doppler %d position1_1 x%x position1_2 x%x'
                  '\n position2_1 x%x lat %d lon %d shortIdFrame x%x'
                  '\n mediumIdLSB %u mediumId_2 x%x' % u)

        return s

    def rxm_measx(self, buf):
        """UBX-RXM-RAW decode"""
        m_len = len(buf)

        u = struct.unpack_from('<BBBBLLLLLHHHHHBBLL', buf, 0)
        s = (' version %u reserved1 %u %u %u gpsTOW %u gloTOW %u\n'
             ' bdsTOW %u reserved2 %u qzssTOW %u gpsTOWacc %u\n'
             ' gloTOWacc %u bdsTOWacc %u reserved3 %u qzssTOWacc %u\n'
             ' numSV %u flags %#x reserved4 %u %u' % u)

        m_len -= 44
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<BBBBllHHLBBH', buf, 44 + i * 24)
            s += ('\n  gnssId %u svId %u cNo %u mpathIndic %u DopplerMS %d\n'
                  '    dopplerHz %d wholeChips %u fracChips %u codephase %u\n'
                  '    intCodePhase %u pseudoRangeRMSErr %u reserved5 %u' % u)
            m_len -= 24
            i += 1

        return s

    rxm_pmreq_flags = {
        1: "backup",
        2: "force",
        }

    rxm_pmreq_wakeup = {
        4: "uartrx",
        0x10: "extint0",
        0x20: "extint1",
        0x40: "spics",
        }

    def rxm_pmreq(self, buf):
        """UBX-RXM-PMREQ decode Power management request

protVer 34 and up
"""

        m_len = len(buf)
        if 4 == m_len:
            # short poll
            u = struct.unpack_from('<Ll', buf, 0)
            s = ' duration %u flags %u' % u
            if gps.VERB_DECODE < self.verbosity:
                s += '\n flags (%s)' % flag_s(u[1], self.rxm_pmreq_flags)
        elif 16 == m_len:
            # long  poll
            u = struct.unpack_from('<BBHLLL', buf, 0)
            s = (' version %u reserved %u %u duration %u flags x%x\n'
                 ' wakeupSources x%x' % u)
            if gps.VERB_DECODE < self.verbosity:
                s += ('\n flags (%s) wakeupSources (s)' %
                      (flag_s(u[4], self.rxm_pmreq_flags),
                       flag_s(u[5], self.rxm_pmreq_wakeup)))
        else:
            s = "  Bad Length %s" % m_len
        return s

    def rxm_raw(self, buf):
        """UBX-RXM-RAW decode"""
        m_len = len(buf)

        u = struct.unpack_from('<lhBB', buf, 0)
        s = ' iTOW %d weeks %d numSV %u res1 %u' % u

        m_len -= 8
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<ddfBbbB', buf, 8 + i * 24)
            s += ('\n  cpMes %f prMes %f doMes %f sv %d mesQI %d\n'
                  '     eno %d lli %d' % u)
            m_len -= 24
            i += 1

        return s

    rxm_rawx_recs = {
        1: "leapSec",
        2: "clkReset",
        }

    rxm_rawx_trkStat = {
        1: "prValid",
        2: "cpValid",
        4: "halfCyc",
        8: "subHalfCyc",
        }

    def rxm_rawx(self, buf):
        """UBX-RXM-RAWX decode"""
        m_len = len(buf)

        # version not here before protver 18, I hope it is zero.
        u = struct.unpack_from('<dHbBBBH', buf, 0)
        s = (' rcvTow %.3f week %u leapS %d numMeas %u recStat %#x'
             ' version %u\n'
             ' reserved1 x%x\n  recStat (' % u)
        s += flag_s(u[4], self.rxm_rawx_recs) + ')'

        m_len -= 16
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<ddfBBBBHBBBBBB', buf, 16 + i * 32)
            s += ('\n  prmes %.3f cpMes %.3f doMes %f\n'
                  '   gnssId %u svId %u sigId %u freqId %u locktime %u '
                  'cno %u\n'
                  '   prStdev %u cpStdev %u doStdev %u trkStat %u '
                  'reserved1 x%x' % u)

            if gps.VERB_DECODE <= self.verbosity:
                s += ('\n      (%s) trkStat (%s)' %
                      (self.gnss_s(u[3], u[4], u[5]),
                       flag_s(u[12], self.rxm_rawx_trkStat)))

            m_len -= 32
            i += 1
        return s

    def rxm_rlm(self, buf):
        """UBX-RXM-RLM decode, Galileo SAR RLM report"""
        m_len = len(buf)

        # common to Short-RLM and Long-RLM report
        u = struct.unpack_from('<BBBBLLB', buf, 0)
        s = ("  version %u type %u svId %u reserved1 %u beacon x%x %x "
             " message %u" % u)
        if 16 == m_len:
            # Short-RLM report
            u = struct.unpack_from('<BBB', buf, 13)
            s += "\n  params %u %u reserved2 %u" % u
        elif 28 == m_len:
            # Long-RLM report
            u = struct.unpack_from('<BBBBBBBBBBBBBBB', buf, 13)
            s += ("\n  params %u %u %u %u %u %u %u %u %u %u %u %u"
                  "\n  reserved2 %u %u %u" % u)

        return s

    rxm_rtcm_flags = {
        1: "crcFailed",
        }

    def rxm_rtcm(self, buf):
        """UBX-RXM-RTCM decode, RTCM Input Status"""

        # present in some u-blox 8 and 9, protVer 20+
        # undocumented, but in NEO-M9N, protVer 32
        u = struct.unpack_from('<BBHHH', buf, 0)
        s = "  version %u flags x%x subtype %u refstation %u msgtype %u" % u
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n    flags (%s)' % flag_s(u[1], self.rxm_rtcm_flags))
        return s

    def rxm_sfrb(self, buf):
        """UBX-RXM-SFRB decode, Subframe Buffer"""

        u = struct.unpack_from('<BBLLLLLLLLLL', buf, 0)
        s = ('  chn %d s svid %3d\n'
             '  dwrd %08x %08x %08x %08x %08x\n'
             '       %08x %08x %08x %08x %08x' % u)

        return s

    rxm_spartn_flags = {
        0: "Unknown",
        1: "Not Used",
        2: "Used",
        3: "Reserved",
    }

    rxm_spartn_type = {
        0: "Orbit",
        1: "HPAC",
        2: "GAD",
        3: "BDS",
    }

    rxm_spartn_subtype = {
        0: "GPS",
        1: "GLO",
        2: "GAL",
        3: "BDS",
    }

    def rxm_spartn(self, buf):
        """UBX-RXM-SPARTN decode, SPAARTN status

present in protVer 27.5, F9R
"""

        u = struct.unpack_from('<BBHHH', buf, 0)
        if 1 != u[0]:
            return "  Unknown version %u" % u[0]

        s = ' version %u flags x%x subType %u reserved0 x%x msgType %u' % u
        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    flags %s type %s subtype %s" %
                  (index_s((u[1] >> 1) & 3, self.rxm_spartn_flags),
                   index_s(u[4], self.rxm_spartn_type),
                   index_s(u[2], self.rxm_spartn_subtype)))

        return s

    def rxm_spartnkey(self, buf):
        """UBX-RXM-SPARTNKEY decode, Get dynamic SPAARTN keys

present in protVer 27.5, F9R
"""

        u = struct.unpack_from('<BBH', buf, 0)
        if 1 != u[0]:
            return "  Unknown version %u" % u[0]

        s = ' version %u numKeys %u reserved1 %u' % u

        # FIXME: decode keys.

        return s

    # decode GPS subframe 5, pages 1 to 24,
    # and subframe 4, pages 2 to 5, and 7 to 10
    def almanac(self, words):
        """Decode GPS Almanac"""

        # [1] Section 20.3.3.5, Figure 20-1 Sheet 4, and Table 20-VI.
        # Current almanac: https://www.navcen.uscg.gov/?pageName=gpsAlmanacs

        # e = Eccentricity
        # toa = Almanac reference time
        # deltai =
        # Omegadot = Rate of Right Ascension
        # SVH = SV Health
        # sqrtA = Square Root of the Semi-Major Axis
        # Omega0 = Longitude of Ascending Node of Orbit Plane at Weekly Epoch
        # omega = Argument of Perigee
        # M0 = Mean Anomaly at Reference Time
        # af0 = SV Clock Bias Correction Coefficient
        # af1 = SV Clock Drift Correction Coefficient
        s = "    Almanac"
        s += ("\n    e %e toa %u deltai %e Omegadot %e"
              "\n    SVH x%x sqrtA %.6f Omega0 %e omega %e"
              "\n    M0 %e af0 %e af1 %e" %
              (unpack_u16(words[2], 6) * (2 ** -21),
               unpack_u8(words[3], 22) * (2 ** 12),
               unpack_s16(words[3], 6) * (2 ** -19),
               unpack_s16(words[4], 14) * (2 ** -38),
               unpack_u8(words[4], 6),
               unpack_u24(words[5], 6) * (2 ** -11),     # sqrtA
               unpack_s24(words[6], 6) * (2 ** -23),
               unpack_s24(words[7], 6) * (2 ** -23),
               unpack_s24(words[8], 6) * (2 ** -23),     # M0
               unpack_s11s(words[9]) * (2 ** -20),
               unpack_s11(words[9], 11) * (2 ** -38)))
        return s

    def _decode_sfrbx_bds(self, words, sigId):
        """Decode UBX-RXM-SFRBX BeiDou frames

See u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
Section 10.4 BeiDou
gotta decode the u-blox munging and the BeiDou packing...
http://en.beidou.gov.cn/SYSTEMS/ICD/
BeiDou Interface Control Document v1.0
"""

        # gnssId 3 sigId 0/1 (BDS B1I D1 and D2) is 10, words
        # gnssId 3 sigId 6 (BDS B1 Cd), B-CNAV1, is 3, 9, or 19, words
        # gnssId 3 sigId 8 (BDS B2 ad) is 9 words
        #   aka: B-CNAV2 Navigation Message

        if 6 == sigId:
            # unmung u-blox 32 bit words in 32 bits
            page = 0
            for word in words:
                page <<= 32
                page |= word & 0x0ffffffff

            # BDS B1 Cd, B-CNAV1
            if len(words) == 3:
                # subframe 1, 72 bits raw, 14 bits encoded
                # unclear what u-blox did to get 3 32-bit words.
                PRN = (words[0] >> 26) & 0x03f
                SOH = (words[0] & 0x0ff) * 18          # WAG????
                return "\n    Subframe 1: PRN %u SOH %u" % (PRN, SOH)

            elif len(words) == 19:
                # Subframe 2, 600 bits + 8 bits padding == 19 32-bit words
                WN = (page >> 595) & 0x01fff
                HOW = (page >> 587) & 0x0ff
                IODC = (page >> 577) & 0x03ff
                Rev = (page >> 32) & 0x07f
                s = ("\n    Subframe 2: WN %u HOW %u IODC %u Rev %u" %
                     (WN, HOW, IODC, Rev))
                return s

            elif len(words) == 9:
                # Subframe 3, 264 bits + 24 bits padding == 9 32-bit words
                # 1,2,4,5, and 60
                PageID = (page >> 282) & 0x03f
                DIF = (page >> 281) & 1
                SIF = (page >> 280) & 1
                AIF = (page >> 279) & 1
                SISMAI = (page >> 275) & 0x0f   # undecumented
                s = ("\n    Subframe 3: PageID %u DIF %u "
                     "SIF %u AIF %u SISMAI %u " %
                     (PageID, DIF, SIF, AIF, SISMAI))
                if 1 == PageID:
                    SISAIoe = (page >> 270) & 0x1f
                    s += ("SISAIoe %u " %
                          (SISAIoe))
                elif 2 == PageID:
                    SISAIoc = (page >> 253) & 0x03fffff
                    s += ("SISAIoe %u " %
                          (SISAIoc))
                elif 3 == PageID:
                    SISAIoe = (page >> 270) & 0x1f
                    s += ("SISAIoe %u " %
                          (SISAIoe))
                elif 4 == PageID:
                    SISAIoc = (page >> 253) & 0x03fffff
                    s += ("SISAIoe %u " %
                          (SISAIoc))
                # else: page 60 is a mystery.
                Rev = (page >> 32) & 0x03ff
                s += "Rev %u" % Rev
                return s

            # We only know the 9 == words case
            return "\n    Unknown words number %u" % len(words)

        if 8 == sigId:
            # BDS B2 ad, B-CNAV2
            if len(words) != 9:
                # We only know the 9 == words case
                return "\n    Number of words error! %u != 9" % len(words)

            # unmung u-blox 32 bit words in 32 bits
            page = 0
            for word in words:
                page <<= 32
                page |= word & 0x0ffffffff

            PRN = (page >> 282) & 0x3f
            mtype = (page >> 276) & 0x3f
            SOW = ((page >> 258) & 0x03ffff)
            HS = (page >> 256) & 3
            DIF = (page >> 255) & 1
            SIF = (page >> 254) & 1
            AIF = (page >> 253) & 1
            SISMAI = (page >> 249) & 0x0f   # undecumented
            s = ("\n    PRN %u mtype %u SOW %u HS %u DIF %u SIF %u AIF %u "
                 "SUSMAI %u" % (PRN, mtype, SOW, HS, DIF, SIF, AIF, SISMAI))

            if 10 == mtype:
                WN = (page >> 245) & 0x01fff
                s += " WN %u" % (WN)
            elif mtype in set([11, 30, 31, 32, 33, 34, 40]):
                HS = (page >> 256) & 3
                if gps.VERB_DECODE <= self.verbosity:
                    s += ' HS %u(%s)' % (HS, self.hs_vals.get(HS, '?'))
                else:
                    s += " HS %u " % (HS)
            else:
                s += " Unknown mtype"
            return s

        # unmung u-blox 30 bit words in 32 bits
        page = 0
        for word in words:
            page <<= 30
            page |= word & 0x03fffffff

        if sigId not in set([0, 1]):
            return ("\n    Can't handle sigId %u wwrds %u page %x" %
                    (sigId, len(words), page))

        if len(words) != 10:
            # We only know the 10 == words case
            return "\n    Number of words error! %u != 10" % len(words)

        # the following only for B1I D1
        Rev = (words[0] >> 15) & 0x0f
        FraID = (words[0] >> 12) & 7

        # sanity check
        if (((page >> 282) & 7) != FraID):
            s = ("\n    BDS: Math Error! %u != %u"
                 "\n      page %u" %
                 (FraID, (page >> 282) & 7, page))
            return s

        # common to all pages
        SOW = ((page >> 274) & 0x0f) << 12
        SOW |= (page >> 258) & 0x0fff
        s = ("\n    Rev %u FraID %i SOW %u" %
             (Rev, FraID, SOW))
        if 1 == FraID:
            SatH1 = (page >> 257) & 1
            AODC = (page >> 252) & 0x01f
            URAI = (page >> 248) & 0x0f
            WN = (page >> 227) & 0x01fff
            t0c = ((page >> 218) & 0x01ff) << 8
            t0c |= (page >> 202) & 0x0ff
            TGD1 = (page >> 192) & 0x3ff
            TGD2 = ((page >> 188) & 0x0f) << 6
            TGD2 |= (page >> 174) & 0x03f
            alpha0 = (page >> 166) & 0x0ff
            alpha1 = (page >> 158) & 0x0ff
            alpha2 = (page >> 142) & 0x0ff
            alpha3 = (page >> 134) & 0x0ff
            beta0 = ((page >> 128) & 0x03f) << 2
            beta0 |= (page >> 118) & 3
            beta1 = (page >> 110) & 0x0ff
            beta2 = (page >> 102) & 0x0ff
            beta3 = ((page >> 98) & 0x0f) << 4
            beta3 |= (page >> 86) & 0x0f
            a2 = (page >> 75) & 0x07ff
            a0 = ((page >> 68) & 0x07f) << 17
            a0 |= (page >> 43) & 0x01ffff
            a1 = ((page >> 38) & 0x01f) << 17
            a1 |= (page >> 13) & 0x01ffff
            AODE = (page >> 8) & 0x01f
            s += ("\n    SatH1 %u AODC %u URAI %u WN %u t0c %u TGD1 %u "
                  "TGD2 %u"
                  "\n      alpha0 %u alpha1 %u alpha2 %u alpha3 %u"
                  "\n      beta0 %u beta1 %u beta2 %u beta3 %u"
                  "\n      a2 %u a0 %u a1 %u AODE %u" %
                  (SatH1, AODC, URAI, WN, t0c, TGD1, TGD2, alpha0, alpha1,
                   alpha2, alpha3, beta0, beta1, beta2, beta3,
                   a2, a0, a1, AODE))
        elif 2 == FraID:
            deltan = ((page >> 248) & 0x03ff) << 6
            deltan |= (page >> 234) & 0x03f
            Cuc = ((page >> 218) & 0x0ffff) << 2
            Cuc |= (page >> 210) & 3
            M0 = ((page >> 188) & 0x0fffff) << 12
            M0 |= (page >> 168) & 0x0fff
            e = ((page >> 158) & 0x03ff) << 22
            e |= (page >> 128) & 0x03fffff
            Cus = (page >> 102) & 0x03ffff
            Crc = ((page >> 98) & 0x0f) << 14
            Crc |= (page >> 76) & 0x03fff
            Crs = ((page >> 68) & 0x0f) << 10
            Crs |= (page >> 50) & 0x03ff
            sqrtA = ((page >> 38) & 0x0fff) << 20
            sqrtA |= (page >> 10) & 0x0fffff
            toeMSB = (page >> 8) & 3
            s += ("\n    deltan %u Cuc %u M0 %u e %u Cus %u Crc %u"
                  "\n    Crs %u sqrtA %u toeMSB %u" %
                  (deltan, Cuc, M0, e, Cus, Crc, Crs, sqrtA, toeMSB))
        elif 3 == FraID:
            toeLSB = ((page >> 248) & 0x03ff) << 5
            toeLSB |= (page >> 235) & 0x01f
            i0 = ((page >> 218) & 0x01ffff) << 15
            i0 |= (page >> 195) & 0x07fff
            Cic = ((page >> 188) & 0x07f) << 11
            Cic |= (page >> 137) & 0x07fff
            Omegadot = ((page >> 158) & 0x07ff) << 13
            Omegadot |= (page >> 137) & 0x01fff
            Cis = ((page >> 128) & 0x01ff) << 9
            Cis |= (page >> 111) & 0x01ff
            IDOT = ((page >> 98) & 0x01fff) << 1
            IDOT |= (page >> 49) & 1
            Omega0 = ((page >> 68) & 0x01fffff) << 11
            Omega0 |= (page >> 49) & 0x07ff
            omega = ((page >> 38) & 0x07ff) << 21
            omega |= (page >> 9) & 0x01fffff
            Rev = (page >> 8) & 1
            s += ("\n    toeLSB %u i0 %u Cic %u Omegadot %u Cis %u"
                  "\n    IDOT %u Omega0 %u omega %u Rev %u" %
                  (toeLSB, i0, Cic, Omegadot, Cis, IDOT, Omega0, omega, Rev))
        elif FraID in [4, 5]:
            Pnum = (page >> 250) & 0x07f
            s += "\n    Pnum %u: " % Pnum
            if (((4 == FraID and (1 <= Pnum <= 24)) or
                 (1 <= Pnum <= 6) or
                 (11 <= Pnum <= 23))):
                # Subfram 4, page 1 to 24: Almanac
                # Subfram 5, page 1 to 6: Almanac
                # Subfram 5, page 11 to 23: maybe Almanac
                AmEpID = (page >> 8) & 3
                if 3 != AmEpID:
                    # not Almanac
                    s += "Reserved AmEpID %u" % AmEpID
                else:
                    sqrtA = ((page >> 248) & 3) << 22
                    sqrtA |= (page >> 218) & 0x03fffff
                    a1 = (page >> 199) & 0x07ff
                    a0 = (page >> 188) & 0x07ff
                    Omega0 = ((page >> 158) & 0x3fffff) << 2
                    Omega0 |= (page >> 148) & 3
                    e = (page >> 131) & 0x01ffff
                    deltai = ((page >> 128) & 3) << 13
                    deltai |= (page >> 107) & 0x01ffff
                    t0a = (page >> 99) & 0x0ff
                    Omegadot = ((page >> 98) & 1) << 16
                    Omegadot |= (page >> 74) & 0x0ffff
                    omega = ((page >> 68) & 0x03f) << 18
                    omega |= (page >> 42) & 0x03ffff
                    M0 = ((page >> 38) & 0x0f) << 20
                    M0 |= (page >> 10) & 0x0fffff
                    s += ("Almanac; sqrtA %u a1 %u a0 %u Omega0 %u"
                          "\n         e %u deltai %u t0a %u Omegadot %u"
                          "\n         omega %u M0 %u AmEpID %u" %
                          (sqrtA, a1, a0, Omega0, e, deltai, t0a, Omegadot,
                           omega, M0, AmEpID))
            elif 5 == FraID:
                if Pnum in [7, 8, 24]:
                    # make a packed integer
                    hlth = 0
                    for i in range(0, 10):
                        hlth <<= 22
                        # remove top two random bits and last 8 bits parity
                        hlth |= (words[i] & 0x3fffffff) >> 8

                if 7 == Pnum:
                    s += "Health 1 to 19:\n   "
                    # remove 7 reserved bits from last word
                    hlth >>= 7
                    for i in range(1, 20):
                        # take 9 bits at a time from the top
                        h = (hlth >> ((19 - i) * 9)) & 0x1ff
                        s += " %3x" % h
                elif 8 == Pnum:
                    # remove 63 reserved bits from LSBs
                    hlth >>= 63
                    WNa = (hlth >> 8) & 0x0ff
                    t0a = hlth & 0x0ff
                    # Hea20 to Hea30 now in the LSB
                    hlth >>= 16
                    s += "Health 20 to 30 WNa %u t0a %u\n       " % (WNa, t0a)
                    for i in range(20, 31):
                        # take 9 bits at a time from the top
                        h = (hlth >> ((30 - i) * 9)) & 0x1ff
                        s += " %3x" % h
                elif 9 == Pnum:
                    A0GPS = (page >> 106) & 0x03fff
                    A1GPS = ((page >> 188) & 0x03) << 14
                    A1GPS |= (page >> 166) & 0x03fff
                    A0GAL = ((page >> 158) & 0x0ff) << 6
                    A0GAL |= (page >> 144) & 0x03f
                    A1GAL = (page >> 128) & 0x0ffff
                    A0GLO = (page >> 106) & 0x03fff
                    A1GLO = ((page >> 98) & 0x0f) << 8
                    A1GLO |= (page >> 82) & 0x0f
                    s += ("Timing A0GPS %u A1GPS %u A0GAL %u A1GAL %u"
                          "\n       A0GLO %u A1GLO %u" %
                          (A0GPS, A1GPS, A0GAL, A1GAL, A0GLO, A1GLO))
                elif 10 == Pnum:
                    deltatLS = ((page >> 248) & 0x03) << 6
                    deltatLS |= (page >> 234) & 0x03f
                    deltatLSF = (page >> 226) & 0x0ff
                    WNLSF = (page >> 218) & 0x0ff
                    A0UTC = ((page >> 188) & 0x03fffff) << 10
                    A0UTC |= (page >> 170) & 0x03ff
                    A1UTC = ((page >> 158) & 0x0fff) << 12
                    A1UTC |= (page >> 138) & 0x0fff
                    DN = (page >> 130) & 0x0ff
                    s += ("Timing: deltatLS %u deltatLSF %u WNLSF %u A0UTC %u"
                          " A1UTC %u" %
                          (deltatLS, deltatLSF, WNLSF, A0UTC, A1UTC))
                elif 24 == Pnum:
                    # ICD calls this AmID and AmEpID
                    AmEpID = (page >> 83) & 3
                    if 3 != AmEpID:
                        # not Almanac
                        s += "Reserved AmEpID %u" % AmEpID
                    else:
                        s += "Health 31 to 43: AmEpID %u" % AmEpID
                        # Hea31 to Hea43 now in the LSB
                        hlth >>= 85
                        s += "Health 31 to 43 t0a %u\n       " % SOW
                        for i in range(31, 44):
                            # take 9 bits at a time from the top
                            h = (hlth >> ((43 - i) * 9)) & 0x1ff
                            s += " %3x" % h
                else:
                    s += "Unknown page number"
            else:
                s += "Unknown page number"

        return s

    def _decode_sfrbx_gal(self, words):
        """Decode UBX-RXM-SFRBX Galileo I/NAV frames"""
        # Galileo_OS_SIS_ICD_v2.0.pdf
        # See u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
        # Section 10.5 Galileo
        # gotta decode the u-blox munging and the Galileo packing...

        if 8 > len(words):
            return "\n    GAL: runt message, len %u" % len(words)

        s = ""
        if 8 != len(words):
            s = "\n    GAL: long message? len %u" % len(words)

        # always zero on E5b-I, always 1 on E1-B
        even = words[0] >> 31
        # zero for nominal page, one for alert page
        page_type = (words[0] >> 30) & 1
        word_type = (words[0] >> 24) & 0x03f
        s += ("\n    GAL: even %u page_type %u word_type %u" %
              (even, page_type, word_type))

        if (1 == page_type):
            # Alerts pages are all "Reserved"
            s += "\n    Alert page"
            return s

        if (1 == even):
            # page flipped!?
            s += "\n    page flipped!?"
            return s

        # untangle u-blox words into a Galileo 128 bit page
        # except we get 130 bits...
        # even (1), page (1), 128 data bits
        page = words[0] << 32
        page |= words[1]
        page <<= 32
        page |= words[2]
        page <<= 18
        page |= (words[3] >> 14) & 0x03ffff
        page <<= 16
        page |= (words[4] >> 14) & 0x0ffff

        # sanity check
        if (((page >> 122) & 0x3f) != word_type):
            s += "\n    Math Error!"
            return s

        # all unscaled
        if (0 == word_type):
            s += "\n    Spare Word"
            time = (page >> 120) & 3
            if 2 == time:
                # valid time
                WN = (page >> 20) & 0x0fff
                TOW = page & 0x0fffff
                s += " WN %u TOW %u" % (WN, TOW)

        elif (1 == word_type):
            IODnav = (page >> 112) & 0x03ff
            toe = (page >> 98) & 0x03fff
            M0 = (page >> 66) & 0x0ffffffff
            e = (page >> 34) & 0x0ffffffff
            sqrt_A = (page >> 2) & 0x0ffffffff
            s += ("\n    Ephemeris 1: IODnav %u toe %u M0 %u e %u  sqrt_A %u" %
                  (IODnav, toe, M0, e, sqrt_A))
        elif (2 == word_type):
            IODnav = (page >> 112) & 0x03ff
            Omega0 = (page >> 80) & 0x0ffffffff
            i0 = (page >> 48) & 0x0ffffffff
            omega = (page >> 16) & 0x0ffffffff
            i_dot = (page >> 2) & 0x03fff
            s += ("\n    Ephemeris 2: IODnav %u Omega0 %u i0 %u"
                  "\n       omega %u i_dot %u" %
                  (IODnav, Omega0, i0, omega, i_dot))
        elif (3 == word_type):
            IODnav = (page >> 112) & 0x03ff
            Omega_dot = (page >> 88) & 0x0ffffff
            delta_n = (page >> 72) & 0x0ffff
            Cuc = (page >> 56) & 0x0ffff
            Cus = (page >> 40) & 0x0ffff
            Crc = (page >> 24) & 0x0ffff
            Crs = (page >> 8) & 0x0ffff
            SISA = page & 0x0ff
            s += ("\n    Ephemeris 3: IODnav %u Omega_dot %u delta_n %u"
                  "\n       Cuc %u Cus %u Crs %u Crs %u SISA %u" %
                  (IODnav, Omega_dot, delta_n, Cuc, Cus, Crc, Crc, SISA))
        elif (4 == word_type):
            IODnav = (page >> 112) & 0x03ff
            SVID = (page >> 106) & 0x03f
            Cic = (page >> 90) & 0x0ffff
            Cis = (page >> 74) & 0x0ffff
            t0c = (page >> 60) & 0x03fff
            af0 = (page >> 29) & 0x07fffffff
            af1 = (page >> 8) & 0x01fffff
            af2 = (page >> 2) & 0x03f
            s += ("\n    Ephemeris 4: IODnav %u SVID %u Cic %u Cis %u"
                  "\n       t0c %u af0 %u af1 %u af2 %u" %
                  (IODnav, SVID, Cic, Cis, t0c, af0, af1, af2))
        elif (5 == word_type):
            Ax_af0 = (page >> 111) & 0x7ff
            Ax_af1 = (page >> 100) & 0x7ff
            Ax_af2 = (page >> 86) & 0x3fff
            Iono1 = (page >> 85) & 1
            Iono2 = (page >> 84) & 1
            Iono3 = (page >> 83) & 1
            Iono4 = (page >> 82) & 1
            Iono5 = (page >> 81) & 1
            BGD_E1E5a = (page >> 71) & 0x3ff
            BGD_E1E5b = (page >> 61) & 0x3ff
            E5BHS = (page >> 59) & 3
            E1BHS = (page >> 57) & 3
            E5BDVS = (page >> 56) & 1
            E1BDVS = (page >> 55) & 1
            WN = (page >> 43) & 0x0fff
            TOW = (page >> 23) & 0x0fffff
            s += ("\n    Ionosphere: Ax_af0 %u Ax_af1 %u Ax_af2 %u"
                  "\n       Iono1 %u Iono2 %u Iono3 %u Iono4 %u Iono5 %u"
                  "\n       BGD_E1E5a %u BGD_E1E5b %u E5BHS %u E1BHS %u"
                  "\n       E5BDVS %u E1BDVS %u WN %u TOW %u" %
                  (Ax_af0, Ax_af1, Ax_af2, Iono1, Iono2, Iono3, Iono4, Iono5,
                   BGD_E1E5a, BGD_E1E5b, E5BHS, E1BHS, E5BDVS, E1BDVS,
                   WN, TOW))
        elif (6 == word_type):
            A0 = (page >> 90) & 0x0ffffffff
            A1 = (page >> 66) & 0x0ffffff
            delta_tLS = (page >> 58) & 0x0ff
            t0t = (page >> 50) & 0x0ff
            WN0t = (page >> 42) & 0x0ff
            WNLSF = (page >> 34) & 0x0ff
            DN = (page >> 31) & 7
            delta_tLSF = (page >> 23) & 0x0ff
            TOW = (page >> 3) & 0x0fffff
            s += ("\n    GST-UTC: A0 %u A1 %u delta_tLS %u t0t %u WN0t %u"
                  "\n       WNLSF %u DN %u delta_tLSF %u TOW %u" %
                  (A0, A1, delta_tLS, t0t, WN0t, WNLSF, DN, delta_tLSF, TOW))
        elif (7 == word_type):
            IODa = (page >> 118) & 0x0f
            WNa = (page >> 116) & 0x03
            t0a = (page >> 106) & 0x03ff
            SVID1 = (page >> 100) & 0x03f
            delta_sqrtA = (page >> 87) & 0x01fff
            e = (page >> 76) & 0x07ff
            omega = (page >> 60) & 0x0ffff
            delta_i = (page >> 49) & 0x07ff
            Omage0 = (page >> 33) & 0x0ffff
            Omage_dot = (page >> 22) & 0x07ff
            M0 = (page >> 6) & 0x0ffff
            s += ("\n    Almanac SVID1 (1/2): IODa %u WNa %u t0a %u SVID1 %u"
                  "\n       delta_sqrtA %u e %u omega %u delta_i %u Omage0 %u"
                  "\n       Omage_dot %u M0 %u" %
                  (IODa, WNa, t0a, SVID1, delta_sqrtA, e, omega, delta_i,
                   Omage0, Omage_dot, M0))
        elif (8 == word_type):
            IODa = (page >> 118) & 0x0f
            af0 = (page >> 102) & 0x0ffff
            af1 = (page >> 89) & 0x01fff
            E5BHS = (page >> 87) & 3
            E1BHS = (page >> 85) & 3
            SVID2 = (page >> 79) & 0x03f
            delta_sqrtA = (page >> 66) & 0x01fff
            e = (page >> 55) & 0x07ff
            omega = (page >> 39) & 0x0ffff
            delta_i = (page >> 28) & 0x07ff
            Omage0 = (page >> 12) & 0x0ffff
            Omage_dot = (page >> 1) & 0x07ff
            s += ("\n    Almanac SVID1 (2/2): IODa %u af0 %u af1 %u E5BHS %u "
                  "E1BHS %u"
                  "\n       SVID2 %u delta_sqrtA %u e %u omega %u delta_i %u"
                  "\n       Omage0 %u Omage_dot %u" %
                  (IODa, af0, af1, E5BHS, E1BHS, SVID2, delta_sqrtA, e, omega,
                   delta_i, Omage0, Omage_dot))
        elif (9 == word_type):
            IODa = (page >> 118) & 0x0f
            WNa = (page >> 116) & 3
            t0a = (page >> 106) & 0x03ff
            M0 = (page >> 90) & 0x0ffff
            af0 = (page >> 74) & 0x0ffff
            af1 = (page >> 61) & 0x01fff
            E5BHS = (page >> 59) & 3
            E1BHS = (page >> 57) & 3
            SVID3 = (page >> 51) & 0x03f
            delta_sqrtA = (page >> 38) & 0x01fff
            e = (page >> 27) & 0x07ff
            omega = (page >> 11) & 0x0ffff
            delta_i = page & 0x07ff
            s += ("\n    Almanac SVID2 (2/2): IODa %u WNa %u t0a %u M0 %u"
                  "\n       af0 %u af1 %u E5BHS %u E1BHS %u"
                  "\n       SVID3 %u delta_sqrtA %u e %u omega %u delta_i %u" %
                  (IODa, WNa, t0a, M0, af0, af1, E5BHS, E1BHS, SVID3,
                   delta_sqrtA, e, omega, delta_i))
        elif (10 == word_type):
            IODa = (page >> 118) & 0x0f
            Omage0 = (page >> 102) & 0x0ffff
            Omage_dot = (page >> 91) & 0x07ff
            M0 = (page >> 75) & 0x0ffff
            af0 = (page >> 59) & 0x0ffff
            af1 = (page >> 46) & 0x01fff
            E5BHS = (page >> 44) & 3
            E1BHS = (page >> 42) & 3
            A0G = (page >> 26) & 0x0ffff
            A1G = (page >> 14) & 0x0fff
            t0G = (page >> 6) & 0x0ff
            WN0G = page & 0x3f
            s += ("\n    Almanac SVID3 (2/2): IODa %u Omage0 %u Omage_dot %u"
                  "\n       M0 %u af0 %u af1 %u E5BHS %u E1BHS %u"
                  "\n       A0G %u A1G %u t0G %u WN0G %u" %
                  (IODa, Omage0, Omage_dot, M0, af0, af1, E5BHS, E1BHS,
                   A0G, A1G, t0G, WN0G))
        elif (16 == word_type):
            deltaAred = (page >> 117) & 0x01f
            exred = (page >> 104) & 0x01fff
            eyred = (page >> 91) & 0x01fff
            deltai0red = (page >> 74) & 0x01ffff
            Omega0red = (page >> 51) & 0x07fffff
            lambda0red = (page >> 28) & 0x07fffff
            af0red = (page >> 6) & 0x03fffff
            af1red = page & 0x03f
            s += ("\n    Reduced Clock and Ephemeris Data: deltaAred %u"
                  "\n       exred %u eyred %u deltai0red %u Omega0red %u"
                  "\n       lambda0red %u af0red %u af1red %u" %
                  (deltaAred, exred, eyred, deltai0red, Omega0red,
                   lambda0red, af0red, af1red))
        elif (17 <= word_type and 20 >= word_type):
            s += "\n    FEC2 Reed-Solomon for Clock and Ephemeris Data"
        elif (63 == word_type):
            s += "\n    Dummy Page"

        return s

    def _decode_sfrbx_glo(self, words):
        """Decode UBX-RXM-SFRBX GLONASS frames

See u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
Section 10.3 GLONASS
L10F and L20F only
ICD_GLONASS_5.1_(2008)_en.pdf "ICD L1, L2 GLONASS"
gotta decode the u-blox munging and the GLONASS packing...
u-blox stripts preamble
"""
        stringnum = (words[0] >> 27) & 0x0f

        page = 0
        for i in range(0, 4):
            page <<= 32
            page |= words[i] & 0x0ffffffff

        # sanity check
        if (((page >> 123) & 0x0f) != stringnum):
            s = ("\n    GLO: Math Error! %u != %u"
                 "\n      page %u" %
                 (stringnum, (page >> 123) & 0xf, page))
            return s

        frame = page & 0x0ff
        superframe = (page >> 16) & 0x0ffff

        s = ("\n    GLO: superframe %u frame %u stringnum %u" %
             (superframe, frame, stringnum))
        if 1 == stringnum:
            P1 = (page >> 119) & 3
            tk = (page >> 107) & 0x0fff
            xnp = (page >> 83) & 0x0ffffffff
            xnpp = (page >> 78) & 0x01f
            xn = (page >> 51) & 0xa30ffffffff
            s += ("\n        Ephemeris 1: P1 %u tk %u xnp %u xnpp %u xn %u" %
                  (P1, tk, xnp, xnpp, xn))
        elif 2 == stringnum:
            Bn = (page >> 120) & 7
            P2 = (page >> 119) & 1
            tb = (page >> 112) & 0x07f
            ynp = (page >> 83) & 0x0ffffffff
            ynpp = (page >> 78) & 0x01f
            yn = (page >> 51) & 0xa30ffffffff
            s += ("\n        Ephemeris 2: Bn %u P2 %u tb %u ynp %u ynpp %u "
                  "yn %u" %
                  (Bn, P2, tb, ynp, ynpp, yn))
        elif 3 == stringnum:
            P3 = (page >> 122) & 1
            lambdan = (page >> 111) & 0x07fff
            p = (page >> 108) & 3
            ln = (page >> 107) & 1
            znp = (page >> 83) & 0x0ffffffff
            znpp = (page >> 78) & 0x01f
            zn = (page >> 51) & 0xa30ffffffff
            s += ("\n        Ephemeris 3: P3 %u znp %u znpp %u zn %u" %
                  (P3, znp, znpp, zn))
        elif 4 == stringnum:
            # n is SVID
            taun = (page >> 101) & 0x03ffffff
            deltataun = (page >> 96) & 0x01f
            En = (page >> 91) & 0x01f
            P4 = (page >> 76) & 1
            FT = (page >> 72) & 0x0f
            NT = (page >> 58) & 0x03fff
            n = (page >> 53) & 0x1f
            M = (page >> 51) & 3
            s += ("\n        Ephemeris 4: taun %u deltataun %u En %u P4 %u"
                  "\n           FT %u NT %u n %u M %u" %
                  (taun, deltataun, En, P4, FT, NT, n, M))
        elif 5 == stringnum:
            NA = (page >> 112) & 0x07ff
            tauc = (page >> 80) & 0x0ffffffff
            N4 = (page >> 74) & 0x01f
            tauGPS = (page >> 52) & 0x03fffff
            ln = (page >> 51) & 1
            s += ("\n        Time: NA %u tauc %u N4 %u tauGPS %u ln %u" %
                  (NA, tauc, N4, tauGPS, ln))
        elif stringnum in [6, 8, 10, 12, 14]:
            if (((5 == frame and
                 14 == stringnum))):
                B1 = unpack_s11g(page, 112)
                B2 = unpack_s10g(page, 102)
                KP = (page >> 100) & 3
                s += "\n        Extra 1: B1 %d B2 %d KP %u" % (B1, B2, KP)
            else:
                Cn = (page >> 122) & 1
                m = (page >> 120) & 3
                nA = (page >> 115) & 0x1f
                tauA = (page >> 105) & 0x03ff
                lambdaA = (page >> 84) & 0x01ffffff
                deltaiA = (page >> 66) & 0x03ffff
                epsilonA = (page >> 51) & 0x07fff
                s += ("\n        Almanac: Cn %u m %u nA %u tauA %u "
                      "lambdaA %u deltaiA %u"
                      "\n          epsilonA %u" %
                      (Cn, m, nA, tauA, lambdaA, deltaiA, epsilonA))
        elif stringnum in [7, 9, 11, 13, 15]:
            if 5 == frame and 15 == stringnum:
                ln = (page >> 51) & 1
                s += "\n        Extra 2: ln %u" % ln
            else:
                omegaA = (page >> 107) & 0x0ffff
                tA = (page >> 86) & 0x01fffff
                deltaTA = (page >> 64) & 0x03ffffff
                deltaTpA = (page >> 57) & 0x07f
                HA = (page >> 52) & 0x01f
                ln = (page >> 51) & 1
                s += ("\n        Almanac: omegaA %u tA %u deltaTA %u "
                      "deltaTpA %u HA %u ln %u" %
                      (omegaA, tA, deltaTA, deltaTpA, HA, ln))

        return s

    def _decode_sfrbx_sbas(self, words):
        """Decode UBX-RXM-SFRBX SBAS subframes"""
        # See u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
        # Section 10.6 SBAS
        # The WAAS Spec is RTCA DO-229, and is not cheap!
        # the function coded w/o access to that document.
        # WAAS message described here:
        # https://gssc.esa.int/navipedia/index.php/The_EGNOS_SBAS_Message_Format_Explained

        # preamble is 83, then 154, the 198, then repeats.
        preamble = (words[0] >> 24) & 0x0ff
        msg_type = (words[0] >> 18) & 0x03f
        # untangle u-blox words into just message type and data
        # 213 bits

        page = 0
        for i in range(0, 8):
            page |= words[i] & 0x0ffffffff
            page <<= 32
        # trim parity and pad
        page >>= 30

        s = "\n   SBAS: preamble %u type %u" % (preamble, msg_type)
        # sanity check
        if (((page >> 244) & 0x03f) != msg_type):
            s += ("\n    Math Error! %u != %u"
                  "\n    x%x" %
                  (msg_type, (page >> 212) & 0x3f, page))
            return s

        if 0 == msg_type:
            s += "\n       Don't use"
        elif 1 == msg_type:
            s += "\n       PRN mask assignments"
        elif 2 == msg_type:
            s += "\n       Fast Corrections 2"
        elif 3 == msg_type:
            s += "\n       Fast Corrections 3"
        elif 4 == msg_type:
            s += "\n       Fast Corrections 4"
        elif 5 == msg_type:
            s += "\n       Fast Corrections 5"
        elif 6 == msg_type:
            s += "\n       Integity information"
        elif 7 == msg_type:
            s += "\n       Degradation Parameters"
        elif 9 == msg_type:
            s += "\n       Geo Navigation message (X,Y,Z, time, etc.)"
        elif 10 == msg_type:
            s += "\n       Degradation parameters"
        elif 12 == msg_type:
            s += "\n       SBAS Network time/UTC offset parameters"
        elif 17 == msg_type:
            s += "\n       Geo satellite almanacs"
        elif 18 == msg_type:
            s += "\n       Ionospheric grid points masks"
        elif 24 == msg_type:
            s += "\n       Mixed fast/long term satellite error corrections"
        elif 25 == msg_type:
            s += "\n       Long term satellite error corrections"
        elif 26 == msg_type:
            s += "\n       Ionospheric delay corrections"
        elif 27 == msg_type:
            s += "\n       SBAS Service message"
        elif 28 == msg_type:
            s += "\n       Clock Ephemeris Covariance Matrix message"
        elif 31 == msg_type:
            s += "\n       L5 Satellite Mask"
        elif 32 == msg_type:
            s += "\n       L5 Clock-Ephemeris Corrections/Covariance Matrix "
        elif 34 == msg_type:
            s += "\n       L5 Integrity message"
        elif 35 == msg_type:
            s += "\n       L5 Integrity message"
        elif 36 == msg_type:
            s += "\n       L5 Integrity message"
        elif 37 == msg_type:
            s += "\n       L5 Degradation Parameters and DREI Scale Table"
        elif 39 == msg_type:
            s += "\n       L5 SBAS Sats Ephemeris and Covariance Matrix"
        elif 40 == msg_type:
            s += "\n       L5 SBAS Sats Ephemeris and Covariance Matrix"
        elif 47 == msg_type:
            s += "\n       L5 SBAS broadcasting Satellite Almanac"
        elif 62 == msg_type:
            s += "\n       Instant Test Message"
        elif 63 == msg_type:
            s += "\n       Null Message"

        return s

    cnav_msgids = {
        10: "Ephemeris 1",
        11: "Ephemeris 2",
        12: "Reduced Almanac",
        13: "Clock Differential Correction",
        14: "Ephemeris Differential Correction",
        15: "Text",
        30: "Clock, IONO & Group Delay",
        31: "Clock & Reduced Almanac",
        32: "Clock & EOP",
        33: "Clock & UTC",
        34: "Clock & Differential Correction",
        35: "Clock & GGTO",
        36: "Clock & Text",
        37: "Clock & Midi Almanac",
        }

    # map subframe 4 SV ID to Page number
    # IS-GPS-200K Table 20-V
    sbfr4_svid_page = {
        0: 0,   # dummy/self
        57: 1,   # Reserved (Dupe, also 6, 11, 16 21)
        25: 2,
        26: 3,
        27: 4,
        28: 5,
        # 57: 6,   # Reserved (Dupe)
        29: 7,
        30: 8,
        31: 9,
        32: 10,
        # 57: 11,  # Reserved (Dupe)
        62: 12,  # reserved (Dupe 12 and 24)
        52: 13,  # navigation message correction table (NMCT)
        53: 14,  # reserved
        54: 15,  # reserved
        # 57: 16,  # Reserved (Dupe)
        55: 17,  # Special messages
        56: 18,  # Ionospheric and UTC data
        58: 19,  # reserved
        59: 20,  # reserved
        # 57: 21,  # Reserved (Dupe)
        60: 22,  # reserved
        61: 23,  # reserved
        # 62: 24,  # reserved (Dupe 12 and 24)
        63: 25,  # A-S Flags/ SV health
        }

    # map subframe 5 SV ID to Page number
    # IS-GPS-200K Table 20-V
    sbfr5_svid_page = {
        0: 0,   # dummy/self
        1: 1,
        2: 2,
        3: 3,
        4: 4,
        5: 5,
        6: 6,
        7: 7,
        8: 8,
        9: 9,
        10: 10,
        11: 11,
        12: 12,
        13: 13,
        14: 14,
        15: 15,
        16: 16,
        17: 17,
        18: 18,
        19: 19,
        20: 20,
        21: 21,
        22: 22,
        23: 23,
        24: 24,
        51: 25,  # SV Health, SC 1 to 24
        }

    # URA Index to URA meters
    ura_meters = {
        0: "2.40 m",
        1: "3.40 m",
        2: "4.85 m",
        3: "6.85 m",
        4: "9.65 m",
        5: "13.65 m",
        6: "24.00 m",
        7: "48.00 m",
        8: "96.00 m",
        9: "192.00 m",
        10: "384.00 m",
        11: "768.00 m",
        12: "1536.00 m",
        13: "3072.00 m",
        14: "6144.00 m",
        15: "Unk",
        }

    codes_on_l2 = {
        0: "Invalid",
        1: "P-code ON",
        2: "C/A-code ON",
        3: "Invalid",
        }

    nmct_ai = {
        0: "OK",
        1: "Encrypted",
        2: "Unavailable",
        3: "Reserved",
        }

    sv_conf = {
        0: "Reserved",
        1: "Block II/Block IIA/IIR SV",
        2: "M-code, L2C, Block IIR-M SV",
        3: "M-code, L2C, L5, Block IIF SV",
        4: "M-code, L2C, L5, No SA, Block III SV",
        5: "Reserved",
        6: "Reserved",
        7: "Reserved",
        }

    def rxm_sfrbx(self, buf):
        """UBX-RXM-SFRBX decode, Broadcast Navigation Data Subframe

in u-blox 8, protver 17 and up, time sync firmware only
in u-blox F9P and HPG only
in u-blox F10N, protVer 27 and up
not present  before u-blox8

The way u-blox packs the subfram data is perverse, and
barely undocumnted.  Even more perverse than native subframes.

from protVer 27.31 and up, buf[2] is sigId, no longer reserved.
"""

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = (' gnssId %u svId %3u sigId %u freqId %u numWords %u\n'
             '  chn %u version %u reserved2 %u' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += '\n  (%s)' % self.gnss_sig_id.get((u[0] << 8) | u[2], '?')

        sigId = u[2]
        if buf[6] not in set([1, 2]):
            s += "\n    WARNING: unknown version %u" % buf[6]
            return s

        elen = 8 + (4 * buf[4])
        if len(buf) != elen:
            s += ("\n    WARNING: expected %u bytes, got %u" %
                  elan, len(buf))
            return s

        words = ()
        for i in range(0, u[4]):
            u1 = struct.unpack_from('<L', buf, 8 + (i * 4))
            words += (u1[0],)

        if gps.VERB_DECODE <= self.verbosity:
            s += '\n    dwrd'
            i = 0
            for word in words:
                s += " %08x" % word
                if 6 == (i % 7):
                    s += "\n        "
                i += 1

        if ((0 == u[0] or
             5 == u[0])):
            # GPS and QZSS
            preamble = words[0] >> 24
            if 0x8b == preamble:
                # CNAV
                msgid = (words[0] >> 12) & 0x3f
                s += ("\n  CNAV: preamble %#x PRN %u msgid %d (%s)\n" %
                      (preamble, (words[0] >> 18) & 0x3f,
                       msgid, index_s(msgid, self.cnav_msgids)))

            else:
                # IS-GPS-200, Figure 20-2
                # LNAV-L, from sat is 10 words of 30 bits
                # from u-blox each of 10 words right aligned into 32 bits
                #             plus something in top 2 bits?
                preamble = words[0] >> 22
                subframe = (words[1] >> 8) & 0x07
                s += ("\n  LNAV-L: preamble %#x TLM %#x ISF %u" %
                      (preamble, (words[0] >> 8) & 0xffff,
                       1 if (words[0] & 0x40) else 0))

                s += ("\n  TOW17 %u Alert %u A-S %u Subframe %u" %
                      (unpack_u17(words[1], 13) * 6,
                       1 if (words[0] & 0x1000) else 0,
                       1 if (words[0] & 0x800) else 0,
                       subframe))

                if 1 == subframe:
                    # not well validated decode, possibly wrong...
                    # [1] Figure 20-1 Sheet 1, Table 20-I
                    # WN = GPS week number
                    # TGD = Group Delay Differential
                    # tOC = Time of Clock
                    # af0 = SV Clock Bias Correction Coefficient
                    # af1 = SV Clock Drift Correction Coefficient
                    # af2 = Drift Rate Correction Coefficient
                    ura = (words[2] >> 14) & 0x0f
                    c_on_l2 = (words[2] >> 18) & 0x03
                    iodc = ((((words[2] >> 6) & 0x03) << 8) |
                            (words[7] >> 24) & 0xff)
                    s += ("\n   WN %u Codes on L2 %u (%s) URA %u (%s) "
                          "SVH %#04x IODC %u" %
                          (words[2] >> 20,
                           c_on_l2, index_s(c_on_l2, self.codes_on_l2),
                           ura, index_s(ura, self.ura_meters),
                           (words[2] >> 8) & 0x3f, iodc))
                    # tOC = Clock Data Reference Time of Week
                    s += ("\n   L2 P DF %u TGD %e tOC %u\n"
                          "   af2 %e af1 %e af0 %e" %
                          ((words[2] >> 29) & 0x03,
                           unpack_s8(words[6], 6) * (2 ** -31),
                           unpack_u16(words[7], 6) * 16,
                           unpack_s8(words[8], 22) * (2 ** -55),
                           unpack_s16(words[8], 6) * (2 ** -43),
                           unpack_s22(words[9], 8) * (2 ** -31)))

                elif 2 == subframe:
                    # not well validated decode, possibly wrong...
                    # [1] Figure 20-1 Sheet 1, Tables 20-II and 20-III
                    # IODE = Issue of Data (Ephemeris)
                    # Crs = Amplitude of the Sine Harmonic Correction
                    #       Term to the Orbit Radius
                    # Deltan = Mean Motion Difference From Computed Value
                    # M0 = Mean Anomaly at Reference Time
                    # Cuc = Amplitude of the Cosine Harmonic Correction
                    #       Term to the Argument of Latitude
                    # e = Eccentricity
                    # Cus = Amplitude of the Sine Harmonic Correction Term
                    #       to the Argument of Latitude
                    # sqrtA = Square Root of the Semi-Major Axis
                    # tOE = Reference Time Ephemeris
                    s += ("\n   IODE %u Crs %e Deltan %e M0 %e"
                          "\n   Cuc %e e %e Cus %e sqrtA %f"
                          "\n   tOE %u" %
                          (unpack_u8(words[2], 22),
                           unpack_s16(words[2], 6) * (2 ** -5),
                           unpack_s16(words[3], 14) * (2 ** -43),
                           # M0
                           unpack_s32s(words[4], words[3]) * (2 ** -31),
                           unpack_s16(words[5], 14) * (2 ** -29),
                           unpack_u32s(words[6], words[5]) * (2 ** -33),
                           unpack_s16(words[7], 14) * (2 ** -29),
                           unpack_u32s(words[8], words[7]) * (2 ** -19),
                           unpack_u16(words[9], 14) * 16))

                elif 3 == subframe:
                    # not well validated decode, possibly wrong...
                    # [1] Figure 20-1 Sheet 3, Table 20-II, Table 20-III
                    # Cic = Amplitude of the Cosine Harmonic Correction
                    #       Term to the Angle of Inclination
                    # Omega0 = Longitude of Ascending Node of Orbit
                    #          Plane at Weekly Epoch
                    # Cis = Amplitude of the Sine Harmonic Correction
                    #       Term to the Orbit Radius
                    # i0 = Inclination Angle at Reference Time
                    # Crc = Amplitude of the Cosine Harmonic Correction
                    #       Term to the Orbit Radius
                    # omega = Argument of Perigee
                    # Omegadot = Rate of Right Ascension
                    # IODE = Issue of Data (Ephemeris)
                    # IODT = Rate of Inclination Angle
                    s += ("\n   Cic %e Omega0 %e Cis %e i0 %e"
                          "\n   Crc %e omega %e Omegadot %e"
                          "\n   IDOE %u IDOT %e" %
                          (unpack_s16(words[2], 14) * (2 ** -29),
                           unpack_s32s(words[3], words[2]) * (2 ** -31),
                           unpack_s16(words[4], 14) * (2 ** -29),
                           unpack_s32s(words[5], words[4]) * (2 ** -31),
                           # Crc
                           unpack_s16(words[6], 14) * (2 ** -5),
                           unpack_s32s(words[7], words[6]) * (2 ** -31),
                           # Omegadot
                           unpack_s24(words[8], 6) * (2 ** -43),
                           unpack_u8(words[9], 22),
                           unpack_s14(words[9], 8) * (2 ** -43)))

                elif 4 == subframe:
                    # pages:
                    #  2 to 5, 7 to 10 almanac data for SV 25 through 32
                    #  13 navigation message correction table (NMCT_
                    #  17 Special Messages
                    #  18 Ionospheric and UTC data
                    #  25 A-S flags/ SV health
                    #  1, 6, 11, 16 and 21 reserved
                    #  12, 19, 20, 22, 23 and 24 reserved
                    #  14 and 15 reserved
                    # as of 2018, data ID is always 1.
                    svid = (words[2] >> 22) & 0x3f
                    # 0 === svid is dummy SV
                    # almanac for dummy sat 0, same as transmitting sat
                    # Sec 3.2.1: "Users shall only use non-dummy satellites"
                    page = index_s(svid, self.sbfr4_svid_page)

                    s += ("\n   dataid %u svid %u (page %s)\n" %
                          (words[2] >> 28, svid, page))
                    if 'Unk' == page:
                        s += "\n   Unknown page ????"
                        return s

                    if 6 == page:
                        s += "    reserved"
                    elif 2 <= page <= 10:
                        s += self.almanac(words)
                    elif 13 == page:
                        # 20.3.3.5.1.9 NMCT.
                        # 30 ERDs, but more sats. A sat skips own ERD.
                        # no ERD for sat 32
                        # erds are signed! 0x20 == NA
                        s += ("    NMCT AI %u(%s)"
                              "\n      ERD1:  %s %s %s %s %s %s %s %s"
                              "\n      ERD9:  %s %s %s %s %s %s %s %s"
                              "\n      ERD17: %s %s %s %s %s %s %s %s"
                              "\n      ERD25: %s %s %s %s %s %s" %
                              ((words[2] >> 22) & 0x3,    # AI
                               index_s((words[2] >> 22) & 0x3, self.nmct_ai),
                               erd_s((words[2] >> 16) & 0x3f),   # erd1
                               erd_s((words[2] >> 8) & 0x3f),
                               erd_s((((words[2] >> 2) & 0x30) |
                                      (words[3] >> 26) & 0x0f)),
                               erd_s((words[3] >> 20) & 0x3f),
                               erd_s((words[3] >> 14) & 0x3f),   # erd5
                               erd_s((words[3] >> 8) & 0x3f),
                               erd_s((((words[3] >> 2) & 0x30) |
                                      (words[4] >> 26) & 0x0f)),
                               erd_s((words[4] >> 20) & 0x3f),
                               erd_s((words[4] >> 14) & 0x3f),   # erd9
                               erd_s((words[4] >> 8) & 0x3f),
                               erd_s((((words[4] >> 2) & 0x30) |
                                      (words[5] >> 26) & 0x0f)),
                               erd_s((words[5] >> 20) & 0x3f),
                               erd_s((words[5] >> 14) & 0x3f),   # erd 13
                               erd_s((words[5] >> 8) & 0x3f),
                               erd_s((((words[5] >> 2) & 0x30) |
                                      (words[6] >> 26) & 0x0f)),
                               erd_s((words[6] >> 20) & 0x3f),
                               erd_s((words[6] >> 14) & 0x3f),   # erd17
                               erd_s((words[6] >> 8) & 0x3f),
                               erd_s((((words[6] >> 2) & 0x30) |
                                      (words[7] >> 26) & 0x0f)),
                               erd_s((words[7] >> 20) & 0x3f),
                               erd_s((words[7] >> 14) & 0x3f),   # erd21
                               erd_s((words[7] >> 8) & 0x3f),
                               erd_s((((words[7] >> 2) & 0x30) |
                                      (words[8] >> 26) & 0x0f)),
                               erd_s((words[8] >> 20) & 0x3f),
                               erd_s((words[8] >> 14) & 0x3f),   # erd25
                               erd_s((words[8] >> 8) & 0x3f),
                               erd_s((((words[8] >> 2) & 0x30) |
                                      (words[9] >> 26) & 0x0f)),
                               erd_s((words[9] >> 20) & 0x3f),
                               erd_s((words[9] >> 14) & 0x3f),   # erd29
                               erd_s((words[9] >> 8) & 0x3f),    # erd30
                               ))
                    elif 17 == page:
                        s += ("    Special messages: " +
                              chr((words[2] >> 14) & 0xff) +
                              chr((words[2] >> 6) & 0xff) +
                              chr((words[3] >> 22) & 0xff) +
                              chr((words[3] >> 14) & 0xff) +
                              chr((words[3] >> 6) & 0xff) +
                              chr((words[4] >> 22) & 0xff) +
                              chr((words[4] >> 14) & 0xff) +
                              chr((words[4] >> 6) & 0xff) +
                              chr((words[5] >> 22) & 0xff) +
                              chr((words[5] >> 14) & 0xff) +
                              chr((words[5] >> 6) & 0xff) +
                              chr((words[6] >> 22) & 0xff) +
                              chr((words[6] >> 14) & 0xff) +
                              chr((words[6] >> 6) & 0xff) +
                              chr((words[7] >> 22) & 0xff) +
                              chr((words[7] >> 14) & 0xff) +
                              chr((words[7] >> 6) & 0xff) +
                              chr((words[8] >> 22) & 0xff) +
                              chr((words[8] >> 14) & 0xff) +
                              chr((words[8] >> 6) & 0xff) +
                              chr((words[9] >> 22) & 0xff) +
                              chr((words[9] >> 14) & 0xff))

                    elif 18 == page:
                        alpha1 = (words[2] >> 14) & 0xff
                        alpha0 = (words[2] >> 6) & 0xff
                        alpha2 = (words[3] >> 22) & 0xff
                        alpha3 = (words[3] >> 14) & 0xff
                        beta0 = (words[3] >> 6) & 0xff
                        beta1 = (words[4] >> 6) & 0xff
                        beta2 = (words[4] >> 22) & 0xff
                        beta3 = (words[4] >> 14) & 0xff
                        A1 = (words[5] >> 6) & 0xffffff
                        A0 = (((words[6] << 2) & 0xffffff00) |
                              ((words[7] >> 22) & 0xff))
                        tot = (words[7] >> 14) & 0xff
                        WNt = (words[7] >> 6) & 0xff
                        deltatls = (words[8] >> 22) & 0xff
                        WNlsf = (words[8] >> 14) & 0xff
                        DN = (words[8] >> 6) & 0xff
                        deltatlsf = (words[9] >> 22) & 0xff
                        s += ("    Ionospheric and UTC data\n"
                              "     alpah0 x%02x alpah1 x%02x "
                              "alpah2 x%02x alpah3 x%02x\n"
                              "     beta0  x%02x beta1  x%02x "
                              "beta2  x%02x beta3  x%02x\n"
                              "     A0  x%08x A1  x%06x tot x%02x WNt x%02x\n"
                              "     deltatls x%02x WNlsf x%02x DN x%02x "
                              "deltatlsf x%02x" %
                              (alpha0, alpha1, alpha2, alpha3,
                               beta0, beta1, beta2, beta3,
                               A0, A1, tot, WNt,
                               deltatls, WNlsf, DN, deltatlsf))
                    elif 25 == page:
                        aspoof = []
                        aspoof.append((words[2] >> 18) & 0x0f)
                        aspoof.append((words[2] >> 14) & 0x0f)
                        aspoof.append((words[2] >> 10) & 0x0f)
                        aspoof.append((words[2] >> 6) & 0x0f)
                        for i in range(3, 7):
                            aspoof.append((words[i] >> 26) & 0x0f)
                            aspoof.append((words[i] >> 22) & 0x0f)
                            aspoof.append((words[i] >> 18) & 0x0f)
                            aspoof.append((words[i] >> 14) & 0x0f)
                            aspoof.append((words[i] >> 10) & 0x0f)
                            aspoof.append((words[i] >> 6) & 0x0f)
                        aspoof.append((words[7] >> 26) & 0x0f)
                        aspoof.append((words[7] >> 22) & 0x0f)
                        aspoof.append((words[7] >> 18) & 0x0f)
                        aspoof.append((words[7] >> 14) & 0x0f)

                        sv = []
                        sv.append((words[7] >> 6) & 0x3f)
                        sv.append((words[8] >> 24) & 0x3f)
                        sv.append((words[8] >> 18) & 0x3f)
                        sv.append((words[8] >> 12) & 0x3f)
                        sv.append((words[8] >> 6) & 0x3f)
                        sv.append((words[9] >> 24) & 0x3f)
                        sv.append((words[9] >> 18) & 0x3f)
                        sv.append((words[9] >> 12) & 0x3f)
                        s += ("    A/S flags:\n"
                              "     as01 x%x as02 x%x as03 x%x as04 x%x "
                              "as05 x%x as06 x%x as07 x%x as08 x%x\n"
                              "     as09 x%x as10 x%x as11 x%x as12 x%x "
                              "as13 x%x as14 x%x as15 x%x as16 x%x\n"
                              "     as17 x%x as18 x%x as19 x%x as20 x%x "
                              "as21 x%x as22 x%x as23 x%x as24 x%x\n"
                              "     as25 x%x as26 x%x as27 x%x as28 x%x "
                              "as29 x%x as30 x%x as31 x%x as32 x%x\n" %
                              tuple(aspoof))
                        if gps.VERB_DECODE <= self.verbosity:
                            for i in range(1, 33):
                                f = aspoof[i - 1]
                                s += ("      as%02d x%x (A-S %s, Conf %s)\n" %
                                      (i, f,
                                       'On' if (f & 8) else 'Off',
                                       index_s(f & 7, self.sv_conf)))

                        s += ("    SV HEALTH:\n"
                              "      sv25 x%2x sv26 x%2x sv27 x%2x sv28 x%2x "
                              "sv29 x%2x sv30 x%2x sv31 x%2x sv32 x%2x" %
                              tuple(sv))
                    else:
                        s += "    Reserved"

                elif 5 == subframe:
                    svid = (words[2] >> 22) & 0x3f
                    # 0 === svid is dummy SV
                    # almanac for dummy sat 0, same as transmitting sat
                    # Sec 3.2.1: "Users shall only use non-dummy satellites"
                    page = index_s(svid, self.sbfr5_svid_page)

                    s += ("\n   dataid %u svid %u (page %s)\n" %
                          (words[2] >> 28, svid, page))
                    if 'Unk' == page:
                        s += "\n   Unknown page ????"
                        return s

                    if 1 <= page <= 24:
                        s += self.almanac(words)
                    elif 25 == page:
                        toa = (words[2] >> 14) & 0xff
                        WNa = (words[2] >> 6) & 0xff
                        sv = []
                        for i in range(3, 9):
                            sv.append((words[i] >> 24) & 0x3f)
                            sv.append((words[i] >> 18) & 0x3f)
                            sv.append((words[i] >> 12) & 0x3f)
                            sv.append((words[i] >> 6) & 0x3f)
                        s += "    SV HEALTH toa %u WNa %u\n" % (toa, WNa)
                        s += ("     sv01 x%2x sv02 x%2x sv03 x%2x sv04 x%2x "
                              "sv05 x%2x sv06 x%2x sv07 x%2x sv08 x%2x\n"
                              "     sv09 x%2x sv10 x%2x sv11 x%2x sv12 x%2x "
                              "sv13 x%2x sv14 x%2x sv15 x%2x sv16 x%2x\n"
                              "     sv17 x%2x sv18 x%2x sv19 x%2x sv20 x%2x "
                              "sv21 x%2x sv22 x%2x sv23 x%2x sv24 x%2x" %
                              tuple(sv))
                    else:
                        s += "    Reserved"

        elif 1 == u[0]:
            # SBAS
            s += self._decode_sfrbx_sbas(words)

        elif 2 == u[0]:
            # Galileo
            s += self._decode_sfrbx_gal(words)

        elif 3 == u[0]:
            # BeiDou
            s += self._decode_sfrbx_bds(words, sigId)

        elif 6 == u[0]:
            # GLONASS
            s += self._decode_sfrbx_glo(words)

        return s

    def rxm_svsi(self, buf):
        """UBX-RXM-SVSI decode, SV Status Info

Use UBX-NAV-ORB instead
Removed in protVer 32 (u-blox 9 and 10)
"""
        m_len = len(buf)

        u = struct.unpack_from('<LhBB', buf, 0)
        s = ' iTOW %d week %d numVis %d numSV %d' % u

        m_len -= 8
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<BBhbB', buf, 8 + i * 6)
            s += '\n  svid %3d svFlag %#x azim %3d elev % 3d age %3d' % u
            m_len -= 6
            i += 1

        return s

    # Broadcom calls this BRM-ASC-
    rxm_ids = {0x10: {'str': 'RAW', 'dec': rxm_raw, 'minlen': 8,
                      'name': 'UBX-RXM-RAW'},      # obsolete
               0x11: {'str': 'SFRB', 'dec': rxm_sfrb, 'minlen': 42,
                      'name': 'UBX-RXM-SFRB'},
               0x13: {'str': 'SFRBX', 'dec': rxm_sfrbx, 'minlen': 8,
                      'name': 'UBX-RXM-SFRBX'},
               0x14: {'str': 'MEASX', 'dec': rxm_measx, 'minlen': 44,
                      'name': 'UBX-RXM-MEASX'},
               0x15: {'str': 'RAWX', 'dec': rxm_rawx, 'minlen': 16,
                      'name': 'UBX-RXM-RAWX'},
               0x20: {'str': 'SVSI', 'dec': rxm_svsi, 'minlen': 8,
                      'name': 'UBX-RXM-SVSI'},
               # deprecated in u-blox 6, 7, raw option only
               0x30: {'str': 'ALM', 'minlen': 1, 'name': 'UBX-RXM-ALM'},
               # deprecated in u-blox 6, 7, raw option only
               0x31: {'str': 'EPH', 'minlen': 1, 'name': 'UBX-RXM-EPH'},
               0x32: {'str': 'RTCM', 'dec': rxm_rtcm, 'minlen': 8,
                      'name': 'UBX-RXM-RTCM'},
               # protVer 27.5. F9P
               0x33: {'str': 'SPARTN', 'dec': rxm_spartn,  'minlen': 8,
                      'name': 'UBX-RXM-SPARTN'},
               0x34: {'str': 'COR', 'minlen': 12, 'name': 'UBX-RXM-COR'},
               # protVer 27.5. F9P
               0x36: {'str': 'SPARTNKEY', 'dec': rxm_spartnkey,  'minlen': 4,
                      'name': 'UBX-RXM-SPARTNKEY'},
               # Broadcom calls this BRM-ASC-SCLEEP
               0x41: {'str': 'PMREQ', 'dec': rxm_pmreq, 'minlen': 4,
                      'name': 'UBX-RXM-PMREQ'},
               0x59: {'str': 'RLM', 'dec': rxm_rlm, 'minlen': 16,
                      'name': 'UBX-RXM-RLM'},
               0x61: {'str': 'IMES', 'dec': rxm_imes, 'minlen': 4,
                      'name': 'UBX-RXM-IMES'},
               # NEO-D9S, 24 to 528 bytes
               0x72: {'str': 'PMP', 'minlen': 24, 'name': 'UBX-RXM-PMP'},
               0x74: {'str': 'TM', 'minlen': 8, 'name': 'UBX-RXM-TM'},
               }

    # UBX-SEC-

    sec_osnma_nmaHeader = (
        (0, 1, "NMA not auth,"),
        (1, 1, "NMA authed,"),
        (0, 6, "OSNMA not auth,"),
        (2, 6, "OSNMA in test,"),
        (4, 6, "OSNMA OK,"),
        (6, 6, "OSNMA invalid,"),
        (0, 0x18, "TESLA ID 0,"),
        (8, 0x18, "TESLA ID 1,"),
        (0x10, 0x18, "TESLA ID 2,"),
        (0x18, 0x18, "TESLA ID 3,"),
        (0, 0xe0, "TESLA NA,"),
        (0x20, 0xe0, "TESLA Nominal"),
        (0x40, 0xe0, "TESLA EOC"),
        (0x60, 0xe0, "TESLA CREV"),
        (0x80, 0xe0, "TESLA NPK"),
        (0xa0, 0xe0, "TESLA PKEV"),
        (0xc0, 0xe0, "TESLA NMT"),
        (0xe0, 0xe0, "TESLA Alert"),
        )

    sec_osnma_osnma = (
        (0, 1, "disabled,"),
        (1, 1, "enabled,"),
        (0x00, 0x60, "Header same,"),
        (0x40, 0xc0, "pending auth,"),
        (0x80, 0xc0, "header problem,"),
        (0xc0, 0xc0, "Unk,"),
        (0x100, 0x100, "noData,"),
        (0x200, 0x200, "wrongData,"),
        (0x400, 0x400, "wrongMaclt,"),
        )

    sec_osnma_timeSync = (
        (0, 1, "disabled,"),
        (1, 1, "enabled,"),
        (0x00, 0x6e, "timeSync none,"),
        (0x20, 0x6e, "timeSync no trusted time,"),
        (0x40, 0x6e, "timeSync trusted time inaccurate,"),
        (0x60, 0x6e, "timeSync trusted time inaccurate,"),
        (0x80, 0x6e, "timeSync passed,"),
        (0xa0, 0x6e, "timeSync replay attack,"),
        (0xc0, 0x6e, "timeSync Unk,"),
        (0xe0, 0x6e, "timeSync Unk,"),
        )

    sec_osnma_dsm = (
        (0x00, "no DSM auth,"),
        (0x01, "DSM-KROOT authed,"),
        (0x02, "DMS-PKR authed,"),
        (0x03, "OSNMA alert,"),
        (0x04, "DSM-KROOT failed,"),
        (0x05, "DSM-KROOT failed,"),
        (0x06, "DSM unkown key,"),
        (0x07, "Pub Key decomp failed,"),
        (0x08, "Config unsupported,"),
        (0x09, "Missing Merkle root,"),
        )

    def sec_osnma(self, buf):
        """UBX-SEC_OSNMA decode, Galileo Open Service Nav Msg Auth

Partial decode."""

        u = struct.unpack_from('<BBHBBHlLL', buf, 0)
        if 3 != u[0]:
            return "  Unknown version %u" % u[0]

        s = (" version %u  nmaHeader x%x osnmaMonitoring x%x "
             "timSyncReq x%x\n"
             " reserved0 x%x %x timeSyncReqDiff %u reserved1 x%x "
             "dsmAuthentication x%x" % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ("\n    nmaHeader (%s)"
                  "\n    nmaHeader (%s)"
                  "\n    numSVs %d"
                  "\n    timeSyncReq (%s)"
                  "\n    dsm (%s)" %
                  (flagm_s(u[1], self.sec_osnma_nmaHeader),
                   flagm_s(u[2], self.sec_osnma_osnma),
                   (u[2] >> 1) & 0x1f,
                   flagm_s(u[3], self.sec_osnma_timeSync),
                   index_s(u[5], self.sec_osnma_dsm)))
        return s

    # UBX-SEC-SESSID in protVer 34 and up

    yes_no = {
        0: "No",
        1: "Yes",
        }

    sec_sig_jamState = {
        0: "Unk",
        1: "None",
        2: "Warning",
        }

    sec_sig_spfState1 = {
        0: "Unk",
        1: "None",
        2: "Warning",
        3: "Critical",
        }

    sec_sig_spfState2 = {
        0: "Unk",
        1: "None",
        2: "Indicated",
        3: "Affirmed",
        }

    def sec_sig(self, buf):
        """UBX-SEC_SIG decode, Signal Security Info"""

        m_len = len(buf)

        # protVer 19, ZED-F9T, ver 1
        # protVer 34, F10-TIM, ver 2
        u = struct.unpack_from('<B', buf, 0)
        if 1 == u[0]:
            if 12 != m_len:
                s = "  bad length %u != 12" % m_len
            else:
                u = struct.unpack_from('<BBHBBHB', buf, 0)
                s = (" version %u reserved0 x%x %x jamFlags x%x reserved1 "
                     "x%x %x\n"
                     " spfFlags x%x \n" % u)
                if gps.VERB_DECODE <= self.verbosity:
                    s += (' jamDetEnabled (%s) jamState (%s)'
                          ' spfDetEnabled (%s) spfState (%s)\n' %
                          (index_s(u[3] & 1, self.yes_no),
                           index_s((u[1] >> 1) & 3, self.sec_sig_jamState),
                           index_s(u[6] & 1, self.yes_no),
                           index_s((u[1] >> 5) & 3, self.sec_sig_spfState1)))
        elif 2 == u[0]:
            u = struct.unpack_from('<BBBB', buf, 0)
            s = (" version %u sigSecFlags x%x reserved0 x%x "
                 "jamNumCentFraqs %u\n" % u)
            if gps.VERB_DECODE <= self.verbosity:
                s += (' jamDetEnabled (%s) jamState (%s)'
                      ' spfDetEnabled (%s) spfState (%s)\n' %
                      (index_s(u[1] & 1, self.yes_no),
                       index_s((u[1] >> 1) & 3, self.sec_sig_jamState),
                       index_s((u[1] >> 4) & 1, self.yes_no),
                       index_s((u[1] >> 5) & 3, self.sec_sig_spfState2)))

            a_len = 4 + (4 * u[3])
            if a_len != m_len:
                s += "  Invalid length %u a/b %u" % (m_len, a_len)
                return s

            for i in range(0, u[3]):
                u1 = struct.unpack_from('<L', buf, 4 + (i * 4))
                s += "   jamStateCentFreq x%08x\n" % u1
                if gps.VERB_DECODE <= self.verbosity:
                    s += ('    centFreq (%u) jammed (%s)\n' %
                          (u1[0] & 0x7fffff,
                           index_s((u1[0] >> 24) & 1, self.yes_no)))

        else:
            s = "  Unknown version %u" % u[0]

        return s

    sec_siglog_detectionType = {
        0: "simulated signal",
        1: "abnormal signal",
        2: "INS/GNSS mismatch",
        3: "abrupt changes in GNSS signal",
        4: "jamming indicated",
        5: "authentication failed",
        6: "replayed signals",
        }

    sec_siglog_eventType = {
        0: "indication started",
        1: "indication stopped",
        2: "indication triggered",
        3: "indication timed-out",
    }

    def sec_siglog(self, buf):
        """UBX-SEC_SIGLOG decode, Signal Security Log"""

        m_len = len(buf)

        u = struct.unpack_from('<BB', buf, 0)
        # protVer 19, ZED-F9T, ver 0
        # protVer 34, F10-TIM, ver 1
        s = " version %u numEvents %u\n" % u
        a_len = 8 + (8 * u[1])
        if a_len != m_len:
            s += "  Invalid length %u a/b %u" % (m_len, a_len)
            return s

        for i in range(0, u[1]):
            u1 = struct.unpack_from('<LBB', buf, 8 + (i * 8))
            s += "   timeElapsed %u detectionType %u eventType %u\n" % u1
            if gps.VERB_DECODE <= self.verbosity:
                s += ('    detectionType (%s) eventType (%s)\n' %
                      (index_s(u1[1], self.sec_siglog_detectionType),
                       index_s(u1[2], self.sec_siglog_eventType)))
        return s

    def sec_sign(self, buf):
        """UBX-SEC_SIGN decode, Signature of a previous message"""

        # protVer 18 to 23
        u = struct.unpack_from('<BBHBBH', buf, 0)
        s = (" version %u reserved %u %u classId x%x messageID x%x "
             " checksum %u\n  hash " % u)
        s += gps.polystr(binascii.hexlify(buf[8:39]))
        return s

    def sec_uniqid(self, buf):
        """UBX-SEC_UNIQID decode Unique chip ID

changed in protVer 34
"""

        # protVer 18 is 9 bytes
        # 10 bytes in protVer 34 and up
        m_len = len(buf)
        u = struct.unpack_from('<BBHBBBBB', buf, 0)
        s = ("  version %u reserved %u %u uniqueId %#02x%02x%02x%02x%02x"
             % u)
        if (9 < m_len):
            # version 2
            u = struct.unpack_from('<B', buf, 9)
            s += "%02x" % u

        return s

    sec_ids = {0x01: {'str': 'SIGN', 'minlen': 40, 'dec': sec_sign,
                      'name': 'UBX-SEC-SIGN'},
               0x03: {'str': 'UNIQID', 'minlen': 9, 'dec': sec_uniqid,
                      'name': 'UBX-SEC-UNIQID'},
               0x09: {'str': 'SIG', 'minlen': 4, 'dec': sec_sig,
                      'name': 'UBX-SEC-SIG'},
               0x0a: {'str': 'OSNMA', 'minlen': 28, 'dec': sec_osnma,
                      'name': 'UBX-SEC-OSNMA'},
               0x10: {'str': 'SIG', 'minlen': 8, 'dec': sec_siglog,
                      'name': 'UBX-SEC-SIGLOG'},
               }

    # UBX-TIM-
    def tim_svin(self, buf):
        """UBX-TIM-SVIN decode, Survey-in data"""

        u = struct.unpack_from('<LlllLLBB', buf, 0)
        s = ('  dur %u meanX %d meanY %d meanZ %d meanV %u\n'
             '  obs %u valid %u active %u' % u)
        return s

    def tim_tm2(self, buf):
        """UBX-TIM-TM2 decode, Time mark data"""

        u = struct.unpack_from('<BBHHHLLLLL', buf, 0)
        s = ('  ch %u flags %#x count %u wnR %u wnF %u\n'
             '  towMsR %u towSubMsR %u towMsF %u towSubMsF %u accEst %u\n' % u)
        return s

    tim_tp_flags = (
        (0, 1, "timebase:GNSS"),
        (1, 1, "timebase:UTC"),
        (0, 2, "UTC:NA"),
        (2, 2, "UTC:OK"),
        (0, 0x0c, "RAIM:NA"),
        (4, 0x0c, "RAIM:inactive"),
        (8, 0x0c, "RAIM:active"),
        (0x0c, 0x0c, "RAIM:Unk"),
        # qErrValid  9-series, protVer 32 and up.
        (0, 0x10, "qErr:Valid"),
        (0x10, 0x10, "qErr:Invalid"),
        # TpNotLocked, 9-series, protVer 32 and up.
        (0, 0x20, "TP:Locked"),
        (0x20, 0x20, "TP:Unlocked"),
        )

    tim_tp_refInfo = (
        (0, 0x0f, "GNSS:GPS"),
        (1, 0x0f, "GNSS:GLONASS"),
        (2, 0x0f, "GNSS:BeiDou"),
        (3, 0x0f, "GNSS:Galileo"),
        (4, 0x0f, "GNSS:NavIc"),
        (5, 0x0f, "GNSS:Unk5"),
        (6, 0x0f, "GNSS:Unk6"),
        (7, 0x0f, "GNSS:Unk7"),
        (8, 0x0f, "GNSS:Unk8"),
        (9, 0x0f, "GNSS:Unk9"),
        (10, 0x0f, "GNSS:Unk10"),
        (11, 0x0f, "GNSS:Unk11"),
        (12, 0x0f, "GNSS:Unk12"),
        (13, 0x0f, "GNSS:Unk13"),
        (14, 0x0f, "GNSS:Unk14"),
        (15, 0x0f, "GNSS:Unk"),
        (0x00, 0xf0, "UTC:Unk"),
        (0x10, 0xf0, "UTC:CRL"),
        (0x20, 0xf0, "UTC:NIST"),
        (0x30, 0xf0, "UTC:USNO"),
        (0x40, 0xf0, "UTC:BIPM"),
        (0x50, 0xf0, "UTC:EL"),
        (0x60, 0xf0, "UTC:SU"),
        (0x70, 0xf0, "UTC:NTSC"),
        (0x80, 0xf0, "UTC:NPLI"),
        (0x90, 0xf0, "UTC:Unk9"),
        (0xa0, 0xf0, "UTC:Unk10"),
        (0xb0, 0xf0, "UTC:Unk11"),
        (0xc0, 0xf0, "UTC:Unk12"),
        (0xd0, 0xf0, "UTC:Unk13"),
        (0xe0, 0xf0, "UTC:Unk14"),
        (0xf0, 0xf0, "UTC:Unk"),
        )

    def tim_tp(self, buf):
        """UBX-TIM-TP decode, Time Pulse Timedata

Note: qErr is for the next PPS edge to be received.

qErrInvalid added in protVer 32 and up
"""

        # towSubMS is usually zero, but have seen 128, and 4294967168.
        # towSubMs == 1 is 233 femto seconds!
        # towSubMS == 128 is 29.802 pico seconds!
        # towSubMS == 4294967168 is 0.9999999701976775 milli seconds
        u = struct.unpack_from('<LLlHbb', buf, 0)
        s = ('  towMS %u towSubMS %d qErr %d week %d\n'
             '  flags x%02x refInfo x%02x' % u)

        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n   flags (%s)'
                  '\n   refInfo (%s)'
                  '\n   ftow %.12f' %
                  (flagm_s(u[4], self.tim_tp_flags),
                   flagm_s(u[5], self.tim_tp_refInfo),
                   (u[0] + (u[1] * 2 ** -32)) / 1000.0))
        return s

    tim_vrfy_flags = {
        0: "no time aiding done",
        2: "source was RTC",
        3: "source was AID-IN",
        }

    def tim_vrfy(self, buf):
        """UBX-TIM-VRFY decode, Sourced Time Verification"""

        u = struct.unpack_from('<llllHBB', buf, 0)
        s = ('  itow %d frac %d deltaMs %d deltaNs %d\n'
             '  wno %u flags x%x reserved1 %u' % u)
        if gps.VERB_DECODE <= self.verbosity:
            s += ('\n   flags (%s)' %
                  index_s(u[5] & 3, self.tim_vrfy_flags))
        return s

    tim_ids = {0x01: {'str': 'TP', 'dec': tim_tp, 'minlen': 16,
                      'name': 'UBX-TIM-TP'},
               0x03: {'str': 'TM2', 'dec': tim_tm2, 'minlen': 28,
                      'name': 'UBX-TIM-TM2'},
               0x04: {'str': 'SVIN', 'dec': tim_svin, 'minlen': 28,
                      'name': 'UBX-TIM-SVIN'},
               0x06: {'str': 'VRFY', 'dec': tim_vrfy, 'minlen': 20,
                      'name': 'UBX-TIM-VRFY'},
               # u-blox 8, FTS only
               0x11: {'str': 'DOSC', 'minlen': 8, 'name': 'UBX-TIM-DOSC'},
               # u-blox 8, FTS only
               0x12: {'str': 'TOS', 'minlen': 56, 'name': 'UBX-TIM-TOS'},
               # u-blox 8, FTS only
               0x13: {'str': 'SMEAS', 'minlen': 12, 'name': 'UBX-TIM-SMEAS'},
               # u-blox 8, FTS only
               0x15: {'str': 'VCOCAL', 'minlen': 1, 'name': 'UBX-TIM-VCOCAL'},
               # u-blox 8, FTS only
               0x16: {'str': 'FCHG', 'minlen': 32, 'name': 'UBX-TIM-FCHG'},
               # u-blox 8, FTS only
               0x17: {'str': 'HOC', 'minlen': 8, 'name': 'UBX-TIM-HOC'},
               }

    # UBX-UPD-
    upd_sos_cmd = {
        0: "Create Backup File in Flash",
        1: "Clear Backup in Flash",
        2: "Backup File Creation Acknowledge",
        3: "System Restored from Backup",
        }

    upd_sos_response2 = {
        0: "Not Acknowledged",
        1: "Acknowledged",
        }

    upd_sos_response3 = {
        0: "Unknown",
        1: "Failed restoring from backup file",
        2: "Restored from backup file",
        3: "Not restored (no backup)",
        }

    def upd_sos(self, buf):
        """UBX-UPD-SOS decode, Backup File stuff"""
        m_len = len(buf)

        if 0 == m_len:
            return "  Poll Backup File Restore Status"

        if 4 > m_len:
            return "  Bad Length %s" % m_len

        u = struct.unpack_from('<BBH', buf, 0)
        s = '  command %u reserved1 x%x %x' % u

        s1 = ""
        if 0 == u[0]:
            # Create Backup in Flash
            pass
        elif 1 == u[0]:
            # Clear Backup in Flash
            pass
        elif 8 > m_len:
            s += "  Bad Length %s" % m_len
        elif 2 == u[0]:
            # Backup File Creation Acknowledge
            u1 = struct.unpack_from('<BBH', buf, 4)
            s += '\n  response %u reserved2 x%x %x' % u1
            s1 = ' response (%s)' % index_s(u1[0], self.upd_sos_response2)
        elif 3 == u[0]:
            # System Restored from Backup
            u1 = struct.unpack_from('<BBH', buf, 4)
            s += '\n  response %u reserved2 x%x %x' % u1
            s1 = ' response (%s)' % index_s(u1[0], self.upd_sos_response3)

        if gps.VERB_DECODE <= self.verbosity:
            s += '\n    cmd (%s)%s' % (index_s(u[0], self.upd_sos_cmd), s1)
        return s

    upd_ids = {
               # undocumented firmware update message
               0x0c: {'str': 'undoc1', 'minlen': 13, 'name': "UBX-UPD-undoc1"},
               0x14: {'str': 'SOS', 'dec': upd_sos, 'name': "UBX-UPD-SOS"},
               # undocumented firmware update message
               0x25: {'str': 'undoc2', 'minlen': 20, 'name': "UBX-UPD-undoc2"},
               }

    classes = {
        0x01: {'str': 'NAV', 'ids': nav_ids},
        0x02: {'str': 'RXM', 'ids': rxm_ids},
        0x04: {'str': 'INF', 'ids': inf_ids},
        0x05: {'str': 'ACK', 'ids': ack_ids},
        0x06: {'str': 'CFG', 'ids': cfg_ids},
        0x09: {'str': 'UPD', 'ids': upd_ids},
        0x0A: {'str': 'MON', 'ids': mon_ids},
        0x0B: {'str': 'AID', 'ids': aid_ids},
        0x0D: {'str': 'TIM', 'ids': tim_ids},
        0x10: {'str': 'ESF', 'ids': esf_ids},
        0x13: {'str': 'MGA', 'ids': mga_ids},
        0x21: {'str': 'LOG', 'ids': log_ids},
        0x27: {'str': 'SEC', 'ids': sec_ids},
        0x28: {'str': 'HNR', 'ids': hnr_ids},
        0x29: {'str': 'NAV2', 'ids': nav2_ids},
        # Antaris 4
        # 0x4x USR, SCK Customer Messages
        0xf0: {'str': 'NMEA', 'ids': nmea_ids},
        0xf5: {'str': 'RTCM', 'ids': rtcm_ids},
    }

    def class_id_s(self, m_class, m_id):
        """Return class and ID numbers as a string."""

        if (((m_class in self.classes and
              'str' in self.classes[m_class]))):
            s = '%s-' % (self.classes[m_class]['str'])
        else:
            s = '%d-' % m_class

        if (((m_class in self.classes and
              'ids' in self.classes[m_class] and
              m_id in self.classes[m_class]['ids'] and
              'str' in self.classes[m_class]['ids'][m_id]))):
            s += '%s' % (self.classes[m_class]['ids'][m_id]['str'])
        else:
            s += '%d' % m_id

        s += ' (x%02x:x%02x)' % (m_class, m_id)

        return s

    def name_s(self, m_class, m_id):
        """Return UBX-cls-id as a string."""

        s = 'UBX-'
        if (((m_class in self.classes and
              'str' in self.classes[m_class]))):
            s += '%s-' % (self.classes[m_class]['str'])
        else:
            s = 'x%02x-' % (m_class)

        if (((m_class in self.classes and
              'ids' in self.classes[m_class] and
              m_id in self.classes[m_class]['ids'] and
              'str' in self.classes[m_class]['ids'][m_id]))):
            s += self.classes[m_class]['ids'][m_id]['str']
        else:
            s += 'x%02x' % m_id

        return s

    def decode_msg(self, out):
        """Decode one message and then return number of chars consumed"""

        state = 'BASE'
        consumed = 0

        # decode state machine
        for this_byte in out:
            consumed += 1
            if isinstance(this_byte, str):
                # a character, probably read from a file
                c = ord(this_byte)
            else:
                # a byte, probably read from a serial port
                c = int(this_byte)

            if gps.VERB_RAW <= self.verbosity:
                if ord(' ') <= c <= ord('~'):
                    # c is printable
                    print("state: %s char %c (%#x)" % (state, chr(c), c))
                else:
                    # c is not printable
                    print("state: %s char %#x" % (state, c))

            if 'BASE' == state:
                # start fresh
                # place to store 'comments'
                comment = ''
                m_class = 0
                m_id = 0
                m_len = 0
                m_raw = bytearray(0)        # class, id, len, payload
                m_payload = bytearray(0)    # just the payload
                m_ck_a = 0
                m_ck_b = 0

                if 0xb5 == c:
                    # got header 1, mu
                    state = 'HEADER1'

                if ord('$') == c:
                    # got $, so NMEA?
                    state = 'NMEA'
                    comment = '$'

                if ord("{") == c:
                    # JSON, treat as comment line
                    state = 'JSON'

                    # start fresh
                    comment = "{"
                    continue

                if ord("#") == c:
                    # comment line
                    state = 'COMMENT'

                    # start fresh
                    comment = "#"
                    continue

                if 0xd3 == c:
                    # RTCM3 Leader 1
                    state = 'RTCM3_1'

                    # start fresh
                    comment = "#"
                    continue

                if (ord('\n') == c) or (ord('\r') == c):
                    # CR or LF, leftovers
                    return 1
                continue

            if state in ('COMMENT', 'JSON'):
                # inside comment
                if ord('\n') == c or ord('\r') == c:
                    # Got newline or linefeed
                    # terminate messages on <CR> or <LF>
                    # Done, got a full message
                    if gps.polystr('{"class":"ERROR"') in comment:
                        # always print gpsd errors
                        if 0 < self.timestamp:
                            timestamp(self.timestamp)
                        print(comment)
                    elif gps.VERB_DECODE <= self.verbosity:
                        if 0 < self.timestamp:
                            timestamp(self.timestamp)
                        print(comment)
                    return consumed

                # else:
                comment += chr(c)
                continue

            if 'NMEA' == state:
                # getting NMEA payload
                if (ord('\n') == c) or (ord('\r') == c):
                    # CR or LF, done, got a full message
                    # terminates messages on <CR> or <LF>
                    if gps.VERB_DECODE <= self.verbosity:
                        if 0 < self.timestamp:
                            timestamp(self.timestamp)
                        print(comment)
                    return consumed

                # else:
                comment += chr(c)
                continue

            if 'RTCM3_1' == state:
                # high 6 bits must be zero,
                if 0 != (c & 0xfc):
                    state = 'BASE'
                else:
                    # low 2 bits are MSB of a 10-bit length
                    m_len = c << 8
                    state = 'RTCM3_2'
                    m_raw.extend([c])
                continue

            if 'RTCM3_2' == state:
                # 8 bits are LSB of a 10-bit length
                m_len |= 0xff & c
                # add 3 for checksum
                m_len += 3
                state = 'RTCM3_PAYLOAD'
                m_raw.extend([c])
                continue

            if 'RTCM3_PAYLOAD' == state:
                m_len -= 1
                m_raw.extend([c])
                m_payload.extend([c])
                if 0 == m_len:
                    state = 'BASE'
                    ptype = m_payload[0] << 4
                    ptype |= 0x0f & (m_payload[1] >> 4)
                    if gps.VERB_DECODE <= self.verbosity:
                        print("RTCM3 packet: type %d\n" % ptype)
                continue

            if ord('b') == c and 'HEADER1' == state:
                # got header 2
                state = 'HEADER2'
                continue

            if 'HEADER2' == state:
                # got class
                state = 'CLASS'
                m_class = c
                m_raw.extend([c])
                continue

            if 'CLASS' == state:
                # got ID
                state = 'ID'
                m_id = c
                m_raw.extend([c])
                continue

            if 'ID' == state:
                # got first length
                state = 'LEN1'
                m_len = c
                m_raw.extend([c])
                continue

            if 'LEN1' == state:
                # got second length
                m_raw.extend([c])
                m_len += 256 * c
                if 0 == m_len:
                    # no payload
                    state = 'CSUM1'
                else:
                    state = 'PAYLOAD'
                continue

            if 'PAYLOAD' == state:
                # getting payload
                m_raw.extend([c])
                m_payload.extend([c])
                if len(m_payload) == m_len:
                    state = 'CSUM1'
                continue

            if 'CSUM1' == state:
                # got ck_a
                state = 'CSUM2'
                m_ck_a = c
                continue

            if 'CSUM2' == state:
                # got a complete, maybe valid, message
                if 0 < self.timestamp:
                    timestamp(self.timestamp)

                # ck_b
                state = 'BASE'
                m_ck_b = c
                # check checksum
                chk = self.checksum(m_raw, len(m_raw))
                if (chk[0] != m_ck_a) or (chk[1] != m_ck_b):
                    print("gps/ubx: ERROR checksum failed, "
                          "was (%02x,%02x) s/b (%02x, %02x)\n" %
                          (m_ck_a, m_ck_b, chk[0], chk[1]))

                s_payload = ''.join('{:02x} '.format(x) for x in m_payload)
                x_payload = ','.join(['%02x' % x for x in m_payload])

                if m_class in self.classes:
                    this_class = self.classes[m_class]
                    if 'ids' in this_class:
                        if m_id in this_class['ids']:
                            # got an entry for this message
                            # name is mandatory
                            this_id = this_class['ids'][m_id]
                            s_payload = this_id['name']
                            s_payload += ':\n'

                            if ((('depver' in this_id) and
                                 (this_id['depver'] <= self.protver))):
                                s_payload += ('WARNING:  protVer is %s, '
                                              ' %s deprecated in %.2f and '
                                              'higher\n' %
                                              (self.protver, this_id['name'],
                                               this_id['depver']))

                            if ((('minlen' in this_id) and
                                 (0 == m_len) and
                                 (0 != this_id['minlen']))):
                                s_payload += "  Poll request"
                            elif (('minlen' in this_id) and
                                  (this_id['minlen'] > m_len)):
                                # failed minimum length for this message
                                s_payload += "  Bad Length %s" % m_len
                            elif 'dec' in this_id:
                                # got a decoder for this message
                                s_payload += this_id['dec'](self, m_payload)
                            else:
                                s_payload += ("  len %#x, raw %s" %
                                              (m_len, x_payload))

                        else:
                            # unknon m_id
                            s_payload = ("%s: Unknown message type\n" %
                                         (self.class_id_s(m_class, m_id)))
                            # FIXME: line lenght.
                            s_payload += ("  len %#x, raw %s" %
                                          (m_len, x_payload))

                if not s_payload:
                    # huh?
                    s_payload = ("%s, len %#x, raw %s" %
                                 (self.class_id_s(m_class, m_id),
                                  m_len, x_payload))

                if gps.VERB_INFO <= self.verbosity:
                    print("%s, len: %#x" %
                          (self.class_id_s(m_class, m_id), m_len))
                    x_raw = ','.join(['%02x' % x for x in m_raw[0:4]])
                    print("header: b5,62,%s" % x_raw)
                    print("payload: %s" % x_payload)
                    print("chksum: %02x,%02x" % (m_ck_a, m_ck_b))
                print("%s\n" % s_payload)
                return consumed

            # give up
            state = 'BASE'

        # fell out of loop, no more chars to look at
        return 0

    def checksum(self, msg, m_len):
        """Calculate u-blox message checksum"""
        # the checksum is calculated over the Message, starting and including
        # the CLASS field, up until, but excluding, the Checksum Field:

        ck_a = 0
        ck_b = 0
        for c in msg[0:m_len]:
            ck_a += c
            ck_b += ck_a

        return [ck_a & 0xff, ck_b & 0xff]

    def make_pkt(self, m_class, m_id, m_data):
        """Make a message packet"""
        # always little endian, leader, class, id, length
        m_len = len(m_data)

        # build core message
        msg = bytearray(m_len + 6)
        struct.pack_into('<BBH', msg, 0, m_class, m_id, m_len)

        # copy payload into message buffer
        i = 0
        while i < m_len:
            msg[i + 4] = m_data[i]
            i += 1

        # add checksum
        chk = self.checksum(msg, m_len + 4)
        m_chk = bytearray(2)
        struct.pack_into('<BB', m_chk, 0, chk[0], chk[1])

        header = b"\xb5\x62"
        return header + msg[:m_len + 4] + m_chk

    def gps_send(self, m_class, m_id, m_data):
        """Build, and send, a message to GPS"""
        m_all = self.make_pkt(m_class, m_id, m_data)
        self.gps_send_raw(m_all)

    def gps_send_raw(self, m_all):
        """Send a raw message to GPS"""
        if not self.read_only:
            self.io_handle.ser.write(m_all)
            if gps.VERB_QUIET < self.verbosity:
                sys.stdout.write("sent:\n")
                if gps.VERB_INFO < self.verbosity:
                    sys.stdout.write(gps.polystr(binascii.hexlify(m_all)))
                    sys.stdout.write("\n")
                self.decode_msg(m_all)
                sys.stdout.flush()

    def send_able(self, able, args, command):
        """Generic dis-/en-able messages"""
        # args == 0 for off, or > 0 for rate
        self.send_cfg_msg(command['mid'][0], command['mid'][1], able)

    def send_able_cfg_batch(self, able, args, command):
        """dis/enable batching, UBX-CFG-BATCH"""

        flags = 0x0d if able else 0x0c
        m_data = bytearray(8)
        struct.pack_into('<BBHHBB', m_data, 0, 0, flags, 128, 0, 0, 0)
        self.gps_send(6, 0x93, m_data)

    def send_able_beidou(self, able, args, command):
        """dis/enable BeiDou"""
        # Two frequency GNSS receivers use BeiDou or GLONASS
        # disable, then enable
        self.send_cfg_gnss1(3, able, args)

    def send_able_binary(self, able, args, command):
        """dis/enable basic binary messages. -e/-d BINARY"""

        # FIXME: does not change UBX-CFG-PRT outProtoMask for current port.
        # FIXME: in u-blox 9, use VAL-SET to ensure NMEA mask on.
        # Workarouund: gpsctl -b
        # try to keep in sync with driver_ubx.c, ubx_cfg_prt()

        # UBX-NAV we always toggle
        # we assume no oddball UBX to toggle
        ubx_nav_toggle = (
            0x04,          # msg id = UBX-NAV-DOP

            # UBX-NAV-TIMEGPS
            # Note: UTC may, or may not be UBX-NAV-TIMEGPS.
            #       depending on UBX-CFG-NAV5 utcStandard
            # Note: We use TIMEGPS to get the leapS
            # UBX-NAV-TIMEGPS is a great cycle ender, NAV-EOE better
            0x20,          # msg id = UBX-NAV-TIMEGPS
            )

        # UBX for protver < 15
        ubx_14_nav_on = (
            # UBX-NAV-SOL is ECEF. deprecated in protver 14, gone in protver 27
            0x06,              # msg id = NAV-SOL
            0x30,              # msg id = NAV-SVINFO
        )

        # UBX for protver >= 15
        ubx_15_nav_on = (
            # Need NAV-POSECEF, NAV-VELECEF and NAV-PVT to replace NAV-SOL
            0x01,              # msg id = NAV-POSECEF
            0x07,              # msg id = NAV-PVT
            0x11,              # msg id = NAV-VELECEF
            0x35,              # msg id = NAV-SAT
            0x61,              # msg id = NAV-EOE, first in protver 18
        )

        # UBX for protver >= 27
        ubx_27_nav_on = (
            # Add NAV-SIG for L1/L2/L5 info
            0x43,              # msg id = NAV-SIG
        )

        # some we always turn off, user can enable later
        ubx_nav_off = (
            0x12,              # msg id = NAV-VELNED
            # turn off NAV-SBAS as the gpsd decode does not go to JSON,
            # so the data is wasted. */
            0x32,              # msg id = NAV-SBAS, in u-blox 4 to 8, not 9
            )

        rate = 1 if able else 0

        # msgClass (UBX-NAV), msgID, rate
        m_data = bytearray([0x01, 0x09, rate])
        for mid in ubx_nav_toggle:
            m_data[1] = mid
            # UBX-CFG-MSG
            self.gps_send(6, 1, m_data)

        if 15 > self.protver:
            for mid in ubx_14_nav_on:
                m_data[1] = mid
                # UBX-CFG-MSG
                self.gps_send(6, 1, m_data)

            # turn off >= 15 messages.  Yes this makes NAKs.
            m_data[2] = 0       # rate off
            for idx in ubx_15_nav_on:
                m_data[1] = idx
                # UBX-CFG-MSG
                self.gps_send(6, 1, m_data)
        else:
            # 15 < protVer
            for id in ubx_15_nav_on:
                m_data[1] = id
                # UBX-CFG-MSG
                self.gps_send(6, 1, m_data)

            if 27 <= self.protver:
                for id in ubx_27_nav_on:
                    m_data[1] = id
                    # UBX-CFG-MSG
                    self.gps_send(6, 1, m_data)

            # turn off < 15 messages.  Yes this may make NAKs.
            m_data[2] = 0       # rate off
            for id in ubx_14_nav_on:
                m_data[1] = id
                # UBX-CFG-MSG
                self.gps_send(6, 1, m_data)

        # msgClass (UBX-NAV), msgID (TIMELS), rate (0xff)
        # 0xff is about every 4 minutes if nav rate is 1Hz
        m_data = bytearray([0x01, 0x26, 0xff])
        # UBX-CFG-MSG
        self.gps_send(6, 1, m_data)

        # always off NAV messages
        m_data[2] = 0       # rate off
        for id in ubx_nav_off:
            m_data[1] = id
            # UBX-CFG-MSG
            self.gps_send(6, 1, m_data)

    def send_able_ecef(self, able, args, command):
        """Enable ECEF messages"""
        # set NAV-POSECEF rate
        self.send_cfg_msg(1, 1, able)
        # set NAV-VELECEF rate
        self.send_cfg_msg(1, 0x11, able)

    def send_able_esf(self, able, args, command):
        """dis/enable basic ESF messages"""

        esf_toggle = (
            ubx.ESF_ALG,
            ubx.ESF_INS,
            # ESF-MEAS too much
            # ESF-RAW too much
            ubx.ESF_STATUS,
            )

        rate = int(args[0]) if able else 0

        m_data = bytearray(3)
        for (cls, mid) in esf_toggle:
            m_data[0] = cls
            m_data[1] = mid
            m_data[2] = rate
            # UBX-CFG-MSG
            self.gps_send(6, 1, m_data)

    def send_able_gps(self, able, args, command):
        """dis/enable GPS/QZSS"""
        # GPS and QZSS both on, or both off, together
        # GPS
        self.send_cfg_gnss1(0, able, args)
        # QZSS
        self.send_cfg_gnss1(5, able, args)

    def send_able_galileo(self, able, args, command):
        """dis/enable GALILEO

"If Galileo is enabled, UBX-CFG-GNSS must be followed by UBX-CFG-RST
with resetMode set to Hardware reset."
"""
        self.send_cfg_gnss1(2, able, args)

    def send_able_glonass(self, able, args, command):
        """dis/enable GLONASS"""
        # Two frequency GPS use BeiDou or GLONASS
        # disable, then enable
        self.send_cfg_gnss1(6, able, args)

    def send_able_hnr(self, able, args, command):
        """dis/enable HNR messages"""

        esf_toggle = (
            ubx.HNR_ATT,
            ubx.HNR_INS,
            ubx.HNR_PVT,
            )

        m_data = bytearray(3)
        for (cls, mid) in esf_toggle:
            m_data[0] = cls
            m_data[1] = mid
            m_data[2] = able
            # UBX-CFG-MSG
            self.gps_send(6, 1, m_data)

    def send_able_logfilter(self, able, args, command):
        """Enable logging"""

        if able:
            m_data = bytearray([1,            # version
                                5,            # flags
                                # All zeros below == log all
                                0, 0,         # minInterval
                                0, 0,         # timeThreshold
                                0, 0,         # speedThreshold
                                0, 0, 0, 0    # positionThreshold
                                ])
        else:
            m_data = bytearray([1,      # version
                                0,      # flags
                                0, 0,   # minInterval
                                0, 0,   # timeThreshold
                                0, 0,   # speedThreshold
                                0, 0, 0, 0    # positionThreshold
                                ])

        # set UBX-CFG-LOGFILTER
        self.gps_send(6, 0x47, m_data)

    def send_able_ned(self, able, args, command):
        """Enable NAV-RELPOSNED and VELNED messages.

protver 15+ required for VELNED
protver 20+, and HP GNSS, required for RELPOSNED
"""
        if 15 > self.protver:
            sys.stderr.write('gps/ubx: WARNING: protver %d too low for NED\n' %
                             (self.protver))
            return

        # set NAV-VELNED rate
        self.send_cfg_msg(1, 0x12, able)

        if 20 > self.protver:
            sys.stderr.write('gps/ubx: WARNING: protver %d too low for '
                             'RELPOSNED\n' %
                             (self.protver))
            return

        # set NAV-RELPOSNED rate
        self.send_cfg_msg(1, 0x3C, able)

    def send_able_nmea(self, able, args, command):
        """dis/enable basic NMEA messages"""

        # try to keep in sync with driver_ubx.c, ubx_cfg_prt()
        # FIXME: does not change UBX-CFG-PRT outProtoMask for current port.
        # FIXME: in u-blox 9, use VAL-SET to ensure NMEA mask on.
        # Workarouund: gpsctl -n

        # we assume no oddball NMEA to toggle
        nmea_toggle = (
            0x00,          # msg id  = GGA
            # 0x01,        # msg id  = GLL, only need RMC */
            0x02,          # msg id  = GSA
            0x03,          # msg id  = GSV
            0x04,          # msg id  = RMC
            0x05,          # msg id  = VTG
            0x07,          # msg id  = GST, GNSS pseudorange error statistics
            0x08,          # msg id  = ZDA, for UTC year
            0x09,          # msg id  = GBS, for RAIM errors
            )

        # msgClass (UBX-NMEA), msgID, rate
        m_data = bytearray([0xf0, 0x09, able])
        for mid in nmea_toggle:
            m_data[1] = mid
            # UBX-CFG-MSG
            self.gps_send(6, 1, m_data)

        # xxGLL, never need it
        m_data = bytearray([0xf0, 0x01, 0])
        self.gps_send(6, 1, m_data)

    def send_able_rtcm3(self, able, args, command):
        """dis/enable RTCM3 1005, 1077, 1087, 1230 messages"""

        # protVer 20+, High Precision only
        # No u-blox outputs RTCM2
        # USB ONLY!

        if able:
            rate = able
            # UBX-CFG-PRT, USB
            # can't really do other portIDs as the messages are different
            # and need data we do not have.
            # does not seem to hurt to enable all in and out, even unsupported
            m_data = bytearray(20)
            m_data[0] = 0x03         # default to USB port
            m_data[12] = 0x27        # in: RTCM3, RTCM2, NMEA and UBX
            # Ensures RTCM3 output (all) are set (outProtoMask)
            # no u-blox has RTCM2 out
            m_data[14] = 0x23        # out:  RTCM3, NMEA and UBX
            self.gps_send(0x06, 0x00, m_data)
        else:
            # leave  UBX-CFG-PRT alone
            rate = 0

        # 1005, Stationary RTK reference station ARP
        m_data = bytearray([0xf5, 0x05, rate])
        self.gps_send(6, 1, m_data)
        # 1077, GPS MSM7
        m_data = bytearray([0xf5, 0x4d, rate])
        self.gps_send(6, 1, m_data)
        # 1087, GLONASS MSM7
        m_data = bytearray([0xf5, 0x57, rate])
        self.gps_send(6, 1, m_data)
        # 1230, GLONASS code-phase biases
        m_data = bytearray([0xf5, 0xe6, rate])
        self.gps_send(6, 1, m_data)
        # we skip Galileo or BeiDou for now, unsupported by u-blox rover

        # ZED-F9P rover requires MSM7 and 4072,0 or 4072,1
        # 4072,0
        m_data = bytearray([0xf5, 0xfe, rate])
        # 4072,1
        m_data = bytearray([0xf5, 0xfd, rate])
        self.gps_send(6, 1, m_data)

    def send_able_rawx(self, able, args, command):
        """dis/enable UBX-RXM-RAW/RAWXX"""

        if 15 > self.protver:
            # u-blox 7 or earlier, use RAW
            sid = 0x10
        else:
            # u-blox 8 or later, use RAWX
            sid = 0x15
        m_data = bytearray([0x2, sid, able])
        self.gps_send(6, 1, m_data)

    def send_able_pps(self, able, args, command):
        """dis/enable PPS, using UBX-CFG-TP5"""

        # This is actually a shortcut for a regular CFG-TP5 message
        tp5_args = [''] * 9
        tp5_args[8] = '0x77' if able else '0x76'

        self.send_cfg_tp5(tp5_args)

    def send_able_sbas(self, able, args, command):
        """dis/enable SBAS"""
        self.send_cfg_gnss1(1, able, args)

    def send_able_sfrbx(self, able, args, command):
        """dis/enable UBX-RXM-SFRB/SFRBX"""

        rate = 1 if able else 0
        if 15 > self.protver:
            # u-blox 7 or earlier, use SFRB
            sid = 0x11
        else:
            # u-blox 8 or later, use SFRBX
            sid = 0x13
        m_data = bytearray([0x2, sid, rate])
        self.gps_send(6, 1, m_data)

    def send_able_time(self, able, args, command):
        """dis/enable NAV-TIME* messages"""

        esf_toggle = (
            ubx.NAV_TIMEBDS,
            ubx.NAV_TIMEGAL,
            ubx.NAV_TIMEGLO,
            ubx.NAV_TIMEGPS,
            ubx.NAV_TIMELS,
            ubx.NAV_TIMENAVIC,
            ubx.NAV_TIMEQZSS,
            ubx.NAV_TIMEUTC,
            )

        m_data = bytearray(3)
        for (cls, mid) in esf_toggle:
            m_data[0] = cls
            m_data[1] = mid
            m_data[2] = able     # able is rate
            # UBX-CFG-MSG
            self.gps_send(6, 1, m_data)

    def send_able_tmode2(self, able, args, command):
        """SURVEYIN, UBX-CFG-TMODE2, set time mode 2 config"""

        m_data = bytearray(28)

        # on a NEO-M8T, with good antenna
        # five minutes, gets about 1 m
        # ten minutes, gets about 0.9 m
        # twenty minutes, gets about 0.7 m
        # one hour, gets about 0.5 m
        # twelve hours, gets about 0.14 m

        # Survey-in minimum duration seconds
        seconds = 300

        # Survey-in position accuracy limit in mm
        # make it big, so the duration decides when to end survey
        mmeters = 50000          # default 50 meters

        if able:
            # enable survey-in
            m_data[0] = 1
            if args and args[0]:
                seconds = int(args[0])
            if 1 < len(args) and args[1]:
                mmeters = int(args[1])

        struct.pack_into('<LL', m_data, 20, seconds, mmeters)
        self.gps_send(6, 0x3d, m_data)

    def send_able_tmode3(self, able, args, command):
        """SURVEYIN3, UBX-CFG-TMODE3, set time mode 3 config"""

        m_data = bytearray(40)

        # Survey-in minimum duration seconds
        seconds = 300

        # Survey-in position accuracy limit in 0.1mm
        # make it big, so the duration decides when to end survey
        # mmeters is 5m!
        mmeters = 500000         # default 50 meters

        if able:
            # enable survey-in
            m_data[2] = 1
            if args and args[0]:
                seconds = int(args[0])
            if 1 < len(args) and args[1]:
                mmeters = int(args[1])

        struct.pack_into('<LL', m_data, 24, seconds, mmeters)
        self.gps_send(6, 0x71, m_data)

    def send_cfg_cfg(self, save_clear, args=[]):
        """UBX-CFG-CFG, save config"""

        # Save: save_clear = 0
        # Clear: save_clear = 1

        # basic configs always available to change:
        # ioPort = 1, msgConf = 2, infMsg = 4, navConf = 8, rxmConf =0x10
        cfg1 = 0x1f
        # senConf = 1, rinvConf = 2, antConf = 4, logConf = 8, ftsConf = 0x10
        cfg2 = 0x0f

        m_data = bytearray(13)

        # clear mask
        # as of protver 27, any bit in clearMask clears all
        if 0 == save_clear:
            # saving, so do not clear
            m_data[0] = 0
            m_data[1] = 0
        else:
            # clearing
            m_data[0] = cfg1
            m_data[1] = cfg2
        m_data[2] = 0       #
        m_data[3] = 0       #

        # save mask
        # as of protver 27, any bit in saveMask saves all
        if 0 == save_clear:
            # saving
            m_data[4] = cfg1
            m_data[5] = cfg2
        else:
            # clearing, so do not save
            m_data[4] = 0
            m_data[5] = 0
        m_data[6] = 0       #
        m_data[7] = 0       #

        # load mask
        # as of protver 27, any bit in loadMask loads all
        if False and 0 == save_clear:
            # saving
            m_data[8] = 0
            m_data[9] = 0
        else:
            # clearing, load it to save a reboot
            m_data[8] = cfg1
            m_data[9] = cfg2
        m_data[10] = 0      #
        m_data[11] = 0      #

        # deviceMask, where to save it, try all options
        # devBBR = 1, devFLASH = 2, devEEPROM = 4, devSpiFlash = 0x10
        m_data[12] = 0x17

        self.gps_send(6, 0x9, m_data)

    def send_cfg_gnss1(self, gnssId, enable, args):
        """UBX-CFG-GNSS, set GNSS config

WARNING: the receiver will  ACK, then ignore, many seemingly valid settings.
Always double check with "-p CFG-GNSS".

Not supported on 9-series and later.
"""

        # FIXME!  Add warning if ,2 is requested on single frequency devices
        # Only u-blox 9 supports L2, except for M9N does not.

        m_data = bytearray(12)
        m_data[0] = 0       # version 0, msgVer
        m_data[1] = 0       # read only protVer 23+, numTrkChHw
        m_data[2] = 0xFF    # read only protVer 23+, numTrkChUse
        m_data[3] = 1       # 1 block follows
        # block 1
        m_data[4] = gnssId  # gnssId

        # m_data[5], resTrkCh, read only protVer 23+
        # m_data[6], maxTrkCh, read only protVer 23+
        m_data[7] = 0       # reserved1
        m_data[8] = enable  # flags
        m_data[9] = 0       # flags, unused

        # m_data[10], sigCfgMask, enable all signals
        if args:
            parm1 = args[0]
        else:
            parm1 = ''

        if 0 == gnssId:
            # GPS.  disable does not seem to work on NEO-M9N?
            m_data[5] = 8   # resTrkCh
            m_data[6] = 16  # maxTrkCh
            if '2' == parm1 and enable:
                # The NEO-M9N ACKS, then ignores if 0x11 is sent
                m_data[10] = 0x11   # flags L1C/A, L2C
            else:
                m_data[10] = 0x01   # flags L1C/A
        elif 1 == gnssId:
            # SBAS
            m_data[5] = 1   # resTrkCh
            m_data[6] = 3   # maxTrkCh
            m_data[10] = 1      # flags L1C/A
        elif 2 == gnssId:
            # Galileo
            m_data[5] = 4   # resTrkCh
            m_data[6] = 8   # maxTrkCh
            if '2' == parm1 and enable:
                # The NEO-M9N ACKS, then ignores if 0x11 is sent
                m_data[10] = 0x21   # flags E1, E5b
            else:
                m_data[10] = 0x01   # flags E1
        elif 3 == gnssId:
            # BeiDou
            m_data[5] = 2   # resTrkCh
            m_data[6] = 16  # maxTrkCh
            if '2' == parm1 and enable:
                # The NEO-M9N ACKS, then ignores if 0x11 is sent
                m_data[10] = 0x11   # flags B1I, B2I
            else:
                m_data[10] = 0x01   # flags B1I
        elif 4 == gnssId:
            # IMES
            m_data[5] = 0   # resTrkCh
            m_data[6] = 8   # maxTrkCh
            m_data[10] = 1      # flags L1
        elif 5 == gnssId:
            # QZSS
            m_data[5] = 0   # resTrkCh
            m_data[6] = 3   # maxTrkCh
            if '2' == parm1 and enable:
                # The NEO-M9N ACKS, then ignores if 0x11 is sent
                m_data[10] = 0x15   # flags L1C/A, L1S, L2C
            else:
                m_data[10] = 0x05   # flags L1C/A, L1S
        elif 6 == gnssId:
            # GLONASS
            m_data[5] = 8   # resTrkCh
            m_data[6] = 14  # maxTrkCh
            if '2' == parm1 and enable:
                # The NEO-M9N ACKS, then ignores if 0x11 is sent
                m_data[10] = 0x11   # flags L1, L2
            else:
                m_data[10] = 0x01   # flags L1
        else:
            # what else?
            m_data[10] = 1      # flags L1

        m_data[11] = 0          # flags, bits 24:31, unused
        self.gps_send(6, 0x3e, m_data)

    def send_poll_cfg_esfalg(self, args):
        """UBX-CFG-ESFALG, poll/set optional doAutoMntAlg"""

        if 0 < len(args):
            # optional set doAutoMntAlg
            m_data = bytearray(12)
            m_data[1] = int(args[0])
        else:
            m_data = bytearray(0)

        self.gps_send(6, 0x56, m_data)

    def send_poll_cfg_hnr(self, args):
        """UBX-CFG-HNR, poll/set optional highNavRate"""

        if 0 < len(args):
            # optional set highNavRate
            m_data = bytearray(4)
            m_data[0] = int(args[0])
        else:
            m_data = bytearray(0)

        self.gps_send(6, 0x5c, m_data)

    def poll_cfg_inf(self):
        """UBX-CFG-INF, poll"""

        m_data = bytearray(1)
        m_data[0] = 0        # UBX
        self.gps_send(6, 0x02, m_data)

        m_data[0] = 1        # NMEA
        self.gps_send(6, 0x02, m_data)

    def send_poll_cfg_msg(self, args):
        """UBX-CFG-MSG, mandatory args: class, ID, optional rate"""

        if 0 == len(args):
            sys.stderr.write('gps/ubx: ERROR: CFG-MSG missing class\n')
            sys.exit(1)
        if 1 == len(args):
            sys.stderr.write('gps/ubx: ERROR: CFG-MSG,x%x missing ID\n' %
                             (args[0]))
            sys.exit(1)

        if 2 < len(args):
            # optional set rate
            m_data = bytearray(3)
            try:
                m_data[2] = int(args[2], base=0)
            except ValueError as e:
                sys.stderr.write('gps/ubx: ERROR: CFG-MSG invalid rate: %s\n' %
                                 e)
                sys.exit(1)
        else:
            m_data = bytearray(2)

        # allow binary, hex and octal.
        try:
            m_data[0] = int(args[0], base=0)
        except ValueError as e:
            sys.stderr.write('gps/ubx: ERROR: CFG-MSG invalid class: %s\n' %
                             e)
            sys.exit(1)
        try:
            m_data[1] = int(args[1], base=0)
        except ValueError as e:
            sys.stderr.write('gps/ubx: ERROR: CFG-MSG invalid ID: %s\n' %
                             e)
            sys.exit(1)

        self.gps_send(6, 1, m_data)

    def send_cfg_nav5_model(self, args):
        """MODEL, UBX-CFG-NAV5, set dynamic platform model"""

        m_data = bytearray(36)
        if 0 < len(args):
            m_data[0] = 1        # just setting dynamic model
            m_data[1] = 0        # just setting dynamic model
            m_data[2] = int(args[0])
        # else:
        # Just polling deprecated

        self.gps_send(6, 0x24, m_data)

    def send_cfg_msg(self, m_class, m_id, rate=None):
        """UBX-CFG-MSG, poll, or set, message rates decode"""
        m_data = bytearray(2)
        m_data[0] = m_class
        m_data[1] = m_id
        if rate is not None:
            m_data.extend([rate])
        self.gps_send(6, 1, m_data)

    def send_cfg_pms(self, args):
        """UBX-CFG-PMS, poll/set Power Management Settings"""

        if 0 < len(args):
            m_data = bytearray(8)
            # set powerSetupValue to mode
            m_data[1] = int(args[0])
            # leave period and onTime zero, which breaks powerSetupValue = 3
        else:
            # just poll, DEPRECATED
            m_data = []

        self.gps_send(6, 0x86, m_data)

    def send_cfg_prt(self, args=None):
        """UBX-CFG-PRT, get I/O Port settings"""

        if 0 == len(args):
            # get current port
            m_data = []
        else:
            # get specified port
            # seems broken on ZED-F9P
            m_data = bytearray([int(args[0])])
        self.gps_send(6, 0x0, m_data)

    def send_cfg_rate(self, args):
        """UBX-CFG-RATE, poll/set rate settings

Deprecated in protVer 23.01
"""

        if 0 == len(args):
            # poll
            self.gps_send(6, 0x08, [])
            return

        m_data = bytearray(6)
        measRate = int(args[0])
        navRate = 1
        timeRef = 1
        if 1 < len(args):
            navRate = int(args[1])
            if 2 < len(args):
                timeRef = int(args[2])

        struct.pack_into('<HHH', m_data, 0, measRate, navRate, timeRef)
        self.gps_send(6, 0x08, m_data)

    def send_cfg_rst(self, reset_type):
        """UBX-CFG-RST, reset"""
        # Always do a hardware reset
        # If on native USB: both Hardware reset (0) and Software reset (1)
        # will disconnect and reconnect, giving you a new /dev/tty.
        m_data = bytearray(4)
        m_data[0] = reset_type & 0xff
        m_data[1] = (reset_type >> 8) & 0xff
        self.gps_send(6, 0x4, m_data)

    def send_cfg_rxm(self, args):
        """UBX-CFG-RXM, poll/set low power mode"""

        if 0 == len(args):
            # poll
            m_data = []
        else:
            # lpMode
            m_data = bytearray([0, int(args[0])])

        self.gps_send(6, 0x11, m_data)

    def send_cfg_slas(self, args):
        """UBX-CFG-SLAS, poll/set SLAS mode"""

        if 0 == len(args):
            # poll
            m_data = []
        else:
            # mode
            m_data = bytearray(4)
            m_data[0] = int(args[0])

        self.gps_send(6, 0x8d, m_data)

    def get_int_arg(self, args, ndx, default=0):
        """Convert args[ndx] to int, return default if not present"""
        if ((type(args) is not list or
             ndx >= len(args) or
             0 == len(args[ndx]))):
            return default
        # Allow hex (0x, decimal, octal (0o) and binary (0b10) string input
        return int(args[ndx], base=0)

    def send_cfg_tp5(self, args):
        """UBX-CFG-TP5, get/set timepulse config. Optional args:

tpIdx, antCableDelay, rfGroupDelay, freqPeriod, freqPeriodLock,
pulseLenRadio, pulseLenRadioLock, userConfigDelay, flags
"""
        if 0 == len(args):
            # poll with default tpIdx 0
            m_data = []
        elif 1 == len(args):
            # poll with the specified tpIdx
            m_data = bytearray([int(args[0])])
        else:
            # get/set timepulse
            tpIdx = self.get_int_arg(args, 0)
            antCableDelay = self.get_int_arg(args, 1, 2)
            rfGroupDelay = self.get_int_arg(args, 2)
            freqPeriod = self.get_int_arg(args, 3, 1000000)        # 1M us
            freqPeriodLock = self.get_int_arg(args, 4, 1000000)    # 1M us
            pulseLenRatio = self.get_int_arg(args, 5)
            pulseLenRatioLock = self.get_int_arg(args, 6, 100000)  # 100k us
            userConfigDelay = self.get_int_arg(args, 7)
            flags = self.get_int_arg(args, 8, 0x77)

            m_data = bytearray(32)
            m_data[0] = tpIdx
            m_data[1] = 1  # version
            m_data[2] = 0  # reserved
            m_data[3] = 0  # reserved
            m_data[4:6] = pack_s16(antCableDelay)
            m_data[6:8] = pack_s16(rfGroupDelay)
            m_data[8:12] = pack_u32(freqPeriod)
            m_data[12:16] = pack_u32(freqPeriodLock)
            m_data[16:20] = pack_u32(pulseLenRatio)
            m_data[20:24] = pack_u32(pulseLenRatioLock)
            m_data[24:28] = pack_s32(userConfigDelay)
            m_data[28:32] = pack_u32(flags)

        self.gps_send(6, 0x31, m_data)

    def send_set_speed(self, speed):
        """"UBX-CFG-PRT, set port"""
        port = self.port
        # FIXME!  Determine and use current port as default
        if port is None:
            port = 1  # Default to port 1 (UART/UART_1)

        if port not in set([1, 2]):
            sys.stderr.write('gps/ubx: Invalid UART port - %d\n' %
                             (port))
            sys.exit(2)

        # FIXME!  Poll current masks, then adjust speed
        m_data = bytearray(20)
        m_data[0] = port
        m_data[4] = 0xc0          # 8N1
        m_data[5] = 0x8           # 8N1

        m_data[8] = speed & 0xff
        m_data[9] = (speed >> 8) & 0xff
        m_data[10] = (speed >> 16) & 0xff
        m_data[11] = (speed >> 24) & 0xff

        m_data[12] = 3             # in, ubx and nmea
        m_data[14] = 3             # out, ubx and nmea
        self.gps_send(6, 0, m_data)

    def send_cfg_valdel(self, keys):
        """UBX-CFG-VALDEL, delete config items by key

present in u-blox NEO-D9S+, protver 24
present in 9-series and higher
"""

        m_data = bytearray(4)
        m_data[0] = 0       # version, 0 = transactionless, 1 = transaction
        m_data[1] = 6       # 2 = BBR, 4 = flash
        # can not delete RAM layer!
        # so options stay set until reset!

        for key in keys:
            k_data = bytearray(4)
            k_data[0] = (key) & 0xff
            k_data[1] = (key >> 8) & 0xff
            k_data[2] = (key >> 16) & 0xff
            k_data[3] = (key >> 24) & 0xff
            m_data.extend(k_data)
        self.gps_send(0x06, 0x8c, m_data)

    def send_cfg_valget(self, keys, layer, position):
        """UBX-CFG-VALGET, get config items by key"""

        # present in u-blox NEO-D9S+, protver 24
        # present in 9-series and higher

        m_data = bytearray(4)
        # version, 0 = request, 1 = answer
        # RAM layer
        # position
        struct.pack_into('<BBH', m_data, 0, 0, 0, position)

        k_data = bytearray(4)
        for key in keys:
            struct.pack_into('<L', k_data, 0, key)
            m_data.extend(k_data)

        if layer is None:
            # blast them for now, should do one at a time...
            for lyr in set([0, 1, 2, 7]):
                m_data[1] = lyr
                self.gps_send(0x06, 0x8b, m_data)
        else:
            m_data[1] = layer
            self.gps_send(0x06, 0x8b, m_data)

    def send_cfg_valset(self, nvs):
        """UBX-CFG-VALSET, set config items by key/val pairs"""

        # present in u-blox NEO-D9S+, protver 24
        # present in 9-series and higher

        # send CFG-VALSET one at a time, yes we could be smarter,
        # but this gives better error messages.
        for nv in nvs:
            m_data = bytearray(4)
            m_data[0] = 0      # version, 0 = request, 1 = transaction
            m_data[1] = 0x7    # RAM layer, 1=RAM, 2=BBR, 4=Flash

            size = 4
            nv_split = nv.split(',')
            name = nv_split[0]
            val = nv_split[1]
            if 3 <= len(nv_split):
                # layer requested
                m_data[1] = int(nv_split[2])

            item = self.cfg_by_name(name)
            key = item[1]
            # val_type = item[2]  # unused

            cfg_type = self.item_to_type(item)

            size = 4 + cfg_type[0]
            frmat = cfg_type[1]
            flavor = cfg_type[2]
            if 'u' == flavor:
                val1 = int(val, 0)      # allow 0x and 0b prefix
            elif 'i' == flavor:
                val1 = int(val, 0)      # allow 0x and 0b prefix
            elif 'f' == flavor:
                val1 = float(val)

            kv_data = bytearray(size)
            kv_data[0] = (key) & 0xff
            kv_data[1] = (key >> 8) & 0xff
            kv_data[2] = (key >> 16) & 0xff
            kv_data[3] = (key >> 24) & 0xff

            struct.pack_into(frmat, kv_data, 4, val1)
            m_data.extend(kv_data)
            self.gps_send(0x06, 0x8a, m_data)

    def send_log_findtime(self, args):
        """UBX-LOG-FINDTIME, search log for y,m,d,h,m,s"""
        m_data = bytearray(10)
        m_data[0] = 0      # version, 0 = request
        m_data[1] = 0      # type, 0 = request
        # the doc says two reserved bytes here, the doc is wrong
        # searches for before 1 Jan 2004 always fail
        year = 2004
        month = 1
        day = 1
        hour = 0
        minute = 0
        second = 0

        if 0 < len(args):
            year = int(args[0])
            if 1 < len(args):
                month = int(args[1])
                if 2 < len(args):
                    day = int(args[2])
                    if 3 < len(args):
                        hour = int(args[3])
                        if 4 < len(args):
                            minute = int(args[4])
                            if 5 < len(args):
                                second = int(args[5])

        m_data[2] = year % 256                # year 1-65635
        m_data[3] = int(year / 256) & 0xff    # year
        m_data[4] = month                     # month 1-12
        m_data[5] = day                       # day 1-31
        m_data[6] = hour                      # hour 0-23
        m_data[7] = minute                    # minute 0-59
        m_data[8] = second                    # second 0-60
        m_data[9] = 0      # reserved
        self.gps_send(0x21, 0x0e, m_data)

    def send_log_retrieve(self, args):
        """UBX-LOG-RETRIEVE, gets logs from start,count"""
        m_data = bytearray(12)

        # defaults
        startNumber = 0
        entryCount = 256      # max 256

        if 0 < len(args):
            startNumber = int(args[0])
            if 1 < len(args):
                entryCount = int(args[1])

        struct.pack_into('<LLB', m_data, 0, startNumber, entryCount, 0)
        self.gps_send(0x21, 0x09, m_data)

    def send_log_string(self, args):
        """UBX-LOG-STRING, send string to log"""

        if 0 < len(args):
            s = args[0][0:256]
        else:
            s = "Hi"

        m_data = gps.polybytes(s)
        self.gps_send(0x21, 0x04, m_data)

    def send_poll(self, m_data):
        """generic send poll request"""
        self.gps_send(m_data[0], m_data[1], m_data[2:])

    def send_cmds(self, cmds):
        """Send a list of commands"""
        # blast them for now, should do one at a time...
        # for some reason NEO-M8U responds in different order!
        # so can not depend on response order, or any response at all.
        for cmd in cmds:
            self.send_poll(cmd)

    CFG_ANT = [0x06, 0x13]
    CFG_BATCH = [0x06, 0x93]
    CFG_DAT = [0x06, 0x06]
    CFG_ESFA = [0x06, 0x4c]
    CFG_ESFALG = [0x06, 0x56]
    CFG_ESFG = [0x06, 0x4d]
    CFG_ESFWT = [0x06, 0x82]
    CFG_GNSS = [0x06, 0x3e]
    CFG_GEOFENCE = [0x06, 0x69]
    CFG_HNR = [0x06, 0x5c]
    CFG_INF_0 = [0x06, 0x02, 0]
    CFG_INF_1 = [0x06, 0x02, 1]
    CFG_LOGFILTER = [0x06, 0x47]
    CFG_ODO = [0x06, 0x1e]
    CFG_PRT = [0x06, 0x00]     # current port only
    CFG_NAV5 = [0x06, 0x24]
    CFG_NAVX5 = [0x06, 0x23]
    CFG_NMEA = [0x06, 0x17]
    CFG_PM2 = [0x06, 0x3b]
    CFG_PMS = [0x06, 0x86]
    CFG_RATE = [0x06, 0x08]
    CFG_RXM = [0x06, 0x11]
    CFG_TMODE3 = [0x06, 0x71]
    CFG_TP5 = [0x06, 0x31]
    CFG_USB = [0x06, 0x1b]
    ESF_MEAS = [0x10, 0x02]
    ESF_ALG = [0x10, 0x14]
    ESF_CAL = [0x10, 0x04]
    ESF_INS = [0x10, 0x15]
    ESF_RESETALG = [0x10, 0x13]
    ESF_STATUS = [0x10, 0x10]
    HNR_ATT = [0x28, 0x01]
    HNR_INS = [0x28, 0x02]
    HNR_PVT = [0x28, 0x00]
    LOG_INFO = [0x21, 0x08]
    MON_COMMS = [0x0a, 0x36]
    MON_GNSS = [0x0a, 0x28]
    MON_HW = [0x0a, 0x09]
    MON_HW2 = [0x0a, 0x0b]
    MON_HW3 = [0x0a, 0x37]
    MON_IO = [0x0a, 0x02]
    MON_MSGPP = [0x0a, 0x06]
    MON_RF = [0x0a, 0x38]
    MON_RXBUF = [0x0a, 0x38]
    MON_SYS = [0x0a, 0x39]
    MON_TXBUF = [0x0a, 0x08]
    MON_VER = [0x0a, 0x04]
    NAV_EELL = [0x01, 0x3d]
    NAV_HPPOSECEF = [0x01, 0x13]
    NAV_HPPOSLLH = [0x01, 0x14]
    NAV_ODO = [0x01, 0x10]
    NAV_POSECEF = [0x01, 0x01]
    NAV_POSLLH = [0x01, 0x02]
    NAV_PVAT = [0x01, 0x17]
    NAV_PVT = [0x01, 0x07]
    NAV_SAT = [0x01, 0x35]
    NAV_SIG = [0x01, 0x43]
    NAV_SVIN = [0x01, 0x3b]
    NAV_TIMEBDS = [0x01, 0x24]
    NAV_TIMEGAL = [0x01, 0x25]
    NAV_TIMEGLO = [0x01, 0x23]
    NAV_TIMEGPS = [0x01, 0x20]
    NAV_TIMELS = [0x01, 0x26]
    NAV_TIMENAVIC = [0x01, 0x63]
    NAV_TIMEQZSS = [0x01, 0x27]
    NAV_TIMEUTC = [0x01, 0x21]
    NAV_VELNED = [0x01, 0x12]
    TIM_SVIN = [0x0d, 0x04]

    def send_poll_esf(self):
        """ESR. poll ESF messages"""

        cmds = [ubx.CFG_ESFA,
                ubx.CFG_ESFALG,
                ubx.CFG_ESFG,
                ubx.CFG_ESFWT,
                ubx.ESF_ALG,
                ubx.ESF_INS,
                ubx.ESF_STATUS,
                ]

        self.send_cmds(cmds)

    def send_poll_hnr(self):
        """HNR. poll HNR messages"""

        cmds = [ubx.CFG_HNR,
                ubx.HNR_ATT,
                ubx.HNR_INS,
                ubx.HNR_PVT,
                ]

        self.send_cmds(cmds)

    def get_config(self):
        """CONFIG. Request a bunch of config messages, by protVer"""

        # these two always
        cmds = [ubx.MON_VER,          # UBX-MON-VER
                ubx.CFG_PRT,          # UBX-CFG-PRT
                ]

        if 29 > self.protver:
            # these are gone - 29
            cmds += [
                ubx.CFG_ANT,          # UBX-CFG-ANT
                ubx.CFG_DAT,          # UBX-CFG-DAT
                # skip UBX-CFG-DGNSS, HP only
                # skip UBX-CFG-DOSC, FTS only
                # skip UBX-CFG-ESRC, FTS only
                ubx.CFG_GEOFENCE,     # UBX-CFG-GEOFENCE
                ubx.CFG_GNSS,         # UBX-CFG-GNSS
                # skip UBX-CFG-HNR, ADR, UDR, only
                ubx.CFG_INF_0,        # UBX-CFG-INF
                ubx.CFG_INF_1,
                # skip UBX-CFG-ITFM
                ubx.CFG_LOGFILTER,    # UBX-CFG-LOGFILTER
                ubx.CFG_NAV5,         # UBX-CFG-NAV5
                ubx.CFG_NAVX5,        # UBX-CFG-NAVX5
                ubx.CFG_NMEA,         # UBX-CFG-NMEA
                ubx.CFG_ODO,          # UBX-CFG-ODO
                ubx.CFG_PM2,          # UBX-CFG-PM2
                ubx.CFG_PMS,          # UBX-CFG-PMS
                ubx.CFG_RATE,         # UBX-CFG-RATE
                ubx.CFG_RXM,          # UBX-CFG-RXM
                ubx.CFG_TP5,          # UBX-CFG-TP5
                ubx.CFG_USB,          # UBX-CFG-USB
                ]
        else:
            # under protVer 29
            if 20 < self.protver:
                cmds.append(ubx.CFG_TMODE3)  # UBX-CFG-TMODE3, protVer 20+
            if 22 < self.protver:
                cmds.append(ubx.CFG_BATCH)   # UBX-CFG-BATCH, protVer 23.01+

        self.send_cmds(cmds)

    def get_status(self):
        """STATUS.  Get a bunch of status messages"""

        cmds = [ubx.MON_VER,          # UBX-MON-VER
                ubx.LOG_INFO,         # UBX-LOG-INFO
                ubx.MON_GNSS,         # UBX-MON-GNSS
                # UBX-MON-PATH, skipping
                ]

        if 27 <= self.protver:
            cmds.extend([ubx.MON_COMMS,        # UBX-MON-COMMS
                         ubx.MON_HW3,          # UBX-MON-HW3
                         ubx.MON_RF,           # UBX-MON-RF
                         ubx.MON_SYS,          # UBX-MON-SYS
                         ])
        else:
            # deprecated in 27+
            cmds.extend([ubx.MON_HW,           # UBX-MON-HW
                         ubx.MON_HW2,          # UBX-MON-HW2
                         ubx.MON_IO,           # UBX-MON-IO
                         ubx.MON_MSGPP,        # UBX-MON-MSGPP
                         ubx.MON_RF,           # UBX-MON-RF
                         ubx.MON_RXBUF,        # UBX-MON-RXBUF
                         ubx.MON_TXBUF,        # UBX-MON-TXBUF
                         ])

        # should only send these for Time or HP products
        cmds.extend([ubx.NAV_SVIN,        # UBX-NAV-SVIN
                     ubx.TIM_SVIN,        # UBX-TIM-SVIN
                     ])

        self.send_cmds(cmds)

    # unified table of poll, preset, able, and disable commants.
    commands = {
        # UBX-AID-* removed from ProtVer 34 and up.
        # UBX-AID-ALM
        "AID-ALM": {"pollcmd": send_poll, "mid": [0x0b, 0x30],
                    "help": "poll UBX-AID-ALM Poll GPS Aiding Almanac Data"},
        # UBX-AID-AOP
        "AID-AOP": {"pollcmd": send_poll, "mid": [0x0b, 0x33],
                    "help": "poll UBX-AID-AOP Poll Poll AssistNow "
                    "Autonomous data"},
        # UBX-AID-DATA
        "AID-DATA": {"pollcmd": send_poll, "mid": [0x0b, 0x10],
                     "help": "Poll all GPS Initial Aiding Data"},
        # UBX-AID-EPH
        "AID-EPH": {"pollcmd": send_poll, "mid": [0x0b, 0x31],
                    "help": "poll UBX-AID-EPH Poll GPS Aiding Ephemeris Data"},
        # UBX-AID-HUI
        "AID-HUI": {"pollcmd": send_poll, "mid": [0x0b, 0x02],
                    "help": "poll UBX-AID-HUI Poll GPS Health, UTC, Iono"},
        # UBX-AID-INI
        "AID-INI": {"pollcmd": send_poll, "mid": [0x0b, 0x01],
                    "help": "poll UBX-AID-INI Poll Aiding position, time,\n"
                    "                    "
                    "frequency, clock drift"},
        # en/dis able BATCH
        "BATCH": {"ablecmd": send_able_cfg_batch,
                  "help": "batching, using CFG-BATCH"},
        # en/dis able BeiDou
        "BEIDOU": {"ablecmd": send_able_beidou,
                   "help": "BEIDOU for B1. BEIDOU,2 for B1 and B2"},
        # en/dis able basic binary messages
        "BINARY": {"ablecmd": send_able_binary,
                   "help": "basic binary messages"},
        # UBX-CFG-ANT
        "CFG-ANT": {"pollcmd": send_poll, "mid": [0x06, 0x13],
                    "help": "poll UBX-CFG-ANT antenna config"},
        # UBX-CFG-BATCH
        # Assume 23 is close enough to the proper 23.01
        "CFG-BATCH": {"pollcmd": send_poll, "mid": [0x06, 0x93],
                      "help": "poll UBX-CFG-BATCH data batching config",
                      "minVer": 23},
        # UBX-CFG-DAT
        "CFG-DAT": {"pollcmd": send_poll, "mid": [0x06, 0x06],
                    "help": "poll UBX-CFG-DAT Datum Setting"},
        # UBX-CFG-DGNSS
        "CFG-DGNSS": {"pollcmd": send_poll, "mid": [0x06, 0x70],
                      "help": "poll UBX-CFG-DGNSS DGNSS configuration"},
        # UBX-CFG-DOSC
        "CFG-DOSC": {"pollcmd": send_poll, "mid": [0x06, 0x61],
                     "help": "poll UBX-CFG-DOSC Disciplined oscillator"
                     "configuration"},
        # UBX-CFG-ESFA
        "CFG-ESFA": {"pollcmd": send_poll, "mid": CFG_ESFA,
                     "help": "poll UBX-CFG-ESFA Accelerometer configuration"},
        # UBX-CFG-ESFALG
        "CFG-ESFALG": {"pollcmd": send_poll_cfg_esfalg,
                       "help": "poll UBX-CFG-ESFALG IMU alignment config\n"
                               "                    "
                               "UBX-CFG-ESFALG[,doAutoMntAlg] optional",
                       "args": 0},
        # UBX-CFG-ESFG
        "CFG-ESFG": {"pollcmd": send_poll, "mid": CFG_ESFG,
                     "help": "poll UBX-CFG-ESFG Gyro configuration"},
        # UBX-CFG-ESWTF
        "CFG-ESFWT": {"pollcmd": send_poll, "mid": CFG_ESFWT,
                      "help": "poll UBX-CFG-ESFWY Wheel tick configuration"},
        # UBX-CFG-ESRC
        "CFG-ESRC": {"pollcmd": send_poll, "mid": [0x06, 0x60],
                     "help": "poll UBX-CFG-ESRC External synchronization "
                     "source config"},
        # UBX-CFG-FXN
        "CFG-FXN": {"pollcmd": send_poll, "mid": [0x06, 0x0e],
                    "help": "poll UBX-CFG-FXN FXN Configuration"},
        # UBX-CFG-GEOFENCE
        "CFG-GEOFENCE": {"pollcmd": send_poll, "mid": [0x06, 0x69],
                         "help": "poll UBX-CFG-GEOFENCE Geofencing "
                         "configuration"},
        # UBX-CFG-GNSS
        "CFG-GNSS": {"pollcmd": send_poll, "mid": [0x06, 0x3e],
                     "help": "poll UBX-CFG-GNSS GNSS config"},
        # UBX-CFG-HNR
        "CFG-HNR": {"pollcmd": send_poll_cfg_hnr,
                    "help": "poll UBX-CFG-HNR Settings\n"
                            "                    "
                            "set UBX-CFG-HNR,[highNavRate]\n"
                            "                    "
                            "highNavRate is optional and sets rate.",
                    "args": 0},
        # UBX-CFG-INF
        "CFG-INF": {"pollcmd": poll_cfg_inf,
                    "help": "poll UBX-CFG-INF Information Message "
                            "Configuration"},
        # UBX-CFG-ITFM
        "CFG-ITFM": {"pollcmd": send_poll, "mid": [0x06, 0x39],
                     "help": "poll UBX-CFG-ITFM Jamming/Interference "
                     "Monitor configuration"},
        # UBX-CFG-LOGFILTER
        "CFG-LOGFILTER": {"pollcmd": send_poll, "mid": [0x06, 0x47],
                          "help": "poll UBX-CFG-LOGFILTER "
                          " Data Logger Configuration",
                          "minVer": 14},
        # UBX-CFG-MSG
        "CFG-MSG": {"pollcmd": send_poll_cfg_msg,
                    "help": "poll/set UBX-CFG-MSG,class,ID[,rate]\n"
                            "                    "
                            "rate is optional and sets rate.",
                    "args": 2},
        # UBX-CFG-NAV5
        "CFG-NAV5": {"pollcmd": send_poll, "mid": [0x06, 0x24],
                     "help": "poll UBX-CFG-NAV5 Nav Engines settings"},
        # UBX-CFG-NAVX5
        "CFG-NAVX5": {"pollcmd": send_poll, "mid": [0x06, 0x23],
                      "help": "poll UBX-CFG-NAVX5 Nav Expert Settings"},
        # UBX-CFG-NMEA
        "CFG-NMEA": {"pollcmd": send_poll, "mid": [0x06, 0x17],
                     "help": "poll UBX-CFG-NMEA Extended NMEA protocol "
                             "configuration V1"},
        # UBX-CFG-ODO
        "CFG-ODO": {"pollcmd": send_poll, "mid": [0x06, 0x1e],
                    "help": "poll UBX-CFG-ODO Odometer, Low-speed COG "
                            "Engine Settings"},
        # UBX-CFG-PM
        "CFG-PM": {"pollcmd": send_poll, "mid": [0x06, 0x32],
                   "help": "poll UBX-CFG-PM Power management settings"},
        # UBX-CFG-PM2
        "CFG-PM2": {"pollcmd": send_poll, "mid": [0x06, 0x3b],
                    "help": "poll UBX-CFG-PM2 Extended power management "
                    "settings"},
        # UBX-CFG-PMS
        "CFG-PMS": {"pollcmd": send_cfg_pms,
                    "help": "poll/set UBX-CFG-PMS power management settings\n"
                            "                    "
                            "CFG-PMS[,powerSetupValue]",
                    "args": 1},
        # UBX-CFG-PRT
        "CFG-PRT": {"pollcmd": send_cfg_prt,
                    "help": "poll UBX-CFG-PRT I/O port settings.\n"
                            "                    "
                            "CFG-PRT[,portID] defaults to current port",
                    "args": 1},
        # TODO: UBX-CFG-PWR
        # UBX-CFG-RATE
        "CFG-RATE": {"pollcmd": send_cfg_rate,
                     "help": "poll/set UBX-CFG-RATE measure/nav settings.\n"
                             "                    "
                             "CFG-RATE[,measRate,[navRate]]",
                     "args": 2},
        # UBX-CFG-RINV
        "CFG-RINV": {"pollcmd": send_poll, "mid": [0x06, 0x34],
                     "help": "poll UBX-CFG-RINV Contents of Remote Inventory"},
        # UBX-CFG-RST, see COLDBOOT, WARMBOOT, HOTBOOT
        # UBX-CFG-RXM
        "CFG-RXM": {"pollcmd": send_cfg_rxm,
                    "help": "poll/set UBX-CFG-RXM RXM configuration.\n"
                            "                    "
                            "CFG-RXM[,lpMode]",
                    "args": 1},
        # UBX-CFG-SBAS
        "CFG-SBAS": {"pollcmd": send_poll, "mid": [0x06, 0x16],
                     "help": "poll UBX-CFG-SBAS SBAS settings"},
        # UBX-CFG-SLAS
        "CFG-SLAS": {"pollcmd": send_cfg_slas,
                     "help": "poll/set UBX-CFG-SLAS SLAS configuration.\n"
                             "                    "
                             "CFG-SLAS[,mode]",
                     "args": 1},
        # UBX-CFG-SMGR
        "CFG-SMGR": {"pollcmd": send_poll, "mid": [0x06, 0x62],
                     "help": "poll UBX-CFG-SMGR Synchronization manager "
                     "configuration"},
        # UBX-CFG-TMODE
        "CFG-TMODE": {"pollcmd": send_poll, "mid": [0x06, 0x1d],
                      "help": "poll UBX-CFG-TMODE time mode settings",
                      "maxVer": 6},
        # UBX-CFG-TMODE2
        "CFG-TMODE2": {"pollcmd": send_poll, "mid": [0x06, 0x3d],
                       "help": "poll UBX-CFG-TMODE2 time mode 2 config",
                       "minVer": 14},
        # UBX-CFG-TMODE3
        "CFG-TMODE3": {"pollcmd": send_poll, "mid": [0x06, 0x71],
                       "help": "poll UBX-CFG-TMODE3 time mode 3 config",
                       "minVer": 20},
        # UBX-CFG-TP
        "CFG-TP": {"pollcmd": send_poll, "mid": [0x06, 0x07],
                   "help": "poll UBX-CFG-TP TimePulse Parameters."},
        # UBX-CFG-TP5
        "CFG-TP5": {"pollcmd": send_cfg_tp5,
                    "help": "poll UBX-TIM-TP5 time pulse decodes.\n"
                            "                    "
                            "CFG-TP5[,tpIdx]  Default tpIdx is 0\n"
                            "                  "
                            "set UBX-TIM-TP5 time pulse decodes.\n"
                            "                    "
                            "CFG-TP5,[tpIdx],[antCableDelay],[rfGroupDelay]\n"
                            "                      "
                            ",[freqPeriod],[freqPeriodLock],[pulseLenRadio]\n"
                            "                      "
                            ",[pulseLenRadioLock],[userConfigDelay],[flags]",
                    "args": 1},
        # UBX-CFG-USB
        "CFG-USB": {"pollcmd": send_poll, "mid": [0x06, 0x1b],
                    "help": "poll UBX-CFG-USB USB config"},
        # UBX-CFG-RST
        "COLDBOOT": {"pollcmd": send_cfg_rst,
                     "help": "UBS-CFG-RST coldboot the GPS",
                     "opt": 0xffff},
        # CONFIG
        "CONFIG": {"pollcmd": get_config,
                   "help": "Get a lot of receiver config"},
        # en/dis able GALILEO
        "GALILEO": {"ablecmd": send_able_galileo,
                    "help": "GALILEO E1. GALILEO,2 for E1 and E5b"},
        # en/dis able GLONASS
        "GLONASS": {"ablecmd": send_able_glonass,
                    "help": "GLONASS L1. GLONASS,2 for L1 and L2"},
        # en/dis able GPS
        "GPS": {"ablecmd": send_able_gps,
                "help": "GPS and QZSS L1C/A. GPS,2 for L1C/A and L2C"},
        # en/dis able HNR messages
        # UBX-HNR-
        "HNR": {"pollcmd": send_poll_hnr, "ablecmd": send_able_hnr,
                "help": "basic HNR messages"},
        # UBX-CFG-RST
        "HOTBOOT": {"pollcmd": send_cfg_rst,
                    "help": "UBX-CFG-RST hotboot the GPS",
                    "opt": 0},
        # en/dis able ECEF
        "ECEF": {"ablecmd": send_able_ecef,
                 "help": "ECEF"},
        # basic ESF messages
        "ESF": {"pollcmd": send_poll_esf, "ablecmd": send_able_esf,
                "help": "basic ESF messages", "args" : 1},
        # UBX-EFS-
        # UBX-ESF-ALG
        "ESF-ALG": {"pollcmd": send_poll, "mid": ESF_ALG,
                    "help": "poll UBX-ESF-ALG IMU alignment information"},
        # UBX-ESF-CAL
        "ESF-CAL": {"pollcmd": send_poll, "mid": ESF_CAL,
                    "help": "poll UBX-ESF-CAL IMU calibration information"},
        # UBX-ESF-INS
        "ESF-INS": {"pollcmd": send_poll, "mid": ESF_INS,
                    "help": "poll UBX-ESF-INS Vehicle dynamics info"},
        # UBX-ESF-MEAS
        "ESF-MEAS": {"pollcmd": send_poll, "mid": ESF_MEAS,
                     "help": "poll UBX-ESF-MEAS External Sensor Fusion "
                     "measurements"},
        # UBX-ESF-RESETALG
        "ESF-RESETALG": {"pollcmd": send_poll, "mid": ESF_RESETALG,
                         "help": "poll UBX-ESF-RESETALG reset IMU"},
        # UBX-ESF-STATUS
        "ESF-STATUS": {"pollcmd": send_poll, "mid": ESF_STATUS,
                       "help": "poll UBX-ESF-STATUS External sensor fusion "
                               "status"},
        # UBX-HNR-ATT
        "HNR-ATT": {"pollcmd": send_poll, "mid": [0x28, 0x01],
                    "help": "poll UBX-HNR-ATT Attitude solution"},
        # UBX-HNR-INS
        "HNR-INS": {"pollcmd": send_poll, "mid": [0x28, 0x02],
                    "help": "poll UBX-HNR-INS Vehicle dynamics information"},
        # UBX-HNR-PVT
        "HNR-PVT": {"pollcmd": send_poll, "mid": [0x28, 0x00],
                    "help": "poll UBX-HNR-PVT HNR PVT solution"},
        # en/dis able LOG
        "LOG": {"ablecmd": send_able_logfilter,
                "help": "Data Logger"},
        # UBX-LOG-CREATE
        "LOG-CREATE": {"pollcmd": send_poll,
                       "opt": [0x21, 0x07, 0, 1, 0, 0, 0, 0, 0, 0],
                       "help": "send UBX-LOG-CREATE",
                       "minVer": 14},
        # UBX-LOG-ERASE
        "LOG-ERASE": {"pollcmd": send_poll, "mid": [0x21, 0x03],
                      "help": "send UBX-LOG-ERASE",
                      "minVer": 14},
        # UBX-LOG-FINDTIME
        "LOG-FINDTIME": {"pollcmd": send_log_findtime,
                         "help": "search logs by time. "
                                 "LOG-FINDTIME,y,m,d,h,m,s\n"
                                 "                    "
                                 "all parameters optional",
                         "args": 6},
        # UBX-LOG-INFO
        "LOG-INFO": {"pollcmd": send_poll, "mid": [0x21, 0x08],
                     "help": "poll UBX-LOG-INFO",
                     "minVer": 14},
        # UBX-LOG-RETRIEVE
        "LOG-RETRIEVE": {"pollcmd": send_log_retrieve,
                         "help": "send UBX-LOG-RETRIEVE. "
                                 "LOG-RETRIEVE[,start,[count]]",
                         "minVer": 14,
                         "args": 2},
        # UBX-LOG-RETRIEVEBATCH
        # Assume 23 is close enough to the proper 23.01
        "LOG-RETRIEVEBATCH": {"pollcmd": send_poll,
                              "opt": [0x21, 0x10, 0, 1, 0, 0],
                              "help": "send UBX-LOG-RETRIEVEBATCH",
                              "minVer": 23},
        # UBX-LOG-STRING
        "LOG-STRING": {"pollcmd": send_log_string,
                       "help": "send UBX-LOG-STRING. LOG-STRING[,string]",
                       "minVer": 14,
                       "args": 1},
        # UBX-MGA-DBD
        "MGA-DBD": {"pollcmd": send_poll, "mid": [0x13, 0x80],
                    "help": "poll UBX-MGA-DBD Poll the Navigation Database"},
        # UBX-MGA-SF
        "MGA-SF": {"pollcmd": send_poll, "mid": [0x13, 0x10],
                   "help": "poll UBX-MGA-SD Poll the Sensor Fusion data"},
        # UBX-CFG-NAV5
        "MODEL": {"pollcmd": send_cfg_nav5_model, "args": 1,
                  "help": "set UBX-CFG-NAV5 Dynamic Platform Model. "
                          "MODEL,model"},
        # UBX-MON-BATCH
        # Assume 23 is close enough to the proper 23.01
        "MON-BATCH": {"pollcmd": send_poll, "mid": [0x0a, 0x32],
                      "help": "poll UBX-MON-BATCH Data batching "
                      "buffer status",
                      "maxVer": 23.99,
                      "minVer": 23},
        # UBX-MON-COMMS
        "MON-COMMS": {"pollcmd": send_poll, "mid": [0x0a, 0x36],
                      "ablecmd": send_able,
                      "help": "poll UBX-MON-COMMS Comm port information"},
        # UBX-MON-GNSS
        "MON-GNSS": {"pollcmd": send_poll, "mid": [0x0a, 0x28],
                     "help": "poll UBX-MON-GNSS major GNSS selection"},
        # UBX-MON-HW
        "MON-HW": {"pollcmd": send_poll, "mid": [0x0a, 0x09],
                   "help": "poll UBX-MON-HW Hardware Status"},
        # UBX-MON-HW2
        "MON-HW2": {"pollcmd": send_poll, "mid": [0x0a, 0x0b],
                    "help": "poll UBX-MON-HW2 Extended Hardware Status"},
        # UBX-MON-HW3
        "MON-HW3": {"pollcmd": send_poll, "mid": [0x0a, 0x37],
                    "help": "poll UBX-MON-HW3 HW I/O pin information"},
        # UBX-MON-IO
        "MON-IO": {"pollcmd": send_poll, "mid": [0x0a, 0x02],
                   "help": "poll UBX-MON-IO I/O Subsystem Status"},
        # UBX-MON-MSGPP
        "MON-MSGPP": {"pollcmd": send_poll, "mid": [0x0a, 0x06],
                      "help": "poll UBX-MON-MSGPP Message Parese and "
                              "Process Status"},
        # UBX-MON-PATCH
        "MON-PATCH": {"pollcmd": send_poll, "mid": [0x0a, 0x27],
                      "help": "poll UBX-MON-PATCH Info on Installed Patches"},
        # UBX-MON-POST
        "MON-POST": {"pollcmd": send_poll, "mid": [0x0a, 0x3b],
                     "help": "poll UBX-MON-POST POST info"},
        # UBX-MON-RF
        "MON-RF": {"pollcmd": send_poll, "mid": [0x0a, 0x38],
                   "help": "poll UBX-MON-RF RF Information"},
        # UBX-MON-RXBUF
        "MON-RXBUF": {"pollcmd": send_poll, "mid": [0x0a, 0x07],
                      "help": "poll UBX-MON-RXBUF Receiver Buffer Status"},
        # UBX-MON-SMGR
        "MON-SMGR": {"pollcmd": send_poll, "mid": [0x0a, 0x2e],
                     "help": "poll UBX-MON-SMGR Synchronization manager "
                     "configuration"},
        # UBX-MON-SPAN
        "MON-SPAN": {"pollcmd": send_poll, "mid": [0x0a, 0x31],
                     "help": "poll UBX-MON-SPAN Signal characteristics"},
        # UBX-MON-SPT
        "MON-SPT": {"pollcmd": send_poll, "mid": [0x0a, 0x2f],
                    "help": "poll UBX-MON-SPT Sensor Production Test"},
        # UBX-MON-SYS
        "MON-SYS": {"pollcmd": send_poll, "mid": [0x0a, 0x39],
                    "help": "poll UBX-MON-SYS System state"},
        # UBX-MON-TXBUF
        "MON-TXBUF": {"pollcmd": send_poll, "mid": [0x0a, 0x08],
                      "help": "poll UBX-MON-TXBUF Transmitter Buffer Status"},
        # UBX-MON-VER
        "MON-VER": {"pollcmd": send_poll, "mid": [0x0a, 0x04],
                    "help": "poll UBX-MON-VER GPS version"},
        # UBX-NAV-AOPSTATUS
        "NAV-AOPSTATUS": {"pollcmd": send_poll, "mid": [0x01, 0x60],
                          "help": "poll UBX-NAV-AOPSTATUS AssistNow "
                          "Autonomous Status"},
        # UBX-NAV-ATT
        "NAV-ATT": {"pollcmd": send_poll, "mid": [0x1, 0x5],
                    "help": "poll UBX-NAV-ATT Attitude Solution"},
        # UBX-NAV-CLOCK
        "NAV-CLOCK": {"pollcmd": send_poll, "mid": [0x01, 0x22],
                      "ablecmd": send_able,
                      "help": "UBX-NAV-CLOCK Clock Solution"},
        # UBX-NAV-DGPS
        "NAV-DGPS": {"pollcmd": send_poll, "mid": [0x01, 0x31],
                     "ablecmd": send_able,
                     "help": "UBX-NAV-DGPS DGPS Data Used for NAV"},
        # UBX-NAV-DOP
        "NAV-DOP": {"ablecmd": send_able, "mid": [0x01, 0x04],
                    "pollcmd": send_poll,
                    "help": "UBX-NAV-DOP Dilution of Precision"},
        # UBX-NAV-EELL
        # en/dis able NAV-EELL message
        "NAV-EELL": {"ablecmd": send_able, "mid": NAV_EELL,
                     "pollcmd": send_poll,
                     "help": "NAV-EELL error ellipse message"},
        # UBX-NAV-GEOFENCE
        "NAV-GEOFENCE": {"pollcmd": send_poll, "mid": [0x01, 0x39],
                         "help": "poll UBX-NAV-GEOFENCE Geofence status"},
        # UBX-NAV-HPPOSECEF
        "NAV-HPPOSECEF": {"ablecmd": send_able, "mid": NAV_HPPOSECEF,
                          "pollcmd": send_poll,
                          "help": "NAV-HPPOSECEF fix message"},
        # UBX-NAV-HPPOSLLH
        "NAV-HPPOSLLH": {"ablecmd": send_able, "mid": NAV_HPPOSLLH,
                         "pollcmd": send_poll,
                         "help": "NAV-HPPOSLLH fix message"},
        # UBX-NAV-ODO
        "NAV-ODO": {"pollcmd": send_poll, "mid": NAV_ODO,
                    "help": "poll UBX-NAV-ODO Odometer Solution"},
        # UBX-NAV-ORB
        "NAV-ORB": {"pollcmd": send_poll, "mid": [0x01, 0x34],
                    "help": "poll UBX-NAV-ORB GNSS Orbit Database Info"},
        "NAV-PL": {"pollcmd": send_poll, "mid": [0x01, 0x62],
                   "help": "poll UBX-NAV-PL Protection level info"},
        # NAV-POSECEF message
        "NAV-POSECEF": {"ablecmd": send_able, "mid": NAV_POSECEF,
                        "pollcmd": send_poll,
                        "help": "NAV-POSECEF fix message"},
        # NAV-POSLLH message
        "NAV-POSLLH": {"ablecmd": send_able, "mid": NAV_POSLLH,
                       "pollcmd": send_poll,
                       "help": "NAV-POSLLH fix message"},
        # UBX-NAV-PVAT
        "NAV-PVAT": {"ablecmd": send_able, "mid": NAV_PVAT,
                     "pollcmd": send_poll,
                     "help": "UBX-NAV-PVAT Navigation Position Velocity "
                             "Attitude Time"},
        # en/dis able NAV-PVT message
        "NAV-PVT": {"ablecmd": send_able, "mid": NAV_PVT,
                    "pollcmd": send_poll,
                    "help": "poll UBX-NAV-PVT Navigation Position Velocity "
                            "Time Solution"},
        # UBX-NAV-RELPOSNED
        # HP only, 20+, otherwise not ACKed or NACKed
        "NAV-RELPOSNED": {"pollcmd": send_poll, "mid": [0x01, 0x3c],
                          "help": "poll UBX-NAV-RELPOSNED Relative "
                                  "Positioning Info in NED frame"},
        # UBX-NAV-RESETODO
        "NAV-RESETODO": {"pollcmd": send_poll, "mid": [0x01, 0x10],
                         "help": "UBX-NAV-RESETODO Reset odometer"},
        # UBX-NAV-SAT
        "NAV-SAT": {"ablecmd": send_able, "mid": NAV_SAT,
                    "pollcmd": send_poll,
                    "help": "NAV-SAT Satellite Information message"},
        # UBX-NAV-SBAS
        "NAV-SBAS": {"pollcmd": send_poll, "mid": [0x01, 0x32],
                     "help": "poll UBX-NAV-SBAS SBAS Status Data"},
        # UBX-NAV-SIG
        "NAV-SIG": {"ablecmd": send_able, "mid": NAV_SIG,
                    "pollcmd": send_poll,
                    "help": "NAV-SIG Signal Information message"},
        # UBX-NAV-SLAS
        "NAV-SLAS": {"pollcmd": send_poll, "mid": [0x01, 0x42],
                     "help": "poll UBX-NAV-SLAS QZSS L1S SLAS Status Data"},
        # UBX-NAV-SOL
        "NAV-SOL": {"pollcmd": send_poll, "mid": [0x01, 0x06],
                    "help": "poll UBX-NAV-SOL Navigation Solution "
                    "Information"},
        # UBX-NAV-STATUS
        "NAV-STATUS": {"pollcmd": send_poll, "mid": [0x01, 0x03],
                       "help": "poll UBX-NAV-STATUS Receiver Nav Status"},
        # UBX-NAV-SVIN
        "NAV-SVIN": {"pollcmd": send_poll, "mid": [0x01, 0x3b],
                     "help": "poll UBX-NAV-SVIN Survey-in data",
                     "minver": 20},
        # UBX-NAV-SVINFO
        "NAV-SVINFO": {"pollcmd": send_poll, "mid": [0x01, 0x30],
                       "help": "poll UBX-NAV-SVINFO Satellite Information"},
        # en/dis able NAV-TIMEBDS message
        "NAV-TIMEBDS": {"ablecmd": send_able, "mid": NAV_TIMEBDS,
                        "pollcmd": send_poll,
                        "help": "NAV-TIMEBDS BDS time message"},
        # en/dis able NAV-TIMEGAL message
        "NAV-TIMEGAL": {"ablecmd": send_able, "mid": NAV_TIMEGAL,
                        "pollcmd": send_poll,
                        "help": "NAV-TIMEGAL GAL time message"},
        # en/dis able NAV-TIMEGLO message
        "NAV-TIMEGLO": {"ablecmd": send_able, "mid": NAV_TIMEGLO,
                        "pollcmd": send_poll,
                        "help": "NAV-TIMEGLO GLO time message"},
        # en/dis able NAV-TIMEGPS message
        "NAV-TIMEGPS": {"ablecmd": send_able, "mid": NAV_TIMEGPS,
                        "pollcmd": send_poll,
                        "help": "NAV-TIMEGPS GPS time message"},
        # en/dis able NAV-TIMELS message
        "NAV-TIMELS": {"ablecmd": send_able, "mid": NAV_TIMELS,
                       "pollcmd": send_poll,
                       "help": "NAV-TIMELS Leap Second message"},
        # en/dis able NAV-TIMENAVIC message
        "NAV-TIMENAVIC": {"ablecmd": send_able, "mid": NAV_TIMENAVIC,
                          "pollcmd": send_poll,
                          "help": "NAV-TIMENAVIC NAVIC time message"},
        # en/dis able NAV-TIMEQZSS message
        "NAV-TIMEQZSS": {"ablecmd": send_able, "mid": NAV_TIMEQZSS,
                         "pollcmd": send_poll,
                         "help": "NAV-TIMEQZSS QZSS time message"},
        # UBX-NAV-TIMETRUSTED, protVer 50, X20
        "NAV-TIMETRUSTED": {"pollcmd": send_poll, "mid": [0x01, 0x64],
                            "help": "poll UBX-NAV-TIMETRUSTED Tuststed Time"},
        # en/dis able NAV-TIMEUTC message
        "NAV-TIMEUTC": {"ablecmd": send_able, "mid": NAV_TIMEUTC,
                        "pollcmd": send_poll,
                        "help": "NAV-TIMEUTC UTC Information message"},
        # UBX-NAV-VELECEF
        "NAV-VELECEF": {"pollcmd": send_poll, "mid": [0x01, 0x11],
                        "help": "poll UBX-NAV-VELECEF ECEF velocity"},
        # UBX-NAV-VELNED
        "NAV-VELNED": {"ablecmd": send_able, "mid": NAV_VELNED,
                       "pollcmd": send_poll,
                       "help": "NAV-VEELNED velocity NED message"},

        # UBX-NAV2-CLOCK
        "NAV2-CLOCK": {"pollcmd": send_poll, "mid": [0x29, 0x22],
                       "help": "poll UBX-NAV2-CLOCK Clock Solution"},
        # UBX-NAV2-COV
        "NAV2-COV": {"pollcmd": send_poll, "mid": [0x29, 0x16],
                     "help": "poll UBX-NAV2-COV Covariance Matrices"},
        # UBX-NAV2-DOP
        "NAV2-DOP": {"pollcmd": send_poll, "mid": [0x29, 0x04],
                     "help": "poll UBX-NAV2-DOP Dilution of Precision"},
        # UBX-NAV2-POSECEF
        "NAV2-POSECEF": {"pollcmd": send_poll, "mid": [0x29, 0x01],
                         "help": "poll UBX-NAV2-POSECEF ECEF position"},
        # UBX-NAV2-POSLLH
        "NAV2-POSLLH": {"pollcmd": send_poll, "mid": [0x29, 0x02],
                        "help": "poll UBX-NAV2-POSLLH LLH position"},
        # UBX-NAV2-PVT
        "NAV2-PVT": {"pollcmd": send_poll, "mid": [0x29, 0x07],
                     "help": "poll UBX-NAV2-PVT Navigation Position Velocity "
                     "Time Solution"},
        # UBX-NAV2-SAT
        "NAV2-SAT": {"pollcmd": send_poll, "mid": [0x29, 0x35],
                     "help": "poll UBX-NAV2-SAT Satellite Information"},
        # UBX-NAV2-SBAS
        "NAV2-SBAS": {"pollcmd": send_poll, "mid": [0x29, 0x32],
                      "help": "poll UBX-NAV2-SBAS SBAS Status Data"},
        # UBX-NAV2-SIG
        "NAV2-SIG": {"pollcmd": send_poll, "mid": [0x29, 0x43],
                     "help": "poll UBX-NAV2-SIG Signal Information"},
        # UBX-NAV2-SLAS
        "NAV2-SLAS": {"pollcmd": send_poll, "mid": [0x29, 0x42],
                      "help": "poll UBX-NAV2-SLAS QZSS SLAS info"},
        # UBX-NAV2-STATUS
        "NAV2-STATUS": {"pollcmd": send_poll, "mid": [0x29, 0x03],
                        "help": "poll UBX-NAV2-STATUS Receiver Nav Status"},
        # UBX-NAV2-TIMEBDS
        "NAV2-TIMEBDS": {"pollcmd": send_poll, "mid": [0x29, 0x24],
                         "help": "poll UBX-NAV2-TIMEBDS BDS Time Solution"},
        # UBX-NAV2-TIMEGAL
        "NAV2-TIMEGAL": {"pollcmd": send_poll, "mid": [0x29, 0x25],
                         "help": "poll UBX-NAV2-TIMEGAL "
                         "Galileo Time Solution"},
        # UBX-NAV2-TIMEGLO
        "NAV2-TIMEGLO": {"pollcmd": send_poll, "mid": [0x29, 0x23],
                         "help": "poll UBX-NAV2-TIMEGLO GLO Time Solution"},
        # UBX-NAV2-TIMEGPS
        "NAV2-TIMEGPS": {"pollcmd": send_poll, "mid": [0x29, 0x20],
                         "help": "poll UBX-NAV2-TIMEGPS GPS Time Solution"},
        # UBX-NAV2-TIMELS
        "NAV2-TIMELS": {"pollcmd": send_poll, "mid": [0x29, 0x26],
                        "help": "poll UBX-NAV2-TIMELS Leap Second Info"},
        # UBX-NAV2-TIMEQZSS
        "NAV2-TIMEQZSS": {"pollcmd": send_poll, "mid": [0x29, 0x27],
                          "help": "poll UBX-NAV2-TIMEQZSS Time Info"},
        # UBX-NAV2-TIMEUTC
        "NAV2-TIMEUTC": {"pollcmd": send_poll, "mid": [0x29, 0x21],
                         "help": "poll UBX-NAV2-TIMEUTC UTC Time Solution"},
        # UBX-NAV2-VELECEF
        "NAV2-VELECEF": {"pollcmd": send_poll, "mid": [0x29, 0x11],
                         "help": "poll UBX-NAV2-VELECEF ECEF velocity"},
        # UBX-NAV2-VELNED
        "NAV2-VELNED": {"pollcmd": send_poll, "mid": [0x29, 0x12],
                        "help": "poll UBX-NAV2-VELNED NED velocity"},

        # en/dis able NED
        "NED": {"ablecmd": send_able_ned,
                "help": "NAV-VELNED and NAV-RELPOSNED"},
        # en/dis able basic NMEA messages
        "NMEA": {"ablecmd": send_able_nmea,
                 "help": "basic NMEA messages"},
        # en/dis able RAW/RAWX
        "RAWX": {"ablecmd": send_able_rawx,
                 "help": "RAW/RAWX measurements"},
        # UBX-CFG-CFG
        "RESET": {"pollcmd": send_cfg_cfg,
                  "help": "UBX-CFG-CFG reset config to defaults",
                  "opt": 1},
        # en/dis able RTCM3 messages 1005, 1077, 1087, 1230
        "RTCM3": {"ablecmd": send_able_rtcm3,
                  "help": "required RTCM3 messages. USB port only"},
        # UBX-RXM-IMES
        "RXM-IMES": {"pollcmd": send_poll, "mid": [0x02, 0x61],
                     "help": "poll UBX-RXM-IMES Indoor Messaging System "
                     "Information"},
        # UBX-RXM-MEASX
        "RXM-MEASX": {"pollcmd": send_poll, "mid": [0x02, 0x14],
                      "ablecmd": send_able,
                      "help": "UBX-RXM-MEASX Satellite Measurements "
                      " for RRLP"},
        # UBX-RXM-RAWX
        "RXM-RAWX": {"pollcmd": send_poll, "mid": [0x02, 0x15],
                     "ablecmd": send_able,
                     "help": "UBX-RXM-RAWX raw measurement data"},
        # UBX-RXM-SPARTN, protVer 27.50, F9P
        # can't poll
        "RXM-SPARTN": {"ablecmd": send_able, "mid": [0x02, 0x33],
                      "help": "UBX-RXM-SPARTN SPARTN status"},
        # UBX-RXM-SPARTNKEY, protVer 27.50, F9P
        "RXM-SPARTNKEY": {"pollcmd": send_poll, "mid": [0x02, 0x36],
                          "help": "UBX-RXM-SPARTNKEY get SPARTNKEY"},

        # en/dis able PPS
        "PPS": {"ablecmd": send_able_pps,
                "help": "PPS on TIMPULSE"},
        # UBX-CFG-CFG
        "SAVE": {"pollcmd": send_cfg_cfg,
                 "help": "UBX-CFG-CFG save current config",
                 "opt": 0},
        # en/dis able SBAS
        "SBAS": {"ablecmd": send_able_sbas,
                 "help": "SBAS L1C"},
        # UBX-SEC-OSNMA
        "SEC-OSNMA": {"pollcmd": send_poll, "mid": [0x27, 0x0a],
                      "help": "poll UBX-SEC-OSNMA GAL OSNMA info"},
        # UBX-SEC-SIG
        "SEC-SIG": {"pollcmd": send_poll, "mid": [0x27, 0x09],
                    "help": "poll UBX-SEC-SIG Signal security info"},
        # UBX-SEC-SIGLOG
        "SEC-SIGLOG": {"pollcmd": send_poll, "mid": [0x27, 0x10],
                       "help": "poll UBX-SEC-SIGLOG Signal security log"},
        # UBX-SEC-UNIQID
        "SEC-UNIQID": {"pollcmd": send_poll, "mid": [0x27, 0x03],
                       "help": "poll UBX-SEC-UNIQID Unique chip ID"},
        # en/dis able SFRB/SFRBX
        "SFRBX": {"ablecmd": send_able_sfrbx,
                  "help": "SFRB/SFRBX subframes"},
        # STATUS
        "STATUS": {"pollcmd": get_status,
                   "help": "Get a lot of receiver status"},
        # en/dis able TMODE2 Survey-in
        "SURVEYIN": {"ablecmd": send_able_tmode2,
                     "help": "Survey-in mode with TMODE2.\n"
                             "                    "
                             " SURVEYIN2[,svinMinDur[,svinAccLimit]]\n"
                             "                    "
                             "Default svinMinDur 300 seconds\n"
                             "                    "
                             "Default svinAccLimit 50000",
                     "args": 1},
        # en/dis able TMODE3 Survey-in
        "SURVEYIN3": {"ablecmd": send_able_tmode3,
                      "help": "Survey-in mode with TMODE3.\n"
                              "                    "
                              " SURVEYIN3[,svinMinDur[,svinAccLimit]]\n"
                              "                    "
                              "Default svinMinDur 300 seconds\n"
                              "                    "
                              "Default svinAccLimit 500000",
                      "args": 1},
        # UBX-TIM-SVIN
        "TIM-SVIN": {"pollcmd": send_poll, "mid": [0x0d, 0x04],
                     "help": "poll UBX-TIM-SVIN survey in data"},
        # UBX-TIM-TM2
        "TIM-TM2": {"pollcmd": send_poll, "mid": [0x0d, 0x03],
                    "help": "poll UBX-TIM-TM2 time mark data"},
        # UBX-TIM-TP
        "TIM-TP": {"ablecmd": send_able, "mid": [0x06, 0x01],
                   "pollcmd": send_poll,
                   "help": "TIM-TP Time Pulse message"},
        # UBX-TIM-VRFY
        "TIM-VRFY": {"pollcmd": send_poll, "mid": [0x0d, 0x06],
                     "help": "poll UBX-TIM-VRFY Sourced Time Verification"},
        # en/dis able all NAV-TIME* messages
        "TIME": {"ablecmd": send_able_time,
                 "help": "All NAV-TIME* messages"},
        # UBX-UPD-SOS
        "UPD-SOS": {"pollcmd": send_poll, "mid": [0x09, 0x14],
                    "help": "poll UBX-UPD-SOS Backup File restore Status"},
        # UBX-UPD-SOS
        "UPD-SOS0": {"pollcmd": send_poll, "opt": [0x09, 0x14, 0, 0, 0, 0],
                     "help": "UBX-UPD-SOS Create Backup File in Flash"},
        # UBX-UPD-SOS
        "UPD-SOS1": {"pollcmd": send_poll, "opt": [0x09, 0x14, 1, 0, 0, 0],
                     "help": "UBX-UPD-SOS Create Clear File in Flash"},
        # UBX-CFG-RST
        "WARMBOOT": {"pollcmd": send_cfg_rst,
                     "help": "UBX-CFG-RST warmboot the GPS",
                     "opt": 1},
    }
    # end class ubx
