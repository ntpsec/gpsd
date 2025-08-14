/* gps.h -- interface of the libgps library
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#ifndef _GPSD_GPS_H_
#define _GPSD_GPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>   // stdint.h would be smaller but not all have it
#include <limits.h>
#include <pthread.h>    // pacifies OpenBSD's compiler
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>   // for struct timespec
#include <sys/types.h>
#include <time.h>

/*
 * 4.1 - Base version for initial JSON protocol (Dec 2009, release 2.90)
 * 4.2 - AIS application IDs split into DAC and FID (July 2010, release 2.95)
 * 5.0 - MAXCHANNELS bumped from 20 to 32 for GLONASS (Mar 2011, release 2.96)
 *       gps_open() becomes reentrant, what gps_open_r() used to be.
 *       gps_poll() removed in favor of gps_read().  The raw hook is gone.
 *       (Aug 2011, release 3.0)
 * 5.1 - GPS_PATH_MAX uses system PATH_MAX; split24 flag added. New
 *       model and serial members in part B of AIS type 24, conforming
 *       with ITU-R 1371-4. New timedrift structure (Nov 2013, release 3.10).
 * 6.0 - AIS type 6 and 8 get 'structured' flag; GPS_PATH_MAX
 *       shortened because devices has moved out of the tail union. Sentence
 *       tag fields dropped from emitted JSON. The shape of the skyview
 *       structure has changed to make working with the satellites-used
 *       bits less confusing. (January 2015, release 3.12).
 * 6.1 - Add navdata_t for more (nmea2000) info.
 * 7.0 - add gps_fix_t.ecef (February 2018)
 *       changed prototype of gps_read() to add buffer parameters
 *       increased length of devconfig_t.subtype
 *       add gnssid:svid:sigid to satellite_t
 *       add mtime to attitude_t
 *       changed MAXCHANNELS
 * 8.0 - Change shape of rawdata_t.
 *       Added values for gps_data_t->status
 *       Remove epe from gps_data_t, it duplicates gps_fix_t eph
 *       Added sep (estimated spherical error, 3D)
 *       Note: Some GPS call eph as epe, others call sep as epe
 *       Add gps_fix_t datum string, and qErr
 *       enlarge subtype to hold ZED-F9 string
 *       MAXCHANNELS bumped from 120 to 140
 *       Try to make PRN be NMEA 2.x-4.0 compliant, not 4.10 or u-blox
 * 9.0   add NED and geoid_sep variables to gps_fix_t
 *       add health variable to satellite_t
 *       change satellite_t elevation and azimuth to double
 *       satellite_t elevation, azimuth, and ss use NAN for unknown value.
 *       add altMSL, and depth, to gps_fix_t
 *       add altHAE to gps_fix_t
 *       mark altitude in gps_fix_t as deprecated and undefined
 *       Move mag_var from gps_device_t to magnetic_var gps_data_t.
 *       add dgps_age and dgps_station, to gps_fix_t
 *       Change gps_fix_t.time from timestamp_t to timespec_t
 *       Change gps_data_t.skyview_time from timestamp_t to timespec_t
 *       Change gps_data_t.online from timestamp_t to timespec_t
 *       Change gpst_t.utctime from double to timespec_t
 *       Change devices.time from timestamp_t to timespec_t
 *       Change sub4_18.d_tot from timestamp_t to time_t t_tot
 *       Change devconfig_t.activated, cycle & mincycle to timespec_t
 *       Remove unused timestamp() and unix_to_iso8601().
 *       Remove unused iso8601_to_unix().
 *       Remove unused struct timestamp_t entirely
 *       Add DEG_NORM()
 *       Move toff and pps out of gps_data_t.union.
 *       Move gps_fix_t.qErr to gps_data_t.
 *       Split devconfig_t.subtype into subtype and subtype1
 * 9.1   Add leap_seconds to gps_data_t
 *       Fix rtcm3_1029_t.text length
 *       Add/change many rtcm2 structs
 *       Add/change many rtcm3 structs
 * 10    Move gps_data_t->status to gps_fix_t.status for better fix merging
 *       Add wspeedt, wspeedr, wanglem, wanglet, wangler to  gps_fix_t
 *       Remove unused gps_data_t.navadata_t.
 *       Add relPosL and relPosH to gps_fix_t.NED
 * 11    long l_toa becomes unsigned long l_toa
 *       fix sub4_18 types.
 * 12    subframe_t expanded for more gnssId's, WN, etc.
 *       Add orbit_t for generic orbital parameters
 *       Add subframe.orbit and subframe.orbit1 to store orbit_t's
 *       Add gyro_temp, gyro_z, msg, timeTag, to attitude_t
 *       add imu[], and matching IMU_SET flag
 *       move attitude out of the union, to stop conflicts.
 * 13    Add rtcm3_msm
 *       fix tow in never used struct rtcm3_1015_t
 *       remove never used struct rtcm3_1016_t and struct rtcm3_1017_t
 *       add struct baseline_t
 * 14    Move three visibilize() into one gps_visibilize() here.
 *       Move gpsd_hexpack() to here as gps_hexpack()
 *       Move gpsd_hexdump() to here as gps_hexdump()
 *       Add prRes, prRate, pr, and qualityInd to satellite_t
 *       Add fixsource_t, watch_t, set_pending to gps_data_t
 *       move privdata_t here from libgps/gps_sock.c
 *       Add rot (Rate Of Turn), mheading, to struct attitude_t
 *       Add wtemp, temp, ant_stat, jam, clockbias and clockddrift to gps_fix_t
 *       Add rtcm3_4076_hdr
 *       change gps_data_t.gps_fd to type gps_fd_t.
 *       devconfig_t add sernum[]
 *       Add val2sstr() and flags2str()
 *       Add ve_err_deviation, vn_err_deviationv, vu_err_deviation to gst_t
 *       Move gst_t out of gps_data_t union.
 *       Add ROWS(), IN() macrosa
 *       MAXCHANNELS bumped from 140 to 185, for ZED-F9T
 * 15    Improve NMEA V4.10, and above, sigid decoding
 *       Add Teseo LIV4F antenna status and firmware version management
 *       Add sigid2str(), convert sigid to a string.
 *       Add sigid2obs(), convert sigid to a RINEX observation code string.
 */
#define GPSD_API_MAJOR_VERSION  14      // bump on incompatible changes
#define GPSD_API_MINOR_VERSION  0       // bump on compatible changes

#define MAXCHANNELS     184     // u-blox 9 tracks 140 signals
#define MAXUSERDEVS     4       // max devices per user
#define GPS_PATH_MAX    128     // for names like /dev/serial/by-id/...

#define GPS_JSON_COMMAND_MAX    80
// u-blox 9 can make really long JSON in "RAW" messages
#define GPS_JSON_RESPONSE_MAX   10240

// number of rows in an array.
#define ROWS(a) (sizeof(a) / sizeof(a[0]))

// IN(v, min, max), True if vl in range min to max, inclusive
#define IN(min, val, max) (((min) <= (val)) && ((val) <= (max)))

// normalize degrees to 0 to 359
#define DEG_NORM(deg) \
    if (0 > (deg)) {(deg) += 360;} else if (360 <= (deg)) {(deg) -= 360;};

/*
 * The structure describing an uncertainty volume in kinematic space.
 * This is what GPSes are meant to produce; all the other info is
 * technical impedimenta.
 *
 * All the information in this structure was considered "valid"
 * by the GPS at the time of update.  But "valid" doubles are likely
 * the same as NaN. Check "gps_mask_t set" before using integers and flags.
 *
 * Error estimates are at 95% confidence.  Except when they are not.
 *
 * WARNING!  loss of precision telling time as a double.
 * UNIX time to nanoSec precision is 62 significant bits
 * UNIX time to nanoSec precision after 2038 is 63 bits
 * That is why gpsd uses struct timespec.
 *
 * For more info:  https://gpsd.io/gpsd-numbers-matter.html
 *
 * The u-blox ZED-F9P reports 0.1 mm, and 1e-9 (0.000000001) degree,
 * precision.  That is about 12 decimal digits of precision.
 * It is certainly not that accurate, maybe soon.
 *
 * this easily fits in a C double which has 15.95 digits of precision
 * printf() format %f defaults to %.6f, which will truncate the result.
 * so print with %.7f, or even %9f, if you have a survey grade GPS.
 *
 * For more info:  https://gpsd.io/gpsd-numbers-matter.html
 *
 * All double values use NAN to indicate data not available.
 * WARNING: Check all floats and doubles with isfinite() before using them!
 * isnan() is not sufficient.
 *
 * For more info:  https://gps.io/gpsd-numbers-matter.html
 */

typedef struct timespec timespec_t;     // Unix time as sec, nsec

/* Baseline data is here.  Some receivers, like the Skytraq
 * PX1172RH_DS, report two baselines.  One from a fixed
 * (surveyed in) base to the mving base, and one from the moving
 * base to the moving rover.
 */
struct baseline_t {
    /* status, aka mode, valid values:
     * STATUS_UNK, STATUS_RTK_FIX, STATUS_RTK_FLT
     */
    int status;
    double east;        // East projection of baseline, meters
    double north;       // North projection of baseline, meters
    double up;          // Up projection of baseline, meters
    double length;      // length, meters.
    double course;      // course, degrees
    double ratio;       // RTK AR Ratio
};

/* GPS error estimates are all over the map, and often unspecified.
 * try for 1-sigma if we can... */
struct gps_fix_t {
    timespec_t time;            // Time of update
    int    mode;                // Mode of fix
#define MODE_NOT_SEEN   0       // mode update not seen yet
#define MODE_NO_FIX     1       // none
#define MODE_2D         2       // good for latitude/longitude
#define MODE_3D         3       // good for altitude/climb too

    /* GPS status, aka fix type, is a modifier (adjective) to
     * gps_data.mode.  It is not a replacement for, or superset of, mode.
     * It is almost, but not quite, the same as the NMEA 4.x xxGGA GPS
     * Quality Indicator Values.  Many GNSS receivers do not supply it.
     */
    int    status;              // What kind of fix?
#define STATUS_UNK      0       // Unknown status
// plain GPS (SPS Mode), without DGPS, PPS, RTK, DR, etc.
#define STATUS_GPS      1
#define STATUS_DGPS     2       // with DGPS
#define STATUS_RTK_FIX  3       // with RTK Fixed
#define STATUS_RTK_FLT  4       // with RTK Float
#define STATUS_DR       5       // with dead reckoning
#define STATUS_GNSSDR   6       // with GNSS + dead reckoning
#define STATUS_TIME     7       // time only (surveyed in, manual)
// Note that STATUS_SIM and MODE_NO_FIX can go together.
#define STATUS_SIM      8       // simulated
/* yes, Precise Positioning Service (PPS)
 * Not to be confused with Pulse per Second (PPS)
 * PPS is the encrypted military P(Y)-code */
#define STATUS_PPS_FIX  9

    double ept;         // Expected time uncertainty, seconds
    double latitude;    // Latitude in degrees (valid if mode >= 2)
    double epy;         // Latitude position uncertainty, meters
    double longitude;   // Longitude in degrees (valid if mode >= 2)
    double epx;         // Longitude position uncertainty, meters
    double altitude;    // DEPRECATED, undefined.
    double altHAE;      /* Altitude, height above ellipsoid.
                         * in meters and probably WGS84
                         * (valid if mode == 3)
                         * MSL = altHAE - geoid_sep */
    double altMSL;      // Altitude MSL (maybe EGM2008)
    double epv;         // Vertical position uncertainty, meters
    double track;       // Course made good (relative to true north)
    double epd;         // Track uncertainty, degrees
    double speed;       // Speed over ground, meters/sec
    double eps;         // Speed uncertainty, meters/sec
    double climb;       // Vertical speed, meters/sec
    double epc;         // Vertical speed uncertainty
    // estimated position error horizontal (2D). Meters, maybe 50%, maybe 95%
    // aka estimated position error (epe)
    double eph;         // estimated position error horizontal (2D)
    // spherical error probability, 3D. Meters, maybe 50%, maybe 95%
    // Garmin, not gpsd, calls this estimated position error (epe)
    double sep;
    /* Geoid separation (ellipsoid separation)
     * Height of MSL ellipsoid (geoid) above WGS84 ellipsoid.
     * Positive is MSL above WGS84. In meters */
    double geoid_sep;

    double magnetic_track;  // Course (relative to Magnetic North)
    double magnetic_var;    // magnetic variation in degrees
    // depth in meters, probably depth of water under the keel
    double depth;
    double wtemp;           // water temp, degrees C
    double temp;            // receiver temp, degrees C
    int ant_stat;           // antenna status
#define ANT_UNK 0
#define ANT_OK 1
#define ANT_OPEN 2
#define ANT_SHORT 3
    /* jamming indicator, 0 == None to 255 == real bad
     * Some receivers on report No/Yes, or 0 to 3.  Those are mapped to
     * 0 to 255. -1 is unset */
    int jam;
    // If clockbias and clockrate are zero, ignore them.
    long clockbias;         // clock bias in ns
    long clockdrift;        // clock drift in ns/s

    // ECEF data, all data in meters, and meters/second, or NaN
    struct {
        double x, y, z;         // ECEF x, y, z
        double pAcc;            // 3D Position Accuracy Estimate, likely SEP
        double vx, vy, vz;      // ECEF x, y, z velocity
        double vAcc;            // Velocity Accuracy Estimate, probably SEP
    } ecef;
    // NED data, all data in meters, and meters/second, or NaN
    struct {
        double relPosN, relPosE, relPosD;   // NED relative positions
        double relPosL, relPosH;            // relative length and heading
        double velN, velE, velD;            // NED velocities
    } NED;
    char datum[40];             // map datum
    // DGPS stuff, often from xxGGA, or xxGNS
    // SiRF II is %.1f, Skytraq is %,4fm others seem to be int
    double dgps_age;       // age of DGPS data in seconds, -1 invalid
    /* DGPS Station used, max size is a guess
     * NMEA 2 says 0000-1023
     * RTCM 3, station ID is 0 to 4095.
     * u-blox UBX-NAV-DGPS is 16 bit integer */
    int dgps_station;           // DGPS station ID, -1 invalid
    double wanglem;             // Wind angle, magnetic, m/s
    double wangler;             // Wind angle, relative, m/s
    double wanglet;             // Wind angle, true, m/s
    double wspeedr;             // Wind speed, relative, m/s
    double wspeedt;             // Wind speed, true, m/s
    struct baseline_t base;     // baseline from fixed base
};

/* Some GNSS receivers, like u-blox 8, can log fixes for later use.
 * That data goes in gps_log_t
 * Check all doubles with isfinite() before using
 */
struct gps_log_t  {
    double lon;           // degrees
    double lat;           // degrees
    double altHAE;        // Height above Ellipsoid, meters
    double altMSL;        // Mean Sea Level, meters
    double gSpeed;        // Ground Speed,  meters per second
    double heading;       // true heading, degrees

    /* The accuracy estimates are near useless as they are not defined
     * statistically as CEP(50), one sigma, two sigma, etc.
     */
    double tAcc;          // Time accuracy estimate, nano seconds
    double hAcc;          // Horizontal accuracy estimate, meters
    double vAcc;          // Vertical accuracy estimate, meters
    double sAcc;          // Speed accuracy estimate, meters per second
    double headAcc;       // Heading accuracy estimate, degrees

    double velN;          // NED north velocity, meters per second
    double velE;          // NED east velocity, meters per second
    double velD;          // NED down velocity, meters per second
    double pDOP;          // Position DOP

    // Odometer data
    double distance;      // Ground distance since last reset, meters
    double totalDistance; // Total cumulative distance, meters
    double distanceStd;   // Ground distance accuracy (1-sigma)

    timespec_t then;      // time of log entry, zero if invalid
    // GPS status -- always valid, same values as gps_fix_t.status
    int    status;        // type of fix?

    uint32_t index_cnt;   // message counter

    char fixType;         // -1 = unset, 0 = None, 2 == 2D, 3 = 3D

    // Number of satellites used in Nav Solution, zero if invalid
    unsigned char numSV;
    char string[257];     // 256 max plus NUL
};

/*
 * Other GNSS birds reuse GPS PRNs.
 * It is an NMEA0183 convention to map them to pseudo-PRNs 65..437.
 * Very dependent on NMEA version.
 * (some other GPS receivers push them to 33 and above).
 */
#define GLONASS_PRN_OFFSET      64

/*
 * The structure describing the pseudorange errors (GPGST, etc.)
 * These are all 1 standard deviation errors.
 */
struct gst_t {
    timespec_t utctime;
    double rms_deviation;
    double smajor_deviation;
    double sminor_deviation;
    double smajor_orientation;
    double lat_err_deviation;          // latitude, meters
    double lon_err_deviation;          // longitude meters
    double alt_err_deviation;          // altitude, meters
    double ve_err_deviation;           // north velocity,  meters/sec
    double vn_err_deviation;           // veast elocity,  meters/sec
    double vu_err_deviation;           // up velocity,  meters/sec
};

/*
 * From the RCTM104 2.x standard:
 *
 * "The 30 bit words (as opposed to 32 bit words) coupled with a 50 Hz
 * transmission rate provides a convenient timing capability where the
 * times of word boundaries are a rational multiple of 0.6 seconds."
 *
 * "Each frame is N+2 words long, where N is the number of message data
 * words. For example, a filler message (type 6 or 34) with no message
 * data will have N=0, and will consist only of two header words. The
 * maximum number of data words allowed by the format is 31, so that
 * the longest possible message will have a total of 33 words."
 */
#define RTCM2_WORDS_MAX 33
#define MAXCORRECTIONS  18      // max correction count in type 1 or 9
#define MAXSTATIONS     10      // maximum stations in almanac, type 5
// RTCM104 doesn't specify this, so give it the largest reasonable value
#define MAXHEALTH       (RTCM2_WORDS_MAX-2)

/*
 * A nominally 30-bit word (24 bits of data, 6 bits of parity)
 * used both in the GPS downlink protocol described in IS-GPS-200
 * and in the format for DGPS corrections used in RTCM-104v2.
 */
typedef uint32_t isgps30bits_t;

/*
 * Values for "system" fields.  Note, the encoding logic is sensitive to the
 * actual values of these; it's not sufficient that they're distinct.
 */
#define NAVSYSTEM_GPS           0
#define NAVSYSTEM_GLONASS       1
#define NAVSYSTEM_GALILEO       2
#define NAVSYSTEM_UNKNOWN       3

struct rtcm2_t {
    // header contents
    unsigned type;      // RTCM message type
    unsigned length;    // payload length (words) not including 2 word header
    double   zcount;    // time within hour: GPS time, no leap secs
    unsigned refstaid;  // reference station ID
    unsigned seqnum;    // message sequence number (modulo 8)
    unsigned stathlth;  // station health

    // message data in decoded form

    // Reference Station data for type 3/4/22/23/24/32 messages
    struct {
        bool valid;            // is message well-formed?
        double x, y, z;        // reference station position ECEF, meters
        double dx, dy, dz;     // delta reference station position ECEF, meters
        double ah;             // antenna height (above RP) meters
        double dx2, dy2, dz2;  // L2 delta ref station position ECEF, meters
        unsigned char gs;      // 0 == GPS, 1 == GLONASS
        char ant_desc[33];     // antenna descriptor
        char ant_serial[33];   // antenna serial number
        char ar;               // 1 == ARP (Type 24) will follow
        unsigned char setup_id;  // 0 == use standard IGS model
    } ref_sta;

    // RTK corrections, type 18/19/20/21
    struct {
        unsigned tom;          // GNSS Time of Measurement, 0-5999999 micro sec
        unsigned char f;       // 00 = L1, 10 = L2, 01 and 11 reserved
        unsigned char sm;      // smoothing interval
        unsigned int nentries;
        struct rtk_sat_t {
            unsigned char m;       // multiple message indicator
            unsigned char pc;      // 0 == C/A code, 1 == P-Code
            unsigned char g;       // 0 == GPS, 1 == GLONASS
            // satellite id (1 - 32), GPS PRN, or GLONASS clot number
            unsigned char ident;
            unsigned char dq;      // data quality
            unsigned char clc;     // cumulative loss of continuity
            unsigned char iod;     // Issue of Data (GPS)/Time of Day (GLO)
            unsigned char me;      // multipath error
            int carrier_phase;     // signed 32 bits, 1/256 cycle
            int pseudorange;       // signed 32 bits, 0.02 m
            unsigned char rrc;     // range rate correction
        } sat[15];
    } rtk;

    union {
        struct {
            unsigned int nentries;
            struct gps_rangesat_t {     // data from messages 1 & 9
                unsigned ident;         // satellite ID
                unsigned udre;          // user differential range error
                unsigned iod;           // issue of data
                double prc;             // range error
                double rrc;             // range error rate
            } sat[MAXCORRECTIONS];
        } gps_ranges;
        struct {                // data from type 4 messages
            bool valid;         // is message well-formed?
            int system;
            int sense;
#define SENSE_INVALID   0
#define SENSE_GLOBAL    1
#define SENSE_LOCAL     2
            char datum[6];
            double dx, dy, dz;
        } reference;
        struct {                // data from type 5 messages
            unsigned int nentries;
            struct consat_t {
                unsigned ident;         // satellite ID
                bool iodl;              // issue of data
                unsigned int health;    // is satellite healthy?
#define HEALTH_NORMAL           (0)     // Radiobeacon operation normal
#define HEALTH_UNMONITORED      (1)     // No integrity monitor operating
#define HEALTH_NOINFO           (2)     // No information available
#define HEALTH_DONOTUSE         (3)     // Do not use this radiobeacon
               int snr;                 // signal-to-noise ratio, dB
#define SNR_BAD -1                      // not reported
                bool health_en;         // health enabled
                bool new_data;          // new data?
                bool los_warning;       // line-of-sight warning
                unsigned int tou;       // time to unhealth, seconds
            } sat[MAXHEALTH];
        } conhealth;
        struct {                // data from type 7 messages
            unsigned int nentries;
            struct station_t {
                double latitude, longitude;     // location
                unsigned int range;             // range in km
                double frequency;               // broadcast freq
                unsigned int health;            // station health
                unsigned int station_id;        // of the transmitter
                unsigned int bitrate;           // of station transmissions
            } station[MAXSTATIONS];
        } almanac;
        struct {                        // data for type 13 messages
            bool status;                // expect a text message
            bool rangeflag;             // station range altered?
            double lat, lon;            // station longitude/latitude
            unsigned int range;         // transmission range in km
        } xmitter;
        struct {                        // data from type 14 messages
            unsigned int week;          // GPS week (0-1023)
            unsigned int hour;          // Hour in week (0-167)
            unsigned int leapsecs;      // Leap seconds (0-63)
        } gpstime;
        struct {                        // RTCM2 type 18
            int tom;                    // time of measurement
            // FIXME: add in sat words
        } rtcm2_18;
        struct {                        // RTCM2 type 19
            int tom;                    // time of measurement
            // FIXME: add in sat words
        } rtcm2_19;
        struct {                        // RTCM2 type 20
            int tom;                    // time of measurement
            // FIXME: add in sat words
        } rtcm2_20;
        struct {                        // RTCM2 type 21
            int tom;                    // time of measurement
            // FIXME: add in sat words
        } rtcm2_21;
        struct {                        // RTCM2 type 22
            unsigned char ecef_dx;      // L1 ECEF delta-x
            unsigned char ecef_dy;      // L1 ECEF delta-y
            unsigned char ecef_dz;      // L1 ECEF delta-z
            unsigned char gs;           // 0 is GPS, 1 is GLONASS
            unsigned char ah_flag;      // 1 if ah is valid
            int ah;                     // Ant Height meters
            unsigned char l2ecef_dx;    // L2 ECEF delta-x
            unsigned char l2ecef_dy;    // L2 ECEF delta-y
            unsigned char l2ecef_dz;    // L2 ECEF delta-z
        } rtcm2_22;
        struct {                        // RTCM2 type 24
            int64_t ecef_x;             // ECEF X (m * 10e-4)
            int64_t ecef_y;             // ECEF Y (m * 10e-4)
            int64_t ecef_z;             // ECEF Z (m * 10e-4)
            unsigned char gs;           // 0 is GPS, 1 is GLONASS
            unsigned char ah_flag;      // 1 if ah is valid
            int ah;                     // Ant Height meters
        } rtcm2_24;
        struct {
            unsigned int nentries;
            struct glonass_rangesat_t {      // data from message type 31
                unsigned ident;              // satellite ID
                unsigned udre;               // user differential range error
                unsigned tod;                // issue of data
                bool change;                 // ephemeris change bit
                double prc;                  // range error
                double rrc;                  // range error rate
            } sat[MAXCORRECTIONS];
        } glonass_ranges;
        // data from type 16 messages
        char message[(RTCM2_WORDS_MAX - 2) * sizeof(isgps30bits_t)];
        // data from messages of unknown type, not including header
        isgps30bits_t   words[RTCM2_WORDS_MAX - 2];
    };
};

// RTCM3 report structures begin here

#define RTCM3_MAX_SATELLITES    64
#define RTCM3_MAX_DESCRIPTOR    31
#define RTCM3_MAX_ANNOUNCEMENTS 32
#define RTCM3_GRID_SIZE         16              // RTCM3_1023
#define RTCM3_DF148_SIZE        10

enum RTCM3_QUALITY_INDICATOR_TRANSFORMATION
{
    TR_UNKNOWN,
    TR_BETTER_0021,
    TR_BETTER_0050,
    TR_BETTER_0200,
    TR_BETTER_0500,
    TR_BETTER_2000,
    TR_BETTER_5000,
    TR_WORSE_5001
};                                              // DF214, DF215

enum RTCM3_QUALITY_INDICATOR_GRID_RESIDUALS
{
    GR_UNKNOWN,
    GR_BETTER_010,
    GR_BETTER_020,
    GR_BETTER_050,
    GR_BETTER_100,
    GR_BETTER_200,
    GR_BETTER_500,
    GR_WORSE_501
};                                              // DF216, DF217

enum RTCM3_INTERPOLATION_INDICATOR
{
    INTERP_BI_LINEAR,
    INTERP_BI_QUADRIC,
    INTERP_BI_SPLINE,
    INTERP_RESERVED
};                                              // DF212, DF213

enum RTCM3_PROJECTION_TYPE
{
    PR_UNKNOWN,
    PR_TM,
    PR_TMS,
    PR_LCC1SP,
    PR_LCC2SP,
    PR_LCCW,
    PR_CS
};                                              // DF170

// Header for all RTCM3 4076 messages. IGS SSR
struct rtcm3_4076_hdr {
    unsigned ssr_vers;         // IDF001, IGS SSR message version.
    unsigned igs_num;          // IDF002, IGS message number
    unsigned ssr_epoch;        // IDF003, SSR Epoch Time in seconds
    unsigned ssr_update;       // IDF004, SSR Update Interval
    unsigned ssr_mmi;          // IDF005, SSR Multiple Message Interval
    unsigned ssr_iod;          // IDF007, IOD SSR
    unsigned ssr_provider;     // IDF008, SSR Provider ID
    unsigned ssr_solution;     // IDF009, SSR Solution ID
};

struct rtcm3_basic_rtk {
    unsigned char indicator;    // Indicator
    // Satellite Frequency Channel Number (GLONASS only)
    unsigned int channel;
    double pseudorange;         // Pseudorange
    double rangediff;           // PhaseRange - Pseudorange in meters
    unsigned char locktime;     // Lock time Indicator
};

struct rtcm3_correction_diff {
    unsigned char ident;        // satellite ID
    enum {RESERVED, CORRECT, WIDELANE, UNCERTAIN} ambiguity;
    unsigned char nonsync;
    // Geometric Carrier Phase Correction Difference (1016, 1017)
    double geometric_diff;
    unsigned char iode;         // GPS IODE (1016, 1017)
    // Ionospheric Carrier Phase Correction Difference (1015, 1017)
    double ionospheric_diff;
};

struct rtcm3_extended_rtk {
    unsigned char indicator;    // Indicator
    // Satellite Frequency Channel Number (GLONASS only)
    unsigned int channel;
    double pseudorange;         // Pseudorange
    double rangediff;           // PhaseRange - L1 Pseudorange
    unsigned char locktime;     // Lock time Indicator
    unsigned char ambiguity;    // Integer Pseudorange Modulus Ambiguity
    double CNR;                 // Carrier-to-Noise Ratio
};

// satellite data from MSM1 and MSM7
struct rtcm3_msm_sat {
    unsigned rr_ms;             // Milliseconds in GNSS Satellite rough ranges
    unsigned ext_info;          // Extended Satellite info
    unsigned rr_m1;             // Rough ranges Modulo 1 Milliseconds
    int rates_rphr;             // Rough PhaseRange rates
};

// signal data from MSM1 and MSM7
struct rtcm3_msm_sig {
    int pseudo_r;               // Signal fine Pseudoranges
    int32_t phase_r;            // Signal fine Phaseranges
    unsigned lti;               // Lock Time Indicator
    unsigned cnr;               // Signal CNRs
    int rates_phr;              // Phase Range Rates
    bool half_amb;              // Half-cycle ambiguity indicator
};

// header data from MSM1 to MSM7
// Used for all GNSS, but their timebases differ
struct rtcm3_msm_hdr {
    unsigned station_id;        // Reference Station ID
    time_t tow;                 // GNSS Epoch Time in ms
    bool sync;                  // Synchronous GNSS Message Flag
    unsigned IODS;              // IODS - Issue of Data Station
    unsigned char reserved;     // reserved
    unsigned char steering;     // Clock Steering Indicator
    unsigned char ext_clk;      // External Clock Indicator
    bool smoothing;             // Divergence-free Smoothing Indicator
    unsigned interval;          // Smoothing Interval
    uint64_t sat_mask;          // Satellite Mask
    uint32_t sig_mask;          // Signal Mask
    uint64_t cell_mask;         // Cell Mask
    // not part of the network message:
    unsigned char gnssid;       // gnssid
    unsigned char msm;          // 1 to 7, MSMx
    unsigned char n_sat;        // Number of satellites derived from sat_mask
    unsigned char n_sig;        // Number of signals derived from sig_mask
    unsigned char n_cell;       // no. of sats * no. of sigs (<=64!)
    struct rtcm3_msm_sat sat[RTCM3_MAX_SATELLITES];
    struct rtcm3_msm_sig sig[RTCM3_MAX_SATELLITES];
};

struct rtcm3_network_rtk_header {
    unsigned int network_id;    // Network ID
    unsigned int subnetwork_id; // Subnetwork ID
    unsigned long tow;          // GPS Epoch Time (TOW).  scale 0.1 s
    bool multimesg;             // GPS Multiple Message Indicator
    unsigned master_id;         // Master Reference Station ID
    unsigned aux_id;            // Auxiliary Reference Station ID
    unsigned char satcount;     // count of GPS satellites
};

// Used for both GPS and GLONASS, but their timebases differ
struct rtcm3_rtk_hdr {          // header data from 1001, 1002, 1003, 1004
    unsigned int station_id;    // Reference Station ID, DF003
    unsigned long tow;          // GNSS Epoch Time in ms, DF004
    bool sync;                  // Synchronous GNSS Message Flag, DF005
    unsigned short satcount;    // # Satellite Signals Processed, DF006
    bool smoothing;             // Divergence-free Smoothing Indicator, DF007
    unsigned int interval;      // Smoothing Interval, DF008
};

struct rtcm3_t {
    // header contents
    unsigned type;      // RTCM 3.x message number (type)
    unsigned length;    // payload length, inclusive of checksum

    union {
        // 1001-1013 were present in the 3.0 version
        struct {
            struct rtcm3_rtk_hdr        header;
            struct rtcm3_1001_t {
                unsigned ident;                 // Satellite ID
                struct rtcm3_basic_rtk L1;
            } rtk_data[RTCM3_MAX_SATELLITES];
        } rtcm3_1001;
        struct {
            struct rtcm3_rtk_hdr        header;
            struct rtcm3_1002_t {
                unsigned ident;                 // Satellite ID
                struct rtcm3_extended_rtk L1;
            } rtk_data[RTCM3_MAX_SATELLITES];
        } rtcm3_1002;
        struct rtcm3_1003_t {
            struct rtcm3_rtk_hdr        header;
            struct {
                unsigned ident;                 // Satellite ID
                struct rtcm3_basic_rtk L1;
                struct rtcm3_basic_rtk L2;
            } rtk_data[RTCM3_MAX_SATELLITES];
        } rtcm3_1003;
        struct rtcm3_1004_t {
            struct rtcm3_rtk_hdr        header;
            struct {
                unsigned ident;                 // Satellite ID
                struct rtcm3_extended_rtk L1;
                struct rtcm3_extended_rtk L2;
            } rtk_data[RTCM3_MAX_SATELLITES];
        } rtcm3_1004;
        struct rtcm3_1005_t {
            unsigned int station_id;            // Reference Station ID
            int system;                         // Which system is it?
            bool reference_station;             // Reference-station indicator
            bool single_receiver;               // Single Receiver Oscillator
            double ecef_x, ecef_y, ecef_z;      // ECEF antenna location
        } rtcm3_1005;
        struct rtcm3_1006_t {
            unsigned int station_id;            // Reference Station ID
            int system;                         // Which system is it?
            bool reference_station;             // Reference-station indicator
            bool single_receiver;               // Single Receiver Oscillator
            double ecef_x, ecef_y, ecef_z;      // ECEF antenna location
            double height;                      // Antenna height
        } rtcm3_1006;
        struct {
            unsigned int station_id;                    // Reference Station ID
            char descriptor[RTCM3_MAX_DESCRIPTOR+1];    // Description string
            unsigned int setup_id;
        } rtcm3_1007;
        struct {
            unsigned int station_id;                    // Reference Station ID
            char descriptor[RTCM3_MAX_DESCRIPTOR+1];    // Description string
            unsigned int setup_id;
            char serial[RTCM3_MAX_DESCRIPTOR+1];        // Serial # string
        } rtcm3_1008;
        struct {
            struct rtcm3_rtk_hdr        header;
            struct rtcm3_1009_t {
                unsigned ident;         // Satellite ID
                struct rtcm3_basic_rtk L1;
            } rtk_data[RTCM3_MAX_SATELLITES];
        } rtcm3_1009;
        struct {
            struct rtcm3_rtk_hdr        header;
            struct rtcm3_1010_t {
                unsigned ident;         // Satellite ID
                struct rtcm3_extended_rtk L1;
            } rtk_data[RTCM3_MAX_SATELLITES];
        } rtcm3_1010;
        struct {
            struct rtcm3_rtk_hdr        header;
            struct rtcm3_1011_t {
                unsigned ident;                 // Satellite ID
                struct rtcm3_extended_rtk L1;
                struct rtcm3_extended_rtk L2;
            } rtk_data[RTCM3_MAX_SATELLITES];
        } rtcm3_1011;
        struct {
            struct rtcm3_rtk_hdr        header;
            struct rtcm3_1012_t {
                unsigned ident;                 // Satellite ID
                struct rtcm3_extended_rtk L1;
                struct rtcm3_extended_rtk L2;
            } rtk_data[RTCM3_MAX_SATELLITES];
        } rtcm3_1012;
        struct {
            unsigned int station_id;    // Reference Station ID
            unsigned short mjd;         // Modified Julian Day (MJD) Number
            unsigned int sod;           // Seconds of Day (UTC)
            unsigned char leapsecs;     // Leap Seconds, GPS-UTC
            unsigned char ncount;       // Count of announcements to follow
            struct rtcm3_1013_t {
                unsigned short id;              // message type ID
                bool sync;
                unsigned short interval;        // interval in 0.1sec units
            } announcements[RTCM3_MAX_ANNOUNCEMENTS];
        } rtcm3_1013;
        // 1014-1017 were added in the 3.1 version
        struct rtcm3_1014_t {
            unsigned int network_id;    // Network ID
            unsigned int subnetwork_id; // Subnetwork ID
            unsigned int stationcount;  // # auxiliary stations transmitted
            unsigned int master_id;     // Master Reference Station ID
            unsigned int aux_id;        // Auxiliary Reference Station ID
            double d_lat, d_lon, d_alt; // Aux-master location delta
        } rtcm3_1014;
        struct rtcm3_1015_t {
            // used for 1015, 1016, and 1017
            struct rtcm3_network_rtk_header     header;
            struct rtcm3_correction_diff corrections[RTCM3_MAX_SATELLITES];
        } rtcm3_1015;
        // 1018-1029 were in the 3.0 version
        struct rtcm3_1019_t {
            unsigned int ident;         // Satellite ID
            unsigned int week;          // GPS Week Number
            unsigned char sv_accuracy;  // GPS SV ACCURACY
            enum {reserved_code, p, ca, l2c} code;
            double idot;
            unsigned char iode;
            // ephemeris fields, not scaled
            unsigned int t_sub_oc;
            signed int a_sub_f2;
            signed int a_sub_f1;
            signed int a_sub_f0;
            unsigned int iodc;
            signed int C_sub_rs;
            signed int delta_sub_n;
            signed int M_sub_0;
            signed int C_sub_uc;
            unsigned int e;
            signed int C_sub_us;
            unsigned int sqrt_sub_A;
            unsigned int t_sub_oe;
            signed int C_sub_ic;
            signed int OMEGA_sub_0;
            signed int C_sub_is;
            signed int i_sub_0;
            signed int C_sub_rc;
            signed int argument_of_perigee;
            signed int omegadot;
            signed int t_sub_GD;
            unsigned char sv_health;
            bool p_data;
            bool fit_interval;
        } rtcm3_1019;
        struct rtcm3_1020_t {
            unsigned int ident;         // Satellite ID
            unsigned short channel;     // Satellite Frequency Channel Number
            // ephemeris fields, not scaled
            bool C_sub_n;
            bool health_avAilability_indicator;
            unsigned char P1;
            unsigned short t_sub_k;
            bool msb_of_B_sub_n;
            bool P2;
            bool t_sub_b;
            signed int x_sub_n_t_of_t_sub_b_prime;
            signed int x_sub_n_t_of_t_sub_b;
            signed int x_sub_n_t_of_t_sub_b_prime_prime;
            signed int y_sub_n_t_of_t_sub_b_prime;
            signed int y_sub_n_t_of_t_sub_b;
            signed int y_sub_n_t_of_t_sub_b_prime_prime;
            signed int z_sub_n_t_of_t_sub_b_prime;
            signed int z_sub_n_t_of_t_sub_b;
            signed int z_sub_n_t_of_t_sub_b_prime_prime;
            bool P3;
            signed int gamma_sub_n_of_t_sub_b;
            unsigned char MP;
            bool Ml_n;
            signed int tau_n_of_t_sub_b;
            signed int M_delta_tau_sub_n;
            unsigned int E_sub_n;
            bool MP4;
            unsigned char MF_sub_T;
            unsigned char MN_sub_T;
            unsigned char MM;
            bool additioinal_data_availability;
            unsigned int N_sup_A;
            unsigned int tau_sub_c;
            unsigned int M_N_sub_4;
            signed int M_tau_sub_GPS;
            bool M_l_sub_n;
        } rtcm3_1020;
        struct rtcm3_1021_t {
            // size_t src_len;
            char src_name[RTCM3_MAX_DESCRIPTOR+1];      // DF144, Source-Name
            // size_t tar_len;
            char tar_name[RTCM3_MAX_DESCRIPTOR+1];      // DF146, Target-Name
            // DF147, System Identification Number
            unsigned int sys_id_num;
            // DF148, Utilized Transformation Message Indicator
            bool ut_tr_msg_id[RTCM3_DF148_SIZE];
            // DF149, Plate Number 0-31 values
            unsigned char plate_number;
            // DF150, Computation Indicator, 0-15 values
            unsigned char computation_id;
            enum {
                H_GEOMETRIC, H_PHYS_TAR, H_PHYS_SRC, H_RESERVED
            } height_id;                // DF151, Height Indicator, 0-3 values
            // Latitude of Origin, Longitude of Origin
            double lat_origin, lon_origin;
            // N/S Extension, E/W Extension
            double lat_extension, lon_extension;
            double x_trans, y_trans, z_trans;           // dX, dY, dZ
            double x_rot, y_rot, z_rot;                 // rX, rY, rZ
            double ds;                                  // DF162, partial scale
            // Major|Minor-Axis-Source, Major|Minor-Axis-Target
            double add_as, add_bs, add_at, add_bt;
            // Horizontal Quality Indicator
            enum RTCM3_QUALITY_INDICATOR_TRANSFORMATION quality_hori;
            // Vertical Quality Indicator
            enum RTCM3_QUALITY_INDICATOR_TRANSFORMATION quality_vert;
        } rtcm3_1021;
        struct {
            // DF147, System Identification Number
            unsigned int sys_id_num;
            // Horizontal Shift indicator
            bool shift_id_hori;
            bool shift_id_vert;                     // Vertical Shift indicator
            // Latitude of Origin, Longitude of Origin
            double lat_origin, lon_origin;
            // grid Extension Latitude, grid Extension Longitude
            double lat_extension, lon_extension;
            // mean offset of latitude, longitude, and elevation
            double lat_mean, lon_mean, hgt_mean;
            // 4*4 residual grid points
            struct rtcm3_1023_t {
                double lat_res, lon_res, hgt_res;
            } residuals[RTCM3_GRID_SIZE];
            // Horizontal interpolation method indicator
            enum RTCM3_INTERPOLATION_INDICATOR interp_meth_id_hori;
            // Vertical interpolation method indicator
            enum RTCM3_INTERPOLATION_INDICATOR interp_meth_id_vert;
            // Horizontal Grid quality indicator
            enum RTCM3_QUALITY_INDICATOR_GRID_RESIDUALS grd_qual_id_hori;
            // Vertical Grid quality indicator
            enum RTCM3_QUALITY_INDICATOR_GRID_RESIDUALS grd_qual_id_vert;
            unsigned short mjd;                         // Modified Julian Date
        } rtcm3_1023;
        struct rtcm3_1025_t {
            // DF147, System Identification Number
            unsigned int sys_id_num;
            enum RTCM3_PROJECTION_TYPE projection_type; // Projection Types
            // Latitude of Origin, Longitude of Origin
            double lat_origin, lon_origin;
            double add_sno;                             // DF173
            double false_east, false_north;             // DF174, DF175
        } rtcm3_1025;
        struct rtcm3_1029_t
        {
            unsigned int station_id;    // Reference Station ID
            unsigned short mjd;         // Modified Julian Day (MJD) Number
            unsigned int sod;           // Seconds of Day (UTC)
            size_t len;                 // # chars to follow
            size_t unicode_units;       // # Unicode units (bytes) in text
            unsigned char text[255];
        } rtcm3_1029;
        struct rtcm3_1033_t {
            unsigned int station_id;                    // Reference Station ID
            char descriptor[RTCM3_MAX_DESCRIPTOR+1];    // Description string
            unsigned int setup_id;
            char serial[RTCM3_MAX_DESCRIPTOR+1];        // Serial # string
            char receiver[RTCM3_MAX_DESCRIPTOR+1];      // Receiver string
            char firmware[RTCM3_MAX_DESCRIPTOR+1];      // Firmware string
        } rtcm3_1033;
        struct {
            unsigned int station_id;                    // Reference Station ID
            unsigned char bias_indicator;
            unsigned char signals_mask;
            int l1_ca_bias;         // GLONASS L1 C/A Code-Phase Bias
            int l1_p_bias;          // GLONASS L1 P Code-Phase Bias
            int l2_ca_bias;         // GLONASS L2 C/A Code-Phase Bias
            int l2_p_bias;          // GLONASS L2 P Code-Phase Bias
        } rtcm3_1230;
        struct rtcm3_msm_hdr rtcm3_msm;
        struct rtcm3_4076_hdr rtcm3_4076;
        unsigned char data[1024];       // Max RTCM3 msg length is 1023 bytes
    } rtcmtypes;
};

// RTCM3 scaling constants
#define GPS_AMBIGUITY_MODULUS           299792.458      // 1004, DF014
#define GLONASS_AMBIGUITY_MODULUS       599584.916      // 1012, DF044
#define MESSAGE_INTERVAL_UNITS          0.1             // 1013, DF047

/*
 * The one true structure for (most) orbital data.
 * Before using any datum, check for validity, or isfinite()
 *
 * Why only store scaled data?  With 7 constellations, each with an
 * almanac and ephemeris, that would be 14 unique non-scaled sets.
 * Too much work.
 *
 */
struct orbit
{
    // orbit type, 0 == invalid, 1 = ephemeris, 2 == almanac
    uint8_t type;
#define ORBIT_INVALID 0
#define ORBIT_EPHEMERIS 1
#define ORBIT_ALMANAC 2
    // The satellite this refers to, zero if struct invalid
    uint8_t sv;
    // SV health data, random format, -1 if invalid
    // Issue of Data, IODa, IODc, IODe., -1 is invalid
    int8_t IODA;
    int8_t IODC;
    int8_t IODE;

    int8_t E5bHS;        // Galileo E5b health status, -1 invalid
    int8_t E1BHS;        // Galileo E1B health status, -1 invalid
    // AODC, Age of Data Clock, BDS, -1 if invalid
    int AODC;
    // AODE, Age of Data Ephemeris, BDS, -1 if invalid
    int AODE;
    // SISA(E1,E5a), -1 if invalid
    int SISAa;
    // SISA(E1,E5b), -1 if invalid
    int SISAb;
    // SV health data, random format, -1 if invalid
    int svh;
    // URAI, BDS, -1 if invalid
    int URAI;
    // Week Number, -1 if invalid
    int WN;

    // longs
    // toa, time of almanac, -1 if invalid
    long toa;
    // toc, time of clock, -1 if invalid
    long toc;
    // toe, time of ephemeris, -1 if invalid
    long toe;
    // LSBs of toe, seconds, -1 if invalid
    long toeLSB;
    // MSBs of toe, seconds, scaled, -1 if invalid
    long toeMSB;

    // af0, aka a0, SV clock correction constant term, seconds
    double af0;
    // af1, aka a1, SV clock correction first order term, seconds/second
    double af1;
    // af2, aka a2, SV clock correction second order term, seconds/second**2
    double af2;

    // alpha0, seconds
    double alpha0;
    // alpha1, seconds/semi-circle
    double alpha1;
    // alpha2, seconds/semi-circle**2
    double alpha2;
    // alpha3, seconds/semi-circle**3
    double alpha3;

    // beta0, seconds
    double beta0;
    // beta1, seconds/semi-circle
    double beta1;
    // beta2, seconds/semi-circle**2
    double beta2;
    // beta3, seconds/semi-circle**3
    double beta3;

    /* Cic, Amplitude of the Cosine Harmonic Correction Term to the
     * Angle of Inclination, radians*/
    double Cic;
    /* Cis, Amplitude of the Sine Harmonic Correction Term to the
     * Angle of Inclination, radians */
    double Cis;
    /* Crc, Amplitude of the Cosine Harmonic Correction Term to the
     * Orbit Radius, meters */
    double Crc;
    /* Crs, Amplitude of the Sine Harmonic Correction Term to the
     * Orbit Radius, signed, meters */
    double Crs;
    /* Cuc, Amplitude of the Cosine Harmonic Correction Term to the
     * Argument of Latitude, 16 bits, radians */
    double Cuc;
    /* Cus, Amplitude of the Sine Harmonic Correction Term to the
     * Argument of Latitude, radians */
    double Cus;
    // deltai, correction to inclination, semi-circles
    double deltai;
    // deltan, Mean motion difference from computed value, semicircles/sec
    double deltan;
    // eccentricity, unsigned, dimensionless
    double eccentricity;

    // i0, Inclination Angle at Reference Time, signed, semi-circles
    // range +/- 1
    double i0;
    // IDOT (BDS), idot (GAL) Rate of Inclination Angle, * semi-circles/sec
    double IDOT;
    // M0, Mean Anomaly at Reference Time, semi-circles, range +/- 1
    double M0;
    /* Omega0, Longitude of Ascending Node of Orbit Plane at Weekly Epoch,
     * semi-circles, range +/- 1 */
    double Omega0;
    // Omega dot, Rate of Right Ascension, semi-circles/sec
    double Omegad;
    // omega, Argument of Perigee, semi-circles, range +/- 1
    double omega;
    // sqrt A, Square Root of the Semi-Major Axis, square_root(meters)
    double sqrtA;
    // TGD1, TGD2, Time Group Delay 1, 2, BDS, seconds
    double TGD1;
    double TGD2;
};
typedef struct orbit orbit_t;

/*
 * Raw IS_GPS subframe data
 */

/* The almanac is a subset of the clock and ephemeris data, with reduced
 * precision. See IS-GPS-200, Table 20-VI  */
struct almanac_t
{
    uint8_t sv;  // The satellite this refers to
    // toa, almanac reference time, 8 bits unsigned, seconds
    uint8_t toa;
    unsigned long l_toa;
    // SV health data, 8 bit unsigned bit map
    uint8_t svh;
    // deltai, correction to inclination, 16 bits signed, semi-circles
    int16_t deltai;
    double d_deltai;
    // M0, Mean Anomaly at Reference Time, 24 bits signed, semi-circles
    int32_t M0;
    double d_M0;
    /* Omega0, Longitude of Ascending Node of Orbit Plane at Weekly Epoch,
     * 24 bits signed, semi-circles */
    int32_t Omega0;
    double d_Omega0;
    // omega, Argument of Perigee, 24 bits signed, semi-circles
    int32_t omega;
    double d_omega;
    /* af0, SV clock correction constant term
     * 11 bits signed, seconds */
    int16_t af0;
    double d_af0;
    /* af1, SV clock correction first order term
     * 11 bits signed, seconds/second */
    int16_t af1;
    double d_af1;
    // eccentricity, 16 bits, unsigned, dimensionless
    uint16_t e;
    double d_eccentricity;
    /* sqrt A, Square Root of the Semi-Major Axis
     * 24 bits unsigned, square_root(meters) */
    uint32_t sqrtA;
    double d_sqrtA;
    // Omega dot, Rate of Right Ascension, 16 bits signed, semi-circles/sec
    int16_t Omegad;
    double d_Omegad;
};

struct subframe_t {
    // different gnss use different subframes.
    uint8_t gnssId;
    // subframe number, GPS, 3 bits, unsigned, 1 to 5, 0 == invalid
    uint8_t subframe_num;
    /* data_id, denotes the NAV data structure of D(t), 2 bits, in
     * IS-GPS-200 always == 0x1 */
    uint8_t data_id;
    // SV/page id used for subframes 4 & 5, 6 bits
    uint8_t pageid;
    // tSVID, SV ID of the sat that transmitted this frame, 6 bits unsigned
    uint8_t tSVID;
    /* TOW17, Time of Week of next Subframe.
     * 17 bits unsigned, 0 to 100,799, scale 6, seconds, -1 if invalid */
    int32_t TOW17;            // 0 to 100,799, or 0 to 604,794
    // integrity, URA bounds flag, 1 bit
    bool integrity;
    /* alert, alert flag, SV URA and/or the SV User Differential Range
     * Accuracy (UDRA) may be worse than indicated, 1 bit */
    bool alert;
    // antispoof, A-S mode is ON in that SV, 1 bit
    bool antispoof;
    // Week Number for sending constellation, -1 for invalid
    int WN;
    // 1 == almanac, 2 == orbit
    int is_almanac;
#define SUBFRAME_ALMANAC 1
#define SUBFRAME_ORBIT 2
    // generic almanac, ephemeris
    orbit_t orbit;
    // Galileo sends two 1/2 almanacs at one time.  2nd one goes here.
    orbit_t orbit1;
    union {
        /* subframe 1, part of ephemeris, see IS-GPS-200, Table 20-II
         * and Table 20-I */
        struct {
            // WN, Week Number, 10 bits unsigned, scale 1, weeks
            uint16_t WN;
            /* IODC, Issue of Data, Clock, 10 bits, unsigned,
             * issued in 8 data ranges at the same time */
            uint16_t IODC;
            /* toc, clock data reference time, 16 bits, unsigned, seconds
             * scale 2**4, issued in 8 data ranges at the same time */
            uint16_t toc;
            long l_toc;
            // l2, code on L2, 2 bits, bit map
            uint8_t l2;
            // l2p, L2 P data flag, 1 bit
            uint8_t l2p;
            // ura, SV accuracy, 4 bits unsigned index
            unsigned int ura;
            // hlth, SV health, 6 bits unsigned bitmap
            unsigned int hlth;
            /* af0, SV clock correction constant term
             * 22 bits signed, scale 2**-31, seconds */
            int32_t af0;
            double d_af0;
            /* af1, SV clock correction first order term
             * 22 bits signed, scale 2**-43, seconds/second */
            int16_t af1;
            double d_af1;
            /* af2, SV clock correction second order term
             * 8 bits signed, scale 2**-55, seconds/second**2 */
            int8_t af2;
            double d_af2;
            /* Tgd,  L1-L2 correction term, 8 bits signed,  scale 2**-31,
             * seconds */
            int8_t Tgd;
            double d_Tgd;
        } sub1;
        /* subframe 2, part of ephemeris, see IS-GPS-200, Table 20-II
         * and Table 20-III */
        struct {
            /* Issue of Data (Ephemeris),
             * equal to the 8 LSBs of the 10 bit IODC of the same data set */
            uint8_t IODE;
            /* Age of Data Offset for the NMCT, 6 bits, scale 900,
             * ignore if all ones, seconds */
            uint8_t AODO;
            uint16_t u_AODO;
            /* fit, FIT interval flag, indicates a fit interval greater than
             * 4 hour, 1 bit */
            uint8_t fit;
            /* toe, Reference Time Ephemeris, 16 bits unsigned, scale 2**4,
             * seconds */
            uint16_t toe;
            unsigned long l_toe;
            /* Crs, Amplitude of the Sine Harmonic Correction Term to the
             * Orbit Radius, 16 bits, scale 2**-5, signed, meters */
            int16_t Crs;
            double d_Crs;
            /* Cus, Amplitude of the Sine Harmonic Correction Term to the
             * Argument of Latitude, 16 bits, signed, scale 2**-29, radians */
            int16_t Cus;
            double d_Cus;
            /* Cuc, Amplitude of the Cosine Harmonic Correction Term to the
             * Argument of Latitude, 16 bits, signed, scale 2**-29, radians */
            int16_t Cuc;
            double d_Cuc;
            /* deltan, Mean Motion Difference From Computed Value
             * Mean Motion Difference From Computed Value
             * 16 bits, signed, scale 2**-43, semi-circles/sec */
            int16_t deltan;
            double d_deltan;
            /* M0, Mean Anomaly at Reference Time, 32 bits signed,
             * scale 2**-31, semi-circles */
            int32_t M0;
            double d_M0;
            // eccentricity, 32 bits, unsigned, scale 2**-33, dimensionless
            uint32_t e;
            double d_eccentricity;
            /* sqrt A, Square Root of the Semi-Major Axis
             * 32 bits unsigned, scale 2**-19, square_root(meters) */
            uint32_t sqrtA;
            double d_sqrtA;
        } sub2;
        /* subframe 3, part of ephemeris, see IS-GPS-200, Table 20-II,
         * Table 20-III */
        struct {
            /* Issue of Data (Ephemeris), 8 bits, unsigned
             * equal to the 8 LSBs of the 10 bit IODC of the same data set */
            uint8_t IODE;
            /* Rate of Inclination Angle, 14 bits signed, scale2**-43,
             * semi-circles/sec */
            int16_t IDOT;
            double d_IDOT;
            /* Cic, Amplitude of the Cosine Harmonic Correction Term to the
             * Angle of Inclination, 16 bits signed, scale 2**-29, radians*/
            int16_t Cic;
            double d_Cic;
            /* Cis, Amplitude of the Sine Harmonic Correction Term to the
             * Angle of Inclination, 16 bits, unsigned, scale 2**-29, radians */
            int16_t Cis;
            double d_Cis;
            /* Crc, Amplitude of the Cosine Harmonic Correction Term to the
             * Orbit Radius, 16 bits signed, scale 2**-5, meters */
            int16_t Crc;
            double d_Crc;
            /* i0, Inclination Angle at Reference Time, 32 bits, signed,
             * scale 2**-31, semi-circles */
            int32_t i0;
            double d_i0;
            /* Omega0, Longitude of Ascending Node of Orbit Plane at Weekly
             * Epoch, 32 bits signed, semi-circles */
            int32_t Omega0;
            double d_Omega0;
            /* omega, Argument of Perigee, 32 bits signed, scale 2**-31,
             * semi-circles */
            int32_t omega;
            double d_omega;
            /* Omega dot, Rate of Right Ascension, 24 bits signed,
             * scale 2**-43, semi-circles/sec */
            int32_t Omegad;
            double d_Omegad;
        } sub3;
        struct {
            struct almanac_t almanac;
        } sub4;
        // subframe 4, page 13
        struct {
            /* mapping ord ERD# to SV # is non trivial
             * leave it alone.  See IS-GPS-200 Section 20.3.3.5.1.9
             * Estimated Range Deviation, 6 bits signed, meters */
            int8_t ERD[33];
            // ai, Availability Indicator, 2bits, bit map
            uint8_t ai;
        } sub4_13;
        // subframe 4, page 17, system message, 23 chars, plus nul
        struct {
            char str[24];
        } sub4_17;
        // subframe 4, page 18
        struct {          // ionospheric and UTC data
            /* A0, Bias coefficient of GPS time scale relative to UTC time
             * scale, 32 bits signed, scale 2**-30, seconds */
            int32_t A0;
            double d_A0;
            /* A1, Drift coefficient of GPS time scale relative to UTC time
             * scale, 24 bits signed, scale 2**-50, seconds/second */
            int32_t A1;
            double d_A1;

            /* alphaX, the four coefficients of a cubic equation representing
             * the amplitude of the vertical delay */

            // alpha0, 8 bits signed, scale w**-30, seconds
            int8_t alpha0;
            double d_alpha0;
            // alpha1, 8 bits signed, scale w**-27, seconds/semi-circle
            int8_t alpha1;
            double d_alpha1;
            // alpha2, 8 bits signed, scale w**-24, seconds/semi-circle**2
            int8_t alpha2;
            double d_alpha2;
            // alpha3, 8 bits signed, scale w**-24, seconds/semi-circle**3
            int8_t alpha3;
            double d_alpha3;

            /* betaX, the four coefficients of a cubic equation representing
             * the period of the model */

            // beta0, 8 bits signed, scale w**11, seconds
            int8_t beta0;
            double d_beta0;
            // beta1, 8 bits signed, scale w**14, seconds/semi-circle
            int8_t beta1;
            double d_beta1;
            // beta2, 8 bits signed, scale w**16, seconds/semi-circle**2
            int8_t beta2;
            double d_beta2;
            // beta3, 8 bits signed, scale w**16, seconds/semi-circle**3
            int8_t beta3;
            double d_beta3;

            /* leap (delta t ls), current leap second, 8 bits signed,
             * scale 1, seconds */
            int8_t leap;
            /* lsf (delta t lsf), future leap second, 8 bits signed,
             * scale 1, seconds */
            int8_t lsf;

            /* tot, reference time for UTC data,
             * 16 bits unsigned, scale 2**12, 0 to 604,784 seconds */
            uint16_t tot;
            unsigned long t_tot;

            // WNt, UTC reference week number, 8 bits unsigned, scale 1, weeks
            uint8_t WNt;
            /* WNlsf, Leap second reference Week Number,
             * 8 bits unsigned, scale 1, weeks */
            uint8_t WNlsf;
            /* DN, Leap second reference Day Number , 8 bits unsigned,
             * scale 1, days */
            uint8_t DN;
        } sub4_18;
        // subframe 4, page 25
        struct {
            /* svf, A-S status and the configuration code of each SV
             * 4 bits unsigned, bitmap */
            unsigned char svf[33];
            /* svh, SV health data for SV 25 through 32
             * 6 bits unsigned bitmap */
            uint8_t svhx[8];
        } sub4_25;
        struct {
            struct almanac_t almanac;
        } sub5;
        struct {
            /* toa, Almanac reference Time, 8 bits unsigned, scale 2**12,
             * seconds */
            uint8_t toa;
            unsigned long l_toa;
            /* WNa, Week Number almanac, 8 bits, scale 2, GPS Week
             * Number % 256 */
            uint8_t WNa;
            // sv, SV health status, 6 bits, bitmap
            uint8_t sv[25];
        } sub5_25;
    };
};

typedef uint64_t gps_mask_t;

/*
 * Is an MMSI number that of an auxiliary associated with a mother ship?
 * We need to be able to test this for decoding AIS Type 24 messages.
 * According to <http://www.navcen.uscg.gov/marcomms/gmdss/mmsi.htm#format>,
 * auxiliary-craft MMSIs have the form 98MIDXXXX, where MID is a country
 * code and XXXX the vessel ID.
 */
#define AIS_AUXILIARY_MMSI(n)   ((n) / 10000000 == 98)

// N/A values and scaling constant for 25/24 bit lon/lat pairs
#define AIS_LON3_NOT_AVAILABLE  181000
#define AIS_LAT3_NOT_AVAILABLE  91000
#define AIS_LATLON3_DIV 60000.0

// N/A values and scaling constant for 28/27 bit lon/lat pairs
#define AIS_LON4_NOT_AVAILABLE  1810000
#define AIS_LAT4_NOT_AVAILABLE  910000
#define AIS_LATLON4_DIV 600000.0

struct route_info {
    unsigned int linkage;       // Message Linkage ID
    unsigned int sender;        // Sender Class
    unsigned int rtype;         // Route Type
    unsigned int month;         // Start month
    unsigned int day;           // Start day
    unsigned int hour;          // Start hour
    unsigned int minute;        // Start minute
    unsigned int duration;      // Duration
    int waycount;               // Waypoint count
    struct waypoint_t {
        signed int lon;         // Longitude
        signed int lat;         // Latitude
    } waypoints[16];
};

struct ais_t
{
    unsigned int        type;           // message type
    unsigned int        repeat;         // Repeat indicator
    unsigned int        mmsi;           // MMSI
    union {
        // Types 1-3 Common navigation info
        struct {
            unsigned int status;                // navigation status
            signed turn;                        // rate of turn
#define AIS_TURN_HARD_LEFT      -127
#define AIS_TURN_HARD_RIGHT     127
#define AIS_TURN_NOT_AVAILABLE  128
            unsigned int speed;         // speed over ground in deciknots
#define AIS_SPEED_NOT_AVAILABLE 1023
#define AIS_SPEED_FAST_MOVER    1022            // >= 102.2 knots
            bool accuracy;                      // position accuracy
#define AIS_LATLON_DIV  600000.0
            int lon;                            // longitude
#define AIS_LON_NOT_AVAILABLE   0x6791AC0
            int lat;                            // latitude
#define AIS_LAT_NOT_AVAILABLE   0x3412140
            unsigned int course;                // course over ground
#define AIS_COURSE_NOT_AVAILABLE        3600
            unsigned int heading;               // true heading
#define AIS_HEADING_NOT_AVAILABLE       511
            /* seconds of UTC time, 0 to 59.
             * 60 == N/A, 61 == manual, 62 == dead reckoning,
             * 63 == inoperative */
            unsigned int second;
#define AIS_SEC_NOT_AVAILABLE   60
#define AIS_SEC_MANUAL          61
#define AIS_SEC_ESTIMATED       62
#define AIS_SEC_INOPERATIVE     63
            unsigned int maneuver;      // maneuver indicator
            // unsigned int spare;       // spare bits
            bool raim;                  // RAIM flag
            unsigned int radio;         // radio status bits
        } type1;
        // Type 4 - Base Station Report & Type 11 - UTC and Date Response
        struct {
            unsigned int year;                  // UTC year
#define AIS_YEAR_NOT_AVAILABLE  0
            unsigned int month;                 // UTC month
#define AIS_MONTH_NOT_AVAILABLE 0
            unsigned int day;                   // UTC day
#define AIS_DAY_NOT_AVAILABLE   0
            unsigned int hour;                  // UTC hour
#define AIS_HOUR_NOT_AVAILABLE  24
            unsigned int minute;                // UTC minute
#define AIS_MINUTE_NOT_AVAILABLE        60
            unsigned int second;                // UTC second
#define AIS_SECOND_NOT_AVAILABLE        60
            bool accuracy;              // fix quality
            int lon;                    // longitude
            int lat;                    // latitude
            unsigned int epfd;          // type of position fix device
            // unsigned int spare;      // spare bits
            bool raim;                  // RAIM flag
            unsigned int radio;         // radio status bits
        } type4;
        // Type 5 - Ship static and voyage related data
        struct {
            unsigned int ais_version;   // AIS version level
            unsigned int imo;           // IMO identification
            // cppcheck-suppress arrayIndexOutOfBounds
            char callsign[7+1];         // callsign
#define AIS_SHIPNAME_MAXLEN     20
            // cppcheck-suppress arrayIndexOutOfBounds
            char shipname[AIS_SHIPNAME_MAXLEN+1];       // vessel name
            unsigned int shiptype;      // ship type code
            unsigned int to_bow;        // dimension to bow
            unsigned int to_stern;      // dimension to stern
            unsigned int to_port;       // dimension to port
            unsigned int to_starboard;  // dimension to starboard
            unsigned int epfd;          // type of position fix device
            unsigned int month;         // UTC month
            unsigned int day;           // UTC day
            unsigned int hour;          // UTC hour
            unsigned int minute;        // UTC minute
            unsigned int draught;       // draft in meters
            char destination[20+1];     // ship destination
            unsigned int dte;           // data terminal enable
            // unsigned int spare;      // spare bits
        } type5;
        // Type 6 - Addressed Binary Message
        struct {
            unsigned int seqno;         // sequence number
            unsigned int dest_mmsi;     // destination MMSI
            bool retransmit;            // retransmit flag
            // unsigned int spare;      // spare bit(s)
            unsigned int dac;           // Application ID
            unsigned int fid;           // Functional ID
            bool structured;            // True match for DAC/FID?
#define AIS_TYPE6_BINARY_MAX    920     // 920 bits
            size_t bitcount;            // bit count of the data
            union {
                // cppcheck-suppress arrayIndexOutOfBounds
                char bitdata[(AIS_TYPE6_BINARY_MAX + 7) / 8];
                // Inland AIS - ETA at lock/bridge/terminal
                struct {
                    char country[2+1];          // UN Country Code
                    char locode[3+1];           // UN/LOCODE
                    char section[5+1];          // Fairway section
                    char terminal[5+1];         // Terminal code
                    char hectometre[5+1];       // Fairway hectometre
                    unsigned int month;         // ETA month
                    unsigned int day;           // ETA day
                    unsigned int hour;          // ETA hour
                    unsigned int minute;        // ETA minute
                    unsigned int tugs;          // Assisting Tugs
                    unsigned int airdraught;    // Air Draught
                } dac200fid21;
                // Inland AIS - ETA at lock/bridge/terminal
                struct {
                    char country[2+1];          // UN Country Code
                    char locode[3+1];           // UN/LOCODE
                    char section[5+1];          // Fairway section
                    char terminal[5+1];         // Terminal code
                    char hectometre[5+1];       // Fairway hectometre
                    unsigned int month;         // RTA month
                    unsigned int day;           // RTA day
                    unsigned int hour;          // RTA hour
                    unsigned int minute;        // RTA minute
                    unsigned int status;        // Status
#define DAC200FID22_STATUS_OPERATIONAL  0
#define DAC200FID22_STATUS_LIMITED      1
#define DAC200FID22_STATUS_OUT_OF_ORDER 2
#define DAC200FID22_STATUS_NOT_AVAILABLE        0
                } dac200fid22;
                // Inland AIS - Number of persons on board
                struct {
                    unsigned int crew;          // # crew on board
                    unsigned int passengers;    // # passengers on board
                    unsigned int personnel;     // # personnel on board
#define DAC200FID55_COUNT_NOT_AVAILABLE 255
                } dac200fid55;
                // GLA - AtoN monitoring data (UK/ROI)
                struct {
                    unsigned int ana_int;       // Analogue (internal)
                    unsigned int ana_ext1;      // Analogue (external #1)
                    unsigned int ana_ext2;      // Analogue (external #2)
                    unsigned int racon;         // RACON status
                    unsigned int light;         // Light status
                    bool alarm;                 // Health alarm
                    unsigned int stat_ext;      // Status bits (external)
                    bool off_pos;               // Off position status
                } dac235fid10;
                // IMO236 - Dangerous Cargo Indication
                struct {
                    char lastport[5+1];         // Last Port Of Call
                    unsigned int lmonth;        // ETA month
                    unsigned int lday;          // ETA day
                    unsigned int lhour;         // ETA hour
                    unsigned int lminute;       // ETA minute
                    char nextport[5+1];         // Next Port Of Call
                    unsigned int nmonth;        // ETA month
                    unsigned int nday;          // ETA day
                    unsigned int nhour;         // ETA hour
                    unsigned int nminute;       // ETA minute
                    char dangerous[20+1];       // Main Dangerous Good
                    char imdcat[4+1];           // IMD Category
                    unsigned int unid;          // UN Number
                    unsigned int amount;        // Amount of Cargo
                    unsigned int unit;          // Unit of Quantity
                } dac1fid12;
                // IMO236 - Extended Ship Static and Voyage Related Data
                struct {
                    unsigned int airdraught;    // Air Draught
                } dac1fid15;
                // IMO236 - Number of Persons on board
                struct {
                    unsigned persons;           // number of persons
                } dac1fid16;
                // IMO289 - Clearance Time To Enter Port
                struct {
                    unsigned int linkage;       // Message Linkage ID
                    unsigned int month;         // Month (UTC)
                    unsigned int day;           // Day (UTC)
                    unsigned int hour;          // Hour (UTC)
                    unsigned int minute;        // Minute (UTC)
                    char portname[20+1];        // Name of Port & Berth
                    char destination[5+1];      // Destination
                    signed int lon;             // Longitude
                    signed int lat;             // Latitude
                } dac1fid18;
                // IMO289 - Berthing Data (addressed)
                struct {
                    unsigned int linkage;         // Message Linkage ID
                    unsigned int berth_length;    // Berth length
                    unsigned int berth_depth;     // Berth Water Depth
                    unsigned int position;        // Mooring Position
                    unsigned int month;           // Month (UTC)
                    unsigned int day;             // Day (UTC)
                    unsigned int hour;            // Hour (UTC)
                    unsigned int minute;          // Minute (UTC)
                    unsigned int availability;    // Services Availability
                    unsigned int agent;           // Agent
                    unsigned int fuel;            // Bunker/fuel
                    unsigned int chandler;        // Chandler
                    unsigned int stevedore;       // Stevedore
                    unsigned int electrical;      // Electrical
                    unsigned int water;           // Potable water
                    unsigned int customs;         // Customs house
                    unsigned int cartage;         // Cartage
                    unsigned int crane;           // Crane(s)
                    unsigned int lift;            // Lift(s)
                    unsigned int medical;         // Medical facilities
                    unsigned int navrepair;       // Navigation repair
                    unsigned int provisions;      // Provisions
                    unsigned int shiprepair;      // Ship repair
                    unsigned int surveyor;        // Surveyor
                    unsigned int steam;           // Steam
                    unsigned int tugs;            // Tugs
                    unsigned int solidwaste;      // Waste disposal (solid)
                    unsigned int liquidwaste;     // Waste disposal (liquid)
                    unsigned int hazardouswaste;  // Waste disposal (hazardous)
                    unsigned int ballast;         // Reserved ballast exchange
                    unsigned int additional;      // Additional services
                    unsigned int regional1;       // Regional reserved 1
                    unsigned int regional2;       // Regional reserved 2
                    unsigned int future1;         // Reserved for future
                    unsigned int future2;         // Reserved for future
                    char berth_name[20+1];        // Name of Berth
                    signed int berth_lon;         // Longitude
                    signed int berth_lat;         // Latitude
                } dac1fid20;
                // IMO289 - Weather observation report from ship
                // *** WORK IN PROGRESS - NOT YET DECODED ***
                struct {
                    bool wmo;                     // true if WMO variant
                    union {
                        struct {
                            char location[20+1];        // Location
                            signed int lon;             // Longitude
                            signed int lat;             // Latitude
                            unsigned int day;           // Report day
                            unsigned int hour;          // Report hour
                            unsigned int minute;        // Report minute
                            bool vislimit;              // Max range?
                            unsigned int visibility;    // Units of 0.1 nm
#define DAC1FID21_VISIBILITY_NOT_AVAILABLE      127
#define DAC1FID21_VISIBILITY_SCALE              10.0
                            unsigned humidity;          // units of 1%
                            unsigned int wspeed;        // average wind speed
                            unsigned int wgust;         // wind gust
#define DAC1FID21_WSPEED_NOT_AVAILABLE          127
                            unsigned int wdir;          // wind direction
#define DAC1FID21_WDIR_NOT_AVAILABLE            360
                            unsigned int pressure;      // air pressure, hpa
#define DAC1FID21_NONWMO_PRESSURE_NOT_AVAILABLE 403
#define DAC1FID21_NONWMO_PRESSURE_HIGH          402     // > 1200hPa
#define DAC1FID21_NONWMO_PRESSURE_OFFSET        400     // N/A
                            unsigned int pressuretend;  // tendency
                            int airtemp;                // temp, units 0.1C
#define DAC1FID21_AIRTEMP_NOT_AVAILABLE         -1024
#define DAC1FID21_AIRTEMP_SCALE                 10.0
                            unsigned int watertemp;     // units 0.1degC
#define DAC1FID21_WATERTEMP_NOT_AVAILABLE       501
#define DAC1FID21_WATERTEMP_SCALE               10.0
                            unsigned int waveperiod;    // in seconds
#define DAC1FID21_WAVEPERIOD_NOT_AVAILABLE      63
                            unsigned int wavedir;       // direction in deg
#define DAC1FID21_WAVEDIR_NOT_AVAILABLE         360
                            unsigned int swellheight;   // in decimeters
                            unsigned int swellperiod;   // in seconds
                            unsigned int swelldir;      // direction in deg
                        } nonwmo_obs;
                        struct {
                            signed int lon;             // Longitude
                            signed int lat;             // Latitude
                            unsigned int month;         // UTC month
                            unsigned int day;           // Report day
                            unsigned int hour;          // Report hour
                            unsigned int minute;        // Report minute
                            unsigned int course;        // course over ground
                            unsigned int speed;         // speed, m/s
#define DAC1FID21_SOG_NOT_AVAILABLE             31
#define DAC1FID21_SOG_HIGH_SPEED                30
#define DAC1FID21_SOG_SCALE                     2.0
                            unsigned int heading;       // true heading
#define DAC1FID21_HDG_NOT_AVAILABLE             127
#define DAC1FID21_HDG_SCALE                     5.0
                            unsigned int pressure;      // units of hPa * 0.1
#define DAC1FID21_WMO_PRESSURE_SCALE            10
#define DAC1FID21_WMO_PRESSURE_OFFSET           90.0
                            unsigned int pdelta;        // units of hPa * 0.1
#define DAC1FID21_PDELTA_SCALE                  10
#define DAC1FID21_PDELTA_OFFSET                 50.0
                            unsigned int ptend;         // enumerated
                            unsigned int twinddir;      // in 5 degree steps
#define DAC1FID21_TWINDDIR_NOT_AVAILABLE        127
                            unsigned int twindspeed;    // meters per second
#define DAC1FID21_TWINDSPEED_SCALE              2
#define DAC1FID21_RWINDSPEED_NOT_AVAILABLE      255
                            unsigned int rwinddir;      // in 5 degree steps
#define DAC1FID21_RWINDDIR_NOT_AVAILABLE        127
                            unsigned int rwindspeed;    // meters per second
#define DAC1FID21_RWINDSPEED_SCALE              2
#define DAC1FID21_RWINDSPEED_NOT_AVAILABLE      255
                            unsigned int mgustspeed;    // meters per second
#define DAC1FID21_MGUSTSPEED_SCALE              2
#define DAC1FID21_MGUSTSPEED_NOT_AVAILABLE      255
                            unsigned int mgustdir;      // in 5 degree steps
#define DAC1FID21_MGUSTDIR_NOT_AVAILABLE        127
                            unsigned int airtemp;       // degrees K
#define DAC1FID21_AIRTEMP_OFFSET                223
                            unsigned humidity;          // units of 1%
#define DAC1FID21_HUMIDITY_NOT_VAILABLE         127
                            // some trailing fields are missing
                        } wmo_obs;
                    };
                } dac1fid21;
                // *** WORK IN PROGRESS ENDS HERE ***
                // IMO289 - Dangerous Cargo Indication
                struct {
                    unsigned int unit;          // Unit of Quantity
                    unsigned int amount;        // Amount of Cargo
                    int ncargos;
                    struct cargo_t {
                        unsigned int code;      // Cargo code
                        unsigned int subtype;   // Cargo subtype
                    } cargos[28];
                } dac1fid25;
                // IMO289 - Route info (addressed)
                struct route_info dac1fid28;
                // IMO289 - Text message (addressed
                struct {
                    unsigned int linkage;
#define AIS_DAC1FID30_TEXT_MAX  154     // 920 bits of six-bit, plus NUL
                    char text[AIS_DAC1FID30_TEXT_MAX];
                } dac1fid30;
                // IMO289 & IMO236 - Tidal Window
                struct {
                    unsigned int month;         // Month
                    unsigned int day;           // Day
                    signed int ntidals;
                    struct tidal_t {
                        signed int lon;         // Longitude
                        signed int lat;         // Latitude
                        unsigned int from_hour; // From UTC Hour
                        unsigned int from_min;  // From UTC Minute
                        unsigned int to_hour;   // To UTC Hour
                        unsigned int to_min;    // To UTC Minute
#define DAC1FID32_CDIR_NOT_AVAILABLE     360
                        unsigned int cdir;      // Current Dir. Predicted
#define DAC1FID32_CSPEED_NOT_AVAILABLE   127
                        unsigned int cspeed;    // Current Speed Predicted
                    } tidals[3];
                } dac1fid32;
            };
        } type6;
        // Type 7 - Binary Acknowledge
        struct {
            unsigned int mmsi1;
            unsigned int seqno1;
            unsigned int mmsi2;
            unsigned int seqno2;
            unsigned int mmsi3;
            unsigned int seqno3;
            unsigned int mmsi4;
            unsigned int seqno4;
            // spares ignored, they're only padding here
        } type7;
        // Type 8 - Broadcast Binary Message
        struct {
            unsigned int dac;           // Designated Area Code
            unsigned int fid;           // Functional ID
#define AIS_TYPE8_BINARY_MAX    952     // 952 bits
            size_t bitcount;            // bit count of the data
            bool structured;            // True match for DAC/FID?
            union {
                // cppcheck-suppress arrayIndexOutOfBounds
                char bitdata[(AIS_TYPE8_BINARY_MAX + 7) / 8];
                // Inland static ship and voyage-related data
                struct {
                    char vin[8+1];              // European Vessel ID
                    unsigned int length;        // Length of ship
                    unsigned int beam;          // Beam of ship
                    unsigned int shiptype;      // Ship/combination type
                    unsigned int hazard;        // Hazardous cargo
#define DAC200FID10_HAZARD_MAX  5
                    unsigned int draught;       // Draught
                    unsigned int loaded;        // Loaded/Unloaded
                    bool speed_q;               // Speed inf. quality
                    bool course_q;              // Course inf. quality
                    bool heading_q;             // Heading inf. quality
                } dac200fid10;
                // Inland AIS EMMA Warning
                struct {
                    unsigned int start_year;    // Start Year
                    unsigned int start_month;   // Start Month
                    unsigned int start_day;     // Start Day
                    unsigned int end_year;      // End Year
                    unsigned int end_month;     // End Month
                    unsigned int end_day;       // End Day
                    unsigned int start_hour;    // Start Hour
                    unsigned int start_minute;  // Start Minute
                    unsigned int end_hour;      // End Hour
                    unsigned int end_minute;    // End Minute
                    signed int start_lon;       // Start Longitude
                    signed int start_lat;       // Start Latitude
                    signed int end_lon;         // End Longitude
                    signed int end_lat;         // End Latitude
                    unsigned int type;          // Type
#define DAC200FID23_TYPE_UNKNOWN        0
                    signed int min;             // Min value
#define DAC200FID23_MIN_UNKNOWN         255
                    signed int max;             // Max value
#define DAC200FID23_MAX_UNKNOWN         255
                    unsigned int intensity;     // Classification
#define DAC200FID23_CLASS_UNKNOWN       0
                    unsigned int wind;          // Wind Direction
#define DAC200FID23_WIND_UNKNOWN        0
                } dac200fid23;
                struct {
                    char country[2+1];          // UN Country Code
                    signed int ngauges;
                    struct gauge_t {
                        unsigned int id;        // Gauge ID
#define DAC200FID24_GAUGE_ID_UNKNOWN    0
                        signed int level;       // Water Level
#define DAC200FID24_GAUGE_LEVEL_UNKNOWN 0
                    } gauges[4];
                } dac200fid24;
                struct {
                    signed int lon;             // Signal Longitude
                    signed int lat;             // Signal Latitude
                    unsigned int form;          // Signal form
#define DAC200FID40_FORM_UNKNOWN        0
                    unsigned int facing;        // Signal orientation
#define DAC200FID40_FACING_UNKNOWN      0
                    unsigned int direction;     // Direction of impact
#define DAC200FID40_DIRECTION_UNKNOWN   0
                    unsigned int status;        // Light Status
#define DAC200FID40_STATUS_UNKNOWN      0
                } dac200fid40;
                /* IMO236  - Meteorological-Hydrological data
                 * Trial message, not to be used after January 2013
                 * Replaced by IMO289 (DAC 1, FID 31)
                 */
                struct {
#define DAC1FID11_LATLON_SCALE          1000
                    int lon;                    // longitude in minutes * .001
#define DAC1FID11_LON_NOT_AVAILABLE     0xFFFFFF
                    int lat;                    // latitude in minutes * .001
#define DAC1FID11_LAT_NOT_AVAILABLE     0x7FFFFF
                    unsigned int day;           // UTC day
                    unsigned int hour;          // UTC hour
                    unsigned int minute;        // UTC minute
                    unsigned int wspeed;        // average wind speed
                    unsigned int wgust;         // wind gust
#define DAC1FID11_WSPEED_NOT_AVAILABLE  127
                    unsigned int wdir;          // wind direction
                    unsigned int wgustdir;      // wind gust direction
#define DAC1FID11_WDIR_NOT_AVAILABLE    511
                    unsigned int airtemp;       // temperature, units 0.1C
#define DAC1FID11_AIRTEMP_NOT_AVAILABLE 2047
#define DAC1FID11_AIRTEMP_OFFSET        600
#define DAC1FID11_AIRTEMP_DIV           10.0
                    unsigned int humidity;      // relative humidity, %
#define DAC1FID11_HUMIDITY_NOT_AVAILABLE 127
                    unsigned int dewpoint;      // dew point, units 0.1C
#define DAC1FID11_DEWPOINT_NOT_AVAILABLE 1023
#define DAC1FID11_DEWPOINT_OFFSET        200
#define DAC1FID11_DEWPOINT_DIV          10.0
                    unsigned int pressure;      // air pressure, hpa
#define DAC1FID11_PRESSURE_NOT_AVAILABLE 511
#define DAC1FID11_PRESSURE_OFFSET        -800
                    unsigned int pressuretend;  // tendency
#define DAC1FID11_PRESSURETREND_NOT_AVAILABLE 3
                    unsigned int visibility;    // units 0.1 nautical miles
#define DAC1FID11_VISIBILITY_NOT_AVAILABLE  255
#define DAC1FID11_VISIBILITY_DIV            10.0
                    int waterlevel;             // decimeters
#define DAC1FID11_WATERLEVEL_NOT_AVAILABLE  511
#define DAC1FID11_WATERLEVEL_OFFSET         100
#define DAC1FID11_WATERLEVEL_DIV            10.0
                    unsigned int leveltrend;    // water level trend code
#define DAC1FID11_WATERLEVELTREND_NOT_AVAILABLE 3
                    unsigned int cspeed;  // surface current speed in deciknots
#define DAC1FID11_CSPEED_NOT_AVAILABLE     255
#define DAC1FID11_CSPEED_DIV               10.0
                    unsigned int cdir;    // surface current dir., degrees
#define DAC1FID11_CDIR_NOT_AVAILABLE       511
                    unsigned int cspeed2;       // current speed in deciknots
                    unsigned int cdir2;         // current dir., degrees
                    unsigned int cdepth2;       // measurement depth, m
#define DAC1FID11_CDEPTH_NOT_AVAILABLE     31
                    unsigned int cspeed3;       // current speed in deciknots
                    unsigned int cdir3;         // current dir., degrees
                    unsigned int cdepth3;       // measurement depth, m
                    unsigned int waveheight;    // in decimeters
#define DAC1FID11_WAVEHEIGHT_NOT_AVAILABLE 255
#define DAC1FID11_WAVEHEIGHT_DIV           10.0
                    unsigned int waveperiod;    // in seconds
#define DAC1FID11_WAVEPERIOD_NOT_AVAILABLE 63
                    unsigned int wavedir;       // direction in degrees
#define DAC1FID11_WAVEDIR_NOT_AVAILABLE    511
                    unsigned int swellheight;   // in decimeters
                    unsigned int swellperiod;   // in seconds
                    unsigned int swelldir;      // direction in degrees
                    unsigned int seastate;      // Beaufort scale, 0-12
#define DAC1FID11_SEASTATE_NOT_AVAILABLE   15
                    unsigned int watertemp;     // units 0.1deg Celsius
#define DAC1FID11_WATERTEMP_NOT_AVAILABLE  1023
#define DAC1FID11_WATERTEMP_OFFSET         100
#define DAC1FID11_WATERTEMP_DIV         10.0
                    unsigned int preciptype;    // 0-7, enumerated
#define DAC1FID11_PRECIPTYPE_NOT_AVAILABLE  7
                    unsigned int salinity;      // units of 0.1ppt
#define DAC1FID11_SALINITY_NOT_AVAILABLE   511
#define DAC1FID11_SALINITY_DIV          10.0
                    unsigned int ice;           // is there sea ice?
#define DAC1FID11_ICE_NOT_AVAILABLE        3
                } dac1fid11;
                // IMO236 - Fairway Closed
                struct {
                    char reason[20+1];          // Reason For Closing
                    char closefrom[20+1];       // Location Of Closing From
                    char closeto[20+1];         // Location of Closing To
                    unsigned int radius;        // Radius extension
#define AIS_DAC1FID13_RADIUS_NOT_AVAILABLE 10001
                    unsigned int extunit;       // Unit of extension
#define AIS_DAC1FID13_EXTUNIT_NOT_AVAILABLE 0
                    unsigned int fday;          // From day (UTC)
                    unsigned int fmonth;        // From month (UTC)
                    unsigned int fhour;         // From hour (UTC)
                    unsigned int fminute;       // From minute (UTC)
                    unsigned int tday;          // To day (UTC)
                    unsigned int tmonth;        // To month (UTC)
                    unsigned int thour;         // To hour (UTC)
                    unsigned int tminute;       // To minute (UTC)
                } dac1fid13;
                // IMO236 - Extended ship and voyage data
                struct {
                    unsigned int airdraught;    // Air Draught
                } dac1fid15;
                // IMO286 - Number of Persons on board
                struct {
                    unsigned persons;           // number of persons
                } dac1fid16;
                // IMO289 - VTS-generated/Synthetic Targets
                struct {
                    signed int ntargets;
                    struct target_t {
#define DAC1FID17_IDTYPE_MMSI           0
#define DAC1FID17_IDTYPE_IMO            1
#define DAC1FID17_IDTYPE_CALLSIGN       2
#define DAC1FID17_IDTYPE_OTHER          3
                        unsigned int idtype;    // Identifier type
                        union target_id {       // Target identifier
                            unsigned int mmsi;
                            unsigned int imo;
#define DAC1FID17_ID_LENGTH             7
                            // cppcheck-suppress arrayIndexOutOfBounds
                            char callsign[DAC1FID17_ID_LENGTH+1];
                            char other[DAC1FID17_ID_LENGTH+1];
                        } id;
                        signed int lat;         // Latitude
                        signed int lon;         // Longitude
#define DAC1FID17_COURSE_NOT_AVAILABLE  360
                        unsigned int course;    // Course Over Ground
                        unsigned int second;    // Time Stamp
#define DAC1FID17_SPEED_NOT_AVAILABLE   255
                        unsigned int speed;     // Speed Over Ground
                    } targets[4];
                } dac1fid17;
                // IMO 289 - Marine Traffic Signal
                struct {
                    unsigned int linkage;       // Message Linkage ID
                    char station[20+1];         // Name of Signal Station
                    signed int lon;             // Longitude
                    signed int lat;             // Latitude
                    unsigned int status;        // Status of Signal
                    unsigned int signal;        // Signal In Service
                    unsigned int hour;          // UTC hour
                    unsigned int minute;        // UTC minute
                    unsigned int nextsignal;    // Expected Next Signal
                } dac1fid19;
                // IMO289 - Route info (broadcast)
                struct route_info dac1fid27;
                // IMO289 - Text message (broadcast)
                struct {
                    unsigned int linkage;
#define AIS_DAC1FID29_TEXT_MAX  162     // 920 bits of six-bit, plus NUL
                    char text[AIS_DAC1FID29_TEXT_MAX];
                } dac1fid29;
                // IMO289 - Meteorological-Hydrological data
                struct {
                    bool accuracy;      // position accuracy, <10m if true
#define DAC1FID31_LATLON_SCALE  1000
                    int lon;            // longitude in minutes * .001
#define DAC1FID31_LON_NOT_AVAILABLE     (181*60*DAC1FID31_LATLON_SCALE)
                    int lat;            // longitude in minutes * .001
#define DAC1FID31_LAT_NOT_AVAILABLE     (91*60*DAC1FID31_LATLON_SCALE)
                    unsigned int day;           // UTC day
                    unsigned int hour;          // UTC hour
                    unsigned int minute;        // UTC minute
                    unsigned int wspeed;        // average wind speed
                    unsigned int wgust;         // wind gust
#define DAC1FID31_WIND_HIGH              126
#define DAC1FID31_WIND_NOT_AVAILABLE     127
                    unsigned int wdir;          // wind direction
                    unsigned int wgustdir;      // wind gust direction
#define DAC1FID31_DIR_NOT_AVAILABLE      360
                    int airtemp;                // temperature, units 0.1C
#define DAC1FID31_AIRTEMP_NOT_AVAILABLE  -1024
#define DAC1FID31_AIRTEMP_DIV            10.0
                    unsigned int humidity;      // relative humidity, %
#define DAC1FID31_HUMIDITY_NOT_AVAILABLE 101
                    int dewpoint;               // dew point, units 0.1C
#define DAC1FID31_DEWPOINT_NOT_AVAILABLE 501
#define DAC1FID31_DEWPOINT_DIV          10.0
                    unsigned int pressure;      // air pressure, hpa
#define DAC1FID31_PRESSURE_NOT_AVAILABLE 511
#define DAC1FID31_PRESSURE_HIGH          402
#define DAC1FID31_PRESSURE_OFFSET        -799
                    unsigned int pressuretend;  // tendency
#define DAC1FID31_PRESSURETEND_NOT_AVAILABLE  3
                    bool visgreater;            // visibility greater than
                    unsigned int visibility;    // units 0.1 nautical miles
#define DAC1FID31_VISIBILITY_NOT_AVAILABLE  127
#define DAC1FID31_VISIBILITY_DIV            10.0
                    int waterlevel;             // cm
#define DAC1FID31_WATERLEVEL_NOT_AVAILABLE 4001
#define DAC1FID31_WATERLEVEL_OFFSET        1000
#define DAC1FID31_WATERLEVEL_DIV           100.0
                    unsigned int leveltrend;    // water level trend code
#define DAC1FID31_WATERLEVELTREND_NOT_AVAILABLE 3
                    unsigned int cspeed;        // current speed in deciknots
#define DAC1FID31_CSPEED_NOT_AVAILABLE     255
#define DAC1FID31_CSPEED_DIV               10.0
                    unsigned int cdir;          // current dir., degrees
                    unsigned int cspeed2;       // current speed in deciknots
                    unsigned int cdir2;         // current dir., degrees
                    unsigned int cdepth2;       // measurement depth, 0.1m
#define DAC1FID31_CDEPTH_NOT_AVAILABLE     301
#define DAC1FID31_CDEPTH_SCALE             10.0
                    unsigned int cspeed3;       // current speed in deciknots
                    unsigned int cdir3;         // current dir., degrees
                    unsigned int cdepth3;       // measurement depth, 0.1m
                    unsigned int waveheight;    // in decimeters
#define DAC1FID31_HEIGHT_NOT_AVAILABLE     31
#define DAC1FID31_HEIGHT_DIV               10.0
                    unsigned int waveperiod;    // in seconds
#define DAC1FID31_PERIOD_NOT_AVAILABLE     63
                    unsigned int wavedir;       // direction in degrees
                    unsigned int swellheight;   // in decimeters
                    unsigned int swellperiod;   // in seconds
                    unsigned int swelldir;      // direction in degrees
                    unsigned int seastate;      // Beaufort scale, 0-12
#define DAC1FID31_SEASTATE_NOT_AVAILABLE   15
                    int watertemp;              // units 0.1deg Celsius
#define DAC1FID31_WATERTEMP_NOT_AVAILABLE  601
#define DAC1FID31_WATERTEMP_DIV         10.0
                    unsigned int preciptype;    // 0-7, enumerated
#define DAC1FID31_PRECIPTYPE_NOT_AVAILABLE 7
                    unsigned int salinity;     // units of 0.1 permil (ca. PSU)
#define DAC1FID31_SALINITY_NOT_AVAILABLE   510
#define DAC1FID31_SALINITY_DIV          10.0
                    unsigned int ice;           // is there sea ice?
#define DAC1FID31_ICE_NOT_AVAILABLE       3
                } dac1fid31;
            };
        } type8;
        // Type 9 - Standard SAR Aircraft Position Report
        struct {
            unsigned int alt;           // altitude in meters
#define AIS_ALT_NOT_AVAILABLE   4095
#define AIS_ALT_HIGH            4094    // 4094 meters or higher
            unsigned int speed;         // speed over ground in deciknots
#define AIS_SAR_SPEED_NOT_AVAILABLE     1023
#define AIS_SAR_FAST_MOVER      1022
            bool accuracy;              // position accuracy
            int lon;                    // longitude
            int lat;                    // latitude
            unsigned int course;        // course over ground
            /* seconds of UTC time, 0 to 59.
             * 60 == N/A, 61 == manual, 62 == dead reckoning,
             * 63 == inoperative */
            unsigned int second;        // seconds of UTC time
            unsigned int regional;      // regional reserved
            unsigned int dte;           // data terminal enable
            // unsigned int spare;      // spare bits
            bool assigned;              // assigned-mode flag
            bool raim;                  // RAIM flag
            unsigned int radio;         // radio status bits
        } type9;
        // Type 10 - UTC/Date Inquiry
        struct {
            // unsigned int spare;
            unsigned int dest_mmsi;     // destination MMSI
            // unsigned int spare2;
        } type10;
        // Type 12 - Safety-Related Message
        struct {
            unsigned int seqno;         // sequence number
            unsigned int dest_mmsi;     // destination MMSI
            bool retransmit;            // retransmit flag
            // unsigned int spare;      // spare bit(s)
#define AIS_TYPE12_TEXT_MAX     157     // 936 bits of six-bit, plus NUL
            char text[AIS_TYPE12_TEXT_MAX];
        } type12;
        // Type 14 - Safety-Related Broadcast Message
        struct {
            // unsigned int spare;      // spare bit(s)
#define AIS_TYPE14_TEXT_MAX     161     // 952 bits of six-bit, plus NUL
            char text[AIS_TYPE14_TEXT_MAX];
        } type14;
        // Type 15 - Interrogation
        struct {
            // unsigned int spare;      // spare bit(s)
            unsigned int mmsi1;
            unsigned int type1_1;
            unsigned int offset1_1;
            // unsigned int spare2;     // spare bit(s)
            unsigned int type1_2;
            unsigned int offset1_2;
            // unsigned int spare3;     // spare bit(s)
            unsigned int mmsi2;
            unsigned int type2_1;
            unsigned int offset2_1;
            // unsigned int spare4;     // spare bit(s)
        } type15;
        // Type 16 - Assigned Mode Command
        struct {
            // unsigned int spare;      // spare bit(s)
            unsigned int mmsi1;
            unsigned int offset1;
            unsigned int increment1;
            unsigned int mmsi2;
            unsigned int offset2;
            unsigned int increment2;
        } type16;
        // Type 17 - GNSS Broadcast Binary Message
        struct {
            // unsigned int spare;      // spare bit(s)
#define AIS_GNSS_LATLON_DIV     600.0
            int lon;                    // longitude
            int lat;                    // latitude
            // unsigned int spare2;     // spare bit(s)
#define AIS_TYPE17_BINARY_MAX   736     // 920 bits
            size_t bitcount;            // bit count of the data
            char bitdata[(AIS_TYPE17_BINARY_MAX + 7) / 8];
        } type17;
        // Type 18 - Standard Class B CS Position Report
        struct {
            unsigned int reserved;      // altitude in meters
            unsigned int speed;         // speed over ground in deciknots
            bool accuracy;              // position accuracy
            int lon;                    // longitude
#define AIS_GNS_LON_NOT_AVAILABLE       0x1a838
            int lat;                    // latitude
#define AIS_GNS_LAT_NOT_AVAILABLE       0xd548
            unsigned int course;        // course over ground
            unsigned int heading;       // true heading
            /* seconds of UTC time, 0 to 59.
             * 60 == N/A, 61 == manual, 62 == dead reckoning,
             * 63 == inoperative */
            unsigned int second;
            unsigned int regional;      // regional reserved
            bool cs;                    // carrier sense unit flag
            bool display;               // unit has attached display?
            bool dsc;                   // unit attached to radio with DSC?
            bool band;                  // unit can switch frequency bands?
            bool msg22;                 // can accept Message 22 management?
            bool assigned;              // assigned-mode flag
            bool raim;                  // RAIM flag
            unsigned int radio;         // radio status bits
        } type18;
        // Type 19 - Extended Class B CS Position Report
        struct {
            unsigned int reserved;      // altitude in meters
            unsigned int speed;         // speed over ground in deciknots
            bool accuracy;              // position accuracy
            int lon;                    // longitude
            int lat;                    // latitude
            unsigned int course;        // course over ground
            unsigned int heading;       // true heading
            /* seconds of UTC time, 0 to 59.
             * 60 == N/A, 61 == manual, 62 == dead reckoning,
             * 63 == inoperative */
            unsigned int second;
            unsigned int regional;      // regional reserved
            // cppcheck-suppress arrayIndexOutOfBounds
            char shipname[AIS_SHIPNAME_MAXLEN+1];               // ship name
            unsigned int shiptype;      // ship type code
            unsigned int to_bow;        // dimension to bow
            unsigned int to_stern;      // dimension to stern
            unsigned int to_port;       // dimension to port
            unsigned int to_starboard;  // dimension to starboard
            unsigned int epfd;          // type of position fix device
            bool raim;                  // RAIM flag
            unsigned int dte;           // date terminal enable
            bool assigned;              // assigned-mode flag
            // unsigned int spare;       spare bits
        } type19;
        // Type 20 - Data Link Management Message
        struct {
            // unsigned int spare;       spare bit(s)
            unsigned int offset1;       // TDMA slot offset
            unsigned int number1;       // number of xlots to allocate
            unsigned int timeout1;      // allocation timeout
            unsigned int increment1;    // repeat increment
            unsigned int offset2;       // TDMA slot offset
            unsigned int number2;       // number of xlots to allocate
            unsigned int timeout2;      // allocation timeout
            unsigned int increment2;    // repeat increment
            unsigned int offset3;       // TDMA slot offset
            unsigned int number3;       // number of xlots to allocate
            unsigned int timeout3;      // allocation timeout
            unsigned int increment3;    // repeat increment
            unsigned int offset4;       // TDMA slot offset
            unsigned int number4;       // number of xlots to allocate
            unsigned int timeout4;      // allocation timeout
            unsigned int increment4;    // repeat increment
        } type20;
        // Type 21 - Aids to Navigation Report
        struct {
            unsigned int aid_type;      // aid type
            char name[35];              // name of aid to navigation
            bool accuracy;              // position accuracy
            int lon;                    // longitude
            int lat;                    // latitude
            unsigned int to_bow;        // dimension to bow
            unsigned int to_stern;      // dimension to stern
            unsigned int to_port;       // dimension to port
            unsigned int to_starboard;  // dimension to starboard
            unsigned int epfd;          // type of EPFD
            /* seconds of UTC time, 0 to 59.
             * 60 == N/A, 61 == manual, 62 == dead reckoning,
             * 63 == inoperative */
            unsigned int second;
            bool off_position;          // off-position indicator
            unsigned int regional;      // regional reserved field
            bool raim;                  // RAIM flag
            bool virtual_aid;           // is virtual station?
            bool assigned;              // assigned-mode flag
            // unsigned int spare;       unused
        } type21;
        // Type 22 - Channel Management
        struct {
            // unsigned int spare;       spare bit(s)
            unsigned int channel_a;     // Channel A number
            unsigned int channel_b;     // Channel B number
            unsigned int txrx;          // transmit/receive mode
            bool power;                 // high-power flag
#define AIS_CHANNEL_LATLON_DIV  600.0
            union {
                struct {
                    int ne_lon;         // NE corner longitude
                    int ne_lat;         // NE corner latitude
                    int sw_lon;         // SW corner longitude
                    int sw_lat;         // SW corner latitude
                } area;
                struct {
                    unsigned int dest1; // addressed station MMSI 1
                    unsigned int dest2; // addressed station MMSI 2
                } mmsi;
            };
            bool addressed;             // addressed vs. broadcast flag
            bool band_a;                // fix 1.5kHz band for channel A
            bool band_b;                // fix 1.5kHz band for channel B
            unsigned int zonesize;      // size of transitional zone
        } type22;
        // Type 23 - Group Assignment Command
        struct {
            int ne_lon;                 // NE corner longitude
            int ne_lat;                 // NE corner latitude
            int sw_lon;                 // SW corner longitude
            int sw_lat;                 // SW corner latitude
            // unsigned int spare;       spare bit(s)
            unsigned int stationtype;   // station type code
            unsigned int shiptype;      // ship type code
            //unsigned int spare2;      spare bit(s)
            unsigned int txrx;          // transmit-enable code
            unsigned int interval;      // report interval
            unsigned int quiet;         // quiet time
            // unsigned int spare3;      spare bit(s)
        } type23;
        // Type 24 - Class B CS Static Data Report
        struct {
            char shipname[AIS_SHIPNAME_MAXLEN+1];       // vessel name
            enum {
                both,
                part_a,
                part_b,
            } part;
            unsigned int shiptype;      // ship type code
            char vendorid[8];           // vendor ID
            unsigned int model;         // unit model code */
            unsigned int serial;        // serial number */
            char callsign[8];           // callsign */
            union {
                unsigned int mothership_mmsi;   // MMSI of main vessel
                struct {
                    unsigned int to_bow;        // dimension to bow
                    unsigned int to_stern;      // dimension to stern
                    unsigned int to_port;       // dimension to port
                    unsigned int to_starboard;  // dimension to starboard
                } dim;
            };
        } type24;
        // Type 25 - Addressed Binary Message
        struct {
            bool addressed;             // addressed-vs.broadcast flag
            bool structured;            // structured-binary flag
            unsigned int dest_mmsi;     // destination MMSI
            unsigned int app_id;        // Application ID
#define AIS_TYPE25_BINARY_MAX   128     // Up to 128 bits
            size_t bitcount;            // bit count of the data
            char bitdata[(AIS_TYPE25_BINARY_MAX + 7) / 8];
        } type25;
        // Type 26 - Addressed Binary Message
        struct {
            bool addressed;             // addressed-vs.broadcast flag
            bool structured;            // structured-binary flag
            unsigned int dest_mmsi;     // destination MMSI
            unsigned int app_id;        // Application ID
#define AIS_TYPE26_BINARY_MAX   1004    // Up to 128 bits
            size_t bitcount;            // bit count of the data
            char bitdata[(AIS_TYPE26_BINARY_MAX + 7) / 8];
            unsigned int radio;         // radio status bits
        } type26;
        // Type 27 - Long Range AIS Broadcast message
        struct {
            bool accuracy;              // position accuracy
            bool raim;                  // RAIM flag
            unsigned int status;        // navigation status
#define AIS_LONGRANGE_LATLON_DIV        600.0
            int lon;                    // longitude
#define AIS_LONGRANGE_LON_NOT_AVAILABLE 0x1a838
            int lat;                    // latitude
#define AIS_LONGRANGE_LAT_NOT_AVAILABLE 0xd548
            unsigned int speed;         // speed over ground in deciknots
#define AIS_LONGRANGE_SPEED_NOT_AVAILABLE 63
            unsigned int course;        // course over ground
#define AIS_LONGRANGE_COURSE_NOT_AVAILABLE 511
            bool gnss;                  // are we reporting GNSS position?
        } type27;
    };
};


// basic data, per PRN, from GPGSA and GPGSV, or GPS binary messages
// Note: u-blox 9 no longer uses PRN
struct satellite_t {
    /* SNR. signal-to-noise ratio, 0 to 254 dB, u-blox can be 0 to 63.
     * -1 for n/a */
    double ss;
    /* PRN of this satellite, 1 to 437, 0 for n/a
     * sadly there is no standard, but many different implementations of
     * how to code PRN
     */
    int16_t PRN;          // PRN numbering per NMEA 2.x to 4.0, not 4.10
    double elevation;     // elevation of satellite, -90 to 90 deg, NAN for n/a
    double azimuth;       // azimuth, 0 to 359 deg, NAN1 for n/a
    double prRes;         // Pseudorange residual, meters
    double prRate;        // Pseudorange Rate of change, m/ss
    double pr;            // Pseudorange meters
    bool used;            // this satellite used in solution
    /* Quality Indicator
     * -1 = invalid, ignore
     * 0 = no signal
     * 1 = searching signal
     * 2 = signal acquired
     * 3 = signal detected but unusable
     * 4 = code locked and time synchronized
     * 5, 6, 7 = code and carrier locked and time synchronized
     */
    int8_t qualityInd;
    /* gnssid:svid:sigid, as defined by u-blox 8/9:
     *  gnssid        svid (native PRN)
     *  0 = GPS           1-32
     *  1 = SBAS          120-158
     *  2 = Galileo       1-36
     *  3 - BeiDou        1-63
     *  4 = IMES          1-10
     *  5 = QZSS          1-10       F10
     *  6 = GLONASS       1-32, 255
     *  7 = NavIC (IRNSS) 1-11       ZED-F9T
     *
     * gnssid:svid:sigid, as defined by NMEA 4.11, NOT USED HERE!
     *  1 = GPS           1-32
     *  1 = SBAS          33-64, 152-158
     *  1 = QZSS          193-202  (u-blox extended NMEA 4.10)
     *  2 = GLONASS       65-96, null
     *  3 = Galileo       1-36
     *  4 - BeiDou        1-63
     *  5 = QZSS          1-10            (NMEA 4.11)
     *  6 = NavIC (IRNSS) 1-14             NMEA 4.11+
     *  x = IMES          173-182 (u-blox extended NMEA 3.10)
     *
     * Note: other GNSS receivers use different mappings!
     */
    uint8_t gnssid;
// defines for u-blox gnssId, as used in satellite_t
#define GNSSID_GPS 0
#define GNSSID_SBAS 1
#define GNSSID_GAL 2
#define GNSSID_BD 3
#define GNSSID_IMES 4
#define GNSSID_QZSS 5
#define GNSSID_GLO 6
#define GNSSID_IRNSS 7            // ZED-F9T
#define GNSSID_CNT 8              // count for array size

    // ignore gnssid and sigid if svid is zero
    uint8_t svid;
    /* sigid as defined by u-blox 9/10, and used here
     * also see sigid2str().
     * BeiDou:   0 = B1I D1, 1 = B1I D2, 2 = B2I D1, 3 = B2I D2, 4 = B3I D1,
     *           5 = B1 Cp,  6 = B1 Cd,  7 = B2 ap, 8 = B2 ad, 10 = B3I D2
     * Galileo:  0 = E1 C, 1 = E1 B, 3 = E5 aI, 4 = E5 aQ, 5 = E5 bl,
     *           6 = E5 bQ, 8 = E6 B, 9 = E6 C, 10 = E6 A
     *           Quectel L1-A s/b what?
     * GLONASS:  0 = L1 OF, 2 = L2 OF
     * GPS:      0 = L1C/A, 3 = L2 CL, 4 = L2 CM, 6 = L5 I, 7 = L5 Q
     * IRNSS:    0 = L5 A
     * QZSS:     0 = L1C/A, 1 = L1 S, 4 = L2 CM, 5 = L2 CL, 8 = L5 I,
     *           9 = L5 Q, 12 = L1 C/B
     * SBAS:     0 = L1C/A, ? = L5I
     *
     * sigid as defined by NMEA 4.10, according to Skytrak, NOT used here
     * BeiDou:
     *   1  B1I               // ALLYSTAR
     *   2  B2I               // ALLYSTAR
     *   3  B3I               // ALLYSTAR
     *   3  B1 Cp, B1 Cd      // NMEA 4.11
     *   4  B2A               // ALLYSTAR
     *   5  B5 ap, B2 ad      // NMEA 4.11
     *   8  B2I D1, B2I D1    // NMEA 4.11
     *   9  B1C               // ALLYSTAR
     *   11 B2I               // NMEA 4.11, according to u-blox
     * Galileo:
     *   0  All signals
     *   1  E5a  (aI and aQ)
     *   2  E5b  (bI and bQ)
     *   3  E5 a+b
     *   4  E6-A
     *   5  E6-BC
     *   6  L1-A               // Quectel LC79D ??
     *   7  L1-B and L1-C (E1-C and E1-B) (L1BC)
     * GLONASS:
     *   0  All signals-
     *   1  G1 C/A  (L1-OF)
     *   2  G1P
     *   3  G2 C/A  (L2 OF)
     *   4  GLONASS (M) G2P
     * GPS:
     *   0  All signals
     *   1  L1 C/A
     *   2  L1 P(Y)
     *   3  L1C
     *   4  L2 P(Y)
     *   5  L2C-M
     *   6  L2C-L   (L2CL)
     *   7  L5-I
     *   8  L5-Q
     *   9  L1C          // ALLYSTAR
     *   11 L6           // ALLYSTAR
     * Navic (IRNSS):
     *   1  ??           // ALLYSTAR
     *   4  L5
     * QZSS:     not defined
     *   1  L1C/A        // NMEA 4.11
     *   4  L1S          // NMEA 4.11
     *   5  L2CM         // NMEA 4.11
     *   6  L2CL         // NMEA 4.11
     *   7  L5 I         // NMEA 4.11
     *   8  L5 Q         // NMEA 4.11
     *
     * Additional sigid as defined by NMEA 4.11
     * BeiDou:
     *   According to u-blox
     *     1 = B1 D1, 1 = B1 D2, 5 = B2 a, 11(8) = B2 D1, 11(8) = B2 D2
     *   According to Skytrak
     *     0  All signals
     *     1  B1  (B1-d1 and B1-d2)
     *     2  B2A
     *     3  B2
     *     4  B3
     *     5  B1C
     * QZSS:     1 = L1C/A
     */
    uint8_t sigid;
    int8_t freqid;              // The GLONASS (Only) frequency, 0 - 13
    uint8_t health;             // 0 = unknown, 1 = healthy, 2 = unhealthy
#define SAT_HEALTH_UNK 0
#define SAT_HEALTH_OK 1
#define SAT_HEALTH_BAD 2

};

/* attitude_t was originally for real IMUs that are synchronous
 * to the GNSS epoch.  Skytrak introduced a "moving base/rover"
 * that is used as a "GNSS Compass".  Essentially a synthetic
 * IMU.  To support this, related data (baseline_t) is also here.
 */
struct attitude_t {
    timespec_t  mtime;  // time of measurement
    // arbitrary time tag (see UBX-ESF-MEAS), 32 bit unsigned
    unsigned long timeTag;
    /* source message name.
     * Used to disambiguate UBX-ESF-RAW and UBX-ESF-MEAS
     * Also used to mark this struct as used
     */
    char msg[16];
    double acc_len;     // unitvector sqrt(x^2 + y^2 +z^2)
    // u-blox, acc_X ==  24 bit signed / 1024
    double acc_x;       // x-axis acceleration (m/s^2)
    double acc_y;       // y-axis acceleration (m/s^2)
    double acc_z;       // x-axis acceleration (m/s^2)
    double depth;
    double dip;
    // u-blox, gyro_temp ==  24 bit signed / 100
    double gyro_temp;   // deg C
    // u-blox, gyro_X ==  24 bit signed / 4096
    double gyro_x;      // deg/s^2
    double gyro_y;      // deg/s^2
    double gyro_z;      // deg/s^2
    double heading;     // true heading
    double mheading;    // magnetic heading
    double mag_len;     // unitvector sqrt(x^2 + y^2 +z^2)
    double mag_x;
    double mag_y;
    double mag_z;
    double pitch;       // deg
    double roll;        // deg
    double rot;         // rate of turn.  degrees / minute
    double temp;        // deg C
    double yaw;         // deg
    // compass status -- TrueNorth (and any similar) devices only
    char mag_st;
    char pitch_st;
    char roll_st;
    char yaw_st;
    struct baseline_t base;  // baseline from moving base
};

struct dop_t {
    // Dilution of precision factors, dimensionless.
    double xdop;       // latitude, easting
    double ydop;       // longitude, northing
    double pdop;       // position, scpherical, 3D
    double hdop;       // horizontal, circular, 2D
    double vdop;       // vertical, altitude
    double tdop;       // time
    double gdop;       // geometric, hyperspherical, 3D 
};

struct rawdata_t {
    // raw measurement data, suitable for RINEX 3
    timespec_t mtime;           /* time of measurement: sec, nsec
                                 * Note: GPS time, not UTC time */
    struct meas_t {
        // gnssid see satellite_t for decode
        unsigned char gnssid;
        // svid see RINEX 3 for decode, not satellite_t
        unsigned char svid;
        // sigid see satellite_t for decode
        unsigned char sigid;
        // SNR.  0 to 100 dB-Hz.  u-blox can be 0 to 63.
        unsigned char snr;
        unsigned char freqid;   // The GLONASS (Only) frequency, 0 - 13
        unsigned char lli;      /* RINEX Loss of Lock Indicator
                                 * bit 0 - Lost Lock
                                 * bit 1 - half-cycle ambiguity/slip possible
                                 * bit 2 - GALILEO BOC-tracking of MBOC signal
                                 */
        char obs_code[4];       // 3 char RINEX observation code
        /* see RINEX documentation
         * GPS: L1: L1C, L1S, L1L, L1X, L1P, L1W, L1N
         *      L2: L2C, L2D, L2S, L2L, L2X, L2P, L2W, L2N
         *      L5: L5I, L5Q
         * GLONASS: G1: L1C, L1P
         *          G2: L2C, L2P
         *          G3: L3I, L3Q, L3X
         * GALILEO: E1: L1B, L1C, L1X
         *          E5A: L5I, L5Ql L5X
         *          E5B: L7I, L7Q, L7X
         *          E5(A+B): L8I, L8Q, L8X
         *          E6: L6B, L6C, L6X
         * QZSS: L1: L1C, L1S, L1L, L1X, L1Z
         *       L2: L2S, L2L, L2X
         *       L5: L5I, L5Q, L5X
         *       LEX(6): L6S, L6L, L6X
         * BeiDou: B1: L2I, L2Q, L2X
         *         B2: L7I, L7Q, L7X
         *         B3: L6I, L6Q, L6X
         * IRNSS: L5: L5A, L5B, L5C, L5X
         *        S: L9A, L9B, L9C, L9X
         */
        double codephase;       // meters
        double carrierphase;    // L1 C/A meters, RINEX L1C
        double pseudorange;     // L1 C/A meters, RINEX C1C
        double deltarange;      // L1 C/A meters/sec, RINEX D1C
        double doppler;         // Hz
#define LOCKMAX         64500   // locktime capped at 64500
        unsigned locktime;      /* Carrier Phase Locktime in ms.
                                 * max 64,500 ms */
        double l2c;             // L2 C/A carrier phase meters, RINEX L2C
        double c2c;             // L2 C/A pseudo-range meters, RINEX C2C
        unsigned satstat;       // tracking status
#define SAT_ACQUIRED    0x01    // satellite acquired
#define SAT_CODE_TRACK  0x02    // code-tracking loop acquired
#define SAT_CARR_TRACK  0x04    // carrier-tracking loop acquired
#define SAT_DATA_SYNC   0x08    // data-bit synchronization done
#define SAT_FRAME_SYNC  0x10    // frame synchronization done
#define SAT_EPHEMERIS   0x20    // ephemeris collected
#define SAT_FIX_USED    0x40    // used for position fix
    } meas[MAXCHANNELS];
};

struct version_t {
    char release[64];                   // external version
    char rev[64];                       // internal revision ID
    int proto_major, proto_minor;       // API major and minor versions
    char remote[GPS_PATH_MAX];          // could be from a remote device
};

#define HEXDATA_MAX 512                 // hex encoded command buffer, max
struct devconfig_t {
    char path[GPS_PATH_MAX];
    int flags;
#define SEEN_GPS        0x01
#define SEEN_RTCM2      0x02
#define SEEN_RTCM3      0x04
#define SEEN_AIS        0x08
    char driver[64];
    // 96 too small for ZED-F9
    char subtype[128];           // maybe hardware version
    char subtype1[128];          // maybe software version
    char sernum[30];             // maybe serial number or uniq id
    // a buffer to hold data to output to GPS
    char hexdata[HEXDATA_MAX];
    timespec_t activated;
    unsigned int baudrate, stopbits;    // RS232 link parameters
    char parity;                        // 'N', 'O', or 'E'
    timespec_t cycle, mincycle;         // refresh cycle time in seconds
    int driver_mode;                    // is driver in native mode or not?
};

struct gps_policy_t {
    bool watcher;                       // is watcher mode on?
    bool json;                          // requesting JSON?
    bool nmea;                          // requesting dumping as NMEA?
    int raw;                            // requesting raw data?
    bool scaled;                        // requesting report scaling?
    bool timing;                        // requesting timing info
    bool split24;                       // requesting split AIS Type 24s
    bool pps;                           // requesting PPS in NMEA/raw modes
    // loglevel presently unused
    int loglevel;                       // requested log level of messages
    char devpath[GPS_PATH_MAX];         // specific device to watch
    // remote presently unused
    char remote[GPS_PATH_MAX];          // ...if this was passthrough
};

#ifndef TIMEDELTA_DEFINED
#define TIMEDELTA_DEFINED

struct timedelta_t {
    timespec_t  real;
    timespec_t  clock;
};
#endif  // TIMEDELTA_DEFINED

struct oscillator_t {
    bool running;                       // oscillator is running
    bool reference;                     // PPS reference is avai/able
    bool disciplined;                   // oscillator is GPS-dis/ipli/ed
    int delta;                          // last observed PPS del/a
};

/*
 * Someday we may support Windows, under which socket_t is a separate type.
 * In the meantime, having a typedef for this semantic kind is no bad thing,
 * as it makes clearer what some declarations are doing without breaking
 * binary compatibility.
 */
typedef intptr_t socket_t;
// BAD_SOCKET() needs to flag UNALLOCATED_FD (01) and PLACEHOLDING_FD (-2)
#define BAD_SOCKET(s)   (0 > (s))
#define INVALIDATE_SOCKET(s)    do { s = -1; } while (0)

// mode flags for setting streaming policy
typedef uint32_t watch_t;
#define WATCH_ENABLE    (watch_t)0x000001u       // enable streaming
#define WATCH_DISABLE   (watch_t)0x000002u       // disable watching
#define WATCH_READONLY  (watch_t)0x000004u       // read only (file input)
#define WATCH_JSON      (watch_t)0x000010u       // JSON output
#define WATCH_NMEA      (watch_t)0x000020u       // output in NMEA
#define WATCH_RARE      (watch_t)0x000040u       // output of packets in hex
#define WATCH_RAW       (watch_t)0x000080u       // output of raw packets
#define WATCH_SCALED    (watch_t)0x000100u       // scale output to floats
#define WATCH_TIMING    (watch_t)0x000200u       // timing information
#define WATCH_DEVICE    (watch_t)0x000800u       // watch specific device
#define WATCH_SPLIT24   (watch_t)0x001000u       // split AIS Type 24s
#define WATCH_PPS       (watch_t)0x002000u       // enable PPS JSON
#define WATCH_NEWSTYLE  (watch_t)0x010000u       // force JSON streaming


// describe a gpsd source
struct fixsource_t
{
    char spec[512];               // original string
    const char *server;           // server name, maybe IP
    const char *server_ip;        // server IP as string, maybe IPv4 or IPv6
    const char *port;
    const char *device;
};

// data buffers for reading files or sockets
struct gps_data_t;   // forward declaration of gpss_data_t;

struct privdata_t
{
    // data buffered from the last read
    ssize_t waiting;       // the number of bytes in the buffer
    char buffer[GPS_JSON_RESPONSE_MAX * 2];
    int waitcount;
    // DBus handler
    void (*handler)(struct gps_data_t *);
    // SHM handler
    void *shmseg;
    int tick;
};

#ifdef USE_QT
    /* we want this to be QTcpSocket *, but that requires QTcpSocket.h, which
     * requires a bunch of other includes, that require __cpplus, and that
     * gets weird fast. */
typedef void* gps_fd_t;
#else
    /* socket or file descriptor to GPS
     * POSIX says this is an int.
     * use socket_t, which is int, for windows compatibility */
typedef socket_t gps_fd_t;
#endif

/*
 * Main structure that includes all previous substructures
 */

struct gps_data_t {
    gps_mask_t set;     // has field been set since this was last cleared?
    // Not required, but a good idea if these match the list in gps/gps.py.in
#define ONLINE_SET      (1llu<<1)
#define TIME_SET        (1llu<<2)
#define TIMERR_SET      (1llu<<3)
#define LATLON_SET      (1llu<<4)
#define ALTITUDE_SET    (1llu<<5)
#define SPEED_SET       (1llu<<6)
#define TRACK_SET       (1llu<<7)
#define CLIMB_SET       (1llu<<8)
#define STATUS_SET      (1llu<<9)
#define MODE_SET        (1llu<<10)
#define DOP_SET         (1llu<<11)
#define HERR_SET        (1llu<<12)
#define VERR_SET        (1llu<<13)
#define ATTITUDE_SET    (1llu<<14)
#define SATELLITE_SET   (1llu<<15)
#define SPEEDERR_SET    (1llu<<16)
#define TRACKERR_SET    (1llu<<17)
#define CLIMBERR_SET    (1llu<<18)
#define DEVICE_SET      (1llu<<19)
#define DEVICELIST_SET  (1llu<<20)
#define DEVICEID_SET    (1llu<<21)
#define RTCM2_SET       (1llu<<22)
#define RTCM3_SET       (1llu<<23)
#define AIS_SET         (1llu<<24)
#define PACKET_SET      (1llu<<25)
#define SUBFRAME_SET    (1llu<<26)
#define GST_SET         (1llu<<27)
#define VERSION_SET     (1llu<<28)
#define POLICY_SET      (1llu<<29)
#define LOGMESSAGE_SET  (1llu<<30)
#define ERROR_SET       (1llu<<31)
#define TOFF_SET        (1llu<<32)      // not yet used
#define PPS_SET         (1llu<<33)
#define NAVDATA_SET     (1llu<<34)
#define OSCILLATOR_SET  (1llu<<35)
#define ECEF_SET        (1llu<<36)
#define VECEF_SET       (1llu<<37)
#define MAGNETIC_TRACK_SET (1llu<<38)
#define RAW_SET         (1llu<<39)
#define NED_SET         (1llu<<40)
#define VNED_SET        (1llu<<41)
#define LOG_SET         (1llu<<42)
#define IMU_SET         (1llu<<43)
#define EOF_SET         (1llu<<44)
#define SET_HIGH_BIT    45
    gps_mask_t set_pending;     // Deferred set, waiting to be sent.
    timespec_t online;          /* NZ if GPS is on line, 0 if not.
                                 *
                                 * Note: gpsd clears this time when sentences
                                 * fail to show up within the GPS's normal
                                 * send cycle time. If the host-to-GPS
                                 * link is lossy enough to drop entire
                                 * sentences, this field will be
                                 * prone to false zero values.
                                 */

    gps_fd_t gps_fd;

    /* the driver can call this to tell the user its fd
     * has changed.
     * This is useful for drivers that have a multi-stage
     * initialisation, which requires closing and opening
     * a connection to a remote server for example (NTRIP
     * does that), where the user will usually only know
     * about the initial fd (when the device is initially
     * opened).
     * Might be NULL for users that are not interested in
     * polling the device, so don't call without checking
     * the value first.
     * - fd: the fd to act on
     * - open: true is the fd is now open, false otherwise
     */
    void (*update_fd)(int fd, bool open);

    struct gps_fix_t    fix;    // accumulated PVT data
    struct gps_log_t    log;    // log data

    int leap_seconds;           // Unix secs to UTC (GPS-UTC offset)
    // precision of fix -- valid if satellites_used > 0
    int satellites_used;        // Number of satellites used in solution
    struct dop_t dop;

    // satellite status -- valid when satellites_visible > 0
    timespec_t skyview_time;    // skyview time
    int satellites_visible;     // # of satellites in view
    struct satellite_t skyview[MAXCHANNELS];

    struct devconfig_t dev;     // device that shipped last update

    struct gps_policy_t policy; // our listening policy

    struct {
        timespec_t time;
        int ndevices;
        struct devconfig_t list[MAXUSERDEVS];
    } devices;

    // pack things never reported together to reduce structure size
#define UNION_SET       (AIS_SET|ERROR_SET|GST_SET| \
                         LOGMESSAGE_SET|OSCILLATOR_SET|PPS_SET|RAW_SET| \
                         RTCM2_SET|RTCM3_SET|SUBFRAME_SET|TOFF_SET|VERSION_SET)

    struct gst_t gst;
    union {
        // unusual forms of sensor data that might come up the pipe
        struct rtcm2_t  rtcm2;
        struct rtcm3_t  rtcm3;
        struct subframe_t subframe;
        struct ais_t ais;
        struct rawdata_t raw;
        struct oscillator_t osc;
        // "artificial" structures for various protocol responses
        struct version_t version;
        char error[256];
    };
    /* attitude and imu are similar
     * attitude is synchronous to the GNSS epoch, and cumulative in the epoch
     * imus is async to the epoch, and sent immediately
     *
     */
    struct attitude_t attitude;
    // u-blox 8 seems to need 10 IMU for UBX-ESF-RAW
    struct attitude_t imu[10];

    // time stuff
    // FIXME! next lib rev need to add a place to put PPS precision
    struct timedelta_t toff;
    struct timedelta_t pps;
    // quantization error adjustment to PPS. aka "sawtooth" correction
    long qErr;                  // offset in picoseconds (ps)
    // time of PPS pulse that qErr applies to
    timespec_t qErr_time;
    struct fixsource_t source;    // source of the gpsd data
    watch_t watch;                // watch flags in use.

    // Private data - client code must not set this
    struct privdata_t *privdata;
};

extern int gps_open(const char *, const char *,
                      struct gps_data_t *);
extern int gps_close(struct gps_data_t *);
extern int gps_send(struct gps_data_t *, const char *, ... );
extern int gps_read(struct gps_data_t *, char *message, int message_len);
extern const char *gps_hexdump(char *, size_t, const unsigned char *, size_t);
extern ssize_t gps_hexpack(const char *, unsigned char *, size_t);
extern int gps_unpack(const char *, struct gps_data_t *);
extern bool gps_waiting(const struct gps_data_t *, int);
extern int gps_stream(struct gps_data_t *, watch_t, const char *);
extern int gps_mainloop(struct gps_data_t *, int,
                        void (*)(struct gps_data_t *));
extern const char *gps_data(const struct gps_data_t *);
extern const char *gps_errstr(const int);
extern char *gps_visibilize(char *outbuf, size_t outlen,
                            const char *inbuf, size_t inlen);

int json_toff_read(const char *buf, struct gps_data_t *,
                  const char **);
int json_pps_read(const char *buf, struct gps_data_t *,
                  const char **);
int json_oscillator_read(const char *buf, struct gps_data_t *,
                         const char **);

// dependencies on struct gps_data_t end here

extern void gpsd_source_spec(const char *fromstring,
                             struct fixsource_t *source);

extern void libgps_trace(int errlevel, const char *, ...);

extern void gps_clear_att(struct attitude_t *);
extern void gps_clear_dop( struct dop_t *);
extern void gps_clear_fix(struct gps_fix_t *);
extern void gps_clear_gst( struct gst_t *);
extern void gps_clear_log(struct gps_log_t *);
extern void gps_merge_fix(struct gps_fix_t *, gps_mask_t, struct gps_fix_t *);
extern void gps_enable_debug(int, FILE *);
extern const char *gps_maskdump(gps_mask_t);

// stuff from libgps/gpsutils.c

// a char list for char2str()
struct clist_t {
    unsigned char ch;
    const char *str;
};
// a flag list for flags2str()
struct flist_t {
    unsigned long val;
    unsigned long mask;
    const char *str;
};

// a value list for val2str()
struct vlist_t {
    unsigned long val;
    const char *str;
};
extern const struct vlist_t vant_status[];     // ant_status to string
extern const struct vlist_t vgnssId[];         // gnssId val to gnssId string
extern const struct vlist_t vmode_str[];       // mode val to mode string
extern const struct vlist_t vstatus_str[];     // status val to tatus string
extern const char *sigid2obs(unsigned char, unsigned char);
extern const char *sigid2str(unsigned char, unsigned char);

extern const char *char2str(unsigned char, const struct clist_t *);
extern const char *flags2str(unsigned long flags, const struct flist_t *flist,
                             char *buffer, size_t buflen);
extern const char *val2str(unsigned long val, const struct vlist_t *vlist);
extern double safe_atof(const char *);
extern time_t mkgmtime(struct tm *);
extern timespec_t iso8601_to_timespec(const char *);
extern char *now_to_iso8601(char[], size_t len);
extern char *timespec_to_iso8601(timespec_t t, char[], size_t len);
extern double earth_distance(double, double, double, double);
extern double earth_distance_and_bearings(double, double, double, double,
                                          double *,
                                          double *);
extern double wgs84_separation(double, double);
extern double mag_var(double, double);
extern void datum_code_string(int code, char *buffer, size_t len);
extern short ubx2_to_prn(int gnssId, int svId);

// some multipliers for interpreting GPS output

/* International Foot to Meters, exact
 * Note: not the same as the USA Survey Foot to meters:
 *   approximately: 0.304800609601.  exactly: (1200 / 3937)
 * Some states use the International Foot, not the USA Survey Foot */
#define FEET_TO_METERS  0.3048                   // intl feet to meters, exact
#define METERS_TO_FEET  (1 / FEET_TO_METERS)     // meters to intl feet, exact

#define MILES_TO_METERS 1.609344                  // Miles to meters, exact
#define METERS_TO_MILES (1 / MILES_TO_METERS)     // Meters to miles, exact
#define FATHOMS_TO_METERS 1.8288                  // Fathoms to meters, exact
#define METERS_TO_FATHOMS (1 / FATHOMS_TO_METER)  // Meters to fathoms, exact

// gpsd uses the international nautical mile, same as USA nautical mile
// different from UK nautical mile
#define KNOTS_TO_MPH    (1852 / 1609.344)   // Knots to miles per hour, exact
#define KNOTS_TO_KPH    1.852          // Knots to kilometers per hour, exact
// Knots to meters per second, exact, sort of.
// (double) needed to prevent non conforming CC from promoting to long double
#define KNOTS_TO_MPS (double)(KNOTS_TO_KPH / 3.6)
#define MPS_TO_KPH      3.6            // Meters per second to klicks/hr, exact
#define MPS_TO_MPH   (1 / 0.44704)    // Meters/second to miles per hour, exact
#define MPS_TO_KNOTS (3600.0 / 1852.0)     // Meters per second to knots, exact
// miles and knots are both the international standard versions of the units

// angle conversion multipliers
// IS-GPS-200, Galileo OS_SIS_ICD, BeiDou ICD_B1I, and QZSS IS-QZSS use
// pi = 3.1415926535898
// FIXME: use the proper pi
#define GPS_PI          3.1415926535897932384626433832795029
#define RAD_2_DEG       57.2957795130823208767981548141051703
#define DEG_2_RAD       0.0174532925199432957692369076848861271

// GLONASS uses one more digit than GPS in pi:
#define GLO_PI          3.14159265358979

// other mathematical constants
#define GPS_LN2         0.693147180559945309417232121458176568


// WGS84(G1674) defining parameters
/* https://en.wikipedia.org/wiki/Geodetic_datum
 * Section #World_Geodetic_System_1984_(WGS_84)
 *
 * http://www.unoosa.org/pdf/icg/2012/template/WGS_84.pdf
 */
#define WGS84A 6378137.0                // equatorial radius (semi-major axis)
#define WGS84F 298.257223563            // flattening
#define WGS84B 6356752.314245           // polar radius (semi-minor axis)
/* 1st eccentricity squared = (WGS84A ^ 2 - WGS84B ^ 2) / (WGS84A ^ 2)
 * precomputed so C does not recompute every time */
#define WGS84E 0.006694379990197585     // 1st eccentricity squared
/* 2nd eccentricity squared = ((WGS84A ^ 2 - WGS84B ^ 2) / (WGS84B ^ 2)
 * precomputed so C does not recompute every time */
#define WGS84E2 0.006739496742333464    // 2nd eccentricy squared

#define CLIGHT      299792458.0         // speed of light, in vacuum (m/s)
#define CLIGHTAIR   299702547.2360      // speed of light, in air (m/s)

// gpsd_open() and netlib_connectsock() error return values
#define NL_NOSERVICE    -1      // can't get service entry
#define NL_NOHOST       -2      // can't get host entry
#define NL_NOPROTO      -3      // can't get protocol entry
#define NL_NOSOCK       -4      // can't create socket
#define NL_NOSOCKOPT    -5      // error SETSOCKOPT SO_REUSEADDR
#define NL_NOCONNECT    -6      // can't connect to host/socket pair
#define SHM_NOSHARED    -7      // shared-memory segment not available
#define SHM_NOATTACH    -8      // shared-memory attach failed
#define DBUS_FAILURE    -9      // DBUS initialization failure
#define FILE_FAIL      -10      // failed to open file
#define SHM_CALLOC     -11      // calloc() fail

#define DEFAULT_GPSD_PORT       "2947"  // IANA assignment
#define DEFAULT_RTCM_PORT       "2101"  // IANA assignment

// special host values for non-socket exports
#define GPSD_DBUS_EXPORT        "DBUS export"
#define GPSD_LOCAL_FILE         "local file"
#define GPSD_SHARED_MEMORY      "shared memory"

#ifdef __cplusplus
}  // End of the 'extern "C"' block
#endif

#endif  // _GPSD_GPS_H_

// vim: set expandtab shiftwidth=4
