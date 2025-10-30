/* gpsd.h -- fundamental types and structures for the gpsd library
 *
 * Nothing in this file should be used by any client.  So safe to change
 * anything here without (directly) affecting the API or ABI.
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#ifndef _GPSD_H_
#define _GPSD_H_

#ifndef GPSD_CONFIG_H
    #error "Missing GPSD_CONFIG_H"
#endif

#include "compiler.h"   // Must be outside extern "C" for "atomic"

# ifdef __cplusplus
extern "C" {
# endif

#include <netinet/in.h>   // for sockaddr_in, sockaddr_in6
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#ifdef HAVE_WINSOCK2_H
#include <winsock2.h>      // for fd_set
#else  // !HAVE_WINSOCK2_H
#include <sys/select.h>    // for fd_set
#endif  // !HAVE_WINSOCK2_H
#include <sys/socket.h>   // for struct sockaddr
#include <time.h>    // for time_t

#include "gps.h"
#include "os_compat.h"
#include "ppsthread.h"
#include "timespec.h"

/*
 * Constants for the VERSION response
 * 3.1: Base JSON version
 * 3.2: Added POLL command and response
 * 3.3: AIS app_id split into DAC and FID
 * 3.4: Timestamps change from seconds since Unix epoch to ISO8601.
 * 3.5: POLL subobject name changes: fixes -> tpv, skyview -> sky.
 *      DEVICE::activated becomes ISO8601 rather than real.
 * 3.6  VERSION, WATCH, and DEVICES from slave gpsds get "remote" attribute.
 * 3.7  PPS message added to repertoire. SDDBT water depth reported as
 *      negative altitude with Mode 3 set.
 * 3.8  AIS course member becomes float in scaled mode (bug fix).
 * 3.9  split24 flag added. Controlled-vocabulary fields are now always
 *      dumped in both numeric and string form, with the string being the
 *      value of a synthesized additional attribute with "_text" appended.
 *      (Thus, the 'scaled' flag no longer affects display of these fields.)
 *      PPS drift message ships nsec rather than msec.
 * 3.10 The obsolete tag field has been dropped from JSON.
 * 3.11 A precision field, log2 of the time source jitter, has been added
 *      to the PPS report.  See ntpshm.h for more details.
 * 3.12 OSC message added to repertoire.
 * 3.13 gnssid:svid added to SAT
 *      time added to ATT
 * 3.19 Added RAW message class.
 *      Add cfg_stage and cfg_step, for initialization
 *      Add oldfix2 for better oldfix
 *      Make subtype longer
 *      Add ubx.protver, ubx.last_msgid and more to gps_device_t.ubx
 *      MAX_PACKET_LENGTH 516 -> 9216
 *      Add stuff to gps_device_t.nmea for NMEA 4.1
 * 3.20
 *      Remove TIMEHINT_ENABLE.  It only worked when enabled.
 *      Remove NTP_ENABLE and NTPSHM_ENABLE.  It only worked when enabled.
 *      Change gps_type_t.min_cycle from double to timespec_t
 *      Change gps_device_t.last_time from double to timespec_t
 *      Change gps_lexer_t.start_time from timestamp_t to timespec_t
 *      Change gps_context_t.gps_tow from double to timespec_t
 *      Change gps_device_t.sor from timestamp_t to timespec_t
 *      Change gps_device_t.this_frac_time, last_frac_time to timespec_t
 *      Change nmea.subseconds from double to timespec_t
 *      Remove gpsd_gpstime_resolve()
 *      Changed order of gpsd_log() arguments.  Add GPSD_LOG().
 *      Remove gps_device_t.back_to_nmea.
 *      Add fixed_port_speed, fixed_port_framing to gps_context_t.
 *      change tsip.superpkt from bool to int.
 *      Add tsip  .machine_id, .hardware_code, .last_tow, last_chan_seen
 *      Split gps_device_t.subtype into subtype and subtype1
 * 3.21
 *      GPSD_PROTO_*_VERSION moved to gpsd_config.h
 *      Add gps_context_t.passive
 *      Add gps_context_t.batteryRTC
 * 3.21.1
 *      Add gps_device_t.ubx.last_protver
 *      Add gps_device_t last_word_gal and last_svid3_gal
 *      Add timespec ts_startCurrentBaud to gps_device_t
 *      ntrip_conn_* to NTRIP_CONN_*
 * 3.24
 *      add ntrip_parse_url(), ntrip_close()
 *      add host, stream_time, to ntrip_stream_t
 *      add shm_clock_unit and shm_pps_unit to device_t
 *      add VALID_UNIT()
 *      remove shm_clock and shm_pps from device_t
 *      add pkt_time to lexer_t
 *      add netlib_connectsock1()
 *      add nmea.gsx_more to gps_device_t
 *      add TSIPv1 stuff
 * 3.25
 *      add end_gsv_talker and gga_sats_used to gps_device_t.nmea
 * 3.26
 *      add chunked to gps_lexer_t, for NTRIP 1.1 chunking.
 *      add packet_get1(), deprecate packet_get()
 *      change lexer_init() prototype
 *      Change RTCM_MAX to RTCM2_MAX to avoid confusiotn, and make it 4 longer.
 *      add shm_clock_lastsec and shm_pps_lastsec to gps_device_t;
 *      add queue, regression, to gps_device_t
 *      add ALL_PACKET
 * 3.27 WIP
 *      add SPARTN_PACKET
 *      gps_lexer_t, add type_mask, to mask our bad types, like SPARTN
 */

#define JSON_DATE_MAX   24      // ISO8601 timestamp with 2 decimal places

// be sure to change BUILD_LEAPSECONDS as needed.
#define BUILD_CENTURY 2000
#define BUILD_LEAPSECONDS 18

#ifndef DEFAULT_GPSD_SOCKET
#define DEFAULT_GPSD_SOCKET     RUNDIR "/gpsd.sock"
#endif

// Some internal capabilities depend on which drivers we're compiling.
#if !defined(AIVDM_ENABLE) && defined(NMEA2000_ENABLE)
#define AIVDM_ENABLE
#endif

#ifdef EARTHMATE_ENABLE
#define ZODIAC_ENABLE
#endif

#ifdef ISYNC_ENABLE
#define STASH_ENABLE
#endif

// First, declarations for the packet layer...

/*
 * NMEA 3.01, Section 5.3 says the max sentence length shall be
 * 82 chars, including the leading $ and terminating \r\n.
 *
 * For Shame:
 *  Some receivers (TN-200, GSW 2.3.2) emit oversized sentences.
 *  The Trimble BX-960 receiver emits a 91-character GGA message.
 *  The Skytraq S2525F8 which emits a 100-character PSTI message.
 *  The Skytraq PX1172RH which emits a 103-character PSTI message.
 *  The current hog champion is the Inertial Sense Inc (ISI) GRX-1 with a
 *  a 119-character $INFO message.
 */
#define NMEA_MAX        130             // max length of NMEA sentence
#define NMEA_MAX_FLD    100             // max fields in an NMEA sentence
#define NMEA_BIG_BUF    (2*NMEA_MAX+1)  // longer than longest NMEA sentence

// a few bits of ISGPS magic
enum isgpsstat_t {
    ISGPS_NO_SYNC, ISGPS_SYNC, ISGPS_SKIP, ISGPS_MESSAGE,
};

// 135 long packets have been seen in the wild.
#define RTCM2_MAX        ((RTCM2_WORDS_MAX * sizeof(isgps30bits_t)) + 4)
/*
 * RTCM3 is more variable length than RTCM 2.
 *
 * In theory, the limit is
 *   1 octet preamble
 *   2 octets payload length (first 6 bits reserved) -->
 *       max payload length 1023 octets
 *   0-1023 octets payload
 *   3 octets CRC
 *   1029 octets maximum
 *
 * Use 1040 out of paranoia
 */
#define RTCM3_MAX       1040

/*
 * The packet buffers need to be as long than the longest packet we
 * expect to see in any protocol, because we have to be able to hold
 * an entire packet for checksumming...
 * First we thought it had to be big enough for a SiRF Measured Tracker
 * Data packet (188 bytes). Then it had to be big enough for a UBX SVINFO
 * packet (206 bytes). Now it turns out that a couple of ITALK messages are
 * over 512 bytes. I know we like verbose output, but this is ridiculous.
 * Whoopie! The u-blox 8 UBX-RXM-RAWX packet is 8214 byte long!
 */
#define MAX_PACKET_LENGTH       9216    // 4 + 16 + (256 * 32) + 2 + fudge

/*
 * UTC of second 0 of week 0 of the first rollover period of GPS time.
 * Used to compute UTC from GPS time. Also, the threshold value
 * under which system clock times are considered unreliable. Often,
 * embedded systems come up thinking it's early 1970 and the system
 * clock will report small positive values until the clock is set.  By
 * choosing this as the cutoff, we'll never reject historical GPS logs
 * that are actually valid.
 */
#define GPS_EPOCH       ((time_t)315964800)     // 6 Jan 1980 00:00:00 UTC

// time constant
#define SECS_PER_DAY    ((time_t)(60*60*24))    // seconds per day
#define SECS_PER_WEEK   (7*SECS_PER_DAY)        // seconds per week
#define GPS_ROLLOVER    (1024*SECS_PER_WEEK)    // rollover period

struct gpsd_errout_t {
    int debug;                          // lexer debug level
    void (*report)(const char *);       // reporting hook for lexer errors
    const char *label;
};

// Careful, must match Class Lexer_t  in ./gps/packet.py.in
struct gps_lexer_t {
    // packet-getter internals
    int type;                           //  current packet type
#define BAD_PACKET              -1
#define COMMENT_PACKET          0
#define NMEA_PACKET             1
#define AIVDM_PACKET            2
#define GARMINTXT_PACKET        3
#define MAX_TEXTUAL_TYPE        3       // increment this as necessary
#define SIRF_PACKET             4
#define ZODIAC_PACKET           5
#define TSIP_PACKET             6
#define EVERMORE_PACKET         7
#define ITALK_PACKET            8
#define GARMIN_PACKET           9
#define NAVCOM_PACKET           10
#define UBX_PACKET              11
#define SUPERSTAR2_PACKET       12
#define ONCORE_PACKET           13
#define GEOSTAR_PACKET          14
#define NMEA2000_PACKET         15
#define GREIS_PACKET            16
#define SKY_PACKET              17
#define ALLYSTAR_PACKET         18
#define CASIC_PACKET            19
#define IS_PACKET               20      // Inertial Sense
#define MAX_GPSPACKET_TYPE      20      // increment this as necessary
// end of GPS type packets

#define RTCM2_PACKET            21
#define RTCM3_PACKET            22
#define JSON_PACKET             23
#define SPARTN_PACKET           24
// end of non GPS type packets, AIVDM is GPS type??
#define PACKET_TYPES            25      // increment this as necessary

#define TEXTUAL_PACKET_TYPE(n)  ((((n)>=NMEA_PACKET) && ((n)<=MAX_TEXTUAL_TYPE)) || (n)==JSON_PACKET)
#define GPS_PACKET_TYPE(n)      (((n)>=NMEA_PACKET) && ((n)<=MAX_GPSPACKET_TYPE))
#define LOSSLESS_PACKET_TYPE(n) (((n)>=RTCM2_PACKET) && ((n)<=RTCM3_PACKET))
#define PACKET_TYPEMASK(n)      (1L << (n))
#define GPS_TYPEMASK    (((2L<<(MAX_GPSPACKET_TYPE+1))-1) &~ PACKET_TYPEMASK(COMMENT_PACKET))

    long type_mask;                      //  packet types to mask
    unsigned int state;
    size_t length;         // if a message has a length field, this is it.
    unsigned char inbuffer[MAX_PACKET_LENGTH*2+1];
    size_t inbuflen;
    unsigned char *inbufptr;
    // outbuffer needs to be able to hold 4 GPGSV records at once
    unsigned char outbuffer[MAX_PACKET_LENGTH*2+1];
    size_t outbuflen;
    unsigned long char_counter;         // count characters processed
    unsigned long retry_counter;        // count sniff retries
    unsigned counter;                   // packets since last driver switch
    struct gpsd_errout_t errout;        // how to report errors
    timespec_t start_time;              // time of first input, sort of
    timespec_t pkt_time;                // time of last packet parsed
    unsigned long start_char;           // char counter at first input
    /*
     * ISGPS200 decoding context.
     *
     * This is not conditionalized on RTCM104_ENABLE because we need to
     * be able to build gpsdecode even when RTCM support is not
     * configured in the daemon.
     */
    struct {
        bool            locked;
        int             curr_offset;
        isgps30bits_t   curr_word;
        unsigned int    bufindex;
        /*
         * Only these should be referenced from elsewhere, and only when
         * RTCM_MESSAGE has just been returned.
         */
        isgps30bits_t   buf[RTCM2_WORDS_MAX];   // packet data
        size_t          buflen;                 // packet length in bytes
    } isgps;
    unsigned int json_depth;
    unsigned int json_after;
#ifdef STASH_ENABLE
    unsigned char stashbuffer[MAX_PACKET_LENGTH];
    size_t stashbuflen;
#endif  // STASH_ENABLE
    bool chunked;             // true if NTRIP/1.1 and the HTTP stream is chunked.
    int chunk_remaining;      // Bytes remaining before end of this chunk.
};

extern void lexer_init(struct gps_lexer_t *, struct gpsd_errout_t *);
extern void packet_reset(struct gps_lexer_t *);
extern void packet_pushback(struct gps_lexer_t *);
extern void packet_parse(struct gps_lexer_t *);
// packet_get()  deprecated Sep 2023, use packet_get1() instead
extern ssize_t packet_get(int, struct gps_lexer_t *);
extern int packet_sniff(struct gps_lexer_t *);

// return the number of bytes waiting in inbuffer
#define packet_buffered_input(lexer) ((lexer)->inbuffer + (lexer)->inbuflen - (lexer)->inbufptr)

// Next, declarations for the core library...

// factors for converting among confidence interval units
#define CEP50_SIGMA     1.18
#define DRMS_SIGMA      1.414
#define CEP95_SIGMA     2.45

// this is where we choose the confidence level to use in reports
#define GPSD_CONFIDENCE CEP95_SIGMA

#define NTPSHMSEGS      (MAX_DEVICES * 2)       // number of NTP SHM segments
#define NTP_MIN_FIXES   3  // # fixes to wait for before shipping NTP time


#define AIVDM_CHANNELS  2               // A, B

struct gps_device_t;

struct gps_context_t {
    int valid;                          // member validity flags
#define LEAP_SECOND_VALID       0x01    // we have or don't need correction
#define GPS_TIME_VALID          0x02    // GPS week/tow is valid
#define CENTURY_VALID           0x04    // have received ZDA or 4-digit year
    struct gpsd_errout_t errout;        // debug verbosity level and hook
    bool readonly;                      // if true, never write to device
    bool passive;                       // if true, never autoconfigure device
    // if true, remove fix gate to time, for some RTC backed receivers.
    // DANGEROUS
    bool batteryRTC;
    speed_t fixed_port_speed;           // Fixed port speed, if non-zero
    char fixed_port_framing[4];         // Fixed port framing, if non-blank
    // DGPS status
    int fixcnt;                         // count of good fixes seen
    // timekeeping
    time_t start_time;                  // local time of daemon startup
    int leap_seconds;                   // Unix secs to UTC (GPS-UTC offset)
    unsigned short gps_week;            // GPS week, usually 10 bits
    timespec_t gps_tow;                 // GPS time of week
    int century;                        // for NMEA-only devices without ZDA
    int rollovers;                      // rollovers since start of run
    int leap_notify;                    // notification state from subframe
#define LEAP_NOWARNING  0x0     // normal, no leap second warning
#define LEAP_ADDSECOND  0x1     // last minute of day has 60 seconds
#define LEAP_DELSECOND  0x2     // last minute of day has 59 seconds
#define LEAP_NOTINSYNC  0x3     // overload, clock is free running
    /* we need the volatile here to tell the C compiler not to
     * 'optimize' as 'dead code' the writes to SHM */
    volatile struct shmTime *shmTime[NTPSHMSEGS];
    bool shmTimeInuse[NTPSHMSEGS];
    void (*pps_hook)(struct gps_device_t *, int, int, struct timedelta_t *);
#ifdef SHM_EXPORT_ENABLE
    /* we don't want the compiler to treat writes to shmexport as dead code,
     * and we don't want them reordered either */
    struct shmexport_t *shmexport;
    int shmid;                          // ID of SHM  (for later IPC_RMID)
#endif
    ssize_t (*serial_write)(struct gps_device_t *,
                            const char *buf, const size_t len);
};

// state for resolving interleaved Type 24 packets
struct ais_type24a_t {
    unsigned int mmsi;
    char shipname[AIS_SHIPNAME_MAXLEN+1];
};
#define MAX_TYPE24_INTERLEAVE   8       // max number of queued type 24s
struct ais_type24_queue_t {
    struct ais_type24a_t ships[MAX_TYPE24_INTERLEAVE];
    int index;
};

// state for resolving AIVDM decodes
struct aivdm_context_t {
    // hold context for decoding AIDVM packet sequences
    int decoded_frags;     // for tracking AIDVM parts in a multipart sequence
    unsigned char bits[2048];
    size_t bitlen;         // how many valid bits
    struct ais_type24_queue_t type24_queue;
};

#define MODE_NMEA       0
#define MODE_BINARY     1

typedef enum {ANY, GPS, RTCM2, RTCM3, AIS} gnss_type;
typedef enum {
    EVENT_WAKEUP,
    EVENT_TRIGGERMATCH,
    EVENT_IDENTIFIED,
    EVENT_CONFIGURE,
    EVENT_DRIVER_SWITCH,
    EVENT_DEACTIVATE,
    EVENT_REACTIVATE,
} event_t;


#define INTERNAL_SET(n) ((gps_mask_t)(1llu<<(SET_HIGH_BIT+(n))))
#define RAW_IS          INTERNAL_SET(1)         // raw pseudoranges available
#define USED_IS         INTERNAL_SET(2)         // sat-used count available
#define DRIVER_IS       INTERNAL_SET(3)         // driver type identified
#define CLEAR_IS        INTERNAL_SET(4)         // starts a reporting cycle
#define REPORT_IS       INTERNAL_SET(5)         // ends a reporting cycle
#define NODATA_IS       INTERNAL_SET(6)         // no data read from fd
#define NTPTIME_IS      INTERNAL_SET(7)         // precision time is available
#define PERR_IS         INTERNAL_SET(8)         // PDOP set
#define PASSTHROUGH_IS  INTERNAL_SET(9)         // passthrough mode
#define EOF_IS          INTERNAL_SET(10)        // synthetic EOF
#define GOODTIME_IS     INTERNAL_SET(11)        // time good even if no pos fix

typedef unsigned int driver_mask_t;
#define DRIVER_NOFLAGS  0x00000000u
#define DRIVER_STICKY   0x00000001u

/*
 * True if a device type is non-null and has control methods.
 */
#define CONTROLLABLE(dp)        ((NULL != (dp)) &&                  \
                                 (NULL != (dp)->speed_switcher ||   \
                                  NULL != (dp)->mode_switcher ||    \
                                  NULL != (dp)->rate_switcher))

/*
 * True if a driver selection of it should be sticky.
 */
#define STICKY(dp)      ((dp) != NULL && ((dp)->flags & DRIVER_STICKY) != 0)

struct gps_type_t {
// GPS method table, describes how to talk to a particular GPS type
    char *type_name;
    int packet_type;
    driver_mask_t flags;        // reserved for expansion
    char *trigger;
    int channels;
    bool (*probe_detect)(struct gps_device_t *session);
    ssize_t (*get_packet)(struct gps_device_t *session);
    gps_mask_t (*parse_packet)(struct gps_device_t *session);
    ssize_t (*rtcm_writer)(struct gps_device_t *session,
                           const char *rtcmbuf, size_t rtcmbytes);
    void (*init_query)(struct gps_device_t *session);
    void (*event_hook)(struct gps_device_t *session, event_t event);
    bool (*speed_switcher)(struct gps_device_t *session,
                                     speed_t speed, char parity, int stopbits);
    void (*mode_switcher)(struct gps_device_t *session, int mode);
    bool (*rate_switcher)(struct gps_device_t *session, double rate);
    timespec_t min_cycle;
    ssize_t (*control_send)(struct gps_device_t *session,
                            char *buf, size_t buflen);
    double (*time_offset)(struct gps_device_t *session);
};

/*
 * Each input source has an associated type.  This is currently used in two
 * ways:
 *
 * (1) To determine if we require that gpsd be the only process opening a
 * device.  We make an exception for PTYs because the master side has to be
 * opened by test code.
 *
 * (2) To determine whether it's safe to send wakeup strings.  These are
 * required on some unusual RS-232 devices (such as the TNT compass and
 * Thales/Ashtech GPSes) but should not be shipped to unidentified USB
 * or Bluetooth devices as we don't even know in advance those are GPSes;
 * they might not cope well.
 *
 * Where it says "case detected but not used" it means that we can identify
 * a source type but no behavior is yet contingent on it.  A "discoverable"
 * device is one for which there is discoverable metadata such as a
 * vendor/product ID.
 *
 * We should never see a block device; that would indicate a serious error
 * in command-line usage or the hotplug system.
 *
 * Order matters, SOURCE_BLOCKDEV >= is used to check for read-only
 */
typedef enum {SOURCE_UNKNOWN,
              SOURCE_BLOCKDEV,  // block devices can't be GPS sources
              SOURCE_RS232,     // potential GPS source, not discoverable
              SOURCE_USB,       // potential GPS source, discoverable
              SOURCE_BLUETOOTH, // potential GPS source, discoverable
              SOURCE_CAN,       // potential GPS source, fixed CAN format
              SOURCE_PTY,       // PTY: we don't require exclusive access
              SOURCE_TCP,       // TCP/IP stream: case detected but not used
              SOURCE_UDP,       // UDP stream: case detected but not used
              SOURCE_GPSD,      // Remote gpsd instance over TCP/IP
              SOURCE_PPS,       // PPS-only device, such as /dev/ppsN
              SOURCE_PIPE,      // Unix FIFO; don't use blocking I/O
              SOURCE_ACM,       // potential GPS source, discoverable, no speed
} sourcetype_t;

/*
 * Each input source also has an associated service type.
 */
typedef enum {SERVICE_UNKNOWN = 0,
              SERVICE_SENSOR,        // local, or network, sensor
              SERVICE_DGPSIP,        // dgpsip://
              SERVICE_NTRIP,         // ntrip://
} servicetype_t;

/*
 * Private state information about an NTRIP stream.
 */
struct ntrip_stream_t
{
    timespec_t stream_time;  // time stream was last opened or closed
    char mountpoint[101];    // stream name
    char credentials[128];   // username:password
    char authStr[128];       // HTTP Authorization: line
    char url[256];           // full url: http://user@pass:host:port/mp
    char host[256];          // hostname or IP
    char port[32];           // in my /etc/services 16 was the longest
    bool set;                // found and set
    enum ntrip_fmt {
        FMT_UNKNOWN = 0,
        FMT_CMRP,            // CMR+, dunno what it is.  ORGN uses it
        FMT_RTCM2,
        FMT_RTCM2_0,
        FMT_RTCM2_1,
        FMT_RTCM2_2,
        FMT_RTCM2_3,
        FMT_RTCM3_0,
        FMT_RTCM3_1,
        FMT_RTCM3_2,
        FMT_RTCM3_3,
        // FMT_SPARTN,        // u-blox SPARTN  UNSUPPORTED!
        FMT_SPARTN_2,         // u-blox SPARTN
    } format;
    int carrier;
    double latitude;
    double longitude;
    int nmea;
    enum {
        CMP_ENC_NONE,
        CMP_ENC_UNKNOWN
    } compr_encryp;
    enum {
        AUTH_NONE,
        AUTH_BASIC,
        AUTH_DIGEST,
        AUTH_UNKNOWN
    } authentication;
    int fee;
    int bitrate;
};


// session object, encapsulates all global state
struct gps_device_t {
    struct gps_data_t gpsdata;
    const struct gps_type_t *device_type;
    unsigned int driver_index;        // numeric index of current driver
    unsigned int drivers_identified;  // bitmask; what drivers have we seen?
    unsigned int cfg_stage;           // configuration stage counter
    unsigned int cfg_step;            // configuration step counter
    const struct gps_type_t *last_controller;
    struct gps_context_t        *context;
    sourcetype_t sourcetype;
    servicetype_t servicetype;
    int mode;
    struct termios ttyset, ttyset_old;
    unsigned int baudindex;
    int saved_baud;
    struct gps_lexer_t lexer;
    int badcount;
    int subframe_count;
    // firmware version or subtype ID, 96 too small for ZED-F9
    char subtype[128];
    char subtype1[128];
    time_t opentime;                  // FIXME: change to timespec_t
    time_t releasetime;
    bool zerokill;
    time_t reawake;
    timespec_t sor;                   // time start of this reporting cycle
    // time start of current autobaud hunt.
    // maybe should be in struct device_t, but should not be client visible.
    timespec_t ts_startCurrentBaud;
    unsigned long chars;              // characters in the cycle
    bool ship_to_ntpd;
#define VALID_UNIT(u)   (0 <= (u) && (u) < NTPSHMSEGS)
    int shm_clock_unit;
    int shm_pps_unit;
    time_t shm_clock_lastsec;         // the last second written to SHM(clock)
    time_t shm_pps_lastsec;           // the last second written to SHM(pps)
    int chrony_clock_fd;              // for talking to chrony
    int chrony_pps_fd;
    volatile struct pps_thread_t pps_thread;
    /*
     * msgbuf needs to hold the hex decode of inbuffer
     * so msgbuf must be 2x the size of inbuffer
     */
    char msgbuf[MAX_PACKET_LENGTH*4+1]; // command message buffer for sends
    size_t msgbuflen;
    long observed;                      // which packet type`s have we seen?
    bool cycle_end_reliable;            // does driver signal REPORT_MASK
    int fixcnt;                         // count of fixes from this device
    int last_word_gal;                  // last subframe word from Galileo
    int last_svid3_gal;                 // last SVID3 from Galileo
    struct gps_fix_t newdata;           // where drivers put their data
    struct gps_fix_t lastfix;           // not quite yet ready for oldfix
    struct gps_fix_t oldfix;            // previous fix for error modeling
    int queue;                          // Next item in init queue to request.
    // 1 if start_time is from regression comment. Ignore WKRO fixup.
    int regression;
    struct {
        unsigned short sats_used[MAXCHANNELS];
        int part, await;                // for tracking GSV parts
        struct tm date;                 // date part of last sentence time
        timespec_t subseconds;          // subsec part of last sentence time
        // pointers to each field in fieldcopy.  Each is NUL terminated.
        char *field[NMEA_MAX_FLD];
        unsigned char fieldcopy[NMEA_MAX+1]; // a copy of the current sentence
        // detect receivers that ship GGA with non-advancing timestamp
        bool latch_mode;
        char last_gga_timestamp[16];
        char last_gga_talker;
        // GSV stuff
        bool seen_bdgsv;                        // BeiDou GSV
        bool seen_gagsv;                        // Galileo GSV
        bool seen_gigsv;                        // IRNSS GSV
        bool seen_glgsv;                        // GLONASS GSV
        bool seen_gngsv;                        // GNSS GSV
        bool seen_gpgsv;                        // GPS GSV
        bool seen_qzgsv;                        // QZSS GSV
        char last_gsv_talker;
        char end_gsv_talker;
        unsigned char last_gsv_sigid;           // NMEA 4.1
        // GSA stuff
        bool seen_bdgsa;                        // BeiDou GSA
        bool seen_gagsa;                        // Galileo GSA
        bool seen_gigsa;                        // IRNSS GSA
        bool seen_glgsa;                        // GLONASS GSA
        bool seen_gpgsa;                        // GPS GSA
        bool seen_gngsa;                        // GNSS GSA
        bool seen_qzgsa;                        // QZSS GSA
        char last_gsa_talker;
        /*
         * State for the cycle-tracking machinery.
         * The reason these timestamps are separate from the
         * general sentence timestamps is that we can
         * use the minutes and seconds part of a sentence
         * with an incomplete timestamp (like GGA) for
         * end-cycle recognition, even if we don't have a previous
         * RMC or ZDA that lets us get full time from it.
         */
        timespec_t this_frac_time, last_frac_time;
        bool latch_frac_time;
        int lasttag;              // index into nmea_phrase[]
        // 140 > number of used entries in nmea_phrase[]
#define NMEA_NUM 170
        // bit map into nmea_phrase[], +1 to shut up coverity
        bool cycle_enders[NMEA_NUM + 1];
        bool cycle_continue;
        bool gsx_more;         // more GSV or GSA to come.
        unsigned gga_sats_used;  // sats used from GGA, GNS or $PASHR
    } nmea;
    /*
     * The rest of this structure is driver-specific private storage.
     * Only put a driver's scratch storage in here if it is never
     * implemented on the same device that supports any mode already
     * in this union; otherwise bad things might happen after a device
     * mode switch.
     */
    union {
#ifdef GEOSTAR_ENABLE
        struct {
            unsigned int physical_port;
        } geostar;
#endif  // GEOSTAR_ENABLE
#ifdef GREIS_ENABLE
        struct {
            uint32_t rt_tod;            // RT message time of day (modulo 1 day)
            bool seen_rt;               // true if seen RT message
            bool seen_uo;               // true if seen UO message
            bool seen_si;               // true if seen SI message
            bool seen_az;               // true if seen AZ message
            bool seen_ec;               // true if seen EC message
            bool seen_el;               // true if seen EL message
            // true if seen a raw measurement message
            bool seen_raw;
        } greis;
#endif  // GREIS_ENABLE
#ifdef SIRF_ENABLE
        struct {
            unsigned int need_ack;      // if NZ we're awaiting ACK
            unsigned int driverstate;   // for private use
#define SIRF_LT_231     0x01            // SiRF at firmware rev < 231
#define SIRF_EQ_231     0x02            // SiRF at firmware rev == 231
#define SIRF_GE_232     0x04            // SiRF at firmware rev >= 232
#define UBLOX           0x08            // u-blox firmware with packet 0x62
            unsigned long satcounter;
            unsigned int time_seen;
            unsigned char lastid;       // ID with last timestamp seen
#define TIME_SEEN_UTC_2 0x08            // Seen UTC time variant 2?
            // fields from Navigation Parameters message
            bool nav_parameters_seen;   // have we seen one?
            unsigned char altitude_hold_mode;
            unsigned char altitude_hold_source;
            int16_t altitude_source_input;
            unsigned char degraded_mode;
            unsigned char degraded_timeout;
            unsigned char dr_timeout;
            unsigned char track_smooth_mode;
            // fields from DGPS Status
            unsigned int dgps_source;
#define SIRF_DGPS_SOURCE_NONE           0  // No DGPS correction type selected
#define SIRF_DGPS_SOURCE_SBAS           1  // SBAS
#define SIRF_DGPS_SOURCE_SERIAL         2  // RTCM corrections
#define SIRF_DGPS_SOURCE_BEACON         3  // Beacon corrections
#define SIRF_DGPS_SOURCE_SOFTWARE       4  // Software API corrections
        } sirf;
#endif  // SIRF_ENABLE
#ifdef SUPERSTAR2_ENABLE
        struct {
            time_t last_iono;
        } superstar2;
#endif  // SUPERSTAR2_ENABLE
#ifdef TSIP_ENABLE
        struct {
            unsigned short sats_used[MAXCHANNELS];
            /* Super Packet mode requested.
             * 0 = None, 1 = old superpacket, 2 = new superpacket (SMT 360) */
            uint8_t superpkt;
            uint8_t machine_id;         // from 0x4b, or x90-01
            uint16_t hardware_code;     // from 0x1c-83, or 0x90-01
            time_t last_41;             // Timestamps for packet requests
            time_t last_48;
            time_t last_5c;
            time_t last_6d;
            time_t last_46;
            time_t req_compact;
            time_t last_a200;           // last xa2-00 received
            time_t last_a311;           // last xa3-11 received
            unsigned int stopbits;      // saved RS232 link parameter
            char parity;
            int subtype;                // hardware ID, sort of
#define TSIP_UNKNOWN               0
#define TSIP_ACEIII               93
#define TSIP_LASSENIQ           1001
// 1002 includes Copernicus II?
#define TSIP_COPERNICUS         1002
// 1003 undocumented?
#define TSIP_COPERNICUS_II      1003
#define TSIP_ACUTIME_GOLD       3001
#define TSIP_REST               3002
#define TSIP_TBOLTE             3007
#define TSIP_RESSMT             3009
// Part Number 99889-xx, Resolution SMTx
#define TSIP_RES_SMTX           3017
// Part Number 97975-xx, Res SMT 360
#define TSIP_RESSMT360          3023
#define TSIP_ICMSMT360          3026
#define TSIP_RES36017x22        3031
#define TSIP_ACUTIME_360        3002
// Part Number 121238-xx, RES 720
#define TSIP_RES720             3100
            uint8_t alt_is_msl;         // 0 if alt is HAE, 1 if MSL
            timespec_t last_tow;        // used to find cycle start
            int last_chan_seen;         // from 0x5c or 0x5d
        } tsip;
#endif  // TSIP_ENABLE
#ifdef GARMIN_ENABLE    // private housekeeping stuff for the Garmin driver
        struct {
            unsigned char Buffer[4096+12];   // Garmin packet buffer
            size_t BufferLen;                // current GarminBuffer Length
        } garmin;
#endif  // GARMIN_ENABLE
#ifdef ZODIAC_ENABLE    // private housekeeping stuff for the Zodiac driver
        struct {
            unsigned short sn;          // packet sequence number
            /*
             * Zodiac chipset channel status from PRWIZCH. Keep it so
             * raw-mode translation of Zodiac binary protocol can send
             * it up to the client.
             */
#define ZODIAC_CHANNELS 12
            unsigned int Zs[ZODIAC_CHANNELS];   // satellite PRNs
            unsigned int Zv[ZODIAC_CHANNELS];   // signal values (0-7)
        } zodiac;
#endif  // ZODIAC_ENABLE

        // driver_ally and driver_ubx both use the next struct.
        struct {
            // FIXME: last_time set but never used?
            timespec_t last_time;               // time of last_msgid
            // iTOW, and last_iTOW, in ms, used for cycle end detect.
            int64_t iTOW;
            int64_t last_iTOW;
            unsigned int end_msgid;             // cycle ender class/ID
            unsigned int last_msgid;            // last class/ID
            unsigned char port_id;
            unsigned char sbas_in_use;
            unsigned char protver;              // u-blox protocol version
            unsigned char last_protver;         // last protocol version
        } ubx;
#ifdef NAVCOM_ENABLE
        struct {
            uint8_t physical_port;
            bool warned;
        } navcom;
#endif  // NAVCOM_ENABLE
#ifdef ONCORE_ENABLE
        struct {
#define ONCORE_VISIBLE_CH 12
            int visible;
            int PRN[ONCORE_VISIBLE_CH];         // PRNs of satellite
            int elevation[ONCORE_VISIBLE_CH];   // elevation of satellite
            int azimuth[ONCORE_VISIBLE_CH];     // azimuth
#define ONCORE_POS_HOLD_MODE_UNKNOWN    0
#define ONCORE_POS_HOLD_MODE_OFF        1
#define ONCORE_POS_HOLD_MODE_ON         2
#define ONCORE_POS_HOLD_MODE_SURVEYING  3
            int pos_hold_mode;
            int pps_offset_ns;
        } oncore;
#endif  // ONCORE_ENABLE
#ifdef NMEA2000_ENABLE
        struct {
            unsigned int can_msgcnt;
            unsigned int can_net;
            unsigned int unit;
            bool unit_valid;
            int mode;
            unsigned int mode_valid;
            unsigned int idx;
//          size_t ptr;
            size_t fast_packet_len;
            int type;
            void *workpgn;
            void *pgnlist;
            unsigned char sid[8];
        } nmea2000;
#endif  // NMEA2000_ENABLE
        /*
         * This is not conditionalized on RTCM104_ENABLE because we need to
         * be able to build gpsdecode even when RTCM support is not
         * configured in the daemon.  It doesn't take up extra space.
         */
        struct {
            // ISGPS200 decoding
            bool            locked;
            int             curr_offset;
            isgps30bits_t   curr_word;
            isgps30bits_t   buf[RTCM2_WORDS_MAX];
            unsigned int    bufindex;
        } isgps;
#ifdef AIVDM_ENABLE
        struct {
            struct aivdm_context_t context[AIVDM_CHANNELS];
            char ais_channel;
        } aivdm;
#endif  // AIVDM_ENABLE
    } driver;

    /*
     * State of an NTRIP connection.  We don't want to zero this on every
     * activation, otherwise the connection state will get lost.  Information
     * in this substructure is only valid if servicetype is SERVICE_NTRIP.
     */
    struct {
        // state information about the stream
        struct ntrip_stream_t stream;

        // state information about our response parsing
        // must match ntrip_states[] in gpsd/net_ntrip.c
        enum {
            NTRIP_CONN_INIT = 0,
            NTRIP_CONN_SENT_PROBE,
            NTRIP_CONN_SENT_GET,
            NTRIP_CONN_ESTABLISHED,
            NTRIP_CONN_ERR,
            NTRIP_CONN_CLOSED,         // connection closed
            NTRIP_CONN_INPROGRESS,     // connection in progress
        } conn_state;   // connection state for multi stage connect
        bool works; // marks a working connection, so we try to reconnect once
        bool sourcetable_parse; // have we read the sourcetable header?
    } ntrip;
    // State of a DGPSIP connection
    struct {
        bool reported;
    } dgpsip;
};

extern ssize_t packet_get1(struct gps_device_t *);

/*
 * These are used where a file descriptor of 0 or greater indicates open device.
 */
#define UNALLOCATED_FD  -1      // this slot is available for reallocation
#define PLACEHOLDING_FD -2      // this slot *not* available for reallocation

// logging levels
#define LOG_ERROR       -1      // errors, display always
#define LOG_SHOUT       0       // not an error but we should always see it
#define LOG_WARN        1       // not errors but may indicate a problem
#define LOG_CLIENT      2       // log JSON reports to clients
#define LOG_INF         3       // key informative messages
#define LOG_PROG        4       // progress messages
#define LOG_IO          5       // IO to and from devices
#define LOG_DATA        6       // log data management messages
#define LOG_SPIN        7       // logging for catching spin bugs
#define LOG_RAW         8       // raw low-level I/O
#define LOG_RAW1        9       // rawer
#define LOG_RAW2       10       // rawest

#define ISGPS_ERRLEVEL_BASE     LOG_RAW

#define IS_HIGHEST_BIT(v,m)     0 == (v & ~((m << 1) - 1))

// driver helper functions
extern void isgps_init(struct gps_lexer_t *);
enum isgpsstat_t isgps_decode(struct gps_lexer_t *,
                              bool (*preamble_match)(isgps30bits_t *),
                              bool (*length_check)(struct gps_lexer_t *),
                              size_t,
                              unsigned int);
extern unsigned int isgps_parity(isgps30bits_t);
extern void isgps_output_magnavox(const isgps30bits_t *, unsigned int, FILE *);

extern enum isgpsstat_t rtcm2_decode(struct gps_lexer_t *, unsigned int);
extern void json_rtcm2_dump(struct rtcm2_t *,
                            const char *, char[], size_t);
extern void rtcm2_unpack(struct gps_device_t *, struct rtcm2_t *, char *);
extern void json_rtcm3_dump(const struct rtcm3_t *,
                            const char *, char[], size_t);
extern void rtcm3_unpack(const struct gps_context_t *,
                         struct rtcm3_t *, const unsigned char *);

// here are the available GPS drivers
extern const struct gps_type_t **gpsd_drivers;

// gpsd library internal prototypes
extern gps_mask_t generic_parse_input(struct gps_device_t *);

extern gps_mask_t nmea_parse(char *, struct gps_device_t *);
extern ssize_t nmea_write(struct gps_device_t *, char *, size_t);
extern ssize_t nmea_send(struct gps_device_t *, const char *, ... );
extern void nmea_add_checksum(char *);

extern gps_mask_t sirf_parse(struct gps_device_t *, unsigned char *, size_t);
extern gps_mask_t spartn_parse(struct gps_device_t *);
extern gps_mask_t evermore_parse(struct gps_device_t *, unsigned char *,
                                 size_t);
extern gps_mask_t navcom_parse(struct gps_device_t *, unsigned char *, size_t);
extern gps_mask_t garmin_ser_parse(struct gps_device_t *);
extern gps_mask_t garmintxt_parse(struct gps_device_t *);
extern gps_mask_t aivdm_parse(struct gps_device_t *);

extern bool netgnss_uri_check(char *);
extern gps_fd_t netgnss_uri_open(struct gps_device_t *, char *);
extern void netgnss_report(struct gps_context_t *,
                         struct gps_device_t *,
                         struct gps_device_t *);
extern void netgnss_autoconnect(struct gps_context_t *, double, double);

extern socket_t dgpsip_open(struct gps_device_t *, const char *);
extern void dgpsip_report(struct gps_context_t *,
                         struct gps_device_t *,
                         struct gps_device_t *);
extern void dgpsip_autoconnect(struct gps_context_t *,
                               double, double, const char *);
extern socket_t ntrip_open(struct gps_device_t *, char *);
extern void ntrip_report(struct gps_context_t *,
                         struct gps_device_t *,
                         struct gps_device_t *);

extern bool gpsd_set_raw(struct gps_device_t *);
extern int gpsd_serial_isatty(const struct gps_device_t *);
extern int gpsd_serial_open(struct gps_device_t *);
extern void gpsd_tty_init(struct gps_device_t *);
extern ssize_t gpsd_serial_write(struct gps_device_t *,
                                 const char *, const size_t);
extern bool gpsd_next_hunt_setting(struct gps_device_t *);
extern int gpsd_switch_driver(struct gps_device_t *, char *);
extern void gpsd_set_speed(struct gps_device_t *, speed_t, char, unsigned int);
extern int gpsd_get_speed(const struct gps_device_t *);
extern int gpsd_get_speed_old(const struct gps_device_t *);
extern int gpsd_get_stopbits(const struct gps_device_t *);
extern char gpsd_get_parity(const struct gps_device_t *);
extern void gpsd_assert_sync(struct gps_device_t *);
extern void gpsd_close(struct gps_device_t *);

extern ssize_t gpsd_write(struct gps_device_t *, const char *, const size_t);

extern void gpsd_time_init(struct gps_context_t *, time_t);
extern void gpsd_set_century(struct gps_device_t *);
extern timespec_t gpsd_gpstime(struct gps_device_t *, unsigned, timespec_t);
extern timespec_t gpsd_gpstime_resolv(struct gps_device_t *, unsigned,
                                      timespec_t);
extern timespec_t gpsd_utc_resolve(struct gps_device_t *);
extern void gpsd_century_update(struct gps_device_t *, int);

extern void gpsd_zero_satellites(struct gps_data_t *sp);
extern gps_mask_t gpsd_interpret_subframe(struct gps_device_t *,
                                          unsigned int,
                                          unsigned int,
                                          uint32_t[]);
extern gps_mask_t gpsd_interpret_subframe_raw(struct gps_device_t *,
                                              unsigned int,
                                              unsigned int,
                                              unsigned int,
                                              uint32_t[],
                                              unsigned int);
extern const char *gpsd_packetdump(char *, size_t, const unsigned char *,
                                   size_t);
extern const char *gpsd_prettydump(struct gps_device_t *);
# ifdef __cplusplus
extern "C" {
# endif
# ifdef __cplusplus
}
# endif
extern ssize_t hex_escapes(char *, const char *);
extern int gpsd_position_fix_dump(struct gps_device_t *,
                                  char[], size_t);
extern void gpsd_clear_data(struct gps_device_t *);
// FIXME: use in libgps, so should not be in gpsd.h!
// deprecated Oct 2021, use netlib_connectsock1() instead
extern socket_t netlib_connectsock(int, const char *, const char *,
                                   const char *);
extern socket_t netlib_connectsock1(int, const char *, const char *,
                                    const char *, int, bool,
                                    char *, size_t);
// end FIXME
extern socket_t netlib_localsocket(const char *, int);
extern const char *netlib_errstr(const int);

/* klugey definition of a socket address struct helps hide
 * IPV4 vs. IPV6 ugliness
 */
typedef union sockaddr_u {
    struct sockaddr sa;
    struct sockaddr_in sa_in;
    struct sockaddr_in6 sa_in6;
} sockaddr_t;
extern char *socka2a(sockaddr_t *, char *, size_t);

extern char *netlib_sock2ip(socket_t);

extern void nmea_tpv_dump(struct gps_device_t *, char[], size_t);
extern void nmea_sky_dump(struct gps_device_t *, char[], size_t);
extern void nmea_subframe_dump(struct gps_device_t *, char[], size_t);
extern void nmea_ais_dump(struct gps_device_t *, char[], size_t);
extern unsigned int ais_binary_encode(struct ais_t *ais, unsigned char *bits,
                                      int flag);

extern void ntrip_close(struct gps_device_t *);
extern int ntrip_parse_url(const struct gpsd_errout_t *,
                           struct ntrip_stream_t *, const char *);
extern void ntp_latch(struct gps_device_t *device,  struct timedelta_t *td);
extern void ntpshm_context_init(struct gps_context_t *);
extern void ntpshm_session_init(struct gps_device_t *);
extern void ntpshm_put(struct gps_device_t *, int unit, int precision,
                       struct timedelta_t *);
extern void chrony_send(struct gps_device_t *, int fd, struct timedelta_t *);
extern void ntpshm_link_deactivate(struct gps_device_t *);
extern void ntpshm_link_activate(struct gps_device_t *);

extern void errout_reset(struct gpsd_errout_t *errout);

extern void gpsd_acquire_reporting_lock(void);
extern void gpsd_release_reporting_lock(void);

extern gps_mask_t ecef_to_wgs84fix(struct gps_fix_t *,
                                   double, double, double,
                                   double, double, double);
extern void clear_dop(struct dop_t *);

// shmexport.c
#define GPSD_SHM_KEY    0x47505344      // "GPSD"
struct shmexport_t
{
    volatile int bookend1;
    struct gps_data_t gpsdata;
    volatile int bookend2;
};
extern bool shm_acquire(struct gps_context_t *);
extern void shm_release(struct gps_context_t *);
extern void shm_update(struct gps_context_t *, struct gps_data_t *);

// dbusexport.c
#if defined(DBUS_EXPORT_ENABLE)
int initialize_dbus_connection (void);
void send_dbus_fix (struct gps_device_t* channel);
#endif  // defined(DBUS_EXPORT_ENABLE)

// a BSD transplant
int b64_ntop(unsigned char const *src, size_t srclength, char *target,
    size_t targsize);

// application interface
extern void gps_context_init(struct gps_context_t *context,
                             const char *label);
extern void gpsd_init(struct gps_device_t *,
                      struct gps_context_t *,
                      const char *);
extern void gpsd_clear(struct gps_device_t *);
extern int parse_uri_dest(char *s, char **host, char **service, char **device);
extern int gpsd_open(struct gps_device_t *);
#define O_CONTINUE      0
#define O_PROBEONLY     1
#define O_OPTIMIZE      2
extern int gpsd_activate(struct gps_device_t *, const int);
extern void gpsd_deactivate(struct gps_device_t *);

#define AWAIT_TIMEOUT 2
#define AWAIT_GOT_INPUT 1
#define AWAIT_NOT_READY 0
#define AWAIT_FAILED    -1
extern int gpsd_await_data(fd_set *,
                           fd_set *,
                           int,
                           fd_set *,
                           struct gpsd_errout_t *,
                           timespec_t);
extern gps_mask_t gpsd_poll(struct gps_device_t *);
#define DEVICE_EOF      -3
#define DEVICE_ERROR    -2
#define DEVICE_UNREADY  -1
#define DEVICE_READY    1
#define DEVICE_UNCHANGED        0
extern int gpsd_multipoll(const bool,
                          struct gps_device_t *,
                          void (*)(struct gps_device_t *, gps_mask_t),
                          float reawake_time);
extern void gpsd_wrap(struct gps_device_t *);
extern bool gpsd_add_device(const char *device_name, bool flag_nowait);

// exceptional driver methods
extern bool ally_write(struct gps_device_t *, unsigned int, unsigned int,
                       const unsigned char *, size_t);
extern bool casic_write(struct gps_device_t *, unsigned int, unsigned int,
                        const unsigned char *, size_t);
extern unsigned casic_checksum(unsigned char * buf, size_t len);
extern bool ubx_write(struct gps_device_t *, unsigned int, unsigned int,
                      const unsigned char *, size_t);
extern bool ais_binary_decode(const struct gpsd_errout_t *errout,
                              struct ais_t *ais,
                              const unsigned char *, size_t,
                              struct ais_type24_queue_t *);

void gpsd_labeled_report(const int, const int,
                         const char *, const char *, va_list);

char *visibilize(char *outbuf, size_t outlen, const char *inbuf, size_t inlen);
// do not call gpsd_log() directly, use GPSD_LOG() to save a lot of cpu time
PRINTF_FUNC(3, 4) void gpsd_log(const int, const struct gpsd_errout_t *,
                                const char *, ...);

/*
 * GPSD_LOG() is the new one debug logger to rule them all.
 *
 * The calling convention is not attractive:
 *     GPSD_LOG(debuglevel, (fmt, ...));
 *     GPSD_LOG(2, ("this will appear on stdout if debug >= %d\n", 2));
 *
 * This saves significant pushing, popping, hexification, etc. when
 * the debug level does not require it.
 */
#define GPSD_LOG(lvl, eo, ...)                 \
    do {                                       \
        if (unlikely((eo)->debug >= (lvl))) {  \
            gpsd_log(lvl, eo, __VA_ARGS__);    \
        }                                      \
    } while (0)



#define NITEMS(x) ((int) (sizeof(x) / sizeof(x[0]) + COMPILE_CHECK_IS_ARRAY(x)))

/*
 * C99 requires NAN to be defined if the implementation supports quiet
 * NANs.  At one point, it seems Solaris did not define NAN; it is not
 * clear if this is still true.
 */
#ifndef NAN
#define NAN (0.0f/0.0f)
#endif

#if !defined(HAVE_CFMAKERAW)
/*
 * POSIX does not specify cfmakeraw, but it is pretty common.  We
 * provide an implementation in serial.c for systems that lack it.
 */
void cfmakeraw(struct termios *);
#endif  // !defined(HAVE_CFMAKERAW)

#define DEVICEHOOKPATH "/" SYSCONFDIR "/gpsd/device-hook"

# ifdef __cplusplus
}
# endif

#endif  // _GPSD_H_
// Local variables:
// mode: c
// end:
// vim: set expandtab shiftwidth=4
