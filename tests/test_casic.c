/*
 * Unit test for the CASIC driver.
 *
 * Everything goes in through the driver_casic method table and comes
 * out as bytes the driver wrote or state it left in gps_device_t, so
 * driver_casic.c can keep its internals static.  Writes are caught by
 * pointing context->serial_write at a capture function.  Messages are
 * fed to parse_packet(), framed as the packetizer would hand them over.
 *
 * The configuration tests are therefore indirect: what the driver
 * worked out about the receiver's ports only shows in which command it
 * then sends.
 *
 * Section and table references are to Quectel's "L76K GNSS Protocol
 * Specification" V1.1, 2021-12-16.
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/gpsd.h"
#include "../include/bits.h"
#include "../include/driver_casic.h"

static bool quiet = false;

extern const struct gps_type_t driver_casic;

/* Classes and IDs come from the driver's header, so they cannot drift.
 * feed() wants them apart, the header keeps them joined.
 */
#define CLS_OF(msgid)   CASIC_CLS_OF(msgid)
#define ID_OF(msgid)    CASIC_ID_OF(msgid)
#define MSGID_OF(cls, id)       CASIC_MSGID(cls, id)

#define CLS_NAV         CLS_OF(CASIC_NAV_PV)
#define CLS_ACK         CLS_OF(CASIC_ACK_ACK)
#define CLS_CFG         CLS_OF(CASIC_CFG_PRT)
#define ID_DOP          ID_OF(CASIC_NAV_DOP)
// no decoder in this driver
#define ID_SOL          ID_OF(CASIC_NAV_SOL)
#define ID_PV           ID_OF(CASIC_NAV_PV)
#define ID_TIMEUTC      ID_OF(CASIC_NAV_TIMEUTC)
#define ID_GPSINFO      ID_OF(CASIC_NAV_GPSINFO)
#define ID_BDSINFO      ID_OF(CASIC_NAV_BDSINFO)
#define ID_ACK          ID_OF(CASIC_ACK_ACK)
#define ID_PRT          ID_OF(CASIC_CFG_PRT)
#define ID_RATE         ID_OF(CASIC_CFG_RATE)

/* ProtoMask values the tests use, section 3.2.2.1 Table 9.  Built from
 * the driver's own bits; the wire value is in the comment, for reading
 * a capture against a test.
 */
// binary in, both out, no text input
#define MASK_BIN_IN     (CASIC_PROTO_IN_BIN | CASIC_PROTO_OUT_BIN | \
                         CASIC_PROTO_OUT_TXT)                   // 0x31
// the widest documented mask
#define MASK_BOTH_IN    (CASIC_PROTO_IN_BOTH | CASIC_PROTO_OUT_BIN | \
                         CASIC_PROTO_OUT_TXT)                   // 0x33
// emits NMEA, accepts nothing
#define MASK_TXT_OUT    CASIC_PROTO_OUT_TXT                     // 0x20
// what a mode switch sends: the input bits it found, one output bit
#define MASK_SET_BIN    (CASIC_PROTO_IN_BOTH | CASIC_PROTO_OUT_BIN)     // 0x13
#define MASK_SET_TXT    (CASIC_PROTO_IN_BIN | CASIC_PROTO_OUT_TXT)      // 0x21
// an undocumented bit, so this mask disagrees with every other
#define MASK_DISAGREES  (CASIC_PROTO_IN_BOTH | 0x04)            // 0x07

// CFG-PRT Mode for 8N1, the framing every test but check_mode_bits uses
#define MODE_8N1        (CASIC_MODE_DATA_8 | CASIC_MODE_PAR_NONE | \
                         CASIC_MODE_STOP_1)                     // 0x08c0

// capturing what the driver writes

#define MAX_WRITES      8
#define MAX_WRITE_LEN   128

static struct {
    unsigned char buf[MAX_WRITE_LEN];
    size_t len;
} wrote[MAX_WRITES];
static unsigned nwrote = 0;
/* A write the capture cannot hold.  Counted, not dropped quietly: a
 * silent drop looks exactly like the driver not writing at all.
 */
static unsigned nlost = 0;

static ssize_t capture_write(struct gps_device_t *session UNUSED,
                             const char *buf, const size_t len)
{
    if (MAX_WRITES <= nwrote ||
        sizeof(wrote[0].buf) < len) {
        nlost++;
        return (ssize_t)len;
    }
    memset(wrote[nwrote].buf, 0, sizeof(wrote[nwrote].buf));
    memcpy(wrote[nwrote].buf, buf, len);
    wrote[nwrote].len = len;
    nwrote++;
    return (ssize_t)len;
}

// payload of a captured CASIC message, NULL if none was sent
static const unsigned char *sent_casic(unsigned cls, unsigned id,
                                       size_t *plen)
{
    unsigned i;

    for (i = 0; i < nwrote; i++) {
        const unsigned char *w = wrote[i].buf;

        if (CASIC_OVERHEAD > wrote[i].len ||
            0xba != w[0] ||
            0xce != w[1] ||
            cls != w[4] ||
            id != w[5]) {
            continue;
        }
        if (NULL != plen) {
            *plen = wrote[i].len - CASIC_OVERHEAD;
        }
        return w + CASIC_PREFIX_LEN;
    }
    return NULL;
}

// A captured NMEA sentence with this prefix, NULL if none was sent.
static const char *sent_nmea(const char *prefix)
{
    size_t want = strnlen(prefix, MAX_WRITE_LEN);
    unsigned i;

    for (i = 0; i < nwrote; i++) {
        if (want <= wrote[i].len &&
            0 == memcmp(wrote[i].buf, prefix, want)) {
            return (const char *)wrote[i].buf;
        }
    }
    return NULL;
}

// true if the driver put anything on the wire
static bool wrote_anything(void)
{
    return 0 != nwrote;
}

static void session_init(struct gps_context_t *context,
                         struct gps_device_t *session)
{
    memset(context, 0, sizeof(*context));
    memset(session, 0, sizeof(*session));
    errout_reset(&context->errout);
    context->serial_write = capture_write;
    session->context = context;
    /* gpsd_init() does this, and a zeroed gps_fix_t is not the same
     * thing: the error estimates read 0.0, which is finite.
     */
    gps_clear_dop(&session->gpsdata.dop);
    gps_clear_fix(&session->gpsdata.fix);
    gps_clear_fix(&session->lastfix);
    gps_clear_fix(&session->newdata);
    gps_clear_fix(&session->oldfix);
    // the link the tests pretend gpsd opened: serial, 115200 8N1
    session->sourcetype = SOURCE_RS232;
    (void)cfsetospeed(&session->ttyset, B115200);
    nwrote = 0;
}

/* Wrap a payload in CASIC framing: 0xba 0xce, U2 payload length,
 * class, ID, payload, U4 checksum.  What the packetizer hands back.
 */
static size_t casic_frame(unsigned char *out, size_t outlen,
                          unsigned cls, unsigned id,
                          const unsigned char *payload, size_t plen)
{
    size_t len = plen + CASIC_OVERHEAD;

    if (outlen < len) {
        return 0;
    }
    memset(out, 0, len);
    out[0] = 0xba;
    out[1] = 0xce;
    out[2] = (unsigned char)(plen & 0xff);
    out[3] = (unsigned char)((plen >> 8) & 0xff);
    out[4] = (unsigned char)cls;
    out[5] = (unsigned char)id;
    if (NULL != payload &&
        0 < plen) {
        memcpy(&out[CASIC_PREFIX_LEN], payload, plen);
    }
    putle32(out, plen + CASIC_PREFIX_LEN,
            casic_checksum(out + 2, plen + CASIC_PREFIX_LEN - 2));
    return len;
}

// hand one message to the driver the way the core does
static gps_mask_t feed(struct gps_device_t *session, unsigned cls,
                       unsigned id, const unsigned char *payload,
                       size_t plen)
{
    session->lexer.outbuflen = casic_frame(session->lexer.outbuffer,
                                           sizeof(session->lexer.outbuffer),
                                           cls, id, payload, plen);
    session->lexer.type = CASIC_PACKET;
    return driver_casic.parse_packet(session);
}

// a CFG-PRT poll answer: one per port, describing that port
static void feed_cfg_prt(struct gps_device_t *session, unsigned char portID,
                         unsigned char proto_mask, unsigned int baud)
{
    unsigned char payload[CASIC_LEN_PRT];

    putbyte(payload, 0, portID);
    putbyte(payload, 1, proto_mask);
    putle16(payload, 2, MODE_8N1);
    putle32(payload, 4, baud);
    (void)feed(session, CLS_CFG, ID_PRT, payload, sizeof(payload));
}

/* CFG-PRT Mode field, section 3.2.2.1 Table 10.
 *
 * Reached by asking for a speed change on a link the driver was told
 * takes binary input and no text input, the state in which
 * casic_speed() sends CFG-PRT rather than $PCAS01.
 */
static struct {
    char parity;
    int stopbits;
    unsigned short mode;
} mode_tests[] = {
    /* 8N1 = 0x08c0, Table 10.  The worked example agrees: both its
     * replies, "bace0800060000ffc008802500008824c708" and
     * "bace080006000107c00800c2010009c9c708", carry mode c0 08 LE.
     */
    {'N', 1, 0x08c0},
    {'n', 1, 0x08c0},
    {'O', 1, 0x02c0},
    {'E', 1, 0x00c0},
    /* Two stop bits means seven data bits to gpsd, so bits 7:6 drop
     * from 11 to 10 and bits 13:12 go to 10: 0xc0 becomes 0x80, plus
     * 0x2000.
     */
    {'N', 2, 0x2880},
    {'O', 2, 0x2280},
    {'e', 2, 0x2080},
    // gpsd only passes N, O or E.  Anything else means no parity
    {'X', 1, 0x08c0},
};

static int check_mode_bits(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    const unsigned char *set;
    size_t setlen = 0;
    int errors = 0;
    size_t i;

    for (i = 0; i < ROWS(mode_tests); i++) {
        const unsigned char *payload;
        size_t plen = 0;
        unsigned mode;

        session_init(&context, &session);
        feed_cfg_prt(&session, 0, MASK_BIN_IN, 115200);
        nwrote = 0;

        if (!driver_casic.speed_switcher(&session, 115200,
                                         mode_tests[i].parity,
                                         mode_tests[i].stopbits)) {
            (void)printf("CFG-PRT %c%d: speed_switcher failed\n",
                         mode_tests[i].parity, mode_tests[i].stopbits);
            errors++;
            continue;
        }
        payload = sent_casic(CLS_CFG, ID_PRT, &plen);
        if (NULL == payload ||
            CASIC_LEN_PRT != plen) {
            (void)printf("CFG-PRT %c%d: no CFG-PRT Set sent\n",
                         mode_tests[i].parity, mode_tests[i].stopbits);
            errors++;
            continue;
        }
        mode = getleu16(payload, 2);
        if (mode != mode_tests[i].mode) {
            (void)printf("CFG-PRT %c%d: mode %04x, expected %04x\n",
                         mode_tests[i].parity, mode_tests[i].stopbits,
                         mode, mode_tests[i].mode);
            errors++;
        }
        // same Set carries portID 0xff and the requested baud
        if (0xff != getub(payload, 0) ||
            115200 != getleu32(payload, 4)) {
            (void)printf("CFG-PRT %c%d: portID %02x baud %u, expected "
                         "ff and 115200\n",
                         mode_tests[i].parity, mode_tests[i].stopbits,
                         getub(payload, 0),
                         (unsigned)getleu32(payload, 4));
            errors++;
        }
    }

    /* $PCAS01 carries no framing, so a port that takes text input
     * still has to be sent CFG-PRT for anything but 8N1.
     */
    session_init(&context, &session);
    feed_cfg_prt(&session, 0, MASK_BOTH_IN, 115200);
    nwrote = 0;
    (void)driver_casic.speed_switcher(&session, 115200, 'N', 1);
    if (NULL == sent_nmea("$PCAS01,") ||
        NULL != sent_casic(CLS_CFG, ID_PRT, NULL)) {
        (void)printf("text-input port, 8N1: expected $PCAS01 alone\n");
        errors++;
    }
    nwrote = 0;
    (void)driver_casic.speed_switcher(&session, 115200, 'N', 2);
    set = sent_casic(CLS_CFG, ID_PRT, &setlen);
    if (NULL == set ||
        CASIC_LEN_PRT != setlen ||
        0x2880 != getleu16(set, 2)) {
        (void)printf("text-input port, 7N2: mode %04x, expected a "
                     "CFG-PRT carrying 2880\n",
                     NULL == set ? 0 : getleu16(set, 2));
        errors++;
    }

    /* 1.5 stop bits is in Table 10 but gpsd never asks for it, and
     * nothing decides the data bits for it.  Refuse.
     */
    session_init(&context, &session);
    feed_cfg_prt(&session, 0, MASK_BIN_IN, 115200);
    nwrote = 0;
    if (driver_casic.speed_switcher(&session, 115200, 'N', 3) ||
        wrote_anything()) {
        (void)printf("CFG-PRT N3: accepted, expected refusal with "
                     "nothing on the wire\n");
        errors++;
    }
    return errors;
}

/* $PCAS01 <CMD> baud rate enum, section 2.3.1.
 *
 * Reached when the port is known to take text input, where
 * casic_speed() prefers $PCAS01 over a CFG-PRT the receiver may not
 * be listening for.
 */
static struct {
    speed_t speed;
    casic_pcas01_t cmd;
} pcas01_tests[] = {
    {4800, CASIC_PCAS01_4800},
    {9600, CASIC_PCAS01_9600},
    {19200, CASIC_PCAS01_19200},
    {38400, CASIC_PCAS01_38400},
    {57600, CASIC_PCAS01_57600},
    {115200, CASIC_PCAS01_115200},
    // rates $PCAS01 cannot express.  Nothing may go on the wire
    {2400, CASIC_PCAS01_NONE},
    {230400, CASIC_PCAS01_NONE},
    {0, CASIC_PCAS01_NONE},
};

static int check_pcas01(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    int errors = 0;
    size_t i;

    for (i = 0; i < ROWS(pcas01_tests); i++) {
        unsigned long speed = (unsigned long)pcas01_tests[i].speed;
        const char *sentence;
        bool ok;
        int cmd = -1;

        session_init(&context, &session);
        feed_cfg_prt(&session, 0, MASK_BOTH_IN, 115200);
        nwrote = 0;
        ok = driver_casic.speed_switcher(&session, pcas01_tests[i].speed,
                                         'N', 1);
        if (CASIC_PCAS01_NONE == pcas01_tests[i].cmd) {
            if (ok ||
                wrote_anything()) {
                (void)printf("$PCAS01 %lu: returned %d after %u writes, "
                             "expected refusal and silence\n",
                             speed, (int)ok, nwrote);
                errors++;
            }
            continue;
        }
        sentence = sent_nmea("$PCAS01,");
        if (!ok ||
            NULL == sentence) {
            (void)printf("$PCAS01 %lu: returned %d, sentence %s\n",
                         speed, (int)ok,
                         NULL == sentence ? "absent" : "present");
            errors++;
            continue;
        }
        if (1 != sscanf(sentence, "$PCAS01,%d*", &cmd) ||
            cmd != (int)pcas01_tests[i].cmd) {
            (void)printf("$PCAS01 %lu: sent \"%.16s\", expected CMD %d\n",
                         speed, sentence, pcas01_tests[i].cmd);
            errors++;
        }
        /* $PCAS01 only.  A CFG-PRT here would flip gpsd's baud while
         * a receiver not listening for binary stayed put.
         */
        if (NULL != sent_casic(CLS_CFG, ID_PRT, NULL)) {
            (void)printf("$PCAS01 %lu: a CFG-PRT went out as well\n", speed);
            errors++;
        }
    }
    return errors;
}

/* CFG-RATE Interval and $PCAS02 <Interval>, section 2.3.2.  Unlike
 * speed, rate is safe to send both ways, so both must appear.
 */
static struct {
    double cycletime;
    unsigned interval;          // 0: the receiver cannot produce it
} rate_tests[] = {
    // the only three the receiver can produce: 5, 2, 1 Hz
    {0.2, 200},
    {0.5, 500},
    {1.0, 1000},
    // near misses, within the driver's slop, still count
    {0.195, 200},
    {0.21, 200},
    {0.995, 1000},
    {1.01, 1000},
    /* Everything else is refused, not rounded.  gpsd records the
     * requested cycle whenever rate_switcher() returns true, so
     * rounding would report a rate the receiver is not running at.
     */
    {0.211, 0},
    {0.25, 0},
    {0.333, 0},
    {0.75, 0},
    {2.0, 0},
    {0.0, 0},
    {-1.0, 0},
};

static int check_rate(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    int errors = 0;
    size_t i;

    for (i = 0; i < ROWS(rate_tests); i++) {
        const unsigned char *payload;
        const char *sentence;
        size_t plen = 0;
        unsigned interval;
        bool ok;
        int got = -1;

        session_init(&context, &session);
        ok = driver_casic.rate_switcher(&session, rate_tests[i].cycletime);
        if (0 == rate_tests[i].interval) {
            if (ok ||
                wrote_anything()) {
                (void)printf("rate %.3f: returned %d after %u writes, "
                             "expected refusal and silence\n",
                             rate_tests[i].cycletime, (int)ok, nwrote);
                errors++;
            }
            continue;
        }
        payload = sent_casic(CLS_CFG, ID_RATE, &plen);
        sentence = sent_nmea("$PCAS02,");
        if (!ok ||
            NULL == payload ||
            CASIC_LEN_RATE != plen ||
            NULL == sentence) {
            (void)printf("rate %.3f: returned %d, CFG-RATE %s, $PCAS02 %s\n",
                         rate_tests[i].cycletime, (int)ok,
                         NULL == payload ? "absent" : "present",
                         NULL == sentence ? "absent" : "present");
            errors++;
            continue;
        }
        interval = getleu16(payload, 0);
        if (interval != rate_tests[i].interval ||
            1 != sscanf(sentence, "$PCAS02,%d*", &got) ||
            (unsigned)got != rate_tests[i].interval) {
            (void)printf("rate %.3f: CFG-RATE %u, $PCAS02 %d, expected %u "
                         "in both\n",
                         rate_tests[i].cycletime, interval, got,
                         rate_tests[i].interval);
            errors++;
        }
    }
    return errors;
}

/* Working out which ProtoMask describes the port gpsd has open.
 *
 * Nothing outside the driver reads that conclusion, it only decides
 * which command a speed switch uses, so that is what is checked.  An
 * 8N1 request goes by $PCAS01 where the port takes text input and by
 * CFG-PRT where it does not; odd parity always forces CFG-PRT, so the
 * mask itself reaches the wire.  Where the port cannot be pinned down
 * at all, nothing may be sent: returning true would re-clock gpsd.
 */
#define NO_PORT 0xff            // "no reply from this port"

// what a speed switch should put on the wire
typedef enum {
    WIRE_NONE   = 0,            // nothing at all
    WIRE_PCAS01 = 1,
    WIRE_CFG_PRT = 2,
} wire_t;

static struct {
    const char *what;
    bool set;                   // command a mode switch first
    struct {
        unsigned char id;       // portID, NO_PORT to end the list
        unsigned char mask;
        unsigned int baud;
    } reply[4];
    wire_t expect_8n1;          // for a plain 8N1 change
    wire_t expect_odd;          // for odd parity
    unsigned char expect_mask;  // ProtoMask a forced CFG-PRT carries
} port_tests[] = {
    {
        // unknown port: neither command may go
        "nothing seen at all", false,
        {{NO_PORT, 0, 0}},
        WIRE_NONE, WIRE_NONE, 0,
    },
    {
        // a Set addresses portID 0xff, so it describes our port
        "a Set, no replies yet", true,
        {{NO_PORT, 0, 0}},
        WIRE_PCAS01, WIRE_CFG_PRT, MASK_SET_BIN,
    },
    {
        // ...and outranks answers describing ports that may not be ours
        "a Set outranks a disagreeing reply", true,
        {{0, MASK_BIN_IN, 115200}, {NO_PORT, 0, 0}},
        WIRE_PCAS01, WIRE_CFG_PRT, MASK_SET_BIN,
    },
    {
        // no Set, so narrow the answers: only one port answered at all
        "no Set, one port answers", false,
        {{0, MASK_BIN_IN, 115200}, {NO_PORT, 0, 0}},
        WIRE_CFG_PRT, WIRE_CFG_PRT, MASK_BIN_IN,
    },
    {
        // our port took a binary command, so it accepts binary input
        "no Set, text-only port ignored", false,
        {{0, MASK_TXT_OUT, 115200}, {1, MASK_BIN_IN, 115200},
         {NO_PORT, 0, 0}},
        WIRE_CFG_PRT, WIRE_CFG_PRT, MASK_BIN_IN,
    },
    {
        // ...and it runs at the speed gpsd opened it at
        "no Set, baud picks the port out", false,
        {{1, MASK_DISAGREES, 9600}, {0, MASK_BIN_IN, 115200},
         {NO_PORT, 0, 0}},
        WIRE_CFG_PRT, WIRE_CFG_PRT, MASK_BIN_IN,
    },
    {
        // two ports fit and agree, so it does not matter which
        "no Set, two candidates agree", false,
        {{0, MASK_BIN_IN, 115200}, {1, MASK_BIN_IN, 115200},
         {NO_PORT, 0, 0}},
        WIRE_CFG_PRT, WIRE_CFG_PRT, MASK_BIN_IN,
    },
    {
        // two fit and disagree, so neither is usable
        "no Set, two candidates disagree", false,
        {{0, MASK_BIN_IN, 115200}, {1, MASK_DISAGREES, 115200},
         {NO_PORT, 0, 0}},
        WIRE_NONE, WIRE_NONE, 0,
    },
};

// what a speed switch actually put on the wire
static wire_t wire_seen(void)
{
    if (NULL != sent_casic(CLS_CFG, ID_PRT, NULL)) {
        return WIRE_CFG_PRT;
    }
    if (NULL != sent_nmea("$PCAS01,")) {
        return WIRE_PCAS01;
    }
    return WIRE_NONE;
}

/* $PCAS01 and CFG-PRT both re-clock the link, and a true return commits
 * gpsd's own flip, so a switch may put at most one of them on the wire.
 * wire_seen() reports the first it finds, and cannot say this alone.
 */
static bool wrote_exactly(wire_t expect)
{
    return nwrote == (WIRE_NONE == expect ? 0U : 1U);
}

static int check_ports(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    int errors = 0;
    size_t i;

    for (i = 0; i < ROWS(port_tests); i++) {
        const unsigned char *payload;
        size_t plen = 0;
        bool ok;
        wire_t seen;
        size_t j;

        session_init(&context, &session);
        if (port_tests[i].set) {
            driver_casic.mode_switcher(&session, MODE_BINARY);
        }
        for (j = 0; NO_PORT != port_tests[i].reply[j].id; j++) {
            feed_cfg_prt(&session, port_tests[i].reply[j].id,
                         port_tests[i].reply[j].mask,
                         port_tests[i].reply[j].baud);
        }

        // (a) plain 8N1 baud change
        nwrote = 0;
        ok = driver_casic.speed_switcher(&session, 115200, 'N', 1);
        seen = wire_seen();
        if (seen != port_tests[i].expect_8n1 ||
            ok != (WIRE_NONE != port_tests[i].expect_8n1) ||
            !wrote_exactly(port_tests[i].expect_8n1)) {
            (void)printf("%s: 8N1 sent %d returned %d after %u writes, "
                         "expected %d\n",
                         port_tests[i].what, (int)seen, (int)ok, nwrote,
                         (int)port_tests[i].expect_8n1);
            errors++;
        }

        // (b) odd parity forces CFG-PRT, so the mask is visible
        nwrote = 0;
        ok = driver_casic.speed_switcher(&session, 115200, 'O', 1);
        seen = wire_seen();
        if (seen != port_tests[i].expect_odd ||
            ok != (WIRE_NONE != port_tests[i].expect_odd) ||
            !wrote_exactly(port_tests[i].expect_odd)) {
            (void)printf("%s: odd parity sent %d returned %d after %u "
                         "writes, expected %d\n",
                         port_tests[i].what, (int)seen, (int)ok, nwrote,
                         (int)port_tests[i].expect_odd);
            errors++;
            continue;
        }
        if (WIRE_CFG_PRT != seen) {
            continue;
        }
        payload = sent_casic(CLS_CFG, ID_PRT, &plen);
        if (NULL == payload ||
            CASIC_LEN_PRT != plen) {
            (void)printf("%s: short CFG-PRT\n", port_tests[i].what);
            errors++;
            continue;
        }
        if (getub(payload, 1) != port_tests[i].expect_mask) {
            (void)printf("%s: ProtoMask %02x, expected %02x\n",
                         port_tests[i].what, getub(payload, 1),
                         port_tests[i].expect_mask);
            errors++;
        }
    }
    return errors;
}

/* mode_switcher only changes the output bits, then polls so the reply
 * can be matched back to our port.
 */
static int check_mode_switch(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    const unsigned char *payload;
    size_t plen = 0;
    int errors = 0;

    // a port that emits NMEA only, and takes commands both ways
    session_init(&context, &session);
    feed_cfg_prt(&session, 0, CASIC_PROTO_IN_BOTH | CASIC_PROTO_OUT_TXT,
                 115200);
    nwrote = 0;

    driver_casic.mode_switcher(&session, MODE_BINARY);
    payload = sent_casic(CLS_CFG, ID_PRT, &plen);
    if (NULL == payload ||
        CASIC_LEN_PRT != plen) {
        (void)printf("mode switch: no CFG-PRT Set sent\n");
        return errors + 1;
    }
    // input bits preserved, output flipped to binary
    if (MASK_SET_BIN != getub(payload, 1)) {
        (void)printf("mode switch to binary: ProtoMask %02x, expected %02x\n",
                     getub(payload, 1), MASK_SET_BIN);
        errors++;
    }
    // and must not disturb the line: same baud we are clocked at
    if (115200 != getleu32(payload, 4)) {
        (void)printf("mode switch: baud %u, expected 115200\n",
                     (unsigned)getleu32(payload, 4));
        errors++;
    }
    // a poll follows, safe only because the baud is unchanged
    if (2 != nwrote) {
        (void)printf("mode switch: %u writes, expected a Set and a poll\n",
                     nwrote);
        errors++;
    }

    session_init(&context, &session);
    feed_cfg_prt(&session, 0, MASK_BIN_IN, 115200);
    nwrote = 0;
    driver_casic.mode_switcher(&session, MODE_NMEA);
    payload = sent_casic(CLS_CFG, ID_PRT, &plen);
    if (NULL == payload ||
        MASK_SET_TXT != getub(payload, 1)) {
        (void)printf("mode switch to NMEA: ProtoMask %02x, expected %02x\n",
                     NULL == payload ? 0 : getub(payload, 1), MASK_SET_TXT);
        errors++;
    }
    return errors;
}

/* -b (readonly) stops every switcher: nothing goes out, the two that
 * answer say no, and mode_switcher must not record a ProtoMask it never
 * sent -- casic_write() returns true under readonly, and
 * casic_our_proto_mask() trusts last_commanded first.
 *
 * -p (passive) must not stop them.  It suppresses autoconfiguration,
 * which is event_hook()'s job, but still allows the manual changes that
 * reach a switcher through ?DEVICE.
 */
static struct {
    const char *what;
    bool readonly;
    bool passive;
    bool expect_write;
} no_config_tests[] = {
    {"readonly", true, false, false},
    {"passive", false, true, true},
};

static int check_no_config(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    int errors = 0;
    size_t i;

    for (i = 0; i < ROWS(no_config_tests); i++) {
        session_init(&context, &session);
        // a port that takes commands both ways, so only the flag refuses
        feed_cfg_prt(&session, 0, MASK_BOTH_IN, 115200);
        context.readonly = no_config_tests[i].readonly;
        context.passive = no_config_tests[i].passive;
        nwrote = 0;

        if (driver_casic.speed_switcher(&session, 9600, 'N', 1) !=
                no_config_tests[i].expect_write) {
            (void)printf("%s: speed_switcher said %d\n",
                         no_config_tests[i].what,
                         (int)!no_config_tests[i].expect_write);
            errors++;
        }
        if (driver_casic.rate_switcher(&session, 1.0) !=
                no_config_tests[i].expect_write) {
            (void)printf("%s: rate_switcher said %d\n",
                         no_config_tests[i].what,
                         (int)!no_config_tests[i].expect_write);
            errors++;
        }
        driver_casic.mode_switcher(&session, MODE_BINARY);

        if (wrote_anything() != no_config_tests[i].expect_write) {
            (void)printf("%s: %u writes, expected %s\n",
                         no_config_tests[i].what, nwrote,
                         no_config_tests[i].expect_write ? "some" : "none");
            errors++;
        }
        if (session.driver.casic.last_commanded_valid !=
                no_config_tests[i].expect_write) {
            (void)printf("%s: last_commanded_valid %d\n",
                         no_config_tests[i].what,
                         (int)session.driver.casic.last_commanded_valid);
            errors++;
        }
    }
    return errors;
}

/* NAV-*INFO decoding, against frames captured from an AT6558R.  The
 * expected elevation, azimuth and SNR are what the same device put in
 * its $GxGSV for the same epoch.  Both frames carry the same run time,
 * so the satellites accumulate into one sky.
 */
static const char nav_gpsinfo[] =
    "99e2e40b0b0a00000a01c16320132d0005e086c01006c1611d0f9c00529d81c0"
    "ff0c40000005100100000000120ec121192c530022245d401d0fc1211616fe00"
    "7105d9411111c1611b444a00ab48dfc00c13c1611e41c10012f85cc00814c121"
    "1115e000461344401616c163213e4600ccb6123f1c18c1611e2b2d017fc655c0"
    "0e1ec1611b12910055b711c0";

static const char nav_bdsinfo[] =
    "99e2e40b04030100011bc1611c324d005beff9bf131cc1611c112a0078bca03f"
    "0d1ec1611f2dab0050acb2bb1b25c0200608720000000000";

// what $GxGSV said about the same epoch
static struct {
    int16_t PRN;
    double el, az, ss;
    bool used;
} sat_expect[] = {
    // NAV-GPSINFO: 11 in view, 10 used.  svid 12 is not tracked.
    {  1, 19,  45, 32, true},
    {  6, 15, 156, 29, true},
    { 12,  5, 272,  0, false},
    { 14, 44,  83, 25, true},
    { 15, 22, 254, 22, true},
    { 17, 68,  74, 27, true},
    { 19, 65, 193, 30, true},
    { 20, 21, 224, 17, true},
    { 22, 62,  70, 33, true},
    { 24, 43, 301, 30, true},
    { 30, 18, 145, 27, true},
    /* NAV-BDSINFO: 4 in view, 3 used.  svid 37 is tracked, cno 6, but
     * not in the fix, which is what pins status bit 0 to "used".
     */
    {427, 50,  77, 28, true},
    {428, 17,  42, 28, true},
    {430, 45, 171, 31, true},
    {437,  8, 114,  6, false},
};

static size_t unhex(const char *hex, unsigned char *out, size_t outlen)
{
    size_t n = 0;

    while ('\0' != hex[0] &&
           '\0' != hex[1] &&
           n < outlen) {
        unsigned byte;

        if (1 != sscanf(hex, "%2x", &byte)) {
            break;
        }
        out[n++] = (unsigned char)byte;
        hex += 2;
    }
    return n;
}

static int check_svinfo(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    unsigned char buf[256];
    int errors = 0;
    size_t len, i;

    session_init(&context, &session);

    // GPS first, then BeiDou: same epoch, so they accumulate into one sky
    len = unhex(nav_gpsinfo, buf, sizeof(buf));
    (void)feed(&session, CLS_NAV, ID_GPSINFO, buf, len);
    if (11 != session.gpsdata.satellites_visible) {
        (void)printf("NAV-GPSINFO: visible %u, expected 11\n",
                     session.gpsdata.satellites_visible);
        errors++;
    }

    len = unhex(nav_bdsinfo, buf, sizeof(buf));
    (void)feed(&session, CLS_NAV, ID_BDSINFO, buf, len);
    if (15 != session.gpsdata.satellites_visible) {
        (void)printf("after NAV-BDSINFO: visible %u, expected 15 "
                     "(11 GPS + 4 BeiDou accumulated)\n",
                     session.gpsdata.satellites_visible);
        errors++;
    }
    if (13 != session.gpsdata.satellites_used) {
        (void)printf("used %d, expected 13 (10 GPS + 3 BeiDou)\n",
                     session.gpsdata.satellites_used);
        errors++;
    }

    for (i = 0; i < ROWS(sat_expect) &&
                i < (size_t)session.gpsdata.satellites_visible; i++) {
        const struct satellite_t *s = &session.gpsdata.skyview[i];

        if (sat_expect[i].PRN != s->PRN ||
            sat_expect[i].el != s->elevation ||
            sat_expect[i].az != s->azimuth ||
            sat_expect[i].ss != s->ss ||
            sat_expect[i].used != s->used) {
            (void)printf("sat %zu: PRN %d el %.0f az %.0f ss %.0f used %d, "
                         "expected PRN %d el %.0f az %.0f ss %.0f used %d\n",
                         i, s->PRN, s->elevation, s->azimuth, s->ss,
                         (int)s->used,
                         sat_expect[i].PRN, sat_expect[i].el,
                         sat_expect[i].az, sat_expect[i].ss,
                         (int)sat_expect[i].used);
            errors++;
        }
    }
    return errors;
}

/* NAV-PV and NAV-TIMEUTC, captured from an AT6558R.  The expected
 * values are what the same device reported over NMEA for that epoch.
 */
static const char nav_pv[] =
    "27e4e40b" "07070711" "0a030400" "f72ca33f"      // tow, valid, numSV, pDOP
    "6b6a6ca2e3945ec0"                               // longitude
    "24ab44d1bbd44740"                               // latitude
    "898a5342" "d1dfacc1" "fa240941" "dd788541"      // alt, geoid, hAcc, vAcc
    "00000000" "00000000" "00000000"                 // velN, velE, velU
    "00000000" "00000000"                            // speed, gspeed
    "442c8f43" "90d4103d" "00247449";                // heading, sAcc, cAcc

static const char nav_timeutc[] =
    "27e4e40b" "b5ff2041" "86649c39"                 // tow, tAcc, mAcc
    // ms, year, month, day, hour, minute, second, valid, timeSrc,
    // dateValid
    "2003" "ea07" "08" "05" "07" "13" "1f" "07" "00" "03";

static int check_pv_time(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    unsigned char buf[128];
    int errors = 0;
    gps_mask_t mask;
    size_t len;

    session_init(&context, &session);

    len = unhex(nav_pv, buf, sizeof(buf));
    if (CASIC_LEN_PV != len) {
        (void)printf("NAV-PV vector is %zu bytes, expected %d\n",
                     len, CASIC_LEN_PV);
        return 1;
    }
    session.newdata.epd = NAN;
    mask = feed(&session, CLS_NAV, ID_PV, buf, len);
    // GGA/RMC for the same epoch: 47.6619817N 122.3263937W, track 286.35
    if (0.0000001 < fabs(47.6619817 - session.newdata.latitude) ||
        0.0000001 < fabs(-122.3263937 - session.newdata.longitude)) {
        (void)printf("NAV-PV: lat %.7f lon %.7f, expected 47.6619817 "
                     "-122.3263937\n",
                     session.newdata.latitude, session.newdata.longitude);
        errors++;
    }
    // GGA said geoidSep -21.6
    if (0.01 < fabs(-21.609 - session.newdata.geoid_sep)) {
        (void)printf("NAV-PV: geoid_sep %.3f, expected -21.609\n",
                     session.newdata.geoid_sep);
        errors++;
    }
    // RMC said track 286.35
    if (0.01 < fabs(286.346 - session.newdata.track)) {
        (void)printf("NAV-PV: track %.3f, expected 286.346\n",
                     session.newdata.track);
        errors++;
    }
    if (MODE_3D != session.newdata.mode) {
        (void)printf("NAV-PV: mode %d, expected %d\n",
                     session.newdata.mode, MODE_3D);
        errors++;
    }
    if (17 != session.gpsdata.satellites_used) {
        (void)printf("NAV-PV: numSV %d, expected 17\n",
                     session.gpsdata.satellites_used);
        errors++;
    }
    /* cAcc here is 00 24 74 49, exactly 1e6: the "heading accuracy
     * unknown" sentinel, not a 1000000 degree error estimate.  It must
     * not reach epd, and TRACKERR_SET must not claim it did.
     */
    if (0 != isfinite(session.newdata.epd) ||
        0 != (mask & TRACKERR_SET)) {
        (void)printf("NAV-PV: epd %f mask TRACKERR %d, expected unset "
                     "(cAcc is the 1e6 unknown sentinel)\n",
                     session.newdata.epd, (int)(0 != (mask & TRACKERR_SET)));
        errors++;
    }

    /* posValid is an enumeration, CASIC section 2.7.3 Remark [1], not
     * a bitmask.  A 2D fix has no height, so ALTITUDE_SET and VERR_SET
     * must stay clear.
     */
    len = unhex(nav_pv, buf, sizeof(buf));
    buf[4] = CASIC_FIX_2D;
    session.newdata.altHAE = NAN;
    mask = feed(&session, CLS_NAV, ID_PV, buf, len);
    if (MODE_2D != session.newdata.mode ||
        0 == (mask & (LATLON_SET | HERR_SET)) ||
        0 != (mask & (ALTITUDE_SET | VERR_SET)) ||
        0 != isfinite(session.newdata.altHAE)) {
        (void)printf("NAV-PV: posValid 6 gave mode %d mask x%llx, expected "
                     "MODE_2D with position but no height\n",
                     session.newdata.mode, (unsigned long long)mask);
        errors++;
    }

    // GNSS + dead reckoning is a 3D fix, flagged STATUS_GNSSDR
    len = unhex(nav_pv, buf, sizeof(buf));
    buf[4] = CASIC_FIX_GNSSDR;
    (void)feed(&session, CLS_NAV, ID_PV, buf, len);
    if (MODE_3D != session.newdata.mode ||
        STATUS_GNSSDR != session.newdata.status) {
        (void)printf("NAV-PV: posValid 8 gave mode %d status %d, expected "
                     "MODE_3D and STATUS_GNSSDR\n",
                     session.newdata.mode, session.newdata.status);
        errors++;
    }

    /* "Last position held over" is not a computed fix, so it must fall
     * to no-fix and publish nothing.
     */
    len = unhex(nav_pv, buf, sizeof(buf));
    buf[4] = CASIC_FIX_HOLD;                    // posValid
    buf[5] = CASIC_FIX_HOLD;                    // velValid
    session.newdata.latitude = NAN;
    session.newdata.track = NAN;
    mask = feed(&session, CLS_NAV, ID_PV, buf, len);
    if (MODE_NO_FIX != session.newdata.mode ||
        0 != (mask & (LATLON_SET | ALTITUDE_SET | TRACK_SET | SPEED_SET)) ||
        0 != isfinite(session.newdata.latitude) ||
        0 != isfinite(session.newdata.track)) {
        (void)printf("NAV-PV: posValid/velValid 3 gave mode %d mask x%llx, "
                     "expected MODE_NO_FIX and no position or velocity\n",
                     session.newdata.mode, (unsigned long long)mask);
        errors++;
    }

    /* hAcc, vAcc and sAcc are variances, CASIC section 2.7.4, so the
     * driver reports their square roots.  Plant 4, 9 and 16, expect
     * 2, 3 and 4.  hAcc and vAcc are variances on orthogonal axes, so
     * sep is the root of their sum, sqrt(4 + 9).
     */
    len = unhex(nav_pv, buf, sizeof(buf));
    putle32(buf, 40, 0x40800000);              // hAcc  4.0 m^2
    putle32(buf, 44, 0x41100000);              // vAcc  9.0 m^2
    putle32(buf, 72, 0x41800000);              // sAcc 16.0 (m/s)^2
    (void)feed(&session, CLS_NAV, ID_PV, buf, len);
    if (0.0001 < fabs(2.0 - session.newdata.eph) ||
        0.0001 < fabs(3.0 - session.newdata.epv) ||
        0.0001 < fabs(4.0 - session.newdata.eps)) {
        (void)printf("NAV-PV: eph %.4f epv %.4f eps %.4f, expected the "
                     "square roots 2 3 4\n", session.newdata.eph,
                     session.newdata.epv, session.newdata.eps);
        errors++;
    }
    if (0 == isfinite(session.newdata.sep) ||
        0.0001 < fabs(sqrt(13.0) - session.newdata.sep)) {
        (void)printf("NAV-PV: sep %.4f, expected sqrt(4 + 9) %.4f\n",
                     session.newdata.sep, sqrt(13.0));
        errors++;
    }

    /* A variance that is not positive is a "not computed" sentinel,
     * not a perfect measurement.  Leaving those unset is what lets
     * gpsd_error_model() substitute its own DOP estimate; a 0.0 would
     * instead reach the client as a claim of no error at all.  sep is
     * built from eph and epv, so it has to drop out with them.
     */
    len = unhex(nav_pv, buf, sizeof(buf));
    putle32(buf, 40, 0);                       // hAcc  0, not computed
    putle32(buf, 44, 0xbf800000);              // vAcc -1.0, impossible
    putle32(buf, 72, 0);                       // sAcc  0, not computed
    session.newdata.eph = session.newdata.epv = NAN;
    session.newdata.eps = session.newdata.sep = NAN;
    (void)feed(&session, CLS_NAV, ID_PV, buf, len);
    if (0 != isfinite(session.newdata.eph) ||
        0 != isfinite(session.newdata.epv) ||
        0 != isfinite(session.newdata.eps) ||
        0 != isfinite(session.newdata.sep)) {
        (void)printf("NAV-PV: hAcc 0 vAcc -1 sAcc 0 gave eph %.4f epv %.4f "
                     "eps %.4f sep %.4f, expected all left unset\n",
                     session.newdata.eph, session.newdata.epv,
                     session.newdata.eps, session.newdata.sep);
        errors++;
    }

    /* velU at offset 56 is up, NED.velD is positive down, so the
     * driver negates it.  Every captured frame has velU 0, so plant
     * one: +2.5 m/s up must become -2.5 m/s down.
     */
    len = unhex(nav_pv, buf, sizeof(buf));
    putle32(buf, 56, 0x40200000);              // velU +2.5 m/s
    (void)feed(&session, CLS_NAV, ID_PV, buf, len);
    if (0.0001 < fabs(-2.5 - session.newdata.NED.velD)) {
        (void)printf("NAV-PV: velU +2.5 gave velD %.4f, expected -2.5\n",
                     session.newdata.NED.velD);
        errors++;
    }

    // a runt NAV-PV must be dropped, not decoded from short data
    session.newdata.latitude = NAN;
    mask = feed(&session, CLS_NAV, ID_PV, buf, (size_t)(CASIC_LEN_PV - 4));
    if (0 != mask ||
        0 != isfinite(session.newdata.latitude)) {
        (void)printf("NAV-PV: a %d byte payload gave mask x%llx, expected "
                     "it dropped\n",
                     CASIC_LEN_PV - 4, (unsigned long long)mask);
        errors++;
    }

    len = unhex(nav_timeutc, buf, sizeof(buf));
    if (CASIC_LEN_TIMEUTC != len) {
        (void)printf("NAV-TIMEUTC vector is %zu bytes, expected %d\n",
                     len, CASIC_LEN_TIMEUTC);
        return errors + 1;
    }
    mask = feed(&session, CLS_NAV, ID_TIMEUTC, buf, len);
    // captured at 2026-08-05 07:19:31.800 UTC
    if (1785914371 != (long long)session.newdata.time.tv_sec ||
        800000000L != session.newdata.time.tv_nsec ||
        (TIME_SET | NTPTIME_IS) != (mask & (TIME_SET | NTPTIME_IS))) {
        (void)printf("NAV-TIMEUTC: %lld.%09ld, expected 1785914371.800000000 "
                     "(2026-08-05T07:19:31.8Z) with TIME_SET|NTPTIME_IS\n",
                     (long long)session.newdata.time.tv_sec,
                     session.newdata.time.tv_nsec);
        errors++;
    }

    /* valid is a bitmask, CASIC section 2.7.5 Remark [1].  All three
     * bits are needed for UTC, so with the leap second clear the time
     * is not yet UTC and must not be published.
     */
    len = unhex(nav_timeutc, buf, sizeof(buf));
    buf[21] = CASIC_TIME_TOW | CASIC_TIME_WEEK;         // no leap second
    session.newdata.time.tv_sec = 0;
    session.newdata.time.tv_nsec = 0;
    mask = feed(&session, CLS_NAV, ID_TIMEUTC, buf, len);
    if (0 != (mask & (TIME_SET | NTPTIME_IS)) ||
        0 != session.newdata.time.tv_sec) {
        (void)printf("NAV-TIMEUTC: valid x03 published time %lld.%09ld "
                     "mask x%llx, expected neither\n",
                     (long long)session.newdata.time.tv_sec,
                     session.newdata.time.tv_nsec,
                     (unsigned long long)mask);
        errors++;
    }
    // a cold receiver's all-zero valid is the quiet case, still no time
    len = unhex(nav_timeutc, buf, sizeof(buf));
    buf[21] = 0x00;
    mask = feed(&session, CLS_NAV, ID_TIMEUTC, buf, len);
    if (0 != (mask & (TIME_SET | NTPTIME_IS))) {
        (void)printf("NAV-TIMEUTC: valid x00 mask x%llx, expected no time\n",
                     (unsigned long long)mask);
        errors++;
    }
    /* No date, CASIC section 2.7.5 Remark [3].  A full valid mask with
     * no date still cannot make a timestamp.
     */
    len = unhex(nav_timeutc, buf, sizeof(buf));
    buf[23] = CASIC_DATE_NONE;
    mask = feed(&session, CLS_NAV, ID_TIMEUTC, buf, len);
    if (0 != (mask & (TIME_SET | NTPTIME_IS))) {
        (void)printf("NAV-TIMEUTC: dateValid 0 mask x%llx, expected no "
                     "time\n", (unsigned long long)mask);
        errors++;
    }
    return errors;
}

/* Epoch handling in casic_parse(), which decides where a reporting
 * cycle begins and ends and so which messages gpsd coalesces into one
 * TPV and one SKY.  Synthetic frames, fed message by message.
 */

// messages check_epoch() replays, counted into the summary line
#define EPOCH_MSGS      15
static unsigned epoch_msgs = 0;

// a NAV message carrying a run time at payload offset 0
static gps_mask_t feed_nav(struct gps_device_t *session, unsigned cls,
                           unsigned id, uint32_t tow,
                           const unsigned char *payload, size_t plen)
{
    unsigned char body[128];

    memset(body, 0, sizeof(body));
    if (NULL != payload &&
        0 < plen) {
        memcpy(body, payload, plen);
    }
    putle32(body, 0, tow);
    epoch_msgs++;
    return feed(session, cls, id, body, plen);
}

// a NAV-GPSINFO payload carrying nsats tracked, all of them used
static size_t gpsinfo_payload(unsigned char *out, unsigned nsats)
{
    unsigned i;

    memset(out, 0, CASIC_LEN_SVINFO + (CASIC_LEN_SVINFO_SV * nsats));
    out[4] = (unsigned char)nsats;      // numViewSv
    // numFixSv must agree with the used bits, or the decoder warns
    out[5] = (unsigned char)nsats;
    for (i = 0; i < nsats; i++) {
        unsigned char *r = &out[CASIC_LEN_SVINFO + (CASIC_LEN_SVINFO_SV * i)];

        r[0] = (unsigned char)i;                // chn
        r[1] = (unsigned char)(i + 1);          // svid
        r[2] = CASIC_SV_USED;                   // status
        r[4] = 30;                              // cno
        r[5] = 45;                              // elev
        r[6] = 90;                              // azim
    }
    return CASIC_LEN_SVINFO + (CASIC_LEN_SVINFO_SV * nsats);
}

static int check_epoch(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    unsigned char sky[CASIC_LEN_SVINFO + (CASIC_LEN_SVINFO_SV * 2)];
    size_t skylen = gpsinfo_payload(sky, 2);
    gps_mask_t mask;
    int errors = 0;

    session_init(&context, &session);

    /* (a) Bootstrap.  Nothing came before the first NAV message, so
     * nothing says where its epoch ends.  REPORT_IS here would ship a
     * TPV built from one message of an incomplete cycle.
     */
    mask = feed_nav(&session, CLS_NAV, ID_DOP, 1000, NULL, CASIC_LEN_DOP);
    if (0 != (mask & REPORT_IS) ||
        0 != session.driver.casic.end_msgid) {
        (void)printf("epoch bootstrap: mask x%llx ender x%04x, expected no "
                     "REPORT_IS and no ender yet\n",
                     (unsigned long long)mask,
                     session.driver.casic.end_msgid);
        errors++;
    }
    // the rest of that first epoch: still nothing to report against
    mask = feed_nav(&session, CLS_NAV, ID_GPSINFO, 1000, sky, skylen);
    mask |= feed_nav(&session, CLS_NAV, ID_PV, 1000, NULL, CASIC_LEN_PV);
    if (0 != (mask & REPORT_IS)) {
        (void)printf("epoch bootstrap: REPORT_IS during the first epoch\n");
        errors++;
    }
    if (2 != session.gpsdata.satellites_visible) {
        (void)printf("epoch bootstrap: visible %u, expected 2\n",
                     session.gpsdata.satellites_visible);
        errors++;
    }

    /* (b) Normal operation.  The second epoch's first message shows
     * the run time moved on, identifying whatever arrived last in the
     * first epoch, NAV-PV, as the cycle ender.  From here REPORT_IS
     * fires once per epoch, on that message.
     */
    mask = feed_nav(&session, CLS_NAV, ID_DOP, 2000, NULL, CASIC_LEN_DOP);
    if (0 == (mask & CLEAR_IS) ||
        0 != (mask & REPORT_IS) ||
        MSGID_OF(CLS_NAV, ID_PV) != session.driver.casic.end_msgid) {
        (void)printf("epoch 2 start: mask x%llx ender x%04x, expected "
                     "CLEAR_IS, no REPORT_IS, ender x%04x\n",
                     (unsigned long long)mask,
                     session.driver.casic.end_msgid,
                     MSGID_OF(CLS_NAV, ID_PV));
        errors++;
    }
    // the boundary also clears the sky the previous epoch accumulated
    if (0 != session.gpsdata.satellites_visible) {
        (void)printf("epoch 2 start: visible %u, expected the sky cleared\n",
                     session.gpsdata.satellites_visible);
        errors++;
    }

    mask = feed_nav(&session, CLS_NAV, ID_GPSINFO, 2000, sky, skylen);
    if (0 != (mask & (REPORT_IS | SATELLITE_SET))) {
        // SKY is raised once, at the ender, not per constellation
        (void)printf("epoch 2 middle: mask x%llx, expected neither REPORT_IS "
                     "nor SATELLITE_SET before the ender\n",
                     (unsigned long long)mask);
        errors++;
    }
    mask = feed_nav(&session, CLS_NAV, ID_PV, 2000, NULL, CASIC_LEN_PV);
    if (0 == (mask & REPORT_IS) ||
        0 == (mask & SATELLITE_SET)) {
        (void)printf("epoch 2 ender: mask x%llx, expected REPORT_IS and "
                     "SATELLITE_SET\n", (unsigned long long)mask);
        errors++;
    }

    /* The run time may jitter a few ms inside one epoch.  That must
     * not read as a new cycle, which would split one epoch's messages
     * across two reports.
     */
    mask = feed_nav(&session, CLS_NAV, ID_DOP, 3000, NULL, CASIC_LEN_DOP);
    if (0 == (mask & CLEAR_IS)) {
        (void)printf("epoch 3 start: mask x%llx, expected CLEAR_IS\n",
                     (unsigned long long)mask);
        errors++;
    }
    mask = feed_nav(&session, CLS_NAV, ID_GPSINFO, 3007, sky, skylen);
    if (0 != (mask & CLEAR_IS) ||
        3000 != session.driver.casic.last_tow) {
        (void)printf("epoch 3 jitter: tow 3007 gave mask x%llx last_tow "
                     "%lld, expected the same epoch and last_tow 3000\n",
                     (unsigned long long)mask,
                     (long long)session.driver.casic.last_tow);
        errors++;
    }

    /* A NAV message with no decoder sits the epoch logic out: its
     * first word is not known to be a run time, so reading one would
     * fabricate a boundary.  Same for a non-NAV message.
     */
    mask = feed_nav(&session, CLS_NAV, ID_SOL, 987654, NULL, CASIC_LEN_SVINFO);
    if (0 != (mask & (CLEAR_IS | REPORT_IS)) ||
        3000 != session.driver.casic.last_tow ||
        MSGID_OF(CLS_NAV, ID_GPSINFO) !=
            (int)session.driver.casic.last_msgid) {
        (void)printf("undecoded NAV-SOL: mask x%llx last_tow %lld last_msgid "
                     "x%04x, expected the epoch state untouched\n",
                     (unsigned long long)mask,
                     (long long)session.driver.casic.last_tow,
                     session.driver.casic.last_msgid);
        errors++;
    }
    mask = feed_nav(&session, CLS_ACK, ID_ACK, 555, NULL, CASIC_LEN_ACK);
    if (0 != (mask & (CLEAR_IS | REPORT_IS)) ||
        3000 != session.driver.casic.last_tow) {
        (void)printf("ACK-ACK: mask x%llx last_tow %lld, expected the epoch "
                     "state untouched\n",
                     (unsigned long long)mask,
                     (long long)session.driver.casic.last_tow);
        errors++;
    }
    mask = feed_nav(&session, CLS_NAV, ID_PV, 3000, NULL, CASIC_LEN_PV);
    if (0 == (mask & REPORT_IS)) {
        (void)printf("epoch 3 ender: mask x%llx, expected REPORT_IS. "
                     "Neither the run time nor the ender should move\n",
                     (unsigned long long)mask);
        errors++;
    }

    /* (c) The messages in an epoch change: NAV-PV stops arriving.
     * The learned ender never comes, so that epoch goes unreported,
     * but the driver must relearn from whatever now arrives last
     * rather than waiting forever.
     */
    mask = feed_nav(&session, CLS_NAV, ID_DOP, 4000, NULL, CASIC_LEN_DOP);
    mask |= feed_nav(&session, CLS_NAV, ID_GPSINFO, 4000, sky, skylen);
    if (0 != (mask & REPORT_IS)) {
        (void)printf("epoch 4: REPORT_IS without the learned ender\n");
        errors++;
    }
    (void)feed_nav(&session, CLS_NAV, ID_DOP, 5000, NULL, CASIC_LEN_DOP);
    if (MSGID_OF(CLS_NAV, ID_GPSINFO) != session.driver.casic.end_msgid) {
        (void)printf("epoch 5 start: ender x%04x, expected it relearned as "
                     "x%04x\n",
                     session.driver.casic.end_msgid,
                     MSGID_OF(CLS_NAV, ID_GPSINFO));
        errors++;
    }
    mask = feed_nav(&session, CLS_NAV, ID_GPSINFO, 5000, sky, skylen);
    if (0 == (mask & REPORT_IS)) {
        (void)printf("epoch 5 ender: mask x%llx, expected REPORT_IS from the "
                     "ender relearned as NAV-GPSINFO\n",
                     (unsigned long long)mask);
        errors++;
    }

    if (EPOCH_MSGS != epoch_msgs) {
        (void)printf("epoch scenario replayed %u messages, expected %u\n",
                     epoch_msgs, (unsigned)EPOCH_MSGS);
        errors++;
    }
    return errors;
}

int main(int argc, char **argv)
{
    int errors = 0;

    if (1 < argc &&
        0 == strcmp("--quiet", argv[1])) {
        quiet = true;
    }

    errors += check_mode_bits();
    errors += check_pcas01();
    errors += check_rate();
    errors += check_ports();
    errors += check_mode_switch();
    errors += check_no_config();
    errors += check_svinfo();
    errors += check_pv_time();
    errors += check_epoch();

    if (0 < nlost) {
        // the capture overflowed, so every write count above is suspect
        (void)printf("capture lost %u writes, MAX_WRITES/MAX_WRITE_LEN "
                     "too small\n", nlost);
        errors++;
    }

    if (0 < errors) {
        (void)printf("test_casic: %d errors\n", errors);
        return 1;
    }
    if (!quiet) {
        (void)printf("test_casic: %zu cases, all OK\n",
                     ROWS(mode_tests) + ROWS(pcas01_tests) +
                     ROWS(rate_tests) + ROWS(port_tests) +
                     ROWS(no_config_tests) + ROWS(sat_expect) + EPOCH_MSGS);
    }
    return 0;
}

// vim: set expandtab shiftwidth=4
