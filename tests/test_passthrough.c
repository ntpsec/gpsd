/*
 * Unit test for the JSON passthrough driver.
 *
 * A gpsd reading another gpsd (a gpsd:// source) passes its JSON on to
 * clients, after json_pass_packet() puts the source's path and a '#'
 * in front of each device name, so clients can tell where it came
 * from.  Packets go in through driver_json_passthrough.parse_packet(),
 * left in lexer.outbuffer as the packetizer would leave them.
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "../include/gpsd.h"

#define SOURCE          "gpsd://example.com:2947"
// the field after outbuffer and outbuflen, which must not change
#define SENTINEL        0x5a5a5a5aUL

static bool quiet = false;

extern const struct gps_type_t driver_json_passthrough;

static void session_init(struct gps_context_t *context,
                         struct gps_device_t *session)
{
    memset(context, 0, sizeof(*context));
    memset(session, 0, sizeof(*session));
    errout_reset(&context->errout);
    session->context = context;
    (void)strlcpy(session->gpsdata.dev.path, SOURCE,
                  sizeof(session->gpsdata.dev.path));
    session->lexer.char_counter = SENTINEL;
}

/* Hand the driver one packet and compare what it passes on.  NULL as
 * expect means the packet must pass unchanged.
 */
static int pass(const char *legend, struct gps_device_t *session,
                const char *expect)
{
    char *outbuf = (char *)session->lexer.outbuffer;
    char sent[sizeof(session->lexer.outbuffer)];
    gps_mask_t mask;

    (void)strlcpy(sent, outbuf, sizeof(sent));
    if (NULL == expect) {
        expect = sent;
    }
    mask = driver_json_passthrough.parse_packet(session);

    if (PASSTHROUGH_IS != mask ||
        SENTINEL != session->lexer.char_counter ||
        strlen(outbuf) != session->lexer.outbuflen ||
        0 != strcmp(outbuf, expect)) {
        size_t elen = strlen(expect);
        size_t glen = strlen(outbuf);

        // the ends of the packets, where the rewrites near the bound are
        (void)printf("%s: mask %#llx, outbuflen %zu, strlen %zu, "
                     "char_counter %#lx\n  expected ...%s\n  got      ...%s\n",
                     legend, (unsigned long long)mask,
                     session->lexer.outbuflen, glen,
                     session->lexer.char_counter,
                     expect + (80 < elen ? elen - 80 : 0),
                     outbuf + (80 < glen ? glen - 80 : 0));
        return 1;
    }
    return 0;
}

static struct {
    const char *legend;
    const char *packet;
    const char *expect;         // NULL: passed unchanged
} rewrite_tests[] = {
    {"TPV",
     "{\"class\":\"TPV\",\"device\":\"/dev/ttyS0\",\"mode\":3}",
     "{\"class\":\"TPV\",\"device\":\"" SOURCE "#/dev/ttyS0\",\"mode\":3}"},
    {"DEVICE",
     "{\"class\":\"DEVICE\",\"path\":\"/dev/ttyS0\",\"driver\":\"u-blox\"}",
     "{\"class\":\"DEVICE\",\"path\":\"" SOURCE "#/dev/ttyS0\","
     "\"driver\":\"u-blox\"}"},
    // every path, the last one ending within a path's length of the end
    {"DEVICES",
     "{\"class\":\"DEVICES\",\"devices\":["
     "{\"class\":\"DEVICE\",\"path\":\"/dev/ttyS0\"},"
     "{\"class\":\"DEVICE\",\"path\":\"/dev/ttyS1\"}]}",
     "{\"class\":\"DEVICES\",\"devices\":["
     "{\"class\":\"DEVICE\",\"path\":\"" SOURCE "#/dev/ttyS0\"},"
     "{\"class\":\"DEVICE\",\"path\":\"" SOURCE "#/dev/ttyS1\"}],"
     "\"remote\":\"" SOURCE "\"}"},
    // already rewritten, as gpsmon may hand it back
    {"TPV seen before",
     "{\"class\":\"TPV\",\"device\":\"" SOURCE "#/dev/ttyS0\"}",
     NULL},
};

static int check_rewrite(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    int errors = 0;
    size_t i;

    for (i = 0; i < ROWS(rewrite_tests); i++) {
        session_init(&context, &session);
        session.lexer.outbuflen = strlcpy((char *)session.lexer.outbuffer,
                                          rewrite_tests[i].packet,
                                          sizeof(session.lexer.outbuffer));
        errors += pass(rewrite_tests[i].legend, &session,
                       rewrite_tests[i].expect);
    }
    return errors;
}

/* The packetizer takes a NUL inside a string like any other byte, but
 * the packet passed on ends there, whatever follows is rewritten or not.
 */
static int check_embedded_nul(void)
{
    static const char packet[] =
        "{\"class\":\"TPV\",\"tag\":\"a\0\",\"device\":\"/dev/ttyS0\"}";
    struct gps_context_t context;
    struct gps_device_t session;

    session_init(&context, &session);
    memcpy(session.lexer.outbuffer, packet, sizeof(packet));
    session.lexer.outbuflen = sizeof(packet) - 1;
    return pass("TPV with a NUL in its tag", &session,
                "{\"class\":\"TPV\",\"tag\":\"a");
}

/* Packets near the size of lexer.outbuffer, up to the longest the
 * packetizer passes: a TPV padded out with its tag, the device last.
 * over is how many bytes the rewritten packet and its NUL would take
 * past the end of outbuffer; unless 0, the packet must pass unchanged.
 */
#define PAD_HEAD        "{\"class\":\"TPV\",\"tag\":\""
#define REWRITE         (sizeof(SOURCE "#") - 1)

static struct {
    const char *legend;
    const char *tail;           // the packet's end, after the tag
    size_t over;
} bound_tests[] = {
    {"path_rewrite fits exactly", "\",\"device\":\"/dev/ttyS0\"}", 0},
    {"path_rewrite one byte over", "\",\"device\":\"/dev/ttyS0\"}", 1},
    // the longest packet, ending in a device name shorter than the path
    {"path_rewrite bound, device last", "\",\"device\":\"x\"}", REWRITE},
};

static int check_path_rewrite_bound(void)
{
    struct gps_context_t context;
    struct gps_device_t session;
    char expect[sizeof(session.lexer.outbuffer) + REWRITE];
    int errors = 0;
    size_t i;

    for (i = 0; i < ROWS(bound_tests); i++) {
        char *outbuf = (char *)session.lexer.outbuffer;
        const char *tail = bound_tests[i].tail;
        size_t len = sizeof(session.lexer.outbuffer) - 1 - REWRITE +
                     bound_tests[i].over;
        size_t pad = len - strlen(PAD_HEAD) - strlen(tail);
        const char *dev = strstr(tail, "\"device\":\"") +
                          strlen("\"device\":\"");

        session_init(&context, &session);
        memcpy(outbuf, PAD_HEAD, strlen(PAD_HEAD));
        memset(outbuf + strlen(PAD_HEAD), 'x', pad);
        (void)strlcpy(outbuf + strlen(PAD_HEAD) + pad, tail,
                      sizeof(session.lexer.outbuffer) -
                      strlen(PAD_HEAD) - pad);
        session.lexer.outbuflen = len;

        if (0 == bound_tests[i].over) {
            // the packet up to the device name, the source, the rest
            size_t head = len - strlen(dev);

            memcpy(expect, outbuf, head);
            (void)strlcpy(expect + head, SOURCE "#", sizeof(expect) - head);
            (void)strlcat(expect, dev, sizeof(expect));
            errors += pass(bound_tests[i].legend, &session, expect);
        } else {
            errors += pass(bound_tests[i].legend, &session, NULL);
        }
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

    errors += check_rewrite();
    errors += check_embedded_nul();
    errors += check_path_rewrite_bound();

    if (0 < errors) {
        (void)printf("test_passthrough: %d errors\n", errors);
        return 1;
    }
    if (!quiet) {
        (void)printf("test_passthrough: %zu cases, all OK\n",
                     ROWS(rewrite_tests) + 1 + ROWS(bound_tests));
    }
    return 0;
}

// vim: set expandtab shiftwidth=4
