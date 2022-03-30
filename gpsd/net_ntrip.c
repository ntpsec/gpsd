/* net_ntrip.c -- gather and dispatch DGNSS data from NTRIP broadcasters
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 * See:
 * https://igs.bkg.bund.de/root_ftp/NTRIP/documentation/NtripDocumentation.pdf
 *
 * NTRIP is not an open protocol.  So this file is based on guesswork.
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/strfuncs.h"

// NTRIP 1.0 caster responses.  Based on Icecast audio servers
#define NTRIP_SOURCETABLE       "SOURCETABLE 200 OK\r\n"
#define NTRIP_ENDSOURCETABLE    "ENDSOURCETABLE"
#define NTRIP_ICY               "ICY 200 OK"

// NTRIP 2.0 caster responses.  Based on HTTP 1.1
#define NTRIP_SOURCETABLE2      "Content-Type: gnss/sourcetable\r\n"
#define NTRIP_BODY              "\r\n\r\n"
#define NTRIP_HTTP              "HTTP/1.1 200 OK"

// sourcetable stuff
#define NTRIP_CAS               "CAS;"
#define NTRIP_NET               "NET;"
#define NTRIP_STR               "STR;"
#define NTRIP_BR                "\r\n"
#define NTRIP_QSC               "\";\""

// HTTP 1.1
#define NTRIP_UNAUTH            "401 Unauthorized"


// table to convert format string to enum ntrip_fmt
static struct ntrip_fmt_s {
    const char *string;
    const enum ntrip_fmt format;
} const ntrip_fmts[] = {
    {"CMR+", FMT_CMRP},
    // RTCM1 required for the SAPOS server in Gemany, confirmed as RTCM2.3
    {"RTCM1_", FMT_RTCM2_3},
    {"RTCM 2.0", FMT_RTCM2_0},
    {"RTCM 2.1", FMT_RTCM2_1},
    {"RTCM 2.2", FMT_RTCM2_2},
    {"RTCM22", FMT_RTCM2_2},
    {"RTCM 2.3", FMT_RTCM2_3},
    {"RTCM2.3", FMT_RTCM2_3},
    {"RTCM 2", FMT_RTCM2},
    {"RTCM2", FMT_RTCM2},
    {"RTCM 3.0", FMT_RTCM3_0},
    {"RTCM3.0", FMT_RTCM3_0},
    {"RTCM 3.1", FMT_RTCM3_1},
    {"RTCM3.1", FMT_RTCM3_1},
    {"RTCM 3.2", FMT_RTCM3_2},
    {"RTCM32", FMT_RTCM3_2},
    {"RTCM 3.3", FMT_RTCM3_3},
    {"RTCM 3", FMT_RTCM3_0},
    {"RTCM3", FMT_RTCM3_0},
    {NULL, FMT_UNKNOWN},
};

/* Return pointer to one NUL terminated source table field
 * Return NULL on error
 * fields are separated by a semicolon (;)
 */
static char *ntrip_field_iterate(char *start,
                                 char *prev,
                                 const char *eol,
                                 const struct gpsd_errout_t *errout)
{
    char *s, *t, *u;

    if (start) {
        s = start;
    } else {
        if (!prev) {
            return NULL;
        }
        s = prev + strlen(prev) + 1;
        if (s >= eol) {
            return NULL;
        }
    }

    // ignore any quoted ; chars as they are part of the field content
    t = s;
    while ((u = strstr(t, NTRIP_QSC))) {
        t = u + sizeof(NTRIP_QSC) - 1;
    }

    if ((t = strstr(t, ";"))) {
        *t = '\0';
    }

    GPSD_LOG(LOG_RAW, errout, "NTRIP: Next source table field %s\n", s);

    return s;
}


/* Decode a stream record from the sourcetable
 * See: http://software.rtcm-ntrip.org/wiki/STR
 */
static void ntrip_str_parse(char *str, size_t len,
                            struct ntrip_stream_t *hold,
                            const struct gpsd_errout_t *errout)
{
    char *s, *eol = str + len;

    memset(hold, 0, sizeof(*hold));

    // <mountpoint>
    if (NULL != (s = ntrip_field_iterate(str, NULL, eol, errout))) {
        (void)strlcpy(hold->mountpoint, s, sizeof(hold->mountpoint));
    }
    // <identifier>
    s = ntrip_field_iterate(NULL, s, eol, errout);
    // <format>
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        struct ntrip_fmt_s const *pfmt;

        hold->format = FMT_UNKNOWN;
        for (pfmt = ntrip_fmts; NULL != pfmt->string; pfmt++) {
            if (0 == strcasecmp(pfmt->string, s)) {
                hold->format = pfmt->format;
                break;
            }
        }
        if (FMT_UNKNOWN == hold->format) {
            GPSD_LOG(LOG_WARN, errout, "NTRIP: Got unknown format '%s'\n", s);
        }
    }
    // <format-details>
    s = ntrip_field_iterate(NULL, s, eol, errout);
    // <carrier>
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        hold->carrier = atoi(s);
    }
    // <nav-system>
    s = ntrip_field_iterate(NULL, s, eol, errout);
    // <network>
    s = ntrip_field_iterate(NULL, s, eol, errout);
    // <country>
    s = ntrip_field_iterate(NULL, s, eol, errout);
    // <latitude>
    hold->latitude = NAN;
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        hold->latitude = safe_atof(s);
    }
    // <longitude>
    hold->longitude = NAN;
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        hold->longitude = safe_atof(s);
    }
    // <nmea>
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        hold->nmea = atoi(s);
    }
    // <solution>
    s = ntrip_field_iterate(NULL, s, eol, errout);
    // <generator>
    s = ntrip_field_iterate(NULL, s, eol, errout);
    // <compr-encryp>
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        if ((0 == strcmp(" ", s)) ||
            (0 == strlen(s)) ||
            (0 == strcasecmp("none", s))) {
            hold->compr_encryp = CMP_ENC_NONE;
        } else {
            hold->compr_encryp = CMP_ENC_UNKNOWN;
            GPSD_LOG(LOG_WARN, errout,
                     "NTRIP: Got unknown {compress,encrypt}ion '%s'\n", s);
        }
    }
    // <authentication>
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        if (0 == strcasecmp("N", s)) {
            hold->authentication = AUTH_NONE;
        } else if (0 == strcasecmp("B", s)) {
            hold->authentication = AUTH_BASIC;
        } else if (0 == strcasecmp("D", s)) {
            hold->authentication = AUTH_DIGEST;
        } else {
            hold->authentication = AUTH_UNKNOWN;
            GPSD_LOG(LOG_WARN, errout,
                     "NTRIP: Got unknown authenticatiion '%s'\n", s);
        }
    }
    // <fee>
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        hold->fee = atoi(s);
    }
    // <bitrate>
    if (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout))) {
        hold->bitrate = atoi(s);
    }
    // ...<misc>
    while (NULL != (s = ntrip_field_iterate(NULL, s, eol, errout)));
}

/* Parse the sourcetable, looking for a match to requested stream.
 *
 * Return 1 -- found match
 *        0 -- no match, maybe more data to parse?
 *        less than zero --  error
 */
static int ntrip_sourcetable_parse(struct gps_device_t *device)
{
    struct ntrip_stream_t hold;
    ssize_t llen, len = 0;
    char *line;
    char buf[BUFSIZ / 2];   // half of BUFSIZE, so we can GPSD_LOG() it
    socket_t fd = device->gpsdata.gps_fd;

    for (;;) {
        ssize_t rlen;

        memset(&buf[len], 0, sizeof(buf) - (size_t)len);
        errno = 0;         // paranoia
        // read, leave room for trailing NUL
        rlen = read(fd, &buf[len], sizeof(buf) - (size_t)(1 + len));
        GPSD_LOG(LOG_RAW, &device->context->errout,
                 "NTRIP: on fd %d len %zd  tried %zd, got %zd\n", fd, len,
                 sizeof(buf) - (size_t)(1 + len), rlen);
        if (-1 == rlen) {
            if (EINTR == errno) {
                continue;
            }
            if (device->ntrip.sourcetable_parse &&
                EAGAIN == errno) {
                // not found a match, but there is no more data
                return 0;
            }
            GPSD_LOG(LOG_ERROR, &device->context->errout,
                     "NTRIP: stream read error %s(%d) on fd %d\n",
                     strerror(errno), errno, fd);
            return -1;
        }
        if (0 == rlen) {     // server closed the connection
            GPSD_LOG(LOG_ERROR, &device->context->errout,
                     "NTRIP: stream unexpected close %s(%d) on fd %d "
                     "during sourcetable read\n",
                     strerror(errno), errno, fd);
            return -2;
        }

        line = buf;
        len += rlen;
        rlen = len;
        // line points to the next char in buf to analyze
        // rlen is length of all data in buf
        // len is length of remaining data in buf, 

        GPSD_LOG(LOG_IO, &device->context->errout,
                 "NTRIP: source table buffer >%.*s<\n", (int)rlen, buf);

        line[rlen] = '\0';      // pacify coverity that this is NUL terminated

        if (!device->ntrip.sourcetable_parse) {
            /* For ntrip v1 the very first line s/b:
             *     "SOURCETABLE 200 OK\r\n"
             * For ntrip v2, the header should contain:
             *     "Content-Type: gnss/sourcetable\r\n"
             */

            if (str_starts_with(line, NTRIP_SOURCETABLE)) {
                // parse SOURCETABLE, NTRIP 1.0
                device->ntrip.sourcetable_parse = true;
            } else if (NULL != strstr(line, NTRIP_SOURCETABLE2)) {
                // parse sourcetable, NTRIP 2.0
                device->ntrip.sourcetable_parse = true;
            } else {
                GPSD_LOG(LOG_WARN, &device->context->errout,
                         "NTRIP: Unexpected reply: %s.\n",
                         buf);
                return -3;
            }
            line = strstr(line, NTRIP_BODY);
            if (NULL == line) {
                return  -4;
            }
            line += 4;        // point to 1st line of body
            len = rlen - (line - buf);
        }

        while (0 < len) {
            char *eol;

            if (str_starts_with(line, NTRIP_ENDSOURCETABLE)) {
                // found ENDSOURCETABLE
                // we got to the end of source table
                return -5;
            }

            eol = strstr(line, NTRIP_BR);
            if (NULL == eol){
                // no full line in the buffer
                break;
            }

            *eol = '\0';
            llen = (ssize_t)(eol - line);

            GPSD_LOG(LOG_IO, &device->context->errout,
                     "NTRIP: checking: >%s<\n", line);

            if (str_starts_with(line, NTRIP_STR)) {
                // parse STR
                ntrip_str_parse(line + strlen(NTRIP_STR),
                                (size_t) (llen - strlen(NTRIP_STR)),
                                &hold, &device->context->errout);

                if (0 == strcmp(device->ntrip.stream.mountpoint,
                                hold.mountpoint)) {
                    // Found a match to requested stream

                    // TODO: support for more formats.  Not that we care
                    // about the format.
                    if (FMT_UNKNOWN == hold.format) {
                        GPSD_LOG(LOG_ERROR, &device->context->errout,
                                 "NTRIP: stream %s format not supported\n",
                                 line);
                        return -6;
                    }
                    // TODO: support encryption and compression algorithms
                    if (CMP_ENC_NONE != hold.compr_encryp) {
                        GPSD_LOG(LOG_ERROR, &device->context->errout,
                                 "NTRIP. stream %s compression/encryption "
                                 "algorithm not supported\n",
                                 line);
                        return -7;
                    }
                    // TODO: support digest authentication
                    if (AUTH_NONE != hold.authentication &&
                        AUTH_BASIC != hold.authentication) {
                        GPSD_LOG(LOG_ERROR, &device->context->errout,
                                 "NTRIP. stream %s authentication method "
                                 "not supported\n",
                                line);
                        return -8;
                    }
                    // no memcpy, so we can keep the other infos
                    device->ntrip.stream.format = hold.format;
                    device->ntrip.stream.carrier = hold.carrier;
                    device->ntrip.stream.latitude = hold.latitude;
                    device->ntrip.stream.longitude = hold.longitude;
                    device->ntrip.stream.nmea = hold.nmea;
                    device->ntrip.stream.compr_encryp = hold.compr_encryp;
                    device->ntrip.stream.authentication = hold.authentication;
                    device->ntrip.stream.fee = hold.fee;
                    device->ntrip.stream.bitrate = hold.bitrate;
                    device->ntrip.stream.set = true;
                    return 1;
                }
                /* TODO: compare stream location to own location to
                 * find nearest stream if user hasn't provided one */
            } else if (str_starts_with(line, NTRIP_CAS)) {
                // TODO: parse CAS, why?
                // See: http://software.rtcm-ntrip.org/wiki/CAS
                GPSD_LOG(LOG_IO, &device->context->errout,
                         "NTRIP: Skipping: '%s'\n", line);
            } else if (str_starts_with(line, NTRIP_NET)) {
                // TODO: parse NET, why?
                // See: http://software.rtcm-ntrip.org/wiki/NET
                GPSD_LOG(LOG_IO, &device->context->errout,
                         "NTRIP: Skipping '%s'\n", line);
            }
            // else ???

            llen += strlen(NTRIP_BR);
            line += llen;        // point to start of next line
            len -= llen;         // calculate remaining data in buf.
            GPSD_LOG(LOG_IO, &device->context->errout,
                     "NTRIP: Remaining source table buffer len %zd\n", len);
        }

        GPSD_LOG(LOG_IO, &device->context->errout,
                 "NTRIP: Remaining source table buffer len %zd\n", len);

        if (0 < len) {
            // shuffle any remaing fragment to front of buf
            // line points to the last fragment in buf
            memmove(buf, line, (size_t)len);
        }
    }

    // fell out of loop, no joy
    return -9;
}

/* Connect to NTRIP caster
 *
 * Warning: Blocking.  if the host is unresponsive, this will hang forever.
 *
 * Return: file descriptor of connection
 *         negative number on failure
 */
static int ntrip_stream_req_probe(const struct ntrip_stream_t *stream,
                                  struct gpsd_errout_t *errout)
{
    int dsock;
    ssize_t r;
    char buf[BUFSIZ];
    char outbuf[BUFSIZ];

    // open blocking
    dsock = netlib_connectsock(AF_UNSPEC, stream->host, stream->port, "tcp");
    if (0 > dsock) {
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: ntrip_stream_req_probe(%s) connect error %s(%d)\n",
                 stream->url, netlib_errstr(dsock), dsock);
        return -1;
    }
    (void)snprintf(buf, sizeof(buf),
            "GET / HTTP/1.1\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: NTRIP gpsd/%s\r\n"
            "Host: %s\r\n"
            "Connection: close\r\n"
            "\r\n", VERSION, stream->host);

    GPSD_LOG(LOG_IO, errout,
             "NTRIP: ntrip_stream_req_probe(%s) fd %d sending >%s<\n",
             stream->url, dsock,
             visibilize(outbuf, sizeof(outbuf), buf,
                        strnlen(buf, sizeof(buf))));

    r = write(dsock, buf, strlen(buf));
    if ((ssize_t)strlen(buf) != r) {
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: stream write error %s on fd %d "
                 "during probe request %zd\n",
                 strerror(errno), dsock, r);
        (void)close(dsock);
        return -1;
    }
    // coverity[leaked_handle] This is an intentional allocation
    return dsock;
}

/* ntrip_auth_encode() - compute the HTTP auth string, if required.
 *
 * Return: 0 == OK
 *         -1 = auth error
 */
static int ntrip_auth_encode(const struct ntrip_stream_t *stream,
                             const char *auth,
                             char buf[],
                             size_t size)
{
    char authenc[64];       // base 64 encoding of auth (username:password)
    int ret = 0;

    memset(buf, 0, size);
    switch (stream->authentication) {
    case AUTH_NONE:
        // nothing to do.
        break;
    case AUTH_BASIC:
        // RFC 7617 Basic Access Authentication.
        // username may not contain a colon (")
        if (NULL == auth) {
            ret = -1;
            break;
        }
        memset(authenc, 0, sizeof(authenc));
        if (0 > b64_ntop((unsigned char *)auth, strlen(auth), authenc,
                         sizeof(authenc) - 1)) {
            ret = -1;
            break;
        }
        (void)snprintf(buf, size - 1, "Authorization: Basic %s\r\n", authenc);
        break;
    case AUTH_DIGEST:
        // TODO: support digest authentication, who needs it?
        // Possibly:  RFC 2617
        //     HTTP Authentication: Basic and Digest Access Authentication)
        /* WWW-Authenticate: Digest realm="testrealm@host.com",
         *              qop="auth,auth-int",
         *              nonce="dcd98b7102dd2f0e8b11d0f600bfb0c093",
         *              opaque="5ccc069c403ebaf9f0171e9517f40e41"
         */
        ret = -1;
        break;
    case AUTH_UNKNOWN:
        // WTF?
        FALLTHROUGH
    default:
        ret = -1;
        break;
    }
    return ret;
}

/* netlib_connectsock() open a blockin socket to host.
 *
 * Return: socket to ntrip server on success
 *         less than zero on error
 */
static socket_t ntrip_stream_get_req(const struct ntrip_stream_t *stream,
                                     const struct gpsd_errout_t *errout)
{
    int dsock;
    char buf[BUFSIZ];
    char outbuf[BUFSIZ];

    // open blocking
    dsock = netlib_connectsock(AF_UNSPEC, stream->host, stream->port, "tcp");
    if (BAD_SOCKET(dsock)) {
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: stream connect error %ss(%d)\n",
                 netlib_errstr(dsock), dsock);
        return -1;
    }

    GPSD_LOG(LOG_SPIN, errout,
             "NTRIP: netlib_connectsock() returns socket on fd %d\n",
             dsock);

    (void)snprintf(buf, sizeof(buf),
            "GET /%s HTTP/1.1\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: NTRIP gpsd/%s\r\n"
            "Host: %s\r\n"
            "Accept: rtk/rtcm, dgps/rtcm\r\n"
            "%s"
            "Connection: close\r\n"
            "\r\n", stream->mountpoint, VERSION, stream->host,
            stream->authStr);

    GPSD_LOG(LOG_IO, errout,
             "NTRIP: netlib_connectsock() sending >%s<\n",
             visibilize(outbuf, sizeof(outbuf), buf,
                        strnlen(buf, sizeof(buf))));

    if ((ssize_t)strlen(buf) != write(dsock, buf, strlen(buf))) {
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: stream write error %s(%d) on fd %d during "
                 "get request\n",
                 strerror(errno), errno, dsock);
        (void)close(dsock);
        return -1;
    }
    return dsock;
}

/* parse the stream header
 * Return: 0 == OK
 *         less than zero == failure
 */
static int ntrip_stream_get_parse(const struct ntrip_stream_t *stream,
                                  const int dsock,
                                  const struct gpsd_errout_t *errout)
{
    char buf[BUFSIZ];
    int opts;

    memset(buf, 0, sizeof(buf));
    while (-1 == read(dsock, buf, sizeof(buf) - 1)) {
        if (EINTR == errno) {
            continue;
        }
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: stream read error %s(%d) on fd %d during get rsp\n",
                 strerror(errno), errno, dsock);
        return -1;
    }
    buf[sizeof(buf) - 1] = '\0';   // pacify coverity about NUL-terminated.

    // parse 401 Unauthorized
    if (NULL != strstr(buf, NTRIP_UNAUTH)) {
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: not authorized for %s\n", stream->url);
        return -1;
    }
    // can't parse SOURCETABLE here
    if (NULL != strstr(buf, NTRIP_SOURCETABLE)) {
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: caster doesn't recognize stream %s:%s/%s\n",
                 stream->host, stream->port, stream->mountpoint);
        return -1;
    }
    // parse "ICY 200 OK" or "HTTP/1.1 200 OK"
    if (NULL == strstr(buf, NTRIP_ICY) &&
        NULL == strstr(buf, NTRIP_HTTP)) {
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: Unknown reply %s from caster: %s:%s/%s\n", buf,
                 stream->host, stream->port, stream->mountpoint);
        return -1;
    }
    opts = fcntl(dsock, F_GETFL);

    if (0 <= opts) {
        (void)fcntl(dsock, F_SETFL, opts | O_NONBLOCK);
    }

    return 0;
}

/*
 * parse an ntrip:// url, "ntrip://" already stripped off
 * see tests/test_timespec.c for possible inputs and results
 * FIXME: merge with test_parse_uri_dest()
 *
 * Return 0 on success
 *        less than zero on failure
 */
int ntrip_parse_url(const struct gpsd_errout_t *errout,
                    struct ntrip_stream_t *stream, const char *fullurl)
{
    char dup[256];                       // working copy of url
    char *at;                            // pointer to at
    char *colon;                         // pointer to colon
    char *slash;                         // pointer to slash
    char *lsb;                           // pointer to left square bracket ([)
    char *rsb;                           // pointer to right square bracket (])
    char *auth = NULL;                   // user:pass
    char *host = NULL;                   // hostname, IPv4 or IPv6
    char *port = NULL;
    char *mountpoint = NULL;             // mount point
    size_t len = 0;

    // save original URL
    strlcpy(stream->url, fullurl, sizeof(stream->url) - 1);

    // make a local copy
    strlcpy(dup, fullurl, sizeof(dup) - 1);

    // find mountpoint, searching from right to left
    if (NULL == (slash = strrchr(dup, '/'))) {
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: can't extract stream from %s\n", dup);
        return -1;
    }
    *slash = '\0';
    mountpoint = slash + 1;
    // dup now ends in host or host:port

    len = strlen(mountpoint);
    if (0 == len) {
        // this also handle the trailing / case
        GPSD_LOG(LOG_ERROR, errout,
                 "NTRIP: ntrip_parse_url(%s) missing mountpoint.\n", fullurl);
        return -1;
    }
    (void)strlcpy(stream->mountpoint,
                  mountpoint, sizeof(stream->mountpoint));

    // dup now contains in order any of  username, password, host and port
    // we know "host" has a dot (hostname or IPv4) or a ] (IPv6)
    at = strrchr(dup, '@');         // user@pass
    colon = strrchr(dup, ':');      // pass:host:port, pass:host, host:port
    rsb = strrchr(dup, ']');        // host is IPv6 literal
    lsb = strrchr(dup, '[');        // host is IPv6 literal

    if (NULL == colon) {
        // no port (:2101), no auth (user:pass@), not IPv6 [fe80::]
        port = NULL;
        auth = NULL;
        host = dup;
    } else {
        /* have a colon, could be:
         *   user@pass:host
         *   user@pass:host:port
         *   [fe80::]
         *   [fe80::]:port
         *   user:pass@[fe80::]:port
         *   user:pass@[fe80::]
         */

        if (NULL == at) {
            // no @, so no auth
            // host:port,  [fe80::], [fe80::]:port
            if (NULL == rsb ||
                NULL == lsb) {
                // allow one of lsb or rsb, could be in the password
                // host:port
                auth = NULL;
                host = dup;
                *colon = '\0';
                port = colon + 1;
            } else {
                // [fe80::], [fe80::]:port
                auth = NULL;
                host = dup + 1;
                *rsb = '\0';
                if (rsb < colon) {
                    // [fe80::]:port
                    *colon = '\0';
                    port = colon + 1;
                } else {
                    // [fe80::]
                    port = NULL;
                }
            }
        } else {
            // NULL != at, have @, so have auth
            auth = dup;
            if (colon < at) {
                // user:pass@host, can't be IPv6, can't have port
                // better not be a colon in the password!
                *at = '\0';
                host = at + 1;
                port = NULL;
            } else {
                // colon > at
                // user:pass@host:port
                // user:pass@[fe80::1]
                // user:pass@[fe80::1]:2101
                *at = '\0';
                if (NULL == rsb ||
                    NULL == lsb) {
                    // user:pass@host:port
                    // allow one of lsb or rsb, could be in the password
                    host = at + 1;
                    *colon = '\0';
                    port = colon + 1;
                } else {
                    // have lsb and rsb
                    // user:pass@[fe80::1]
                    // user:pass@[fe80::1]:2101
                    host = lsb + 1;
                    *rsb = '\0';
                    if (rsb < colon) {
                        // user:pass@[fe80::1]:2101
                        port = rsb + 2;
                    } else {
                        // user:pass@[fe80::1]
                        port = NULL;
                    }
                }
            }
        }
    }
    if (NULL != auth) {
        (void)strlcpy(stream->credentials, auth, sizeof(stream->credentials));
    }

    if (NULL == port ||
        '\0' == port[0]) {
        port = "rtcm-sc104";
        if (NULL == getservbyname(port, "tcp")) {
            // Debian does not have rtcm-sc104 in /etc/services!!
            port = DEFAULT_RTCM_PORT;
        }
    }
    // port ought to be non-NULL by now, but just in case appease Coverity.
    if (NULL != port) {
        (void)strlcpy(stream->port, port, sizeof(stream->port));
    }

    // host ought to be non-NULL by now, but just in case appease Coverity.
    if (NULL != host) {
        (void)strlcpy(stream->host, host, sizeof(stream->host));
    }

    GPSD_LOG(LOG_PROG, errout,
             "NTRIP: ntrip_parse_url(%s) credentials %s host %s port %s "
             "moutpoint %s\n", fullurl,
             stream->credentials,
             stream->host,
             stream->port,
             stream->mountpoint);
    return 0;
}

/* reopen a nonblocking connection to an NTRIP broadcaster
 * Need to already have the sourcetable from a successful ntrip_open()
 *
 * Return: socket on success
 *         -1 on error
 *         PLACEHOLDING_FD (-2) on no connect
 */
static int ntrip_reconnect(struct gps_device_t *device)
{
#if defined(SOCK_NONBLOCK)
    socket_t dsock = -1;
    char addrbuf[50];         // INET6_ADDRSTRLEN

    GPSD_LOG(LOG_PROG, &device->context->errout,
             "NTRIP: ntrip_reconnect() %.60s\n",
             device->gpsdata.dev.path);
    dsock = netlib_connectsock1(AF_UNSPEC, device->ntrip.stream.host,
                                device->ntrip.stream.port,
                                "tcp", SOCK_NONBLOCK, addrbuf, sizeof(addrbuf));
    device->gpsdata.gps_fd = dsock;
    // nonblocking means we have the fd, but the connection is not
    // finished yet.  Connection may fail, later.
    if (0 > dsock) {
        // no way to recover from this, except wait and try again later
        GPSD_LOG(LOG_ERROR, &device->context->errout,
                 "NTRIP: ntrip_reconnect(%s) IP %s, failed: %s(%d)\n",
                 device->gpsdata.dev.path, addrbuf,
                 netlib_errstr(dsock), dsock);
        // set time for retry
        (void)clock_gettime(CLOCK_REALTIME, &device->ntrip.stream.stream_time);
        // leave in connextion closed state for later retry.
        device->ntrip.conn_state = NTRIP_CONN_CLOSED;
        return PLACEHOLDING_FD;
    }
    // will have to wait for select() to confirm conenction, then send
    // the ntrip request again.
    device->ntrip.conn_state = NTRIP_CONN_INPROGRESS;
    GPSD_LOG(LOG_PROG, &device->context->errout,
             "NTRIP: ntrip_reconnect(%s) IP %s, fd %d NTRIP_CONN_INPROGRESS \n",
             device->gpsdata.dev.path, addrbuf, dsock);
#else  // no SOCK_NONBLOCK
    GPSD_LOG(LOG_PROG, &device->context->errout,
             "NTRIP: ntrip_reconnect(%s) no SOCK_NONBLOCK, can't reconnect.\n",
             device->gpsdata.dev.path);
    device->gpsdata.gps_fd = -1;
#endif  // no SOCK_NONBLOCK
    return device->gpsdata.gps_fd;
}

/* open a connection to a NTRIP broadcaster
 * orig contains full url
 *
 * Return: 0 on success, or the new fd
 *         less than zero on failure
 */
int ntrip_open(struct gps_device_t *device, char *orig)
{
    socket_t ret = -1;
    char buf[BUFSIZ];
    char outbuf[BUFSIZ];

    GPSD_LOG(LOG_PROG, &device->context->errout,
             "NTRIP: ntrip_open(%s) fd %d state = %d\n",
             orig, device->gpsdata.gps_fd,
             device->ntrip.conn_state);

    switch (device->ntrip.conn_state) {
    case NTRIP_CONN_INIT:     // state = 0
        /* this has to be done here,
         * because it is needed for multi-stage connection */
        // strlcpy() ensures dup is NUL terminated.
        device->servicetype = SERVICE_NTRIP;
        device->ntrip.works = false;
        device->ntrip.sourcetable_parse = false;
        device->ntrip.stream.set = false;
        device->gpsdata.gps_fd = PLACEHOLDING_FD;

        ret = ntrip_parse_url(&device->context->errout, &device->ntrip.stream,
                              orig);
        if (0 > ret) {
            // failed to parse url
            device->gpsdata.gps_fd = PLACEHOLDING_FD;
            device->ntrip.conn_state = NTRIP_CONN_ERR;
            return -1;
        }

        ret = ntrip_stream_req_probe(&device->ntrip.stream,
                                     &device->context->errout);
        GPSD_LOG(LOG_PROG, &device->context->errout,
                 "NTRIP: ntrip_stream_req_probe(%s) ret %d\n",
                 device->ntrip.stream.url, ret);
        if (-1 == ret) {
            device->gpsdata.gps_fd = PLACEHOLDING_FD;
            device->ntrip.conn_state = NTRIP_CONN_ERR;
            return -1;
        }
        // set timeouts to give time for caster to reply.
        // cant use device->lexer.pkt_time and gpsd_clear() reset it
        (void)clock_gettime(CLOCK_REALTIME, &device->ntrip.stream.stream_time);

        device->gpsdata.gps_fd = ret;
        device->ntrip.conn_state = NTRIP_CONN_SENT_PROBE;
        return ret;
    case NTRIP_CONN_SENT_PROBE:     // state = 1
        ret = ntrip_sourcetable_parse(device);
        GPSD_LOG(LOG_PROG, &device->context->errout,
                 "NTRIP: ntrip_sourcetable_parse(%s) = %d\n",
                 device->ntrip.stream.mountpoint, ret);
        if (0 > ret) {
            device->ntrip.conn_state = NTRIP_CONN_ERR;
            return -1;
        }
        if (0 == ret &&
            false == device->ntrip.stream.set) {
            return ret;
        }
        (void)close(device->gpsdata.gps_fd);
        device->gpsdata.gps_fd = PLACEHOLDING_FD;
        GPSD_LOG(LOG_PROG, &device->context->errout,
                 "NTRIP: found %s: %s: %d,%d,%f,%f,%d,%d,%d,%d,%d\n",
                 device->ntrip.stream.url,
                 device->ntrip.stream.mountpoint,
                 device->ntrip.stream.format,
                 device->ntrip.stream.carrier,
                 device->ntrip.stream.latitude,
                 device->ntrip.stream.longitude,
                 device->ntrip.stream.nmea,
                 device->ntrip.stream.compr_encryp,
                 device->ntrip.stream.authentication,
                 device->ntrip.stream.fee,
                 device->ntrip.stream.bitrate);
        if (0 != ntrip_auth_encode(&device->ntrip.stream,
                                   device->ntrip.stream.credentials,
                                   device->ntrip.stream.authStr,
                                   sizeof(device->ntrip.stream.authStr))) {
            device->ntrip.conn_state = NTRIP_CONN_ERR;
            return -1;
        }
        ret = ntrip_stream_get_req(&device->ntrip.stream,
                                   &device->context->errout);
        if (-1 == ret) {
            device->gpsdata.gps_fd = PLACEHOLDING_FD;
            device->ntrip.conn_state = NTRIP_CONN_ERR;
            return -1;
        }
        device->gpsdata.gps_fd = ret;
        device->ntrip.conn_state = NTRIP_CONN_SENT_GET;
        break;
    case NTRIP_CONN_SENT_GET:          // state = 2
        ret = ntrip_stream_get_parse(&device->ntrip.stream,
                                     device->gpsdata.gps_fd,
                                     &device->context->errout);
        if (-1 == ret) {
            (void)close(device->gpsdata.gps_fd);
            device->gpsdata.gps_fd = PLACEHOLDING_FD;
            device->ntrip.conn_state = NTRIP_CONN_ERR;
            return -1;
        }
        device->ntrip.conn_state = NTRIP_CONN_ESTABLISHED;
        device->ntrip.works = true;   // we know, this worked.
        break;
    case NTRIP_CONN_CLOSED:           // state = 5
        if (6 > llabs((time(NULL) - device->ntrip.stream.stream_time.tv_sec))) {
            // wait a bit longer
            ret = PLACEHOLDING_FD;
            break;
        }
        ret = ntrip_reconnect(device);
        break;
    case NTRIP_CONN_INPROGRESS:      // state = 6
        // Need to send GET within about 40 seconds or caster tiems out.
        // FIXME: partially duplicates  ntrip_stream_get_req()
        // try a write, it will fail if connection still in process, or failed.
        (void)snprintf(buf, sizeof(buf),
                "GET /%s HTTP/1.1\r\n"
                "Ntrip-Version: Ntrip/2.0\r\n"
                "User-Agent: NTRIP gpsd/%s\r\n"
                "Host: %s\r\n"
                "Accept: rtk/rtcm, dgps/rtcm\r\n"
                "%s"
                "Connection: close\r\n"
                "\r\n", device->ntrip.stream.mountpoint, VERSION,
                device->ntrip.stream.host,
                device->ntrip.stream.authStr);

        GPSD_LOG(LOG_IO, &device->context->errout,
                 "NTRIP: ntrip_open() sending >%s<\n",
                  visibilize(outbuf, sizeof(outbuf), buf, strlen(buf)));

        if ((ssize_t)strlen(buf) != write(device->gpsdata.gps_fd, buf,
                                          strlen(buf))) {
            GPSD_LOG(LOG_ERROR, &device->context->errout,
                     "NTRIP: stream write error %s(%d) on fd %d during "
                     "get request\n",
                     strerror(errno), errno, device->gpsdata.gps_fd);
            device->ntrip.conn_state = NTRIP_CONN_ERR;
            // leave FD so deactivate_device() can remove from the
            // select() loop
        } else {
            GPSD_LOG(LOG_ERROR, &device->context->errout,
                     "NTRIP: stream write success get request\n");
            device->ntrip.conn_state = NTRIP_CONN_ESTABLISHED;
        }
        ret = device->gpsdata.gps_fd;
        break;
    case NTRIP_CONN_ESTABLISHED:     // state = 3
        FALLTHROUGH
    case NTRIP_CONN_ERR:             // state = 4
        return -1;
    }
    return ret;
}

// may be time to ship a usage report to the NTRIP caster
void ntrip_report(struct gps_context_t *context,
                  struct gps_device_t *gps,
                  struct gps_device_t *caster)
{
    static int count;

    /*
     * 10 is an arbitrary number, the point is to have gotten several good
     * fixes before reporting usage to our NTRIP caster.
     *
     * count % 5 is as arbitrary a number as the fixcnt. But some delay
     * was needed here
     */
    count ++;
    if (0 != caster->ntrip.stream.nmea &&
        10 < context->fixcnt &&
        0 == (count % 5)) {
        if (-1 < caster->gpsdata.gps_fd) {
            char buf[BUFSIZ];
            ssize_t ret;

            gpsd_position_fix_dump(gps, buf, sizeof(buf));
            ret = write(caster->gpsdata.gps_fd, buf, strlen(buf));
            if ((ssize_t)strlen(buf) == ret) {
                GPSD_LOG(LOG_IO, &context->errout, "NTRIP: => caster %s\n",
                         buf);
            } else if (0 > ret) {
                GPSD_LOG(LOG_ERROR, &context->errout,
                         "NTRIP: ntrip_report() write(%d) error %s(%d)\n",
                         caster->gpsdata.gps_fd, strerror(errno), errno);
            } else {
                GPSD_LOG(LOG_ERROR, &context->errout,
                         "NTRIP: ntrip_report() short write(%d) = %zd\n",
                         caster->gpsdata.gps_fd, ret);
            }
        }
    }
}

// Close an ntrip connection
void ntrip_close(struct gps_device_t *session)
{
    if (0 > session->gpsdata.gps_fd) {
        // UNALLOCATED_FD (-1) or PLACEHOLDING_FD (-2). Nothing to do.
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NTRIP: ntrip_close(%s), close(%d) bad fd\n",
                 session->gpsdata.dev.path, session->gpsdata.gps_fd);
        session->gpsdata.gps_fd = PLACEHOLDING_FD;
        return;
    }

    if (-1 == close(session->gpsdata.gps_fd)) {
        GPSD_LOG(LOG_ERROR, &session->context->errout,
                 "NTRIP: ntrip_close(%s), close(%d), %s(%d)\n",
                 session->gpsdata.dev.path,
                 session->gpsdata.gps_fd, strerror(errno), errno);
    } else {
        GPSD_LOG(LOG_IO, &session->context->errout,
                 "NTRIP: ntrip_close(%s), close(%d)\n",
                 session->gpsdata.dev.path,
                 session->gpsdata.gps_fd);
    }
    // Prepare for a retry, don't use opentime as that gets reset elsewhere
    (void)clock_gettime(CLOCK_REALTIME, &session->ntrip.stream.stream_time);

    session->gpsdata.gps_fd = PLACEHOLDING_FD;
    session->ntrip.conn_state = NTRIP_CONN_CLOSED;
}
// vim: set expandtab shiftwidth=4
