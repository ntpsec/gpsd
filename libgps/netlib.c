/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#ifdef HAVE_ARPA_INET_H
#  include <arpa/inet.h>              // for htons() and friends
#endif  // HAVE_ARPA_INET_H
#include <errno.h>                    // for errno
#include <fcntl.h>
#include <sys/types.h>   // FreeBSD needs it for netinet/ip.h
#ifdef HAVE_NETDB_H
#  include <netdb.h>
#endif  // HAVE_NETDB_H
#ifndef INADDR_ANY
#  ifdef HAVE_NETINET_IN_H
#    include <netinet/in.h>
#  endif  // HAVE_NETINET_IN_H
#endif  // INADDR_ANY
#ifdef HAVE_NETINET_IN_H
#  include <netinet/ip.h>
#endif  // HAVE_NETINET_IN_H
#include <string.h>
#ifndef AF_UNSPEC
#  include <sys/stat.h>
#  ifdef HAVE_SYS_SOCKET_H
#    include <sys/socket.h>
#  endif  // HAVE_SYS_SOCKET_H
#endif  // AF_UNSPEC
#ifdef HAVE_SYS_UN_H
#  include <sys/un.h>
#endif  // HAVE_SYS_UN_H
#include <unistd.h>
#ifdef HAVE_WINSOCK2_H
#  include <winsock2.h>
#  include <ws2tcpip.h>
#endif // HAVE_WINSOCK2_H

#include "../include/gpsd.h"
#include "../include/sockaddr.h"

/* work around the unfinished ipv6 implementation on hurd and OSX <10.6 */
#ifndef IPV6_TCLASS
#  if defined(__GNU__)
#    define IPV6_TCLASS 61
#  elif defined(__APPLE__)
#    define IPV6_TCLASS 36
# endif
#endif

/* connect to host, using service (port) on protocol (TCP/UDP)
 * af - Adress Family
 * host - host to connect to
 * service -- aka port
 * protocol
 * flags -- can be SOCK_NONBLOCK for non-blocking connect
 *          Note: macOS does not have SOCK_NONBLOCK
 * addrbuf -- 50 char buf to put string of IP address conencting
 *            INET6_ADDRSTRLEN
 * addrbuf_sz -- sizeof(adddrbuf)
 *
 * Notes on nonblocking:
 * The connect may be non-blocking, but the DNS lookup is blocking.
 * On non-blocking connect only the first DNS entry is ever used.
 * FIXME: cache DNS to avoid DNS lookup on re-connect
 *
 * return socket on success
 *        less than zero on error (NL_*)
 */
socket_t netlib_connectsock1(int af, const char *host, const char *service,
                             const char *protocol, int flags,
                             char *addrbuf, size_t addrbuf_sz)
{
    struct protoent *ppe;
    struct addrinfo hints;
    struct addrinfo *result = NULL;
    struct addrinfo *rp;
    int ret, type, proto, one = 1;
    socket_t s;
    bool bind_me;

    if (NULL != addrbuf) {
        addrbuf[0] = '\0';
    }
    INVALIDATE_SOCKET(s);
    ppe = getprotobyname(protocol);
    if (0 == strcmp(protocol, "udp")) {
        type = SOCK_DGRAM;
        proto = (ppe) ? ppe->p_proto : IPPROTO_UDP;
    } else if (0 == strcmp(protocol, "tcp")) {
        type = SOCK_STREAM;
        proto = (ppe) ? ppe->p_proto : IPPROTO_TCP;
    } else {
        // Unknown protocol (sctp, etc.)
        return NL_NOPROTO;
    }

    /* we probably ought to pass this in as an explicit flag argument */
    bind_me = (SOCK_DGRAM == type);

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = af;
    hints.ai_socktype = type;
    hints.ai_protocol = proto;
    if (bind_me) {
        hints.ai_flags = AI_PASSIVE;
    }
    // FIXME: need a way to bypass these DNS calls if host is an IP.
    if ((ret = getaddrinfo(host, service, &hints, &result))) {
        // result is unchanged on error, so we need to have set it to NULL
        // freeaddrinfo() checks for NULL, the NULL we provided.
        freeaddrinfo(result);
        result = NULL;
        // quick check to see if the problem was host or service
        ret = getaddrinfo(NULL, service, &hints, &result);
        freeaddrinfo(result);
        if (ret) {
            return NL_NOSERVICE;
        }
        return NL_NOHOST;
    }

    /*
     * Try to connect to each of the DNS returned addresses, one at a time.
     * Until success, or no more addresses.
     *
     * From getaddrinfo(3):
     *     Normally, the application should try using the addresses in the
     *     order in which they are returned.  The sorting function used within
     *     getaddrinfo() is defined in RFC 3484).
     * From RFC 3484 (Section 10.3):
     *     The default policy table gives IPv6 addresses higher precedence than
     *     IPv4 addresses.
     * Thus, with the default parameters, we get IPv6 addresses first.
     */
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        ret = NL_NOCONNECT;
        // flags might be zero or SOCK_NONBLOCK
        if (0 > (s = socket(rp->ai_family, rp->ai_socktype | flags,
                            rp->ai_protocol))) {
            // can't get a socket.  Maybe should give up right away?
            ret = NL_NOSOCK;
            continue;
        }
        // allow reuse of local address is in TIMEWAIT state
        // useful on a quick gpsd restart to reuse the address.
        if (-1 == setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char *)&one,
                                    sizeof(one))) {
            ret = NL_NOSOCKOPT;
        } else if (bind_me) {
            // want a passive socket (SOCK_DGRAM), UDP.
            if (0 == bind(s, rp->ai_addr, rp->ai_addrlen)) {
                // got a good one
                ret = 0;
                break;
            }
        } else if (0 == connect(s, rp->ai_addr, rp->ai_addrlen)) {
            // got a good connection
            ret = 0;
            break;
        } else if (EINPROGRESS == errno) {
            // EINPROGRESS means non-blocking connect() in progress
            // we will not try next address...
            // separate case from 0 == connect() to ease debug.
            ret = 0;
            break;
        }
        if (NULL != addrbuf) {
            // save the IP used, as a string. into addrbuf
            if (NULL == inet_ntop(af, rp->ai_addr, addrbuf, addrbuf_sz)) {
                addrbuf[0] = '\0';
            }
        }

        if (!BAD_SOCKET(s)) {
#ifdef HAVE_WINSOCK2_H
          (void)closesocket(s);
#else
          (void)close(s);
#endif
        }
    }
    freeaddrinfo(result);
    if (0 != ret ||
        BAD_SOCKET(s)) {
        return ret;
    }

#ifdef IPTOS_LOWDELAY
    {
        int opt = IPTOS_LOWDELAY;
        (void)setsockopt(s, IPPROTO_IP, IP_TOS, &opt, sizeof(opt));
#ifdef IPV6_TCLASS
        (void)setsockopt(s, IPPROTO_IPV6, IPV6_TCLASS, &opt, sizeof(opt));
#endif
    }
#endif
#ifdef TCP_NODELAY
    /*
     * This is a good performance enhancement when the socket is going to
     * be used to pass a lot of short commands.  It prevents them from being
     * delayed by the Nagle algorithm until they can be aggreagated into
     * a large packet.  See https://en.wikipedia.org/wiki/Nagle%27s_algorithm
     * for discussion.
     */
    if (SOCK_STREAM == type) {
        setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char *)&one, sizeof(one));
    }
#endif

    // set socket to noblocking
#ifdef HAVE_FCNTL
    (void)fcntl(s, F_SETFL, fcntl(s, F_GETFL) | O_NONBLOCK);
#elif defined(HAVE_WINSOCK2_H)
    u_long one1 = 1;
    (void)ioctlsocket(s, FIONBIO, &one1);
#endif
    return s;
}

// legacy entry point
// just call netlib_connectsock() with options = 0;
// return the result
socket_t netlib_connectsock(int af, const char *host, const char *service,
                            const char *protocol)
{
    return netlib_connectsock1(af, host, service, protocol, 0, NULL, 0);
}

//  Convert NL_* error code to a string
const char *netlib_errstr(const int err)
{
    switch (err) {
    case NL_NOSERVICE:
        return "can't get service entry";
    case NL_NOHOST:
        return "can't get host entry";
    case NL_NOPROTO:
        return "can't get protocol entry";
    case NL_NOSOCK:
        return "can't create socket";
    case NL_NOSOCKOPT:
        return "error SETSOCKOPT SO_REUSEADDR";
    case NL_NOCONNECT:
        return "can't connect to host/port pair";
    default:
        return "unknown error";
    }
}

// acquire a connection to an existing Unix-domain socket
socket_t netlib_localsocket(const char *sockfile, int socktype)
{
#ifdef HAVE_SYS_UN_H
    int sock;

    if (0 > (sock = socket(AF_UNIX, socktype, 0))) {
        return -1;
    } else {
        struct sockaddr_un saddr;

        memset(&saddr, 0, sizeof(struct sockaddr_un));
        saddr.sun_family = AF_UNIX;
        (void)strlcpy(saddr.sun_path, sockfile, sizeof(saddr.sun_path));

        if (0 < connect(sock, (struct sockaddr *)&saddr, SUN_LEN(&saddr))) {
            (void)close(sock);
            return -2;
        }

        return sock;
    }
#else
    return -1;
#endif  // HAVE_SYS_UN_H
}

// retrieve the IP address corresponding to a socket
char *netlib_sock2ip(socket_t fd)
{
    static char ip[INET6_ADDRSTRLEN];
#ifdef HAVE_INET_NTOP
    int r;
    sockaddr_t fsin;
    socklen_t alen = (socklen_t) sizeof(fsin);
    r = getpeername(fd, &(fsin.sa), &alen);
    if (0 == r) {
        switch (fsin.sa.sa_family) {
        case AF_INET:
            r = !inet_ntop(fsin.sa_in.sin_family, &(fsin.sa_in.sin_addr),
                           ip, sizeof(ip));
            break;
        case AF_INET6:
            r = !inet_ntop((int)fsin.sa_in6.sin6_family,
                           &(fsin.sa_in6.sin6_addr), ip, sizeof(ip));
            break;
        default:
            (void)strlcpy(ip, "<unknown AF>", sizeof(ip));
            return ip;
        }
    }
    // Ugh...
    if (0 != r)
#endif  // HAVE_INET_NTOP
        (void)strlcpy(ip, "<unknown>", sizeof(ip));
    return ip;
}
// vim: set expandtab shiftwidth=4
