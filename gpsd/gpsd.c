/*
 * This is the main sequence of the gpsd daemon. The IO dispatcher, main
 * select loop, and user command handling lives here.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <arpa/inet.h>               // for htons() and friends
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <float.h>                   // for FLT_EVAL_METHOD
#ifdef HAVE_GETOPT_LONG
   #include <getopt.h>
#endif
#include <grp.h>                     // for setgroups()
#include <math.h>
#include <netdb.h>
#include <pthread.h>
#include <pwd.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>                  // for uint32_t, etc.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>                  // for strlcat(), strcpy(), etc.
#include <syslog.h>
#include <sys/param.h>               // for setgroups()
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>                  // for setgroups()

#ifndef AF_UNSPEC
    #include <sys/socket.h>
#endif  // AF_UNSPEC
#ifndef INADDR_ANY
#include <netinet/in.h>
#endif  // INADDR_ANY

#include "../include/gpsd.h"
#include "../include/gps_json.h"         // needs gpsd.h
#include "../include/sockaddr.h"
#include "../include/strfuncs.h"

#if defined(SYSTEMD_ENABLE)
#include "../include/sd_socket.h"
#endif

/*
 * The name of a tty device from which to pick up whatever the local
 * owning group for tty devices is.  Used when we drop privileges.
 */
#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
#define PROTO_TTY "/dev/tty00"  // correct for *BSD
#else
#define PROTO_TTY "/dev/ttyS0"  // correct for Linux
#endif

#define ACK "{\"class\":\"ACK\"}\r\n"
#define ERROR "{\"class\":\"ERROR\"}\r\n"
#define OK "{\"class\":\"OK\"}\r\n"

/*
 * Timeout policy.  We can't rely on clients closing connections
 * correctly, so we need timeouts to tell us when it's OK to
 * reclaim client fds.  COMMAND_TIMEOUT fends off programs
 * that open connections and just sit there, not issuing a WATCH or
 * doing anything else that triggers a device assignment.  Clients
 * in watcher or raw mode that don't read their data will get dropped
 * when throttled_write() fills up the outbound buffers and the
 * NOREAD_TIMEOUT expires.
 *
 * RELEASE_TIMEOUT sets the amount of time we hold a device
 * open after the last subscriber closes it; this is nonzero so a
 * client that does open/query/close will have time to come back and
 * do another single-shot query, if it wants to, before the device is
 * actually closed.  The reason this matters is because some Bluetooth
 * GPSes not only shut down the GPS receiver on close to save battery
 * power, they actually shut down the Bluetooth RF stage as well and
 * only re-wake it periodically to see if an attempt to raise the
 * device is in progress.  The result is that if you close the device
 * when it's powered up, a re-open can fail with EIO and needs to be
 * tried repeatedly.  Better to avoid this...
 *
 * DEVICE_REAWAKE says how long to wait before repolling after a zero-length
 * read. It's there so we avoid spinning forever on an EOF condition.
 *
 * DEVICE_RECONNECT sets interval on retries when (re)connecting to
 * a device.  In seconds.
 */
#define COMMAND_TIMEOUT         60*15
#define NOREAD_TIMEOUT          60*3
#define RELEASE_TIMEOUT         60
#define DEVICE_REAWAKE          0.01
#define DEVICE_RECONNECT        2

#define QLEN                    5

/*
 * If ntpshm is enabled, we renice the process to this priority level.
 * For precise timekeeping increase priority.
 */
#define NICEVAL -10

#if (defined(TIMESERVICE_ENABLE) || \
     !defined(SOCKET_EXPORT_ENABLE))
    /*
     * Force nowait in two circumstances:
     *
     * (1) Socket export has been disabled.  In this case we have no
     * way to know when client apps are watching the export channels,
     * so we need to be running all the time.
     *
     * (2) timeservice mode where we want the GPS always on for timing.
     */
#define FORCE_NOWAIT
#endif  // TIMESERVICE_ENABLE || !SOCKET_EXPORT_ENABLE

#ifdef SOCKET_EXPORT_ENABLE
/* IP version used by the program */
/* AF_UNSPEC: all
 * AF_INET: IPv4 only
 * AF_INET6: IPv6 only
 */
static const int af_allowed = AF_UNSPEC;
#endif  // SOCKET_EXPORT_ENABLE/

#define AFCOUNT 2

static struct gps_context_t context;
static fd_set all_fds;
static int highwater;
static bool listen_global = false;
static int maxfd;
#ifdef FORCE_NOWAIT
    static bool nowait = true;
#else  // FORCE_NOWAIT
    static bool nowait = false;
#endif  // FORCE_NOWAIT
static jmp_buf restartbuf;
#if defined(SYSTEMD_ENABLE)
    static int sd_socket_count = 0;
#endif

// work around the unfinished ipv6 implementation on hurd and OSX <10.6
#ifndef IPV6_TCLASS
# if defined(__GNU__)
#  define IPV6_TCLASS 61
# elif defined(__APPLE__)
#  define IPV6_TCLASS 36
# endif
#endif

static volatile sig_atomic_t signalled;

// signal handler
static void onsig(int sig)
{
    // just set a variable, and deal with it in the main loop
    signalled = (sig_atomic_t) sig;
}

// list installed drivers and enabled features
static void typelist(void)
{
    const struct gps_type_t **dp;

    for (dp = gpsd_drivers; *dp; dp++) {
        if (COMMENT_PACKET == (*dp)->packet_type) {
            continue;
        }
        if (NULL == (*dp)->mode_switcher) {
            (void)fputc('\t', stdout);
        } else {
            (void)fputs("n\t", stdout);
        }
        if (NULL == (*dp)->speed_switcher) {
            (void)fputc('\t', stdout);
        } else {
            (void)fputs("b\t", stdout);
        }
        if (NULL == (*dp)->rate_switcher) {
            (void)fputc('\t', stdout);
        } else {
            (void)fputs("c\t", stdout);
        }
        if (NMEA_PACKET < (*dp)->packet_type) {
            (void)fputs("*\t", stdout);
        } else {
            (void)fputc('\t', stdout);
        }
        (void)puts((*dp)->type_name);
    }
    (void)printf("# n: mode switch, b: speed switch, "
                 "c: rate switch, *: non-NMEA packet type.\n");
#if defined(CONTROL_SOCKET_ENABLE)
    (void)printf("# Control socket for hotplug notifications enabled.\n");
#endif
#if defined(DBUS_EXPORT_ENABLE)
    (void)printf("# DBUS export enabled.\n");
#endif
#if defined(HAVE_SYS_TIMEPPS_H)
    (void)printf("# KPPS enabled.\n");
#endif
#if defined(MAGIC_HAT_ENABLE)
    (void)printf("# Magic Hat enabled.\n");
#endif
    (void)printf("# Netfeed enabled.\n"
                 "# NTRIP enabled.\n");
#if defined(SHM_EXPORT_ENABLE)
    (void)printf("# Shared memory export enabled.\n");
#endif
#if defined(SOCKET_EXPORT_ENABLE)
    (void)printf("# Socket export enabled.\n");
#endif
#if defined(SYSTEMD_ENABLE)
    (void)printf("# systemd socket activation enabled.\n");
#endif
    exit(EXIT_SUCCESS);
}

static void usage(void)
{
    (void)printf("usage: gpsd [OPTIONS] device...\n\n\
  Options include: \n\
  -?, -h, --help            = help message\n\
  -b, --readonly            = bluetooth-safe: open data sources read-only\n\
  -D, --debug integer       = set debug level, default 0 \n\
  -F, --sockfile sockfile   = specify control socket location, default none\n\
  -f, --framing FRAMING     = fix device framing to FRAMING (8N1, 8O1, etc.)\n\
  -G, --listenany           = make gpsd listen on INADDR_ANY\n\
  -l, --drivers             = list compiled in drivers, and exit.\n\
  -n, --nowait              = don't wait for client connects to poll GPS\n"
#ifdef FORCE_NOWAIT
"                             forced on in this binary\n"
#endif  // FORCE_NOWAIT
"  -N, --foreground          = don't go into background\n\
  -P, --pidfile pidfile     = set file to record process ID\n\
  -p, --passive             = do not reconfigure the receiver automatically\n\
  -r, --badtime             = use GPS time even if no fix\n\
  -S, --port PORT           = set port for daemon, default %s\n\
  -s, --speed SPEED         = fix device speed to SPEED, default none\n\
  -V, --version             = emit version and exit.\n"
"\nA device may be a local serial device for GNSS input, plus an optional\n\
PPS device, or a URL in one of the following forms:\n\
     tcp://host[:port]\n\
     udp://host[:port]\n\
     {dgpsip|ntrip}://[user:passwd@]host[:port][/stream]\n\
     gpsd://host[:port][:/device]\n\
in which case it specifies an input source for device, DGPS or ntrip data.\n"
"\n\
The following driver types are compiled into this gpsd instance:\n",
                 DEFAULT_GPSD_PORT);
    typelist();
    if (8 > sizeof(time_t)) {
        (void)printf("\nWARNING: This system has a 32-bit time_t.\n"
                     "WARNING: This gpsd will fail at 2038-01-19T03:14:07Z.\n");
    }

}

#ifdef CONTROL_SOCKET_ENABLE
static socket_t filesock(char *filename)
{
    struct sockaddr_un addr;
    socket_t sock;

    if (BAD_SOCKET(sock = socket(AF_UNIX, SOCK_STREAM, 0))) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "Can't create device-control socket. %s(%d)\n",
                 strerror(errno), errno);
        return -1;
    }
    (void)strlcpy(addr.sun_path, filename, sizeof(addr.sun_path));
    addr.sun_family = (sa_family_t)AF_UNIX;
    if (0 > bind(sock, (struct sockaddr *)&addr, (socklen_t)sizeof(addr))) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "can't bind to local socket %s. %s(%d)\n",
                 filename, strerror(errno), errno);
        (void)close(sock);
        return -1;
    }
    if (-1 == listen(sock, QLEN)) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "can't listen on local socket %s %s(%d)\n",
                 filename, strerror(errno), errno);
        (void)close(sock);
        return -1;
    }

    // coverity[leaked_handle] This is an intentional allocation
    return sock;
}
#endif  // CONTROL_SOCKET_ENABLE

#define sub_index(s) (int)((s) - subscribers)
#define allocated_device(devp)   ('\0' != (devp)->gpsdata.dev.path[0])
#define free_device(devp)        (devp)->gpsdata.dev.path[0] = '\0'
#define initialized_device(devp) (NULL != (devp)->context)

/*
 * This array fills from the bottom, so as an extreme case you can
 * reduce MAX_DEVICES to 1 in the build recipe.
 * The static" ensure devices is zerod on startup.
 * Some compilers, in 2012, still choke on  = {0}.
 */
static struct gps_device_t devices[MAX_DEVICES];

// track the largest fd currently in use
static void adjust_max_fd(int fd, bool on)
{
    if (on) {
        if (fd > maxfd) {
            maxfd = fd;
        }
    } else if (fd == maxfd) {
        int tfd;

        for (maxfd = tfd = 0; tfd < (int)FD_SETSIZE; tfd++) {
            if (FD_ISSET(tfd, &all_fds)) {
                maxfd = tfd;
            }
        }
    }
}

#ifdef SOCKET_EXPORT_ENABLE
#  ifndef IPTOS_LOWDELAY
#    define IPTOS_LOWDELAY 0x10
#  endif  // !IPTOS_LOWDELAY

// bind a passive command socket for the daemon
static socket_t passivesock_af(int af, char *service, char *tcp_or_udp,
                               int qlen)
{
    volatile socket_t s;
    /*
     * af = address family,
     * service = IANA protocol name or number.
     * tcp_or_udp = TCP or UDP
     * qlen = maximum wait-queue length for connections
     */
    struct servent *pse;
    struct protoent *ppe;
    sockaddr_t sat;
    size_t sin_len = 0;
    int type, proto, one = 1;
    in_port_t port;
    char *af_str = "";
    const int dscp = IPTOS_LOWDELAY; // Prioritize packet

    INVALIDATE_SOCKET(s);
    if (0 != (pse = getservbyname(service, tcp_or_udp))) {
        port = ntohs((in_port_t) pse->s_port);
    } else if (0 == (port = (in_port_t) atoi(service))) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "can't get \"%s\" service entry.\n", service);
        return -1;
    }
    ppe = getprotobyname(tcp_or_udp);
    if (0 == strcmp(tcp_or_udp, "udp")) {
        type = SOCK_DGRAM;
        proto = (ppe) ? ppe->p_proto : IPPROTO_UDP;
    } else {
        type = SOCK_STREAM;
        proto = (ppe) ? ppe->p_proto : IPPROTO_TCP;
    }

    switch (af) {
    case AF_INET:
        sin_len = sizeof(sat.sa_in);

        memset((char *)&sat.sa_in, 0, sin_len);
        sat.sa_in.sin_family = (sa_family_t) AF_INET;
        if (listen_global) {
            sat.sa_in.sin_addr.s_addr = htonl(INADDR_ANY);
        } else {
            sat.sa_in.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        }
        sat.sa_in.sin_port = htons(port);

        af_str = "IPv4";
        // see PF_INET6 case below
        s = socket(PF_INET, type, proto);
        if (-1 < s) {
            // Set packet priority
            if ( -1 == setsockopt(s, IPPROTO_IP, IP_TOS, &dscp,
                                  (socklen_t)sizeof(dscp))) {
                GPSD_LOG(LOG_WARN, &context.errout,
                         "Warning: SETSOCKOPT TOS failed\n");
            }
        }

        break;
    case AF_INET6:
        sin_len = sizeof(sat.sa_in6);

        memset((char *)&sat.sa_in6, 0, sin_len);
        sat.sa_in6.sin6_family = (sa_family_t) AF_INET6;
        if (listen_global) {
            sat.sa_in6.sin6_addr = in6addr_any;
        } else {
            sat.sa_in6.sin6_addr = in6addr_loopback;
        }
        sat.sa_in6.sin6_port = htons(port);

        /*
         * Traditionally BSD uses "communication domains", named by
         * constants starting with PF_ as the first argument for
         * select.  In practice PF_INET has the same value as AF_INET
         * (on BSD and Linux, and probably everywhere else).  POSIX
         * leaves much of this unspecified, but requires that AF_INET
         * be recognized.  We follow tradition here.
         */
        af_str = "IPv6";
        s = socket(PF_INET6, type, proto);

        /*
         * On some network stacks, including Linux's, an IPv6 socket
         * defaults to listening on IPv4 as well. Unless we disable
         * this, trying to listen on in6addr_any will fail with the
         * address-in-use error condition.
         */
        if (-1 < s) {
            int on = 1;

            if (-1 == setsockopt(s, IPPROTO_IPV6, IPV6_V6ONLY, &on,
                                 (socklen_t)sizeof(on))) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "Error: SETSOCKOPT IPV6_V6ONLY, %s(%d)\n",
                         strerror(errno), errno);
                (void)close(s);
                return -1;
            }
#ifdef IPV6_TCLASS
            // Set packet priority
            if (-1 == setsockopt(s, IPPROTO_IPV6, IPV6_TCLASS, &dscp,
                                 (socklen_t)sizeof(dscp))) {
                GPSD_LOG(LOG_WARN, &context.errout,
                         "Warning: SETSOCKOPT TOS failed\n");
            }
#endif  // IPV6_TCLASS
        }
        break;
    default:
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "unhandled address family %d\n", af);
        return -1;
    }
    GPSD_LOG(LOG_IO, &context.errout,
             "opening %s socket\n", af_str);

    if (BAD_SOCKET(s)) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "can't create %s socket\n", af_str);
        return -1;
    }
    if (-1 == setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char *)&one,
                         (socklen_t)sizeof(one))) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "Error: SETSOCKOPT SO_REUSEADDR %s(%d)\n",
                 strerror(errno), errno);
        (void)close(s);
        return -1;
    }
    if (0 > bind(s, &sat.sa, (socklen_t)sin_len)) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "can't bind to %s port %s, %s(%d)\n", af_str,
                 service, strerror(errno), errno);
        if (EADDRINUSE == errno) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "maybe gpsd is already running!  "
                     "Or systemd has the port?\n");
        }
        (void)close(s);
        return -1;
    }
    if (SOCK_STREAM == type &&
        -1 == listen(s, qlen)) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "can't listen on port %s, %s(%d)\n",
                 service, strerror(errno), errno);
        (void)close(s);
        return -1;
    }

    GPSD_LOG(LOG_SPIN, &context.errout, "passivesock_af() -> %d\n", s);
    return s;
}

static int passivesocks(char *service, char *tcp_or_udp,
                        int qlen, socket_t socks[])
{
    int numsocks = AFCOUNT;
    int i;

    for (i = 0; i < AFCOUNT; i++) {
        INVALIDATE_SOCKET(socks[i]);
    }

#if defined(SYSTEMD_ENABLE)
    if (0 < sd_socket_count) {
        for (i = 0; i < AFCOUNT && i < sd_socket_count - 1; i++) {
            socks[i] = SD_SOCKET_FDS_START + i + 1;
        }
        return sd_socket_count - 1;
    }
#endif

    if (AF_UNSPEC == af_allowed ||
        AF_INET == af_allowed) {
        socks[0] = passivesock_af(AF_INET, service, tcp_or_udp, qlen);
    }

    if (AF_UNSPEC == af_allowed ||
        AF_INET6 == af_allowed) {
        socks[1] = passivesock_af(AF_INET6, service, tcp_or_udp, qlen);
    }

    for (i = 0; i < AFCOUNT; i++) {
        if (0 > socks[i]) {
            numsocks--;
        }
    }

    /* Return the number of successfully opened sockets
     * The failed ones are identified by negative values */
    return numsocks;
}

struct subscriber_t
{
    int fd;                       // client file descriptor. -1 if unused
    time_t active;                // when subscriber last polled for data
    struct gps_policy_t policy;   // configurable bits
    pthread_mutex_t mutex;        // serialize access to fd
};

#define subscribed(sub, devp)    (sub->policy.watcher && (sub->policy.devpath[0]=='\0' || strcmp(sub->policy.devpath, devp->gpsdata.dev.path)==0))

// indexed by client file descriptor
static struct subscriber_t subscribers[MAX_CLIENTS];

static void lock_subscriber(struct subscriber_t *sub)
{
    (void)pthread_mutex_lock(&sub->mutex);
}

static void unlock_subscriber(struct subscriber_t *sub)
{
    (void)pthread_mutex_unlock(&sub->mutex);
}

// return the address of a subscriber structure allocated for a new session
static struct subscriber_t *allocate_client(void)
{
    int si;

#if UNALLOCATED_FD == 0
#error client allocation code will fail horribly
#endif
    for (si = 0; si < NITEMS(subscribers); si++) {
        if (UNALLOCATED_FD == subscribers[si].fd) {
            subscribers[si].fd = 0;     // mark subscriber as allocated
            return &subscribers[si];
        }
    }
    return NULL;
}

// detach a client and terminate the session
static void detach_client(struct subscriber_t *sub)
{
    char *c_ip;

    lock_subscriber(sub);
    if (UNALLOCATED_FD == sub->fd) {
        unlock_subscriber(sub);
        return;
    }
    c_ip = netlib_sock2ip(sub->fd);
    (void)shutdown(sub->fd, SHUT_RDWR);
    GPSD_LOG(LOG_SPIN, &context.errout, "close(%d) in detach_client()\n",
             sub->fd);
    (void)close(sub->fd);
    GPSD_LOG(LOG_INF, &context.errout,
             "detaching %s (sub %d, fd %d) in detach_client\n",
             c_ip, sub_index(sub), sub->fd);
    FD_CLR(sub->fd, &all_fds);
    adjust_max_fd(sub->fd, false);
    sub->active = 0;
    sub->policy.watcher = false;
    sub->policy.json = false;
    sub->policy.nmea = false;
    sub->policy.raw = 0;
    sub->policy.scaled = false;
    sub->policy.timing = false;
    sub->policy.split24 = false;
    sub->policy.devpath[0] = '\0';
    sub->fd = UNALLOCATED_FD;
    unlock_subscriber(sub);
}

// write to client -- throttle if it's gone or we're close to buffer overrun
static ssize_t throttled_write(struct subscriber_t *sub, char *buf,
                               const size_t len)
{
    ssize_t status;

    if (LOG_CLIENT <= context.errout.debug) {
        if (isprint((unsigned char) buf[0])) {
            GPSD_LOG(LOG_CLIENT, &context.errout,
                     "=> client(%d) len %zu: %s\n",
                     sub_index(sub), len, buf);
        } else {
            char *cp, buf2[MAX_PACKET_LENGTH * 3];
            buf2[0] = '\0';
            for (cp = buf; cp < buf + len; cp++)
                str_appendf(buf2, sizeof(buf2),
                               "%02x", (unsigned int)(*cp & 0xff));
            GPSD_LOG(LOG_CLIENT, &context.errout,
                     "=> client(%d) len %zu: =%s\n",
                     sub_index(sub), len, buf2);
        }
    }

    gpsd_acquire_reporting_lock();
    status = write(sub->fd, buf, len);
    gpsd_release_reporting_lock();

    if ((ssize_t)len == status) {
        return status;
    }
    if (-1 < status) {
        GPSD_LOG(LOG_INF, &context.errout,
                 "short write disconnecting client(%d), %s(%d)\n",
                 sub_index(sub), strerror(errno), errno);
        detach_client(sub);
        return 0;
    }
    if (EAGAIN == errno ||
        EINTR == errno) {
        // no data written, and errno says to retry
        GPSD_LOG(LOG_INF, &context.errout, "client(%d) write: %s(%d)\n",
                 sub_index(sub), strerror(errno), errno);
        return 0;
    }
    if (EBADF == errno) {
        GPSD_LOG(LOG_WARN, &context.errout, "client(%d) has vanished.\n",
                 sub_index(sub));
    } else if (EWOULDBLOCK == errno &&
               NOREAD_TIMEOUT < (time(NULL) - sub->active)) {
        GPSD_LOG(LOG_INF, &context.errout, "client(%d) timed out.\n",
                 sub_index(sub));
    } else {
        GPSD_LOG(LOG_INF, &context.errout, "client(%d) write: %s(%d)\n",
                 sub_index(sub), strerror(errno), errno);
    }
    detach_client(sub);
    return status;
}

// notify all JSON-watching clients of a given device about an event
static void notify_watchers(struct gps_device_t *device,
                            bool onjson, bool onpps,
                            const char *sentence, ...)
{
    va_list ap;
    char buf[BUFSIZ];
    struct subscriber_t *sub;
    int len;

    va_start(ap, sentence);
    len = vsnprintf(buf, sizeof(buf), sentence, ap);
    va_end(ap);
    if (1 > len) {
        // uh, oh
        return;
    }

    for (sub = subscribers; sub < subscribers + MAX_CLIENTS; sub++) {
        if (0 != sub->active &&
            subscribed(sub, device)) {
            if ((onjson &&
                 sub->policy.json) ||
                (onpps && sub->policy.pps)) {
                // codacy hates strlen()
                (void)throttled_write(sub, buf, len);
            }
        }
    }
}
#endif  // SOCKET_EXPORT_ENABLE

// deactivate device, but leave it in the pool (do not free it)
static void deactivate_device(struct gps_device_t *device)
{
#ifdef SOCKET_EXPORT_ENABLE
    notify_watchers(device, true, false,
                    "{\"class\":\"DEVICE\",\"path\":\"%s\","
                    "\"activated\":0}\r\n",
                    device->gpsdata.dev.path);
#endif  // SOCKET_EXPORT_ENABLE
    if (!BAD_SOCKET(device->gpsdata.gps_fd)) {
        FD_CLR(device->gpsdata.gps_fd, &all_fds);
        adjust_max_fd(device->gpsdata.gps_fd, false);
        ntpshm_link_deactivate(device);
        gpsd_deactivate(device);
    }
}

#if defined(SOCKET_EXPORT_ENABLE) || defined(CONTROL_SOCKET_ENABLE)
// find the device block for an existing device name
static struct gps_device_t *find_device(const char *device_name)
{
    struct gps_device_t *devp;

    if (NULL == device_name) {
        return NULL;
    }
    for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
        if (allocated_device(devp) &&
            0 == strcmp(devp->gpsdata.dev.path, device_name)) {
            return devp;
        }
    }
    return NULL;
}
#endif  // SOCKET_EXPORT_ENABLE) || CONTROL_SOCKET_ENABLE)

/* open the input device
 * return: false on failure
 *         true on success
 */
static bool open_device( struct gps_device_t *device)
{
    int activated = -1;

    if (NULL == device) {
        // can't happen, to shut up compilers
        return false;
    }
    GPSD_LOG(LOG_PROG, &context.errout,
             "CORE: open_device(%s) fd %d\n",
             device->gpsdata.dev.path,
             device->gpsdata.gps_fd);

    activated = gpsd_activate(device, O_OPTIMIZE);
    if (0 > activated &&
        PLACEHOLDING_FD != activated) {
        // failed to open device, and not a /dev/ppsX or ntrip://, etc.
        GPSD_LOG(LOG_PROG, &context.errout,
                 "CORE: from gpsd_activate() fd %d\n", activated);
        return false;
    }

    // do not open ntpshm for NTRIP
    if (SERVICE_NTRIP != device->servicetype) {
        /*
         * Now is the right time to grab the shared memory segment(s)
         * to communicate the navigation message derived and (possibly)
         * 1PPS derived time data to ntpd/chrony.
         */
        ntpshm_link_activate(device);
        if (LOG_INF <= context.errout.debug) {
            char buf1[10], buf2[10];

            if (VALID_UNIT(device->shm_clock_unit)) {
                (void)snprintf(buf1, sizeof(buf1), " NTP%d,",
                               device->shm_clock_unit);
            } else {
                buf1[0] = '\0';
            }
            if (VALID_UNIT(device->shm_pps_unit)) {
                (void)snprintf(buf2, sizeof(buf2), " NTP%d",
                               device->shm_pps_unit);
            } else {
                buf2[0] = '\0';
            }
            GPSD_LOG(LOG_INF, &context.errout,
                     "SHM: ntpshm_link_activate(%s):%.4s%.4s "
                     "activated %d\n",
                     device->gpsdata.dev.path, buf1, buf2, activated);
        }

        if (PLACEHOLDING_FD == activated) {
            // it is a /dev/ppsX, or something, no need to wait on it
            return true;
        }
    }
    FD_SET(device->gpsdata.gps_fd, &all_fds);
    adjust_max_fd(device->gpsdata.gps_fd, true);
    ++highwater;
    return true;
}

/* add a device to the pool; open it right away if in nowait mode
 * return: false on failure
 *         true on success
 */
bool gpsd_add_device(const char *device_name, bool flag_nowait)
{
    struct gps_device_t *devp;
    bool ret = false;
#ifdef SOCKET_EXPORT_ENABLE
    char tbuf[JSON_DATE_MAX+1];
#endif  // SOCKET_EXPORT_ENABLE

    // we can't handle paths longer than GPS_PATH_MAX, so don't try
    // codacy hates strlen()
    if (GPS_PATH_MAX <= strnlen(device_name, GPS_PATH_MAX)) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "ignoring device %s: path length exceeds maximum %d\n",
                 device_name, GPS_PATH_MAX);
        return false;
    }
    // stash devicename away for probing when the first client connects
    for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
        if (!allocated_device(devp)) {
            gpsd_init(devp, &context, device_name);
            ntpshm_session_init(devp);
            GPSD_LOG(LOG_INF, &context.errout,
                     "stashing device %s at slot %d\n",
                     device_name, (int)(devp - devices));
            if (flag_nowait) {
                ret = open_device(devp);
            } else {
                devp->gpsdata.gps_fd = UNALLOCATED_FD;
                ret = true;
            }
#ifdef SOCKET_EXPORT_ENABLE
            notify_watchers(devp, true, false,
                            "{\"class\":\"DEVICE\",\"path\":\"%s\","
                            "\"activated\":\"%s\"}\r\n",
                            devp->gpsdata.dev.path,
                            now_to_iso8601(tbuf, sizeof(tbuf)));
#endif  // SOCKET_EXPORT_ENABLE
            break;
        }
    }
    return ret;
}

#if defined(SOCKET_EXPORT_ENABLE) || defined(CONTROL_SOCKET_ENABLE)
/* convert hex, with length len, to binary, write it, unchanged, to GPS
 * Returns: NULL, or pointer to error string
 */
static const char *write_gps(char *device, char *hex, size_t len) {
    struct gps_device_t *devp;
    int st;

    if (NULL == (devp = find_device(device))) {
        GPSD_LOG(LOG_INF, &context.errout, "GPS <=: %s not active\n", device);
        return "Device not active";
    }
    if (devp->context->readonly ||
        SOURCE_BLOCKDEV >= devp->sourcetype) {
        GPSD_LOG(LOG_WARN, &context.errout,
                 "GPS <=: attempted to write to a read-only device\n");
        return "Attempted to write to a read-only device";
    }

    // NOTE: this destroys the original buffer contents
    st = gpsd_hexpack(hex, hex, len);
    if (0 >= st) {
        GPSD_LOG(LOG_INF, &context.errout,
                 "GPS <=: invalid hex string (error %d).\n", st);
        return  "invalid hex string";
    }
    GPSD_LOG(LOG_INF, &context.errout,
             "GPS <=: writing %d bytes fromhex(%s) to %s\n",
             st, hex, device);
    if (0 >= write(devp->gpsdata.gps_fd, hex, (size_t)st)) {
        GPSD_LOG(LOG_WARN, &context.errout,
                 "GPS <=: write to device failed. %s(%d)\n",
                 strerror(errno), errno);
        return "write to device failed";
    }
    return NULL;
}
#endif  // SOCKET_EXPORT_ENABLE || CONTROL_SOCKET_ENABLE

#ifdef CONTROL_SOCKET_ENABLE
// copy the rest of the command line, before CR-LF
static char *snarfline(char *p, char **out)
{
    char *q;
    static char stash[BUFSIZ];

    for (q = p;
         isprint((unsigned char) *p) &&
         !isspace((unsigned char) *p) &&
         (p - q < (ssize_t)(sizeof(stash) - 1));
         p++) {
        continue;
    }
    (void)memcpy(stash, q, (size_t)(p - q));
    stash[p - q] = '\0';
    *out = stash;
    return p;
}

// handle privileged commands coming through the control socket
// FIXME ignore_return(write()) s/b replaced by throttled_write().
static void handle_control(int sfd, char *buf)
{
    char *stash;
    struct gps_device_t *devp;

     /*
      * The only other place in the code that knows about the format
      * of the + and - commands is the gpsd_control() function in
      * gpsdctl.c.  Be careful about keeping them in sync, or
      * hotplugging will have mysterious failures.
      */
    if ('-' == buf[0]) {
        // remove device named after -
        (void)snarfline(buf + 1, &stash);
        GPSD_LOG(LOG_INF, &context.errout,
                 "<= control(%d): removing %s\n", sfd, stash);
        if ((devp = find_device(stash))) {
            deactivate_device(devp);
            free_device(devp);
            ignore_return(write(sfd, ACK, sizeof(ACK) - 1));
        } else
            ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
    } else if ('+' == buf[0]) {
        // add device named after +
        (void)snarfline(buf + 1, &stash);
        if (find_device(stash)) {
            GPSD_LOG(LOG_INF, &context.errout,
                     "<= control(%d): %s already active \n", sfd,
                     stash);
            ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
        } else {
            GPSD_LOG(LOG_INF, &context.errout,
                     "<= control(%d): adding %s\n", sfd, stash);
            if (gpsd_add_device(stash, nowait))
                ignore_return(write(sfd, ACK, sizeof(ACK) - 1));
            else {
                ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
                GPSD_LOG(LOG_INF, &context.errout,
                         "control(%d): adding %s failed, too many "
                         "devices active\n",
                         sfd, stash);
            }
        }
    } else if ('!' == buf[0]) {
        // split line after ! into device=string, send string to device
        char *eq;
        (void)snarfline(buf + 1, &stash);
        eq = strchr(stash, '=');
        if (NULL == eq) {
            GPSD_LOG(LOG_WARN, &context.errout,
                     "<= control(%d): ill-formed command \n",
                     sfd);
            ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
        } else {
            *eq++ = '\0';
            if ((devp = find_device(stash))) {
                if (devp->context->readonly ||
                    (SOURCE_BLOCKDEV >= devp->sourcetype)) {
                    GPSD_LOG(LOG_WARN, &context.errout,
                             "<= control(%d): attempted to write to a "
                             "read-only device\n",
                             sfd);
                    ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
                } else {
                    GPSD_LOG(LOG_INF, &context.errout,
                             "<= control(%d): writing to %s \n", sfd,
                             stash);
                    if (0 >= write(devp->gpsdata.gps_fd, eq,
                                   eq - (stash + 1))) {
                        GPSD_LOG(LOG_WARN, &context.errout,
                                 "<= control(%d): device write failed %s(%d)\n",
                                 sfd, strerror(errno), errno);
                        ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
                    } else {
                        ignore_return(write(sfd, ACK, sizeof(ACK) - 1));
                    }
                }
            } else {
                GPSD_LOG(LOG_INF, &context.errout,
                         "<= control(%d): %s not active \n", sfd,
                         stash);
                ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
            }
        }
    } else if ('&' == buf[0]) {
        // split line after & into dev=hexdata, send unpacked hexdata to dev
        char *eq;

        (void)snarfline(buf + 1, &stash);
        eq = strchr(stash, '=');
        if (NULL == eq) {
            GPSD_LOG(LOG_WARN, &context.errout,
                     "<= control(%d): ill-formed command\n",
                     sfd);
            ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
        } else {
            const char *rtn;
            *eq++ = '\0';
            rtn = write_gps(stash, eq, eq - (stash + 1));
            if (NULL == rtn) {
                ignore_return(write(sfd, ACK, sizeof(ACK) - 1));
            } else {
                ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
            }
        }
    } else if (strstr(buf, "?devices") == buf) {
        // write back devices list followed by OK
        for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
            ignore_return(write(sfd, devp->gpsdata.dev.path,
                                strnlen(devp->gpsdata.dev.path,
                                        sizeof(devp->gpsdata.dev.path))));
            ignore_return(write(sfd, "\n", 1));
        }
        ignore_return(write(sfd, OK, sizeof(OK) - 1));
    } else {
        // unknown command
        ignore_return(write(sfd, ERROR, sizeof(ERROR) - 1));
    }
}
#endif  // CONTROL_SOCKET_ENABLE

#ifdef SOCKET_EXPORT_ENABLE
/* awaken a device and notify all watchers
 *
 * Return: true if open
 *         false on error
 */
static bool awaken(struct gps_device_t *device)
{
    int ret;

    GPSD_LOG(LOG_PROG, &context.errout,
             "awaken(%d) fd %d, path %s\n",
             (int)(device - devices),
             device->gpsdata.gps_fd, device->gpsdata.dev.path);

    // open that device
    if ((!initialized_device(device) &&
         !open_device(device))) {
        GPSD_LOG(LOG_WARN, &context.errout, "%s: open failed\n",
                 device->gpsdata.dev.path);
        free_device(device);
        return false;
    }

    if (!BAD_SOCKET(device->gpsdata.gps_fd)) {
        GPSD_LOG(LOG_PROG, &context.errout,
                 "device %d (fd=%d, path %s) already active.\n",
                 (int)(device - devices),
                 device->gpsdata.gps_fd, device->gpsdata.dev.path);
        return true;
    }

    ret = gpsd_activate(device, O_OPTIMIZE);
    if (0 > ret) {
        if (PLACEHOLDING_FD == ret) {
            /* wait and try again later.
             * or maybe is is /dev/ppsX */
            GPSD_LOG(LOG_PROG, &context.errout,
                     "awaken(): gpsd_activate() = %d\n", ret);
            return true;
        }

        // failed to open device, and not a /dev/ppsX or ntrip://, etc.
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "%s: device activation failed, freeing device.\n",
                 device->gpsdata.dev.path);
        // FIXME: works around a crash bug, but prevents retries
        free_device(device);
        return false;
    }

    GPSD_LOG(LOG_RAW, &context.errout,
             "flagging descriptor %d in assign_channel()\n",
             device->gpsdata.gps_fd);
    FD_SET(device->gpsdata.gps_fd, &all_fds);
    adjust_max_fd(device->gpsdata.gps_fd, true);
    return true;
}

#ifdef __UNUSED_RECONFIGURE__
// is this channel privileged to change a device's behavior?
static bool privileged_user(struct gps_device_t *device)
{
    // grant user privilege if he's the only one listening to the device
    struct subscriber_t *sub;
    int subcount = 0;

    for (sub = subscribers; sub < subscribers + MAX_CLIENTS; sub++) {
        if (subscribed(sub, device)) {
            subcount++;
        }
    }
    /*
     * Yes, zero subscribers is possible. For example, gpsctl talking
     * to the daemon connects but doesn't necessarily issue a ?WATCH
     * before shipping a request, which means it isn't marked as a
     * subscriber.
     */
    return subcount <= 1;
}
#endif  // __UNUSED_RECONFIGURE__

// set serial parameters for a device from a speed and modestring
static void set_serial(struct gps_device_t *device,
                       speed_t speed, char *modestring)
{
    unsigned int stopbits = device->gpsdata.dev.stopbits;
    char parity = device->gpsdata.dev.parity;
    int wordsize = 8;
    struct timespec delay;

#ifndef __clang_analyzer__
    while (isspace((unsigned char) *modestring)) {
        modestring++;
    }
    if (*modestring &&
        NULL != strchr("78", *modestring)) {
        wordsize = (int)(*modestring++ - '0');
        if (*modestring &&
            NULL != strchr("NOE", *modestring)) {
            parity = *modestring++;
            while (isspace((unsigned char) *modestring)) {
                modestring++;
            }
            if (*modestring &&
                NULL != strchr("12", *modestring)) {
                stopbits = (unsigned int)(*modestring - '0');
            }
        }
    }
#endif  // __clang_analyzer__

    GPSD_LOG(LOG_PROG, &context.errout,
             "SER: set_serial(%s,%u,%s) %c%d\n",
             device->gpsdata.dev.path,
             (unsigned int)speed, modestring, parity, stopbits);
    // no support for other word sizes yet
    if (wordsize == (int)(9 - stopbits) &&
        NULL != device->device_type->speed_switcher) {
        if (device->device_type->speed_switcher(device, speed,
                                                parity, (int)stopbits)) {
            /*
             * Deep black magic is required here. We have to
             * allow the control string time to register at the
             * GPS before we do the baud rate switch, which
             * effectively trashes the UART's buffer.
             *
             * This definitely fails below 40 milliseconds on a
             * BU-303b. 50ms is also verified by Chris Kuethe on
             *  Pharos iGPS360 + GSW 2.3.1ES + prolific
             *  Rayming TN-200 + GSW 2.3.1 + ftdi
             *  Rayming TN-200 + GSW 2.3.2 + ftdi
             * so it looks pretty solid.
             *
             * The minimum delay time is probably constant
             * across any given type of UART.
             */
            if (0 != tcdrain(device->gpsdata.gps_fd)) {
                GPSD_LOG(LOG_ERROR, &device->context->errout,
                         "SER: set_serial(%d) tcdrain() failed: %s(%d)\n",
                         device->gpsdata.gps_fd,
                         strerror(errno), errno);
            }

            // wait 50,000 uSec
            delay.tv_sec = 0;
            delay.tv_nsec = 50000000L;
            nanosleep(&delay, NULL);

            gpsd_set_speed(device, speed, parity, stopbits);
        }
    }
}

#ifdef SOCKET_EXPORT_ENABLE
static void json_devicelist_dump(char *reply, size_t replylen)
{
    struct gps_device_t *devp;
    (void)strlcpy(reply, "{\"class\":\"DEVICES\",\"devices\":[", replylen);

    for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
        size_t reply_len = strnlen(reply, GPS_JSON_RESPONSE_MAX - 3);
        size_t path_len = strnlen(devp->gpsdata.dev.path, GPS_PATH_MAX);
        if (allocated_device(devp) &&
            (reply_len + path_len + 3) < (replylen - 1)) {
            char *cp;
            json_device_dump(devp, reply + reply_len, replylen - reply_len);
            cp = reply + strnlen(reply, GPS_JSON_RESPONSE_MAX - 3);
            *--cp = '\0';
            *--cp = '\0';
            (void)strlcat(reply, ",", replylen);
        }
    }

    str_rstrip_char(reply, ',');
    (void)strlcat(reply, "]}\r\n", replylen);
}
#endif  // SOCKET_EXPORT_ENABLE

// strip trailing \r\n\t\SP from a string
static void rstrip(char *str)
{
    char *strend;

    strend = str + strlen(str) - 1;
    while (isspace((unsigned char)*strend)) {
        *strend = '\0';
        --strend;
    }
}

static void handle_request(struct subscriber_t *sub, const char *buf,
                           size_t bufsize, const char **after,
                           char *reply, size_t replylen)
{
    struct gps_device_t *devp;
    struct gps_policy_t policy_copy;
    const char *end = NULL;
    char watch_buf[GPS_JSON_RESPONSE_MAX];  // buffer for re-written policy
    int len;

    if (str_starts_with(buf, "?DEVICES;")) {
        buf += 9;
        json_devicelist_dump(reply, replylen);
    } else if (str_starts_with(buf, "?WATCH") &&
               (';' == buf[6] ||
                '=' == buf[6])) {
        const char *start = buf;

        buf += 6;
        if (';' == *buf) {
            ++buf;
        } else {
            char *host, *port, *device;  // for parse_uri_dest()
            int status = json_watch_read(buf + 1, &sub->policy, &end);

            if (NULL == end) {
                buf += strnlen(buf, bufsize - 1);
            } else {
                if (';' == *end) {
                    ++end;
                }
                buf = end;
            }
            if (0 != status) {
                // failed to parse ?WATCH.
                (void)snprintf(reply, replylen,
                               "{\"class\":\"ERROR\",\"message\":"
                               "\"Invalid WATCH: %s\"}\r\n",
                               json_error_string(status));
                GPSD_LOG(LOG_ERROR, &context.errout, "response: %s\n", reply);
            } else if (sub->policy.watcher) {
                // enable:true
                if (sub->policy.devpath[0] == '\0') {
                    // awaken all devices
                    for (devp = devices; devp < devices + MAX_DEVICES; devp++)
                        if (allocated_device(devp)) {
                            (void)awaken(devp);
                            if (SOURCE_GPSD == devp->sourcetype) {
                                // wake all, so no devpath/remote issues
                                (void)gpsd_write(devp, start,
                                                 (size_t)(end-start));
                            }
                        }
                } else {
                    // awaken specific device
#ifdef __UNUSED__
                    char outbuf[GPS_JSON_RESPONSE_MAX];
                    json_watch_dump(&sub->policy, outbuf, sizeof(outbuf));
                    GPSD_LOG(0, &context.errout, "policy: %s\n", outbuf);
#endif
                    devp = find_device(sub->policy.devpath);
                    if (NULL == devp) {
                        (void)snprintf(reply, replylen,
                                       "{\"class\":\"ERROR\",\"message\":"
                                       "\"No such device as %s\"}\r\n",
                                       sub->policy.devpath);
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "response: %s\n", reply);
                        goto bailout;
                    } else if (awaken(devp)) {
                        if (SOURCE_GPSD == devp->sourcetype) {
                            /* FIXME: the device into this daemon is
                             * not the device to pass to the remote daemon.
                             * local device = gpsd://host::/device
                             * remote device = /device
                             */
                            policy_copy = sub->policy; // struct copy
                            // parse uri, skip the gpsd://
                            if (0 == parse_uri_dest(policy_copy.devpath + 7,
                                                     &host, &port, &device) &&
                                NULL != device) {
                                // remove gpsd://host:port part
                                (void)strlcpy(policy_copy.devpath,
                                              device,
                                              sizeof(policy_copy.devpath));
                            } else {
                                // no remote device part
                                policy_copy.devpath[0] = '\0';
                            }
                            (void)json_policy_to_watch(&policy_copy,
                                                       watch_buf,
                                                       sizeof(watch_buf));
                            (void)gpsd_write(devp, watch_buf,
                                             strnlen(watch_buf,
                                                     sizeof(watch_buf)));
                        }
                    } else {
                        (void)snprintf(reply, replylen,
                                       "{\"class\":\"ERROR\","
                                       "\"message\":\"Can't assign %s\"}\r\n",
                                       sub->policy.devpath);
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "response: %s\n", reply);
                        goto bailout;
                    }
                }
            }
            // else enable:false ???
        }
        // return a device list and the user's policy
        json_devicelist_dump(reply + strlen(reply), replylen - strlen(reply));
        json_watch_dump(&sub->policy,
                        reply + strlen(reply), replylen - strlen(reply));
    } else if (str_starts_with(buf, "?DEVICE") &&
               (';' == buf[7] ||
                '=' == buf[7])) {
        struct devconfig_t devconf;

        buf += 7;
        devconf.path[0] = '\0';    // initially, no device selection
        if (';' == *buf) {
            ++buf;
        } else {
            struct gps_device_t *device;

            // first, select a device to operate on
            int status = json_device_read(buf + 1, &devconf, &end);
            if (NULL == end) {
                buf += strnlen(buf, bufsize);
            } else {
                if (';' == *end) {
                    ++end;
                }
                buf = end;
            }
            device = NULL;
            if (0 != status) {
                (void)snprintf(reply, replylen,
                               "{\"class\":\"ERROR\","
                               "\"message\":\"Invalid DEVICE: \"%s\"}\r\n",
                               json_error_string(status));
                GPSD_LOG(LOG_ERROR, &context.errout, "response: %s\n", reply);
                goto bailout;
            } else {
                if ('\0' != devconf.path[0]) {
                    // user specified a path, try to assign it
                    device = find_device(devconf.path);
                    // do not optimize away, we need 'device' later on!
                    if (NULL == device ||
                        !awaken(device)) {
                        (void)snprintf(reply, replylen,
                                       "{\"class\":\"ERROR\","
                                       "\"message\":\"Can't open %s.\"}\r\n",
                                       devconf.path);
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "response: %s\n", reply);
                        goto bailout;
                    }
                } else {
                    // no path specified
                    int devcount = 0;

                    for (devp = devices; devp < devices + MAX_DEVICES; devp++)
                        if (allocated_device(devp)) {
                            device = devp;
                            devcount++;
                        }
                    if (0 == devcount) {
                        (void)strlcat(reply,
                                      "{\"class\":\"ERROR\",\"message\":"
                                      "\"Can't perform DEVICE configuration, "
                                      "no devices attached.\"}\r\n",
                                      replylen);
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "response: %s\n", reply);
                        goto bailout;
                    } else if (1 < devcount) {
                        str_appendf(reply, replylen,
                                    "{\"class\":\"ERROR\",\"message\":"
                                    "\"No path specified in DEVICE, but "
                                    "multiple devices are attached.\"}\r\n");
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "response: %s\n", reply);
                        goto bailout;
                    }
                    // we should have exactly one device now
                }
                if (NULL == device->device_type) {
                    str_appendf(reply, replylen,
                                   "{\"class\":\"ERROR\","
                                   "\"message\":\"Type of %s is unknown.\""
                                   "}\r\n",
                                   device->gpsdata.dev.path);
                } else {
                    timespec_t delta1, delta2;
                    const struct gps_type_t *dt = device->device_type;
                    bool no_serial_change =
                        (DEVDEFAULT_BPS == devconf.baudrate &&
                         DEVDEFAULT_PARITY == devconf.parity &&
                         DEVDEFAULT_STOPBITS == devconf.stopbits);

                    // interpret defaults
                    if (DEVDEFAULT_BPS == devconf.baudrate) {
                        devconf.baudrate =
                            (unsigned int)gpsd_get_speed(device);
                    }
                    if (DEVDEFAULT_PARITY == devconf.parity) {
                        devconf.parity = device->gpsdata.dev.parity;
                    }
                    if (DEVDEFAULT_STOPBITS == devconf.stopbits) {
                        devconf.stopbits = device->gpsdata.dev.stopbits;
                    }
                    /* make sure that the cycle is positive, if not, use
                     * current value as to not change cycle later */
                    if (!TS_GZ(&devconf.cycle)) {
                        devconf.cycle = device->gpsdata.dev.cycle;
                    }

                    // now that channel is selected, apply changes
                    if (devconf.driver_mode !=
                            device->gpsdata.dev.driver_mode &&
                        DEVDEFAULT_NATIVE != devconf.driver_mode &&
                        NULL != dt->mode_switcher) {
                        dt->mode_switcher(device, devconf.driver_mode);
                    }
                    if (!no_serial_change) {
                        char serialmode[3];

                        serialmode[0] = devconf.parity;
                        serialmode[1] = '0' + devconf.stopbits;
                        serialmode[2] = '\0';
                        set_serial(device,
                                   (speed_t) devconf.baudrate, serialmode);
                    }
                    TS_SUB(&delta1, &devconf.cycle, &device->gpsdata.dev.cycle);
                    if (TS_NZ(&delta1)) {
                        // different cycle time than before
                        TS_SUB(&delta2, &devconf.cycle, &dt->min_cycle);
                        if (TS_GZ(&delta2) &&
                            NULL != dt->rate_switcher) {
                            // longer than minimum cycle time
                            if (dt->rate_switcher(device,
                                                  TSTONS(&devconf.cycle))) {
                                device->gpsdata.dev.cycle = devconf.cycle;
                            }
                        }
                    }
                    if ('\0' != devconf.hexdata[0]) {
                        const char *rtn = write_gps(device->gpsdata.dev.path,
                            devconf.hexdata,
                            strnlen(devconf.hexdata, sizeof(devconf.hexdata)));
                        if (NULL == rtn) {
                            (void)strlcpy(reply, ACK, replylen);
                        } else {
                            (void)snprintf(reply, replylen,
                                           "{\"class\":\"ERROR\",\"message\":"
                                           "\"%s\"}\r\n", rtn);
                        }
                    }
                }
            }
        }
        // dump a response for each selected channel
        len = strlen(reply);
        for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
            if (!allocated_device(devp)) {
                continue;
            }
            if ('\0' != devconf.path[0] &&
                0 != strcmp(devp->gpsdata.dev.path, devconf.path)) {
                continue;
            }
            json_device_dump(devp, reply + len, replylen - len);
        }
    } else if (str_starts_with(buf, "?POLL;")) {
        char tbuf[JSON_DATE_MAX+1];
        int active = 0;

        buf += 6;
        for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
            if (allocated_device(devp) && subscribed(sub, devp)) {
                if (0 != (devp->observed & GPS_TYPEMASK)) {
                    active++;
                }
            }
        }

        (void)snprintf(reply, replylen,
                       "{\"class\":\"POLL\",\"time\":\"%s\",\"active\":%d,"
                       "\"tpv\":[",
                       now_to_iso8601(tbuf, sizeof(tbuf)), active);
        for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
            if (allocated_device(devp) && subscribed(sub, devp)) {
                if (0 != (devp->observed & GPS_TYPEMASK)) {
                    json_tpv_dump(NAVDATA_SET, devp, &sub->policy,
                                  reply + strlen(reply),
                                  replylen - strlen(reply));
                    rstrip(reply);
                    (void)strlcat(reply, ",", replylen);
                }
            }
        }
        str_rstrip_char(reply, ',');
        (void)strlcat(reply, "],\"gst\":[", replylen);
        for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
            if (allocated_device(devp) && subscribed(sub, devp)) {
                if (0 != (devp->observed & GPS_TYPEMASK)) {
                    json_noise_dump(&devp->gpsdata,
                                    reply + strlen(reply),
                                    replylen - strlen(reply));
                    rstrip(reply);
                    (void)strlcat(reply, ",", replylen);
                }
            }
        }
        str_rstrip_char(reply, ',');
        (void)strlcat(reply, "],\"sky\":[", replylen);
        for (devp = devices; devp < devices + MAX_DEVICES; devp++) {
            if (allocated_device(devp) && subscribed(sub, devp)) {
                if (0 != (devp->observed & GPS_TYPEMASK)) {
                    json_sky_dump(&devp->gpsdata,
                                  reply + strlen(reply),
                                  replylen - strlen(reply));
                    rstrip(reply);
                    (void)strlcat(reply, ",", replylen);
                }
            }
        }
        str_rstrip_char(reply, ',');
        (void)strlcat(reply, "]}\r\n", replylen);
    } else if (str_starts_with(buf, "?VERSION;")) {
        buf += 9;
        json_version_dump(reply, replylen);
    } else {
        const char *errend;
        // a buffer to put the "quoted" bad json into
        char quoted_error[GPS_JSON_RESPONSE_MAX];

        errend = buf + strnlen(buf, bufsize);
        while (isspace((unsigned char) *errend) && errend > buf) {
            --errend;
        }
        len = snprintf(reply, replylen,
                       "{\"class\":\"ERROR\",\"message\":"
                       "\"Unrecognized request '%s'\"}\r\n",
                       json_quote(buf, quoted_error, errend - buf,
                                  sizeof(quoted_error)));
        GPSD_LOG(LOG_ERROR, &context.errout, "ERROR response: %s\n", reply);
        if (0 < len) {
            buf += len;
        }
    }
  bailout:
    *after = buf;
}

// report a raw packet to a subscriber
static void raw_report(struct subscriber_t *sub, struct gps_device_t *device)
{
    /*
     * NMEA and other textual sentences are simply
     * copied to all clients that are in raw or nmea
     * mode.
     */
    if (TEXTUAL_PACKET_TYPE(device->lexer.type) &&
        (0 < sub->policy.raw ||
         sub->policy.nmea)) {
        (void)throttled_write(sub,
                              (char *)device->lexer.outbuffer,
                              device->lexer.outbuflen);
        return;
    }

    /*
     * Also, simply copy if user has specified
     * super-raw mode.
     */
    if (1 < sub->policy.raw) {
        (void)throttled_write(sub,
                              (char *)device->lexer.outbuffer,
                              device->lexer.outbuflen);
        return;
    }
#ifdef BINARY_ENABLE
    /*
     * Maybe the user wants a binary packet hexdumped.
     */
    if (1 == sub->policy.raw) {
        const char *hd =
            gpsd_hexdump(device->msgbuf, sizeof(device->msgbuf),
                         (char *)device->lexer.outbuffer,
                         device->lexer.outbuflen);
        (void)strlcat((char *)hd, "\r\n", sizeof(device->msgbuf));
        (void)throttled_write(sub, (char *)hd,
                              strnlen(hd, sizeof(device->msgbuf)));
    }
#endif  // BINARY_ENABLE
}

// report pseudo-NMEA in appropriate circumstances
// FIXME: duplicated in clients/gpsdecode.c
static void pseudonmea_report(struct subscriber_t *sub,
                              gps_mask_t changed,
                              struct gps_device_t *device)
{
    GPSD_LOG(LOG_DATA, &context.errout,
             "pseudonmea_report() %s mode %d\n",
             gps_maskdump(changed),  device->gpsdata.fix.mode);

    if (GPS_PACKET_TYPE(device->lexer.type) &&
        !TEXTUAL_PACKET_TYPE(device->lexer.type)) {
        char buf[MAX_PACKET_LENGTH * 3 + 2];

        if (0 != (changed & REPORT_IS)) {
            nmea_tpv_dump(device, buf, sizeof(buf));
            GPSD_LOG(LOG_IO, &context.errout,
                     "<= GPS (binary tpv) %s: %s\n",
                     device->gpsdata.dev.path, buf);
            (void)throttled_write(sub, buf, strnlen(buf, sizeof(buf)));
        }

        if (0 != (changed & (DOP_SET | SATELLITE_SET | USED_IS))) {
            nmea_sky_dump(device, buf, sizeof(buf));
            GPSD_LOG(LOG_IO, &context.errout,
                     "<= GPS (binary sky) %s: %s\n",
                     device->gpsdata.dev.path, buf);
            (void)throttled_write(sub, buf, strnlen(buf, sizeof(buf)));
        }

        if (0 != (changed & SUBFRAME_SET)) {
            nmea_subframe_dump(device, buf, sizeof(buf));
            GPSD_LOG(LOG_IO, &context.errout,
                     "<= GPS (binary subframe) %s: %s\n",
                     device->gpsdata.dev.path, buf);
            (void)throttled_write(sub, buf, strnlen(buf, sizeof(buf)));
        }
#ifdef AIVDM_ENABLE
        if (0 != (changed & AIS_SET)) {
            nmea_ais_dump(device, buf, sizeof(buf));
            GPSD_LOG(LOG_IO, &context.errout,
                     "<= AIS (binary ais) %s: %s\n",
                     device->gpsdata.dev.path, buf);
            (void)throttled_write(sub, buf, strnlen(buf, sizeof(buf)));
        }
#endif  // AIVDM_ENABLE
    }
}
#endif  // SOCKET_EXPORT_ENABLE

// report on the current packet from a specified device
static void all_reports(struct gps_device_t *device, gps_mask_t changed)
{
#ifdef SOCKET_EXPORT_ENABLE
    struct subscriber_t *sub;

    GPSD_LOG(LOG_DATA, &context.errout, "all_reports(): changed %s\n",
             gps_maskdump(changed));

    // add any just-identified device to watcher lists
    if (0 != (changed & DRIVER_IS)) {
        bool listeners = false;
        for (sub = subscribers;
             sub < subscribers + MAX_CLIENTS; sub++) {
            if (0 != sub->active &&
                sub->policy.watcher &&
                subscribed(sub, device)) {
                listeners = true;
            }
        }
        if (listeners) {
            (void)awaken(device);
        }
    }

    // handle laggy response to a firmware version query
    if (0 != (changed & (DEVICEID_SET | DRIVER_IS))) {
        if (NULL == device->device_type) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "internal error - device type of %s not set "
                     "when expected\n",
                     device->gpsdata.dev.path);
        } else {
            char id2[GPS_JSON_RESPONSE_MAX];

            json_device_dump(device, id2, sizeof(id2));
            notify_watchers(device, true, false, id2);
        }
    }
#endif  // SOCKET_EXPORT_ENABLE

    /*
     * If the device provided an RTCM packet, repeat it to all devices.
     */
    if (0 != (changed & RTCM2_SET) ||
        0 != (changed & RTCM3_SET)) {
        if (0 != (changed & RTCM2_SET) &&
            RTCM_MAX < device->lexer.outbuflen) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "overlong RTCM packet (%zd bytes)\n",
                     device->lexer.outbuflen);
        } else if (0 != (changed & RTCM3_SET) &&
                   RTCM3_MAX < device->lexer.outbuflen) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "overlong RTCM3 packet (%zd bytes)\n",
                     device->lexer.outbuflen);
        } else {
            struct gps_device_t *dp;
            for (dp = devices; dp < (devices + MAX_DEVICES); dp++) {
                if (!allocated_device(dp) ||
                    0 > device->gpsdata.gps_fd) {
                    continue;
                }
                if (NULL != dp->device_type &&
                    NULL != dp->device_type->rtcm_writer) {
                    // FIXME: don't write back to source
                    ssize_t ret = dp->device_type->rtcm_writer(dp,
                                     (const char *)device->lexer.outbuffer,
                                     device->lexer.outbuflen);
                    if (0 < ret) {
                        GPSD_LOG(LOG_IO, &context.errout,
                                 "<= DGPS/NTRIP: %zd bytes of RTCM relayed.\n",
                                 device->lexer.outbuflen);
                    } else if (0 == ret) {
                        // nothing written, probably read_only
                    } else {
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "<= DGPS/NTRIP: Write to RTCM sink failed, "
                                 " type %s\n",
                                 dp->device_type->type_name);
                    }
                }
            }
        }
    }

    /*
     * Time is eligible for shipping to NTPD if the driver has
     * asserted NTPTIME_IS at any point in the current cycle.
     */
    if (0 != (changed & CLEAR_IS)) {
        device->ship_to_ntpd = false;
    }
    if (0 != (changed & NTPTIME_IS)) {
        device->ship_to_ntpd = true;
    }
    /*
     * Only update the NTP time if we've seen the leap-seconds data.
     * Else we may be providing GPS time.
     */
    if (0 == (changed & TIME_SET)) {
        // GPSD_LOG(LOG_PROG, &context.errout, "NTP: No time this packet\n");
    } else if (NTP_MIN_FIXES >= device->fixcnt &&
               false == context.batteryRTC) {
        /* Many GPS spew random times until after several valid GPS fixes.
         * Garmin says wait at least 3.
         * Allow override with -r option as some GPS say they always output
         * good time from an RTC */
        // GPSD_LOG(LOG_PROG, &context.errout, "NTP: no fix\n");
    } else if (0 == device->newdata.time.tv_sec) {
        // GPSD_LOG(LOG_PROG, &context.errout, "NTP: bad new time\n");
    } else if (device->newdata.time.tv_sec <=
               device->pps_thread.fix_in.real.tv_sec) {
        // GPSD_LOG(LOG_PROG, &context.errout, "NTP: Not a new time\n");
    } else if (!device->ship_to_ntpd) {
        // GPSD_LOG(LOG_PROG, &context.errout,
        //          "NTP: No precision time report\n");
    } else {
        struct timedelta_t td;
        struct gps_device_t *ppsonly;
        // only serial time passes this way, so precision -1
        // maybe should be better for ttyACM and such.
        int precision  = -1;

        ntp_latch(device, &td);

        // propagate this in-band-time to all PPS-only devices
        for (ppsonly = devices; ppsonly < (devices + MAX_DEVICES); ppsonly++)
            if (SOURCE_PPS == ppsonly->sourcetype) {
                pps_thread_fixin(&ppsonly->pps_thread, &td);
            }

        if (VALID_UNIT(device->shm_clock_unit)) {
            ntpshm_put(device, device->shm_clock_unit, precision, &td);
        }
        // why not device->shm_pps_unit here too?

#ifdef SOCKET_EXPORT_ENABLE
        notify_watchers(device, false, true,
                        "{\"class\":\"TOFF\",\"device\":\"%s\",\"real_sec\":"
                        "%lld, \"real_nsec\":%ld,\"clock_sec\":%lld,"
                        "\"clock_nsec\":%ld,\"precision\":%d,"
                        "\"shm\":\"NTP%d\"}\r\n",
                        device->gpsdata.dev.path,
                        (long long)td.real.tv_sec, td.real.tv_nsec,
                        (long long)td.clock.tv_sec, td.clock.tv_nsec,
                        precision, device->shm_clock_unit);
#endif  // SOCKET_EXPORT_ENABLE

    }

    /*
     * If no reliable end of cycle, must report every time
     * a sentence changes position or mode. Likely to
     * cause display jitter.
     */
    if (!device->cycle_end_reliable &&
        0 != (changed & (ATTITUDE_SET | LATLON_SET | MODE_SET))) {
        changed |= REPORT_IS;
    }

    // a few things are not per-subscriber reports
    if (0 != (changed & REPORT_IS)) {
        if (MODE_3D == device->gpsdata.fix.mode) {
            struct gps_device_t *dgnss;

            /*
             * Pass the fix to every potential caster, here.
             * netgnss_report() individual caster types get to
             * make filtering decisiona.
             */
            for (dgnss = devices; dgnss < (devices + MAX_DEVICES); dgnss++) {
                if (dgnss != device) {
                    netgnss_report(&context, device, dgnss);
                }
            }
        }
#if defined(DBUS_EXPORT_ENABLE)
        if (MODE_NO_FIX < device->gpsdata.fix.mode) {
            send_dbus_fix(device);
        }
#endif  // defined(DBUS_EXPORT_ENABLE)
    }

#ifdef SHM_EXPORT_ENABLE
    // should match clients/gpsdecode.c decode()
    if (0 != (changed & (AIS_SET | ATTITUDE_SET | GST_SET | DOP_SET |
                         IMU_SET | REPORT_IS| RTCM2_SET | RTCM3_SET |
                         SATELLITE_SET | SUBFRAME_SET))) {
        // SHM clients updated more often than TCP clients.
        shm_update(&context, &device->gpsdata);
    }
#endif  // SHM_EXPORT_ENABLE

#ifdef SOCKET_EXPORT_ENABLE
    // update all subscribers associated with this device
    for (sub = subscribers; sub < (subscribers + MAX_CLIENTS); sub++) {
        if (0 == sub->active ||
            !subscribed(sub, device)) {
            continue;
        }

        // this is for passing through JSON packets
        if (0 != (changed & PASSTHROUGH_IS)) {
            (void)strlcat((char *)device->lexer.outbuffer, "\r\n",
                          sizeof(device->lexer.outbuffer));
            (void)throttled_write(sub,
                                  (char *)device->lexer.outbuffer,
                                  device->lexer.outbuflen+2);
            continue;
        }

        // report raw packets to users subscribed to those
        raw_report(sub, device);

        // some listeners may be in watcher mode
        if (sub->policy.watcher) {
            if ((changed & DATA_IS) ||
                (changed & REPORT_IS)) {
                GPSD_LOG(LOG_PROG, &context.errout,
                         "Changed mask: %s with %sreliable "
                         "cycle detection\n",
                         gps_maskdump(changed),
                         device->cycle_end_reliable ? "" : "un");
                if (0 != (changed & REPORT_IS)) {
                    GPSD_LOG(LOG_PROG, &context.errout,
                             "time to report a fix\n");
                }

                if (sub->policy.nmea) {
                    pseudonmea_report(sub, changed, device);
                }

                if (sub->policy.json) {
                    char buf[GPS_JSON_RESPONSE_MAX * 4];

                    if (0 != (changed & AIS_SET) &&
                        24 == device->gpsdata.ais.type &&
                        device->gpsdata.ais.type24.part != both &&
                        !sub->policy.split24) {
                        continue;
                    }

                    json_data_report(changed, device, &sub->policy,
                                     buf, sizeof(buf));
                    if ('\0' != buf[0]) {
                        (void)throttled_write(sub, buf,
                                              strnlen(buf, sizeof(buf)));
                    }
                }
            }
        }
    }   // subscribers
#endif  // SOCKET_EXPORT_ENABLE
}

#ifdef SOCKET_EXPORT_ENABLE
/* Execute GPSD requests (?POLL, ?WATCH, etc.) from a buffer.
 * The entire request must be in the buffer.
 */
static int handle_gpsd_request(struct subscriber_t *sub, const char *buf,
                               size_t bufsize)
{
    char reply[GPS_JSON_RESPONSE_MAX + 1];

    reply[0] = '\0';
    if ('?' == buf[0]) {
        const char *end;

        for (end = buf; *buf != '\0'; buf = end) {
            if (isspace((unsigned char)*buf)) {
                end = buf + 1;
            } else {
                size_t len = strnlen(reply, sizeof(reply));
                handle_request(sub, buf, bufsize, &end,
                               reply + len, sizeof(reply) - len);
            }
        }
    }
    return (int)throttled_write(sub, reply, strnlen(reply, sizeof(reply)));
}
#endif  // SOCKET_EXPORT_ENABLE

#if defined(CONTROL_SOCKET_ENABLE) && defined(SOCKET_EXPORT_ENABLE)
/* on PPS interrupt, ship a message to all clients
 * use passed in precision
 *
 * Return: void
 */
static void ship_pps_message(struct gps_device_t *session, int unit,
                             int precision, struct timedelta_t *td)
{
    char buf[GPS_JSON_RESPONSE_MAX];
    char ts_str[TIMESPEC_LEN];

    GPSD_LOG(LOG_DATA, &session->context->errout,
             "ship_pps: qErr_time %s qErr %ld, pps.tv_sec %lld\n",
             timespec_str(&session->gpsdata.qErr_time, ts_str, sizeof(ts_str)),
             session->gpsdata.qErr,
             (long long)td->real.tv_sec);

    // FIXME: reports /dev/ttyAMA0 instead of /dev/pps0 whith MAGIC_HAT

    /* real_XXX - the time the GPS thinks it is at the PPS edge
     * clock_XXX - the time the system clock thinks it is at the PPS edge */
    (void)snprintf(buf, sizeof(buf),
                   "{\"class\":\"PPS\",\"device\":\"%s\",\"real_sec\":%lld,"
                   "\"real_nsec\":%ld,\"clock_sec\":%lld,\"clock_nsec\":%ld,"
                   "\"precision\":%d,\"shm\":\"NTP%d\"",
                   session->gpsdata.dev.path,
                   (long long)td->real.tv_sec, td->real.tv_nsec,
                   (long long)td->clock.tv_sec, td->clock.tv_nsec,
                   precision, unit);

    // output qErr if timestamps line up
    if (td->real.tv_sec == session->gpsdata.qErr_time.tv_sec) {
        str_appendf(buf, sizeof(buf), ",\"qErr\":%ld",
                    session->gpsdata.qErr);
    }
    (void)strlcat(buf, "}\r\n", sizeof(buf));
    notify_watchers(session, true, true, buf);

    /*
     * PPS receipt resets the device's timeout.  This keeps PPS-only
     * devices, which never deliver in-band data, from timing out.
     *
     * FIXME: this only works when there is a JSON client active
     */
    (void)clock_gettime(CLOCK_REALTIME, &session->gpsdata.online);
}
#endif


#ifdef __UNUSED_AUTOCONNECT__
#define DGPS_THRESHOLD  1600000   // max. useful dist. from DGPS server (m)
#define SERVER_SAMPLE   12        // # of servers within threshold to check

struct dgps_server_t
{
    double lat, lon;
    char server[257];
    double dist;
};

static int srvcmp(const void *s, const void *t)
{
    // fixes: warning: cast discards qualifiers from pointer target type
    return (int)(((const struct dgps_server_t *)s)->dist -
                 ((const struct dgps_server_t *)t)->dist);
}

// tell the library to talk to the nearest DGPSIP server
static void netgnss_autoconnect(struct gps_context_t *context,
                                double lat, double lon, const char *serverlist)
{
    struct dgps_server_t keep[SERVER_SAMPLE], hold, *sp, *tp;
    char buf[BUFSIZ];
    FILE *sfp = fopen(serverlist, "r");

    if (NULL == sfp) {
        GPSD_LOG(LOG_ERROR, &context.errout, "no DGPS server list found.\n");
        return;
    }

    for (sp = keep; sp < (keep + SERVER_SAMPLE); sp++) {
        sp->dist = DGPS_THRESHOLD;
        sp->server[0] = '\0';
    }
    hold.lat = hold.lon = 0;
    while (fgets(buf, (int)sizeof(buf), sfp)) {
        char *cp = strchr(buf, '#');
        if (NULL != cp) {
            *cp = '\0';
        }
        if (sscanf(buf, "%32lf %32lf %256s",
                   &hold.lat, &hold.lon, hold.server) == 3) {
            hold.dist = earth_distance(lat, lon, hold.lat, hold.lon);
            tp = NULL;
            /*
             * The idea here is to look for a server in the sample array
             * that is (a) closer than the one we're checking, and (b)
             * furtherest away of all those that are closer.  Replace it.
             * In this way we end up with the closest possible set.
             */
            for (sp = keep; sp < keep + SERVER_SAMPLE; sp++) {
                if (hold.dist < sp->dist &&
                    (NULL == tp ||
                     hold.dist > tp->dist)) {
                    tp = sp;
                }
            }
            if (NULL != tp) {
                *tp = hold;
            }
        }
    }
    (void)fclose(sfp);

    if ('\0' == keep[0].server[0]) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "no DGPS servers within %dm.\n",
                 (int)(DGPS_THRESHOLD / 1000));
        return;
    }

    // sort them and try the closest first
    qsort((void *)keep, SERVER_SAMPLE, sizeof(struct dgps_server_t), srvcmp);
    for (sp = keep; sp < keep + SERVER_SAMPLE; sp++) {
        if ('\0' != sp->server[0]) {
            GPSD_LOG(LOG_INF, &context.errout,
                     "%s is %dkm away.\n", sp->server,
                     (int)(sp->dist / 1000));
            if (0 <= dgpsip_open(context, sp->server)) {
                break;
            }
        }
    }
}
#endif  // __UNUSED_AUTOCONNECT__

// finish cleanly, reverting device configuration
static void gpsd_terminate(struct gps_context_t *context)
{
    int dfd;

    for (dfd = 0; dfd < MAX_DEVICES; dfd++) {
        if (allocated_device(&devices[dfd])) {
            (void)gpsd_wrap(&devices[dfd]);
        }
    }
    context->pps_hook = NULL;   // tell any PPS-watcher thread to die
}

int main(int argc, char *argv[])
{
    // some of these statics suppress -W warnings due to longjmp()
#ifdef SOCKET_EXPORT_ENABLE
    static char *gpsd_service = NULL;
    struct subscriber_t *sub;
#endif  // SOCKET_EXPORT_ENABLE
    fd_set rfds;
#ifdef CONTROL_SOCKET_ENABLE
    fd_set control_fds;
#endif  // CONTROL_SOCKET_ENABLE
#ifdef CONTROL_SOCKET_ENABLE
    static socket_t csock;
    socket_t cfd;
    static char *control_socket = NULL;
#endif  // CONTROL_SOCKET_ENABLE
#if defined(SOCKET_EXPORT_ENABLE) || defined(CONTROL_SOCKET_ENABLE)
    sockaddr_t fsin;
#endif  // SOCKET_EXPORT_ENABLE || CONTROL_SOCKET_ENABLE
    static char *pid_file = NULL;
    struct gps_device_t *device;
    int i;
    int msocks[2] = {-1, -1};
    bool device_opened = false;
    bool go_background = true;
    volatile bool in_restart;
    struct timespec now, delta;
    const char *sudo = getenv("SUDO_COMMAND");
    int uid;

    gps_context_init(&context, "gpsd");

#ifdef CONTROL_SOCKET_ENABLE
    INVALIDATE_SOCKET(csock);
#  if defined(SOCKET_EXPORT_ENABLE)
    context.pps_hook = ship_pps_message;
#  endif  // SOCKET_EXPORT_ENABLE
#endif  // CONTROL_SOCKET_ENABLE

    while (1) {
        const char *optstring = "?bD:F:f:GhlNnpP:rS:s:V";
        int ch;

#ifdef HAVE_GETOPT_LONG
        int option_index = 0;
        static struct option long_options[] = {
            {"badtime", no_argument, NULL, 'r'},
            {"debug", required_argument, NULL, 'D'},
            {"drivers", no_argument, NULL, 'l'},
            {"foreground", no_argument, NULL, 'N'},
            {"framing", required_argument, NULL, 'f'},
            {"help", no_argument, NULL, 'h'},
            {"listenany", no_argument, NULL, 'G' },
            {"nowait", no_argument, NULL, 'n' },
            {"readonly", no_argument, NULL, 'b'},
            {"passive", no_argument, NULL, 'p'},
            {"pidfile", required_argument, NULL, 'P'},
            {"port", required_argument, NULL, 'S'},
            {"sockfile", required_argument, NULL, 'F'},
            {"speed", required_argument, NULL, 's'},
            {"version", no_argument, NULL, 'V' },
            {NULL, 0, NULL, 0},
        };

        ch = getopt_long(argc, argv, optstring, long_options, &option_index);
#else
        ch = getopt(argc, argv, optstring);
#endif
        if (-1 == ch) {
            break;
        }

        switch (ch) {
        case 'b':
            context.readonly = true;
            break;
        case 'D':
            // accept decimal, octal and hex
            context.errout.debug = (int)strtol(optarg, 0, 0);
            gps_enable_debug(context.errout.debug, stderr);
            break;
#ifdef CONTROL_SOCKET_ENABLE
        case 'F':
            control_socket = optarg;
            break;
#endif  // CONTROL_SOCKET_ENABLE
        case 'f':
            // framing
            if (3 == strnlen(optarg, 4) &&
                ('7' == optarg[0] || '8' == optarg[0]) &&
                ('E' == optarg[1] || 'N' == optarg[1] ||
                 'O' == optarg[1]) &&
                ('0' <= optarg[2] && '2' >= optarg[2])) {
                // [78][ENO][012]
                (void)strlcpy(context.fixed_port_framing, optarg,
                              sizeof(context.fixed_port_framing));
            } else {
                // invalid framing
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "-f has invalid framing %s\n", optarg);
                exit(1);
            }
            break;
        case 'G':
            listen_global = true;
            break;
        case 'l':               // list known device types and exit
            typelist();
            break;
        case 'N':
            go_background = false;
            break;
        case 'n':
            nowait = true;
            break;
        case 'p':
            context.passive = true;
            break;
        case 'P':
            pid_file = optarg;
            break;
        case 'r':
            // -r, --badtime, remove fix checks for good time. DANGEROUS
            context.batteryRTC = true;
            break;
        case 'S':
#ifdef SOCKET_EXPORT_ENABLE
            gpsd_service = optarg;
#endif  // SOCKET_EXPORT_ENABLE
            break;
        case 's':
            {
                char *endptr;

                // accept decimal, octal and hex
                long speed = strtol(optarg, &endptr, 0);
                if ('\0' != *endptr) {
                    // check only numeric, some try to suffix with junk (N1).
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "-s has invalid speed %s\n", optarg);
                    exit(1);
                }
                if (0 < speed) {
                    // allow weird speeds
                    context.fixed_port_speed = (speed_t)speed;
                } else {
                    // invalid speed
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "-s has invalid speed %ld\n", speed);
                    exit(1);
                }
            }
            break;
        case 'V':
            (void)printf("%s: %s (revision %s)\n", argv[0], VERSION, REVISION);
            exit(EXIT_SUCCESS);
        case 'h':
            FALLTHROUGH
        case '?':
            FALLTHROUGH
        default:
            usage();
            exit(EXIT_SUCCESS);
        }
    }

    // sanity check
    if (MAX_DEVICES < (argc - optind)) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "Too many devices on command line.\n");
        exit(1);
    }

    if (8 > sizeof(time_t)) {
        GPSD_LOG(LOG_WARN, &context.errout,
                 "This system has a 32-bit time_t.  "
                 "This gpsd will fail at 2038-01-19T03:14:07Z.\n");
    }

#ifdef FLT_EVAL_METHOD
    if (0 != FLT_EVAL_METHOD) {
        GPSD_LOG(LOG_WARN, &context.errout,
                 "FLT_EVAL_METHOD is %d, s/b 0\n", FLT_EVAL_METHOD);
    }
#else
    GPSD_LOG(LOG_WARN, &context.errout,
             "FLT_EVAL_METHOD is missing\n");
#endif

#ifdef __STDC_IEC_599__
    if (1 != __STDC_IEC_599__) {
        GPSD_LOG(LOG_WARN, &context.errout,
                 "__STDC_IEC_599__ is %d, s/b 0\n", __STDC_IEC_599__);
    }
#else
    GPSD_LOG(LOG_WARN, &context.errout,
             "__STDC_IEC_599__ is missing\n");
#endif

    uid = getuid();
    if (0 != uid) {
       GPSD_LOG(LOG_WARN, &context.errout,
                "gpsd not started as root, can not drop privileges.\n");
    } else if (NULL != sudo &&
               0 == strcmp(argv[0], sudo)) {
       GPSD_LOG(LOG_WARN, &context.errout,
                "gpsd running under sudo. Some functions impaired.\n");
    }
#if defined(SYSTEMD_ENABLE) && defined(CONTROL_SOCKET_ENABLE)
    sd_socket_count = sd_get_socket_count();
    if (0 < sd_socket_count &&
        NULL == control_socket) {
        GPSD_LOG(LOG_WARN, &context.errout,
                 "control socket passed on command line ignored\n");
        control_socket = NULL;
    }
#endif

#if defined(CONTROL_SOCKET_ENABLE) || defined(SYSTEMD_ENABLE)
    if (
#ifdef CONTROL_SOCKET_ENABLE
        NULL == control_socket &&
#endif
#ifdef SYSTEMD_ENABLE
        0 >= sd_socket_count &&
#endif
        optind >= argc) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "can't run with neither control socket nor devices\n");
        exit(EXIT_FAILURE);
    }

    /*
     * Control socket has to be created before we go background in order to
     * avoid a race condition in which hotplug scripts can try opening
     * the socket before it's created.
     */
#if defined(SYSTEMD_ENABLE) && defined(CONTROL_SOCKET_ENABLE)
    if (0 < sd_socket_count) {
        csock = SD_SOCKET_FDS_START;
        FD_SET(csock, &all_fds);
        adjust_max_fd(csock, true);
    }
#endif
#ifdef CONTROL_SOCKET_ENABLE
    if (control_socket &&
        '\0' != control_socket[0]) {
        if(0 == unlink(control_socket)) {
            GPSD_LOG(LOG_PROG, &context.errout,
                     "stale control socket %s removed\n", control_socket);
        } else {
            GPSD_LOG(LOG_WARN, &context.errout,
                     "removing stale control socket %s failed: %s(%d)\n",
                     control_socket, strerror(errno), errno);
        }
        if (BAD_SOCKET(csock = filesock(control_socket))) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "control socket %s create failed, netlib error %d\n",
                     control_socket, csock);
            exit(EXIT_FAILURE);
        } else {
            GPSD_LOG(LOG_PROG, &context.errout,
                     "control socket %s is fd %d\n",
                     control_socket, csock);
        }
        FD_SET(csock, &all_fds);
        adjust_max_fd(csock, true);
    }
#endif  // CONTROL_SOCKET_ENABLE
#else
    if (optind >= argc) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "can't run with no devices specified\n");
        exit(EXIT_FAILURE);
    }
#endif  // CONTROL_SOCKET_ENABLE || SYSTEMD_ENABLE

    // might be time to daemonize
    if (go_background) {
        // not SuS/POSIX portable, but we have our own fallback version
        if (0 != os_daemon(0, 0)) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "daemonization failed: %s(%d)\n", strerror(errno), errno);
        }
    }

    if (NULL != pid_file) {
        FILE *fp;

        if (NULL == (fp = fopen(pid_file, "w"))) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "Cannot create PID file: %s. %s(%d)\n",
                     pid_file, strerror(errno), errno);
        } else {
            (void)fprintf(fp, "%u\n", (unsigned int)getpid());
            (void)fclose(fp);
        }
    }

    /* LOG_PID: log our PID
     * LOG_CONS: log to console if syslog down
     * LOG_NDELAY: open now before dropping root */
    openlog("gpsd", LOG_PID | LOG_CONS | LOG_NDELAY, LOG_USER);

    // Do this after openlog(), so this goes in syslog()
    if (LOG_INF <= context.errout.debug) {
        char buf[2048];
        int cnt;

        GPSD_LOG(LOG_INF, &context.errout,
                 "launching (Version " VERSION ", revision " REVISION ")\n");
        GPSD_LOG(LOG_INF, &context.errout, "starting uid %d, gid %d\n",
                 uid, getgid());

        // log command line, maybe log all parsed options too?
        buf[0] = '\0';
        for (cnt = 0; cnt < argc; cnt++) {
            (void)strlcat(buf, argv[cnt], sizeof(buf));
            (void)strlcat(buf, " ", sizeof(buf));
        }

        GPSD_LOG(LOG_INF, &context.errout, "Command line: %s\n", buf);
    }


#ifdef SOCKET_EXPORT_ENABLE
    if (!gpsd_service) {
        gpsd_service =
            getservbyname("gpsd", "tcp") ? "gpsd" : DEFAULT_GPSD_PORT;
    }
    if (1 > passivesocks(gpsd_service, "tcp", QLEN, msocks)) {
        GPSD_LOG(LOG_ERROR, &context.errout,
                 "command sockets creation failed, netlib errors %d, %d\n",
                 msocks[0], msocks[1]);
        if (NULL != pid_file) {
            (void)unlink(pid_file);
        }
        exit(EXIT_FAILURE);
    }
    GPSD_LOG(LOG_INF, &context.errout, "listening on port %s\n",
                       gpsd_service);
#endif  // SOCKET_EXPORT_ENABLE

    if (0 == getuid()) {
        errno = 0;
        // nice() can ONLY succeed when run as root!
        // do not even bother as non-root
        if (-1 == nice(NICEVAL) &&
            0 !=  errno) {
            GPSD_LOG(LOG_WARN, &context.errout,
                     "PPS: o=priority setting failed. Time accuracy "
                     "will be degraded, %s(%d)\n", strerror(errno), errno);
        }
    }
    /*
     * By initializing before we drop privileges, we guarantee that even
     * hotplugged devices added *after* we drop privileges will be able
     * to use segments 0 and 1.
     */
    (void)ntpshm_context_init(&context);

#if defined(DBUS_EXPORT_ENABLE)
    // we need to connect to dbus as root
    if (initialize_dbus_connection()) {
        // the connection could not be started, maybe user does not want it
        GPSD_LOG(LOG_WARNING, &context.errout,
                 "unable to connect to the DBUS system bus\n");
    } else {
        GPSD_LOG(LOG_PROG, &context.errout,
                 "successfully connected to the DBUS system bus\n");
    }
#endif  // defined(DBUS_EXPORT_ENABLE)

#ifdef SHM_EXPORT_ENABLE
    // create the shared segment as root so readers can't mess with it
    (void)shm_acquire(&context);
#endif  // SHM_EXPORT_ENABLE

    /*
     * We open devices specified on the command line *before* dropping
     * privileges in case one of them is a serial device with PPS support
     * and we need to set the line discipline, which requires root.
     */
    in_restart = false;
    for (i = optind; i < argc; i++) {
      if (gpsd_add_device(argv[i], nowait)) {
            device_opened = true;
        } else {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "initial GPS device %s open failed\n", argv[i]);
        }
    }

    if (
#ifdef CONTROL_SOCKET_ENABLE
       NULL == control_socket &&
#endif
#ifdef SYSTEMD_ENABLE
       0 >= sd_socket_count &&
#endif
       !device_opened) {
       GPSD_LOG(LOG_ERROR, &context.errout,
                "can't run with neither control socket nor devices open\n");
       exit(EXIT_FAILURE);
    }


    // drop privileges
    if (0 == getuid()) {
        struct passwd *pw;
        struct stat stb;

        /* Make default devices accessible even after we drop privileges.
         * Modifying file system permissions! */
        for (i = optind; i < argc; i++) {
            if (GPS_PATH_MAX <= strnlen(argv[i], GPS_PATH_MAX)) {
               // pacify coverity
               GPSD_LOG(LOG_ERROR, &context.errout,
                        "Over long device path %s\n", argv[i]);
            }
            if (0 == stat(argv[i], &stb)) {
                /* This fails if not running as root, or have group
                 * access to the file. */
                (void)chmod(argv[i], stb.st_mode | S_IRGRP | S_IWGRP);
            }
        }
        /*
         * Drop privileges.  Up to now we've been running as root.
         * Instead, set the user ID to 'nobody' (or whatever the gpsd
         * user set by the build is) and the group ID to the owning
         * group of a prototypical TTY device. This limits the scope
         * of any compromises in the code.  It requires that all GPS
         * devices have their group read/write permissions set.
         */
        if (0 != setgroups(0, NULL)) {
            GPSD_LOG(LOG_ERROR, &context.errout,
                     "setgroups() failed, errno %s(%d)\n",
                     strerror(errno), errno);
        }
#ifdef GPSD_GROUP
        {
            struct group *grp = getgrnam(GPSD_GROUP);
            if (grp) {
                if (0 != setgid(grp->gr_gid)) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "setgid() failed, %s(%d)\n",
                             strerror(errno), errno);
                }
            }
        }
#else
        if ((optind < argc &&
             0 == stat(argv[optind], &stb)) ||
            0 == stat(PROTO_TTY, &stb)) {
            GPSD_LOG(LOG_PROG, &context.errout,
                     "changing to group %d\n", stb.st_gid);
            if (0 != setgid(stb.st_gid)) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "setgid() failed, %s(%d)\n", strerror(errno), errno);
            }
        }
#endif
        pw = getpwnam(GPSD_USER);
        if (pw) {
            if (0 != setuid(pw->pw_uid)) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                            "setuid() failed, %s(%d)\n",
                            strerror(errno), errno);
            }
        }
    }
    // sometimes getegid() and geteuid() are longs
    GPSD_LOG(LOG_INF, &context.errout,
             "running with effective group ID %ld\n", (long)getegid());
    GPSD_LOG(LOG_INF, &context.errout,
             "running with effective user ID %ld\n", (long)geteuid());

#ifdef SOCKET_EXPORT_ENABLE
    for (i = 0; i < NITEMS(subscribers); i++) {
        subscribers[i].fd = UNALLOCATED_FD;
        (void)pthread_mutex_init(&subscribers[i].mutex, NULL);
    }
#endif  // SOCKET_EXPORT_ENABLE

    {
        struct sigaction sa;

        sa.sa_flags = 0;
#ifdef __COVERITY__
        /*
         * Obsolete and unused.  We're only doing this to pacify Coverity
         * which otherwise throws an UNINIT event here. Don't swap with the
         * handler initialization, they're unioned on some architectures.
         */
        sa.sa_restorer = NULL;
#endif  // __COVERITY__
        sa.sa_handler = onsig;
        (void)sigfillset(&sa.sa_mask);
        (void)sigaction(SIGHUP, &sa, NULL);
        (void)sigaction(SIGINT, &sa, NULL);
        (void)sigaction(SIGTERM, &sa, NULL);
        (void)sigaction(SIGQUIT, &sa, NULL);
        (void)signal(SIGPIPE, SIG_IGN);
    }

    // daemon got termination or interrupt signal
    if (0 < setjmp(restartbuf)) {
        gpsd_terminate(&context);
        in_restart = true;
        GPSD_LOG(LOG_WARN, &context.errout, "gpsd restarted by SIGHUP\n");
    }

    signalled = 0;

    for (i = 0; i < AFCOUNT; i++) {
        if (0 <= msocks[i]) {
            FD_SET(msocks[i], &all_fds);
            adjust_max_fd(msocks[i], true);
        }
    }
#ifdef CONTROL_SOCKET_ENABLE
    FD_ZERO(&control_fds);
#endif  // CONTROL_SOCKET_ENABLE

    // initialize the GPS context's time fields
    gpsd_time_init(&context, time(NULL));

    /*
     * If we got here via SIGINT, reopen any command-line devices. PPS
     * through these won't work, as we've dropped privileges and can
     * no longer change line disciplines.
     */
    if (in_restart) {
        for (i = optind; i < argc; i++) {
          if (!gpsd_add_device(argv[i], nowait)) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "GPS device %s open failed\n",
                         argv[i]);
            }
        }
    }

    while (0 == signalled) {
        fd_set efds;
        // static here suppresses longjmp warning
        static const timespec_t ts_timeout = {2, 0};   // timeout for pselect()
        timespec_t before, after;        // time before/after gpsd_await_data()
        int await;
        bool time_warp;

        time_warp = false;
        GPSD_LOG(LOG_RAW1, &context.errout, "await data\n");
        (void)clock_gettime(CLOCK_REALTIME, &before);
        await = gpsd_await_data(&rfds, &efds, maxfd, &all_fds, &context.errout,
                                ts_timeout);
        (void)clock_gettime(CLOCK_REALTIME, &after);
        TS_SUB(&delta, &after, &before);
        if ((1 + ts_timeout.tv_sec) <= llabs(delta.tv_sec)) {
            GPSD_LOG(LOG_WARN, &context.errout,
                     "Let's do the time warp again %lld.  "
                     "It's just a jump to the left\n",
                     (long long)delta.tv_sec);
            time_warp = true;
        }
        switch(await) {
        case AWAIT_GOT_INPUT:
            FALLTHROUGH
        case AWAIT_TIMEOUT:
            break;
        case AWAIT_NOT_READY:
            for (device = devices; device < devices + MAX_DEVICES; device++) {
                /*
                 * The file descriptor validity check is required on some ARM
                 * platforms to prevent a core dump.  This may be due to an
                 * implementation error in FD_ISSET().
                 */
                if (allocated_device(device) &&
                    0 <= device->gpsdata.gps_fd &&
                    (socket_t)FD_SETSIZE > device->gpsdata.gps_fd &&
                    FD_ISSET(device->gpsdata.gps_fd, &efds)) {
                    deactivate_device(device);
                    free_device(device);
                }
            }
            continue;
        case AWAIT_FAILED:
            exit(EXIT_FAILURE);
        }

#ifdef SOCKET_EXPORT_ENABLE
        // always be open to new client connections
        for (i = 0; i < AFCOUNT; i++) {
            if (0 <= msocks[i] &&
                FD_ISSET(msocks[i], &rfds)) {
                socklen_t alen = (socklen_t) sizeof(fsin);
                socket_t ssock =
                    accept(msocks[i], (struct sockaddr *)&fsin, &alen);

                if (BAD_SOCKET(ssock)) {
                    GPSD_LOG(LOG_ERROR, &context.errout,
                             "accept: fail: %s(%d)\n", strerror(errno), errno);
                } else {
                    struct subscriber_t *client = NULL;
                    int opts = fcntl(ssock, F_GETFL);
                    static struct linger linger = { 1, RELEASE_TIMEOUT };
                    char *c_ip;

                    if (0 <= opts) {
                        (void)fcntl(ssock, F_SETFL, opts | O_NONBLOCK);
                    }

                    c_ip = netlib_sock2ip(ssock);
                    client = allocate_client();
                    if (NULL == client) {
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "Client %s connect on fd %d -"
                                 "no subscriber slots available\n", c_ip,
                                    ssock);
                        (void)close(ssock);
                    } else if (-1 == setsockopt(ssock,
                                                SOL_SOCKET, SO_LINGER,
                                                (char *)&linger,
                                                (int)sizeof(struct linger))) {
                        GPSD_LOG(LOG_ERROR, &context.errout,
                                 "Error: SETSOCKOPT SO_LINGER. %s(%d)\n",
                                 strerror(errno), errno);
                        (void)close(ssock);
                    } else {
                        char announce[GPS_JSON_RESPONSE_MAX];
                        FD_SET(ssock, &all_fds);
                        adjust_max_fd(ssock, true);
                        client->fd = ssock;
                        client->active = time(NULL);
                        GPSD_LOG(LOG_SPIN, &context.errout,
                                 "client %s (%d) connect on fd %d\n", c_ip,
                                 sub_index(client), ssock);
                        json_version_dump(announce, sizeof(announce));
                        (void)throttled_write(client, announce,
                                              strnlen(announce,
                                                      sizeof(announce)));
                    }
                }
                FD_CLR(msocks[i], &rfds);
            }
        }
#endif  // SOCKET_EXPORT_ENABLE

#ifdef CONTROL_SOCKET_ENABLE
        // also be open to new control-socket connections
        if (-1 < csock &&
            FD_ISSET(csock, &rfds)) {
            socklen_t alen = (socklen_t) sizeof(fsin);
            socket_t ssock = accept(csock, (struct sockaddr *)&fsin, &alen);

            if (BAD_SOCKET(ssock)) {
                GPSD_LOG(LOG_ERROR, &context.errout,
                         "accept: %s(%d)\n", strerror(errno), errno);
            } else {
                GPSD_LOG(LOG_INF, &context.errout,
                         "control socket connect on fd %d\n",
                         ssock);
                FD_SET(ssock, &all_fds);
                FD_SET(ssock, &control_fds);
                adjust_max_fd(ssock, true);
            }
            FD_CLR(csock, &rfds);
        }

        // read any commands that came in over the control socket
        // Linux man page says FD_* and select() are obsolete...
        GPSD_LOG(LOG_RAW1, &context.errout, "read control commands");
        for (cfd = 0; cfd < (int)FD_SETSIZE; cfd++) {
            // Do we really need to check all 1024 possible file descriptors?
            if (FD_ISSET(cfd, &control_fds)) {
                char buf[BUFSIZ];
                ssize_t rd;

                while (0 < (rd = read(cfd, buf, sizeof(buf) - 1))) {
                    buf[rd] = '\0';
                    GPSD_LOG(LOG_CLIENT, &context.errout,
                             "<= control(%d): %s\n", cfd, buf);
                    // coverity[tainted_data] Safe, never handed to exec
                    handle_control(cfd, buf);
                }
                GPSD_LOG(LOG_SPIN, &context.errout,
                         "close(%d) of control socket\n", cfd);
                (void)close(cfd);
                FD_CLR(cfd, &all_fds);
                FD_CLR(cfd, &control_fds);
                adjust_max_fd(cfd, false);
            }
        }
#endif  // CONTROL_SOCKET_ENABLE

        // poll all active devices
        GPSD_LOG(LOG_RAW1, &context.errout, "poll active devices\n");
        for (device = devices; device < devices + MAX_DEVICES; device++) {
            int multipoll_ret;

            if (!allocated_device(device) ||
                0 >= device->gpsdata.gps_fd) {
                continue;
            }

            multipoll_ret = gpsd_multipoll(FD_ISSET(device->gpsdata.gps_fd,
                                           &rfds), device, all_reports,
                                           DEVICE_REAWAKE);
            GPSD_LOG(LOG_DATA, &context.errout,
                     "gpsd_multipoll(%d) = %d\n",
                     device->gpsdata.gps_fd, multipoll_ret);
            switch (multipoll_ret) {
            case DEVICE_READY:
                FD_SET(device->gpsdata.gps_fd, &all_fds);
                adjust_max_fd(device->gpsdata.gps_fd, true);
                break;
            case DEVICE_UNREADY:
                FD_CLR(device->gpsdata.gps_fd, &all_fds);
                adjust_max_fd(device->gpsdata.gps_fd, false);
                break;
            case DEVICE_ERROR:
                FALLTHROUGH
            case DEVICE_EOF:
                deactivate_device(device);
                break;
            case DEVICE_UNCHANGED:
                /* pselect() returned.  Most likely data on one
                 * of the connections.  Maybe this one, maybe another
                 * one.  Maybe a timeout.
                 *
                 * Roes not mean no data this cycle on this device.
                 *
                 * So no data on this device, if it is a tty, tells us
                 * nothing about if data not coming in on this device
                 * due to wrong speed.
                 *
                 * gpsd_next_hunt_setting() will try next hunt speed
                 * if device is a tty. */

                // This device has either never received a message.
                (void)clock_gettime(CLOCK_REALTIME, &now);
                if (0 == device->lexer.pkt_time.tv_sec) {
                    // just activated
                    device->lexer.pkt_time = now;
                }
                // or hasn't received a message for the last 5 seconds,
                TS_SUB(&delta, &now, &device->lexer.pkt_time);
                // llabs() in case the system time jumped
                if (5 <= llabs(delta.tv_sec)) {
                    GPSD_LOG(LOG_PROG, &context.errout,
                        "gpsd_multipoll(%d) DEVICE_UNCHANGED for %lld\n",
                        device->gpsdata.gps_fd, (long long)delta.tv_sec);
                    if (time_warp) {
                        // ugh, start over...
                        device->lexer.pkt_time = now;
                    } else if (0 < gpsd_serial_isatty(device)) {
                        // then try the next hunt speed.
                        gpsd_next_hunt_setting(device);
                    } else {
                        // gpsd://, tcp:// etc.  Just reset timer for now.
                        device->lexer.pkt_time = now;
                        if (SERVICE_NTRIP == device->servicetype) {
                            // ntrip://
                            // likely NTRIP_CONN_INPROGRESS, move it along
                            ntrip_open(device, "");
                        }
                    }
                }
                break;
            default:
                // huh?
                GPSD_LOG(LOG_WARN, &context.errout,
                         "gpsd_multipoll(%d) = unknown return value %d\n",
                         device->gpsdata.gps_fd, multipoll_ret);
                break;
            }
        }

#ifdef __UNUSED_AUTOCONNECT__
        if (0 < context.fixcnt &&
            !context.autconnect) {
            for (device = devices; device < devices + MAX_DEVICES; device++) {
                if (MODE_NO_FIX < device->gpsdata.fix.mode) {
                    netgnss_autoconnect(&context,
                                        device->gpsdata.fix.latitude,
                                        device->gpsdata.fix.longitude);
                    context.autconnect = True;
                    break;
                }
            }
        }
#endif  // __UNUSED_AUTOCONNECT_

#ifdef SOCKET_EXPORT_ENABLE
        // accept and execute commands for all clients
        for (sub = subscribers; sub < subscribers + MAX_CLIENTS; sub++) {
            if (0 == sub->active) {
                continue;
            }

            lock_subscriber(sub);
            if (FD_ISSET(sub->fd, &rfds)) {
                char buf[BUFSIZ];
                ssize_t buflen;

                unlock_subscriber(sub);

                GPSD_LOG(LOG_PROG, &context.errout,
                         "checking client(%d)\n",
                         sub_index(sub));
                buflen = recv(sub->fd, buf, sizeof(buf) - 1, 0);
                if (0 > buflen) {
                    // recv() error, give up.
                    detach_client(sub);
                    GPSD_LOG(LOG_CLIENT, &context.errout,
                             "<= client(%d): error read\n", sub_index(sub));
                } else if (0 == buflen) {
                    /* Ugh, "man recv" says recv() returns 0 on disconnect!
                     * So we have to disconnect client.
                     * But somehow, dormant serial connections also
                     * return 0.  Should FD_ISSET() have prevented getting
                     * here in that case?
                     */
                    detach_client(sub);
                    GPSD_LOG(LOG_CLIENT, &context.errout,
                             "<= client(%d): eof read\n", sub_index(sub));
                } else {
                    if ('\n' != buf[buflen - 1]) {
                        buf[buflen++] = '\n';
                    }
                    buf[buflen] = '\0';
                    GPSD_LOG(LOG_CLIENT, &context.errout,
                             "<= client(%d): %s\n", sub_index(sub), buf);

                    /*
                     * When a command comes in, update subscriber.active to
                     * timestamp() so we don't close the connection
                     * after COMMAND_TIMEOUT seconds. This makes
                     * COMMAND_TIMEOUT useful.
                     */
                    sub->active = time(NULL);
                    if (0 > handle_gpsd_request(sub, buf, sizeof(buf))) {
                        detach_client(sub);
                    }
                }
            } else {
                unlock_subscriber(sub);

                if (!sub->policy.watcher &&
                    COMMAND_TIMEOUT < (time(NULL) - sub->active)) {
                    GPSD_LOG(LOG_WARN, &context.errout,
                             "client(%d) timed out on command wait.\n",
                             sub_index(sub));
                    detach_client(sub);
                }
            }
        }

        /*
         * Mark devices with an identified packet type but no
         * remaining subscribers to be closed in RELEASE_TIME seconds.
         * See the explanation of RELEASE_TIME for the reasoning.
         *
         * Re-poll devices that are disconnected, but have potential
         * subscribers in the same cycle.
         */
        for (device = devices; device < devices + MAX_DEVICES; device++) {
            bool device_needed = nowait;

            if (!allocated_device(device)) {
                continue;
            }
            if (!device_needed) {
                for (sub = subscribers; sub < subscribers + MAX_CLIENTS;
                     sub++) {
                    if (0 == sub->active) {
                        continue;
                    }
                    device_needed = subscribed(sub, device);
                    if (device_needed) {
                        break;
                    }
                }
            }

            if (device_needed) {
                // device needed
                if (BAD_SOCKET(device->gpsdata.gps_fd) &&
                    SOURCE_PPS != device->sourcetype &&
                    (0 == device->opentime  ||
                     DEVICE_RECONNECT < (time(NULL) - device->opentime))) {
                    device->opentime = time(NULL);
                    GPSD_LOG(LOG_INF, &context.errout,
                             "reconnection attempt on device %d, %s\n",
                             (int)(device - devices),
                             device->gpsdata.dev.path);
                    (void)awaken(device);
                }
            } else {
                // not device needed
                if (-1 < device->gpsdata.gps_fd &&
                    BAD_PACKET != device->lexer.type) {
                    if (0 == device->releasetime) {
                        device->releasetime = time(NULL);
                        GPSD_LOG(LOG_PROG, &context.errout,
                                 "device %d (fd %d) released\n",
                                 (int)(device - devices),
                                 device->gpsdata.gps_fd);
                    } else if (RELEASE_TIMEOUT <
                               (time(NULL) - device->releasetime)) {
                        GPSD_LOG(LOG_PROG, &context.errout,
                                 "device %d closed\n",
                                 (int)(device - devices));
                        GPSD_LOG(LOG_RAW, &context.errout,
                                 "unflagging descriptor %d\n",
                                 device->gpsdata.gps_fd);
                        deactivate_device(device);
                    }
                }
            }
        }
#endif  // SOCKET_EXPORT_ENABLE

        /*
         * Might be time for graceful shutdown if no command-line
         * devices were specified, there are no subscribers, there are
         * no active devices, and there *have been* active
         * devices. The goal is to go away and free up text space when
         * the daemon was hotplug-activated but there are no
         * subscribers and the last GPS has unplugged, and the point
         * of the last check is to prevent shutdown when the daemon
         * has been launched but not yet received its first device
         * over the socket.
         */
        if (argc == optind &&
            0 < highwater) {
            int subcount = 0, devcount = 0;
#ifdef SOCKET_EXPORT_ENABLE
            for (sub = subscribers; sub < (subscribers + MAX_CLIENTS); sub++) {
                if (0 != sub->active) {
                    ++subcount;
                }
            }
#endif  // SOCKET_EXPORT_ENABLE
            for (device = devices; device < devices + MAX_DEVICES; device++) {
                if (allocated_device(device)) {
                    ++devcount;
                }
            }
            if (0 == subcount &&
                0 == devcount) {
                GPSD_LOG(LOG_SHOUT, &context.errout,
                         "no subscribers or devices, shutting down.\n");
                goto shutdown;
            }
        }
    }

    // if we make it here, we got a signal... deal with it
    // restart on SIGHUP, clean up and exit otherwise
    if (SIGHUP == (int)signalled) {
        longjmp(restartbuf, 1);
    }

    GPSD_LOG(LOG_WARN, &context.errout,
             "received terminating signal %d.\n", (int)signalled);
shutdown:
    gpsd_terminate(&context);

    GPSD_LOG(LOG_WARN, &context.errout, "exiting.\n");

#ifdef SOCKET_EXPORT_ENABLE
    /*
     * A linger option was set on each client socket when it was
     * created.  Now, shut them down gracefully, letting I/O drain.
     * This is an attempt to avoid the sporadic race errors at the ends
     * of our regression tests.
     */
    for (sub = subscribers; sub < (subscribers + MAX_CLIENTS); sub++) {
        if (0 != sub->active) {
            detach_client(sub);
        }
    }
#endif  // SOCKET_EXPORT_ENABLE

#ifdef SHM_EXPORT_ENABLE
    shm_release(&context);
#endif  // SHM_EXPORT_ENABLE

#ifdef CONTROL_SOCKET_ENABLE
    if (control_socket) {
        (void)unlink(control_socket);
    }
#endif  // CONTROL_SOCKET_ENABLE
    if (pid_file) {
        (void)unlink(pid_file);
    }
    return 0;
}

// vim: set expandtab shiftwidth=4
