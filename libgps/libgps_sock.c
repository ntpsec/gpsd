/* libgps_sock.c -- client interface library for the gpsd daemon
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <locale.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef USE_QT
    #ifdef HAVE_SYS_SOCKET_H
        #include <sys/socket.h>
    #endif  // HAVE_SYS_SOCKET_H
    #ifdef HAVE_WINSOCK2_H
    #include <winsock2.h>
    #endif  // HAVE_WINSOCK2_H
#else
    #include <QTcpSocket>
#endif  // USE_QT

#include "../include/gps.h"
#include "../include/gpsd.h"          // FIXME: clients chould not use gpsd.h!
#include "../include/libgps.h"
#include "../include/strfuncs.h"
#include "../include/timespec.h"      // for NS_IN_SEC
#ifdef SOCKET_EXPORT_ENABLE
#include "../include/gps_json.h"

struct privdata_t
{
    bool newstyle;
    // data buffered from the last read
    ssize_t waiting;
    char buffer[GPS_JSON_RESPONSE_MAX * 2];
    int waitcount;
};

#ifdef HAVE_WINSOCK2_H
static bool need_init = TRUE;
static bool need_finish = TRUE;

// Ensure socket networking is initialized for Windows.
static bool windows_init(void)
{
    WSADATA wsadata;
    // request access to Windows Sockets API version 2.2
    int res = WSAStartup(MAKEWORD(2, 2), &wsadata);
    if (0 != res) {
        libgps_debug_trace((DEBUG_CALLS, "WSAStartup returns error %d\n", res));
    }
    return (0 == res);
}

// Shutdown Windows Sockets.
static bool windows_finish(void)
{
    int res = WSACleanup();
    if (0 != res) {
        libgps_debug_trace((DEBUG_CALLS, "WSACleanup returns error %d\n", res));
    }
    return (0 == res);
}
#endif  // HAVE_WINSOCK2_H

int gps_sock_open(const char *host, const char *port,
                  struct gps_data_t *gpsdata)
{
#ifdef USE_QT
    QTcpSocket *sock;
#else
    socket_t sock;
#endif  // USE_QT

    if (NULL == host) {
        host = "localhost";
    }
    if (NULL == port) {
        port = DEFAULT_GPSD_PORT;
    }

    libgps_debug_trace((DEBUG_CALLS, "gps_sock_open(%s, %s)\n", host, port));

#ifdef USE_QT
        // FIXNE: prevent CWE-690 warning: dereference of possibly-NULL pointer
        sock = new QTcpSocket();
        if (NULL == sock) {
            // out of memory
            exit(1);
        }
        gpsdata->gps_fd = sock;
        sock->connectToHost(host, QString(port).toInt());
        if (!sock->waitForConnected()) {
            qDebug() << "libgps::connect error: " << sock->errorString();
        } else {
            qDebug() << "libgps::connected!";
        }
#else  // USE_QT
#ifdef HAVE_WINSOCK2_H
        if (need_init) {
          need_init != windows_init();
        }
#endif  // HAVE_WINSOCK2_H
        sock = netlib_connectsock(AF_UNSPEC, host, port, "tcp");
        if (0 > sock) {
            gpsdata->gps_fd = PLACEHOLDING_FD;
            errno = sock;
            libgps_debug_trace((DEBUG_CALLS,
                               "netlib_connectsock() returns error %s(%d)\n",
                               netlib_errstr(sock), sock));
            return -1;
        }
        gpsdata->gps_fd = sock;
        libgps_debug_trace((DEBUG_CALLS,
            "netlib_connectsock() returns socket on fd %d\n",
            gpsdata->gps_fd));
#endif  // USE_QT

    // set up for line-buffered I/O over the daemon socket
    gpsdata->privdata = (void *)malloc(sizeof(struct privdata_t));
    if (NULL == gpsdata->privdata) {
        return -1;
    }
    PRIVATE(gpsdata)->newstyle = false;
    PRIVATE(gpsdata)->waiting = 0;
    PRIVATE(gpsdata)->buffer[0] = 0;

    PRIVATE(gpsdata)->waitcount = 0;
    return 0;
}

/* check if there input waiting from the GPS?
 * timeout is in uSec */
bool gps_sock_waiting(const struct gps_data_t *gpsdata, int timeout)
{
#ifdef USE_QT
    return ((QTcpSocket *)(gpsdata->gps_fd))->waitForReadyRead(timeout / 1000);
#else
    struct timespec to;

    libgps_debug_trace((DEBUG_CALLS, "gps_waiting(%d): %d\n",
                       timeout, PRIVATE(gpsdata)->waitcount++));
    if (0 < PRIVATE(gpsdata)->waiting) {
        return true;
    }

    USTOTS(&to, timeout);
    // all error conditions return "not waiting" -- crude but effective
    return nanowait(gpsdata->gps_fd, &to);
#endif  // USE_QT
}

// close a gpsd connection
int gps_sock_close(struct gps_data_t *gpsdata)
{
    free(PRIVATE(gpsdata));
    gpsdata->privdata = NULL;
#ifdef USE_QT
    QTcpSocket *sock = (QTcpSocket *) gpsdata->gps_fd;
    sock->disconnectFromHost();
    delete sock;
    gpsdata->gps_fd = NULL;
    return 0;
#else   // USE_QT
    int status;
#ifdef HAVE_WINSOCK2_H
    status = closesocket(gpsdata->gps_fd);
    if (need_finish) {
      need_finish != windows_finish();
    }
#else   // HAVE_WINSOCK2_H
    status = close(gpsdata->gps_fd);
#endif  // HAVE_WINSOCK2_H
    gpsdata->gps_fd = -1;
    return status;
#endif  // USE_QT
}

// wait for and read data being streamed from the daemon
int gps_sock_read(struct gps_data_t *gpsdata, char *message, int message_len)
{
    char *eol;
    ssize_t response_length;
    int status = -1;
    char *eptr;

    errno = 0;
    gpsdata->set &= ~PACKET_SET;

    // scan to find end of message (\n), or end of buffer
    eol = PRIVATE(gpsdata)->buffer;
    eptr = eol + PRIVATE(gpsdata)->waiting;

    while ((eol < eptr) && (*eol != '\n')) {
        eol++;
    }

    if (eol >= eptr) {
        // no full message found, try to fill buffer
        if ((ssize_t)sizeof(PRIVATE(gpsdata)->buffer) <=
            PRIVATE(gpsdata)->waiting) {
            // buffer is full but still didn't get a message
            return -1;
        }

#ifdef USE_QT
        status =
            ((QTcpSocket *)(gpsdata->gps_fd))->read(PRIVATE(gpsdata)->buffer +
                 PRIVATE(gpsdata)->waiting,
                 sizeof(PRIVATE(gpsdata)->buffer) - PRIVATE(gpsdata)->waiting);
#else   // USE_QT
        // read data: return -1 if no data waiting or buffered, 0 otherwise
        status = (int)recv(gpsdata->gps_fd,
               PRIVATE(gpsdata)->buffer + PRIVATE(gpsdata)->waiting,
               sizeof(PRIVATE(gpsdata)->buffer) - PRIVATE(gpsdata)->waiting, 0);
#endif  // USE_QT

#ifdef HAVE_WINSOCK2_H
        int wserr = WSAGetLastError();
#endif  // HAVE_WINSOCK2_H

#ifdef USE_QT
        if (0 > status) {
            /* All negative statuses are error for QT
             *
             * read: https://doc.qt.io/qt-5/qiodevice.html#read
             *
             * Reads at most maxSize bytes from the device into data,
             * and returns the number of bytes read.
             * If an error occurs, such as when attempting to read from
             * a device opened in WriteOnly mode, this function returns -1.
             *
             * 0 is returned when no more data is available for reading.
             * However, reading past the end of the stream is considered
             * an error, so this function returns -1 in those cases
             * (that is, reading on a closed socket or after a process
             * has died).
             */
            return -1;
        }

#else   // not USE_QT
        if (0 >= status) {
            /* 0 or negative
             *
             * read:
             *  https://pubs.opengroup.org/onlinepubs/007908775/xsh/read.html
             *
             * If nbyte is 0, read() will return 0 and have no other results.
             * ...
             * When attempting to read a file (other than a pipe or FIFO)
             * that supports non-blocking reads and has no data currently
             * available:
             *    - If O_NONBLOCK is set,
             *            read() will return a -1 and set errno to [EAGAIN].
             *    - If O_NONBLOCK is clear,
             *            read() will block the calling thread until some
             *            data becomes available.
             *    - The use of the O_NONBLOCK flag has no effect if there
             *       is some data available.
             * ...
             * If a read() is interrupted by a signal before it reads any
             * data, it will return -1 with errno set to [EINTR].
             * If a read() is interrupted by a signal after it has
             * successfully read some data, it will return the number of
             * bytes read.
             *
             * recv:
             *   https://pubs.opengroup.org/onlinepubs/007908775/xns/recv.html
             *
             * If no messages are available at the socket and O_NONBLOCK
             * is not set on the socket's file descriptor, recv() blocks
             * until a message arrives.
             * If no messages are available at the socket and O_NONBLOCK
             * is set on the socket's file descriptor, recv() fails and
             * sets errno to [EAGAIN] or [EWOULDBLOCK].
             * ...
             * Upon successful completion, recv() returns the length of
             * the message in bytes. If no messages are available to be
             * received and the peer has performed an orderly shutdown,
             * recv() returns 0. Otherwise, -1 is returned and errno is
             * set to indicate the error.
             *
             * Summary:
             * if nbytes 0 and read return 0 -> out of the free buffer
             * space but still didn't get correct json -> report an error
             * -> return -1
             * if read return 0 but requested some bytes to read -> other
             * side disconnected -> report an error -> return -1
             * if read return -1 and errno is in [EAGAIN, EINTR, EWOULDBLOCK]
             * -> not an error, we'll retry later -> return 0
             * if read return -1 and errno is not in [EAGAIN, EINTR,
             * EWOULDBLOCK] -> error -> return -1
             *
             */

            /*
             * check for not error cases first: EAGAIN, EINTR, etc
             */
             if (0 > status ) {
#ifdef HAVE_WINSOCK2_H
                if (WSAEINTR  == wserr ||
                    WSAEWOULDBLOCK == wserr) {
                    return 0;
                }
#else
                if (EINTR == errno ||
                    EAGAIN == errno ||
                    EWOULDBLOCK == errno) {
                    return 0;
                }
#endif  // HAVE_WINSOCK2_H
             }

             // disconnect or error
             return -1;
        }
#endif  // USE_QT

        // if we just received data from the socket, it's in the buffer
        PRIVATE(gpsdata)->waiting += status;

        // there's new buffered data waiting, check for full message
        eol = PRIVATE(gpsdata)->buffer;
        eptr = eol + PRIVATE(gpsdata)->waiting;

        while ((eol < eptr) && ('\n' != *eol)) {
            eol++;
        }

        if (eol >= eptr) {
            // still no full message, give up for now
            return 0;
        }
    }

    // eol now points to trailing \n in a full message
    *eol = '\0';
    if (NULL != message) {
        strlcpy(message, PRIVATE(gpsdata)->buffer, message_len);
    }
    (void)clock_gettime(CLOCK_REALTIME, &gpsdata->online);
    // unpack the JSON message
    status = gps_unpack(PRIVATE(gpsdata)->buffer, gpsdata);

    /*
        why the 1?

                |0|1|2|3|4|5| 6|7|
                |1|2|3|4|5|6|\n|X|
           buffer^         eol^

        buffer = 0
        eol = 6

        eol-buffer = 6-0 = 6, size of the line data is 7 bytes with \n

        eol-buffer+1 = 6-0+1 = 7

    */

    response_length = eol - PRIVATE(gpsdata)->buffer + 1;

    // calculate length of good data still in buffer
    PRIVATE(gpsdata)->waiting -= response_length;

    if (0 >= PRIVATE(gpsdata)->waiting) {
        // no waiting data, or overflow, clear the buffer, just in case
        *PRIVATE(gpsdata)->buffer = '\0';
        PRIVATE(gpsdata)->waiting = 0;
    } else {
        memmove(PRIVATE(gpsdata)->buffer,
                PRIVATE(gpsdata)->buffer + response_length,
                PRIVATE(gpsdata)->waiting);
    }
    gpsdata->set |= PACKET_SET;

    return (0 == status) ? (int)response_length : status;
}

/* unpack a gpsd response into a status structure, buf must be writeable.
 * gps_unpack() currently returns 0 in all cases, but should it ever need to
 * return an error status, it must be < 0.
 */
int gps_unpack(char *buf, struct gps_data_t *gpsdata)
{
    libgps_debug_trace((DEBUG_CALLS, "gps_unpack(%s)\n", buf));

    // detect and process a JSON response
    if ('{' == buf[0]) {
        const char *jp = buf, **next = &jp;

        while (NULL != next &&
               NULL != *next &&
               '\0' != next[0][0]) {
            libgps_debug_trace((DEBUG_CALLS,
                               "gps_unpack() segment parse '%s'\n",
                               *next));
            if (-1 == libgps_json_unpack(*next, gpsdata, next)) {
                break;
            }
            if (1 <= libgps_debuglevel) {
                libgps_dump_state(gpsdata);
            }
        }
    }

#ifndef USE_QT
    libgps_debug_trace((DEBUG_CALLS,
                        "final flags: (0x%08lx) %s\n",
                        (unsigned long)gpsdata->set,
                        gps_maskdump(gpsdata->set)));
#endif  // USE_QT
    return 0;
}

// return the contents of the client data buffer
const char *gps_sock_data(const struct gps_data_t *gpsdata)
{
    // no length data, so pretty useless...
    return PRIVATE(gpsdata)->buffer;
}

/* send a command to the gpsd instance
 *
 * Return: 0 -- success
 * Return: negative -- fail
 */
int gps_sock_send(struct gps_data_t *gpsdata, const char *buf)
{
#ifdef USE_QT
    QTcpSocket *sock = (QTcpSocket *) gpsdata->gps_fd;
    sock->write(buf, strlen(buf));
    if (sock->waitForBytesWritten()) {
        return 0;
    }

    qDebug() << "libgps::send error: " << sock->errorString();
#else   // USE_QT
    ssize_t sent;
#ifdef HAVE_WINSOCK2_H
    sent = send(gpsdata->gps_fd, buf, strlen(buf), 0);
#else
    sent = write(gpsdata->gps_fd, buf, strlen(buf));
#endif /* HAVE_WINSOCK2_H */
    if ((ssize_t)strlen(buf) == sent) {
        return 0;
    }
    (void)fprintf(stderr, "gps_sock_send() write %ld, s/b %ld\n",
                  (long)sent, (long)strlen(buf));
#endif  // USE_QT
    return -1;
}

// ask gpsd to stream reports at you, hiding the command details
int gps_sock_stream(struct gps_data_t *gpsdata, unsigned int flags, void *d)
{
    char buf[GPS_JSON_COMMAND_MAX] = "?WATCH={\"enable\":";

    if (0 == (flags & (WATCH_JSON | WATCH_NMEA | WATCH_RAW))) {
        flags |= WATCH_JSON;
    }
    if (0 != (flags & WATCH_DISABLE)) {
        (void)strlcat(buf, "false", sizeof(buf));
        if (flags & WATCH_JSON) {
            (void)strlcat(buf, ",\"json\":false", sizeof(buf));
        }
        if (flags & WATCH_NMEA) {
            (void)strlcat(buf, ",\"nmea\":false", sizeof(buf));
        }
        if (flags & WATCH_RAW) {
            (void)strlcat(buf, ",\"raw\":1", sizeof(buf));
        }
        if (flags & WATCH_RARE) {
            (void)strlcat(buf, ",\"raw\":0", sizeof(buf));
        }
        if (flags & WATCH_SCALED) {
            (void)strlcat(buf, ",\"scaled\":false", sizeof(buf));
        }
        if (flags & WATCH_TIMING) {
            (void)strlcat(buf, ",\"timing\":false", sizeof(buf));
        }
        if (flags & WATCH_SPLIT24) {
            (void)strlcat(buf, ",\"split24\":false", sizeof(buf));
        }
        if (flags & WATCH_PPS) {
            (void)strlcat(buf, ",\"pps\":false", sizeof(buf));
        }
        // no device here?
    } else {                    // if (0 != (flags & WATCH_ENABLE)) */
        (void)strlcat(buf, "true", sizeof(buf));
        if (flags & WATCH_JSON) {
            (void)strlcat(buf, ",\"json\":true", sizeof(buf));
        }
        if (flags & WATCH_NMEA) {
            (void)strlcat(buf, ",\"nmea\":true", sizeof(buf));
        }
        if (flags & WATCH_RARE) {
            (void)strlcat(buf, ",\"raw\":1", sizeof(buf));
        }
        if (flags & WATCH_RAW) {
            (void)strlcat(buf, ",\"raw\":2", sizeof(buf));
        }
        if (flags & WATCH_SCALED) {
            (void)strlcat(buf, ",\"scaled\":true", sizeof(buf));
        }
        if (flags & WATCH_TIMING) {
            (void)strlcat(buf, ",\"timing\":true", sizeof(buf));
        }
        if (flags & WATCH_SPLIT24) {
            (void)strlcat(buf, ",\"split24\":true", sizeof(buf));
        }
        if (flags & WATCH_PPS) {
            (void)strlcat(buf, ",\"pps\":true", sizeof(buf));
        }
        if (flags & WATCH_DEVICE) {
            str_appendf(buf, sizeof(buf), ",\"device\":\"%s\"", (char *)d);
        }
    }
    (void)strlcat(buf, "};", sizeof(buf));
    libgps_debug_trace((DEBUG_CALLS, "gps_sock_stream() command: %s\n", buf));
    return gps_send(gpsdata, buf);
}

/* run a socket main loop with a specified handler
 *
 * Returns: -1 on timeout
 *          -2 on read error
 * FIXME: read error should return different than timeout
 */
int gps_sock_mainloop(struct gps_data_t *gpsdata, int timeout,
                      void (*hook)(struct gps_data_t *gpsdata))
{

    for (;;) {
        int status;

        if (!gps_waiting(gpsdata, timeout)) {
            return -1;
        }
        status = gps_read(gpsdata, NULL, 0);

        if (-1 == status) {
            break;
        }
        if (0 < status) {
            (*hook)(gpsdata);
        }
    }
    return -2;
}

#endif  // SOCKET_EXPORT_ENABLE

// vim: set expandtab shiftwidth=4
