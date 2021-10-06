/* net_dgpsip.c -- gather and dispatch DGPS data from DGPSIP servers
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <errno.h>                    // for errno
#include <fcntl.h>
#include <netdb.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "../include/gpsd.h"

/* open a connection to a DGPSIP server
 * Return: socket on success
 *         less than zero on failure
 */
socket_t dgpsip_open(struct gps_device_t *device, const char *dgpsserver)
{
    char *colon, *dgpsport = "rtcm-sc104";
    int opts;
    char hn[256], buf[BUFSIZ];
    socket_t dsock;

    device->servicetype = SERVICE_DGPSIP;
    device->dgpsip.reported = false;
    if (NULL != (colon = strchr(dgpsserver, ':'))) {
        dgpsport = colon + 1;
        *colon = '\0';
    }
    if (!getservbyname(dgpsport, "tcp")) {
        dgpsport = DEFAULT_RTCM_PORT;
    }

    dsock = netlib_connectsock(AF_UNSPEC, dgpsserver, dgpsport, "tcp");
    if (0 <= dsock) {
        GPSD_LOG(LOG_ERROR, &device->context->errout,
                 "DGPS: can't connect to DGPS server %s, netlib error %s(%d).\n",
                 dgpsserver, netlib_errstr(dsock), dsock);
        device->gpsdata.gps_fd = PLACEHOLDING_FD;
        return dsock;
    }
    GPSD_LOG(LOG_PROG, &device->context->errout,
             "DGPS: connection to DGPS server %s established. fd=%d\n",
             dgpsserver, dsock);
    device->gpsdata.gps_fd = dsock;
    (void)gethostname(hn, sizeof(hn));
    // greeting required by some RTCM104 servers; others will ignore it
    (void)snprintf(buf, sizeof(buf), "HELO %s gpsd %s\r\nR\r\n", hn,
                   VERSION);
    if ((ssize_t)strlen(buf) !=
        write(device->gpsdata.gps_fd, buf, strlen(buf))) {
        GPSD_LOG(LOG_ERROR, &device->context->errout,
                 "DGPS: hello to DGPS server %s failed\n",
                 dgpsserver);
    }
    opts = fcntl(device->gpsdata.gps_fd, F_GETFL);

    if (0 <= opts) {
        (void)fcntl(device->gpsdata.gps_fd, F_SETFL, opts | O_NONBLOCK);
    } else {
        GPSD_LOG(LOG_ERROR, &device->context->errout,
                 "DGPS: fcntl %s failed. %s(%d)\n",
                 dgpsserver, strerror(errno), errno);
    }
    return device->gpsdata.gps_fd;
}

void dgpsip_report(struct gps_context_t *context,
                   struct gps_device_t *gps,
                   struct gps_device_t *dgpsip)
/* may be time to ship a usage report to the DGPSIP server */
{
    /*
     * 10 is an arbitrary number, the point is to have gotten several good
     * fixes before reporting usage to our DGPSIP server.
     */
    if (context->fixcnt > 10 && !dgpsip->dgpsip.reported) {
        dgpsip->dgpsip.reported = true;
        if (dgpsip->gpsdata.gps_fd > -1) {
            char buf[BUFSIZ];
            (void)snprintf(buf, sizeof(buf), "R %0.8f %0.8f %0.2f\r\n",
                           gps->gpsdata.fix.latitude,
                           gps->gpsdata.fix.longitude,
                           gps->gpsdata.fix.altMSL);
            if (write(dgpsip->gpsdata.gps_fd, buf, strlen(buf)) ==
                (ssize_t) strlen(buf))
                GPSD_LOG(LOG_IO, &context->errout, "DGPS: => dgps %s\n", buf);
            else
                GPSD_LOG(LOG_IO, &context->errout, "DGPS: write to dgps FAILED\n");
        }
    }
}

// vim: set expandtab shiftwidth=4
