/* net_gnss_dispatch.c -- common interface to a number of Network GNSS services
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/strfuncs.h"

#define NETGNSS_TCP     "tcp://"
#define NETGNSS_UDP     "udp://"
#define NETGNSS_DGPSIP  "dgpsip://"
#define NETGNSS_NTRIP   "ntrip://"
#define NETGNSS_GPSD    "gpsd://"

// is given string a valid URI for network/service? Is so, which one?
net_link_type netgnss_uri_type(char *name)
{
    if (str_starts_with(name, NETGNSS_DGPSIP)) {
        return NET_DGPSIP;
    }

    if (str_starts_with(name, NETGNSS_GPSD)) {
        return NET_GPSD;
    }

    if (str_starts_with(name, NETGNSS_NTRIP)) {
        return NET_NTRIP;
    }

    if (str_starts_with(name, NETGNSS_TCP)) {
        return NET_TCP;
    }

    if (str_starts_with(name, NETGNSS_UDP)) {
        return NET_UDP;
    }

    return NET_LOCAL;
}

// is given string a valid URI for DGPS/NTRIP service?
// FIXME: replace with netgnss_uri_type()
bool netgnss_uri_check(char *name)
{
    switch (netgnss_uri_type(name)) {
    case NET_DGPSIP:
        FALLTHROUGH
    case NET_NTRIP:
        return true;
    default:
        return false;
    }
}


// open a connection to a DGNSS service
gps_fd_t netgnss_uri_open(struct gps_device_t *dev, char *netgnss_service)
{
    GPSD_LOG(LOG_IO, &dev->context->errout,
             "DGNSS/NTRIP: netgnss_uri_open(%s)\n", netgnss_service);

    switch (netgnss_uri_type(netgnss_service)) {
    case NET_DGPSIP:
        return dgpsip_open(dev, netgnss_service + sizeof(NETGNSS_DGPSIP) - 1);
    case NET_NTRIP:
        // could be initial open, or reopen, or...
        return ntrip_open(dev, netgnss_service + sizeof(NETGNSS_NTRIP) - 1);
    default:
        GPSD_LOG(LOG_ERROR, &dev->context->errout,
                 "DGNSS/NTRIP: Unknown/unspecified protocol for service %s\n",
                 netgnss_service);
        return -1;
    }
}

// may be time to ship a usage report to the DGNSS service
void netgnss_report(struct gps_context_t *context,
                    struct gps_device_t *gps, struct gps_device_t *dgnss)
{
    if (SERVICE_DGPSIP == dgnss->servicetype) {
        dgpsip_report(context, gps, dgnss);
    } else if (SERVICE_NTRIP == dgnss->servicetype) {
        ntrip_report(context, gps, dgnss);
    }
}

// end
// vim: set expandtab shiftwidth=4
