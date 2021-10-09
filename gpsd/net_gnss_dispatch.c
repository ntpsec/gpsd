/* net_gnss_dispatch.c -- common interface to a number of Network GNSS services
 *
 * This file is Copyright 2010 by the GPSD project
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

#define NETGNSS_DGPSIP  "dgpsip://"
#define NETGNSS_NTRIP   "ntrip://"

// is given string a valid URI for GNSS/DGPS service?
bool netgnss_uri_check(char *name)
{
    return
        str_starts_with(name, NETGNSS_NTRIP) ||
        str_starts_with(name, NETGNSS_DGPSIP);
}


// open a connection to a DGNSS service
int netgnss_uri_open(struct gps_device_t *dev, char *netgnss_service)
{
    GPSD_LOG(LOG_IO, &dev->context->errout,
             "DGNSS/NTRIP: netgnss_uri_open(%s)\n", netgnss_service);

    if (str_starts_with(netgnss_service, NETGNSS_NTRIP)) {
        // could be initial open, or reopen, or...
        return ntrip_open(dev, netgnss_service + strlen(NETGNSS_NTRIP));
    }

    if (str_starts_with(netgnss_service, NETGNSS_DGPSIP)) {
        return dgpsip_open(dev, netgnss_service + strlen(NETGNSS_DGPSIP));
    }

#ifndef REQUIRE_DGNSS_PROTO
    return dgpsip_open(dev, netgnss_service);
#else
    GPSD_LOG(LOG_ERROR, &dev->context.errout,
             "DGNSS/NTRIP: Unknown or unspecified protocol for service %s\n",
             netgnss_service);
    return -1;
#endif
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
