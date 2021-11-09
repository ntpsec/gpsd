/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 * D-Bus API is here:
 *  https://dbus.freedesktop.org/doc/api/html/group__DBusMessage.html
 *
 * See "man 8 gpsd" for description of the gpsd D-Bus message.
 */
#include "../include/gpsd_config.h"    // must be before all includes

#if defined(DBUS_EXPORT_ENABLE)
#include <dbus/dbus.h>
#include "../include/gpsd.h"

static DBusConnection *connection = NULL;

/*
 * Does what is required to initialize the dbus connection
 * This is pretty basic at this point, as we don't receive commands via dbus.
 *
 * Return: 0 OK.
 *         1 fail
 */
int initialize_dbus_connection(void)
{
    DBusError error;

    dbus_error_init(&error);
    connection = dbus_bus_get(DBUS_BUS_SYSTEM, &error);
    if (NULL == connection) {
        dbus_error_free(&error);
        // report error
        return 1;
    }
    return 0;
}

// sends the current fix data for this channel via dbus
void send_dbus_fix(struct gps_device_t *channel)
{
    struct gps_data_t *gpsdata;
    struct gps_fix_t *gpsfix;
    DBusMessage *message;
    // DBusMessageIter   iter;
    dbus_uint32_t serial;       // collected, but not used
    char *gpsd_devname;
    double dtime;               // time as a double, loss of precision!

    // if the connection is non existent, return without doing anything
    if (NULL == connection) {
        return;
    }

    gpsdata = &(channel->gpsdata);
    gpsfix = &(gpsdata->fix);
    gpsd_devname = gpsdata->dev.path;

    // Send the named signal.
    /* the dbus/locationd doc fails to specify altitude as WGS84 or MSL.
     * assume altMSL
     */
    message = dbus_message_new_signal("/org/gpsd", "org.gpsd", "fix");
    dtime = TSTONS(&gpsfix->time);
    dbus_message_append_args(message,
                             DBUS_TYPE_DOUBLE, &dtime,
                             DBUS_TYPE_INT32, &(gpsfix->mode),
                             DBUS_TYPE_DOUBLE, &(gpsfix->ept),
                             DBUS_TYPE_DOUBLE, &(gpsfix->latitude),
                             DBUS_TYPE_DOUBLE, &(gpsfix->longitude),
                             DBUS_TYPE_DOUBLE, &(gpsfix->eph),
                             DBUS_TYPE_DOUBLE, &(gpsfix->altMSL),
                             DBUS_TYPE_DOUBLE, &(gpsfix->epv),
                             DBUS_TYPE_DOUBLE, &(gpsfix->track),
                             DBUS_TYPE_DOUBLE, &(gpsfix->epd),
                             DBUS_TYPE_DOUBLE, &(gpsfix->speed),
                             DBUS_TYPE_DOUBLE, &(gpsfix->eps),
                             DBUS_TYPE_DOUBLE, &(gpsfix->climb),
                             DBUS_TYPE_DOUBLE, &(gpsfix->epc),
                             DBUS_TYPE_STRING, &gpsd_devname,
                             DBUS_TYPE_INVALID);
    dbus_message_set_no_reply(message, TRUE);
    dbus_connection_send(connection, message, &serial);
    dbus_message_unref(message);
}

#endif  // DBUS_EXPORT_ENABLE
// vim: set expandtab shiftwidth=4
