/* Written by: parazyd <parazyd@dyne.org>
 *
 * To build:
  gcc gpsd-dbus.c -o gpsd-dbus \
     $(pkg-config dbus-1 glib-2.0 dbus-glib-1 --cflags --libs)
 *
 * This file is Copyright 2020 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
*/
#include <stdlib.h>
#include <stdio.h>

#include <dbus/dbus.h>
#include <dbus/dbus-glib-lowlevel.h>
#include <glib.h>


static int on_signal(int unused, DBusMessage *msg, gpointer obj) {
    DBusError error;

    /* message args */
    double time;
    int    mode;
    double time_uncertainty;
    double lattitude;
    double longitude;
    double horizontal_uncertainty;
    double altitude;
    double altitude_uncertainty;
    double course;
    double course_uncertainty;
    double speed;
    double speed_uncertainty;
    double climb;
    double climb_uncertainty;
    const char* name;

    const char *member;

    (void)obj;
    (void)unused;

    member = dbus_message_get_member(msg);

    if (strcmp(member, "fix") != 0) {
        return 0;
    }

    dbus_error_init(&error);

    if (!dbus_message_get_args(msg, &error,
            DBUS_TYPE_DOUBLE, &time,
            DBUS_TYPE_INT32,  &mode,
            DBUS_TYPE_DOUBLE, &time_uncertainty,
            DBUS_TYPE_DOUBLE, &lattitude,
            DBUS_TYPE_DOUBLE, &longitude,
            DBUS_TYPE_DOUBLE, &horizontal_uncertainty,
            DBUS_TYPE_DOUBLE, &altitude,
            DBUS_TYPE_DOUBLE, &altitude_uncertainty,
            DBUS_TYPE_DOUBLE, &course,
            DBUS_TYPE_DOUBLE, &course_uncertainty,
            DBUS_TYPE_DOUBLE, &speed,
            DBUS_TYPE_DOUBLE, &speed_uncertainty,
            DBUS_TYPE_DOUBLE, &climb,
            DBUS_TYPE_DOUBLE, &climb_uncertainty,
            DBUS_TYPE_STRING, &name,
            NULL)) {
        fprintf(stderr, "DBUS ERROR: %s\n", error.message);
        dbus_error_free(&error);
    } else {
        fprintf(stderr,
                "name: %s, time: %lf, longitude: %lf, lattitude: %lf\n",
                name, time, longitude, lattitude);
    }

    return 0;
}

GMainLoop *mainloop = NULL;

DBusConnection *system_bus = NULL;

int main() {
    mainloop = g_main_loop_new(NULL, FALSE);

    system_bus = dbus_bus_get_private(DBUS_BUS_SYSTEM, NULL);
    dbus_connection_setup_with_g_main(system_bus, NULL);


    // signal time=1604331638.623891 sender=:1.401 ->
    //  destination=(null destination) serial=6 path=/org/gpsd;
    //  interface=org.gpsd; member=fix
    dbus_bus_add_match(system_bus,
        "type='signal',interface='org.gpsd'",
        NULL);
    dbus_connection_add_filter(system_bus,
        (DBusHandleMessageFunction)on_signal,
        NULL,
        NULL);

    g_main_loop_run(mainloop);
}
