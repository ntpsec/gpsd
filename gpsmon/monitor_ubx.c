/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  // must be before all includes

#include <math.h>
#include <stdint.h>           // for int64_t (tow in display_ubx_nav)
#include <stdlib.h>           // for labs()
#include <string.h>           // for memset()
#include <time.h>

#include "../include/gpsd.h"
#include "../include/bits.h"
#include "../include/gpsmon.h"

#ifdef UBLOX_ENABLE
#include "../include/driver_ubx.h"
extern const struct gps_type_t driver_ubx;
static WINDOW *satwin, *navsolwin, *dopwin, *ppswin;

#define display (void)mvwprintw

static bool ubx_initialize(void)
{
    int i;

    // "heavily inspired" by monitor_nmea.c
    if (NULL == (satwin = derwin(devicewin, 19, 28, 0, 0))) {
        return false;
    }
    (void)wborder(satwin, 0, 0, 0, 0, 0, 0, 0, 0), (void)syncok(satwin, true);
    (void)wattrset(satwin, A_BOLD);
    display(satwin, 1, 1, "Ch PRN  Az  El S/N H Q U");
    for (i = 0; i < 16; i++)
        display(satwin, i + 2, 1, "%2d", i);
    display(satwin, 18, 7, " NAV_SVINFO ");
    (void)wattrset(satwin, A_NORMAL);

    // "heavily inspired" by monitor_nmea.c
    if (NULL == (navsolwin = derwin(devicewin, 13, 51, 0, 28))) {
        return false;
    }
    (void)wborder(navsolwin, 0, 0, 0, 0, 0, 0, 0, 0),
        (void)wattrset(navsolwin, A_BOLD);
    (void)wmove(navsolwin, 1, 1);
    (void)wprintw(navsolwin, "ECEF Pos:");
    (void)wmove(navsolwin, 2, 1);
    (void)wprintw(navsolwin, "ECEF Vel:");

    (void)wmove(navsolwin, 4, 1);
    (void)wprintw(navsolwin, "LTP Pos:");
    (void)wmove(navsolwin, 5, 1);
    (void)wprintw(navsolwin, "LTP Vel:");

    (void)wmove(navsolwin, 7, 1);
    (void)wprintw(navsolwin, "Time:");
    (void)wmove(navsolwin, 8, 1);
    (void)wprintw(navsolwin, "Time GPS:                     Day:");

    (void)wmove(navsolwin, 10, 1);
    (void)wprintw(navsolwin, "Est Pos Err       m Est Vel Err       m/s");
    (void)wmove(navsolwin, 11, 1);
    (void)wprintw(navsolwin, "PRNs: ## PDOP: xx.x Fix 0x..");

    display(navsolwin, 12, 22, " NAV ");
    (void)wattrset(navsolwin, A_NORMAL);

    if (NULL == (dopwin = derwin(devicewin, 3, 51, 13, 28))) {
        return false;
    }
    (void)wborder(dopwin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)wattrset(dopwin, A_BOLD);
    (void)wmove(dopwin, 1, 1);
    (void)wprintw(dopwin, "DOP [H]      [V]      [P]      [T]      [G]");
    display(dopwin, 2, 20, " NAV_DOP ");
    (void)wattrset(dopwin, A_NORMAL);

    if (NULL == (ppswin = derwin(devicewin, 3, 51, 16, 28))) {
        return false;
    }
    (void)wborder(ppswin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)syncok(ppswin, true);
    (void)wattrset(ppswin, A_BOLD);
#define TOFF_LINE       1
#define TOFF_COLUMN     1
    (void)mvwaddstr(ppswin, TOFF_LINE, TOFF_COLUMN, "TOFF: ");
    (void)mvwaddstr(ppswin, TOFF_LINE, TOFF_COLUMN + 10, "N/A");
#define PPS_LINE        1
#define PPS_COLUMN      26
    (void)mvwaddstr(ppswin, PPS_LINE, PPS_COLUMN, "PPS: ");
    (void)mvwaddstr(ppswin, PPS_LINE, PPS_COLUMN + 10, "N/A");
    (void)wattrset(ppswin, A_NORMAL);

    return true;
}


#define MAXSKYCHANS 16
static void display_ubx_sat(void)
{
    int nchan, i;
    nchan = session.gpsdata.satellites_visible;
    if (nchan > MAXSKYCHANS)
        nchan = MAXSKYCHANS;

#define SV session.gpsdata.skyview[i]
    for (i = 0; i < nchan; i++) {
        (void)mvwprintw(satwin, i + 2, 4, "%3d --- ---  -- %d %d %c",
                        SV.PRN, SV.health,
                        SV.qualityInd, (SV.used) ? 'Y' : ' ');
        pastef(satwin, i + 2,  8, 3, "%3.0f", SV.azimuth);
        pastef(satwin, i + 2, 12, 3, "%3.0f", SV.elevation);
        pastef(satwin, i + 2, 17, 3, "%2.0f", SV.ss);
    }
    (void)wmove(navsolwin, 11, 7);
    (void)wprintw(navsolwin, "%2d", session.gpsdata.satellites_used);
    pastef(navsolwin, 11, 15, 5, "%5.1f", session.gpsdata.dop.pdop);
#undef SV

    /* clear potentially stale sat lines unconditionally */
    for (; i < MAXSKYCHANS; i++) {
        (void)wmove(satwin, i + 2, 4);
        (void)wprintw(satwin, "%21s", " ");
    }
#undef MAXSKYCHANS

    /* update pane label, in case NAV-SAT was previously displayed */
    (void)wborder(satwin, 0, 0, 0, 0, 0, 0, 0, 0), (void)syncok(satwin, true);
    (void)wattrset(satwin, A_BOLD);

    (void)wattrset(satwin, A_NORMAL);
    (void)wnoutrefresh(satwin);
    (void)wnoutrefresh(navsolwin);

    return;
}


static void display_ubx_dop(void)
{
    pastef(dopwin, 1,  9, 4, "%4.1f", session.gpsdata.dop.hdop);
    pastef(dopwin, 1, 18, 4, "%4.1f", session.gpsdata.dop.vdop);
    pastef(dopwin, 1, 27, 4, "%4.1f", session.gpsdata.dop.pdop);
    pastef(dopwin, 1, 36, 4, "%4.1f", session.gpsdata.dop.tdop);
    pastef(dopwin, 1, 45, 4, "%4.1f", session.gpsdata.dop.gdop);
    //pastef(dopwin, 1, 9, 4, "%4.1f", session.gpsdata.dop.hdop);
}


static void display_ubx_nav(gps_mask_t mask)
{
    int64_t tow;

#define SE session.newdata.ecef
    if (0 != (ECEF_SET & mask)) {
        (void)mvwprintw(navsolwin, 1, 11, "%11sm %11sm %11sm", "", "", "");
        pastef(navsolwin, 1, 11, 10, "%+10.2f", SE.x);
        pastef(navsolwin, 1, 24, 10, "%+10.2f", SE.y);
        pastef(navsolwin, 1, 37, 10, "%+10.2f", SE.z);
    }
    if (0 != (VECEF_SET & mask)) {
        (void)wmove(navsolwin, 2, 11);
        (void)wprintw(navsolwin, "%9sm/s %9sm/s %9sm/s", "", "", "");
        pastef(navsolwin, 2, 11, 10, "%+9.2f", SE.vx);
        pastef(navsolwin, 2, 24, 10, "%+9.2f", SE.vy);
        pastef(navsolwin, 2, 37, 10, "%+9.2f", SE.vz);
    }
#undef SE

#define SF session.newdata
    (void)wmove(navsolwin, 4, 11);
    (void)wattrset(navsolwin, A_UNDERLINE);
    (void)mvwprintw(navsolwin, 4, 48, "m");
    if (0 != (LATLON_SET & mask)) {
        pastef(navsolwin, 4, 11, 12, "%12.9f", SF.latitude);
        pastef(navsolwin, 4, 25, 13, "%13.9f", SF.longitude);
    }
    if (0 != (ALTITUDE_SET & mask)) {
        pastef(navsolwin, 4, 40,  8, "%8.2f", SF.altHAE);
    }
    (void)mvwaddch(navsolwin, 4, 23, ACS_DEGREE);
    (void)mvwaddch(navsolwin, 4, 38, ACS_DEGREE);
    (void)wmove(navsolwin, 5, 11);
    // coverity says g.fix.track never set.
    (void)wprintw(navsolwin, "%6sm/s%6so%7sm/s",
                  "", "", "");
    if (0 != (SPEED_SET & mask)) {
        pastef(navsolwin, 5, 11, 6, "%6.2f", SF.speed);
    }
    if (0 != (TRACK_SET & mask)) {
        pastef(navsolwin, 5, 21, 5, "%5.1f", SF.track);
    }
    pastef(navsolwin, 5, 28, 6, "%6.2f", SF.climb);
    (void)mvwaddch(navsolwin, 5, 26, ACS_DEGREE);
    (void)wattrset(navsolwin, A_NORMAL);

    pastef(navsolwin, 10, 12, 7, "%7.2f", session.newdata.eph);
    pastef(navsolwin, 10, 33, 6, "%6.2f", session.newdata.epv);

    (void)wmove(navsolwin, 11, 25);
    (void)wprintw(navsolwin, "0x%02x", session.newdata.mode);
#undef SF

    if (0 != (TIME_SET & mask)) {
        // Note: iTOW is GPS time, not UTC.
        tow = session.driver.ubx.iTOW;
        unsigned ms = (unsigned)(tow % 1000);
        uint64_t tod = tow / 1000UL;              // remove ms
        unsigned s = (unsigned)(tod % 60);
        unsigned m = (unsigned)((tod % 3600UL) / 60);
        unsigned h = (unsigned)((tod / 3600UL) % 24);
        unsigned day = (unsigned)(tod / 86400UL);

        (void)wmove(navsolwin, 7, 7);
        (void)wattrset(navsolwin, A_UNDERLINE);
        (void)wprintw(navsolwin, "%u %02u:%02u:%02u.%02u", day, h, m, s, ms);
        (void)wattrset(navsolwin, A_NORMAL);

        (void)wmove(navsolwin, 8, 11);
        (void)wprintw(navsolwin, "%d+%10.3lf", session.context->gps_week,
                      (double)(tow / 1000.0));
        (void)wmove(navsolwin, 8, 36);
        // FIXME: isn't this just day?
        (void)wprintw(navsolwin, "%llu",
                      (unsigned long long)(tow / 86400000UL));
    }

    (void)wnoutrefresh(navsolwin);
}


static void ubx_update(void)
{
    gps_mask_t mask = 0;

    mask = session.device_type->parse_packet(&session);
    if (0 != (SATELLITE_SET & mask)) {
        display_ubx_sat();
    }
    display_ubx_nav(mask);
    if (0 != (DOP_SET & mask)) {
        display_ubx_dop();
    }

    toff_update(ppswin, TOFF_LINE, TOFF_COLUMN + 6);
    pps_update(ppswin, PPS_LINE, PPS_COLUMN + 5);
}

static int ubx_command(char line[]UNUSED)
{
    return COMMAND_UNKNOWN;
}

static void ubx_wrap(void)
{
    (void)delwin(satwin);
    return;
}

const struct monitor_object_t ubx_mmt = {
    .initialize = ubx_initialize,
    .update = ubx_update,
    .command = ubx_command,
    .wrap = ubx_wrap,
    .min_y = 19,.min_x = 80,    /* size of the device window */
    .driver = &driver_ubx,
};
#endif
// vim: set expandtab shiftwidth=4
