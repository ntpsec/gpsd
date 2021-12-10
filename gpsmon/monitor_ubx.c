/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <math.h>
#include <stdlib.h> /* for labs() */
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

    /* "heavily inspired" by monitor_nmea.c */
    if ((satwin = derwin(devicewin, 19, 28, 0, 0)) == NULL)
        return false;
    (void)wborder(satwin, 0, 0, 0, 0, 0, 0, 0, 0), (void)syncok(satwin, true);
    (void)wattrset(satwin, A_BOLD);
    display(satwin, 1, 1, "Ch PRN  Az  El S/N Flag U");
    for (i = 0; i < 16; i++)
        display(satwin, (int)(i + 2), 1, "%2d", i);
    display(satwin, 18, 7, " NAV_SVINFO ");
    (void)wattrset(satwin, A_NORMAL);

    /* "heavily inspired" by monitor_nmea.c */
    if ((navsolwin = derwin(devicewin, 13, 51, 0, 28)) == NULL)
        return false;
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
    (void)wprintw(navsolwin, "PRNs: ## PDOP: xx.x Fix 0x.. Flags 0x..");

    display(navsolwin, 12, 20, " NAV_SOL ");
    (void)wattrset(navsolwin, A_NORMAL);

    if ((dopwin = derwin(devicewin, 3, 51, 13, 28)) == NULL)
        return false;
    (void)wborder(dopwin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)wattrset(dopwin, A_BOLD);
    (void)wmove(dopwin, 1, 1);
    (void)wprintw(dopwin, "DOP [H]      [V]      [P]      [T]      [G]");
    display(dopwin, 2, 20, " NAV_DOP ");
    (void)wattrset(dopwin, A_NORMAL);

    if ((ppswin = derwin(devicewin, 3, 51, 16, 28)) == NULL)
        return false;
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

static void display_nav_svinfo(unsigned char *buf, size_t data_len)
{
    unsigned i, j, nchan;

    /* very coarse sanity check (minimal length for valid message reached?) */
    if (data_len < 8)
        return;

    nchan = getub(buf, 4);
    if (nchan > 16)
        nchan = 16;

    for (i = 0; i < nchan; i++) {
        unsigned off = 8 + 12 * i;
        unsigned ss, prn;
        int el;
        int az;
        unsigned fl;

        prn = getub(buf, off + 1);
        fl = getleu16(buf, off + 2);
        ss = getub(buf, off + 4);
        el = getsb(buf, off + 5);
        az = getles16(buf, off + 6);
        (void)wmove(satwin, (int)(i + 2), 4);
        (void)wprintw(satwin, "%3d %3d %3d  %2d %04x %c",
                      prn, az, el, ss, fl, (fl & UBX_SAT_USED) ? 'Y' : ' ');
    }
    /* clear potentially stale sat lines unconditionally */
    for (j = i; j < 16; j++) {
        (void)wmove(satwin, (int)(j + 2), 4);
        (void)wprintw(satwin, "%22s", " ");
    }

    /* update pane label, in case NAV-SAT was previously displayed */
    (void)wattrset(satwin, A_BOLD);
    display(satwin, 18, 13, "VINFO ");
    (void)wattrset(satwin, A_NORMAL);
    (void)wnoutrefresh(satwin);
    return;
}

static void display_nav_sat(unsigned char *buf, size_t data_len)
{
    unsigned i, j, nchan;

    /* very coarse sanity check (minimal length for valid message reached?) */
    if (data_len < 8)
        return;

    nchan = getub(buf, 5);
    if (nchan > 16)
        nchan = 16;

    for (i = 0; i < nchan; i++) {
        unsigned off = 8 + 12 * i;
        unsigned ss, prn, gnss;
        int el;
        int az;
        unsigned fl;

        gnss = getub(buf, off);
        prn = getub(buf, off + 1);
        fl = getleu16(buf, off + 8);
        ss = getub(buf, off + 2);
        el = getsb(buf, off + 3);
        az = getles16(buf, off + 4);

        /* Translate sat numbering to the one used in UBX-NAV-SVINFO */
        if (gnss == 2) {
            prn += 210;  // Galileo
        } else if (gnss == 3 && prn <= 5) {
            prn += 158;  // BeiDou
        } else if (gnss == 3 && prn >= 6) {
            prn += 27;   // BeiDou (continued)
        } else if (gnss == 4) {
            prn += 172;  // IMES
        } else if (gnss == 5) {
            prn += 192;  // QZSS
        } else if (gnss == 6 && prn != 255) {
            prn += 64;   // GLONASS
        }

        (void)wmove(satwin, (int)(i + 2), 4);
        (void)wprintw(satwin, "%3d %3d %3d  %2d %04x %c",
                      prn, az, el, ss, fl,
                      (fl & (UBX_SAT_USED << 3)) ? 'Y' : ' ');
    }
    /* clear potentially stale sat lines unconditionally */
    for (j = i; j < 16; j++) {
        (void)wmove(satwin, (int)(j + 2), 4);
        (void)wprintw(satwin, "%22s", " ");
    }

    /* redraw frame to close gap to shorter label */
    (void)wborder(satwin, 0, 0, 0, 0, 0, 0, 0, 0), (void)syncok(satwin, true);
    /* update pane label */
    (void)wattrset(satwin, A_BOLD);
    display(satwin, 18, 7, " NAV-SAT ");
    (void)wattrset(satwin, A_NORMAL);
    (void)wnoutrefresh(satwin);
    return;
}

static void display_nav_sol(unsigned char *buf, size_t data_len)
{
    unsigned short gw = 0;
    unsigned int tow = 0, flags;
    double epx, epy, epz, evx, evy, evz;
    unsigned char navmode;
    struct gps_data_t g;

    if (52 != data_len) {
        return;
    }
    // pacify coverity
    memset(&g, 0, sizeof(g));

    navmode = (unsigned char)getub(buf, 10);
    flags = (unsigned int)getub(buf, 11);

    if ((flags & (UBX_SOL_VALID_WEEK | UBX_SOL_VALID_TIME)) != 0) {
        tow = (unsigned int)getleu32(buf, 0);
        gw = (unsigned short)getles16(buf, 8);
    }

    epx = (double)(getles32(buf, 12) / 100.0);
    epy = (double)(getles32(buf, 16) / 100.0);
    epz = (double)(getles32(buf, 20) / 100.0);
    evx = (double)(getles32(buf, 28) / 100.0);
    evy = (double)(getles32(buf, 32) / 100.0);
    evz = (double)(getles32(buf, 36) / 100.0);
    (void)ecef_to_wgs84fix(&g.fix, epx, epy, epz, evx, evy, evz);
    /* maybe should check the ecef_to_wgs84fix() return code? */

    g.fix.epx = g.fix.epy = (double)(getles32(buf, 24) / 100.0);
    g.fix.eps = (double)(getles32(buf, 40) / 100.0);
    g.dop.pdop = (double)(getleu16(buf, 44) / 100.0);
    g.satellites_used = (int)getub(buf, 47);

    (void)wmove(navsolwin, 1, 11);
    (void)wprintw(navsolwin, "%+10.2fm %+10.2fm %+10.2fm", epx, epy, epz);
    (void)wmove(navsolwin, 2, 11);
    (void)wprintw(navsolwin, "%+9.2fm/s %+9.2fm/s %+9.2fm/s", evx, evy, evz);

    (void)wmove(navsolwin, 4, 11);
    (void)wattrset(navsolwin, A_UNDERLINE);
    (void)wprintw(navsolwin, "%12.9f  %13.9f  %8.2fm",
                  g.fix.latitude, g.fix.longitude, g.fix.altHAE);
    (void)mvwaddch(navsolwin, 4, 23, ACS_DEGREE);
    (void)mvwaddch(navsolwin, 4, 38, ACS_DEGREE);
    (void)wmove(navsolwin, 5, 11);
    // coverity says g.fix.track never set.
    (void)wprintw(navsolwin, "%6.2fm/s %5.1fo %6.2fm/s",
                  g.fix.speed, g.fix.track, g.fix.climb);
    (void)mvwaddch(navsolwin, 5, 26, ACS_DEGREE);
    (void)wattrset(navsolwin, A_NORMAL);

    (void)wmove(navsolwin, 7, 7);
    {
        unsigned int day = tow / 86400000;
        unsigned int tod = tow % 86400000;
        unsigned int h = tod / 3600000;
        unsigned int m = tod % 3600000;
        unsigned int s = m % 60000;

        m = (m - s) / 60000;

        (void)wattrset(navsolwin, A_UNDERLINE);
        (void)wprintw(navsolwin, "%u %02u:%02u:%05.2f",
                      day, h, m, (double)s / 1000);
        (void)wattrset(navsolwin, A_NORMAL);
    }
    (void)wmove(navsolwin, 8, 11);
    if ((flags & (UBX_SOL_VALID_WEEK | UBX_SOL_VALID_TIME)) != 0) {
        (void)wprintw(navsolwin, "%d+%10.3lf", gw, (double)(tow / 1000.0));
        (void)wmove(navsolwin, 8, 36);
        (void)wprintw(navsolwin, "%d", (tow / 86400000));
    }

    /* relies on the fact that epx and epy are set to same value */
    (void)wmove(navsolwin, 10, 12);
    (void)wprintw(navsolwin, "%7.2f", g.fix.epx);
    (void)wmove(navsolwin, 10, 33);
    (void)wprintw(navsolwin, "%6.2f", g.fix.epv);
    (void)wmove(navsolwin, 11, 7);
    (void)wprintw(navsolwin, "%2d", g.satellites_used);
    (void)wmove(navsolwin, 11, 15);
    (void)wprintw(navsolwin, "%5.1f", g.dop.pdop);
    (void)wmove(navsolwin, 11, 25);
    (void)wprintw(navsolwin, "0x%02x", navmode);
    (void)wmove(navsolwin, 11, 36);
    (void)wprintw(navsolwin, "0x%02x", flags);
    (void)wnoutrefresh(navsolwin);
}

static void display_nav_dop(unsigned char *buf, size_t data_len)
{
    if (data_len != 18)
        return;
    (void)wmove(dopwin, 1, 9);
    (void)wprintw(dopwin, "%4.1f", getleu16(buf, 12) / 100.0);
    (void)wmove(dopwin, 1, 18);
    (void)wprintw(dopwin, "%4.1f", getleu16(buf, 10) / 100.0);
    (void)wmove(dopwin, 1, 27);
    (void)wprintw(dopwin, "%4.1f", getleu16(buf, 6) / 100.0);
    (void)wmove(dopwin, 1, 36);
    (void)wprintw(dopwin, "%4.1f", getleu16(buf, 8) / 100.0);
    (void)wmove(dopwin, 1, 45);
    (void)wprintw(dopwin, "%4.1f", getleu16(buf, 4) / 100.0);
    (void)wnoutrefresh(dopwin);
}

static void ubx_update(void)
{
    unsigned char *buf;
    size_t data_len;
    unsigned short msgid;

    buf = session.lexer.outbuffer;
    msgid = (unsigned short)((buf[2] << 8) | buf[3]);
    data_len = (size_t) getles16(buf, 4);
    switch (msgid) {
    case UBX_NAV_SVINFO:
        display_nav_svinfo(&buf[6], data_len);
        break;
    case UBX_NAV_SAT:
        display_nav_sat(&buf[6], data_len);
        break;
    case UBX_NAV_DOP:
        display_nav_dop(&buf[6], data_len);
        break;
    case UBX_NAV_SOL:
        display_nav_sol(&buf[6], data_len);
        break;
    default:
        break;
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
