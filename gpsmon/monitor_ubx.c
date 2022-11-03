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
    display(satwin, 1, 1, "Ch PRN  Az  El S/N FLAG U");
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
    (void)mvwprintw(navsolwin,  1,  1, "ECEF Pos:");
    (void)mvwprintw(navsolwin,  2,  1, "ECEF Vel:");

    (void)mvwprintw(navsolwin,  4,  1, "LTP Pos:");
    (void)mvwprintw(navsolwin,  5,  1, "LTP Vel:");

    (void)mvwprintw(navsolwin,  7,  1, "Time:");
    (void)mvwprintw(navsolwin,  8,  1, "Time GPS:                     Day:");

    (void)mvwprintw(navsolwin, 10,  1, "Est Pos Err       m Est Vel Err       m/s");
    (void)mvwprintw(navsolwin, 11,  1, "PRNs: ## PDOP: xx.x Fix 0x..");

    (void)mvwprintw(navsolwin, 12, 22, " NAV ");
    (void)wattrset(navsolwin, A_NORMAL);
    display(navsolwin, 1, 22, "m %11sm %11sm", "", "");
    display(navsolwin, 2, 20, "m/s %9sm/s %9sm/s", "", "");
    display(navsolwin, 4, 48, "m");
    display(navsolwin, 5, 17, "m/s%6so", "");


    if (NULL == (dopwin = derwin(devicewin, 3, 51, 13, 28))) {
        return false;
    }
    (void)wborder(dopwin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)wattrset(dopwin, A_BOLD);
    (void)mvwprintw(dopwin,  1,  1, "DOP [H]      [V]      [P]      [T]      [G]");
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
static void display_nav_svinfo(unsigned char *buf, size_t data_len)
{
    int az, el, i, nchan;
    unsigned fl, off, prn, ss;

    // very coarse sanity check (minimal length for valid message reached?)
    if (data_len < 8)
        return;

    nchan = getub(buf, 4);
    if (nchan > MAXSKYCHANS)
        nchan = MAXSKYCHANS;

    for (i = 0; i < nchan; i++) {
        off = 8 + 12 * i;

        prn = getub(buf, off + 1);
        fl = getleu16(buf, off + 2);
        ss = getub(buf, off + 4);
        el = getsb(buf, off + 5);
        az = getles16(buf, off + 6);
        (void)mvwprintw(satwin, i + 2,  4, "%3d %3d %3d  %2d %04x %c",
                        prn, az, el, ss, fl, (fl & UBX_SAT_USED) ? 'Y' : ' ');
    }
    // clear potentially stale sat lines unconditionally
    for (;i < MAXSKYCHANS; i++) {
        mvwprintw(satwin, (int)(i + 2), 4, "%22s", "");
    }

    // update pane label, in case NAV-SAT was previously displayed
    (void)wattrset(satwin, A_BOLD);
    display(satwin, 18, 13, "VINFO ");
    (void)wattrset(satwin, A_NORMAL);
    (void)wnoutrefresh(satwin);
    return;
}


static void display_nav_sat(unsigned char *buf, size_t data_len)
{
    int az, el, i, nchan;
    unsigned fl, gnss, off, prn, ss;

    // very coarse sanity check (minimal length for valid message reached?)
    if (data_len < 8) {
        return;
    }

    nchan = getub(buf, 5);
    if (nchan > MAXSKYCHANS) {
        nchan = MAXSKYCHANS;
    }

#define SV session.gpsdata.skyview[i]
    for (i = 0; i < nchan; i++) {
        off = 8 + 12 * i;
        gnss = getub(buf, off);
        prn = getub(buf, off + 1);
        fl = getleu16(buf, off + 8);
        ss = getub(buf, off + 2);
        el = getsb(buf, off + 3);
        az = getles16(buf, off + 4);

        // Translate sat numbering to the one used in UBX-NAV-SVINFO
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

        (void)mvwprintw(satwin, i + 2, 4, "%3d %3d %3d  %2d %04x %c",
                        prn, az, el, ss, fl,
                        (fl & (UBX_SAT_USED << 3)) ? 'Y' : ' ');
    }
    (void)mvwprintw(navsolwin, 11,  7, "%2d", session.gpsdata.satellites_used);
    pastef(navsolwin, 11, 15, 4, "%5.1f", session.gpsdata.dop.pdop);
#undef SV

    // clear potentially stale sat lines unconditionally
    for (; i < MAXSKYCHANS; i++) {
        (void)mvwprintw(satwin, i + 2,  4, "%22s", "");
    }
#undef MAXSKYCHANS

    // update pane label, in case NAV-SAT was previously displayed
    (void)wborder(satwin, 0, 0, 0, 0, 0, 0, 0, 0), (void)syncok(satwin, true);
    (void)wattrset(satwin, A_BOLD);
    display(satwin, 18, 7, " NAV-SAT ");
    (void)wattrset(satwin, A_NORMAL);
    (void)wnoutrefresh(satwin);
    (void)wnoutrefresh(navsolwin);

    return;
}


static void display_nav_dop(unsigned char *buf, size_t data_len)
{
    if (data_len != 18) {
        return;
    }
    pastef(dopwin, 1,  9, 3, "%4.1f", getleu16(buf, 12) / 100.0);
    pastef(dopwin, 1, 18, 3, "%4.1f", getleu16(buf, 10) / 100.0);
    pastef(dopwin, 1, 27, 3, "%4.1f", getleu16(buf,  6) / 100.0);
    pastef(dopwin, 1, 36, 3, "%4.1f", getleu16(buf,  8) / 100.0);
    pastef(dopwin, 1, 45, 3, "%4.1f", getleu16(buf,  4) / 100.0);
}


static void display_nav_sol(unsigned char *buf, size_t data_len)
{
    gps_mask_t outmask;
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
    outmask = ecef_to_wgs84fix(&g.fix, epx, epy, epz, evx, evy, evz);

    g.fix.epx = g.fix.epy = (double)(getles32(buf, 24) / 100.0);
    g.fix.eps = (double)(getles32(buf, 40) / 100.0);
    g.dop.pdop = (double)(getleu16(buf, 44) / 100.0);
    g.satellites_used = (int)getub(buf, 47);

    pastef(navsolwin, 1, 11, 9, "%+10.2f", epx);
    pastef(navsolwin, 1, 24, 9, "%+10.2f", epy);
    pastef(navsolwin, 1, 37, 9, "%+10.2f", epz);
    pastef(navsolwin, 2, 11, 8, "%+9.2f", evx);
    pastef(navsolwin, 2, 24, 8, "%+9.2f", evy);
    pastef(navsolwin, 2, 37, 8, "%+9.2f", evz);
    (void)wattrset(navsolwin, A_UNDERLINE);
    if (0 != (outmask && LATLON_SET)) {
        (void)mvwprintw(navsolwin,  4, 11, "%12.9f  %13.9f  %8.2fm",
                  g.fix.latitude, g.fix.longitude, g.fix.altHAE);
    }
    (void)mvwaddch(navsolwin, 4, 23, ACS_DEGREE);
    (void)mvwaddch(navsolwin, 4, 38, ACS_DEGREE);
    // coverity says g.fix.track never set.
    if (0 != (outmask && VNED_SET)) {
        (void)mvwprintw(navsolwin,  5, 11, "%6.2fm/s %5.1fo %6.2fm/s",
                  g.fix.speed, NAN, g.fix.climb);
    }
    (void)mvwaddch(navsolwin, 5, 26, ACS_DEGREE);
    (void)wattrset(navsolwin, A_NORMAL);

    {
        uint64_t tod = tow / 1000UL;              // remove ms
        unsigned s = (unsigned)(tod % 60);
        unsigned m = (unsigned)((tod % 3600UL) / 60);
        unsigned h = (unsigned)((tod / 3600UL) % 24);
        unsigned day = (unsigned)(tod / 86400UL);

        (void)wattrset(navsolwin, A_UNDERLINE);
        (void)mvwprintw(navsolwin,  7,  7, "%u %02u:%02u:%02d.%02d",
                        day, h, m, s, (tow % 1000)  / 10);
        (void)wattrset(navsolwin, A_NORMAL);
    }
    if ((flags & (UBX_SOL_VALID_WEEK | UBX_SOL_VALID_TIME)) != 0) {
        (void)mvwprintw(navsolwin,  8, 11, "%d+%10.3lf", gw, (double)(tow / 1000.0));
        (void)mvwprintw(navsolwin,  8, 36, "%d", (tow / 86400000));
    }

    // relies on the fact that epx and epy are set to same value
    (void)mvwprintw(navsolwin, 10, 12, "%7.2f", g.fix.epx);
    (void)mvwprintw(navsolwin, 10, 33, "%6.2f", g.fix.epv);
    (void)mvwprintw(navsolwin, 11,  7, "%2d", g.satellites_used);
    (void)mvwprintw(navsolwin, 11, 15, "%5.1f", g.dop.pdop);
    (void)mvwprintw(navsolwin, 11, 25, "0x%02x", navmode);
    (void)mvwprintw(navsolwin, 11, 36, "0x%02x", flags);
    (void)wnoutrefresh(navsolwin);
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
    .min_y = 19,.min_x = 80,    // size of the device window
    .driver = &driver_ubx,
};
#endif
// vim: set expandtab shiftwidth=4
