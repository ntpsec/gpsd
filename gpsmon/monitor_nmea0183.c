/*
 * monitor_nmea0183.c - gpsmon support for NMEA devices.
 *
 * To do: Support for GPGLL, GPGBS, GPZDA, PASHR NMEA sentences.
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>                   // for labs()
#include <string.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/gpsmon.h"
#include "../include/gpsdclient.h"
#include "../include/strfuncs.h"

extern const struct gps_type_t driver_nmea0183;

static WINDOW *cookedwin, *nmeawin, *satwin, *gprmcwin;
static WINDOW *gpggawin, *gpgsawin, *gpgstwin;
static timespec_t last_tick, tick_interval;
// string to store the message types seen
// 132 for no good reason, longer than the field it goes into
static char sentences[132];

/*****************************************************************************
 *
 * NMEA0183 support
 *
 *****************************************************************************/

#define SENTENCELINE    1       // index of sentences line in the NMEA window

// define all window width constants at one location
// WIDTH shall be >= 80
#define WIDTH_L 25
#define WIDTH_M 27
#define WIDTH_R 30
#define WIDTH (WIDTH_L + WIDTH_M + WIDTH_R - 2)

#define HEIGHT_1 3
#define HEIGHT_2 3
#define HEIGHT_3 9
// set to 6 for 80x24 screen, set to 7 for 80x25 screen
#define HEIGHT_4 6
#define HEIGHT (HEIGHT_1 + HEIGHT_2 + HEIGHT_3 + HEIGHT_4)
// max satellites we can display
#define MAXSATS (HEIGHT_3 + HEIGHT_4 - 3)

static bool nmea_initialize(void)
{
    cookedwin = derwin(devicewin, HEIGHT_1, WIDTH, 0, 0);
    assert(cookedwin !=NULL);
    (void)wborder(cookedwin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)syncok(cookedwin, true);
    (void)wattrset(cookedwin, A_BOLD);
    (void)mvwaddstr(cookedwin, 1, 1, "Time: ");
    (void)mvwaddstr(cookedwin, 1, 34, "Lat:");
    (void)mvwaddstr(cookedwin, 1, 57, "Lon: ");
    (void)mvwaddstr(cookedwin, HEIGHT_1-1, WIDTH/2 - 6, " Cooked TPV ");
    (void)wattrset(cookedwin, A_NORMAL);

    nmeawin = derwin(devicewin, HEIGHT_2, WIDTH, HEIGHT_1, 0);
    assert(nmeawin !=NULL);
    (void)wborder(nmeawin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)syncok(nmeawin, true);
    (void)wattrset(nmeawin, A_BOLD);
    (void)mvwaddstr(nmeawin, HEIGHT_2-1, WIDTH/2 - 6, " Sentences ");
    (void)wattrset(nmeawin, A_NORMAL);

    satwin = derwin(devicewin, MAXSATS + 3, WIDTH_L, HEIGHT_1 + HEIGHT_2, 0);
    if (NULL == satwin ||
        OK != wborder(satwin, 0, 0, 0, 0, 0, 0, 0, 0) ||
        OK != syncok(satwin, true) ||
        OK != wattrset(satwin, A_BOLD) ||
        OK != mvwprintw(satwin, 1, 1, " SVID  PRN  Az El SN HU") ||
        OK != mvwprintw(satwin, MAXSATS+2, WIDTH_L/2 - 3, " GSV ") ||
        OK != wattrset(satwin, A_NORMAL)) {
        (void)fputs("gpsmon:ERROR: creating satwin foiled\n", stderr);
        exit(EXIT_FAILURE);
    }

    gprmcwin = derwin(devicewin, HEIGHT_3, WIDTH_M, HEIGHT_1 + HEIGHT_2,
                      WIDTH_L - 1);
    assert(gprmcwin !=NULL);
    (void)wborder(gprmcwin, 0, 0, 0, 0, 0, 0, 0, 0),
        (void)syncok(gprmcwin, true);
    (void)wattrset(gprmcwin, A_BOLD);
    (void)mvwprintw(gprmcwin, 1, 1, "Time: ");
    (void)mvwprintw(gprmcwin, 2, 1, "Latitude:");
    (void)mvwprintw(gprmcwin, 3, 1, "Longitude:");
    (void)mvwprintw(gprmcwin, 4, 1, "Speed: ");
    (void)mvwprintw(gprmcwin, 5, 1, "Course: ");
    (void)mvwprintw(gprmcwin, 6, 1, "Status:            FAA: ");
    (void)mvwprintw(gprmcwin, 7, 1, "MagVar: ");
    (void)mvwprintw(gprmcwin, HEIGHT_3-1, WIDTH_M/2 - 3, " RMC ");
    (void)wattrset(gprmcwin, A_NORMAL);

    gpgsawin = derwin(devicewin, HEIGHT_4, WIDTH_M,
                      HEIGHT_1 + HEIGHT_2 + HEIGHT_3, WIDTH_L - 1);
    assert(gpgsawin !=NULL);
    (void)wborder(gpgsawin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)syncok(gpgsawin, true);
    (void)wattrset(gpgsawin, A_BOLD);
#define MODE_LINE       1
    (void)mvwprintw(gpgsawin, MODE_LINE, 1, "Mode: ");

#if HEIGHT_4 > 6
// show SATS in own line to gain more space
#define SATS_LINE       2
#define SATS_COL        1
#else
// show SATS together with MODE in one line (show less SATS)*/
#define SATS_LINE       1
#define SATS_COL        10
#endif
    (void)mvwprintw(gpgsawin, SATS_LINE, SATS_COL, "Sats: ");
#define DOP_LINE        (SATS_LINE + 1)
    (void)mvwprintw(gpgsawin, DOP_LINE, 1, "DOP H=     V=     P=");
#define TOFF_LINE       (SATS_LINE + 2)
    (void)mvwprintw(gpgsawin, TOFF_LINE, 1, "TOFF: ");
    (void)mvwaddstr(gpgsawin, TOFF_LINE, 7, "N/A");
#define PPS_LINE        (SATS_LINE + 3)
    (void)mvwprintw(gpgsawin, PPS_LINE, 1, "PPS: ");
    (void)mvwaddstr(gpgsawin, PPS_LINE, 6, "N/A");
    (void)mvwprintw(gpgsawin, HEIGHT_4-1, WIDTH_M/2 - 6, " GSA + PPS ");
    (void)wattrset(gpgsawin, A_NORMAL);
    (void)syncok(gpgsawin, true);

    gpggawin = derwin(devicewin, HEIGHT_3, WIDTH_R,
                      HEIGHT_1 + HEIGHT_2, WIDTH_L + WIDTH_M - 2);
    assert(gpggawin !=NULL);
    (void)wborder(gpggawin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)syncok(gpggawin, true);
    (void)wattrset(gpggawin, A_BOLD);
    (void)mvwprintw(gpggawin, 1, 1, "Time: ");
    (void)mvwprintw(gpggawin, 2, 1, "Latitude: ");
    (void)mvwprintw(gpggawin, 3, 1, "Longitude: ");
    (void)mvwprintw(gpggawin, 4, 1, "Altitude: ");
    (void)mvwprintw(gpggawin, 5, 1, "Quality:       Sats: ");
    (void)mvwprintw(gpggawin, 6, 1, "HDOP: ");
    (void)mvwprintw(gpggawin, 7, 1, "Geoid: ");
    (void)mvwprintw(gpggawin, HEIGHT_3-1, WIDTH_R/2 - 3, " GGA ");
    (void)wattrset(gpggawin, A_NORMAL);

    gpgstwin = derwin(devicewin, HEIGHT_4, WIDTH_R,
                      HEIGHT_1 + HEIGHT_2 + HEIGHT_3, WIDTH_L + WIDTH_M - 2);
    assert(gpgstwin !=NULL);
    (void)wborder(gpgstwin, 0, 0, 0, 0, 0, 0, 0, 0);
    (void)syncok(gpgstwin, true);
    (void)wattrset(gpgstwin, A_BOLD);
    (void)mvwprintw(gpgstwin, 1,  1, "UTC:");
    (void)mvwprintw(gpgstwin, 1, 16, "RMS:");
    (void)mvwprintw(gpgstwin, 2,  1, "MAJ:");
    (void)mvwprintw(gpgstwin, 2, 16, "MIN:");
    (void)mvwprintw(gpgstwin, 3,  1, "ORI:");
    (void)mvwprintw(gpgstwin, 3, 16, "LAT:");
    (void)mvwprintw(gpgstwin, 4,  1, "LON:");
    (void)mvwprintw(gpgstwin, 4, 16, "ALT:");
    (void)mvwprintw(gpgstwin, HEIGHT_4-1, WIDTH_R/2 - 3, " GST ");
    (void)wattrset(gpgstwin, A_NORMAL);


    (void)clock_gettime(CLOCK_REALTIME, &last_tick);

    sentences[0] = '\0';

    return (nmeawin != NULL);
}

static void cooked_pvt(void)
{
    char scr[128];

    if (0 < session.gpsdata.fix.time.tv_sec) {
        (void)timespec_to_iso8601(session.gpsdata.fix.time, scr, sizeof(scr));
    } else {
        (void)snprintf(scr, sizeof(scr), "n/a");
    }
    (void)mvwprintw(cookedwin, 1, 7, "%-24s", scr);


    if (session.gpsdata.fix.mode >= MODE_2D) {
        deg_to_str2(deg_ddmm, session.gpsdata.fix.latitude,
                    scr, sizeof(scr), " N", " S");
    } else {
        (void)strlcpy(scr, "n/a", sizeof(scr));
    }
    (void)mvwprintw(cookedwin, 1, 38, "%-17s", scr);

    if (session.gpsdata.fix.mode >= MODE_2D) {
        deg_to_str2(deg_ddmm, session.gpsdata.fix.longitude,
                    scr, sizeof(scr), " E", " W");
    } else {
        (void)strlcpy(scr, "n/a", sizeof(scr));
    }
    (void)mvwprintw(cookedwin, 1, 62, "%-17s", scr);
}

// display as much as we can of a satlist in a specified window
static void monitor_satlist(WINDOW *win, int y, int x)
{
    int ymax, xmax;
    char scr[128], tmp[128];
    int i;

    assert(win != NULL);
    if (OK != wmove(win, y, x) ||
        OK != wclrtoeol(win)) {
        (void)fputs("gpsmon:ERROR: monitor_satlist foiled\n", stderr);
        exit(EXIT_FAILURE);
    }
    scr[0] = '\0';
    tmp[0] = '\0';
    getmaxyx(win, ymax, xmax);
    assert(ymax != 0);  // suppress compiler warning

    for (i = 0; i < MAXCHANNELS; i++) {
        if (session.gpsdata.skyview[i].used) {
            str_appendf(tmp, sizeof(tmp),
                        "%d ", session.gpsdata.skyview[i].PRN);
            if ((int)strnlen(tmp, sizeof(tmp)) < xmax - 1 - x) {
                str_appendf(scr, sizeof(scr),
                            "%d ", session.gpsdata.skyview[i].PRN);
            } else {
                // no more roome.
                str_appendf(scr, sizeof(scr), "+");
                break;
            }
        }
    }

    if (OK != mvwaddnstr(win, y, x, scr, xmax - 1 - x)) {
        (void)fputs("gpsmon:ERROR: monitor_satlist foiled\n", stderr);
        exit(EXIT_FAILURE);
    }
}

/* sort the skyviews
 * Used = Y first, then used = N
 * then sort by PRN, and sigid
 */
static int sat_cmp(const void *p1, const void *p2)
{
   int ret = ((const struct satellite_t*)p2)->used -
              ((const struct satellite_t*)p1)->used;
   if (ret) {
        return ret;
   }
   ret = ((const struct satellite_t*)p1)->PRN -
          ((const struct satellite_t*)p2)->PRN;
   if (ret) {
        return ret;
   }
   ret = ((const struct satellite_t*)p1)->sigid -
          ((const struct satellite_t*)p2)->sigid;
   return ret;
}

static void nmea_update(void)
{
    char **fields;

    assert(cookedwin != NULL);
    assert(nmeawin != NULL);
    assert(gpgsawin != NULL);
    assert(gpggawin != NULL);
    assert(gprmcwin != NULL);
    assert(gpgstwin != NULL);

    // can be NULL if packet was overlong
    fields = session.nmea.field;

    // why do we care about outbuffer??
    if ((unsigned char)'$' == session.lexer.outbuffer[0] &&
        NULL != fields &&
        NULL != fields[0]) {

        int ymax, xmax;
        timespec_t now;
        timespec_t ts_diff;
        size_t field0_len = strnlen(fields[0], NMEA_MAX_FLD);

        getmaxyx(nmeawin, ymax, xmax);
        assert(ymax > 0);
        assert(xmax > 0);

        // Add message type to sentence line??
        if (NULL == strstr(sentences, fields[0]) &&
            10 < xmax) {
            /* limit to smaller of window width or sentence[]
             * leave space in the window for line draw chars */
            size_t max = (size_t)(xmax - 2) < sizeof(sentences) ? \
                         (size_t)(xmax - 2) : sizeof(sentences);
            size_t s_len = strnlen(sentences, max);
            char *s_end = sentences + s_len;

            if ((s_len + field0_len + 2) < max) {
                // room for more
                *s_end++ = ' ';
                *s_end = '\0';
                (void)strlcpy(s_end, fields[0], field0_len + 2);
            } else if (3 < (s_end - sentences)) {
                // no room for more, but room for '...'
                *--s_end = '.';
                *--s_end = '.';
                *--s_end = '.';
            }
            (void)mvwaddstr(nmeawin, SENTENCELINE, 1, sentences);
        }

        /*
         * If the interval between this and last update is
         * the longest we've seen yet, boldify the corresponding
         * tag.
         */
        (void)clock_gettime(CLOCK_REALTIME, &now);
        TS_SUB(&ts_diff, &now, &last_tick);
        if (TS_GZ(&ts_diff) && TS_GT(&ts_diff, &tick_interval)) {
            char *findme = strstr(sentences, fields[0]);

            tick_interval = ts_diff;
            if (NULL != findme) {
                (void)mvwchgat(nmeawin, SENTENCELINE, 1, xmax - 13, A_NORMAL,
                               0, NULL);
                (void)mvwchgat(nmeawin, SENTENCELINE, 1 + (findme - sentences),
                               (int)field0_len, A_BOLD, 0, NULL);
            }
        }
        last_tick = now;

        if (5 != field0_len) {
            // we only decode a few sentences
        } else if (0 == strcmp(fields[0] + 2, "GSV") &&
                   0 == strcmp(fields[1], fields[2])) {
            /* this is a fake, GSV not decoded here, using sats from JSON
             * fields[1] is current GSV sentence, fields[2] is total sentences.
             * So we only look at the last one.
             */
            int i;
            int nsats =
                (session.gpsdata.satellites_visible <
                 MAXSATS) ? session.gpsdata.satellites_visible : MAXSATS;

            // sort, so at least we see used.
            qsort(session.gpsdata.skyview, session.gpsdata.satellites_visible,
                  sizeof( struct satellite_t), sat_cmp);
            for (i = 0; i < nsats; i++) {
                // FIXME: gnssid and svid to string s/b common to all monitors.
                char *gnssid = "  ";
                char sigid = ' ';

                switch (session.gpsdata.skyview[i].gnssid) {
                default:
                    gnssid = "  ";
                    break;
                case GNSSID_GPS:
                    gnssid = "GP";  // GPS
                    break;
                case GNSSID_SBAS:
                    gnssid = "SB";  // SBAS
                    break;
                case GNSSID_GAL:
                    gnssid = "GA";  // GALILEO
                    break;
                case GNSSID_BD:
                    gnssid = "BD";  // BeiDou
                    break;
                case GNSSID_IMES:
                    gnssid = "IM";  // IMES
                    break;
                case GNSSID_QZSS:
                    gnssid = "QZ";  // QZSS
                    break;
                case GNSSID_GLO:
                    gnssid = "GL";  // GLONASS
                    break;
                case GNSSID_IRNSS:
                    gnssid = "IR";  // IRNSS
                    break;
                }
                if (1 < session.gpsdata.skyview[i].sigid &&
                    8 > session.gpsdata.skyview[i].sigid) {
                    // Do not display L1, or missing
                    // max is 8
                    sigid = '0' + session.gpsdata.skyview[i].sigid;
                }
                if (OK != wmove(satwin, i + 2, 1) ||
                    OK != wprintw(satwin, "%.2s%3d%c %3d %3d %2d %2.0f %c%c",
                              gnssid,
                              // svid can be 3 digits
                              session.gpsdata.skyview[i].svid,
                              sigid,
                              session.gpsdata.skyview[i].PRN,
                              // degrees, 000..359
                              (int)session.gpsdata.skyview[i].azimuth,
                              // degrees, 00..90
                              (int)session.gpsdata.skyview[i].elevation,
                              // 00-99 dB-Hz, NAN, or zero, when not tracking
                              // FIXME: check isfinite()
                              session.gpsdata.skyview[i].ss,
                              SAT_HEALTH_BAD ==
                                  session.gpsdata.skyview[i].health ? 'u' : ' ',
                              session.gpsdata.skyview[i].used ? 'Y' : 'N'
                             )) {
                    (void)fputs("gpsmon:ERROR: filling satwin failed\n",
                                stderr);
                    exit(EXIT_FAILURE);
                }
            }
            // clear the rest of the sat lines
            // use i from above loop
            for (; i < MAXSATS; i++) {
                if (OK != mvwprintw(satwin, i + 2, 1,
                                    "                       ")) {
                    (void)fputs("gpsmon:ERROR: filling satwin failed\n",
                                stderr);
                    exit(EXIT_FAILURE);
                }
            }
            // add overflow mark to the display
            if (OK != mvwaddch(satwin, MAXSATS + 2, 4,
                        session.gpsdata.satellites_visible <= MAXSATS ?
                            ACS_HLINE : ACS_DARROW)) {
                (void)fputs("gpsmon:ERROR: overflow satwin failed\n", stderr);
                exit(EXIT_FAILURE);
            }
        } else if (0 == strcmp(fields[0] + 2, "RMC")) {
            // time, lat, lon, course, speed
            if (OK != mvwaddstr(gprmcwin, 1, 11, fields[1]) ||
                OK != mvwprintw(gprmcwin, 2, 11, "%12s %s",
                               fields[3], fields[4]) ||
                OK != mvwprintw(gprmcwin, 3, 11, "%12s %s",
                               fields[5], fields[6]) ||
                OK != mvwaddstr(gprmcwin, 4, 11, fields[7]) ||
                OK != mvwaddstr(gprmcwin, 5, 11, fields[8]) ||
                // the status field, FAA code, and magnetic variation
                OK != mvwaddstr(gprmcwin, 6, 11, fields[2]) ||
                OK != mvwaddstr(gprmcwin, 6, 24, fields[12]) ||
                OK != mvwprintw(gprmcwin, 7, 11, "%-5s%s",
                               fields[10], fields[11])) {
                (void)fputs("gpsmon:ERROR: gprmcwin failed\n", stderr);
                exit(EXIT_FAILURE);
            }
            cooked_pvt();       // cooked version of TPV
        } else if (0 == strcmp(fields[0] + 2, "GSA")) {
            if (OK != mvwprintw(gpgsawin, MODE_LINE, 7, "%1s%s",
                                fields[1], fields[2]) ||
                OK != mvwprintw(gpgsawin, DOP_LINE, 7, "%-5s", fields[16]) ||
                OK != mvwprintw(gpgsawin, DOP_LINE, 14, "%-5s", fields[17]) ||
                OK != mvwprintw(gpgsawin, DOP_LINE, 21, "%-5s", fields[15])) {
                (void)fputs("gpsmon:ERROR: gpgsawin failed\n", stderr);
                exit(EXIT_FAILURE);
            }
            monitor_satlist(gpgsawin, SATS_LINE, SATS_COL + 6);
            monitor_fixframe(gpgsawin);
        } else if (0 == strcmp(fields[0] + 2, "GGA")) {
            if (OK != mvwprintw(gpggawin, 1, 12, "%-17s", fields[1]) ||
                OK != mvwprintw(gpggawin, 2, 12, "%-17s", fields[2]) ||
                OK != mvwprintw(gpggawin, 3, 12, "%-17s", fields[4]) ||
                OK != mvwprintw(gpggawin, 4, 12, "%-17s", fields[9]) ||
                OK != mvwprintw(gpggawin, 5, 12, "%1.1s", fields[6]) ||
                OK != mvwprintw(gpggawin, 5, 22, "%2.2s", fields[7]) ||
                OK != mvwprintw(gpggawin, 6, 12, "%-5.5s", fields[8]) ||
                OK != mvwprintw(gpggawin, 7, 12, "%-5.5s", fields[11])) {
                (void)fputs("gpsmon:ERROR: gpggawin failed\n", stderr);
                exit(EXIT_FAILURE);
            }
        } else if (0 == strcmp(fields[0] + 2, "GST")) {
            if (OK != mvwprintw(gpgstwin, 1,  6, "%-10s", fields[1]) ||
                OK != mvwprintw(gpgstwin, 1, 21,  "%-8s", fields[2]) ||
                OK != mvwprintw(gpgstwin, 2,  6, "%-10s", fields[3]) ||
                OK != mvwprintw(gpgstwin, 2, 21,  "%-8s", fields[4]) ||
                OK != mvwprintw(gpgstwin, 3,  6, "%-10s", fields[5]) ||
                OK != mvwprintw(gpgstwin, 3, 21,  "%-8s", fields[6]) ||
                OK != mvwprintw(gpgstwin, 4,  6, "%-10s", fields[7]) ||
                OK != mvwprintw(gpgstwin, 4, 21,  "%-8s", fields[8])) {
                (void)fputs("gpsmon:ERROR: gpgstwin failed\n", stderr);
                exit(EXIT_FAILURE);
            }
        }
        toff_update(gpgsawin, TOFF_LINE, 7);
    }

    pps_update(gpgsawin, PPS_LINE, 6);
}

#undef SENTENCELINE

static void nmea_wrap(void)
{
    if (OK != delwin(nmeawin) ||
        OK != delwin(gpgsawin) ||
        OK != delwin(gpggawin) ||
        OK != delwin(gprmcwin)) {
        (void)fputs("gpsmon:ERROR: nmea_wrap failed\n", stderr);
        exit(EXIT_FAILURE);
    }
}

const struct monitor_object_t nmea_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT,.min_x = WIDTH,
    .driver = &driver_nmea0183,
};

/*****************************************************************************
 *
 * Extended NMEA support
 *
 *****************************************************************************/

static void monitor_nmea_send(const char *fmt, ...)
{
    char buf[BUFSIZ];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf) - 5, fmt, ap);
    va_end(ap);
    (void)monitor_control_send((unsigned char *)buf, strnlen(buf, sizeof(buf)));
}

/*
 * Yes, it's OK for most of these to be clones of the generic NMEA monitor
 * object except for the pointer to the GPSD driver.  That pointer makes
 * a difference, as it will automatically enable stuff like speed-switcher
 * and mode-switcher commands.  It's really only necessary to write a
 * separate monitor object if you want to change the device-window
 * display or implement device-specific commands.
 */

#if defined(GARMIN_ENABLE)
extern const struct gps_type_t driver_garmin;

const struct monitor_object_t garmin_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_garmin,
};
#endif  // GARMIN_ENABLE

extern const struct gps_type_t driver_ashtech;

#define ASHTECH_SPEED_9600 5
#define ASHTECH_SPEED_57600 8

static int ashtech_command(char line[])
{
    switch (line[0]) {
    case 'N':                   // normal = 9600, GGA+GSA+GSV+RMC+ZDA
        monitor_nmea_send("$PASHS,NME,ALL,A,OFF");  // silence outbound chatter
        monitor_nmea_send("$PASHS,NME,ALL,B,OFF");
        monitor_nmea_send("$PASHS,NME,GGA,A,ON");
        monitor_nmea_send("$PASHS,NME,GSA,A,ON");
        monitor_nmea_send("$PASHS,NME,GSV,A,ON");
        monitor_nmea_send("$PASHS,NME,RMC,A,ON");
        monitor_nmea_send("$PASHS,NME,ZDA,A,ON");

        monitor_nmea_send("$PASHS,INI,%d,%d,,,0,",
                          ASHTECH_SPEED_9600, ASHTECH_SPEED_9600);
        (void)sleep(6);         // it takes 4-6 sec for the receiver to reboot
        monitor_nmea_send("$PASHS,WAS,ON");     // enable WAAS
        break;

    case 'R':                 // raw = 57600, normal+XPG+POS+SAT+MCA+PBN+SNV
        monitor_nmea_send("$PASHS,NME,ALL,A,OFF");  // silence outbound chatter
        monitor_nmea_send("$PASHS,NME,ALL,B,OFF");
        monitor_nmea_send("$PASHS,NME,GGA,A,ON");
        monitor_nmea_send("$PASHS,NME,GSA,A,ON");
        monitor_nmea_send("$PASHS,NME,GSV,A,ON");
        monitor_nmea_send("$PASHS,NME,RMC,A,ON");
        monitor_nmea_send("$PASHS,NME,ZDA,A,ON");

        monitor_nmea_send("$PASHS,INI,%d,%d,,,0,",
                          ASHTECH_SPEED_57600, ASHTECH_SPEED_9600);
        (void)sleep(6);         // it takes 4-6 sec for the receiver to reboot
        monitor_nmea_send("$PASHS,WAS,ON");     // enable WAAS

        monitor_nmea_send("$PASHS,NME,POS,A,ON");     // Ashtech TPV solution
        monitor_nmea_send("$PASHS,NME,SAT,A,ON");   // Ashtech Satellite status
        monitor_nmea_send("$PASHS,NME,MCA,A,ON");     // MCA measurements
        monitor_nmea_send("$PASHS,NME,PBN,A,ON");     // ECEF TPV solution
        monitor_nmea_send("$PASHS,NME,SNV,A,ON,10");  // Almanac data

        monitor_nmea_send("$PASHS,NME,XMG,A,ON");     // exception messages
        break;

    default:
        return COMMAND_UNKNOWN;
    }

    return COMMAND_UNKNOWN;
}

const struct monitor_object_t ashtech_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = ashtech_command,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_ashtech,
};

#ifdef FV18_ENABLE
extern const struct gps_type_t driver_fv18;

const struct monitor_object_t fv18_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_fv18,
};
#endif // FV18_ENABLE

#ifdef GPSCLOCK_ENABLE
extern const struct gps_type_t driver_gpsclock;

const struct monitor_object_t gpsclock_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_gpsclock,
};
#endif // GPSCLOCK_ENABLE

extern const struct gps_type_t driver_mtk3301;

const struct monitor_object_t mtk3301_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_mtk3301,
};

#ifdef AIVDM_ENABLE
extern const struct gps_type_t driver_aivdm;

const struct monitor_object_t aivdm_mmt = {
    .initialize = nmea_initialize,
    .update = nmea_update,
    .command = NULL,
    .wrap = nmea_wrap,
    .min_y = HEIGHT, .min_x = WIDTH,
    .driver = &driver_aivdm,
};
#endif  // AIVDM_ENABLE

// vim: set expandtab shiftwidth=4:
