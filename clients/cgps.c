/*
 * Copyright 2005 Jeff Francis <jeff@gritch.org>
 *
 * This file is Copyright 2005 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

/*
  Jeff Francis
  jeff@gritch.org

  Kind of a curses version of xgps for use with gpsd.
*/

/* ==================================================================
   These #defines should be modified if changing the number of fields
   to be displayed.
   ================================================================== */

// Width of Compass/IMU window
#define IMU_WIDTH 80

/* This defines how much overhead is contained in the 'datawin' window
   (eg, box around the window takes two lines). */
#define DATAWIN_OVERHEAD 2

/* This defines how much overhead is contained in the 'satellites'
   window (eg, box around the window takes two lines, plus the column
   headers take another line). */
#define SATWIN_OVERHEAD 3

/* Minimum display rows are output in the 'datawin' window
   when in GPS mode.  Change this value if you add or remove fields
   from the 'datawin' window for the GPS mode. */
#define DATAWIN_GPS_ROWS 8

// rows of DOPS we can show
#define DATAWIN_DOPS_ROWS 7

// rows of more DOPS we can show
#define DATAWIN_MDOPS_ROWS 4

// rows of ECEF we can show
#define DATAWIN_ECEF_ROWS 3

/* This is how many display fields are output in the 'datawin' window
   when in COMPASS (IMU) mode.  Change this value if you add or remove fields
   from the 'datawin' window for the COMPASS mode. */
#define DATAWIN_COMPASS_FIELDS 18

/* This is how far over in the 'datawin' window to indent the field
   descriptions. */
#define DATAWIN_DESC_OFFSET 2

// This is how far over in the 'datawin' window to indent the field values.
#define DATAWIN_VALUE_OFFSET 17

/* This is the width of the 'datawin' window.  It's recommended to
   keep DATAWIN_WIDTH + SATELLITES_WIDTH <= 80 so it'll fit on a
   "standard" 80x24 screen. */
#define DATAWIN_WIDTH 45

/* This is the width of the 'satellites' window.  It's recommended to
   keep DATAWIN_WIDTH + SATELLITES_WIDTH <= 80 so it'll fit on a
   "standard" 80x24 screen. */
#define SATELLITES_WIDTH 35

/* ================================================================
   You shouldn't have to modify any #define values below this line.
   ================================================================ */

/* This is the minimum ysize we'll accept for the 'datawin' window in
   COMPASS mode. */
#define MIN_COMPASS_DATAWIN_YSIZE (DATAWIN_COMPASS_FIELDS + DATAWIN_OVERHEAD)

#include "../include/gpsd_config.h"    // must be before all includes

#include <ctype.h>
#include <curses.h>
#include <errno.h>
#ifdef HAVE_GETOPT_LONG
       #include <getopt.h>
#endif
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../include/gps.h"
#include "../include/gps_json.h"    // for GPS_JSON_RESPONSE_MAX
#include "../include/compiler.h"    // for UNUSED
#include "../include/gpsdclient.h"
#include "../include/os_compat.h"
#include "../include/timespec.h"

// FILE *dlog = NULL;     // debug

static struct gps_data_t gpsdata;
static time_t status_timer;     // Time of last state change.
static int state = 0;           // or MODE_NO_FIX=1, MODE_2D=2, MODE_3D=3
static float altfactor = METERS_TO_FEET;
static float speedfactor = MPS_TO_MPH;
static char *altunits = "ft";
static char *speedunits = "mph";
static struct fixsource_t source;
static int debug;

static WINDOW *datawin, *satellites, *messages, *slop;

static bool raw_flag = false;            // show raw JSON data
static bool show_dops = false;           // tall screen, show DOPs
static bool show_ecefs = false;          // taller screen, show ECEFs
static bool show_more_dops = false;      // tall screen, show more DOPs
static bool silent_flag = false;         // force raw JSON data off
static bool magnetic_flag = false;       // use magnetic, not true, heading
static int window_ysize = 0;             // rows in datawin
static int display_sats = 0;             // number of rows of sats to display
static bool imu_flag = false;

// pseudo-signals indicating reason for termination
#define CGPS_QUIT       0       // voluntary termination
#define GPS_GONE        -1      // GPS device went away
#define GPS_ERROR       -2      // low-level failure in GPS read
#define GPS_TIMEOUT     -3      // low-level failure in GPS waiting

/* range test an int,
 * Return: chars + NUL
 */
static const char *int_to_str(int val, int min, int max)
{
    static char buf[20];

    if (val < min ||
        val > max) {
        return "n/a";
    }
    (void)snprintf(buf, sizeof(buf), "%3d", val);
    return buf;
}

// range test an double, to tenths, return 5 chars + NUL
static const char *tenth_to_str(double val, double min, double max)
{
    static char buf[20];

    if (0 == isfinite(val) ||
        val < min ||
        val > max) {
        return "  n/a";
    }
    (void)snprintf(buf, sizeof(buf), "%5.1f", val);
    return buf;
}

// format a DOP into a 5 char string, handle NAN, INFINITE
static char *dop_to_str(double dop)
{
    static char buf[20];

    if (0 == isfinite(dop)) {
        return " n/a ";
    }
    (void)snprintf(buf, sizeof(buf), "%5.2f", dop);
    return buf;
}

// format an EP into a string, handle NAN, INFINITE
static char *ep_to_str(double ep, double factor, char *units)
{
    static char buf[20];
    double val;

    if (0 == isfinite(ep)) {
        return " n/a  ";
    }
    // somehow these go negative now and then...
    val = fabs(ep * factor);
    if (100 <= val) {
        (void)snprintf(buf, sizeof(buf), "+/-%5d %.5s", (int)val, units);
    } else {
        (void)snprintf(buf, sizeof(buf), "+/-%5.1f %.5s", val, units);
    }
    return buf;
}

// format an ECEF p and v into a string, handle NAN, INFINITE
static char *ecef_to_str(double pos, double vel)
{
    static char buf[128];

    if (0 == isfinite(pos)) {
        if (0 == isfinite(vel)) {
            // no position, no velocity
            return "             n/a    n/a      ";
        } else {
            // no position, have velocity
            (void)snprintf(buf, sizeof(buf), "  n/a % 8.3f %2.2s/s",
                           vel * altfactor, altunits);
        }
    } else {
        if (0 == isfinite(vel)) {
            // have position, no velocity
            (void)snprintf(buf, sizeof(buf), "% 14.3f %2.2s   n/a       ",
                           pos * altfactor, altunits);
        } else {
            // have position, have velocity
            (void)snprintf(buf, sizeof(buf), "% 14.3f %2.2s % 8.3f %2.2s/s",
                           pos * altfactor, altunits,
                           vel * altfactor, altunits);
        }
    }
    return buf;
}

/* Function to call when we're all done.  Does a bit of clean-up.
 *
 * Print msg to stderr, if there is one.
 *
 * never returns, it exits.
 */
static void die(int sig, const char *msg)
{
    if (!isendwin()) {
        // Move the cursor to the bottom left corner.
        (void)mvcur(0, COLS - 1, LINES - 1, 0);

        // Put input attributes back the way they were.
        (void)echo();

        // Done with curses.
        (void)endwin();
    }
    if (NULL != msg &&
        '\0' != msg[0]) {
        fputs(msg, stderr);
        fputs("\n", stderr);
    }

    // We're done talking to gpsd.
    (void)gps_close(&gpsdata);

    switch (sig) {
    case CGPS_QUIT:
        break;
    case GPS_GONE:
        (void)fprintf(stderr, "cgps: GPS hung up.\n");
        break;
    case GPS_ERROR:
        (void)fprintf(stderr, "cgps: GPS read returned error\n");
        break;
    case GPS_TIMEOUT:
        (void)fprintf(stderr, "cgps: GPS timeout\n");
        break;
    default:
        (void)fprintf(stderr, "cgps: caught signal %d\n", sig);
        break;
    }

    // Bye!
    exit(EXIT_SUCCESS);
}

static enum deg_str_type deg_type = deg_dd;

// initialize curses and set up screen windows
static void windowsetup(void)
{
    /* Set the window sizes per the following criteria:
     *
     * 1.  Set the window size to display the maximum number of
     * satellites possible, but not more than can be fit in a
     * window the size of the GPS report window. We have to set
     * the limit that way because MAXCHANNELS has been made large
     * in order to prepare for survey-grade receivers..
     *
     * 2.  If the screen size will not allow for the full complement of
     * satellites to be displayed, set the windows sizes smaller, but
     * not smaller than the number of lines necessary to display all of
     * the fields in the 'datawin'.  The list of displayed satellites
     * will be truncated to fit the available window size.  (TODO: If
     * the satellite list is truncated, omit the satellites not used to
     * obtain the current fix.)
     *
     * 3.  If the screen is tall enough to display all possible
     * satellites (MAXCHANNELS - 2) with space still left at the bottom,
     * add a window at the bottom in which to scroll raw gpsd data.
     *
     * 4.  If the screen is tall enough to display extra data, expand
     * data window down to show DOPs, ECEFs, etc.
     */
    int ysize;                // actual screen lines
    int ysize_gps;            // ysize, minus rows reserved for raw
    int row = 1;
    int slop_width;           // width of the slop window

    (void)initscr();
    // initscr sets up COLS and LINES
    ysize_gps = ysize = LINES;

    (void)noecho();
    // cbreak() ??

//    fprintf(dlog, "windowsetup(), LINES = %d COLS %d\n", LINES, COLS);
//    fflush(dlog);

    // turn off cursor
    curs_set(0);

    if (imu_flag) {
        // We're an IMU, set up accordingly.
        if (IMU_WIDTH > COLS) {
            die(0, "Your terminal not wide enough.  80 columns required.");
        }

        if (MIN_COMPASS_DATAWIN_YSIZE == ysize) {
            raw_flag = false;
            window_ysize = MIN_COMPASS_DATAWIN_YSIZE;
        } else if (MIN_COMPASS_DATAWIN_YSIZE < ysize) {
            raw_flag = true;
            window_ysize = MIN_COMPASS_DATAWIN_YSIZE;
        } else {
            die(0, "Your terminal does not have enough rows run cgps.");
        }

        datawin = newwin(window_ysize, IMU_WIDTH, 0, 0);

        // do not block waiting for user input
        (void)nodelay(datawin, true);

        if (NULL != messages) {
            (void)delwin(messages);
            messages = NULL;
        }
        if (raw_flag) {
            messages = newwin(0, 0, window_ysize, 0);

            (void)scrollok(messages, true);
            (void)wsetscrreg(messages, 0, ysize - (window_ysize));
        }

        // Do the initial compass field label setup.
        (void)mvwaddstr(datawin, row++, DATAWIN_DESC_OFFSET, "msg:");
        (void)mvwaddstr(datawin, row++, DATAWIN_DESC_OFFSET, "Time:");
        (void)mvwaddstr(datawin, row++, DATAWIN_DESC_OFFSET, "timeTag:");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Accel X:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "m/s^2");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Accel Y:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "m/s^2");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Accel Z:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "m/s^2");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Gyro T:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "deg C");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Gyro X:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "deg/s^2");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Gyro Y:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "deg/s^2");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Gyro Z:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "deg/s^2");
        (void)mvwaddstr(datawin, row++, DATAWIN_DESC_OFFSET, "Mag X:");
        (void)mvwaddstr(datawin, row++, DATAWIN_DESC_OFFSET, "Mag Y:");
        (void)mvwaddstr(datawin, row++, DATAWIN_DESC_OFFSET, "Mag Z:");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Yaw:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "deg");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Pitch:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "deg");
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET, "Roll:");
        (void)mvwaddstr(datawin, row++, IMU_WIDTH - 8, "deg");
        (void)wborder(datawin, 0, 0, 0, 0, 0, 0, 0, 0);
        // done with IMU setup

        // make it so
        (void)refresh();
        return;
    }

    if ((DATAWIN_WIDTH + SATELLITES_WIDTH) > COLS) {
        die(0, "Your terminal not wide enough");
    }

    // We're a GPS, set up accordingly.
    if (silent_flag) {
        // no messages window, use full height
        raw_flag = false;
        ysize_gps = ysize;
    } else {
        // leave 4 rows for messages window
        ysize_gps = ysize - 4;
    }
    if ((DATAWIN_OVERHEAD + DATAWIN_GPS_ROWS + DATAWIN_DOPS_ROWS +
         DATAWIN_MDOPS_ROWS + DATAWIN_ECEF_ROWS) <= ysize_gps) {
        // everything fits
        raw_flag = true;
        show_dops = true;
        show_ecefs = true;
        show_more_dops = true;
    } else if ((DATAWIN_OVERHEAD + DATAWIN_GPS_ROWS + DATAWIN_DOPS_ROWS +
                DATAWIN_MDOPS_ROWS) <= ysize_gps) {
        // everything fits, except ecef
        raw_flag = true;
        show_dops = true;
        show_ecefs = false;
        show_more_dops = true;
    } else if ((DATAWIN_OVERHEAD + DATAWIN_GPS_ROWS + DATAWIN_DOPS_ROWS +
                DATAWIN_ECEF_ROWS) <= ysize_gps) {
        // everything fits, except more dops
        raw_flag = true;
        show_dops = true;
        show_ecefs = true;
        show_more_dops = false;
    } else if ((DATAWIN_OVERHEAD + DATAWIN_GPS_ROWS +
                DATAWIN_DOPS_ROWS) <= ysize_gps) {
        // everything fits, except more dops, and ecefs
        raw_flag = true;
        show_dops = true;
        show_ecefs = false;
        show_more_dops = false;
    } else if ((DATAWIN_OVERHEAD + DATAWIN_GPS_ROWS) <= ysize_gps) {
        // barely fits, no dops,  more dops, or ecefs
        raw_flag = false;
        show_dops = true;
        show_ecefs = false;
        show_more_dops = false;
    } else {
        die(0, "Your screen is too small to run cgps.");
    }

    // compute datawin rows
    window_ysize = DATAWIN_OVERHEAD + DATAWIN_GPS_ROWS;
    if (show_dops) {
        window_ysize += DATAWIN_DOPS_ROWS;
        if (show_more_dops) {
            window_ysize += DATAWIN_MDOPS_ROWS;
        }
    } else {
        // should never happen
        show_more_dops = false;
    }
    if (show_ecefs) {
        window_ysize += DATAWIN_ECEF_ROWS;
    }

    if (silent_flag) {
        raw_flag = false;
        window_ysize = ysize;        // use full height
    }
    display_sats = window_ysize - SATWIN_OVERHEAD;

    datawin = newwin(window_ysize, DATAWIN_WIDTH, 0, 0);
    satellites = newwin(window_ysize, SATELLITES_WIDTH, 0, DATAWIN_WIDTH);

    // slop is the area to the right, past satellites, that gathers lint
    slop_width = COLS - (DATAWIN_WIDTH + SATELLITES_WIDTH);
    if (0 < slop_width) {
        slop = newwin(window_ysize, slop_width, 0,
                      DATAWIN_WIDTH + SATELLITES_WIDTH);
        (void)werase(slop);
        (void)wrefresh(slop);
    }

    // do not block waiting for user input
    (void)nodelay(datawin, true);

    if (NULL != messages) {
        (void)delwin(messages);
        messages = NULL;
    }
    if (raw_flag) {
        messages = newwin(ysize - (window_ysize), COLS, window_ysize, 0);
        (void)scrollok(messages, true);
        (void)wsetscrreg(messages, 0, ysize - (window_ysize));
    }

    (void)werase(datawin);
    (void)wborder(datawin, 0, 0, 0, 0, 0, 0, 0, 0);

    // make it so
    (void)refresh();
}


#define LINE(val)                                           \
    if (0 != isfinite(val)) {                               \
        (void)mvwprintw(datawin, row, col, "% 8.4f", val);  \
    }                                                       \
    row++;

static void update_imu(struct attitude_t *datap, int col)
{
    char scr[128];
    int row = 1;
    int col_width = 10;

    (void)mvwprintw(datawin, row++, col, "%-*s", col_width, datap->msg);
    // Print time/date.
    if (0 < datap->mtime.tv_sec) {
        (void)timespec_to_iso8601(datap->mtime, scr, sizeof(scr));
        (void)mvwprintw(datawin, row, col, "%-*s", col_width, scr);
    }
    row++;
    // Print timeTag
    if (0 != datap->timeTag) {
        (void)mvwprintw(datawin, row, col, "%10lu", datap->timeTag);
    }
    row++;

    // Fill in the accelerometers
    LINE(datap->acc_x);
    LINE(datap->acc_y);
    LINE(datap->acc_z);

    // Gyro
    LINE(datap->gyro_temp);
    LINE(datap->gyro_x);
    LINE(datap->gyro_y);
    LINE(datap->gyro_z);

    // Magnetic
    LINE(datap->mag_x);
    LINE(datap->mag_y);
    LINE(datap->mag_z);

    LINE(datap->yaw);
    LINE(datap->pitch);
    LINE(datap->roll);
}

// This gets called once for each new sentence.
static void update_imu_panel(struct gps_data_t *gpsdata,
                                 const char *message)
{
    int update = 0;
    struct attitude_t *datap;

    datap = &gpsdata->attitude;
    if (0 < datap->mtime.tv_sec) {
        if ('\0' == datap->msg[0]) {
            strncpy(datap->msg, "  ATT", sizeof(datap->msg));
        }
        update_imu(datap, 12);
        update = 1;
    }

    datap = &gpsdata->imu[0];
    if ('\0' != datap->msg[0]) {
        if (0 == strcmp("UBX-ESF-MEAS", datap->msg)) {
            update_imu(datap, 40);
            update = 1;
        }
        if (0 == strcmp("UBX-ESF-RAW", datap->msg)) {
            update_imu(datap, 60);
            update = 1;
        }
    }
    if (0 != update) {
        (void)wrefresh(datawin);
    }

    if (raw_flag && !silent_flag) {
        // Be quiet if the user requests silence.
        (void)waddstr(messages, message);
        (void)wrefresh(messages);
    }
}

/* sort the skyviews
 * Used = Y first, then used = N
 * then sort by PRN
 */
static int sat_cmp(const void *p1, const void *p2)
{
    int diff = ((struct satellite_t*)p2)->used - ((struct satellite_t*)p1)->used;
    if (0 == diff) {
        return ((struct satellite_t*)p1)->PRN - ((struct satellite_t*)p2)->PRN;
    }
    return (diff);
}


// This gets called once for each new GPS sentence.
static void update_gps_panel(struct gps_data_t *gpsdata, char *message)
{
    int newstate;
    char scr[80];
    char buf1[20], buf2[20];
    int row;
    const char *mag_str;

    /* This is for the satellite status display.  Originally lifted from
     * xgps.c.  Note that the satellite list may be truncated based on
     * available screen size, or may only show satellites used for the
     * fix.  */

     // just repaint every time.  Hides a multitude of mistakes.
    (void)werase(satellites);
    (void)mvwaddstr(satellites, 1, 1,
                    "GNSS   PRN  Elev   Azim   SNR Use");
    (void)wborder(satellites, 0, 0, 0, 0, 0, 0, 0, 0);

    (void)mvwprintw(satellites, 0, 19, "Seen %2d/Used %2d",
                    gpsdata->satellites_visible,
                    gpsdata->satellites_used);

    if (0 != (VERSION_SET &gpsdata->set)) {
        // got version, check it
        if (0 != strcmp(gpsdata->version.release, VERSION)) {
            // expected API version not available
            (void)fprintf(stderr,
                          "cgps: WARNING gpsd server release %s, expected %s, "
                          "API: %d.%d",
                          gpsdata->version.release,
                          VERSION,
                          gpsdata->version.proto_major,
                          gpsdata->version.proto_minor);
            sleep(4);
        }
    }

    if (0 != gpsdata->satellites_visible) {
        int sat_no;
        int loop_end = (display_sats < gpsdata->satellites_visible) ? \
                display_sats : gpsdata->satellites_visible;

        qsort( gpsdata->skyview, gpsdata->satellites_visible,
                sizeof( struct satellite_t), sat_cmp);
        // displayed all sats that fit, maybe all of them
        for (sat_no = 0; sat_no < loop_end; sat_no++) {
            int column = 1;     // column to write to
            char *gnssid;
            char sigid[2] = " ";
            char health = ' ';

            if (0 == gpsdata->skyview[sat_no].svid) {
                gnssid = "  ";
            } else {
                switch (gpsdata->skyview[sat_no].gnssid) {
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
                if (1 < gpsdata->skyview[sat_no].sigid &&
                    8 > gpsdata->skyview[sat_no].sigid) {
                    // Do not display L1, or missing
                    // max is 8
                    sigid[0] = '0' + gpsdata->skyview[sat_no].sigid;
                    sigid[1] = '\0';
                }
            }
            (void)mvwaddstr(satellites, sat_no + 2, column, gnssid);
            column += 2;
            (void)mvwaddstr(satellites, sat_no + 2, column,
                    int_to_str(gpsdata->skyview[sat_no].svid, 0, 500));
            column += 3;
            (void)mvwaddstr(satellites, sat_no + 2, column, sigid);
            column += 2;

            /* PRN is not unique for all GNSS systems.
             * Each GNSS (GPS, GALILEO, BeiDou, etc.) numbers their PRN from 1.
             * What we really have here is USI, Universal Sat ID
             * The USI for each GNSS satellite is unique, starting at 1.
             * Not all GPS receivers compute the USI the same way. YMMV
             *
             * Javad (GREIS) GPS receivers compute USI this way:
             * GPS is USI 1-37, GLONASS 38-70, GALILEO 71-119, SBAS 120-142,
             * QZSS 193-197, BeiDou 211-247
             *
             * Geostar GPS receivers compute USI this way:
             * GPS is USI 1 to 32, SBAS is 33 to 64, GLONASS is 65 to 96 */

            /* no GPS uses PRN 0, NMEA 4.0 here, NMEA 4.0 uses 1-437 */
            (void)mvwaddstr(satellites, sat_no + 2, column,
                            int_to_str(gpsdata->skyview[sat_no].PRN,
                                       1, 438));
            column += 4;
            (void)mvwaddstr(satellites, sat_no + 2, column,
                            tenth_to_str(gpsdata->skyview[sat_no].elevation,
                                       -90.0, 90.0));
            column += 7;
            (void)mvwaddstr(satellites, sat_no + 2, column,
                            tenth_to_str(gpsdata->skyview[sat_no].azimuth,
                                       0.0, 359.0));
            column += 6;
            (void)mvwaddstr(satellites, sat_no + 2, column,
                            tenth_to_str(gpsdata->skyview[sat_no].ss,
                                       0.0, 254.0));
            column += 5;
            if (SAT_HEALTH_BAD == gpsdata->skyview[sat_no].health) {
                // only mark known unhealthy
                health = 'u';
            }
            (void)mvwprintw(satellites, sat_no + 2, column, " %c%c ",
                            health,
                            gpsdata->skyview[sat_no].used ? 'Y' : 'N');
        }

        // Display More... ?
        if (sat_no < gpsdata->satellites_visible) {
            // Too many sats to show them all, tell the user.
            if (ERR == mvwprintw(satellites, display_sats + 2, 1, "%s",
                                 "More...")) {
            die(0, "failed to print sat win More");
        }
        }
    }
    //  else  no sats to display, screen already cleared...

    // turn off cursor
    curs_set(0);

    row = 1;

    // Print time/date. with (leap_second)
    if (0 < gpsdata->fix.time.tv_sec) {
        (void)timespec_to_iso8601(gpsdata->fix.time, scr, sizeof(scr));
    } else {
        (void)strlcpy(scr, "  n/a", sizeof(scr));
    }
    (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Time         %-21s (%2d)",
                    scr, gpsdata->leap_seconds);

    // Fill in the latitude.
    if (MODE_2D <= gpsdata->fix.mode) {
        deg_to_str2(deg_type, gpsdata->fix.latitude,
                    scr, sizeof(scr), " N", " S");
    } else {
        (void)strlcpy(scr, "n/a", sizeof(scr));
    }
    (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Latitude         %-25s", scr);

    // Fill in the longitude.
    if (MODE_2D <= gpsdata->fix.mode) {
        deg_to_str2(deg_type, gpsdata->fix.longitude,
                    scr, sizeof(scr), " E", " W");
    } else {
        (void)strlcpy(scr, "n/a", sizeof(scr));
    }
    (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Longitude        %-25s", scr);

    // Fill in the altitudes.
    if (MODE_3D <= gpsdata->fix.mode) {
        if (0 == isfinite(gpsdata->fix.altHAE)) {
            (void)strlcpy(buf1, "        n/a", sizeof(buf1));
        } else {
            (void)snprintf(buf1, sizeof(buf1), "%11.3f",
                           gpsdata->fix.altHAE * altfactor);
        }

        if (0 == isfinite(gpsdata->fix.altMSL)) {
            (void)strlcpy(buf2, "        n/a", sizeof(buf2));
        } else {
            (void)snprintf(buf2, sizeof(buf2), "%11.3f",
                           gpsdata->fix.altMSL * altfactor);
        }

    } else {
        (void)strlcpy(buf1, "        n/a", sizeof(buf1));
        (void)strlcpy(buf2, "        n/a", sizeof(buf2));
    }
    (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Alt (HAE, MSL) %-11s,%-11s %.2s",
                    buf1, buf2, altunits);

    // Fill in the speed.
    if (0 == isfinite(gpsdata->fix.speed)) {
        (void)strlcpy(scr, "  n/a", sizeof(scr));
    } else {
        (void)snprintf(scr, sizeof(scr), "%8.2f",
                       gpsdata->fix.speed * speedfactor);
    }
    (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Speed          %-21s%5s ", scr, speedunits);

    /* Fill in the track. */
    if (magnetic_flag) {
        mag_str = "(mag, var)";
    } else {
        mag_str = "(true, var)";
    }
    if (MODE_2D <= gpsdata->fix.mode &&
        0 != isfinite(gpsdata->fix.track)) {
        if (!magnetic_flag ||
            0 == isfinite(gpsdata->fix.magnetic_track)) {
            (void)snprintf(buf1, sizeof(buf1), "%5.1f",
                           gpsdata->fix.track);
        } else {
            (void)snprintf(buf1, sizeof(buf1), "%5.1f",
                gpsdata->fix.magnetic_track);
        }
        if (0 == isfinite(gpsdata->fix.magnetic_var)) {
            (void)strlcat(scr, "      ", sizeof(scr));
        } else {
            (void)snprintf(buf2, sizeof(buf2), "%6.1f",
                           gpsdata->fix.magnetic_var);
        }
    } else {
        (void)strlcpy(buf1, "n/a", sizeof(buf1));
        (void)strlcpy(buf2, "n/a", sizeof(buf2));
    }
    (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Track %-14s %6s,%6s    deg", mag_str, buf1, buf2);

    // Fill in the rate of climb.
    if (0 == isfinite(gpsdata->fix.climb)) {
        (void)strlcpy(scr, "  n/a", sizeof(scr));
    } else {
        (void)snprintf(scr, sizeof(scr), "%8.2f",
                       gpsdata->fix.climb * altfactor * 60);
    }
    (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Climb          %-*s %5s/min ", 16, scr, altunits);

    // Fill in the GPS status and the time since the last state change.
    if (0 == gpsdata->online.tv_sec &&
        0 == gpsdata->online.tv_nsec) {
        newstate = 0;
        (void)strlcpy(scr, "OFFLINE", sizeof(scr));
    } else {
        const char *fmt;
        const char *mod = "";

        newstate = gpsdata->fix.mode;
        switch (gpsdata->fix.status) {
        case STATUS_DGPS:
            mod = "DGPS ";
            break;
        case STATUS_RTK_FIX:
            mod = "RTK ";
            break;
        case STATUS_RTK_FLT:
            mod = "RTK ";
            break;
        case STATUS_DR:
            mod = "DR ";
            break;
        case STATUS_GNSSDR:
            mod = "GNSSDR ";
            break;
        case STATUS_TIME:
            mod = "FIXED ";
            break;
        case STATUS_PPS_FIX:
            mod = "P(Y) ";
            break;
        case STATUS_SIM:
            mod = "SIM ";
            break;
        default:
            // ignore:
            mod = "";
            break;
        }
        switch (gpsdata->fix.mode) {
        case MODE_2D:
            fmt = "2D %sFIX (%d secs)";
            break;
        case MODE_3D:
            if (STATUS_TIME == gpsdata->fix.status) {
                fmt = "%sSURVEYED (%d secs)";
            } else {
                fmt = "3D %sFIX (%d secs)";
            }
            break;
        default:
            fmt = "NO %sFIX (%d secs)";
            break;
        }
        (void)snprintf(scr, sizeof(scr), fmt,  mod,
                       (int)(time(NULL) - status_timer));
    }
    (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Status          %-*s", 26, scr);

    /* Note that the following fields are exceptions to the
     * sizing rule.  The minimum window size does not include these
     * fields, if the window is too small, they get excluded.  This
     * may or may not change if/when the output for these fields is
     * fixed and/or people request their permanence.  They're only
     * there in the first place because I arbitrarily thought they
     * sounded interesting. ;^) */

    if (show_dops) {
        char *ep_str;
        char *dop_str;
        static time_t last_time;

        // Fill in the estimated latitude position error, XDOP.
        ep_str = ep_to_str(gpsdata->fix.epx, altfactor, altunits);
        dop_str = dop_to_str(gpsdata->dop.xdop);
        (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Long Err  (XDOP, EPX)  %s, %-11s",
                    dop_str, ep_str);

        // Fill in the estimated longitude position error, YDOP.
        ep_str = ep_to_str(gpsdata->fix.epy, altfactor, altunits);
        dop_str = dop_to_str(gpsdata->dop.ydop);
        (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Lat Err   (YDOP, EPY)  %s, %-11s",
                    dop_str, ep_str);

        // Fill in the estimated velocity error, VDOP.
        ep_str = ep_to_str(gpsdata->fix.epv, altfactor, altunits);
        dop_str = dop_to_str(gpsdata->dop.vdop);
        (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Alt Err   (VDOP, EPV)  %s, %-11s",
                    dop_str, ep_str);

        // extra tall screen, show more DOPs
        if (show_more_dops) {

            // Fill in the estimated horizontal (2D) error, HDOP
            ep_str = ep_to_str(gpsdata->fix.eph, altfactor, altunits);
            dop_str = dop_to_str(gpsdata->dop.hdop);
            (void)mvwprintw(datawin, row++, 2,
                            "2D Err    (HDOP, CEP)  %s, %-11s",
                            dop_str, ep_str);

            // (spherical) position error, 3D error, PDOP
            ep_str = ep_to_str(gpsdata->fix.sep, altfactor, altunits);
            dop_str = dop_to_str(gpsdata->dop.pdop);
            (void)mvwprintw(datawin, row++, 2,
                            "3D Err    (PDOP, SEP)  %s, %-11s",
                            dop_str, ep_str);

            // time dilution of precision, TDOP
            // FIXME: time ep?
            dop_str = dop_to_str(gpsdata->dop.tdop);
            (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Time Err  (TDOP)       %-18s", dop_str);

            // geometric dilution of precision, GDOP
            // FIXME: gdop ep?
            dop_str = dop_to_str(gpsdata->dop.gdop);
            (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                    "Geo Err   (GDOP)       %-18s", dop_str);
        }

        // Fill in the estimated speed error, EPS.
        ep_str = ep_to_str(gpsdata->fix.eps, speedfactor, speedunits);
        (void)mvwprintw(datawin, row++, 2,
                        "Speed Err (EPS)            %-11s ", ep_str);

        // Fill in the estimated track error, EPD.
        ep_str = ep_to_str(gpsdata->fix.epd, speedfactor, "deg");
        (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                        "Track Err (EPD)        %-14s ", ep_str);

        // Fill in the time offset, milliseconds.  If we have a time.
        // Only the first in every epoch.
        // Use TOFF??
        (void)mvwaddstr(datawin, row, DATAWIN_DESC_OFFSET,
                        "Time offset");
        if (0 < gpsdata->fix.time.tv_sec) {
            if (last_time != gpsdata->fix.time.tv_sec) {
                last_time = gpsdata->fix.time.tv_sec;
                timespec_t ts_now, ts_diff;
                char ts_str[TIMESPEC_LEN];

                (void)clock_gettime(CLOCK_REALTIME, &ts_now);
                TS_SUB(&ts_diff, &ts_now, &gpsdata->fix.time);

                (void)mvwprintw(datawin, row, DATAWIN_VALUE_OFFSET + 8,
                                "%-16s s",
                                timespec_str(&ts_diff, ts_str, sizeof(ts_str)));
            }
        }
        row++;

        // Fill in the grid square (esr thought *this* one was interesting).
        // maidenhead checks for invalid lat/lon
        (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET,
                        "Grid Square             %-18s",
                        maidenhead(gpsdata->fix.latitude,
                                   gpsdata->fix.longitude));
    }
    // extra large screen, show ECEF
    if (show_ecefs) {
        char *estr;

        // Fill in the ECEF's.
        estr = ecef_to_str(gpsdata->fix.ecef.x, gpsdata->fix.ecef.vx);
        (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET, "ECEF X, VX %-*s",
                        27, estr);

        estr = ecef_to_str(gpsdata->fix.ecef.y, gpsdata->fix.ecef.vy);
        (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET, "ECEF Y, VY %-*s",
                        27, estr);

        estr = ecef_to_str(gpsdata->fix.ecef.z, gpsdata->fix.ecef.vz);
        (void)mvwprintw(datawin, row++, DATAWIN_DESC_OFFSET, "ECEF Z, VZ %-*s",
                        27, estr);
    }

    // short screen, warn user to expand up/down
    if (!show_dops ||
        !show_ecefs ||
        !show_more_dops) {
        if (ERR == mvwprintw(datawin, display_sats + 2, 2, "%s", "More...")) {
            die(0, "failed to print datawin More");
        }
    }

    // Be quiet if the user requests silence.
    if (!silent_flag && raw_flag) {
        if (NULL != message) {
            size_t message_len = strlen(message);
            if (0 < message_len) {
                if ( '\r' == message[message_len - 1]) {
                    // remove any trailing \r
                    message[message_len - 1] = '\0';
                }
                (void)wprintw(messages, "\n%s", message);
                (void)wrefresh(messages);
            }
        }
    }

    // Reset the status_timer if the state has changed.
    if (newstate != state) {
        status_timer = time(NULL);
        state = newstate;
    }

    (void)wrefresh(datawin);
    (void)wrefresh(satellites);
}

static void usage(char *prog,  int exit_code)
{
    (void)fprintf(stderr,
        "Usage: %s [-h] [-l {d|m|s}] [-m] [-s] [-V] "
        "[server[:port:[device]]]\n\n"
        "  -?                  Show this help, then exit\n"
#ifdef HAVE_GETOPT_LONG
        "  --debug DEBUG       Set debug level\n"
        "  --help              Show this help, then exit\n"
        "  --imu               Display IMU data, not GNSS data\n"
        "  --llfmt FMT         Select lat/lon format, same as -l\n"
        "  --magtrack          Display track as estimated magnetic track.\n"
        "  --silent            Be silent, don't print raw gpsd JSON.\n"
        "  --units U           Select distance and speed units, same as -u.\n"
        "  --version           Show version, then exit\n"
#endif
        "  -D DEBUG            Set debug level\n"
        "  -h                  Show this help, then exit\n"
        "  -i                  Display IMU data, not GNSS data\n"
        "  -l {d|m|s}          Select lat/lon format\n"
        "                          d = DD.ddddddd\n"
        "                          m = DD MM.mmmmmm'\n"
        "                          s = DD MM' SS.sssss\"\n"
        "  -m                  Display track as the estimated magnetic track\n"
        "  -s                  Be silent, don't print raw gpsd JSON.\n"
        "  -u {i|m|k}          Select distance and speed units\n"
        "                          i = imperial\n"
        "                          m = metric\n"
        "                          n = nautical\n"
        "  -V                  Show version, then exit\n",
        prog);

    exit(exit_code);
}

/*
 * No protocol dependencies above this line
 */

// popup code shameless taken from "man overlay".

/*
 *   Pop-up a window on top of curscr.  If row and/or col
 *   are -1 then that dimension will be centered within
 *   curscr.  Return 0 for success or -1 if malloc() failed.
 *   Pass back the working window and the saved window for the
 *   pop-up.  The saved window should not be modified.
 */
static int popup(WINDOW **work, WINDOW **save, int nrows, int ncols,
                 int row, int col)
{

    // Windows are limited to the size of curscr.
    if (LINES < nrows) {
        nrows = LINES;
    }
    if (COLS < ncols) {
        ncols = COLS;
    }
    // Center dimensions.
    if (row == -1) {
        row = (LINES - nrows) / 2;
    }
    if (col == -1) {
        col = (COLS - ncols) / 2;
    }
    // The window must fit entirely in curscr.
    if (LINES < (row + nrows)) {
        row = 0;
    }
    if (COLS < (col + ncols)) {
        col = 0;
    }
    // sanity check for coverity
    if (0 >= nrows ||
        0 >= ncols) {
        return -1;
    }
    *work = newwin(nrows, ncols, row, col);
    if (NULL == *work) {
        return -1;
    }
    if (NULL == (*save = dupwin(*work))) {
        delwin(*work);
        return -1;
    }
    overwrite(curscr, *save);
    return 0;
}

/*
 * Restore the region covered by a pop-up window.
 * Delete the working window and the saved window.
 * This function is the complement to popup().  Return
 * 0 for success or -1 for an error.
 */
static void popdown(WINDOW *work, WINDOW *save)
{
    (void)wnoutrefresh(save);
    (void)delwin(save);
    (void)delwin(work);
}

/*
 * Compute the size of a dialog box that would fit around
 * the string.
 */
static void dialsize(char *str, int *nrows, int *ncols)
{
    int rows, cols, col;

    for (rows = 1, cols = col = 0; *str != '\0'; ++str) {
        if ('\n' == *str) {
            if (cols < col)
                cols = col;
            col = 0;
            ++rows;
        } else {
            ++col;
        }
    }
    if (cols < col) {
        cols = col;
    }
    *nrows = rows;
    *ncols = cols;
}

/*
 * Write a string into a dialog box.
 */
static void dialfill(WINDOW *w, char *s)
{
    int row;

    (void)wmove(w, 1, 1);
    for (row = 1; *s != '\0'; ++s) {
        // FIXME: don't do one char at a time...
        (void)waddch(w, *((unsigned char*) s));
        if (*s == '\n') {
            wmove(w, ++row, 1);
        }
    }
    box(w, 0, 0);
}

// popup a dialog box containing str
static void dialog(char *str)
{
    WINDOW *work, *save;
    int nrows, ncols;

    // Figure out size of window.
    dialsize(str, &nrows, &ncols);
    // Create a centered working window with extra room for a border.
    (void)popup(&work, &save, nrows + 2, ncols + 2, -1, -1);
    // Write text into the working window.
    dialfill(work, str);
    // Pause for input.  wgetch() will do a wrefresh() for us.
    (void)wgetch(work);
    // Restore curscr and free windows.
    popdown(work, save);
    // Redraw curscr to remove window from physical screen.
    (void)doupdate();
}
// end popup code shameless taken from "man overlay".

// Set global degree format from c
static int set_degree(char c)
{
    int ret = 0;

    switch (c) {
    case 'd':
        FALLTHROUGH
    case 'D':
        deg_type = deg_dd;
        break;
    case 'm':
        FALLTHROUGH
    case 'M':
        deg_type = deg_ddmm;
        break;
    case 's':
        FALLTHROUGH
    case 'S':
        deg_type = deg_ddmmss;
        break;
    default:
        ret = -1;
        break;
    }
    return ret;
}

// Set global units from c
static int set_units(char c)
{
    int ret = 0;

    switch (c) {
    case 'i':
        FALLTHROUGH
    case imperial:
        altfactor = METERS_TO_FEET;
        altunits = "ft";
        speedfactor = MPS_TO_MPH;
        speedunits = "mph";
        break;
    case 'n':
        FALLTHROUGH
    case nautical:
        altfactor = METERS_TO_FEET;
        altunits = "ft";
        speedfactor = MPS_TO_KNOTS;
        speedunits = "knots";
        break;
    case 'm':
        FALLTHROUGH
    case metric:
        altfactor = 1;
        altunits = "m";
        speedfactor = MPS_TO_KPH;
        speedunits = "km/h";
        break;
    default:
        // huh?
        ret = 1;
        break;
    }
    return ret;
}

// resize_flag gets used in signal handler, so we use volatile and
// increment/decrement it, but are too lazy to do an atomic operation.
static volatile int resize_flag = 0;

// cope with terminal resize signal
static void resize(int sig UNUSED)
{
    // CWE-479: Signal Handler Use of a Non-reentrant Function
    // See: The C Standard, 7.14.1.1, paragraph 5 [ISO/IEC 9899:2011]
    // Can't log in a signal handler.  Can't even call exit().
    resize_flag++;
}

// finally do resize signal to do the terminal resize
static void do_resize(void)
{
    resize_flag--;
    if (0 > resize_flag) {
        // huh?
        resize_flag = 0;
    }
    // don'l leak memory
    if (NULL != datawin) {
        (void)delwin(datawin);
        datawin = NULL;
    }
    if (NULL != satellites) {
        (void)delwin(satellites);
        satellites = NULL;
    }
    if (NULL != slop) {
        (void)delwin(slop);
        slop = NULL;
    }
    if (NULL != messages) {
        (void)delwin(messages);
        messages = NULL;
    }
    // the only way to resize (set LINES and COLUMNS) is to end and start over
    (void)endwin();
    windowsetup();
}

static int sig_flag = 0;

static void quit_handler(int signum)
{
    // CWE-479: Signal Handler Use of a Non-reentrant Function
    // See: The C Standard, 7.14.1.1, paragraph 5 [ISO/IEC 9899:2011]
    // Can't log in a signal handler.  Can't even call exit().
    sig_flag = signum;
    return;
}

int main(int argc, char *argv[])
{
    unsigned int flags = WATCH_ENABLE;
    int wait_clicks = 0;      // cycles to wait before gpsd timeout
    // buffer to hold one JSON message
    char message[GPS_JSON_RESPONSE_MAX];
    const char *optstring = "?D:hil:msu:V";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"debug", required_argument, NULL, 'D'},
        {"help", no_argument, NULL, 'h'},
        {"imu", no_argument, NULL, 'i'},
        {"llfmt", required_argument, NULL, 'l'},
        {"magtrack", no_argument, NULL, 'm' },
        {"silent", no_argument, NULL, 's' },
        {"units", required_argument, NULL, 'u'},
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif
    // dlog = fopen("cgps.log", "w");     // debug

    // FIXME: set_degree() too...
    (void)set_units(gpsd_units());

    // Process the options.  Print help if requested.
    while (1) {
        int ch;
#ifdef HAVE_GETOPT_LONG
        ch = getopt_long(argc, argv, optstring, long_options, &option_index);
#else
        ch = getopt(argc, argv, optstring);
#endif

        if (ch == -1) {
            break;
        }

        switch (ch) {
        case 'D':
            debug = atoi(optarg);
            gps_enable_debug(debug, stderr);
            break;
        case 'i':
            imu_flag = true;
            break;
        case 'l':
            if (0 != set_degree(optarg[0])) {
                (void)fprintf(stderr, "Unknown -l argument: %s\n", optarg);
                exit(EXIT_FAILURE);
            }
            break;
        case 'm':
            magnetic_flag = true;
            break;
        case 's':
            silent_flag = true;
            break;
        case 'u':
            if (0 != set_units(optarg[0])) {
                (void)fprintf(stderr, "Unknown -u argument: %s\n", optarg);
                exit(EXIT_FAILURE);
            }
            break;
        case 'V':
            (void)fprintf(stderr, "%s: %s (revision %s)\n",
                          argv[0], VERSION, REVISION);
            exit(EXIT_SUCCESS);
        case '?':
            FALLTHROUGH
        case 'h':
            usage(argv[0], EXIT_SUCCESS);
            // never returns
            break;
        default:
            usage(argv[0], EXIT_FAILURE);
            // never returns
            break;
        }
    }

    // Grok the server, port, and device.
    if (optind < argc) {
        gpsd_source_spec(argv[optind], &source);
    } else {
        gpsd_source_spec(NULL, &source);
    }

    // Open the stream to gpsd.
    if (0 != gps_open(source.server, source.port, &gpsdata)) {
        (void)fprintf(stderr,
                      "cgps: no gpsd running or network error: %d, %s\n",
                      errno, gps_errstr(errno));
        exit(EXIT_FAILURE);
    }

    // note: we're assuming BSD-style reliable signals here
    (void)signal(SIGINT, quit_handler);
    (void)signal(SIGHUP, quit_handler);

    // Fire up curses
    windowsetup();

    // ready to handle screen resize events
    (void)signal(SIGWINCH, resize);

    status_timer = time(NULL);

    if (NULL != source.device) {
        flags |= WATCH_DEVICE;
    }
    (void)gps_stream(&gpsdata, flags, source.device);

    // heart of the client
    for (;;) {
        int ret;

        if (0 != sig_flag) {
            die(sig_flag, NULL);
        }
        if (0 != resize_flag) {
            do_resize();
        }

        // wait 1/2 second for gpsd
        ret = gps_waiting(&gpsdata, 500000);
        if (0 != sig_flag) {
            die(sig_flag, NULL);
        }
        if (0 != resize_flag) {
            do_resize();
        }
        if (!ret) {
            // 240 tries at 0.5 seconds a try is a 2 minute timeout
            if (240 < wait_clicks++) {
                die(GPS_TIMEOUT, "cgps: timeout contactong gpsd\n");
            }
        } else {
            wait_clicks = 0;
            errno = 0;
            *message = '\0';
            if (-1 == gps_read(&gpsdata, message, sizeof(message))) {
                // reconnect?
                die(errno == 0 ? GPS_GONE : GPS_ERROR,
                    "cgps: socket error 4\n");
            }
            // Here's where updates go now that things are established.
            if (imu_flag) {
                update_imu_panel(&gpsdata, message);
            } else {
                update_gps_panel(&gpsdata, message);
            }
        }
        if (0 != sig_flag) {
            die(sig_flag, NULL);
        }
        if (0 != resize_flag) {
            do_resize();
        }

        // Check for user input.
        switch (wgetch(datawin)) {
        case '?':
            FALLTHROUGH
        case 'h':
            dialog(
"Help:\n"
"c -- clear raw data area\n"
"d -- toggle dd.ddd, dd mm.m and dd mm ss.s\n"
"h -- this help\n"
"i -- imperial units\n"
"m -- metric units\n"
"n -- nautical units\n"
"q -- quit\n"
"s -- toggle raw data output\n"
"t -- toggle true/magnetic track");

            break;
        case 'c':
            // Clear the spewage area.
            (void)werase(messages);
            break;
        case 'd':
            if (deg_dd == deg_type) {;
                deg_type = deg_ddmm;
            } else if (deg_ddmm == deg_type) {
                deg_type = deg_ddmmss;
            } else {
                deg_type = deg_dd;
            }
            break;
        case 'i':
            // set imperial units
            (void)set_units('i');
            break;
        case 'm':
            // set metric units
            (void)set_units('m');
            break;
        case 'n':
            // set nautical units
            (void)set_units('n');
            break;
        case 'q':
            // Quit
            die(CGPS_QUIT, NULL);
            break;
        case 's':
            // Toggle (pause/unpause) spewage of raw gpsd data.
            silent_flag = !silent_flag;
            resize(0);
            break;
        case 't':
            // Toggle magnetic/true track
            magnetic_flag = !magnetic_flag;
            break;
        default:
            break;
        }
    }
}

// vim: set expandtab shiftwidth=4
