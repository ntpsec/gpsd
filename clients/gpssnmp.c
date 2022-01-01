/* gpssnmp - poll local gpsd for SNMP variables
 * 
 * To build this:
 *     gcc -o gpssnmp gpssnmp.c -lgps
 *
 * Copyright 2016 David Taylor <gpsd@david.taylor.name>
 *
 * Copyright 2018 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  // must be before all includes

#ifdef HAVE_GETOPT_LONG
   #include <getopt.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>                  // for strlcpy()

#include "../include/compiler.h"     // for FALLTHROUGH
#include "../include/gps.h"
#include "../include/gpsdclient.h"   // for gpsd_source_spec()
#include "../include/os_compat.h"    // for strlcpy() if needed

#define OID_VISIBLE ".1.3.6.1.2.1.25.1.31"
#define OID_USED ".1.3.6.1.2.1.25.1.32"
#define OID_SNR_AVG ".1.3.6.1.2.1.25.1.33"

static void usage(char *prog_name) {
    // "%s [-h] [-g OID] [server[:port[:device]]]\n\n"
    printf("Usage:\n"
        "%s [-h] [-g OID]\n\n"
        "Examples:\n"
        "to get OID_VISIBLE\n"
        "   $ gpssnmp -g .1.3.6.1.2.1.25.1.31\n"
        "   .1.3.6.1.2.1.25.1.31\n"
        "   gauge\n"
        "   13\n\n"
        "to get OID_USED\n"
        "   $ gpssnmp -g .1.3.6.1.2.1.25.1.32\n"
        "   .1.3.6.1.2.1.25.1.32\n"
        "   gauge\n"
        "   4\n\n"
        "to get OID_SNR_AVG\n"
        "   $ gpssnmp -g .1.3.6.1.2.1.25.1.33\n"
        "   .1.3.6.1.2.1.25.1.33\n"
        "   gauge\n"
        "   22.250000\n\n", prog_name);
}

int main (int argc, char **argv)
{
    struct gps_data_t gpsdata;
    int i;
    double snr_total=0;
    double snr_avg = 0.0;
    int status, used, visible;
    char oid[30] = "";       // requested OID
    int debug = 0;
    struct fixsource_t source;

    const char *optstring = "?D:g:hV";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"debug", required_argument, NULL, 'D'},
        {"help", no_argument, NULL, 'h'},
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif  // HAVE_GETOPT_LONG

    /* Process the options.  Print help if requested. */
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
        case '?':
            FALLTHROUGH
        case 'h':
            usage(argv[0]);
            exit(0);
            break;
        case 'g':
            strlcpy(oid, optarg, sizeof(oid));
            break;
        case 'D':
            debug = atoi(optarg);
            gps_enable_debug(debug, stderr);
            break;
        case 'V':
            (void)fprintf(stderr, "%s: %s (revision %s)\n",
                          argv[0], VERSION, REVISION);
            exit(EXIT_SUCCESS);
        default:
            usage(argv[0]);
            exit(0);
            break;
        }
    }

    if ('\0' == oid[0]) {
        (void)fprintf(stderr, "%s: ERROR: Missing option\n\n", argv[0]);
        usage(argv[0]);
        exit(1);
    }

    /* Grok the server, port, and device. */
    if (optind < argc) {
        gpsd_source_spec(argv[optind], &source);
    } else {
        gpsd_source_spec(NULL, &source);
    }

    /* Open the stream to gpsd. */
    // broken, used shared memory.
    // status = gps_open(source.server, source.port, &gpsdata);
    status = gps_open(GPSD_SHARED_MEMORY, DEFAULT_GPSD_PORT, &gpsdata);
    if (0 != status) {
        (void)fprintf(stderr, "gpssnmp: ERROR: connection failed\n");
        exit(1);
    }
    status = gps_read(&gpsdata, NULL, 0);
    if (-1 == status) {
        (void)fprintf(stderr, "gpssnmp: ERROR: read failed %d\n", status);
        exit(1);
    }
    used  = gpsdata.satellites_used;
    visible = gpsdata.satellites_visible;
    for(i = 0; i <= used; i++) {
        if (0 < gpsdata.skyview[i].used &&
            1 <  gpsdata.skyview[i].ss) {
            // printf("i: %d, P:%d, ss: %f\n", i, gpsdata.skyview[i].PRN,
            //         gpsdata.skyview[i].ss);
            snr_total+=gpsdata.skyview[i].ss;
        }
    }
    gps_close (&gpsdata);
    if (0 < used) {
        snr_avg = snr_total / used;
    }
    if (strcmp(OID_VISIBLE, oid) == 0) {
        printf(OID_VISIBLE);
        printf("\ngauge\n%d\n", visible);
    } else if (strcmp(OID_USED, oid) == 0) {
        printf(OID_USED);
        printf("\ngauge\n%d\n", used);
    } else if (strcmp(OID_SNR_AVG, oid) == 0) {
        printf(OID_SNR_AVG);
        printf("\ngauge\n%lf\n", snr_avg);
    } else {
        (void)fprintf(stderr, "%s: ERROR: Unknown OID %s\n\n",
                      argv[0], oid);
        usage(argv[0]);
        exit(1);
    }

    return 0;
}
