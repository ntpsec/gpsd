/*
 * A simple command-line exerciser for the library.
 * Not really useful for anything but debugging.
 * SPDX-License-Identifier: BSD-2-clause
 */
#include "../include/gpsd_config.h"  // must be before all includes

#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/gps.h"
#include "../include/libgps.h"
#include "../include/gpsdclient.h"

static void onsig(int sig)
{
    (void)fprintf(stderr, "libgps: died with signal %d\n", sig);
    exit(EXIT_FAILURE);
}

#ifdef SOCKET_EXPORT_ENABLE
/* Must start zeroed, otherwise the unit test will try to chase garbage
 * pointer fields. */
static struct gps_data_t gpsdata;
#endif

int main(int argc, char *argv[])
{
    struct gps_data_t collect;
    struct fixsource_t source;
    char buf[BUFSIZ];
    int option;
    bool batchmode = false;
    bool forwardmode = false;
    char *fmsg = NULL;
    int debug = 0;

    // FIXME: signal() not portable.  Use sigaction() instead.
    (void)signal(SIGSEGV, onsig);
#ifdef SIGBUS
    (void)signal(SIGBUS, onsig);
#endif

    while (-1 != (option = getopt(argc, argv, "bf:hsD:?"))) {
	switch (option) {
	case 'b':
	    batchmode = true;
	    break;
	case 'f':
	    forwardmode = true;
	    fmsg = optarg;
	    break;
	case 's':
	    (void)printf("Sizes: fix=%zu gpsdata=%zu rtcm2=%zu rtcm3=%zu "
                         "ais=%zu compass=%zu raw=%zu devices=%zu policy=%zu "
                         "version=%zu, noise=%zu\n",
		         sizeof(struct gps_fix_t),
		         sizeof(struct gps_data_t), sizeof(struct rtcm2_t),
		         sizeof(struct rtcm3_t), sizeof(struct ais_t),
		         sizeof(struct attitude_t), sizeof(struct rawdata_t),
		         sizeof(collect.devices), sizeof(struct gps_policy_t),
		         sizeof(struct version_t), sizeof(struct gst_t));
	    exit(EXIT_SUCCESS);
	case 'D':
	    debug = atoi(optarg);
	    break;
	case '?':
	case 'h':
	default:
	    (void)fputs("usage: test_libgps [-b] [-f fwdmsg] [-D lvl] "
                        "[-s] [server[:port:[device]]]\n", stderr);
	    exit(EXIT_FAILURE);
	}
    }

    // Grok the server, port, and device.
    if (optind < argc) {
	gpsd_source_spec(argv[optind], &source);
    } else {
	gpsd_source_spec(NULL, &source);
    }

    gps_enable_debug(debug, stdout);
    if (batchmode) {
#ifdef SOCKET_EXPORT_ENABLE
	while (NULL != fgets(buf, sizeof(buf), stdin)) {
	    if ('{'  == buf[0] ||
                isalpha( (int) buf[0])) {
		gps_unpack(buf, &gpsdata);
		libgps_dump_state(&gpsdata);
	    }
	}
#endif
    } else if (0 != gps_open(source.server, source.port, &collect)) {
	(void)fprintf(stderr,
		      "test_libgps: no gpsd running or network error: %d, %s\n",
		      errno, gps_errstr(errno));
	exit(EXIT_FAILURE);
    } else if (forwardmode) {
	if (-1 == gps_send(&collect, fmsg)) {
	  (void)fprintf(stderr,
			"test_libgps: gps send error: %d, %s\n",
			errno, gps_errstr(errno));
	}
	if (-1 == gps_read(&collect, NULL, 0)) {
	  (void)fprintf(stderr,
			"test_libgps: gps read error: %d, %s\n",
			errno, gps_errstr(errno));
	}
#ifdef SOCKET_EXPORT_ENABLE
	libgps_dump_state(&collect);
#endif
	(void)gps_close(&collect);
    } else {
	int tty = isatty(0);

	if (tty) {
	    (void)fputs("This is the gpsd exerciser.\n", stdout);
        }
	for (;;) {
	    if (tty) {
		(void)fputs("> ", stdout);
            }
	    if (NULL == fgets(buf, sizeof(buf), stdin)) {
		if (tty) {
		    putchar('\n');
                }
		break;
	    }
	    collect.set = 0;
	    (void)gps_send(&collect, buf);
	    (void)gps_read(&collect, NULL, 0);
#ifdef SOCKET_EXPORT_ENABLE
	    libgps_dump_state(&collect);
#endif
	}
	(void)gps_close(&collect);
    }
    return 0;
}

