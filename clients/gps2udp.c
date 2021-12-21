/*
 * gps2udp
 *
 * Dump NMEA to UDP socket for AIShub
 *      gps2udp -u data.aishub.net:1234
 *
 * Author: Fulup Ar Foll (directly inspired from gpspipe.c)
 * Date:   2013-03-01
 *
 * This file is Copyright 2013 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h"  /* must be before all includes */

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#ifdef HAVE_GETOPT_LONG
       #include <getopt.h>   // for getopt_long()
#endif
#include <netdb.h>        /* for gethostbyname() */
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
// do not use strsep() it is not POSIX
#include <string.h>      /* for strlcpy(), strtok(), etc. */
#include <strings.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/gpsdclient.h"
#include "../include/strfuncs.h"
#include "../include/timespec.h"

#define MAX_TIME_LEN 80
#define MAX_GPSD_RETRY 10

static struct gps_data_t gpsdata;

/* UDP socket variables */
#define MAX_UDP_DEST 5
static struct sockaddr_in remote[MAX_UDP_DEST];
static int sock[MAX_UDP_DEST];
static int udpchannel;

// gpsclient source
static struct fixsource_t gpsd_source;
static unsigned int flags;
static unsigned int debug = 0;
static bool aisonly = false;
static bool tpvonly = false;

// return local time hh:mm:ss
static char* time2string(void)
{
   static char buffer[MAX_TIME_LEN];
   time_t curtime;
   struct tm *loctime;

   // Get the current time.
   curtime = time(NULL);

   // Convert it to local time representation.
   loctime = localtime(&curtime);

   // Print it out in a nice format.
   (void)strftime(buffer, sizeof(buffer), "%H:%M:%S", loctime);

   return buffer;
}

static int send_udp(char *nmeastring, size_t ind)
{
    char message[MAX_PACKET_LENGTH];
    char *buffer;
    int  channel;
    static char mstr[] = "{\"class\":\"TPV\",";

    // if string length is unknown make a copy and compute it
    if (0 == ind) {
        // compute message size and add 0x0a 0x0d
        for (ind=0; nmeastring [ind] != '\0'; ind ++) {
            if ((sizeof(message) - 3) <= ind) {
                (void)fprintf(stderr, "gps2udp: too big [%s] \n", nmeastring);
                return -1;
            }
            message[ind] = nmeastring[ind];
        }
        buffer = message;
    } else {
        // use directly nmeastring but change terminition
        buffer = nmeastring;
        ind = ind-1;
    }
    // Add termination to NMEA feed for AISHUB
    buffer[ind] = '\r'; ind++;
    buffer[ind] = '\n'; ind++;
    buffer[ind] = '\0';

    if (0 == (flags & WATCH_JSON) &&
        '{' == buffer[0]) {
        // do not send JSON when not configured to do so
        // JSON, skip it
        if (1 < debug) {
            (void)fprintf(stdout, "...j [%s] '%s'\n", time2string(), buffer);
        }
        return 0;
    }
    if (tpvonly &&
        (0 != strncmp(mstr, buffer, sizeof(mstr) - 1))) {
        // only TPV requests, but not TPV, skip it
        if (1 < debug) {
            (void)fprintf(stdout, "...t [%s] '%s'\n", time2string(), buffer);
        }
        return 0;
    }

    // send message on udp channel
    for (channel=0; channel < udpchannel; channel ++) {
        ssize_t status = sendto(sock[channel],
                                buffer,
                                ind,
                                0,
                                (struct sockaddr *)&remote[channel],
                                (int)sizeof(remote));
        if (status < (ssize_t)ind) {
            (void)fprintf(stderr, "gps2udp: failed to send [%s] \n",
                          buffer);
            return -1;
        }
    }
    return 0;
}


// Open and bind udp socket to host
static int open_udp(char **hostport)
{
   int channel;

   for (channel = 0; channel < udpchannel; channel++) {
       char *hostname = NULL;
       char *portname = NULL;
       char *endptr = NULL;
       int  portnum;
       struct hostent *hp;

       if (NULL == hostport[channel]) {
           // pacify coverity
           (void)fprintf(stderr, "gps2udp: syntax is [-u hostname:port]\n");
           return -1;
       }
       // parse argument
       hostname = strtok(hostport[channel], ":");
       // NULL tells strtok() to resume search from last found token
       portname = strtok(NULL, ":");
       if ((NULL == hostname) ||
           (NULL == portname)) {
           (void)fprintf(stderr, "gps2udp: syntax is [-u hostname:port]\n");
           return -1;
       }

       errno = 0;
       portnum = (int)strtol(portname, &endptr, 10);
       if (1 > portnum || 65535 < portnum || '\0' != *endptr || 0 != errno) {
           (void)fprintf(stderr, "gps2udp: syntax is [-u hostname:port] "
                         "[%s] is not a valid port number\n", portname);
           return -1;
       }

       sock[channel]= socket(AF_INET, SOCK_DGRAM, 0);
       if (0 > sock[channel]) {
           (void)fprintf(stderr, "gps2udp: error creating UDP socket\n");
           return -1;
       }

       remote[channel].sin_family = (sa_family_t)AF_INET;
       hp = gethostbyname(hostname);
       if (NULL == hp) {
           (void)fprintf(stderr,
                         "gps2udp: syntax is [-u hostname:port] [%s]"
                         " is not a valid hostname\n",
                         hostname);
           return -1;
       }

       memcpy( &remote[channel].sin_addr, hp->h_addr_list[0], hp->h_length);
       remote[channel].sin_port = htons((in_port_t)portnum);
    }
    return 0;
}

static void usage(void)
{
    (void)fprintf(stderr,
                  "Usage: gps2udp [OPTIONS] [server[:port[:device]]]\n\n"
#ifdef HAVE_GETOPT_LONG
                  "  -?                  Show this help, then exit\n"
                  "  --ais               Select AIS messages only.\n"
                  "  --count COUNT       exit after count packets.\n"
                  "  --daemon            Daemonize\n"
                  "  --debug DEBUGLEVEL  See -d for DEBUGLEVEL\n"
                  "  --help              Show this help, then exit\n"
                  "  --json              Feed JSON messages only.\n"
                  "  --nmea              Feed NMEA messages only.\n"
                  "  --tpv               Feed TPV JSON messages only.\n"
                  "                      Implies --json.\n"
                  "  --udp HOST:PORT     Send UDP feed to host:port.\n"
                  "                      Up to five --udp accepted.\n"
                  "  --version           Show version, then exit\n"
#endif
                  "  -a                  Select AIS messages only.\n"
                  "  -b                  Run in background as a daemon.\n"
                  "  -c COUNT            Exit after count packets.\n"
                  "  -d [0-2]            1 display sent packets, "
                  "2 display ignored packets.\n"
                  "  -h                  Show this help.\n"
                  "  -j                  Feed JSON.\n"
                  "  -n                  Feed NMEA.\n"
                  "  -t                  Feed TPV JSON messages only.\n"
                  "                      Implies --json.\n"
                  "  -u HOST:PORT        Send UDP NMEA/JSON feed to "
                  "host:port.\n"
                  "                      Up to five -u accepted.\n"
                  "  -V                  Print version and exit.\n"
                  "\n"
                  "example: gps2udp -a -n -c 2 -d 1 -u data.aishub.net:2222 "
                  "fridu.net\n"
                  );
}

// loop until we connect with gpsd
static void connect2gpsd(bool restart)
{
    unsigned int delay;

    if (restart) {
        (void)gps_close(&gpsdata);
        if (0 < debug) {
            (void)fprintf(stdout,
                          "gps2udp [%s] reset gpsd connection\n",
                          time2string());
        }
    }

    // loop until we reach GPSd
    for (delay = 10; ; delay = delay * 2) {
        int status = gps_open(gpsd_source.server, gpsd_source.port, &gpsdata);
        if (0 != status) {
            (void)fprintf(stderr,
                          "gps2udp [%s] connection failed at %s:%s\n",
                          time2string(), gpsd_source.server, gpsd_source.port);
           (void)sleep(delay);
        } else {
            if (0 < debug) {
                (void)fprintf(stdout, "gps2udp [%s] connect to gpsd %s:%s\n",
                              time2string(), gpsd_source.server,
                              gpsd_source.port);
            }
            break;
        }
    }
    // select the right set of gps data
    (void)gps_stream(&gpsdata, flags, gpsd_source.device);

}

// get data from gpsd
static ssize_t read_gpsd(char *message, size_t len)
{
    int ind;
    char c;
    int retry = 0;
    struct timespec to;

    // allow room for trailing NUL
    len--;

    // loop until we get some data or an error
    for (ind = 0; ind < (int)len;) {
        // prepare for a blocking read with a 10s timeout
        to.tv_sec = 10;
        to.tv_nsec = 0;
        int result = nanowait(gpsdata.gps_fd, &to);

        switch (result) {
        case 1:
            // we have data waiting, let's process them
            // FIXME!  Do not do one at a time!
            result = (int)read(gpsdata.gps_fd, &c, 1);

            // If we lost gpsd connection reset it
            if (1 != result ) {
                connect2gpsd (true);
            }

            if (('\n' == c) ||
                ('\r' == c)) {
                message[ind]='\0';

                if (0 < ind) {
                    if (0 < retry) {
                        if (1 == debug) {
                            (void)fprintf (stdout,"\r");
                        } else if (1 < debug) {
                            (void)fprintf(stdout,
                                          " [%s] No Data for: %ds\n",
                                          time2string(), retry*10);
                        }
                    }

                    if (tpvonly &&
                        '{' != message[0]) {
                        if (1 < debug) {
                            (void)fprintf(stdout,
                                          "...{ [%s %d] '%s'\n", time2string(),
                                          ind, message);
                        }
                        return 0;
                    }
                    if (aisonly &&
                        '!' != message[0]) {
                        if (1 < debug) {
                            (void)fprintf(stdout,
                                          "...! [%s %d] '%s'\n", time2string(),
                                          ind, message);
                        }
                        return 0;
                    }
                }

                return (ssize_t)ind + 1;
            } else {
                message[ind]= c;
                ind++;
            }
            break;

        case 0: // no data fail in timeout
            retry++;
            // if too many empty packets are received reset gpsd connection
            if (MAX_GPSD_RETRY < retry) {
                connect2gpsd(true);
                retry = 0;
            }
            if (0 < debug) {
                ignore_return(write (1, ".", 1));
            }
            break;

        default:        // we lost connection with gpsd
            connect2gpsd(true);
            break;
        }
    }
    message[ind] = '\0';
    (void)fprintf (stderr,"\n gps2udp: message too big [%s]\n", message);
    return -1;
}

// 6 bits decoding of AIS payload
static unsigned char AISto6bit(unsigned char c)
{
    unsigned char cp = c;

    if((unsigned char)0x30 > c) {
        return (unsigned char)-1;
    }
    if((unsigned char)0x77 < c) {
        return (unsigned char)-1;
    }
    if(((unsigned char)0x57 < c) &&
       ((unsigned char)0x60 > c)) {
        return (unsigned char)-1;
    }

    cp += (unsigned char)0x28;

    if((unsigned char)0x80 < cp) {
        cp += (unsigned char)0x20;
    } else {
        cp += (unsigned char)0x28;
    }
    return (unsigned char)(cp & (unsigned char)0x3f);
}

// get MMSI from AIS bit string
static unsigned int AISGetInt(unsigned char *bitbytes, unsigned int sp,
                              unsigned int len)
{
    unsigned int acc = 0;
    unsigned int s0p = sp-1;                          // to zero base
    unsigned int i;

    for(i = 0; i < len; i++) {
        unsigned int cp, cx, c0;
        acc  = acc << 1;
        cp = (s0p + i) / 6;
        cx = (unsigned int)bitbytes[cp];      // what if cp >= byte_length?
        c0 = (cx >> (5 - ((s0p + i) % 6))) & 1;
        acc |= c0;
    }

    return acc;
}

int main(int argc, char **argv)
{
    bool daemonize = false;
    long count = -1;
    char *udphostport[MAX_UDP_DEST];
    const char *optstring = "?abc:d:hjntu:V";
#ifdef HAVE_GETOPT_LONG
    int option_index = 0;
    static struct option long_options[] = {
        {"ais", no_argument, NULL, 'a'},
        {"count", required_argument, NULL, 'c'},
        {"daemon", no_argument, NULL, 'b'},
        {"debug", required_argument, NULL, 'd'},
        {"help", no_argument, NULL, 'h'},
        {"json", no_argument, NULL, 'j'},
        {"nmea", no_argument, NULL, 'n'},
        {"udp", required_argument, NULL, 'u'},
        {"version", no_argument, NULL, 'V' },
        {NULL, 0, NULL, 0},
    };
#endif

    // pacify covarity
    memset(udphostport, 0, sizeof(udphostport));

    flags = WATCH_ENABLE;
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
        case 'a':
            aisonly = true;
            if (0 < debug)
                (void)fprintf(stdout, "AIS only selected\n");
            break;
        case 'b':
            daemonize = true;
            if (0 < debug)
                (void)fprintf(stdout, "Daemonize selected\n");
            break;
        case 'c':
            count = atol(optarg);
            if (0 < debug)
                (void)fprintf(stdout, "Count %ld selected\n", count);
            break;
        case 'd':
            debug = atoi(optarg);
            if (2 < debug) {
                usage();
                exit(1);
            }
            if (0 < debug)
                (void)fprintf(stdout, "Debug %u selected\n", debug);
            break;
        case 'j':
            if (0 < debug)
                (void)fprintf(stdout, "JSON selected\n");
            flags |= WATCH_JSON;
            break;
        case 'n':
            if (0 < debug) {
                (void)fprintf(stdout, "NMEA selected\n");
            }
            flags |= WATCH_NMEA;
            break;
        case 't':
            if (0 < debug) {
                (void)fprintf(stdout, "TPV and JSON selected\n");
            }
            flags |= WATCH_JSON;
            tpvonly = true;
            break;
        case 'u':
            if (udpchannel >= MAX_UDP_DEST) {
                (void)fprintf(stderr,
                              "gps2udp: too many UDP destinations (max=%d).\n",
                              MAX_UDP_DEST);
            } else {
                udphostport[udpchannel++] = optarg;
                if (0 < debug) {
                    (void)fprintf(stdout, "UDP %s added.\n", optarg);
                }
            }
            break;
        case '?':
        case 'h':
        default:
            usage();
            exit(1);
        case 'V':
            (void)fprintf(stderr, "%s: %s (revision %s)\n",
                          argv[0], VERSION, REVISION);
            exit(0);
        }
    }

    // Grok the server, port, and device.
    if (optind < argc) {
        gpsd_source_spec(argv[optind], &gpsd_source);
    } else {
        gpsd_source_spec(NULL, &gpsd_source);
    }
    if (NULL == gpsd_source.device) {
        if (0 < debug) {
            (void)fprintf(stdout, "gpsd source %s:%s\n",
            gpsd_source.server, gpsd_source.port);
        }
    } else {
        flags |= WATCH_DEVICE;
        if (0 < debug) {
            (void)fprintf(stdout, "gpsd source %s:%s:%s\n",
            gpsd_source.server, gpsd_source.port, gpsd_source.device);
        }
    }

    // check before going background if we can connect to gpsd
    connect2gpsd(false);

    // Open UDP port
    if (0 < udpchannel) {
        int status = open_udp(udphostport);

        if (0 != status) {
            exit(1);
        }
    }

    // Daemonize if the user requested it.
    if (daemonize) {
        if (0 != os_daemon(0, 0)) {
            (void)fprintf(stderr,
                          "gps2udp: daemonization failed: %s\n",
                          strerror(errno));
        }
    }

    // infinite loop to get data from gpsd and push them to aggregators
    for (;;) {
        char buffer[MAX_PACKET_LENGTH];
        ssize_t  len;

        len = read_gpsd(buffer, sizeof(buffer));

        // ignore empty message
        if (3 < len) {
            if (0 < debug) {
                (void)fprintf (stdout,"---> [%s] -- %s",time2string(),buffer);

                // Try to extract MMSI from AIS payload
                if (str_starts_with(buffer, "!AIVDM")) {
#define MAX_INFO 6
                    int  i, j;
                    char packet[MAX_PACKET_LENGTH];
                    char *adrpkt = packet;
                    unsigned char *info[MAX_INFO];
                    unsigned int  mmsi;
                    unsigned char bitstrings[255];
                    int info_5_len;

                    // pacify coverity.
                    memset(bitstrings, 0, sizeof(bitstrings));

                    // strtok break original string
                    (void)strlcpy(packet, buffer, sizeof(packet));
                    for (j = 0; j < MAX_INFO; j++) {
                        info[j] = (unsigned char *)strtok(adrpkt, ",");
                        // have strtok() continue from last position
                        adrpkt = NULL;
                    }

                    // codacy does not like strlen()
                    info_5_len = (int)strnlen((char *)info[5],
                                              sizeof(bitstrings));
                    if (((int)sizeof(bitstrings) - 1) <= info_5_len) {
                        info_5_len = (int)sizeof(bitstrings) - 1;
                    }
                    for(i = 0 ; i < info_5_len; i++)  {
                        bitstrings[i] = AISto6bit(info[5][i]);
                    }

                    mmsi = AISGetInt(bitstrings, 9, 30);
                    (void)fprintf(stdout," MMSI=%9u", mmsi);

                }
                (void)fprintf(stdout,"\n");
            }

            // send to all UDP destinations
            if (0 < udpchannel) {
                (void)send_udp(buffer, (size_t)len);
            }

            // if we count messages check it now
            if (0 <= count) {
                if (count-- == 0) {
                    // completed count
                    (void)fprintf(stderr,
                                  "gpsd2udp: normal exit after counted "
                                  "packets\n");
                    exit (0);
                }
            }  // end count
        } // end len > 3
    } // end for (;;)

    // This is an infinite loop, should never be here
    (void)fprintf (stderr, "gpsd2udp ERROR abnormal exit\n");
    exit (-1);
}

// vim: set expandtab shiftwidth=4
