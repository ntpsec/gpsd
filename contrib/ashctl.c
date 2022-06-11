/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>  /* For nanosleep() */
#include <unistd.h>

#define MODE_RAW 0
#define MODE_NORMAL 1
#define ASHSPD_9600 5
#define ASHSPD_57600 8

static int nmea_send(int , const char *, ... );
static void nmea_add_checksum(char *);
static void serial_speed(int, int);
static void config_raw(int);
static void config_normal(int);

int main(int argc, char **argv) {
    int i, speed, op, fd;
    int rates[] = {57600, 9600, 115200, 4800, 19200, 1200, 0};  // RTFM
    char buf[BUFSIZ];

    // check number of args
    if (3 != argc){
u:      fprintf(stderr, "usage: ashctl <port> [raw|normal]\n"
                "normal = 9600, GGA+GSA+GSV+RMC+ZDA\n"
                "raw = 57600, normal+XPG+POS+SAT+MCA+PBN+SNV\n");
        return 1;
    }

    // validate command
    if (0 == strcmp(argv[2], "raw")) {
        op = MODE_RAW;
    } else if (0 == strcmp(argv[2], "normal")) {
        op = MODE_NORMAL;
    } else {
        goto u;
    }

    fd = open(argv[1], O_RDWR | O_NONBLOCK | O_NOCTTY, 0644);
    if (-1 == fd) {
        err(1, "open");
    }

    i = 0;
    // spam receiver w/ config messages
    while((speed = rates[i++])){
        ssize_t read_ret;

        fprintf(stderr,
                "\010\010\010\010\010\010\010\010"
                "\010\010\010\010\010\010\010\010"
                "\010\010\010\010\010\010\010\010"
                "\010\010\010\010\010\010\010\010"
                "configuring at %d bps...  ", speed);
        serial_speed(fd, speed);
        if (MODE_NORMAL == op) {
            config_normal(fd);
            serial_speed(fd, 9600);
        } else if (MODE_RAW == op) {
            config_raw(fd);
            serial_speed(fd, 57600);
        }

        sleep(1);
        // Flawfinder: ignore
        read_ret = read(fd, buf, BUFSIZ-1);
        if (0 > read_ret) {
            fprintf(stderr, "ERROR: read() failed %d\n", errno);
            return -1;
        }
        buf[read_ret] = '\0';
        if (strstr(buf, "$PASH") ||
            strstr(buf, "$GP")) {
            break;
        }
    }
    fprintf(stderr,
            "\010\010\010\010\010\010\010\010"
            "\010\010\010\010\010\010\010\010"
            "\010\010\010\010\010\010\010\010"
            "\010\010\010\010\010\010\010\010"
            "receiver configuration done         \n");
    return 0;
}

static void serial_speed(int fd, int speed){
    struct termios term;

    tcgetattr(fd, &term);
    cfmakeraw(&term);
    cfsetospeed(&term, speed);
    cfsetispeed(&term, speed);
    if (-1 == tcsetattr(fd, TCSANOW | TCSAFLUSH, &term)) {
        err(1, "tcsetattr");
    }

    tcflush(fd, TCIOFLUSH);
    return;
}

static void config_normal(int fd){
    nmea_send(fd, "$PASHS,NME,ALL,A,OFF");  // silence outbound chatter
    nmea_send(fd, "$PASHS,NME,ALL,B,OFF");
    nmea_send(fd, "$PASHS,NME,GGA,A,ON");
    nmea_send(fd, "$PASHS,NME,GSA,A,ON");
    nmea_send(fd, "$PASHS,NME,GSV,A,ON");
    nmea_send(fd, "$PASHS,NME,RMC,A,ON");
    nmea_send(fd, "$PASHS,NME,ZDA,A,ON");

    nmea_send(fd, "$PASHS,INI,%d,%d,,,0,",ASHSPD_9600, ASHSPD_9600);
    sleep(6);   // it takes 4-6 sec for the receiver to reboot
    nmea_send(fd, "$PASHS,WAS,ON");   // enable WAAS
}

static void config_raw(int fd)
{
    nmea_send(fd, "$PASHS,NME,ALL,A,OFF");  // silence outbound chatter
    nmea_send(fd, "$PASHS,NME,ALL,B,OFF");
    nmea_send(fd, "$PASHS,NME,GGA,A,ON");
    nmea_send(fd, "$PASHS,NME,GSA,A,ON");
    nmea_send(fd, "$PASHS,NME,GSV,A,ON");
    nmea_send(fd, "$PASHS,NME,RMC,A,ON");
    nmea_send(fd, "$PASHS,NME,ZDA,A,ON");

    nmea_send(fd, "$PASHS,INI,%d,%d,,,0,",ASHSPD_57600, ASHSPD_9600);
    sleep(6);     // it takes 4-6 sec for the receiver to reboot
    nmea_send(fd, "$PASHS,WAS,ON");           // enable WAAS

    nmea_send(fd, "$PASHS,NME,POS,A,ON");     // Ashtech PVT solution
    nmea_send(fd, "$PASHS,NME,SAT,A,ON");     // Ashtech Satellite status
    nmea_send(fd, "$PASHS,NME,MCA,A,ON");     // MCA measurements
    nmea_send(fd, "$PASHS,NME,PBN,A,ON");     // ECEF PVT solution
    nmea_send(fd, "$PASHS,NME,SNV,A,ON,10");  // Almanac data

    nmea_send(fd, "$PASHS,NME,XMG,A,ON");     // exception messages
}

// add NMEA checksum to a possibly  *-terminated sentence
static void nmea_add_checksum(char *sentence)
{
    char *p = sentence;

    if ('$' == *p) {
        unsigned char sum = '\0';
        char c;

        p++;
        while (((c = *p) != '*') &&
                (c != '\0')) {
            sum ^= c;
            p++;
        }
        *p++ = '*';
        (void)snprintf(p, 5, "%02X\r\n", (unsigned int)sum);
    }
}

// ship a command to the GPS, adding * and correct checksum
static int nmea_send(int fd, const char *fmt, ... )
{
    size_t status;
    char buf[BUFSIZ];
    va_list ap;
    struct timespec delay;

    va_start(ap, fmt) ;
    // leave room in buf for checksum
    (void)vsnprintf(buf, sizeof(buf) - 6, fmt, ap);
    va_end(ap);
    (void)strncat(buf, "*", sizeof(buf) - 2);
    buf[BUFSIZ - 1] = '\0';      // strncat paranoia

    nmea_add_checksum(buf);
    // (void)fputs(buf, stderr);  // debug output
    tcflush(fd, TCIOFLUSH);
    status = (size_t)write(fd, buf, strnlen(buf, BUFSIZ));
    tcdrain(fd);
    // wait 100,000 uSec
    delay.tv_sec = 0;
    delay.tv_nsec = 100000000L;
    nanosleep(&delay, NULL);

    if (strnlen(buf, BUFSIZ) == status) {
        return (int)status;
    } else {
        perror("nmea_send");
        return -1;
    }
}
