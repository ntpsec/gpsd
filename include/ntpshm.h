/*
 * This file is Copyright 2015 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#ifndef GPSD_NTPSHM_H
#define GPSD_NTPSHM_H

#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define NTPD_BASE       0x4e545030      // "NTP0"

/*
 * How to read and write fields in an NTP shared segment.
 * This definition of shmTime is from ntpd source ntpd/refclock_shm.c
 *
 * The fields aren't documented there.  It appears the only use of
 * nsamples is internal to the (obsolete and deprecated) EES M201
 * receiver refclock. The precision field is nominally log(2) of the
 * source's jitter in seconds:
 *      -1 is about 100mSec jitter
 *      -10 is about 1 mSec jitter (GR-601W or other USB with 1ms poll interval)
 *      -13 is about 100 uSec
 *      -20 is about 1 uSec (typical for serial PPS)
 */

struct shmTime
{
    int mode;   /* 0 - if valid set
                 *       use values,
                 *       clear valid
                 * 1 - if valid set
                 *       if count before and after read of values is equal,
                 *         use values
                 *       clear valid
                 */
    volatile int count;
#if defined(_TIME_BITS) && 64 == _TIME_BITS
    /* libc default time_t is 32-bits, but we are using 64-bits.
     * glibc 2.34 and later.
     * Lower 31 bits, no sign bit, are here. */
    unsigned clockTimeStampSec;
#else
    time_t clockTimeStampSec;
#endif
    int clockTimeStampUSec;
#if defined(_TIME_BITS) && 64 == _TIME_BITS
    /* libc default time_t is 32-bits, but we are using 64-bits.
     * glibc 2.34 and later.
     * Lower 31 bits, no sign bit, are here. */
    unsigned receiveTimeStampSec;
#else
    time_t receiveTimeStampSec;
#endif
    int receiveTimeStampUSec;
    int leap;                   // not leapsecond offset, a notification code
    int precision;              // log(2) of source jitter
    int nsamples;               // not used
    volatile int valid;
    unsigned        clockTimeStampNSec;     // Unsigned ns timestamps
    unsigned        receiveTimeStampNSec;   // Unsigned ns timestamps
    /* Change previous dummy[0,1] to hold top bits.
     * Zero until 2038.  */
    unsigned top_clockTimeStampSec;
    unsigned top_receiveTimeStampSec;
    int             dummy[6];
};


/*
 * These types are internal to GPSD
 */
enum segstat_t {OK, NO_SEGMENT, NOT_READY, BAD_MODE, CLASH};

struct shm_stat_t {
    enum segstat_t status;
    struct timespec tvc;    // System time when SHM read, for debug only
    struct timespec tvr;    // System time at GPS time
    struct timespec tvt;    // GPS time
    int precision;
    int leap;
};

#ifndef TIMEDELTA_DEFINED

struct timedelta_t {
    struct timespec     real;
    struct timespec     clock;
};

#define TIMEDELTA_DEFINED
#endif  // TIMEDELTA_DEFINED

struct shmTime *shm_get(int, bool, bool);
extern char *ntp_name(const int);
enum segstat_t ntp_read(struct shmTime *, struct shm_stat_t *, const bool);
void ntp_write(volatile struct shmTime *, struct timedelta_t *, int, int);

#endif  // GPSD_NTPSHM_H

// vim: set expandtab shiftwidth=4
