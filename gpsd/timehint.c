/*
 * timehint.c - put time information in SHM segment for ntpd, or to chrony
 *
 * Note that for easy debugging all logging from this file is prefixed
 * with PPS or NTP.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"   // must be before all includes

#include <errno.h>
#include <libgen.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>        // for timespec
#include <unistd.h>

#include "../include/timespec.h"
#include "../include/gpsd.h"

#include "../include/ntpshm.h"

/* Note: you can start gpsd as non-root, and have it work with ntpd.
 * However, it will then only use the ntpshm segments 2 3, and higher.
 *
 * Ntpd always runs as root (to be able to control the system clock).
 * After that it often (depending on its host configuration) drops to run as
 * user ntpd and group ntpd.
 *
 * As of February 2015 its rules for the creation of ntpshm segments are:
 *
 * Segments 0 and 1: permissions 0600, i.e. other programs can only
 *                   read and write as root.
 *
 * Segments 2, 3, and higher:
 *                   permissions 0666, i.e. other programs can read
 *                   and write as any user.  I.e.: if ntpd has been
 *                   configured to use these segments, any
 *                   unprivileged user is allowed to provide data
 *                   for synchronisation.
 *
 * By default ntpd creates 0 segments (though the documentation is
 * written in such a way as to suggest it creates 4).  It can be
 * configured to create up to 217.  gpsd creates two segments for each
 * device it can drive; by default this is 8 segments for 4
 * devices,but can be higher if it was compiled with a larger value of
 * MAX_DEVICES.
 *
 * Started as root, gpsd does as ntpd when attaching (creating) the
 * segments.  In contrast to ntpd, which only attaches (creates)
 * configured segments, gpsd creates all segments.  Thus a gpsd will
 * by default create eight segments 0-7 that an ntpd with default
 * configuration does not watch.
 *
 * Started as non-root, gpsd will only attach (create) segments 2 and
 * above, with permissions 0666.  As the permissions are for any user,
 * the creator does not matter.
 *
 * For each GPS module gpsd controls, it will use the attached ntpshm
 * segments in pairs (for coarse clock and pps source, respectively)
 * starting from the first found segments.  I.e. started as root, one
 * GPS will deliver data on all segments including 0 and 1; started as
 * non-root, gpsd will be deliver data only on segments 2 and higher.
 *
 * Segments are allocated to activated devices on a first-come-first-served
 * basis. A device's segment is marked unused when the device is closed and
 * may be re-used by devices connected later.
 *
 * To debug, try looking at the live segments this way:
 *
 *  ipcs -m
 *
 * results  should look like this:
 * ------ Shared Memory Segments --------
 *  key        shmid      owner      perms      bytes      nattch     status
 *  0x4e545030 0          root       700        96         2
 *  0x4e545031 32769      root       700        96         2
 *  0x4e545032 163842     root       666        96         1
 *  0x4e545033 196611     root       666        96         1
 *  0x4e545034 253555     root       666        96         1
 *  0x4e545035 367311     root       666        96         1
 *
 * For a bit more data try this:
 *  cat /proc/sysvipc/shm
 *
 * If gpsd can not open the segments be sure you are not running SELinux
 * or apparmor.
 *
 * if you see the shared segments (keys 1314148400 -- 1314148405), and
 * no gpsd or ntpd is running, you can remove them like this:
 *
 * ipcrm  -M 0x4e545030
 * ipcrm  -M 0x4e545031
 * ipcrm  -M 0x4e545032
 * ipcrm  -M 0x4e545033
 * ipcrm  -M 0x4e545034
 * ipcrm  -M 0x4e545035
 *
 * Removing these segments is usually not necessary, as the operating system
 * garbage-collects them when they have no attached processes.
 */

static volatile struct shmTime *getShmTime(struct gps_context_t *context,
                                           int unit)
{
    int shmid;
    unsigned int perms;
    volatile struct shmTime *p;

    // set the SHM perms the way ntpd does
    if (2 > unit) {
        // we are root, be careful
        perms = 0600;
    } else {
        // we are not root, try to work anyway
        perms = 0666;
    }

    /*
     * Note: this call requires root under BSD, and possibly on
     * well-secured Linux systems.  This is why ntpshm_context_init() has to be
     * called before privilege-dropping.
     */
    shmid = shmget((key_t)(NTPD_BASE + unit),
                   sizeof(struct shmTime), (int)(IPC_CREAT | perms));
    if (shmid == -1) {
        GPSD_LOG(LOG_ERROR, &context->errout,
                 "NTP:SHM: shmget(NTP%d, %zd, %o) fail: %s(%d)\n",
                 unit, sizeof(struct shmTime),
                 (int)perms, strerror(errno), errno);
        return NULL;
    }
    p = (struct shmTime *)shmat(shmid, 0, 0);
    if ((int)(long)p == -1) {
        GPSD_LOG(LOG_ERROR, &context->errout,
                 "NTP:SHM: shmat failed,  unit %d: %s(%d)\n",
                 unit, strerror(errno), errno);
        return NULL;
    }
    GPSD_LOG(LOG_PROG, &context->errout,
             "NTP:SHM: shmat(%d,0,0) succeeded, unit %d\n",
             shmid, unit);
    return p;
}

// Attach all NTP SHM segments. Called once at startup, while still root.
void ntpshm_context_init(struct gps_context_t *context)
{
    int unit;

    // Only grab the first two when running as root.
    // then grab all the rest
    if (0 == getuid()) {
        unit  = 0;
    } else {
        unit  = 2;
    }
    for (; unit < NTPSHMSEGS; unit++) {
        context->shmTime[unit] = getShmTime(context, unit);
    }
    memset(context->shmTimeInuse, 0, sizeof(context->shmTimeInuse));
}

/* allocate NTP SHM segment
 * Return: Allocated unit
 *         -1 on failure
 */
static int ntpshm_alloc(struct gps_device_t *session)
{
    int unit;
    struct gps_context_t *context = session->context;

    // look at all possible SHM slots
    for (unit = 0; unit < NTPSHMSEGS; unit++) {
        // look for unused slot
        if (NULL != context->shmTime[unit] &&
            !context->shmTimeInuse[unit]) {
            context->shmTimeInuse[unit] = true;

            /*
             * In case this segment gets sent to ntpd before an
             * ephemeris is available, the LEAP_NOTINSYNC value will
             * tell ntpd that this source is in a "clock alarm" state
             * and should be ignored.  The goal is to prevent ntpd
             * from declaring the GPS a falseticker before it gets
             * all its marbles together.
             */
            memset((void *)context->shmTime[unit], 0, sizeof(struct shmTime));
            context->shmTime[unit]->mode = 1;
            context->shmTime[unit]->leap = LEAP_NOTINSYNC;
            context->shmTime[unit]->precision = -20;  // initially 1 micro sec
            context->shmTime[unit]->nsamples = 3;     // stages of median filter

            return unit;
        }
    }

    // no SHM free
    return -1;
}

/* free an NTP SHM segment
 *
 * Return: void
 */
static void ntpshm_free(struct gps_context_t * context, int unit)
{

    if (VALID_UNIT(unit)) {
        context->shmTimeInuse[unit] = false;
    }

    return;
}

void ntpshm_session_init(struct gps_device_t *session)
{
    // mark NTPD shared memory segments as unused
    session->shm_clock_unit = -1;
    session->shm_pps_unit = -1;
}

/* put a received fix time into shared memory for NTP
 *  unit is the SHM unit to use
 *  precision is the NTP precision
 *      Any NMEA will be about -1 or -2. Garmin GPS-18/USB can be -6 or -7
 *      PPS over USB, then precision = -10, 1 milli sec
 *      PPS over serial, precision = -20, 1 micro sec, maybe bettter
 *  td is the time delta to send
 *
 * Return: void
 */
void ntpshm_put(struct gps_device_t *session, int unit, int precision,
                struct timedelta_t *td)
{
    volatile struct shmTime *shmseg;
    char real_str[TIMESPEC_LEN];
    char clock_str[TIMESPEC_LEN];


    if (!VALID_UNIT(unit)) {
        GPSD_LOG(LOG_RAW, &session->context->errout,
                 "NTP:SHM:  ntpshm_put(,%d,) invalid unit\n", unit);
        return;
    }

    shmseg = session->context->shmTime[unit];
    ntp_write(shmseg, td, precision, session->context->leap_notify);

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "NTP:SHM: ntpshm_put(NTP%d, %d) %s, %s @ %s\n",
             unit, precision,
             session->gpsdata.dev.path,
             timespec_str(&td->real, real_str, sizeof(real_str)),
             timespec_str(&td->clock, clock_str, sizeof(clock_str)));

    return;
}

#define SOCK_MAGIC 0x534f434b
struct sock_sample {
    struct timeval tv;
    double offset;
    int pulse;
    int leap;       // notify that a leap second is upcoming
    int _pad;
    int magic;      // must be SOCK_MAGIC
};

// for chrony SOCK interface, which allows nSec timekeeping
static void init_hook(struct gps_device_t *session)
{
    // open the chrony socket
    char chrony_path[GPS_PATH_MAX];

    session->chronyfd = -1;
    if (0 == getuid()) {
        /* this case will fire on command-line devices;
         * they're opened before priv-dropping.  Matters because
         * usually only root can use /run or /var/run.
         */
        (void)snprintf(chrony_path, sizeof (chrony_path),
                       RUNDIR "/chrony.%s.sock",
                       basename(session->gpsdata.dev.path));
    } else {
        (void)snprintf(chrony_path, sizeof (chrony_path),
                       "/tmp/chrony.%s.sock",
                       basename(session->gpsdata.dev.path));
    }

    if (0 != access(chrony_path, F_OK)) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                "NTP:%s chrony socket %s doesn't exist\n",
                session->gpsdata.dev.path, chrony_path);
    } else {
        session->chronyfd = netlib_localsocket(chrony_path, SOCK_DGRAM);
        if (0 > session->chronyfd) {
            GPSD_LOG(LOG_PROG, &session->context->errout,
                     "NTP:%s connect chrony socket failed: %s, error: %d, "
                     "%s(%d)\n",
                     session->gpsdata.dev.path,
                     chrony_path, session->chronyfd, strerror(errno), errno);
        } else {
            GPSD_LOG(LOG_RAW, &session->context->errout,
                     "NTP:%s using chrony socket: %s\n",
                     session->gpsdata.dev.path, chrony_path);
        }
    }
}


/* td is the real time and clock time of the edge
 * offset is actual_ts - clock_ts
 */
static void chrony_send(struct gps_device_t *session, struct timedelta_t *td)
{
    char real_str[TIMESPEC_LEN];
    char clock_str[TIMESPEC_LEN];
    struct sock_sample sample;
    struct tm tm;
    int leap_notify = session->context->leap_notify;

    /*
     * insist that leap seconds only happen in june and december
     * GPS emits leap pending for 3 months prior to insertion
     * NTP expects leap pending for only 1 month prior to insertion
     * Per http://bugs.ntp.org/1090
     *
     * ITU-R TF.460-6, Section 2.1, says leap seconds can be primarily
     * in Jun/Dec but may be in March or September
     */
    (void)gmtime_r( &(td->real.tv_sec), &tm);
    if (5 != tm.tm_mon &&
        11 != tm.tm_mon) {
        // Not june, not December, no way
        leap_notify = LEAP_NOWARNING;
    }


    // chrony expects tv-sec since Jan 1970
    sample.pulse = 0;
    sample.leap = leap_notify;
    sample.magic = SOCK_MAGIC;
    /* chronyd wants a timeval, not a timspec, not to worry, it is
     * just the top of the second */
    TSTOTV(&sample.tv, &td->clock);
    /* calculate the offset as a timespec to not lose precision */
    /* if tv_sec greater than 2 then tv_nsec loses precision, but
     * not a big deal as slewing will be required */
    sample.offset = TS_SUB_D(&td->real, &td->clock);
    sample._pad = 0;

    GPSD_LOG(LOG_RAW, &session->context->errout,
             "NTP: chrony_send %s @ %s Offset: %0.9f\n",
             timespec_str(&td->real, real_str, sizeof(real_str)),
             timespec_str(&td->clock, clock_str, sizeof(clock_str)),
             sample.offset);
    (void)send(session->chronyfd, &sample, sizeof (sample), 0);
}

// ship the time of a PPS event to ntpd and/or chrony
static char *report_hook(volatile struct pps_thread_t *pps_thread,
                         struct timedelta_t *td)
{
    char *log1;
    struct gps_device_t *session = (struct gps_device_t *)pps_thread->context;
    int precision;

    /* PPS only source never get any serial info
     * so no NTPTIME_IS or fixcnt */
    if (SOURCE_PPS != session->sourcetype) {
        // FIXME! these two validations need to move back into ppsthread.c

        if (!session->ship_to_ntpd) {
            return "skipped ship_to_ntp=0";
        }

        /*
         * Only listen to PPS after several consecutive fixes,
         * otherwise time may be inaccurate.  We know this is
         * required on all Garmin and u-blox.  Safest to do it
         * for all cases as we have no other general way to know
         * if PPS is good.
         * Allow override with batteryRTC to allow foot shots.
         */
        if (false == session->context->batteryRTC &&
            NTP_MIN_FIXES >= session->fixcnt &&
            0 == (session->gpsdata.set & GOODTIME_IS)) {
            return "no fix";
        }
    }

    // FIXME?  how to log socket AND shm reported?
    log1 = "accepted";
    if (0 <= session->chronyfd) {
        log1 = "accepted chrony sock";
        chrony_send(session, td);
    }

    // precision is a floor so do not make it tight
    if (SOURCE_USB == session->sourcetype ||
        SOURCE_ACM == session->sourcetype) {
        // if PPS over USB, then precision = -10, 1 milli sec
        precision = -10;
    } else {
        // likely PPS over serial, precision = -20, 1 micro sec
        precision = -20;
    }

    if (VALID_UNIT(session->shm_pps_unit)) {
        ntpshm_put(session, session->shm_pps_unit, precision, td);
    }

    // session context might have a hook set, too
    if (NULL != session->context->pps_hook) {
        session->context->pps_hook(session, session->shm_pps_unit,
                                   precision, td);
    }

    return log1;
}

// release ntpshm storage for a session
void ntpshm_link_deactivate(struct gps_device_t *session)
{
    if (VALID_UNIT(session->shm_clock_unit)) {
        ntpshm_free(session->context, session->shm_clock_unit);
        session->shm_clock_unit = -1;
    }
    if (VALID_UNIT(session->shm_pps_unit)) {
        pps_thread_deactivate(&session->pps_thread);
        if (-1 != session->chronyfd) {
            // how do we know chronyfd is related to this shm_pps_unit?
            (void)close(session->chronyfd);
        }
        ntpshm_free(session->context, session->shm_pps_unit);
        session->shm_pps_unit = -1;
    }
}

// set up ntpshm storage for a session
void ntpshm_link_activate(struct gps_device_t *session)
{
    struct gps_context_t *context = session->context;

    GPSD_LOG(LOG_PROG, &context->errout,
             "NTP:SHM: ntpshm_link_activate(%s), sourcetype %d fd %d\n",
             session->gpsdata.dev.path, session->sourcetype,
             session->gpsdata.gps_fd);

    /* don't talk to NTP when we're:
     *   reading from a file
     *   reading from a pipe
     *   reading from a remote gpsd
     *   running inside the test harness (PTY)
     *   over TCP or UDP
     */
    if (SOURCE_BLOCKDEV == session->sourcetype ||
        SOURCE_GPSD == session->sourcetype ||
        SOURCE_PIPE == session->sourcetype ||
        SOURCE_PTY == session->sourcetype ||
        SOURCE_TCP == session->sourcetype ||
        SOURCE_UDP == session->sourcetype) {
        return;
    }

    if (SOURCE_PPS != session->sourcetype) {
        // allocate a shared-memory segment for "NMEA" time data
        session->shm_clock_unit = ntpshm_alloc(session);

        if (!VALID_UNIT(session->shm_clock_unit)) {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NTP:SHM: ntpshm_alloc(shm_clock) failed\n");
            return;
        }
        GPSD_LOG(LOG_PROG, &context->errout,
                 "NTP:SHM: ntpshm_alloc(%s), sourcetype %d "
                 "shm_clock using SHM(%d)\n",
                 session->gpsdata.dev.path, session->sourcetype,
                 session->shm_clock_unit);
    }

    if (SOURCE_USB == session->sourcetype ||
        SOURCE_ACM == session->sourcetype ||
        SOURCE_RS232 == session->sourcetype ||
        SOURCE_PPS == session->sourcetype) {
        /* We also have the 1pps capability, allocate a shared-memory segment
         * for the 1pps time data and launch a thread to capture the 1pps
         * transitions
         */
        session->shm_pps_unit = ntpshm_alloc(session);
        if (VALID_UNIT(session->shm_pps_unit)) {
            GPSD_LOG(LOG_PROG, &context->errout,
                     "NTP:SHM: ntpshm_alloc(%s), sourcetype %d "
                     "shm_pps using SHM(%d)\n",
                     session->gpsdata.dev.path, session->sourcetype,
                     session->shm_pps_unit);
            init_hook(session);
            session->pps_thread.report_hook = report_hook;
#ifdef MAGIC_HAT_ENABLE
            /*
             * The HAT kludge. If we're using the HAT GPS on a
             * Raspberry Pi or a workalike like the ODROIDC2, and
             * there is a static "first PPS", and we have access because
             * we're root, assume we want to use KPPS.
             */
            if (0 == strcmp(session->pps_thread.devicename, MAGIC_HAT_GPS) ||
                0 == strcmp(session->pps_thread.devicename, MAGIC_LINK_GPS)) {
                const char *first_pps = pps_get_first();
                if (0 == access(first_pps, R_OK | W_OK)) {
                    session->pps_thread.devicename = first_pps;
                    GPSD_LOG(LOG_PROG, &context->errout,
                             "NTP:SHM: ntpshm_link_activate() MAGIC_HAT "
                             "using %s for SHM(%d)\n", first_pps,
                             session->shm_pps_unit);
                } else {
                    GPSD_LOG(LOG_ERROR, &context->errout,
                             "NTP:SHM: ntpshm_link_activate() unable to "
                             "read %s. %s(%d)\n",
                             first_pps, strerror(errno), errno);
                }
            }
#endif  // MAGIC_HAT_ENABLE
            pps_thread_activate(&session->pps_thread);
        } else {
            GPSD_LOG(LOG_WARN, &session->context->errout,
                     "NTP:SHM: ntpshm_alloc(shm_pps) failed\n");
        }
    }
}

// vim: set expandtab shiftwidth=4
