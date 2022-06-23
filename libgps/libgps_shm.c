/****************************************************************************

NAME
   libgps_shm.c - reader access to shared-memory export

DESCRIPTION
   This is a very lightweight alternative to JSON-over-sockets.  Clients
won't be able to filter by device, and won't get device activation/deactivation
notifications.  But both client and daemon will avoid all the marshalling and
unmarshalling overhead.

PERMISSIONS
   This file is Copyright 2010 by the GPSD project
   SPDX-License-Identifier: BSD-2-clause

***************************************************************************/

#include "../include/gpsd_config.h"

#ifdef SHM_EXPORT_ENABLE

#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/time.h>

#include "../include/gpsd.h"
#include "../include/libgps.h"


/* open a shared-memory connection to the daemon
 *
 * Return: 0 == OK
 *        less than zero is an error
 *         -1 shmget() error
 *         -2 shmat() error
 *         -3 calloc() error
 */
int gps_shm_open(struct gps_data_t *gpsdata)
{
    int shmid;

    long shmkey = getenv("GPSD_SHM_KEY") ?
                     strtol(getenv("GPSD_SHM_KEY"), NULL, 0) : GPSD_SHM_KEY;

    libgps_debug_trace((DEBUG_CALLS, "gps_shm_open()\n"));

    gpsdata->privdata = NULL;
    shmid = shmget((key_t)shmkey, sizeof(struct gps_data_t), 0);
    if (-1 == shmid) {
        // daemon isn't running or failed to create shared segment
        libgps_debug_trace((DEBUG_CALLS, "gps_shm_open(x%lx) %s(%d)\n",
                            (unsigned long)shmkey, strerror(errno),  errno));
        return -1;
    }
    gpsdata->privdata =
        (struct privdata_t *)calloc(1, sizeof(struct privdata_t));
    if (NULL == gpsdata->privdata) {
        libgps_debug_trace((DEBUG_CALLS, "calloc() %s(%d)\n",
                            strerror(errno),  errno));
        return -3;
    }

    PRIVATE(gpsdata)->shmseg = shmat(shmid, 0, 0);
    if ((void *)-1 == PRIVATE(gpsdata)->shmseg) {
        // attach failed for sume unknown reason
        libgps_debug_trace((DEBUG_CALLS, "shmat() %s(%d)\n",
                            strerror(errno),  errno));
        free(gpsdata->privdata);
        gpsdata->privdata = NULL;
        return -2;
    }
#ifndef USE_QT
    gpsdata->gps_fd = SHM_PSEUDO_FD;
#else
    gpsdata->gps_fd = (void *)(intptr_t)SHM_PSEUDO_FD;
#endif  // USE_QT
    return 0;
}

/* check to see if new data has been written
 * timeout is in uSec */
bool gps_shm_waiting(const struct gps_data_t *gpsdata, int timeout)
{
    volatile struct shmexport_t *shared =
            (struct shmexport_t *)PRIVATE(gpsdata)->shmseg;
    volatile bool newdata = false;
    timespec_t endtime;

    (void)clock_gettime(CLOCK_REALTIME, &endtime);
    endtime.tv_sec += timeout / 1000000;
    endtime.tv_nsec += (timeout % 1000000) * 1000;
    TS_NORM(&endtime);

    // busy-waiting sucks, but there's not really an alternative
    for (;;) {
        volatile int bookend1, bookend2;
        timespec_t now;

        memory_barrier();
        bookend1 = shared->bookend1;
        memory_barrier();
        bookend2 = shared->bookend2;
        memory_barrier();
        if (bookend1 == bookend2 && bookend1 > PRIVATE(gpsdata)->tick) {
            newdata = true;
            break;
        }
        (void)clock_gettime(CLOCK_REALTIME, &now);
        if (TS_GT(&now, &endtime)) {
            break;
        }
    }

    return newdata;
}

// read an update from the shared-memory segment
int gps_shm_read(struct gps_data_t *gpsdata)
{
    if (NULL == gpsdata->privdata) {
        return -1;
    } else {
        int before1, before2, after1, after2;
        struct privdata_t *private_save = gpsdata->privdata;
        struct shmexport_t *shared =
            (struct shmexport_t *)PRIVATE(gpsdata)->shmseg;
        struct gps_data_t noclobber;

        /*
         * Following block of instructions must not be reordered,
         * otherwise havoc will ensue.  The memory_barrier() call
         * should prevent reordering of the data accesses.
         * for those lucky enough to have a working memory_barrier()
         *
         * bookends are volatile, so that should force
         * them to be read in order.
         *
         * This is a simple optimistic-concurrency technique.  We wrote
         * the second bookend first, then the data, then the first bookend.
         * Reader copies what it sees in normal order; that way, if we
         * start to write the segment during the read, the second bookend will
         * get clobbered first and the data can be detected as bad.
         *
         * Excwpt with mutil-treading and CPU caches, order is iffy...
         */
        before1 = shared->bookend1;
        before2 = shared->bookend2;
        memory_barrier();
        // memcpy() and (volatile) don't play well together.
        (void)memcpy((void *)&noclobber,
                     (void *)&shared->gpsdata,
                     sizeof(struct gps_data_t));
        memory_barrier();
        after1 = shared->bookend1;
        after2 = shared->bookend2;

        if (before1 != after1 ||
            before1 != after2 ||
            before1 != before2) {
            // bookend mismatch, throw away the data
            // FIXME: retry?
            return 0;
        } else {
            (void)memcpy((void *)gpsdata,
                         (void *)&noclobber,
                         sizeof(struct gps_data_t));
            gpsdata->privdata = private_save;
#ifndef USE_QT
            gpsdata->gps_fd = SHM_PSEUDO_FD;
#else
            gpsdata->gps_fd = (void *)(intptr_t)SHM_PSEUDO_FD;
#endif  // USE_QT
            PRIVATE(gpsdata)->tick = after2;
            if (0 != (gpsdata->set & REPORT_IS)) {
                gpsdata->set = STATUS_SET;
            }
            return (int)sizeof(struct gps_data_t);
        }
    }
}

void gps_shm_close(struct gps_data_t *gpsdata)
{
    if (PRIVATE(gpsdata)) {
        if (NULL != PRIVATE(gpsdata)->shmseg) {
            (void)shmdt((const void *)PRIVATE(gpsdata)->shmseg);
        }
        free(PRIVATE(gpsdata));
        gpsdata->privdata = NULL;
    }
}

/* run a shm main loop with a specified handler
 *
 * Returns: -1 on timeout
 *          -2 on error
 * FIXME: read error should return different than timeout
 */
int gps_shm_mainloop(struct gps_data_t *gpsdata, int timeout,
                     void (*hook)(struct gps_data_t *gpsdata))
{

    for (;;) {
        int status;

        if (!gps_shm_waiting(gpsdata, timeout)) {
            return -1;
        }
        status = gps_shm_read(gpsdata);

        if (-1 == status) {
            break;
        }
        if (0 < status) {
            (*hook)(gpsdata);
        }
    }
    return -2;
}

#endif  // SHM_EXPORT_ENABLE

// vim: set expandtab shiftwidth=4
