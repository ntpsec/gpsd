/****************************************************************************

NAME
   shmexport.c - shared-memory export from the daemon

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
#include "../include/libgps.h"    // for SHM_PSEUDO_FD

/* initialize the shared-memory segment to be used for export
 *
 * Return: true = OK
 *         false: failed
 */
bool shm_acquire(struct gps_context_t *context)
{
    long shmkey = getenv("GPSD_SHM_KEY") ? \
                      strtol(getenv("GPSD_SHM_KEY"), NULL, 0) : GPSD_SHM_KEY;

    int shmid = shmget((key_t)shmkey, sizeof(struct shmexport_t),
                       (int)(IPC_CREAT|0666));
    context->shmid = shmid;
    if (-1 == shmid) {
        GPSD_LOG(LOG_ERROR, &context->errout,
                 "SHM: shmget(0x%lx, %zd, 0666) SHM export failed: %s(%d)\n",
                 shmkey,
                 sizeof(struct shmexport_t),
                 strerror(errno), errno);
        return false;
    }

    GPSD_LOG(LOG_PROG, &context->errout,
             "SHM: shmget(0x%lx, %zd, 0666) for SHM export succeeded\n",
             shmkey,
             sizeof(struct shmexport_t));

    context->shmexport = (void *)shmat(shmid, 0, 0);
    if ((void *)-1 == context->shmexport) {
        GPSD_LOG(LOG_ERROR, &context->errout,
                 "SHM: shmat failed: %s(%d)\n", strerror(errno), errno);
        context->shmexport = NULL;
        // close shmid
        shm_release(context);
        return false;
    }

    // mark SHM to be destroeyd after last user is gone
    // do it now while we are still uid of the owner/creator
    if (-1 == shmctl(shmid, IPC_RMID, NULL)) {
        GPSD_LOG(LOG_WARN, &context->errout,
                 "SHM: shmctl(%d) for IPC_RMID failed, %s(%d)\n",
                 context->shmid, strerror(errno), errno);
    }

    GPSD_LOG(LOG_PROG, &context->errout,
             "SHM: shmat() for SHM export succeeded, segment %d\n", shmid);
    return true;
}

// release the shared-memory segment used for export
void shm_release(struct gps_context_t *context)
{

    /* Mark shmid to go away when no longer used
     * Having it linger forever is bad, and when the size enlarges
     * it can no longer be opened
     */

    /* debug
    * GPSD_LOG(LOG_SHOUT, &context->errout,
    *          "SHM: shm_release() shmid %d shmexport %p\n",
    *          context->shmid, context->shmexport);
    */

    // detach from segment, it we were last user, it should be deleted.
    if (NULL != context->shmexport) {
        if (-1 == shmdt((const void *)context->shmexport)) {
            GPSD_LOG(LOG_WARN, &context->errout,
                     "SHM: shmdt() for shmid %d failed: %s(%d)\n",
                     context->shmid, strerror(errno), errno);
        }
    }

    context->shmid = -1;

}

// export an update to all listeners
void shm_update(struct gps_context_t *context, struct gps_data_t *gpsdata)
{
    if (NULL != context->shmexport) {
        static int tick;
        volatile struct shmexport_t *shared = \
                            (struct shmexport_t *)context->shmexport;

        ++tick;
        /*
         * Following block of instructions must not be reordered, otherwise
         * havoc will ensue.
         *
         * This is a simple optimistic-concurrency technique.  We write
         * the second bookend first, then the data, then the first bookend.
         * Reader copies what it sees in normal order; that way, if we
         * start to write the segment during the read, the second bookend will
         * get clobbered first and the data can be detected as bad.
         *
         * Of course many architectures, like Intel, make no guarantees
         * about the actual memory read or write order into RAM, so this
         * is partly wishful thinking.  Thus the need for the memory_barriers()
         * to enforce the required order.
         */
        shared->bookend2 = tick;
        memory_barrier();
        shared->gpsdata = *gpsdata;
        memory_barrier();
#ifndef USE_QT
        shared->gpsdata.gps_fd = SHM_PSEUDO_FD;
#else
        shared->gpsdata.gps_fd = (void *)(intptr_t)SHM_PSEUDO_FD;
#endif  // USE_QT
        memory_barrier();
        shared->bookend1 = tick;
    }
}


#endif  // SHM_EXPORT_ENABLE

// vim: set expandtab shiftwidth=4
