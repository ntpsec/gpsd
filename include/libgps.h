/* libgps.h -- prototypes for internals of the libgps library
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#ifndef _GPSD_LIBGPS_H_
#define _GPSD_LIBGPS_H_

// values to poke in the gps_fd member if we get it via something special
#define SHM_PSEUDO_FD   -1
#define DBUS_PSEUDO_FD  -2

#include "gps.h"
#include "gpsd_config.h"
#include "compiler.h"

#ifdef __cplusplus
extern "C" {
#endif
extern int gps_sock_open(const char *, const char *, struct gps_data_t *);
extern int gps_sock_close(struct gps_data_t *);
extern int gps_sock_send(struct gps_data_t *, const char *);
extern int gps_sock_read(struct gps_data_t *, char *message, int message_len);
extern bool gps_sock_waiting(const struct gps_data_t *, int);
extern int gps_sock_stream(struct gps_data_t *, unsigned int, const char *);
extern const char *gps_sock_data(const struct gps_data_t *);
extern int gps_sock_mainloop(struct gps_data_t *, int,
                             void (*)(struct gps_data_t *));
extern int gps_shm_open(struct gps_data_t *);
extern void gps_shm_close(struct gps_data_t *);
extern bool gps_shm_waiting(const struct gps_data_t *, int);
extern int gps_shm_read(struct gps_data_t *);
extern int gps_shm_mainloop(struct gps_data_t *, int,
                            void (*)(struct gps_data_t *));
extern int gps_dbus_open(struct gps_data_t *);
extern int gps_dbus_mainloop(struct gps_data_t *, int,
                             void (*)(struct gps_data_t *));

extern int json_ais_read(const char *, char *, size_t, struct ais_t *,
                         const char **);

// debugging apparatus for the client library
extern FILE *debugfp;

#define DEBUG_CALLS     1       // shallowest debug level
#define DEBUG_JSON      5       // minimum level for verbose JSON debugging

// HOTCODE!  Do not change without profiling.
// major speedup by checking debuglvl before callin json_trace()
#define libgps_debug_trace(lvl, fmt, ...)             \
    do {                                              \
        if (unlikely((lvl) <= libgps_debuglevel &&    \
                      NULL != debugfp) ) {            \
            libgps_trace(fmt, __VA_ARGS__);           \
        }                                             \
    } while (0)

extern int libgps_debuglevel;
extern void libgps_dump_state(struct gps_data_t *);

#ifdef __cplusplus
}
#endif

#define PRIVATE(gpsdata) ((gpsdata)->privdata)

#endif  // _GPSD_LIBGPS_H_
// vim: set expandtab shiftwidth=4
