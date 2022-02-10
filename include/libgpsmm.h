#ifndef _GPSD_GPSMM_H_
#define _GPSD_GPSMM_H_

/*
 * Copyright 2005 Alfredo Pironti
 * This file is Copyright 2005 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */
#include <sys/types.h>
#include "gps.h" //the C library we are going to wrap

#ifndef USE_QT
class gpsmm {
#else

#include <QtCore/qglobal.h>

#if defined(LIBQGPSMM_LIBRARY)
#  define LIBQGPSMMSHARED_EXPORT Q_DECL_EXPORT
#else
#  define LIBQGPSMMSHARED_EXPORT Q_DECL_IMPORT
#endif

class LIBQGPSMMSHARED_EXPORT gpsmm {
#endif
    public:
        // cppcheck-suppress uninitVar
        gpsmm(const char *host, const char *port) : to_user(0), _gps_state() {
                gps_inner_open(host, port);
        }
#ifdef __UNUSED__
        // cppcheck-suppress uninitVar
        gpsmm(void) : to_user(0) {
                gps_inner_open("localhost", DEFAULT_GPSD_PORT);
        }
#endif
        virtual ~gpsmm();
        // put a command to gpsd and return the updated struct
        struct gps_data_t* send(const char *request);
        struct gps_data_t* stream(int); //set watcher and policy flags
        // check for data from gpsd, then return the updated struct gps_data_t
        // non-blocking by default
        struct gps_data_t* read(void);
        const char *data(void); // return the client data buffer
        bool waiting(int);      // blocking check for data waiting
        void clear_fix(void);
        void enable_debug(int, FILE*);
        bool is_open(void);     // check for constructor success
    private:
        struct gps_data_t *to_user;
        /* we return the user a copy of the internal structure. This way
         * she can modify it without integrity loss for the entire class
         */
        struct gps_data_t* gps_inner_open(const char *host,
                                          const char *port);
        struct gps_data_t _gps_state;
        struct gps_data_t * gps_state() { return &_gps_state; }
        struct gps_data_t* backup(void) {
            if (NULL == to_user) { return to_user; }
             //return the backup copy
            *to_user=*gps_state();
            return to_user;
        };
};
#endif // _GPSD_GPSMM_H_
// vim: set expandtab shiftwidth=4
