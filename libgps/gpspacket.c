/*
 * Foreign function interface binding for the packet module.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "../include/gpsd_config.h" /* must be before all includes */

#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include "../include/gpsd.h"
#include "../include/strfuncs.h"

#define LOG_SHOUT 0

struct gps_lexer_t *ffi_Lexer_init(void);   // For FFI Python interface.
void gpsd_vlog(const struct gpsd_errout_t*, const int, char*,
               size_t, const char*, va_list);

// FIXME: this duplicates visibilize()
static void visibilize1(char *outbuf, size_t outlen, const char *inbuf,
                       size_t inlen) {
  const char *sp;

  outbuf[0] = '\0';
  for (sp = inbuf; sp < inbuf + inlen && strlen(outbuf) + 6 < outlen; sp++)
    if (isprint((unsigned char)*sp) || (sp[0] == '\n' && sp[1] == '\0') ||
        (sp[0] == '\r' && sp[2] == '\0'))
      (void)snprintf(outbuf + strlen(outbuf), 2, "%c", *sp);
    else
      (void)snprintf(outbuf + strlen(outbuf), 6, "\\x%02x",
                     0x00ff & (unsigned)*sp);
}

/* assemble msg in vprintf(3) style, use errout hook or syslog for delivery */
void gpsd_vlog(const struct gpsd_errout_t *errout, const int errlevel,
               char *outbuf, size_t outlen, const char *fmt, va_list ap)
{
  if (errout->debug >= errlevel) {
    char buf[BUFSIZ];
    char *err_str;

    switch (errlevel) {
    case LOG_ERROR:
      err_str = "ERROR: ";
      break;
    case LOG_SHOUT:
      err_str = "SHOUT: ";
      break;
    case LOG_WARN:
      err_str = "WARN: ";
      break;
    case LOG_CLIENT:
      err_str = "CLIENT: ";
      break;
    case LOG_INF:
      err_str = "INFO: ";
      break;
    case LOG_DATA:
      err_str = "DATA: ";
      break;
    case LOG_PROG:
      err_str = "PROG: ";
      break;
    case LOG_IO:
      err_str = "IO: ";
      break;
    case LOG_SPIN:
      err_str = "SPIN: ";
      break;
    case LOG_RAW:
      err_str = "RAW: ";
      break;
    default:
      err_str = "UNK: ";
    }

    assert(errout->label != NULL);
    (void)strlcpy(buf, errout->label, sizeof(buf));
    (void)strlcat(buf, ":", sizeof(buf));
    (void)strlcat(buf, err_str, sizeof(buf));
    str_vappendf(buf, sizeof(buf), fmt, ap);

    visibilize1(outbuf, outlen, buf, strlen(buf));

    if (getpid() == getsid(getpid()))
      syslog((errlevel <= LOG_SHOUT) ? LOG_ERR : LOG_NOTICE, "%s", outbuf);
    else if (errout->report != NULL)
      errout->report(outbuf);
    else
      (void)fputs(outbuf, stderr);
  }
}

/* assemble msg in printf(3) style, use errout hook or syslog for delivery */
void gpsd_log(const int errlevel, const struct gpsd_errout_t *errout,
              const char *fmt, ...)
{
  char buf[BUFSIZ];
  va_list ap;

  // cppcheck wants is to check for NULL == errout.  Should never happen?
  if (NULL == errout || errout->debug < errlevel) {
    /* nothing to do, get out */
    return;
  }

  buf[0] = '\0';
  va_start(ap, fmt);
  gpsd_vlog(errout, errlevel, buf, sizeof(buf), fmt, ap);
  va_end(ap);
}

static void basic_report(const char *buf) {
  (void)fputs(buf, stderr);
}

void errout_reset(struct gpsd_errout_t *errout) {
  errout->debug = LOG_SHOUT;
  errout->report = basic_report;
}

size_t fvi_size_lexer = sizeof(struct gps_lexer_t);
size_t fvi_size_buffer = (MAX_PACKET_LENGTH * 2) + 1;

struct gps_lexer_t *ffi_Lexer_init() {
  struct gps_lexer_t *result;
  result = calloc(1, fvi_size_lexer);
  if (result == NULL) {
    return NULL;
  }
  packet_reset(result);
  return result;
}
// vim: set expandtab shiftwidth=4
