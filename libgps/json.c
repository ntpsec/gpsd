/****************************************************************************

NAME
   json.c - parse JSON into fixed-extent data structures

DESCRIPTION
   This module parses a large subset of JSON (JavaScript Object
Notation).  Unlike more general JSON parsers, it doesn't use malloc(3)
and doesn't support polymorphism; you need to give it a set of
template structures describing the expected shape of the incoming
JSON, and it will error out if that shape is not matched.  When the
parse succeeds, attribute values will be extracted into static
locations specified in the template structures.

   The "shape" of a JSON object in the type signature of its
attributes (and attribute values, and so on recursively down through
all nestings of objects and arrays).  This parser is indifferent to
the order of attributes at any level, but you have to tell it in
advance what the type of each attribute value will be and where the
parsed value will be stored. The template structures may supply
default values to be used when an expected attribute is omitted.

   The preceding paragraph told one fib.  A single attribute may
actually have a span of multiple specifications with different
syntactically distinguishable types (e.g. string vs. real vs. integer
vs. boolean, but not signed integer vs. unsigned integer).  The parser
will match the right spec against the actual data.

   The dialect this parses has some limitations.  First, it cannot
recognize the JSON "null" value.  Secondly, arrays may not have
character values as elements (this limitation could be easily removed
if required). Third, all elements of an array must be of the same
type.  Fourth, it can not handle NaN's in doubles (Issue 53150).

   There are separate entry points for beginning a parse of either
JSON object or a JSON array. JSON "float" quantities are actually
stored as doubles.

   This parser processes object arrays in one of two different ways,
defending on whether the array subtype is declared as object or
structobject.

   Object arrays take one base address per object subfield, and are
mapped into parallel C arrays (one per subfield).  Strings are not
supported in this kind of array, as they don't have a "natural" size
to use as an offset multiplier.

   Structobjects arrays are a way to parse a list of objects to a set
of modifications to a corresponding array of C structs.  The trick is
that the array object initialization has to specify both the C struct
array's base address and the stride length (the size of the C struct).
If you initialize the offset fields with the correct offsetof calls,
everything will work. Strings are supported but all string storage
has to be inline in the struct.

NOTE
   This code has been spun out, packaged, and documented as a
reusable module; search for "microjson".

PERMISSIONS
   This file is Copyright 2010 by the GPSD project
   SPDX-License-Identifier: BSD-2-clause

***************************************************************************/
#include "../include/gpsd_config.h"  /* must be before all includes */

#include <ctype.h>
#include <math.h>       /* for HUGE_VAL */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/compiler.h"   // for FALLTHROUGH
#include "../include/os_compat.h"
#include "../include/json.h"

#include "../include/gps.h"                /* for safe_atof() prototype */
#include "../include/strfuncs.h"
#include "../include/timespec.h"

static int debuglevel = 0;
static FILE *debugfp;

void json_enable_debug(int level, FILE * fp)
/* control the level and destination of debug trace messages */
{
    debuglevel = level;
    debugfp = fp;
}

static void json_trace(int errlevel, const char *fmt, ...)
/* assemble command in printf(3) style */
{
    if (errlevel <= debuglevel && NULL != debugfp) {
        char buf[BUFSIZ] = {0};
        va_list ap;

        (void)strlcpy(buf, "json: ", sizeof(buf));
        va_start(ap, fmt);
        str_vappendf(buf, sizeof(buf), fmt, ap);
        va_end(ap);

        (void)fputs(buf, debugfp);
    }
}

#define json_debug_trace(args) (void) json_trace args

static char *json_target_address(const struct json_attr_t *cursor,
                                             const struct json_array_t
                                             *parent, int offset)
{
    char *targetaddr = NULL;
    if (parent == NULL || parent->element_type != t_structobject) {
        /* ordinary case - use the address in the cursor structure */
        switch (cursor->type) {
        case t_byte:
            targetaddr = (char *)&cursor->addr.byte[offset];
            break;
        case t_ubyte:
            targetaddr = (char *)&cursor->addr.ubyte[offset];
            break;
        case t_ignore:
            targetaddr = NULL;
            break;
        case t_integer:
            targetaddr = (char *)&cursor->addr.integer[offset];
            break;
        case t_uinteger:
            targetaddr = (char *)&cursor->addr.uinteger[offset];
            break;
        case t_longint:
            targetaddr = (char *)&cursor->addr.longint[offset];
            break;
        case t_ulongint:
            targetaddr = (char *)&cursor->addr.ulongint[offset];
            break;
        case t_short:
            targetaddr = (char *)&cursor->addr.shortint[offset];
            break;
        case t_ushort:
            targetaddr = (char *)&cursor->addr.ushortint[offset];
            break;
        case t_time:
            targetaddr = (char *)&cursor->addr.ts[offset];
            break;
        case t_timespec:
            targetaddr = (char *)&cursor->addr.ts[offset];
            break;
        case t_real:
            targetaddr = (char *)&cursor->addr.real[offset];
            break;
        case t_string:
            targetaddr = cursor->addr.string;
            break;
        case t_boolean:
            targetaddr = (char *)&cursor->addr.boolean[offset];
            break;
        case t_character:
            targetaddr = (char *)&cursor->addr.character[offset];
            break;
        default:
            targetaddr = NULL;
            break;
        }
    } else
        /* tricky case - hacking a member in an array of structures */
        targetaddr =
            parent->arr.objects.base + (offset * parent->arr.objects.stride) +
            cursor->addr.offset;
    json_debug_trace((1, "Target address for %s (offset %d) is %p\n",
                      cursor->attribute, offset, targetaddr));
    return targetaddr;
}


static int json_internal_read_object(const char *cp,
                                     const struct json_attr_t *attrs,
                                     const struct json_array_t *parent,
                                     int offset,
                                     const char **end)
{
    enum
    { init, await_attr, in_attr, await_value, in_val_string,
        in_escape, in_val_token, post_val, post_element
    } state = 0;
    char *statenames[] = {
        "init", "await_attr", "in_attr", "await_value", "in_val_string",
        "in_escape", "in_val_token", "post_val", "post_element",
    };
    char attrbuf[JSON_ATTR_MAX + 1], *pattr = NULL;
    char valbuf[JSON_VAL_MAX + 1], *pval = NULL;
    bool value_quoted = false;
    char uescape[5];            /* enough space for 4 hex digits and a NUL */
    const struct json_attr_t *cursor;
    int substatus, maxlen = 0;
    unsigned int u;
    const struct json_enum_t *mp;
    char *lptr;

    if (end != NULL)
        *end = NULL;    /* give it a well-defined value on parse failure */

    /* stuff fields with defaults in case they're omitted in the JSON input */
    for (cursor = attrs; cursor->attribute != NULL; cursor++)
        if (!cursor->nodefault) {
            lptr = json_target_address(cursor, parent, offset);
            if (lptr != NULL)
                switch (cursor->type) {
                case t_byte:
                    lptr[0] = cursor->dflt.byte;
                    break;
                case t_ubyte:
                    lptr[0] = cursor->dflt.ubyte;
                    break;
                case t_integer:
                    memcpy(lptr, &cursor->dflt.integer, sizeof(int));
                    break;
                case t_uinteger:
                    memcpy(lptr, &cursor->dflt.uinteger, sizeof(unsigned int));
                    break;
                case t_longint:
                    memcpy(lptr, &cursor->dflt.longint, sizeof(long));
                    break;
                case t_ulongint:
                    memcpy(lptr, &cursor->dflt.ulongint,
                           sizeof(unsigned long));
                    break;
                case t_short:
                    memcpy(lptr, &cursor->dflt.shortint, sizeof(short));
                    break;
                case t_ushort:
                    memcpy(lptr, &cursor->dflt.ushortint,
                           sizeof(unsigned short));
                    break;
                case t_time:
                    memcpy(lptr, &cursor->dflt.ts, sizeof(timespec_t));
                    break;
                case t_timespec:
                    memcpy(lptr, &cursor->dflt.ts, sizeof(timespec_t));
                    break;
                case t_real:
                    memcpy(lptr, &cursor->dflt.real, sizeof(double));
                    break;
                case t_string:
                    if (parent != NULL
                        && parent->element_type != t_structobject
                        && offset > 0)
                        return JSON_ERR_NOPARSTR;
                    lptr[0] = '\0';
                    break;
                case t_boolean:
                    memcpy(lptr, &cursor->dflt.boolean, sizeof(bool));
                    break;
                case t_character:
                    lptr[0] = cursor->dflt.character;
                    break;
                case t_object:  /* silences a compiler warning */
                case t_structobject:
                case t_array:
                case t_check:
                case t_ignore:
                    break;
                }
        }

    json_debug_trace((1, "JSON parse of '%s' begins.\n", cp));

    /* parse input JSON */
    for (; *cp != '\0'; cp++) {
        json_debug_trace((2, "State %-14s, looking at '%c' (%p)\n",
                          statenames[state], *cp, cp));
        switch (state) {
        case init:
            if (isspace((unsigned char) *cp))
                continue;
            else if (*cp == '{')
                state = await_attr;
            else {
                json_debug_trace((1,
                                  "Non-WS when expecting object start.\n"));
                if (end != NULL)
                    *end = cp;
                return JSON_ERR_OBSTART;
            }
            break;
        case await_attr:
            if (isspace((unsigned char) *cp))
                continue;
            else if (*cp == '"') {
                state = in_attr;
                pattr = attrbuf;
                if (end != NULL)
                    *end = cp;
            } else if (*cp == '}')
                break;
            else {
                json_debug_trace((1, "Non-WS when expecting attribute.\n"));
                if (end != NULL)
                    *end = cp;
                return JSON_ERR_ATTRSTART;
            }
            break;
        case in_attr:
            if (pattr == NULL)
                /* don't update end here, leave at attribute start */
                return JSON_ERR_NULLPTR;
            if (*cp == '"') {
                *pattr++ = '\0';
                json_debug_trace((1, "Collected attribute name %s\n",
                                  attrbuf));
                for (cursor = attrs; cursor->attribute != NULL; cursor++) {
                    json_debug_trace((2, "Checking against %s\n",
                                      cursor->attribute));
                    if (strcmp(cursor->attribute, attrbuf) == 0)
                        break;
                    if (cursor->type == t_ignore &&
                        strncmp(cursor->attribute, "", 1) == 0) {
                        break;
                    }
                }
                if (cursor->attribute == NULL) {
                    json_debug_trace((1,
                                      "Unknown attribute name '%s'"
                                      " (attributes begin with '%s').\n",
                                      attrbuf, attrs->attribute));
                    /* don't update end here, leave at attribute start */
                    return JSON_ERR_BADATTR;
                }
                state = await_value;
                if (cursor->type == t_string)
                    maxlen = (int)cursor->len - 1;
                else if (cursor->type == t_check)
                    maxlen = (int)strlen(cursor->dflt.check);
                else if (cursor->type == t_time || cursor->type == t_ignore)
                    maxlen = JSON_VAL_MAX;
                else if (cursor->map != NULL)
                    maxlen = (int)sizeof(valbuf) - 1;
                pval = valbuf;
            } else if (pattr >= attrbuf + JSON_ATTR_MAX - 1) {
                json_debug_trace((1, "Attribute name too long.\n"));
                /* don't update end here, leave at attribute start */
                return JSON_ERR_ATTRLEN;
            } else
                *pattr++ = *cp;
            break;
        case await_value:
            if (isspace((unsigned char) *cp) || *cp == ':')
                continue;
            else if (*cp == '[') {
                if (cursor->type != t_array) {
                    json_debug_trace((1,
                                      "Saw [ when not expecting array.\n"));
                    if (end != NULL)
                        *end = cp;
                    return JSON_ERR_NOARRAY;
                }
                substatus = json_read_array(cp, &cursor->addr.array, &cp);
                if (substatus != 0)
                    return substatus;
                state = post_element;
            } else if (cursor->type == t_array) {
                json_debug_trace((1,
                                  "Array element was specified, but no [.\n"));
                if (end != NULL)
                    *end = cp;
                return JSON_ERR_NOBRAK;
            } else if (*cp == '"') {
                value_quoted = true;
                state = in_val_string;
                pval = valbuf;
            } else {
                value_quoted = false;
                state = in_val_token;
                pval = valbuf;
                *pval++ = *cp;
            }
            break;
        case in_val_string:
            if (pval == NULL)
                /* don't update end here, leave at value start */
                return JSON_ERR_NULLPTR;
            if (*cp == '\\')
                state = in_escape;
            else if (*cp == '"') {
                *pval++ = '\0';
                json_debug_trace((1, "Collected string value %s\n", valbuf));
                state = post_val;
            } else if (pval > valbuf + JSON_VAL_MAX - 1
                       || pval > valbuf + maxlen - 1) {
                json_debug_trace((1, "String value too long.\n"));
                /* don't update end here, leave at value start */
                return JSON_ERR_STRLONG;        /*  */
            } else
                *pval++ = *cp;
            break;
        case in_escape:
            if (pval == NULL)
                /* don't update end here, leave at value start */
                return JSON_ERR_NULLPTR;
            else if (pval > valbuf + JSON_VAL_MAX - 1
                       || pval > valbuf + maxlen) {
                json_debug_trace((1, "String value too long.\n"));
                /* don't update end here, leave at value start */
                return JSON_ERR_STRLONG;        /*  */
            }
            switch (*cp) {
            case 'b':
                *pval++ = '\b';
                break;
            case 'f':
                *pval++ = '\f';
                break;
            case 'n':
                *pval++ = '\n';
                break;
            case 'r':
                *pval++ = '\r';
                break;
            case 't':
                *pval++ = '\t';
                break;
            case 'u':
                {
                    unsigned n;

                    cp++;                   /* skip the 'u' */
                    /* NetBSD 6 wants the cast */
                    for (n = 0; n < 4 && isxdigit((int)*cp); n++)
                        uescape[n] = *cp++;
                    uescape[n] = '\0';      /* terminate */
                    --cp;
                    /* ECMA-404 says JSON \u must have 4 hex digits */
                    if ((4 != n) || (1 != sscanf(uescape, "%4x", &u))) {
                        return JSON_ERR_BADSTRING;
                    }
                    /* truncate values above 0xff */
                    *pval++ = (unsigned char)u;
                }
                break;
            default:            /* handles double quote and solidus */
                *pval++ = *cp;
                break;
            }
            state = in_val_string;
            break;
        case in_val_token:
            if (pval == NULL)
                /* don't update end here, leave at value start */
                return JSON_ERR_NULLPTR;
            if (isspace((unsigned char) *cp) || *cp == ',' || *cp == '}') {
                *pval = '\0';
                json_debug_trace((1, "Collected token value %s.\n", valbuf));
                state = post_val;
                if (*cp == '}' || *cp == ',')
                    --cp;
            } else if (pval > valbuf + JSON_VAL_MAX - 1) {
                json_debug_trace((1, "Token value too long.\n"));
                /* don't update end here, leave at value start */
                return JSON_ERR_TOKLONG;
            } else
                *pval++ = *cp;
            break;
            /* coverity[unterminated_case] */
        case post_val:
            // Ignore whitespace after either string or token values.
            if (isspace(*cp)) {
                while (*cp != '\0' && isspace((unsigned char) *cp)) {
                    ++cp;
                }
                json_debug_trace((1, "Skipped trailing whitespace: value \"%s\"\n", valbuf));
            }
            /*
             * We know that cursor points at the first spec matching
             * the current attribute.  We don't know that it's *the*
             * correct spec; our dialect allows there to be any number
             * of adjacent ones with the same attrname but different
             * types.  Here's where we try to seek forward for a
             * matching type/attr pair if we're not looking at one.
             */
            for (;;) {
                int seeking = cursor->type;
                if (value_quoted && (cursor->type == t_string
                    || cursor->type == t_time))
                    break;
                if ((strcmp(valbuf, "true")==0 || strcmp(valbuf, "false")==0)
                        && seeking == t_boolean)
                    break;
                if (isdigit((unsigned char) valbuf[0])) {
                    bool decimal = strchr(valbuf, '.') != NULL;
                    if (decimal && seeking == t_real)
                        break;
                    if (!decimal && (seeking == t_byte ||
                                     seeking == t_ubyte ||
                                     seeking == t_integer ||
                                     seeking == t_uinteger ||
                                     seeking == t_longint ||
                                     seeking == t_ulongint ||
                                     seeking == t_short ||
                                     seeking == t_ushort))
                        break;
                }
                if (NULL == cursor[1].attribute)  /* out of possibilities */
                    break;
                if (0 != strcmp(cursor[1].attribute, attrbuf))
                    break;
                ++cursor;
            }
            if (value_quoted &&
                (cursor->type != t_string &&
                 cursor->type != t_character &&
                 cursor->type != t_check &&
                 cursor->type != t_time &&
                 cursor->type != t_ignore &&
                 cursor->map == 0)) {
                json_debug_trace((1, "Saw quoted value when expecting"
                                  " non-string.\n"));
                return JSON_ERR_QNONSTRING;
            }
            if (!value_quoted
                && (cursor->type == t_string || cursor->type == t_check
                    || cursor->type == t_time || cursor->map != 0)) {
                json_debug_trace((1, "Didn't see quoted value when expecting"
                                  " string.\n"));
                return JSON_ERR_NONQSTRING;
            }
            if (cursor->map != 0) {
                for (mp = cursor->map; mp->name != NULL; mp++)
                    if (strcmp(mp->name, valbuf) == 0) {
                        goto foundit;
                    }
                json_debug_trace((1, "Invalid enumerated value string %s.\n",
                                  valbuf));
                return JSON_ERR_BADENUM;
              foundit:
                (void)snprintf(valbuf, sizeof(valbuf), "%d", mp->value);
            }
            if (cursor->type == t_check) {
                lptr = cursor->dflt.check;
            } else {
                lptr = json_target_address(cursor, parent, offset);
            }
            if (lptr != NULL) {
                switch (cursor->type) {
                case t_byte:
                    {
                        int tmp = atoi(valbuf);
                        lptr[0] = (char)tmp;
                    }
                    break;
                case t_ubyte:
                    {
                        int tmp = atoi(valbuf);
                        lptr[0] = (unsigned char)tmp;
                    }
                    break;
                case t_integer:
                    {
                        int tmp = atoi(valbuf);
                        memcpy(lptr, &tmp, sizeof(int));
                    }
                    break;
                case t_uinteger:
                    {
                        unsigned int tmp = (unsigned int)atol(valbuf);
                        memcpy(lptr, &tmp, sizeof(unsigned int));
                    }
                    break;
                case t_longint:
                    {
                        long tmp = atol(valbuf);
                        memcpy(lptr, &tmp, sizeof(long));
                    }
                    break;
                case t_ulongint:
                    {
                        unsigned long tmp = (unsigned long)atoll(valbuf);
                        memcpy(lptr, &tmp, sizeof(unsigned long));
                    }
                    break;
                case t_short:
                    {
                        short tmp = atoi(valbuf);
                        memcpy(lptr, &tmp, sizeof(short));
                    }
                    break;
                case t_ushort:
                    {
                        unsigned short tmp = (unsigned int)atoi(valbuf);
                        memcpy(lptr, &tmp, sizeof(unsigned short));
                    }
                    break;
                case t_time:
                    {
                        timespec_t ts_tmp = iso8601_to_timespec(valbuf);
                        memcpy(lptr, &ts_tmp, sizeof(timespec_t));
                    }
                    break;
                case t_timespec:
                    {
                        double sec_tmp = safe_atof(valbuf);
                        timespec_t ts_tmp;
                        if (0 != isfinite(sec_tmp)) {
                            DTOTS(&ts_tmp, sec_tmp);
                            memcpy(lptr, &ts_tmp, sizeof(timespec_t));
                        } // else leave at .dflt
                    }
                    break;
                case t_real:
                    {
                        double tmp = safe_atof(valbuf);
                        if (0 != isfinite(tmp)) {
                            memcpy(lptr, &tmp, sizeof(double));
                        } // else leave at .dflt
                    }
                    break;
                case t_string:
                    if (parent != NULL
                        && parent->element_type != t_structobject
                        && offset > 0)
                        return JSON_ERR_NOPARSTR;
                    (void)strlcpy(lptr, valbuf, cursor->len);
                    break;
                case t_boolean:
                    {
                        bool tmp = (strcmp(valbuf, "true") == 0);
                        memcpy(lptr, &tmp, sizeof(bool));
                    }
                    break;
                case t_character:
                    if (strlen(valbuf) > 1)
                        /* don't update end here, leave at value start */
                        return JSON_ERR_STRLONG;
                    else
                        lptr[0] = valbuf[0];
                    break;
                case t_ignore:  /* silences a compiler warning */
                case t_object:  /* silences a compiler warning */
                case t_structobject:
                case t_array:
                    break;
                case t_check:
                    if (strcmp(cursor->dflt.check, valbuf) != 0) {
                        json_debug_trace((1, "Required attribute value %s"
                                          " not present.\n",
                                          cursor->dflt.check));
                        /* don't update end here, leave at start of attribute */
                        return JSON_ERR_CHECKFAIL;
                    }
                    break;
                }
            }
            FALLTHROUGH
        case post_element:
            if (isspace((unsigned char) *cp))
                continue;
            else if (*cp == ',')
                state = await_attr;
            else if (*cp == '}') {
                ++cp;
                goto good_parse;
            } else {
                json_debug_trace((1, "Garbage while expecting comma or }\n"));
                if (end != NULL)
                    *end = cp;
                return JSON_ERR_BADTRAIL;
            }
            break;
        }
    }
    if (state == init) {
        json_debug_trace((1, "Input was empty or white-space only\n"));
        return JSON_ERR_EMPTY;
    }

  good_parse:
    /* in case there's another object following, consume trailing WS */
    while (isspace((unsigned char) *cp))
        ++cp;
    if (end != NULL)
        *end = cp;
    json_debug_trace((1, "JSON parse ends.\n"));
    return 0;
}

int json_read_array(const char *cp, const struct json_array_t *arr,
                    const char **end)
{
    int substatus, offset, arrcount;
    char *tp;

    if (end != NULL)
        *end = NULL;    /* give it a well-defined value on parse failure */

    json_debug_trace((1, "Entered json_read_array()\n"));

    while (isspace((unsigned char) *cp))
        cp++;
    if (*cp != '[') {
        json_debug_trace((1, "Didn't find expected array start\n"));
        return JSON_ERR_ARRAYSTART;
    } else
        cp++;

    tp = arr->arr.strings.store;
    arrcount = 0;

    /* Check for empty array */
    while (isspace((unsigned char) *cp))
        cp++;
    if (*cp == ']')
        goto breakout;

    for (offset = 0; offset < arr->maxlen; offset++) {
        char *ep = NULL;
        json_debug_trace((1, "Looking at %s\n", cp));
        switch (arr->element_type) {
        case t_string:
            if (isspace((unsigned char) *cp))
                cp++;
            if (*cp != '"')
                return JSON_ERR_BADSTRING;
            else
                ++cp;
            arr->arr.strings.ptrs[offset] = tp;
            for (; tp - arr->arr.strings.store < arr->arr.strings.storelen;
                 tp++)
                if (*cp == '"') {
                    ++cp;
                    *tp++ = '\0';
                    goto stringend;
                } else if (*cp == '\0') {
                    json_debug_trace((1,
                                      "Bad string syntax in string list.\n"));
                    return JSON_ERR_BADSTRING;
                } else {
                    *tp = *cp++;
                }
            json_debug_trace((1, "Bad string syntax in string list.\n"));
            return JSON_ERR_BADSTRING;
          stringend:
            break;
        case t_object:
        case t_structobject:
            substatus =
                json_internal_read_object(cp, arr->arr.objects.subtype, arr,
                                          offset, &cp);
            if (substatus != 0) {
                if (end != NULL)
                    *end = cp;
                return substatus;
            }
            break;
        case t_integer:
            arr->arr.integers.store[offset] = (int)strtol(cp, &ep, 0);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_uinteger:
            arr->arr.uintegers.store[offset] = (unsigned int)strtoul(cp,
                                                                     &ep, 0);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_longint:
            arr->arr.longint.store[offset] = strtol(cp, &ep, 0);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_ulongint:
            arr->arr.ulongint.store[offset] = strtoul(cp, &ep, 0);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_byte:
            arr->arr.bytes.store[offset] = (char)strtol(cp, &ep, 0);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_ubyte:
            arr->arr.ubytes.store[offset] = (unsigned char)strtoul(cp,
                                                                   &ep, 0);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_short:
            arr->arr.shorts.store[offset] = (short)strtol(cp, &ep, 0);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_ushort:
            arr->arr.ushorts.store[offset] = (unsigned short)strtoul(cp,
                                                                     &ep, 0);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_time:
            {
                timespec_t ts_tmp;
                if (*cp != '"')
                    return JSON_ERR_BADSTRING;
                else
                    ++cp;
                ts_tmp = iso8601_to_timespec(cp);
                arr->arr.timespecs.store[offset] = ts_tmp;
                while (*cp && *cp != '"')
                    cp++;
                if (*cp != '"')
                    return JSON_ERR_BADSTRING;
                else
                    ++cp;
            }
            break;
        case t_timespec:
            // TODO not sure how to implement this
            return JSON_ERR_BADNUM;
            break;
        case t_real:
            arr->arr.reals.store[offset] = strtod(cp, &ep);
            if (ep == cp)
                return JSON_ERR_BADNUM;
            else
                cp = ep;
            break;
        case t_boolean:
            if (str_starts_with(cp, "true")) {
                arr->arr.booleans.store[offset] = true;
                cp += 4;
            }
            else if (str_starts_with(cp, "false")) {
                arr->arr.booleans.store[offset] = false;
                cp += 5;
            }
            break;
        case t_character:
        case t_array:
        case t_check:
        case t_ignore:
            json_debug_trace((1, "Invalid array subtype.\n"));
            return JSON_ERR_SUBTYPE;
        }
        arrcount++;
        if (isspace((unsigned char) *cp))
            cp++;
        if (*cp == ']') {
            json_debug_trace((1, "End of array found.\n"));
            goto breakout;
        } else if (*cp == ',')
            cp++;
        else {
            json_debug_trace((1, "Bad trailing syntax on array.\n"));
            return JSON_ERR_BADSUBTRAIL;
        }
    }
    json_debug_trace((1, "Too many elements in array.\n"));
    if (end != NULL)
        *end = cp;
    return JSON_ERR_SUBTOOLONG;
  breakout:
    if (arr->count != NULL)
        *(arr->count) = arrcount;
    if (end != NULL)
        *end = cp;
    json_debug_trace((1, "leaving json_read_array() with %d elements\n",
                      arrcount));
    return 0;
}

int json_read_object(const char *cp, const struct json_attr_t *attrs,
                     const char **end)
{
    int st;

    json_debug_trace((1, "json_read_object() sees '%s'\n", cp));
    st = json_internal_read_object(cp, attrs, NULL, 0, end);
    return st;
}

const char *json_error_string(int err)
{
    const char *errors[] = {
        "unknown error while parsing JSON",
        "non-whitespace when expecting object start",
        "non-whitespace when expecting attribute start",
        "unknown attribute name",
        "attribute name too long",
        "saw [ when not expecting array",
        "array element specified, but no [",
        "string value too long",
        "token value too long",
        "garbage while expecting comma or } or ]",
        "didn't find expected array start",
        "error while parsing object array",
        "too many array elements",
        "garbage while expecting array comma",
        "unsupported array element type",
        "error while string parsing",
        "check attribute not matched",
        "can't support strings in parallel arrays",
        "invalid enumerated value",
        "saw quoted value when expecting nonstring",
        "didn't see quoted value when expecting string",
        "other data conversion error",
        "unexpected null value or attribute pointer",
        "object element specified, but no {",
        "input was empty or white-space only",
    };

    if (err <= 0 || err >= (int)(sizeof(errors) / sizeof(errors[0])))
        return errors[0];
    else
        return errors[err];
}

/* quote a JSON string so it can be used as a simple JSON string.
 * Used to output the JSON as a literal JSON string
 * escape control chars, escape double quote.
 * stop at NUL, in_len or bad unicode char
 */
char *json_quote(const char *in_buffer, char *out_buffer, size_t in_len,
                 size_t out_len)
{
    const char *escape_match = "'\"/\\\b\f\n\r\t";
    const char *escaped_bit = "'\"/\\bfnrt";
    unsigned out_index = 0;
    const char *escape_ptr;
    unsigned in_index = 0;
    unsigned to_copy = 0;

    out_buffer[0] = '\0';

    // check in string, stop at NUL, done in_len, or out_buffer full
    for (in_index = 0; in_buffer[in_index] != '\0'; in_index++) {

        if (in_index >= in_len) {
            // got all from input buffer
            break;
        }

        if (out_index > (out_len - 8) ) {
            /* output out_buffer full.  Not enough space for a 4-byte UTF + NUL,
             * or \uxxxx + NUL.  Safer to check once, at the top,
             * than a lot of specific size checks later in the loop.
             */
            break;
        }

        if (in_buffer[in_index] & 0x80) {
            // highbit set. assume unicode
            to_copy = 0;    // always reset before use, to shut up coverity

            // check in_len so we don't overrun in_buffer
            if ((in_len > (in_index + 1)) &&
                (0xC0 == (0xE0 & (uint8_t)in_buffer[in_index])) &&
                (0x80 == (0xC0 & (uint8_t)in_buffer[in_index + 1]))) {
                // utf-8 ish 16bit rune - deg, plusm, mplus etc.
                to_copy = 2;
            } else if ((in_len > (in_index + 2)) &&
                       (0xE0 == (0xF0 & (uint8_t)in_buffer[in_index])) &&
                       (0x80 == (0xC0 & (uint8_t)in_buffer[in_index + 1])) &&
                       (0x80 == (0xC0 & (uint8_t)in_buffer[in_index + 2]))) {
                // utf-8 ish 24 bit rune - (double) prime etc.
                to_copy = 3;
            } else if ((in_len > (in_index + 3)) &&
                       (0xF0 == (0xF8 & (uint8_t)in_buffer[in_index])) &&
                       (0x80 == (0xC0 & (uint8_t)in_buffer[in_index + 1])) &&
                       (0x80 == (0xC0 & (uint8_t)in_buffer[in_index + 2])) &&
                       (0x80 == (0xC0 & (uint8_t)in_buffer[in_index + 3]))) {
                // utf-8 ish 32 bit rune - musical symbol g clef etc.
                to_copy = 4;
            } else {
                // WTF??  Short UTF?  Bad UTF?
                str_appendf(out_buffer, out_len,
                            "\\u%04x", in_buffer[in_index] & 0x0ff);
                out_index += 6;
                continue;
            }

            memcpy((void*)&out_buffer[out_index],
                   (void*)&in_buffer[in_index], to_copy);
            out_index += to_copy;
            // minus one as the for loop does in_index++
            in_index += to_copy - 1;
            out_buffer[out_index] = '\0';
            continue;
        }

        /* Try to find current byte from in buffer in string escape
         * match if it is there append '\', the corresponding byte
         * from escaped bit, and a null byte to end of out buffer.
         */
        escape_ptr = strchr(escape_match, in_buffer[in_index]);
        if (escape_ptr >= escape_match) {
            out_buffer[out_index++] = '\\';
            out_buffer[out_index++] = escaped_bit[escape_ptr-escape_match];
            out_buffer[out_index]   = 0;
            continue;
        }

        // Escape 0-31 and 127 if not previously handled (0-x01f,x7f)
        if ('\x1f' >= in_buffer[in_index] || '\x7f' == in_buffer[in_index]) {
            str_appendf(out_buffer, out_len, "\\u%04x",
                        in_buffer[in_index] & 0x0ff);
            out_index += 6;
            continue;
        }
        // pass through everything not escaped.
        out_buffer[out_index++] = in_buffer[in_index];
        out_buffer[out_index] = '\0';
    }
    return out_buffer;
}

/* end */

// vim: set expandtab shiftwidth=4
