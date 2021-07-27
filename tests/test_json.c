/* json.c - unit test for JSON parsing into fixed-extent structures
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include "../include/gpsd_config.h"

#include <getopt.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>              // for struct timespec

#include "../include/gpsd.h"
#include "../include/gps_json.h"

// Note: JSON_MINIMAL no longer exists

static int debug = 0;
static int current_test = 0;

static void assert_case(int status)
{
    if (status != 0) {
        (void)fprintf(stderr, "case %d FAILED\n", current_test);
        (void)fprintf(stderr, "status %d (%s).\n",
                      status, json_error_string(status));
        exit(EXIT_FAILURE);
    }
}

static void assert_string(char *attr, char *fld, char *val)
{
    if (strcmp(fld, val)) {
        (void)fprintf(stderr, "case %d FAILED\n", current_test);
        (void)fprintf(stderr,
                      "'%s' string attribute eval failed, value = %s.\n",
                      attr, fld);
        exit(EXIT_FAILURE);
    }
}

static void assert_string1(char *desc, char *got, char *sb)
{
    if (2 < debug) {
        (void)fprintf(stderr, "test string: >%s<\n", sb);
    }
    if (strcmp(got, sb)) {
        (void)fprintf(stderr, "case %d/%s FAILED\n", current_test, desc);
        (void)fprintf(stderr, "got = >%s<, s/b >%s<\n", got, sb);
        exit(EXIT_FAILURE);
    }
}

static void assert_int(const char *attr, const char *type, long fld, long val)
{
    if (fld != val) {
        (void)fprintf(stderr, "case %d FAILED\n", current_test);
        (void)fprintf(stderr,
                      "'%s' %s eval failed, value = %ld s/b %ld.\n",
                      attr, type, fld, val);
        exit(EXIT_FAILURE);
    }
}

static void assert_uint(const char *attr, const char *type, unsigned long fld,
                        unsigned long val)
{
    if (fld != val) {
        (void)fprintf(stderr, "case %d FAILED\n", current_test);
        (void)fprintf(stderr,
                      "'%s' %s eval failed, value = %lu s/b %lu.\n",
                      attr, type, fld, val);
        exit(EXIT_FAILURE);
    }
}

static void assert_boolean(char *attr, bool fld, bool val)
{
    if (fld != val) {
        (void)fprintf(stderr, "case %d FAILED\n", current_test);
        (void)fprintf(stderr,
                      "'%s' boolean attribute eval failed, value = %s.\n",
                      attr, fld ? "true" : "false");
        exit(EXIT_FAILURE);
    }
}

static void assert_ts(const char *attr, struct timespec fld,
                      struct timespec val)
{
    if (fld.tv_sec != val.tv_sec ||
        fld.tv_nsec != val.tv_nsec) {
        (void)fprintf(stderr,"case %d FAILED\n"
                      "  '%s' timespec eval failed, value = %lld %ld s/b "
                      "%lld %ld.\n",
                      current_test, attr,
                      (long long)fld.tv_sec, fld.tv_nsec,
                      (long long)val.tv_sec, val.tv_nsec);
        exit(EXIT_FAILURE);
    }
}

static void assert_other(char *desc, int val, int val1)
{
    if (val != val1) {
        (void)fprintf(stderr, "case %d FAILED\n", current_test);
        (void)fprintf(stderr, "'%s' was %d, s/b %d\n", desc, val, val1);
        exit(EXIT_FAILURE);
    }
}

/*
 * Floating point comparisons are iffy, but at least if any of these fail
 * the output will make it clear whether it was a precision issue
 */
static void assert_real(char *attr, double fld, double val)
{
    if (fld != val) {
        (void)fprintf(stderr, "case %d FAILED\n", current_test);
        (void)fprintf(stderr,
                      "'%s' real attribute eval failed, value = %f.\n", attr,
                      fld);
        exit(EXIT_FAILURE);
    }
}


static struct gps_data_t gpsdata;

/* Case 1: TPV report */

/* *INDENT-OFF* */
static const char json_str1[] =
    "{\"class\":\"TPV\","
    "\"device\":\"GPS#1\",\"time\":\"2005-06-19T08:12:41.89Z\","
    "\"lon\":46.498203637,\"lat\":7.568074350,\"altHAE\":1327.780,"
    "\"epx\":21.000,\"epy\":23.000,\"epv\":124.484,\"mode\":3}";

/*
 * Case 2: SKY report
 *
 * The fields of the last satellite entry are arranged in the reverse order
 * of the structure fields, in order to test for field overflow.
 */

static const char *json_str2 = "{\"class\":\"SKY\",\
         \"time\":\"2005-06-19T12:12:42.03Z\",   \
         \"satellites\":[\
         {\"PRN\":10,\"el\":45,\"az\":196,\"ss\":34,\"used\":true},\
         {\"PRN\":29,\"el\":67,\"az\":310,\"ss\":40,\"used\":true},\
         {\"PRN\":28,\"el\":59,\"az\":108,\"ss\":42,\"used\":true},\
         {\"PRN\":26,\"el\":51,\"az\":304,\"ss\":43,\"used\":true},\
         {\"PRN\":8,\"el\":44,\"az\":58,\"ss\":41,\"used\":true},\
         {\"PRN\":27,\"el\":16,\"az\":66,\"ss\":39,\"used\":true},\
         {\"az\":301,\"el\":10,\"PRN\":21,\"used\":false,\"ss\":0}]}";

/* Case 3: String list syntax */

static const char *json_str3 = "[\"foo\",\"bar\",\"baz\"]";

static char *stringptrs[3];
static char stringstore[256];
static int stringcount;

static const struct json_array_t json_array_3 = {
    .element_type = t_string,
    .arr.strings.ptrs = stringptrs,
    .arr.strings.store = stringstore,
    .arr.strings.storelen = sizeof(stringstore),
    .count = &stringcount,
    .maxlen = sizeof(stringptrs)/sizeof(stringptrs[0]),
};

/* Case 4: test defaulting of unspecified attributes */

static const char *json_str4 = "{\"flag1\":true,\"flag2\":false}";

static bool flag1, flag2;
static double dftreal;
// char can be signed or unsigned!  We can only use range 0 to 127
static char dftbyte;
static char maxbyte;
static char minbyte;
static unsigned char dftubyte;
static int dftinteger;
static int maxint;
static int minint;
static unsigned int maxuint;
static unsigned int dftuinteger;
static long dftlongint;
static unsigned long dftulongint;
static struct timespec ts;
static struct timespec maxts;

static const struct json_attr_t json_attrs_4[] = {
    // t_byte can be signed, or unsigned, so can only use range 0 - 127
    {"dftbyte",  t_byte, .addr.byte = &dftbyte, .dflt.byte = 1},
    {"dftubyte", t_ubyte, .addr.ubyte = &dftubyte, .dflt.ubyte = 9},
    {"dftint",  t_integer, .addr.integer = &dftinteger, .dflt.integer = -5},
    {"dftuint", t_uinteger, .addr.uinteger = &dftuinteger, .dflt.uinteger = 10},
    {"dftlongint",  t_longint, .addr.longint = &dftlongint, .dflt.longint = -6},
    {"dftulongint", t_ulongint, .addr.ulongint = &dftulongint,
     .dflt.ulongint = 11},
    {"dftreal", t_real,    .addr.real = &dftreal,       .dflt.real = 23.17},
    {"maxbyte",  t_byte, .addr.byte = &maxbyte, .dflt.byte = 127},
    {"minbyte",  t_byte, .addr.byte = &minbyte, .dflt.byte = 0},
    {"maxint",  t_integer, .addr.integer = &maxint, .dflt.integer = 32767},
    {"minint",  t_integer, .addr.integer = &minint, .dflt.integer = -32767},
    {"maxuint",  t_uinteger, .addr.uinteger = &maxuint, .dflt.uinteger = 65535},
    {"flag1",   t_boolean, .addr.boolean = &flag1,},
    {"flag2",   t_boolean, .addr.boolean = &flag2,},
    {"dftts",  t_timespec, .addr.ts = &ts, .dflt.ts = {0,0}},
    {"maxts",  t_timespec, .addr.ts = &maxts, .dflt.ts = {0x0ffff,9}},
    {NULL},
};

/* Case 5: test DEVICE parsing */

static const char *json_str5 = "{\"class\":\"DEVICE\",\
           \"path\":\"/dev/ttyUSB0\",\
           \"flags\":5,\
           \"driver\":\"Foonly\",\"subtype\":\"Foonly Frob\",\
           \"cycle\":1.1,\"mincycle\":0.002\
           }";

/* Case 6: test parsing of subobject list into array of structures */

static const char *json_str6 = "{\"parts\":[\
           {\"name\":\"Urgle\", \"flag\":true, \"count\":3},\
           {\"name\":\"Burgle\",\"flag\":false,\"count\":1},\
           {\"name\":\"Witter\",\"flag\":true, \"count\":4},\
           {\"name\":\"Thud\",  \"flag\":false,\"count\":1}]}";

struct dumbstruct_t {
    char name[64];
    bool flag;
    int count;
};
static struct dumbstruct_t dumbstruck[5];
static int dumbcount;

static const struct json_attr_t json_attrs_6_subtype[] = {
    {"name",  t_string,  .addr.offset = offsetof(struct dumbstruct_t, name),
                         .len = 64},
    {"flag",  t_boolean, .addr.offset = offsetof(struct dumbstruct_t, flag),},
    {"count", t_integer, .addr.offset = offsetof(struct dumbstruct_t, count),},
    {NULL},
};

static const struct json_attr_t json_attrs_6[] = {
    {"parts", t_array,
     .addr.array.element_type = t_structobject,
     .addr.array.arr.objects.base = (char*)&dumbstruck,
     .addr.array.arr.objects.stride = sizeof(struct dumbstruct_t),
     .addr.array.arr.objects.subtype = json_attrs_6_subtype,
     .addr.array.count = &dumbcount,
     .addr.array.maxlen = sizeof(dumbstruck)/sizeof(dumbstruck[0])},
    {NULL},
};

/* Case 7: test parsing of version response */

static const char *json_str7 = "{\"class\":\"VERSION\",\
           \"release\":\"" VERSION "\",\"rev\":\"dummy-revision\",\
           \"proto_major\":3,\"proto_minor\":1}";

/* Case 8: test parsing arrays of enumerated types */

static const char *json_str8 =
     "{\"fee\":\"FOO\",\"fie\":\"BAR\",\"foe\":\"BAZ\"}";
static const struct json_enum_t enum_table[] = {
    {"BAR", 6}, {"FOO", 3}, {"BAZ", 14}, {NULL}
};

static int fee, fie, foe;
static const struct json_attr_t json_attrs_8[] = {
    {"fee",  t_integer, .addr.integer = &fee, .map=enum_table},
    {"fie",  t_integer, .addr.integer = &fie, .map=enum_table},
    {"foe",  t_integer, .addr.integer = &foe, .map=enum_table},
    {NULL},
};

/* Case 9: Like case 6 but w/ an empty array */

static const char *json_str9 = "{\"parts\":[]}";

/* Case 10: test parsing of PPS message  */

static const char *json_strPPS = "{\"class\":\"PPS\",\"device\":\"GPS#1\"," \
    "\"real_sec\":1428001514, \"real_nsec\":1000000," \
    "\"clock_sec\":1428001513,\"clock_nsec\":999999999," \
    "\"precision\":-20,\"qErr\":-123456}";

/* Case 11: test parsing of TOFF message  */

static const char *json_strTOFF = "{\"class\":\"TOFF\",\"device\":\"GPS#1\"," \
    "\"real_sec\":1428001514, \"real_nsec\":1000000," \
    "\"clock_sec\":1428001513,\"clock_nsec\":999999999}";

/* Case 12: test parsing of OSC message */

static const char *json_strOSC = "{\"class\":\"OSC\",\"device\":\"GPS#1\"," \
    "\"running\":true,\"reference\":true,\"disciplined\":false," \
    "\"delta\":67}";

/* Case 13: test parsing of ERROR message, and some escape sequences */

static char *json_strErr = "{\"class\":\"ERROR\",\"message\":" \
                           "\"Hello\b\f\n\r\t\"}";

/* Case 14: test parsing of ERROR message and \u escape */
/* per ECMA-404, \u must be followed by 4 hex digits */

static char *json_strErr1 = "{\"class\":\"ERROR\",\"message\":\"0\\u00334\"}";

/* Case 15: test buffer overflow of short string destination */

static char *json_strOver = "{\"name\":\"\\u0033\\u0034\\u0035\\u0036\"}";

char json_short_string_dst[2];
int json_short_string_cnt = 5;
static const struct json_attr_t json_short_string[] = {
    {"name", t_string,
        .addr.string = json_short_string_dst,
        .len = sizeof(json_short_string_dst)},
    {"count", t_integer, .addr.integer = &json_short_string_cnt},
    {NULL},
};

/* Case 16: test buffer overflow of short string destination */

static char json_strOver2[7 * JSON_VAL_MAX];  /* dynamically built */

/* Case 18: Ignore part of VERSION sentence */

static char *json_str18 =
    "{\"class\":\"VERSION\",\"release\":\"" VERSION "\","
    "\"rev\":\"release-dummy\",\"proto_major\":3,\"proto_minor\":14}";

char release[50];
int pvhi, pvlo;
static const struct json_attr_t json_attrs_18[] = {
    {"class", t_check, .dflt.check = "VERSION"},
    {"release", t_string, .addr.string = (char *)&release, .len = 50},
    {"proto_major", t_integer, .addr.integer = &pvhi},
    {"proto_minor", t_integer, .addr.integer = &pvlo},
    {"", t_ignore},
    {NULL},
};

/* Case 19: Ignore part of WATCH sentence */

static char *json_str19 =
    "{\"class\":\"WATCH\",\"enable\":true,\"json\":true,\"nmea\":false,\"raw\":"
    "0,\"scaled\":false,\"timing\":false,\"split24\":false,\"pps\":false,"
    "\"device\":\"/dev/ttyUSB0\"}";

bool enable, json;
static const struct json_attr_t json_attrs_19[] = {
    {"class", t_check, .dflt.check = "WATCH"},
    {"device", t_check, .dflt.check = "/dev/ttyUSB0"},
    {"enable", t_boolean, .addr.boolean = &enable},
    {"json", t_boolean, .addr.boolean = &json},
    {"", t_ignore},
    {NULL},
};

/* Case 20: Ignore part of TPV sentence */

static char *json_str20 =
    "{\"class\":\"TPV\",\"device\":\"/dev/"
    "ttyUSB0\",\"mode\":3,\"time\":\"2019-10-04T08:51:34.000Z\",\"ept\":0.005,"
    "\"lat\":46.367303831,\"lon\":-116.963791235,\"altHAE\":460.834,\"altMSL\":"
    "476.140,\"epx\":7.842,\"epy\":12.231,\"epv\":30.607,\"track\":57.1020,"
    "\"magtrack\":70.9299,\"magvar\":13.8,\"speed\":0.065,\"climb\":-0.206,"
    "\"eps\":24.46,\"epc\":61.21,\"ecefx\":-1999242.00,\"ecefy\":-3929871.00,"
    "\"ecefz\":4593848.00,\"ecefvx\":0.12,\"ecefvy\":0.12,\"ecefvz\":-0.12,"
    "\"velN\":0.035,\"velE\":0.055,\"velD\":0.206,\"geoidSep\":-15.307,\"eph\":"
    "15.200,\"sep\":31.273}";

int gps_mode;
double ept;
char gps_time[50];
static const struct json_attr_t json_attrs_20[] = {
    {"class", t_check, .dflt.check = "TPV"},
    {"device", t_check, .dflt.check = "/dev/ttyUSB0"},
    {"mode", t_integer, .addr.integer = &gps_mode, .dflt.integer = -1},
    {"time", t_string, .addr.string = (char *)&gps_time, .len = 50},
    {"ept", t_real, .addr.real = &ept, .dflt.real = NAN},
    {"", t_ignore},
    {NULL},
};

/* Case 21: Read array of integers */

static const char *json_strInt = "[23,-17,5]";
static int intstore[4], intcount;

static const struct json_array_t json_array_Int = {
    .element_type = t_integer,
    .arr.integers.store = intstore,
    .count = &intcount,
    .maxlen = sizeof(intstore)/sizeof(intstore[0]),
};

/* Case 22: Read array of booleans */

static const char *json_strBool = "[true,false,true]";
static bool boolstore[4];
static int boolcount;

static const struct json_array_t json_array_Bool = {
    .element_type = t_boolean,
    .arr.booleans.store = boolstore,
    .count = &boolcount,
    .maxlen = sizeof(boolstore)/sizeof(boolstore[0]),
};

/* Case 23: Read array of reals */

static const char *json_strReal = "[23.1,-17.2,5.3]";
static double realstore[4];
static int realcount;

static const struct json_array_t json_array_Real = {
    .element_type = t_real,
    .arr.reals.store = realstore,
    .count = &realcount,
    .maxlen = sizeof(realstore)/sizeof(realstore[0]),
};

// Case 24: ascii, bfnrt, lows unicode string encoding
char ee24a[] = "This, that, the other thing.",
     ee24b[] = "\b\f\n\r\t\'\"\\/",
     // test for NUL
     ee24c[] = "This, that, the other thing.\0Not This",
     // test for good trailing unicode
     ee24d[] = "Hello\xc2\xb0",
     // test for bad trailing unicode
     ee24e[] = "Hello\xc2",
     // test for short output buffer
     ee24f[] = "Hello\xc2",
     ee24l[] = "\x01\x07\x15",
     /* Note the char after the "13" is a "double prime", U+2033
      * not a double quote! */
     ee24u[] = "±176°42′13″ 𠜎 𠜱 𠝹 𠱓";
char ed24a[] = "This, that, the other thing.",
     ed24b[] = "\\b\\f\\n\\r\\t\\'\\\"\\\\\\/",
     ed24c[] = "This, that, the other thing.",
     ed24d[] = "Hello\xc2\xb0",
     ed24e[] = "Hello\\u00c2",
     ed24f[] = "Hello",
     ed24l[] = "\\u0001\\u0007\\u0015",
     ed24u[] = "±176°42′13″ 𠜎 𠜱 𠝹 𠱓";

static char *json_str25a = "{\"class\":\"\",\"mode\":-1}";
static char *json_str25b = "{\"class\":\"f\",\"mode\":-2}";
static char *json_str25c = "{\"class\":\"fo\",\"mode\":-3}";
static char *json_str25d = "{\"class\":\"foo\",\"mode\":-4}";
static char *json_str25e = "{\"class\":\"foob\",\"mode\":-5}";
static char *json_str25f = "{\"class\":\"fooba\",\"mode\":-6}";
static char *json_str25t = "{\"class\":\"TPV\",\"mode\":3}";

int i25 = 25;
static const struct json_attr_t json_attrs_25[] = {
    {"class", t_check, .dflt.check = "TPV"},
    {"mode", t_integer, .addr.integer = &i25, .dflt.integer = -9},
    {NULL},
};


char str32[] = "\f\n\r\t\v";
/* *INDENT-ON* */

static void jsontest(int i)
{
    int status = 0;   /* libgps_json_unpack() returned status */
    int n;            /* generic index */
    char buffer[500];
    char *pbuf;
    struct timespec expected_ts;

    if (0 < debug) {
        (void)fprintf(stderr, "Running test #%d.\n", i);
    }
    current_test = i;

    /* do not keep old data! */
    memset((void *)&gpsdata, 0, sizeof(gpsdata));

    switch (i)
    {
    case 1:
        status = libgps_json_unpack(json_str1, &gpsdata, NULL);
        assert_case(status);
        assert_string("device", gpsdata.dev.path, "GPS#1");
        assert_int("mode", "t_integer", gpsdata.fix.mode, 3);
        assert_int("time.tv_sec", "t_integer", gpsdata.fix.time.tv_sec,
                   1119168761);
        assert_int("time.tv_nsec", "t_integer",
                   gpsdata.fix.time.tv_nsec / 10000000, 89);
        assert_real("lon", gpsdata.fix.longitude, 46.498203637);
        assert_real("lat", gpsdata.fix.latitude, 7.568074350);
        break;

    case 2:
        status = libgps_json_unpack(json_str2, &gpsdata, NULL);
        assert_case(status);
        assert_int("used", "t_integer", gpsdata.satellites_used, 6);
        assert_int("PRN[0]", "t_integer", gpsdata.skyview[0].PRN, 10);
        assert_int("el[0]", "t_integer", gpsdata.skyview[0].elevation, 45);
        assert_int("az[0]", "t_integer", gpsdata.skyview[0].azimuth, 196);
        assert_real("ss[0]", gpsdata.skyview[0].ss, 34);
        assert_boolean("used[0]", gpsdata.skyview[0].used, true);
        assert_int("PRN[6]", "t_integer", gpsdata.skyview[6].PRN, 21);
        assert_int("el[6]", "t_integer", gpsdata.skyview[6].elevation, 10);
        assert_int("az[6]", "t_integer", gpsdata.skyview[6].azimuth, 301);
        assert_real("ss[6]", gpsdata.skyview[6].ss, 0);
        assert_boolean("used[6]", gpsdata.skyview[6].used, false);
        break;

    case 3:
        status = json_read_array(json_str3, &json_array_3, NULL);
        assert_case(status);
        assert_other("stringcount", stringcount, 3);
        assert_other("stringptrs[0] == foo", strcmp(stringptrs[0], "foo"), 0);
        assert_other("stringptrs[1] == bar", strcmp(stringptrs[1], "bar"), 0);
        assert_other("stringptrs[2] == baz", strcmp(stringptrs[2], "baz"), 0);
        break;

    case 4:
        status = json_read_object(json_str4, json_attrs_4, NULL);
        assert_case(status);
        // did the defaults work?
        assert_int("dftbyte", "t_byte", dftbyte, 1);
        assert_uint("dftubyte", "t_ubyte", dftubyte, 9);
        assert_int("dftint", "t_integer", dftinteger, -5);
        assert_uint("dftuint", "t_uinteger", dftuinteger, 10);
        assert_int("dftlongint", "t_longint", dftlongint, -6);
        assert_uint("dftulongint", "t_ulongint", dftulongint, 11);
        assert_real("dftreal", dftreal, 23.17);
        assert_int("maxbyte", "t_byte", maxbyte, 127);
        assert_int("minbyte", "t_byte", minbyte, 0);
        assert_int("maxint", "t_integer", maxint, 32767);
        assert_int("minint", "t_integer", minint, -32767);
        assert_int("maxuint", "t_uinteger", maxuint, 65535);
        assert_boolean("flag1", flag1, true);
        assert_boolean("flag2", flag2, false);
        expected_ts.tv_sec = 0;
        expected_ts.tv_nsec = 0;
        assert_ts("dflts", ts, expected_ts);
        expected_ts.tv_sec = 0x0ffff;
        expected_ts.tv_nsec = 9;
        assert_ts("maxts", maxts, expected_ts);
        break;

    case 5:
        status = libgps_json_unpack(json_str5, &gpsdata, NULL);
        assert_case(status);
        assert_string("path", gpsdata.dev.path, "/dev/ttyUSB0");
        assert_int("flags", "t_integer", gpsdata.dev.flags, 5);
        assert_string("driver", gpsdata.dev.driver, "Foonly");
        expected_ts.tv_sec = 1;
        expected_ts.tv_nsec = 100000000;
        assert_ts("cycle", gpsdata.dev.cycle, expected_ts);
        expected_ts.tv_sec = 0;
        expected_ts.tv_nsec = 2000000;
        assert_ts("mincycle", gpsdata.dev.mincycle, expected_ts);
        break;

    case 6:
        status = json_read_object(json_str6, json_attrs_6, NULL);
        assert_case(status);
        assert_int("dumbcount", "t_integer", dumbcount, 4);
        assert_string("dumbstruck[0].name", dumbstruck[0].name, "Urgle");
        assert_string("dumbstruck[1].name", dumbstruck[1].name, "Burgle");
        assert_string("dumbstruck[2].name", dumbstruck[2].name, "Witter");
        assert_string("dumbstruck[3].name", dumbstruck[3].name, "Thud");
        assert_boolean("dumbstruck[0].flag", dumbstruck[0].flag, true);
        assert_boolean("dumbstruck[1].flag", dumbstruck[1].flag, false);
        assert_boolean("dumbstruck[2].flag", dumbstruck[2].flag, true);
        assert_boolean("dumbstruck[3].flag", dumbstruck[3].flag, false);
        assert_int("dumbstruck[0].count", "t_integer", dumbstruck[0].count, 3);
        assert_int("dumbstruck[1].count", "t_integer", dumbstruck[1].count, 1);
        assert_int("dumbstruck[2].count", "t_integer", dumbstruck[2].count, 4);
        assert_int("dumbstruck[3].count", "t_integer", dumbstruck[3].count, 1);
        break;

    case 7:
        status = libgps_json_unpack(json_str7, &gpsdata, NULL);
        assert_case(status);
        assert_string("release", gpsdata.version.release, VERSION);
        assert_string("rev", gpsdata.version.rev, "dummy-revision");
        assert_int("proto_major", "t_integer", gpsdata.version.proto_major, 3);
        assert_int("proto_minor", "t_integer", gpsdata.version.proto_minor, 1);
        break;

    case 8:
        status = json_read_object(json_str8, json_attrs_8, NULL);
        assert_case(status);
        assert_int("fee", "t_integer", fee, 3);
        assert_int("fie", "t_integer", fie, 6);
        assert_int("foe", "t_integer", foe, 14);
        break;

    case 9:
        /* yes, the '6' in the next line is correct */
        status = json_read_object(json_str9, json_attrs_6, NULL);
        assert_case(status);
        assert_int("dumbcount", "t_integer", dumbcount, 0);
        break;

    case 10:
        status = json_pps_read(json_strPPS, &gpsdata, NULL);
        assert_case(status);
        assert_string("device", gpsdata.dev.path, "GPS#1");
        assert_int("real_sec", "t_integer", gpsdata.pps.real.tv_sec, 1428001514);
        assert_int("real_nsec", "t_integer", gpsdata.pps.real.tv_nsec, 1000000);
        assert_int("clock_sec", "t_integer", gpsdata.pps.clock.tv_sec,
                   1428001513);
        assert_int("clock_nsec", "t_integer", gpsdata.pps.clock.tv_nsec,
                   999999999);
        assert_int("qErr", "t_integer", gpsdata.qErr, -123456);
        break;

    case 11:
        status = json_toff_read(json_strTOFF, &gpsdata, NULL);
        assert_case(status);
        assert_string("device", gpsdata.dev.path, "GPS#1");
        assert_int("real_sec", "t_integer", gpsdata.toff.real.tv_sec,
                   1428001514);
        assert_int("real_nsec", "t_integer", gpsdata.toff.real.tv_nsec,
                   1000000);
        assert_int("clock_sec", "t_integer", gpsdata.toff.clock.tv_sec,
                   1428001513);
        assert_int("clock_nsec", "t_integer", gpsdata.toff.clock.tv_nsec,
                   999999999);
        break;

    case 12:
        status = json_oscillator_read(json_strOSC, &gpsdata, NULL);
        assert_case(status);
        assert_string("device", gpsdata.dev.path, "GPS#1");
        assert_boolean("running", gpsdata.osc.running, true);
        assert_boolean("reference", gpsdata.osc.reference, true);
        assert_boolean("disciplined", gpsdata.osc.disciplined, false);
        assert_int("delta", "t_integer", gpsdata.osc.delta, 67);
        break;

    case 13:
        if (2 < debug) {
            (void)fprintf(stderr, "test string: %s.\n", json_strErr);
        }
        status = libgps_json_unpack(json_strErr, &gpsdata, NULL);
        assert_case(status);
        assert_string("message", gpsdata.error, "Hello\b\f\n\r\t");
        break;

    case 14:
        if (2 < debug) {
            (void)fprintf(stderr, "test string: %s.\n", json_strErr1);
        }
        status = libgps_json_unpack(json_strErr1, &gpsdata, NULL);
        assert_case(status);
        assert_string("message", gpsdata.error, "034");
        break;

    case 15:
        /* check for string overrun caught */
        if (2 < debug) {
            (void)fprintf(stderr, "test string: %s.\n", json_strOver);
        }
        json_short_string_cnt = 7;
        status = json_read_object(json_strOver, json_short_string, NULL);
        assert_case(JSON_ERR_STRLONG != status);
        assert_string("name", json_short_string_dst, "");
        assert_int("count", "t_integer", json_short_string_cnt, 0);
        break;

    case 16:
        /* check for string overrun caught */
        json_strOver2[0] = '\0';
        /* build a LONG test string */
        strlcat(json_strOver2, "{\"name\":\"", sizeof(json_strOver2));
        for (n = 0; n < (2 * JSON_VAL_MAX); n++) {
            strlcat(json_strOver2, "\\u0033", sizeof(json_strOver2));
        }
        strlcat(json_strOver2, "\"}", sizeof(json_strOver2));

        if (2 < debug) {
            (void)fprintf(stderr, "test string: %s.\n", json_strOver2);
        }
        json_short_string_cnt = 7;
        status = json_read_object(json_strOver2, json_short_string, NULL);
        assert_case(JSON_ERR_STRLONG != status);
        assert_string("name", json_short_string_dst, "");
        assert_int("count", "t_integer", json_short_string_cnt, 0);
        break;

    case 17:
        /* check for a different string overrun caught */
        json_strOver2[0] = '\0';
        /* build a LONG test string */
        strlcat(json_strOver2, "{\"name\":\"", sizeof(json_strOver2));
        for (n = 0; n < (2 * JSON_VAL_MAX); n++) {
            strlcat(json_strOver2, "\\A", sizeof(json_strOver2));
        }
        strlcat(json_strOver2, "\"}", sizeof(json_strOver2));

        if (2 < debug) {
            (void)fprintf(stderr, "test string: %s.\n", json_strOver2);
        }
        json_short_string_cnt = 7;
        status = json_read_object(json_strOver2, json_short_string, NULL);
        assert_case(JSON_ERR_STRLONG != status);
        assert_string("name", json_short_string_dst, "");
        assert_int("count", "t_integer", json_short_string_cnt, 0);
        break;

    case 18:
        status = json_read_object(json_str18, json_attrs_18, NULL);
        assert_int("proto_major", "t_integer", pvhi, 3);
        assert_int("proto_minor", "t_integer", pvlo, 14);
        assert_string("release", release, VERSION);
        assert_int("return", "t_integer", status, 0);
        break;

    case 19:
        status = json_read_object(json_str19, json_attrs_19, NULL);
        assert_boolean("enable", enable, true);
        assert_boolean("json", json, true);
        assert_int("return", "t_integer", status, 0);
        break;

    case 20:
        status = json_read_object(json_str20, json_attrs_20, NULL);
        assert_int("mode", "t_integer", gps_mode, 3);
        assert_string("time", gps_time, "2019-10-04T08:51:34.000Z");
        assert_real("ept", ept, 0.005);
        assert_int("return", "t_integer", status, 0);
        break;

    case 21:
        status = json_read_array(json_strInt, &json_array_Int, NULL);
        assert_case(status);
        assert_int("count", "t_integer", intcount, 3);
        assert_int("intstore[0]", "t_integer", intstore[0], 23);
        assert_int("intstore[1]", "t_integer", intstore[1], -17);
        assert_int("intstore[2]", "t_integer", intstore[2], 5);
        assert_int("intstore[3]", "t_integer", intstore[3], 0);
        break;

    case 22:
        status = json_read_array(json_strBool, &json_array_Bool, NULL);
        assert_case(status);
        assert_int("count", "t_integer", boolcount, 3);
        assert_boolean("boolstore[0]", boolstore[0], true);
        assert_boolean("boolstore[1]", boolstore[1], false);
        assert_boolean("boolstore[2]", boolstore[2], true);
        assert_boolean("boolstore[3]", boolstore[3], false);
        break;

    case 23:
        status = json_read_array(json_strReal, &json_array_Real, NULL);
        assert_case(status);
        assert_int("count", "t_integer", realcount, 3);
        assert_real("realstore[0]", realstore[0], 23.1);
        assert_real("realstore[1]", realstore[1], -17.2);
        assert_real("realstore[2]", realstore[2], 5.3);
        assert_real("realstore[3]", realstore[3], 0);
        break;

    case 24:
        // test w/o the trailing NUL
        pbuf = json_quote(ee24a, buffer, sizeof(ee24a) - 1, sizeof(buffer));
        assert_string1("Ascii", pbuf, ed24a);

        pbuf = json_quote(ee24b, buffer, sizeof(ee24b), sizeof(buffer));
        assert_string1("bfnrt", pbuf, ed24b);

        pbuf = json_quote(ee24c, buffer, sizeof(ee24c), sizeof(buffer));
        assert_string1("NUL", pbuf, ed24c);

        pbuf = json_quote(ee24d, buffer, sizeof(ee24d), sizeof(buffer));
        assert_string1("trailing utf", pbuf, ed24d);

        pbuf = json_quote(ee24e, buffer, sizeof(ee24e), sizeof(buffer));
        assert_string1("Bad trailing utf", pbuf, ed24e);

        // test for short output buffer
        pbuf = json_quote(ee24f, buffer, sizeof(ee24f), (size_t)6);
        assert_string1("Bad trailing utf", pbuf, ed24f);

        pbuf = json_quote(ee24l, buffer, sizeof(ee24l), sizeof(buffer));
        assert_string1("low", pbuf, ed24l);

        pbuf = json_quote(ee24u, buffer, sizeof(ee24u), sizeof(buffer));
        assert_string1("unicode", pbuf, ed24u);
        break;

    // Check for strings from (25) "" to (28) "foo" --
    // should return JSON_ERR_CHECKFAIL (16)
    case 25:
        status = json_read_object(json_str25a, json_attrs_25, NULL);
        assert_int("mode", "t_integer", i25, -9);
        assert_int("status", "t_integer", status, JSON_ERR_CHECKFAIL);
        break;

    case 26:
        status = json_read_object(json_str25b, json_attrs_25, NULL);
        assert_int("mode", "t_integer", i25, -9);
        assert_int("status", "t_integer", status, JSON_ERR_CHECKFAIL);
        break;

    case 27:
        status = json_read_object(json_str25c, json_attrs_25, NULL);
        assert_int("mode", "t_integer", i25, -9);
        assert_int("status", "t_integer", status, JSON_ERR_CHECKFAIL);
        break;

    case 28:
        status = json_read_object(json_str25d, json_attrs_25, NULL);
        assert_int("mode", "t_integer", i25, -9);
        assert_int("status", "t_integer", status, JSON_ERR_CHECKFAIL);
        break;

    // check strings "foob" and "fooba" --  should return JSON_ERR_STRLONG (7)
    case 29:
        status = json_read_object(json_str25e, json_attrs_25, NULL);
        assert_int("mode", "t_integer", i25, -9);
        assert_int("status", "t_integer", status, JSON_ERR_STRLONG);
        break;

    case 30:
        status = json_read_object(json_str25f, json_attrs_25, NULL);
        assert_int("mode", "t_integer", i25, -9);
        assert_int("status", "t_integer", status, JSON_ERR_STRLONG);
        break;

    case 31: // Check string "TPV" -- should return success (0)
        status = json_read_object(json_str25t, json_attrs_25, NULL);
        assert_int("mode", "t_integer", i25, 3);
        assert_int("status", "t_integer", status, 0);
        break;

    case 32: // Check that whitespace-only JSON returns JSON_ERR_EMPTY (25)
        status = json_read_object(str32, json_attrs_25, NULL);
        assert_int("status", "t_integer", status, JSON_ERR_EMPTY);
        break;

#define MAXTEST 32

    default:
        (void)fputs("Unknown test number\n", stderr);
        exit(EXIT_FAILURE);
    }
}

int main(int argc UNUSED, char *argv[]UNUSED)
{
    int option;
    int individual = 0;

    while ((option = getopt(argc, argv, "D:hn:V?")) != -1) {
        switch (option) {
        case 'D':
            debug = atoi(optarg);
            gps_enable_debug(debug, stdout);
            break;
        case 'n':
            individual = atoi(optarg);
            break;
        case '?':
        case 'h':
        default:
            (void)fprintf(stderr,
                        "usage: %s [-D lvl] [-n tst] [-V]\n"
                        "       -D lvl      set debug level\n"
                        "       -n tst      run only test tst\n"
                        "       -V          Print version and exit\n",
                        argv[0]);
            exit(EXIT_FAILURE);
        case 'V':
            (void)fprintf(stderr, "%s: %s (revision %s)\n",
                          argv[0], VERSION, REVISION);
            exit(EXIT_SUCCESS);
        }
    }

    (void)fprintf(stderr, "JSON unit tests\n");

    if (individual)
        jsontest(individual);
    else {
        int i;
        for (i = 1; i <= MAXTEST; i++) {
            jsontest(i);
        }
    }

    (void)fprintf(stderr, "succeeded.\n");

    exit(EXIT_SUCCESS);
}

/* end */
// vim: set expandtab shiftwidth=4
