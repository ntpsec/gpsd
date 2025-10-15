/*****************************************************************************

This is a stub  decoder for SPARTN Version 2 protocol.

The protocol does not play nice with other protocols.   Reliable
Packet detection is impossible whern miced with other protocols.

It is disabled by default.

This file is Copyright by the GPSD project
SPDX-License-Identifier: BSD-2-clause

*****************************************************************************/

#include "../include/gpsd_config.h"  // must be before all includes

#include <string.h>

#include "../include/gpsd.h"
#include "../include/bits.h"

#define ugrab(width)    (bitcount += width, ubits(buf, \
                         bitcount - width, width, false))
#define sgrab(width)    (bitcount += width, sbits(buf,  \
                         bitcount - width, width, false))

static const struct vlist_t vspartn_mtype[] = {
        {0, "Orbit"},
        {1, "HPAC"},
        {2, "GAD"},
        {3, "BDS"},
};

static const struct vlist_t vspartn_mstype[] = {
        {0, "GPS"},
        {1, "GLO"},
        {2, "GAL"},
        {3, "BDS"},
};

/* stub decoder for SPARTN
 *
 * Return: void
 */
gps_mask_t spartn_parse(struct gps_device_t *session)
{
    static gps_mask_t mask = ONLINE_SET;
    const unsigned char *buf =  session->lexer.outbuffer;
    int bitcount = 0;
    unsigned preamble;
    unsigned msg_type, msg_subtype;
    unsigned pay_length;
    unsigned eaf;
    unsigned crc_type;
    unsigned frame_crc;
    unsigned time_tag_type;
    uint64_t time_tag;
    unsigned sol_ID;
    unsigned sol_proc_ID;
    unsigned enc_ID;
    unsigned enc_seq_num;
    unsigned eal;

    preamble = ugrab(8);
    if (0x73 != preamble) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "SPARTN: Invalid preamble x%x, s/b 0x73\n", preamble);
        return mask;
    }
    msg_type = ugrab(7);
    pay_length = ugrab(10);
    eaf = ugrab(1);
    crc_type = ugrab(2);
    frame_crc = ugrab(4);
    msg_subtype = ugrab(4);
    time_tag_type = ugrab(1);
    if (0 == time_tag_type) {
        time_tag = ugrab(16);
    } else {
        time_tag = ugrab(32);
    }
    sol_ID = ugrab(7);
    sol_proc_ID = ugrab(4);
    if (1 == eaf) {
        enc_ID = ugrab(4);
        enc_seq_num = ugrab(6);
        eal = ugrab(3);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "SPARTN: mtype %u msubtype %u len %u eaf %u crct %u fcrc %u "
             "tt %u tt %llu sol_ID %u, sol_proc_ID %u\n",
             msg_type, msg_subtype, pay_length, eaf, crc_type, frame_crc,
             time_tag_type, (unsigned long long)time_tag, sol_ID, sol_proc_ID);

    GPSD_LOG(LOG_IO, &session->context->errout,
             "SPARTN: mtype %s msubtype %s\n",
             val2str(msg_type, vspartn_mtype),
             val2str(msg_type, vspartn_mstype));

    if (1 == eaf) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "SPARTN: enc_ID %u enc_seq_num %u eal %u\n",
                 enc_ID, enc_seq_num, eal);
    }

    return mask;
}

// vim: set expandtab shiftwidth=4
