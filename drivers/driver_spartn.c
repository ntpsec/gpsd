/*****************************************************************************

This is a stub  decoder for SPARTN Version 2 protocol.

The protocol does not play nice with other protocols.   Reliable
Packet detection is impossible whern miced with other protocols.

It is disabled by default.

This file is Copyright by the GPSD project
SPDX-License-Identifier: BSD-2-clause

*****************************************************************************/

#include "../include/gpsd_config.h"  // must be before all includes

#include "../include/gpsd.h"
#include "../include/bits.h"
#include "../include/crc24q.h"       // for crc24q_check()

#define ugrab(width)    (bitcount += width, ubits(buf, \
                         bitcount - width, width, false))
#define sgrab(width)    (bitcount += width, sbits(buf,  \
                         bitcount - width, width, false))

static const struct vlist_t vspartn_crc_type[] = {
    {0, "CRC-8-CCITT"},
    {1, "CRC-16-CCITT"},
    {2, "CRC-24-Radix-64"},
    {3, "CRC-32-CCITT"},
};

static const struct vlist_t vspartn_mtype[] = {
    {0, "Orbit"},
    {1, "HPAC"},
    {2, "GAD"},
    {3, "BDS"},
    {4, "QZSS"},
};

static const struct vlist_t vspartn_mstype[] = {
    {0, "GPS"},
    {1, "GLO"},
    {2, "GAL"},
    {3, "BDS"},
};

static const struct vlist_t vspartn_m120stype[] = {
    {0,  "In-house"},
    {1,  "u-blox"},
    {2,  "Swift"},
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
    unsigned long time_tag;
    unsigned sol_ID;
    unsigned sol_proc_ID;
    unsigned enc_ID = 0;
    unsigned enc_seq_num = 0;
    unsigned ai = 0;
    unsigned eal = 0;
    unsigned pay_offset;        // offset of payload

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
        ai = ugrab(3);
        eal = ugrab(3);
    }
    // payload follows.
    pay_offset = bitcount / 8;  // should be even bytes Prolly 13 or 15.

    // assume, for now, no Embedded Auth data

    // 1 to 4 CRC bytes, usually 3
    //  CRC is all bytes after the leader 's'.
    if (2 != crc_type) {
        // we only know crc-24-radix64
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "SPARTN: unsupported CRC  type %u\n", crc_type);
    } else if (!crc24q_check(&session->lexer.outbuffer[1],
                       pay_offset + pay_length + 3)) {
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "SPARTN: crc24 fail %x vs %02x %02x %02x \n "
                 "SPARTN: pay_offset %x pay-length %02x\n",
                 crc24q_hash(&session->lexer.outbuffer[1],
                             pay_offset + pay_length + 3),
                 session->lexer.outbuffer[pay_offset + pay_length + 1],
                 session->lexer.outbuffer[pay_offset + pay_length + 2],
                 session->lexer.outbuffer[pay_offset + pay_length + 3],
                 pay_offset, pay_length);
    }

    GPSD_LOG(LOG_PROG, &session->context->errout,
             "SPARTN: mtype %u msubtype %u len %u eaf %u crct %u fcrc %u "
             "tt %u tt %llu sol_ID %u, sol_proc_ID %u\n",
             msg_type, msg_subtype, pay_length, eaf, crc_type, frame_crc,
             time_tag_type, (unsigned long long)time_tag, sol_ID, sol_proc_ID);

    if (LOG_IO <= session->context->errout.debug) {
        const char *msg_subtype_s;

        msg_subtype_s = "TBD";
        switch (msg_type) {
        case 0:
            FALLTHROUGH
        case 1:
            msg_subtype_s = val2str(msg_subtype, vspartn_mstype);
            break;
        case 2:
            // GAD
            if (0 == msg_subtype) {
                msg_subtype_s = "GAD";
            }
            break;
        case 3:
            // BPAC
            if (0 == msg_subtype) {
                msg_subtype_s = "BPAC Polynomial";
            }
            break;
        case 120:
            // Prorietary
            msg_subtype_s = val2str(msg_subtype, vspartn_m120stype);
            break;
        default:
            break;
        }

        GPSD_LOG(LOG_IO, &session->context->errout,
                 "SPARTN: mtype %s msubtype %s crct %s\n",
                 val2str(msg_type, vspartn_mtype),
                 msg_subtype_s,
                 val2str(crc_type, vspartn_crc_type));
    }

    if (1 == eaf) {
        GPSD_LOG(LOG_PROG, &session->context->errout,
                 "SPARTN: enc_ID %u enc_seq_num %u ai %u eal %u\n",
                 enc_ID, enc_seq_num, ai, eal);
    }

    mask |= SPARTN_SET;
    return mask;
}

// vim: set expandtab shiftwidth=4
