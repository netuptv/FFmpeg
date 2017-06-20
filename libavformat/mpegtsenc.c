/*
 * MPEG-2 transport stream (aka DVB) muxer
 * Copyright (c) 2003 Fabrice Bellard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/avassert.h"
#include "libavutil/bswap.h"
#include "libavutil/crc.h"
#include "libavutil/dict.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

#include "libavcodec/internal.h"

#include "avformat.h"
#include "avio_internal.h"
#include "internal.h"
#include "mpegts.h"

//FIXME: wrap all pthread code with ifdefs?
#if HAVE_PTHREADS
#include <pthread.h>
#endif

static const int64_t TSI_MAX_PCR_ADJUSTMENT = 5 * 90000LL / 1000; // FIXME: move

#define PCR_TIME_BASE 27000000

/* write DVB SI sections */

#define DVB_PRIVATE_NETWORK_START 0xff01

/*********************************************/
/* mpegts section writer */

typedef struct MpegTSSection {
    int pid;
    int cc;
    int discontinuity;
    void (*write_packet)(struct MpegTSSection *s, const uint8_t *packet);
    void *opaque;
} MpegTSSection;

typedef struct MpegTSService {
    MpegTSSection pmt; /* MPEG-2 PMT table context */
    int sid;           /* service ID */
    char *name;
    char *provider_name;
    int pcr_pid;
    int pcr_packet_count;
    int pcr_packet_period;
    AVProgram *program;

    int64_t last_pcr;
    int64_t last_pcr_bytes;
    int64_t last_pcr_remainder; //TODO: rename
    int64_t tsi_dbg_prev_pcr_adjustment;
    int is_buffered;
    int64_t max_dts_buffered; // TODO: local var?
    int64_t min_time; // TODO: local var?
    int64_t time_offset; //streamtime_offset?
} MpegTSService;

// service_type values as defined in ETSI 300 468
enum {
    MPEGTS_SERVICE_TYPE_DIGITAL_TV                   = 0x01,
    MPEGTS_SERVICE_TYPE_DIGITAL_RADIO                = 0x02,
    MPEGTS_SERVICE_TYPE_TELETEXT                     = 0x03,
    MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_RADIO = 0x0A,
    MPEGTS_SERVICE_TYPE_MPEG2_DIGITAL_HDTV           = 0x11,
    MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_SDTV  = 0x16,
    MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_HDTV  = 0x19,
    MPEGTS_SERVICE_TYPE_HEVC_DIGITAL_HDTV            = 0x1F,
};
typedef struct MpegTSWrite {
    const AVClass *av_class;
    MpegTSSection pat; /* MPEG-2 PAT table */
    MpegTSSection sdt; /* MPEG-2 SDT table context */
    MpegTSService **services;
    int sdt_packet_count;
    int sdt_packet_period;
    int pat_packet_count;
    int pat_packet_period;
    int nb_services;
    int onid;
    int tsid;
    int64_t first_pcr;
    int mux_rate; ///< set to 1 when VBR
    int pes_payload_size;

    int transport_stream_id;
    int original_network_id;
    int service_id;
    int service_type;

    int pmt_start_pid;
    int start_pid;
    int m2ts_mode;

    int reemit_pat_pmt; // backward compatibility

    int pcr_period;
#define MPEGTS_FLAG_REEMIT_PAT_PMT  0x01
#define MPEGTS_FLAG_AAC_LATM        0x02
#define MPEGTS_FLAG_PAT_PMT_AT_FRAMES           0x04
#define MPEGTS_FLAG_SYSTEM_B        0x08
#define MPEGTS_FLAG_DISCONT         0x10
    int flags;
    int copyts;
    int tables_version;
    double pat_period;
    double sdt_period;
    int64_t last_pat_ts;
    int64_t last_sdt_ts;

    int omit_video_pes_length;

    // FIXME: rename?
    int64_t last_pat;
    int64_t last_sdt;
    int64_t rate_last_streamtime;
    int64_t rate_last_bytes;
    int64_t ts_stream_time;

#if HAVE_PTHREADS
    pthread_t tsi_thread;
    pthread_mutex_t tsi_mutex;
    pthread_cond_t tsi_cond;
    int tsi_thread_exit;
    int tsi_is_realtime;
#endif
    int64_t tsi_dbg_mpegts_write_pes_enter_time;
    int64_t tsi_dbg_buffer_report_time;
} MpegTSWrite;

/* a PES packet header is generated every DEFAULT_PES_HEADER_FREQ packets */
#define DEFAULT_PES_HEADER_FREQ  16
#define DEFAULT_PES_PAYLOAD_SIZE ((DEFAULT_PES_HEADER_FREQ - 1) * 184 + 170)

/* The section length is 12 bits. The first 2 are set to 0, the remaining
 * 10 bits should not exceed 1021. */
#define SECTION_LENGTH 1020

/* NOTE: 4 bytes must be left at the end for the crc32 */
static void mpegts_write_section(MpegTSSection *s, uint8_t *buf, int len)
{
    unsigned int crc;
    unsigned char packet[TS_PACKET_SIZE];
    const unsigned char *buf_ptr;
    unsigned char *q;
    int first, b, len1, left;

    crc = av_bswap32(av_crc(av_crc_get_table(AV_CRC_32_IEEE),
                            -1, buf, len - 4));

    buf[len - 4] = (crc >> 24) & 0xff;
    buf[len - 3] = (crc >> 16) & 0xff;
    buf[len - 2] = (crc >>  8) & 0xff;
    buf[len - 1] =  crc        & 0xff;

    /* send each packet */
    buf_ptr = buf;
    while (len > 0) {
        first = buf == buf_ptr;
        q     = packet;
        *q++  = 0x47;
        b     = s->pid >> 8;
        if (first)
            b |= 0x40;
        *q++  = b;
        *q++  = s->pid;
        s->cc = s->cc + 1 & 0xf;
        *q++  = 0x10 | s->cc;
        if (s->discontinuity) {
            q[-1] |= 0x20;
            *q++ = 1;
            *q++ = 0x80;
            s->discontinuity = 0;
        }
        if (first)
            *q++ = 0; /* 0 offset */
        len1 = TS_PACKET_SIZE - (q - packet);
        if (len1 > len)
            len1 = len;
        memcpy(q, buf_ptr, len1);
        q += len1;
        /* add known padding data */
        left = TS_PACKET_SIZE - (q - packet);
        if (left > 0)
            memset(q, 0xff, left);

        s->write_packet(s, packet);

        buf_ptr += len1;
        len     -= len1;
    }
}

static inline void put16(uint8_t **q_ptr, int val)
{
    uint8_t *q;
    q      = *q_ptr;
    *q++   = val >> 8;
    *q++   = val;
    *q_ptr = q;
}

static int mpegts_write_section1(MpegTSSection *s, int tid, int id,
                                 int version, int sec_num, int last_sec_num,
                                 uint8_t *buf, int len)
{
    uint8_t section[1024], *q;
    unsigned int tot_len;
    /* reserved_future_use field must be set to 1 for SDT */
    unsigned int flags = tid == SDT_TID ? 0xf000 : 0xb000;

    tot_len = 3 + 5 + len + 4;
    /* check if not too big */
    if (tot_len > 1024)
        return AVERROR_INVALIDDATA;

    q    = section;
    *q++ = tid;
    put16(&q, flags | (len + 5 + 4)); /* 5 byte header + 4 byte CRC */
    put16(&q, id);
    *q++ = 0xc1 | (version << 1); /* current_next_indicator = 1 */
    *q++ = sec_num;
    *q++ = last_sec_num;
    memcpy(q, buf, len);

    mpegts_write_section(s, section, tot_len);
    return 0;
}

/*********************************************/
/* mpegts writer */

#define DEFAULT_PROVIDER_NAME   "FFmpeg"
#define DEFAULT_SERVICE_NAME    "Service01"

/* we retransmit the SI info at this rate */
#define SDT_RETRANS_TIME 500
#define PAT_RETRANS_TIME 100
#define PCR_RETRANS_TIME 20

typedef struct MpegTSPesPacket {
    uint8_t *payload;
    int64_t pts;
    int64_t dts;
    int payload_size;
    int key;
    int stream_id;
}MpegTSPesPacket;

typedef struct MpegTSWriteStream {
    struct MpegTSService *service;
    int pid; /* stream associated pid */
    int cc;
    int discontinuity;
    int payload_size;
    int first_pts_check; ///< first pts check needed
    int prev_payload_key;
    int64_t payload_pts;
    int64_t payload_dts;
    int payload_flags;
    uint8_t *payload;
    AVFormatContext *amux;
    AVRational user_tb;

    /* For Opus */
    int opus_queued_samples;
    int opus_pending_trim_start;
    //
    MpegTSPesPacket packets[1024];
    int64_t packets_head;
    int64_t packets_tail;
    int64_t service_time;
    int64_t buffer_duration_dts;
    int64_t buffer_bytes;
    int64_t buffer_packets;

    int64_t packet_start_time;
    int64_t packet_end_time;
    int packet_consumed_bytes;
} MpegTSWriteStream;

static void mpegts_write_pat(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService *service;
    uint8_t data[SECTION_LENGTH], *q;
    int i;

    q = data;
    for (i = 0; i < ts->nb_services; i++) {
        service = ts->services[i];
        put16(&q, service->sid);
        put16(&q, 0xe000 | service->pmt.pid);
    }
    mpegts_write_section1(&ts->pat, PAT_TID, ts->tsid, ts->tables_version, 0, 0,
                          data, q - data);
}

/* NOTE: !str is accepted for an empty string */
static void putstr8(uint8_t **q_ptr, const char *str, int write_len)
{
    uint8_t *q;
    int len;

    q = *q_ptr;
    if (!str)
        len = 0;
    else
        len = strlen(str);
    if (write_len)
        *q++ = len;
    memcpy(q, str, len);
    q     += len;
    *q_ptr = q;
}

static int mpegts_write_pmt(AVFormatContext *s, MpegTSService *service)
{
    MpegTSWrite *ts = s->priv_data;
    uint8_t data[SECTION_LENGTH], *q, *desc_length_ptr, *program_info_length_ptr;
    int val, stream_type, i, err = 0;

    q = data;
    put16(&q, 0xe000 | service->pcr_pid);

    program_info_length_ptr = q;
    q += 2; /* patched after */

    /* put program info here */

    val = 0xf000 | (q - program_info_length_ptr - 2);
    program_info_length_ptr[0] = val >> 8;
    program_info_length_ptr[1] = val;

    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        AVDictionaryEntry *lang = av_dict_get(st->metadata, "language", NULL, 0);

        if (s->nb_programs) {
            int k, found = 0;
            AVProgram *program = service->program;

            for (k = 0; k < program->nb_stream_indexes; k++)
                if (program->stream_index[k] == i) {
                    found = 1;
                    break;
                }

            if (!found)
                continue;
        }

        if (q - data > SECTION_LENGTH - 32) {
            err = 1;
            break;
        }
        switch (st->codecpar->codec_id) {
        case AV_CODEC_ID_MPEG1VIDEO:
        case AV_CODEC_ID_MPEG2VIDEO:
            stream_type = STREAM_TYPE_VIDEO_MPEG2;
            break;
        case AV_CODEC_ID_MPEG4:
            stream_type = STREAM_TYPE_VIDEO_MPEG4;
            break;
        case AV_CODEC_ID_H264:
            stream_type = STREAM_TYPE_VIDEO_H264;
            break;
        case AV_CODEC_ID_HEVC:
            stream_type = STREAM_TYPE_VIDEO_HEVC;
            break;
        case AV_CODEC_ID_CAVS:
            stream_type = STREAM_TYPE_VIDEO_CAVS;
            break;
        case AV_CODEC_ID_DIRAC:
            stream_type = STREAM_TYPE_VIDEO_DIRAC;
            break;
        case AV_CODEC_ID_VC1:
            stream_type = STREAM_TYPE_VIDEO_VC1;
            break;
        case AV_CODEC_ID_MP2:
        case AV_CODEC_ID_MP3:
            if (   st->codecpar->sample_rate > 0
                && st->codecpar->sample_rate < 32000) {
                stream_type = STREAM_TYPE_AUDIO_MPEG2;
            } else {
                stream_type = STREAM_TYPE_AUDIO_MPEG1;
            }
            break;
        case AV_CODEC_ID_AAC:
            stream_type = (ts->flags & MPEGTS_FLAG_AAC_LATM)
                          ? STREAM_TYPE_AUDIO_AAC_LATM
                          : STREAM_TYPE_AUDIO_AAC;
            break;
        case AV_CODEC_ID_AAC_LATM:
            stream_type = STREAM_TYPE_AUDIO_AAC_LATM;
            break;
        case AV_CODEC_ID_AC3:
            stream_type = (ts->flags & MPEGTS_FLAG_SYSTEM_B)
                          ? STREAM_TYPE_PRIVATE_DATA
                          : STREAM_TYPE_AUDIO_AC3;
            break;
        case AV_CODEC_ID_EAC3:
            stream_type = (ts->flags & MPEGTS_FLAG_SYSTEM_B)
                          ? STREAM_TYPE_PRIVATE_DATA
                          : STREAM_TYPE_AUDIO_EAC3;
            break;
        case AV_CODEC_ID_DTS:
            stream_type = STREAM_TYPE_AUDIO_DTS;
            break;
        case AV_CODEC_ID_TRUEHD:
            stream_type = STREAM_TYPE_AUDIO_TRUEHD;
            break;
        case AV_CODEC_ID_OPUS:
            stream_type = STREAM_TYPE_PRIVATE_DATA;
            break;
        case AV_CODEC_ID_TIMED_ID3:
            stream_type = STREAM_TYPE_METADATA;
            break;
        default:
            stream_type = STREAM_TYPE_PRIVATE_DATA;
            break;
        }

        *q++ = stream_type;
        put16(&q, 0xe000 | ts_st->pid);
        desc_length_ptr = q;
        q += 2; /* patched after */

        /* write optional descriptors here */
        switch (st->codecpar->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            if (st->codecpar->codec_id==AV_CODEC_ID_AC3 && (ts->flags & MPEGTS_FLAG_SYSTEM_B)) {
                *q++=0x6a; // AC3 descriptor see A038 DVB SI
                *q++=1; // 1 byte, all flags sets to 0
                *q++=0; // omit all fields...
            }
            if (st->codecpar->codec_id==AV_CODEC_ID_EAC3 && (ts->flags & MPEGTS_FLAG_SYSTEM_B)) {
                *q++=0x7a; // EAC3 descriptor see A038 DVB SI
                *q++=1; // 1 byte, all flags sets to 0
                *q++=0; // omit all fields...
            }
            if (st->codecpar->codec_id==AV_CODEC_ID_S302M) {
                *q++ = 0x05; /* MPEG-2 registration descriptor*/
                *q++ = 4;
                *q++ = 'B';
                *q++ = 'S';
                *q++ = 'S';
                *q++ = 'D';
            }
            if (st->codecpar->codec_id==AV_CODEC_ID_OPUS) {
                /* 6 bytes registration descriptor, 4 bytes Opus audio descriptor */
                if (q - data > SECTION_LENGTH - 6 - 4) {
                    err = 1;
                    break;
                }

                *q++ = 0x05; /* MPEG-2 registration descriptor*/
                *q++ = 4;
                *q++ = 'O';
                *q++ = 'p';
                *q++ = 'u';
                *q++ = 's';

                *q++ = 0x7f; /* DVB extension descriptor */
                *q++ = 2;
                *q++ = 0x80;

                if (st->codecpar->extradata && st->codecpar->extradata_size >= 19) {
                    if (st->codecpar->extradata[18] == 0 && st->codecpar->channels <= 2) {
                        /* RTP mapping family */
                        *q++ = st->codecpar->channels;
                    } else if (st->codecpar->extradata[18] == 1 && st->codecpar->channels <= 8 &&
                               st->codecpar->extradata_size >= 21 + st->codecpar->channels) {
                        static const uint8_t coupled_stream_counts[9] = {
                            1, 0, 1, 1, 2, 2, 2, 3, 3
                        };
                        static const uint8_t channel_map_a[8][8] = {
                            {0},
                            {0, 1},
                            {0, 2, 1},
                            {0, 1, 2, 3},
                            {0, 4, 1, 2, 3},
                            {0, 4, 1, 2, 3, 5},
                            {0, 4, 1, 2, 3, 5, 6},
                            {0, 6, 1, 2, 3, 4, 5, 7},
                        };
                        static const uint8_t channel_map_b[8][8] = {
                            {0},
                            {0, 1},
                            {0, 1, 2},
                            {0, 1, 2, 3},
                            {0, 1, 2, 3, 4},
                            {0, 1, 2, 3, 4, 5},
                            {0, 1, 2, 3, 4, 5, 6},
                            {0, 1, 2, 3, 4, 5, 6, 7},
                        };
                        /* Vorbis mapping family */

                        if (st->codecpar->extradata[19] == st->codecpar->channels - coupled_stream_counts[st->codecpar->channels] &&
                            st->codecpar->extradata[20] == coupled_stream_counts[st->codecpar->channels] &&
                            memcmp(&st->codecpar->extradata[21], channel_map_a[st->codecpar->channels-1], st->codecpar->channels) == 0) {
                            *q++ = st->codecpar->channels;
                        } else if (st->codecpar->channels >= 2 && st->codecpar->extradata[19] == st->codecpar->channels &&
                                   st->codecpar->extradata[20] == 0 &&
                                   memcmp(&st->codecpar->extradata[21], channel_map_b[st->codecpar->channels-1], st->codecpar->channels) == 0) {
                            *q++ = st->codecpar->channels | 0x80;
                        } else {
                            /* Unsupported, could write an extended descriptor here */
                            av_log(s, AV_LOG_ERROR, "Unsupported Opus Vorbis-style channel mapping");
                            *q++ = 0xff;
                        }
                    } else {
                        /* Unsupported */
                        av_log(s, AV_LOG_ERROR, "Unsupported Opus channel mapping for family %d", st->codecpar->extradata[18]);
                        *q++ = 0xff;
                    }
                } else if (st->codecpar->channels <= 2) {
                    /* Assume RTP mapping family */
                    *q++ = st->codecpar->channels;
                } else {
                    /* Unsupported */
                    av_log(s, AV_LOG_ERROR, "Unsupported Opus channel mapping");
                    *q++ = 0xff;
                }
            }

            if (lang) {
                char *p;
                char *next = lang->value;
                uint8_t *len_ptr;

                *q++     = 0x0a; /* ISO 639 language descriptor */
                len_ptr  = q++;
                *len_ptr = 0;

                for (p = lang->value; next && *len_ptr < 255 / 4 * 4; p = next + 1) {
                    if (q - data > SECTION_LENGTH - 4) {
                        err = 1;
                        break;
                    }
                    next = strchr(p, ',');
                    if (strlen(p) != 3 && (!next || next != p + 3))
                        continue; /* not a 3-letter code */

                    *q++ = *p++;
                    *q++ = *p++;
                    *q++ = *p++;

                    if (st->disposition & AV_DISPOSITION_CLEAN_EFFECTS)
                        *q++ = 0x01;
                    else if (st->disposition & AV_DISPOSITION_HEARING_IMPAIRED)
                        *q++ = 0x02;
                    else if (st->disposition & AV_DISPOSITION_VISUAL_IMPAIRED)
                        *q++ = 0x03;
                    else
                        *q++ = 0; /* undefined type */

                    *len_ptr += 4;
                }

                if (*len_ptr == 0)
                    q -= 2; /* no language codes were written */
            }
            break;
        case AVMEDIA_TYPE_SUBTITLE:
        {
           const char default_language[] = "und";
           const char *language = lang && strlen(lang->value) >= 3 ? lang->value : default_language;

           if (st->codecpar->codec_id == AV_CODEC_ID_DVB_SUBTITLE) {
               uint8_t *len_ptr;
               int extradata_copied = 0;

               *q++ = 0x59; /* subtitling_descriptor */
               len_ptr = q++;

               while (strlen(language) >= 3) {
                   if (sizeof(data) - (q - data) < 8) { /* 8 bytes per DVB subtitle substream data */
                       err = 1;
                       break;
                   }
                   *q++ = *language++;
                   *q++ = *language++;
                   *q++ = *language++;
                   /* Skip comma */
                   if (*language != '\0')
                       language++;

                   if (st->codecpar->extradata_size - extradata_copied >= 5) {
                       *q++ = st->codecpar->extradata[extradata_copied + 4]; /* subtitling_type */
                       memcpy(q, st->codecpar->extradata + extradata_copied, 4); /* composition_page_id and ancillary_page_id */
                       extradata_copied += 5;
                       q += 4;
                   } else {
                       /* subtitling_type:
                        * 0x10 - normal with no monitor aspect ratio criticality
                        * 0x20 - for the hard of hearing with no monitor aspect ratio criticality */
                       *q++ = (st->disposition & AV_DISPOSITION_HEARING_IMPAIRED) ? 0x20 : 0x10;
                       if ((st->codecpar->extradata_size == 4) && (extradata_copied == 0)) {
                           /* support of old 4-byte extradata format */
                           memcpy(q, st->codecpar->extradata, 4); /* composition_page_id and ancillary_page_id */
                           extradata_copied += 4;
                           q += 4;
                       } else {
                           put16(&q, 1); /* composition_page_id */
                           put16(&q, 1); /* ancillary_page_id */
                       }
                   }
               }

               *len_ptr = q - len_ptr - 1;
           } else if (st->codecpar->codec_id == AV_CODEC_ID_DVB_TELETEXT) {
               uint8_t *len_ptr = NULL;
               int extradata_copied = 0;

               /* The descriptor tag. teletext_descriptor */
               *q++ = 0x56;
               len_ptr = q++;

               while (strlen(language) >= 3 && q - data < sizeof(data) - 6) {
                   *q++ = *language++;
                   *q++ = *language++;
                   *q++ = *language++;
                   /* Skip comma */
                   if (*language != '\0')
                       language++;

                   if (st->codecpar->extradata_size - 1 > extradata_copied) {
                       memcpy(q, st->codecpar->extradata + extradata_copied, 2);
                       extradata_copied += 2;
                       q += 2;
                   } else {
                       /* The Teletext descriptor:
                        * teletext_type: This 5-bit field indicates the type of Teletext page indicated. (0x01 Initial Teletext page)
                        * teletext_magazine_number: This is a 3-bit field which identifies the magazine number.
                        * teletext_page_number: This is an 8-bit field giving two 4-bit hex digits identifying the page number. */
                       *q++ = 0x08;
                       *q++ = 0x00;
                   }
               }

               *len_ptr = q - len_ptr - 1;
            }
        }
        break;
        case AVMEDIA_TYPE_VIDEO:
            if (stream_type == STREAM_TYPE_VIDEO_DIRAC) {
                *q++ = 0x05; /*MPEG-2 registration descriptor*/
                *q++ = 4;
                *q++ = 'd';
                *q++ = 'r';
                *q++ = 'a';
                *q++ = 'c';
            } else if (stream_type == STREAM_TYPE_VIDEO_VC1) {
                *q++ = 0x05; /*MPEG-2 registration descriptor*/
                *q++ = 4;
                *q++ = 'V';
                *q++ = 'C';
                *q++ = '-';
                *q++ = '1';
            }
            break;
        case AVMEDIA_TYPE_DATA:
            if (st->codecpar->codec_id == AV_CODEC_ID_SMPTE_KLV) {
                *q++ = 0x05; /* MPEG-2 registration descriptor */
                *q++ = 4;
                *q++ = 'K';
                *q++ = 'L';
                *q++ = 'V';
                *q++ = 'A';
            } else if (st->codecpar->codec_id == AV_CODEC_ID_TIMED_ID3) {
                const char *tag = "ID3 ";
                *q++ = 0x26; /* metadata descriptor */
                *q++ = 13;
                put16(&q, 0xffff);    /* metadata application format */
                putstr8(&q, tag, 0);
                *q++ = 0xff;        /* metadata format */
                putstr8(&q, tag, 0);
                *q++ = 0;            /* metadata service ID */
                *q++ = 0xF;          /* metadata_locator_record_flag|MPEG_carriage_flags|reserved */
            }
            break;
        }

        val = 0xf000 | (q - desc_length_ptr - 2);
        desc_length_ptr[0] = val >> 8;
        desc_length_ptr[1] = val;
    }

    if (err)
        av_log(s, AV_LOG_ERROR,
               "The PMT section cannot fit stream %d and all following streams.\n"
               "Try reducing the number of languages in the audio streams "
               "or the total number of streams.\n", i);

    mpegts_write_section1(&service->pmt, PMT_TID, service->sid, ts->tables_version, 0, 0,
                          data, q - data);
    return 0;
}

static void mpegts_write_sdt(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService *service;
    uint8_t data[SECTION_LENGTH], *q, *desc_list_len_ptr, *desc_len_ptr;
    int i, running_status, free_ca_mode, val;

    q = data;
    put16(&q, ts->onid);
    *q++ = 0xff;
    for (i = 0; i < ts->nb_services; i++) {
        service = ts->services[i];
        put16(&q, service->sid);
        *q++              = 0xfc | 0x00; /* currently no EIT info */
        desc_list_len_ptr = q;
        q                += 2;
        running_status    = 4; /* running */
        free_ca_mode      = 0;

        /* write only one descriptor for the service name and provider */
        *q++         = 0x48;
        desc_len_ptr = q;
        q++;
        *q++         = ts->service_type;
        putstr8(&q, service->provider_name, 1);
        putstr8(&q, service->name, 1);
        desc_len_ptr[0] = q - desc_len_ptr - 1;

        /* fill descriptor length */
        val = (running_status << 13) | (free_ca_mode << 12) |
              (q - desc_list_len_ptr - 2);
        desc_list_len_ptr[0] = val >> 8;
        desc_list_len_ptr[1] = val;
    }
    mpegts_write_section1(&ts->sdt, SDT_TID, ts->tsid, ts->tables_version, 0, 0,
                          data, q - data);
}

static MpegTSService *mpegts_add_service(MpegTSWrite *ts, int sid,
                                         const char *provider_name,
                                         const char *name)
{
    MpegTSService *service;

    service = av_mallocz(sizeof(MpegTSService));
    if (!service)
        return NULL;
    service->pmt.pid       = ts->pmt_start_pid + ts->nb_services;
    service->sid           = sid;
    service->pcr_pid       = 0x1fff;
    service->provider_name = av_strdup(provider_name);
    service->name          = av_strdup(name);
    if (!service->provider_name || !service->name)
        goto fail;
    if (av_dynarray_add_nofree(&ts->services, &ts->nb_services, service) < 0)
        goto fail;

    service->last_pcr = AV_NOPTS_VALUE;    // FIXME: move?
    service->time_offset = AV_NOPTS_VALUE;

    return service;
fail:
    av_freep(&service->provider_name);
    av_freep(&service->name);
    av_free(service);
    return NULL;
}

static int64_t get_pcr(const MpegTSWrite *ts, AVIOContext *pb)
{
    return av_rescale(avio_tell(pb) + 11, 8 * PCR_TIME_BASE, ts->mux_rate) +
           ts->first_pcr;
}

static void mpegts_prefix_m2ts_header(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    if (ts->m2ts_mode) {
        int64_t pcr = get_pcr(s->priv_data, s->pb);
        uint32_t tp_extra_header = pcr % 0x3fffffff;
        tp_extra_header = AV_RB32(&tp_extra_header);
        avio_write(s->pb, (unsigned char *) &tp_extra_header,
                   sizeof(tp_extra_header));
    }
}

static void section_write_packet(MpegTSSection *s, const uint8_t *packet)
{
    AVFormatContext *ctx = s->opaque;
    mpegts_prefix_m2ts_header(ctx);
    avio_write(ctx->pb, packet, TS_PACKET_SIZE);
}

static void* tsi_thread(void *opaque); // FIXME: ?

static int mpegts_init(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st;
    MpegTSService *service;
    AVStream *st, *pcr_st = NULL;
    AVDictionaryEntry *title, *provider;
    int i, j;
    const char *service_name;
    const char *provider_name;
    int *pids;
    int ret;

    if (s->max_delay < 0) /* Not set by the caller */
        s->max_delay = 0;

    // round up to a whole number of TS packets
    ts->pes_payload_size = (ts->pes_payload_size + 14 + 183) / 184 * 184 - 14;

    ts->tsid = ts->transport_stream_id;
    ts->onid = ts->original_network_id;
    if (!s->nb_programs) {
        /* allocate a single DVB service */
        title = av_dict_get(s->metadata, "service_name", NULL, 0);
        if (!title)
            title = av_dict_get(s->metadata, "title", NULL, 0);
        service_name  = title ? title->value : DEFAULT_SERVICE_NAME;
        provider      = av_dict_get(s->metadata, "service_provider", NULL, 0);
        provider_name = provider ? provider->value : DEFAULT_PROVIDER_NAME;
        service       = mpegts_add_service(ts, ts->service_id,
                                           provider_name, service_name);

        if (!service)
            return AVERROR(ENOMEM);

        service->pmt.write_packet = section_write_packet;
        service->pmt.opaque       = s;
        service->pmt.cc           = 15;
        service->pmt.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
    } else {
        for (i = 0; i < s->nb_programs; i++) {
            AVProgram *program = s->programs[i];
            title = av_dict_get(program->metadata, "service_name", NULL, 0);
            if (!title)
                title = av_dict_get(program->metadata, "title", NULL, 0);
            service_name  = title ? title->value : DEFAULT_SERVICE_NAME;
            provider      = av_dict_get(program->metadata, "service_provider", NULL, 0);
            provider_name = provider ? provider->value : DEFAULT_PROVIDER_NAME;
            service       = mpegts_add_service(ts, program->id,
                                               provider_name, service_name);

            if (!service)
                return AVERROR(ENOMEM);

            service->pmt.write_packet = section_write_packet;
            service->pmt.opaque       = s;
            service->pmt.cc           = 15;
            service->pmt.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
            service->program          = program;
        }
    }

    ts->pat.pid          = PAT_PID;
    /* Initialize at 15 so that it wraps and is equal to 0 for the
     * first packet we write. */
    ts->pat.cc           = 15;
    ts->pat.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
    ts->pat.write_packet = section_write_packet;
    ts->pat.opaque       = s;

    ts->sdt.pid          = SDT_PID;
    ts->sdt.cc           = 15;
    ts->sdt.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
    ts->sdt.write_packet = section_write_packet;
    ts->sdt.opaque       = s;

    pids = av_malloc_array(s->nb_streams, sizeof(*pids));
    if (!pids) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    /* assign pids to each stream */
    for (i = 0; i < s->nb_streams; i++) {
        AVProgram *program;
        st = s->streams[i];

        ts_st = av_mallocz(sizeof(MpegTSWriteStream));
        if (!ts_st) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        st->priv_data = ts_st;

        ts_st->service_time = AV_NOPTS_VALUE; // FIXME: check this
        ts_st->packet_start_time = AV_NOPTS_VALUE; //FIXME: needed?
        ts_st->packet_end_time = AV_NOPTS_VALUE; //FIXME: needed?

        ts_st->user_tb = st->time_base;
        avpriv_set_pts_info(st, 33, 1, 90000);

        ts_st->payload = av_mallocz(ts->pes_payload_size);
        if (!ts_st->payload) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        program = av_find_program_from_stream(s, NULL, i);
        if (program) {
            for (j = 0; j < ts->nb_services; j++) {
                if (ts->services[j]->program == program) {
                    service = ts->services[j];
                    break;
                }
            }
        }

        ts_st->service = service;
        /* MPEG pid values < 16 are reserved. Applications which set st->id in
         * this range are assigned a calculated pid. */
        if (st->id < 16) {
            ts_st->pid = ts->start_pid + i;
        } else if (st->id < 0x1FFF) {
            ts_st->pid = st->id;
        } else {
            av_log(s, AV_LOG_ERROR,
                   "Invalid stream id %d, must be less than 8191\n", st->id);
            ret = AVERROR(EINVAL);
            goto fail;
        }
        if (ts_st->pid == service->pmt.pid) {
            av_log(s, AV_LOG_ERROR, "Duplicate stream id %d\n", ts_st->pid);
            ret = AVERROR(EINVAL);
            goto fail;
        }
        for (j = 0; j < i; j++) {
            if (pids[j] == ts_st->pid) {
                av_log(s, AV_LOG_ERROR, "Duplicate stream id %d\n", ts_st->pid);
                ret = AVERROR(EINVAL);
                goto fail;
            }
        }
        pids[i]                = ts_st->pid;
        ts_st->payload_pts     = AV_NOPTS_VALUE;
        ts_st->payload_dts     = AV_NOPTS_VALUE;
        ts_st->first_pts_check = 1;
        ts_st->cc              = 15;
        ts_st->discontinuity   = ts->flags & MPEGTS_FLAG_DISCONT;
        /* update PCR pid by using the first video stream */
        if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
            service->pcr_pid == 0x1fff) {
            service->pcr_pid = ts_st->pid;
            pcr_st           = st;
        }
        if (st->codecpar->codec_id == AV_CODEC_ID_AAC &&
            st->codecpar->extradata_size > 0) {
            AVStream *ast;
            ts_st->amux = avformat_alloc_context();
            if (!ts_st->amux) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            ts_st->amux->oformat =
                av_guess_format((ts->flags & MPEGTS_FLAG_AAC_LATM) ? "latm" : "adts",
                                NULL, NULL);
            if (!ts_st->amux->oformat) {
                ret = AVERROR(EINVAL);
                goto fail;
            }
            if (!(ast = avformat_new_stream(ts_st->amux, NULL))) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            ret = avcodec_parameters_copy(ast->codecpar, st->codecpar);
            if (ret != 0)
                goto fail;
            ast->time_base = st->time_base;
            ret = avformat_write_header(ts_st->amux, NULL);
            if (ret < 0)
                goto fail;
        }
        if (st->codecpar->codec_id == AV_CODEC_ID_OPUS) {
            ts_st->opus_pending_trim_start = st->codecpar->initial_padding * 48000 / st->codecpar->sample_rate;
        }
    }

    av_freep(&pids);

    /* if no video stream, use the first stream as PCR */
    if (service->pcr_pid == 0x1fff && s->nb_streams > 0) {
        pcr_st           = s->streams[0];
        ts_st            = pcr_st->priv_data;
        service->pcr_pid = ts_st->pid;
    } else
        ts_st = pcr_st->priv_data;

    if (ts->mux_rate > 1) {
        service->pcr_packet_period = (int64_t)ts->mux_rate * ts->pcr_period /
                                     (TS_PACKET_SIZE * 8 * 1000);
        ts->sdt_packet_period      = (int64_t)ts->mux_rate * SDT_RETRANS_TIME /
                                     (TS_PACKET_SIZE * 8 * 1000);
        ts->pat_packet_period      = (int64_t)ts->mux_rate * PAT_RETRANS_TIME /
                                     (TS_PACKET_SIZE * 8 * 1000);

        if (ts->copyts < 1)
            ts->first_pcr = av_rescale(s->max_delay, PCR_TIME_BASE, AV_TIME_BASE);
    } else {
        /* Arbitrary values, PAT/PMT will also be written on video key frames */
        ts->sdt_packet_period = 200;
        ts->pat_packet_period = 40;
        if (pcr_st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            int frame_size = av_get_audio_frame_duration2(pcr_st->codecpar, 0);
            if (!frame_size) {
                av_log(s, AV_LOG_WARNING, "frame size not set\n");
                service->pcr_packet_period =
                    pcr_st->codecpar->sample_rate / (10 * 512);
            } else {
                service->pcr_packet_period =
                    pcr_st->codecpar->sample_rate / (10 * frame_size);
            }
        } else {
            // max delta PCR 0.1s
            // TODO: should be avg_frame_rate
            service->pcr_packet_period =
                ts_st->user_tb.den / (10 * ts_st->user_tb.num);
        }
        if (!service->pcr_packet_period)
            service->pcr_packet_period = 1;
    }

    ts->last_pat_ts = AV_NOPTS_VALUE;
    ts->last_sdt_ts = AV_NOPTS_VALUE;
    // The user specified a period, use only it
    if (ts->pat_period < INT_MAX/2) {
        ts->pat_packet_period = INT_MAX;
    }
    if (ts->sdt_period < INT_MAX/2) {
        ts->sdt_packet_period = INT_MAX;
    }

    // output a PCR as soon as possible
    service->pcr_packet_count = service->pcr_packet_period;
    ts->pat_packet_count      = ts->pat_packet_period - 1;
    ts->sdt_packet_count      = ts->sdt_packet_period - 1;

    if (ts->mux_rate == 1)
        av_log(s, AV_LOG_VERBOSE, "muxrate VBR, ");
    else
        av_log(s, AV_LOG_VERBOSE, "muxrate %d, ", ts->mux_rate);
    av_log(s, AV_LOG_VERBOSE,
           "pcr every %d pkts, sdt every %d, pat/pmt every %d pkts\n",
           service->pcr_packet_period,
           ts->sdt_packet_period, ts->pat_packet_period);

    if (ts->m2ts_mode == -1) {
        if (av_match_ext(s->filename, "m2ts")) {
            ts->m2ts_mode = 1;
        } else {
            ts->m2ts_mode = 0;
        }
    }

    // FIXME: move? error handling
    {
        ts->last_pat = AV_NOPTS_VALUE;
        ts->last_sdt = AV_NOPTS_VALUE;
        ts->rate_last_streamtime = AV_NOPTS_VALUE;
        ts->tsi_thread_exit = 0;
        ret = pthread_mutex_init(&ts->tsi_mutex, NULL);
        if (ret != 0) {
            //av_log(h, AV_LOG_ERROR, "pthread_mutex_init failed : %s\n", strerror(ret));
            goto fail;
        }
        ret = pthread_cond_init(&ts->tsi_cond, NULL);
        if (ret != 0) {
            //av_log(h, AV_LOG_ERROR, "pthread_cond_init failed : %s\n", strerror(ret));
            goto fail;
        }
        if (ts->tsi_is_realtime) {
            ret = pthread_create(&ts->tsi_thread, NULL, &tsi_thread, s);
            if (ret != 0) {
                //av_log(h, AV_LOG_ERROR, "pthread_create failed : %s\n", strerror(ret));
                goto fail;
            }
        }
    }

    return 0;

fail:
    av_freep(&pids);
    return ret;
}

/* send SDT, PAT and PMT tables regularly */
static void retransmit_si_info(AVFormatContext *s, int force_pat, int64_t dts)
{
    MpegTSWrite *ts = s->priv_data;
    int i;

    if (++ts->sdt_packet_count == ts->sdt_packet_period ||
        (dts != AV_NOPTS_VALUE && ts->last_sdt_ts == AV_NOPTS_VALUE) ||
        (dts != AV_NOPTS_VALUE && dts - ts->last_sdt_ts >= ts->sdt_period*90000.0)
    ) {
        ts->sdt_packet_count = 0;
        if (dts != AV_NOPTS_VALUE)
            ts->last_sdt_ts = FFMAX(dts, ts->last_sdt_ts);
        mpegts_write_sdt(s);
    }
    if (++ts->pat_packet_count == ts->pat_packet_period ||
        (dts != AV_NOPTS_VALUE && ts->last_pat_ts == AV_NOPTS_VALUE) ||
        (dts != AV_NOPTS_VALUE && dts - ts->last_pat_ts >= ts->pat_period*90000.0) ||
        force_pat) {
        ts->pat_packet_count = 0;
        if (dts != AV_NOPTS_VALUE)
            ts->last_pat_ts = FFMAX(dts, ts->last_pat_ts);
        mpegts_write_pat(s);
        for (i = 0; i < ts->nb_services; i++)
            mpegts_write_pmt(s, ts->services[i]);
    }
}

static int write_pcr_bits(uint8_t *buf, int64_t pcr)
{
    int64_t pcr_low = pcr % 300, pcr_high = pcr / 300;

    *buf++ = pcr_high >> 25;
    *buf++ = pcr_high >> 17;
    *buf++ = pcr_high >>  9;
    *buf++ = pcr_high >>  1;
    *buf++ = pcr_high <<  7 | pcr_low >> 8 | 0x7e;
    *buf++ = pcr_low;

    return 6;
}

/* Write a single null transport stream packet */
static void mpegts_insert_null_packet(AVFormatContext *s)
{
    uint8_t *q;
    uint8_t buf[TS_PACKET_SIZE];

    q    = buf;
    *q++ = 0x47;
    *q++ = 0x00 | 0x1f;
    *q++ = 0xff;
    *q++ = 0x10;
    memset(q, 0x0FF, TS_PACKET_SIZE - (q - buf));
    mpegts_prefix_m2ts_header(s);
    avio_write(s->pb, buf, TS_PACKET_SIZE);
}

/* Write a single transport stream packet with a PCR and no payload */
static void mpegts_insert_pcr_only(AVFormatContext *s, AVStream *st)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = st->priv_data;
    uint8_t *q;
    uint8_t buf[TS_PACKET_SIZE];

    q    = buf;
    *q++ = 0x47;
    *q++ = ts_st->pid >> 8;
    *q++ = ts_st->pid;
    *q++ = 0x20 | ts_st->cc;   /* Adaptation only */
    /* Continuity Count field does not increment (see 13818-1 section 2.4.3.3) */
    *q++ = TS_PACKET_SIZE - 5; /* Adaptation Field Length */
    *q++ = 0x10;               /* Adaptation flags: PCR present */
    if (ts_st->discontinuity) {
        q[-1] |= 0x80;
        ts_st->discontinuity = 0;
    }

    /* PCR coded into 6 bytes */
    q += write_pcr_bits(q, get_pcr(ts, s->pb));

    /* stuffing bytes */
    memset(q, 0xFF, TS_PACKET_SIZE - (q - buf));
    mpegts_prefix_m2ts_header(s);
    avio_write(s->pb, buf, TS_PACKET_SIZE);
}

static void write_pts(uint8_t *q, int fourbits, int64_t pts)
{
    int val;

    val  = fourbits << 4 | (((pts >> 30) & 0x07) << 1) | 1;
    *q++ = val;
    val  = (((pts >> 15) & 0x7fff) << 1) | 1;
    *q++ = val >> 8;
    *q++ = val;
    val  = (((pts) & 0x7fff) << 1) | 1;
    *q++ = val >> 8;
    *q++ = val;
}

/* Set an adaptation field flag in an MPEG-TS packet*/
static void set_af_flag(uint8_t *pkt, int flag)
{
    // expect at least one flag to set
    av_assert0(flag);

    if ((pkt[3] & 0x20) == 0) {
        // no AF yet, set adaptation field flag
        pkt[3] |= 0x20;
        // 1 byte length, no flags
        pkt[4] = 1;
        pkt[5] = 0;
    }
    pkt[5] |= flag;
}

/* Extend the adaptation field by size bytes */
static void extend_af(uint8_t *pkt, int size)
{
    // expect already existing adaptation field
    av_assert0(pkt[3] & 0x20);
    pkt[4] += size;
}

/* Get a pointer to MPEG-TS payload (right after TS packet header) */
static uint8_t *get_ts_payload_start(uint8_t *pkt)
{
    if (pkt[3] & 0x20)
        return pkt + 5 + pkt[4];
    else
        return pkt + 4;
}

/* Add a PES header to the front of the payload, and segment into an integer
 * number of TS packets. The final TS packet is padded using an oversized
 * adaptation header to exactly fill the last TS packet.
 * NOTE: 'payload' contains a complete PES payload. */
static int mpegts_write_pes1(AVFormatContext *s, const AVStream *st,
                             const uint8_t * const payload, const int payload_size,
                             int64_t pts, int64_t dts, const int key, const int stream_id, int is_start, int64_t service_time)
{
    MpegTSWriteStream *ts_st = st->priv_data;
    MpegTSWrite *ts = s->priv_data;
    uint8_t buf[TS_PACKET_SIZE];
    uint8_t *q;
    int val, len, header_len, write_pcr;
    const int is_dvb_subtitle = (st->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) && (st->codecpar->codec_id == AV_CODEC_ID_DVB_SUBTITLE);
    int64_t pcr = -1; /* avoid warning */
    int64_t delay = av_rescale(s->max_delay, 90000, AV_TIME_BASE);
    int force_pat = st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO && key && !ts_st->prev_payload_key;

    av_assert0(ts_st->payload != buf || st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO);
    if (ts->flags & MPEGTS_FLAG_PAT_PMT_AT_FRAMES && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        force_pat = 1;
    }

    if(!is_start)
        force_pat = 0;

    if (1) {
        //retransmit_si_info(s, force_pat, dts);

        write_pcr = 0;
#if 0
        if (ts_st->pid == ts_st->service->pcr_pid) {
            if (ts->mux_rate > 1 || is_start) // VBR pcr period is based on frames
                ts_st->service->pcr_packet_count++;
            if (ts_st->service->pcr_packet_count >=
                ts_st->service->pcr_packet_period) {
                ts_st->service->pcr_packet_count = 0;
                write_pcr = 1;
            }
        }
        if (ts->mux_rate > 1 && dts != AV_NOPTS_VALUE &&
            (dts - get_pcr(ts, s->pb) / 300) > delay) {
            /* pcr insert gets priority over null packet insert */
            if (write_pcr)
                mpegts_insert_pcr_only(s, st);
            else
                mpegts_insert_null_packet(s);
            /* recalculate write_pcr and possibly retransmit si_info */
            return 0;
        }
#endif

        /* prepare packet header */
        q    = buf;
        *q++ = 0x47;
        val  = ts_st->pid >> 8;
        if (is_start)
            val |= 0x40;
        *q++      = val;
        *q++      = ts_st->pid;
        ts_st->cc = ts_st->cc + 1 & 0xf;
        *q++      = 0x10 | ts_st->cc; // payload indicator + CC
        if (ts_st->discontinuity) {
            set_af_flag(buf, 0x80);
            q = get_ts_payload_start(buf);
            ts_st->discontinuity = 0;
        }
        if (key && is_start && pts != AV_NOPTS_VALUE) {
            // set Random Access for key frames
            if (ts_st->pid == ts_st->service->pcr_pid)
                write_pcr = 1;
            set_af_flag(buf, 0x40);
            q = get_ts_payload_start(buf);
        }
        if (ts_st->pid == ts_st->service->pcr_pid && ts_st->service->last_pcr + 3000*300 < service_time)
            write_pcr = 1;
        if (write_pcr) {
            int64_t adjustment = 0;
            int64_t bytes = avio_tell(s->pb);
            service_time -=  TSI_MAX_PCR_ADJUSTMENT*300;
            if (ts->mux_rate > 1 && ts_st->service->last_pcr != AV_NOPTS_VALUE){
                int64_t bytes_diff = bytes - ts_st->service->last_pcr_bytes;
                int64_t pcr_diff = service_time - ts_st->service->last_pcr;
                int64_t tmp = ts_st->service->last_pcr_remainder + bytes_diff * 8 * PCR_TIME_BASE;
                int64_t exp = tmp / ts->mux_rate;
                ts_st->service->last_pcr_remainder = tmp % ts->mux_rate;
                adjustment = exp - pcr_diff;
                if (llabs(adjustment)>=TSI_MAX_PCR_ADJUSTMENT * 300) {// 5*PCR_TIME_BASE/1000) {
                    // TODO: disable warning after buffering ?
                    av_log(s, AV_LOG_WARNING, "[tsi] pcr ajustment exceeds limit, resetting to zero (was %9.6fsec, prev %.6fsec, pcr-diff %.6fsec, bytes %d) \n",
                           adjustment / (300.0 * 90000.0),
                           ts_st->service->tsi_dbg_prev_pcr_adjustment / (300.0 * 90000.0),
                           (service_time - ts_st->service->last_pcr) / (90000.0 * 300),
                           (int)bytes_diff);
                    adjustment = 0;
                }
            }
            ts_st->service->last_pcr_bytes = bytes;
            ts_st->service->tsi_dbg_prev_pcr_adjustment = adjustment;

            set_af_flag(buf, 0x10);
            q = get_ts_payload_start(buf);

            // add 11, pcr references the last byte of program clock reference base
            if (ts->mux_rate > 1)
                pcr = get_pcr(ts, s->pb);
            else
                pcr = (dts - delay) * 300;

            pcr = service_time + adjustment;
            ts_st->service->last_pcr = pcr;

            if (dts != AV_NOPTS_VALUE && dts < pcr / 300)
                av_log(s, AV_LOG_WARNING, "dts < pcr, TS is invalid\n");
            extend_af(buf, write_pcr_bits(q, pcr));
            q = get_ts_payload_start(buf);
        }
        if (is_start) {
            int pes_extension = 0;
            int pes_header_stuffing_bytes = 0;
            /* write PES header */
            *q++ = 0x00;
            *q++ = 0x00;
            *q++ = 0x01;
            int is_dvb_teletext = 0;
            if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                if (st->codecpar->codec_id == AV_CODEC_ID_DIRAC)
                    *q++ = 0xfd;
                else
                    *q++ = 0xe0;
            } else if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
                       (st->codecpar->codec_id == AV_CODEC_ID_MP2 ||
                        st->codecpar->codec_id == AV_CODEC_ID_MP3 ||
                        st->codecpar->codec_id == AV_CODEC_ID_AAC)) {
                *q++ = 0xc0;
            } else if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
                        st->codecpar->codec_id == AV_CODEC_ID_AC3 &&
                        ts->m2ts_mode) {
                *q++ = 0xfd;
            } else if (st->codecpar->codec_type == AVMEDIA_TYPE_DATA &&
                       st->codecpar->codec_id == AV_CODEC_ID_TIMED_ID3) {
                *q++ = 0xbd;
            } else if (st->codecpar->codec_type == AVMEDIA_TYPE_DATA) {
                *q++ = stream_id != -1 ? stream_id : 0xfc;

                if (stream_id == 0xbd) /* asynchronous KLV */
                    pts = dts = AV_NOPTS_VALUE;
            } else {
                *q++ = 0xbd;
                if (st->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) {
                    if (st->codecpar->codec_id == AV_CODEC_ID_DVB_TELETEXT) {
                        is_dvb_teletext = 1;
                    }
                }
            }
            header_len = 0;
            int flags      = 0;
            if (pts != AV_NOPTS_VALUE) {
                header_len += 5;
                flags      |= 0x80;
            }
            if (dts != AV_NOPTS_VALUE && pts != AV_NOPTS_VALUE && dts != pts) {
                header_len += 5;
                flags      |= 0x40;
            }
            if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
                st->codecpar->codec_id == AV_CODEC_ID_DIRAC) {
                /* set PES_extension_flag */
                pes_extension = 1;
                flags        |= 0x01;

                /* One byte for PES2 extension flag +
                 * one byte for extension length +
                 * one byte for extension id */
                header_len += 3;
            }
            /* for Blu-ray AC3 Audio the PES Extension flag should be as follow
             * otherwise it will not play sound on blu-ray
             */
            if (ts->m2ts_mode &&
                st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
                st->codecpar->codec_id == AV_CODEC_ID_AC3) {
                        /* set PES_extension_flag */
                        pes_extension = 1;
                        flags |= 0x01;
                        header_len += 3;
            }
            if (is_dvb_teletext) {
                pes_header_stuffing_bytes = 0x24 - header_len;
                header_len = 0x24;
            }
            len = payload_size + header_len + 3;
            /* 3 extra bytes should be added to DVB subtitle payload: 0x20 0x00 at the beginning and trailing 0xff */
            if (is_dvb_subtitle) {
                len += 3;
            }
            if (len > 0xffff)
                len = 0;
            if (ts->omit_video_pes_length && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                len = 0;
            }
            *q++ = len >> 8;
            *q++ = len;
            val  = 0x80;
            /* data alignment indicator is required for subtitle and data streams */
            if (st->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE || st->codecpar->codec_type == AVMEDIA_TYPE_DATA)
                val |= 0x04;
            *q++ = val;
            *q++ = flags;
            *q++ = header_len;
            if (pts != AV_NOPTS_VALUE) {
                write_pts(q, flags >> 6, pts);
                q += 5;
            }
            if (dts != AV_NOPTS_VALUE && pts != AV_NOPTS_VALUE && dts != pts) {
                write_pts(q, 1, dts);
                q += 5;
            }
            if (pes_extension && st->codecpar->codec_id == AV_CODEC_ID_DIRAC) {
                flags = 0x01;  /* set PES_extension_flag_2 */
                *q++  = flags;
                *q++  = 0x80 | 0x01; /* marker bit + extension length */
                /* Set the stream ID extension flag bit to 0 and
                 * write the extended stream ID. */
                *q++ = 0x00 | 0x60;
            }
            /* For Blu-ray AC3 Audio Setting extended flags */
          if (ts->m2ts_mode &&
              pes_extension &&
              st->codecpar->codec_id == AV_CODEC_ID_AC3) {
                      flags = 0x01; /* set PES_extension_flag_2 */
                      *q++ = flags;
                      *q++ = 0x80 | 0x01; /* marker bit + extension length */
                      *q++ = 0x00 | 0x71; /* for AC3 Audio (specifically on blue-rays) */
              }


            if (is_dvb_subtitle) {
                /* First two fields of DVB subtitles PES data:
                 * data_identifier: for DVB subtitle streams shall be coded with the value 0x20
                 * subtitle_stream_id: for DVB subtitle stream shall be identified by the value 0x00 */
                *q++ = 0x20;
                *q++ = 0x00;
            }
            if (is_dvb_teletext) {
                memset(q, 0xff, pes_header_stuffing_bytes);
                q += pes_header_stuffing_bytes;
            }
            is_start = 0;
        }
        /* header size */
        header_len = q - buf;
        /* data len */
        len = FFMIN(TS_PACKET_SIZE - header_len, payload_size + (is_dvb_subtitle?1:0) );
        int stuffing_len = TS_PACKET_SIZE - header_len - len;
        if (stuffing_len > 0) {
            /* add stuffing with AFC */
            if (buf[3] & 0x20) {
                /* stuffing already present: increase its size */
                int afc_len = buf[4] + 1;
                memmove(buf + 4 + afc_len + stuffing_len,
                        buf + 4 + afc_len,
                        header_len - (4 + afc_len));
                buf[4] += stuffing_len;
                memset(buf + 4 + afc_len, 0xff, stuffing_len);
            } else {
                /* add stuffing */
                memmove(buf + 4 + stuffing_len, buf + 4, header_len - 4);
                buf[3] |= 0x20;
                buf[4]  = stuffing_len - 1;
                if (stuffing_len >= 2) {
                    buf[5] = 0x00;
                    memset(buf + 6, 0xff, stuffing_len - 2);
                }
            }
        }

        if (is_dvb_subtitle && payload_size+1 == len) {
            memcpy(buf + TS_PACKET_SIZE - len, payload, len - 1);
            buf[TS_PACKET_SIZE - 1] = 0xff; /* end_of_PES_data_field_marker: an 8-bit field with fixed contents 0xff for DVB subtitle */
        } else {
            memcpy(buf + TS_PACKET_SIZE - len, payload, len);
        }

        mpegts_prefix_m2ts_header(s);

        int64_t t0 = av_gettime_relative();

        avio_write(s->pb, buf, TS_PACKET_SIZE);

        { //TODO: remove debug
            int64_t t = av_gettime_relative();
            if (t - t0 > 10000 ) {
                av_log(s, AV_LOG_WARNING, "[tsi] avio_write: call duration %.3f sec\n",
                    (t - t0) / 1000000.0);
            }
        }
    }
    if(len>=payload_size){
        ts_st->prev_payload_key = key;
    }
    return FFMIN(len, payload_size);
}

//TODO: AVOption ?
static const int64_t TSI_DISCONTINUITY_THRESHOLD = 1000 * 90000LL / 1000;
static const int64_t TSI_WINDOW = 950 * 90000LL / 1000;
static const int64_t TSI_BUFFER_LOW  = 2000 * 90000LL / 1000;
static const int64_t TSI_BUFFER_TARGET = 4000 * 90000LL / 1000;
static const int64_t TSI_BUFFER_HIGH = 15000 * 90000LL / 1000;
static const int64_t TSI_MAX_AUDIO_PACKET_DURATION = 120 * 90000LL / 1000;

static MpegTSPesPacket* tsi_buffer_head(MpegTSWriteStream *ts_st)
{
    return &ts_st->packets[ts_st->packets_head % (sizeof(ts_st->packets)/sizeof(ts_st->packets[0]))];
}
static MpegTSPesPacket* tsi_buffer_tail(MpegTSWriteStream *ts_st)
{
    return &ts_st->packets[(ts_st->packets_tail-1) % (sizeof(ts_st->packets)/sizeof(ts_st->packets[0]))];
}
static MpegTSPesPacket* tsi_buffer_next(MpegTSWriteStream *ts_st, MpegTSPesPacket* current)
{
    //TODO: rewrite
    if (current==tsi_buffer_tail(ts_st)) return NULL;
    MpegTSPesPacket *last = &ts_st->packets[sizeof(ts_st->packets)/sizeof(ts_st->packets[0]) - 1];
    return current == last ? &ts_st->packets[0] : current+1;
}

static void tsi_schedule_first_packet(AVFormatContext *s, AVStream *st)
{
    //TODO: a/v offsets?
    //int64_t offs1 = (st && st->codecpar->codec_type==AVMEDIA_TYPE_VIDEO) ? 90000/2 : 0;
    MpegTSWrite* ts = s->priv_data;
    MpegTSWriteStream* ts_st = st->priv_data;

    //ts_st->packet_end_time = ts_st->packets[ts_st->packets_head % (sizeof(ts_st->packets)/sizeof(ts_st->packets[0]))].dts;
    //return;
#if 0
    int64_t res;
    int64_t service_time = ts_st->service_time;
    MpegTSPesPacket *end = tsi_buffer_head(ts_st);
    uint64_t first_bytes = end->payload_size;
    uint64_t bytes=0;

    if (1) {
        struct fraction_t { int64_t num; int64_t den; } min_val={0,0}, cur_val;
        for( ;end && end->dts-service_time<TSI_BUFFER_TARGET; end = tsi_buffer_next(ts_st, end)){
            bytes += end->payload_size;
            cur_val = (struct fraction_t){(end->dts-service_time)/* *first_bytes*/, bytes};
            if (min_val.den == 0 || cur_val.num*min_val.den < min_val.num*cur_val.den){
                min_val = cur_val;
            }
        }
        res = first_bytes*min_val.num/min_val.den;
    } else {
        int64_t min_time = INT64_MAX;
        for( ;end && end->dts-service_time<TSI_BUFFER_TARGET; end = tsi_buffer_next(ts_st, end)){
            bytes += end->payload_size;
            min_time = FFMIN(min_time, ((end->dts-service_time)) * first_bytes / bytes);
        }
        res = min_time;
    }
    ts_st->packet_end_time = service_time + res;
#else
    {
        MpegTSPesPacket *pkt = tsi_buffer_head(ts_st);
        int64_t first_bytes = pkt->payload_size;
        int64_t bytes = 0;
        int64_t duration = 0;
        int64_t min_time = ts_st->packet_start_time;
        int64_t max_time = pkt->dts;
        int64_t prev_dts = ts_st->packet_start_time;
        for ( ; pkt && duration < TSI_BUFFER_TARGET; pkt = tsi_buffer_next(ts_st, pkt)) {
            bytes += pkt->payload_size;
            //FIXME: different discontinuities?
            if (pkt->dts - prev_dts < TSI_DISCONTINUITY_THRESHOLD) { // FIXME: what else? += prev delta?
                duration += pkt->dts - prev_dts;
            }
            // FIXME: rewrite
            int64_t time = ts_st->packet_start_time + duration * first_bytes / bytes;
            if (time < min_time) {
                max_time = FFMIN(max_time, min_time);
                //av_log(s, AV_LOG_ERROR, "[pid 0x%x] INTERESTING1\n", ts_st->pid);
                break;
            } else {
                max_time = FFMIN(max_time, time);
                if (duration > TSI_WINDOW) {
                    int64_t time = ts_st->packet_start_time +
                            (duration - TSI_WINDOW) * first_bytes / bytes;
                    min_time = FFMAX(min_time, time);
                }
            }
            prev_dts = pkt->dts;
        }
        ts_st->packet_end_time = max_time;
        if (max_time + TSI_WINDOW == tsi_buffer_head(ts_st)->dts && ts->mux_rate > 1) // FIXME: log it when PCR ajustment failed?
            av_log(s, AV_LOG_WARNING, "[tsi] [pid 0x%x] Consider increasing TSI_WINDOW: offset=%.3fsec\n", ts_st->pid,
                   (tsi_buffer_head(ts_st)->dts - max_time) / 90000.0);
    }
#endif
    //av_log(s, AV_LOG_ERROR, "[pid 0x%x] endtime DIFF %5.3f sec\n", ts_st->pid,
    //       (ts_st->packets_first->dts-ts_st->packets_first->end_streamtime)/90000.0/300 );
    /*
    if(ts_st->packets_first->end_streamtime<ts_st->packets_first->start_streamtime)
        av_log(NULL, AV_LOG_ERROR, "DIFF instream %d!!!\n", st->index);
    //int64_t bw = ts_st->packets_first->payload_size*8*90000/1000/( ts_st->packets_first->end_streamtime-ts_st->packets_first->start_streamtime);
    int64_t bw = ((int64_t)ts_st->packets_first->payload_size)*8*90/1/( ts_st->packets_first->end_streamtime-ts_st->packets_first->start_streamtime);
    av_log(s, AV_LOG_ERROR, "%d: bandwidth %4ld Kbit/s!!!\n", st->index, bw);
    */
}

static void tsi_dbg_sprintf_buffers(char* buf, int size, AVFormatContext *s, MpegTSService *service)
{
    int first = 1;
    for (int j = 0; j < s->nb_streams; j++) {
        MpegTSWriteStream *ts_st2 = s->streams[j]->priv_data;
        int cnt;
        if (service && ts_st2->service != service) continue;
        cnt = snprintf(buf, size, first ? "pid#%x %.3fsec/%dKB/%dpkt" : " #%x %.3f/%d/%d",
            ts_st2->pid, ts_st2->buffer_duration_dts / 90000.0, (int)ts_st2->buffer_bytes / 1024,
            (int)ts_st2->buffer_packets);
        first = 0;
        if (cnt >= size)
            break;
        else{
            size -= cnt;
            buf += cnt;
        }
    }
}

static void tsi_schedule_services(AVFormatContext *s, int flush)
{
    MpegTSWrite *ts = s->priv_data;

    { //TODO: remove debug
        int64_t t = av_gettime_relative();
        if (ts->tsi_dbg_buffer_report_time == 0) ts->tsi_dbg_buffer_report_time = t;

        if (t - ts->tsi_dbg_buffer_report_time > 60LL * 60 * 1000000) {
            ts->tsi_dbg_buffer_report_time = t;
            for(int i = 0; i < ts->nb_services; i++){
                char buf[4096];
                tsi_dbg_sprintf_buffers(buf, 4096, s, ts->services[i]);
                av_log(s, AV_LOG_INFO, "[tsi] service#%d buffers: %s\n", i, buf);
            }
        }
    }


    for (int i = 0; i < ts->nb_services; i++) {
        ts->services[i]->max_dts_buffered = 0;
        ts->services[i]->min_time = INT64_MAX;
    }
    for (int i = 0; i < s->nb_streams; i++){
        MpegTSWriteStream *ts_st2 = s->streams[i]->priv_data;
        //FIXME: remove empty packets here ?
        //FIXME: set streamtime if NOPTS && first_packet ?
        if (ts_st2->service_time == AV_NOPTS_VALUE) {
            if(ts_st2->packets_head==ts_st2->packets_tail) continue;
            ts_st2->service_time = ts_st2->packet_start_time; //FIXME:!!!
        }
        ts_st2->service->max_dts_buffered = FFMAX(ts_st2->service->max_dts_buffered, ts_st2->buffer_duration_dts);
        ts_st2->service->min_time = FFMIN(ts_st2->service->min_time, ts_st2->service_time);
    }
    for (int i = 0; i < ts->nb_services; i++) {
        MpegTSService *service = ts->services[i];
        if (service->is_buffered && service->max_dts_buffered < TSI_BUFFER_LOW) {
            if (!flush) {
                char buf[4096];
                tsi_dbg_sprintf_buffers(buf, 4096, s, service);
                av_log(s, AV_LOG_WARNING, "[tsi] service#%d buffering started : %s\n", i, buf);
            }
            service->is_buffered = 0;
        }
        if (!service->is_buffered && service->max_dts_buffered > TSI_BUFFER_TARGET) {
            char buf[4096];
            tsi_dbg_sprintf_buffers(buf, 4096, s, service);
            av_log(s, AV_LOG_INFO, "[tsi] service#%d buffering complete: %s\n", i, buf);
            service->is_buffered = 1;
            service->time_offset = ts->ts_stream_time - service->min_time;
        }
        if (service->min_time != INT64_MAX && (service->is_buffered || flush) ) { //TODO: check this only on new packet||flush ?
            if (ts->ts_stream_time > service->min_time + service->time_offset) {
                av_log(s, AV_LOG_ERROR, "[tsi] service#%d incorrect stream interleaving. Time jumped back\n", i);
                service->time_offset = ts->ts_stream_time - service->min_time;
            }
            if (ts->ts_stream_time + TSI_WINDOW < service->min_time + service->time_offset) { // TODO: TSI_WINDOW vs new const?
                av_log(s, AV_LOG_WARNING, "[tsi] service#%d detected large gap in playback, resyncing (gap=%.3fsec)\n",
                       i,
                       (service->min_time + service->time_offset - ts->ts_stream_time) / 90000.0);
                service->time_offset = ts->ts_stream_time - service->min_time;
            }
        }
    }
}

static AVStream* tsi_get_earliest_stream(AVFormatContext *s, int flush)
{
    AVStream *st = NULL;
    MpegTSWriteStream *ts_st = NULL;
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st2 = s->streams[i];
        MpegTSWriteStream *ts_st2 = st2->priv_data;

        if (!ts_st2->service->is_buffered && !flush) continue;
        if(ts_st2->service_time == AV_NOPTS_VALUE) continue;

        if(!st || ts_st2->service_time + ts_st2->service->time_offset < ts_st->service_time + ts_st->service->time_offset) {
            st = st2;
            ts_st = ts_st2;
        }
    }
    return st;
}

static void tsi_remove_empty_packet(MpegTSWriteStream *ts_st)
{
    MpegTSPesPacket *old_head = tsi_buffer_head(ts_st);
    ts_st->buffer_packets -= 1;
    ts_st->buffer_bytes -= old_head->payload_size;
    av_freep(&old_head->payload);
    ts_st->packets_head++;

    if (ts_st->packets_head != ts_st->packets_tail) {
        MpegTSPesPacket *new_head = tsi_buffer_head(ts_st);
        int64_t dts_diff = new_head->dts - old_head->dts;
        if (dts_diff < TSI_DISCONTINUITY_THRESHOLD) {
            ts_st->buffer_duration_dts -= dts_diff;
            ts_st->packet_start_time = ts_st->packet_end_time;
            ts_st->packet_end_time = AV_NOPTS_VALUE;
            //FIXME: update stream_time?
        } else {
            //FIXME: wtf???
            ts_st->packet_start_time = new_head->dts - (old_head->dts - ts_st->packet_end_time); // FIXME: sub prev dts delta?
            ts_st->service_time = ts_st->packet_start_time;
        }
    } else {
        ts_st->service_time = AV_NOPTS_VALUE; //FIXME: need this?
    }
    ts_st->packet_consumed_bytes = 0;
}

static void tsi_drop_overflowed_buffers(AVFormatContext *s)
{
    /*FIXME: check this
    for (int i=0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        if (ts_st->buffer_duration_dts > TSI_BUFFER_HIGH) {
            char buf_before[4096], buf_after[4096];
            MpegTSPesPacket *end = tsi_buffer_head(ts_st);
            MpegTSPesPacket *tail = tsi_buffer_tail(ts_st);
            MpegTSPesPacket *pkt;
            int64_t duration = 0;
            tsi_dbg_sprintf_buffers(buf_before, sizeof(buf_after), s, ts_st->service);
            while (end != tail && duration < TSI_BUFFER_TARGET) {
                MpegTSPesPacket *next = tsi_buffer_next(ts_st, end);
                if (next->dts - end->dts < TSI_DISCONTINUITY_THRESHOLD)
                    duration += next->dts - end->dts;
                end = next;
            }
            pkt = end;
            while (pkt) {
                ts_st->buffer_packets -= 1;
                ts_st->buffer_bytes -= pkt->payload_size;
                //FIXME: buffered_dts!
                av_free(pkt->payload);
                pkt = tsi_buffer_next(ts_st, pkt);
            }
            //FIXME: tail

            tsi_dbg_sprintf_buffers(buf_after, sizeof(buf_after), s, ts_st->service);
            av_log(s, AV_LOG_ERROR, "[tsi] service#%x pid#%x buffer overflow, dropping it (%s) -> (%s)\n",
                ts_st->service->sid, ts_st->pid, buf_before, buf_after);
        }
    }
    */
}

static void tsi_drain_interleaving_buffer(AVFormatContext *s, int64_t duration, int flush)
{
    MpegTSWrite *ts = s->priv_data;
    tsi_schedule_services(s, flush);
    //TODO: report stream time here

    while (1) {
        AVStream *st = tsi_get_earliest_stream(s, flush);
        MpegTSWriteStream *ts_st    = !st ? NULL : st->priv_data;
        int64_t new_stream_time     = !st ? 0    : ts_st->service_time + ts_st->service->time_offset;
        MpegTSPesPacket* pes_packet = !st ? NULL : tsi_buffer_head(ts_st);
        if (!st) break;

        if (duration == AV_NOPTS_VALUE) { // non-realtime
            int64_t max_dts_buffered = 0;
            for (int i=0; i<ts->nb_services; i++)
                max_dts_buffered = FFMAX(max_dts_buffered, ts->services[i]->max_dts_buffered);
            if (max_dts_buffered < TSI_BUFFER_TARGET && !flush)
                break;
        } else {
            if (new_stream_time > ts->ts_stream_time + duration) {
                ts->ts_stream_time += duration;
                break;
            }
            duration -= new_stream_time - ts->ts_stream_time;
        }

        if (new_stream_time < ts->ts_stream_time) { // TODO: this should not happen, remove check
            av_log(s, AV_LOG_ERROR, "[tsi] [WTF] new_stream_time < ts->ts_stream_time (%ld < %ld)\n", new_stream_time, ts->ts_stream_time);
        }
        ts->ts_stream_time = new_stream_time;

        if (ts_st->packet_end_time == AV_NOPTS_VALUE) { // TODO (consumed==0) packet start?
            tsi_schedule_first_packet(s, st);
        }

        // SDT packets
        if (ts->last_sdt + 1*90000 < new_stream_time) {
            ts->last_sdt = new_stream_time;
            mpegts_write_sdt(s);
        }
        // PAT/PMT packets
        int force_pat = 0;
        if (ts_st->packet_consumed_bytes == 0 && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            force_pat = pes_packet->key;// FIXME: && !ts_st->prev_payload_key;
            force_pat = force_pat || ts->flags & MPEGTS_FLAG_PAT_PMT_AT_FRAMES;
        }
        if(ts->last_pat + 8000 < new_stream_time || force_pat) {
            ts->last_pat = new_stream_time;
            mpegts_write_pat(s);
            for (int i = 0; i < ts->nb_services; i++) // FIXME: send only one PMT at a time?
                mpegts_write_pmt(s, ts->services[i]);
        }
        // NULL packets
        if (ts->mux_rate > 1) {// && ts_st->pid == ts_st->service->pcr_pid){ // TODO: only pcr pid?
            int64_t expected; // TODO: rename
            int64_t bytes = avio_tell(s->pb);
            if(ts->rate_last_streamtime==0){ // FIXME: ?
                ts->rate_last_streamtime = ts->ts_stream_time;
                ts->rate_last_bytes = bytes;
            }
            expected = ts->mux_rate * (ts->ts_stream_time - ts->rate_last_streamtime) / (8 * 90000)
                    - (bytes - ts->rate_last_bytes);
            if (expected < -188 * 20 ) { // TODO: threshold?
                av_log(s, AV_LOG_WARNING, "[tsi] bitrate too high, resyncing (%d packets)\n",
                       (int) -expected / 188);
                ts->rate_last_streamtime = ts->ts_stream_time;
                ts->rate_last_bytes = bytes;
            }
            if (expected > (ts->mux_rate / 8) / 20 ) {
                av_log(s, AV_LOG_WARNING, "[tsi] [WTF] NULL packet burst (%d packets)\n",(int)expected/188);
            }
            while (expected > 188) {
                mpegts_insert_null_packet(s);
                expected -= 188;
            }
        }

        //FIXME: perform PCR ajustment here +-delta

        int64_t t0 = av_gettime_relative();
        int len = mpegts_write_pes1(s, st,
                                    pes_packet->payload + ts_st->packet_consumed_bytes,
                                    pes_packet->payload_size - ts_st->packet_consumed_bytes,
                                    pes_packet->pts,
                                    pes_packet->dts,
                                    pes_packet->key,
                                    pes_packet->stream_id,
                                    ts_st->packet_consumed_bytes == 0,
                                    ts_st->service_time * 300
                                    );
        { //TODO: remove debug
            int64_t t = av_gettime_relative();
            if (t - t0 > 10000 ) {
                av_log(s, AV_LOG_WARNING, "[tsi] mpegts_write_pes1: call duration %.3f sec\n",
                    (t - t0) / 1000000.0);
            }
        }
        ts_st->packet_consumed_bytes += len;
        ts_st->service_time = ts_st->packet_start_time + (ts_st->packet_end_time - ts_st->packet_start_time) * ts_st->packet_consumed_bytes / pes_packet->payload_size;

        if (ts_st->packet_consumed_bytes == pes_packet->payload_size) {
            tsi_remove_empty_packet(ts_st);
            tsi_schedule_services(s, flush);
        }
    }
    if (duration != AV_NOPTS_VALUE) { // realtime
        tsi_drop_overflowed_buffers(s);
    }
}

static int tsi_interleave_packet(AVFormatContext *s, AVPacket *out, AVPacket *in, int flush)
{
    //return ff_interleave_packet_per_dts(s, out, in, flush);
    if (in) {
        // out packet is uninitialized here so we can't use av_packet_ref
        av_copy_packet(out, in);
    }
    return in != NULL;
}

static void* tsi_thread(void *opaque)
{
    AVFormatContext *s = opaque;
    MpegTSWrite *ts = s->priv_data;
    int64_t t_prev = av_gettime_relative() * 9/*0000*/ / 100/*0000*/;
    while (!ts->tsi_thread_exit) {
        int64_t t = av_gettime_relative() * 9/*0000*/ / 100/*0000*/;
        int64_t duration = t - t_prev;
        if (duration > 90000 / 10) {
            av_log(s, AV_LOG_WARNING, "[tsi] tsi_thread: duration too large %.3f sec\n", duration / 90000.0);
            duration = 90000 / 10;
        }
        pthread_mutex_lock(&ts->tsi_mutex);
        tsi_drain_interleaving_buffer(s, duration, 0);
        //tsi_drain_interleaving_buffer(s, AV_NOPTS_VALUE, 0);
        pthread_mutex_unlock(&ts->tsi_mutex);

        av_usleep(10000); //TODO: AVOption?
        t_prev = t;
    }
    return NULL;
}

static void mpegts_write_pes(AVFormatContext *s, const AVStream *st,
                             const uint8_t * payload, int payload_size,
                             int64_t pts, int64_t dts, const int key, const int stream_id)
{
    // TODO: NOPTS, buffer overflow, no-sub-stream, MPTS?
    //TODO: flush called on stream close
    //TODO: pass buffer ref to mpegts_write_pes

    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = st->priv_data;
    MpegTSPesPacket* pes_packet;

    if (dts == AV_NOPTS_VALUE) {
        av_log(s, AV_LOG_ERROR, "[tsi] [pid 0x%x] Dropping packet with dts==NOPTS \n", ts_st->pid);
        return;
    }

    //TODO: remove debug
    {
        int64_t t = av_gettime_relative();
        if (ts->tsi_dbg_mpegts_write_pes_enter_time == 0) ts->tsi_dbg_mpegts_write_pes_enter_time = t;
        if (t - ts->tsi_dbg_mpegts_write_pes_enter_time > 600000) {
            char buf[4096];
            tsi_dbg_sprintf_buffers(buf, 4096, s, ((MpegTSWriteStream*)st->priv_data)->service);
            av_log(s, AV_LOG_WARNING, "[tsi] mpegts_write_pes: last packet was %.3f sec ago (buffers %s)\n",
                (t - ts->tsi_dbg_mpegts_write_pes_enter_time) / 1000000.0, buf);
        }
        ts->tsi_dbg_mpegts_write_pes_enter_time = t;
    }

#if 1
    pthread_mutex_lock(&ts->tsi_mutex);

    MpegTSPesPacket *last_packet = tsi_buffer_tail(ts_st);

    if (ts_st->buffer_packets>0 && dts < last_packet->dts) {
        av_log(s, AV_LOG_ERROR, "[tsi] [pid 0x%x] Dropping packet with dts<prev_dts (%ld<%ld) \n", ts_st->pid,
               pes_packet->dts, last_packet->dts);
        pthread_mutex_unlock(&ts->tsi_mutex);
        return;
    }

    // join audio pes-packets
    if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO
            && ts_st->buffer_packets > 1
            && last_packet->dts + TSI_MAX_AUDIO_PACKET_DURATION >= dts) {
        last_packet->payload = av_realloc(last_packet->payload, last_packet->payload_size + payload_size);
        memcpy(last_packet->payload + last_packet->payload_size, payload, payload_size);
        last_packet->payload_size += payload_size;
        ts_st->buffer_bytes += payload_size;
        pthread_mutex_unlock(&ts->tsi_mutex);
        return;
    }

    ts_st->packets_tail++;
    pes_packet = tsi_buffer_tail(ts_st);
    pes_packet->payload = av_malloc(payload_size);
    memcpy(pes_packet->payload, payload, payload_size);
    pes_packet->payload_size = payload_size;
    pes_packet->pts = pts;
    pes_packet->dts = dts;
    pes_packet->key = key;
    pes_packet->stream_id = stream_id;

    ts_st->buffer_packets += 1;
    ts_st->buffer_bytes += payload_size;

    if (ts_st->buffer_packets>1) {
        int64_t dts_diff = pes_packet->dts - last_packet->dts;
        if (dts_diff < TSI_DISCONTINUITY_THRESHOLD) {// TODO: add discontinuity flag?
            ts_st->buffer_duration_dts += dts_diff;
        } else {
            //FIXME: move somewhere? only for pcr-pid?
            av_log(s, AV_LOG_WARNING, "[tsi] [pid 0x%x] dts discontinuity! \n", ts_st->pid);
        }
    } else {
        av_log(s, AV_LOG_INFO, "[tsi] [pid 0x%x] got first packet \n", ts_st->pid); // FIXME: remove
        // We don't know how to properly redistribute time in a first frame.
        // Assume worst case - it's a huge packet occupying full TSI_WINDOW.
        ts_st->packet_start_time = pes_packet->dts - TSI_WINDOW; // FIXME: move to update_info
        ts_st->service_time = ts_st->packet_start_time;
    }

    pthread_mutex_unlock(&ts->tsi_mutex);

    if (!ts->tsi_is_realtime) {
        tsi_drain_interleaving_buffer(s, AV_NOPTS_VALUE, 0);
    }

    { //TODO: remove debug
        int64_t t = av_gettime_relative();
        if (t - ts->tsi_dbg_mpegts_write_pes_enter_time > 100000 ) {
            av_log(s, AV_LOG_WARNING, "[tsi] mpegts_write_pes: call duration %.3f sec\n",
                (t - ts->tsi_dbg_mpegts_write_pes_enter_time) / 1000000.0);
        }
    }

#else
    int is_start = 1;
    do {
        int len = mpegts_write_pes1(s, st, payload, payload_size, pts, dts, key, stream_id, is_start, 0);
        payload += len;
        payload_size -= len;
        is_start = 0;
    } while (payload_size > 0);
#endif
}


int ff_check_h264_startcode(AVFormatContext *s, const AVStream *st, const AVPacket *pkt)
{
    if (pkt->size < 5 || AV_RB32(pkt->data) != 0x0000001 && AV_RB24(pkt->data) != 0x000001) {
        if (!st->nb_frames) {
            av_log(s, AV_LOG_ERROR, "H.264 bitstream malformed, "
                   "no startcode found, use the video bitstream filter 'h264_mp4toannexb' to fix it "
                   "('-bsf:v h264_mp4toannexb' option with ffmpeg)\n");
            return AVERROR_INVALIDDATA;
        }
        av_log(s, AV_LOG_WARNING, "H.264 bitstream error, startcode missing, size %d", pkt->size);
        if (pkt->size)
            av_log(s, AV_LOG_WARNING, " data %08"PRIX32, AV_RB32(pkt->data));
        av_log(s, AV_LOG_WARNING, "\n");
    }
    return 0;
}

static int check_hevc_startcode(AVFormatContext *s, const AVStream *st, const AVPacket *pkt)
{
    if (pkt->size < 5 || AV_RB32(pkt->data) != 0x0000001 && AV_RB24(pkt->data) != 0x000001) {
        if (!st->nb_frames) {
            av_log(s, AV_LOG_ERROR, "HEVC bitstream malformed, no startcode found\n");
            return AVERROR_PATCHWELCOME;
        }
        av_log(s, AV_LOG_WARNING, "HEVC bitstream error, startcode missing, size %d", pkt->size);
        if (pkt->size)
            av_log(s, AV_LOG_WARNING, " data %08"PRIX32, AV_RB32(pkt->data));
        av_log(s, AV_LOG_WARNING, "\n");
    }
    return 0;
}

/* Based on GStreamer's gst-plugins-base/ext/ogg/gstoggstream.c
 * Released under the LGPL v2.1+, written by
 * Vincent Penquerc'h <vincent.penquerch@collabora.co.uk>
 */
static int opus_get_packet_samples(AVFormatContext *s, AVPacket *pkt)
{
    static const int durations[32] = {
      480, 960, 1920, 2880,       /* Silk NB */
      480, 960, 1920, 2880,       /* Silk MB */
      480, 960, 1920, 2880,       /* Silk WB */
      480, 960,                   /* Hybrid SWB */
      480, 960,                   /* Hybrid FB */
      120, 240, 480, 960,         /* CELT NB */
      120, 240, 480, 960,         /* CELT NB */
      120, 240, 480, 960,         /* CELT NB */
      120, 240, 480, 960,         /* CELT NB */
    };
    int toc, frame_duration, nframes, duration;

    if (pkt->size < 1)
        return 0;

    toc = pkt->data[0];

    frame_duration = durations[toc >> 3];
    switch (toc & 3) {
    case 0:
        nframes = 1;
        break;
    case 1:
        nframes = 2;
        break;
    case 2:
        nframes = 2;
        break;
    case 3:
        if (pkt->size < 2)
            return 0;
        nframes = pkt->data[1] & 63;
        break;
    }

    duration = nframes * frame_duration;
    if (duration > 5760) {
        av_log(s, AV_LOG_WARNING,
               "Opus packet duration > 120 ms, invalid");
        return 0;
    }

    return duration;
}

static int mpegts_write_packet_internal(AVFormatContext *s, AVPacket *pkt)
{
    AVStream *st = s->streams[pkt->stream_index];
    int size = pkt->size;
    uint8_t *buf = pkt->data;
    uint8_t *data = NULL;
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = st->priv_data;
    const int64_t delay = av_rescale(s->max_delay, 90000, AV_TIME_BASE) * 2;
    int64_t dts = pkt->dts, pts = pkt->pts;
    int opus_samples = 0;
    int side_data_size;
    char *side_data = NULL;
    int stream_id = -1;

    side_data = av_packet_get_side_data(pkt,
                                        AV_PKT_DATA_MPEGTS_STREAM_ID,
                                        &side_data_size);
    if (side_data)
        stream_id = side_data[0];

    if (ts->reemit_pat_pmt) {
        av_log(s, AV_LOG_WARNING,
               "resend_headers option is deprecated, use -mpegts_flags resend_headers\n");
        ts->reemit_pat_pmt = 0;
        ts->flags         |= MPEGTS_FLAG_REEMIT_PAT_PMT;
    }

    if (ts->flags & MPEGTS_FLAG_REEMIT_PAT_PMT) {
        ts->pat_packet_count = ts->pat_packet_period - 1;
        ts->sdt_packet_count = ts->sdt_packet_period - 1;
        ts->flags           &= ~MPEGTS_FLAG_REEMIT_PAT_PMT;
    }

    if (ts->copyts < 1) {
        if (pts != AV_NOPTS_VALUE)
            pts += delay;
        if (dts != AV_NOPTS_VALUE)
            dts += delay;
    }

    if (ts_st->first_pts_check && pts == AV_NOPTS_VALUE) {
        av_log(s, AV_LOG_ERROR, "first pts value must be set\n");
        return AVERROR_INVALIDDATA;
    }
    ts_st->first_pts_check = 0;

    if (st->codecpar->codec_id == AV_CODEC_ID_H264) {
        const uint8_t *p = buf, *buf_end = p + size;
        uint32_t state = -1;
        int extradd = (pkt->flags & AV_PKT_FLAG_KEY) ? st->codecpar->extradata_size : 0;
        int ret = ff_check_h264_startcode(s, st, pkt);
        if (ret < 0)
            return ret;

        if (extradd && AV_RB24(st->codecpar->extradata) > 1)
            extradd = 0;

        do {
            p = avpriv_find_start_code(p, buf_end, &state);
            av_log(s, AV_LOG_TRACE, "nal %"PRId32"\n", state & 0x1f);
            if ((state & 0x1f) == 7)
                extradd = 0;
        } while (p < buf_end && (state & 0x1f) != 9 &&
                 (state & 0x1f) != 5 && (state & 0x1f) != 1);

        if ((state & 0x1f) != 5)
            extradd = 0;
        if ((state & 0x1f) != 9) { // AUD NAL
            data = av_malloc(pkt->size + 6 + extradd);
            if (!data)
                return AVERROR(ENOMEM);
            memcpy(data + 6, st->codecpar->extradata, extradd);
            memcpy(data + 6 + extradd, pkt->data, pkt->size);
            AV_WB32(data, 0x00000001);
            data[4] = 0x09;
            data[5] = 0xf0; // any slice type (0xe) + rbsp stop one bit
            buf     = data;
            size    = pkt->size + 6 + extradd;
            av_log(s, AV_LOG_ERROR, "AUD NAL!!!\n");
        }
    } else if (st->codecpar->codec_id == AV_CODEC_ID_AAC) {
        if (pkt->size < 2) {
            av_log(s, AV_LOG_ERROR, "AAC packet too short\n");
            return AVERROR_INVALIDDATA;
        }
        if ((AV_RB16(pkt->data) & 0xfff0) != 0xfff0) {
            int ret;
            AVPacket pkt2;

            if (!ts_st->amux) {
                av_log(s, AV_LOG_ERROR, "AAC bitstream not in ADTS format "
                                        "and extradata missing\n");
            } else {
            av_init_packet(&pkt2);
            pkt2.data = pkt->data;
            pkt2.size = pkt->size;
            av_assert0(pkt->dts != AV_NOPTS_VALUE);
            pkt2.dts = av_rescale_q(pkt->dts, st->time_base, ts_st->amux->streams[0]->time_base);

            ret = avio_open_dyn_buf(&ts_st->amux->pb);
            if (ret < 0)
                return AVERROR(ENOMEM);

            ret = av_write_frame(ts_st->amux, &pkt2);
            if (ret < 0) {
                ffio_free_dyn_buf(&ts_st->amux->pb);
                return ret;
            }
            size            = avio_close_dyn_buf(ts_st->amux->pb, &data);
            ts_st->amux->pb = NULL;
            buf             = data;
            }
        }
    } else if (st->codecpar->codec_id == AV_CODEC_ID_HEVC) {
        const uint8_t *p = buf, *buf_end = p + size;
        uint32_t state = -1;
        int extradd = (pkt->flags & AV_PKT_FLAG_KEY) ? st->codecpar->extradata_size : 0;
        int ret = check_hevc_startcode(s, st, pkt);
        if (ret < 0)
            return ret;

        if (extradd && AV_RB24(st->codecpar->extradata) > 1)
            extradd = 0;

        do {
            p = avpriv_find_start_code(p, buf_end, &state);
            av_log(s, AV_LOG_TRACE, "nal %"PRId32"\n", (state & 0x7e)>>1);
            if ((state & 0x7e) == 2*32)
                extradd = 0;
        } while (p < buf_end && (state & 0x7e) != 2*35 &&
                 (state & 0x7e) >= 2*32);

        if ((state & 0x7e) < 2*16 && (state & 0x7e) >= 2*24)
            extradd = 0;
        if ((state & 0x7e) != 2*35) { // AUD NAL
            data = av_malloc(pkt->size + 7 + extradd);
            if (!data)
                return AVERROR(ENOMEM);
            memcpy(data + 7, st->codecpar->extradata, extradd);
            memcpy(data + 7 + extradd, pkt->data, pkt->size);
            AV_WB32(data, 0x00000001);
            data[4] = 2*35;
            data[5] = 1;
            data[6] = 0x50; // any slice type (0x4) + rbsp stop one bit
            buf     = data;
            size    = pkt->size + 7 + extradd;
        }
    } else if (st->codecpar->codec_id == AV_CODEC_ID_OPUS) {
        if (pkt->size < 2) {
            av_log(s, AV_LOG_ERROR, "Opus packet too short\n");
            return AVERROR_INVALIDDATA;
        }

        /* Add Opus control header */
        if ((AV_RB16(pkt->data) >> 5) != 0x3ff) {
            uint8_t *side_data;
            int side_data_size;
            int i, n;
            int ctrl_header_size;
            int trim_start = 0, trim_end = 0;

            opus_samples = opus_get_packet_samples(s, pkt);

            side_data = av_packet_get_side_data(pkt,
                                                AV_PKT_DATA_SKIP_SAMPLES,
                                                &side_data_size);

            if (side_data && side_data_size >= 10) {
                trim_end = AV_RL32(side_data + 4) * 48000 / st->codecpar->sample_rate;
            }

            ctrl_header_size = pkt->size + 2 + pkt->size / 255 + 1;
            if (ts_st->opus_pending_trim_start)
              ctrl_header_size += 2;
            if (trim_end)
              ctrl_header_size += 2;

            data = av_malloc(ctrl_header_size);
            if (!data)
                return AVERROR(ENOMEM);

            data[0] = 0x7f;
            data[1] = 0xe0;
            if (ts_st->opus_pending_trim_start)
                data[1] |= 0x10;
            if (trim_end)
                data[1] |= 0x08;

            n = pkt->size;
            i = 2;
            do {
                data[i] = FFMIN(n, 255);
                n -= 255;
                i++;
            } while (n >= 0);

            av_assert0(2 + pkt->size / 255 + 1 == i);

            if (ts_st->opus_pending_trim_start) {
                trim_start = FFMIN(ts_st->opus_pending_trim_start, opus_samples);
                AV_WB16(data + i, trim_start);
                i += 2;
                ts_st->opus_pending_trim_start -= trim_start;
            }
            if (trim_end) {
                trim_end = FFMIN(trim_end, opus_samples - trim_start);
                AV_WB16(data + i, trim_end);
                i += 2;
            }

            memcpy(data + i, pkt->data, pkt->size);
            buf     = data;
            size    = ctrl_header_size;
        } else {
            /* TODO: Can we get TS formatted data here? If so we will
             * need to count the samples of that too! */
            av_log(s, AV_LOG_WARNING, "Got MPEG-TS formatted Opus data, unhandled");
        }
    }

#if 1
    if (1) { // if tsi
        mpegts_write_pes(s, st, buf, size, pts, dts,
                         pkt->flags & AV_PKT_FLAG_KEY, stream_id);
        av_free(data);
        return 0;
    }
#else
    if (pkt->dts != AV_NOPTS_VALUE) {
        int i;
        for(i=0; i<s->nb_streams; i++) {
            AVStream *st2 = s->streams[i];
            MpegTSWriteStream *ts_st2 = st2->priv_data;
            if (   ts_st2->payload_size
               && (ts_st2->payload_dts == AV_NOPTS_VALUE || dts - ts_st2->payload_dts > delay/2)) {
                mpegts_write_pes(s, st2, ts_st2->payload, ts_st2->payload_size,
                                 ts_st2->payload_pts, ts_st2->payload_dts,
                                 ts_st2->payload_flags & AV_PKT_FLAG_KEY, stream_id);
                ts_st2->payload_size = 0;
            }
        }
    }

    if (ts_st->payload_size && (ts_st->payload_size + size > ts->pes_payload_size ||
        (dts != AV_NOPTS_VALUE && ts_st->payload_dts != AV_NOPTS_VALUE &&
         av_compare_ts(dts - ts_st->payload_dts, st->time_base,
                       s->max_delay, AV_TIME_BASE_Q) >= 0) ||
        ts_st->opus_queued_samples + opus_samples >= 5760 /* 120ms */)) {
        mpegts_write_pes(s, st, ts_st->payload, ts_st->payload_size,
                         ts_st->payload_pts, ts_st->payload_dts,
                         ts_st->payload_flags & AV_PKT_FLAG_KEY, stream_id);
        ts_st->payload_size = 0;
        ts_st->opus_queued_samples = 0;
    }

    if (st->codecpar->codec_type != AVMEDIA_TYPE_AUDIO || size > ts->pes_payload_size) {
        av_assert0(!ts_st->payload_size);
        // for video and subtitle, write a single pes packet
        mpegts_write_pes(s, st, buf, size, pts, dts,
                         pkt->flags & AV_PKT_FLAG_KEY, stream_id);
        ts_st->opus_queued_samples = 0;
        av_free(data);
        return 0;
    }

    if (!ts_st->payload_size) {
        ts_st->payload_pts   = pts;
        ts_st->payload_dts   = dts;
        ts_st->payload_flags = pkt->flags;
    }

    memcpy(ts_st->payload + ts_st->payload_size, buf, size);
    ts_st->payload_size += size;
    ts_st->opus_queued_samples += opus_samples;

    av_free(data);
#endif

    return 0;
}

static void mpegts_write_flush(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    int i;

    /* flush current packets */
    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        if (ts_st->payload_size > 0) {
            mpegts_write_pes(s, st, ts_st->payload, ts_st->payload_size,
                             ts_st->payload_pts, ts_st->payload_dts,
                             ts_st->payload_flags & AV_PKT_FLAG_KEY, -1);
            ts_st->payload_size = 0;
            ts_st->opus_queued_samples = 0;
        }
    }

    if (!ts->tsi_is_realtime)
        tsi_drain_interleaving_buffer(s, AV_NOPTS_VALUE, 1);
}

static int mpegts_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    if (!pkt) {
        mpegts_write_flush(s);
        return 1;
    } else {
        return mpegts_write_packet_internal(s, pkt);
    }
}

static int mpegts_write_end(AVFormatContext *s)
{
    if (s->pb)
        mpegts_write_flush(s);

    return 0;
}

static void mpegts_deinit(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService *service;
    int i;

    //FIXME: check this
    {
        int ret;
        if (!ts->tsi_is_realtime) {
            ts->tsi_thread_exit = 1;
            ret = pthread_join(ts->tsi_thread, NULL);
            if (ret != 0) {
                //av_log(h, AV_LOG_ERROR, "pthread_join(): %s\n", strerror(ret));
            }
        }
        pthread_mutex_destroy(&ts->tsi_mutex);
        pthread_cond_destroy(&ts->tsi_cond);
    }

    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        if (ts_st) {
            for (MpegTSPesPacket *pkt = tsi_buffer_head(ts_st); pkt; pkt = tsi_buffer_next(ts_st, pkt))
                av_free(pkt->payload);

            av_freep(&ts_st->payload);
            if (ts_st->amux) {
                avformat_free_context(ts_st->amux);
                ts_st->amux = NULL;
            }
        }
    }

    for (i = 0; i < ts->nb_services; i++) {
        service = ts->services[i];
        av_freep(&service->provider_name);
        av_freep(&service->name);
        av_freep(&service);
    }
    av_freep(&ts->services);
}

static int mpegts_check_bitstream(struct AVFormatContext *s, const AVPacket *pkt)
{
    int ret = 1;
    AVStream *st = s->streams[pkt->stream_index];

    if (st->codecpar->codec_id == AV_CODEC_ID_H264) {
        if (pkt->size >= 5 && AV_RB32(pkt->data) != 0x0000001 &&
                             (AV_RB24(pkt->data) != 0x000001 ||
                              (st->codecpar->extradata_size > 0 &&
                               st->codecpar->extradata[0] == 1)))
            ret = ff_stream_add_bitstream_filter(st, "h264_mp4toannexb", NULL);
    } else if (st->codecpar->codec_id == AV_CODEC_ID_HEVC) {
        if (pkt->size >= 5 && AV_RB32(pkt->data) != 0x0000001 &&
                             (AV_RB24(pkt->data) != 0x000001 ||
                              (st->codecpar->extradata_size > 0 &&
                               st->codecpar->extradata[0] == 1)))
            ret = ff_stream_add_bitstream_filter(st, "hevc_mp4toannexb", NULL);
    }

    return ret;
}

static const AVOption options[] = {
    { "mpegts_transport_stream_id", "Set transport_stream_id field.",
      offsetof(MpegTSWrite, transport_stream_id), AV_OPT_TYPE_INT,
      { .i64 = 0x0001 }, 0x0001, 0xffff, AV_OPT_FLAG_ENCODING_PARAM },
    { "mpegts_original_network_id", "Set original_network_id field.",
      offsetof(MpegTSWrite, original_network_id), AV_OPT_TYPE_INT,
      { .i64 = DVB_PRIVATE_NETWORK_START }, 0x0001, 0xffff, AV_OPT_FLAG_ENCODING_PARAM },
    { "mpegts_service_id", "Set service_id field.",
      offsetof(MpegTSWrite, service_id), AV_OPT_TYPE_INT,
      { .i64 = 0x0001 }, 0x0001, 0xffff, AV_OPT_FLAG_ENCODING_PARAM },
    { "mpegts_service_type", "Set service_type field.",
      offsetof(MpegTSWrite, service_type), AV_OPT_TYPE_INT,
      { .i64 = 0x01 }, 0x01, 0xff, AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "digital_tv", "Digital Television.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_DIGITAL_TV }, 0x01, 0xff,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "digital_radio", "Digital Radio.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_DIGITAL_RADIO }, 0x01, 0xff,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "teletext", "Teletext.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_TELETEXT }, 0x01, 0xff,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "advanced_codec_digital_radio", "Advanced Codec Digital Radio.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_RADIO }, 0x01, 0xff,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "mpeg2_digital_hdtv", "MPEG2 Digital HDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_MPEG2_DIGITAL_HDTV }, 0x01, 0xff,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "advanced_codec_digital_sdtv", "Advanced Codec Digital SDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_SDTV }, 0x01, 0xff,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "advanced_codec_digital_hdtv", "Advanced Codec Digital HDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_HDTV }, 0x01, 0xff,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "hevc_digital_hdtv", "HEVC Digital Television Service.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_HEVC_DIGITAL_HDTV }, 0x01, 0xff,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_service_type" },
    { "mpegts_pmt_start_pid", "Set the first pid of the PMT.",
      offsetof(MpegTSWrite, pmt_start_pid), AV_OPT_TYPE_INT,
      { .i64 = 0x1000 }, 0x0010, 0x1f00, AV_OPT_FLAG_ENCODING_PARAM },
    { "mpegts_start_pid", "Set the first pid.",
      offsetof(MpegTSWrite, start_pid), AV_OPT_TYPE_INT,
      { .i64 = 0x0100 }, 0x0010, 0x0f00, AV_OPT_FLAG_ENCODING_PARAM },
    { "mpegts_m2ts_mode", "Enable m2ts mode.",
      offsetof(MpegTSWrite, m2ts_mode), AV_OPT_TYPE_BOOL,
      { .i64 = -1 }, -1, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { "muxrate", NULL,
      offsetof(MpegTSWrite, mux_rate), AV_OPT_TYPE_INT,
      { .i64 = 1 }, 0, INT_MAX, AV_OPT_FLAG_ENCODING_PARAM },
    { "pes_payload_size", "Minimum PES packet payload in bytes",
      offsetof(MpegTSWrite, pes_payload_size), AV_OPT_TYPE_INT,
      { .i64 = DEFAULT_PES_PAYLOAD_SIZE }, 0, INT_MAX, AV_OPT_FLAG_ENCODING_PARAM },
    { "mpegts_flags", "MPEG-TS muxing flags",
      offsetof(MpegTSWrite, flags), AV_OPT_TYPE_FLAGS, { .i64 = 0 }, 0, INT_MAX,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_flags" },
    { "resend_headers", "Reemit PAT/PMT before writing the next packet",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_REEMIT_PAT_PMT }, 0, INT_MAX,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_flags" },
    { "latm", "Use LATM packetization for AAC",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_AAC_LATM }, 0, INT_MAX,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_flags" },
    { "pat_pmt_at_frames", "Reemit PAT and PMT at each video frame",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_PAT_PMT_AT_FRAMES}, 0, INT_MAX,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_flags" },
    { "system_b", "Conform to System B (DVB) instead of System A (ATSC)",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_SYSTEM_B }, 0, INT_MAX,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_flags" },
    { "initial_discontinuity", "Mark initial packets as discontinuous",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_DISCONT }, 0, INT_MAX,
      AV_OPT_FLAG_ENCODING_PARAM, "mpegts_flags" },
    // backward compatibility
    { "resend_headers", "Reemit PAT/PMT before writing the next packet",
      offsetof(MpegTSWrite, reemit_pat_pmt), AV_OPT_TYPE_INT,
      { .i64 = 0 }, 0, INT_MAX, AV_OPT_FLAG_ENCODING_PARAM },
    { "mpegts_copyts", "don't offset dts/pts",
      offsetof(MpegTSWrite, copyts), AV_OPT_TYPE_BOOL,
      { .i64 = -1 }, -1, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { "tables_version", "set PAT, PMT and SDT version",
      offsetof(MpegTSWrite, tables_version), AV_OPT_TYPE_INT,
      { .i64 = 0 }, 0, 31, AV_OPT_FLAG_ENCODING_PARAM },
    { "omit_video_pes_length", "Omit the PES packet length for video packets",
      offsetof(MpegTSWrite, omit_video_pes_length), AV_OPT_TYPE_BOOL,
      { .i64 = 1 }, 0, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { "pcr_period", "PCR retransmission time in milliseconds",
      offsetof(MpegTSWrite, pcr_period), AV_OPT_TYPE_INT,
      { .i64 = PCR_RETRANS_TIME }, 0, INT_MAX, AV_OPT_FLAG_ENCODING_PARAM },
    { "pat_period", "PAT/PMT retransmission time limit in seconds",
      offsetof(MpegTSWrite, pat_period), AV_OPT_TYPE_DOUBLE,
      { .dbl = INT_MAX }, 0, INT_MAX, AV_OPT_FLAG_ENCODING_PARAM },
    { "sdt_period", "SDT retransmission time limit in seconds",
      offsetof(MpegTSWrite, sdt_period), AV_OPT_TYPE_DOUBLE,
      { .dbl = INT_MAX }, 0, INT_MAX, AV_OPT_FLAG_ENCODING_PARAM },
    { "mpeg_realtime", "Realtime streaming (flush_packets should be 0 for realtime streaming)",
      offsetof(MpegTSWrite, tsi_is_realtime), AV_OPT_TYPE_BOOL,
      { .i64 = 0}, 0, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { NULL },
};

static const AVClass mpegts_muxer_class = {
    .class_name = "MPEGTS muxer",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVOutputFormat ff_mpegts_muxer = {
    .name           = "mpegts",
    .long_name      = NULL_IF_CONFIG_SMALL("MPEG-TS (MPEG-2 Transport Stream)"),
    .mime_type      = "video/MP2T",
    .extensions     = "ts,m2t,m2ts,mts",
    .priv_data_size = sizeof(MpegTSWrite),
    .audio_codec    = AV_CODEC_ID_MP2,
    .video_codec    = AV_CODEC_ID_MPEG2VIDEO,
    .init           = mpegts_init,
    .write_packet   = mpegts_write_packet,
    .write_trailer  = mpegts_write_end,
    .interleave_packet = tsi_interleave_packet,
    .deinit         = mpegts_deinit,
    .check_bitstream = mpegts_check_bitstream,
    .flags          = AVFMT_ALLOW_FLUSH | AVFMT_VARIABLE_FPS,
    .priv_class     = &mpegts_muxer_class,
};
