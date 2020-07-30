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
    uint8_t name[256];
    uint8_t provider_name[256];
    int pcr_pid;
    AVProgram *program;

    struct {
        int64_t time_offset;
        int is_buffered;
        int64_t max_buffer_duration;
        int64_t min_time;
        struct MpegTSWriteStream *min_time_ts_st;
        int64_t last_pcr;
        int64_t last_pcr_bytes;
        int64_t last_pcr_remainder;
        //debug
        int64_t last_pcr_adjustment;
        int64_t last_packet_realtime;
    } tsi;
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
    int64_t sdt_period; /* SDT period in PCR time base */
    int64_t pat_period; /* PAT/PMT period in PCR time base */
    int nb_services;
    int64_t first_pcr;
    int64_t next_pcr;
    int mux_rate; ///< set to 1 when VBR
    int pes_payload_size;

    int transport_stream_id;
    int original_network_id;
    int service_id;
    int service_type;

    int pmt_start_pid;
    int start_pid;
    int m2ts_mode;
    int m2ts_video_pid;
    int m2ts_audio_pid;
    int m2ts_pgssub_pid;
    int m2ts_textsub_pid;

    int pcr_period_ms;
#define MPEGTS_FLAG_REEMIT_PAT_PMT  0x01
#define MPEGTS_FLAG_AAC_LATM        0x02
#define MPEGTS_FLAG_PAT_PMT_AT_FRAMES           0x04
#define MPEGTS_FLAG_SYSTEM_B        0x08
#define MPEGTS_FLAG_DISCONT         0x10
    int flags;
    int copyts;
    int tables_version;
    int64_t pat_period_us;
    int64_t sdt_period_us;
    int64_t last_pat_ts;
    int64_t last_sdt_ts;

    int omit_video_pes_length;

    struct {
        int64_t ts_time;
        int64_t last_pat_time;
        int64_t last_sdt_time;
        int64_t last_muxrate_time;
        int64_t last_muxrate_bytes;
        int64_t buffer_report_realtime; // debug

        int is_active;
        int is_realtime;
        int sync_service_times;
#if HAVE_PTHREADS
        pthread_t thread;
        pthread_mutex_t mutex;
        pthread_cond_t cond;
        volatile int thread_exit;
#endif
    } tsi;
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
#define DEFAULT_SERVICE_NAME    "Service"

/* we retransmit the SI info at this rate */
#define SDT_RETRANS_TIME 500
#define PAT_RETRANS_TIME 100
#define PCR_RETRANS_TIME 20

typedef struct MpegTSPesPacket {
    AVBufferRef *buf;
    uint8_t *payload;
    int64_t pts;
    int64_t dts;
    int payload_size;
    int guessed_paketized_size;
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
    int data_st_warning;

    int64_t pcr_period; /* PCR period in PCR time base */
    int64_t last_pcr;

    /* For Opus */
    int opus_queued_samples;
    int opus_pending_trim_start;

    struct {
        MpegTSPesPacket packets[1024];
        int64_t packets_head;
        int64_t packets_tail;
        int64_t time;
        int64_t buffer_duration;
        int64_t buffer_bytes;
        int64_t packet_since;
        int64_t packet_till;
        int packet_consumed_bytes;
        int last_packet_key;
    } tsi;
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
    mpegts_write_section1(&ts->pat, PAT_TID, ts->transport_stream_id, ts->tables_version, 0, 0,
                          data, q - data);
}

static void putbuf(uint8_t **q_ptr, const uint8_t *buf, size_t len)
{
    memcpy(*q_ptr, buf, len);
    *q_ptr += len;
}

static void put_registration_descriptor(uint8_t **q_ptr, uint32_t tag)
{
    uint8_t *q = *q_ptr;
    *q++ = 0x05; /* MPEG-2 registration descriptor*/
    *q++ = 4;
    *q++ = tag;
    *q++ = tag >> 8;
    *q++ = tag >> 16;
    *q++ = tag >> 24;
    *q_ptr = q;
}

static int get_dvb_stream_type(AVFormatContext *s, AVStream *st)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = st->priv_data;
    int stream_type;

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
    case AV_CODEC_ID_DVB_SUBTITLE:
    case AV_CODEC_ID_DVB_TELETEXT:
        stream_type = STREAM_TYPE_PRIVATE_DATA;
        break;
    case AV_CODEC_ID_SMPTE_KLV:
        if (st->codecpar->profile == FF_PROFILE_KLVA_SYNC) {
            stream_type = STREAM_TYPE_METADATA;
        } else {
            stream_type = STREAM_TYPE_PRIVATE_DATA;
        }
        break;
    default:
        av_log_once(s, AV_LOG_WARNING, AV_LOG_DEBUG, &ts_st->data_st_warning,
                    "Stream %d, codec %s, is muxed as a private data stream "
                    "and may not be recognized upon reading.\n", st->index,
                    avcodec_get_name(st->codecpar->codec_id));
        stream_type = STREAM_TYPE_PRIVATE_DATA;
        break;
    }

    return stream_type;
}

static int get_m2ts_stream_type(AVFormatContext *s, AVStream *st)
{
    int stream_type;
    MpegTSWriteStream *ts_st = st->priv_data;

    switch (st->codecpar->codec_id) {
    case AV_CODEC_ID_MPEG2VIDEO:
        stream_type = STREAM_TYPE_VIDEO_MPEG2;
        break;
    case AV_CODEC_ID_H264:
        stream_type = STREAM_TYPE_VIDEO_H264;
        break;
    case AV_CODEC_ID_VC1:
        stream_type = STREAM_TYPE_VIDEO_VC1;
        break;
    case AV_CODEC_ID_HEVC:
        stream_type = STREAM_TYPE_VIDEO_HEVC;
        break;
    case AV_CODEC_ID_PCM_BLURAY:
        stream_type = 0x80;
        break;
    case AV_CODEC_ID_AC3:
        stream_type = 0x81;
        break;
    case AV_CODEC_ID_DTS:
        stream_type = (st->codecpar->channels > 6) ? 0x85 : 0x82;
        break;
    case AV_CODEC_ID_TRUEHD:
        stream_type = 0x83;
        break;
    case AV_CODEC_ID_EAC3:
        stream_type = 0x84;
        break;
    case AV_CODEC_ID_HDMV_PGS_SUBTITLE:
        stream_type = 0x90;
        break;
    case AV_CODEC_ID_HDMV_TEXT_SUBTITLE:
        stream_type = 0x92;
        break;
    default:
        av_log_once(s, AV_LOG_WARNING, AV_LOG_DEBUG, &ts_st->data_st_warning,
                    "Stream %d, codec %s, is muxed as a private data stream "
                    "and may not be recognized upon reading.\n", st->index,
                    avcodec_get_name(st->codecpar->codec_id));
        stream_type = STREAM_TYPE_PRIVATE_DATA;
        break;
    }

    return stream_type;
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
    if (ts->m2ts_mode) {
        put_registration_descriptor(&q, MKTAG('H', 'D', 'M', 'V'));
        *q++ = 0x88;        // descriptor_tag - hdmv_copy_control_descriptor
        *q++ = 0x04;        // descriptor_length
        put16(&q, 0x0fff);  // CA_System_ID
        *q++ = 0xfc;        // private_data_byte
        *q++ = 0xfc;        // private_data_byte
    }

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

        stream_type = ts->m2ts_mode ? get_m2ts_stream_type(s, st) : get_dvb_stream_type(s, st);

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
            if (st->codecpar->codec_id==AV_CODEC_ID_S302M)
                put_registration_descriptor(&q, MKTAG('B', 'S', 'S', 'D'));
            if (st->codecpar->codec_id==AV_CODEC_ID_OPUS) {
                /* 6 bytes registration descriptor, 4 bytes Opus audio descriptor */
                if (q - data > SECTION_LENGTH - 6 - 4) {
                    err = 1;
                    break;
                }

                put_registration_descriptor(&q, MKTAG('O', 'p', 'u', 's'));

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
                put_registration_descriptor(&q, MKTAG('d', 'r', 'a', 'c'));
            } else if (stream_type == STREAM_TYPE_VIDEO_VC1) {
                put_registration_descriptor(&q, MKTAG('V', 'C', '-', '1'));
            } else if (stream_type == STREAM_TYPE_VIDEO_HEVC && s->strict_std_compliance <= FF_COMPLIANCE_NORMAL) {
                put_registration_descriptor(&q, MKTAG('H', 'E', 'V', 'C'));
            }
            break;
        case AVMEDIA_TYPE_DATA:
            if (st->codecpar->codec_id == AV_CODEC_ID_SMPTE_KLV) {
                put_registration_descriptor(&q, MKTAG('K', 'L', 'V', 'A'));
            } else if (st->codecpar->codec_id == AV_CODEC_ID_TIMED_ID3) {
                const char *tag = "ID3 ";
                *q++ = 0x26; /* metadata descriptor */
                *q++ = 13;
                put16(&q, 0xffff);    /* metadata application format */
                putbuf(&q, tag, strlen(tag));
                *q++ = 0xff;        /* metadata format */
                putbuf(&q, tag, strlen(tag));
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
    put16(&q, ts->original_network_id);
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
        putbuf(&q, service->provider_name, service->provider_name[0] + 1);
        putbuf(&q, service->name, service->name[0] + 1);
        desc_len_ptr[0] = q - desc_len_ptr - 1;

        /* fill descriptor length */
        val = (running_status << 13) | (free_ca_mode << 12) |
              (q - desc_list_len_ptr - 2);
        desc_list_len_ptr[0] = val >> 8;
        desc_list_len_ptr[1] = val;
    }
    mpegts_write_section1(&ts->sdt, SDT_TID, ts->transport_stream_id, ts->tables_version, 0, 0,
                          data, q - data);
}

/* This stores a string in buf with the correct encoding and also sets the
 * first byte as the length. !str is accepted for an empty string.
 * If the string is already encoded, invalid UTF-8 or has no multibyte sequence
 * then we keep it as is, otherwise we signal UTF-8 encoding. */
static int encode_str8(uint8_t *buf, const char *str)
{
    size_t str_len;
    if (!str)
        str = "";
    str_len = strlen(str);
    if (str[0] && (unsigned)str[0] >= 0x20) {   /* Make sure the string is not already encoded. */
        const uint8_t *q = str;
        int has_multibyte = 0;
        while (*q) {
            uint32_t code;
            GET_UTF8(code, *q++, goto invalid;) /* Is it valid UTF-8? */
            has_multibyte |= (code > 127);      /* Does it have multibyte UTF-8 chars in it? */
        }
        if (has_multibyte) {                    /* If we have multibyte chars and valid UTF-8, then encode as such! */
            if (str_len > 254)
                return AVERROR(EINVAL);
            buf[0] = str_len + 1;
            buf[1] = 0x15;
            memcpy(&buf[2], str, str_len);
            return 0;
        }
    }
invalid:
    /* Otherwise let's just encode the string as is! */
    if (str_len > 255)
        return AVERROR(EINVAL);
    buf[0] = str_len;
    memcpy(&buf[1], str, str_len);
    return 0;
}

static int64_t get_pcr(const MpegTSWrite *ts, AVIOContext *pb)
{
    return av_rescale(avio_tell(pb) + 11, 8 * PCR_TIME_BASE, ts->mux_rate) +
           ts->first_pcr;
}

static void write_packet(AVFormatContext *s, const uint8_t *packet)
{
    MpegTSWrite *ts = s->priv_data;
    if (ts->m2ts_mode) {
        int64_t pcr = get_pcr(s->priv_data, s->pb);
        uint32_t tp_extra_header = pcr % 0x3fffffff;
        tp_extra_header = AV_RB32(&tp_extra_header);
        avio_write(s->pb, (unsigned char *) &tp_extra_header,
                   sizeof(tp_extra_header));
    }
    avio_write(s->pb, packet, TS_PACKET_SIZE);
}

static void section_write_packet(MpegTSSection *s, const uint8_t *packet)
{
    AVFormatContext *ctx = s->opaque;
    write_packet(ctx, packet);
}

/* TSI START */
static void mpegts_insert_null_packet(AVFormatContext *s);
static void mpegts_write_pes(AVFormatContext *s, AVStream *st,
                             const uint8_t *payload, int payload_size,
                             int64_t pts, int64_t dts, int key, int stream_id);

//TODO: AVOption ?
//FIXME: write constraints/dependencies for this values
static const int64_t TSI_MAX_AUDIO_PACKET_DURATION =  120 * 90000LL / 1000;
static const int64_t TSI_DISCONTINUITY_THRESHOLD   = 1000 * 90000LL / 1000;
static const int64_t TSI_WINDOW        =   700 * 90000LL / 1000;
static const int64_t TSI_BUFFER_LOW    =  2000 * 90000LL / 1000;
static const int64_t TSI_BUFFER_TARGET =  4000 * 90000LL / 1000;
static const int64_t TSI_BUFFER_HIGH   = 15000 * 90000LL / 1000;
static const int64_t TSI_SDT_PERIOD    =  1000 * 90000LL / 1000;
static const int64_t TSI_PAT_PERIOD    =    90 * 90000LL / 1000;
static const int64_t TSI_PCR_PERIOD         = 35 * 27000000LL / 1000; // pcr period in 27MHz units
static const int64_t TSI_MAX_PCR_ADJUSTMENT =  5 * 27000000LL / 1000; // in 27MHz units

#define tsi_log(s, level, format, ...) \
    av_log(s, level, "[tsi] t#%.3f " format, \
    ((MpegTSWrite*)s->priv_data)->tsi.ts_time/90000.0, ##__VA_ARGS__)

#define tsi_log_svc(s, svc, level, format, ...) \
    tsi_log(s, level, "service#%x " format, svc->sid, ##__VA_ARGS__)

#define tsi_log_ts_st(s, ts_st, level, format, ...) \
    tsi_log_svc(s, ts_st->service, level, "st#%.3f pid#%x " format, \
    ts_st->tsi.time/90000.0, ts_st->pid, ##__VA_ARGS__)
/*
__attribute__ ((format (printf, 4, 5)))
static void tsi_log(AVFormatContext* s, MpegTSWriteStream* ts_st, int level, const char *fmt, ...)
{
    char fmt1[4096];
    snprintf(fmt1, sizeof(fmt1), "[tsi] service#%x pid#%x %s", ts_st->service->sid, ts_st->pid, fmt);
    va_list vl;
    va_start(vl, fmt);
    av_vlog(s, level, fmt, vl);
    va_end(vl);
}
*/

static size_t tsi_buffer_size(MpegTSWriteStream *ts_st)
{
    return (size_t)(ts_st->tsi.packets_tail - ts_st->tsi.packets_head);
}

static size_t tsi_buffer_capacity(MpegTSWriteStream *ts_st)
{
    return sizeof(ts_st->tsi.packets) / sizeof(ts_st->tsi.packets[0]);
}

static MpegTSPesPacket* tsi_buffer_head(MpegTSWriteStream *ts_st)
{
    if (tsi_buffer_size(ts_st) == 0)
        return NULL;
    return &ts_st->tsi.packets[(unsigned)ts_st->tsi.packets_head % tsi_buffer_capacity(ts_st)];
}

static MpegTSPesPacket* tsi_buffer_tail(MpegTSWriteStream *ts_st)
{
    return &ts_st->tsi.packets[(unsigned)(ts_st->tsi.packets_tail - 1) % tsi_buffer_capacity(ts_st)];
}

static MpegTSPesPacket* tsi_buffer_next(MpegTSWriteStream *ts_st, MpegTSPesPacket* current)
{
    if (current == tsi_buffer_tail(ts_st))
        return NULL;
    return &ts_st->tsi.packets[(unsigned)(current - ts_st->tsi.packets + 1) % tsi_buffer_capacity(ts_st) ];
}

static int64_t tsi_get_adjusted_pcr(AVFormatContext* s, MpegTSWriteStream *ts_st, int force)
{
    MpegTSWrite *ts = s->priv_data;
    int64_t pcr = ts_st->tsi.time * 300;
    int64_t bytes;
    MpegTSService *service = ts_st->service;
    if (ts->tsi.sync_service_times) {
        pcr += ts_st->service->tsi.time_offset * 300;
        service = ts->services[0];
    }
    if (service->tsi.last_pcr + TSI_PCR_PERIOD - TSI_MAX_PCR_ADJUSTMENT > pcr && !force)
        return AV_NOPTS_VALUE;
    if (ts->mux_rate <= 1) {
        service->tsi.last_pcr = pcr;
        return pcr;
    }
    bytes = avio_tell(s->pb);
    pcr -=  TSI_MAX_PCR_ADJUSTMENT;
    if (service->tsi.last_pcr != AV_NOPTS_VALUE){
        int64_t adjustment = 0;
        int64_t bytes_diff = bytes - service->tsi.last_pcr_bytes;
        int64_t pcr_diff = pcr - service->tsi.last_pcr;
        int64_t tmp = service->tsi.last_pcr_remainder + bytes_diff * 8 * PCR_TIME_BASE;
        int64_t expected_pcr_diff = tmp / ts->mux_rate;
        service->tsi.last_pcr_remainder = tmp % ts->mux_rate;
        adjustment = expected_pcr_diff - pcr_diff;
        if (llabs(adjustment) >= TSI_MAX_PCR_ADJUSTMENT) {
            tsi_log_ts_st(s, ts_st, AV_LOG_WARNING, "pcr ajustment exceeds limit, resetting to zero (adj=%9.6fsec, last_adj=%.6fsec, pcr_diff=%.6fsec, bytes_diff=%d)\n",
                   adjustment / (300 * 90000.0),
                   service->tsi.last_pcr_adjustment / (300 * 90000.0),
                   (pcr - service->tsi.last_pcr) / (300 * 90000.0),
                   (int)bytes_diff);
            adjustment = 0;
        }
        pcr += adjustment;
        service->tsi.last_pcr_adjustment = adjustment;
    }
    service->tsi.last_pcr_bytes = bytes;
    service->tsi.last_pcr = pcr;
    return pcr;
}

static int64_t tsi_guess_paketized_size(int64_t pes_size)
{
    // TODO: check this
    // first ts packet have room for ~170 payload bytes, other ~184
    return 188 * (1 + (FFMAX(0, pes_size-170)+184-1)/184 );
}

typedef struct TSIRational {
    int64_t num, den;
} TSIRational;

static int tsi_rational_compare(TSIRational a, TSIRational b) {
    int64_t aa = a.num * b.den, bb = b.num * a.den;
    return aa < bb ? -1 : (aa > bb ? 1 : 0);
}

static void tsi_schedule_first_packet(AVFormatContext *s, AVStream *st)
{
    //TODO: a/v offsets?
    MpegTSWrite* ts = s->priv_data;
    MpegTSWriteStream* ts_st = st->priv_data;

    //ts_st->tsi.packet_end_time = tsi_buffer_head(ts_st)->dts;
    //return;
    MpegTSPesPacket *pkt = tsi_buffer_head(ts_st);
    int64_t first_bytes = pkt->guessed_paketized_size;
    int64_t bytes = first_bytes;
    int64_t duration = pkt->dts - ts_st->tsi.packet_since;
    TSIRational min_bitrate = {0,1}, max_bitrate = {1,0}; // [0, +infinity]
    do {
        MpegTSPesPacket *next = tsi_buffer_next(ts_st, pkt);
        TSIRational new_min = {bytes, duration};
        TSIRational new_max = {bytes, FFMAX(0, duration-TSI_WINDOW)};
        if (tsi_rational_compare(new_min, min_bitrate) > 0) {
            if (tsi_rational_compare(new_min, max_bitrate) > 0) {
                //tsi_log_ts_st(s, ts_st, AV_LOG_INFO, "INTERESTING1\n");
                break;
            }
            min_bitrate = new_min;
        }
        if (tsi_rational_compare(new_max, max_bitrate) < 0) {
            if (tsi_rational_compare(new_max, min_bitrate) < 0) {
                //tsi_log_ts_st(s, ts_st, AV_LOG_INFO, "INTERESTING2\n");
                break;
            }
            max_bitrate = new_max;
        }
        if (!next)
            break;
        bytes += next->guessed_paketized_size;
        if (next->dts - pkt->dts < TSI_DISCONTINUITY_THRESHOLD) // FIXME: what else? += prev delta?
            duration += next->dts - pkt->dts;
        pkt = next;
    } while (duration<TSI_BUFFER_TARGET);
    //FIXME: rounding?
    ts_st->tsi.packet_till = ts_st->tsi.packet_since + first_bytes * min_bitrate.den / min_bitrate.num;
    /*
    tsi_log_ts_st(s, ts_st, AV_LOG_WARNING, "tsi_schedule_first_packet: bytes=%7d/%7d dts=%9.3f time=%9.3f - %9.3f, dts_diff=%9.3f\n",
           first_bytes, bytes,
           tsi_buffer_head(ts_st)->dts / 90000.0,
           ts_st->packet_start_time / 90000.0,
           ts_st->packet_end_time / 90000.0,
           (tsi_buffer_head(ts_st)->dts - ts_st->packet_end_time) / 90000.0
           );
    */
    //FIXME: log it when PCR ajustment failed?
    //if (max_time + TSI_WINDOW == tsi_buffer_head(ts_st)->dts && ts->mux_rate > 1)
    //  av_log(s, AV_LOG_WARNING, "[tsi] [pid 0x%x] Consider increasing TSI_WINDOW: offset=%.3fsec\n", ts_st->pid,
    //  (tsi_buffer_head(ts_st)->dts - max_time) / 90000.0);
    { //debug
        MpegTSPesPacket *pkt = tsi_buffer_head(ts_st);
        if (ts_st->tsi.packet_since > ts_st->tsi.packet_till
                || ts_st->tsi.packet_till + TSI_WINDOW < pkt->dts
                || ts_st->tsi.packet_till > pkt->dts)
            tsi_log_ts_st(s, ts_st, AV_LOG_ERROR, "[WTF] pes packet scheduled %.3fsec - %.3fsec (dts=%.3fsec)\n",
                   ts_st->tsi.packet_since /90000.0, ts_st->tsi.packet_till / 90000.0, pkt->dts / 90000.0);
    }
}

static int tsi_dbg_snprintf_buffers(char* buf, size_t size, AVFormatContext *s, MpegTSService *service, MpegTSWriteStream* ts_st)
{
    int offs = 0;
    for (unsigned j = 0; j < s->nb_streams; j++) {
        MpegTSWriteStream *ts_st2 = s->streams[j]->priv_data;
        int cnt;
        if((service && service != ts_st2->service) || (ts_st && ts_st != ts_st2))
            continue;
        cnt = snprintf(buf + offs, size - FFMIN(size, (size_t)offs),
                        offs == 0 ? "pid#%x %.3fsec/%dKB/%dpkt" : " pid#%x %.3f/%d/%d",
                        ts_st2->pid, ts_st2->tsi.buffer_duration / 90000.0, (int)ts_st2->tsi.buffer_bytes / 1024,
                        (int)tsi_buffer_size(ts_st2));
        if (cnt < 0)
            return cnt;
        offs += cnt;
    }
    return offs;
}

static void tsi_schedule_services(AVFormatContext *s, int flush)
{
    MpegTSWrite *ts = s->priv_data;

    for (int i = 0; i < ts->nb_services; i++) {
        ts->services[i]->tsi.max_buffer_duration = 0;
        ts->services[i]->tsi.min_time = INT64_MAX;
        ts->services[i]->tsi.min_time_ts_st = NULL;
    }
    for (int i = 0; i < s->nb_streams; i++){
        MpegTSWriteStream *ts_st = s->streams[i]->priv_data;
        MpegTSService *service = ts_st->service;
        //FIXME: remove empty packets here ?
        if (ts_st->tsi.time == AV_NOPTS_VALUE) {
            if(tsi_buffer_size(ts_st)==0)
                continue;
            // We don't know how to properly redistribute time in a first packet.
            // Assume worst case - it's a huge packet occupying full TSI_WINDOW.
            ts_st->tsi.time = ts_st->tsi.packet_since = tsi_buffer_head(ts_st)->dts - TSI_WINDOW;
        }
        service->tsi.max_buffer_duration = FFMAX(service->tsi.max_buffer_duration, ts_st->tsi.buffer_duration);
        if (ts_st->tsi.time < service->tsi.min_time) {
            service->tsi.min_time = ts_st->tsi.time;
            service->tsi.min_time_ts_st = ts_st;
        }
    }
    for (int i = 0; i < ts->nb_services; i++) {
        MpegTSService *service = ts->services[i];
        if (service->tsi.is_buffered && service->tsi.max_buffer_duration < TSI_BUFFER_LOW) {
            if (!flush) {
                char buf[4096];
                tsi_dbg_snprintf_buffers(buf, sizeof(buf), s, service, NULL);
                tsi_log_svc(s, service, AV_LOG_WARNING, "buffering started : %s\n", buf);
            }
            service->tsi.is_buffered = 0;
        }
        if (!service->tsi.is_buffered && service->tsi.max_buffer_duration > TSI_BUFFER_TARGET) {
            char buf[4096];
            tsi_dbg_snprintf_buffers(buf, sizeof(buf), s, service, NULL);
            tsi_log_svc(s, service, AV_LOG_INFO, "buffering complete: %s\n", buf);
            service->tsi.is_buffered = 1;
            service->tsi.time_offset = ts->tsi.ts_time - service->tsi.min_time;
        }
        if (service->tsi.min_time != INT64_MAX && (service->tsi.is_buffered || flush) ) { //TODO: check this only on new packet||flush ?
            if (flush && service->tsi.time_offset == AV_NOPTS_VALUE) {
                service->tsi.time_offset = ts->tsi.ts_time - service->tsi.min_time;
            } else if (ts->tsi.ts_time > service->tsi.min_time + service->tsi.time_offset) {
                tsi_log_ts_st(s, service->tsi.min_time_ts_st, AV_LOG_ERROR, "incorrect stream interleaving. Time jumped back (time=%.3fsec diff=%.3fsec)\n",
                       ts->tsi.ts_time / 90000.0, -(service->tsi.min_time + service->tsi.time_offset - ts->tsi.ts_time) / 90000.0);
                service->tsi.time_offset = ts->tsi.ts_time - service->tsi.min_time;
            }
            if (ts->tsi.ts_time + TSI_WINDOW < service->tsi.min_time + service->tsi.time_offset) { // TODO: TSI_WINDOW vs new const?
                tsi_log_ts_st(s, service->tsi.min_time_ts_st, AV_LOG_WARNING, "detected large gap in playback, resyncing (time=%3fsec diff=%.3fsec)\n",
                       ts->tsi.ts_time / 90000.0, (service->tsi.min_time + service->tsi.time_offset - ts->tsi.ts_time) / 90000.0);
                service->tsi.time_offset = ts->tsi.ts_time - service->tsi.min_time;
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

        if (!ts_st2->service->tsi.is_buffered && !flush) continue;
        if(ts_st2->tsi.time == AV_NOPTS_VALUE) continue;

        if(!st || ts_st2->tsi.time + ts_st2->service->tsi.time_offset < ts_st->tsi.time + ts_st->service->tsi.time_offset) {
            st = st2;
            ts_st = ts_st2;
        }
    }
    return st;
}

static void tsi_remove_finished_packet(AVFormatContext *s, MpegTSWriteStream *ts_st)
{
    MpegTSPesPacket *old_head = tsi_buffer_head(ts_st);
    ts_st->tsi.last_packet_key = old_head->key;
    ts_st->tsi.buffer_bytes -= old_head->payload_size;
    if(old_head->buf)
        av_buffer_unref(&old_head->buf);
    else
        av_freep(&old_head->payload);

    ts_st->tsi.packets_head++;
    ts_st->tsi.packet_consumed_bytes = 0;

    if (tsi_buffer_size(ts_st) == 0) {
        ts_st->tsi.time = AV_NOPTS_VALUE;
    } else {
        MpegTSPesPacket *new_head = tsi_buffer_head(ts_st);
        int64_t dts_diff = new_head->dts - old_head->dts;
        if (dts_diff < TSI_DISCONTINUITY_THRESHOLD) {
            ts_st->tsi.buffer_duration -= dts_diff;
            ts_st->tsi.packet_since = ts_st->tsi.packet_till;
        } else {
            tsi_log_ts_st(s, ts_st, AV_LOG_INFO,
                "dts discontinuity (diff=%.3fsec prev_dts=%.3fsec new_dts=%.3fsec packet_till=%.3fsec)\n",
                dts_diff / 90000.0, old_head->dts / 90000.0, new_head->dts / 90000.0, ts_st->tsi.packet_till / 90000.0);
            ts_st->tsi.packet_since = new_head->dts - (old_head->dts - ts_st->tsi.packet_till);
            ts_st->tsi.time = ts_st->tsi.packet_since;
            //TODO: disable warning after discontinuity in PCR pid?
            //TODO: ts_st->discontinuity = 1 ?
        }
    }
}

static void tsi_drain_interleaving_buffer(AVFormatContext *s, int64_t duration, int flush)
{
    MpegTSWrite *ts = s->priv_data;
    AVStream *st;

    { //TODO: LOG_DEBUG? stream time instead of realtime?
        int64_t t = av_gettime_relative();
        if (ts->tsi.buffer_report_realtime == 0) ts->tsi.buffer_report_realtime = t;

        if (t - ts->tsi.buffer_report_realtime > 60LL * 60 * 1000000) {
            ts->tsi.buffer_report_realtime = t;
            for(int i = 0; i < ts->nb_services; i++){
                char buf[4096];
                tsi_dbg_snprintf_buffers(buf, sizeof(buf), s, ts->services[i], NULL);
                tsi_log_svc(s, ts->services[i], AV_LOG_INFO, "buffers: %s\n", buf);
            }
        }
    }

    tsi_schedule_services(s, flush);
    while ((st = tsi_get_earliest_stream(s, flush))) {
        MpegTSWriteStream *ts_st    = st->priv_data;
        int64_t new_stream_time     = ts_st->tsi.time + ts_st->service->tsi.time_offset;
        MpegTSPesPacket* pkt = tsi_buffer_head(ts_st);
        int force_pat = 0;

        if (duration == AV_NOPTS_VALUE) { // non-realtime
            int64_t max_buffer_duration = 0;
            for (int i=0; i<ts->nb_services; i++)
                max_buffer_duration = FFMAX(max_buffer_duration, ts->services[i]->tsi.max_buffer_duration);
            if (max_buffer_duration < TSI_BUFFER_TARGET && !flush)
                break;
        } else {
            if (new_stream_time > ts->tsi.ts_time + duration) {
                ts->tsi.ts_time += duration;
                break;
            }
            duration -= new_stream_time - ts->tsi.ts_time;
        }

        if (new_stream_time < ts->tsi.ts_time) {
            tsi_log_ts_st(s, ts_st, AV_LOG_ERROR, "[WTF] new_stream_time < ts->ts_stream_time (%.3f < %.3f)\n",
                   new_stream_time/90000.0, ts->tsi.ts_time/90000.0);
        }
        ts->tsi.ts_time = new_stream_time;

        // SDT packets
        if (ts->tsi.last_sdt_time + TSI_SDT_PERIOD < new_stream_time) {
            ts->tsi.last_sdt_time = new_stream_time;
            mpegts_write_sdt(s);
        }
        // PAT/PMT packets
        if (ts_st->tsi.packet_consumed_bytes == 0 && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            force_pat = (pkt->key && !ts_st->tsi.last_packet_key) || ts->flags & MPEGTS_FLAG_PAT_PMT_AT_FRAMES;
        }
        if(ts->tsi.last_pat_time + TSI_PAT_PERIOD < new_stream_time || force_pat) {
            ts->tsi.last_pat_time = new_stream_time;
            mpegts_write_pat(s);
            for (int i = 0; i < ts->nb_services; i++) // TODO: send only one PMT at a time?
                mpegts_write_pmt(s, ts->services[i]);
        }
        // NULL packets
        if (ts->mux_rate > 1) {
            int64_t expected; // TODO: rename
            int64_t bytes = avio_tell(s->pb);
            if(ts->tsi.last_muxrate_time == AV_NOPTS_VALUE) {
                ts->tsi.last_muxrate_time = ts->tsi.ts_time;
                ts->tsi.last_muxrate_bytes = bytes;
            }
            //av_log(s, AV_LOG_WARNING, "[tsi] pid#%03x consumed=%6d time=%.3f\n", ts->ts_stream_time/90000.0,
            //       ts_st->pid, ts_st->packet_consumed_bytes);
            expected = ts->mux_rate * (ts->tsi.ts_time - ts->tsi.last_muxrate_time) / (8 * 90000)
                    - (bytes - ts->tsi.last_muxrate_bytes);
            if (expected < -188 * 20 ) { // TODO: threshold?
                tsi_log(s, AV_LOG_WARNING, "bitrate too high, resyncing (%d packets)\n",
                       (int) -expected / 188);
                ts->tsi.last_muxrate_time = ts->tsi.ts_time;
                ts->tsi.last_muxrate_bytes = bytes;
            }
            if (expected > (ts->mux_rate / 8) / 20 ) {
                tsi_log(s, AV_LOG_WARNING, "NULL packet burst (%d packets)\n", (int)expected / 188);
            }
            while (expected > 188) {
                mpegts_insert_null_packet(s);
                expected -= 188;
            }
        }

        if (ts_st->tsi.packet_consumed_bytes == 0)
            tsi_schedule_first_packet(s, st);
        //int64_t t0 = av_gettime_relative();
        mpegts_write_pes(s, st,
            pkt->payload + ts_st->tsi.packet_consumed_bytes,
            pkt->payload_size - ts_st->tsi.packet_consumed_bytes,
            pkt->pts + (ts->tsi.sync_service_times ? ts_st->service->tsi.time_offset : 0),
            pkt->dts + (ts->tsi.sync_service_times ? ts_st->service->tsi.time_offset : 0),
            pkt->key,
            pkt->stream_id);
        /*{ //TODO: remove debug / calc duration for N call?
            int64_t t = av_gettime_relative();
            if (t - t0 > 10000 ) {
                av_log(s, AV_LOG_WARNING, "[tsi] mpegts_write_pes: call duration %.3f sec\n",
                    (t - t0) / 1000000.0);
            }
        }*/
        // ts_st->packet_consumed_bytes updated in mpegts_write_pes
        int64_t prev_stream_time = ts_st->tsi.time;
        ts_st->tsi.time = ts_st->tsi.packet_since + (ts_st->tsi.packet_till - ts_st->tsi.packet_since) * ts_st->tsi.packet_consumed_bytes / pkt->payload_size;
        if (ts_st->tsi.time < prev_stream_time) {
            tsi_log_ts_st(s, ts_st, AV_LOG_ERROR, "[WTF] ts_st->service_time decreased %.3f < %.3f (interval=%.3f-%.3f bytes=%d/%d)\n",
                    ts_st->tsi.time/90000.0, prev_stream_time/90000.0,
                    ts_st->tsi.packet_since/90000.0, ts_st->tsi.packet_till/90000.0,
                    ts_st->tsi.packet_consumed_bytes, pkt->payload_size
                    );
        }

        if (ts_st->tsi.packet_consumed_bytes == pkt->payload_size) {
            tsi_remove_finished_packet(s, ts_st);
            tsi_schedule_services(s, flush);
        }
    }
}

static int tsi_interleave_packet(AVFormatContext *s, AVPacket *out, AVPacket *in, int flush)
{
    MpegTSWrite *ts = s->priv_data;
    if (!ts->tsi.is_active)
        return ff_interleave_packet_per_dts(s, out, in, flush);
    if (in)
        av_copy_packet(out, in); // out packet is uninitialized here so we can't use av_packet_ref
    return in != NULL;
}

static void* tsi_thread(void *opaque)
{
    //int64_t oops=0;
    AVFormatContext *s = opaque;
    MpegTSWrite *ts = s->priv_data;
    int64_t t_prev = av_gettime_relative() * 9/*0000*/ / 100/*0000*/;
    while (!ts->tsi.thread_exit) {
        int64_t t = av_gettime_relative() * 9/*0000*/ / 100/*0000*/;
        int64_t duration = t - t_prev;
        if (duration > 90000 / 10) {
            tsi_log(s, AV_LOG_WARNING, "tsi_thread: duration too large %.3f sec (usually this means swapping or network problems)\n", duration / 90000.0);
            duration = 90000 / 10;
        }
        pthread_mutex_lock(&ts->tsi.mutex);
        tsi_drain_interleaving_buffer(s, duration, 0);
        pthread_mutex_unlock(&ts->tsi.mutex);

        av_usleep(10000); //TODO: AVOption?
        t_prev = t;
        /*if( (++oops)%(100*40) == 0 ) {
            av_usleep(4*1000*1000); // oops!
        }*/
    }
    return NULL;
}

static void tsi_mpegts_write_pes(AVFormatContext *s, AVStream *st,
                             /*TODO: const?*/ uint8_t * payload, int payload_size,
                             int64_t pts, int64_t dts, int key, int stream_id, AVBufferRef *owned_buf)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = st->priv_data;
    MpegTSPesPacket *last_pkt, *new_pkt;

    //if (owned_buf && owned_buf->data != payload) //TODO: remove debug
    //    tsi_log(s, AV_LOG_INFO, "strange packet offs=%d size=%d/%d\n", (int)(payload - owned_buf->data), payload_size, owned_buf->size);

    { //TODO: remove debug
        int64_t t = av_gettime_relative();
        int64_t *last = &ts_st->service->tsi.last_packet_realtime;
        if (*last == 0) *last = t;
        if (t - *last > 600000) {
            char buf[4096];
            tsi_dbg_snprintf_buffers(buf, sizeof(buf), s, ts_st->service, NULL);
            tsi_log_svc(s, ts_st->service, AV_LOG_WARNING, "mpegts_write_pes: last packet was %.3f sec ago (buffers %s)\n",
                   (t - *last) / 1000000.0, buf);
        }
        *last = t;
    }

    if (dts == AV_NOPTS_VALUE) {
        tsi_log_ts_st(s, ts_st, AV_LOG_ERROR, "dropping packet with dts==NOPTS\n");
        av_buffer_unref(&owned_buf);
        return;
    }

    pthread_mutex_lock(&ts->tsi.mutex);

    last_pkt = tsi_buffer_tail(ts_st);

    if (tsi_buffer_size(ts_st) > 0 && dts < last_pkt->dts) {
        tsi_log_ts_st(s, ts_st, AV_LOG_ERROR, "dropping packet with dts<prev_dts (%ld<%ld) \n",
               dts, last_pkt->dts);
        av_buffer_unref(&owned_buf);
        pthread_mutex_unlock(&ts->tsi.mutex);
        return;
    }

    if (tsi_buffer_size(ts_st) == tsi_buffer_capacity(ts_st) || ts_st->tsi.buffer_duration > TSI_BUFFER_HIGH) {
        //TODO: check byte size? drop packets in all service streams?
        char buf_before[4096], buf_after[4096];
        av_buffer_unref(&owned_buf);
        tsi_dbg_snprintf_buffers(buf_before, sizeof(buf_after), s, NULL, ts_st);
        while (tsi_buffer_size(ts_st) > tsi_buffer_capacity(ts_st) / 2 || ts_st->tsi.buffer_duration > TSI_BUFFER_TARGET) {
            MpegTSPesPacket *prev_pkt, *pkt = tsi_buffer_tail(ts_st);
            if (pkt->buf)
                av_buffer_unref(&pkt->buf);
            else
                av_freep(&pkt->payload);
            ts_st->tsi.buffer_bytes -= pkt->payload_size;
            ts_st->tsi.packets_tail--;
            prev_pkt = tsi_buffer_tail(ts_st);
            int64_t dts_diff = pkt->dts - prev_pkt->dts;
            if (dts_diff < TSI_DISCONTINUITY_THRESHOLD)
                ts_st->tsi.buffer_duration -= dts_diff;
        }
        tsi_dbg_snprintf_buffers(buf_after, sizeof(buf_after), s, NULL, ts_st);
        tsi_log_ts_st(s, ts_st, AV_LOG_ERROR, "buffer overflow, dropping packets (%s) -> (%s)\n",
            buf_before, buf_after);
        pthread_mutex_unlock(&ts->tsi.mutex);
        return;
    }

    //TODO: join first audio packet?
    //join audio pes-packets
    if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO
            && tsi_buffer_size(ts_st) > 1
            && last_pkt->dts + TSI_MAX_AUDIO_PACKET_DURATION >= dts) {
        if (last_pkt->buf) {
            //TODO: check last_pkt->buf->data == last_pkt->payload ?
            av_buffer_realloc(&last_pkt->buf, FFMAX(4096, last_pkt->payload_size + payload_size)); //TODO: check ENOMEM?
            last_pkt->payload = last_pkt->buf->data;
        } else {
            last_pkt->payload = av_realloc(last_pkt->payload, last_pkt->payload_size + payload_size);
        }
        memcpy(last_pkt->payload + last_pkt->payload_size, payload, payload_size);
        last_pkt->payload_size += payload_size;
        last_pkt->guessed_paketized_size = tsi_guess_paketized_size(last_pkt->payload_size);
        ts_st->tsi.buffer_bytes += payload_size;
        av_buffer_unref(&owned_buf);
        pthread_mutex_unlock(&ts->tsi.mutex);
        return;
    }

    ts_st->tsi.packets_tail++;
    new_pkt = tsi_buffer_tail(ts_st);
    if (!owned_buf) {
        void* tmp = av_malloc(payload_size);
        memcpy(tmp, payload, payload_size);
        payload = tmp;
    }
    *new_pkt = (MpegTSPesPacket){
        .buf = owned_buf,
        .payload = payload,
        .payload_size = payload_size,
        .guessed_paketized_size = tsi_guess_paketized_size(payload_size),
        .pts = pts,
        .dts = dts,
        .key = key,
        .stream_id = stream_id,
    };
    ts_st->tsi.buffer_bytes += payload_size;

    if (tsi_buffer_size(ts_st) > 1) {
        int64_t dts_diff = new_pkt->dts - last_pkt->dts;
        ts_st->tsi.buffer_duration += dts_diff < TSI_DISCONTINUITY_THRESHOLD ? dts_diff : 0;
    }

    pthread_mutex_unlock(&ts->tsi.mutex);

    if (!ts->tsi.is_realtime) {
        tsi_drain_interleaving_buffer(s, AV_NOPTS_VALUE, 0);
    }

    { //TODO: remove debug
        int64_t t = av_gettime_relative();
        int64_t *prev = &ts_st->service->tsi.last_packet_realtime;
        if (t - *prev > 100000 ) {
            tsi_log_svc(s, ts_st->service, AV_LOG_WARNING, "mpegts_write_pes: call duration %.3f sec\n",
                (t - *prev) / 1000000.0);
        }
    }
}

static void tsi_init(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    int ret;
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        ts_st->tsi.time = AV_NOPTS_VALUE;
    }
    for (int i = 0; i < ts->nb_services; i++) {
        MpegTSService *service = ts->services[i];
        service->tsi.last_pcr = AV_NOPTS_VALUE;
        service->tsi.time_offset = AV_NOPTS_VALUE;
    }

    ts->tsi.last_pat_time = AV_NOPTS_VALUE;
    ts->tsi.last_sdt_time = AV_NOPTS_VALUE;
    ts->tsi.last_muxrate_time = AV_NOPTS_VALUE;
    ret = pthread_mutex_init(&ts->tsi.mutex, NULL);
    if (ret != 0) {
        av_log(s, AV_LOG_ERROR, "pthread_mutex_init failed : %s\n", strerror(ret));
    }
    ret = pthread_cond_init(&ts->tsi.cond, NULL);
    if (ret != 0) {
        av_log(s, AV_LOG_ERROR, "pthread_cond_init failed : %s\n", strerror(ret));
    }
    if (ts->tsi.is_realtime) {
        //TODO: log "packet flush disabled"?
        s->flush_packets = 0;                  // disable AVIOContext flushing from another thread
        s->flags &= ~AVFMT_FLAG_FLUSH_PACKETS; // two flags for the same thing. One from ffmpeg, other from libav.

        ts->tsi.thread_exit = 0;
        ret = pthread_create(&ts->tsi.thread, NULL, &tsi_thread, s);
        if (ret != 0) {
            av_log(s, AV_LOG_ERROR, "pthread_create failed : %s\n", strerror(ret));
        }
    }
}

static void tsi_deinit(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    int ret;
    if (ts->tsi.is_realtime) {
        ts->tsi.thread_exit = 1;
        ret = pthread_join(ts->tsi.thread, NULL);
        if (ret != 0) {
            tsi_log(s, AV_LOG_ERROR, "pthread_join(): %s\n", strerror(ret));
        }
    }
    pthread_mutex_destroy(&ts->tsi.mutex);
    pthread_cond_destroy(&ts->tsi.cond);

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        for (MpegTSPesPacket *pkt = tsi_buffer_head(ts_st); pkt; pkt = tsi_buffer_next(ts_st, pkt))
            if (pkt->buf)
                av_buffer_unref(&pkt->buf);
            else
                av_freep(&pkt->payload);
    }
}

/* TSI END */

static MpegTSService *mpegts_add_service(AVFormatContext *s, int sid,
                                         const AVDictionary *metadata,
                                         AVProgram *program)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService *service;
    AVDictionaryEntry *title, *provider;
    char default_service_name[32];
    const char *service_name;
    const char *provider_name;

    title = av_dict_get(metadata, "service_name", NULL, 0);
    if (!title)
        title = av_dict_get(metadata, "title", NULL, 0);
    snprintf(default_service_name, sizeof(default_service_name), "%s%02d", DEFAULT_SERVICE_NAME, ts->nb_services + 1);
    service_name  = title ? title->value : default_service_name;
    provider      = av_dict_get(metadata, "service_provider", NULL, 0);
    provider_name = provider ? provider->value : DEFAULT_PROVIDER_NAME;

    service = av_mallocz(sizeof(MpegTSService));
    if (!service)
        return NULL;
    service->pmt.pid       = ts->pmt_start_pid + ts->nb_services;
    service->sid           = sid;
    service->pcr_pid       = 0x1fff;
    if (encode_str8(service->provider_name, provider_name) < 0 ||
        encode_str8(service->name, service_name) < 0) {
        av_log(s, AV_LOG_ERROR, "Too long service or provider name\n");
        goto fail;
    }
    if (av_dynarray_add_nofree(&ts->services, &ts->nb_services, service) < 0)
        goto fail;

    service->pmt.write_packet = section_write_packet;
    service->pmt.opaque       = s;
    service->pmt.cc           = 15;
    service->pmt.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
    service->program          = program;

    return service;
fail:
    av_free(service);
    return NULL;
}

static void enable_pcr_generation_for_stream(AVFormatContext *s, AVStream *pcr_st)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = pcr_st->priv_data;

    if (ts->mux_rate > 1 || ts->pcr_period_ms >= 0) {
        int pcr_period_ms = ts->pcr_period_ms == -1 ? PCR_RETRANS_TIME : ts->pcr_period_ms;
        ts_st->pcr_period = av_rescale(pcr_period_ms, PCR_TIME_BASE, 1000);
    } else {
        /* By default, for VBR we select the highest multiple of frame duration which is less than 100 ms. */
        int64_t frame_period = 0;
        if (pcr_st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            int frame_size = av_get_audio_frame_duration2(pcr_st->codecpar, 0);
            if (!frame_size) {
               av_log(s, AV_LOG_WARNING, "frame size not set\n");
               frame_size = 512;
            }
            frame_period = av_rescale_rnd(frame_size, PCR_TIME_BASE, pcr_st->codecpar->sample_rate, AV_ROUND_UP);
        } else if (pcr_st->avg_frame_rate.num) {
            frame_period = av_rescale_rnd(pcr_st->avg_frame_rate.den, PCR_TIME_BASE, pcr_st->avg_frame_rate.num, AV_ROUND_UP);
        }
        if (frame_period > 0 && frame_period <= PCR_TIME_BASE / 10)
            ts_st->pcr_period = frame_period * (PCR_TIME_BASE / 10 / frame_period);
        else
            ts_st->pcr_period = 1;
    }

    // output a PCR as soon as possible
    ts_st->last_pcr   = ts->first_pcr - ts_st->pcr_period;
}

static void select_pcr_streams(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;

    for (int i = 0; i < ts->nb_services; i++) {
        MpegTSService *service = ts->services[i];
        AVStream *pcr_st = NULL;
        AVProgram *program = service->program;
        int nb_streams = program ? program->nb_stream_indexes : s->nb_streams;

        for (int j = 0; j < nb_streams; j++) {
            AVStream *st = s->streams[program ? program->stream_index[j] : j];
            if (!pcr_st ||
                pcr_st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
            {
                pcr_st = st;
            }
        }

        if (pcr_st) {
            MpegTSWriteStream *ts_st = pcr_st->priv_data;
            service->pcr_pid = ts_st->pid;
            enable_pcr_generation_for_stream(s, pcr_st);
            av_log(s, AV_LOG_VERBOSE, "service %i using PCR in pid=%i, pcr_period=%"PRId64"ms\n",
                service->sid, service->pcr_pid, av_rescale(ts_st->pcr_period, 1000, PCR_TIME_BASE));
        }
    }
}

static int mpegts_init(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService **service = NULL;
    int i, j;
    int ret;

    if (ts->m2ts_mode == -1) {
        if (av_match_ext(s->url, "m2ts")) {
            ts->m2ts_mode = 1;
        } else {
            ts->m2ts_mode = 0;
        }
    }

    ts->m2ts_video_pid   = M2TS_VIDEO_PID;
    ts->m2ts_audio_pid   = M2TS_AUDIO_START_PID;
    ts->m2ts_pgssub_pid  = M2TS_PGSSUB_START_PID;
    ts->m2ts_textsub_pid = M2TS_TEXTSUB_PID;

    if (ts->m2ts_mode) {
        ts->pmt_start_pid = M2TS_PMT_PID;
        if (s->nb_programs > 1) {
            av_log(s, AV_LOG_ERROR, "Only one program is allowed in m2ts mode!\n");
            return AVERROR(EINVAL);
        }
    }

    if (s->max_delay < 0) /* Not set by the caller */
        s->max_delay = 0;

    // round up to a whole number of TS packets
    ts->pes_payload_size = (ts->pes_payload_size + 14 + 183) / 184 * 184 - 14;

    if (!s->nb_programs) {
        /* allocate a single DVB service */
        service = (MpegTSService **)malloc(sizeof(MpegTSService *));
        service[0] = mpegts_add_service(s, ts->service_id, s->metadata, NULL);
        if (!service[0]) {
            free(service);
            return AVERROR(ENOMEM);
        }
    } else {
        service = (MpegTSService **)malloc(s->nb_programs * sizeof(MpegTSService *));
        for (i = 0; i < s->nb_programs; i++) {
            AVProgram *program = s->programs[i];
            service[i] = mpegts_add_service(s, program->id, program->metadata, program);
            if (!service[i]) {
                free(service);
                return AVERROR(ENOMEM);
            }
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

    /* assign pids to each stream */
    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st;

        ts_st = av_mallocz(sizeof(MpegTSWriteStream));
        if (!ts_st) {
            free(service);
            return AVERROR(ENOMEM);
        }
        st->priv_data = ts_st;
        ts_st->service = service[s->nb_programs ? st->program_num : 0];

        avpriv_set_pts_info(st, 33, 1, 90000);

        ts_st->payload = av_mallocz(ts->pes_payload_size);
        if (!ts_st->payload) {
            free(service);
            return AVERROR(ENOMEM);
        }

        /* MPEG pid values < 16 are reserved. Applications which set st->id in
         * this range are assigned a calculated pid. */
        if (st->id < 16) {
            if (ts->m2ts_mode) {
                switch (st->codecpar->codec_type) {
                case AVMEDIA_TYPE_VIDEO:
                    ts_st->pid = ts->m2ts_video_pid++;
                    break;
                case AVMEDIA_TYPE_AUDIO:
                    ts_st->pid = ts->m2ts_audio_pid++;
                    break;
                case AVMEDIA_TYPE_SUBTITLE:
                    switch (st->codecpar->codec_id) {
                    case AV_CODEC_ID_HDMV_PGS_SUBTITLE:
                        ts_st->pid = ts->m2ts_pgssub_pid++;
                        break;
                    case AV_CODEC_ID_HDMV_TEXT_SUBTITLE:
                        ts_st->pid = ts->m2ts_textsub_pid++;
                        break;
                    }
                    break;
                }
                if (ts->m2ts_video_pid   > M2TS_VIDEO_PID + 1          ||
                    ts->m2ts_audio_pid   > M2TS_AUDIO_START_PID + 32   ||
                    ts->m2ts_pgssub_pid  > M2TS_PGSSUB_START_PID + 32  ||
                    ts->m2ts_textsub_pid > M2TS_TEXTSUB_PID + 1        ||
                    ts_st->pid < 16) {
                    av_log(s, AV_LOG_ERROR, "Cannot automatically assign PID for stream %d\n", st->index);
                    free(service);
                    return AVERROR(EINVAL);
                }
            } else {
                ts_st->pid = ts->start_pid + i;
            }
        } else {
            ts_st->pid = st->id;
        }
        if (ts_st->pid >= 0x1FFF) {
            av_log(s, AV_LOG_ERROR,
                   "Invalid stream id %d, must be less than 8191\n", st->id);
            free(service);
            return AVERROR(EINVAL);
        }
        for (j = 0; j < ts->nb_services; j++) {
            if (ts->services[j]->pmt.pid > LAST_OTHER_PID) {
                av_log(s, AV_LOG_ERROR,
                       "Invalid PMT PID %d, must be less than %d\n", ts->services[j]->pmt.pid, LAST_OTHER_PID + 1);
                free(service);
                return AVERROR(EINVAL);
            }
            if (ts_st->pid == ts->services[j]->pmt.pid) {
                av_log(s, AV_LOG_ERROR, "PID %d cannot be both elementary and PMT PID\n", ts_st->pid);
                free(service);
                return AVERROR(EINVAL);
            }
        }
        for (j = 0; j < i; j++) {
            MpegTSWriteStream *ts_st_prev = s->streams[j]->priv_data;
            if (ts_st_prev->pid == ts_st->pid) {
                av_log(s, AV_LOG_ERROR, "Duplicate stream id %d\n", ts_st->pid);
                free(service);
                return AVERROR(EINVAL);
            }
        }
        ts_st->payload_pts     = AV_NOPTS_VALUE;
        ts_st->payload_dts     = AV_NOPTS_VALUE;
        ts_st->first_pts_check = 1;
        ts_st->cc              = 15;
        ts_st->discontinuity   = ts->flags & MPEGTS_FLAG_DISCONT;
        if (st->codecpar->codec_id == AV_CODEC_ID_AAC &&
            st->codecpar->extradata_size > 0) {
            AVStream *ast;
            ts_st->amux = avformat_alloc_context();
            if (!ts_st->amux) {
                free(service);
                return AVERROR(ENOMEM);
            }
            ts_st->amux->oformat =
                av_guess_format((ts->flags & MPEGTS_FLAG_AAC_LATM) ? "latm" : "adts",
                                NULL, NULL);
            if (!ts_st->amux->oformat) {
                free(service);
                return AVERROR(EINVAL);
            }
            if (!(ast = avformat_new_stream(ts_st->amux, NULL))) {
                free(service);
                return AVERROR(ENOMEM);
            }
            ret = avcodec_parameters_copy(ast->codecpar, st->codecpar);
            if (ret != 0) {
                free(service);
                return ret;
            }
            ast->time_base = st->time_base;
            ret = avformat_write_header(ts_st->amux, NULL);
            if (ret < 0) {
                free(service);
                return ret;
            }
        }
        if (st->codecpar->codec_id == AV_CODEC_ID_OPUS) {
            ts_st->opus_pending_trim_start = st->codecpar->initial_padding * 48000 / st->codecpar->sample_rate;
        }
    }

    if (ts->copyts < 1)
        ts->first_pcr = av_rescale(s->max_delay, PCR_TIME_BASE, AV_TIME_BASE);

    select_pcr_streams(s);

    ts->last_pat_ts = AV_NOPTS_VALUE;
    ts->last_sdt_ts = AV_NOPTS_VALUE;
    ts->pat_period = av_rescale(ts->pat_period_us, PCR_TIME_BASE, AV_TIME_BASE);
    ts->sdt_period = av_rescale(ts->sdt_period_us, PCR_TIME_BASE, AV_TIME_BASE);

    if (ts->mux_rate == 1)
        av_log(s, AV_LOG_VERBOSE, "muxrate VBR, ");
    else
        av_log(s, AV_LOG_VERBOSE, "muxrate %d, ", ts->mux_rate);
    av_log(s, AV_LOG_VERBOSE,
           "sdt every %"PRId64" ms, pat/pmt every %"PRId64" ms\n",
           av_rescale(ts->sdt_period, 1000, PCR_TIME_BASE),
           av_rescale(ts->pat_period, 1000, PCR_TIME_BASE));

    tsi_init(s);
    free(service);
    return 0;
}

/* send SDT, PAT and PMT tables regularly */
static void retransmit_si_info(AVFormatContext *s, int force_pat, int force_sdt, int64_t pcr)
{
    MpegTSWrite *ts = s->priv_data;
    int i;

    if ((pcr != AV_NOPTS_VALUE && ts->last_sdt_ts == AV_NOPTS_VALUE) ||
        (pcr != AV_NOPTS_VALUE && pcr - ts->last_sdt_ts >= ts->sdt_period) ||
        force_sdt
    ) {
        if (pcr != AV_NOPTS_VALUE)
            ts->last_sdt_ts = FFMAX(pcr, ts->last_sdt_ts);
        mpegts_write_sdt(s);
    }
    if ((pcr != AV_NOPTS_VALUE && ts->last_pat_ts == AV_NOPTS_VALUE) ||
        (pcr != AV_NOPTS_VALUE && pcr - ts->last_pat_ts >= ts->pat_period) ||
        force_pat) {
        if (pcr != AV_NOPTS_VALUE)
            ts->last_pat_ts = FFMAX(pcr, ts->last_pat_ts);
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
    write_packet(s, buf);
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
    write_packet(s, buf);
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
static void mpegts_write_pes(AVFormatContext *s, AVStream *st,
                             const uint8_t *payload, int payload_size,
                             int64_t pts, int64_t dts, int key, int stream_id)
{
    MpegTSWriteStream *ts_st = st->priv_data;
    MpegTSWrite *ts = s->priv_data;
    uint8_t buf[TS_PACKET_SIZE];
    uint8_t *q;
    int val, len, header_len, write_pcr = 0;
    const int is_dvb_subtitle = (st->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) && (st->codecpar->codec_id == AV_CODEC_ID_DVB_SUBTITLE);
    const int is_dvb_teletext = (st->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) && (st->codecpar->codec_id == AV_CODEC_ID_DVB_TELETEXT);
    const int tsi_active = ts->tsi.is_active;
    int is_start = !tsi_active || ts_st->tsi.packet_consumed_bytes == 0;
    int64_t pcr = -1; /* avoid warning */
    int64_t delay = av_rescale(s->max_delay, 90000, AV_TIME_BASE);
    int force_pat = st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO && key && !ts_st->prev_payload_key;
    int force_sdt = 0;

    av_assert0(ts_st->payload != buf || st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO);
    if (ts->flags & MPEGTS_FLAG_PAT_PMT_AT_FRAMES && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        force_pat = 1;
    }

    if (ts->flags & MPEGTS_FLAG_REEMIT_PAT_PMT) {
        force_pat = 1;
        force_sdt = 1;
        ts->flags &= ~MPEGTS_FLAG_REEMIT_PAT_PMT;
    }

    if (is_dvb_subtitle)
      ++payload_size;

    while (payload_size > 0) {
        if (!tsi_active) { // keep ffmpeg indention in this block
        int64_t pcr = AV_NOPTS_VALUE;
        if (ts->mux_rate > 1)
            pcr = get_pcr(ts, s->pb);
        else if (dts != AV_NOPTS_VALUE)
            pcr = (dts - delay) * 300;

        retransmit_si_info(s, force_pat, force_sdt, pcr);
        force_pat = 0;
        force_sdt = 0;

        write_pcr = 0;
        if (ts->mux_rate > 1) {
            /* Send PCR packets for all PCR streams if needed */
            pcr = get_pcr(ts, s->pb);
            if (pcr >= ts->next_pcr) {
                int64_t next_pcr = INT64_MAX;
                for (int i = 0; i < s->nb_streams; i++) {
                    /* Make the current stream the last, because for that we
                     * can insert the pcr into the payload later */
                    int st2_index = i < st->index ? i : (i + 1 == s->nb_streams ? st->index : i + 1);
                    AVStream *st2 = s->streams[st2_index];
                    MpegTSWriteStream *ts_st2 = st2->priv_data;
                    if (ts_st2->pcr_period) {
                        if (pcr - ts_st2->last_pcr >= ts_st2->pcr_period) {
                            ts_st2->last_pcr = FFMAX(pcr - ts_st2->pcr_period, ts_st2->last_pcr + ts_st2->pcr_period);
                            if (st2 != st) {
                                mpegts_insert_pcr_only(s, st2);
                                pcr = get_pcr(ts, s->pb);
                            } else {
                                write_pcr = 1;
                            }
                        }
                        next_pcr = FFMIN(next_pcr, ts_st2->last_pcr + ts_st2->pcr_period);
                    }
                }
                ts->next_pcr = next_pcr;
            }
            if (dts != AV_NOPTS_VALUE && (dts - pcr / 300) > delay) {
                /* pcr insert gets priority over null packet insert */
                if (write_pcr)
                    mpegts_insert_pcr_only(s, st);
                else
                    mpegts_insert_null_packet(s);
                /* recalculate write_pcr and possibly retransmit si_info */
                continue;
            }
        } else if (ts_st->pcr_period && pcr != AV_NOPTS_VALUE) {
            if (pcr - ts_st->last_pcr >= ts_st->pcr_period && is_start) {
                ts_st->last_pcr = FFMAX(pcr - ts_st->pcr_period, ts_st->last_pcr + ts_st->pcr_period);
                write_pcr = 1;
            }
        }
        } // if (!tsi_active)

        /* prepare packet header */
        q    = buf;
        *q++ = 0x47;
        val  = ts_st->pid >> 8;
        if (ts->m2ts_mode && st->codecpar->codec_id == AV_CODEC_ID_AC3)
            val |= 0x20;
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
        if (key && is_start && pts != AV_NOPTS_VALUE
            && !is_dvb_teletext /* adaptation+payload forbidden for teletext (ETSI EN 300 472 V1.3.1 4.1) */ ) {
            // set Random Access for key frames
            if (ts_st->pcr_period)
                write_pcr = 1;
            set_af_flag(buf, 0x40);
            q = get_ts_payload_start(buf);
        }
        if (tsi_active && ts_st->pid == ts_st->service->pcr_pid
                && (pcr=tsi_get_adjusted_pcr(s, ts_st, write_pcr))!=AV_NOPTS_VALUE)
            write_pcr = 1;
        if (write_pcr) {
            set_af_flag(buf, 0x10);
            q = get_ts_payload_start(buf);

            if (!tsi_active) { // keep ffmpeg indention in this block
            if (dts != AV_NOPTS_VALUE && dts < pcr / 300)
                av_log(s, AV_LOG_WARNING, "dts < pcr, TS is invalid\n");
            } // if (!tsi_active)
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
                //FIXME: handle case when last ts packet contains single trailing 0xFF (tsi.packet_consumed_bytes)
                len += 2; // third byte added in "if (is_dvb_subtitle) payload_size++;"
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
        len = TS_PACKET_SIZE - header_len;
        if (len > payload_size)
            len = payload_size;
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

        if (is_dvb_subtitle && payload_size == len) {
            memcpy(buf + TS_PACKET_SIZE - len, payload, len - 1);
            buf[TS_PACKET_SIZE - 1] = 0xff; /* end_of_PES_data_field_marker: an 8-bit field with fixed contents 0xff for DVB subtitle */
        } else {
            memcpy(buf + TS_PACKET_SIZE - len, payload, len);
        }

        payload      += len;
        payload_size -= len;
        write_packet(s, buf);

        if (tsi_active) { // send only one ts packet in tsi mode
            ts_st->tsi.packet_consumed_bytes += is_dvb_subtitle && payload_size<=0 ? len-1 : len;
            return;
        }
    }
    ts_st->prev_payload_key = key;
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
    const int64_t max_audio_delay = av_rescale(s->max_delay, 90000, AV_TIME_BASE) / 2;
    int64_t dts = pkt->dts, pts = pkt->pts;
    int opus_samples = 0;
    int side_data_size;
    uint8_t *side_data = NULL;
    int stream_id = -1;

    side_data = av_packet_get_side_data(pkt,
                                        AV_PKT_DATA_MPEGTS_STREAM_ID,
                                        &side_data_size);
    if (side_data)
        stream_id = side_data[0];

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
                return ret;

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

        if ((state & 0x7e) < 2*16 || (state & 0x7e) >= 2*24)
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

    if (ts->tsi.is_active) {
        AVBufferRef *tmp = NULL;
        if (data) {
            tmp = av_buffer_create(data, size, av_buffer_default_free, NULL, 0);
        } else if (pkt->buf) {
            tmp = av_buffer_ref(pkt->buf);
        } else {
            av_buffer_realloc(&tmp, size);
            memcpy(tmp->data, buf, size);
            buf = tmp->data;
        }
        tsi_mpegts_write_pes(s, st, buf, size, pts, dts,
                         pkt->flags & AV_PKT_FLAG_KEY, stream_id, tmp);
        if (!tmp)
            av_free(data);
        return 0;
    }

    if (ts_st->payload_size && (ts_st->payload_size + size > ts->pes_payload_size ||
        (dts != AV_NOPTS_VALUE && ts_st->payload_dts != AV_NOPTS_VALUE &&
         dts - ts_st->payload_dts >= max_audio_delay) ||
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

    if (((MpegTSWrite*)s->priv_data)->tsi.is_active && !((MpegTSWrite*)s->priv_data)->tsi.is_realtime)
        tsi_drain_interleaving_buffer(s, AV_NOPTS_VALUE, 1);

    if (ts->m2ts_mode) {
        int packets = (avio_tell(s->pb) / (TS_PACKET_SIZE + 4)) % 32;
        while (packets++ < 32)
            mpegts_insert_null_packet(s);
    }
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

    tsi_deinit(s);

    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        if (ts_st) {
            av_freep(&ts_st->payload);
            if (ts_st->amux) {
                avformat_free_context(ts_st->amux);
                ts_st->amux = NULL;
            }
        }
    }

    for (i = 0; i < ts->nb_services; i++) {
        service = ts->services[i];
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

#define OFFSET(x) offsetof(MpegTSWrite, x)
#define ENC AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "mpegts_transport_stream_id", "Set transport_stream_id field.",
      OFFSET(transport_stream_id), AV_OPT_TYPE_INT, { .i64 = 0x0001 }, 0x0001, 0xffff, ENC },
    { "mpegts_original_network_id", "Set original_network_id field.",
      OFFSET(original_network_id), AV_OPT_TYPE_INT, { .i64 = DVB_PRIVATE_NETWORK_START }, 0x0001, 0xffff, ENC },
    { "mpegts_service_id", "Set service_id field.",
      OFFSET(service_id), AV_OPT_TYPE_INT, { .i64 = 0x0001 }, 0x0001, 0xffff, ENC },
    { "mpegts_service_type", "Set service_type field.",
      OFFSET(service_type), AV_OPT_TYPE_INT, { .i64 = 0x01 }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "digital_tv", "Digital Television.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_DIGITAL_TV }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "digital_radio", "Digital Radio.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_DIGITAL_RADIO }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "teletext", "Teletext.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_TELETEXT }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "advanced_codec_digital_radio", "Advanced Codec Digital Radio.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_RADIO }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "mpeg2_digital_hdtv", "MPEG2 Digital HDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_MPEG2_DIGITAL_HDTV }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "advanced_codec_digital_sdtv", "Advanced Codec Digital SDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_SDTV }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "advanced_codec_digital_hdtv", "Advanced Codec Digital HDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_HDTV }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "hevc_digital_hdtv", "HEVC Digital Television Service.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_HEVC_DIGITAL_HDTV }, 0x01, 0xff, ENC, "mpegts_service_type" },
    { "mpegts_pmt_start_pid", "Set the first pid of the PMT.",
      OFFSET(pmt_start_pid), AV_OPT_TYPE_INT, { .i64 = 0x1000 }, FIRST_OTHER_PID, LAST_OTHER_PID, ENC },
    { "mpegts_start_pid", "Set the first pid.",
      OFFSET(start_pid), AV_OPT_TYPE_INT, { .i64 = 0x0100 }, FIRST_OTHER_PID, LAST_OTHER_PID, ENC },
    { "mpegts_m2ts_mode", "Enable m2ts mode.", OFFSET(m2ts_mode), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, ENC },
    { "muxrate", NULL, OFFSET(mux_rate), AV_OPT_TYPE_INT, { .i64 = 1 }, 0, INT_MAX, ENC },
    { "pes_payload_size", "Minimum PES packet payload in bytes",
      OFFSET(pes_payload_size), AV_OPT_TYPE_INT, { .i64 = DEFAULT_PES_PAYLOAD_SIZE }, 0, INT_MAX, ENC },
    { "mpegts_flags", "MPEG-TS muxing flags", OFFSET(flags), AV_OPT_TYPE_FLAGS, { .i64 = 0 }, 0, INT_MAX, ENC, "mpegts_flags" },
    { "resend_headers", "Reemit PAT/PMT before writing the next packet",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_REEMIT_PAT_PMT }, 0, INT_MAX, ENC, "mpegts_flags" },
    { "latm", "Use LATM packetization for AAC",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_AAC_LATM }, 0, INT_MAX, ENC, "mpegts_flags" },
    { "pat_pmt_at_frames", "Reemit PAT and PMT at each video frame",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_PAT_PMT_AT_FRAMES}, 0, INT_MAX, ENC, "mpegts_flags" },
    { "system_b", "Conform to System B (DVB) instead of System A (ATSC)",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_SYSTEM_B }, 0, INT_MAX, ENC, "mpegts_flags" },
    { "initial_discontinuity", "Mark initial packets as discontinuous",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_DISCONT }, 0, INT_MAX, ENC, "mpegts_flags" },
    { "mpegts_copyts", "don't offset dts/pts", OFFSET(copyts), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, ENC },
    { "tables_version", "set PAT, PMT and SDT version", OFFSET(tables_version), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 31, ENC },
    { "omit_video_pes_length", "Omit the PES packet length for video packets",
      OFFSET(omit_video_pes_length), AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, ENC },
    { "pcr_period", "PCR retransmission time in milliseconds",
      OFFSET(pcr_period_ms), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, INT_MAX, ENC },
    { "pat_period", "PAT/PMT retransmission time limit in seconds",
      OFFSET(pat_period_us), AV_OPT_TYPE_DURATION, { .i64 = PAT_RETRANS_TIME * 1000LL }, 0, INT64_MAX, ENC },
    { "sdt_period", "SDT retransmission time limit in seconds",
      OFFSET(sdt_period_us), AV_OPT_TYPE_DURATION, { .i64 = SDT_RETRANS_TIME * 1000LL }, 0, INT64_MAX, ENC },
    /* tsi options */
    { "tsi_active", "Activate tsi mode",
      OFFSET(tsi.is_active), AV_OPT_TYPE_BOOL, { .i64 = 0}, 0, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { "tsi_realtime", "Realtime streaming",
      OFFSET(tsi.is_realtime), AV_OPT_TYPE_BOOL, { .i64 = 0}, 0, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { "tsi_sync_service_times", "Synchronize MPTS service times by shifting pts/dts/pcr",
      OFFSET(tsi.sync_service_times), AV_OPT_TYPE_BOOL, { .i64 = 0}, 0, 1, AV_OPT_FLAG_ENCODING_PARAM },
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
    .flags          = AVFMT_ALLOW_FLUSH | AVFMT_VARIABLE_FPS | AVFMT_NODIMENSIONS,
    .priv_class     = &mpegts_muxer_class,
};
