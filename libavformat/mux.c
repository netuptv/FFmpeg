/*
 * muxing functions for use within FFmpeg
 * Copyright (c) 2000, 2001, 2002 Fabrice Bellard
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

#include "avformat.h"
#include "avio_internal.h"
#include "internal.h"
#include "libavcodec/internal.h"
#include "libavcodec/bytestream.h"
#include "libavutil/opt.h"
#include "libavutil/dict.h"
#include "libavutil/pixdesc.h"
#include "libavutil/timestamp.h"
#include "metadata.h"
#include "id3v2.h"
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/internal.h"
#include "libavutil/mathematics.h"
#include "libavutil/parseutils.h"
#include "libavutil/time.h"
#include "riff.h"
#include "audiointerleave.h"
#include "url.h"
#include <stdarg.h>
#if CONFIG_NETWORK
#include "network.h"
#endif

/**
 * @file
 * muxing functions for use within libavformat
 */

/* fraction handling */

/**
 * f = val + (num / den) + 0.5.
 *
 * 'num' is normalized so that it is such as 0 <= num < den.
 *
 * @param f fractional number
 * @param val integer value
 * @param num must be >= 0
 * @param den must be >= 1
 */
static void frac_init(AVFrac *f, int64_t val, int64_t num, int64_t den)
{
    num += (den >> 1);
    if (num >= den) {
        val += num / den;
        num  = num % den;
    }
    f->val = val;
    f->num = num;
    f->den = den;
}

/**
 * Fractional addition to f: f = f + (incr / f->den).
 *
 * @param f fractional number
 * @param incr increment, can be positive or negative
 */
static void frac_add(AVFrac *f, int64_t incr)
{
    int64_t num, den;

    num = f->num + incr;
    den = f->den;
    if (num < 0) {
        f->val += num / den;
        num     = num % den;
        if (num < 0) {
            num += den;
            f->val--;
        }
    } else if (num >= den) {
        f->val += num / den;
        num     = num % den;
    }
    f->num = num;
}

AVRational ff_choose_timebase(AVFormatContext *s, AVStream *st, int min_precision)
{
    AVRational q;
    int j;

    q = st->time_base;

    for (j=2; j<14; j+= 1+(j>2))
        while (q.den / q.num < min_precision && q.num % j == 0)
            q.num /= j;
    while (q.den / q.num < min_precision && q.den < (1<<24))
        q.den <<= 1;

    return q;
}

enum AVChromaLocation ff_choose_chroma_location(AVFormatContext *s, AVStream *st)
{
    AVCodecContext *avctx = st->codec;
    const AVPixFmtDescriptor *pix_desc = av_pix_fmt_desc_get(avctx->pix_fmt);

    if (avctx->chroma_sample_location != AVCHROMA_LOC_UNSPECIFIED)
        return avctx->chroma_sample_location;

    if (pix_desc) {
        if (pix_desc->log2_chroma_h == 0) {
            return AVCHROMA_LOC_TOPLEFT;
        } else if (pix_desc->log2_chroma_w == 1 && pix_desc->log2_chroma_h == 1) {
            if (avctx->field_order == AV_FIELD_UNKNOWN || avctx->field_order == AV_FIELD_PROGRESSIVE) {
                switch (avctx->codec_id) {
                case AV_CODEC_ID_MJPEG:
                case AV_CODEC_ID_MPEG1VIDEO: return AVCHROMA_LOC_CENTER;
                }
            }
            if (avctx->field_order == AV_FIELD_UNKNOWN || avctx->field_order != AV_FIELD_PROGRESSIVE) {
                switch (avctx->codec_id) {
                case AV_CODEC_ID_MPEG2VIDEO: return AVCHROMA_LOC_LEFT;
                }
            }
        }
    }

    return AVCHROMA_LOC_UNSPECIFIED;

}

int avformat_alloc_output_context2(AVFormatContext **avctx, AVOutputFormat *oformat,
                                   const char *format, const char *filename)
{
    AVFormatContext *s = avformat_alloc_context();
    int ret = 0;

    *avctx = NULL;
    if (!s)
        goto nomem;

    if (!oformat) {
        if (format) {
            oformat = av_guess_format(format, NULL, NULL);
            if (!oformat) {
                av_log(s, AV_LOG_ERROR, "Requested output format '%s' is not a suitable output format\n", format);
                ret = AVERROR(EINVAL);
                goto error;
            }
        } else {
            oformat = av_guess_format(NULL, filename, NULL);
            if (!oformat) {
                ret = AVERROR(EINVAL);
                av_log(s, AV_LOG_ERROR, "Unable to find a suitable output format for '%s'\n",
                       filename);
                goto error;
            }
        }
    }

    s->oformat = oformat;
    if (s->oformat->priv_data_size > 0) {
        s->priv_data = av_mallocz(s->oformat->priv_data_size);
        if (!s->priv_data)
            goto nomem;
        if (s->oformat->priv_class) {
            *(const AVClass**)s->priv_data= s->oformat->priv_class;
            av_opt_set_defaults(s->priv_data);
        }
    } else
        s->priv_data = NULL;

    if (filename)
        av_strlcpy(s->filename, filename, sizeof(s->filename));
    *avctx = s;
    return 0;
nomem:
    av_log(s, AV_LOG_ERROR, "Out of memory\n");
    ret = AVERROR(ENOMEM);
error:
    avformat_free_context(s);
    return ret;
}

static int validate_codec_tag(AVFormatContext *s, AVStream *st)
{
    const AVCodecTag *avctag;
    int n;
    enum AVCodecID id = AV_CODEC_ID_NONE;
    int64_t tag  = -1;

    /**
     * Check that tag + id is in the table
     * If neither is in the table -> OK
     * If tag is in the table with another id -> FAIL
     * If id is in the table with another tag -> FAIL unless strict < normal
     */
    for (n = 0; s->oformat->codec_tag[n]; n++) {
        avctag = s->oformat->codec_tag[n];
        while (avctag->id != AV_CODEC_ID_NONE) {
            if (avpriv_toupper4(avctag->tag) == avpriv_toupper4(st->codec->codec_tag)) {
                id = avctag->id;
                if (id == st->codec->codec_id)
                    return 1;
            }
            if (avctag->id == st->codec->codec_id)
                tag = avctag->tag;
            avctag++;
        }
    }
    if (id != AV_CODEC_ID_NONE)
        return 0;
    if (tag >= 0 && (s->strict_std_compliance >= FF_COMPLIANCE_NORMAL))
        return 0;
    return 1;
}


static int init_muxer(AVFormatContext *s, AVDictionary **options)
{
    int ret = 0, i;
    AVStream *st;
    AVDictionary *tmp = NULL;
    AVCodecContext *codec = NULL;
    AVOutputFormat *of = s->oformat;
    AVDictionaryEntry *e;

    if (options)
        av_dict_copy(&tmp, *options, 0);

    if ((ret = av_opt_set_dict(s, &tmp)) < 0)
        goto fail;
    if (s->priv_data && s->oformat->priv_class && *(const AVClass**)s->priv_data==s->oformat->priv_class &&
        (ret = av_opt_set_dict2(s->priv_data, &tmp, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto fail;

#if FF_API_LAVF_BITEXACT
    if (s->nb_streams && s->streams[0]->codec->flags & CODEC_FLAG_BITEXACT)
        s->flags |= AVFMT_FLAG_BITEXACT;
#endif

    // some sanity checks
    if (s->nb_streams == 0 && !(of->flags & AVFMT_NOSTREAMS)) {
        av_log(s, AV_LOG_ERROR, "No streams to mux were specified\n");
        ret = AVERROR(EINVAL);
        goto fail;
    }

    for (i = 0; i < s->nb_streams; i++) {
        st    = s->streams[i];
        codec = st->codec;

#if FF_API_LAVF_CODEC_TB
FF_DISABLE_DEPRECATION_WARNINGS
        if (!st->time_base.num && codec->time_base.num) {
            av_log(s, AV_LOG_WARNING, "Using AVStream.codec.time_base as a "
                   "timebase hint to the muxer is deprecated. Set "
                   "AVStream.time_base instead.\n");
            avpriv_set_pts_info(st, 64, codec->time_base.num, codec->time_base.den);
        }
FF_ENABLE_DEPRECATION_WARNINGS
#endif

        if (!st->time_base.num) {
            /* fall back on the default timebase values */
            if (codec->codec_type == AVMEDIA_TYPE_AUDIO && codec->sample_rate)
                avpriv_set_pts_info(st, 64, 1, codec->sample_rate);
            else
                avpriv_set_pts_info(st, 33, 1, 90000);
        }

        switch (codec->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            if (codec->sample_rate <= 0) {
                av_log(s, AV_LOG_ERROR, "sample rate not set\n");
                ret = AVERROR(EINVAL);
                goto fail;
            }
            if (!codec->block_align)
                codec->block_align = codec->channels *
                                     av_get_bits_per_sample(codec->codec_id) >> 3;
            break;
        case AVMEDIA_TYPE_VIDEO:
            if ((codec->width <= 0 || codec->height <= 0) &&
                !(of->flags & AVFMT_NODIMENSIONS)) {
                av_log(s, AV_LOG_ERROR, "dimensions not set\n");
                ret = AVERROR(EINVAL);
                goto fail;
            }
            if (av_cmp_q(st->sample_aspect_ratio, codec->sample_aspect_ratio)
                && FFABS(av_q2d(st->sample_aspect_ratio) - av_q2d(codec->sample_aspect_ratio)) > 0.004*av_q2d(st->sample_aspect_ratio)
            ) {
                if (st->sample_aspect_ratio.num != 0 &&
                    st->sample_aspect_ratio.den != 0 &&
                    codec->sample_aspect_ratio.num != 0 &&
                    codec->sample_aspect_ratio.den != 0) {
                    av_log(s, AV_LOG_ERROR, "Aspect ratio mismatch between muxer "
                           "(%d/%d) and encoder layer (%d/%d)\n",
                           st->sample_aspect_ratio.num, st->sample_aspect_ratio.den,
                           codec->sample_aspect_ratio.num,
                           codec->sample_aspect_ratio.den);
                    ret = AVERROR(EINVAL);
                    goto fail;
                }
            }
            break;
        }

        if (of->codec_tag) {
            if (   codec->codec_tag
                && codec->codec_id == AV_CODEC_ID_RAWVIDEO
                && (   av_codec_get_tag(of->codec_tag, codec->codec_id) == 0
                    || av_codec_get_tag(of->codec_tag, codec->codec_id) == MKTAG('r', 'a', 'w', ' '))
                && !validate_codec_tag(s, st)) {
                // the current rawvideo encoding system ends up setting
                // the wrong codec_tag for avi/mov, we override it here
                codec->codec_tag = 0;
            }
            if (codec->codec_tag) {
                if (!validate_codec_tag(s, st)) {
                    char tagbuf[32], tagbuf2[32];
                    av_get_codec_tag_string(tagbuf, sizeof(tagbuf), codec->codec_tag);
                    av_get_codec_tag_string(tagbuf2, sizeof(tagbuf2), av_codec_get_tag(s->oformat->codec_tag, codec->codec_id));
                    av_log(s, AV_LOG_ERROR,
                           "Tag %s/0x%08x incompatible with output codec id '%d' (%s)\n",
                           tagbuf, codec->codec_tag, codec->codec_id, tagbuf2);
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }
            } else
                codec->codec_tag = av_codec_get_tag(of->codec_tag, codec->codec_id);
        }

        if (of->flags & AVFMT_GLOBALHEADER &&
            !(codec->flags & CODEC_FLAG_GLOBAL_HEADER))
            av_log(s, AV_LOG_WARNING,
                   "Codec for stream %d does not use global headers "
                   "but container format requires global headers\n", i);

        if (codec->codec_type != AVMEDIA_TYPE_ATTACHMENT)
            s->internal->nb_interleaved_streams++;
    }

    if (!s->priv_data && of->priv_data_size > 0) {
        s->priv_data = av_mallocz(of->priv_data_size);
        if (!s->priv_data) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        if (of->priv_class) {
            *(const AVClass **)s->priv_data = of->priv_class;
            av_opt_set_defaults(s->priv_data);
            if ((ret = av_opt_set_dict2(s->priv_data, &tmp, AV_OPT_SEARCH_CHILDREN)) < 0)
                goto fail;
        }
    }

    /* set muxer identification string */
    if (!(s->flags & AVFMT_FLAG_BITEXACT)) {
        av_dict_set(&s->metadata, "encoder", LIBAVFORMAT_IDENT, 0);
    } else {
        av_dict_set(&s->metadata, "encoder", NULL, 0);
    }

    for (e = NULL; e = av_dict_get(s->metadata, "encoder-", e, AV_DICT_IGNORE_SUFFIX); ) {
        av_dict_set(&s->metadata, e->key, NULL, 0);
    }

    if (options) {
         av_dict_free(options);
         *options = tmp;
    }

    return 0;

fail:
    av_dict_free(&tmp);
    return ret;
}

static int init_pts(AVFormatContext *s)
{
    int i;
    AVStream *st;

    /* init PTS generation */
    for (i = 0; i < s->nb_streams; i++) {
        int64_t den = AV_NOPTS_VALUE;
        st = s->streams[i];

        switch (st->codec->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            den = (int64_t)st->time_base.num * st->codec->sample_rate;
            break;
        case AVMEDIA_TYPE_VIDEO:
            den = (int64_t)st->time_base.num * st->codec->time_base.den;
            break;
        default:
            break;
        }
        if (den != AV_NOPTS_VALUE) {
            if (den <= 0)
                return AVERROR_INVALIDDATA;

            frac_init(&st->pts, 0, 0, den);
        }
    }

    return 0;
}

int avformat_write_header(AVFormatContext *s, AVDictionary **options)
{
    int ret = 0;

    if ((ret = init_muxer(s, options)) < 0)
        return ret;

    if (s->oformat->write_header) {
        ret = s->oformat->write_header(s);
        if (ret >= 0 && s->pb && s->pb->error < 0)
            ret = s->pb->error;
        if (ret < 0)
            return ret;
        if (s->flush_packets && s->pb && s->pb->error >= 0 && s->flags & AVFMT_FLAG_FLUSH_PACKETS)
            avio_flush(s->pb);
    }

    if ((ret = init_pts(s)) < 0)
        return ret;

    if (s->avoid_negative_ts < 0) {
        av_assert2(s->avoid_negative_ts == AVFMT_AVOID_NEG_TS_AUTO);
        if (s->oformat->flags & (AVFMT_TS_NEGATIVE | AVFMT_NOTIMESTAMPS)) {
            s->avoid_negative_ts = 0;
        } else
            s->avoid_negative_ts = AVFMT_AVOID_NEG_TS_MAKE_NON_NEGATIVE;
    }

    return 0;
}

#define AV_PKT_FLAG_UNCODED_FRAME 0x2000

/* Note: using sizeof(AVFrame) from outside lavu is unsafe in general, but
   it is only being used internally to this file as a consistency check.
   The value is chosen to be very unlikely to appear on its own and to cause
   immediate failure if used anywhere as a real size. */
#define UNCODED_FRAME_PACKET_SIZE (INT_MIN / 3 * 2 + (int)sizeof(AVFrame))


//FIXME merge with compute_pkt_fields
static int compute_pkt_fields2(AVFormatContext *s, AVStream *st, AVPacket *pkt)
{
    int delay = FFMAX(st->codec->has_b_frames, st->codec->max_b_frames > 0);
    int num, den, i;
    int frame_size;

    if (s->debug & FF_FDEBUG_TS)
        av_log(s, AV_LOG_TRACE, "compute_pkt_fields2: pts:%s dts:%s cur_dts:%s b:%d size:%d st:%d\n",
            av_ts2str(pkt->pts), av_ts2str(pkt->dts), av_ts2str(st->cur_dts), delay, pkt->size, pkt->stream_index);

    if (pkt->duration < 0 && st->codec->codec_type != AVMEDIA_TYPE_SUBTITLE) {
        av_log(s, AV_LOG_WARNING, "Packet with invalid duration %d in stream %d\n",
               pkt->duration, pkt->stream_index);
        pkt->duration = 0;
    }

    /* duration field */
    if (pkt->duration == 0) {
        ff_compute_frame_duration(s, &num, &den, st, NULL, pkt);
        if (den && num) {
            pkt->duration = av_rescale(1, num * (int64_t)st->time_base.den * st->codec->ticks_per_frame, den * (int64_t)st->time_base.num);
        }
    }

    if (pkt->pts == AV_NOPTS_VALUE && pkt->dts != AV_NOPTS_VALUE && delay == 0)
        pkt->pts = pkt->dts;

    //XXX/FIXME this is a temporary hack until all encoders output pts
    if ((pkt->pts == 0 || pkt->pts == AV_NOPTS_VALUE) && pkt->dts == AV_NOPTS_VALUE && !delay) {
        static int warned;
        if (!warned) {
            av_log(s, AV_LOG_WARNING, "Encoder did not produce proper pts, making some up.\n");
            warned = 1;
        }
        pkt->dts =
//        pkt->pts= st->cur_dts;
            pkt->pts = st->pts.val;
    }

    //calculate dts from pts
    if (pkt->pts != AV_NOPTS_VALUE && pkt->dts == AV_NOPTS_VALUE && delay <= MAX_REORDER_DELAY) {
        st->pts_buffer[0] = pkt->pts;
        for (i = 1; i < delay + 1 && st->pts_buffer[i] == AV_NOPTS_VALUE; i++)
            st->pts_buffer[i] = pkt->pts + (i - delay - 1) * pkt->duration;
        for (i = 0; i<delay && st->pts_buffer[i] > st->pts_buffer[i + 1]; i++)
            FFSWAP(int64_t, st->pts_buffer[i], st->pts_buffer[i + 1]);

        pkt->dts = st->pts_buffer[0];
    }

    if (st->cur_dts && st->cur_dts != AV_NOPTS_VALUE &&
        ((!(s->oformat->flags & AVFMT_TS_NONSTRICT) &&
          st->codec->codec_type != AVMEDIA_TYPE_SUBTITLE &&
          st->cur_dts >= pkt->dts) || st->cur_dts > pkt->dts)) {
        av_log(s, AV_LOG_ERROR,
               "Application provided invalid, non monotonically increasing dts to muxer in stream %d: %s >= %s\n",
               st->index, av_ts2str(st->cur_dts), av_ts2str(pkt->dts));
        return AVERROR(EINVAL);
    }
    if (pkt->dts != AV_NOPTS_VALUE && pkt->pts != AV_NOPTS_VALUE && pkt->pts < pkt->dts) {
        av_log(s, AV_LOG_ERROR,
               "pts (%s) < dts (%s) in stream %d\n",
               av_ts2str(pkt->pts), av_ts2str(pkt->dts),
               st->index);
        return AVERROR(EINVAL);
    }

    if (s->debug & FF_FDEBUG_TS)
        av_log(s, AV_LOG_TRACE, "av_write_frame: pts2:%s dts2:%s\n",
            av_ts2str(pkt->pts), av_ts2str(pkt->dts));

    st->cur_dts = pkt->dts;
    st->pts.val = pkt->dts;

    /* update pts */
    switch (st->codec->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        frame_size = (pkt->flags & AV_PKT_FLAG_UNCODED_FRAME) ?
                     ((AVFrame *)pkt->data)->nb_samples :
                     av_get_audio_frame_duration(st->codec, pkt->size);

        /* HACK/FIXME, we skip the initial 0 size packets as they are most
         * likely equal to the encoder delay, but it would be better if we
         * had the real timestamps from the encoder */
        if (frame_size >= 0 && (pkt->size || st->pts.num != st->pts.den >> 1 || st->pts.val)) {
            frac_add(&st->pts, (int64_t)st->time_base.den * frame_size);
        }
        break;
    case AVMEDIA_TYPE_VIDEO:
        frac_add(&st->pts, (int64_t)st->time_base.den * st->codec->time_base.num);
        break;
    }
    return 0;
}

/**
 * Make timestamps non negative, move side data from payload to internal struct, call muxer, and restore
 * sidedata.
 *
 * FIXME: this function should NEVER get undefined pts/dts beside when the
 * AVFMT_NOTIMESTAMPS is set.
 * Those additional safety checks should be dropped once the correct checks
 * are set in the callers.
 */
static int write_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, did_split;

    if (s->output_ts_offset) {
        AVStream *st = s->streams[pkt->stream_index];
        int64_t offset = av_rescale_q(s->output_ts_offset, AV_TIME_BASE_Q, st->time_base);

        if (pkt->dts != AV_NOPTS_VALUE)
            pkt->dts += offset;
        if (pkt->pts != AV_NOPTS_VALUE)
            pkt->pts += offset;
    }

    if (s->avoid_negative_ts > 0) {
        AVStream *st = s->streams[pkt->stream_index];
        int64_t offset = st->mux_ts_offset;
        int64_t ts = s->internal->avoid_negative_ts_use_pts ? pkt->pts : pkt->dts;

        if (s->internal->offset == AV_NOPTS_VALUE && ts != AV_NOPTS_VALUE &&
            (ts < 0 || s->avoid_negative_ts == AVFMT_AVOID_NEG_TS_MAKE_ZERO)) {
            s->internal->offset = -ts;
            s->internal->offset_timebase = st->time_base;
        }

        if (s->internal->offset != AV_NOPTS_VALUE && !offset) {
            offset = st->mux_ts_offset =
                av_rescale_q_rnd(s->internal->offset,
                                 s->internal->offset_timebase,
                                 st->time_base,
                                 AV_ROUND_UP);
        }

        if (pkt->dts != AV_NOPTS_VALUE)
            pkt->dts += offset;
        if (pkt->pts != AV_NOPTS_VALUE)
            pkt->pts += offset;

        if (s->internal->avoid_negative_ts_use_pts) {
            if (pkt->pts != AV_NOPTS_VALUE && pkt->pts < 0) {
                av_log(s, AV_LOG_WARNING, "failed to avoid negative "
                    "pts %s in stream %d.\n"
                    "Try -avoid_negative_ts 1 as a possible workaround.\n",
                    av_ts2str(pkt->dts),
                    pkt->stream_index
                );
            }
        } else {
            av_assert2(pkt->dts == AV_NOPTS_VALUE || pkt->dts >= 0 || s->max_interleave_delta > 0);
            if (pkt->dts != AV_NOPTS_VALUE && pkt->dts < 0) {
                av_log(s, AV_LOG_WARNING,
                    "Packets poorly interleaved, failed to avoid negative "
                    "timestamp %s in stream %d.\n"
                    "Try -max_interleave_delta 0 as a possible workaround.\n",
                    av_ts2str(pkt->dts),
                    pkt->stream_index
                );
            }
        }
    }

    did_split = av_packet_split_side_data(pkt);
    if ((pkt->flags & AV_PKT_FLAG_UNCODED_FRAME)) {
        AVFrame *frame = (AVFrame *)pkt->data;
        av_assert0(pkt->size == UNCODED_FRAME_PACKET_SIZE);
        ret = s->oformat->write_uncoded_frame(s, pkt->stream_index, &frame, 0);
        av_frame_free(&frame);
    } else {
        ret = s->oformat->write_packet(s, pkt);
    }

    if (s->flush_packets && s->pb && ret >= 0 && s->flags & AVFMT_FLAG_FLUSH_PACKETS)
        avio_flush(s->pb);

    if (did_split)
        av_packet_merge_side_data(pkt);

    return ret;
}

static int check_packet(AVFormatContext *s, AVPacket *pkt)
{
    if (!pkt)
        return 0;

    if (pkt->stream_index < 0 || pkt->stream_index >= s->nb_streams) {
        av_log(s, AV_LOG_ERROR, "Invalid packet stream index: %d\n",
               pkt->stream_index);
        return AVERROR(EINVAL);
    }

    if (s->streams[pkt->stream_index]->codec->codec_type == AVMEDIA_TYPE_ATTACHMENT) {
        av_log(s, AV_LOG_ERROR, "Received a packet for an attachment stream.\n");
        return AVERROR(EINVAL);
    }

    return 0;
}

int av_write_frame(AVFormatContext *s, AVPacket *pkt)
{
    int ret;

    ret = check_packet(s, pkt);
    if (ret < 0)
        return ret;

    if (!pkt) {
        if (s->oformat->flags & AVFMT_ALLOW_FLUSH) {
            ret = s->oformat->write_packet(s, NULL);
            if (s->flush_packets && s->pb && s->pb->error >= 0 && s->flags & AVFMT_FLAG_FLUSH_PACKETS)
                avio_flush(s->pb);
            if (ret >= 0 && s->pb && s->pb->error < 0)
                ret = s->pb->error;
            return ret;
        }
        return 1;
    }

    ret = compute_pkt_fields2(s, s->streams[pkt->stream_index], pkt);

    if (ret < 0 && !(s->oformat->flags & AVFMT_NOTIMESTAMPS))
        return ret;

    ret = write_packet(s, pkt);
    if (ret >= 0 && s->pb && s->pb->error < 0)
        ret = s->pb->error;

    if (ret >= 0)
        s->streams[pkt->stream_index]->nb_frames++;
    return ret;
}

#define CHUNK_START 0x1000

int ff_interleave_add_packet(AVFormatContext *s, AVPacket *pkt,
                             int (*compare)(AVFormatContext *, AVPacket *, AVPacket *))
{
    int ret;
    AVPacketList **next_point, *this_pktl;
    AVStream *st   = s->streams[pkt->stream_index];
    int chunked    = s->max_chunk_size || s->max_chunk_duration;

    this_pktl      = av_mallocz(sizeof(AVPacketList));
    if (!this_pktl)
        return AVERROR(ENOMEM);
    this_pktl->pkt = *pkt;
#if FF_API_DESTRUCT_PACKET
FF_DISABLE_DEPRECATION_WARNINGS
    pkt->destruct  = NULL;           // do not free original but only the copy
FF_ENABLE_DEPRECATION_WARNINGS
#endif
    pkt->buf       = NULL;
    pkt->side_data = NULL;
    pkt->side_data_elems = 0;
    if ((pkt->flags & AV_PKT_FLAG_UNCODED_FRAME)) {
        av_assert0(pkt->size == UNCODED_FRAME_PACKET_SIZE);
        av_assert0(((AVFrame *)pkt->data)->buf);
    } else {
        // Duplicate the packet if it uses non-allocated memory
        if ((ret = av_dup_packet(&this_pktl->pkt)) < 0) {
            av_free(this_pktl);
            return ret;
        }
    }

    if (s->streams[pkt->stream_index]->last_in_packet_buffer) {
        next_point = &(st->last_in_packet_buffer->next);
    } else {
        next_point = &s->internal->packet_buffer;
    }

    if (chunked) {
        uint64_t max= av_rescale_q_rnd(s->max_chunk_duration, AV_TIME_BASE_Q, st->time_base, AV_ROUND_UP);
        st->interleaver_chunk_size     += pkt->size;
        st->interleaver_chunk_duration += pkt->duration;
        if (   (s->max_chunk_size && st->interleaver_chunk_size > s->max_chunk_size)
            || (max && st->interleaver_chunk_duration           > max)) {
            st->interleaver_chunk_size      = 0;
            this_pktl->pkt.flags |= CHUNK_START;
            if (max && st->interleaver_chunk_duration > max) {
                int64_t syncoffset = (st->codec->codec_type == AVMEDIA_TYPE_VIDEO)*max/2;
                int64_t syncto = av_rescale(pkt->dts + syncoffset, 1, max)*max - syncoffset;

                st->interleaver_chunk_duration += (pkt->dts - syncto)/8 - max;
            } else
                st->interleaver_chunk_duration = 0;
        }
    }
    if (*next_point) {
        if (chunked && !(this_pktl->pkt.flags & CHUNK_START))
            goto next_non_null;

        if (compare(s, &s->internal->packet_buffer_end->pkt, pkt)) {
            while (   *next_point
                   && ((chunked && !((*next_point)->pkt.flags&CHUNK_START))
                       || !compare(s, &(*next_point)->pkt, pkt)))
                next_point = &(*next_point)->next;
            if (*next_point)
                goto next_non_null;
        } else {
            next_point = &(s->internal->packet_buffer_end->next);
        }
    }
    av_assert1(!*next_point);

    s->internal->packet_buffer_end = this_pktl;
next_non_null:

    this_pktl->next = *next_point;

    s->streams[pkt->stream_index]->last_in_packet_buffer =
        *next_point                                      = this_pktl;

    return 0;
}

static int interleave_compare_dts(AVFormatContext *s, AVPacket *next,
                                  AVPacket *pkt)
{
    AVStream *st  = s->streams[pkt->stream_index];
    AVStream *st2 = s->streams[next->stream_index];
    int comp      = av_compare_ts(next->dts, st2->time_base, pkt->dts,
                                  st->time_base);
    if (s->audio_preload && ((st->codec->codec_type == AVMEDIA_TYPE_AUDIO) != (st2->codec->codec_type == AVMEDIA_TYPE_AUDIO))) {
        int64_t ts = av_rescale_q(pkt ->dts, st ->time_base, AV_TIME_BASE_Q) - s->audio_preload*(st ->codec->codec_type == AVMEDIA_TYPE_AUDIO);
        int64_t ts2= av_rescale_q(next->dts, st2->time_base, AV_TIME_BASE_Q) - s->audio_preload*(st2->codec->codec_type == AVMEDIA_TYPE_AUDIO);
        if (ts == ts2) {
            ts= ( pkt ->dts* st->time_base.num*AV_TIME_BASE - s->audio_preload*(int64_t)(st ->codec->codec_type == AVMEDIA_TYPE_AUDIO)* st->time_base.den)*st2->time_base.den
               -( next->dts*st2->time_base.num*AV_TIME_BASE - s->audio_preload*(int64_t)(st2->codec->codec_type == AVMEDIA_TYPE_AUDIO)*st2->time_base.den)* st->time_base.den;
            ts2=0;
        }
        comp= (ts<ts2) - (ts>ts2);
    }

    if (comp == 0)
        return pkt->stream_index < next->stream_index;
    return comp > 0;
}

int ff_interleave_packet_per_dts(AVFormatContext *s, AVPacket *out,
                                 AVPacket *pkt, int flush)
{
    AVPacketList *pktl;
    int stream_count = 0;
    int noninterleaved_count = 0;
    int i, ret;

    if (pkt) {
        if ((ret = ff_interleave_add_packet(s, pkt, interleave_compare_dts)) < 0)
            return ret;
    }

    for (i = 0; i < s->nb_streams; i++) {
        if (s->streams[i]->last_in_packet_buffer) {
            ++stream_count;
        } else if (s->streams[i]->codec->codec_type != AVMEDIA_TYPE_ATTACHMENT &&
                   s->streams[i]->codec->codec_id != AV_CODEC_ID_VP8 &&
                   s->streams[i]->codec->codec_id != AV_CODEC_ID_VP9) {
            ++noninterleaved_count;
        }
    }

    if (s->internal->nb_interleaved_streams == stream_count)
        flush = 1;

    if (s->max_interleave_delta > 0 &&
        s->internal->packet_buffer &&
        !flush &&
        s->internal->nb_interleaved_streams == stream_count+noninterleaved_count
    ) {
        AVPacket *top_pkt = &s->internal->packet_buffer->pkt;
        int64_t delta_dts = INT64_MIN;
        int64_t top_dts = av_rescale_q(top_pkt->dts,
                                       s->streams[top_pkt->stream_index]->time_base,
                                       AV_TIME_BASE_Q);

        for (i = 0; i < s->nb_streams; i++) {
            int64_t last_dts;
            const AVPacketList *last = s->streams[i]->last_in_packet_buffer;

            if (!last)
                continue;

            last_dts = av_rescale_q(last->pkt.dts,
                                    s->streams[i]->time_base,
                                    AV_TIME_BASE_Q);
            delta_dts = FFMAX(delta_dts, last_dts - top_dts);
        }

        if (delta_dts > s->max_interleave_delta) {
            av_log(s, AV_LOG_DEBUG,
                   "Delay between the first packet and last packet in the "
                   "muxing queue is %"PRId64" > %"PRId64": forcing output\n",
                   delta_dts, s->max_interleave_delta);
            flush = 1;
        }
    }

    if (stream_count && flush) {
        AVStream *st;
        pktl = s->internal->packet_buffer;
        *out = pktl->pkt;
        st   = s->streams[out->stream_index];

        s->internal->packet_buffer = pktl->next;
        if (!s->internal->packet_buffer)
            s->internal->packet_buffer_end = NULL;

        if (st->last_in_packet_buffer == pktl)
            st->last_in_packet_buffer = NULL;
        av_freep(&pktl);

        return 1;
    } else {
        av_init_packet(out);
        return 0;
    }
}

/**
 * Interleave an AVPacket correctly so it can be muxed.
 * @param out the interleaved packet will be output here
 * @param in the input packet
 * @param flush 1 if no further packets are available as input and all
 *              remaining packets should be output
 * @return 1 if a packet was output, 0 if no packet could be output,
 *         < 0 if an error occurred
 */
static int interleave_packet(AVFormatContext *s, AVPacket *out, AVPacket *in, int flush)
{
    if (s->oformat->interleave_packet) {
        int ret = s->oformat->interleave_packet(s, out, in, flush);
        if (in)
            av_free_packet(in);
        return ret;
    } else
        return ff_interleave_packet_per_dts(s, out, in, flush);
}

int av_interleaved_write_frame(AVFormatContext *s, AVPacket *pkt)
{
    int ret, flush = 0;

    ret = check_packet(s, pkt);
    if (ret < 0)
        goto fail;

    if (pkt) {
        AVStream *st = s->streams[pkt->stream_index];

        if (s->debug & FF_FDEBUG_TS)
            av_log(s, AV_LOG_TRACE, "av_interleaved_write_frame size:%d dts:%s pts:%s\n",
                pkt->size, av_ts2str(pkt->dts), av_ts2str(pkt->pts));

        if ((ret = compute_pkt_fields2(s, st, pkt)) < 0 && !(s->oformat->flags & AVFMT_NOTIMESTAMPS))
            goto fail;

        if (pkt->dts == AV_NOPTS_VALUE && !(s->oformat->flags & AVFMT_NOTIMESTAMPS)) {
            ret = AVERROR(EINVAL);
            goto fail;
        }
    } else {
        av_log(s, AV_LOG_TRACE, "av_interleaved_write_frame FLUSH\n");
        flush = 1;
    }

    for (;; ) {
        AVPacket opkt;
        int ret = interleave_packet(s, &opkt, pkt, flush);
        if (pkt) {
            memset(pkt, 0, sizeof(*pkt));
            av_init_packet(pkt);
            pkt = NULL;
        }
        if (ret <= 0) //FIXME cleanup needed for ret<0 ?
            return ret;

        ret = write_packet(s, &opkt);
        if (ret >= 0)
            s->streams[opkt.stream_index]->nb_frames++;

        av_free_packet(&opkt);

        if (ret < 0)
            return ret;
        if(s->pb && s->pb->error)
            return s->pb->error;
    }
fail:
    av_packet_unref(pkt);
    return ret;
}

int av_write_trailer(AVFormatContext *s)
{
    int ret, i;

    for (;; ) {
        AVPacket pkt;
        ret = interleave_packet(s, &pkt, NULL, 1);
        if (ret < 0)
            goto fail;
        if (!ret)
            break;

        ret = write_packet(s, &pkt);
        if (ret >= 0)
            s->streams[pkt.stream_index]->nb_frames++;

        av_free_packet(&pkt);

        if (ret < 0)
            goto fail;
        if(s->pb && s->pb->error)
            goto fail;
    }

fail:
    if (s->oformat->write_trailer)
        if (ret >= 0) {
        ret = s->oformat->write_trailer(s);
        } else {
            s->oformat->write_trailer(s);
        }

    if (s->pb)
       avio_flush(s->pb);
    if (ret == 0)
       ret = s->pb ? s->pb->error : 0;
    for (i = 0; i < s->nb_streams; i++) {
        av_freep(&s->streams[i]->priv_data);
        av_freep(&s->streams[i]->index_entries);
    }
    if (s->oformat->priv_class)
        av_opt_free(s->priv_data);
    av_freep(&s->priv_data);
    return ret;
}

int av_get_output_timestamp(struct AVFormatContext *s, int stream,
                            int64_t *dts, int64_t *wall)
{
    if (!s->oformat || !s->oformat->get_output_timestamp)
        return AVERROR(ENOSYS);
    s->oformat->get_output_timestamp(s, stream, dts, wall);
    return 0;
}

int ff_write_chained(AVFormatContext *dst, int dst_stream, AVPacket *pkt,
                     AVFormatContext *src, int interleave)
{
    AVPacket local_pkt;
    int ret;

    local_pkt = *pkt;
    local_pkt.stream_index = dst_stream;
    if (pkt->pts != AV_NOPTS_VALUE)
        local_pkt.pts = av_rescale_q(pkt->pts,
                                     src->streams[pkt->stream_index]->time_base,
                                     dst->streams[dst_stream]->time_base);
    if (pkt->dts != AV_NOPTS_VALUE)
        local_pkt.dts = av_rescale_q(pkt->dts,
                                     src->streams[pkt->stream_index]->time_base,
                                     dst->streams[dst_stream]->time_base);
    if (pkt->duration)
        local_pkt.duration = av_rescale_q(pkt->duration,
                                          src->streams[pkt->stream_index]->time_base,
                                          dst->streams[dst_stream]->time_base);

    if (interleave) ret = av_interleaved_write_frame(dst, &local_pkt);
    else            ret = av_write_frame(dst, &local_pkt);
    pkt->buf = local_pkt.buf;
    pkt->destruct = local_pkt.destruct;
    return ret;
}

static int av_write_uncoded_frame_internal(AVFormatContext *s, int stream_index,
                                           AVFrame *frame, int interleaved)
{
    AVPacket pkt, *pktp;

    av_assert0(s->oformat);
    if (!s->oformat->write_uncoded_frame)
        return AVERROR(ENOSYS);

    if (!frame) {
        pktp = NULL;
    } else {
        pktp = &pkt;
        av_init_packet(&pkt);
        pkt.data = (void *)frame;
        pkt.size         = UNCODED_FRAME_PACKET_SIZE;
        pkt.pts          =
        pkt.dts          = frame->pts;
        pkt.duration     = av_frame_get_pkt_duration(frame);
        pkt.stream_index = stream_index;
        pkt.flags |= AV_PKT_FLAG_UNCODED_FRAME;
    }

    return interleaved ? av_interleaved_write_frame(s, pktp) :
                         av_write_frame(s, pktp);
}

int av_write_uncoded_frame(AVFormatContext *s, int stream_index,
                           AVFrame *frame)
{
    return av_write_uncoded_frame_internal(s, stream_index, frame, 0);
}

int av_interleaved_write_uncoded_frame(AVFormatContext *s, int stream_index,
                                       AVFrame *frame)
{
    return av_write_uncoded_frame_internal(s, stream_index, frame, 1);
}

int av_write_uncoded_frame_query(AVFormatContext *s, int stream_index)
{
    av_assert0(s->oformat);
    if (!s->oformat->write_uncoded_frame)
        return AVERROR(ENOSYS);
    return s->oformat->write_uncoded_frame(s, stream_index, NULL,
                                           AV_WRITE_UNCODED_FRAME_QUERY);
}
