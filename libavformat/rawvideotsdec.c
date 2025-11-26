/*
 * RAW video with timestamp demuxer
 * Copyright (c) 2024
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

#include "config_components.h"

#include "libavutil/imgutils.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavutil/intreadwrite.h"
#include "demux.h"
#include "internal.h"
#include "avformat.h"

typedef struct RawVideoTSDemuxerContext {
    const AVClass *class;     /**< Class for private options. */
    int width, height;        /**< Integers describing video size, set by a private option. */
    char *pixel_format;       /**< Set by a private option. */
    AVRational framerate;     /**< AVRational describing framerate, set by a private option. */
    int packet_size;          /**< Size of a single video frame in bytes */
    AVPacket *prev_pkt;       /**< Buffered previous packet for duration calculation */
    int64_t prev_pts;         /**< Previous frame PTS for duration calculation */
    int64_t default_duration; /**< Default frame duration in microseconds for last frame */
} RawVideoTSDemuxerContext;

static int rawvideots_read_header(AVFormatContext *ctx)
{
    RawVideoTSDemuxerContext *s = ctx->priv_data;
    enum AVPixelFormat pix_fmt;
    AVStream *st;
    int ret;

    st = avformat_new_stream(ctx, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;

    if ((pix_fmt = av_get_pix_fmt(s->pixel_format)) == AV_PIX_FMT_NONE) {
        av_log(ctx, AV_LOG_ERROR, "No such pixel format: %s.\n",
               s->pixel_format);
        return AVERROR(EINVAL);
    }

    /* Set time base to microseconds (1/1000000) */
    avpriv_set_pts_info(st, 64, 1, 1000000);

    ret = av_image_check_size(s->width, s->height, 0, ctx);
    if (ret < 0)
        return ret;

    st->codecpar->width  = s->width;
    st->codecpar->height = s->height;
    st->codecpar->format = pix_fmt;

    s->packet_size = av_image_get_buffer_size(pix_fmt, s->width, s->height, 1);
    if (s->packet_size < 0)
        return s->packet_size;

    if (s->packet_size == 0)
        return AVERROR(EINVAL);

    /* Calculate bit rate based on frame size and framerate.
     * Note: For VFR content, this is only an estimate based on the
     * configured framerate and may not reflect actual bitrate. */
    if (s->framerate.num > 0 && s->framerate.den > 0) {
        st->codecpar->bit_rate = av_rescale_q(s->packet_size * 8,
                                           s->framerate,
                                           (AVRational){1, 1});
        /* Pre-calculate default duration for last frame */
        s->default_duration = av_rescale_q(1, av_inv_q(s->framerate), (AVRational){1, 1000000});
    } else {
        s->default_duration = 0;
    }

    s->prev_pkt = NULL;
    s->prev_pts = AV_NOPTS_VALUE;

    return 0;
}

static int rawvideots_read_packet(AVFormatContext *ctx, AVPacket *pkt)
{
    RawVideoTSDemuxerContext *s = ctx->priv_data;
    uint8_t ts_buf[8];
    int64_t pts_us;
    int ret;
    AVPacket *tmp_pkt;

    /* Loop to handle first packet buffering without recursion */
    while (1) {
        /* If we have a buffered packet from the previous call, return it now */
        if (s->prev_pkt) {
            /* Read the next timestamp to calculate duration for the buffered packet */
            ret = avio_read(ctx->pb, ts_buf, 8);
            if (ret < 0) {
                /* Error reading next timestamp, return buffered packet with estimated duration */
                av_packet_move_ref(pkt, s->prev_pkt);
                av_packet_free(&s->prev_pkt);
                pkt->duration = s->default_duration;
                /* Return success with buffered packet, not the error */
                return 0;
            }
            if (ret < 8) {
                /* EOF - return buffered packet with estimated duration */
                av_packet_move_ref(pkt, s->prev_pkt);
                av_packet_free(&s->prev_pkt);
                pkt->duration = s->default_duration;
                return 0;
            }

            pts_us = AV_RL64(ts_buf);

            /* Check for non-monotonic or duplicate PTS */
            if (pts_us <= s->prev_pts) {
                av_log(ctx, AV_LOG_ERROR, "Non-monotonic or duplicate PTS detected: previous=%"PRId64" current=%"PRId64"\n",
                       s->prev_pts, pts_us);
                av_packet_free(&s->prev_pkt);
                return AVERROR_INVALIDDATA;
            }

            /* Set duration for buffered packet */
            s->prev_pkt->duration = pts_us - s->prev_pts;

            /* Return the buffered packet */
            av_packet_move_ref(pkt, s->prev_pkt);

            /* Allocate new packet for current frame */
            tmp_pkt = av_packet_alloc();
            if (!tmp_pkt)
                return AVERROR(ENOMEM);

            /* Read current frame data */
            ret = av_get_packet(ctx->pb, tmp_pkt, s->packet_size);
            if (ret < 0) {
                av_packet_free(&tmp_pkt);
                return ret;
            }
            if (ret < s->packet_size) {
                av_log(ctx, AV_LOG_ERROR, "Incomplete frame: expected %d bytes, got %d\n",
                       s->packet_size, ret);
                av_packet_free(&tmp_pkt);
                return AVERROR_EOF;
            }

            tmp_pkt->pts = pts_us;
            tmp_pkt->dts = pts_us;
            tmp_pkt->stream_index = 0;

            /* Store this packet for next call */
            s->prev_pkt = tmp_pkt;
            s->prev_pts = pts_us;

            return 0;
        }

        /* First packet - read and buffer it, then loop to read next */
        ret = avio_read(ctx->pb, ts_buf, 8);
        if (ret < 0)
            return ret;
        if (ret < 8)
            return AVERROR_EOF;

        pts_us = AV_RL64(ts_buf);

        /* Allocate packet */
        tmp_pkt = av_packet_alloc();
        if (!tmp_pkt)
            return AVERROR(ENOMEM);

        /* Read frame data */
        ret = av_get_packet(ctx->pb, tmp_pkt, s->packet_size);
        if (ret < 0) {
            av_packet_free(&tmp_pkt);
            return ret;
        }
        if (ret < s->packet_size) {
            av_log(ctx, AV_LOG_ERROR, "Incomplete frame: expected %d bytes, got %d\n",
                   s->packet_size, ret);
            av_packet_free(&tmp_pkt);
            return AVERROR_EOF;
        }

        tmp_pkt->pts = pts_us;
        tmp_pkt->dts = pts_us;
        tmp_pkt->stream_index = 0;

        /* Buffer this packet and loop to read the next one */
        s->prev_pkt = tmp_pkt;
        s->prev_pts = pts_us;
        /* Continue loop to read next packet */
    }
}

static int rawvideots_read_close(AVFormatContext *ctx)
{
    RawVideoTSDemuxerContext *s = ctx->priv_data;
    av_packet_free(&s->prev_pkt);
    return 0;
}

#define OFFSET(x) offsetof(RawVideoTSDemuxerContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM
static const AVOption rawvideots_options[] = {
    { "pixel_format", "set pixel format", OFFSET(pixel_format), AV_OPT_TYPE_STRING, {.str = "yuv420p"}, 0, 0, DEC },
    { "video_size", "set frame size", OFFSET(width), AV_OPT_TYPE_IMAGE_SIZE, {.str = NULL}, 0, 0, DEC },
    { "framerate", "set frame rate (for initial duration estimate)", OFFSET(framerate), AV_OPT_TYPE_VIDEO_RATE, {.str = "25"}, 0, INT_MAX, DEC },
    { NULL },
};

static const AVClass rawvideots_demuxer_class = {
    .class_name = "rawvideo_ts demuxer",
    .item_name  = av_default_item_name,
    .option     = rawvideots_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if CONFIG_RAWVIDEO_TS_DEMUXER
const FFInputFormat ff_rawvideo_ts_demuxer = {
    .p.name         = "rawvideo_ts",
    .p.long_name    = NULL_IF_CONFIG_SMALL("raw video with timestamps"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "rvts",
    .p.priv_class   = &rawvideots_demuxer_class,
    .priv_data_size = sizeof(RawVideoTSDemuxerContext),
    .read_header    = rawvideots_read_header,
    .read_packet    = rawvideots_read_packet,
    .read_close     = rawvideots_read_close,
    .raw_codec_id   = AV_CODEC_ID_RAWVIDEO,
};
#endif // CONFIG_RAWVIDEO_TS_DEMUXER
