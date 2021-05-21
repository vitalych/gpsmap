// Copyright (c) 2020 Vitaly Chipounov
// Copyright (c) 2003 Fabrice Bellard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <sstream>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/mathematics.h>
#include <libavutil/opt.h>
#include <libavutil/timestamp.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

#include <gpsmap/encoder.h>

using namespace gpsmap;

bool g_logpacket = false;

#define STREAM_PIX_FMT AV_PIX_FMT_YUV420P
#define SCALE_FLAGS SWS_BICUBIC

static inline std::string cpp_av_err2str(int errnum) {
    char errbuf[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(errnum, errbuf, sizeof(errbuf));
    return errbuf;
}

static inline std::string cpp_av_ts2str(int64_t ts) {
    char str[AV_TS_MAX_STRING_SIZE];
    av_ts_make_string(str, ts);
    return str;
}

static inline std::string cpp_av_ts2timestr(int64_t ts, AVRational *tb) {
    char buf[AV_TS_MAX_STRING_SIZE];

    if (ts == AV_NOPTS_VALUE) {
        snprintf(buf, AV_TS_MAX_STRING_SIZE, "NOPTS");
    } else {
        snprintf(buf, AV_TS_MAX_STRING_SIZE, "%.10g", av_q2d(*tb) * ts);
    }
    return buf;
}

static void log_packet(const AVFormatContext *fmt_ctx, const AVPacket *pkt) {
    AVRational *time_base = &fmt_ctx->streams[pkt->stream_index]->time_base;

    auto pts = cpp_av_ts2str(pkt->pts);
    auto dts = cpp_av_ts2str(pkt->dts);
    auto duration = cpp_av_ts2str(pkt->duration);
    auto dts_tb = cpp_av_ts2timestr(pkt->dts, time_base);
    auto duration_tb = cpp_av_ts2timestr(pkt->duration, time_base);
    auto pts_tb = cpp_av_ts2timestr(pkt->pts, time_base);

    printf("pts:%10s pts_time:%15s dts:%10s dts_time:%15s duration:%15s duration_time:%10s stream_index:%d\n",
           pts.c_str(), pts_tb.c_str(), dts.c_str(), dts_tb.c_str(), duration.c_str(), duration_tb.c_str(),
           pkt->stream_index);
}

VideoEncoder::VideoEncoder(const std::string &filePath, int w, int h, AVRational fps, FrameGeneratorCallback cb) {
    m_filePath = filePath;
    m_width = w;
    m_height = h;
    m_fps = fps;
    m_generateFrame = cb;

    m_oc = nullptr;
    m_fmt = nullptr;
    m_opt = nullptr;
}

VideoEncoder::~VideoEncoder() {
    Finalize();
}

int VideoEncoder::WriteFrame(AVCodecContext *c, AVStream *st, AVFrame *frame) {
    int ret;

    // send the frame to the encoder
    ret = avcodec_send_frame(c, frame);
    if (ret < 0) {
        auto err = cpp_av_err2str(ret);
        fprintf(stderr, "Error sending a frame to the encoder: %s\n", err.c_str());
        exit(1);
    }

    while (ret >= 0) {
        AVPacket pkt = {0};

        ret = avcodec_receive_packet(c, &pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        else if (ret < 0) {
            auto err = cpp_av_err2str(ret);
            fprintf(stderr, "Error encoding a frame: %s\n", err.c_str());
            exit(1);
        }

        /* rescale output packet timestamp values from codec to stream timebase */
        av_packet_rescale_ts(&pkt, c->time_base, st->time_base);
        pkt.stream_index = st->index;

        /* Write the compressed frame to the media file. */
        if (g_logpacket) {
            log_packet(m_oc, &pkt);
        }
        ret = av_interleaved_write_frame(m_oc, &pkt);
        av_packet_unref(&pkt);
        if (ret < 0) {
            auto err = cpp_av_err2str(ret);
            fprintf(stderr, "Error while writing output packet: %s\n", err.c_str());
            exit(1);
        }
    }

    return ret == AVERROR_EOF ? 1 : 0;
}

/* Add an output stream. */
OutputStreamPtr VideoEncoder::AddStream(enum AVCodecID codec_id) {
    OutputStreamPtr ret = OutputStreamPtr(new OutputStream);

    AVCodecContext *c;

    ret->codec = avcodec_find_encoder(codec_id);
    if (!ret->codec) {
        fprintf(stderr, "Could not find encoder for '%s'\n", avcodec_get_name(codec_id));
        exit(1);
    }

    ret->st = avformat_new_stream(m_oc, NULL);
    if (!ret->st) {
        fprintf(stderr, "Could not allocate stream\n");
        exit(1);
    }
    ret->st->id = m_oc->nb_streams - 1;
    c = avcodec_alloc_context3(ret->codec);
    if (!c) {
        fprintf(stderr, "Could not alloc an encoding context\n");
        exit(1);
    }
    ret->enc = c;

    std::stringstream codecParams;
    codecParams << "pools=1:numa-pools=1:log-level=1:bframes=0:keyint=30";
    auto cp = codecParams.str();

    ret->st->time_base = av_inv_q(m_fps);
    ret->st->r_frame_rate = m_fps;
    ret->st->avg_frame_rate = m_fps;

    switch (ret->codec->type) {
        case AVMEDIA_TYPE_VIDEO:
            c->codec_id = codec_id;

            c->bit_rate = 400000;
            /* Resolution must be a multiple of two. */
            c->width = m_width;
            c->height = m_height;
            /* timebase: This is the fundamental unit of time (in seconds) in terms
             * of which frame timestamps are represented. For fixed-fps content,
             * timebase should be 1/framerate and timestamp increments should be
             * identical to 1. */
            c->ticks_per_frame = 1;
            c->time_base = av_inv_q(m_fps);
            c->framerate = m_fps;

            // Doesn't work with x265
            // c->thread_count = 1;
            // c->thread_type = FF_THREAD_FRAME;
            av_opt_set(c->priv_data, "x265-params", cp.c_str(), 0);

            c->gop_size = 12; /* emit one intra frame every twelve frames at most */
            c->pix_fmt = STREAM_PIX_FMT;
            if (c->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
                /* just for testing, we also add B-frames */
                c->max_b_frames = 2;
            }
            if (c->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
                /* Needed to avoid using macroblocks in which some coeffs overflow.
                 * This does not happen with normal video, it just happens here as
                 * the motion of the chroma plane does not match the luma plane. */
                c->mb_decision = 2;
            }
            break;

        default:
            break;
    }

    /* Some formats want stream headers to be separate. */
    if (m_oc->oformat->flags & AVFMT_GLOBALHEADER) {
        c->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    return ret;
}

/**************************************************************/
/* video output */

static AVFrame *alloc_picture(enum AVPixelFormat pix_fmt, int width, int height) {
    AVFrame *picture;
    int ret;

    picture = av_frame_alloc();
    if (!picture)
        return NULL;

    picture->format = pix_fmt;
    picture->width = width;
    picture->height = height;

    /* allocate the buffers for the frame data */
    ret = av_frame_get_buffer(picture, 0);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate frame data.\n");
        exit(1);
    }

    return picture;
}

bool VideoEncoder::OpenVideo(OutputStream *ost, AVDictionary *opt_arg) {
    int ret;
    AVCodecContext *c = ost->enc;
    AVDictionary *opt = NULL;

    av_dict_copy(&opt, opt_arg, 0);

    /* open the codec */
    ret = avcodec_open2(c, ost->codec, &opt);
    av_dict_free(&opt);
    if (ret < 0) {
        auto err = cpp_av_err2str(ret);
        fprintf(stderr, "Could not open video codec: %s\n", err.c_str());
        return false;
    }

    /* allocate and init a re-usable frame */
    ost->frame = alloc_picture(c->pix_fmt, c->width, c->height);
    if (!ost->frame) {
        fprintf(stderr, "Could not allocate video frame\n");
        return false;
    }

    /* If the output format is not YUV420P, then a temporary YUV420P
     * picture is needed too. It is then converted to the required
     * output format. */
    ost->tmp_frame = alloc_picture(AV_PIX_FMT_RGBA, c->width, c->height);
    if (!ost->tmp_frame) {
        fprintf(stderr, "Could not allocate temporary picture\n");
        return false;
    }

    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        fprintf(stderr, "Could not copy the stream parameters\n");
        return false;
    }

    return true;
}

AVFrame *VideoEncoder::GetVideoFrame() {
    AVCodecContext *c = m_video->enc;

    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally; make sure we do not overwrite it here */
    if (av_frame_make_writable(m_video->frame) < 0) {
        exit(1);
    }

    // We work with RGBA source, so we must convert it to YUV
    if (!m_video->sws_ctx) {
        m_video->sws_ctx = sws_getContext(c->width, c->height, AV_PIX_FMT_RGBA, c->width, c->height, c->pix_fmt,
                                          SCALE_FLAGS, NULL, NULL, NULL);
        if (!m_video->sws_ctx) {
            fprintf(stderr, "Could not initialize the conversion context\n");
            exit(1);
        }
    }

    if (!m_generateFrame(*this, *m_video)) {
        return nullptr;
    }

    sws_scale(m_video->sws_ctx, (const uint8_t *const *) m_video->tmp_frame->data, m_video->tmp_frame->linesize, 0,
              c->height, m_video->frame->data, m_video->frame->linesize);

    m_video->frame->pts = m_video->next_pts++;

    return m_video->frame;
}

/*
 * encode one video frame and send it to the muxer
 * return 1 when encoding is finished, 0 otherwise
 */
int VideoEncoder::WriteVideoFrame() {
    auto frame = GetVideoFrame();
    return WriteFrame(m_video->enc, m_video->st, frame);
}

OutputStream::~OutputStream() {
    if (enc) {
        avcodec_free_context(&enc);
    }

    if (frame) {
        av_frame_free(&frame);
    }

    if (tmp_frame) {
        av_frame_free(&tmp_frame);
    }

    if (sws_ctx) {
        sws_freeContext(sws_ctx);
    }

    if (swr_ctx) {
        swr_free(&swr_ctx);
    }
}

void VideoEncoder::Finalize() {
    if (!m_oc) {
        return;
    }

    m_video = nullptr;

    if (m_oc->pb && !(m_fmt->flags & AVFMT_NOFILE)) {
        /* Write the trailer, if any. The trailer must be written before you
         * close the CodecContexts open when you wrote the header; otherwise
         * av_write_trailer() may try to use memory that was freed on
         * av_codec_close(). */
        av_write_trailer(m_oc);

        avio_closep(&m_oc->pb);
    }

    avformat_free_context(m_oc);

    m_oc = nullptr;
}

bool VideoEncoder::Initialize() {
    avformat_alloc_output_context2(&m_oc, NULL, NULL, m_filePath.c_str());
    if (!m_oc) {
        return false;
    }

    m_fmt = m_oc->oformat;

    m_video = AddStream(AV_CODEC_ID_HEVC);
    if (!m_video) {
        return false;
    }

    if (!OpenVideo(m_video.get(), m_opt)) {
        return false;
    }

    av_dump_format(m_oc, 0, m_filePath.c_str(), 1);

    /* open the output file, if needed */
    if (!(m_fmt->flags & AVFMT_NOFILE)) {
        auto ret = avio_open(&m_oc->pb, m_filePath.c_str(), AVIO_FLAG_WRITE);
        if (ret < 0) {
            auto err = cpp_av_err2str(ret);
            fprintf(stderr, "Could not open '%s': %s\n", m_filePath.c_str(), err.c_str());
            return false;
        }
    }

    /* Write the stream header, if any. */
    auto ret = avformat_write_header(m_oc, &m_opt);
    if (ret < 0) {
        auto err = cpp_av_err2str(ret);
        fprintf(stderr, "Error occurred when opening output file: %s\n", err.c_str());
        return false;
    }

    return true;
}

void VideoEncoder::EncodeLoop() {
    bool encode_video = true;
    while (encode_video) {
        encode_video = !WriteVideoFrame();
    }
}

// TODO: use imagebuf for that
void VideoEncoder::ClearFrame(OutputStream &os) {
    int width = Width();
    int height = Height();

    auto pict = os.tmp_frame;
    int ls = pict->linesize[0] / sizeof(uint32_t);
    uint32_t *pixels = (uint32_t *) pict->data[0];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            pixels[y * ls + x] = rgba(0, 0, 0, 0xff);
        }
    }
}

VideoEncoderPtr VideoEncoder::Create(const std::string &filePath, int w, int h, AVRational fps,
                                     FrameGeneratorCallback cb) {
    av_log_set_level(AV_LOG_QUIET);
    auto ret = VideoEncoderPtr(new VideoEncoder(filePath, w, h, fps, cb));
    if (!ret->Initialize()) {
        return nullptr;
    }

    return ret;
}
