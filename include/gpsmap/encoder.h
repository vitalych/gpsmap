// Copyright (c) 2020 Vitaly Chipounov
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

#ifndef ENCODER_H

#define ENCODER_H

#include <functional>
#include <memory>
#include <string>

extern "C" {
struct AVOutputFormat;
struct AVFormatContext;
struct AVDictionary;
struct AVCodec;
struct AVStream;
struct AVCodecContext;
struct AVFrame;
#include <libavcodec/avcodec.h>
#include <libavutil/rational.h>
}

namespace gpsmap {

struct OutputStream {
    AVCodec *codec = nullptr;
    AVStream *st = nullptr;
    AVCodecContext *enc = nullptr;

    /* pts of the next frame that will be generated */
    int64_t next_pts = 0;
    int samples_count = 0;

    AVFrame *frame = nullptr;
    AVFrame *tmp_frame = nullptr;

    float t = 0.0f, tincr = 0.0f, tincr2 = 0.0f;

    struct SwsContext *sws_ctx = nullptr;
    struct SwrContext *swr_ctx = nullptr;

    ~OutputStream();

    void *GetPixels() {
        return tmp_frame->data[0];
    }
};

using OutputStreamPtr = std::shared_ptr<OutputStream>;

class VideoEncoder;
using VideoEncoderPtr = std::shared_ptr<VideoEncoder>;

using FrameGeneratorCallback = std::function<bool(VideoEncoder &encoder, OutputStream &os)>;

class VideoEncoder {
private:
    std::string m_filePath;
    int m_width;
    int m_height;
    AVRational m_fps;

    AVOutputFormat *m_fmt;
    AVFormatContext *m_oc;
    AVDictionary *m_opt;

    OutputStreamPtr m_video;

    FrameGeneratorCallback m_generateFrame;

    VideoEncoder(const std::string &filePath, int w, int h, AVRational fps, FrameGeneratorCallback cb);
    OutputStreamPtr AddStream(enum AVCodecID codec_id);
    bool OpenVideo(OutputStream *ost, AVDictionary *opt_arg);
    AVFrame *GetVideoFrame();
    int WriteFrame(AVCodecContext *c, AVStream *st, AVFrame *frame);
    int WriteVideoFrame();

    bool Initialize();

public:
    ~VideoEncoder();

    static VideoEncoderPtr Create(const std::string &filePath, int w, int h, AVRational fps, FrameGeneratorCallback cb);
    void EncodeLoop();
    void Finalize();

    void ClearFrame(OutputStream &os);

    int Height() const {
        return m_height;
    }

    int Width() const {
        return m_width;
    }

    double GetFPS() const {
        return av_q2d(m_fps);
    }
};

static inline uint32_t rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    return (a << 24) | (r << 16) | (g << 8) | b;
}

} // namespace gpsmap

#endif
