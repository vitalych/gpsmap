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

#include <gps/decoder.h>
#include <iostream>

extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/mathematics.h>
#include <libavutil/opt.h>
#include <libavutil/timestamp.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

VideoDecoder::VideoDecoder(const std::string &filePath) {
    m_filePath = filePath;
    m_fmtctx = nullptr;
}

VideoDecoder::~VideoDecoder() {
    if (m_videoctx) {
        avcodec_free_context(&m_videoctx);
    }
    if (m_fmtctx) {
        avformat_close_input(&m_fmtctx);
    }
}

VideoDecoderPtr VideoDecoder::LoadFromFile(const std::string &filePath) {
    auto decoder = std::shared_ptr<VideoDecoder>(new VideoDecoder(filePath));
    if (!decoder->LoadFile()) {
        return nullptr;
    }

    return decoder;
}

bool VideoDecoder::LoadFile() {
    if (avformat_open_input(&m_fmtctx, m_filePath.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Could not open " << m_filePath << std::endl;
        return false;
    }

    if (avformat_find_stream_info(m_fmtctx, nullptr) < 0) {
        std::cerr << "Could not find stream info in " << m_filePath.c_str() << std::endl;
        return false;
    }

    LoadMetadata();

    auto streamIndex = av_find_best_stream(m_fmtctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (streamIndex < 0) {
        std::cerr << "Could not find video stream in " << m_filePath << std::endl;
        return false;
    }

    auto stream = m_fmtctx->streams[streamIndex];

    auto dec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!dec) {
        std::cerr << "Failed to find video codec" << std::endl;
        return false;
    }

    m_videoctx = avcodec_alloc_context3(dec);
    if (!m_videoctx) {
        std::cerr << "Failed to allocate codec context" << std::endl;
    }

    int ret;
    if ((ret = avcodec_parameters_to_context(m_videoctx, stream->codecpar)) < 0) {
        std::cerr << "Failed to copy codec params to decoder context: " << ret << std::endl;
        return false;
    }

    AVDictionary *opts = nullptr; // XXX: free this?
    if ((ret = avcodec_open2(m_videoctx, dec, &opts)) < 0) {
        std::cerr << "Failed to open codec" << std::endl;
        return ret;
    }

    m_width = m_videoctx->width;
    m_height = m_videoctx->height;
    m_duration = (double) m_fmtctx->duration / (double) AV_TIME_BASE;

    return true;
}

void VideoDecoder::LoadMetadata() {
    AVDictionaryEntry *tag = nullptr;

    while ((tag = av_dict_get(m_fmtctx->metadata, "", tag, AV_DICT_IGNORE_SUFFIX))) {
        m_metadata[tag->key] = tag->value;
    }
}
