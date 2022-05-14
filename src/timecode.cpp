// Copyright (c) 2021 Vitaly Chipounov
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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string.h>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/imageio.h>

#include <gpsmap/encoder.h>
#include <gpsmap/json.h>
#include <gpsmap/utils.h>

using namespace gpsmap;
OIIO_NAMESPACE_USING

static const int LABEL_HEIGHT = 32;
static const int LABEL_WIDTH = 512;

struct FrameState {
    std::string fontPath;
    VideoInfo vi;
    time_t timestamp = 0;
    std::string label;
    OIIO::ImageBuf buf = OIIO::ImageBuf(OIIO::ImageSpec(LABEL_WIDTH, LABEL_HEIGHT, 4));
};

static bool GenerateTimeCodeFrame(VideoEncoder &encoder, OutputStream &os, FrameState &state) {
    auto frameIndex = os.next_pts;
    if (frameIndex >= state.vi.FrameCount) {
        return false;
    }

    encoder.ClearFrame(os);

    int width = encoder.Width();
    int height = encoder.Height();
    auto pixels = os.GetPixels();

    ImageBuf ib(ImageSpec(width, height, 4), pixels);

    auto fr = av_q2d(state.vi.FrameRate);
    time_t ts = state.vi.Start + (time_t) ((double) frameIndex / fr);
    if (ts != state.timestamp) {
        state.label = time_to_str(ts);
        state.timestamp = ts;

        if (!ImageBufAlgo::render_box(state.buf, 0, 0, width, LABEL_HEIGHT, 1.0f, true, {}, 1)) {
            std::cerr << "Could not render box" << std::endl;
            return false;
        }

        if (!ImageBufAlgo::render_text(state.buf, width / 2, LABEL_HEIGHT - 5, state.label.c_str(), LABEL_HEIGHT,
                                       state.fontPath, {0.0f, 0.0f, 0.0f, 1.0f}, ImageBufAlgo::TextAlignX::Center,
                                       ImageBufAlgo::TextAlignY::Baseline, 0, {}, 1)) {
            fprintf(stderr, "Could not render text\n");
            return false;
        }
    }

    if (!ImageBufAlgo::paste(ib, 0, 0, 0, 0, state.buf, {}, 1)) {
        std::cerr << "Could not paste" << std::endl;
        return false;
    }

    return true;
}

static bool ProcessGenerateTimeCodeVideos(const std::vector<std::string> &argv) {
    if (argv.size() != 2) {
        std::cerr << "Invalid number of args: " << argv.size() << "\n";
        return false;
    }

    const std::string &descFile = argv[0];
    boost::filesystem::path outDir(argv[1]);

    std::vector<VideoInfo> videoInfo;

    if (!DeserializeVideoInfos(descFile, videoInfo)) {
        std::cerr << "Could not deserialize segments info\n";
        return false;
    }

    auto tp = default_thread_pool();
    task_set tasks(tp);

    for (const auto &vi : videoInfo) {
        auto taskFunc = [&](int unused, const VideoInfo &vi) {
            boost::filesystem::path path(vi.Path);
            auto fileName = path.filename().string();

            boost::filesystem::path videoPath(outDir);
            videoPath.append(fileName + ".TC.MOV");

            FrameState fs = {.fontPath = "/home/vitaly/perso/gps/rsrc/LiberationSans-Regular.ttf", .vi = vi};

            auto cb = std::bind(&GenerateTimeCodeFrame, std::placeholders::_1, std::placeholders::_2, fs);
            auto encoder = VideoEncoder::Create(videoPath.string(), LABEL_WIDTH, LABEL_HEIGHT, vi.FrameRate, cb);
            if (!encoder) {
                return;
            }

            encoder->EncodeLoop();
            encoder->Finalize();
        };

        tasks.push(tp->push(taskFunc, vi));
    }

    tasks.wait(true);

    return true;
}

int main(int argc, char **argv) {
    std::vector<std::string> mainArgs(argv + 1, argv + argc);

    if (!ProcessGenerateTimeCodeVideos(mainArgs)) {
        return -1;
    }

    return 0;
}