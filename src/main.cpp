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

#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/imageio.h>

#include <gps/encoder.h>
#include <gps/gpx.h>
#include <gps/mapgen.h>
#include <gps/tilemanager.h>

OIIO_NAMESPACE_USING

struct Arguments {
    std::string TilesRootPath;
    std::string TilesUrl;
    std::string InputVideoPath;
    boost::filesystem::path OutputDirectory;
    std::vector<std::string> InputGPXPaths;
};

static bool ParseCommandLine(int argc, char **argv, Arguments &args) {
    for (auto i = 1; i + 1 < argc; i += 2) {
        if (!strcmp(argv[i], "-gpx")) {
            args.InputGPXPaths.push_back(argv[i + 1]);
        } else if (!strcmp(argv[i], "-outdir")) {
            args.OutputDirectory = boost::filesystem::path(argv[i + 1]);
            if (!boost::filesystem::exists(args.OutputDirectory)) {
                std::cerr << args.OutputDirectory << " does not exist" << std::endl;
                return false;
            }
        } else if (!strcmp(argv[i], "-tiles")) {
            args.TilesRootPath = argv[i + 1];
        } else if (!strcmp(argv[i], "-tiles-url")) {
            args.TilesUrl = argv[i + 1];
        } else {
            std::cerr << "Invalid argument " << i << ": " << argv[i] << "\n";
            return false;
        }
    }

    return args.InputGPXPaths.size() > 0 && args.OutputDirectory.size() > 0 && args.InputGPXPaths.size() > 0;
}

static inline uint32_t rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    return (a << 24) | (r << 16) | (g << 8) | b;
}

// TODO: use imagebuf for that
void ClearFrame(VideoEncoder &encoder, OutputStream &os) {
    int width = encoder.Width();
    int height = encoder.Height();

    auto pict = os.tmp_frame;
    int ls = pict->linesize[0] / sizeof(uint32_t);
    uint32_t *pixels = (uint32_t *) pict->data[0];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            pixels[y * ls + x] = rgba(0, 0, 0, 0xff);
        }
    }
}

struct FrameState {
    LabelGeneratorPtr labelGen;
    GeoTrackerPtr geoTracker;
    MapSwitcherPtr mapSwitcher;
};

bool GenerateFrame(VideoEncoder &encoder, OutputStream &os, FrameState &state) {
    auto frame_index = os.next_pts;
    auto second = (double) frame_index / (double) encoder.GetFPS();

    if ((frame_index % encoder.GetFPS()) == 0) {
        std::cout << "Frame " << frame_index << " " << second << "s" << std::endl;
    }

    ClearFrame(encoder, os);

    int width = encoder.Width();
    int height = encoder.Height();
    auto pixels = os.GetPixels();

    ImageBuf ib(ImageSpec(width, height, 4), pixels);

    if (!state.geoTracker->UpdateFrame(frame_index, encoder.GetFPS())) {
        return false;
    }

    if (!state.mapSwitcher->Generate(second, ib)) {
        return false;
    }

    if (!state.labelGen->Generate(ib)) {
        return false;
    }

    return true;
}

static std::string StripSpecialCharacters(const std::string &str) {
    std::stringstream ss;
    for (auto c : str) {
        if (c == ':') {
            c = '-';
        }
        ss << c;
    }
    return ss.str();
}

int main(int argc, char **argv) {
    Arguments args;

    if (!ParseCommandLine(argc, argv, args)) {
        printf("usage: %s -gpx file1.gpx [-gpx file2.gpx...] -vout output.mp4\n", argv[0]);
        return -1;
    }

    auto tiles = TileManager::Create(args.TilesRootPath, args.TilesUrl);
    if (!tiles) {
        std::cerr << "Could not create tile manager" << std::endl;
        exit(-1);
    }

    auto dot = OIIO::ImageBuf("dot.png");
    OIIO::ROI roi(0, 32, 0, 32, 0, 1, /*chans:*/ 0, dot.nchannels());
    dot = OIIO::ImageBufAlgo::resize(dot, "", 0, roi);

    std::vector<std::pair<MapImageGeneratorPtr, int>> zoomedMaps;

    // Round robin zoom
    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(tiles, dot, 5), 5));
    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(tiles, dot, 7), 5));
    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(tiles, dot, 11), 5));
    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(tiles, dot, 16), 60));

    for (auto f : args.InputGPXPaths) {
        auto gpx = gpx::GPX::Create();
        std::cout << "Loading " << f << std::endl;
        gpx->LoadFromFile(f);
        gpx->CreateSegments();

        int i = 0;

        for (const auto &seg : gpx->GetSegments()) {
            gpx::TrackItem start, end;
            gpx->GetItem(seg.first, start);
            gpx->GetItem(seg.second, end);
            std::cout << "Segment start=" << seg.first << " end=" << seg.second << std::endl;
            std::cout << "  " << start << "\n";
            std::cout << "  " << end << "\n";

            std::stringstream videoFileName;
            boost::filesystem::path videoPath(args.OutputDirectory);

            videoFileName << std::setw(3) << std::setfill('0') << i << " - "
                          << StripSpecialCharacters(start.OriginalTimestamp) << ".mp4";
            ++i;
            videoPath.append(videoFileName.str());

            std::cout << "Encoding to " << videoPath << std::endl;

            FrameState fs;
            fs.geoTracker = GeoTracker::Create(gpx, seg.first, seg.second);
            fs.labelGen = LabelGenerator::Create(fs.geoTracker);
            fs.mapSwitcher = MapSwitcher::Create(fs.geoTracker);
            for (auto m : zoomedMaps) {
                fs.mapSwitcher->AddMapGenerator(m.first, m.second);
            }

            auto cb = std::bind(&GenerateFrame, std::placeholders::_1, std::placeholders::_2, fs);
            auto encoder = VideoEncoder::Create(videoPath.string(), 512, 512, 25, cb);
            if (!encoder) {
                return -1;
            }

            encoder->EncodeLoop();
            encoder->Finalize();
        }
    }

    return 0;
}
