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

#include <atomic>
#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/imageio.h>

#include <gpsmap/encoder.h>
#include <gpsmap/gpx.h>
#include <gpsmap/mapgen.h>
#include <gpsmap/resources.h>
#include <gpsmap/tilemanager.h>

OIIO_NAMESPACE_USING

using namespace gpsmap;

struct Arguments {
    boost::filesystem::path ResourceDir;
    std::string TilesRootPath;
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
        } else if (!strcmp(argv[i], "-rsrcdir")) {
            args.ResourceDir = boost::filesystem::path(argv[i + 1]);
            if (!boost::filesystem::exists(args.ResourceDir)) {
                std::cerr << args.ResourceDir << " does not exist" << std::endl;
                return false;
            }
        } else if (!strcmp(argv[i], "-tiles")) {
            args.TilesRootPath = argv[i + 1];
        } else {
            std::cerr << "Invalid argument " << i << ": " << argv[i] << "\n";
            return false;
        }
    }

    return args.InputGPXPaths.size() > 0 && args.OutputDirectory.size() > 0 && args.InputGPXPaths.size() > 0 &&
           args.ResourceDir.size() > 0;
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

struct GeoCoords {
    double Latitude;
    double Longitude;
};

struct EncodingParams {
    TileManagerPtr tiles;
    ResourcesPtr resources;
    gpsmap::GPXPtr gpx;
    gpsmap::Segment seg;
    gpsmap::GPXPtr wholeTrack;
    int segmentSequenceId;
    int fileSequenceId;
    GeoCoords startMarker;
    GeoCoords endMarker;
    boost::filesystem::path OutputDirectory;
};

struct FrameState {
    LabelGeneratorPtr labelGen;
    GeoTrackerPtr geoTracker;
    MapSwitcherPtr mapSwitcher;
    bool failed;
    const EncodingParams *params;
};

std::atomic_uint g_processedFrames(0);

bool GenerateFrame(VideoEncoder &encoder, OutputStream &os, FrameState &state) {
    auto frame_index = os.next_pts;

    ++g_processedFrames;

    ClearFrame(encoder, os);

    int width = encoder.Width();
    int height = encoder.Height();
    auto pixels = os.GetPixels();

    ImageBuf ib(ImageSpec(width, height, 4), pixels);

    if (!state.geoTracker->Generate(ib, frame_index, encoder.GetFPS())) {
        return false;
    }

    if (!state.mapSwitcher->Generate(ib, frame_index, encoder.GetFPS())) {
        state.failed = true;
        return false;
    }

    if (!state.labelGen->Generate(ib, frame_index, encoder.GetFPS())) {
        state.failed = true;
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

using ZoomedMaps = std::vector<std::pair<MapImageGeneratorPtr, int>>;

static std::vector<std::string> s_filesWithErrors;

static Markers GetMarkers(EncodingParams &p) {
    Markers markers;

    // Start marker
    {
        Marker m;
        m.Image = &p.resources->Start();
        m.Latitude = p.startMarker.Latitude;
        m.Longitude = p.startMarker.Longitude;

        auto mw = m.Image->spec().width;
        auto mh = m.Image->spec().height;

        m.x = mw / 2;
        m.y = mh;
        markers.push_back(m);
    }

    // End marker
    {
        Marker m;
        m.Image = &p.resources->Finish();
        m.Latitude = p.endMarker.Latitude;
        m.Longitude = p.endMarker.Longitude;

        auto mw = m.Image->spec().width;

        m.x = mw / 2;
        m.y = 0;
        markers.push_back(m);
    }

    return markers;
}

static void EncodeOneSegment(int unused, EncodingParams &p) {

    ZoomedMaps zoomedMaps;

    gpsmap::TrackItem start, end;
    p.gpx->GetItem(p.seg.first, start);
    p.gpx->GetItem(p.seg.second, end);
    std::cout << "Segment start=" << p.seg.first << " end=" << p.seg.second << std::endl;
    std::cout << "  " << start << "\n";
    std::cout << "  " << end << "\n";

    auto duration = end.Timestamp - start.Timestamp;
    auto largestZoomLevelIndex = 0;

    auto overrideCb = [&](int second, int &index) {
        // show largest zoom level at the beginning or end of a segment, to simplify synchronization
        if ((second > duration - 40) || (second < 20)) {
            index = largestZoomLevelIndex;
            return true;
        }
        return false;
    };

    FrameState fs;
    fs.params = &p;
    fs.failed = false;
    fs.geoTracker = GeoTracker::Create(p.gpx, p.seg.first, p.seg.second);
    fs.labelGen = LabelGenerator::Create(fs.geoTracker, p.resources->GetFontPath().string());

    auto markers = GetMarkers(p);

    // Round robin zoom
    if (duration > 120) {
        // Don't cycle through short segments.
        // It's easier to do video synchronization with a precise map at all times.
        MapImageGenerator::Params p1 = {p.wholeTrack, fs.geoTracker, p.tiles, p.resources, 5, markers};
        MapImageGenerator::Params p2 = {p.wholeTrack, fs.geoTracker, p.tiles, p.resources, 7, markers};
        MapImageGenerator::Params p3 = {p.wholeTrack, fs.geoTracker, p.tiles, p.resources, 11, markers};

        zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p1), 5));
        zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p2), 5));
        zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p3), 5));
    }

    MapImageGenerator::Params p4 = {p.wholeTrack, fs.geoTracker, p.tiles, p.resources, 16, markers};
    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p4), 60));
    largestZoomLevelIndex = zoomedMaps.size() - 1;

    std::stringstream videoFileName;
    boost::filesystem::path videoPath(p.OutputDirectory);

    videoFileName << std::setw(3) << std::setfill('0') << p.fileSequenceId << "-" << std::setw(3) << std::setfill('0')
                  << p.segmentSequenceId << " - " << StripSpecialCharacters(start.OriginalTimestamp) << ".mp4";

    videoPath.append(videoFileName.str());

    std::cout << "Encoding to " << videoPath << std::endl;

    fs.mapSwitcher = MapSwitcher::Create(fs.geoTracker, overrideCb);
    for (auto m : zoomedMaps) {
        fs.mapSwitcher->AddMapGenerator(m.first, m.second);
    }

    auto cb = std::bind(&GenerateFrame, std::placeholders::_1, std::placeholders::_2, fs);
    auto encoder = VideoEncoder::Create(videoPath.string(), 512, 512, 25, cb);
    if (!encoder) {
        return;
    }

    encoder->EncodeLoop();
    encoder->Finalize();

    if (fs.failed) {
        std::cerr << "Error while generating map\n";
        s_filesWithErrors.push_back(videoPath.string());
    }
}

int main(int argc, char **argv) {
    Arguments args;

    if (!ParseCommandLine(argc, argv, args)) {
        printf("usage: %s -gpx file1.gpx [-gpx file2.gpx...] -tiles /path/to/tiles/dir -rsrcdir /path/to/resources "
               "-outdir /path/to/out/dir\n",
               argv[0]);
        return -1;
    }

    auto resources = Resources::Create(args.ResourceDir);
    if (!resources) {
        return -1;
    }

    auto tiles = TileManager::Create(args.TilesRootPath, resources->GetMapPath().string());
    if (!tiles) {
        std::cerr << "Could not create tile manager" << std::endl;
        exit(-1);
    }

    volatile bool terminated = false;
    auto printStats = [&] {
        while (!terminated) {
            auto totalSeconds = g_processedFrames / 25;
            auto seconds = totalSeconds % 60;
            auto minutes = totalSeconds / 60;
            std::cout << g_processedFrames << " frames - " << std::setfill('0') << std::setw(2) << minutes << ":"
                      << std::setfill('0') << std::setw(2) << seconds << "\n";
            sleep(1);
        }
        std::cout << "Stats thread terminated\n";
    };
    std::thread thr(printStats);
    auto tp = default_thread_pool();

    task_set tasks(tp);
    {
        std::sort(args.InputGPXPaths.begin(), args.InputGPXPaths.end());

        double initialDistance = 0.0;
        std::vector<GPXPtr> gpxs;
        auto wholeTrack = gpsmap::GPX::Create();

        for (auto f : args.InputGPXPaths) {
            auto gpx = gpsmap::GPX::Create();
            std::cout << "Loading " << f << std::endl;
            gpx->SetInitialDistance(initialDistance);
            gpx->LoadFromFile(f);
            gpx->CreateSegments();

            wholeTrack->LoadFromFile(f);
            initialDistance = gpx->Last().TotalDistance;
            gpxs.push_back(gpx);
        }

        wholeTrack->CreateSegments();

        auto firstItem = (*gpxs.begin())->First();
        auto lastItem = (*gpxs.rbegin())->Last();

        int j = 0;
        for (auto gpx : gpxs) {
            int i = 0;
            for (const auto &seg : gpx->GetSegments()) {
                EncodingParams p;
                p.wholeTrack = wholeTrack;
                p.gpx = gpx;
                p.seg = seg;
                p.tiles = tiles;
                p.resources = resources;
                p.fileSequenceId = j;
                p.segmentSequenceId = i;
                p.OutputDirectory = args.OutputDirectory;
                p.startMarker.Latitude = firstItem.Latitude;
                p.startMarker.Longitude = firstItem.Longitude;
                p.endMarker.Latitude = lastItem.Latitude;
                p.endMarker.Longitude = lastItem.Longitude;
                tasks.push(tp->push(EncodeOneSegment, p));
                ++i;
            }
            initialDistance = gpx->Last().TotalDistance;
            ++j;
        }
    }
    tasks.wait(true);
    terminated = true;
    thr.join();

    if (s_filesWithErrors.size() > 0) {
        std::cerr << "Encoding failed for these files:\n";
        for (const auto &f : s_filesWithErrors) {
            std::cerr << f << "\n";
        }
        return -1;
    }

    return 0;
}
