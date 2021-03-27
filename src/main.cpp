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
#include <gpsmap/utils.h>

OIIO_NAMESPACE_USING

using namespace gpsmap;

struct Arguments {
    boost::filesystem::path ResourceDir;
    std::string TilesRootPath;
    std::vector<std::string> InputVideoPaths;
    std::vector<std::string> InputVideoGpxPaths;
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
        } else if (!strcmp(argv[i], "-vid")) {
            args.InputVideoPaths.push_back(argv[i + 1]);
        } else if (!strcmp(argv[i], "-vid-gpx")) {
            args.InputVideoGpxPaths.push_back(argv[i + 1]);
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

// These are constant parameters, they don't change
// between tasks and can be accessed concurrently.
struct ResourceBundle {
    TileManagerPtr tiles;
    ResourcesPtr resources;
    boost::filesystem::path OutputDirectory;
    GeoCoords startMarker;
    GeoCoords endMarker;
    gpsmap::GPXSegmentPtr wholeTrack;
    double fps;
};

// Describes a video file to encode.
struct EncodingParams {
    gpsmap::GPXSegmentPtr seg;

    int segmentSequenceId;
    int fileSequenceId;
    int startFrame;
    int frameCount;
};

struct EncodingFrameParams {
    EncodingParams params;
    LabelGeneratorPtr labelGen;
    MapSwitcherPtr mapSwitcher;
    bool failed;
};

std::atomic_uint g_processedFrames(0);
double g_fps = 25;

bool GenerateFrame(VideoEncoder &encoder, OutputStream &os, EncodingFrameParams &state) {
    auto frameIndex = os.next_pts;
    auto actualFrameIndex = state.params.startFrame + frameIndex;

    if (frameIndex >= state.params.frameCount) {
        return false;
    }

    assert(actualFrameIndex < (unsigned) state.params.seg->size());

    ++g_processedFrames;

    ClearFrame(encoder, os);

    int width = encoder.Width();
    int height = encoder.Height();
    auto pixels = os.GetPixels();

    ImageBuf ib(ImageSpec(width, height, 4), pixels);

    const auto &frameDesc = (*state.params.seg)[actualFrameIndex];

    if (!state.mapSwitcher->Generate(ib, frameDesc, frameIndex, encoder.GetFPS())) {
        state.failed = true;
        return false;
    }

    if (!state.labelGen->Generate(ib, frameDesc, frameIndex, encoder.GetFPS())) {
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

static std::vector<std::string> s_filesWithErrors;

static Markers GetMarkers(ResourceBundle &p) {
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

MapSwitcherPtr CreateMapSwitcher(GPXSegmentPtr wholeTrack, ResourceBundle &p, int duration) {
    ZoomedMaps zoomedMaps;
    auto markers = GetMarkers(p);

    // Round robin zoom
    MapImageGenerator::Params p1 = {wholeTrack, p.tiles, p.resources, 5, markers};
    MapImageGenerator::Params p2 = {wholeTrack, p.tiles, p.resources, 7, markers};
    MapImageGenerator::Params p3 = {wholeTrack, p.tiles, p.resources, 11, markers};
    MapImageGenerator::Params p4 = {wholeTrack, p.tiles, p.resources, 16, markers};

    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p1), 5));
    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p2), 5));
    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p3), 5));
    zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p4), 60));

    auto overrideCb = [&](int second, unsigned &index) {
        if (duration < 120) {
            // Don't cycle through short segments.
            // It's easier to do video synchronization with a precise map at all times.
            index = -1;
            return true;
        }

        // show largest zoom level at the beginning or end of a segment, to simplify synchronization
        if ((second > duration - 40) || (second < 20)) {
            index = -1;
            return true;
        }
        return false;
    };

    auto ms = MapSwitcher::Create(overrideCb);
    for (auto m : zoomedMaps) {
        ms->AddMapGenerator(m.first, m.second);
    }

    return ms;
}

static void EncodeOneSegment(int unused, ResourceBundle &r, EncodingParams &p) {
    EncodingFrameParams fs;
    fs.params = p;
    fs.failed = false;
    fs.labelGen = LabelGenerator::Create(r.resources->GetFontPath().string());
    fs.mapSwitcher = CreateMapSwitcher(r.wholeTrack, r, 0);

    assert(fs.params.seg);
    assert((size_t) p.startFrame < fs.params.seg->size());
    auto &first = *(fs.params.seg->begin() + p.startFrame);
    auto t = time_to_str(first.Timestamp);

    std::stringstream videoFileName;
    videoFileName << std::setw(3) << std::setfill('0') << p.fileSequenceId << "-" << std::setw(3) << std::setfill('0')
                  << p.segmentSequenceId << " - " << StripSpecialCharacters(t) << ".mkv";

    boost::filesystem::path videoPath(r.OutputDirectory);
    videoPath.append(videoFileName.str());

    std::cout << "Encoding to " << videoPath << std::endl;

    auto cb = std::bind(&GenerateFrame, std::placeholders::_1, std::placeholders::_2, fs);
    auto encoder = VideoEncoder::Create(videoPath.string(), 512, 512, r.fps, cb);
    if (!encoder) {
        return;
    }

    encoder->EncodeLoop();
    encoder->Finalize();
}

static bool LoadVideoInfo(const std::vector<std::string> &inputVideoPaths, std::vector<VideoInfo> &videoInfo) {
    for (const auto &it : inputVideoPaths) {
        VideoInfo info;
        if (!GetVideoInfo(it, info)) {
            std::cerr << "Could not get video info for " << it << "\n";
            return false;
        }
        std::cout << it << ": fileId=" << info.FileId << " fileSeq=" << info.FileSequence
                  << " frameCount=" << info.FrameCount << " frameRate=" << info.FrameRate
                  << " duration=" << (double) info.FrameCount / info.FrameRate << "\n";
        videoInfo.push_back(info);
    }
    return true;
}

static bool LoadVideoGpx(const std::vector<std::string> &inputVideoGpxPaths) {
    for (const auto &f : inputVideoGpxPaths) {
        std::cout << "Loading " << f << std::endl;
        auto gpx = gpsmap::GPX::Create();
        gpx->LoadFromFile(f, 0);

        for (auto segment : gpx->GetTrackSegments()) {
            GPXInfo info;
            if (!segment->GetInfo(info)) {
                std::cerr << "Could not get info for " << f << "\n";
                return false;
            }

            std::cout << f << ": start=" << time_to_str(info.Start) << " duration=" << info.Duration << "\n";
        }
    }
    return true;
}

static bool LoadSegments(const std::vector<std::string> &inputGPXPaths, GPXSegments &segments, unsigned fps,
                         bool splitIdleParts) {
    double initialDistance = 0.0;

    for (auto f : inputGPXPaths) {
        auto gpx = gpsmap::GPX::Create();
        std::cout << "Loading " << f << std::endl;
        gpx->SetInitialDistance(initialDistance);
        gpx->LoadFromFile(f, fps);

        for (auto seg : *gpx) {
            if (splitIdleParts) {
                GPXSegments idleSegments;
                seg->SplitIdleSegments(idleSegments);
                for (auto idle : idleSegments) {
                    segments.push_back(idle);
                }
            } else {
                segments.push_back(seg);
            }
        }

        initialDistance = gpx->back()->back().TotalDistance;
    }

    return true;
}

static void OneVideoPerSegment(task_set &tasks, thread_pool *tp, ResourceBundle &resources, GPXSegments &segments) {
    int i = 0;
    for (auto segment : segments) {
        EncodingParams p;
        p.fileSequenceId = 0;
        p.segmentSequenceId = i;
        p.startFrame = 0;
        p.frameCount = segment->size();
        p.seg = segment;

        tasks.push(tp->push(EncodeOneSegment, resources, p));
        ++i;
    }
}

static void OneVideoPerSegmentParallel(task_set &tasks, thread_pool *tp, ResourceBundle &resources,
                                       GPXSegments &segments) {
    auto cores = tp->size();

    // Compute number of frames we have
    auto frames = 0;
    for (auto segment : segments) {
        frames += segment->size();
    }

    // Try to keep all cores busy
    size_t avg = frames / cores;

    int i = 0;
    for (auto segment : segments) {
        auto segframes = segment->size();
        auto startframe = 0l;
        auto chunk = 0l;

        while (segframes > 0) {
            auto framecount = avg >= segframes ? segframes : avg;

            EncodingParams p;
            p.fileSequenceId = i;
            p.segmentSequenceId = chunk;
            p.startFrame = startframe;
            p.frameCount = framecount;
            p.seg = segment;
            tasks.push(tp->push(EncodeOneSegment, resources, p));

            startframe += framecount;
            segframes -= framecount;
            ++chunk;
        }

        ++i;
    }
}

static volatile bool s_terminated = false;

static void StatsPrinter() {
    while (!s_terminated) {
        auto totalSeconds = g_processedFrames / (int) g_fps;
        auto seconds = totalSeconds % 60;
        auto minutes = totalSeconds / 60;
        std::cout << g_processedFrames << " frames - " << std::setfill('0') << std::setw(2) << minutes << ":"
                  << std::setfill('0') << std::setw(2) << seconds << "\n";
        sleep(1);
    }
    std::cout << "Stats thread terminated\n";
}

int main(int argc, char **argv) {
    Arguments args;

    if (!ParseCommandLine(argc, argv, args)) {
        printf("usage: %s -gpx file1.gpx [-gpx file2.gpx...] -tiles /path/to/tiles/dir -rsrcdir /path/to/resources "
               "-outdir /path/to/out/dir\n",
               argv[0]);
        return -1;
    }

    std::vector<VideoInfo> videoInfo;
    if (!LoadVideoInfo(args.InputVideoPaths, videoInfo)) {
        return -1;
    }

    std::vector<GPX> videoGpx;
    if (!LoadVideoGpx(args.InputVideoGpxPaths)) {
        return -1;
    }

    auto resources = Resources::Create(args.ResourceDir);
    if (!resources) {
        return -1;
    }

    auto tiles = TileManager::Create(args.TilesRootPath, resources->GetMapPath().string());
    if (!tiles) {
        std::cerr << "Could not create tile manager" << std::endl;
        return -1;
    }

    std::sort(args.InputGPXPaths.begin(), args.InputGPXPaths.end());
    GPXSegments segments;
    if (!LoadSegments(args.InputGPXPaths, segments, g_fps, true)) {
        std::cerr << "Could not load segments\n";
        return -1;
    }

    GPXSegmentPtr wholeTrack = MergeSegments(segments);
    auto firstItem = wholeTrack->front();
    auto lastItem = wholeTrack->back();

    ResourceBundle resourcesBundle;
    resourcesBundle.wholeTrack = wholeTrack;
    resourcesBundle.resources = resources;
    resourcesBundle.tiles = tiles;
    resourcesBundle.OutputDirectory = args.OutputDirectory;
    resourcesBundle.startMarker.Latitude = firstItem.Latitude;
    resourcesBundle.startMarker.Longitude = firstItem.Longitude;
    resourcesBundle.endMarker.Latitude = lastItem.Latitude;
    resourcesBundle.endMarker.Longitude = lastItem.Longitude;
    resourcesBundle.fps = g_fps;

    // Compute which zoomed map to show for each frame
    for (auto segment : segments) {
        auto &first = segment->front();
        auto &last = segment->back();
        auto duration = last.Timestamp - first.Timestamp;
        auto ms = CreateMapSwitcher(wholeTrack, resourcesBundle, duration);
        auto i = 0;
        for (TrackItem &item : *segment) {
            ms->ComputeState(item, i, g_fps);
            ++i;
        }
    }

    std::thread thr(StatsPrinter);
    auto tp = default_thread_pool();

    task_set tasks(tp);

    OneVideoPerSegmentParallel(tasks, tp, resourcesBundle, segments);

    tasks.wait(true);
    s_terminated = true;
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
