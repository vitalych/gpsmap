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
#include <fstream>
#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/imageio.h>

#include <gpsmap/encoder.h>
#include <gpsmap/gpx.h>
#include <gpsmap/json.h>
#include <gpsmap/map.h>
#include <gpsmap/resources.h>
#include <gpsmap/tilemanager.h>
#include <gpsmap/utils.h>

OIIO_NAMESPACE_USING

using namespace gpsmap;

struct Arguments {
    boost::filesystem::path ResourceDir;
    std::string TilesRootPath;
    std::string VideoSegmentsPath;
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
        } else if (!strcmp(argv[i], "-vid-segments")) {
            args.VideoSegmentsPath = argv[i + 1];
        } else {
            std::cerr << "Invalid argument " << i << ": " << argv[i] << "\n";
            return false;
        }
    }

    return args.InputGPXPaths.size() > 0 && args.OutputDirectory.size() > 0 && args.ResourceDir.size() > 0;
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
    AVRational fps;
};

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

struct EncodingParams {
    TrackItems frames;

    int segmentSequenceId;
    int fileSequenceId;

    std::string getFileName() const {
        assert(frames.size() > 0);
        auto &first = frames[0];
        auto t = time_to_str(first.Timestamp);

        std::stringstream videoFileName;
        videoFileName << std::setw(3) << std::setfill('0') << fileSequenceId << "-" << std::setw(3) << std::setfill('0')
                      << segmentSequenceId << " - " << StripSpecialCharacters(t) << ".mp4";

        return videoFileName.str();
    }
};

struct EncodingFrameParams {
    EncodingParams params;
    LabelGeneratorPtr labelGen;
    MapSwitcherPtr mapSwitcher;
    bool failed;
};

std::atomic_uint g_processedFrames(0);
AVRational g_fps = {60000, 1001};

bool GenerateFrame(VideoEncoder &encoder, OutputStream &os, EncodingFrameParams &state) {
    auto frameIndex = os.next_pts;

    if ((size_t) frameIndex >= state.params.frames.size()) {
        return false;
    }

    ++g_processedFrames;

    encoder.ClearFrame(os, rgba(0, 0, 0, 0xff));

    int width = encoder.Width();
    int height = encoder.Height();
    auto pixels = os.GetPixels();

    ImageBuf ib(ImageSpec(width, height, 4), pixels);

    const auto &frameDesc = state.params.frames[frameIndex];
    if (!frameDesc.Valid) {
        return true;
    }

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

struct ZoomInfo {
    int Level;
    int Duration;
    ZoomInfo(int level, int duration):Level(level), Duration(duration) {}
};

std::vector<ZoomInfo> GetZoomLevels() {
    std::vector<ZoomInfo> ret;
    ret.push_back(ZoomInfo(5, 5));
    ret.push_back(ZoomInfo(7, 5));
    ret.push_back(ZoomInfo(11, 5));
    ret.push_back(ZoomInfo(16, 60));
    return ret;
}

MapSwitcherPtr CreateMapSwitcher(GPXSegmentPtr wholeTrack, ResourceBundle &p, int duration) {
    ZoomedMaps zoomedMaps;
    auto markers = GetMarkers(p);
    auto zoomLevels = GetZoomLevels();

    // Round robin zoom
    for (auto zoom : zoomLevels) {
        MapImageGenerator::Params p1 = {wholeTrack, p.tiles, p.resources, zoom.Level, markers};
        zoomedMaps.push_back(std::make_pair(MapImageGenerator::Create(p1), zoom.Duration));
    }

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

    assert(fs.params.frames.size() > 0);
    auto &first = fs.params.frames[0];
    auto t = time_to_str(first.Timestamp);

    boost::filesystem::path videoPath(r.OutputDirectory);
    videoPath.append(p.getFileName());

    std::cout << "Encoding to " << videoPath << std::endl;

    auto cb = std::bind(&GenerateFrame, std::placeholders::_1, std::placeholders::_2, fs);
    auto encoder = VideoEncoder::Create(videoPath.string(), 512, 512, r.fps, cb);
    if (!encoder) {
        return;
    }

    encoder->EncodeLoop();
    encoder->Finalize();
}

static bool MatchExternalGPXWithEmbeddedVideoTimestamps(const Arguments &args, task_set &tasks, thread_pool *tp,
                                                        ResourceBundle &resources, GPXSegments &segments) {

    std::vector<VideoInfo> videoInfo;

    if (!DeserializeVideoInfos(args.VideoSegmentsPath, videoInfo)) {
        std::cerr << "Could not deserialize segments info\n";
        return false;
    }

    std::vector<GPXInfo> gpxInfo;
    for (const auto &vi : videoInfo) {
        GPXInfo info;
        info.Start = vi.Start;
        info.Duration = (double) vi.FrameCount / ((double) vi.FrameRate.num / (double) vi.FrameRate.den);
        gpxInfo.push_back(info);
    }

    for (auto seg : segments) {
        seg->FillSegment(gpxInfo);
    }

    auto fps = av_q2d(g_fps);
    // Maximum video size is 5 minutes
    unsigned framesPerChunk = fps * 60.0 * 5;

    // Now we have start date and number of frames in original video.
    // Match the ranges from the extern gpx data.
    // It's possible to have gaps: no external data for (part of) a given range.
    auto i = 0;
    for (const auto &vi : videoInfo) {
        TrackItems trackItems;
        if (!GetSegmentRange(segments, trackItems, vi.Start, vi.Duration, vi.FrameCount, fps)) {
            std::cerr << "Could not find matching external gpx data for file id " << vi.FileId << "\n";
            continue;
        }

        assert(trackItems.size() == vi.FrameCount);

        unsigned frameCount = vi.FrameCount;
        unsigned j = 0;
        unsigned startFrame = 0;
        while (frameCount > 0) {
            unsigned framesInChunk = frameCount > framesPerChunk ? framesPerChunk : frameCount;

            EncodingParams p;
            p.fileSequenceId = vi.FileId;
            p.segmentSequenceId = j;

            p.frames.resize(framesInChunk);
            for (auto i = 0u; i < framesInChunk; ++i) {
                p.frames[i] = trackItems[i + startFrame];
            }

            tasks.push(tp->push(EncodeOneSegment, resources, p));
            frameCount -= framesInChunk;
            startFrame += framesInChunk;

            ++j;
        }
        ++i;
    }

    return true;
}

[[maybe_unused]] static void OneVideoPerSegment(task_set &tasks, thread_pool *tp, ResourceBundle &resources,
                                                GPXSegments &segments) {
    int i = 0;
    for (auto segment : segments) {
        EncodingParams p;
        p.fileSequenceId = 0;
        p.segmentSequenceId = i;
        p.frames.resize(segment->size());
        std::copy(segment->begin(), segment->end(), p.frames.begin());

        tasks.push(tp->push(EncodeOneSegment, resources, p));
        ++i;
    }
}

static volatile bool s_terminated = false;
static bool s_print_stats = true;

static void StatsPrinter() {
    while (!s_terminated) {
        if (s_print_stats) {
            auto fps = av_q2d(g_fps);
            auto totalSeconds = (int) ((double) g_processedFrames / fps);
            auto seconds = totalSeconds % 60;
            auto minutes = totalSeconds / 60;
            std::cout << g_processedFrames << " frames - " << std::setfill('0') << std::setw(2) << minutes << ":"
                      << std::setfill('0') << std::setw(2) << seconds << "\n";
        }
        sleep(1);
    }
    std::cout << "Stats thread terminated\n";
}

bool LoadTiles(TileManager &mgr, const GPXSegments &segments) {
    auto zoomLevels = GetZoomLevels();

    for (const auto seg:segments) {
        for (const auto &ti:*seg) {
            for (const auto &zoom : zoomLevels) {

                int x = 0, y = 0, px = 0, py = 0;
                mgr.GetTileCoords(ti.Latitude, ti.Longitude, zoom.Level, x, y, px, py);

                // Download all tiles around the center.
                for (int tx = x - 1; tx <= x + 1; tx++) {
                    for (int ty = y - 1; ty <= y + 1; ty++) {
                        if (tx < 0 || ty < 0) {
                            continue;
                        }

                        auto tile = mgr.GetTile(tx, ty, zoom.Level);
                        if (tile) {
                            continue;
                        }

                        bool success = false;
                        for (int tries = 0; tries < 3; ++tries) {
                            std::cout << "Loading tile x=" << tx << " y=" << ty << " z=" << zoom.Level << "\n";
                            tile = mgr.DownloadTile(tx, ty, zoom.Level);
                            if (!tile) {
                                std::cerr << "Could not load tile";
                                continue;
                            }
                            success = true;
                            break;
                        }

                        if (!success) {
                            return false;
                        }
                    }
                }
            }
        }
    }

    return true;
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
        return -1;
    }

    std::sort(args.InputGPXPaths.begin(), args.InputGPXPaths.end());
    GPXSegments segments;
    auto fps = av_q2d(g_fps);
    if (!LoadSegments(args.InputGPXPaths, segments, fps, false)) {
        std::cerr << "Could not load segments\n";
        return -1;
    }

    if (!LoadTiles(*tiles, segments)) {
        std::cerr << "Could not load tiles\n";
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
            ms->ComputeState(item, i, fps);
            ++i;
        }
    }

    std::thread thr(StatsPrinter);
    auto tp = default_thread_pool();

    task_set tasks(tp);

    // OneVideoPerSegment(tasks, tp, resourcesBundle, segments);
    MatchExternalGPXWithEmbeddedVideoTimestamps(args, tasks, tp, resourcesBundle, segments);

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
