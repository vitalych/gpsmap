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

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/imageio.h>

#include <gpsmap/encoder.h>
#include <gpsmap/json.h>
#include <gpsmap/utils.h>

using namespace gpsmap;

// TODO: deduplicate
double g_fps = 25;

struct Arguments {
    std::vector<std::string> InputVideoPaths;
    std::vector<std::string> InputVideoGpxPaths;
    std::string OutputFile;
};

static bool ParseGenerateJsonParams(const std::vector<std::string> &argv, Arguments &args) {
    for (size_t i = 0; i + 1 < argv.size(); i += 2) {
        if (argv[i] == "-vid") {
            args.InputVideoPaths.push_back(argv[i + 1]);
        } else if (argv[i] == "-vid-gpx") {
            args.InputVideoGpxPaths.push_back(argv[i + 1]);
        } else if (argv[i] == "-out") {
            args.OutputFile = argv[i + 1];
        } else {
            std::cerr << "Invalid argument " << i << ": " << argv[i] << "\n";
            return false;
        }
    }

    return args.InputVideoPaths.size() > 0 && args.InputVideoGpxPaths.size() > 0 && args.OutputFile.size() > 0;
}

struct GenFramesArguments {
    std::vector<std::string> InputGPXPaths;
    std::string Segments;
    std::string OutputFile;
};

struct GenTimeCodeArguments {
    std::vector<std::string> InputVideoPaths;
    std::vector<std::string> InputVideoJsonPaths;
    std::string OutputDir;
};

static bool ParseGenerateTimeCodeParams(const std::vector<std::string> &argv, GenTimeCodeArguments &args) {
    for (size_t i = 0; i + 1 < argv.size(); i += 2) {
        if (argv[i] == "-vid") {
            args.InputVideoPaths.push_back(argv[i + 1]);
        } else if (argv[i] == "-vid-json") {
            args.InputVideoJsonPaths.push_back(argv[i + 1]);
        } else if (argv[i] == "-outdir") {
            args.OutputDir = argv[i + 1];
        } else {
            std::cerr << "Invalid argument " << i << ": " << argv[i] << "\n";
            return false;
        }
    }

    return args.InputVideoPaths.size() > 0 && args.InputVideoJsonPaths.size() > 0 && args.OutputDir.size() > 0 &&
           (args.InputVideoPaths.size() == args.InputVideoJsonPaths.size());
}

static bool LoadData(Arguments &args, std::vector<VideoInfo> &videoInfo, std::vector<GPXInfo> &gpxInfo) {

    if (!LoadVideoInfo(args.InputVideoPaths, videoInfo)) {
        return false;
    }

    if (!LoadVideoGpx(args.InputVideoGpxPaths, gpxInfo)) {
        return false;
    }

    if (videoInfo.size() != gpxInfo.size()) {
        std::cerr << "Videos and corresponding gpx files must match\n";
        return false;
    }

    return true;
}

static bool ProcessExtractVideoInfo(const std::vector<std::string> &argv) {

    Arguments args;
    if (!ParseGenerateJsonParams(argv, args)) {
        std::cerr << "Invalid params\n";
        return false;
    }

    std::vector<VideoInfo> videoInfo;
    std::vector<GPXInfo> gpxInfo;
    if (!LoadData(args, videoInfo, gpxInfo)) {
        return false;
    }

    rapidjson::Document Doc;
    Doc.SetObject();

    auto &Allocator = Doc.GetAllocator();
    Doc.AddMember("video_info", SerializeVideoInfos(Doc, videoInfo), Allocator);
    Doc.AddMember("gpx_info", SerializeGPXInfos(Doc, gpxInfo), Allocator);

    if (!WriteDocument(Doc, args.OutputFile)) {
        return false;
    }

    return true;
}

static bool ProcessComputeSegments(const std::vector<std::string> &argv) {
    if (argv.size() != 2) {
        return false;
    }

    const auto &input = argv[0];
    const auto &output = argv[1];

    rapidjson::Document inputDoc;
    if (!ReadDocument(inputDoc, input)) {
        return false;
    }

    std::vector<VideoInfo> videoInfo;
    if (!DeserializeVideoInfos(inputDoc["video_info"], videoInfo)) {
        std::cout << "Could not deserialize video info\n";
        return false;
    }

    std::vector<GPXInfo> gpxInfo;
    if (!DeserializeGPXInfos(inputDoc["gpx_info"], gpxInfo)) {
        std::cout << "Could not deserialize video info\n";
        return false;
    }

    std::vector<VideoInfo> segmentInfo;
    if (!ComputeMapSegmentsForGpxVideos(videoInfo, gpxInfo, segmentInfo)) {
        std::cerr << "Could not compute segments\n";
        return false;
    }

    rapidjson::Document outputDoc;
    outputDoc.SetObject();

    auto &allocator = outputDoc.GetAllocator();
    outputDoc.AddMember("segments", SerializeVideoInfos(outputDoc, segmentInfo), allocator);

    if (!WriteDocument(outputDoc, output)) {
        return false;
    }

    return true;
}

static bool GetGPSInfo(const std::string &jsonInfoFile, VideoInfo &info) {
    rapidjson::Document doc;
    if (!ReadDocument(doc, jsonInfoFile)) {
        return false;
    }

    std::vector<time_t> timestamps;

    auto array = doc.GetArray();
    for (auto it = array.Begin(); it != array.End(); ++it) {
        auto &val = *it;
        if (val.HasMember("timestamp")) {
            timestamps.push_back(val["timestamp"].GetInt());
        } else {
            return false;
        }
    }

    if (timestamps.size() == 0) {
        return false;
    }

    info.Start = *timestamps.begin();
    info.Duration = *timestamps.rbegin() - info.Start;
    return true;
}

static bool ProcessGenerateTimeCodeDescription(const std::vector<std::string> &argv) {
    GenTimeCodeArguments args;
    if (!ParseGenerateTimeCodeParams(argv, args)) {
        std::cerr << "Invalid params\n";
        return false;
    }

    std::vector<VideoInfo> info, segments;

    for (size_t i = 0; i < args.InputVideoPaths.size(); ++i) {
        const auto &video = args.InputVideoPaths[i];
        VideoInfo vi;
        if (!LoadVideoInfo(video, vi)) {
            return false;
        }

        const auto &json = args.InputVideoJsonPaths[i];
        if (!GetGPSInfo(json, vi)) {
            return false;
        }

        info.push_back(vi);
    }

    if (!ComputeMapSegmentsForGpxVideos(info, segments)) {
        return false;
    }

    rapidjson::Document doc;
    doc.SetObject();

    auto &Allocator = doc.GetAllocator();
    doc.AddMember("segments", SerializeVideoInfos(doc, segments), Allocator);

    if (!WriteDocument(doc, args.OutputDir)) {
        return false;
    }

    return true;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " command ...\n";
        return -1;
    }

    std::string cmd = argv[1];
    std::vector<std::string> mainArgs(argv + 2, argv + argc);

    if (!cmd.compare("extract-video-info")) {
        if (!ProcessExtractVideoInfo(mainArgs)) {
            return -1;
        }
    } else if (!cmd.compare("compute-segments")) {
        if (!ProcessComputeSegments(mainArgs)) {
            return -1;
        }
    } else if (!cmd.compare("generate-timecode")) {
        if (!ProcessGenerateTimeCodeDescription(mainArgs)) {
            return -1;
        }
    } else {
        std::cerr << "Unknown command: " << cmd << "\n";
        return -1;
    }

    return 0;
}
