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

#include <array>
#include <assert.h>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

#include <gpsmap/gpx.h>
#include <gpsmap/utils.h>

namespace fs = std::filesystem;

static bool operator!=(const AVRational &a, const AVRational &b) {
    return !(a.num == b.num && a.den == b.den);
}

namespace gpsmap {

std::string exec(const char *cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);

    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    return result;
}

bool GetVideoInfo(const std::string &filePath, VideoInfo &info) {
    info.Path = filePath;

    std::stringstream ss;
    ss << "ffprobe -v error -select_streams v:0 -show_entries stream=nb_frames,r_frame_rate -show_entries "
          "stream_tags=creation_time -of "
          "default=nokey=1:noprint_wrappers=1 ";
    ss << "\"" << filePath << "\"";
    auto cmd = ss.str();
    auto result = exec(cmd.c_str());

    // First line is frame rate
    std::istringstream iss(result);
    std::string frameRate;
    std::string creationTime;
    iss >> frameRate;
    iss >> info.FrameCount;
    iss >> creationTime;

    auto fileName = fs::path(filePath).stem().string();
    sscanf(fileName.c_str(), "GX-%d-%d", &info.FileId, &info.FileSequence);

    int n = 0, d = 0;
    sscanf(frameRate.c_str(), "%d/%d", &n, &d);

    if (!n || !d) {
        throw std::runtime_error("Invalid frame rate");
    }

    info.FrameRate = {n, d};

    return true;
}

bool LoadVideoInfo(const std::string &inputVideoPath, VideoInfo &info) {
    if (!GetVideoInfo(inputVideoPath, info)) {
        std::cerr << "Could not get video info for " << inputVideoPath << "\n";
        return false;
    }

    auto fr = av_q2d(info.FrameRate);

    std::cout << inputVideoPath << ": fileId=" << info.FileId << " fileSeq=" << info.FileSequence
              << " frameCount=" << info.FrameCount << " frameRate=" << fr
              << " duration=" << (double) info.FrameCount / fr << "\n";
    return true;
}

bool LoadVideoInfo(const std::vector<std::string> &inputVideoPaths, std::vector<VideoInfo> &videoInfo) {
    for (const auto &it : inputVideoPaths) {
        VideoInfo info;
        if (!LoadVideoInfo(it, info)) {
            return false;
        }
        videoInfo.push_back(info);
    }
    return true;
}

bool LoadVideoGpx(const std::vector<std::string> &inputVideoGpxPaths, std::vector<GPXInfo> &gpxInfo) {
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
            gpxInfo.push_back(info);
        }
    }
    return true;
}

bool ComputeMapSegmentsForGpxVideos(const std::vector<VideoInfo> &videoInfo, std::vector<VideoInfo> &segments) {
    size_t i = 0;

    int fileId = -1;

    while (i < videoInfo.size()) {
        auto i0 = videoInfo[i];
        assert(i0.FileId > fileId);

        size_t j = i + 1;
        auto seq = i0.FileSequence;
        while (j < videoInfo.size()) {
            const auto &i1 = videoInfo[j];
            if (i1.FileId != i0.FileId) {
                break;
            }

            if (i1.FileSequence != seq + 1) {
                std::cerr << "Invalid sequence id\n";
                return false;
            }

            if (i1.FrameRate != i0.FrameRate) {
                std::cerr << "All videos must have identical frame rate\n";
                return false;
            }

            i0.FrameCount += i1.FrameCount;
            assert(i0.Start < videoInfo[j].Start);
            i0.Duration = videoInfo[j].Start - i0.Start + videoInfo[j].Duration;
            seq = i1.FileSequence;
            fileId = i1.FileId;
            ++j;
        }

        segments.push_back(i0);
        i = j;
    }

    return true;
}

bool ComputeMapSegmentsForGpxVideos(const std::vector<VideoInfo> &videoInfo, const std::vector<GPXInfo> &gpxInfo,
                                    std::vector<VideoInfo> &segments) {
    size_t i = 0;

    if (gpxInfo.size() != videoInfo.size()) {
        return false;
    }

    int fileId = -1;

    while (i < videoInfo.size()) {
        auto i0 = videoInfo[i];
        assert(i0.FileId > fileId);

        i0.Start = gpxInfo[i].Start;
        i0.Duration = gpxInfo[i].Duration;

        size_t j = i + 1;
        auto seq = i0.FileSequence;
        while (j < videoInfo.size()) {
            const auto &i1 = videoInfo[j];
            if (i1.FileId != i0.FileId) {
                break;
            }

            if (i1.FileSequence != seq + 1) {
                std::cerr << "Invalid sequence id\n";
                return false;
            }

            if (i1.FrameRate != i0.FrameRate) {
                std::cerr << "All videos must have identical frame rate\n";
                return false;
            }

            i0.FrameCount += i1.FrameCount;
            assert(i0.Start < gpxInfo[j].Start);
            i0.Duration = gpxInfo[j].Start - i0.Start + gpxInfo[j].Duration;
            seq = i1.FileSequence;
            fileId = i1.FileId;
            ++j;
        }

        segments.push_back(i0);
        i = j;
    }

    return true;
}

bool LoadSegments(const std::vector<std::string> &inputGPXPaths, GPXSegments &segments, double fps,
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

        initialDistance = gpx->TotalDistance();
    }

    return true;
}

bool GetSegmentRange(GPXSegments &segments, time_t start, double duration, SegmentRange &range) {
    for (const auto &seg : segments) {
        size_t nextItem = 0;
        TrackItem item;
        if (!seg->GetClosestItem(start, nextItem, item)) {
            continue;
        }

        range.segment = seg;
        range.startIndex = nextItem;

        if (!seg->GetClosestItem(start + duration, nextItem, item)) {
            continue;
        }

        range.endIndex = nextItem;
        return true;
    }

    return false;
}

} // namespace gpsmap