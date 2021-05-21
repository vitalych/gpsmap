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

#include <string>
#include <vector>
#include "gpx.h"

#ifndef _GPSMAP_UTILS_H_
#define _GPSMAP_UTILS_H_

extern "C" {
#include <libavutil/rational.h>
}

namespace gpsmap {

struct VideoInfo {
    std::string Path;
    int FileId = 0;
    int FileSequence = 0;
    AVRational FrameRate = {0, 1};
    unsigned FrameCount = 0;

    // These come from the GPX data and may not fully match
    // the actual frame count in terms of duration.
    time_t Start = 0;
    double Duration = 0.0f;
};

struct SegmentRange {
    GPXSegmentPtr segment;
    unsigned startIndex;
    unsigned endIndex;
};

bool GetVideoInfo(const std::string &filePath, VideoInfo &info);

bool LoadVideoInfo(const std::string &inputVideoPath, VideoInfo &videoInfo);

bool LoadVideoInfo(const std::vector<std::string> &inputVideoPaths, std::vector<VideoInfo> &videoInfo);

bool LoadVideoGpx(const std::vector<std::string> &inputVideoGpxPaths, std::vector<GPXInfo> &gpxInfo);

bool ComputeMapSegmentsForGpxVideos(const std::vector<VideoInfo> &videoInfo, const std::vector<GPXInfo> &gpxInfo,
                                    std::vector<VideoInfo> &segments);

bool ComputeMapSegmentsForGpxVideos(const std::vector<VideoInfo> &videoInfo, std::vector<VideoInfo> &segments);

bool LoadSegments(const std::vector<std::string> &inputGPXPaths, GPXSegments &segments, unsigned fps,
                  bool splitIdleParts);

bool GetSegmentRange(GPXSegments &segments, time_t start, double duration, SegmentRange &range);

} // namespace gpsmap

#endif
