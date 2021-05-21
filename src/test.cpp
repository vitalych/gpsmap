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

#include <gpsmap/utils.h>
#include <gtest/gtest.h>

using namespace gpsmap;

namespace {

TEST(ComputeMapSegmentsForGpxVideos, T1) {
    std::vector<VideoInfo> videoInfo;
    std::vector<GPXInfo> gpxInfo;
    std::vector<VideoInfo> segmentInfo;

    auto fr = (AVRational){60, 1};

    videoInfo.push_back({.FileId = 0, .FileSequence = 0, .FrameRate = fr, .FrameCount = 60 * 100});
    gpxInfo.push_back({.Start = 123, .Duration = 100.0});

    auto ret = ComputeMapSegmentsForGpxVideos(videoInfo, gpxInfo, segmentInfo);
    EXPECT_TRUE(ret);

    ASSERT_EQ(1, segmentInfo.size());
    ASSERT_EQ(0, segmentInfo[0].FileId);
    ASSERT_EQ(0, segmentInfo[0].FileSequence);
    ASSERT_EQ(123, segmentInfo[0].Start);
    ASSERT_EQ(60 * 100, segmentInfo[0].FrameCount);
    ASSERT_EQ(100.0, segmentInfo[0].Duration);

    //////////////

    videoInfo.push_back({.FileId = 0, .FileSequence = 1, .FrameRate = fr, .FrameCount = 60 * 111});
    gpxInfo.push_back({.Start = 123 + 100, .Duration = 111.0});

    videoInfo.push_back({.FileId = 0, .FileSequence = 2, .FrameRate = fr, .FrameCount = 60 * 222});
    gpxInfo.push_back({.Start = 123 + 100 + 111, .Duration = 222.0});

    segmentInfo.clear();
    ret = ComputeMapSegmentsForGpxVideos(videoInfo, gpxInfo, segmentInfo);
    EXPECT_TRUE(ret);
    ASSERT_EQ(1, segmentInfo.size());
    ASSERT_EQ(0, segmentInfo[0].FileId);
    ASSERT_EQ(0, segmentInfo[0].FileSequence);
    ASSERT_EQ(123, segmentInfo[0].Start);
    ASSERT_EQ(60 * (100 + 111 + 222), segmentInfo[0].FrameCount);
    ASSERT_EQ(100.0 + 111.0 + 222.0, segmentInfo[0].Duration);

    //////////////

    videoInfo.push_back({.FileId = 1, .FileSequence = 0, .FrameRate = fr, .FrameCount = 60 * 444});
    gpxInfo.push_back({.Start = 1230, .Duration = 444.0});

    videoInfo.push_back({.FileId = 1, .FileSequence = 1, .FrameRate = fr, .FrameCount = 60 * 555});
    gpxInfo.push_back({.Start = 1230 + 444, .Duration = 555.0});

    segmentInfo.clear();
    ret = ComputeMapSegmentsForGpxVideos(videoInfo, gpxInfo, segmentInfo);
    EXPECT_TRUE(ret);
    ASSERT_EQ(2, segmentInfo.size());
    ASSERT_EQ(1, segmentInfo[1].FileId);
    ASSERT_EQ(0, segmentInfo[1].FileSequence);
    ASSERT_EQ(1230, segmentInfo[1].Start);
    ASSERT_EQ(60 * (444 + 555), segmentInfo[1].FrameCount);
    ASSERT_EQ(444.0 + 555.0, segmentInfo[1].Duration);
}
} // namespace

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
