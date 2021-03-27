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
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gpsmap/utils.h>

namespace fs = std::filesystem;

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
    std::stringstream ss;
    ss << "ffprobe -v error -select_streams v:0 -show_entries stream=nb_frames,r_frame_rate -of "
          "default=nokey=1:noprint_wrappers=1 ";
    ss << "\"" << filePath << "\"";
    auto cmd = ss.str();
    auto result = exec(cmd.c_str());

    // First line is frame rate
    std::istringstream iss(result);
    std::string frameRate;
    iss >> frameRate;
    iss >> info.FrameCount;

    auto fileName = fs::path(filePath).stem().string();
    sscanf(fileName.c_str(), "GX-%d-%d", &info.FileId, &info.FileSequence);

    unsigned n = 0, d = 0;
    sscanf(frameRate.c_str(), "%d/%d", &n, &d);

    if (!n || !d) {
        throw std::runtime_error("Invalid frame rate");
    }

    info.FrameRate = (float) n / (float) d;
    return true;
}

} // namespace gpsmap