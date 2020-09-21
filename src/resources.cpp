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

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/imageio.h>

#include <gps/resources.h>

OIIO_NAMESPACE_USING

static bool LoadFromFile(const boost::filesystem::path &dir, const std::string &fileName, OIIO::ImageBuf &img) {
    auto filePath = boost::filesystem::path(dir).append(fileName);
    if (!boost::filesystem::exists(filePath)) {
        std::cerr << "Could not find " << filePath.string() << std::endl;
        return false;
    }

    img = OIIO::ImageBuf(filePath.string());
    return true;
}

static OIIO::ImageBuf Resize(const OIIO::ImageBuf in, int nw, int nh) {
    auto w = in.spec().width;
    auto h = in.spec().height;

    if (w != nw || h != nh) {
        OIIO::ROI roi(0, nw, 0, nh, 0, 1, /*chans:*/ 0, in.nchannels());
        return OIIO::ImageBufAlgo::resize(in, "", 0, roi, 1);
    } else {
        return in;
    }
}

bool Resources::Load() {
    if (!LoadFromFile(m_dir, "dot32.png", m_dot)) {
        return false;
    }
    m_dot = Resize(m_dot, 16, 16);

    if (!LoadFromFile(m_dir, "pin_start.png", m_startPin)) {
        return false;
    }
    m_startPin = Resize(m_startPin, 45, 64);

    if (!LoadFromFile(m_dir, "pin_finish.png", m_finishPin)) {
        return false;
    }
    m_finishPin = Resize(m_finishPin, 45, 64);
    m_finishPin = OIIO::ImageBufAlgo::flip(m_finishPin);

    m_map = boost::filesystem::path(m_dir).append("OpenStreetMap-HiDPI.xml");
    if (!boost::filesystem::exists(m_map)) {
        std::cerr << "Could not find " << m_map.string() << std::endl;
        return false;
    }

    m_font = boost::filesystem::path(m_dir).append("LiberationSans-Regular.ttf");
    if (!boost::filesystem::exists(m_font)) {
        std::cerr << "Could not find " << m_font.string() << std::endl;
        return false;
    }

    // Initialize arrows in various angles
    if (!LoadFromFile(m_dir, "arrow.png", m_arrow)) {
        return false;
    }
    m_arrow = Resize(m_arrow, 96, 96);

    return true;
}

const OIIO::ImageBuf &Resources::GetArrow(int angle) {
    std::unique_lock<std::mutex> lock(m_lock);
    {
        angle = angle % 360;
        auto it = m_arrows.find(angle);
        if (it != m_arrows.end()) {
            return it->second;
        }

        auto rotated = ImageBufAlgo::rotate(m_arrow, angle * M_PI / 180.0, string_view(), 0.0f, false, {}, 1);
        m_arrows[angle] = rotated;
        return m_arrows[angle];
    }
}
