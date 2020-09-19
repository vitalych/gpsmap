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
    m_dot = Resize(m_dot, 32, 32);

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

    return true;
}
