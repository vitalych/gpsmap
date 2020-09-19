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

#ifndef _MAPGEN_RESOURCES_H_

#define _MAPGEN_RESOURCES_H_

#include <OpenImageIO/imagebuf.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <string>

class Resources;
using ResourcesPtr = std::shared_ptr<Resources>;

class Resources {
private:
    const boost::filesystem::path m_dir;
    OIIO::ImageBuf m_dot;
    boost::filesystem::path m_map;
    boost::filesystem::path m_font;

    Resources(const boost::filesystem::path &dir) : m_dir(dir) {
    }

    bool Load();

public:
    static ResourcesPtr Create(const boost::filesystem::path &dir) {
        auto ret = ResourcesPtr(new Resources(dir));
        if (ret->Load()) {
            return ret;
        }
        return nullptr;
    }

    const OIIO::ImageBuf &Dot() const {
        return m_dot;
    }

    const boost::filesystem::path &GetMapPath() const {
        return m_map;
    }

    const boost::filesystem::path &GetFontPath() const {
        return m_font;
    }
};

#endif
