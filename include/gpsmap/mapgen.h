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

#ifndef MAPGEN_H

#define MAPGEN_H

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imageio.h>
#include <functional>
#include <memory>
#include <vector>

#include "gpx.h"
#include "resources.h"
#include "tilemanager.h"

namespace gpsmap {

using FrameState = TrackItem;

class IFrameGenerator {
public:
    virtual bool Generate(OIIO::ImageBuf &ib, const FrameState &state, int frameIndex, double fps) = 0;
};

class MapImageGenerator;
using MapImageGeneratorPtr = std::shared_ptr<MapImageGenerator>;

struct Marker {
    double Latitude;
    double Longitude;

    const OIIO::ImageBuf *Image;

    // Coordinates in Image that correspond to latitude, longitude
    int x;
    int y;
};

using Markers = std::vector<Marker>;

class MapImageGenerator : public IFrameGenerator {
public:
    struct Params {
        GPXSegmentPtr gpx;
        TileManagerPtr tiles;
        ResourcesPtr resources;
        int zoom;
        Markers markers;
    };

private:
    GPXSegmentPtr m_gpx;
    ResourcesPtr m_res;
    OIIO::ImageBuf m_grid;
    TileManagerPtr m_tiles;
    int m_centerx, m_centery;
    int m_viewportx, m_viewporty;
    int m_zoom;

    Markers m_markers;

    // TODO: clean hard-coded constants
    MapImageGenerator(Params &params)
        : m_gpx(params.gpx), m_res(params.resources), m_grid(OIIO::ImageSpec(512 * 3, 512 * 3, 4)),
          m_tiles(params.tiles), m_centerx(0), m_centery(0), m_zoom(params.zoom), m_markers(params.markers) {
    }

    void ToViewPortCoordinates(double lat, double lon, int &x, int &y) const;
    void ToGridCoordinates(double lat, double lon, int &x, int &y) const;

    bool LoadGrid(TilePtr tile);
    bool DrawDot(OIIO::ImageBuf &ib);
    void DrawMarkers(OIIO::ImageBuf &ib, double bearing);
    void DrawTrack(OIIO::ImageBuf &ib);
    bool DrawArrow(OIIO::ImageBuf &ib, double bearing);

public:
    static MapImageGeneratorPtr Create(Params &params) {
        return MapImageGeneratorPtr(new MapImageGenerator(params));
    }

    bool Generate(OIIO::ImageBuf &ib, const FrameState &state, int frameIndex, double fps);
};

class LabelGenerator;
using LabelGeneratorPtr = std::shared_ptr<LabelGenerator>;

class LabelGenerator : public IFrameGenerator {
private:
    std::string m_fontPath;

    // Cached labels
    std::string m_lbl1, m_lbl2;
    OIIO::ImageBuf m_buf1, m_buf2;

    LabelGenerator(const std::string &fontPath)
        : m_fontPath(fontPath), m_buf1(OIIO::ImageSpec(512, 32, 4)), m_buf2(OIIO::ImageSpec(512, 32, 4)){};

public:
    static LabelGeneratorPtr Create(const std::string &fontPath) {
        return LabelGeneratorPtr(new LabelGenerator(fontPath));
    }

    bool Generate(OIIO::ImageBuf &ib, const FrameState &state, int frameIndex, double fps);
};

class MapSwitcher;
using MapSwitcherPtr = std::shared_ptr<MapSwitcher>;
using MapSwitcherCb = std::function<bool(int, unsigned &)>;
using ZoomedMaps = std::vector<std::pair<MapImageGeneratorPtr, int>>;

class MapSwitcher : public IFrameGenerator {
private:
    ZoomedMaps m_maps;
    MapSwitcherCb m_cb;

    unsigned m_currentMapIndex;
    int m_remainingDuration;
    int m_prevSecond;

    MapSwitcher(MapSwitcherCb cb) : m_cb(cb) {
        m_currentMapIndex = 0;
        m_remainingDuration = 0;
        m_prevSecond = -1;
    };

public:
    static MapSwitcherPtr Create(MapSwitcherCb cb) {
        return MapSwitcherPtr(new MapSwitcher(cb));
    }

    void AddMapGenerator(MapImageGeneratorPtr map, int duration) {
        m_maps.push_back(std::make_pair(map, duration));
        m_currentMapIndex = 0;
        m_remainingDuration = m_maps[0].second;
    }

    bool Generate(OIIO::ImageBuf &ib, const FrameState &state, int frameIndex, double fps);
    void ComputeState(FrameState &state, int frameIndex, double fps);
};

} // namespace gpsmap

#endif
