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

class MapImageGenerator;
using MapImageGeneratorPtr = std::shared_ptr<MapImageGenerator>;

class MapImageGenerator {
private:
    gpx::GPXPtr m_gpx;
    ResourcesPtr m_res;
    OIIO::ImageBuf m_grid;
    TileManagerPtr m_tiles;
    int m_centerx, m_centery;
    int m_viewportx, m_viewporty;
    int m_zoom;

    // TODO: clean hard-coded constants
    MapImageGenerator(gpx::GPXPtr &gpx, TileManagerPtr &tiles, ResourcesPtr &resources, int zoom)
        : m_gpx(gpx), m_res(resources), m_grid(OIIO::ImageSpec(512 * 3, 512 * 3, 4)), m_tiles(tiles), m_centerx(0),
          m_centery(0), m_zoom(zoom) {
    }

    bool ToViewPortCoordinates(OIIO::ImageBuf &ib, double lat, double lon, int &x, int &y);

    bool DrawDot(OIIO::ImageBuf &ib);
    void DrawMarkers(OIIO::ImageBuf &ib);

public:
    static MapImageGeneratorPtr Create(gpx::GPXPtr &gpx, TileManagerPtr &tiles, ResourcesPtr &resources, int zoom) {
        return MapImageGeneratorPtr(new MapImageGenerator(gpx, tiles, resources, zoom));
    }

    bool Generate(OIIO::ImageBuf &ib, double lat, double lon);
};

// Tracks geo data associated with each frame
class GeoTracker;
using GeoTrackerPtr = std::shared_ptr<GeoTracker>;

class GeoTracker {
private:
    gpx::GPXPtr m_gpx;
    size_t m_firstItemIdx;
    size_t m_nextItemIdx;
    size_t m_lastItemIdx;

    double m_lon, m_lat, m_speed, m_dist, m_elevation, m_grade;
    time_t m_date;

    GeoTracker(gpx::GPXPtr gpx, size_t first, size_t last) : m_gpx(gpx) {
        m_nextItemIdx = 0;
        m_firstItemIdx = first;
        m_lastItemIdx = last;
        m_nextItemIdx = first;
    };

public:
    static GeoTrackerPtr Create(gpx::GPXPtr gpx, size_t first, size_t last) {
        return GeoTrackerPtr(new GeoTracker(gpx, first, last));
    }

    double Longitude() const {
        return m_lon;
    }

    double Latitude() const {
        return m_lat;
    }

    double Speed() const {
        return m_speed;
    }

    double Distance() const {
        return m_dist;
    }

    double Elevation() const {
        return m_elevation;
    }

    double Grade() const {
        return m_grade;
    }

    time_t Date() const {
        return m_date;
    }

    bool UpdateFrame(int frameIndex, int fps);
};

class LabelGenerator;
using LabelGeneratorPtr = std::shared_ptr<LabelGenerator>;

class LabelGenerator {
private:
    GeoTrackerPtr m_geo;
    std::string m_fontPath;

    // Cached labels
    std::string m_lbl1, m_lbl2;
    OIIO::ImageBuf m_buf1, m_buf2;

    LabelGenerator(GeoTrackerPtr geo, const std::string &fontPath)
        : m_geo(geo), m_fontPath(fontPath), m_buf1(OIIO::ImageSpec(512, 32, 4)), m_buf2(OIIO::ImageSpec(512, 32, 4)){};

public:
    static LabelGeneratorPtr Create(GeoTrackerPtr geo, const std::string &fontPath) {
        return LabelGeneratorPtr(new LabelGenerator(geo, fontPath));
    }

    bool Generate(OIIO::ImageBuf &ib);
};

class MapSwitcher;
using MapSwitcherPtr = std::shared_ptr<MapSwitcher>;
using MapSwitcherCb = std::function<bool(int, int &)>;

class MapSwitcher {
private:
    using MapImageGenerators = std::vector<std::pair<MapImageGeneratorPtr, int>>;

    GeoTrackerPtr m_geo;
    MapImageGenerators m_maps;
    MapSwitcherCb m_cb;

    int m_currentMapIndex;
    int m_remainingDuration;
    int m_prevSecond;

    MapSwitcher(GeoTrackerPtr geo, MapSwitcherCb cb) : m_geo(geo), m_cb(cb) {
        m_currentMapIndex = 0;
        m_remainingDuration = 0;
        m_prevSecond = 0;
    };

public:
    static MapSwitcherPtr Create(GeoTrackerPtr geo, MapSwitcherCb cb) {
        return MapSwitcherPtr(new MapSwitcher(geo, cb));
    }

    void AddMapGenerator(MapImageGeneratorPtr map, int duration) {
        m_maps.push_back(std::make_pair(map, duration));
        m_currentMapIndex = 0;
        m_remainingDuration = m_maps[0].second;
    }

    bool Generate(int second, OIIO::ImageBuf &ib);
};

#endif
