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
#include <memory>
#include <vector>

#include "gpx.h"
#include "tilemanager.h"

class MapImageGenerator;
using MapImageGeneratorPtr = std::shared_ptr<MapImageGenerator>;

class MapImageGenerator {
private:
    const OIIO::ImageBuf &m_dot;
    OIIO::ImageBuf m_grid;
    TileManagerPtr m_tiles;
    int m_centerx, m_centery;
    int m_zoom;

    // TODO: clean hard-coded constants
    MapImageGenerator(TileManagerPtr &tiles, const OIIO::ImageBuf &dot, int zoom)
        : m_dot(dot), m_grid(OIIO::ImageSpec(512 * 3, 512 * 3, 4)), m_tiles(tiles), m_centerx(0), m_centery(0),
          m_zoom(zoom) {
    }

public:
    static MapImageGeneratorPtr Create(TileManagerPtr &tiles, const OIIO::ImageBuf &dot, int zoom) {
        return MapImageGeneratorPtr(new MapImageGenerator(tiles, dot, zoom));
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

    LabelGenerator(GeoTrackerPtr geo) : m_geo(geo){};

public:
    static LabelGeneratorPtr Create(GeoTrackerPtr geo) {
        return LabelGeneratorPtr(new LabelGenerator(geo));
    }

    bool Generate(OIIO::ImageBuf &ib);
};

class MapSwitcher;
using MapSwitcherPtr = std::shared_ptr<MapSwitcher>;

class MapSwitcher {
private:
    using MapImageGenerators = std::vector<std::pair<MapImageGeneratorPtr, int>>;

    GeoTrackerPtr m_geo;
    MapImageGenerators m_maps;

    int m_currentMapIndex;
    int m_remainingDuration;
    int m_prevSecond;

    MapSwitcher(GeoTrackerPtr geo) : m_geo(geo) {
        m_currentMapIndex = 0;
        m_remainingDuration = 0;
        m_prevSecond = 0;
    };

public:
    static MapSwitcherPtr Create(GeoTrackerPtr geo) {
        return MapSwitcherPtr(new MapSwitcher(geo));
    }

    void AddMapGenerator(MapImageGeneratorPtr map, int duration) {
        m_maps.push_back(std::make_pair(map, duration));
        m_currentMapIndex = 0;
        m_remainingDuration = m_maps[0].second;
    }

    bool Generate(int second, OIIO::ImageBuf &ib);
};

#endif
