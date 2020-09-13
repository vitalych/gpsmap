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

#include <gps/mapgen.h>

OIIO_NAMESPACE_USING

#define FONT_PATH "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"

bool MapImageGenerator::Generate(ImageBuf &ib, double lat, double lon) {
    int px, py;
    auto tile = m_tiles->GetTile(lat, lon, px, py, m_zoom);
    if (!tile) {
        std::cerr << "Could not get tile for " << lat << " " << lon << std::endl;
        return false;
    }
    auto td = tile->Desc();

    auto tw = tile->GetImage().xmax() + 1;
    auto th = tile->GetImage().ymax() + 1;

    if (m_centerx != td.x || m_centery != td.y) {
        TilePtr grid[3][3];
        grid[1][1] = tile;
        for (auto i = -1; i <= 1; i++) {
            for (auto j = -1; j <= 1; j++) {
                auto t = m_tiles->GetTile(td.x + i, td.y + j, m_zoom);
                if (!t) {
                    std::cerr << "Could not get tile x=" << td.x + i << " y=" << td.y + j << "\n";
                    return false;
                }
                auto img = t->GetImage();
                grid[j + 1][i + 1] = t;
            }
        }

        // Paint the whole grid
        int cx = 0;
        for (auto i = -1; i <= 1; i++) {
            int cy = 0;
            for (auto j = -1; j <= 1; j++) {
                auto img = grid[j + 1][i + 1]->GetImage();
                if (!ImageBufAlgo::paste(m_grid, cx, cy, 0, 0, img)) {
                    std::cerr << "Could not copy image" << std::endl;
                    return false;
                }
                cy += th;
            }
            cx += tw;
        }
        m_centerx = td.x;
        m_centery = td.y;
    }

    // Render centered image
    px += tw;
    py += th;
    if (!ImageBufAlgo::paste(ib, -(px - tw / 2), -(py - th / 2), 0, 0, m_grid)) {
        std::cerr << "Could not paste" << std::endl;
        return false;
    }

    // Draw the dot
    auto w = m_dot.spec().width / 2;
    auto h = m_dot.spec().height / 2;
    px = ib.spec().width / 2;
    py = ib.spec().height / 2;
    OIIO::ROI roi(px - w, px + w, py - h, py + h, 0, 1, /*chans:*/ 0, ib.nchannels());

    auto subimg = ImageBufAlgo::copy(ib, TypeUnknown, roi);
    subimg.set_origin(0, 0);

    auto comp = ImageBufAlgo::over(m_dot, subimg);
    if (comp.has_error()) {
        return false;
    }

    // Paste the dot over the final grid
    if (!ImageBufAlgo::paste(ib, px - w, py - w, 0, 0, comp)) {
        std::cerr << "Could not paste" << std::endl;
        return false;
    }

    return true;
}

bool GeoTracker::UpdateFrame(int frameIndex, int fps) {
    gpx::TrackItem firstItem;
    if (!m_gpx->GetItem(m_firstItemIdx, firstItem)) {
        // Nothing more to render
        return false;
    }

    auto second = (double) frameIndex / (double) fps;
    auto date = (double) firstItem.Timestamp + second;

    gpx::TrackItem item;
    size_t prev_item = m_nextItemIdx;

    if (prev_item >= m_lastItemIdx) {
        return false;
    }

    if (!m_gpx->GetClosestItem(date, m_nextItemIdx, item)) {
        // Nothing more to render
        return false;
    }

    // Interpolate speed and position
    auto speedInc = item.Speed;
    auto lat = item.Latitude;
    auto lon = item.Longitude;
    auto distInc = item.TotalDistance;

    if (prev_item == m_nextItemIdx) {
        gpx::TrackItem next_item;
        if (m_gpx->GetItem(m_nextItemIdx + 1, next_item)) {
            auto timeDiff = next_item.Timestamp - item.Timestamp;
            auto speedDiff = next_item.Speed - item.Speed;
            auto speedIncrement = speedDiff / timeDiff * (date - item.Timestamp);
            speedInc += speedIncrement;

            auto distDiff = next_item.TotalDistance - item.TotalDistance;
            auto distIncrement = distDiff / timeDiff * (date - item.Timestamp);
            distInc += distIncrement;

            lon += (next_item.Longitude - item.Longitude) / timeDiff * (date - item.Timestamp);
            lat += (next_item.Latitude - item.Latitude) / timeDiff * (date - item.Timestamp);
        }
    }

    m_lon = lon;
    m_lat = lat;
    m_dist = distInc;
    m_speed = speedInc;
    m_grade = item.Grade;
    m_elevation = item.Elevation;
    m_date = date;

    return true;
}

static std::string PrintTime(time_t ts) {
    char buff[32];
    struct tm res;
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime_r(&ts, &res));
    return buff;
}

bool LabelGenerator::Generate(OIIO::ImageBuf &ib) {
    int width = ib.spec().width;
    int height = ib.spec().height;

    std::stringstream lbl1;
    lbl1 << (int) (m_geo->Speed() * 3600 / 1000) << " km/h";
    lbl1 << "  " << (int) m_geo->Elevation() << " m"
         << "  " << (int) m_geo->Grade() << "%";
    lbl1 << "  " << std::fixed << std::setprecision(2) << m_geo->Distance() / 1000.0 << " km";
    auto lbl1Str = lbl1.str();

    std::stringstream lbl2;
    lbl2 << PrintTime(m_geo->Date());
    auto lbl2Str = lbl2.str();

    // Top label
    if (!ImageBufAlgo::render_box(ib, 0, 0, width, 32, 1.0f, true)) {
        std::cerr << "Could not render box" << std::endl;
        return false;
    }

    if (!ImageBufAlgo::render_text(ib, width / 2, 32 - 5, lbl2Str.c_str(), 32, FONT_PATH, {0.0f, 0.0f, 0.0f, 1.0f},
                                   ImageBufAlgo::TextAlignX::Center)) {
        fprintf(stderr, "Could not render text\n");
        return false;
    }

    // Bottom label
    if (!ImageBufAlgo::render_box(ib, 0, height - 32 - 5, width, height, 1.0f, true)) {
        std::cerr << "Could not render box" << std::endl;
        return false;
    }

    if (!ImageBufAlgo::render_text(ib, width / 2, height - 5, lbl1Str.c_str(), 32, FONT_PATH, {0.0f, 0.0f, 0.0f, 1.0f},
                                   ImageBufAlgo::TextAlignX::Center)) {
        fprintf(stderr, "Could not render text\n");
        return false;
    }

    return true;
}

bool MapSwitcher::Generate(int second, OIIO::ImageBuf &ib) {
    if (second != m_prevSecond) {
        --m_remainingDuration;
        if (!m_remainingDuration) {
            m_currentMapIndex = (m_currentMapIndex + 1) % m_maps.size();
            m_remainingDuration = m_maps[m_currentMapIndex].second;
        }
        m_prevSecond = second;
    }

    return m_maps[m_currentMapIndex].first->Generate(ib, m_geo->Latitude(), m_geo->Longitude());
}
