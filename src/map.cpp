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

#include <gpsmap/map.h>

OIIO_NAMESPACE_USING

namespace gpsmap {

bool Overlay(ImageBuf &dest, const ImageBuf &source, int x, int y) {
    auto w = source.spec().width;
    auto h = source.spec().height;
    OIIO::ROI roi(x, x + w, y, y + h, 0, 1, /*chans:*/ 0, dest.nchannels());

    auto subimg = ImageBufAlgo::copy(dest, TypeUnknown, roi, 1);
    subimg.set_origin(0, 0);

    auto comp = ImageBufAlgo::over(source, subimg, {}, 1);
    if (comp.has_error()) {
        return false;
    }

    if (!ImageBufAlgo::paste(dest, x, y, 0, 0, comp, {}, 1)) {
        std::cerr << "Could not paste" << std::endl;
        return false;
    }

    return true;
}

bool MapImageGenerator::LoadGrid(TilePtr tile) {
    auto td = tile->Desc();
    auto tw = tile->GetImage().xmax() + 1;
    auto th = tile->GetImage().ymax() + 1;

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
            if (!ImageBufAlgo::paste(m_grid, cx, cy, 0, 0, img, {}, 1)) {
                std::cerr << "Could not copy image" << std::endl;
                return false;
            }
            cy += th;
        }
        cx += tw;
    }

    m_centerx = td.x;
    m_centery = td.y;

    return true;
}

bool MapImageGenerator::Generate(OIIO::ImageBuf &ib, const FrameState &state, int frameIndex, double fps) {
    int px, py;
    auto lat = state.Latitude;
    auto lon = state.Longitude;

    auto tile = m_tiles->GetTile(lat, lon, px, py, m_zoom);
    if (!tile) {
        std::cerr << "Could not get tile for " << lat << " " << lon << std::endl;
        return false;
    }
    auto td = tile->Desc();
    auto tw = tile->GetImage().xmax() + 1;
    auto th = tile->GetImage().ymax() + 1;

    if (m_centerx != td.x || m_centery != td.y) {
        if (!LoadGrid(tile)) {
            return false;
        }

        DrawTrack(m_grid);
    }

    // Render centered image
    px += tw;
    py += th;

    if (!ImageBufAlgo::paste(ib, -(px - tw / 2), -(py - th / 2), 0, 0, m_grid, {}, 1)) {
        std::cerr << "Could not paste" << std::endl;
        return false;
    }

    // Save the coordinates of the grid that correspond to (0, 0) of the viewport
    m_viewportx = px - tw / 2;
    m_viewporty = py - th / 2;

    DrawMarkers(ib, state.Bearing);

    return true;
}

bool MapImageGenerator::DrawDot(ImageBuf &ib) {
    // Draw the dot
    auto dot = m_res->Dot();

    auto w = dot.spec().width / 2;
    auto h = dot.spec().height / 2;
    auto px = ib.spec().width / 2;
    auto py = ib.spec().height / 2;

    return Overlay(ib, dot, px - w, py - h);
}

bool MapImageGenerator::DrawArrow(ImageBuf &ib, double bearing) {
    auto angle = bearing;
    auto arrow = m_res->GetArrow(angle);

    auto w = arrow.spec().width / 2;
    auto h = arrow.spec().height / 2;
    auto px = ib.spec().width / 2;
    auto py = ib.spec().height / 2;

    return Overlay(ib, arrow, px - w, py - h);
}

void MapImageGenerator::DrawMarkers(ImageBuf &ib, double bearing) {
    if (m_zoom >= 11) {
        DrawArrow(ib, bearing);
    } else {
        DrawDot(ib);
    }

    int vx, vy;
    for (const auto &marker : m_markers) {
        ToViewPortCoordinates(marker.Latitude, marker.Longitude, vx, vy);
        auto &img = *marker.Image;
        Overlay(ib, img, vx - marker.x, vy - marker.y);
    }
}

// TODO: optimize this, do not scan the entire list of items
void MapImageGenerator::DrawTrack(ImageBuf &ib) {
    bool hasPrev = false;
    int prevx, prevy;

    for (const auto &item : m_gpx->GetItems()) {
        int x, y;
        ToGridCoordinates(item.Latitude, item.Longitude, x, y);
        if (hasPrev && !item.IsSegmentStart) {
            ImageBufAlgo::render_line(ib, prevx, prevy, x, y, {0.0f, 0.0f, 1.0f}, false, {}, 1);
            ImageBufAlgo::render_line(ib, prevx, prevy, x + 1, y, {0.0f, 0.0f, 1.0f}, false, {}, 1);
        }
        prevx = x;
        prevy = y;
        hasPrev = true;
    }
}

// This function may return negative coordinates.
// The drawing functions will do the clipping.
void MapImageGenerator::ToViewPortCoordinates(double lat, double lon, int &x, int &y) const {
    ToGridCoordinates(lat, lon, x, y);

    // 3. Translate grid coords to viewport coords
    auto vx = x - m_viewportx;
    auto vy = y - m_viewporty;

    x = vx;
    y = vy;
}

// This function may return negative coordinates.
// The drawing functions will do the clipping.
void MapImageGenerator::ToGridCoordinates(double lat, double lon, int &x, int &y) const {
    // 1. Get tile coordinates
    int xt, yt, px, py;
    m_tiles->GetTileCoords(lat, lon, m_zoom, xt, yt, px, py);

    // 2. Translate to pixel coords in the internal grid
    auto topx = m_centerx - 1;
    auto topy = m_centery - 1;

    xt -= topx;
    yt -= topy;
    px += xt * 512;
    py += yt * 512;

    x = px;
    y = py;
}

static std::string PrintTime(time_t ts) {
    char buff[32];
    struct tm res;
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime_r(&ts, &res));
    return buff;
}

bool LabelGenerator::Generate(OIIO::ImageBuf &ib, const FrameState &state, int frameIndex, double fps) {
    int width = ib.spec().width;
    int height = ib.spec().height;

    std::stringstream lbl1;
    lbl1 << (int) (state.Speed * 3600 / 1000) << " km/h";
    lbl1 << "  " << (int) state.Elevation << " m";
    lbl1 << "  " << std::fixed << std::setprecision(2) << state.TotalDistance / 1000.0 << " km";
    auto lbl1Str = lbl1.str();

    std::stringstream lbl2;
    lbl2 << PrintTime((time_t) state.Timestamp);
    auto lbl2Str = lbl2.str();

    if (m_lbl2 != lbl2.str()) {

        // Top label
        if (!ImageBufAlgo::render_box(m_buf2, 0, 0, width, 32, 1.0f, true, {}, 1)) {
            std::cerr << "Could not render box" << std::endl;
            return false;
        }

        if (!ImageBufAlgo::render_text(m_buf2, width / 2, 32 - 5, lbl2Str.c_str(), 32, m_fontPath,
                                       {0.0f, 0.0f, 0.0f, 1.0f}, ImageBufAlgo::TextAlignX::Center,
                                       ImageBufAlgo::TextAlignY::Baseline, 0, {}, 1)) {
            fprintf(stderr, "Could not render text\n");
            return false;
        }

        m_lbl2 = lbl2.str();
    }

    if (!ImageBufAlgo::paste(ib, 0, 0, 0, 0, m_buf2, {}, 1)) {
        std::cerr << "Could not paste" << std::endl;
        return false;
    }

    if (m_lbl1 != lbl1.str()) {
        // Bottom label
        if (!ImageBufAlgo::render_box(m_buf1, 0, 0, width, 32, 1.0f, true, {}, 1)) {
            std::cerr << "Could not render box" << std::endl;
            return false;
        }

        if (!ImageBufAlgo::render_text(m_buf1, width / 2, 32 - 5, lbl1Str.c_str(), 32, m_fontPath,
                                       {0.0f, 0.0f, 0.0f, 1.0f}, ImageBufAlgo::TextAlignX::Center,
                                       ImageBufAlgo::TextAlignY::Baseline, 0, {}, 1)) {
            fprintf(stderr, "Could not render text\n");
            return false;
        }

        m_lbl1 = lbl1.str();
    }

    if (!ImageBufAlgo::paste(ib, 0, height - 32, 0, 0, m_buf1, {}, 1)) {
        std::cerr << "Could not paste" << std::endl;
        return false;
    }

    return true;
}

bool MapSwitcher::Generate(OIIO::ImageBuf &ib, const FrameState &state, int frameIndex, double fps) {
    assert(state.MapIndex < m_maps.size());
    return m_maps[state.MapIndex].first->Generate(ib, state, frameIndex, fps);
}

void MapSwitcher::ComputeState(FrameState &state, int frameIndex, double fps) {
    int second = frameIndex / fps;

    if (second != m_prevSecond) {
        if (m_prevSecond >= 0) {
            --m_remainingDuration;
        }

        if (!m_remainingDuration) {
            m_currentMapIndex = (m_currentMapIndex + 1) % m_maps.size();
            m_remainingDuration = m_maps[m_currentMapIndex].second;
        }

        // Let clients override the displayed map if needed
        if (m_maps.size() > 1) {
            unsigned overrideIndex = 0;
            if (m_cb(second, overrideIndex)) {
                if (overrideIndex < m_maps.size()) {
                    m_currentMapIndex = overrideIndex;
                } else {
                    m_currentMapIndex = m_maps.size() - 1;
                }
            }
        }

        m_prevSecond = second;
    }

    assert(m_currentMapIndex < m_maps.size());

    state.MapIndex = m_currentMapIndex;
}

} // namespace gpsmap
