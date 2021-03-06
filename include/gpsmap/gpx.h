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

#ifndef GPX_H

#define GPX_H

#include <math.h>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

static inline double to_rad(double deg) {
    return deg * M_PI / 180.0;
}

static inline double to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

namespace gpsmap {
struct TrackItem {
    time_t Timestamp;
    std::string OriginalTimestamp;
    double Latitude;
    double Longitude;
    double Speed;
    double Elevation;
    float Grade;
    double DistanceDelta;
    double TotalDistance;
    double Bearing;
    bool IsTrackStart;

    bool operator<(const TrackItem &item) {
        return Timestamp < item.Timestamp;
    }
};

std::ostream &operator<<(std::ostream &os, TrackItem const &m);

class GPX;
using GPXPtr = std::shared_ptr<GPX>;

using Segment = std::pair<int, int>;
using Segments = std::vector<Segment>;

class GPX {
private:
    GPX() {
    }

    std::vector<TrackItem> m_trackItems;
    Segments m_segments;
    double m_initialDistance;

public:
    void LoadFromFile(const std::string &path);

    void CreateSegments();

    const Segments &GetSegments() const {
        return m_segments;
    }

    bool GetClosestItem(time_t timestamp, size_t &nextItem, TrackItem &item);

    bool GetItem(size_t index, TrackItem &item);

    const std::vector<TrackItem> &GetItems() const {
        return m_trackItems;
    }

    const TrackItem &First() const {
        return m_trackItems.front();
    }

    const TrackItem &Last() const {
        return m_trackItems.back();
    }

    void SetInitialDistance(double distance) {
        m_initialDistance = distance;
    }

    static GPXPtr Create();
};

time_t parse_time(const std::string &iso);
std::string time_to_str(time_t t);
} // namespace gpsmap

#endif
