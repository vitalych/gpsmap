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

#include <assert.h>
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
    bool Valid = true;

    std::string OriginalTimestamp;
    double Timestamp = 0.0;
    double Latitude = 0.0;
    double Longitude = 0.0;
    double Speed = 0.0;
    double Elevation = 0.0;
    double DistanceDelta = 0.0;
    double TotalDistance = 0.0;
    double Bearing = 0.0;
    bool IsTrackStart = false;
    bool IsSegmentStart = false;

    // TODO: move this out of here
    unsigned MapIndex = 0;

    bool operator<(const TrackItem &item) {
        return Timestamp < item.Timestamp;
    }
};

struct GPXInfo {
    time_t Start;
    double Duration;
};

std::ostream &operator<<(std::ostream &os, TrackItem const &m);

class GPX;
using GPXPtr = std::shared_ptr<GPX>;

class GPXSegment;
using GPXSegmentPtr = std::shared_ptr<GPXSegment>;
using GPXSegments = std::vector<GPXSegmentPtr>;
using TrackItems = std::vector<TrackItem>;

class GPXSegment {
private:
    double m_initialDistance;
    double m_frequency;
    TrackItems m_items;

    GPXSegment(double initialDistance, double frequency) : m_initialDistance(initialDistance), m_frequency(frequency) {
    }

public:
    static GPXSegmentPtr Create(double initialDistance, double frequency) {
        return GPXSegmentPtr(new GPXSegment(initialDistance, frequency));
    }

    void AddItem(const TrackItem &item) {
        assert(!m_items.size() || (m_items.back().Timestamp <= item.Timestamp));
        m_items.push_back(item);
    }

    void UpdateDistances();
    void UpdateBearing();

    void SplitIdleSegments(GPXSegments &segments) const;

    GPXSegmentPtr Interpolate(double frequency) const;

    const TrackItem &operator[](std::size_t idx) const {
        assert(idx < m_items.size());
        return m_items[idx];
    }

    void AppendFront(const std::vector<TrackItem> &items) {
        m_items.insert(m_items.begin(), items.begin(), items.end());
    }

    GPXSegmentPtr Extract(unsigned start, unsigned end) const {
        if (start >= m_items.size() || end > m_items.size() || end < start) {
            return nullptr;
        }

        auto ret = GPXSegment::Create(0, m_frequency);
        ret->m_items.insert(ret->m_items.end(), m_items.begin() + start, m_items.begin() + end);
        assert(ret->m_items.size() == end - start);
        return ret;
    }

    const TrackItems::const_iterator begin() const {
        return m_items.begin();
    }

    const TrackItems::const_iterator end() const {
        return m_items.end();
    }

    const TrackItems::iterator begin() {
        return m_items.begin();
    }

    const TrackItems::iterator end() {
        return m_items.end();
    }

    const TrackItem &front() const {
        return m_items.front();
    }

    const TrackItem &back() const {
        return m_items.back();
    }

    TrackItem &front() {
        return m_items.front();
    }

    TrackItem &back() {
        return m_items.back();
    }

    const size_t size() const {
        return m_items.size();
    }

    bool GetClosestItem(time_t timestamp, size_t &nextItem, TrackItem &item);

    bool GetItem(size_t index, TrackItem &item);

    const std::vector<TrackItem> &GetItems() const {
        return m_items;
    }

    bool GetInfo(GPXInfo &info) const;

    double GetFrequency() const {
        return m_frequency;
    }

    bool FillSegment(const std::vector<GPXInfo> &info);
};

GPXSegmentPtr MergeSegments(const GPXSegments &segments);

class GPX {
private:
    GPXSegments m_trackSegments;
    double m_initialDistance;

    GPX() {
    }

public:
    bool LoadFromFile(const std::string &path, double interpolationFrequency);

    void SetInitialDistance(double distance) {
        m_initialDistance = distance;
    }

    const double TotalDistance() const {
        if (size() == 0) {
            return m_initialDistance;
        }

        auto seg_it = back();
        if (seg_it->size() == 0) {
            return m_initialDistance;
        }

        return back()->back().TotalDistance;
    }

    const size_t size() const {
        return m_trackSegments.size();
    }

    const GPXSegmentPtr back() const {
        return m_trackSegments.back();
    }

    const GPXSegments::const_iterator begin() const {
        return m_trackSegments.begin();
    }

    const GPXSegments::const_iterator end() const {
        return m_trackSegments.end();
    }

    const GPXSegments &GetTrackSegments() const {
        return m_trackSegments;
    }

    static GPXPtr Create();
};

time_t parse_time(const std::string &iso);
std::string time_to_str(time_t t);

} // namespace gpsmap

#endif
