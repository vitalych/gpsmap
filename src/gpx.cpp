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

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <iomanip>
#include <iostream>
#include <math.h>

#include <gpsmap/gpx.h>

namespace pt = boost::property_tree;

namespace gpsmap {

std::ostream &operator<<(std::ostream &os, TrackItem const &m) {
    os << "TrackItem " << m.OriginalTimestamp << " lat=" << m.Latitude << " lon=" << m.Longitude << " speed=" << m.Speed
       << " alt=" << m.Elevation;
    return os;
}

time_t parse_time(const std::string &iso) {
    std::tm t = {};
    int y, M, d, h, m;
    float s;
    sscanf(iso.c_str(), "%d-%d-%dT%d:%d:%fZ", &y, &M, &d, &h, &m, &s);

    t.tm_year = y - 1900; // Year since 1900
    t.tm_mon = M - 1;     // 0-11
    t.tm_mday = d;        // 1-31
    t.tm_hour = h;        // 0-23
    t.tm_min = m;         // 0-59
    t.tm_sec = (int) s;   // 0-61 (0-60 in C++11)

    return std::mktime(&t) - timezone;
    /*std::stringstream ss(iso);
    std::tm t = {};
    if (ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%SZ")) {
        return std::mktime(&t) - timezone;
    }
    return 0;*/
}

std::string time_to_str(time_t t) {
    char buff[64];
    struct tm *timeinfo;
    timeinfo = localtime(&t);
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", timeinfo);
    return buff;
}

// Returns the distance in meters between the given coordinates (in degrees)
static double distance(double lat1, double lon1, double lat2, double lon2) {
    // Radius of Earth in meters, R = 6371 * 1000
    long double R = 6371 * 1000;

    lat1 = to_rad(lat1);
    lon1 = to_rad(lon1);
    lat2 = to_rad(lat2);
    lon2 = to_rad(lon2);

    // Haversine Formula
    long double dlon = lon2 - lon1;
    long double dlat = lat2 - lat1;

    long double ans = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);

    return R * 2 * asin(sqrt(ans));
}

static double bearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 = to_rad(lat1);
    lon1 = to_rad(lon1);
    lat2 = to_rad(lat2);
    lon2 = to_rad(lon2);

    auto x = cos(lat2) * sin(lon2 - lon1);
    auto y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    auto bearing = atan2(x, y);
    return to_deg(bearing);
}

static double bearing(const TrackItem &i1, const TrackItem &i2) {
    return bearing(i1.Latitude, i1.Longitude, i2.Latitude, i2.Longitude);
}

static double angle_distance(double a1, double a2) {
    double result;
    auto phi = fmod(abs(a2 - a1), 360);
    auto sign = 1;
    if (!(((a1 - a2) >= 0 && (a1 - a2) <= 180) || ((a1 - a2) <= -180 && (a1 - a2) >= -360))) {
        sign = -1;
    }
    if (phi > 180) {
        result = 360 - phi;
    } else {
        result = phi;
    }
    return result * sign;
}

void GPXSegment::UpdateBearing() {
    if (m_items.size() < 2) {
        return;
    }

    for (size_t i = 0; i < m_items.size() - 1; ++i) {
        auto b = bearing(m_items[i], m_items[i + 1]);
        if (b == 0) {
            if (i > 1) {
                m_items[i].Bearing = m_items[i - 1].Bearing;
            }
        } else {
            m_items[i].Bearing = b;
        }
    }
}

void GPXSegment::UpdateDistances() {
    if (m_items.size() < 1) {
        return;
    }

    std::sort(m_items.begin(), m_items.end());

    for (size_t i = 1; i < m_items.size(); ++i) {
        auto &ti0 = m_items[i - 1];
        if (i == 1) {
            ti0.TotalDistance = m_initialDistance;
        }

        auto &ti1 = m_items[i];
        ti1.DistanceDelta = distance(ti0.Latitude, ti0.Longitude, ti1.Latitude, ti1.Longitude);
        ti1.TotalDistance = ti0.TotalDistance + ti1.DistanceDelta;
    }
}

void GPXSegment::SplitIdleSegments(GPXSegments &segments) const {
    std::vector<bool> idle;

    if (m_items.size() < 1) {
        return;
    }

    idle.push_back(false);

    for (size_t i = 1; i < m_items.size(); ++i) {
        const auto &prevItem = m_items[i - 1];
        const auto &item = m_items[i];

        auto latDelta = fabs(item.Latitude - prevItem.Latitude);
        auto lonDelta = fabs(item.Longitude - prevItem.Longitude);
        auto epsilon = 0.00000001;

        if (latDelta < epsilon && lonDelta < epsilon) {
            idle.push_back(true);
        } else {
            idle.push_back(false);
        }
    }
    assert(idle.size() == size());

    auto start = 0;
    auto curstate = idle[0];
    for (size_t i = 0; i < idle.size(); ++i) {
        auto isNewState = idle[i] != curstate;
        auto last = i == idle.size() - 1;
        if (isNewState || last) {
            auto seg = Extract(start, last ? i + 1 : i);
            assert(seg);
            segments.push_back(seg);
            curstate = idle[i];
            start = i;
        }
    }

    // Check that we took all items
    size_t totalsz = 0;
    for (auto seg : segments) {
        totalsz += seg->size();
    }
    assert(totalsz == size());
}

bool GPXSegment::GetItem(size_t index, TrackItem &item) {
    if (index >= m_items.size() || index < 0) {
        return false;
    }

    item = m_items[index];
    return true;
}

// Return the first element i such that i.timestamp <= timestamp < (i+1).timestamp
bool GPXSegment::GetClosestItem(time_t timestamp, size_t &nextItem, TrackItem &item) {
    if (nextItem >= m_items.size() || nextItem < 0) {
        return false;
    }

    if (timestamp < m_items[nextItem].Timestamp) {
        return false;
    }

    for (auto i = nextItem; i + 1 < m_items.size(); ++i) {
        const auto &i1 = m_items[i];
        const auto &i2 = m_items[i + 1];
        if (i1.Timestamp <= timestamp && timestamp < i2.Timestamp) {
            item = i1;
            nextItem = i;
            return true;
        }
    }

    return false;
}

bool GPXSegment::GetInfo(GPXInfo &info) const {
    if (m_items.size() == 0) {
        return false;
    }

    auto first = front();
    auto last = back();
    auto delta = last.Timestamp - first.Timestamp;

    auto freq = m_items.size() / delta;
    // std::cout << freq << "\n";
    int i = 0;
    auto lastTs = last.Timestamp;
    for (auto it = m_items.rbegin(); it != m_items.rend(); ++it) {
        if ((*it).Timestamp != lastTs) {
            break;
        }
        ++i;
    }

    info.Start = first.Timestamp;
    // TODO: make this more precise
    info.Duration = delta + (double) i / freq;
    return true;
}

GPXSegmentPtr GPXSegment::Interpolate(double frequency) const {
    if (m_items.size() < 2) {
        return nullptr;
    }

    auto ret = GPXSegment::Create(m_initialDistance);

    unsigned i = 0;
    bool isFirst = true;

    while (i < m_items.size() - 1) {
        auto &a = m_items[i];
        auto &b = m_items[i + 1];
        auto td = b.Timestamp - a.Timestamp;
        if (td == 0) {
            std::cerr << "Time diff between trace items is zero\n";
            return nullptr;
        }

        auto frames = double(td * frequency);
        auto timeStampDelta = double(td) / frames;
        auto latitudeDelta = (b.Latitude - a.Latitude) / frames;
        auto longitudeDelta = (b.Longitude - a.Longitude) / frames;
        auto speedDelta = (b.Speed - a.Speed) / frames;
        auto elevationDelta = (b.Elevation - a.Elevation) / frames;
        auto bearingDelta = angle_distance(b.Bearing, a.Bearing) / frames;

        for (auto t = 0; t < frames; ++t) {
            TrackItem newItem;
            newItem.Timestamp = a.Timestamp + timeStampDelta * t;
            newItem.Latitude = a.Latitude + latitudeDelta * t;
            newItem.Longitude = a.Longitude + longitudeDelta * t;
            newItem.Speed = a.Speed + speedDelta * t;
            newItem.Elevation = a.Elevation + elevationDelta * t;
            newItem.Bearing = a.Bearing + bearingDelta * t;
            newItem.IsTrackStart = a.IsTrackStart && isFirst;
            newItem.IsSegmentStart = a.IsSegmentStart && isFirst;
            isFirst = false;
            ret->AddItem(newItem);
        }

        ++i;
    }

    return ret;
}

GPXSegmentPtr MergeSegments(const GPXSegments &segments) {
    auto ret = GPXSegment::Create(0.0);
    for (auto &seg : segments) {
        for (auto &item : *seg) {
            ret->AddItem(item);
        }
    }
    return ret;
}

std::shared_ptr<GPX> GPX::Create() {
    return std::shared_ptr<GPX>(new GPX());
}

bool GPX::LoadFromFile(const std::string &path, double interpolationFrequency) {
    pt::ptree tree;
    pt::read_xml(path, tree);

    auto first = true;
    auto initialDistance = m_initialDistance;

    BOOST_FOREACH (pt::ptree::value_type &trkseg, tree.get_child("gpx.trk")) {
        bool firstSegment = true;

        auto segment = GPXSegment::Create(initialDistance);

        BOOST_FOREACH (pt::ptree::value_type &trkpt, trkseg.second) {
            TrackItem item;
            auto lat = trkpt.second.get_child("<xmlattr>.lat").get_value("");
            auto lon = trkpt.second.get_child("<xmlattr>.lon").get_value("");

            auto elevation = trkpt.second.get_child("ele").get_value("");
            auto time = trkpt.second.get_child("time").get_value("");

            std::string speed;
            auto _speed = trkpt.second.get_child_optional("speed");
            if (_speed) {
                speed = (*_speed).get_value("");
            } else {
                _speed = trkpt.second.get_child_optional("extensions.gpxtpx:TrackPointExtension.gpxtpx:speed");
                if (_speed) {
                    speed = (*_speed).get_value("");
                } else {
                    std::cerr << "Could not get speed\n";
                    speed = "0.0";
                }
            }

            item.Speed = strtod(speed.c_str(), nullptr);
            item.Latitude = strtod(lat.c_str(), nullptr);
            item.Longitude = strtod(lon.c_str(), nullptr);
            item.Elevation = strtod(elevation.c_str(), nullptr);
            item.OriginalTimestamp = time;
            item.Timestamp = parse_time(time);
            item.DistanceDelta = 0.0;
            item.TotalDistance = 0.0;
            item.Bearing = 0.0;
            item.IsTrackStart = first;
            item.IsSegmentStart = firstSegment;
            first = false;
            firstSegment = false;

            segment->AddItem(item);
        }

        segment->UpdateBearing();

        if (interpolationFrequency) {
            segment = segment->Interpolate(interpolationFrequency);
            if (!segment) {
                std::cerr << "Could not interpolate\n";
                return false;
            }
        }

        segment->UpdateDistances();

        if (segment->size()) {
            initialDistance = segment->back().TotalDistance;
            m_trackSegments.push_back(segment);
        } else {
            std::cerr << "Segment has no track items in " << path << "\n";
        }
    }

    return true;
}

} // namespace gpsmap
