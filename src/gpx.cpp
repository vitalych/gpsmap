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
    std::stringstream ss(iso);
    std::tm t = {};
    if (ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%SZ")) {
        return std::mktime(&t) - timezone;
    }
    return 0;
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

static double GetGrade(const std::vector<TrackItem> &items, int i) {
    const double maxDist = 50.0;

    // Get altitude and cumulative distance on the left
    double ldist = 0.0;
    double lelev = 0.0;
    for (auto l = i; l >= 0; --l) {
        ldist += items[l].DistanceDelta;
        lelev = items[l].Elevation;
        if (ldist >= maxDist) {
            break;
        }
    }

    // Get altitude and cumulative distance on the right
    double rdist = 0.0;
    double relev = 0.0;
    for (size_t l = i; l < items.size(); ++l) {
        rdist += items[l].DistanceDelta;
        relev = items[l].Elevation;
        if (rdist >= maxDist) {
            break;
        }
    }

    auto td = ldist + rdist;
    if (td < 0.01) {
        return 0.0;
    }

    return (relev - lelev) / td * 100;
}

std::shared_ptr<GPX> GPX::Create() {
    return std::shared_ptr<GPX>(new GPX());
}

void GPX::LoadFromFile(const std::string &path) {
    pt::ptree tree;
    pt::read_xml(path, tree);

    BOOST_FOREACH (pt::ptree::value_type &trkseg, tree.get_child("gpx.trk")) {
        BOOST_FOREACH (pt::ptree::value_type &trkpt, trkseg.second) {
            TrackItem item;
            auto lat = trkpt.second.get_child("<xmlattr>.lat").get_value("");
            auto lon = trkpt.second.get_child("<xmlattr>.lon").get_value("");
            auto speed = trkpt.second.get_child("speed").get_value("");
            auto elevation = trkpt.second.get_child("ele").get_value("");
            auto time = trkpt.second.get_child("time").get_value("");

            item.Speed = strtod(speed.c_str(), nullptr);
            item.Latitude = strtod(lat.c_str(), nullptr);
            item.Longitude = strtod(lon.c_str(), nullptr);
            item.Elevation = strtod(elevation.c_str(), nullptr);
            item.OriginalTimestamp = time;
            item.Timestamp = parse_time(time);
            item.Grade = 0.0;
            item.DistanceDelta = 0.0;
            item.TotalDistance = 0.0;
            item.Bearing = 0.0;

            m_trackItems.push_back(item);
        }
    }

    std::sort(m_trackItems.begin(), m_trackItems.end());

    for (size_t i = 1; i < m_trackItems.size(); ++i) {
        auto &ti0 = m_trackItems[i - 1];
        if (i == 1) {
            ti0.TotalDistance = m_initialDistance;
        }

        auto &ti1 = m_trackItems[i];
        ti1.DistanceDelta = distance(ti0.Latitude, ti0.Longitude, ti1.Latitude, ti1.Longitude);
        ti1.TotalDistance = ti0.TotalDistance + ti1.DistanceDelta;
    }

    for (size_t i = 0; i < m_trackItems.size() - 1; ++i) {
        m_trackItems[i].Grade = GetGrade(m_trackItems, i);
        auto b = bearing(m_trackItems[i], m_trackItems[i + 1]);
        if (b == 0) {
            if (i > 1) {
                m_trackItems[i].Bearing = m_trackItems[i - 1].Bearing;
            }
        } else {
            m_trackItems[i].Bearing = b;
        }
    }
}

void GPX::CreateSegments() {
    std::vector<int> boundaries;

    if (m_trackItems.size() == 0) {
        return;
    }

    boundaries.push_back(0);

    for (size_t i = 1; i < m_trackItems.size(); ++i) {
        const auto &prevItem = m_trackItems[i - 1];
        const auto &item = m_trackItems[i];

        if (item.Latitude == prevItem.Latitude && item.Longitude == prevItem.Longitude) {
            boundaries.push_back(i);
        }
    }

    auto bs = boundaries.size();
    for (size_t i = 0; i < bs; ++i) {
        int start = boundaries[i];
        int end = i == bs - 1 ? m_trackItems.size() - 1 : boundaries[i + 1];

        if (i == bs - 1) {
            if (start < end) {
                m_segments.push_back(Segment(start, end));
            }
        } else {
            assert(start < end);
            m_segments.push_back(Segment(start, end));
        }
    }
}

bool GPX::GetItem(size_t index, TrackItem &item) {
    if (index >= m_trackItems.size() || index < 0) {
        return false;
    }

    item = m_trackItems[index];
    return true;
}

// Return the first element i such that i.timestamp <= timestamp < (i+1).timestamp
bool GPX::GetClosestItem(time_t timestamp, size_t &nextItem, TrackItem &item) {
    if (nextItem >= m_trackItems.size() || nextItem < 0) {
        return false;
    }

    if (timestamp < m_trackItems[nextItem].Timestamp) {
        return false;
    }

    for (auto i = nextItem; i + 1 < m_trackItems.size(); ++i) {
        const auto &i1 = m_trackItems[i];
        const auto &i2 = m_trackItems[i + 1];
        if (i1.Timestamp <= timestamp && timestamp < i2.Timestamp) {
            item = i1;
            nextItem = i;
            return true;
        }
    }

    return false;
}

} // namespace gpsmap
