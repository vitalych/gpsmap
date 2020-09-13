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

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <curl/curl.h>
#include <iostream>
#include <math.h>

#include <gps/tilemanager.h>

namespace pt = boost::property_tree;

// https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#C.2FC.2B.2B
template <typename T> static T long2tilex(double lon, int z) {
    return (T)(((lon + 180.0) / 360.0 * (double) (1 << z)));
}

template <typename T> static T lat2tiley(double lat, int z) {
    double latrad = lat * M_PI / 180.0;
    return (T)(((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (double) (1 << z)));
}

static bool DownloadFile(const std::string &url, const std::string &filePath) {
    static bool inited = false;
    CURL *curl = nullptr;
    CURLcode res = CURLE_OK;
    bool ret = false;
    FILE *fp = nullptr;

    if (!inited) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        inited = true;
    }

    std::cout << "Downloading " << url << std::endl;

    curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Could not init curl" << std::endl;
        goto err;
    }

    fp = fopen(filePath.c_str(), "wb");
    if (!fp) {
        std::cerr << "Could not open " << filePath << std::endl;
        goto err;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);

    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Could not download " << url << std::endl;
        goto err;
    }

    ret = true;
err:
    if (curl) {
        curl_easy_cleanup(curl);
    }

    if (fp) {
        fclose(fp);
    }

    return ret;
}

Tile::Tile(const std::string &filePath, const TileDesc &desc) : m_filePath(filePath), m_desc(desc) {
    m_image = OIIO::ImageBuf(filePath);

    int channelorder[] = {0, 1, 2, -1 /*use a float value*/};
    float channelvalues[] = {0 /*ignore*/, 0 /*ignore*/, 0 /*ignore*/, 1.0};
    std::string channelnames[] = {"", "", "", "A"};
    m_image = OIIO::ImageBufAlgo::channels(m_image, 4, channelorder, channelvalues, channelnames);
}

TilePtr Tile::Create(const std::string &filePath, const TileDesc &desc) {
    return TilePtr(new Tile(filePath, desc));
}

TileManagerPtr TileManager::Create(const std::string &tilesRootPath, const std::string &mapDescPath) {
    if (!boost::filesystem::is_directory(tilesRootPath)) {
        std::cerr << tilesRootPath << " does not exist or is not a directory." << std::endl;
        return nullptr;
    }

    pt::ptree tree;
    pt::read_xml(mapDescPath, tree);
    auto mapUrl = tree.get_child("map.url").get_value("");
    if (mapUrl.size() == 0) {
        std::cerr << "Could not get map url from " << mapDescPath << std::endl;
        return nullptr;
    }

    std::cout << "Using map URL " << mapUrl << std::endl;

    return TileManagerPtr(new TileManager(tilesRootPath, mapUrl));
}

TilePtr TileManager::DownloadTile(const TileDesc &tile) const {
    auto url = m_mapUrl;

    boost::replace_all(url, "$x", std::to_string(tile.x));
    boost::replace_all(url, "$y", std::to_string(tile.y));
    boost::replace_all(url, "$z", std::to_string(tile.z));

    std::stringstream fp;
    fp << m_tilesRootPath << "/" << tile.z << "/" << tile.x;

    auto dir = fp.str();
    if (!boost::filesystem::is_directory(dir)) {
        if (!boost::filesystem::create_directories(dir)) {
            std::cerr << "Could not create directory " << dir << std::endl;
            return nullptr;
        }
    }

    fp << "/" << tile.y << ".png";
    auto filePath = fp.str();

    if (boost::filesystem::exists(filePath)) {
        if (boost::filesystem::file_size(filePath) > 0) {
            return Tile::Create(filePath, tile);
        } else {
            boost::filesystem::remove(filePath);
        }
    }

    if (!DownloadFile(url, filePath)) {
        boost::filesystem::remove(filePath);
        return nullptr;
    }

    return Tile::Create(fp.str(), tile);
}

TilePtr TileManager::GetTile(int x, int y, int zoom) {
    auto d = TileDesc(x, y, zoom);
    auto it = m_tiles.find(d);

    TilePtr ret;
    if (it != m_tiles.end()) {
        ret = it->second;
    } else {
        ret = DownloadTile(d);
        if (!ret) {
            return nullptr;
        }

        m_tiles[d] = ret;
    }
    return ret;
}

TilePtr TileManager::GetTile(double lat, double lon, int &xt, int &yt, int zoom) {
    auto x = long2tilex<double>(lon, zoom);
    auto y = lat2tiley<double>(lat, zoom);

    TilePtr ret = GetTile(x, y, zoom);
    if (!ret) {
        return nullptr;
    }

    // Get the pixel coordinates inside the tile
    double dummy;
    auto w = ret->GetImage().spec().width;
    auto h = ret->GetImage().spec().height;
    auto px = modf(x, &dummy) * w;
    auto py = modf(y, &dummy) * h;

    xt = px;
    yt = py;
    return ret;
}
