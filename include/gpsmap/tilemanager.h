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

#ifndef TILEMANAGER_H

#define TILEMANAGER_H

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <OpenImageIO/imageio.h>

namespace gpsmap {

class TileManager;
using TileManagerPtr = std::shared_ptr<TileManager>;

class Tile;
using TilePtr = std::shared_ptr<Tile>;

struct TileDesc {
    int x, y, z;
    TileDesc() : x(0), y(0), z(0) {
    }
    TileDesc(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {
    }

    bool operator==(const TileDesc &o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

} // namespace gpsmap

namespace std {

template <> struct hash<gpsmap::TileDesc> {
    std::size_t operator()(const gpsmap::TileDesc &k) const {
        using std::hash;
        using std::size_t;
        using std::string;

        return ((hash<int>()(k.x) ^ (hash<int>()(k.y) << 1)) >> 1) ^ (hash<int>()(k.z) << 1);
    }
};

} // namespace std

namespace gpsmap {

class Tile {
public:
    enum State { LOADING, LOADED, FAILED };

private:
    std::string m_filePath;
    TileDesc m_desc;

    OIIO::ImageBuf m_image;

    std::mutex m_lock;
    std::condition_variable m_cv;
    State m_state;

    Tile(const std::string &filePath, const TileDesc &desc);
    Tile(const TileDesc &desc);

public:
    OIIO::ImageBuf &GetImage() {
        return m_image;
    }

    const TileDesc &Desc() const {
        return m_desc;
    }

    static TilePtr Create(const std::string &filePath, const TileDesc &desc);
    static TilePtr Create(const TileDesc &desc);
    void Load(const std::string &filePath);
    void Fail();
    bool Failed() {
        return m_state == FAILED;
    }

    const std::string &FilePath() const {
        return m_filePath;
    }

    bool WaitUntilDownloaded();
};

using Tiles = std::unordered_map<TileDesc, TilePtr>;

class TileManager {
private:
    std::mutex m_lock;
    std::string m_tilesRootPath;
    std::string m_mapUrl;
    Tiles m_tiles;

    int m_tileWidth;
    int m_tileHeight;

    TileManager(const std::string &tilesRootPath, const std::string &mapUrl) {
        m_tilesRootPath = tilesRootPath;
        m_mapUrl = mapUrl;

        // TODO: auto-detect sizes
        m_tileWidth = 512;
        m_tileHeight = 512;
    }

    bool DownloadTile(TilePtr &tile) const;

    TilePtr GetTileFast(int x, int y, int zoom);

public:
    TilePtr GetTile(int x, int y, int zoom);
    TilePtr GetTile(double lat, double lon, int &xt, int &yt, int zoom);
    void GetTileCoords(double lat, double lon, int zoom, int &xt, int &yt, int &px, int &py) const;
    static TileManagerPtr Create(const std::string &tilesRootPath, const std::string &mapDescPath);
};

} // namespace gpsmap

#endif
