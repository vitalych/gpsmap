// Copyright (c) 2021 Vitaly Chipounov
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

#ifndef __GPSMAP_JSON__
#define __GPSMAP_JSON__

#include <string>
#include <vector>

#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <gpsmap/utils.h>

namespace gpsmap {

bool WriteDocument(rapidjson::Document &doc, const std::string &file);
bool ReadDocument(rapidjson::Document &doc, const std::string &file);
rapidjson::Value SerializeVideoInfos(rapidjson::Document &Doc, const std::vector<VideoInfo> &info);
bool DeserializeVideoInfos(const rapidjson::Value &value, std::vector<VideoInfo> &info);
bool DeserializeVideoInfos(const std::string &descFile, std::vector<VideoInfo> &info);
rapidjson::Value SerializeGPXInfos(rapidjson::Document &Doc, const std::vector<GPXInfo> &info);
bool DeserializeGPXInfos(const rapidjson::Value &value, std::vector<GPXInfo> &info);

} // namespace gpsmap

#endif