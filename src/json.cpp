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

#include <fstream>
#include <iostream>

#include <gpsmap/json.h>

namespace gpsmap {

bool WriteDocument(rapidjson::Document &doc, const std::string &file) {
    rapidjson::StringBuffer Buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer, rapidjson::Document::EncodingType, rapidjson::UTF8<>> Writer(
        Buffer);
    doc.Accept(Writer);

    std::cout << "Writing json to " << file << "\n";
    std::ofstream of(file, std::ofstream::out);
    if (of.fail()) {
        std::cerr << "Could not open " << file << "\n";
        return false;
    }

    of << Buffer.GetString() << "\n";
    of.close();

    std::cout << Buffer.GetString() << "\n";

    return true;
}

bool ReadDocument(rapidjson::Document &doc, const std::string &file) {
    std::ifstream ifs(file, std::ifstream::in);
    if (ifs.fail()) {
        std::cerr << "Could not open " << file << "\n";
        return false;
    }

    rapidjson::IStreamWrapper isw(ifs);
    doc.ParseStream(isw);
    return true;
}

rapidjson::Value SerializeVideoInfos(rapidjson::Document &Doc, const std::vector<VideoInfo> &info) {
    auto &Allocator = Doc.GetAllocator();

    rapidjson::Value List(rapidjson::kArrayType);
    for (const auto &vi : info) {
        rapidjson::Value Info(rapidjson::kObjectType);
        Info.AddMember("path", vi.Path, Allocator);
        Info.AddMember("file_id", vi.FileId, Allocator);
        Info.AddMember("file_seq", vi.FileSequence, Allocator);
        Info.AddMember("frame_rate", av_q2d(vi.FrameRate), Allocator);
        Info.AddMember("frame_rate_num", vi.FrameRate.num, Allocator);
        Info.AddMember("frame_rate_den", vi.FrameRate.den, Allocator);
        Info.AddMember("frame_count", vi.FrameCount, Allocator);

        if (vi.Start) {
            Info.AddMember("start", vi.Start, Allocator);
        }

        if (vi.Duration) {
            Info.AddMember("duration", vi.Duration, Allocator);
        }

        List.PushBack(Info, Allocator);
    }
    return List;
}

bool DeserializeVideoInfos(const rapidjson::Value &value, std::vector<VideoInfo> &info) {
    const auto array = value.GetArray();
    for (auto it = array.Begin(); it != array.End(); ++it) {
        VideoInfo vi;
        auto &val = *it;

        if (val.HasMember("path")) {
            vi.Path = val["path"].GetString();
        } else {
            return false;
        }

        if (val.HasMember("file_id")) {
            vi.FileId = val["file_id"].GetInt();
        } else {
            return false;
        }

        if (val.HasMember("file_seq")) {
            vi.FileSequence = val["file_seq"].GetInt();
        } else {
            return false;
        }

        if (val.HasMember("frame_rate_num") && val.HasMember("frame_rate_den")) {
            vi.FrameRate.num = val["frame_rate_num"].GetInt();
            vi.FrameRate.den = val["frame_rate_den"].GetInt();
        } else {
            return false;
        }

        if (val.HasMember("frame_count")) {
            vi.FrameCount = val["frame_count"].GetInt();
        } else {
            return false;
        }

        if (val.HasMember("start")) {
            vi.Start = val["start"].GetInt();
        }

        if (val.HasMember("duration")) {
            vi.Duration = val["duration"].GetDouble();
        }

        info.push_back(vi);
    }

    return true;
}

bool DeserializeVideoInfos(const std::string &descFile, std::vector<VideoInfo> &info) {
    rapidjson::Document inputDoc;
    if (!ReadDocument(inputDoc, descFile)) {
        std::cerr << "Could not read " << descFile << "\n";
        return false;
    }

    if (!inputDoc.HasMember("segments")) {
        std::cerr << "No segments field in " << descFile << "\n";
        return false;
    }

    if (!DeserializeVideoInfos(inputDoc["segments"], info)) {
        std::cerr << "Could not deserialize segments info\n";
        return false;
    }

    return true;
}

rapidjson::Value SerializeGPXInfos(rapidjson::Document &Doc, const std::vector<GPXInfo> &info) {
    auto &Allocator = Doc.GetAllocator();

    rapidjson::Value List(rapidjson::kArrayType);
    for (const auto &vi : info) {
        rapidjson::Value Info(rapidjson::kObjectType);
        Info.AddMember("start", vi.Start, Allocator);
        Info.AddMember("duration", vi.Duration, Allocator);
        List.PushBack(Info, Allocator);
    }

    return List;
}

bool DeserializeGPXInfos(const rapidjson::Value &value, std::vector<GPXInfo> &info) {
    const auto array = value.GetArray();
    for (auto it = array.Begin(); it != array.End(); ++it) {
        GPXInfo vi;
        auto &val = *it;

        if (val.HasMember("start")) {
            vi.Start = val["start"].GetInt();
        } else {
            return false;
        }

        if (val.HasMember("duration")) {
            vi.Duration = val["duration"].GetDouble();
        } else {
            return false;
        }

        info.push_back(vi);
    }

    return true;
}

} // namespace gpsmap