# Copyright (c) 2020 Vitaly Chipounov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

add_library(
    gpsmap
    encoder.cpp
    gpx.cpp
    tilemanager.cpp
    map.cpp
    resources.cpp
    utils.cpp
    json.cpp
)

add_executable(
    gps
    main.cpp
)

TARGET_LINK_LIBRARIES(gps gpsmap)

add_executable(
    gpstest
    test.cpp
)

TARGET_LINK_LIBRARIES(gpstest gpsmap)

add_executable(
    gpstool
    tool.cpp
)

add_executable(
    gpstimecode
    timecode.cpp
)

TARGET_LINK_LIBRARIES(gpstool gpsmap)
TARGET_LINK_LIBRARIES(gpstimecode gpsmap)

SET(LIBS avcodec avutil avformat swscale swresample OpenImageIO boost_system boost_filesystem curl m pthread)

target_link_libraries(gps ${LIBS})
target_link_libraries(gpstool ${LIBS})
target_link_libraries(gpstimecode ${LIBS})
target_link_libraries(gpstest gtest ${LIBS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
