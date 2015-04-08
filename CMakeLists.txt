cmake_minimum_required(VERSION 2.8)
project(evil_server)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

# This Boost version is simply the oldest tested one. If your system has a
# different one, go ahead and change this, but be sure to thoroughly test the
# software afterwards.
find_package(Boost 1.57.0 REQUIRED log system)

find_package(ZeroMQ 4.0 REQUIRED)

# Boost.Asio fails to recognize std::chrono as working with GCC 4.9, yet it
# seems to work fine for our purposes.
add_definitions(-DBOOST_ASIO_HAS_STD_CHRONO)

# This is a hack to get the Boost code properly on Linux distributions that ship
# Boost as a shared library. Should probably be auto-detected somehow.
add_definitions(-DBOOST_ALL_DYN_LINK)

include_directories(SYSTEM lib/azmq lib/msgpack)

add_subdirectory(src)
