cmake_minimum_required(VERSION 2.8)
project(evil_server)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(Boost 1.57.0 REQUIRED log system)
find_package(ZeroMQ 4.0 REQUIRED)

add_definitions(-DBOOST_ASIO_HAS_STD_CHRONO -DBOOST_ALL_DYN_LINK)

include_directories(SYSTEM lib/azmq lib/msgpack)

add_subdirectory(src)
