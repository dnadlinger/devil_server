cmake_minimum_required(VERSION 2.8)
project(evil_server)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(Boost 1.57.0 REQUIRED system)
find_package(ZeroMQ 4.0 REQUIRED)

add_subdirectory(src)
