cmake_minimum_required(VERSION 2.8)
project(evil_server)

find_package(Boost 1.57.0 REQUIRED system)

add_executable(evil_server src/main.cpp)
