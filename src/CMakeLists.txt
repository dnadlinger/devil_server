add_subdirectory(fliquer)

add_executable(fliquer_observe fliquer_observe.cpp)
target_link_libraries(fliquer_observe fliquer ${Boost_LIBRARIES})
