add_subdirectory(fliquer)

add_executable(fliquer_observe fliquer_observe.cpp)
target_link_libraries(fliquer_observe fliquer ${Boost_LIBRARIES})

add_executable(fliquer_fake_resource fliquer_fake_resource.cpp)
target_link_libraries(fliquer_fake_resource fliquer ${Boost_LIBRARIES})
