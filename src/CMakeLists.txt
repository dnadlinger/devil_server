include_directories(.)

add_subdirectory(devil)
add_subdirectory(fliquer)

add_executable(devil_monitor_servers devil_monitor_servers)
target_link_libraries(devil_monitor_servers fliquer ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} pthread)

add_executable(devil_observe_devices devil_observe_devices.cpp)
target_link_libraries(devil_observe_devices devil ${Boost_LIBRARIES} udev pthread)

add_executable(devild devild.cpp)
target_link_libraries(devild devil fliquer ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} udev pthread)

add_executable(fliquer_observe fliquer_observe.cpp)
target_link_libraries(fliquer_observe fliquer ${Boost_LIBRARIES} pthread)

add_executable(fliquer_fake_resource fliquer_fake_resource.cpp)
target_link_libraries(fliquer_fake_resource fliquer ${Boost_LIBRARIES} pthread)
