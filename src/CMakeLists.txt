include_directories(.)

add_subdirectory(evil)
add_subdirectory(fliquer)

add_executable(evil_monitor_servers evil_monitor_servers)
target_link_libraries(evil_monitor_servers fliquer ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} pthread)

add_executable(evil_observe_devices evil_observe_devices.cpp)
target_link_libraries(evil_observe_devices evil ${Boost_LIBRARIES} udev)

add_executable(evil_server evil_server.cpp)
target_link_libraries(evil_server evil fliquer ${Boost_LIBRARIES} ${ZeroMQ_LIBRARIES} udev pthread)

add_executable(fliquer_observe fliquer_observe.cpp)
target_link_libraries(fliquer_observe fliquer ${Boost_LIBRARIES})

add_executable(fliquer_fake_resource fliquer_fake_resource.cpp)
target_link_libraries(fliquer_fake_resource fliquer ${Boost_LIBRARIES})
