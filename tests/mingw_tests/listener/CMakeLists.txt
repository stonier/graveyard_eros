rosbuild_add_executable(listener listener.cpp)
target_link_libraries(listener ${WINLIBS})
rosbuild_link_boost(listener thread signals filesystem system regex)
