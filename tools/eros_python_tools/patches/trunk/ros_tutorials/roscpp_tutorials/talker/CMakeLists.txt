rosbuild_add_executable(talker talker.cpp)
target_link_libraries(talker ${WINLIBS})
rosbuild_link_boost(talker thread signals filesystem system regex)
