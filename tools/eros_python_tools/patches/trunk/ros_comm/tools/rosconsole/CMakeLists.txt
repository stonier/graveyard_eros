cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

include_directories(include/rosconsole)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_add_library(${PROJECT_NAME} src/rosconsole/rosconsole.cpp)
target_link_libraries(${PROJECT_NAME} log4cxx)
rosbuild_link_boost(${PROJECT_NAME} thread regex)

set(CMAKE_VERBOSE_MAKEFILE TRUE)
rosbuild_add_executable(example examples/example.cpp)
target_link_libraries(example ${PROJECT_NAME})
# static libraries all need to be called for mingw. 
# later this would be better if we could pick up rosconsole's libflags
# (which would be tailed for mingw_cross) and use that directly here.
if(MINGW) 
  rosbuild_link_boost(example thread regex)
  target_link_libraries(example log4cxx -laprutil-1 expat apr-1 rpcrt4 shell32 ws2_32 advapi32 kernel32 msvcrt)
endif()

rosbuild_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest ${PROJECT_NAME})

if(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
  rosbuild_add_gtest(test/assertion_test test/assertion_test.cpp)
  target_link_libraries(test/assertion_test ${PROJECT_NAME})
endif(${CMAKE_SYSTEM_NAME} STREQUAL Linux)

rosbuild_add_gtest(test/thread_test test/thread_test.cpp)
target_link_libraries(test/thread_test ${PROJECT_NAME})

#rosbuild_add_executable(speed_test test/speed_test.cpp)
#target_link_libraries(speed_test ${PROJECT_NAME})
#set_target_properties(speed_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

