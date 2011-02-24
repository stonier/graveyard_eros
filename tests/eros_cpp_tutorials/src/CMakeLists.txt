macro(add_mingw_test tgt)
  rosbuild_add_executable(${tgt} ${ARGN})
endmacro()

add_mingw_test(talker talker.cpp common.cpp)
add_mingw_test(listener listener.cpp common.cpp)
add_mingw_test(nodelist nodelist.cpp common.cpp)
add_mingw_test(rostimers rostime.cpp)
