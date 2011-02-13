macro(add_mingw_test tgt)
  rosbuild_add_executable(${tgt} ${ARGN})
  target_link_libraries(${tgt} ${WINLIBS})
  rosbuild_link_boost(${tgt} thread signals filesystem system regex)
endmacro(ecl_add_example)

add_mingw_test(talker talker.cpp common.cpp)
#add_mingw_test(nodelist nodelist.cpp common.cpp)

#add_mingw_test(listener listener.cpp)
