##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

##############################################################################
# Ros - pick up user's rostoolchain.cmake & rosconfig.cmake if exists
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_include(eros_build eros_package)

##############################################################################
# Configuration
##############################################################################

eros_3rd_party_build_init()

##############################################################################
# Variables
##############################################################################

set(SOURCE_NAME "apr-1.4.2")
set(SOURCE_TARBALL "${SOURCE_NAME}.tar.gz")
# Use ecl's repo, mirrors at apache keep bloody changing and they don't have a constant alias to use!
set(SOURCE_TARBALL_URL "http://embedded-control-library.googlecode.com/files/${SOURCE_TARBALL}")
set(SOURCE_DIR ${CMAKE_BINARY_DIR}/${SOURCE_NAME})
set(PREFIX ${CMAKE_BINARY_DIR}/fakeroot)

set(PACKAGE_OPTIONS --disable-static --enable-shared)
# The compilers and ros compile flags are all set by rostoolchain.cmake and rosconfig.cmake
set(COMMON_ARGS --prefix=${PREFIX} --exec-prefix=${PREFIX} C=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER} CFLAGS=${ROS_COMPILE_FLAGS} CXXFLAGS=${ROS_COMPILE_FLAGS})

if(CMAKE_CROSSCOMPILING)
  set(PRESET_VARIABLES ac_cv_file__dev_zero=yes ac_cv_func_setpgrp_void=yes apr_cv_tcp_nodelay_with_cork=no apr_cv_process_shared_works=no apr_cv_mutex_robust_shared=no ac_cv_sizeof_struct_iovec=8 apr_cv_mutex_recurive=yes)
  set(CONFIGURE_COMMAND ./configure  --host=${TOOLCHAIN_TUPLE} --target=${TOOLCHAIN_TUPLE} ${PACKAGE_OPTIONS} ${PRESET_VARIABLES} ${COMMON_ARGS}) 
else()
  set(CONFIGURE_COMMAND ./configure ${PACKAGE_OPTIONS} ${COMMON_ARGS})
endif()

##############################################################################
# Build
##############################################################################

ecl_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
ecl_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})
ecl_autotools_compile("${CONFIGURE_COMMAND}" ${SOURCE_DIR} ${CMAKE_BINARY_DIR}/extracted)

add_custom_target(
        source ALL
        DEPENDS ${SOURCE_DIR}/compiled
        COMMENT "This is ${PROJECT_NAME}."
        )

##############################################################################
# Install
##############################################################################

install(DIRECTORY ${PREFIX}/include/
        DESTINATION include
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${PREFIX}/lib/
        DESTINATION lib
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${PREFIX}/bin/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${PREFIX}/build-1/
        DESTINATION build-1
        USE_SOURCE_PERMISSIONS
        )
        