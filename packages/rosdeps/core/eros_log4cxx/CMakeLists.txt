##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

##############################################################################
# Init Configuration
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_include(eros_build eros_package)
eros_3rd_party_init()

##############################################################################
# Variables
##############################################################################

set(SOURCE_NAME "apache-log4cxx-0.10.0")
set(SOURCE_TARBALL "apache-log4cxx-0.10.0-wg_patched.tar.gz")
set(SOURCE_TARBALL_URL "http://pr.willowgarage.com/downloads/${SOURCE_TARBALL}")
set(SOURCE_DIR ${CMAKE_BINARY_DIR}/${SOURCE_NAME})
set(PREFIX ${CMAKE_BINARY_DIR}/fakeroot)

set(PACKAGE_OPTIONS --with-apr=${CMAKE_INSTALL_PREFIX} --with-apr-util=${CMAKE_INSTALL_PREFIX})
set(COMMON_ARGS --prefix=${PREFIX} --exec-prefix=${PREFIX} C=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER})

if(CMAKE_CROSSCOMPILING)
  # Might need a check to properly evaluate ac_cv_sizeof_struct_iovec
  set(PRESET_VARIABLES ac_cv_file__dev_zero=yes ac_cv_func_setpgrp_void=yes apr_cv_tcp_nodelay_with_cork=no apr_cv_process_shared_works=no apr_cv_mutex_robust_shared=no ac_cv_sizeof_struct_iovec=8)
  set(CONFIGURE_COMMAND ./configure  --host=${TOOLCHAIN_TUPLE} --target=${TOOLCHAIN_TUPLE} ${PACKAGE_OPTIONS} ${PRESET_VARIABLES} ${COMMON_ARGS}) 
else()
  set(CONFIGURE_COMMAND ./configure ${PACKAGE_OPTIONS} ${COMMON_ARGS})
endif()

##############################################################################
# DOWNLOADING/GENERATING THE SOURCES
##############################################################################

set(PATCHED_FLAG ${CMAKE_BINARY_DIR}/patched)

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})

# Patch
add_custom_command(OUTPUT ${PATCHED_FLAG}
    COMMAND patch -p0 < ${CMAKE_SOURCE_DIR}/patch_console
    COMMAND touch ${PATCHED_FLAG}
    DEPENDS ${CMAKE_BINARY_DIR}/extracted
    WORKING_DIRECTORY ${SOURCE_DIR}/src/examples/cpp
    COMMENT "Patching ${SOURCE_DIR}."
    VERBATIM
    )


##############################################################################
# Build
##############################################################################

eros_autotools_compile("${CONFIGURE_COMMAND}" ${SOURCE_DIR} ${PATCHED_FLAG})

# By adding ALL here, it automatically gets built before the packaging command.
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

install(DIRECTORY ${PREFIX}/share/
        DESTINATION share
        USE_SOURCE_PERMISSIONS
        )

