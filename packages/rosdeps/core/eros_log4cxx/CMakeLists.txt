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

if(MINGW)                    
  # Fix a bug in mingw cross compiler not setting this macro for you.
  set(ROS_COMPILE_FLAGS "${ROS_COMPILE_FLAGS} -D_WIN32_WINNT=0x0500")
  # This is wierd, i used this to find the bug, but it then fails again if I remove it
  set(ROS_LINK_FLAGS "${ROS_LINK_FLAGS} -no-undefined")
endif()

if(CMAKE_CROSSCOMPILING)
  if(MINGW)
    set(PLATFORM_OPTIONS --disable-shared --enable-static)
    set(PRESET_VARIABLES ) # Nothing here
  else() # Arm
    set(PLATFORM_OPTIONS  --disable-static --enable-shared)
    # Might need a check to properly evaluate ac_cv_sizeof_struct_iovec
    set(PRESET_VARIABLES ac_cv_file__dev_zero=yes ac_cv_func_setpgrp_void=yes apr_cv_tcp_nodelay_with_cork=no apr_cv_process_shared_works=no apr_cv_mutex_robust_shared=no ac_cv_sizeof_struct_iovec=8)
  endif()
  set(PLATFORM_OPTIONS ${PLATFORM_OPTIONS} --host=${TOOLCHAIN_TUPLE} --target=${TOOLCHAIN_TUPLE}) 
else()
  set(PLATFORM_OPTIONS --disable-static --enable-shared)
endif()

set(COMMON_ARGS --prefix=${CMAKE_INSTALL_PREFIX} 
                --exec-prefix=${CMAKE_INSTALL_PREFIX} 
                --with-apr=${CMAKE_INSTALL_PREFIX}
                --with-apr-util=${CMAKE_INSTALL_PREFIX}
                C=${CMAKE_C_COMPILER} 
                CXX=${CMAKE_CXX_COMPILER}
                CFLAGS=${ROS_COMPILE_FLAGS} 
                CXXFLAGS=${ROS_COMPILE_FLAGS}
                LDFLAGS=${ROS_LINK_FLAGS}
                )
set(CONFIGURE_COMMAND ./configure  ${PLATFORM_OPTIONS} ${PRESET_VARIABLES} ${COMMON_ARGS}) 

##############################################################################
# DOWNLOADING/GENERATING THE SOURCES
##############################################################################

set(PATCHED_FLAG ${CMAKE_BINARY_DIR}/patched)

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})

if(MINGW)
  set(PATCH_THREAD_COMMAND patch -d ${SOURCE_DIR}/src/main/include/log4cxx/helpers -p0 < ${CMAKE_SOURCE_DIR}/patches/mingw/patch_thread)
else()
  set(PATCH_THREAD_COMMAND echo "Only need to patch for threads on mingw.")
endif()

# Patch
add_custom_command(OUTPUT ${PATCHED_FLAG}
    COMMAND patch -p0 < ${CMAKE_SOURCE_DIR}/patches/all/patch_console
    COMMAND ${PATCH_THREAD_COMMAND}
    COMMAND touch ${PATCHED_FLAG}
    DEPENDS ${CMAKE_BINARY_DIR}/extracted
    WORKING_DIRECTORY ${SOURCE_DIR}/src/examples/cpp
    COMMENT "Patching ${SOURCE_DIR}."
    VERBATIM
    )

##############################################################################
# Build
##############################################################################

eros_autotools_compile_only(CONFIGURE_COMMAND ${SOURCE_DIR} ${PATCHED_FLAG})

# By adding ALL here, it automatically gets built before the packaging command.
add_custom_target(
        source ALL
        DEPENDS ${SOURCE_DIR}/compiled
        COMMENT "This is ${PROJECT_NAME}."
        )

##############################################################################
# Install
##############################################################################

add_custom_target(
        custom_install
        make install
        DEPENDS ${SOURCE_DIR}/compiled
        WORKING_DIRECTORY ${SOURCE_DIR}
        COMMENT "Installing ${PROJECT_NAME}."
        )

add_custom_target(
        custom_uninstall
        make install
        DEPENDS ${SOURCE_DIR}/compiled
        WORKING_DIRECTORY ${SOURCE_DIR}
        COMMENT "Uninstalling ${PROJECT_NAME}."
        )
       

