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
# Verbosity
##############################################################################

set(CMAKE_VERBOSE_MAKEFILE TRUE)

##############################################################################
# Variables
##############################################################################

set(SOURCE_NAME mingw-cross-env-2.16)
set(SOURCE_DIR ${CMAKE_BINARY_DIR}/mingw-cross-env-2.16)
set(SOURCE_TARBALL ${SOURCE_NAME}.tar.gz)
set(SOURCE_TARBALL_URL http://download.savannah.gnu.org/releases-redirect/mingw-cross-env/${SOURCE_TARBALL})

##############################################################################
# Download
##############################################################################

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})

##############################################################################
# Compiling
##############################################################################

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/compiled
    COMMAND make $ENV{ROS_PARALLEL_JOBS} gcc qt
    COMMAND touch ${CMAKE_BINARY_DIR}/compiled
    DEPENDS ${CMAKE_BINARY_DIR}/extracted
    WORKING_DIRECTORY ${SOURCE_DIR}
    COMMENT "Pre-compiling mingw gcc and qt."
    VERBATIM
    )

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/links
    COMMAND mkdir -p /opt/mingw
    COMMAND cp -r ${SOURCE_DIR}/* /opt/mingw
    COMMAND mkdir -p /usr/local/bin
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-addr2line /usr/local/bin/i686-pc-mingw32-addr2line
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-ar /usr/local/bin/i686-pc-mingw32-ar
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-as /usr/local/bin/i686-pc-mingw32-as
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-c++ /usr/local/bin/i686-pc-mingw32-c++
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-c++filt /usr/local/bin/i686-pc-mingw32-c++filt
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-cpp /usr/local/bin/i686-pc-mingw32-cpp
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-dlltool /usr/local/bin/i686-pc-mingw32-dlltool
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-dllwrap /usr/local/bin/i686-pc-mingw32-dllwrap
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-g++ /usr/local/bin/i686-pc-mingw32-g++
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-gcc /usr/local/bin/i686-pc-mingw32-gcc
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-gccbug /usr/local/bin/i686-pc-mingw32-gccbug
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-gcov /usr/local/bin/i686-pc-mingw32-gcov
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-gfortran /usr/local/bin/i686-pc-mingw32-gfortran
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-gprof /usr/local/bin/i686-pc-mingw32-gprof
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-ld /usr/local/bin/i686-pc-mingw32-ld
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-moc /usr/local/bin/i686-pc-mingw32-moc
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-nm /usr/local/bin/i686-pc-mingw32-nm
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-objcopy /usr/local/bin/i686-pc-mingw32-objcopy
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-objdump /usr/local/bin/i686-pc-mingw32-objdump
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-pg_config /usr/local/bin/i686-pc-mingw32-pg_config
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-pkg-config /usr/local/bin/i686-pc-mingw32-pkg-config
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-ranlib /usr/local/bin/i686-pc-mingw32-ranlib
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-rcc /usr/local/bin/i686-pc-mingw32-rcc
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-readelf /usr/local/bin/i686-pc-mingw32-readelf
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-size /usr/local/bin/i686-pc-mingw32-size
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-strings /usr/local/bin/i686-pc-mingw32-strings
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-strip /usr/local/bin/i686-pc-mingw32-strip
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-uic /usr/local/bin/i686-pc-mingw32-uic
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-windmc /usr/local/bin/i686-pc-mingw32-windmc
    COMMAND ln -s /opt/mingw/usr/bin/i686-pc-mingw32-windres /usr/local/bin/i686-pc-mingw32-windres
    COMMAND make $ENV{ROS_PARALLEL_JOBS} gcc qt
    COMMAND touch ${CMAKE_BINARY_DIR}/compiled
    DEPENDS ${CMAKE_BINARY_DIR}/compiled
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Copying root and setting up symbolic links."
    VERBATIM
    )


##############################################################################
# Target
##############################################################################

# Forcing here, not ideal, but easiest solution to get it into the preferred
# location without human error.
set(CMAKE_INSTALL_PREFIX "/opt" CACHE PATH "Install prefix, used if a relative path is given." FORCE)

# By adding ALL here, it automatically gets built before the packaging command.
add_custom_target(
        source ALL
        DEPENDS ${CMAKE_BINARY_DIR}/links
        COMMENT "Run 'make install' -> /opt. To change location, edit CMAKE_INSTALL_PREFIX in the CMakeLists.txt."
        )


##############################################################################
# Install
##############################################################################

add_custom_target(
        custom_install
        DEPENDS ${CMAKE_BINARY_DIR}/links
        COMMENT "Installing to /opt/mingw."
        )
add_dependencies(custom_install source)

add_custom_target(
        custom_uninstall
        rm -f /usr/local/bin/i686-pc-mingw32*
        rm -rf /opt/mingw
        COMMENT "Uninstalling."
        )
