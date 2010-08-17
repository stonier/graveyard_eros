###############################################################################
# Toolchain - arm-samsung-linux-gnueabi
#
# See the companion .dox file for more information.
#
###############################################################################

# This one is important
set(CMAKE_SYSTEM_NAME "Linux")
set(CMAKE_SYSTEM_PROCESSOR "arm")

# Set the ecl root variables if it is not already set.
set(TOOLCHAIN_TUPLE "arm-samsung-linux-gnueabi" CACHE STRING "Toolchain signature identifying cpu-vendor-platform-clibrary.")
set(BUILD_INSTALL_PREFIX "/usr/local" CACHE STRING "Install path prepended to install dirs for build platform components.")
set(TARGET_ROOT "/usr/${TOOLCHAIN_TUPLE}" CACHE STRING "Root of the target development environment (libraries, headers etc).")
set(TARGET_INSTALL_PREFIX "${TARGET_ROOT}/usr/local" CACHE STRING "Install path prepended to install dirs for target platform components.")

# Specify the cross compiler - make sure it is in your PATH
set(CMAKE_C_COMPILER   ${TOOLCHAIN_TUPLE}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_TUPLE}-g++)

# Configure search directory for headers and libs (for the find function).
set(CMAKE_FIND_ROOT_PATH ${TARGET_ROOT} CACHE STRING "Cmake search variable for finding libraries/headers.")
set(CMAKE_INSTALL_PREFIX ${TARGET_INSTALL_PREFIX} CACHE PATH "Install prefix, used if a relative path is given.")
MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX CMAKE_FIND_ROOT_PATH)

# Search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# For libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Advanced variables - hide from the front page of cache
MARK_AS_ADVANCED(CMAKE_GENERATOR CMAKE_TOOLCHAIN_FILE)
