###############################################################################
# Build Platform : crossdev
# Toolchain : i686-pc-linux-gnu
# Target Root : /usr/i686-pc-linux-gnu
###############################################################################

# This one is important
set(CMAKE_SYSTEM_NAME Linux)

# Some useful variables that uniquely define this toolchain module
set(BUILD_PLATFORM "crossdev")
set(TOOLCHAIN_TUPLE "i686-pc-linux-gnu" CACHE STRING "Toolchain signature identifying cpu-vendor-platform-clibrary.")
set(TARGET_ROOT "/usr/${TOOLCHAIN_TUPLE}" CACHE STRING "Root of the target development environment (libraries, headers etc).")

# Specify the cross compiler - make sure it is in your PATH
set(CMAKE_C_COMPILER   ${TOOLCHAIN_TUPLE}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_TUPLE}-g++)

# Configure search directory for headers and libs (for the find function).
set(CMAKE_FIND_ROOT_PATH ${TARGET_ROOT} CACHE STRING "Cmake search variable for finding libraries/headers.")
MARK_AS_ADVANCED(CMAKE_FIND_ROOT_PATH)

# Search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# For libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Advanced variables - hide from the front page of cache
MARK_AS_ADVANCED(CMAKE_GENERATOR CMAKE_TOOLCHAIN_FILE)
