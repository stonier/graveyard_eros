###############################################################################
# Family : mingw_cross
# Tuple : i686-pc-mingw32
# Sysroot : /opt/mingw/usr/i686-pc-mingw32
###############################################################################

# Some useful custom variables that uniquely define this toolchain module
set(TOOLCHAIN_FAMILY "mingw_cross")
set(TOOLCHAIN_TUPLE "i686-pc-mingw32" CACHE STRING "Toolchain signature identifying cpu-vendor-platform-clibrary.")
set(TOOLCHAIN_SYSROOT "/opt/mingw/usr/${TOOLCHAIN_TUPLE}" CACHE STRING "Root of the target development environment (libraries, headers etc).")
set(TOOLCHAIN_INSTALL_PREFIX "${TOOLCHAIN_SYSROOT}" CACHE STRING "Preferred install location when using the toolchain.")

# Now the cmake variables
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_C_COMPILER   ${TOOLCHAIN_TUPLE}-gcc) # Make sure these are in your PATH
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_TUPLE}-g++)
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_SYSROOT} CACHE STRING "Cmake search variable for finding libraries/headers.")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER) # Don't search for programs in sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)  # Headers and libs from sysroot only
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Boost needs to be forced to use static libs - bad place for this, put in an eros_build macro.
#set(ROS_COMPILE_FLAGS "${ROS_COMPILE_FLAGS} -DBOOST_THREAD_USE_LIB")

###############################
# Prepare Qt Environment
###############################
set(QT_IS_STATIC 1) # Works on my gentoo (cmake 2.8.1), fails on lucid ubuntu (cmake 2.8.0)
set(QT_QMAKE_EXECUTABLE ${TOOLCHAIN_TUPLE}-qmake) 

# Hide from cache's front page
MARK_AS_ADVANCED(CMAKE_GENERATOR CMAKE_FIND_ROOT_PATH CMAKE_TOOLCHAIN_FILE TOOLCHAIN_FAMILY TOOLCHAIN_TUPLE TOOLCHAIN_SYSROOT)
