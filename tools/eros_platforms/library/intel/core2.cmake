###############################################################################
# Family : intel
# Platform : core2
###############################################################################

# Some useful custom variables that uniquely define this platform module
set(PLATFORM_FAMILY "intel" CACHE STRING "Platform family, usually referring to intel/arm etc.")
set(PLATFORM_NAME "core2" CACHE STRING "Platform name, usually referring to the cpu architecture.")

# Flags
rosbuild_check_for_sse() # this sets the SSE_FLAGS variable
set(PLATFORM_COMPILE_FLAGS "-march=core2 -pipe -mmmx ${SSE_FLAGS}" CACHE STRING "Compile flags specific to this platform.")
set(PLATFORM_LINK_FLAGS "" CACHE STRING "Link flags specific to this platform.")

