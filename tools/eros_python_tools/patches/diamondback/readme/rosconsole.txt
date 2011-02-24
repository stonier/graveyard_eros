===== private.cmake =====

File : CMakeLists.txt, include/ros/assert.h, include/ros/console.h, src/rosconsole/rosconsole.cpp
Last Changed Revision: 12775, Jan 06, 2011
Ticket: https://code.ros.org/trac/ros/ticket/3334
Modification: 
    Critical fix in a find_file argument that forces find_file with
    NO_CMAKE_FIND_ROOT_PATH. Without this it won't look outside the toolchain root.
    
    Also removes the -fPIC for mingw library builds (not in diamondback, but patched in trunk).
    
