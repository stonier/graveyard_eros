Two patches here.

===== private.cmake =====

URL: https://code.ros.org/svn/ros/stacks/ros/trunk/core/rosbuild/private.cmake
Revision: 12874
Ticket: https://code.ros.org/trac/ros/attachment/ticket/2994/
Modification: 
    Critical fix in a find_file argument that forces find_file with
    NO_CMAKE_FIND_ROOT_PATH. Without this it won't look outside the toolchain root.

===== public.cmake =====

URL: https://code.ros.org/svn/ros/stacks/ros/trunk/core/rosbuild/public.cmake
Revision: 12722
Ticket: https://code.ros.org/trac/ros/ticket/3275
Modification:
    Restricts the search path to the toolchain when looking for gtest.
