Two patches here.

===== private.cmake =====

URL: https://code.ros.org/svn/ros/stacks/ros/trunk/core/rosbuild/private.cmake
Revision: 12874
Ticket: https://code.ros.org/trac/ros/attachment/ticket/2994/
Modification: 
    Critical fix in a find_file argument that forces find_file with
    NO_CMAKE_FIND_ROOT_PATH. Without this it won't look outside the toolchain root.

===== rostoolchain.cmake =====

This just removes a rather noisy comment made by cmake that informs the user
that a toolchain has been selected. Unfortunately it is really spammy when 
cross-compiling and not really useful anyway (you can see you're cross-compiling
by viewing the cmake output for compiler and such pretty easily anyway).

Not yet committed upstream (not urgent).