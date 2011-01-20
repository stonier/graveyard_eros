
===== init.cpp, spinner.cpp, poll_manager.cpp, xmlrpc_manager.cpp  =====

URL: https://code.ros.org/svn/ros/stacks/ros_comm/trunk/clients/cpp/roscpp/src/libros
Revision: 12874
Ticket: https://code.ros.org/trac/ros/ticket/2935.
Modification:
    Cedric's timer patches to improve ros performance under load. This reduces
    latencies for timers in that effect various loop times for things that are 
    actually related to just one-off events, and so not that important for 
    runtime operation.

