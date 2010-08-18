'''
Created on 17/08/2010

Various routines which protect the ros stacks from recompilation
via the use of the ROS_NOBUILD file flag.

@author: Daniel Stonier
'''

import os
import sys
import subprocess
from eros_python_tools.core import ros_version
from eros_python_tools.unprotect import unprotect_ros_core

def min_packages():
    ''' Return the minimal set of packages needed for a cross compiled ros runtime. '''
    version = ros_version()
    if ( version == "boxturtle" ):
        packages = ["rospack", "genmsg_cpp", "roslang", "xmlrpcpp", "pycrypto", "paramiko", "roslib", "rospy", "rosconsole", "roscpp", "rosout", "roslaunch", "rostest"]
    elif ( version == "cturtle" ):
        packages = ["rospack", "genmsg_cpp", "rosbag", "rosclean", "rosdoc", "rosgraph", "roslang", "roslib", "rosmaster", "rosmsg", "rospy", "rosconsole", "roscpp", "rosout", "roslaunch", "rostest", "rxdeps", "topic_tools", "xmlrpcpp"]
    else:
        print "Unsupported version - defaulting to build/protect the latest official release's minimal set."
        packages = ["rospack", "genmsg_cpp", "rosbag", "rosclean", "rosdoc", "rosgraph", "roslang", "roslib", "rosmaster", "rosmsg", "rospy", "rosconsole", "roscpp", "rosout", "roslaunch", "rostest", "rxdeps", "topic_tools", "xmlrpcpp"]
    return packages

# Often we need to kickstart the process manually (without rospack, we can't do rosmake!)
def make_rospack():
    ''' Cleans and recompiles rospack. '''
    rospack_dir = os.getenv("ROS_ROOT")+"/tools/rospack"
    unused_retcode = subprocess.call(["make", "clean"],cwd=rospack_dir)
    unused_retcode = subprocess.call(["make"],cwd=rospack_dir)
    
def protect_ros_core():
    ''' Cleans, then recompiles the ros core stack, finally adds ROS_NOBUILD to all packages.'''
    unprotect_ros_core()
    make_rospack()
    ros_package_path = os.getenv('ROS_PACKAGE_PATH')
    os.environ['ROS_PACKAGE_PATH'] = os.getenv('ROS_ROOT')
    p = subprocess.Popen(["rosmake", "-ia", "--pre-clean", "--rosdep-install", "--rosdep-yes"])
    p.wait()
    os.environ['ROS_PACKAGE_PATH'] = ros_package_path

def protect_min():
    ''' Cleans, then recompiles a minimal set of ros core packages, finally adds ROS_NOBUILD to all packages.'''
    unprotect_ros_core()
    make_rospack()
    p = subprocess.Popen(["rosmake", "-i", "--pre-clean", "--rosdep-install", "--rosdep-yes"] + min_packages())
    p.wait()

def protect_main():
    ''' Main entry point for creating an erosprotect script.'''
    from optparse import OptionParser
    usage = "Usage: %prog [options] <core|minimal>"
    parser = OptionParser(usage=usage)
    parser.add_option("-u","--unprotect", action="store_true", dest="unprotect", default=False, help="Unprotect the ros core stack.")
    options, args = parser.parse_args()
    
    ###################
    # Abort Checks
    ###################
    if not args:
        parser.error("You must specify a target. Valid targets include [core|minimal].")
    
    ###################
    # Action
    ###################
    if ( options.unprotect ):
        unprotect_ros_core()
        sys.exit(0);
    if args:
        target = args[0]
        if ( target == "core" ):
            protect_ros_core()
        elif ( target == "minimal" ):
            protect_min()
        parser.error("Only options are permitted, run with --help to see options.")
        
if __name__ == "__main__":
    protect_main()
