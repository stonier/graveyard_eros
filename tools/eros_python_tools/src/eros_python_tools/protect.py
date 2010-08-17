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

def min_packages():
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
    rospack_dir = os.getenv("ROS_ROOT")+"/tools/rospack"
    unused_retcode = subprocess.call(["make", "clean"],cwd=rospack_dir)
    unused_retcode = subprocess.call(["make"],cwd=rospack_dir)
    
def protect_ros_core():
    unprotect_ros_core()
    make_rospack()
    ros_package_path = os.getenv('ROS_PACKAGE_PATH')
    os.environ['ROS_PACKAGE_PATH'] = os.getenv('ROS_ROOT')
    p = subprocess.Popen(["rosmake", "-ia", "--pre-clean", "--rosdep-install", "--rosdep-yes"])
    p.wait()
    os.environ['ROS_PACKAGE_PATH'] = ros_package_path

def unprotect_ros_core():
    ros_package_path = os.getenv('ROS_PACKAGE_PATH')
    os.environ['ROS_PACKAGE_PATH'] = os.getenv('ROS_ROOT')
    p = subprocess.Popen(["rosmake", "-u", "-a"])
    p.wait()
    os.environ['ROS_PACKAGE_PATH'] = ros_package_path

def protect_min():
    unprotect_ros_core()
    make_rospack()
    p = subprocess.Popen(["rosmake", "-i", "--pre-clean", "--rosdep-install", "--rosdep-yes"] + min_packages())
    p.wait()

def protect_main():
    from optparse import OptionParser
    usage = "Usage: %prog [options]"
    parser = OptionParser(usage=usage)
    parser.add_option("-r","--ros", action="store_true", dest="ros", default=False, help="Build/protect the entire ros core stack.")
    parser.add_option("-m","--minimal", action="store_true", dest="minimal", default=False, help="Only build/protect the minimal set of packages required for cross-compiling.")
    parser.add_option("-u","--unprotect", action="store_true", dest="unprotect", default=False, help="Unprotect the ros core stack.")
    options, args = parser.parse_args()
    
    ###################
    # Abort Checks
    ###################
    if args:
        parser.error("Only options are permitted, run with --help to see options.")

    ###################
    # Action
    ###################
    if ( options.unprotect ):
        unprotect_ros_core()
    if options.ros:
        protect_ros_core()
        sys.exit(0);
    elif options.minimal:
        protect_min()
        sys.exit(0);
    else:
        parser.error("No options supplied - run with --help to see options.")
        
if __name__ == "__main__":
    protect_main()
