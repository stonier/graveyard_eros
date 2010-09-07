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

def unprotect_ros_core():
    ''' Removes any ROS_NOBUILD flags from all ros core stack packages.'''
    ros_package_path = os.getenv('ROS_PACKAGE_PATH')
    os.environ['ROS_PACKAGE_PATH'] = os.getenv('ROS_ROOT')
    p = subprocess.Popen(["rosmake", "-u", "-a"])
    p.wait()
    os.environ['ROS_PACKAGE_PATH'] = ros_package_path

def protect_minimal():
    ''' Cleans, then recompiles a minimal set of ros core packages, finally adds ROS_NOBUILD to all packages.'''
    unprotect_ros_core()
    make_rospack()
    p = subprocess.Popen(["rosmake", "-i", "--pre-clean", "--rosdep-install", "--rosdep-yes"] + min_packages())
    p.wait()

def main():
    ''' Main entry point for creating an rosprotect script.'''
    from optparse import OptionParser
    usage = "Usage: %prog [options]\n\n\
Description:\n\
  Cleans, builds and then protects the entire core ros stack. Use with --minimal if you wish\n\
  to rosprotect only a minimal set of packages necessary for a cross-compile environment.\n\
  You can reverse the process with --unprotect." 
    parser = OptionParser(usage=usage)
    parser.add_option("-m","--minimal", action="store_true", dest="minimal", default=False, help="build/protect only a minimal set of ros core packages necessary for a cross-compile environment.")
    parser.add_option("-u","--unprotect", action="store_true", dest="unprotect", default=False, help="unprotect the ros core stack.")
    options, unused_args = parser.parse_args()
    
    if options.unprotect :
        unprotect_ros_core()
        return 0
    if options.minimal :
        protect_minimal()
        return 0
    # Else, do the whole stack.
    protect_ros_core()
    return 0
        
if __name__ == "__main__":
    sys.exit(main())
