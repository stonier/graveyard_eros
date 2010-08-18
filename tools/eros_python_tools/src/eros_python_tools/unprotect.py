'''
Created on Aug 18, 2010

Reverses any protection thats been applied to packages in the ros core stack
(i.e. removes the ROS_NOBUILD markers.

@author: Daniel Stonier
'''

import os
import subprocess

def unprotect_ros_core():
    ''' Removes any ROS_NOBUILD flags from all ros core stack packages.'''
    ros_package_path = os.getenv('ROS_PACKAGE_PATH')
    os.environ['ROS_PACKAGE_PATH'] = os.getenv('ROS_ROOT')
    p = subprocess.Popen(["rosmake", "-u", "-a"])
    p.wait()
    os.environ['ROS_PACKAGE_PATH'] = ros_package_path

