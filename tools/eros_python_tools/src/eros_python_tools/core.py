'''
Created on 17/08/2010

Various common core functions used by the ecl, both generic and ros related.

@author: Daniel Stonier
'''

import os
import re
import sys
import subprocess

bold = "\033[1m"
reset = "\033[0;0m"
red = "\033[31m"

def print_error(msg):
    """print warning to screen (bold red)"""
    print >> sys.stderr, '\033[31m%s\033[0m'%msg

def ros_version():
    rosversion_exe = os.getenv("ROS_ROOT")+"/bin/rosversion"
    rosversion_exists = os.path.exists(rosversion_exe)
    if ( not rosversion_exists ):
        print "Could not find rosversion - make sure ROS_ROOT is set appropriately."
        sys.exit(1)
    version = subprocess.Popen([rosversion_exe, "ros"], stdout=subprocess.PIPE).communicate()[0]
    if ( re.match(r"1\.0.*", version) ): # Boxturtle
        version = "boxturtle"
    elif ( re.match(r"1\.2.*", version) ): # Cturtle
        version = "cturtle"
    else: 
        version = "unknown"
    return version
