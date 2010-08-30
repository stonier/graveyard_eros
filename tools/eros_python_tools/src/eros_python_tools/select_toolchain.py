'''
Created on 17/08/2010

This manages the installation of a toolchain cmake module from the toolchain library (eros_toolchains) 
in ${ROS_ROOT}/rostoolchain.cmake. It also ensures an appropriate platform is set and configured. 

@author: Daniel Stonier
'''

import os
import sys
import roslib

def rostoolchain_cmake():
    return os.path.join(os.getenv("ROS_ROOT"),"rostoolchain.cmake")

def eros_toolchain_dir():
    return os.path.join(roslib.packages.get_pkg_dir('eros_toolchains'),"library")

def user_toolchain_dir():
    return os.path.join(roslib.rosenv.get_ros_home(),"toolchains")

def toolchain_list(dir):
    toolchains = []
    for root, unused_dirs, files in os.walk(dir):
        for name in files:
            if name.endswith('.cmake'):
                toolchains += [os.path.join(root,name)]
    return toolchains 
    #return [files for root, dirs, files in os.walk(eros_toolchain_dir()) if f.endswith('.cmake')]
    
def check_platform():
    print "Check Platform"
    
def patch_ros():
    print "Patch ros"

   
def list_toolchains():
    print
    print "Eros toolchains:"
    print
    toolchain_exists = os.path.exists(rostoolchain_cmake())
    user_toolchains = toolchain_list(user_toolchain_dir())
    print
    print "User toolchains:"
    print
    
    for toolchain in toolchain_list(eros_toolchain_dir():
        print toolchain                                    
#        toolchain_name = os.path.splitext(toolchain)[0]
#        current = False
#        if toolchain_exists:
#            current = toolchain_name in open(file).read()
#        if current:
#            print "  - %s\033[31m*\033[0m"%toolchain_name
#        else:
#            print "  - %s"%toolchain_name
#    print
    
def select_toolchain(toolchain):
    print "Select toolchain %s",toolchain

def select_toolchain_main():
    from optparse import OptionParser
    usage = "Usage: [options] %prog <toolchain_name>"
    parser = OptionParser(usage=usage)
    parser.add_option("-l","--list", action="store_true", dest="list", help="List available toolchains.")
    parser.add_option("-c","--clear", action="store_true", dest="clear", help="Clear current toolchain configuration.")
    options, args = parser.parse_args()
    
    ###################
    # Abort Checks
    ###################
    if options.list:
        list_toolchains()
        sys.exit(0)
    if options.clear:
        if os.path.exists(rostoolchain_cmake()):
            os.remove(rostoolchain_cmake())
            print "Toolchain configuration cleared."
        else:
            print "Aborting, no toolchain configuration present."
        sys.exit(0)
    if not args:
        parser.error("You must specify either '--list', '--clear' or a toolchain name to select.")

    toolchain = args[0]
    select_toolchain(toolchain)
    # Not currently needing it, but anyway, its good to have.
    check_platform()
    patch_ros()
    
    print ""
    print "If doing a full cross, or using boost in a partial cross, ensure ROS_BOOST_ROOT is exported from your shell environment."
    print ""

if __name__ == "__main__":
    select_toolchain_main()
