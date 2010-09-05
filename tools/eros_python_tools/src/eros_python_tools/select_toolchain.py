'''
Created on 17/08/2010

This manages the installation of a toolchain cmake module from the toolchain library (eros_toolchains) 
in ${ROS_ROOT}/rostoolchain.cmake. It also ensures an appropriate platform is set and configured. 

@author: Daniel Stonier
'''
import roslib
roslib.load_manifest('eros_python_tools')

import os
import sys
import core
import shutil

class Toolchain:
    '''
    Represents data about a toolchain:
      pathname : full pathname to the toolchain
      name : typically the toolchain tuple, e.g. i686-pc-linux-gnu
      group : valid groups are eros||user
      family : which family the toolchain belongs, e.g. crossdev||ubuntu||...
      current : true if it is the currently configured ros toolchain
    '''

    def __init__(self, toolchain_dir, toolchain_pathname):
        # e.g. 
        #  toolchain_dir = /home/snorri/.ros/toolchains
        #  toolchain_pathname = /home/snorri/.ros/toolchains/crossdev/i686-pc-linux-gnu.cmake
        self.pathname = toolchain_pathname
        # Could check here, but if calling from toolchain_list, will always be arg 1 we want.
        tail = self.pathname.split(toolchain_dir)[1] # e.g. /crossdev/i686-pc-linux-gnu.cmake 
        cmake_name = os.path.basename(tail) # e.g. i686-pc-linux-gnu.cmake
        self.name = os.path.splitext(cmake_name)[0] # e.g. i686-pc-linux-gnu
        if ( self.pathname.find(eros_toolchain_dir()) != -1 ):
            self.group = "eros"
        else:
            self.group = "user"
        self.family = os.path.dirname(toolchain_pathname).split(toolchain_dir)[1] # e.g. /crossdev
        self.family = os.path.split(self.family)[1] # remove dir separators e.g. crossdev
        if ( self.family == '' ):
            self.family = "unknown"
        toolchain_exists = os.path.exists(core.rostoolchain_cmake())
        if ( toolchain_exists ) :
            self.current = self.name in open(core.rostoolchain_cmake()).read()
        else:
            self.current = False
        self.id = Toolchain.id_counter
        Toolchain.id_counter += 1
#        print "Pathname: " + self.pathname
#        print "Name: " + self.name
#        print "Group: " + self.group
#        print "Family: " + self.family
#        print "Current: " + self.current
#        print "Id: " + self.id

    # Python attribute (aka static variable)            
    id_counter = 1
    
def eros_toolchain_dir():
    return os.path.join(roslib.packages.get_pkg_dir('eros_toolchains'),"library")

def user_toolchain_dir():
    return os.path.join(roslib.rosenv.get_ros_home(),"toolchains")

def toolchain_list():
    toolchains = []
    for root, unused_dirs, files in os.walk(eros_toolchain_dir()):
        for name in files:
            if name.endswith('.cmake'):
                toolchains += [Toolchain(eros_toolchain_dir(),os.path.join(root,name))]
    for root, unused_dirs, files in os.walk(user_toolchain_dir()):
        for name in files:
            if name.endswith('.cmake'):
                toolchains += [Toolchain(user_toolchain_dir(),os.path.join(root,name))]
    return toolchains 
    #return [files for root, dirs, files in os.walk(eros_toolchain_dir()) if f.endswith('.cmake')]
    

def list_toolchains():
    toolchains = toolchain_list()
    print
    print core.bold_string("Eros toolchains:")
    print
    for toolchain in toolchains:
        if ( toolchain.group == 'eros' ):
            if ( toolchain.current ):
                print "  %d) %s->%s%s" %(toolchain.id,toolchain.family,toolchain.name,core.red_string("*"))
            else:
                print "  %d) %s->%s" %(toolchain.id,toolchain.family,toolchain.name)
    print
    print core.bold_string("User toolchains:")
    print
    for toolchain in toolchains:
        if ( toolchain.group == 'user' ):
            print "  " + toolchain.family + "->" + toolchain.name
    
def show_current_toolchain():
    '''
    Print the identity of the currently configured toolchain:
      - checks eros/user toolchain libraries for a match
      - if not eros/user toolchain, checks if toolchain configured, but unknown
      - otherwise prints none 
    '''
    pretext = core.bold_string("Current toolchain: ")
    toolchains = toolchain_list()
    found = False
    for toolchain in toolchains:
        if ( toolchain.current ):
            found = True
            current_toolchain = toolchain
    if ( found ):
        print pretext + current_toolchain.name
    else:
        if ( os.path.exists(core.rostoolchain_cmake()) ):
            print pretext + "unknown"
        else:
            print pretext + "none"

def select_toolchain(toolchain_id):
    '''
    Selects and configures the specified ros toolchain.
      - return true or false depending on success/failure.
    '''
    toolchains = toolchain_list()
    found_toolchain = False
    for toolchain in toolchains:
        if ( toolchain_id.isdigit() ):
            if ( toolchain.id == int(toolchain_id) ):
                found_toolchain = True
                selected_toolchain = toolchain
        elif ( toolchain.name == toolchain_id ):
            found_toolchain = True
            selected_toolchain = toolchain
    if ( not found_toolchain ):
        return False
    else:
        shutil.copyfile(selected_toolchain.pathname,core.rostoolchain_cmake())
        return True

def check_platform():
    rosconfig_exists = os.path.exists(core.rosconfig_cmake())
    if rosconfig_exists:
        print "Remember to confirm that the existing platform (rosconfig.cmake) is compatible."
        print
    else:
        print "As yet no rosconfig.cmake exists -> generating a blank (Vanilla) template."
        #select_platform.select_platform("Vanilla",None)

def patch_ros():
    version = core.ros_version()
    if ( version == 'cturtle' ):
        print "Applying various patches for cturtle"
    else:
        print "At this point in time, only cturtle is supported for patching."  

def main():
    from optparse import OptionParser
    usage = "\n\
  %prog         : shows currently configured toolchain\n\
  %prog --clear : clears the currently configured toolchain\n\
  %prog --list  : lists available toolchains\n\
  %prog <id>    : select toolchain with id # (use with --list)\n\
  %prog <tuple> : select toolchain with specified tuple (use with --list)"
    parser = OptionParser(usage=usage)
    parser.add_option("-l","--list", action="store_true", dest="list", help="List available toolchains.")
    parser.add_option("-c","--clear", action="store_true", dest="clear", help="Clear current toolchain configuration.")
    options, args = parser.parse_args()
    
    ###################
    # List
    ###################
    if options.list:
        list_toolchains()
        return 0
    ###################
    # Clear
    ###################
    if options.clear:
        if os.path.exists(core.rostoolchain_cmake()):
            os.remove(core.rostoolchain_cmake())
            print "Toolchain configuration cleared - remember to reconfigure ROS_BOOST_ROOT if necessary."
        else:
            print "Nothing to do (no toolchain configuration present)."
        return 0
    ###################
    # Show current
    ###################
    if not args:
        show_current_toolchain()
        return 0

    ###################
    # Select
    ###################
    toolchain = args[0]
    if ( not select_toolchain(toolchain) ):
        print "The specified toolchain [%s] is not available." %toolchain
        print "  To see available toolchains, supply the --list argument."
        return 1
    # Not currently needing it, but anyway, its good to have.
    # check_platform()
    # patch_ros()
    
    print ""
    print "If doing a full cross, or using boost in a partial cross, ensure ROS_BOOST_ROOT is exported from your shell environment."
    print ""
    return 0

if __name__ == "__main__":
    sys.exit(main())
