'''
Created on 17/08/2010

This manages the installation of a toolchain cmake module from the toolchain library (eros_toolchains) 
in ${ROS_ROOT}/rostoolchain.cmake. It also ensures an appropriate platform is set and configured. 

@author: Daniel Stonier
'''
###############################################################################
# Imports
###############################################################################

import roslib
import os
import sys
import core
import shutil
import platform

###############################################################################
# Interface [Toolchain]
###############################################################################

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
        self.current = False
        if ( toolchain_exists ) :
            name_string = "TOOLCHAIN_TUPLE \"" + self.name + "\""
            if name_string in open(core.rostoolchain_cmake()).read():
                family_string = "TOOLCHAIN_FAMILY \"" + self.family + "\""
                if family_string in open(core.rostoolchain_cmake()).read():
                    self.current = True
        self.id = Toolchain.id_counter
        Toolchain.id_counter += 1
#        print "Pathname: " + self.pathname
#        print "Tuple " + self.name
#        print "Group: " + self.group
#        print "Family: " + self.family
#        if ( self.current ):
#            print "Current: True"
#        else:
#            print "Current: False"
#        print "Id: " + repr(self.id)

    # Python attribute (aka static variable)            
    id_counter = 1

###############################################################################
# Methods
###############################################################################
    
def eros_toolchain_template():
    return os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"templates","rostoolchain.template.cmake")
    
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
                print "  %d) %s%s%s%s" %(toolchain.id,toolchain.family,os.sep,toolchain.name,core.red_string("*"))
            else:
                print "  %d) %s%s%s" %(toolchain.id,toolchain.family,os.sep,toolchain.name)
    print
    print core.bold_string("User toolchains:")
    print
    for toolchain in toolchains:
        if ( toolchain.group == 'user' ):
            if ( toolchain.current ):
                print "  %d) %s%s%s%s" %(toolchain.id,toolchain.family,os.sep,toolchain.name,core.red_string("*"))
            else:
                print "  %d) %s%s%s" %(toolchain.id,toolchain.family,os.sep,toolchain.name)
    print
    
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

def create_toolchain():
    '''
    Create a cross-compiler cmake configuration. Note: hardwired for gcc
    cross compiler configurations at this point in time - is there even a use
    case that is different right now?
    '''
    print
    print core.bold_string("  Creating a User-Defined Eros Toolchain")
    print
    print "This is a interactive assistant to help define a new eros style cmake toolchain."
    print "It will prompt you for a few custom strings and then save the configured toolchain"
    print "in ROS_HOME/toolchains (~/.ros/toolchains) on linux platforms). It can then be"
    print "listed and selected in the same way as as a regular eros toolchain."
    print
    print core.bold_string("  Toolchain Family")
    print 
    print "  This is simply a convenience variable that helps sort toolchains in the eros"
    print "  user-defined toolchain libraries. Common examples include: crossdev, ubuntu,"
    print "  openembedded, etc."
    print
    toolchain_family = raw_input('  Enter a string for the toolchain family [custom]: ')
    if ( toolchain_family == '' ):
        toolchain_family = 'custom'
    print
    print core.bold_string("  Toolchain Tuple")
    print 
    print "  This is essential so that cmake can find the gcc cross-compiler. The toolchain"
    print "  tuple should match the prefix to your toolchain's cross-compilers, e.g. if your"
    print "  cross-compiler is i686-pc-linux-gnu-gcc, then the tuple is i686-pc-linux-gnu."
    print "  The c/c++ cross-compilers should be in your PATH, this script will attempt"
    print "  to validate that it can find them before continuing."
    print
    toolchain_tuple = raw_input('  Enter a string for the toolchain tuple: ')
    if ( platform.system == 'Windows' ):
        toolchain_gcc = toolchain_tuple + "-gcc.exe"
        toolchain_gpp = toolchain_tuple + "-g++.exe"
    else:
        toolchain_gcc = toolchain_tuple + "-gcc"
        toolchain_gpp = toolchain_tuple + "-g++"
    toolchain_gcc_found = False 
    toolchain_gpp_found = False 
    path = os.environ['PATH']
    paths = path.split(os.pathsep)
    for dir in paths:
        if ( os.path.isdir(dir) ):
            pathname = os.path.join(dir,toolchain_gcc)
            if ( os.path.isfile(pathname) ):
                toolchain_gcc_pathname = pathname
                toolchain_gcc_found = True
            pathname = os.path.join(dir,toolchain_gpp)
            if ( os.path.isfile(pathname) ):
                toolchain_gpp_pathname = pathname
                toolchain_gpp_found = True
    if ( ( not toolchain_gcc_found ) or ( not toolchain_gpp_found ) ):
        print core.red_string("  Aborting, cross compiler binaries not found in your path.")
        return 1
    print
    print "  Compilers validated: "
    print "    gcc : " + toolchain_gcc_pathname
    print "    gpp : " + toolchain_gpp_pathname
    print
    print core.bold_string("  Toolchain Sysroot")
    print 
    print "  This is the root directory from which system headers and libraries can be found."
    print "  e.g. if toolchain pthreads header is /usr/i686-pc-linux-gnu/usr/include/pthread.h"
    print "  then your sysroot would be /usr/i686-pc-linux-gnu."
    print 
    toolchain_sysroot_default = "/usr/" + toolchain_tuple
    toolchain_sysroot = raw_input("  Enter a string for the toolchain sysroot [" + toolchain_sysroot_default + "]: ")
    if ( toolchain_sysroot == ''):
        toolchain_sysroot = toolchain_sysroot_default
    print
    # Should really validate somehow here 
    
    toolchain_template = open(eros_toolchain_template()).read()
    toolchain_template = toolchain_template.replace('${toolchain_family}',toolchain_family)
    toolchain_template = toolchain_template.replace('${toolchain_tuple}',toolchain_tuple)
    toolchain_template = toolchain_template.replace('${toolchain_sysroot}',toolchain_sysroot)
    #print toolchain_template
    user_defined_toolchain_pathname = os.path.join(user_toolchain_dir(),toolchain_family,toolchain_tuple+'.cmake')
    if ( os.path.exists(user_defined_toolchain_pathname) ):
        print core.red_string("  Aborting, this toolchain configuration already exists (use --delete to remove).")
        return 1
    if not os.path.exists( os.path.dirname(user_defined_toolchain_pathname) ): # Make sure the dir exists before we open for writing
        os.makedirs(os.path.dirname(user_defined_toolchain_pathname))
    f = open(user_defined_toolchain_pathname, 'w')
    f.write(toolchain_template)
    f.close()
    print "Toolchain " + toolchain_tuple + " saved to " + user_defined_toolchain_pathname 

    

    
    print "Toolchain family: %s" %toolchain_family
    print "Toolchain tuple: %s" %toolchain_tuple
    print "Toolchain sysroot: %s" %toolchain_sysroot
    
    
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

###############################################################################
# Main
###############################################################################

def main():
    from optparse import OptionParser
    usage = "\n\
  %prog [option] : call with the specified option [see below]\n\
  %prog          : shows currently configured toolchain\n\
  %prog <id>     : select toolchain with id # (use with --list)\n\
  %prog <tuple>  : select toolchain with specified tuple (use with --list)"
    parser = OptionParser(usage=usage)
    parser.add_option("-a","--create", action="store_true", dest="create", help="interactively create a new user-defined toolchain")
    parser.add_option("-c","--clear", action="store_true", dest="clear", help="clear the current toolchain configuration")
    parser.add_option("-l","--list", action="store_true", dest="list", help="list available eros and user-defined toolchains")
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
    # Create
    ###################
    if options.create:
        return create_toolchain()
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
    print 
    print "If doing a full cross, or using boost in a partial cross, ensure ROS_BOOST_ROOT"
    print "is exported from your shell environment."
    print 
    return 0

if __name__ == "__main__":
    sys.exit(main())
