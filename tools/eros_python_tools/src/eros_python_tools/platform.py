'''
Created on Sep 7, 2010

Handles management of ros' rosconfig.cmake with a particular focus to customising
for platform/cpu characteristics.

@author: Daniel Stonier
'''
###############################################################################
# Modules
###############################################################################

import os
import shutil
import sys
import roslib
import core

###############################################################################
# Objects
###############################################################################

class Platform:
    '''
    Represents data about a platform:
      pathname : full pathname to the platform
      name : usually the filename minus the extension e.g. core2
      group : valid groups are eros||user
      family : the architecture of cpu's to which it belongs
      current : true if it is the currently configured ros platform
    '''

    def __init__(self, platform_dir, platform_pathname, platform_id):
        # e.g. 
        #  platform_dir = /home/snorri/.ros/platforms
        #  platform_pathname = /home/snorri/.ros/platforms/arm/arm1176jzf-s.cmake
        self.pathname = platform_pathname
        # Could check here, but if calling from platform_list, will always be arg 1 we want.
        tail = self.pathname.split(platform_dir)[1] # e.g. /arm/arm1176jzf-s.cmake 
        cmake_name = os.path.basename(tail) # e.g. arm1176jzf-s.cmake
        self.name = os.path.splitext(cmake_name)[0] # e.g. arm1176jzf-s
        if ( self.pathname.find(eros_platform_dir()) != -1 ):
            self.group = "eros"
        else:
            self.group = "user"
        self.family = os.path.dirname(platform_pathname).split(platform_dir)[1] # e.g. /arm
        self.family = os.path.split(self.family)[1] # remove dir separators e.g. arm
        if ( self.family == '' ):
            self.family = "unknown"
        platform_exists = os.path.exists(core.rosconfig_cmake())
        self.current = False
        if ( platform_exists ) :
            name_string = "PLATFORM_NAME \"" + self.name + "\""
            if name_string in open(core.rosconfig_cmake()).read():
                family_string = "PLATFORM_FAMILY \"" + self.family + "\""
                if family_string in open(core.rosconfig_cmake()).read():
                    self.current = True
        self.id = platform_id
#        print "Pathname: " + self.pathname
#        print "Name " + self.name
#        print "Group: " + self.group
#        if ( self.current ):
#            print "Current: True"
#        else:
#            print "Current: False"
#        print "Id: " + repr(self.id)

###############################################################################
# Supporting Methods
###############################################################################

def eros_rosconfig_tail():
    return os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"templates","rosconfig.tail.cmake")

def eros_platform_template():
    return os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"templates","rosconfig.template.cmake")

def eros_platform_dir():
    return os.path.join(roslib.packages.get_pkg_dir('eros_platforms'),"library")

def user_platform_dir():
    return os.path.join(roslib.rosenv.get_ros_home(),"platforms")

platforms = []

def platform_list():
    """
    Both provides access to and populates the platform list for both eros and user-defined
    platform libraries.
    """
    global platforms
    if len(platforms) != 0 :
        return platforms
    # else populate from eros and user-defined libraries
    id_counter = 1
    for root, unused_dirs, files in os.walk(eros_platform_dir()):
        for name in files:
            if name.endswith('.cmake'):
                platforms += [Platform(eros_platform_dir(),os.path.join(root,name), id_counter)]
                id_counter += 1
    for root, unused_dirs, files in os.walk(user_platform_dir()):
        for name in files:
            if name.endswith('.cmake'):
                platforms += [Platform(user_platform_dir(),os.path.join(root,name), id_counter)]
                id_counter += 1
    return platforms

###############################################################################
# Primary Methods
###############################################################################

def show_current_platform():
    '''
    Print the identity of the currently configured platform:
      - checks eros/user platform libraries for a match
      - if not eros/user platform, checks if platform configured, but unknown
      - otherwise prints none 
    '''
    pretext = core.bold_string("Current platform: ")
    platforms = platform_list()
    found = False
    for platform in platforms:
        if ( platform.current ):
            found = True
            current_platform = platform
    if ( found ):
        print pretext + current_platform.family + os.sep + current_platform.name
    else:
        if ( os.path.exists(core.rosconfig_cmake()) ):
            print pretext + "unknown"
        else:
            print pretext + "none"
    
def list_eros_platforms():
    platforms = platform_list()
    print
    print core.bold_string("Eros platforms:")
    print
    for platform in platforms:
        if ( platform.group == 'eros' ):
            if ( platform.current ):
                print "  %d) %s%s%s%s" %(platform.id,platform.family,os.sep,platform.name,core.red_string("*"))
            else:
                print "  %d) %s%s%s" %(platform.id,platform.family,os.sep,platform.name)

def list_user_platforms():
    platforms = platform_list()
    print
    print core.bold_string("User platforms:")
    print
    for platform in platforms:
        if ( platform.group == 'user' ):
            if ( platform.current ):
                print "  %d) %s%s%s%s" %(platform.id,platform.family,os.sep,platform.name,core.red_string("*"))
            else:
                print "  %d) %s%s%s" %(platform.id,platform.family,os.sep,platform.name)

def list_platforms():
    list_eros_platforms()
    list_user_platforms()
    print
    
def select_platform():
    '''
    Interactively selects and sets an ros platform.
      - return 1 if failure, 0 if success
    '''
    list_platforms()
    platform_id_string = raw_input("Enter a platform id #: ")
    if ( not platform_id_string.isdigit() ):
        print core.red_string("Aborting, invalid id #.")
        return 1
    platform_id = int(platform_id_string)
    platforms = platform_list()
    if not platform_id <= len(platforms):
        print core.red_string("Aborting, invalid id #.")
        return 1
    found_platform = False
    for platform in platforms:
        if ( platform.id == platform_id ):
            found_platform = True
            selected_platform = platform
    if ( not found_platform ):
        print core.red_string("Aborting, platform not found.")
        return 1
    else:
        # Copy across the platform specific file and append the default section as well
        shutil.copyfile(selected_platform.pathname,core.rosconfig_cmake())
        rosconfig_tail = open(eros_rosconfig_tail()).read() 
        f = open(core.rosconfig_cmake(), 'a')
        f.write(rosconfig_tail)
        f.close()
        return 0

def delete_platform():
    '''
    Interactively deletes a user-defined platform.
      - return 1 if failure, 0 if success
    '''
    list_user_platforms()
    print
    platform_id_string = raw_input("Enter a platform id #: ")
    if ( not platform_id_string.isdigit() ):
        print core.red_string("Aborting, invalid id #.")
        return 1
    platform_id = int(platform_id_string)
    platforms = platform_list()
    if not platform_id <= len(platforms):
        print core.red_string("Aborting, invalid id #.")
        return 1
    found_platform = False
    for platform in platforms:
        if ( platform.id == platform_id ):
            if ( platform.group == 'user' ):
                found_platform = True
                selected_platform = platform
    if ( not found_platform ):
        print core.red_string("Aborting, invalid id #.") # probably passed an eros platform id
        return 1
    else:
        os.remove(selected_platform.pathname)
        return 0

def create_platform():
    print "Create a platform"
    

    
###############################################################################
# Main
###############################################################################

def main():
    from optparse import OptionParser
    usage = "\n\
  %prog          : shows the currently set ros platform\n\
  %prog clear    : clear the currently set ros platform\n\
  %prog create   : create a user-defined platform configuration\n\
  %prog delete   : delete a preconfigured platform\n\
  %prog list     : list available eros and user-defined platforms\n\
  %prog select   : select a preconfigured platform\n\
  %prog validate : attempt to validate a platform (not yet implemented)"
    parser = OptionParser(usage=usage)
    #parser.add_option("-v","--validate", action="store_true", dest="validate", help="when creating, attempt to validate the configuration")
    unused_options, args = parser.parse_args()
    
    ###################
    # Show current
    ###################
    if not args:
        show_current_platform()
        return 0

    command = args[0]
        
    ###################
    # List
    ###################
    if command == 'list':
        list_platforms()
        return 0
    
    ###################
    # Clear
    ###################
    if command == 'clear':
        if os.path.exists(core.rosconfig_cmake()):
            os.remove(core.rosconfig_cmake())
            print "Platform configuration cleared."
        else:
            print "Nothing to do (no $ROS_ROOT/rosconfig.cmake)."
        return 0
    ###################
    # Create
    ###################
    if command == 'create':
        return create_platform()

    ###################
    # Delete
    ###################
    if command == 'delete':
        return delete_platform()

    ###################
    # Select
    ###################
    if command == 'select': 
        if not select_platform():
            return 1
        print 
        return 0
    
    ###################
    # Validate
    ###################
    if command == 'validate':
        print
        print "This command is not yet available."
        print
        return 0 
    
    # If we reach here, we have not received a valid command.
    print "Not a valid command [" + command + "], rerun with --help to list valid commands"
    return 1

if __name__ == "__main__":
    sys.exit(main())
