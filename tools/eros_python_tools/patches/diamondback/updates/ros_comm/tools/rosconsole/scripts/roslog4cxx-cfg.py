#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import os
#import string
#from glob import glob
#import subprocess
#import platform
#from optparse import OptionParser
#from cStringIO import StringIO
#import tempfile
import roslib

def print_usage_and_exit():
  print >> sys.stderr, "Usage: roslog4cxx-cfg --cflags"
  # print >> sys.stderr, "       roslog4cxx-cfg --lflags"
  print >> sys.stderr, "       roslog4cxx-cfg --libs"
  # print >> sys.stderr, "       roslog4cxx-cfg --include_dirs"
  # print >> sys.stderr, "       roslog4cxx-cfg --lib_dirs"
  sys.exit(1)

def check_for_toolchain_tuple():
    if ( not os.path.exists(os.path.join(roslib.rosenv.get_ros_root(),"rostoolchain.cmake"))):
        return ''
    cmake_script = tempfile.NamedTemporaryFile(mode='w+t')
    cmake_script.write("include($ENV{ROS_ROOT}/rostoolchain.cmake)\n")
    cmake_script.write("message(${TOOLCHAIN_TUPLE})\n")
    cmake_script.seek(0)
    result = subprocess.Popen(['cmake', '-P', cmake_script.name], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
    # print "Result: " + result[0].rstrip()
    # print "Error: " + result[1].rstrip()
    cmake_script.close()
    # no idea why cmake makes this come out on stderr, but its fine
    return result[1].rstrip()

def main():
    if (len(sys.argv) < 2):
      print_usage_and_exit()
    
    parser = OptionParser()
    parser.add_option("-l", "--libs", dest="libs", type="string", help="")
    parser.add_option("-c", "--cflags", dest="cflags", action="store_true", default=False, help="")
    # parser.add_option("-i", "--include_dirs", dest="include_dirs", action="store_true", default=False, help="")
    # parser.add_option("-d", "--lib_dirs", dest="lib_dirs", action="store_true", help="")
    # parser.add_option("-f", "--lflags", dest="lflags", type="string", help="")
    
    (options, args) = parser.parse_args()
    
              
    ver = find_boost(search_paths(options.sysroot))
    if (ver is None):
        raise BoostError("Cannot find boost in any of %s"%search_paths(options.sysroot))
        sys.exit(0)
    
    if (options.version):
        check_one_option(options, 'version')
        print '%s.%s.%s root=%s include_dir=%s'%(ver.major, ver.minor, ver.patch, ver.root, ver.include_dir)
        return
    
    if (ver.major < 1 or (ver.major == 1 and ver.minor < 37)):
        raise BoostError('Boost version %s.%s.%s does not meet the minimum requirements of boost 1.37.0'%(ver.major, ver.minor, ver.patch))

    sysroot = check_for_toolchain_sysroot()
    
    output = ""
    if (options.root):
        check_one_option(options, 'root')
        output = ver.root
    elif (options.libs):
        check_one_option(options, 'libs')
        output = libs(ver, options.libs.split(','))
    elif (options.include_dirs):
        check_one_option(options, 'include_dirs')
        output = include_dirs(ver)
    elif (options.lib_dirs):
        check_one_option(options, 'lib_dirs')
        output = lib_dirs(ver)
    elif (options.cflags):
        check_one_option(options, 'cflags')
        output = cflags(ver)
    elif (options.lflags):
        check_one_option(options, 'lflags')
        output = lflags(ver, options.lflags.split(','))
    else:
        print_usage_and_exit()
    
    print output.strip()

if __name__ == "__main__":
    main()
