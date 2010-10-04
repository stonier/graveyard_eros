###############################################################################
# Build utilities
###############################################################################

###############################
# Download
###############################
# Adds a custom command for downloading a url to a 
# particular file.
#
# Depends : -
# Outputs : ${file}
#
# Note: ${file} can be 
# - relative in which case it is rel to ${CMAKE_BINARY_DIR}
# - absolute (e.g. ${CMAKE_BINARY_DIR}/dude.tar.gz
#
# Example:
#
#   TARBALL=dude.tar.gz
#   URL=http://snorriheim.dnsdojo.com/tmp/${TARBALL}
#   eros_download(${TARBALL} ${URL})
#
macro(eros_download url file)
    add_custom_command(OUTPUT ${file}
        COMMAND wget ${url} -O ${file}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Downloading ${url}->${file}."
        )
endmacro()

###############################
# Extract
###############################
# Adds a custom command for untar'ing a tarball in the specified dir.
#
# Depends : ${tarball}
# Outputs : ${dir}/extracted
#
# 
# Example:
# 
#   TARBALL=${CMAKE_BINARY_DIR}/dude.tar.gz
#   URL=http://snorriheim.dnsdojo.com/tmp/${TARBALL}
#   eros_download(${TARBALL} ${URL})
#   eros_extract_tarball(${TARBALL} ${CMAKE_BINARY_DIR}/fakeroot) 
#
macro(eros_extract_tarball tarball dir)
    add_custom_command(OUTPUT ${dir}/extracted
        COMMAND mkdir -p ${dir}
        COMMAND tar -xvzf ${tarball} -C ${dir}
        COMMAND touch ${dir}/extracted
        DEPENDS ${tarball}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Extracting ${tarball} -> ${dir}."
        VERBATIM
        )
endmacro()

# Similar to the untarball command, but for bzips.
#
macro(eros_extract_bzip2 bzip2 dir)
    add_custom_command(OUTPUT ${dir}/extracted
        COMMAND tar -xvjf ${bzip2} -C ${dir}
        COMMAND touch ${dir}/extracted
        DEPENDS ${bzip2}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Extracting ${bzip2} -> ${dir}."
        VERBATIM
        )
endmacro()

###############################
# Autotools Compile
###############################
# This adds a custom command for a typical autotools compile.
# Be sure to set up the configure command correctly before calling.
#
# Depends: ${depends} (input argument)
# Outputs: ${dir}/compiled
#
macro(eros_autotools_compile configure_command dir depends)
    add_custom_command(OUTPUT ${dir}/compiled
        COMMAND ${configure_command}
        COMMAND make $ENV{ROS_PARALLEL_JOBS} # If not ros, then that is just empty anyway
        COMMAND make install
        COMMAND touch ${dir}/compiled
        DEPENDS ${depends}
        WORKING_DIRECTORY ${dir}
        COMMENT "Compiling ${dir}."
        VERBATIM
        )
endmacro()

###############################
# Uninstall
###############################
# Creates an uninstall target.
# 
# To do this, it needs to find the uninstall template and prep it.
# In ros, we can grab it from eros_build, otherwise we use pkgconfig
# to find it from an already installed ecl.
macro(eros_add_uninstall_target)

    # Find the template
    rosbuild_find_ros_package(eros_build)
    set(EROS_CMAKE_TEMPLATES_PATH ${eros_build_PACKAGE_PATH}/cmake/templates)

    # Prep it
    configure_file(
        "${EROS_CMAKE_TEMPLATES_PATH}/uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
        IMMEDIATE @ONLY)

    # Add the target
    add_custom_target(uninstall
        "${CMAKE_COMMAND}" -P
        "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake")
endmacro()
