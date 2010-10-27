#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)


#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
# The CMake 'target' names are identical to the first argument of the
# macro, except for orocos_typekit, where the target is fully
# controlled by generated code of 'typegen'.
#
project(dual_arm_rtt)
set(ROS_ROOT $ENV{ROS_ROOT})
if(ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package(rtt)
  message("rtt_path: ${rtt_PACKAGE_PATH}")
endif()
find_package(Orocos-RTT PATHS ${rtt_PACKAGE_PATH}/install)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.

# Creates a component library libdual_arm_rtt-<target>.so
# and installs in the directory lib/orocos/dual_arm_rtt/
#
orocos_component(dual_arm_rtt dual_arm_rtt-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Building a typekit (recommended):
#
# Creates a typekit library libdual_arm_rtt-types-<target>.so
# and installs in the directory lib/orocos/dual_arm_rtt/types/
#
#orocos_typekit(dual_arm_rtt-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typekit statement !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


#
# Building a Plugin or Service (optional):
#
# Creates a plugin library libdual_arm_rtt-service-<target>.so or libdual_arm_rtt-plugin-<target>.so
# and installs in the directory lib/orocos/dual_arm_rtt/plugins/
#
orocos_plugin(dual_arm_rtt-service CircleGeneratorService.cpp) # ...you may add multiple source files
#orocos_plugin(dual_arm_rtt-plugin dual_arm_rtt-plugin.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_plugin statements.


#
# Additional headers (not in typekit):
#
# Installs in the include/orocos/dual_arm_rtt/ directory
#
orocos_install_headers( dual_arm_rtt-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Generates and installs our orocos-dual_arm_rtt-<target>.pc and manifest.xml file
#
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/orocos-dual_arm_rtt.pc.in ${CMAKE_CURRENT_BINARY_DIR}/orocos-dual_arm_rtt-${OROCOS_TARGET}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/orocos-dual_arm_rtt-${OROCOS_TARGET}.pc DESTINATION  lib/pkgconfig )
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/manifest.xml DESTINATION  lib/orocos/dual_arm_rtt )

#
# Provided 'uninstall' target
#
#orocos_uninstall_target()
