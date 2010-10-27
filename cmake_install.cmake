# Install script for directory: /home/rsmits/ros/dual_arm_rtt

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/libdual_arm_rtt-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/libdual_arm_rtt-gnulinux.so"
         RPATH "/home/rsmits/ros/orocos_toolchain_ros/ocl/lib:/home/rsmits/ros/orocos_toolchain_ros/rtt/install/lib:/opt/ros/cturtle/ros/tools/rosrecord/lib:/opt/ros/cturtle/ros/tools/rosbag/lib:/opt/ros/cturtle/ros/tools/topic_tools/lib:/opt/ros/cturtle/ros/core/roscpp/lib:/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/lib:/opt/ros/cturtle/ros/core/rosconsole/lib:/opt/ros/cturtle/ros/core/roslib/lib:/opt/ros/cturtle/ros/tools/rospack/lib:/opt/ros/cturtle/ros/3rdparty/gtest/gtest/lib")
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/libdual_arm_rtt-gnulinux.so")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos" TYPE SHARED_LIBRARY FILES "/home/rsmits/ros/dual_arm_rtt/lib/orocos/libdual_arm_rtt-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/libdual_arm_rtt-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/libdual_arm_rtt-gnulinux.so"
         OLD_RPATH "/home/rsmits/ros/orocos_toolchain_ros/ocl/lib:/home/rsmits/ros/orocos_toolchain_ros/rtt/install/lib:/opt/ros/cturtle/ros/tools/rosrecord/lib:/opt/ros/cturtle/ros/tools/rosbag/lib:/opt/ros/cturtle/ros/tools/topic_tools/lib:/opt/ros/cturtle/ros/core/roscpp/lib:/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/lib:/opt/ros/cturtle/ros/core/rosconsole/lib:/opt/ros/cturtle/ros/core/roslib/lib:/opt/ros/cturtle/ros/tools/rospack/lib:/opt/ros/cturtle/ros/3rdparty/gtest/gtest/lib:"
         NEW_RPATH "/home/rsmits/ros/orocos_toolchain_ros/ocl/lib:/home/rsmits/ros/orocos_toolchain_ros/rtt/install/lib:/opt/ros/cturtle/ros/tools/rosrecord/lib:/opt/ros/cturtle/ros/tools/rosbag/lib:/opt/ros/cturtle/ros/tools/topic_tools/lib:/opt/ros/cturtle/ros/core/roscpp/lib:/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/lib:/opt/ros/cturtle/ros/core/rosconsole/lib:/opt/ros/cturtle/ros/core/roslib/lib:/opt/ros/cturtle/ros/tools/rospack/lib:/opt/ros/cturtle/ros/3rdparty/gtest/gtest/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/libdual_arm_rtt-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/libdual_arm_rtt-gnulinux.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/plugins/libdual_arm_rtt-service-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/plugins/libdual_arm_rtt-service-gnulinux.so"
         RPATH "/home/rsmits/ros/orocos_toolchain_ros/ocl/lib:/home/rsmits/ros/orocos_toolchain_ros/rtt/install/lib:/opt/ros/cturtle/ros/tools/rosrecord/lib:/opt/ros/cturtle/ros/tools/rosbag/lib:/opt/ros/cturtle/ros/tools/topic_tools/lib:/opt/ros/cturtle/ros/core/roscpp/lib:/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/lib:/opt/ros/cturtle/ros/core/rosconsole/lib:/opt/ros/cturtle/ros/core/roslib/lib:/opt/ros/cturtle/ros/tools/rospack/lib:/opt/ros/cturtle/ros/3rdparty/gtest/gtest/lib")
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/plugins/libdual_arm_rtt-service-gnulinux.so")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/plugins" TYPE SHARED_LIBRARY FILES "/home/rsmits/ros/dual_arm_rtt/lib/orocos/plugins/libdual_arm_rtt-service-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/plugins/libdual_arm_rtt-service-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/plugins/libdual_arm_rtt-service-gnulinux.so"
         OLD_RPATH "/home/rsmits/ros/orocos_toolchain_ros/ocl/lib:/home/rsmits/ros/orocos_toolchain_ros/rtt/install/lib:/opt/ros/cturtle/ros/tools/rosrecord/lib:/opt/ros/cturtle/ros/tools/rosbag/lib:/opt/ros/cturtle/ros/tools/topic_tools/lib:/opt/ros/cturtle/ros/core/roscpp/lib:/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/lib:/opt/ros/cturtle/ros/core/rosconsole/lib:/opt/ros/cturtle/ros/core/roslib/lib:/opt/ros/cturtle/ros/tools/rospack/lib:/opt/ros/cturtle/ros/3rdparty/gtest/gtest/lib:/home/rsmits/ros/dual_arm_rtt:"
         NEW_RPATH "/home/rsmits/ros/orocos_toolchain_ros/ocl/lib:/home/rsmits/ros/orocos_toolchain_ros/rtt/install/lib:/opt/ros/cturtle/ros/tools/rosrecord/lib:/opt/ros/cturtle/ros/tools/rosbag/lib:/opt/ros/cturtle/ros/tools/topic_tools/lib:/opt/ros/cturtle/ros/core/roscpp/lib:/opt/ros/cturtle/ros/3rdparty/xmlrpcpp/lib:/opt/ros/cturtle/ros/core/rosconsole/lib:/opt/ros/cturtle/ros/core/roslib/lib:/opt/ros/cturtle/ros/tools/rospack/lib:/opt/ros/cturtle/ros/3rdparty/gtest/gtest/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/plugins/libdual_arm_rtt-service-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/plugins/libdual_arm_rtt-service-gnulinux.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/dual_arm_rtt" TYPE FILE FILES "/home/rsmits/ros/dual_arm_rtt/dual_arm_rtt-component.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rsmits/ros/dual_arm_rtt/orocos-dual_arm_rtt-gnulinux.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/dual_arm_rtt" TYPE FILE FILES "/home/rsmits/ros/dual_arm_rtt/manifest.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/rsmits/ros/dual_arm_rtt/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/rsmits/ros/dual_arm_rtt/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
