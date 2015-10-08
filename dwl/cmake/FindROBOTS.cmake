# - Try to find all the robot generated code by RobCoGen tool
#
# This module assume that you install the robot code in /usr/local/include/iit/robots
#   find_package(ROBOTS)
#
# Once done this will define
#
#  robotname_FOUND - system has robotname lib
#  robotname_INCLUDE_DIR - the robotname include directory
#  robotname_LIBRARIES - the robotname libraries
#
# Copyright (c) 2015 Carlos Mastalli, <carlos.mastalli@iit.it>
# Redistribution and use is allowed according to the terms of the XXX license.


set(ROBOTS_DIR /usr/local/include/iit/robots)

macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
        list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()


subdirlist(ROBOTDIRS ${ROBOTS_DIR})
foreach(robot ${ROBOTDIRS})
	set(${robot}_INCLUDE_DIRS ${ROBOTS_DIR}/${robot})
	find_library(${robot}_LIBRARIES iitgen${robot} /usr/local/lib)

	include(FindPackageHandleStandardArgs)
	find_package_handle_standard_args(${robot} DEFAULT_MSG ${robot}_LIBRARIES ${robot}_INCLUDE_DIRS)
	mark_as_advanced(${robot}_INCLUDE_DIRS)

	string(TOUPPER ${robot} ROBOT)

	set(${robot}_FOUND ${${ROBOT}_FOUND})
endforeach()
