# - Try to find the urdf dependencies
#
# This module assume that you install the robot code in /usr/include or /usr/local/include
#   find_package(URDF)
#
# Once done this will define
#
#  URDF_FOUND - system has found urdf library
#  URDF_INCLUDE_DIR - the urdf include directory
#  URDF_LIBRARIES - the urdf libraries
#
# Copyright (c) 2015 Carlos Mastalli, <carlos.mastalli@iit.it>
# Redistribution and use is allowed according to the terms of the XXX license.

set(URDF_FOUND FALSE)
set(CONSOLE_BRIDGE_FOUND FALSE)
set(URDFDOM_HEADERS_FOUND FALSE)
set(URDFDOM_FOUND FALSE)

find_path(CONSOLE_BRIDGE_DIR console_bridge/console.h
	/usr/local/include
	/usr/include)

find_library(CONSOLE_BRIDGE_LIBRARY NAMES console_bridge PATHS
	/usr/local/include
	/usr/include)

find_path(URDFDOM_HEADERS_DIR urdf_model/model.h
	/usr/local/include
	/usr/include)

find_path(URDFDOM_DIR urdf_parser/urdf_parser.h
	/usr/local/include
	/usr/include)

find_library(URDFDOM_MODEL_LIBRARY NAMES urdfdom_model PATHS
	/usr/local/include
	/usr/include)

find_library(URDFDOM_WORLD_LIBRARY NAMES urdfdom_world PATHS
 	/usr/local/include
 	/usr/include)

if(NOT CONSOLE_BRIDGE_DIR OR NOT CONSOLE_BRIDGE_LIBRARY)
	message(ERROR "Could not find URDF: console_bridge not found")
else(NOT CONSOLE_BRIDGE_DIR OR NOT CONSOLE_BRIDGE_LIBRARY)
	set(CONSOLE_BRIDGE_FOUND TRUE)
endif(NOT CONSOLE_BRIDGE_DIR OR NOT CONSOLE_BRIDGE_LIBRARY)

if(NOT URDFDOM_HEADERS_DIR)
	message(ERROR "Could not find URDF: urdfdom_headers not found")
else(NOT URDFDOM_HEADERS_DIR)
	set(URDFDOM_HEADERS_FOUND TRUE)
endif(NOT URDFDOM_HEADERS_DIR)

if(NOT URDFDOM_DIR OR NOT URDFDOM_MODEL_LIBRARY OR NOT URDFDOM_WORLD_LIBRARY)
	message(ERROR "Could not find URDF: urdfdom_model or urdfdom_world library not found")
else(NOT URDFDOM_DIR OR NOT URDFDOM_MODEL_LIBRARY OR NOT URDFDOM_WORLD_LIBRARY)
	set(URDFDOM_FOUND TRUE)
endif(NOT URDFDOM_DIR OR NOT URDFDOM_MODEL_LIBRARY OR NOT URDFDOM_WORLD_LIBRARY)

if(CONSOLE_BRIDGE_FOUND AND URDFDOM_HEADERS_FOUND AND URDFDOM_FOUND)
	set(URDF_FOUND TRUE)
endif(CONSOLE_BRIDGE_FOUND AND URDFDOM_HEADERS_FOUND AND URDFDOM_FOUND)

if(URDF_FOUND)
   if(NOT URDF_FIND_QUIETLY)
		 message(STATUS "Found URDF console_bridge: ${CONSOLE_BRIDGE_LIBRARY}")
		 message(STATUS "Found URDF urdfdom_headers: ${URDFDOM_HEADERS_DIR}")
		 message(STATUS "Found URDF urdfdom: ${URDFDOM_LIBRARY}")
  endif(NOT URDF_FIND_QUIETLY)
	set(URDF_INCLUDE_DIRS
		${CONSOLE_BRIDGE_DIR}
		${URDFDOM_HEADERS_DIR}
		${URDFDOM_DIR})
	set(URDF_LIBRARIES
		${CONSOLE_BRIDGE_LIBRARY}
		${URDFDOM_WORLD_LIBRARY}
		${URDFDOM_MODEL_LIBRARY})
	message(STATUS "URDF Libraries: ${URDF_LIBRARIES}")
else(URDF_FOUND)
   if(URDF_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find URDF")
   endif(URDF_FIND_REQUIRED)
endif(URDF_FOUND)

mark_as_advanced(${URDF_INCLUDE_DIRS} ${URDF_LIBRARIES})
