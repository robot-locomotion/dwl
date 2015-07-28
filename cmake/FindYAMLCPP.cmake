# - Try to find yaml-cpp dependencies
#
# This module find yaml-cpp, e.g. you can do
#   find_package(YAMLCPP)
#
# Once done this will define
#  YAMLCPP_FOUND - system has found yaml-cpp library
#  YAMLCPP_INCLUDE_DIRS - the yaml-cpp include directory
#  YAMLCPP_LIBRARIES - the yaml-cpp libraries
#
# Copyright (c) 2015 Carlos Mastalli, <carlos.mastalli@iit.it>
# Redistribution and use is allowed according to the terms of the XXX license.


# attempt to find static library first if this is set
if(YAMLCPP_STATIC_LIBRARY)
	set(YAMLCPP_STATIC libyaml-cpp.a)
endif()

# find the yaml-cpp include directory
find_path(YAMLCPP_INCLUDE_DIRS  yaml-cpp/yaml.h
							    PATH_SUFFIXES include
							    PATHS
							    /usr/local/include/)

# find the yaml-cpp library
find_library(YAMLCPP_LIBRARIES  NAMES ${YAMLCPP_STATIC} yaml-cpp
							    /usr/local)

# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(YAMLCPP DEFAULT_MSG YAMLCPP_INCLUDE_DIRS YAMLCPP_LIBRARIES)

mark_as_advanced(YAMLCPP_INCLUDE_DIRS YAMLCPP_LIBRARIES)
