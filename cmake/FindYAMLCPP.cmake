# - Try to find yaml/cpp lib
#
# This module find yaml-cpp, e.g. you can do
#   find_package(YAMLCPP)
#
# This module defines
# YAMLCPP_FOUND, if false, do not try to link to yaml-cpp
# YAMLCPP_LIBRARIES, where to find yaml-cpp
# YAMLCPP_INCLUDE_DIRS, where to find yaml.h
#
# By default, the dynamic libraries of yaml-cpp will be found. To find the static ones instead,
# you must set the YAMLCPP_STATIC_LIBRARY variable to TRUE before calling find_package(YamlCpp ...).
#
# If yaml-cpp is not installed in a standard path, you can use the YAMLCPP_DIR CMake variable
# to tell CMake where yaml-cpp is.

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
