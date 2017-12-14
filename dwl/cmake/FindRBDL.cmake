# - Try to find rbdl dependencies
#
# This module find rbdl, e.g. you can do
#   find_package(RBDL)
#
# Once done this will define
#  RBDL_FOUND - system has found rbdl library
#  RBDL_INCLUDE_DIRS - the rbdl include directory
#  RBDL_LIBRARIES - the rbdl libraries
#
# Copyright (c) 2014-2018 Carlos Mastalli, <carlos.mastalli@laas.fr>
# Redistribution and use is allowed according to the terms of the XXX license.

# find the rbdl include directory
find_path(RBDL_INCLUDE_DIRS  rbdl/rbdl.h
                             PATH_SUFFIXES include
                             HINTS ${INSTALL_DEPS_PREFIX} /usr /usr/local)

# find the rbdl library
find_library(RBDL_LIBRARIES  NAMES rbdl
                             PATH_SUFFIXES lib
                             HINTS ${INSTALL_DEPS_PREFIX} /usr /usr/local)
find_library(RBDL_URDF_LIBRARIES  NAMES  rbdl_urdfreader
                                  PATH_SUFFIXES lib
                                  HINTS ${INSTALL_DEPS_PREFIX} /usr /usr/local)
list(APPEND RBDL_LIBRARIES  ${RBDL_URDF_LIBRARIES})

# handle the QUIETLY and REQUIRED arguments and set RBDL_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RBDL DEFAULT_MSG RBDL_INCLUDE_DIRS RBDL_LIBRARIES)

mark_as_advanced(RBDL_INCLUDE_DIR RBDL_LIBRARIES)
