# - Try to find QPOASES library
#
# This module assume that you install the robot code in thirdparty/qpOASES
#   find_package(QPOASES)
#
# Once done this will define
#
#  QPOASES_FOUND - system has qpOASES library
#  QPOASES_INCLUDE_DIRS - the qpOASES include directory
#  QPOASES_LIBRARIES - the qpOASES libraries
#
# Copyright (c) 2014-2018 Carlos Mastalli, <carlos.mastalli@iit.it>
# Redistribution and use is allowed according to the terms of the XXX license.

# find the qpOASES include directory
find_path(QPOASES_INCLUDE_DIRS  qpOASES/QProblem.hpp
                                PATH_SUFFIXES include
                                HINTS ${INSTALL_DEPS_PREFIX} /usr /usr/local)

# find the qpOASES library
find_library(QPOASES_LIBRARIES  NAMES qpOASES
                                PATH_SUFFIXES lib
                                HINTS ${INSTALL_DEPS_PREFIX} /usr /usr/local)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QPOASES DEFAULT_MSG QPOASES_LIBRARIES QPOASES_INCLUDE_DIRS)
mark_as_advanced(QPOASES_INCLUDE_DIRS)

set(qpoases_FOUND ${QPOASES_FOUND})
set(qpoases_INCLUDE_DIRS ${QPOASES_INCLUDE_DIRS})
set(qpoases_LIBRARIES ${QPOASES_LIBRARIES})
set(qpoases_LIBRARY_DIR ${QPOASES_LIBRARY_DIR})
