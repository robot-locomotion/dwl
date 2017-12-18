cmake_minimum_required(VERSION 2.8.3)
project(dwl)

set(DWL_VERSION 1.2.3 CACHE STRING "DWL version")
message("Dynamic Whole-Body Locomotion (DWL) library, version ${DWL_VERSION}")
message("Copyright (C) 2014-2018 Carlos Mastalli, <carlos.mastalli@laas.fr>")
message("All rights reserved.")
message("Released under the BSD 3-Clause License.")

# Defining the dwl install prefix
set(INSTALL_DEPS_PREFIX /usr/local CACHE STRINGS "DWL thirdparty install path")


# Checking if the project is compiling with Catkin or CMake
set(CATKIN_CALL "${CATKIN_DEVEL_PREFIX}")
if(CATKIN_CALL)
	message("Compiling DWL library as a Catkin project")
	find_package(catkin REQUIRED)
else()
	message("Compiling DWL library as a CMake project")
	# Setting the output directories for libraries and executables
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/lib)
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/lib)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/bin)
endif()

# Set a default build type to 'Release' if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	message(STATUS "Setting build type to 'Release' as none was specified.")
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
	# Set the possible values of build type for cmake-gui
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Setting flags for optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -march=native -mno-avx -mtune=native -D EIGEN_NO_DEBUG")

# Appending the thirdparty path for CMake and package config variables
set(CMAKE_LIBRARY_PATH ${INSTALL_DEPS_PREFIX}/lib ${CMAKE_LIBRARY_PATH})
set(CMAKE_SYSTEM_PREFIX_PATH ${INSTALL_DEPS_PREFIX} ${CMAKE_SYSTEM_PREFIX_PATH})
set(ENV{PKG_CONFIG_PATH} "${INSTALL_DEPS_PREFIX}/lib/pkgconfig/:$ENV{PKG_CONFIG_PATH}")

# Appending the dwl module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


# Adding all the options for compilation
option(DWL_WITH_DOC "Enable the documentation generation" OFF)
option(DWL_WITH_PYTHON "Compile the Python front-end" OFF)
option(DWL_WITH_SAMPLE "Compile the sample code" OFF)
option(DWL_WITH_UNIT_TEST "Compile the code for unit testing" OFF)
option(DWL_WITH_BENCHMARK "Compile the code for benchmarking" OFF)


# Installation location for Windows
if(WIN32 AND NOT CYGWIN)
  set(DEF_LIB_PREFIX casadi)
  set(DEF_BIN_PREFIX casadi)
  set(DEF_INCLUDE_PREFIX dwl/include)
  set(DEF_CMAKE_PREFIX dwl/cmake)
  set(DEF_PYTHON_PREFIX) # Note: Files installed in a dwl subdirectory
else()
  set(DEF_LIB_PREFIX lib)
  set(DEF_BIN_PREFIX bin)
  set(DEF_INCLUDE_PREFIX include)
  set(DEF_CMAKE_PREFIX share/dwl/cmake)
  set(DEF_PYTHON_PREFIX lib/python2.7/dist-packages)
endif()

# Installation directories are cache entries that can be modified
set(INSTALL_LIB_PREFIX ${DEF_LIB_PREFIX} CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_PREFIX ${DEF_BIN_PREFIX} CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_PREFIX ${DEF_INCLUDE_PREFIX} CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_PREFIX ${DEF_CMAKE_PREFIX} CACHE PATH "Installation directory for CMake files")
set(INSTALL_PYTHON_PREFIX ${DEF_PYTHON_PREFIX} CACHE PATH "Installation directory for Python files")


# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE PYTHON)
    set(var INSTALL_${p}_PREFIX)
    if(${var})
        if(NOT IS_ABSOLUTE "${${var}}")
          # Turn relative path into absolute
          set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
        endif()
    else()
        # Flat installation hierarchy
        set(${var} "${CMAKE_INSTALL_PREFIX}")
    endif()
endforeach()

# Set rpath
# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${LIB_PREFIX}" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${LIB_PREFIX}")
endif()


# Adding the dwl source
add_subdirectory(dwl)

# Including the dwl include directories for compiling sample, test or benchmark executables
include_directories(${EXPORT_INCLUDE_DIRS})

# Adding the doxygen documentation
if(DWL_WITH_DOC)
	set(DOXYFILE_IN ${CMAKE_SOURCE_DIR}/doc/Doxyfile.in)
	set(DOXYFILE_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/doc/doxygen)
	include(UseDoxygen OPTIONAL)
endif()

# Building the python module
if(DWL_WITH_PYTHON)# OR WITH_MATLAB OR WITH_OCTAVE OR WITH_JSON)
	add_subdirectory(swig)
endif()

# Adding the sample executables
if(DWL_WITH_SAMPLE)
	add_subdirectory(sample)
endif()

# Adding the test executables
if(DWL_WITH_UNIT_TEST)
	add_subdirectory(tests)
endif()

# Adding the benchmark executables
if(DWL_WITH_BENCHMARK)
	add_subdirectory(benchmark)
endif()
