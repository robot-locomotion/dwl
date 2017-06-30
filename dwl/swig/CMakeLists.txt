cmake_minimum_required(VERSION 2.8.6)

find_package(SWIG REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)


if(WITH_WERROR)
  # This will make the swig command return an error so that "make" will fail,
  # but swig will emit the output files anyways so subsequent calls to "make"
  # will succeed. At least buildbot will report failures.
  set(USE_SWIG_FLAGS ${USE_SWIG_FLAGS} "-Werror")
  set(MAYBE_WERROR "-Werror")
else()
  set(MAYBE_WERROR "")
endif()


# Include the use SWIG file
#include(UseSWIG)
include(${CMAKE_MODULE_PATH}/FindSWIGMOD.cmake)

# Set SWIG flags
set(CMAKE_SWIG_FLAGS)
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-DSWIG_TYPE_TABLE=casadi")
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-fvirtual")
#set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-fcompact")
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../doc/doc.i)
  set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-DWITH_DOXDOC")
endif()

if(WITH_DEPRECATED_FEATURES)
  set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-DWITH_DEPRECATED_FEATURES")
endif()

if(WITH_PYTHON)
   add_subdirectory(python)
endif()
