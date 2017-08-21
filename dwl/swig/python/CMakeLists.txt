cmake_minimum_required(VERSION 2.8.6)

# Python front-end
add_custom_target(python DEPENDS _dwl)

# Workaround for OS X
if(APPLE)
  if(WITH_PYTHON3)
  set(PYTHON_CONFIG_NAME python3-config)
  else()
  set(PYTHON_CONFIG_NAME python-config)
  endif()
  find_program(PYTHON_CONFIG_EXECUTABLE
               NAMES ${PYTHON_CONFIG_NAME} DOC "python-config executable")
  if(PYTHON_CONFIG_EXECUTABLE)
    execute_process(COMMAND ${PYTHON_CONFIG_EXECUTABLE} --prefix
                    OUTPUT_VARIABLE PYTHON_PREFIX_STRING
                    RESULT_VARIABLE PYTHON_PREFIX_FAILED
                    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT PYTHON_PREFIX_FAILED)
      file(GLOB LOCAL_PYTHON_NAME RELATIVE ${PYTHON_PREFIX_STRING}/lib "${PYTHON_PREFIX_STRING}/lib/python*.*")
      find_library(PYTHON_LIBRARY NAMES ${LOCAL_PYTHON_NAME}
        PATHS ${PYTHON_PREFIX_STRING}/lib NO_DEFAULT_PATH)
      file(GLOB LOCAL_PYTHON_NAME "${PYTHON_PREFIX_STRING}/include/python*")
      find_path(PYTHON_INCLUDE_DIR
        NAMES Python.h
        PATHS ${LOCAL_PYTHON_NAME} NO_DEFAULT_PATH)
    endif()
    message(STATUS ${PYTHON_INCLUDE_DIR})
  endif()
endif()

# Find packages
if(WITH_PYTHON3)
set(MINPYVERSION "3")
else()
set(MINPYVERSION "")
endif()
find_package(PythonInterp ${MINPYVERSION} REQUIRED)
find_package(PythonLibs ${MINPYVERSION} REQUIRED)
find_package(NUMPY REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PYTHON_INCLUDE_PATH})
include_directories(${NUMPY_INCLUDED_DIRS})

# a python library is built in the build directory inside swig/python
make_directory(${PROJECT_BINARY_DIR}/python/dwl)

if(WITH_PYTHON_INTERRUPTS)
  set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-DWITH_PYTHON_INTERRUPTS")
endif()

set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-DPy_USING_UNICODE")
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-noproxydel")
if("${PYTHON_VERSION_MAJOR}" STREQUAL "3")
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-py3")
set(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-DWITH_PYTHON3")
endif()

# Generate SWIG wrapper
if(WITH_DOC)
    set_source_files_properties(../dwl_doc.i  PROPERTIES  CPLUSPLUS ON)
    swig_add_module(dwl python ../dwl_doc.i)
    add_dependencies(dwl swig_doxygen)
else()
    set_source_files_properties(../dwl.i  PROPERTIES  CPLUSPLUS ON)
    swig_add_module(dwl python ../dwl.i)
endif()
swig_link_libraries(dwl ${PYTHON_LIBRARIES} dwl)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set_target_properties(_dwl PROPERTIES
                             COMPILE_FLAGS "-Wno-dynamic-class-memaccess -Wno-self-assign ${MAYBE_WERROR}")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(_dwl PROPERTIES
                             COMPILE_FLAGS "-Wno-dynamic-class-memaccess -Wno-self-assign -Wno-maybe-uninitialized ${MAYBE_WERROR}")
endif()

# the name used for the shared module for the python interface is determined
get_target_property(DWL_PYTHON_LIBRARY_SUFFIX ${SWIG_MODULE_dwl_REAL_NAME} SUFFIX)
if(NOT ${DWL_PYTHON_LIBRARY_SUFFIX})
  if( WIN32 )
    set(DWL_PYTHON_LIBRARY_SUFFIX ".pyd")
  else()
    set(DWL_PYTHON_LIBRARY_SUFFIX ${CMAKE_SHARED_MODULE_SUFFIX})
  endif()
endif()
set(DWL_PYTHON_LIBRARY ${SWIG_MODULE_dwl_REAL_NAME}${DWL_PYTHON_LIBRARY_SUFFIX})

# Custom installation command for Python
add_custom_target(install_python
COMMAND ${CMAKE_COMMAND}
  -D COMPONENT=install_python
  -D CMAKE_INSTALL_PREFIX="${PYTHON_PREFIX}"
  -P cmake_install.cmake
)
add_dependencies(install_python _dwl)

# Install C++ wrapper library
install(TARGETS _dwl
  DESTINATION "${PYTHON_PREFIX}/dwl"
  COMPONENT install_python
)

# Install Python proxy classes
install(FILES ${PROJECT_BINARY_DIR}/swig/python/dwl.py
  DESTINATION "${PYTHON_PREFIX}/dwl"
  COMPONENT install_python
)

# Install Python tools
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tools
#  DESTINATION "${PYTHON_PREFIX}/dwl"
#  COMPONENT install_python
#  USE_SOURCE_PERMISSIONS
#  PATTERN .pyc EXCLUDE
#  PATTERN .svn EXCLUDE
#)

# Install Python package initialization
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
  DESTINATION "${PYTHON_PREFIX}/dwl"
  COMPONENT install_python
)
