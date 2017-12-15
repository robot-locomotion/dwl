cmake_minimum_required(VERSION 2.8.3)
project(dwl)

set(DWL_VERSION 1.2.3 CACHE STRING "DWL version")
message("Dynamic Whole-Body Locomotion (DWL) library, version ${DWL_VERSION}")
message("Copyright (C) 2014-2018 Carlos Mastalli - LAAS-CNRS & Istituto Italiano di Tecnologia.")
message("Released under the XXX License.")

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


# Finding dependencies
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3>=3.2.10 REQUIRED)
find_package(RBDL REQUIRED)
find_package(URDF REQUIRED)
find_package(YAMLCPP REQUIRED)
find_package(QPOASES)
pkg_check_modules(IPOPT ipopt)
pkg_check_modules(LIBCMAES libcmaes)
find_package(octomap)

# Setting the thirdparties directories and libraries
set(DEPENDENCIES_INCLUDE_DIRS  ${EIGEN3_INCLUDE_DIRS} ${URDF_INCLUDE_DIRS} ${RBDL_INCLUDE_DIRS})
set(DEPENDENCIES_LIBRARIES  ${RBDL_LIBRARIES} ${URDF_LIBRARIES} ${YAMLCPP_LIBRARIES})
set(DEPENDENCIES_LIBRARY_DIRS  ${RBDL_LIBRARY_DIRS})


# Including directories
set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
include_directories(${INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${RBDL_INCLUDE_DIRS})
include_directories(${YAMLCPP_INCLUDE_DIRS})

# Setting the project sources
set(${PROJECT_NAME}_SOURCES  src/WholeBodyState.cpp
                             src/ReducedBodyState.cpp
                             src/RobotStates.cpp
                             src/locomotion/PlanningOfMotionSequence.cpp 
							 src/locomotion/HierarchicalPlanning.cpp
							 src/locomotion/MotionPlanning.cpp
							 src/locomotion/ContactPlanning.cpp
							 src/locomotion/WholeBodyTrajectoryOptimization.cpp
							 src/solver/SearchTreeSolver.cpp	
							 src/solver/OptimizationSolver.cpp
							 src/solver/Dijkstrap.cpp
							 src/solver/AStar.cpp
							 src/solver/AnytimeRepairingAStar.cpp
							 src/solver/QuadraticProgram.cpp
							 src/solver/QuadProg++QP.cpp
 							 src/model/FloatingBaseSystem.cpp
							 src/model/WholeBodyKinematics.cpp
							 src/model/WholeBodyDynamics.cpp
							 src/model/AdjacencyModel.cpp
							 src/model/GridBasedBodyAdjacency.cpp
							 src/model/LatticeBasedBodyAdjacency.cpp
							 src/model/OptimizationModel.cpp
							 src/ocp/OptimalControl.cpp
							 src/ocp/Constraint.cpp
							 src/ocp/DynamicalSystem.cpp
							 src/ocp/FullDynamicalSystem.cpp
							 src/ocp/CentroidalDynamicalSystem.cpp
							 src/ocp/ConstrainedDynamicalSystem.cpp
							 src/ocp/ComplementaryConstraint.cpp
							 src/ocp/InelasticContactModelConstraint.cpp
							 src/ocp/InelasticContactVelocityConstraint.cpp
							 src/ocp/SupportPolygonConstraint.cpp
							 src/ocp/PointConstraint.cpp
							 src/ocp/Cost.cpp
							 src/ocp/TerminalStateTrackingEnergyCost.cpp
							 src/ocp/IntegralStateTrackingEnergyCost.cpp
							 src/ocp/IntegralControlEnergyCost.cpp
							 src/simulation/PreviewLocomotion.cpp
							 src/simulation/LinearControlledCartTableModel.cpp
							 src/simulation/FootSplinePatternGenerator.cpp
							 src/behavior/MotorPrimitives.cpp
							 src/behavior/BodyMotorPrimitives.cpp
							 src/environment/TerrainMap.cpp
							 src/environment/SpaceDiscretization.cpp
							 src/environment/Feature.cpp
							 src/robot/Robot.cpp
							 src/utils/Geometry.cpp
							 src/utils/Algebra.cpp
							 src/utils/Orientation.cpp
							 src/utils/FrameTF.cpp
							 src/utils/RigidBodyDynamics.cpp
							 src/utils/URDF.cpp
							 src/utils/SplineInterpolation.cpp
							 src/utils/YamlWrapper.cpp
							 src/utils/CollectData.cpp)

# Adding qpOASES components of the project
if (qpoases_FOUND)
	list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${QPOASES_INCLUDE_DIRS})
	list(APPEND DEPENDENCIES_LIBRARY_DIRS  ${QPOASES_LIBRARY_DIR})
	list(APPEND DEPENDENCIES_LIBRARIES  ${QPOASES_LIBRARIES})
    
	include_directories(${QPOASES_INCLUDE_DIRS})
	set(${PROJECT_NAME}_SOURCES  ${${PROJECT_NAME}_SOURCES}
								 src/solver/qpOASES.cpp)
endif()

# Adding Ipopt components of the project
if(IPOPT_FOUND)
	list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${IPOPT_INCLUDE_DIRS})
	list(APPEND DEPENDENCIES_LIBRARY_DIRS  ${IPOPT_LIBRARY_DIR})
	list(APPEND DEPENDENCIES_LIBRARIES  ${IPOPT_LIBRARIES} ${IPOPT_LDFLAGS})

	include_directories(${IPOPT_INCLUDE_DIRS})
	set(${PROJECT_NAME}_SOURCES  ${${PROJECT_NAME}_SOURCES}
								 src/solver/IpoptWrapper.cpp
								 src/solver/IpoptNLP.cpp)
endif()

# Adding the CMA-ES components of the project
if(LIBCMAES_FOUND)
	list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${LIBCMAES_INCLUDE_DIRS})
	list(APPEND DEPENDENCIES_LIBRARY_DIRS  ${LIBCMAES_LIBRARY_DIR})
	list(APPEND DEPENDENCIES_LIBRARIES  ${LIBCMAES_LIBRARIES} glog)
    
	include_directories(${LIBCMAES_INCLUDE_DIRS})
	set(${PROJECT_NAME}_SOURCES  ${${PROJECT_NAME}_SOURCES}
								 src/solver/cmaesSOFamily.cpp)
endif()

# Adding Octomap components of the project
if(octomap_FOUND)
	include_directories(${OCTOMAP_INCLUDE_DIRS}) 
	list(APPEND ${PROJECT_NAME}_SOURCES  src/environment/ObstacleMap.cpp)
endif()

# Adding the dwl library
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES_LIBRARIES})


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


# Exporting the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE dwl)

# Adding the description of the dwl library
if(CATKIN_CALL)
	catkin_package(
		INCLUDE_DIRS  include ${DEPENDENCIES_INCLUDE_DIRS}
		LIBRARIES  ${PROJECT_NAME} 
		DEPENDS  ${DEPENDENCIES_LIBRARIES}
		         ${DEPENDENCIES_LIBRARY_DIRS}
		         ${DEPENDENCIES_INCLUDE_DIRS})
else()
	# Creating a dwl-config.cmake file for the use from the build tree
	set(DWL_INCLUDE_DIRS  ${INCLUDE_DIRS}:${INSTALL_INCLUDE_PREFIX}
						  ${DEPENDENCIES_INCLUDE_DIRS})
	set(DWL_LIBRARY_DIRS  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
						  ${DEPENDENCIES_LIBRARY_DIRS})
	set(DWL_LIBRARIES  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libdwl${CMAKE_SHARED_LIBRARY_SUFFIX}
					   ${DEPENDENCIES_LIBRARIES})
	configure_file(dwl-config.cmake.in
		"${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake" @ONLY)

    # Installing catkin package.xml and dwl-config.cmake for dwl as pure cmake project
    install(FILES package.xml DESTINATION ${INSTALL_LIB_PREFIX}/share/dwl)
    install(FILES "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake"
                  DESTINATION ${INSTALL_CMAKE_PREFIX})

    # Install C++ library and header files
    install(TARGETS dwl
            LIBRARY DESTINATION "${INSTALL_LIB_PREFIX}")
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
            DESTINATION DESTINATION "${INSTALL_INCLUDE_PREFIX}"
            FILES_MATCHING PATTERN "*.h*")
endif()
