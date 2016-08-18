cmake_minimum_required(VERSION 2.8.3)
project(dwl)

set(DWL_VERSION 1.1.4 CACHE STRING "DWL version")
message("Dynamic Whole-Body Locomotion (DWL) library, version ${DWL_VERSION}")
message("Copyright (C) 2014-2016 Carlos Mastalli - Istituto Italiano di Tecnologia.")
message("Released under the XXX License.")

# Defining the dwl install prefix
set(DWL_INSTALL_PREFIX /usr/local/dwl CACHE STRINGS "DWL Install Prefix")

# Defining the unit testing environment
set(UNIT_TEST false CACHE BOOL "Unit testing")

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

# Appending the project module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(ENV{PKG_CONFIG_PATH} "${DWL_INSTALL_PREFIX}/lib/pkgconfig/:$ENV{PKG_CONFIG_PATH}")

# Finding dependencies
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3=3.2.7 REQUIRED)
pkg_check_modules(RBDL rbdl REQUIRED)
find_package(URDF REQUIRED)
find_package(YAMLCPP REQUIRED)
find_package(QPOASES)
pkg_check_modules(IPOPT ipopt)
pkg_check_modules(LIBCMAES libcmaes)
find_package(octomap)

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

# Setting the thirdparties directories and libraries
set(DEPENDENCIES_INCLUDE_DIRS  ${EIGEN3_INCLUDE_DIRS} ${URDF_INCLUDE_DIRS} ${RBDL_INCLUDE_DIRS})
set(DEPENDENCIES_LIBRARIES  ${RBDL_LIBRARIES} rbdl_urdfreader ${URDF_LIBRARIES} ${YAMLCPP_LIBRARIES})
set(DEPENDENCIES_LIBRARY_DIRS  ${RBDL_LIBRARY_DIRS})

# Including directories
set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
include_directories(${INCLUDE_DIRS})
include_directories(src)
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
							 src/locomotion/SearchBasedBodyMotionPlanning.cpp
							 src/locomotion/ContactPlanning.cpp
							 src/locomotion/GreedyFootstepPlanning.cpp
							 src/locomotion/WholeBodyTrajectoryOptimization.cpp
							 src/locomotion/PreviewTrajectoryOptimization.cpp
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
							 src/ocp/PreviewOptimization.cpp
							 src/simulation/PreviewLocomotion.cpp
							 src/simulation/LinearControlledCartTableModel.cpp
							 src/simulation/FootSplinePatternGenerator.cpp
							 src/behavior/MotorPrimitives.cpp
							 src/behavior/BodyMotorPrimitives.cpp
							 src/environment/TerrainMap.cpp
							 src/environment/SpaceDiscretization.cpp
							 src/environment/Feature.cpp
							 src/environment/SlopeFeature.cpp
							 src/environment/CurvatureFeature.cpp
							 src/environment/HeightDeviationFeature.cpp
							 src/environment/PotentialLegCollisionFeature.cpp
							 src/environment/PotentialBodyOrientationFeature.cpp
							 src/environment/LegCollisionFeature.cpp
							 src/environment/BodyOrientationFeature.cpp
							 src/environment/SupportTriangleFeature.cpp
							 src/environment/KinematicFeasibilityFeature.cpp
							 src/environment/StancePostureFeature.cpp
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
	list(APPEND ${PROJECT_NAME}_SOURCES  src/environment/RewardMap.cpp
										 src/environment/ObstacleMap.cpp)
endif()

# Adding the dwl library
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES_LIBRARIES})


# Adding sample executables
add_executable(fb_sample  sample/FloatingBaseSample.cpp)
target_link_libraries(fb_sample ${PROJECT_NAME})

add_executable(kin_sample  sample/WholeBodyKinematicsSample.cpp)
target_link_libraries(kin_sample ${PROJECT_NAME})

add_executable(dyn_sample  sample/WholeBodyDynamicsSample.cpp)
target_link_libraries(dyn_sample ${PROJECT_NAME})

add_executable(cd_sample sample/CollectDataSample.cpp)
target_link_libraries(cd_sample ${PROJECT_NAME})


# Adding the test executables
if(UNIT_TEST)
	message("-- Building unit testing")
	include_directories(tests)
    add_executable(ws_utest  tests/WholeBodyStateUTest.cpp)
	target_link_libraries(ws_utest ${PROJECT_NAME})
	if(IPOPT_FOUND)
		add_executable(ipopt_utest  tests/IpoptDWLTest.cpp
									tests/model/HS071DynamicalSystem.cpp
									tests/model/HS071Cost.cpp)
		target_link_libraries(ipopt_utest ${PROJECT_NAME})
	endif()

	if(LIBCMAES_FOUND)
		add_executable(cmaes_utest  tests/cmaesFamilyDWLTest.cpp
									tests/model/HS071DynamicalSystem.cpp
									tests/model/HS071Cost.cpp)
		target_link_libraries(cmaes_utest ${PROJECT_NAME})
	endif()

	add_executable(support_utest  tests/SupportPolygonConstraintTest.cpp)
	target_link_libraries(support_utest ${PROJECT_NAME})

	add_executable(yaml_utest  tests/YamlWrapperTest.cpp)
	target_link_libraries(yaml_utest ${PROJECT_NAME})

	add_executable(numdiff_utest  tests/NumDiffEigenTest.cpp)
	target_link_libraries(numdiff_utest ${PROJECT_NAME})
endif()

# Exporting the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE dwl)

# Adding the description of the dwl library
if(CATKIN_CALL)
	catkin_package(
		INCLUDE_DIRS include ${DEPENDENCIES_INCLUDE_DIRS}
		LIBRARIES ${PROJECT_NAME} 
		DEPENDS system_lib ${DEPENDENCIES_LIBRARIES} ${DEPENDENCIES_LIBRARY_DIRS} ${DEPENDENCIES_INCLUDE_DIRS})
else()
    # Installing catkin package.xml and dwl-config.cmake for dwl as pure cmake project
    install(FILES package.xml DESTINATION share/dwl)
    install(FILES "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake" DESTINATION share/dwl/cmake)

	# Creating a dwl-config.cmake file for the use from the build tree
	set(DWL_INCLUDE_DIRS  ${INCLUDE_DIRS}
						  ${DEPENDENCIES_INCLUDE_DIRS})
	set(DWL_LIBRARY_DIRS  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
						  ${DEPENDENCIES_LIBRARY_DIRS})
	set(DWL_LIBRARIES  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libdwl${CMAKE_SHARED_LIBRARY_SUFFIX}
					   ${DEPENDENCIES_LIBRARIES})
	configure_file(dwl-config.cmake.in
		"${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake" @ONLY)
endif()
