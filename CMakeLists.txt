cmake_minimum_required(VERSION 2.8.3)
project(dwl)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
#find_package(yaml-cpp REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

set(DWL_VERSION 0.0.1 CACHE STRING "DWL version")
message("Compiling the version ${DWL_VERSION} of Dynamic Whole-Body Locomotion (DWL) library")

# Setting the release mode, and flags for optimization
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -march=native -mtune=native -D EIGEN_NO_DEBUG")

# Set output directories for libraries and executables
set(BASE_DIR ${CMAKE_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin)


#if (UNIX)
# find_library(YAMLCPP_LIBRARY NAMES yaml-cpp)
# find_file(YAMLCPP_HAVE_H yaml-cpp/yaml.h )
# find_path(YAMLCPP_H_INCLUDE_DIR yaml-cpp/yaml.h )
# SET(YAMLCPP_INCLUDE_DIR ${YAMLCPP_H_INCLUDE_DIR}/yaml-cpp)
#else()
# find_library(
#  YAMLCPP_LIBRARY
#  NAMES yamlcpp.lib
#  NO_DEFAULT_PATH 
#  PATHS 
#  $ENV{YAMLCPP_ROOT}/lib
# )
# find_file(YAMLCPP_HAVE_H include/yaml.h PATHS $ENV{YAMLCPP_ROOT} NO_DEFAULT_PATH)
# find_path(YAMLCPP_H_INCLUDE_DIR include/yaml.h PATHS $ENV{YAMLCPP_ROOT} NO_DEFAULT_PATH)
# SET(YAMLCPP_INCLUDE_DIR ${YAMLCPP_H_INCLUDE_DIR}/include)
#endif(UNIX)
#
#SET(YAMLCPP_FOUND FALSE)
#if (YAMLCPP_HAVE_H)
# if (YAMLCPP_LIBRARY)
#  SET(YAMLCPP_FOUND TRUE)
# endif (YAMLCPP_LIBRARY)
#endif (YAMLCPP_HAVE_H)
#
#if(NOT YAMLCPP_FOUND)
# message(STATUS "YamlCpp Can't be found")
# break()
#else()
# message(STATUS "YamlCpp Library: " ${YAMLCPP_LIBRARY} )
# message(STATUS "YamlCpp Header: " ${YAMLCPP_INCLUDE_DIR} )
#endif(NOT YAMLCPP_FOUND)


#include_directories(include)
set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
include_directories(${INCLUDE_DIRS})
include_directories(src)
#include_directories(${OCTOMAP_INCLUDE_DIRS}) #TODO Check this in the intense pc 

set(${PROJECT_NAME}_SOURCES src/planning/WholeBodyLocomotion.cpp
							src/planning/PlanningOfMotionSequences.cpp 
							src/planning/HierarchicalPlanning.cpp
							src/planning/MotionPlanning.cpp
							src/planning/SearchBasedBodyMotionPlanning.cpp
							src/planning/ContactPlanning.cpp
							src/planning/GreedyFootstepPlanning.cpp
							src/planning/Solver.cpp
							src/planning/Cost.cpp
							src/planning/Constraint.cpp
							src/planning/Dijkstrap.cpp
							src/planning/AStar.cpp
							src/planning/AnytimeRepairingAStar.cpp
							src/behavior/MotorPrimitives.cpp
							src/behavior/BodyMotorPrimitives.cpp
							src/environment/EnvironmentInformation.cpp
							src/environment/AdjacencyEnvironment.cpp
							src/environment/GridBasedBodyAdjacency.cpp
							src/environment/LatticeBasedBodyAdjacency.cpp
							src/environment/RewardMap.cpp
							src/environment/RewardOctoMap.cpp
							src/environment/ObstacleMap.cpp
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
							src/robot/Robot.cpp
							src/robot/KinematicConstraints.cpp
							src/robot/StabilityConstraints.cpp
							src/robot/StateCost.cpp
							src//utils/Math.cpp
							src/utils/Orientation.cpp) 

#set(${PROJECT_NAME}_HEADERS )


add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} yaml-cpp)# ${YAMLCPP_LIBRARY})


add_executable(my_planner tests/HierarchicalPlanningTest.cpp)
target_link_libraries(my_planner ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})


add_executable(dikstrap_algorithm tests/DijkstraExample.cpp)
add_executable(boost_dikstrap_algorithm tests/DijkstraExampleBoost.cpp)


# Install catkin package.xml and dw-config.cmake
install(FILES package.xml DESTINATION share/dwl)
install(FILES "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake" DESTINATION share/dwl/cmake)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE dwl)

# Create a octomap-config.cmake file for the use from the build tree
set(DWL_INCLUDE_DIRS "${INCLUDE_DIRS}")
set(DWL_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# not used right now (export depends?)
#set(DWL_CMAKE_DIR "${PROJECT_BINARY_DIR}")
configure_file(dwl-config.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake" @ONLY)
#configure_file(octomap-config-version.cmake.in
#  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-config-version.cmake" @ONLY)