cmake_minimum_required(VERSION 2.8.3)
project(dwl)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

set(DWL_VERSION 0.0.1 CACHE STRING "DWL version")
message("Compiling the version ${DWL_VERSION} of Dynamic Whole-Body Locomotion (DWL) library")

# Set output directories for libraries and executables
set(BASE_DIR ${CMAKE_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin)


#set(CMAKE_BUILD_TYPE Release)
#include_directories(include)
set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
include_directories(${INCLUDE_DIRS})
include_directories(src)
include_directories(${OCTOMAP_INCLUDE_DIRS})


set(${PROJECT_NAME}_SOURCES src/planning/WholeBodyLocomotion.cpp
							src/planning/PlanningOfMotionSequences.cpp 
							src/planning/HierarchicalPlanning.cpp
							src/planning/Solver.cpp
							src/planning/Cost.cpp
							src/planning/Constraint.cpp
							src/planning/Dijkstrap.cpp
							src/planning/AStar.cpp
							src/behavior/MotorPrimitives.cpp
							src/behavior/BodyMotorPrimitives.cpp
							src/environment/EnvironmentInformation.cpp
							src/environment/AdjacencyEnvironment.cpp
							src/environment/GridBasedBodyAdjacency.cpp
							src/environment/LatticeBasedBodyAdjacency.cpp
							src/environment/RewardMap.cpp
							src/environment/RewardOctoMap.cpp
							src/environment/PlaneGrid.cpp
							src/environment/Feature.cpp
							src/environment/SlopeFeature.cpp
							src/environment/HeightDeviationFeature.cpp
							src/robot/Robot.cpp
							src/robot/KinematicConstraints.cpp
							src/robot/StabilityConstraints.cpp
							src/robot/StateCost.cpp
							src//utils/Math.cpp
							src/utils/Orientation.cpp) 

#set(${PROJECT_NAME}_HEADERS )


add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})


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



