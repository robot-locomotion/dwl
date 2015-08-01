cmake_minimum_required(VERSION 2.8.3)
project(dwl)

set(DWL_VERSION 0.2.9 CACHE STRING "DWL version")
message("Dynamic Whole-Body Locomotion (DWL) library, version ${DWL_VERSION}")
message("Copyright (C) 2014-2016 Carlos Mastalli - Istituto Italiano di Tecnologia.")
message("Released under the XXX License.")


# Appending the project module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Finding dependencies
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(RBDL rbdl REQUIRED)
find_package(URDF REQUIRED)
find_package(YAMLCPP REQUIRED)
find_package(ROBOTS)
find_package(IPOPT)
find_package(octomap)


# Set a default build type to 'Release' if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Setting flags for optimization
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -march=native -mno-avx -mtune=native -D EIGEN_NO_DEBUG")

# Seting the output directories for libraries and executables
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/bin)

# Setting the thirdparties directories and libraries
set(DEPENDENCIES_INCLUDE_DIRS  ${EIGEN3_INCLUDE_DIRS} ${URDF_INCLUDE_DIRS})
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
set(${PROJECT_NAME}_SOURCES  src/locomotion/PlanningOfMotionSequence.cpp 
							 src/locomotion/HierarchicalPlanning.cpp
							 src/locomotion/MotionPlanning.cpp
							 src/locomotion/SearchBasedBodyMotionPlanning.cpp
							 src/locomotion/ContactPlanning.cpp
							 src/locomotion/GreedyFootstepPlanning.cpp	
							 src/solver/OptimizationSolver.cpp
							 src/solver/Solver.cpp
							 src/solver/Dijkstrap.cpp
							 src/solver/AStar.cpp
							 src/solver/AnytimeRepairingAStar.cpp
							 src/model/OptimizationModel.cpp
							 src/model/Constraint.cpp
							 src/model/DynamicalSystem.cpp
							 src/model/FullDynamicalSystem.cpp
							 src/model/ConstrainedDynamicalSystem.cpp
							 src/model/KinematicConstraint.cpp
							 src/model/StabilityConstraint.cpp
							 src/model/Cost.cpp
							 src/model/StateTrackingEnergyCost.cpp
							 src/model/ControlEnergyCost.cpp
							 src/model/RobCoGenWholeBodyKinematics.cpp
							 src/model/RobCoGenWholeBodyDynamics.cpp
							 src/model/WholeBodyKinematics.cpp
							 src/model/WholeBodyDynamics.cpp
							 src/behavior/MotorPrimitives.cpp
							 src/behavior/BodyMotorPrimitives.cpp
							 src/environment/EnvironmentInformation.cpp
							 src/environment/AdjacencyEnvironment.cpp
							 src/environment/GridBasedBodyAdjacency.cpp
							 src/environment/LatticeBasedBodyAdjacency.cpp
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
							 src/utils/RigidBodyDynamics.cpp
							 src/utils/SplineInterpolation.cpp
							 src/utils/YamlWrapper.cpp)

# Adding robot components of the project
if(hyl_FOUND)
    list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${hyl_INCLUDE_DIRS})
    list(APPEND DEPENDENCIES_LIBRARIES  ${hyl_LIBRARIES})

    include_directories(${hyl_INCLUDE_DIRS})
	list(APPEND ${PROJECT_NAME}_SOURCES  src/robot/HyLWholeBodyKinematics.cpp
								         src/robot/HyLWholeBodyDynamics.cpp)
endif()

if(hyq_FOUND)
    list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${hyq_INCLUDE_DIRS})
    list(APPEND DEPENDENCIES_LIBRARIES  ${hyq_LIBRARIES})

    include_directories(${hyq_INCLUDE_DIRS})
	list(APPEND ${PROJECT_NAME}_SOURCES  src/robot/HyQWholeBodyKinematics.cpp)
								         #src/robot/HyQWholeBodyDynamics.cpp
endif()


# Adding Ipopt components of the project
if(ipopt_FOUND)
    list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${IPOPT_INCLUDE_DIRS})
    list(APPEND DEPENDENCIES_LIBRARY_DIRS  ${IPOPT_LIBRARY_DIR})
    list(APPEND DEPENDENCIES_LIBRARIES  ${IPOPT_LIBRARIES})
    
    include_directories(${IPOPT_INCLUDE_DIRS})
    set(${PROJECT_NAME}_SOURCES  ${${PROJECT_NAME}_SOURCES}
                                 src/solver/IpoptWrapper.cpp
                                 src/solver/IpoptNLP.cpp)
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


add_executable(my_planner  tests/HierarchicalPlanningTest.cpp)
target_link_libraries(my_planner ${PROJECT_NAME})

include_directories(tests)
if(ipopt_FOUND)
	add_executable(ipopt_test  tests/IpoptDWLTest.cpp
	                           tests/model/HS071DynamicalSystem.cpp
	                           tests/model/HS071Cost.cpp)
	target_link_libraries(ipopt_test ${PROJECT_NAME})
endif()

add_executable(yaml_bridge  tests/YamlWrapperTest.cpp)
target_link_libraries(yaml_bridge ${PROJECT_NAME})

add_executable(dikstrap_algorithm  tests/DijkstraExample.cpp)
add_executable(boost_dikstrap_algorithm  tests/DijkstraExampleBoost.cpp)


add_executable(numdiff  tests/NumDiffEigenTest.cpp)
target_link_libraries(numdiff ${PROJECT_NAME})



# Installing catkin package.xml and dw-config.cmake
install(FILES package.xml DESTINATION share/dwl)
install(FILES "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake" DESTINATION share/dwl/cmake)

# Exporting the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE dwl)

# Creating a dwl-config.cmake file for the use from the build tree
set(DWL_INCLUDE_DIRS  ${INCLUDE_DIRS}
                      ${DEPENDENCIES_INCLUDE_DIRS})
set(DWL_LIBRARY_DIRS  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                      ${DEPENDENCIES_LIBRARY_DIRS})
set(DWL_LIBRARIES  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libdwl${CMAKE_SHARED_LIBRARY_SUFFIX}
                   ${DEPENDENCIES_LIBRARIES})
configure_file(dwl-config.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake" @ONLY)
