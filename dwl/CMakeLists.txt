cmake_minimum_required(VERSION 2.8.3)


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
set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
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


# Exporting the include directories to the parent CMake file
set(EXPORT_INCLUDE_DIRS  ${INCLUDE_DIRS}
						 ${INSTALL_INCLUDE_PREFIX}
						 ${DEPENDENCIES_INCLUDE_DIRS}
						 PARENT_SCOPE)


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
	set(DWL_INCLUDE_DIRS  ${INCLUDE_DIRS}
						  ${INSTALL_INCLUDE_PREFIX}
						  ${DEPENDENCIES_INCLUDE_DIRS}
						  PARENT_SCOPE)
	set(DWL_LIBRARY_DIRS  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
						  ${DEPENDENCIES_LIBRARY_DIRS})
	set(DWL_LIBRARIES  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libdwl${CMAKE_SHARED_LIBRARY_SUFFIX}
					   ${DEPENDENCIES_LIBRARIES})
	configure_file(../dwl-config.cmake.in
		"${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake" @ONLY)

    # Installing catkin package.xml and dwl-config.cmake for dwl as pure cmake project
    install(FILES ../package.xml DESTINATION ${INSTALL_LIB_PREFIX}/share/dwl)
    install(FILES "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dwl/dwl-config.cmake"
                  DESTINATION ${INSTALL_CMAKE_PREFIX})

    # Install C++ library and header files
    install(TARGETS dwl
            LIBRARY DESTINATION "${INSTALL_LIB_PREFIX}")
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
            DESTINATION DESTINATION "${INSTALL_INCLUDE_PREFIX}"
            FILES_MATCHING PATTERN "*.h*")
endif()
