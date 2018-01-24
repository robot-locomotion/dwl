cmake_minimum_required(VERSION 2.8.3)


# Finding dependencies
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3>=3.2 REQUIRED)
find_package(RBDL REQUIRED)
find_package(URDF REQUIRED)
find_package(YAMLCPP REQUIRED)
find_package(QPOASES)
pkg_check_modules(IPOPT ipopt>=3.12.4)
pkg_check_modules(LIBCMAES libcmaes>=0.9.5)
find_package(octomap)

# Setting the thirdparties directories and libraries
set(DEPENDENCIES_INCLUDE_DIRS  ${EIGEN3_INCLUDE_DIRS} ${URDF_INCLUDE_DIRS} ${RBDL_INCLUDE_DIRS})
set(DEPENDENCIES_LIBRARIES  ${RBDL_LIBRARIES} ${URDF_LIBRARIES} ${YAMLCPP_LIBRARIES})
set(DEPENDENCIES_LIBRARY_DIRS  ${RBDL_LIBRARY_DIRS})


# Including directories
set(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${RBDL_INCLUDE_DIRS})
include_directories(${YAMLCPP_INCLUDE_DIRS})


# Setting the project sources
set(${PROJECT_NAME}_SOURCES  dwl/WholeBodyState.cpp
							 dwl/ReducedBodyState.cpp
							 dwl/RobotStates.cpp
							 dwl/locomotion/PlanningOfMotionSequence.cpp 
							 dwl/locomotion/HierarchicalPlanning.cpp
							 dwl/locomotion/MotionPlanning.cpp
							 dwl/locomotion/ContactPlanning.cpp
							 dwl/locomotion/WholeBodyTrajectoryOptimization.cpp
							 dwl/solver/SearchTreeSolver.cpp	
							 dwl/solver/OptimizationSolver.cpp
							 dwl/solver/Dijkstrap.cpp
							 dwl/solver/AStar.cpp
							 dwl/solver/AnytimeRepairingAStar.cpp
							 dwl/solver/QuadraticProgram.cpp
							 dwl/solver/QuadProg++QP.cpp
 							 dwl/model/FloatingBaseSystem.cpp
							 dwl/model/WholeBodyKinematics.cpp
							 dwl/model/WholeBodyDynamics.cpp
							 dwl/model/AdjacencyModel.cpp
							 dwl/model/GridBasedBodyAdjacency.cpp
							 dwl/model/LatticeBasedBodyAdjacency.cpp
							 dwl/model/OptimizationModel.cpp
							 dwl/ocp/OptimalControl.cpp
							 dwl/ocp/Constraint.cpp
							 dwl/ocp/DynamicalSystem.cpp
							 dwl/ocp/FullDynamicalSystem.cpp
							 dwl/ocp/CentroidalDynamicalSystem.cpp
							 dwl/ocp/ConstrainedDynamicalSystem.cpp
							 dwl/ocp/ComplementaryConstraint.cpp
							 dwl/ocp/InelasticContactModelConstraint.cpp
							 dwl/ocp/InelasticContactVelocityConstraint.cpp
							 dwl/ocp/SupportPolygonConstraint.cpp
							 dwl/ocp/PointConstraint.cpp
							 dwl/ocp/Cost.cpp
							 dwl/ocp/TerminalStateTrackingEnergyCost.cpp
							 dwl/ocp/IntegralStateTrackingEnergyCost.cpp
							 dwl/ocp/IntegralControlEnergyCost.cpp
							 dwl/simulation/PreviewLocomotion.cpp
							 dwl/simulation/LinearControlledCartTableModel.cpp
							 dwl/simulation/FootSplinePatternGenerator.cpp
							 dwl/behavior/MotorPrimitives.cpp
							 dwl/behavior/BodyMotorPrimitives.cpp
							 dwl/environment/TerrainMap.cpp
							 dwl/environment/SpaceDiscretization.cpp
							 dwl/environment/Feature.cpp
							 dwl/robot/Robot.cpp
							 dwl/utils/Geometry.cpp
							 dwl/utils/Algebra.cpp
							 dwl/utils/Orientation.cpp
							 dwl/utils/FrameTF.cpp
							 dwl/utils/RigidBodyDynamics.cpp
							 dwl/utils/URDF.cpp
							 dwl/utils/SplineInterpolation.cpp
							 dwl/utils/YamlWrapper.cpp
							 dwl/utils/CollectData.cpp)

# Adding qpOASES components of the project
if (qpoases_FOUND)
	list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${QPOASES_INCLUDE_DIRS})
	list(APPEND DEPENDENCIES_LIBRARY_DIRS  ${QPOASES_LIBRARY_DIR})
	list(APPEND DEPENDENCIES_LIBRARIES  ${QPOASES_LIBRARIES})
    
	include_directories(${QPOASES_INCLUDE_DIRS})
	set(${PROJECT_NAME}_SOURCES  ${${PROJECT_NAME}_SOURCES}
								 dwl/solver/qpOASES.cpp)
endif()

# Adding Ipopt components of the project
if(IPOPT_FOUND)
	list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${IPOPT_INCLUDE_DIRS})
	list(APPEND DEPENDENCIES_LIBRARY_DIRS  ${IPOPT_LIBRARY_DIR})
	list(APPEND DEPENDENCIES_LIBRARIES  ${IPOPT_LIBRARIES} ${IPOPT_LDFLAGS})

	include_directories(${IPOPT_INCLUDE_DIRS})
	set(${PROJECT_NAME}_SOURCES  ${${PROJECT_NAME}_SOURCES}
								 dwl/solver/IpoptWrapper.cpp
								 dwl/solver/IpoptNLP.cpp)
endif()

# Adding the CMA-ES components of the project
if(LIBCMAES_FOUND)
	list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${LIBCMAES_INCLUDE_DIRS})
	list(APPEND DEPENDENCIES_LIBRARY_DIRS  ${LIBCMAES_LIBRARY_DIR})
	list(APPEND DEPENDENCIES_LIBRARIES  ${LIBCMAES_LIBRARIES} glog)
    
	include_directories(${LIBCMAES_INCLUDE_DIRS})
	set(${PROJECT_NAME}_SOURCES  ${${PROJECT_NAME}_SOURCES}
								 dwl/solver/cmaesSOFamily.cpp)
endif()

# Adding Octomap components of the project
if(octomap_FOUND)
	include_directories(${OCTOMAP_INCLUDE_DIRS}) 
	list(APPEND ${PROJECT_NAME}_SOURCES  dwl/environment/ObstacleMap.cpp)
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
						  ${DEPENDENCIES_INCLUDE_DIRS})
	set(DWL_LIBRARY_DIRS  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
						  ${DEPENDENCIES_LIBRARY_DIRS})
	set(DWL_LIBRARIES  ${INSTALL_LIB_PREFIX}/libdwl${CMAKE_SHARED_LIBRARY_SUFFIX}
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
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/dwl/
            DESTINATION DESTINATION "${INSTALL_INCLUDE_PREFIX}"
            FILES_MATCHING PATTERN "*.h*")
endif()
