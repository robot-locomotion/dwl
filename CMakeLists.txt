cmake_minimum_required(VERSION 2.8.3)
project(dwl)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

set(DWL_VERSION 0.0.1)
message("Compiling the version ${DWL_VERSION} of Dynamic Whole-Body Locomotion (DWL) library")


include_directories(include)
include_directories(src)

set(${PROJECT_NAME}_SOURCES src/planning/WholeBodyLocomotion.cpp
							src/planning/PlanningOfMotionSequences.cpp 
							src/planning/HierarchicalPlanning.cpp
							src/planning/Solver.cpp
							src/planning/DijkstrapAlgorithm.cpp
							src/hyq/KinematicConstraints.cpp
							src/hyq/StateCost.cpp) 

#set(${PROJECT_NAME}_HEADERS )


add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})


add_executable(my_planner tests/HierarchicalPlanningTest.cpp)
target_link_libraries(my_planner ${PROJECT_NAME})