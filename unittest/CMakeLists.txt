cmake_minimum_required(VERSION 2.8.6)

# Including this directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
message("-- Building unit testing")

# Adding unit test executables
add_executable(ws_utest  WholeBodyState.cpp)
target_link_libraries(ws_utest ${PROJECT_NAME})
if(IPOPT_FOUND)
	add_executable(ipopt_utest  IpoptDWLTest.cpp
								model/HS071DynamicalSystem.cpp
								model/HS071Cost.cpp)
	target_link_libraries(ipopt_utest ${PROJECT_NAME})
endif()

if(LIBCMAES_FOUND)
	add_executable(cmaes_utest  cmaesFamilyDWLTest.cpp
								model/HS071DynamicalSystem.cpp
								model/HS071Cost.cpp)
	target_link_libraries(cmaes_utest ${PROJECT_NAME})
endif()
add_executable(fbs_utest  FloatingBaseSystem.cpp)
target_link_libraries(fbs_utest ${PROJECT_NAME})
set_target_properties(fbs_utest  PROPERTIES
                                 COMPILE_DEFINITIONS
                                 DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(support_utest  SupportPolygonConstraintTest.cpp)
target_link_libraries(support_utest ${PROJECT_NAME})
