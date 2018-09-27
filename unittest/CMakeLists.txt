cmake_minimum_required(VERSION 2.8.6)

# Including this directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
message("-- Building unit testing")

# Adding unit test executables
add_executable(ws_utest  WholeBodyState.cpp)
target_link_libraries(ws_utest ${PROJECT_NAME})

add_executable(fbs_utest  FloatingBaseSystem.cpp)
target_link_libraries(fbs_utest ${PROJECT_NAME})
set_target_properties(fbs_utest  PROPERTIES
                                 COMPILE_DEFINITIONS
                                 DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(wkin_utest  WholeBodyKinematics.cpp)
target_link_libraries(wkin_utest ${PROJECT_NAME})
set_target_properties(wkin_utest  PROPERTIES
								  COMPILE_DEFINITIONS
								  DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
