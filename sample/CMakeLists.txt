cmake_minimum_required(VERSION 2.8.6)

# Including this directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
message("-- Building sample code")


# Adding sample executables
add_executable(fbs_sample  FloatingBaseSample.cpp)
target_link_libraries(fbs_sample  ${PROJECT_NAME})
set_target_properties(fbs_sample  PROPERTIES
                                  COMPILE_DEFINITIONS
                                  DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(wkin_sample  WholeBodyKinematicsSample.cpp)
target_link_libraries(wkin_sample ${PROJECT_NAME})
set_target_properties(wkin_sample  PROPERTIES
                                   COMPILE_DEFINITIONS
                                   DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(wdyn_sample  WholeBodyDynamicsSample.cpp)
target_link_libraries(wdyn_sample  ${PROJECT_NAME})
set_target_properties(wdyn_sample  PROPERTIES
                                   COMPILE_DEFINITIONS
                                   DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(cd_sample  CollectDataSample.cpp)
target_link_libraries(cd_sample ${PROJECT_NAME})

add_executable(yaml_sample  YamlWrapperSample.cpp)
target_link_libraries(yaml_sample ${PROJECT_NAME})
set_target_properties(yaml_sample  PROPERTIES
                                   COMPILE_DEFINITIONS
                                   DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")