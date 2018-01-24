cmake_minimum_required(VERSION 2.8.6)

# Including this directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
message("-- Building sample code")


# Adding sample executables
add_executable(fb_sample  FloatingBaseSample.cpp)
target_link_libraries(fb_sample  ${PROJECT_NAME})
set_target_properties(fb_sample  PROPERTIES
                                 COMPILE_DEFINITIONS
                                 DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(kin_sample  WholeBodyKinematicsSample.cpp)
target_link_libraries(kin_sample ${PROJECT_NAME})
set_target_properties(kin_sample  PROPERTIES
                                  COMPILE_DEFINITIONS
                                  DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(dyn_sample  WholeBodyDynamicsSample.cpp)
target_link_libraries(dyn_sample  ${PROJECT_NAME})
set_target_properties(dyn_sample  PROPERTIES
                                  COMPILE_DEFINITIONS
                                  DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(cd_sample  CollectDataSample.cpp)
target_link_libraries(cd_sample ${PROJECT_NAME})

add_executable(yaml_sample  YamlWrapperSample.cpp)
target_link_libraries(yaml_sample ${PROJECT_NAME})
set_target_properties(yaml_sample  PROPERTIES
                                   COMPILE_DEFINITIONS
                                   DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
