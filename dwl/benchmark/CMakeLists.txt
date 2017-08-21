cmake_minimum_required(VERSION 2.8.6)

# Including this directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
message("-- Building benchmark")

# Adding benchmarck executables
add_executable(wif_benchmark  WholeBodyInterface.cpp)
target_link_libraries(wif_benchmark ${PROJECT_NAME})
set_target_properties(wif_benchmark PROPERTIES COMPILE_DEFINITIONS DWL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")