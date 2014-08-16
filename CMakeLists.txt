cmake_minimum_required(VERSION 2.8.3)
project(terrain_server)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  octomap_msgs
  message_generation
  tf
  tf_conversions)

find_package(dwl REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen REQUIRED)
#find_library(DWL_LIBRARY REQUIRED NAMES dwl HINTS ${dwl_LIBRARY_DIRS})


add_message_files(
  FILES
  RewardCell.msg
  RewardMap.msg
  Cell.msg
  ObstacleMap.msg)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


generate_messages(
  DEPENDENCIES
  std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp octomap_msgs message_runtime
  DEPENDS system_lib eigen)


include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${dwl_INCLUDE_DIRS}
                    ${OCTOMAP_INCLUDE_DIRS})


link_directories(${catkin_LIBRARY_DIRS}
                 ${OCTOMAP_LIBRARY_DIRS}
                 ${dwl_LIBRARY_DIRS}
                 ${Eigen_LIBRARY_DIRS}) 


## Declare a cpp library
# add_library(reward_map
#   src/${PROJECT_NAME}/reward_map.cpp
# )

## Declare a cpp executable
add_executable(reward_map_server src/RewardMapServer.cpp)
add_dependencies(reward_map_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(reward_map_server
                      ${catkin_LIBRARIES}
                      ${dwl_LIBRARIES}
                      ${OCTOMAP_LIBRARIES})


add_executable(obstacle_map_server src/ObstacleMapServer.cpp)
add_dependencies(obstacle_map_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_map_server
                      ${catkin_LIBRARIES}
                      ${dwl_LIBRARIES}
                      ${OCTOMAP_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS cost_map cost_map_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
