cmake_minimum_required(VERSION 2.8.3)
project(dwl_planners)


find_package(catkin REQUIRED COMPONENTS
  dwl
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  tf_conversions)

find_package(dwl REQUIRED)


add_message_files(
   FILES
    Region.msg
    Contact.msg
    ContactSequence.msg
    ContactRegion.msg
    JointState.msg
    WholeBodyState.msg
    WholeBodyTrajectory.msg)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs)
   

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS system_lib)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${dwl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${dwl_LIBRARY_DIRS})


add_executable(footstep_hierarchical_planner_node  src/FootstepHierarchicalPlanner.cpp)
add_dependencies(footstep_hierarchical_planner_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(footstep_hierarchical_planner_node  ${catkin_LIBRARIES}
                                                          ${dwl_LIBRARIES})
add_dependencies(footstep_hierarchical_planner_node ${PROJECT_NAME}_gencpp)

add_executable(constrained_whole_body_planner_node  src/ConstrainedWholeBodyPlanner.cpp)
add_dependencies(constrained_whole_body_planner_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(constrained_whole_body_planner_node  ${catkin_LIBRARIES}
                                                           ${dwl_LIBRARIES})
add_dependencies(constrained_whole_body_planner_node ${PROJECT_NAME}_gencpp)

add_executable(dynamic_whole_body_planner_node  src/DynamicWholeBodyPlanner.cpp)
add_dependencies(dynamic_whole_body_planner_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_whole_body_planner_node  ${catkin_LIBRARIES}
                                                       ${dwl_LIBRARIES})
add_dependencies(dynamic_whole_body_planner_node ${PROJECT_NAME}_gencpp)

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
# install(TARGETS dwl_planners dwl_planners_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dwl_planners.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
