cmake_minimum_required(VERSION 2.8.3)
project(dwl_rviz_plugin)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz)

find_package(dwl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz
  DEPENDS system_lib)


include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${dwl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${dwl_LIBRARY_DIRS})
                 
qt4_wrap_cpp(MOC_FILES
  include/dwl_rviz_plugin/RewardMapDisplay.h
  include/dwl_rviz_plugin/ObstacleMapDisplay.h
  include/dwl_rviz_plugin/FootstepRegionDisplay.h
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

set(SOURCE_FILES
  src/RewardMapDisplay.cpp
  src/ObstacleMapDisplay.cpp
  src/FootstepRegionDisplay.cpp
  ${MOC_FILES})

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${dwl_LIBRARIES} ${catkin_LIBRARIES} -ldefault_plugin)




