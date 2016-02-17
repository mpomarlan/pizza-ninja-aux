cmake_minimum_required(VERSION 2.8.3)
project(pizzaninja-controllers)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  roslib 
  giskard 
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  kdl_conversions)

## Finding system dependencies which come without cmake
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/yaml.h PATH_SUFFIXES include)
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp)

catkin_package(
  CATKIN_DEPENDS roscpp roslib giskard tf2_ros tf2_geometry_msgs geometry_msgs kdl_conversions
  DEPENDS yaml_cpp
)

##############
## Building ##
##############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

add_executable(pr2_point2point src/${PROJECT_NAME}/pr2_point2point.cpp)
target_link_libraries(pr2_point2point
  ${catkin_LIBRARIES} yaml-cpp)

add_executable(pr2_segfollower src/${PROJECT_NAME}/pr2_segfollower.cpp)
target_link_libraries(pr2_segfollower
  ${catkin_LIBRARIES} yaml-cpp)

