cmake_minimum_required(VERSION 3.0.2)
project(simple_layers)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_layers
#  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_layer src/simple_layer.cpp)

add_library(grid_layer src/grid_layer.cpp)