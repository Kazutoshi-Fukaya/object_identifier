cmake_minimum_required(VERSION 3.0.2)
project(object_identifier)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  object_detector_msgs
  object_identifier_msgs
)

catkin_package(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
#  LIBRARIES object_identifier
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/object_identifier_node.cpp
  src/object_identifier.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)