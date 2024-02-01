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
  visualization_msgs
  object_detector_msgs
  object_identifier_msgs
  multi_localizer_msgs
)

# OpenCV
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# DBoW
add_library(DBoW3 SHARED
  dbow3/descriptors_manipulator.cpp
  dbow3/feature_vector.cpp
  dbow3/scoring_object.cpp
  dbow3/quicklz.cpp
  dbow3/vocabulary.cpp
  dbow3/query_results.cpp
  dbow3/bow_vector.cpp
  dbow3/database.cpp
)

add_executable(${PROJECT_NAME}_node
  src/object_identifier_node.cpp
  src/object_identifier.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  DBoW3
)

add_executable(identify_object_test
  src/identify_object_test.cpp
)
target_link_libraries(identify_object_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  DBoW3
)

add_executable(debug_object_map_publisher
  src/debug_object_map_publisher.cpp
)
target_link_libraries(debug_object_map_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  DBoW3
)