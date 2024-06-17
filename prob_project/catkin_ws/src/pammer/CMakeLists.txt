cmake_minimum_required(VERSION 3.0.2)
project(pammer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  message_filters
)

find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ekf_node src/ekf.cpp)
add_executable(goals_node src/goals.cpp)

target_link_libraries(ekf_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(goals_node
  ${catkin_LIBRARIES}
)