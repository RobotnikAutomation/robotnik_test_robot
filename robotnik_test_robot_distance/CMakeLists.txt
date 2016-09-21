cmake_minimum_required(VERSION 2.8.3)
project(robotnik_test_robot_distance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(distance src/distance.cpp)
target_link_libraries(distance ${catkin_LIBRARIES})
