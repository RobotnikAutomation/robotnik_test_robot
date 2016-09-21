cmake_minimum_required(VERSION 2.8.3)
project(robotnik_test_robot_test_square)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_square src/test_square.cpp)
target_link_libraries(test_square ${catkin_LIBRARIES})
