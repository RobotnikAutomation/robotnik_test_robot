cmake_minimum_required(VERSION 2.8.3)
project(robotnik_test_robot_test_battery)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_battery src/test_battery.cpp)
target_link_libraries(test_battery ${catkin_LIBRARIES})
