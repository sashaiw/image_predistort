cmake_minimum_required(VERSION 3.0.2)
project(image_predistort)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(image_predistort src/image_predistort.cpp)
target_link_libraries(image_predistort ${catkin_LIBRARIES})
