cmake_minimum_required(VERSION 3.0.2)
project(elevation_map_merger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  grid_map_ros
  grid_map_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(merge_maps src/merge_maps.cpp)
target_link_libraries(merge_maps ${catkin_LIBRARIES})
