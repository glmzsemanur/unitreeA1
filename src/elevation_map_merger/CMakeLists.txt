cmake_minimum_required(VERSION 3.0.2)
project(elevation_map_merger)

find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  grid_map_ros
  grid_map_msgs
  octomap
  octomap_msgs
  octomap_ros
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)
add_executable(merge_maps src/merge_maps.cpp)
add_executable(elevation_to_octo src/elevation_to_octo.cpp)

target_link_libraries(merge_maps ${catkin_LIBRARIES})
target_link_libraries(elevation_to_octo
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)