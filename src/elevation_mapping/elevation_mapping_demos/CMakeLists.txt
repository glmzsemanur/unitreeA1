cmake_minimum_required(VERSION 3.5.1)
project(elevation_mapping_demos)

find_package(catkin REQUIRED COMPONENTS
  elevation_mapping
  grid_map_rviz_plugin
  rviz
  roscpp
  grid_map_ros
  nav_msgs
  grid_map_msgs
  pcl_ros
  pcl_conversions
  tf2_eigen
  octomap
  octomap_msgs
  octomap_ros
  rosbag
)

find_package(Eigen3 REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS roscpp grid_map_ros nav_msgs grid_map_msgs elevation_mapping grid_map_rviz_plugin rviz
  DEPENDS Eigen3
  #DEPENDS
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY config doc launch rviz sample_data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/tf_to_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(elevation_to_occupancy src/elevation_to_occupancy.cpp)
target_link_libraries(elevation_to_occupancy ${catkin_LIBRARIES})
