cmake_minimum_required(VERSION 3.0.2)
project(octomap_to_occupancy)

## Compile as C++14
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  octomap_msgs
  tf
)

find_package(octomap REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs octomap_msgs tf
  DEPENDS octomap
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(octomap_to_occupancy
  src/octomap_to_occupancy.cpp
)

target_link_libraries(
  octomap_to_occupancy
  octomap
  octomath
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS octomap_to_occupancy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
