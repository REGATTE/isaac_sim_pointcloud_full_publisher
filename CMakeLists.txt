cmake_minimum_required(VERSION 3.0.2)
project(isaac_sim_pointcloud_full_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

## Find PCL
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros pcl_conversions
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(full_pcd_pub src/full_pcd_pub.cpp)

target_link_libraries(full_pcd_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS full_pcd_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY Params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Params
)
