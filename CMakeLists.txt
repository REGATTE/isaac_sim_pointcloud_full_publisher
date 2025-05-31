cmake_minimum_required(VERSION 3.0.2)
project(isaac_sim_pointcloud_full_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add the executable
add_executable(full_pcd_pub src/full_pcd_pub.cpp)
target_link_libraries(full_pcd_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install the executable - make sure this is before catkin_package
install(TARGETS full_pcd_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install parameter files
install(
  DIRECTORY Params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Params
)

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros pcl_conversions
)