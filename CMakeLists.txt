cmake_minimum_required(VERSION 2.8.3)
project(imu_gps_fusion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_gps_fusion
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(imu_gps_fusion src/imu_gps_fusion.cpp src/wgs_conversions.cpp)

add_executable(imu_gps_fusion_node src/imu_gps_fusion_node.cpp)
target_link_libraries(imu_gps_fusion_node ${catkin_LIBRARIES} imu_gps_fusion)
