cmake_minimum_required(VERSION 2.8.3)
project(lgsvl_simulator)

find_package(catkin REQUIRED COMPONENTS
  autoware_vehicle_msgs
  geo_pos_conv
  perception_launch
  roscpp
  sensing_launch
  tf
  vehicle_launch
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lgsvl_simulator
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_raw_proxy
  src/imu_raw_node.cpp
  src/imu_raw_proxy.cpp
)

target_link_libraries(imu_raw_proxy
  ${catkin_LIBRARIES}
)

add_executable(odom_to_twist
  src/odom_to_twist_core.cpp
  src/odom_to_twist_node.cpp
)

target_link_libraries(odom_to_twist
  ${catkin_LIBRARIES}
)

add_executable(nmea2tfpose
  src/nmea2tfpose_core.cpp
  src/nmea2tfpose_node.cpp
)

target_link_libraries(nmea2tfpose
  ${catkin_LIBRARIES}
)

add_executable(points_raw_proxy
  src/points_raw_node.cpp
  src/points_raw_proxy.cpp
)

target_link_libraries(points_raw_proxy
  ${catkin_LIBRARIES}
)

add_executable(vehicle_cmd_proxy
  src/vehicle_cmd_node.cpp
  src/vehicle_cmd_proxy.cpp
)

target_link_libraries(vehicle_cmd_proxy
  ${catkin_LIBRARIES}
)

install(TARGETS imu_raw_proxy odom_to_twist nmea2tfpose points_raw_proxy vehicle_cmd_proxy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    config
    data
    launch
    rviz
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
