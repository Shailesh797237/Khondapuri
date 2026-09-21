cmake_minimum_required(VERSION 3.8)
project(dual_scan_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

# -------- Manual Angle Merge Node --------
add_executable(dual_scan_merger src/dual_scan_merger.cpp)

ament_target_dependencies(dual_scan_merger
  rclcpp
  sensor_msgs
  message_filters
)

# -------- TF Based Merger --------
add_executable(tf_scan_merger src/tf_merger.cpp)

ament_target_dependencies(tf_scan_merger
  rclcpp
  sensor_msgs
  laser_geometry
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

# -------- Install Both Nodes --------
install(TARGETS
  dual_scan_merger
  tf_scan_merger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()