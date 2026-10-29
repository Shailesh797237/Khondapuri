cmake_minimum_required(VERSION 3.8)
project(lidar3d_obstacle_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters octree)

# ── obstacle_detector_3d node ─────────────────────────────────────
add_executable(obstacle_detector_3d
  src/obstacle_detector_3d.cpp
)

target_include_directories(obstacle_detector_3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(obstacle_detector_3d
  ${PCL_LIBRARIES}
)

ament_target_dependencies(obstacle_detector_3d
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  pcl_conversions
  pcl_ros
)

# ── Install ───────────────────────────────────────────────────────
install(TARGETS obstacle_detector_3d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
