cmake_minimum_required(VERSION 3.8)
project(lidar3d_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common io filters registration
)

# ── ndt_localizer node ────────────────────────────────────────────
add_executable(ndt_localizer
  src/ndt_localizer.cpp
)

target_include_directories(ndt_localizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(ndt_localizer
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(ndt_localizer
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  pcl_ros
  yaml-cpp
)

# ── Install ───────────────────────────────────────────────────────
install(TARGETS ndt_localizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
