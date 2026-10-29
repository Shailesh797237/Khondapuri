cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(MODBUS REQUIRED libmodbus)

# =====================
# Library (hardware plugin)
# =====================
add_library(my_robot_hardware SHARED
  src/mobile_base_hardware_interface.cpp
  src/plc_sto_handler.cpp
)

target_include_directories(my_robot_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MODBUS_INCLUDE_DIRS}
)

target_link_libraries(my_robot_hardware
  ${MODBUS_LIBRARIES}
  snap7
)

ament_target_dependencies(
  my_robot_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
)

# =====================
# Plugin export (IMPORTANT)
# =====================
pluginlib_export_plugin_description_file(
  hardware_interface
  my_robot_hardware_interface.xml
)

# =====================
# Install
# =====================
install(
  TARGETS my_robot_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES my_robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
)

ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(my_robot_hardware)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)

# include_directories(include)


# ament_package()
