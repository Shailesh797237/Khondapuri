cmake_minimum_required(VERSION 3.8)
project(plc_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  /usr/local/include
)

link_directories(
  /usr/local/lib
)

add_library(${PROJECT_NAME}_lib SHARED
  src/plc_bridge.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib
  snap7
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
)

add_executable(plc_bridge_node src/plc_bridge_node.cpp)
target_link_libraries(plc_bridge_node ${PROJECT_NAME}_lib)
ament_target_dependencies(plc_bridge_node
  rclcpp
  std_msgs
)

add_executable(plc_communication_node src/plc_bridge_node.cpp)
target_link_libraries(plc_communication_node ${PROJECT_NAME}_lib)
ament_target_dependencies(plc_communication_node
  rclcpp
  std_msgs
)

install(TARGETS
  ${PROJECT_NAME}_lib
  plc_bridge_node
  plc_communication_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp std_msgs)

ament_package()
