cmake_minimum_required(VERSION 3.8)
project(battery_communication)

find_package(ament_cmake REQUIRED)

# Install executable Python scripts
install(PROGRAMS
  scripts/battery_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
