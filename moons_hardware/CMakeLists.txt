cmake_minimum_required(VERSION 3.8)
project(moons_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

include_directories(${MODBUS_INCLUDE_DIRS})
link_directories(${MODBUS_LIBRARY_DIRS})


include_directories(include)

add_executable(moons_hardware src/moons_hardware.cpp)

ament_target_dependencies(  
  moons_hardware
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(
  moons_hardware
  ${MODBUS_LIBRARIES}
)


install(
  TARGETS moons_hardware
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()




# cmake_minimum_required(VERSION 3.8)
# project(moons_hardware)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)

# include_directories(include)

# add_executable(moons_hardware src/moons_hardware.cpp)
# ament_terget_dependencies(moons_hardware rclcpp hardware_interface pluginlib)

# install(TARGETS
#   moons_hardware
#   DESTINATION lib/${PROJECT_NAME}/
# )


# ament_package()
