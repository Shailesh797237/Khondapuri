cmake_minimum_required(VERSION 3.8)
project(amr_pure_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── ROS2 dependencies ─────────────────────────────────────────────
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(tf2_ros       REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp      REQUIRED)

# ── Find snap7 (installed from source to /usr/local) ─────────────
find_library(SNAP7_LIB
  NAMES snap7
  PATHS /usr/local/lib /usr/lib
  NO_DEFAULT_PATH
)
find_path(SNAP7_INCLUDE
  NAMES snap7.h
  PATHS /usr/local/include /usr/include
  NO_DEFAULT_PATH
)

if(SNAP7_LIB AND SNAP7_INCLUDE)
  set(SNAP7_FOUND TRUE)
  message(STATUS "snap7 found:")
  message(STATUS "  lib     : ${SNAP7_LIB}")
  message(STATUS "  include : ${SNAP7_INCLUDE}")
else()
  set(SNAP7_FOUND FALSE)
  message(WARNING
    "snap7 NOT found — plc_s7_bridge will NOT be built.\n"
    "Build from source:\n"
    "  git clone https://github.com/SCADA-LTS/snap7.git\n"
    "  cd snap7/build/unix && make -f x86_64_linux.mk all\n"
    "  sudo cp ../bin/x86_64-linux/libsnap7.so /usr/local/lib/\n"
    "  sudo cp ../../src/sys/snap7.h /usr/local/include/\n"
    "  sudo ldconfig")
endif()

# ── Common include dir ────────────────────────────────────────────
set(PKG_INCLUDE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── 1. Waypoint Navigator ─────────────────────────────────────────
add_executable(waypoint_navigator
  src/waypoint_navigator.cpp
)
target_include_directories(waypoint_navigator PUBLIC ${PKG_INCLUDE})
ament_target_dependencies(waypoint_navigator
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros tf2_geometry_msgs)
target_link_libraries(waypoint_navigator yaml-cpp)

# ── 2. AMR Teleop ─────────────────────────────────────────────────
add_executable(amr_teleop
  src/amr_teleop.cpp
)
target_include_directories(amr_teleop PUBLIC ${PKG_INCLUDE})
ament_target_dependencies(amr_teleop
  rclcpp geometry_msgs)

# ── 2b. TCP Mission Bridge ────────────────────────────────────────
add_executable(tcp_mission_bridge
  src/tcp_mission_bridge.cpp
)
target_include_directories(tcp_mission_bridge PUBLIC ${PKG_INCLUDE})
ament_target_dependencies(tcp_mission_bridge
  rclcpp std_msgs)

# ── 3. PLC S7 Bridge ──────────────────────────────────────────────
if(SNAP7_FOUND)
  add_executable(plc_s7_bridge
    src/plc_s7_bridge.cpp
  )
  target_include_directories(plc_s7_bridge PUBLIC
    ${PKG_INCLUDE}
    ${SNAP7_INCLUDE}
  )
  ament_target_dependencies(plc_s7_bridge
    rclcpp std_msgs)
  target_link_libraries(plc_s7_bridge
    yaml-cpp
    ${SNAP7_LIB}
  )
  install(TARGETS plc_s7_bridge
    DESTINATION lib/${PROJECT_NAME})

  # ── 4. PLC Station Monitor (simple single-bit monitor) ──────────
  add_executable(plc_station_monitor
    src/plc_station_monitor.cpp
  )
  target_include_directories(plc_station_monitor PUBLIC
    ${PKG_INCLUDE}
    ${SNAP7_INCLUDE}
  )
  ament_target_dependencies(plc_station_monitor
    rclcpp std_msgs)
  target_link_libraries(plc_station_monitor
    ${SNAP7_LIB}
  )
  install(TARGETS plc_station_monitor
    DESTINATION lib/${PROJECT_NAME})

  # ── 5. PLC Lidar Monitor (front lidar obstacle bit DB1.DBX0.0) ──
  add_executable(plc_lidar_monitor
    src/plc_lidar_monitor.cpp
  )
  target_include_directories(plc_lidar_monitor PUBLIC
    ${PKG_INCLUDE}
    ${SNAP7_INCLUDE}
  )
  ament_target_dependencies(plc_lidar_monitor
    rclcpp std_msgs)
  target_link_libraries(plc_lidar_monitor
    ${SNAP7_LIB}
  )
  install(TARGETS plc_lidar_monitor
    DESTINATION lib/${PROJECT_NAME})

  # ── 6. PLC Emergency Monitor (emergency button DB1.DBX0.5) ──────
  add_executable(plc_emergency_monitor
    src/plc_emergency_monitor.cpp
  )
  target_include_directories(plc_emergency_monitor PUBLIC
    ${PKG_INCLUDE}
    ${SNAP7_INCLUDE}
  )
  ament_target_dependencies(plc_emergency_monitor
    rclcpp std_msgs)
  target_link_libraries(plc_emergency_monitor
    ${SNAP7_LIB}
  )
  install(TARGETS plc_emergency_monitor
    DESTINATION lib/${PROJECT_NAME})
endif()

# ── Install ───────────────────────────────────────────────────────
install(TARGETS
  waypoint_navigator
  amr_teleop
  tcp_mission_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# ── Install Python scripts ────────────────────────────────────────
install(PROGRAMS
  scripts/pose_persistence.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_persistence
)

install(PROGRAMS
  scripts/mission_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_publisher
)


ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(amr_pure_navigation)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
# set(CMAKE_CXX_STANDARD 17)

# find_package(ament_cmake   REQUIRED)
# find_package(rclcpp        REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs      REQUIRED)
# find_package(sensor_msgs   REQUIRED)
# find_package(std_msgs      REQUIRED)
# find_package(yaml-cpp      REQUIRED)

# # ── Waypoint Navigator ────────────────────────────────────────────
# add_executable(waypoint_navigator
#   src/waypoint_navigator.cpp
# )
# target_include_directories(waypoint_navigator PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(waypoint_navigator
#   rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
# target_link_libraries(waypoint_navigator yaml-cpp)

# # ── AMR Teleop ────────────────────────────────────────────────────
# add_executable(amr_teleop
#   src/amr_teleop.cpp
# )
# ament_target_dependencies(amr_teleop
#   rclcpp geometry_msgs)

# # ── Install ───────────────────────────────────────────────────────
# install(TARGETS
#   waypoint_navigator
#   amr_teleop
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION include)

# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME})

# ament_package()




#work_done

# cmake_minimum_required(VERSION 3.8)
# project(amr_pure_navigation)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
# set(CMAKE_CXX_STANDARD 17)

# find_package(ament_cmake   REQUIRED)
# find_package(rclcpp        REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs      REQUIRED)
# find_package(sensor_msgs   REQUIRED)
# find_package(std_msgs      REQUIRED)
# find_package(yaml-cpp      REQUIRED)

# add_executable(waypoint_navigator
#   src/waypoint_navigator.cpp
# )

# target_include_directories(waypoint_navigator PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(waypoint_navigator
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   std_msgs
# )

# target_link_libraries(waypoint_navigator yaml-cpp)

# install(TARGETS waypoint_navigator
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION include)

# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME})

# ament_package()





#currect cmake 


# cmake_minimum_required(VERSION 3.8)
# project(amr_pure_navigation)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
# set(CMAKE_CXX_STANDARD 17)

# find_package(ament_cmake   REQUIRED)
# find_package(rclcpp        REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs      REQUIRED)
# find_package(sensor_msgs   REQUIRED)
# find_package(yaml-cpp      REQUIRED)

# add_executable(waypoint_navigator
#   src/waypoint_navigator.cpp
# )

# target_include_directories(waypoint_navigator PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(waypoint_navigator
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
# )

# target_link_libraries(waypoint_navigator yaml-cpp)

# install(TARGETS waypoint_navigator
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION include)

# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME})

# ament_package()