cmake_minimum_required(VERSION 3.14)
project(legged_ros2_control)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Dependencies
# =============================================================================

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(ROS_DEPENDENCIES
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclpy
  realtime_tools
  rosgraph_msgs
  std_msgs
  urdf
)

foreach(dep ${ROS_DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()

# External dependencies
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")
find_package(unitree_sdk2 REQUIRED)
find_package(spdlog REQUIRED)

# =============================================================================
# Libraries
# =============================================================================

# Legged system interface (hardware plugin)
add_library(legged_system_interface SHARED src/legged_system_interface.cpp)
target_include_directories(legged_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(legged_system_interface
  PRIVATE LEGGED_ROS2_CONTROL_BUILDING_LIBRARY
)
ament_target_dependencies(legged_system_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(unitree_system_interface SHARED 
  src/robots/g1_system_interface.cpp
  src/robots/go2_system_interface.cpp
)
target_compile_features(unitree_system_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(unitree_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(unitree_system_interface
  legged_system_interface
  $<BUILD_INTERFACE:unitree_sdk2>
)
ament_target_dependencies(unitree_system_interface ${ROS_DEPENDENCIES})
# Ensure correct CycloneDDS libraries are used at runtime
# (avoids conflict with ROS Humble's version which lacks libddscxx)
set_target_properties(unitree_system_interface PROPERTIES
  LINK_FLAGS "-Wl,--disable-new-dtags,-rpath,/usr/local/lib"
)


# Legged ros2 control library
add_library(legged_ros2_control_lib SHARED src/legged_ros2_control.cpp)
target_include_directories(legged_ros2_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(legged_ros2_control_lib legged_system_interface)
ament_target_dependencies(legged_ros2_control_lib
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  rosgraph_msgs
  std_msgs
)

# =============================================================================
# Executables
# =============================================================================

set(UNITREE_NODE_DEPS
  controller_manager_msgs
  geometry_msgs
  rclcpp
  std_msgs
)

# G1 robot node
add_executable(g1_node node/g1_node.cpp)
target_include_directories(g1_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(g1_node legged_ros2_control_lib unitree_sdk2 spdlog::spdlog)
ament_target_dependencies(g1_node ${UNITREE_NODE_DEPS})
set_target_properties(g1_node PROPERTIES
  LINK_FLAGS "-Wl,--disable-new-dtags,-rpath,/usr/local/lib"
)

# Go2 robot node
add_executable(go2_node node/go2_node.cpp)
target_include_directories(go2_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(go2_node legged_ros2_control_lib unitree_sdk2 spdlog::spdlog)
ament_target_dependencies(go2_node ${UNITREE_NODE_DEPS})
set_target_properties(go2_node PROPERTIES
  LINK_FLAGS "-Wl,--disable-new-dtags,-rpath,/usr/local/lib"
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/legged_teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Installation
# =============================================================================

pluginlib_export_plugin_description_file(legged_ros2_control legged_ros2_control_plugins.xml)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS legged_system_interface legged_ros2_control_lib unitree_system_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS g1_node go2_node
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Export
# =============================================================================

ament_export_include_directories(include)
ament_export_libraries(legged_system_interface legged_ros2_control_lib unitree_system_interface)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${ROS_DEPENDENCIES})

ament_package()
