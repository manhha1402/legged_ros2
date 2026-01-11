find_package(fmt REQUIRED)

add_executable(g1_node
  g1_node.cpp
  ../../../legged_ros2_control.cpp
  ../../../legged_system_interface.cpp
)
target_compile_features(g1_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(g1_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(g1_node ${THIS_PACKAGE_DEPENDS})
target_link_libraries(g1_node
  legged_system_interface
  unitree_sdk2
  fmt::fmt
)

# Ensure correct CycloneDDS libraries are used at runtime
# (avoids conflict with ROS Humble's version which lacks libddscxx)
# RPATH takes precedence over LD_LIBRARY_PATH, unlike RUNPATH
set_target_properties(g1_node PROPERTIES
  LINK_FLAGS "-Wl,--disable-new-dtags,-rpath,/usr/local/lib"
)

install(
  TARGETS g1_node
  DESTINATION lib/${PROJECT_NAME}
)