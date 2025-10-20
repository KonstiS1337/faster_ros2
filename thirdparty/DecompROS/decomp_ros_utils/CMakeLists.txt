cmake_minimum_required(VERSION 3.5)
project(decomp_ros_utils)

# Require C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)           
find_package(rviz_default_plugins REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(faster_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Include directories
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${decomp_util_INCLUDE_DIRS}
  include
)

# Automoc for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

# Source files
set(SOURCE_FILES
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/polyhedron_array_display.cpp
)

# Create library
add_library(decomp_rviz_plugins SHARED ${SOURCE_FILES})

# Link libraries
target_link_libraries(decomp_rviz_plugins
  Qt5::Widgets
)

ament_target_dependencies(decomp_rviz_plugins
  rclcpp
  Eigen3
  decomp_util
  decomp_ros_msgs
  rviz_common
  rviz_default_plugins
  pluginlib
  nav_msgs
  faster_msgs
)

# Export plugin descriptions for rviz2 if needed
# (Usually via pluginlib XML, not shown here)

# Install library
install(TARGETS decomp_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers if any
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
