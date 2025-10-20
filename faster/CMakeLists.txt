cmake_minimum_required(VERSION 3.10)
project(faster)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release build type by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Custom message packages
find_package(faster_msgs REQUIRED)

# JPS / Decomp

find_package(decomp_ros_utils REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
set(DECOMP_ROS_UTILS_INCLUDE_DIRS "/home/ks/orcas/ws/install/decomp_ros_utils/include") #fix this
include_directories(${DECOMP_ROS_UTILS_INCLUDE_DIRS})




find_package(decomp_util REQUIRED)
find_package(jps3d REQUIRED)
message("jps3d found: ${jps3d_FOUND}")
message("Found jps3d lib  ${jps3d_LIBRARIES}")
message("Found jps3d inc dirs ${jps3d_INCLUDE_DIRS}")
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Gurobi
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
message(${CMAKE_MODULE_PATH})
find_package(GUROBI REQUIRED)
if(GUROBI_FOUND)
  message(STATUS "GUROBI FOUND")
else()
  message(FATAL_ERROR "GUROBI NOT FOUND!")
endif()
include_directories(${GUROBI_INCLUDE_DIRS})

# Collect Gurobi libraries
file(GLOB GurobiSOFiles "$ENV{GUROBI_HOME}/lib/libgurobi*[0-9].so")
set(GUROBI_LIBRARIES 
  "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a"
  ${GurobiSOFiles}
  "$ENV{GUROBI_HOME}/lib"
)

# Include project headers
include_directories(
  include
  ${decomp_util_INCLUDE_DIRS}
  ${decomp_ros_utils_INCLUDE_DIRS}
)

# Define executable
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/faster.cpp
  src/faster_ros.cpp
  src/utils.cpp
  src/jps_manager.cpp
  src/solverGurobi.cpp
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${GUROBI_LIBRARIES}
  jps3d::jps_lib
)

# Declare ament dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  faster_msgs
  decomp_util
  decomp_ros_utils
  decomp_ros_msgs
  pcl_conversions
  tf2_geometry_msgs
  tf2_sensor_msgs
  message_filters
  )

# Install target and headers if needed
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory (if you're using headers in `include/`)
install(DIRECTORY include/
  DESTINATION include/
)

# Export package
ament_package()
