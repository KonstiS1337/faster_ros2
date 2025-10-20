cmake_minimum_required(VERSION 3.8)
project(jps3d)

# Use C++11 and show warnings
add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to release build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add the JPS library
add_library(jps_lib
  src/jps_planner/graph_search.cpp
  src/jps_planner/jps_planner.cpp
)

ament_target_dependencies(jps_lib
  rclcpp
  pcl_ros
  yaml-cpp
)

target_link_libraries(jps_lib
  ${PCL_LIBRARIES}
  yaml-cpp
)

# Optional test executable
add_executable(create_map test/create_map.cpp)
target_link_libraries(create_map
  yaml-cpp
  ${PCL_LIBRARIES}
)

ament_export_dependencies(
  pcl_ros
  rclcpp
  yaml-cpp
  PCL
)
ament_export_include_directories(include)   
ament_export_libraries(jps_lib)        
ament_export_targets(jps3dTargets HAS_LIBRARY_TARGET)

# Install targets
install(TARGETS jps_lib
  EXPORT jps3dTargets          # <-- export the target
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT jps3dTargets
  FILE jps3dTargets.cmake
  NAMESPACE jps3d::            # <-- gives  jps3d::jps_lib
  DESTINATION share/jps3d/cmake
)

ament_package()
