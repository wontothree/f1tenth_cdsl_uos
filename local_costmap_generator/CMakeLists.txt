cmake_minimum_required(VERSION 3.5)
project(local_costmap_generator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2_ros)
find_package(tf2_eigen)
find_package(PCL REQUIRED)

find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(OpenMP REQUIRED)

# Include Cpp "include" directory
include_directories(include/local_costmap_generator)

# Create Cpp executable
add_executable(local_costmap_generator_node src/local_costmap_generator.cpp src/local_costmap_generator_node.cpp)
ament_target_dependencies(local_costmap_generator_node 
  rclcpp
  sensor_msgs
  laser_geometry
  tf2
  pcl_conversions
  pcl_ros
  geometry_msgs
  tf2_ros
  tf2_eigen
  PCL
  grid_map_ros
  grid_map_core
  grid_map_cv
  grid_map_msgs
  OpenMP
)


# Install Cpp executables
install(TARGETS
  local_costmap_generator_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

