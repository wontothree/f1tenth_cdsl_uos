cmake_minimum_required(VERSION 3.5)
project(local_planner)

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

include_directories(include/)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(local_planner_node 
  src/local_planner_node.cpp
  src/local_planner_ros.cpp
)

ament_target_dependencies(local_planner_node
  rclcpp
  grid_map_msgs
  nav_msgs
  ackermann_msgs
  Eigen3
)

# target_include_directories(local_planner_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

target_include_directories(local_planner_node PRIVATE ${EIGEN3_INCLUDE_DIR})


install(TARGETS local_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
