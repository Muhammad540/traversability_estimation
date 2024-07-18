cmake_minimum_required(VERSION 3.5)
project(traversability_estimation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(traversability_estimation_filters REQUIRED)
find_package(traversability_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(filters REQUIRED)
# find_package(param_io QUIET) 
find_package(Eigen3 REQUIRED)
find_package(kindr QUIET)
find_package(PkgConfig REQUIRED)
if(NOT kindr_FOUND)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

## Specify additional locations of header files
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  src/TraversabilityMap.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  grid_map_ros
  grid_map_core
  grid_map_msgs
  grid_map_filters
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  traversability_estimation_filters
  traversability_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  Eigen3
  kindr
  filters
  # param_io 
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
  src/traversability_estimation_node.cpp
  src/TraversabilityEstimation.cpp
)

add_executable(yaml_parsing_test
  src/test_yaml_parsing.cpp
)

ament_target_dependencies(yaml_parsing_test rclcpp)

install(TARGETS yaml_parsing_test
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

## Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install targets
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

## Install include directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

## Install other files (e.g. launch and config files)
install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  grid_map_ros
  grid_map_core
  grid_map_msgs
  grid_map_filters
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  traversability_estimation_filters
  traversability_msgs
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  Eigen3
  kindr
  # param_io 
)

ament_package()
