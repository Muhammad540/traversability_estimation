cmake_minimum_required(VERSION 3.5)
project(traversability_estimation_filters)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  src/SlopeFilter.cpp
  src/StepFilter.cpp
  src/RoughnessFilter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
  grid_map_ros
  grid_map_msgs
  filters
  pluginlib
  Eigen3
)

pluginlib_export_plugin_description_file(filters filter_plugins.xml)

## Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/filters/
  DESTINATION include/${PROJECT_NAME}/filters
)

install(FILES filter_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  grid_map_core
  grid_map_ros
  grid_map_msgs
  filters
  pluginlib
  Eigen3
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
