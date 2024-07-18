cmake_minimum_required(VERSION 3.5)
project(traversability_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootprintPath.msg"
  "msg/TraversabilityResult.msg"
  "srv/CheckFootprintPath.srv"
  "srv/Overwrite.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
