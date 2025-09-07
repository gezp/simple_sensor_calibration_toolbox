cmake_minimum_required(VERSION 3.5)
project(ssct_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CalibrationStatus.msg"
  "msg/CalibrationCommand.msg"
  "msg/CameraIntrinsicParam.msg"
  "msg/ExtrinsicParam.msg"
DEPENDENCIES
  builtin_interfaces
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
