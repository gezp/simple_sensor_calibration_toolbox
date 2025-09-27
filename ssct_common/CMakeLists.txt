cmake_minimum_required(VERSION 3.5)
project(ssct_common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ssct_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
# find_package(backward_ros REQUIRED)

include_directories(include)

# libaray
set(dependencies
  rclcpp
  pcl_ros
  tf2_ros
  tf2_eigen
  cv_bridge
  geometry_msgs
  sensor_msgs
  ssct_interfaces
)

add_library(${PROJECT_NAME} SHARED
  # publisher
  src/publisher/cloud_publisher.cpp
  src/publisher/image_publisher.cpp
  src/publisher/imu_publisher.cpp
  # subscriber
  src/subscriber/image_subscriber.cpp
  src/subscriber/imu_subscriber.cpp
  # dummy_sensor
  src/dummy_sensor/dummy_camera_node.cpp
  src/dummy_sensor/dummy_lidar_node.cpp
  src/dummy_sensor/dummy_imu_node.cpp
  # utils
  src/calib_param_manager.cpp
  src/msg_utils.cpp
  src/tic_toc.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${CERES_LIBRARIES}
)

# dummy lidar node
add_executable(dummy_lidar_node
  src/dummy_sensor/dummy_lidar_main.cpp)
target_link_libraries(dummy_lidar_node ${PROJECT_NAME})

# dummy camera node
add_executable(dummy_camera_node
  src/dummy_sensor/dummy_camera_main.cpp)
target_link_libraries(dummy_camera_node ${PROJECT_NAME})

# dummy imu node
add_executable(dummy_imu_node
  src/dummy_sensor/dummy_imu_main.cpp)
target_link_libraries(dummy_imu_node ${PROJECT_NAME})

# install include directories
install(DIRECTORY include/
  DESTINATION include
)

# install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install executables
install(TARGETS
    dummy_lidar_node
    dummy_camera_node
    dummy_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# install executable scripts
install(PROGRAMS
  scripts/calibration_client.py
  DESTINATION lib/${PROJECT_NAME})

# install directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_dependencies(Ceres)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
