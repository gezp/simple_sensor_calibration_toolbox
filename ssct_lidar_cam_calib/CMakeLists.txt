cmake_minimum_required(VERSION 3.5)
project(ssct_lidar_cam_calib)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(ssct_common REQUIRED)
find_package(ssct_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(backward_ros REQUIRED)

include_directories(include)

# libaray
set(dependencies
  ssct_common
  ssct_interfaces
)

add_library(${PROJECT_NAME} SHARED
  src/image_undistorter.cpp
  src/manual_calibration_node.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
)

# manual calibration node
add_executable(manual_calibration_node
  src/manual_calibration_main.cpp)

target_link_libraries(manual_calibration_node ${PROJECT_NAME})


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
    manual_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
