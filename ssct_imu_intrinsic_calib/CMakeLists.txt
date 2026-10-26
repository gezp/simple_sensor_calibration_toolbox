cmake_minimum_required(VERSION 3.5)
project(ssct_imu_intrinsic_calib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(ssct_common REQUIRED)
find_package(ssct_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

include_directories(include ${CERES_INCLUDE_DIRS})

set(dependencies
  ssct_common
  ssct_interfaces
)

add_library(${PROJECT_NAME} SHARED
  src/allan_calib/allan_plot.cpp
  src/allan_calib/allan_calibrator.cpp
  src/imu_tk_calib/imu_tk_plot.cpp
  src/imu_tk_calib/static_pose_collector.cpp
  src/imu_tk_calib/imu_tk_calibrator.cpp
  src/calibration_node.cpp
  src/utils.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

add_executable(calibration_node
  src/calibration_main.cpp)
target_link_libraries(calibration_node ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config resource
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
