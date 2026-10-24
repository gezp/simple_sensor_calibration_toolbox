cmake_minimum_required(VERSION 3.8)
project(ssct_calib_client)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/web_client.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY webui
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
