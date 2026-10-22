cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/diff_drive_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
