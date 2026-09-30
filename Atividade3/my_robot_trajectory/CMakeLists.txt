cmake_minimum_required(VERSION 3.8)
project(my_robot_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(NODES
  pose_controller          # ex. 1.1 - controle de pose continuo
  three_step_controller    # ex. 1.1 (bonus) - rotacao, translacao, rotacao
  mission_follower         # ex. 1.2 - seguimento de waypoints
  figure8_tracker          # ex. 2.1 - trajetoria em 8
)

foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE include)
  target_compile_features(${node} PRIVATE cxx_std_17)
  ament_target_dependencies(${node}
    rclcpp control_toolbox geometry_msgs nav_msgs std_msgs)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/figure8_experiments.sh
  scripts/compare_mission.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
