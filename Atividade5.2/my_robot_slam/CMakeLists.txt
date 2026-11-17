cmake_minimum_required(VERSION 3.8)
project(my_robot_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf worlds DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/odom_tf_node.py
  scripts/waypoint_driver.py
  scripts/slam_logger.py
  scripts/save_map.py
  scripts/analyze.py
  scripts/analyze_kidnap.py
  scripts/run_slam_experiment.sh
  scripts/run_kidnap.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
