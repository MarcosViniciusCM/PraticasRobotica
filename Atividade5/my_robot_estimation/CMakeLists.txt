cmake_minimum_required(VERSION 3.8)
project(my_robot_estimation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/noisy_sensors.py
  scripts/state_logger.py
  scripts/analyze.py
  scripts/run_experiment.sh
  scripts/make_video.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
