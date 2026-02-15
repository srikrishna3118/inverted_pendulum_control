cmake_minimum_required(VERSION 3.8)
project(inverted_pendulum_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  control/control_pid.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  model
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
