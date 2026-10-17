cmake_minimum_required(VERSION 3.8)
project(key_controll)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/wasd_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
