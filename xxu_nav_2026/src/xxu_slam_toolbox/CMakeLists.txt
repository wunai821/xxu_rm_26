cmake_minimum_required(VERSION 3.8)
project(xxu_slam_toolbox)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/map_autosaver.py
    scripts/save_map.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
