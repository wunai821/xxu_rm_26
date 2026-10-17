cmake_minimum_required(VERSION 3.8)
project(xxu_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps models rviz waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auto_initial_pose.py
    scripts/amcl_particle_diagnostic.py
    scripts/nav2_navigation_startup.py
    scripts/wait_for_odom.py
    scripts/rm_sim_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
