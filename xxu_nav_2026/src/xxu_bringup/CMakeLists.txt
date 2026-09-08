cmake_minimum_required(VERSION 3.8)
project(xxu_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auto_initial_pose.py
    scripts/mock_robot_io.py
    scripts/nav2_navigation_startup.py
    scripts/wait_for_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
