cmake_minimum_required(VERSION 3.8)
project(xxu_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/cmd_vel_watchdog.py
    scripts/cmd_vel_odometry.py
    scripts/gimbal_stabilizer.py
    scripts/gimbal_imu_compensator.py
    scripts/imu_frame_republisher.py
    scripts/pointcloud_frame_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
