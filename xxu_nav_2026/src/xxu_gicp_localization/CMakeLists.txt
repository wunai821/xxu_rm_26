cmake_minimum_required(VERSION 3.16)
project(xxu_gicp_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(small_gicp REQUIRED)

add_executable(gicp_localization_node
  src/gicp_localization_node.cpp
)

target_include_directories(gicp_localization_node PRIVATE
  include
)
target_compile_options(gicp_localization_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(gicp_localization_node
  small_gicp::small_gicp
)
ament_target_dependencies(gicp_localization_node
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS gicp_localization_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/generate_complex_mapping_pcd.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
