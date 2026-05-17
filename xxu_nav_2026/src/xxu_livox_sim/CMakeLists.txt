cmake_minimum_required(VERSION 3.8)
project(xxu_livox_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_executable(livox_pointcloud_shaper src/livox_pointcloud_shaper.cpp)
ament_target_dependencies(
  livox_pointcloud_shaper
  rclcpp
  sensor_msgs
)

add_library(livox_mid360_system SHARED src/livox_mid360_system.cpp)
target_link_libraries(
  livox_mid360_system
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-plugin2::gz-plugin2
)

install(
  TARGETS
    livox_pointcloud_shaper
    livox_mid360_system
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
