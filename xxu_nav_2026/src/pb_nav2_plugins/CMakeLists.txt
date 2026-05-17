cmake_minimum_required(VERSION 3.8)
project(pb_nav2_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

set(dependencies
  geometry_msgs
  laser_geometry
  message_filters
  nav2_behaviors
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  nav2_voxel_grid
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(pb_back_up_free_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)
target_include_directories(pb_back_up_free_space_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pb_back_up_free_space_behavior ${dependencies})

add_library(pb_intensity_voxel_layer SHARED
  src/layers/intensity_voxel_layer.cpp
)
target_include_directories(pb_intensity_voxel_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pb_intensity_voxel_layer ${dependencies})

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    pb_back_up_free_space_behavior
    pb_intensity_voxel_layer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES behavior_plugin.xml costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(pb_back_up_free_space_behavior pb_intensity_voxel_layer)
ament_export_dependencies(${dependencies})

ament_package()
