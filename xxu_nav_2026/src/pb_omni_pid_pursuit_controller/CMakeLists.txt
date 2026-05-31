# 最低 CMake 版本要求
cmake_minimum_required(VERSION 3.10)
# 项目名称，同时作为库名
project(pb_omni_pid_pursuit_controller)

# 使用 C++17 标准（ROS2 Jazzy 推荐）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 ament_cmake_auto 构建系统
# ament_auto 自动处理 include 路径和链接，简化 CMake 配置
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 构建共享库（.so），包含以下源文件：
#   - omni_pid_pursuit_controller.cpp: 全向 PID 纯追踪控制器
#   - pid.cpp: 离散 PID 控制器
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
)

# 代码检查与格式化（仅在测试构建时启用）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 排除 uncrustify（使用 clang_format 代替）
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

# 导出插件描述文件，使 Nav2 能通过 pluginlib 发现并加载本控制器
pluginlib_export_plugin_description_file(nav2_core pb_omni_pid_pursuit_controller.xml)

# ament_auto_package: 自动处理安装规则、ament index 注册等
ament_auto_package()
