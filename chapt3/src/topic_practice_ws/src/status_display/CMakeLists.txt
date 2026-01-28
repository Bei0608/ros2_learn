cmake_minimum_required(VERSION 3.8)
project(status_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(status_interfaces REQUIRED)

# 2. 开启 MOC (Qt必须)
set(CMAKE_AUTOMOC ON)

# ==========================================
# 目标 1: hello_qt (纯 Qt 测试程序)
# ==========================================
add_executable(hello_qt src/hello_qt.cpp)
# 必须链接 Qt 库，否则找不到头文件
target_link_libraries(hello_qt Qt5::Widgets)

# ==========================================
# 目标 2: sys_status_display (ROS 2 + Qt 节点)
# ==========================================
add_executable(sys_status_display src/sys_status_display.cpp)

# 链接 ROS 相关的依赖 (rclcpp, 接口包)
ament_target_dependencies(sys_status_display 
  rclcpp 
  status_interfaces
)

# 链接 Qt 相关的依赖
target_link_libraries(sys_status_display
  Qt5::Widgets
)

# ==========================================
# 安装规则
# ==========================================
install(TARGETS 
        hello_qt
        sys_status_display
      DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()