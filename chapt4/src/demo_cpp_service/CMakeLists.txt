cmake_minimum_required(VERSION 3.8)
project(demo_cpp_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(chapt4_interfaces REQUIRED)  # ← 关键

# 可执行文件
add_executable(turtle_control src/turtle_control.cpp)

ament_target_dependencies(turtle_control
  rclcpp
  geometry_msgs
  turtlesim
  chapt4_interfaces
)

install(TARGETS
  turtle_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
