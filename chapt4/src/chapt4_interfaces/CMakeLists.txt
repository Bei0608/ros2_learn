cmake_minimum_required(VERSION 3.8)
project(chapt4_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # ROSIDL 默认生成器
find_package(sensor_msgs REQUIRED)   # 图像消息接口  sensor_msgs/smg/Image


# 注册、声明自定义接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FaceDetector.srv"   # 添加自定义服务接口文件
  DEPENDENCIES sensor_msgs  # 指定依赖的消息包
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
