cmake_minimum_required(VERSION 3.8)
project(pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # 生成编译命令

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # 添加依赖

# [Node] publisher : 编译 链接 ROS 安装
add_executable(publisher # 可执行文件名，也就是节点(运行的文件)名
  app/publisher.cpp # 源文件
)
# target_link_libraries(video_reader ${OpenCV_LIBRARIES}) # 如果有
ament_target_dependencies(publisher
  rclcpp    # 添加 ROS 包: rclcpp
  std_msgs  # 添加 ROS 包: std_msgs
)
install(TARGETS     publisher           # 安装的可执行文件
        DESTINATION lib/${PROJECT_NAME} # 安装的位置 (这里会比较奇怪，一般二进制放bin，但是遵循ROS的习惯)
)

# [Node] subscriber : 编译 链接 ROS 安装
add_executable(subscriber app/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)
install(TARGETS subscriber DESTINATION lib/${PROJECT_NAME})


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
