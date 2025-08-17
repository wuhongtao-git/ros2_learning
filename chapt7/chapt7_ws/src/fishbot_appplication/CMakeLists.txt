cmake_minimum_required(VERSION 3.8)
project(fishbot_appplication)

# 设置 C++ 版本为 C++17（必须）
set(CMAKE_CXX_STANDARD 17)

# 编译器选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找所有依赖（重要修复顺序）
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)      # 先于其他依赖
find_package(lifecycle_msgs REQUIRED)      # 先于 Nav2 依赖
find_package(nav2_msgs REQUIRED)            # 包含 Nav2 消息
find_package(nav2_util REQUIRED)            # Nav2 工具库
find_package(rclcpp REQUIRED)               # ROS2 C++ 客户端
find_package(rclcpp_action REQUIRED)        # ROS2 动作支持

include_directories(
  SYSTEM
  /opt/ros/${ROS_DISTRO}/include
  ${nav2_util_INCLUDE_DIRS}
)

# 替代方案：显式检查找到的包（Debug）
message(STATUS "Found geometry_msgs: ${geometry_msgs_FOUND}")
message(STATUS "Found nav2_msgs: ${nav2_msgs_FOUND}")
message(STATUS "Found nav2_util: ${nav2_util_FOUND}")
message(STATUS "Found lifecycle_msgs: ${lifecycle_msgs_FOUND}")

# 创建可执行文件
add_executable(init_robot_pose src/init_robot_pose.cpp)

# 链接依赖（非命名空间版本）
target_link_libraries(init_robot_pose PRIVATE
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav2_msgs_LIBRARIES}
  ${nav2_util_LIBRARIES}
  ${lifecycle_msgs_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
)

# 如果非命名空间版本无效，尝试以下替代：
# target_link_libraries(fishbot_appplication PRIVATE
#   geometry_msgs::geometry_msgs
#   nav2_msgs::nav2_msgs
#   nav2_util::nav2_util
#   lifecycle_msgs::lifecycle_msgs
#   rclcpp::rclcpp
#   rclcpp_action::rclcpp_action
# )

# 安装可执行文件
install(TARGETS
  init_robot_pose
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖（确保包含所有依赖）
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav2_msgs
  nav2_util
  lifecycle_msgs
  rclcpp_action
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()