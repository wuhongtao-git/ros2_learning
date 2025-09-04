cmake_minimum_required(VERSION 3.8)
project(autopatrol_robot_cpp)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 编译器警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(autopatrol_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(patrol_node
  src/patrol_node.cpp
)

add_executable(speaker_node
  src/speaker_node.cpp
)

# 使用ament_target_dependencies添加ROS依赖
ament_target_dependencies(patrol_node 
  rclcpp 
  rclcpp_action 
  geometry_msgs 
  tf2_geometry_msgs 
  nav2_msgs 
  tf2_ros 
  rosgraph_msgs 
  autopatrol_interfaces
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(speaker_node 
  rclcpp 
  autopatrol_interfaces
)

# 安装目标
install(TARGETS patrol_node speaker_node
        DESTINATION lib/${PROJECT_NAME}
)

# 安装配置和启动文件
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()