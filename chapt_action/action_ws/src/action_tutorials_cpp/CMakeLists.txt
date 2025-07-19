cmake_minimum_required(VERSION 3.8)
project(action_tutorials_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(fibonacci_action_server_node src/fibonacci_action_server.cpp)
ament_target_dependencies(fibonacci_action_server_node rclcpp action_tutorials_interfaces rclcpp_action)

add_executable(fibonacci_action_client_node src/fibonacci_action_client.cpp)
ament_target_dependencies(fibonacci_action_client_node rclcpp action_tutorials_interfaces rclcpp_action)

install(TARGETS
        fibonacci_action_server_node fibonacci_action_client_node
        DESTINATION lib/${PROJECT_NAME})

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
