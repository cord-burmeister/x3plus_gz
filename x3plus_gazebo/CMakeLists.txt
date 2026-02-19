cmake_minimum_required(VERSION 3.8)
project(x3plus_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pt_joint_command_bridge src/pt_joint_command_bridge.cpp)
ament_target_dependencies(pt_joint_command_bridge rclcpp std_msgs)

install(
  TARGETS pt_joint_command_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install the config files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install the launch files
install(
  DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the worlds files
install(
  DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
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
