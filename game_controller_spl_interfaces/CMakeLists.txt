cmake_minimum_required(VERSION 3.8)
project(game_controller_spl_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # RCGCD 14
  msg/RCGCD14.msg
  msg/RobotInfo14.msg
  msg/TeamInfo14.msg

  # RCGCD 15
  msg/RCGCD15.msg
  msg/RobotInfo15.msg
  msg/TeamInfo15.msg

  # RCGCRD 4
  msg/RCGCRD4.msg
)

# export dependencies
ament_export_dependencies(rosidl_default_runtime)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
