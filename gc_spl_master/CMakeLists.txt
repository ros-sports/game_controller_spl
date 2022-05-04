cmake_minimum_required(VERSION 3.8)
project(gc_spl_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcgcd_14 REQUIRED)
find_package(rcgcrd_4 REQUIRED)
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  rcgcd_14
  rcgcrd_4
  Boost)

# build
add_library(gc_spl_node SHARED
  src/gc_spl_node.cpp)
target_include_directories(gc_spl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(gc_spl_node
  PLUGIN "gc_spl_master::GCSPLNode"
  EXECUTABLE gc_spl)

ament_target_dependencies(gc_spl_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS gc_spl_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# package
ament_package()
