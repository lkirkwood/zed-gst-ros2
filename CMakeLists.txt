cmake_minimum_required(VERSION 3.8)
project(zed-gst)

add_compile_options(-Wall -Wextra -Wpedantic)
include_directories(include)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utsma_lifecycle_manager_msgs REQUIRED)
find_package(utsma_common REQUIRED)

find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
add_definitions(${GSTREAMER_CFLAGS})

add_executable(stream src/main.cpp)
ament_target_dependencies(stream rclcpp std_msgs lifecycle_msgs rclcpp_lifecycle utsma_lifecycle_manager_msgs)

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
