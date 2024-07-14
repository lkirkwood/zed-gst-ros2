cmake_minimum_required(VERSION 3.8)
project(zed-gst)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utsma_lifecycle_manager_msgs REQUIRED)
find_package(utsma_common REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0)

add_executable(streamer src/main.cpp src/zed_streamer.cpp)

target_include_directories(streamer PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(streamer PkgConfig::gstreamer)

ament_target_dependencies(streamer rclcpp std_msgs lifecycle_msgs rclcpp_lifecycle utsma_lifecycle_manager_msgs utsma_common)


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

install(TARGETS streamer DESTINATION lib/${PROJECT_NAME})
ament_package()
