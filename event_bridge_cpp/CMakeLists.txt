cmake_minimum_required(VERSION 3.8)
project(event_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(event_camera_legacy_tools REQUIRED)
find_package(libcaer_driver REQUIRED)
find_package(event_camera_tools REQUIRED)
find_package(event_camera_renderer REQUIRED)

add_executable(bridge_node src/bridge_node.cpp)

target_include_directories(bridge_node PRIVATE include)

ament_target_dependencies(
  bridge_node
  rclcpp
  event_camera_msgs
  event_camera_codecs
  event_camera_legacy_tools
  libcaer_driver
  event_camera_tools
  event_camera_renderer
)

install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
