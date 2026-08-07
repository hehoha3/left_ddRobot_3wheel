cmake_minimum_required(VERSION 3.8)
project(test_dd_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(test_dd_bot_hardware SHARED src/test_dd_bot_hardware.cpp)
target_include_directories(test_dd_bot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(test_dd_bot_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle)
target_compile_features(test_dd_bot_hardware PUBLIC cxx_std_17)

add_executable(demo_motion_node src/demo_motion_node.cpp)
target_compile_features(demo_motion_node PUBLIC cxx_std_17)
ament_target_dependencies(demo_motion_node rclcpp geometry_msgs)

install(TARGETS
  test_dd_bot_hardware
  demo_motion_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config description DESTINATION share/${PROJECT_NAME})
install(FILES test_dd_bot_hardware.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(hardware_interface test_dd_bot_hardware.xml)

ament_export_dependencies(rclcpp geometry_msgs hardware_interface pluginlib rclcpp_lifecycle)
ament_package()