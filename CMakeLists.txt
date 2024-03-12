cmake_minimum_required(VERSION 3.8)
project(marron_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(marron_driver SHARED
  src/kvh1750_driver_component.cpp
  src/motor_command_sender_component.cpp
  src/serial_bridge_component.cpp)
rclcpp_components_register_nodes(marron_driver
  "marron_driver::KVH1750DriverComponent")
rclcpp_components_register_nodes(marron_driver
  "marron_driver::MotorCommandSenderComponent")
rclcpp_components_register_nodes(marron_driver
  "drivers::serial_driver::SerialBridgeComponent")

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
