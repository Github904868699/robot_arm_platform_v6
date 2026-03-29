cmake_minimum_required(VERSION 3.16)
project(robot_arm_hardware_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/backend_factory.cpp
  src/fake_robot_hardware_backend.cpp
  src/joint_router.cpp
  src/real_mixed_robot_backend.cpp
  src/robot_arm_hardware_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface robot_arm_hardware_system_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES robot_arm_hardware_system_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
