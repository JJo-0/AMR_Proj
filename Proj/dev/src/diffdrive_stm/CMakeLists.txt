cmake_minimum_required(VERSION 3.5)
project(diffdrive_stm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

add_library(diffdrive_stm SHARED src/diffdrive_stm.cpp src/wheel.cpp src/my_node.cpp)

target_include_directories(
  diffdrive_stm
  PRIVATE
  include
)
ament_target_dependencies(
  diffdrive_stm
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
  std_msgs
)
# 추가한 헤더 파일이 있는 디렉토리를 포함시킵니다.


pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

add_library(fake_robot SHARED src/fake_robot.cpp src/wheel.cpp)
set_property(TARGET fake_robot PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  fake_robot
  PRIVATE
  include
)
ament_target_dependencies(
  fake_robot
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface fake_robot_hardware.xml)

install(
  TARGETS diffdrive_stm fake_robot
  DESTINATION lib
)

install(
  DIRECTORY controllers launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  diffdrive_stm
  fake_robot
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

ament_package()