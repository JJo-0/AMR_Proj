cmake_minimum_required(VERSION 3.5)
project(gui)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()