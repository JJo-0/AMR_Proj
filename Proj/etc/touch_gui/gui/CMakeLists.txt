cmake_minimum_required(VERSION 3.5)
project(gui)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveGoal.srv"
)

ament_package()