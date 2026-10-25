cmake_minimum_required(VERSION 3.16)
project(rollchain_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config urdf rviz launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
