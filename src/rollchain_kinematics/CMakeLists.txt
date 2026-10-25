cmake_minimum_required(VERSION 3.16)
project(rollchain_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rollchain_interfaces REQUIRED)
find_package(rollchain_model REQUIRED)

# Parameter adapters: ROS parameters <-> rollchain_model structs.
add_library(${PROJECT_NAME}_params STATIC src/ros_params.cpp)
target_include_directories(${PROJECT_NAME}_params PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME}_params PUBLIC rclcpp::rclcpp rollchain_model::rollchain_model)

add_executable(rollchain_state_publisher src/rollchain_state_publisher.cpp)
target_link_libraries(rollchain_state_publisher PRIVATE
  ${PROJECT_NAME}_params
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${rollchain_interfaces_TARGETS})

add_executable(rod_tension_publisher src/rod_tension_publisher.cpp)
target_link_libraries(rod_tension_publisher PRIVATE rclcpp::rclcpp ${rollchain_interfaces_TARGETS})

install(TARGETS rollchain_state_publisher rod_tension_publisher
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/tension_slider_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tension_slider_gui)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ros_params test/test_ros_params.cpp)
  target_link_libraries(test_ros_params ${PROJECT_NAME}_params)
endif()

ament_package()
