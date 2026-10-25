cmake_minimum_required(VERSION 3.16)
project(rollchain_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Pure C++ model of the robot: no ROS dependency (see rollchain_architecture.md, decision 2).
add_library(${PROJECT_NAME}
  src/params.cpp
  src/topology.cpp
  src/joints.cpp
  src/mechanics.cpp
  src/tensions.cpp
  src/kinematics.cpp
  src/loads.cpp
  src/statics.cpp
  src/coupled.cpp
  src/outputs.cpp
  src/naming.cpp
  src/model.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_params.cpp
    test/test_topology.cpp
    test/test_joints.cpp
    test/test_mechanics.cpp
    test/test_statics.cpp
    test/test_kinematics.cpp
    test/test_loads.cpp)
  target_include_directories(test_${PROJECT_NAME} PRIVATE test)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_package()
