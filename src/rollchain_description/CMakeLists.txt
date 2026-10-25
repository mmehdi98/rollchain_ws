cmake_minimum_required(VERSION 3.16)
project(rollchain_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(
  DIRECTORY config urdf rviz launch
  DESTINATION share/${PROJECT_NAME}/
)

# Prints the URDF for a robot config, with link meshes generated from it.
install(PROGRAMS scripts/rollchain_urdf DESTINATION lib/${PROJECT_NAME})

# Link meshes for every config in config/, generated here and installed under
# share/, so the URDF can reference them as package:// paths that resolve on any
# machine with the package. Each config gets its own directory, named after the
# hash of its geometry: a launch uses the installed meshes when the hash matches,
# and falls back to a local cache when it does not.
file(GLOB CONFIGS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/config/*.yaml)
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/meshes.stamp
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_BINARY_DIR}/meshes
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rollchain_urdf
          --build-meshes ${CMAKE_CURRENT_BINARY_DIR}/meshes ${CONFIGS}
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/meshes.stamp
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rollchain_urdf ${CONFIGS}
  COMMENT "Generating the rollchain link meshes"
  VERBATIM
)
add_custom_target(rollchain_meshes ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/meshes.stamp)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
