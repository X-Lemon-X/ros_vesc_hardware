cmake_minimum_required(VERSION 3.8)
project(vesc_hardware)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set required dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  can_device
  ari_shared_types
)

set(CMAKE_CXX_STANDARD 20)
# find dependencies
# find_package(ros2_control_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# add_executable(barometer_node hardware/barometer_node.cpp)
# ament_target_dependencies(barometer_node rclcpp hardware_interface)
# target_link_libraries(barometer_node vesc_hardware)
# install(TARGETS barometer_node DESTINATION lib/${PROJECT_NAME})


add_library(${PROJECT_NAME} 
  SHARED
  src/vesc_hardware.cpp  
)

# target_link_libraries(
#   ${PROJECT_NAME}
#   INTERFACE
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)


# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY  description/ros2_control # description/urdf description/launch
  DESTINATION share/${PROJECT_NAME}
)


# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/${PROJECT_NAME}
# )

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# # old
# install(TARGETS 
#   ${PROJECT_NAME} 
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   DIRECTORY include/
#   DESTINATION include
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
