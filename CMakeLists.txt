cmake_minimum_required(VERSION 3.16)
project(mecanum_robot LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  mecanum_robot
  SHARED
  hardware/mecanum_system.cpp
  hardware/serial_comm.cpp
)
target_compile_features(mecanum_robot PUBLIC cxx_std_17)
target_include_directories(mecanum_robot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/mecanum_robot>
${LIBSERIAL_INCLUDE_DIRS}
)
target_link_libraries(mecanum_robot PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${LIBSERIAL_LIBRARIES}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mecanum_robot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/mecanum_robot
)
install(
  DIRECTORY config launch description meshes worlds maps
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS mecanum_robot
  EXPORT export_mecanum_robot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_mecanum_robot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
