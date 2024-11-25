cmake_minimum_required(VERSION 3.8)
project(vel_saltis_ears)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_library(vel_saltis_ears SHARED src/vel_saltis_ears.cpp  src/can.cpp src/frame.cpp) 

find_package(kapibara_interfaces REQUIRED)


target_include_directories(
  vel_saltis_ears
  PRIVATE
  include
)
ament_target_dependencies(
  vel_saltis_ears PUBLIC
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
  kapibara_interfaces
)

pluginlib_export_plugin_description_file(hardware_interface vel_saltis_ears.xml)

install(
  TARGETS vel_saltis_ears
  DESTINATION lib
)

ament_package()
