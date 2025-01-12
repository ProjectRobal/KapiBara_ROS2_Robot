cmake_minimum_required(VERSION 3.8)
project(gym)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kapibara_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

#add_executable(cpp_executable src/fusion.cpp)
#ament_target_dependencies(cpp_executable rclcpp sensor_msgs geometry_msgs std_msgs kapibara_interfaces)


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


install(
  DIRECTORY description config launch worlds props
  DESTINATION share/${PROJECT_NAME}
)

#install(TARGETS
#  cpp_executable
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
