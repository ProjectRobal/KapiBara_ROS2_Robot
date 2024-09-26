cmake_minimum_required(VERSION 3.8)
project(vel_saltis_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kapibara_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuCFG.msg"
  "msg/FusionCFG.msg"
  "msg/MotorCFG.msg"
  "msg/PIDCFG.msg"
  "msg/ServoCFG.msg"
  "srv/GetFusionCFG.srv"
  "srv/SetFusionCFG.srv"
  "srv/GetImuCFG.srv"
  "srv/SetImuCFG.srv"
  "srv/GetMotorCFG.srv"
  "srv/SetMotorCFG.srv"
  "srv/GetPIDCFG.srv"
  "srv/SetPIDCFG.srv"
  "srv/GetServoCFG.srv"
  "srv/SetServoCFG.srv"
  DEPENDENCIES geometry_msgs kapibara_interfaces
)

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

ament_package()
