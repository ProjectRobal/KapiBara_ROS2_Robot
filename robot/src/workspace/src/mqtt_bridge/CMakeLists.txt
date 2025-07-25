cmake_minimum_required(VERSION 3.8)
project(mqtt_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kapibara_interfaces REQUIRED)

find_library(PahoMqttC_LIBRARY libpaho-mqtt3as.so.1 REQUIRED)
find_library(PahoMqttCpp_LIBRARY libpaho-mqttpp3.so.1 REQUIRED)

if(nlohmann_json_FOUND)
  include_directories(${nlohmann_json_INCLUDE_DIRS})
else()
  message(STATUS "nlohmann_json NOT Found !")
endif(nlohmann_json_FOUND)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB all_SRCS
        "include/${PROJECT_NAME}/*.h"
        "include/${PROJECT_NAME}/*.hpp"
        "include/*.h"
        "include/*.hpp"
        "src/*.cpp"
        "src/*.c"
        )

    
add_executable(mqtt_bridge ${all_SRCS})

target_link_libraries(mqtt_bridge
  ${PahoMqttC_LIBRARY}
  ${PahoMqttCpp_LIBRARY}
)

ament_target_dependencies(mqtt_bridge rclcpp kapibara_interfaces)

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
  TARGETS mqtt_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
