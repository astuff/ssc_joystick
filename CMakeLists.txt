cmake_minimum_required(VERSION 3.5)
project(ssc_joystick)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(automotive_platform_msgs REQUIRED)
find_package(automotive_navigation_msgs REQUIRED)

include_directories(include)

add_executable(ssc_joystick
  src/ssc_joystick_ros.cpp
)
ament_target_dependencies(
  ssc_joystick
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "automotive_platform_msgs"
  "automotive_navigation_msgs"
)
target_link_libraries(
  ssc_joystick
  ASCommon
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
