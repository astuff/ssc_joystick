cmake_minimum_required(VERSION 2.8.3)
project(joystick_vehicle_test)

add_definitions(-std=c++11 -Wall -Wextra)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  joy
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
  std_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
)

## Build executables
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_vehicle_test
  src/joystick_vehicle_test_ros.cpp
)

add_dependencies(joystick_vehicle_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_vehicle_test ${catkin_LIBRARIES} ASCommon)

roslint_cpp()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES "joystick_vehicle_test.json"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
