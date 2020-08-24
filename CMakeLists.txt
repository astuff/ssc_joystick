cmake_minimum_required(VERSION 2.8.3)
project(ssc_joystick)

add_definitions(-std=c++11 -Wall -Wextra)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_msgs
  genmsg
  automotive_platform_msgs
  automotive_navigation_msgs
  message_generation
  message_filters
  pacmod_msgs
  joy
)

add_message_files(DIRECTORY msg FILES
  TractorControlMode.msg
)

generate_messages(DEPENDENCIES std_msgs)


## Declare a catkin package
catkin_package(CATKIN_DEPENDS
  std_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  message_runtime
)

## Build executables
include_directories(include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ssc_joystick
  src/ssc_joystick_ros.cpp
)

target_link_libraries(ssc_joystick ASCommon ${catkin_LIBRARIES})
add_dependencies(ssc_joystick ${catkin_EXPORTED_TARGETS})

roslint_cpp()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES "ssc_joystick.json"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}

)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
