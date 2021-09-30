cmake_minimum_required(VERSION 2.8.3)
project(ssc_joystick)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  nodelet
  std_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  joy
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    LIBRARIES ${PROJECT_NAME}_nodelet
)

###########
## Build ##
###########

## Build executables
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# nodelet
add_library(${PROJECT_NAME}_nodelet
    src/${PROJECT_NAME}_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
)

# node
add_executable(${PROJECT_NAME}_node
    src/ssc_joystick_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

roslint_cpp()

#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
