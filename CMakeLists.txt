cmake_minimum_required(VERSION 3.5)
project(ssc_joystick)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/ssc_joystick.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_exec
  src/ssc_joystick_exec.cpp
)

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify()
endif()

#############
## Install ##
#############

# Install
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
