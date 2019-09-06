cmake_minimum_required(VERSION 2.8.3)
project(sia_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs joy roscpp sensor_msgs
)

set(SIA_TELEOP_SOURCES
  src/DiffdriveTeleop.cpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sia_teleop_joy ${SIA_TELEOP_SOURCES})
target_link_libraries(sia_teleop_joy ${catkin_LIBRARIES})
add_dependencies(sia_teleop_joy ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS 
  sia_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark headers for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark launchfiles for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)