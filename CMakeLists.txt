cmake_minimum_required(VERSION 2.8.3)
project(sia_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  sensor_msgs
)

find_package(Sialib)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs joy roscpp sensor_msgs
)

set(SIA_TELEOP_SOURCES
  src/DiffdriveTeleop.cpp
  src/AckermannToDiffdriveTeleop.cpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SIALIB_INCLUDE_DIRS}
)

# Joy
add_executable(sia_teleop_joy 
  ${SIA_TELEOP_SOURCES} 
  src/AckermannToDiffdriveTeleopJoy.cpp)

target_link_libraries(sia_teleop_joy 
  ${catkin_LIBRARIES})

add_dependencies(sia_teleop_joy ${catkin_EXPORTED_TARGETS})

# SiA for control using s32k148 board and automotive hardware
if(${SIALIB_FOUND})
  add_executable(sia_teleop_uart
    ${SIA_TELEOP_SOURCES} 
    src/AckermannToDiffdriveTeleopSia.cpp
    src/AckermannToDiffdriveTeleopSiaUart.cpp)

  target_link_libraries(sia_teleop_uart
    ${catkin_LIBRARIES} 
    ${SIALIB_LIBRARIES})

  add_dependencies(sia_teleop_uart ${catkin_EXPORTED_TARGETS})


  add_executable(sia_teleop_udp
    ${SIA_TELEOP_SOURCES} 
    src/AckermannToDiffdriveTeleopSia.cpp
    src/AckermannToDiffdriveTeleopSiaUdp.cpp)

  target_link_libraries(sia_teleop_udp
    ${catkin_LIBRARIES} 
    ${SIALIB_LIBRARIES})

  add_dependencies(sia_teleop_udp ${catkin_EXPORTED_TARGETS})
endif()


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS 
  sia_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(${SIALIB_FOUND})
  install(TARGETS 
    sia_teleop_uart
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(TARGETS 
    sia_teleop_udp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()


## Mark headers for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark launchfiles for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)