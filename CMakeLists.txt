cmake_minimum_required(VERSION 2.8.3)
project(irl_robot_drivers)

set(CMAKE_VERBOSE_MAKEFILE on)

add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## export COPPELIA_SIM_PATH="/root/Data/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04"
set(VREP_PATH "$ENV{COPPELIA_SIM_PATH}")


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${VREP_PATH}/programming/remoteApi
  ${VREP_PATH}/programming/include
  include/coppelia_controller
)

## Declare a C++ executable
list(APPEND COPPELIA_CONT_SRC
    src/coppelia_controller_main.cpp
    src/coppelia_controller.cpp
    src/sim_devices/robot_sim.cpp
    ${VREP_PATH}/programming/remoteApi/extApi.c
    ${VREP_PATH}/programming/remoteApi/extApiPlatform.c
)

add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

add_executable(
    coppelia_controller
    ${COPPELIA_CONT_SRC}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(coppelia_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(coppelia_controller
  ${catkin_LIBRARIES}
)
