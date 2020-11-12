cmake_minimum_required(VERSION 2.8.3)
project(picopter)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  genmsg
  message_generation
)

## Declare ROS messages and services
add_message_files(
  FILES 
  Elevation.msg
  Gps.msg
  Imu.msg
  Motors.msg
)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build the nodes
include_directories(include ${catkin_INCLUDE_DIRS})

## Build the ahrs node
add_executable(ahrs 
  src/ahrs_node.cpp 
  src/RazorAHRS.cpp)
target_link_libraries(ahrs ${catkin_LIBRARIES})
add_dependencies(ahrs picopter_generate_messages_cpp)

## Build the autopilot node
add_executable(autopilot
  src/autopilot_node.cpp
  src/autopilot.cpp
  src/Controllers.cpp
  src/StandardPID.cpp
  src/SuperTwistingSMC.cpp)
target_link_libraries(autopilot ${catkin_LIBRARIES})
add_dependencies(autopilot picopter_generate_messages_cpp)

