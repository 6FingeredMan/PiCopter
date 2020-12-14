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
  Altitude_msg.msg
  Autopilot_msg.msg 
  Elevation_msg.msg
  Gps_msg.msg
  Imu_msg.msg
  Motors_msg.msg
  Navigator_msg.msg
  Sim_msg.msg
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
  src/StandardAltitude.cpp
  src/SuperTwistingSMC.cpp)
target_link_libraries(autopilot ${catkin_LIBRARIES})
add_dependencies(autopilot picopter_generate_messages_cpp)

## Build the navigator node
add_executable(navigator
  src/navigator_node.cpp
  src/navigator.cpp)
target_link_libraries(navigator ${catkin_LIBRARIES})
add_dependencies(navigator picopter_generate_messages_cpp)

## Build the simulator node
add_executable(simulator
  src/simulator_node.cpp
  src/simulator.cpp)
target_link_libraries(simulator ${catkin_LIBRARIES})
add_dependencies(simulator picopter_generate_messages_cpp)

## Build the simulated AHRS node
add_executable(ahrs_sim
  src/ahrs_sim_node.cpp
  src/ahrs_sim.cpp)
target_link_libraries(ahrs_sim ${catkin_LIBRARIES})
add_dependencies(ahrs_sim picopter_generate_messages_cpp)

catkin_install_python(PROGRAMS 
  scripts/PCA9685.py
  scripts/VL53L1X.py
  scripts/berryGPS.py
  scripts/bmp388.py
  scripts/bmp388_sim.py
  scripts/berryGPS_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)