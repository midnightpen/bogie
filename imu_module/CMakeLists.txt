cmake_minimum_required(VERSION 3.0.2)
project(imu_module)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
  message_generation
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs)


################################################
## Declare ROS messages, services and actions ##
################################################






################################################
## Declare ROS dynamic reconfigure parameters ##
################################################




###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_port
  CATKIN_DEPENDS roscpp serial rospy std_msgs message_runtime message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS} 
)


add_executable(serial_imu src/serial_imu.cpp src/ch_serial.c)
target_link_libraries(serial_imu ${catkin_LIBRARIES}) 

add_executable(sub_spec src/sub_spec.cpp)
add_dependencies(sub_spec ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                          ${catkin_EXPORTED_TARGETS} 
                          ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(sub_spec ${catkin_LIBRARIES})


#############
## Install ##
#############





#############
## Testing ##
#############

