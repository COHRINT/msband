cmake_minimum_required(VERSION 2.8.3)
project(msband)
find_package(catkin REQUIRED COMPONENTS
		    std_msgs
		    sensor_msgs
		    message_generation)
		    
add_message_files(
  FILES
  Heartrate.msg
  GSR.msg
  DeviceContact.msg
  SkinTemp.msg
  AmbientLight.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
   DEPENDS )
