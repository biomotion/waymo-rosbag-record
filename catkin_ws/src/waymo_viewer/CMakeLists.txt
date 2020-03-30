cmake_minimum_required(VERSION 2.8.3)
project(waymo_viewer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation 
)

add_message_files(
  FILES
  Box.msg
  Metadata.msg
  Label.msg
  Lidar.msg
  CameraImage.msg
  CameraImageArray.msg
  LidarArray.msg
  VehicleInfo.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waymo_viewer
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

