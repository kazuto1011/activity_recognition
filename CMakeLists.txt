cmake_minimum_required(VERSION 2.8.3)
project(activity_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  compressed_image_transport
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  robot_tts.srv
  user_voice.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES activity_recognition
   CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport compressed_image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

#VLFeat include directory
set(VLFeat_INCLUDE_DIRS "~/vlfeat-0.9.19")

# VLFeat library path
set(VLFeat_LIBRARY "~/vlfeat-0.9.19/bin/glnxa64/libvl.so")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${VLFeat_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## server node
add_executable(activity_recognition_server
  src/activity_recognition_server.cpp
  src/vector_encoding.cpp
  src/Classifier.cpp
  src/AndroidDevice.cpp
  src/ServiceRobot.cpp
  libsvm/svm.cpp
)
add_dependencies(activity_recognition_server activity_recognition_generate_messages_cpp)
target_link_libraries(activity_recognition_server
  ${VLFeat_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

## smartpal tts for demo
add_executable(virtual_smartpal
  src/virtual_smartpal.cpp
)
add_dependencies(virtual_smartpal activity_recognition_generate_messages_cpp)
target_link_libraries(virtual_smartpal
  ${catkin_LIBRARIES}
)
