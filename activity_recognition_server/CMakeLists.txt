cmake_minimum_required(VERSION 2.8.3)
project(activity_recognition_server)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  nodelet
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

## Set the plugin name
set(PLUGIN_NAME ActivityRecognitionServer)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  robot_tts.srv
  user_voice.srv
  classify.srv
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
  INCLUDE_DIRS include
  LIBRARIES ${PLUGIN_NAME}
  CATKIN_DEPENDS pluginlib nodelet roscpp rospy std_msgs sensor_msgs cv_bridge image_transport compressed_image_transport
)

###########
## Build ##
###########

#VLFeat include directory
set(VLFeat_INCLUDE_DIRS "~/vlfeat")

# VLFeat library path
set(VLFeat_LIBRARY "~/vlfeat/bin/glnxa64/libvl.so")

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${VLFeat_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## nodelet library
add_library(${PLUGIN_NAME}
  src/Nodelets.cpp
  src/Classifier.cpp
  src/FeatureDescriptor.cpp
  src/ServiceRobot.cpp
  src/vector_encoding.cpp
  libsvm/svm.cpp
)
add_dependencies(${PLUGIN_NAME} activity_recognition_server_generate_messages_cpp)
target_link_libraries(${PLUGIN_NAME}
  ${VLFeat_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

## nodelet manager
add_executable(nodelet_server
  src/nodelet_server.cpp
)
add_dependencies(nodelet_server activity_recognition_server_generate_messages_cpp)
target_link_libraries(nodelet_server
  ${catkin_LIBRARIES}
)

## server node
add_executable(activity_recognition_server
  src/server.cpp
  src/vector_encoding.cpp
  src/Classifier.cpp
  src/FeatureDescriptor.cpp
  src/ServiceRobot.cpp
  libsvm/svm.cpp
)
add_dependencies(activity_recognition_server activity_recognition_server_generate_messages_cpp tms_msg_db_generate_messages_cpp)
target_link_libraries(activity_recognition_server
  ${VLFeat_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

## smartpal tts for demo
add_executable(virtual_smartpal
  src/virtual_smartpal.cpp
)
add_dependencies(virtual_smartpal activity_recognition_server_generate_messages_cpp)
target_link_libraries(virtual_smartpal
  ${catkin_LIBRARIES}
)
