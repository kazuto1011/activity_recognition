/*
 * activity_recognition_server.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: kazuto
 */

#include "common.h"

#include "AndroidDevice.h"
#include "Classifier.h"
#include "ServiceRobot.h"
#include <pthread.h>

namespace fs = boost::filesystem;

//----------------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "activity_recognition_server");
  ros::NodeHandle nh;

  ros::MultiThreadedSpinner spinner(0);

  pthread_t classifier_thread;
  pthread_t android_thread;
  pthread_t robot_thread;

  // execute human activity classifier
  Classifier classifier(&nh);

  // execute Android device subscriber
  AndroidDevice android_device(&nh, &classifier);

  // execute virtual service-robot
  ServiceRobot service_robot(&nh);

  if (pthread_create(&classifier_thread, NULL, Classifier::run_thread, &classifier))
  {
      ROS_ERROR("could not create a thread");
      abort();
  }
  if (pthread_create(&android_thread, NULL, AndroidDevice::run_thread, &android_device))
  {
      ROS_ERROR("could not create a thread");
      abort();
  }
  if (pthread_create(&robot_thread, NULL, ServiceRobot::run_thread, &service_robot))
  {
      ROS_ERROR("could not create a thread");
      abort();
  }

  ROS_INFO("Multi threading");
  spinner.spin();
  return 0;
}
