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

//----------------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "activity_recognition_server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);

  ROS_INFO_STREAM(INIT_ENCODE);

  int encoding_mode = 0;
  std::cin >> encoding_mode;

  Classifier classifier(&nh, encoding_mode);
  AndroidDevice android_device(&nh, &classifier);
  ServiceRobot service_robot(&nh);

  ROS_INFO("Multi threading");
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
