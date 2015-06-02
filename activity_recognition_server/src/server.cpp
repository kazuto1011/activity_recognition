/*
 * activity_recognition_server.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: kazuto
 */

#include "common.h"

#include "FeatureDescriptor.h"
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
  FeatureDescriptor descriptor(&nh);
  ServiceRobot service_robot(&nh);

  spinner.start();

  ROS_INFO("Multi threading");
  ros::waitForShutdown();

  return 0;
}
