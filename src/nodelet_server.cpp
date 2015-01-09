/*
 * activity_recognition_server.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: kazuto
 */

#include "common.h"
#include <nodelet/loader.h>

//----------------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "server");
  ros::AsyncSpinner spinner(0);

  // nodelet manager
  nodelet::Loader manager(true);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // load plugins
  manager.load("classifier", "activity_recognition::ClassifierNodelet", remappings, my_argv);
  manager.load("android", "activity_recognition::AndroidDeviceNodelet", remappings, my_argv);
  manager.load("robot", "activity_recognition::ServiceRobotNodelet", remappings, my_argv);

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
