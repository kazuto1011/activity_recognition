/*activity_recognition
 * activity_recognition_server.cpp
 *
 *  Created on: Jan 9, 2015
 *      Author: kazuto
 */

#include "common.h"
#include <nodelet/loader.h>

//----------------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "activity_recognition_server");

  // nodelet manager
  nodelet::Loader manager(true);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // load plugins
  manager.load("classifier", "activity_recognition_server::ClassifierNodelet", remappings, my_argv);
  manager.load("descriptor", "activity_recognition_server::FeatureDescriptorNodelet", remappings, my_argv);
  manager.load("robot", "activity_recognition_server::ServiceRobotNodelet", remappings, my_argv);
  ros::waitForShutdown();

  return 0;
}
