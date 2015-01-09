/*
 * ClassifierNodelet.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: kazuto
 */

#include "Classifier.h"
#include "AndroidDevice.h"
#include "ServiceRobot.h"

#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace activity_recognition {

//----------------------------------------------------------------------------------
// Classifier nodelet
//----------------------------------------------------------------------------------
Classifier* classifier_;

class ClassifierNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  int encoding_mode_;

public:
  virtual void onInit()
  {
    NODELET_INFO("Classifier nodelet");

    encoding_mode_ = 0;
    nh_ = getMTNodeHandle();
    nh_private_ = getMTPrivateNodeHandle();

    classifier_ = new Classifier(&nh_, encoding_mode_);
  }
};

PLUGINLIB_DECLARE_CLASS(activity_recognition, ClassifierNodelet,
                        activity_recognition::ClassifierNodelet, nodelet::Nodelet)

//----------------------------------------------------------------------------------
// Android device nodelet
//----------------------------------------------------------------------------------
class AndroidDeviceNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  AndroidDevice* android_device_;

public:
  virtual void onInit()
  {
    NODELET_INFO("Android device nodelet");
    nh_ = getMTNodeHandle();
    nh_private_ = getMTPrivateNodeHandle();

    android_device_ = new AndroidDevice(&nh_, classifier_);
  }
};

PLUGINLIB_DECLARE_CLASS(activity_recognition, AndroidDeviceNodelet,
                        activity_recognition::AndroidDeviceNodelet, nodelet::Nodelet)

//----------------------------------------------------------------------------------
// ServiceRobot nodelet
//----------------------------------------------------------------------------------
class ServiceRobotNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ServiceRobot* service_robot_;

public:
  virtual void onInit()
  {
    NODELET_INFO("ServiceRobot nodelet");

    nh_ = getMTNodeHandle();
    nh_private_ = getMTPrivateNodeHandle();

    service_robot_ = new ServiceRobot(&nh_);
  }
};

PLUGINLIB_DECLARE_CLASS(activity_recognition, ServiceRobotNodelet,
                        activity_recognition::ServiceRobotNodelet, nodelet::Nodelet)
}
