/*
 * ClassifierNodelet.cpp
 *
 *  Created on: Jan 9, 2015
 *      Author: kazuto
 */

#include "Classifier.h"
#include "FeatureDescriptor.h"
#include "ServiceRobot.h"

#include <boost/shared_ptr.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace activity_recognition_server {

//----------------------------------------------------------------------------------
// Classifier nodelet
//----------------------------------------------------------------------------------

class ClassifierNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  int encoding_mode_;
  boost::shared_ptr<Classifier> classifier_;

public:
  virtual void onInit()
  {
    NODELET_INFO("Classifier nodelet");

    ros::AsyncSpinner spinner(0);
    encoding_mode_ = 0;
    nh_ = getMTNodeHandle();
    nh_private_ = getMTPrivateNodeHandle();
    classifier_.reset(new Classifier(&nh_, encoding_mode_));

    spinner.start();
  }
};

PLUGINLIB_EXPORT_CLASS(activity_recognition_server::ClassifierNodelet, nodelet::Nodelet)

//----------------------------------------------------------------------------------
// Android device nodelet
//----------------------------------------------------------------------------------
class FeatureDescriptorNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  boost::shared_ptr<FeatureDescriptor> descriptor_;

public:
  virtual void onInit()
  {
    NODELET_INFO("FeatureDescriptor nodelet");

    ros::AsyncSpinner spinner(0);
    nh_ = getMTNodeHandle();
    nh_private_ = getMTPrivateNodeHandle();
    descriptor_.reset(new FeatureDescriptor(&nh_));

    spinner.start();
  }
};

PLUGINLIB_EXPORT_CLASS(activity_recognition_server::FeatureDescriptorNodelet, nodelet::Nodelet)

//----------------------------------------------------------------------------------
// ServiceRobot nodelet
//----------------------------------------------------------------------------------
class ServiceRobotNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  boost::shared_ptr<ServiceRobot> robot_;

public:
  virtual void onInit()
  {
    NODELET_INFO("ServiceRobot nodelet");

    ros::AsyncSpinner spinner(0);
    nh_ = getMTNodeHandle();
    nh_private_ = getMTPrivateNodeHandle();
    robot_.reset(new ServiceRobot(&nh_));

    spinner.start();
  }
};

PLUGINLIB_EXPORT_CLASS(activity_recognition_server::ServiceRobotNodelet, nodelet::Nodelet)
}
