/*
 * ServiceRobot.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: kazuto
 */

#include "ServiceRobot.h"

//----------------------------------------------------------------------------------
// ServiceRobot
//----------------------------------------------------------------------------------
ServiceRobot::ServiceRobot(ros::NodeHandle* nh) :
    nh_(nh), user_activity_(-1)
{
  ROS_INFO("ServiceRobot constructor");

  // initialize some nodes
  server_status_ = nh_->advertise<std_msgs::String>("server_status", 1);
  user_status_ = nh_->subscribe("user_activity", 1, &ServiceRobot::setActivity, this);
  voice_server_ = nh_->advertiseService("user_voice_command", &ServiceRobot::voiceCallBack, this);
  tts_client_ = nh_->serviceClient<activity_recognition::robot_tts>("smartpal5_tts");
}

//----------------------------------------------------------------------------------
ServiceRobot::~ServiceRobot()
{
  ROS_INFO("ServiceRobot destructor");
}

//----------------------------------------------------------------------------------
void ServiceRobot::setActivity(const std_msgs::StringConstPtr& msg)
{
  // assign a label
  if (!msg->data.compare("eat_a_meal"))
    user_activity_ = 0;
  else if (!msg->data.compare("gaze_at_a_robot"))
  {
    user_activity_ = 1;
    this->robotTTS(1);
  }
  else if (!msg->data.compare("gaze_at_a_tree"))
    user_activity_ = 2;
  else if (!msg->data.compare("look_around"))
  {
    user_activity_ = 3;
    this->robotTTS(1);
  }
  else if (!msg->data.compare("read_a_book"))
    user_activity_ = 4;
  else
    user_activity_ = -1;
}

//----------------------------------------------------------------------------------
// TODO: refering the tms database
bool ServiceRobot::voiceCallBack(activity_recognition::user_voice::Request &req,
                                 activity_recognition::user_voice::Response &res)
{
  ROS_INFO_STREAM("Received text: " << req.text);
  if (!req.text.compare("bring me some water"))
  {
    // taking-water service
    this->robotTTS(0);
    res.result = 1;
  }
  else
  {
    ROS_INFO("invalid voice command");
    res.result = 0;
  }
  return true;
}

//----------------------------------------------------------------------------------
void ServiceRobot::robotTTS(int service)
{
  activity_recognition::robot_tts srv;

  switch (service)
  {
    case 0: // taking-water service
      switch (user_activity_)
      {
        case 0: // eating
          srv.request.text = "Would you need a bottle of tea ?";
          break;
        case 2: // planting
          srv.request.text = "Would you need a watering can ?";
          break;
        case 4: // reading
          srv.request.text = "Would you need a cup of coffee ?";
          break;
        default:
          srv.request.text = "What kind of water would you need ?";
          break;
      }
      break;
    case 1: // pro-active suggestion
      switch (user_activity_)
      {
        case 1: // gazing at the robot
          srv.request.text = "Can I help you ?";
          break;
        case 3: // looking around
          srv.request.text = "Are you looking for anything ?";
          break;
        default:
          break;
      }
      break;
  }

  // service call
  if (tts_client_.call(srv))
  {
    ROS_INFO_STREAM("Service robot: " << srv.request.text);
  }
  else
  {
    ROS_ERROR("Failed to call tts service");
  }
}
