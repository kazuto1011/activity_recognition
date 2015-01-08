/*
 * ServiceRobot.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: kazuto
 */

#include "ServiceRobot.h"

#define STATE_CHECK 0

namespace xpr = boost::xpressive;

//----------------------------------------------------------------------------------
// ServiceRobot
//----------------------------------------------------------------------------------
ServiceRobot::ServiceRobot(ros::NodeHandle* nh) :
    nh_(nh), user_activity_(-1)
{
  ROS_INFO("ServiceRobot constructor");

  // initialize the tag_list_
  tag_list_.push_back("water");
  tag_list_.push_back("drink");
  tag_list_.push_back("coffee");
  tag_list_.push_back("bottle");

#if 0
  vec_itr itr = tag_list_.begin();
  hash_table_.insert(std::make_pair("water",&(*itr)));
  hash_table_.insert(std::make_pair("waterr",&(*itr)));
  hash_table_.insert(std::make_pair("waterrr",&(*itr)));
  itr++;
  hash_table_.insert(std::make_pair("drink",&(*itr)));
  hash_table_.insert(std::make_pair("drinkk",&(*itr)));
  hash_table_.insert(std::make_pair("drinkkk",&(*itr)));
#endif

  // initialize some nodes
  server_status_ = nh_->advertise<std_msgs::String>("server_status", 1);
  user_status_ = nh_->subscribe("user_activity", 1, &ServiceRobot::setActivity, this);
  voice_server_ = nh_->advertiseService("user_voice_command", &ServiceRobot::voiceCallBack, this);
  //tts_client_ = nh_->serviceClient<activity_recognition::robot_tts>("smartpal5_tts");
  tts_client_ = nh_->serviceClient<activity_recognition::robot_tts>("test");
  tms_db_client_ = nh_->serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");

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
  {
    user_activity_ = 0;
  }
  else if (!msg->data.compare("gaze_at_a_robot"))
  {
    user_activity_ = 1;
    this->robotTTS(3, NULL);
  }
  else if (!msg->data.compare("gaze_at_a_tree"))
  {
    user_activity_ = 2;
  }
  else if (!msg->data.compare("look_around"))
  {
    user_activity_ = 3;
    this->robotTTS(4, NULL);
  }
  else if (!msg->data.compare("read_a_book"))
  {
    user_activity_ = 4;
  }
  else
  {
    user_activity_ = -1;
  }
}

#if 0 // this part never refers tms database
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
#else
typedef std::vector<std::string>::iterator str_vec_itr;
typedef std::vector<tms_msg_db::Tmsdb>::iterator tmsdb_vec_itr;

//----------------------------------------------------------------------------------
bool ServiceRobot::voiceCallBack(activity_recognition::user_voice::Request &req,
                                 activity_recognition::user_voice::Response &res)
{
  std::string received_text = req.text;
  std::vector<std::string> match_list;

  std::cout << "-------------------------------------\n";
  ROS_INFO("Received: \033[1;32m%s\033[0m", received_text.c_str());

  for (str_vec_itr itr = tag_list_.begin(); itr != tag_list_.end(); itr++)
  {
    if (strstr(received_text.c_str(), (*itr).c_str()))
    {
      ROS_INFO("Tag: \033[1;34m%s\033[0m", (*itr).c_str());
      match_list.push_back(*itr);
    }
  }

  if (!match_list.empty())
  {
    // something has matched
    tms_msg_db::TmsdbGetData srv;
    for (str_vec_itr itr = match_list.begin(); itr != match_list.end(); itr++)
    {
      srv.request.tmsdb.tag = *itr;
      if (tms_db_client_.call(srv))
      {
        ROS_INFO("Search database for \"%s\"", srv.request.tmsdb.tag.c_str());
      }
      else
      {
        ROS_ERROR("Failed to call service");
        return false;
      }

      if (srv.response.tmsdb.empty())
      {
        ROS_INFO("No matched tag on database");
      }
      else
      {
        for (int i = 0; i < srv.response.tmsdb.size(); i++)
        {
#if STATE_CHECK
          if (srv.response.tmsdb[i].state == 1) {
#endif
            object_list_.push_back(srv.response.tmsdb[i]);
            ROS_INFO("matched: \033[1;32m%s\033[0m", srv.response.tmsdb[i].name.c_str());
#if STATE_CHECK
          }
          else
          {
            ROS_ERROR("Unavailable ID: %d", srv.request.tmsdb.id);
          }
#endif
        }
      }
    }
  }
  else
  {
    ROS_ERROR("Not supported command");
    this->robotTTS(5, "");
    res.result = 0;
    return false;
  }

  if (!object_list_.empty())
  {
    // e.g "greentea_bottle" -> "green tea bottle"
    for (tmsdb_vec_itr itr = object_list_.begin(); itr != object_list_.end(); itr++)
    {
      size_t c;
      std::string space = " ";
      while((c = itr->name.find_first_of("_")) != std::string::npos)
      {
        itr->name.replace(c, space.length(), space);
      }
      if((c = itr->name.find("tea")) != std::string::npos)
      {
        itr->name.insert(c, " ");
      }
    }

    // TODO: Sort the object list

    // TODO: Step into an another thread to communicate with user
    // Robot: Would you need XX ?
    // User:  No
    // Robot: Would you need XX ?
    // User:  Yes
    // Robot: Okay, I'll bring you XX ?

    this->robotTTS(0, object_list_[0].name);
    res.result = 1;
  }
  else
  {
    ROS_INFO("No object has listed");
    this->robotTTS(6, "");
    res.result = 0;
  }

  std::vector<tms_msg_db::Tmsdb>().swap(object_list_);
  return true;
}


//----------------------------------------------------------------------------------
void ServiceRobot::robotTTS(int service, std::string object_name)
{
  activity_recognition::robot_tts srv;

  switch (service)
  {
    case 0:
      srv.request.text = "Would you need a " + object_name + " ?";
      break;
    case 1:
      srv.request.text = "Would you like a " + object_name + " ?";
      break;
    case 2:
      srv.request.text = "What kind of water would you need ?";
      break;
    case 3:
      srv.request.text = "Can I help you ?";
      break;
    case 4:
      srv.request.text = "Are you looking for anything ?";
      break;
    case 5:
      srv.request.text = "I'm afraid but I don't understand what you mentioned.";
      break;
    case 6:
      srv.request.text = " ";
      break;
    default:
      break;
  }

  // service call
  if (tts_client_.call(srv))
  {
    ROS_INFO("\033[1;34m%s\033[0m", srv.request.text.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call tts service");
  }
}
#endif
