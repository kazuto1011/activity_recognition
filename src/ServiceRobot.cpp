/*
 * ServiceRobot.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: kazuto
 */

#include "ServiceRobot.h"

#define STATE_CHECK 0

//----------------------------------------------------------------------------------
// ServiceRobot
//----------------------------------------------------------------------------------
ServiceRobot::ServiceRobot(ros::NodeHandle* nh) :
    nh_(nh), user_activity_(-1)
{
  ROS_INFO("ServiceRobot constructor");

  // initialize the tag_list_
  tag_list_.push_back("water");
  tag_list_.push_back("tea");
  tag_list_.push_back("coffee");
  tag_list_.push_back("pot");

  // initialize some nodes
  server_status_ = nh_->advertise<std_msgs::String>("server_status", 1);
  user_status_ = nh_->subscribe("user_activity", 1, &ServiceRobot::setActivity, this);
  voice_server_ = nh_->advertiseService("user_voice_command", &ServiceRobot::voiceCallBack, this);
  tts_client_ = nh_->serviceClient<activity_recognition::robot_tts>("smartpal5_tts");
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
  sort_key_.clear();

  // assign a label
  if (!msg->data.compare("eat_a_meal"))
  {
    user_activity_ = 0;
    sort_key_.push_back("tea");
    sort_key_.push_back("drink");
  }
  else if (!msg->data.compare("gaze_at_a_robot"))
  {
    user_activity_ = 1;
    this->robotTTS(3, "");
  }
  else if (!msg->data.compare("gaze_at_a_tree"))
  {
    user_activity_ = 2;
    sort_key_.push_back("pot");
  }
  else if (!msg->data.compare("look_around"))
  {
    user_activity_ = 3;
    this->robotTTS(4, "");
  }
  else if (!msg->data.compare("read_a_book"))
  {
    user_activity_ = 4;
    sort_key_.push_back("coffee");
    sort_key_.push_back("drink");
  }
  else
  {
    user_activity_ = -1;
  }
}

typedef std::vector<std::string>::iterator str_vec_itr;
typedef std::vector<tms_msg_db::Tmsdb>::iterator tmsdb_itr;

//----------------------------------------------------------------------------------
bool ServiceRobot::voiceCallBack(activity_recognition::user_voice::Request &req,
                                 activity_recognition::user_voice::Response &res)
{
  int tts_mode = 0;
  std::string received_text = req.text;
  std::vector<std::string> match_list, sort_key;

  std::cout << "-------------------------------------\n";
  ROS_INFO("received: \033[1;32m%s\033[0m", received_text.c_str());

  for (str_vec_itr itr = tag_list_.begin(); itr != tag_list_.end(); itr++)
  {
    if (strstr(received_text.c_str(), (*itr).c_str()))
    {
      ROS_INFO("tag: \033[1;34m%s\033[0m", (*itr).c_str());
      match_list.push_back(*itr);
    }
  }

  if (!match_list.empty())
  {
    // if something has matched with tags
    tms_msg_db::TmsdbGetData srv;
    for (str_vec_itr itr = match_list.begin(); itr != match_list.end(); itr++)
    {
      srv.request.tmsdb.tag = *itr;

      // send a requrst to get the object list
      if (tms_db_client_.call(srv))
      {
        ROS_INFO("search database for \"%s\"", srv.request.tmsdb.tag.c_str());
      }
      else
      {
        ROS_ERROR("failed to call service dbreader");
        return false;
      }

      // state checking
      if (srv.response.tmsdb.empty())
      {
        ROS_INFO("no matched tag on database");
      }
      else
      {
        for (int i = 0; i < srv.response.tmsdb.size(); i++)
        {
#if STATE_CHECK
          if (srv.response.tmsdb[i].state == 1) {
#endif
            object_list_.push_back(srv.response.tmsdb[i]);
            ROS_INFO_STREAM("matched: " << srv.response.tmsdb[i].name);
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
    ROS_ERROR("not supported command");
    //this->robotTTS(5, "");
    res.result = 0;
    return false;
  }

  if (!object_list_.empty())
  {
    for (tmsdb_itr obj_itr = object_list_.begin(); obj_itr != object_list_.end(); obj_itr++)
    {
      // e.g "greentea_bottle" -> "green tea bottle"
      size_t c;
      std::string space = " ";
      while((c = obj_itr->name.find_first_of("_")) != std::string::npos)
      {
        obj_itr->name.replace(c, space.length(), space);
      }
      if((c = obj_itr->name.find("tea")) != std::string::npos)
      {
        obj_itr->name.insert(c, " ");
      }
    }

    ServiceRobot::compare_struct compare(this);
    std::sort(object_list_.begin(), object_list_.end(), compare);

    for (tmsdb_itr obj_itr = object_list_.begin(); obj_itr != object_list_.end(); obj_itr++)
    {
      ROS_INFO("sorted: \033[1;32m%s\033[0m", obj_itr->name.c_str());
    }

    // recommendation??
    if (strstr(object_list_[0].name.c_str(), "coffee"))
    {
      this->robotTTS(1, object_list_[0].name);
    }
    else
    {
      this->robotTTS(tts_mode, object_list_[0].name);
    }

    res.result = 1;
  }
  else
  {
    ROS_INFO("no object has listed");
    this->robotTTS(6, "");
    res.result = 0;
  }

  std::vector<std::string>().swap(match_list);
  std::vector<std::string>().swap(sort_key);
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
      srv.request.text = "May I help you ?";
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
    ROS_ERROR("failed to call tts service");
  }
}
