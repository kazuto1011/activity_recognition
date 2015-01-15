/*
 * ServiceRobot.h
 *
 *  Created on: Dec 11, 2014
 *      Author: kazuto
 */

#ifndef _SERVICE_ROBOT_H_
#define _SERVICE_ROBOT_H_

#include "common.h"
#include "activity_recognition/robot_tts.h"
#include "activity_recognition/user_voice.h"
#include "tms_msg_db/TmsdbGetData.h"
#include "tms_msg_db/Tmsdb.h"

typedef std::vector<std::string>::iterator str_itr;

//----------------------------------------------------------------------------------
// ServiceRobot
//----------------------------------------------------------------------------------
class ServiceRobot
{
  // non-static compare method
  struct compare_struct
  {
    ServiceRobot* ptr_;
    compare_struct(ServiceRobot* ptr): ptr_(ptr) {}
    bool operator () (tms_msg_db::Tmsdb db1, tms_msg_db::Tmsdb db2)
    {
      int flag[2] = {};
      for (str_itr itr = ptr_->sort_key_.begin(); itr != ptr_->sort_key_.end(); itr++)
      {
        if (strstr(db1.tag.c_str(), (*itr).c_str()))
        {
          flag[0]++;
        }
        if (strstr(db2.tag.c_str(), (*itr).c_str()))
        {
          flag[1]++;
        }
      }
      return flag[0] > flag[1];
    }
  };

private:
  ros::NodeHandle* nh_;
  ros::Publisher server_status_;
  ros::Subscriber user_status_;
  ros::ServiceServer voice_server_;
  ros::ServiceClient tts_client_;
  ros::ServiceClient tms_db_client_;

  int user_activity_;
  std::vector<std::string> tag_list_;
  std::vector<std::string> sort_key_;
  std::vector<tms_msg_db::Tmsdb> object_list_;

public:
  ServiceRobot(ros::NodeHandle* nh);
  ~ServiceRobot();
  void setActivity(const std_msgs::StringConstPtr& msg);
  bool voiceCallBack(activity_recognition::user_voice::Request &req,
                     activity_recognition::user_voice::Response &res);
  void robotTTS(int service, std::string object_name);
};

#endif /* _SERVICE_ROBOT_H_ */
