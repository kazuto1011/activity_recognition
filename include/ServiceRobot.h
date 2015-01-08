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

//----------------------------------------------------------------------------------
// ServiceRobot
//----------------------------------------------------------------------------------
class ServiceRobot
{
private:
  ros::NodeHandle* nh_;
  ros::Publisher server_status_;
  ros::Subscriber user_status_;
  ros::ServiceServer voice_server_;
  ros::ServiceClient tts_client_;
  ros::ServiceClient tms_db_client_;
  int user_activity_;
  std::map<std::string, std::string*> hash_table_;
  std::vector<std::string> tag_list_;
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
