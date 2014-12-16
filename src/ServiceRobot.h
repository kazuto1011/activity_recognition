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
  int user_activity_;

public:
  ServiceRobot(ros::NodeHandle* nh);
  ~ServiceRobot();
  void setActivity(const std_msgs::StringConstPtr& msg);
  bool voiceCallBack(activity_recognition::user_voice::Request &req,
                     activity_recognition::user_voice::Response &res);
  void robotTTS(int service);
};

#endif /* _SERVICE_ROBOT_H_ */
