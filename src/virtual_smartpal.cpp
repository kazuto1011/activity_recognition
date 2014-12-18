#include <ros/ros.h>
#include "activity_recognition/robot_tts.h"

bool robotTTS(activity_recognition::robot_tts::Request &req, activity_recognition::robot_tts::Response &res)
{
  char cmd[26];
  char tts[128];
  char say[14];

  strcpy(cmd, "pico2wave --wave ~/say.wav ");
  strcpy(say, "aplay ~/say.wav");

  snprintf(tts, 128, "%s\"%s\"", cmd, req.text.c_str());
  system(tts);
  system(say);

  res.result = 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "virtual_smartpal");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("test", robotTTS);
  ros::spin();
  return 0;
}
