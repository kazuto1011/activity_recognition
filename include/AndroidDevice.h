/*
 * AndroidDev.h
 *
 *  Created on: Dec 9, 2014
 *      Author: kazuto
 */

#ifndef _ANDROIDDEV_H_
#define _ANDROIDDEV_H_

#include "common.h"

// forward declaration
class Classifier;

//----------------------------------------------------------------------------------
// AndroidDevice
//----------------------------------------------------------------------------------
class AndroidDevice
{
private:
  ros::NodeHandle* nh_;
  std_msgs::String msg_;
  ros::Publisher server_status_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber subscriber_;
  image_transport::Subscriber screen_;

  Classifier* classifier_;

  cv::VideoWriter video_writer_;
  std::string output_dir_;
  cv::Size size_;
  double fps_;

  bool flag;
  std::string ffmpeg_;
  std::string stipdet_;
  boost::circular_buffer<cv::Mat> img_buf_;
  cv::Mat screen_img_;

public:
  AndroidDevice(ros::NodeHandle* nh, Classifier* classifier);
  ~AndroidDevice();
  void storeMat(const sensor_msgs::ImageConstPtr& msg);
  void showScreen(const sensor_msgs::ImageConstPtr& msg);
  void createVideo();
  bool detectFeatures();
};

#endif /* _ANDROIDDEV_H_ */
