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
  image_transport::ImageTransport it_;
  image_transport::Subscriber subscriber_;
  ros::Publisher server_status_;
  Classifier* classifier_;

  std::string output_dir_;
  double fps_;
  cv::Size size_;
  cv::VideoWriter video_writer_;

  int num_frames_;
  int flag;
  boost::circular_buffer<cv::Mat> img_buf_;
  std_msgs::String msg_;
public:
  AndroidDevice(ros::NodeHandle* nh, Classifier* classifier);
  ~AndroidDevice();
  void storeMat(const sensor_msgs::ImageConstPtr& msg);
  void createVideo();
  bool detectFeatures();
};

#endif /* CV_TEST_INCLUDE_CV_TEST_ANDROIDDEV_H_ */
