/*
 * AndroidDev.h
 *
 *  Created on: Dec 9, 2014
 *      Author: kazuto
 */

#ifndef _DESCRIPTOR_H_
#define _DESCRIPTOR_H_

#include "common.h"
#include "activity_recognition_server/classify.h"

//----------------------------------------------------------------------------------
// FeatureDescriptor
//----------------------------------------------------------------------------------
class FeatureDescriptor
{
private:
  ros::NodeHandle* nh_;
  ros::Publisher server_status_;
  ros::ServiceClient classify_client_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber camera_;
  image_transport::Subscriber screen_;

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
  FeatureDescriptor(ros::NodeHandle* nh);
  ~FeatureDescriptor();
  void imgBuffer(const sensor_msgs::ImageConstPtr& msg);
  void screenForDemo(const sensor_msgs::ImageConstPtr& msg);
  void generateVideo();
  bool detectFeatures();
};

#endif /* _DESCRIPTOR_H_ */
