/*
 * AndroidDev.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: kazuto
 */

#include "common.h"
#include "FeatureDescriptor.h"

//----------------------------------------------------------------------------------
// FeatureDescriptor
//----------------------------------------------------------------------------------
FeatureDescriptor::FeatureDescriptor(ros::NodeHandle* nh) :
    nh_(nh), it_(*nh),
    fps_(FPS), size_(cv::Size(320, 240)), img_buf_(NUM_FRAME),
    flag(true), output_dir_(OUTPUT_DIR),
    ffmpeg_("ffmpeg"), stipdet_("~/stip-2.0-linux/bin/stipdet")
{
  ROS_INFO("FeatureDescriptor constructor");

  // image subscriber
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  camera_ = it_.subscribe("moverio/camera", 1, &FeatureDescriptor::imgBuffer, this, hints);
  screen_ = it_.subscribe ("moverio/screen", 1, &FeatureDescriptor::screenForDemo, this, hints);

  // user status publisher
  server_status_ = nh_->advertise<std_msgs::String>("server_status", 1);

  // classify service client
  classify_client_ = nh_->serviceClient<activity_recognition::classify>("feature_directory", 1);

  // initialize a preview window
  cv::namedWindow("moverio");
  cv::startWindowThread();
  cv::Mat img = cv::imread("/home/kazuto/catkin_ws/src/activity_recognition/moverio.jpg", 1);
  cv::imshow("moverio", img);

  // ffmpeg operation
  // input source
  ffmpeg_ += " -i ";
  ffmpeg_ += "~/catkin_ws/src/activity_recognition/video/moverio.avi";
  // output codec
  ffmpeg_ += " -vcodec ";
  ffmpeg_ += "libxvid";
  // output size
  ffmpeg_ += " -s ";
  ffmpeg_ += "320x240";
  // output path
  ffmpeg_ += " -y ";
  ffmpeg_ += "~/catkin_ws/src/activity_recognition/video/moverio_converted.avi";

  // STIP detection
  // input list file
  stipdet_ += " -i ";
  stipdet_ += "~/catkin_ws/src/activity_recognition/video/video_list.txt";
  // input video path
  stipdet_ += " -vpath ";
  stipdet_ += "~/catkin_ws/src/activity_recognition/video/";
  // output path
  stipdet_ += " -o ";
  stipdet_ += "~/catkin_ws/src/activity_recognition/moverio.txt";
}

//----------------------------------------------------------------------------------
FeatureDescriptor::~FeatureDescriptor()
{
  ROS_INFO("FeatureDescriptor destructor");
  cv::destroyWindow("moverio");
}

//----------------------------------------------------------------------------------
void FeatureDescriptor::imgBuffer(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  cv::Mat roi;

  // cv_bridge conversion; sensor_msgs/Image -> cv::Mat
  try
  {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }

  // Show the received image
  cv::cvtColor(img, img, CV_GRAY2BGR);

  if (!screen_img_.empty())
  {
    roi = cv::Mat(screen_img_, cv::Rect(30, 86, img.cols, img.rows));
    img.copyTo(roi);
    cv::imshow("moverio", screen_img_);
    cv::waitKey(10);
  }

  // circular buffer for image
  img_buf_.push_back(img);

  if (img_buf_.full() && flag)
  {
    flag = false;
    boost::thread video_thread(&FeatureDescriptor::generateVideo, this);
  }
}

//----------------------------------------------------------------------------------
void FeatureDescriptor::screenForDemo(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;

  // cv_bridge conversion; sensor_msgs/Image -> cv::Mat
  try
  {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }

  cv::cvtColor(img, img, CV_GRAY2BGR);
  img.copyTo(screen_img_);
}

//----------------------------------------------------------------------------------
void FeatureDescriptor::generateVideo()
{
  video_writer_.open(output_dir_, CV_FOURCC_DEFAULT, fps_, size_);

  for (int i = 0; i < NUM_FRAME; i++)
  {
    video_writer_ << img_buf_[i];
  }
  video_writer_.release();

  // ffmpeg
  system(ffmpeg_.c_str());
  ROS_INFO("Created a new video & converted the codec");

  this->detectFeatures();
  flag = true;
}

//----------------------------------------------------------------------------------
inline bool FeatureDescriptor::detectFeatures()
{
  std_msgs::String msg;
  activity_recognition::classify srv;

  msg.data = "STIP detecting...";
  server_status_.publish(msg);

  // stip detection
  system(stipdet_.c_str());

  srv.request.text = TEXT_DIR;
  if (classify_client_.call(srv))
  {
    switch(srv.response.result)
    {
    case 1:
      ROS_INFO("Succeeded to classify");
      break;
    default:
      ROS_ERROR("Failed to classify");
      break;
    }
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return false;
  }
}
