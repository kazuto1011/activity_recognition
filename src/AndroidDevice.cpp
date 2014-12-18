/*
 * AndroidDev.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: kazuto
 */

#include "AndroidDevice.h"
#include "common.h"
#include "Classifier.h"

//----------------------------------------------------------------------------------
// AndroidDevice
//----------------------------------------------------------------------------------
AndroidDevice::AndroidDevice(ros::NodeHandle* nh, Classifier* classifier) :
    nh_(nh), it_(*nh),
    fps_(FPS), size_(cv::Size(320, 240)), img_buf_(NUM_FRAME),
    flag(true), output_dir_(OUTPUT_DIR),
    ffmpeg_("ffmpeg"), stipdet_("~/stip-2.0-linux/bin/stipdet"),
    classifier_(classifier)
{
  ROS_INFO("AndroidDevice constructor");

  // image subscriber
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  subscriber_ = it_.subscribe("moverio/camera", 1, &AndroidDevice::storeMat, this, hints);

  // user status publisher
  server_status_ = nh_->advertise<std_msgs::String>("server_status", 1);

  // initialize a preview window
  cv::namedWindow("cv_moverio");
  cv::startWindowThread();
  cv::Mat img = cv::imread("/home/kazuto/catkin_ws/src/activity_recognition/moverio.jpg", 1);
  cv::imshow("cv_moverio", img);

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
  stipdet_ += "~/catkin_ws/src/activity_recognition/video_list.txt";
  // input video path
  stipdet_ += " -vpath ";
  stipdet_ += "~/catkin_ws/src/activity_recognition/video/";
  // output path
  stipdet_ += " -o ";
  stipdet_ += "~/catkin_ws/src/activity_recognition/moverio.txt";
}

//----------------------------------------------------------------------------------
AndroidDevice::~AndroidDevice()
{
  ROS_INFO("AndroidDevice destructor");
  cv::destroyWindow("cv_moverio");
}

//----------------------------------------------------------------------------------
void AndroidDevice::storeMat(const sensor_msgs::ImageConstPtr& msg)
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

  // Show the received image
  cv::cvtColor(img, img, CV_GRAY2BGR);
  cv::imshow("cv_moverio", img);
  cv::waitKey(10);

  // circular buffer for image
  img_buf_.push_back(img);

  if (img_buf_.full() && flag)
  {
    flag = false;
    boost::thread video_thread(&AndroidDevice::createVideo, this);
  }
}

//----------------------------------------------------------------------------------
void AndroidDevice::createVideo()
{
  video_writer_.open(output_dir_, CV_FOURCC_DEFAULT, fps_, size_);

  for (int i = 0; i < NUM_FRAME; i++)
  {
    video_writer_ << img_buf_[i];
  }
  video_writer_.release();

  // ffmpeg
  system(ffmpeg_.c_str());
  ROS_INFO("created a new video & converted the codec");

  this->detectFeatures();
  flag = true;
}

//----------------------------------------------------------------------------------
bool AndroidDevice::detectFeatures()
{
  msg_.data = "STIP detecting...";
  server_status_.publish(msg_);

  // stip detection
  system(stipdet_.c_str());

  std::vector<Video> video_list;

  int num_data = 1;
  cv::Mat hog_mat = cv::Mat_<float>(num_data, HOG_DIM);
  cv::Mat hof_mat = cv::Mat_<float>(num_data, HOF_DIM);
  cv::Mat hoghof_mat = cv::Mat_<float>(num_data, HOGHOF_DIM);

  msg_.data = "Loading features...";
  server_status_.publish(msg_);

  // file stream
  std::ifstream ifs(TEXT_DIR);
  if (!ifs)
  {
    ROS_ERROR("cannot open the file");
    return false;
  }

  std::string buf;
  int count[3] = {};
  float tmp;
  size_t p;

  while (ifs && getline(ifs, buf))
  {
    if (buf.substr(0, 1) != "#")
    {
      count[0] = 9;  // info of the features
      count[1] = 72; // the dimension of hog
      count[2] = 1;  // check;

      // point-type x y t sigma2 tau2 detector-confidence
      // point-type y-norm x-norm t-norm y x t sigma2 tau2
      while (((p = buf.find(' ')) != buf.npos) && count[0])
      {
        buf = buf.substr(p + 1);
        count[0]--;
      }

      // dscr-hog(72)
      count[0] = 0;
      while (((p = buf.find(' ')) != buf.npos) && count[1])
      {
        tmp = atof(buf.substr(0, p + 1).c_str());
        hog_mat.at<float>(num_data - 1, count[0]) = tmp;
        hoghof_mat.at<float>(num_data - 1, count[0]++) = tmp;
        buf = buf.substr(p + 1);
        count[1]--;
      }

      // dscr-hof(90)
      count[1] = count[0];
      count[0] = 0;
      while ((p = buf.find(' ')) != buf.npos)
      {
        tmp = atof(buf.substr(0, p + 1).c_str());
        hof_mat.at<float>(num_data - 1, count[0]++) = tmp;
        hoghof_mat.at<float>(num_data - 1, count[1]++) = tmp;
        buf = buf.substr(p + 1);
      }
      hog_mat.resize(++num_data);
      hof_mat.resize(num_data);
      hoghof_mat.resize(num_data);
    }
  }

  hog_mat.resize(--num_data);
  hof_mat.resize(num_data);
  hoghof_mat.resize(num_data);
  video_list.push_back(Video(0, num_data, -1));

  if (!count[2])
  {
    ROS_ERROR("no feature has detected");
  }
  else
  {
    msg_.data = "PCA, Vector encoding...";
    server_status_.publish(msg_);
    classifier_->Classify(hog_mat, video_list);
  }

  // release
  std::vector<Video>().swap(video_list);
  return true;
}
