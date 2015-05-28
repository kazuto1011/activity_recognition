/*
 * Classifier.h
 *
 *  Created on: Dec 9, 2014
 *      Author: kazuto
 */

#ifndef _CLASSIFIER_H_
#define _CLASSIFIER_H_

#include "common.h"
#include "activity_recognition_server/classify.h"

//----------------------------------------------------------------------------------
// Classifier
//----------------------------------------------------------------------------------
class Classifier
{
private:
  ros::NodeHandle* nh_;
  std_msgs::String msg_;
  ros::Publisher user_status_;
  ros::Publisher server_status_;
  ros::ServiceServer classify_server_;

  int encoding_mode_;
  cv::PCA pca_;
  FisherVector* fisher_;
  VLAD* vlad_;
  BoVW* bovw_;
  struct svm_model* model_;

  std::vector<Video> video_list_;
  cv::Mat data_;

public:
  Classifier(ros::NodeHandle* nh, int encoding_mode);
  ~Classifier();
  bool Classify(activity_recognition_server::classify::Request &req,
                activity_recognition_server::classify::Response &res);
  std::string RestoreLabel(int label);
  cv::Mat GetFisherMat(FisherVector& fisherVec, cv::Mat& comp_mat, std::vector<Video>& video_list);
  cv::Mat GetVLADMat(VLAD& vlad, cv::Mat& comp_mat, std::vector<Video>& video_list);
  cv::Mat GetBoVWMat(BoVW& bovw, cv::Mat& comp_mat, std::vector<Video>& video_list);
  void LoadPCA(cv::Mat& eigenvectors, cv::Mat& eigenvalues, cv::Mat& mean, const char* file_dir);
};

#endif /* CV_TEST_INCLUDE_CV_TEST_CLASSIFIER_H_ */
