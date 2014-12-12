/*
 * Classifier.h
 *
 *  Created on: Dec 9, 2014
 *      Author: kazuto
 */

#ifndef _CLASSIFIER_H_
#define _CLASSIFIER_H_

#include "common.h"

//----------------------------------------------------------------------------------
// Classifier
//----------------------------------------------------------------------------------
class Classifier
{
private:
  ros::NodeHandle* nh_;
  ros::Publisher status_;

  struct svm_model* model_;
  cv::PCA pca_;
  FisherVector fisher_;

  std::vector<Video> video_list_;
  cv::Mat data_;
  std_msgs::String msg_;
public:
  Classifier(ros::NodeHandle* nh);
  ~Classifier();
  void run();
  static void* run_thread(void *obj);
  void Classify(cv::Mat& data, std::vector<Video>& video_list);
};

std::string RestoreLabel(int label);
cv::Mat GetFisherMat(FisherVector& fisherVec, cv::Mat& comp_mat, std::vector<Video>& video_list);
cv::Mat GetVLADMat(VLAD& vlad, cv::Mat& comp_mat, std::vector<Video>& video_list);
cv::Mat GetBoVWMat(BoVW& bovw, cv::Mat& comp_mat, std::vector<Video>& video_list);
void LoadPCA(cv::Mat& eigenvectors, cv::Mat& eigenvalues, cv::Mat& mean, const char* file_dir);
bool LoadMat(const std::string& file_dir, cv::Mat& data);

#endif /* CV_TEST_INCLUDE_CV_TEST_CLASSIFIER_H_ */
