/*
 * Classifier.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: kazuto
 */

#include "common.h"
#include "AndroidDevice.h"
#include "Classifier.h"

//----------------------------------------------------------------------------------
// Classifier
//----------------------------------------------------------------------------------
Classifier::Classifier(ros::NodeHandle* nh):
    nh_(nh)
{
  ROS_INFO("Classifier constructor");

  // initialize SVM, PCA, and Fisher Vector encoder
  LoadPCA(pca_.eigenvectors, pca_.eigenvalues, pca_.mean, PCA_DIR);

  // TODO
  fisher_ = FisherVector(GMM_DIR);
  model_ = svm_load_model(SVM_DIR);

  // status publisher
  status_ = nh_->advertise<std_msgs::String>("user_activity", 1);
}

//----------------------------------------------------------------------------------
Classifier::~Classifier()
{
  ROS_INFO("Classifier destructor");
  svm_free_and_destroy_model(&model_);
  nh_->shutdown();
}

//----------------------------------------------------------------------------------
void Classifier::Classify(cv::Mat& data, std::vector<Video>& video_list)
{
  cv::Mat comp_mat = cv::Mat_<float>(data.rows, pca_.eigenvalues.rows);
  pca_.project(data, comp_mat);
  data_ = GetFisherMat(fisher_, comp_mat, video_list);

  // store the test data
  svm_node* x_node = new svm_node[data_.cols + 1]; // "+1" means the malloc for the end
  int idx = 0;
  for (int i = 0; i < data_.cols; i++)
  {
    x_node[idx].index = i + 1;
    x_node[idx].value = data_.at<float>(0, i); // from the histogram
    idx++;
  }
  x_node[idx].index = -1; // the end of a vector

  // probability estimation
  int num_class = svm_get_nr_class(model_);
  double *pre_prob = new double[num_class];
  double predict_label = svm_predict_probability(model_, x_node, pre_prob);

  std::cout << "The predicted result: " << RestoreLabel((int)predict_label) << std::endl;
  for (int i = 0; i < num_class; i++)
  {
    std::cout << "Class " << i << ": " << pre_prob[i] << std::endl;
  }

  // notify clients of the result
  msg_.data = RestoreLabel((int)predict_label);
  status_.publish(msg_);

  // release
  delete[] x_node;
  delete[] pre_prob;
}

//----------------------------------------------------------------------------------
std::string RestoreLabel(int label)
{
  switch (label)
  {
    case 0:
      return "eat_a_meal";
    case 1:
      return "gaze_at_a_robot";
    case 2:
      return "gaze_at_a_tree";
    case 3:
      return "look_around";
    case 4:
      return "read_a_book";
    default:
      return "-1";
  }
}

//----------------------------------------------------------------------------------
cv::Mat GetFisherMat(FisherVector& fisherVec, cv::Mat& comp_mat, std::vector<Video>& video_list)
{
  cv::Mat fisherMat = cv::Mat_<float>(video_list.size(), fisherVec.GetDimension());

  int idx = 0, j = 0;
  cv::Mat dataToEncode;
  for (std::vector<Video>::iterator itr = video_list.begin(); itr != video_list.end(); itr++)
  {
    dataToEncode = comp_mat.rowRange(cv::Range(itr->GetIdx(), itr->GetIdx() + itr->GetNumData()));
    fisherVec.FvEncode(dataToEncode).copyTo(fisherMat.row(j++));
    idx = itr->GetIdx() + itr->GetNumData();
  }

  return fisherMat;
}

//----------------------------------------------------------------------------------
cv::Mat GetVLADMat(VLAD& vlad, cv::Mat& comp_mat, std::vector<Video>& video_list)
{
  cv::Mat vladMat = cv::Mat_<float>(video_list.size(), vlad.GetDimension());

  int idx = 0, j = 0;
  cv::Mat dataToEncode;
  for (std::vector<Video>::iterator itr = video_list.begin(); itr != video_list.end(); itr++)
  {
    dataToEncode = comp_mat.rowRange(cv::Range(itr->GetIdx(), itr->GetIdx() + itr->GetNumData()));
    vlad.VladEncode(dataToEncode).copyTo(vladMat.row(j++));
    idx = itr->GetIdx() + itr->GetNumData();
  }

  return vladMat;
}

//----------------------------------------------------------------------------------
cv::Mat GetBoVWMat(BoVW& bovw, cv::Mat& comp_mat, std::vector<Video>& video_list)
{
  cv::Mat bovwMat = cv::Mat_<float>(video_list.size(), bovw.GetDimension());

  int idx = 0, j = 0;
  cv::Mat dataToEncode;
  for (std::vector<Video>::iterator itr = video_list.begin(); itr != video_list.end(); itr++)
  {
    dataToEncode = comp_mat.rowRange(cv::Range(itr->GetIdx(), itr->GetIdx() + itr->GetNumData()));
    bovw.BuidHistogram(dataToEncode).copyTo(bovwMat.row(j++));
    idx = itr->GetIdx() + itr->GetNumData();
  }

  return bovwMat;
}

//----------------------------------------------------------------------------------
void LoadPCA(cv::Mat& eigenvectors, cv::Mat& eigenvalues, cv::Mat& mean, const char* file_dir)
{
  std::ifstream ifs(file_dir, std::ios_base::in | std::ios_base::binary);

  int rows, cols, type;
  ifs.read((char*)&rows, sizeof(int));
  ifs.read((char*)&cols, sizeof(int));
  ifs.read((char*)&type, sizeof(int));
  eigenvectors.release();
  eigenvectors.create(rows, cols, type);
  ifs.read((char*)eigenvectors.data, eigenvectors.elemSize() * eigenvectors.total());

  ifs.read((char*)&rows, sizeof(int));
  ifs.read((char*)&cols, sizeof(int));
  ifs.read((char*)&type, sizeof(int));
  eigenvalues.release();
  eigenvalues.create(rows, cols, type);
  ifs.read((char*)eigenvalues.data, eigenvalues.elemSize() * eigenvalues.total());

  ifs.read((char*)&rows, sizeof(int));
  ifs.read((char*)&cols, sizeof(int));
  ifs.read((char*)&type, sizeof(int));
  mean.release();
  mean.create(rows, cols, type);
  ifs.read((char*)mean.data, mean.elemSize() * mean.total());

  ifs.close();
}

//----------------------------------------------------------------------------------
bool LoadMat(const std::string& file_dir, cv::Mat& data)
{
  std::ifstream ifs(file_dir.c_str(), std::ios_base::binary);
  if (!ifs)
  {
    return false;
  }

  int rows, cols, type;
  ifs.read((char*)&rows, sizeof(int));
  ifs.read((char*)&cols, sizeof(int));
  ifs.read((char*)&type, sizeof(int));
  data.release();
  data.create(rows, cols, type);
  ifs.read((char*)data.data, data.elemSize() * data.total());

  ifs.close();
  return true;
}
