/*
 * Classifier.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: kazuto
 */

#include "common.h"
#include "Classifier.h"

namespace fs = boost::filesystem;

//----------------------------------------------------------------------------------
// Classifier
//----------------------------------------------------------------------------------
Classifier::Classifier(ros::NodeHandle* nh, int encoding_mode) :
    nh_(nh), encoding_mode_(encoding_mode)
{
  ROS_INFO("Classifier constructor");

  fs::path pca_path, encode_path, svm_path;

  switch(encoding_mode_)
  {
  case 0:
      pca_path    = FISHER_PARAMS_DIR / fs::path("eigen.pca");
      encode_path = FISHER_PARAMS_DIR / fs::path("fisher.gmm");
      svm_path    = FISHER_PARAMS_DIR / fs::path("svm.model");
      LoadPCA(pca_.eigenvectors, pca_.eigenvalues, pca_.mean, pca_path.string().c_str());
      fisher_ = new FisherVector(encode_path.string().c_str());
      model_ = svm_load_model(svm_path.string().c_str());
      break;
  case 1:
      pca_path    = VLAD_PARAMS_DIR / fs::path("eigen.pca");
      encode_path = VLAD_PARAMS_DIR / fs::path("vlad.kmeans");
      svm_path    = VLAD_PARAMS_DIR / fs::path("svm.model");
      LoadPCA(pca_.eigenvectors, pca_.eigenvalues, pca_.mean, pca_path.string().c_str());
      vlad_ = new VLAD(encode_path.string().c_str());
      model_ = svm_load_model(svm_path.string().c_str());
      break;
  case 2:
      pca_path    = BOVW_PARAMS_DIR / fs::path("eigen.pca");
      encode_path = BOVW_PARAMS_DIR / fs::path("bovw.kmeans");
      svm_path    = BOVW_PARAMS_DIR / fs::path("svm.model");
      LoadPCA(pca_.eigenvectors, pca_.eigenvalues, pca_.mean, pca_path.string().c_str());
      bovw_ = new BoVW(encode_path.string().c_str());
      model_ = svm_load_model(svm_path.string().c_str());
      break;
  }

  // status publisher
  user_status_ = nh_->advertise<std_msgs::String>("user_activity", 1);
  server_status_ = nh_->advertise<std_msgs::String>("server_status", 1);

  // classidy service server
  classify_server_ = nh_->advertiseService("feature_directory", &Classifier::Classify, this);
}

//----------------------------------------------------------------------------------
Classifier::~Classifier()
{
  ROS_INFO("Classifier destructor");
  svm_free_and_destroy_model(&model_);
  nh_->shutdown();
}

//----------------------------------------------------------------------------------
bool Classifier::Classify(activity_recognition::classify::Request &req,
                          activity_recognition::classify::Response &res)
{
  std_msgs::String msg;
  std::vector<Video> video_list;

  msg.data = "loading features";
  server_status_.publish(msg);

  int num_data = 1;
  cv::Mat hog_mat = cv::Mat_<float>(num_data, HOG_DIM);
  cv::Mat hof_mat = cv::Mat_<float>(num_data, HOF_DIM);
  cv::Mat hoghof_mat = cv::Mat_<float>(num_data, HOGHOF_DIM);

  // file stream
  std::ifstream ifs(req.text.c_str());
  if (!ifs)
  {
    ROS_ERROR("cannot open the file");
    res.result = 0;
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
    ROS_ERROR("No feature has detected");
    res.result = 0;
    return false;
  }
  else
  {
    msg_.data = "dimensionality reduction";
    server_status_.publish(msg_);
  }

  // address-referencing
  cv::Mat data_mat = hog_mat;

  // Principal Component Analysis
  cv::Mat comp_mat = cv::Mat_<float>(data_mat.rows, pca_.eigenvalues.rows);
  pca_.project(data_mat, comp_mat);

  msg_.data = "encoding features";
  server_status_.publish(msg_);

  switch(encoding_mode_)
  {
  case 0:
      data_ = GetFisherMat(*fisher_, comp_mat, video_list);
      break;
  case 1:
      data_ = GetVLADMat(*vlad_, comp_mat, video_list);
      break;
  case 2:
      data_ = GetBoVWMat(*bovw_, comp_mat, video_list);
      break;
  }

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

  if (predict_label == 1 || predict_label == 3)
  {
    if (pre_prob[(int)predict_label] > 0.5)
    {
      // notify clients of the result
      msg_.data = RestoreLabel((int)predict_label);
      user_status_.publish(msg_);
    }
  }
  else
  {
    // notify clients of the result
    msg_.data = RestoreLabel((int)predict_label);
    user_status_.publish(msg_);
  }

  // release
  delete[] x_node;
  delete[] pre_prob;
  std::vector<Video>().swap(video_list);

  res.result = 1;
  return true;
}

//----------------------------------------------------------------------------------
inline std::string Classifier::RestoreLabel(int label)
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
inline cv::Mat Classifier::GetFisherMat(FisherVector& fisherVec, cv::Mat& comp_mat, std::vector<Video>& video_list)
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
inline cv::Mat Classifier::GetVLADMat(VLAD& vlad, cv::Mat& comp_mat, std::vector<Video>& video_list)
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
inline cv::Mat Classifier::GetBoVWMat(BoVW& bovw, cv::Mat& comp_mat, std::vector<Video>& video_list)
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
inline void Classifier::LoadPCA(cv::Mat& eigenvectors, cv::Mat& eigenvalues, cv::Mat& mean, const char* file_dir)
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
