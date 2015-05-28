//------------------------------------------------------------------------------
// @file   : main.h
// @brief  : 
// @author : Kazuto Nakashima
//------------------------------------------------------------------------------
// Standard
#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <chrono> // for system_clock

// VLFeat
extern "C" {
#include <vl/generic.h>
#include <vl/fisher.h>
#include <vl/gmm.h>
#include <vl/vlad.h>
#include <vl/mathop.h>
}

// OpenCV
#include "opencv_lib.h"

// LibSVM
#include "libsvm.h"

// BoW, Fisher Vector, VLAD
#include "vector_encoding.h"

const std::string INPUT_DIR = "C:\\Users\\kazut_000\\Desktop\\data\\stip_hoghof";
const std::string EXT = ".txt";
const int HOG_DIM = 72;
const int HOF_DIM = 90;
const int HOGHOF_DIM = 162;
const int FEAT_DIM = HOGHOF_DIM;

const int NUM_FV_VW = 50; // the number of GMM clusters (visual words)
const int NUM_VLAD_VW = 50; // the number of kmeans clusters (visual words)
const int NUM_BOVW_VW = 50; // the number of kmeans clusters (visual words)
const int NUM_TEST = 25;

const std::string PCA_DIR = "C:\\Users\\kazut_000\\Desktop\\data\\params\\eigen.pca";
const std::string GMM_DIR = "C:\\Users\\kazut_000\\Desktop\\data\\params\\fisher.gmm";
const std::string VLAD_KMEANS_DIR = "C:\\Users\\kazut_000\\Desktop\\data\\params\\vlad.kmeans";
const std::string BOVW_KMEANS_DIR = "C:\\Users\\kazut_000\\Desktop\\data\\params\\bovw.kmeans";
const std::string SVM_DIR = "C:\\Users\\kazut_000\\Desktop\\data\\params\\svm.model";

const int REPETITION = 100;
#define ENCODE 0
#define VISUAL_WORDS 1
#define LIBSVM 1

//----------------------------------------------------------------------------------
// video class
//----------------------------------------------------------------------------------
class Video {
private:
    int start_idx;
    int num_data;
    int label;
public:
    Video(int i, int j, int k) { this->start_idx = i; this->num_data = j; this->label = k; };
    int GetIdx(){ return this->start_idx; };
    int GetNumData(){ return this->num_data; };
    int GetLabel(){ return this->label; };
    void SetIdx(int i){ this->start_idx = i; };
    void SetNumData(int i){ this->num_data = i; };
    void SetLabel(int i){ this->label = i; };
};

//----------------------------------------------------------------------------------
// Support Vector Machine
//----------------------------------------------------------------------------------
namespace SVM {
    void SetParams(svm_parameter *param);
    void StoreTrainData(svm_problem *prob, cv::Mat& train_data, std::vector<Video> video_list);
}

std::string AssignLabel(std::string str);
std::string RestoreLabel(int label);

cv::Mat GetFisherMat(FisherVector* fisherVec, cv::Mat& comp_mat, std::vector<Video>& video_list);
cv::Mat GetVLADMat(VLAD* vlad, cv::Mat& comp_mat, std::vector<Video>& video_list);
cv::Mat GetBoVWMat(BoVW* bovw, cv::Mat& comp_mat, std::vector<Video>& video_list);

void SavePCA(cv::PCA& pca, const std::string& file_dir);
void LoadPCA(cv::Mat& eigenvectors, cv::Mat& eigenvalues, cv::Mat& mean, const std::string& file_dir);

bool LoadMat(const std::string& file_dir, cv::Mat& data);

bool CreateTestSet(cv::Mat& data, cv::Mat& test, std::vector<Video>& video_list, std::vector<Video>& test_list, const int num_test);