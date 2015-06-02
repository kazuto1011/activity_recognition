//------------------------------------------------------------------------------
// @file   : main.h
// @brief  : 
// @author : Kazuto Nakashima
//------------------------------------------------------------------------------
// Standard
#include <iostream>
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

#define STIPDET    "C:\\stip-1.1-winlinux\\bin\\stipdet"
#define VIS        " -vis yes" // visualization option: yes/no

#define TEST_DIR   "C:\\Users\\kazut_000\\Desktop\\test"
#define PCA_DIR  "C:\\Users\\kazut_000\\Desktop\\data\\params\\eigen.pca"
#define SVM_DIR    "C:\\Users\\kazut_000\\Desktop\\data\\params\\svm.model"
#define GMM_DIR    "C:\\Users\\kazut_000\\Desktop\\data\\params\\fisher.gmm"
#define KMEANS_DIR "C:\\Users\\kazut_000\\Desktop\\data\\params\\vlad.kmeans"

#define HOG        "_hog"
#define HOF        "_hof"
#define HOGHOF     "_hoghof"
#define PREFIX     ".avi"
#define FEATURES   hog_mat

//----------------------------------------------------------------------------------
// video class
//----------------------------------------------------------------------------------
class Video {
private:
    int idx;   // end point
    int label;
public:
    Video(int i, int j) { this->idx = i; this->label = j; };
    int GetIdx(){ return this->idx; };
    int GetLabel(){ return this->label; };
};

//----------------------------------------------------------------------------------
// time measurement
//----------------------------------------------------------------------------------
std::chrono::system_clock::time_point GetTime();
long long GetTime(std::chrono::system_clock::time_point start, std::chrono::system_clock::time_point end);

//----------------------------------------------------------------------------------
// Support Vector Machine
//----------------------------------------------------------------------------------
namespace SVM {
    void SetParams(svm_parameter *param);
    void StoreTrainData(svm_problem *prob, cv::Mat& train_data, std::vector<Video> video_list);
}

//------------------------------------------------------------------------------
// 0: eat a meal
// 1: gaze at a robot
// 2: look around
// 3: read a book
//------------------------------------------------------------------------------
std::string AssignLabel(std::string str);
std::string RestoreLabel(int label);

cv::Mat GetFisherMat(FisherVector& fisherVec, cv::Mat& comp_mat, std::vector<Video> video_list);
cv::Mat GetVladMat(VLAD& vlad, cv::Mat& comp_mat, std::vector<Video> video_list);

void SavePCA(cv::PCA& pca, const char* file_dir);
void LoadPCA(cv::Mat& eigenvectors, cv::Mat& eigenvalues, cv::Mat& mean, const char* file_dir);