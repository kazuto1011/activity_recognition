//------------------------------------------------------------------------------
// @file   : vector_encoding.h
// @brief  : Fisher Vector, VLAD, BoW
// @author : Kazuto Nakashima
//------------------------------------------------------------------------------
#ifndef VECTOR_ENCODING
#define VECTOR_ENCODING

// VLFeat
extern "C" {
#include <vl/generic.h>
#include <vl/fisher.h>
#include <vl/gmm.h>
#include <vl/vlad.h>
#include <vl/mathop.h>
}

#include "opencv_lib.h"

//----------------------------------------------------------------------------------
// Fisher Vector encoding
//----------------------------------------------------------------------------------
class FisherVector {
private:
    vl_size fv_dimension; // the dimension of fisher vecrtor
    VlGMM* gmm;           // gaussian mixture model object
    float* means;         // gmm means
    float* covariances;   // gmm covariances
    float* priors;        // gmm priors
    vl_size num_clusters; // the number of gaussians
    vl_type data_type;    // type of data
    vl_size dimension;    // data dimension
public:
    FisherVector();
    FisherVector(const char* file_dir);
    ~FisherVector();
    void GmmCluster(cv::Mat& data, int num_visualwords);
    cv::Mat FvEncode(cv::Mat& dataToEncode);

    void SaveGMM(const char* file_dir);
    vl_size GetDimension() { return this->fv_dimension; }
    float* GetMeans() { return this->means; }
    float* GetCovarances() { return this->covariances; }
    float* GetPriors() { return this->priors; }
};

//----------------------------------------------------------------------------------
// VLAD encoding
//----------------------------------------------------------------------------------
class VLAD {
private:
    vl_size vlad_dimension; // the dimension of vlad
    VlKMeans* kmeans;       // kmeans object
    float* means;           // kmeans centers
    vl_uint32* indexes;     // index of nearest centers
    float* assignments;     // soft assignment, row-major order
    VlVectorComparisonType distance_type;
    vl_size num_centers; // the number of centers
    vl_type data_type;   // type of data
    vl_size dimension;   // data dimension
public:
    VLAD();
    VLAD(const char* file_dir);
    ~VLAD();
    void KmeansCluster(cv::Mat& data, int num_visualwords);
    void KmeansQuantize(cv::Mat& data);
    cv::Mat VladEncode(cv::Mat& dataToEncode);

    void SaveKmeans(const char* file_dir);
    vl_size GetDimension() { return this->vlad_dimension; }
    float* GetMeans() { return this->means; }
};

//----------------------------------------------------------------------------------
// Bag of Visual Words
//----------------------------------------------------------------------------------
class BoVW {
private:
    vl_size bovw_dimension; // the dimension of bag of visual words
    VlKMeans* kmeans;       // kmeans object
    float* means;           // kmeans centers
    vl_uint32* indexes;     // index of nearest centers
    VlVectorComparisonType distance_type;
    vl_size num_centers; // the number of centers
    vl_type data_type;   // type of data
    vl_size dimension;   // data dimension
public:
    BoVW();
    ~BoVW();
    void KmeansCluster(cv::Mat& data, int  num_visualwords);
    void KmeansQuantize(cv::Mat& data);
    cv::Mat BuidHistogram(cv::Mat& dataToBuild);

    void SaveKmeans(const char* file_dir);
    vl_size GetDimension() { return this->bovw_dimension; }
    float* GetMeans() { return this->means; }
};

#endif