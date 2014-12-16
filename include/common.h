/*
 * main.h
 *
 *  Created on: Dec 8, 2014
 *      Author: kazuto
 */

#ifndef _DEFINE_H
#define _DEFINE_H

// Standard
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <pthread.h>

// VLFeat
extern "C" {
#include <vl/generic.h>
#include <vl/fisher.h>
#include <vl/gmm.h>
#include <vl/vlad.h>
#include <vl/mathop.h>
}

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>

// LibSVM
#include "svm.h"

// BoW, Fisher Vector, VLAD
#include "vector_encoding.h"

#define EXT        ".txt"
#define HOG_DIM    72
#define HOF_DIM    90
#define HOGHOF_DIM 162
#define FEAT_DIM   HOG_DIM

// parameters for cv::VideoWriter
#define NUM_FRAME  150
#define FPS        30

#define SVM_DIR    "/home/kazuto/catkin_ws/src/activity_recognition/params/fisher_vector/svm.model"
#define GMM_DIR    "/home/kazuto/catkin_ws/src/activity_recognition/params/fisher_vector/fisher.gmm"
#define PCA_DIR    "/home/kazuto/catkin_ws/src/activity_recognition/params/fisher_vector/eigen.pca"
#define OUTPUT_DIR "/home/kazuto/catkin_ws/src/activity_recognition/video/moverio.avi"
#define TEXT_DIR   "/home/kazuto/catkin_ws/src/activity_recognition/moverio.txt"

//----------------------------------------------------------------------------------
// video class
//----------------------------------------------------------------------------------
class Video {
private:
    int start_idx;
    int num_data;
    int label;
public:
    Video(int i, int j, int k) { this->start_idx = i; this->num_data = j; this->label = k; }
    int GetIdx(){ return this->start_idx; }
    int GetNumData(){ return this->num_data; }
    int GetLabel(){ return this->label; }
    void SetIdx(int i){ this->start_idx = i; }
    void SetNumData(int i){ this->num_data = i; }
    void SetLabel(int i){ this->label = i; }
};

#endif /* _DEFINE_H */
