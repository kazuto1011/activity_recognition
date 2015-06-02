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
#include "libsvm/svm.h"

// BoW, Fisher Vector, VLAD
#include "vector_encoding.h"

//----------------------------------------------------------------------------------
// const
//----------------------------------------------------------------------------------
// parameters for feature representation
const std::string EXT = ".txt";
const int HOG_DIM = 72;
const int HOF_DIM = 90;
const int HOGHOF_DIM = 162;
const int FEAT_DIM = HOG_DIM;

// parameters for cv::VideoWriter
const int NUM_FRAME = 150;
const int FPS = 30;

namespace fs = boost::filesystem;

// directory paths
const fs::path FISHER_PARAMS_DIR("/home/kazuto/catkin_ws/src/activity_recognition/activity_recognition_server/params/fisher_vector");
const fs::path VLAD_PARAMS_DIR("/home/kazuto/catkin_ws/src/activity_recognition/activity_recognition_server/params/vlad");
const fs::path BOVW_PARAMS_DIR("/home/kazuto/catkin_ws/src/activity_recognition/activity_recognition_server/params/bag_of_visual_words");

const std::string OUTPUT_DIR = "/home/kazuto/catkin_ws/src/activity_recognition/activity_recognition_server/video/moverio.avi";
const std::string TEXT_DIR   = "/home/kazuto/catkin_ws/src/activity_recognition/activity_recognition_server/moverio.txt";

// init description
const std::string INIT_ENCODE = "Classifier init\n---\n0: Fisher Vector(defalut)\n1: Vector of Locally Aggregated Descriptors\n2: Bag of Visual Words\n---\n";

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
