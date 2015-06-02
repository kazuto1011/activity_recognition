#include <stdlib.h>
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

// boost
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>

namespace fs = boost::filesystem;

//----------------------------------------------------------------------------------
// time measurement
//----------------------------------------------------------------------------------
std::chrono::system_clock::time_point GetTime() {
    return std::chrono::system_clock::now();
}

long long GetTime(std::chrono::system_clock::time_point start,
    std::chrono::system_clock::time_point end) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

void RemoveContent(const char* file_dir) {
    fs::path path(file_dir);
    BOOST_FOREACH(const fs::path& activity_dir, std::make_pair(fs::directory_iterator(path), fs::directory_iterator())) {
        if (fs::is_directory(activity_dir)) {
            BOOST_FOREACH(const fs::path& p, std::make_pair(fs::directory_iterator(activity_dir), fs::directory_iterator())) {
                std::cout << p << std::endl;
                fs::remove(p);
            }
        }
    }
}

void SaveMat(const std::string& file_dir, cv::Mat& data) {
    std::ofstream ofs(file_dir, std::ios_base::out | std::ios_base::binary);

    int type = data.type();
    ofs.write((char*)&data.rows, sizeof(int));
    ofs.write((char*)&data.cols, sizeof(int));
    ofs.write((char*)&type, sizeof(int));
    ofs.write((char*)data.data, data.elemSize() * data.total());

    ofs.close();
}