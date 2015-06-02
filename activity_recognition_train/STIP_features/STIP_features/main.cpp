//------------------------------------------------------------------------------
// @file   : main.cpp
// @brief  : STIP detection, HOG/HOF extraction
// @author : Kazuto Nakashima
//------------------------------------------------------------------------------
#include "main.h"

#define STIPDET    "C:\\stip-1.1-winlinux\\bin\\stipdet"
#define VIS        " -vis no" // visualization option: yes/no

#define INPUT_DIR  "C:\\Users\\kazut_000\\Desktop\\data\\videos"
#define OUTPUT_DIR "C:\\Users\\kazut_000\\Desktop\\data\\stip_output"
#define HOG_DIR    "C:\\Users\\kazut_000\\Desktop\\data\\stip_hog"
#define HOF_DIR    "C:\\Users\\kazut_000\\Desktop\\data\\stip_hof"
#define HOGHOF_DIR "C:\\Users\\kazut_000\\Desktop\\data\\stip_hoghof"

#define HOG        "_hog"
#define HOF        "_hof"
#define HOGHOF     "_hoghof"
#define PREFIX     ".avi"
#define BREAK printf("hit any key"); getchar()

namespace fs = boost::filesystem;
namespace algo = boost::algorithm;

int main()
{
    std::ofstream log("C:\\Users\\kazut_000\\Desktop\\data\\log\\STIP_features.log");

    fs::path output_path, hog_path, hof_path, hoghof_path;
    fs::path file_name;
    fs::path input_dir(INPUT_DIR);
    fs::path categ;

    auto start = GetTime();

#if 1
    RemoveContent(OUTPUT_DIR);
    RemoveContent(HOG_DIR);
    RemoveContent(HOF_DIR);
    RemoveContent(HOGHOF_DIR);
#endif

    BOOST_FOREACH(const fs::path& video_path, 
        std::make_pair(fs::directory_iterator(input_dir) , fs::directory_iterator())) {
        categ = video_path.leaf();
        if (fs::is_directory(video_path)) {
            int num_video = 0;
            BOOST_FOREACH(const fs::path& input_path, 
                std::make_pair(fs::recursive_directory_iterator(video_path), fs::recursive_directory_iterator())) {
                if (fs::is_directory(input_path) || !algo::iends_with(input_path.string(), PREFIX)) // dir or not .avi
                continue;
                
                int num_data = 1;
                cv::Mat hog_mat = cv::Mat_<float>(num_data, 72);
                cv::Mat hof_mat = cv::Mat_<float>(num_data, 90);
                cv::Mat hoghof_mat = cv::Mat_<float>(num_data, 162);

                // Changes the prefix and extracts the file name
                output_path = input_path;
                output_path = output_path.filename();
                file_name = output_path.replace_extension("txt");

                // Edit the file names
                hog_path = file_name.stem().string() + HOG + file_name.extension().string();
                hof_path = file_name.stem().string() + HOF + file_name.extension().string();
                hoghof_path = file_name.stem().string() + HOGHOF + file_name.extension().string();

                // Concatenation
                output_path = OUTPUT_DIR / categ / file_name;
                hog_path = HOG_DIR / categ / hog_path;
                hof_path = HOF_DIR / categ / hof_path;
                hoghof_path = HOGHOF_DIR / categ / hoghof_path;

                // STIP detector and deescriptor
                std::string stip_det = STIPDET;
                stip_det = stip_det + " -f " + input_path.string();
                stip_det = stip_det + " -o " + output_path.string();
                stip_det += VIS;
                system(stip_det.c_str());

                // File stream
                std::ifstream ifs(output_path.c_str()); // from stip_output
                if (!ifs) return -1; 
                
                std::string buf;
                int count[3] = {};
                float tmp;
                size_t p;
                
                auto load_start = GetTime();
                while(ifs && getline(ifs, buf)) {
                    if (buf.substr(0, 1) != "#") {
                        count[0] = 7;  // info of the features
                        count[1] = 72; // the dimension of hog
                        count[2] = 1;  // check

                        // point-type x y t sigma2 tau2 detector-confidence
                        while (((p = buf.find('\t')) != buf.npos) && count[0]) {
                            buf = buf.substr(p + 1);
                            count[0]--;
                        }

                        // dscr-hog(72)
                        count[0] = 0;
                        while (((p = buf.find('\t')) != buf.npos) && count[1]) {
                            tmp = std::stof(buf.substr(0, p + 1));
                            hog_mat.at<float>(num_data - 1, count[0]) = tmp;
                            hoghof_mat.at<float>(num_data - 1, count[0]++) = tmp;
                            buf = buf.substr(p + 1);
                            count[1]--;
                        }
                        
                        // dscr-hof(90)
                        count[1] = count[0];
                        count[0] = 0;
                        while ((p = buf.find('\t')) != buf.npos) {
                            tmp = std::stof(buf.substr(0, p + 1));
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
                ifs.close();
                auto load_end = GetTime();
                log << "load time: " << GetTime(load_start, load_end) << std::endl;

                if (!count[2]) {
                    log << categ << ":\tno feature " << input_path << std::endl;
                }
                else {
                    SaveMat(hog_path.string(), hog_mat);
                    SaveMat(hof_path.string(), hof_mat);
                    SaveMat(hoghof_path.string(), hoghof_mat);
                    num_video++;
                }

                std::cout << "<< STIP output >>" << std::endl;
                std::cout << "num_data: " << num_data << std::endl;
                std::cout << "hog_mat.cols: " << hog_mat.cols << std::endl;
                std::cout << "hof_mat.cols: " << hof_mat.cols << std::endl;
                std::cout << "hoghof_mat.cols: " << hoghof_mat.cols << std::endl;
            }
            log << categ << ":\t" << num_video << std::endl;
        }
    }

    auto end = GetTime();
    log << "\n<< Time >>" << std::endl;
    log << GetTime(start, end) / 1000;
    log << "[sec]" << std::endl;

    return 0;
}