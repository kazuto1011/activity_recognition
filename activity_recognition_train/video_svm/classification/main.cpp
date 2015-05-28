//------------------------------------------------------------------------------
// @file   : main.cpp
// @brief  : 
// @author : Kazuto Nakashima
//------------------------------------------------------------------------------
#include "main.h"

#include "opencv_lib.h"
#include "vector_encoding.h"
#include "libsvm.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>

namespace fs = boost::filesystem;

int main(int argc, const char * argv[]) {
    //----------------------------------------------------------------------------------
    // init
    //----------------------------------------------------------------------------------
    std::ofstream log("C:\\Users\\kazut_000\\Desktop\\data\\log\\classification.log");

    std::vector<Video> video_list;
    
    int num_data = 1;
    cv::Mat hog_mat = cv::Mat_<float>(num_data, 72);
    cv::Mat hof_mat = cv::Mat_<float>(num_data, 90);
    cv::Mat hoghof_mat = cv::Mat_<float>(num_data, 162);
    
    //-----------------------------------------------------------------------------------
    // STIP
    //-----------------------------------------------------------------------------------
    fs::path input_path, output_path;
    fs::path video("test.avi");

    input_path = TEST_DIR / video;
    output_path = input_path;
    output_path.replace_extension("txt");

    std::cout << "Test video:  " << input_path << std::endl;
    std::cout << "STIP output: " << output_path << std::endl;

#if 1
    // STIP detector and deescriptor
    std::string stip_det = STIPDET;
    stip_det = stip_det + " -f " + input_path.string();
    stip_det = stip_det + " -o " + output_path.string();
    stip_det += VIS;
    system(stip_det.c_str());
#endif

    // File stream
    std::ifstream ifs(output_path.c_str());
    if (!ifs) { return -1; }

    std::string buf;
    int count[2];
    float tmp;
    size_t p;

    while (ifs && getline(ifs, buf)) {
        if (buf.substr(0, 1) != "#") {
            count[0] = 7;  // info of the features
            count[1] = 72; // the dimension of hog
            
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

    video_list.push_back(Video(num_data - 1, -1));
    
    std::cout << "<< STIP output >>" << std::endl;
    std::cout << "num_data: " << num_data << std::endl;
    std::cout << "hog_mat.cols: " << hog_mat.cols << std::endl;
    std::cout << "hof_mat.cols: " << hof_mat.cols << std::endl;
    std::cout << "hoghof_mat.cols: " << hoghof_mat.cols << std::endl;

    //-----------------------------------------------------------------------------------
    // PCA (Principal component analysis)
    //-----------------------------------------------------------------------------------
    cv::PCA pca;

    // Load the parameters for the principal components
    LoadPCA(pca.eigenvectors, pca.eigenvalues, pca.mean, PCA_DIR);

    // demension-compressed data
    cv::Mat comp_mat = cv::Mat_<float>(num_data, pca.eigenvalues.rows);
    pca.project(FEATURES, comp_mat);

    std::cout << "<< PCA >>" << std::endl;
    std::cout << "comp_mat.rows: " << comp_mat.rows << std::endl;
    std::cout << "comp_mat.cols: " << comp_mat.cols << std::endl;

    for (auto itr = video_list.begin(); itr != video_list.end(); itr++)
        std::cout << "index: " << itr->GetIdx() << "\tlabel: " << itr->GetLabel() << std::endl;

#if 1
    //----------------------------------------------------------------------------------
    // Fisher Vector encoding
    //----------------------------------------------------------------------------------
    FisherVector fisher(GMM_DIR);

    std::cout << "<< Fisher Vector >>" << std::endl;
    cv::Mat fisher_mat = GetFisherMat(fisher, comp_mat, video_list);
    
    cv::Mat svm_mat = fisher_mat;
#else
    //----------------------------------------------------------------------------------
    // VLAD encoding
    //----------------------------------------------------------------------------------
    VLAD vlad(KMEANS_DIR);

    std::cout << "<< VLAD >>" << std::endl;
    cv::Mat vlad_mat = GetVladMat(vlad, comp_mat, video_list);
    
    cv::Mat svm_mat = vlad_mat;
#endif

    //----------------------------------------------------------------------------------
    // SVM prediction
    //----------------------------------------------------------------------------------
    struct svm_model* model;    // The model for SVM

    model = svm_load_model(SVM_DIR);

    // store the test data
    svm_node* x_node = new svm_node[svm_mat.cols + 1]; // "+1" means the malloc for the end
    int idx = 0;
    for (int i = 0; i < svm_mat.cols; i++) {
        x_node[idx].index = i + 1;
        x_node[idx].value = svm_mat.at<float>(0, i); // from the histogram
        idx++;
    }
    x_node[idx].index = -1; // the end of a vector

    // probability estimation
    int num_class = svm_get_nr_class(model);
    double *pre_prob = new double[num_class];
    double predict_label = svm_predict_probability(model, x_node, pre_prob);

    std::cout << "The predicted result: " << RestoreLabel((int)predict_label) << std::endl;
    for (int i = 0; i < num_class; i++) {
        std::cout << "Class " << i << ": " << pre_prob[i] << std::endl;
    }

    svm_free_and_destroy_model(&model);
    delete[] x_node;

    std::cout << "hit any key" << std::endl;
    getchar();
    return 0;
}

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

//----------------------------------------------------------------------------------
// Support Vector Machine
//----------------------------------------------------------------------------------
namespace SVM {
    void SetParams(svm_parameter *param) {
        param->svm_type = C_SVC;
        param->kernel_type = LINEAR;
        param->gamma = 2;
        param->C = 1000;
        param->degree = 2;
        param->coef0 = 0.8;
        param->nu = 1000;
        param->cache_size = 10;
        param->eps = 1e-3;
        param->p = 0.1;
        param->shrinking = 1;
        param->probability = 1;
        param->nr_weight = 0;
        param->weight_label = NULL;
        param->weight = NULL;
    }

    void StoreTrainData(svm_problem *prob, cv::Mat& train_data,
        std::vector<Video> video_list) {
        int num_dimension = train_data.cols;
        int idx;

        svm_node **x_space = new svm_node*[prob->l];
        for (int i = 0; i < prob->l; i++) {
            x_space[i] = new svm_node[num_dimension + 1];
            idx = 0;
            prob->y[i] = video_list[i].GetLabel();
            for (int j = 0; j < num_dimension; j++) {
                x_space[i][idx].index = j + 1;
                x_space[i][idx].value = train_data.at<float>(i, j);
                idx++;
            }
            x_space[i][idx].index = -1; // the end of a vector
            prob->x[i] = x_space[i];
        }
    }
}

//------------------------------------------------------------------------------
// 0: eat a meal
// 1: gaze at a robot
// 2: gaze at a tree
// 3: look around
// 4: read a book
//------------------------------------------------------------------------------
std::string AssignLabel(std::string str) {
    if (!str.compare("eat_a_meal")) return "0";
    else if (!str.compare("gaze_at_a_robot")) return "1";
    else if (!str.compare("gaze_at_a_tree")) return "2";
    else if (!str.compare("look_around")) return "3";
    else if (!str.compare("read_a_book")) return "4";
    else return "-1";
}

std::string RestoreLabel(int label) {
    switch (label) {
    case 0: return "eat_a_meal";
    case 1: return "gaze_at_a_robot";
    case 2: return "gaze_at_a_tree";
    case 3: return "look_around";
    case 4: return "read_a_book";
    default: return "-1";
    }
}

cv::Mat GetFisherMat(FisherVector& fisherVec, cv::Mat& comp_mat, std::vector<Video> video_list) {
    cv::Mat fisherMat = cv::Mat_<float>(video_list.size(), fisherVec.GetDimension());

    int idx = 0, j = 0;
    for (auto itr = video_list.begin(); itr != video_list.end(); itr++) {
        cv::Mat dataToEncode = comp_mat.rowRange(cv::Range(idx, itr->GetIdx() + 1));
        fisherVec.FvEncode(dataToEncode).copyTo(fisherMat.row(j++));
        idx = itr->GetIdx() + 1;
    }

    return fisherMat;
}

cv::Mat GetVladMat(VLAD& vlad, cv::Mat& comp_mat, std::vector<Video> video_list) {
    cv::Mat vladMat = cv::Mat_<float>(video_list.size(), vlad.GetDimension());

    int idx = 0, j = 0;
    for (auto itr = video_list.begin(); itr != video_list.end(); itr++) {
        cv::Mat dataToEncode = comp_mat.rowRange(cv::Range(idx, itr->GetIdx() + 1));
        vlad.KmeansQuantize(dataToEncode);
        vlad.VladEncode(dataToEncode).copyTo(vladMat.row(j++));
        idx = itr->GetIdx() + 1;
    }

    return vladMat;
}

void SavePCA(cv::PCA& pca, const char* file_dir) {
    cv::Mat eigenvectors = pca.eigenvectors;
    cv::Mat eigenvalues = pca.eigenvalues;
    cv::Mat mean = pca.mean;

    std::ofstream ofs(file_dir, std::ios_base::out | std::ios_base::binary);

    int type = eigenvectors.type();
    ofs.write((char*)&eigenvectors.rows, sizeof(int));
    ofs.write((char*)&eigenvectors.cols, sizeof(int));
    ofs.write((char*)&type, sizeof(int));
    ofs.write((char*)eigenvectors.data, eigenvectors.elemSize() * eigenvectors.total());

    type = eigenvalues.type();
    ofs.write((char*)&eigenvalues.rows, sizeof(int));
    ofs.write((char*)&eigenvalues.cols, sizeof(int));
    ofs.write((char*)&type, sizeof(int));
    ofs.write((char*)eigenvalues.data, eigenvalues.elemSize() * eigenvalues.total());

    type = mean.type();
    ofs.write((char*)&mean.rows, sizeof(int));
    ofs.write((char*)&mean.cols, sizeof(int));
    ofs.write((char*)&type, sizeof(int));
    ofs.write((char*)mean.data, mean.elemSize() * mean.total());

    ofs.close();
}

void LoadPCA(cv::Mat& eigenvectors, cv::Mat& eigenvalues, cv::Mat& mean, const char* file_dir) {
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

