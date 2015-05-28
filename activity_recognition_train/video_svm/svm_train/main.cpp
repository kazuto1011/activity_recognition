//------------------------------------------------------------------------------
// @file   : main.cpp
// @brief  : Dimension reduction, Fisher Vector encoding, VLAD encoding, SVM
// @author : Kazuto Nakashima
//------------------------------------------------------------------------------
#include "main.h"
#include "opencv_lib.h"
#include "vector_encoding.h"
#include "libsvm.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/timer/timer.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace fs = boost::filesystem;
namespace algo = boost::algorithm;

#if _DEBUG
#include <cstdlib>
#include <crtdbg.h>
#define malloc(X) _malloc_dbg(X,_NORMAL_BLOCK,__FILE__,__LINE__) 
#define new ::new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

int main(int argc, const char * argv[]) {
#if _DEBUG
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif
    //----------------------------------------------------------------------------------
    // init
    //----------------------------------------------------------------------------------
    std::ofstream log("C:\\Users\\kazut_000\\Desktop\\data\\log\\svm_train.log");
    std::ofstream log2("C:\\Users\\kazut_000\\Desktop\\data\\log\\for_thesis.log");
    std::ofstream log3("C:\\Users\\kazut_000\\Desktop\\data\\log\\for_thesis_2.log");
    log2 << "NumData,DimData,PCA%,DimPCA,VecDim\n";
    log3 << "VisualWords,Precision,Time,ValidTime\n";

    boost::timer::cpu_timer entire_time;
    boost::timer::cpu_timer partial_time;
    
    int num_data = 0;
    cv::Mat data_mat = cv::Mat_<float>(num_data, FEAT_DIM);
    std::vector<Video> video_list;

    int num_vw[5] = {10, 20, 50, 100, 200};

#if _DEBUG
    log << "<< Init >>" << std::endl;
    log << "data_mat.rows: " << data_mat.rows << std::endl;
    log << "data_mat.cols: " << data_mat.cols << std::endl;
#endif

    //----------------------------------------------------------------------------------
    // Loading
    //----------------------------------------------------------------------------------
    partial_time.start();
    fs::path input_dir(INPUT_DIR);
    std::string categ;
    int idx = 0;
    cv::Mat tmp_mat;

    BOOST_FOREACH(const fs::path& video_path, std::make_pair(fs::directory_iterator(input_dir), fs::directory_iterator())) {
        categ = AssignLabel(video_path.leaf().string());
        if (fs::is_directory(video_path)) {
            BOOST_FOREACH(const fs::path& input_path, std::make_pair(fs::recursive_directory_iterator(video_path), fs::recursive_directory_iterator())) {
                if (fs::is_directory(input_path) || !algo::iends_with(input_path.string(), EXT)) // dir or not .txt
                    continue;

                std::cout << input_path << std::endl;

                if (!LoadMat(input_path.string(), tmp_mat)) {
                    log << "failed to open error: " << input_path << std::endl;
                    exit(0);
                }
                else {
                    num_data += tmp_mat.rows;
                    video_list.push_back(Video(idx, tmp_mat.rows, atoi(categ.c_str())));
                    data_mat.resize(num_data);
                    tmp_mat.copyTo(data_mat.rowRange(cv::Range(idx, num_data)));
                    tmp_mat.release();
                    idx = num_data;
                }

            }
        }
    }

#if _DEBUG
    log << "\n<< Features loading >>" << std::endl;
    log << "data_mat.rows: " << data_mat.rows << std::endl;
    log << "data_mat.cols: " << data_mat.cols << std::endl;
    for (auto itr = video_list.begin(); itr != video_list.end(); itr++)
        log << "index: " << itr->GetIdx() << "\tnum_data: " << itr->GetNumData() << "\tlabel: " << itr->GetLabel() << std::endl;
    log << "Time: " << partial_time.format();
#endif

    int repetition, encoding_mode = 2, vw_mode = 0;

#if ENCODE
    for (encoding_mode = 0; encoding_mode < 3; encoding_mode++) {
#endif
#if VISUAL_WORDS
        for (vw_mode = 0; vw_mode < 5; vw_mode++) {
#endif
            boost::timer::cpu_timer validate_time;
            validate_time.stop();

            int results[5] = {};
            entire_time.start();

            for (repetition = 0; repetition < REPETITION; repetition++) {
                srand((unsigned)time(NULL));
                //----------------------------------------------------------------------------------
                // Data set
                //----------------------------------------------------------------------------------
                partial_time.start();

                // Test set
                cv::Mat test_mat = cv::Mat_<float>(1, FEAT_DIM);
                std::vector<Video> test_list;

                // Train set
                cv::Mat train_mat = data_mat;
                std::vector<Video> train_list;
                std::copy(video_list.begin(), video_list.end(), std::back_inserter(train_list));

                CreateTestSet(train_mat, test_mat, train_list, test_list, NUM_TEST);
#if _DEBUG
                log << "\n<< Train set >>" << std::endl;
                log << "train_mat.rows: " << train_mat.rows << std::endl;
                log << "train_mat.cols: " << train_mat.cols << std::endl;
                for (auto itr = train_list.begin(); itr != train_list.end(); itr++)
                    log << "index: " << itr->GetIdx() << "\tnum_data: " << itr->GetNumData() << "\tlabel: " << itr->GetLabel() << std::endl;

                log << "\n<< Test set >>" << std::endl;
                log << "test_mat.rows: " << test_mat.rows << std::endl;
                log << "test_mat.cols: " << test_mat.cols << std::endl;
                for (auto itr = test_list.begin(); itr != test_list.end(); itr++)
                    log << "index: " << itr->GetIdx() << "\tnum_data: " << itr->GetNumData() << "\tlabel: " << itr->GetLabel() << std::endl;
                log << "Time: " << partial_time.format();
#endif
                log2 << train_mat.rows << "," << train_mat.cols << ",";
                //-----------------------------------------------------------------------------------
                // PCA (Principal component analysis)
                //-----------------------------------------------------------------------------------
                partial_time.start();
                cv::PCA pca(train_mat, cv::Mat(), CV_PCA_DATA_AS_ROW, 0);
                cv::Mat eigen = pca.eigenvalues;
#if 1
                float total = 0.0;
                for (int i = 0; i < pca.eigenvalues.rows; i++){
                    total += ((float*)eigen.data)[i];
                }

                float cont = 0.0;
                int num_component = 0;
                while (1) {
                    // a coefficient of determination
                    cont += ((float*)eigen.data)[num_component++] / total;
                    if (cont > 0.95) break;
                }
#else
                float total = 0.0;
                for (int i = 0; i < pca.eigenvalues.rows; i++){
                    total += ((float*)eigen.data)[i];
                }

                float cont = 0.0;
                int num_component = 0;
                for (; num_component < 35; num_component++) {
                    cont += ((float*)eigen.data)[num_component] / total;
                }
#endif
                pca.operator()(train_mat, cv::Mat(), CV_PCA_DATA_AS_ROW, num_component);
                SavePCA(pca, PCA_DIR);

                // load the params
                LoadPCA(pca.eigenvectors, pca.eigenvalues, pca.mean, PCA_DIR);
                validate_time.resume();

                // demension-compressed data
                cv::Mat comp_train = cv::Mat_<float>(train_mat.rows, num_component);
                cv::Mat comp_test = cv::Mat_<float>(test_mat.rows, num_component);

                pca.project(train_mat, comp_train);
                pca.project(test_mat, comp_test);
#if _DEBUG
                log << "\n<< PCA >>" << std::endl;
                log << "coefficient of determination: " << cont << std::endl;
                log << "comp_train.rows: " << comp_train.rows << std::endl;
                log << "comp_train.cols: " << comp_train.cols << std::endl;
                log << "Time: " << partial_time.format();
#endif
                log2 << cont << "," << comp_train.cols << ",";

                cv::Mat train_svm;
                cv::Mat test_svm;

                validate_time.stop();
                partial_time.start();
                if (encoding_mode == 0) {
                    //----------------------------------------------------------------------------------
                    // Fisher Vector encoding
                    //----------------------------------------------------------------------------------
                    FisherVector* fisher;

                    fisher = new FisherVector();
                    fisher->GmmCluster(comp_train, num_vw[vw_mode]);
                    fisher->SaveGMM(GMM_DIR.c_str());
                    delete fisher;

                    fisher = new FisherVector(GMM_DIR.c_str());
                    validate_time.resume();
                    cv::Mat fisher_train = GetFisherMat(fisher, comp_train, train_list);
                    cv::Mat fisher_test = GetFisherMat(fisher, comp_test, test_list);
                    delete fisher;
#if _DEBUG
                    log << "\n<< Fisher Vector >>" << std::endl;
                    log << "fisher_train.rows: " << fisher_train.rows << std::endl;
                    log << "fisher_train.cols: " << fisher_train.cols << std::endl;
                    log << "Time: " << partial_time.format();
#endif                
                    train_svm = fisher_train;
                    test_svm = fisher_test;
                }
                else if (encoding_mode == 1) {
                    //----------------------------------------------------------------------------------
                    // VLAD encoding
                    //----------------------------------------------------------------------------------
                    VLAD* vlad;

                    vlad = new VLAD();
                    vlad->KmeansCluster(comp_train, num_vw[vw_mode]);
                    vlad->SaveKmeans(VLAD_KMEANS_DIR.c_str());
                    delete vlad;

                    vlad = new VLAD(VLAD_KMEANS_DIR.c_str());
                    validate_time.resume();
                    cv::Mat vlad_train = GetVLADMat(vlad, comp_train, train_list);
                    cv::Mat vlad_test = GetVLADMat(vlad, comp_test, test_list);
                    delete vlad;
#if _DEBUG
                    log << "\n<< VLAD >>" << std::endl;
                    log << "vlad_mat.rows: " << vlad_train.rows << std::endl;
                    log << "vlad_mat.cols: " << vlad_train.cols << std::endl;
                    log << "Time: " << partial_time.format();
#endif
                    train_svm = vlad_train;
                    test_svm = vlad_test;
                }
                else {
                    //----------------------------------------------------------------------------------
                    // Bag of Visual Words
                    //----------------------------------------------------------------------------------
                    BoVW* bovw;

                    bovw = new BoVW();
                    bovw->KmeansCluster(comp_train, num_vw[vw_mode]);
                    bovw->SaveKmeans(BOVW_KMEANS_DIR.c_str());
                    delete bovw;

                    bovw = new BoVW(BOVW_KMEANS_DIR.c_str());
                    validate_time.resume();
                    cv::Mat bovw_train = GetBoVWMat(bovw, comp_train, train_list);
                    cv::Mat bovw_test = GetBoVWMat(bovw, comp_test, test_list);
                    delete bovw;
#if _DEBUG
                    log << "\n<< Bag of Visual Words >>" << std::endl;
                    log << "bovw_mat.rows: " << bovw_train.rows << std::endl;
                    log << "bovw_mat.cols: " << bovw_train.cols << std::endl;
                    log << "Time: " << partial_time.format();
#endif
                    train_svm = bovw_train;
                    test_svm = bovw_test;
                }
                log2 << train_svm.cols << std::endl;

                validate_time.stop();
                //----------------------------------------------------------------------------------
                // SVM (Support Vector Machine)
                //----------------------------------------------------------------------------------
                partial_time.start();
                struct svm_problem prob;    // Training data
                struct svm_parameter param; // The parameters for SVM
                struct svm_model *model;    // The model for SVM

                // memory allocation for train data
                prob.l = train_svm.rows;
                prob.y = new double[prob.l];
                prob.x = new svm_node*[prob.l];

                // set the parameters
                SVM::SetParams(&param);

                // store the train data
                int idx;
                svm_node **x_space = new svm_node*[prob.l];

                for (int i = 0; i < prob.l; i++) {
                    x_space[i] = new svm_node[train_svm.cols + 1];
                    idx = 0;
                    prob.y[i] = train_list[i].GetLabel();
                    for (int j = 0; j < train_svm.cols; j++) {
                        x_space[i][idx].index = j + 1;
                        x_space[i][idx].value = train_svm.at<float>(i, j);
                        idx++;
                    }
                    x_space[i][idx].index = -1; // the end of a vector
                    prob.x[i] = x_space[i];
                }

                // training
                model = svm_train(&prob, &param);

                // save the svm model
                if (svm_save_model(SVM_DIR.c_str(), model)) {
                    std::cout << "Failed to save the svm model" << std::endl;
                    exit(0);
                }
#if _DEBUG
                log << "\n<< SVM training >>" << std::endl;
                log << "Time: " << partial_time.format();
#endif
                //load the svm model
                svm_free_and_destroy_model(&model);
                model = svm_load_model(SVM_DIR.c_str());

                validate_time.resume();

                svm_node* x_node = new svm_node[test_svm.cols + 1]; // "+1" means the malloc for the end
                int num_class = svm_get_nr_class(model);
                double *pre_prob = new double[num_class];

                for (unsigned int j = 0; j < test_list.size(); j++) {
                    // store the test data
                    int idx = 0;
                    for (int i = 0; i < test_svm.cols; i++) {
                        x_node[idx].index = i + 1;
                        x_node[idx].value = test_svm.at<float>(j, i);
                        idx++;
                    }
                    x_node[idx].index = -1; // the end of a vector

                    // probability estimation
                    double predict_label = svm_predict_probability(model, x_node, pre_prob);

                    std::cout << "result " << j << ": " << RestoreLabel((int)predict_label) << std::endl;
                    std::cout << "label  " << j << ": " << RestoreLabel(test_list[j].GetLabel()) << std::endl;
                    if (((int)predict_label) == (test_list[j].GetLabel())) {
                        results[(int)predict_label]++;
                    }
                    for (int i = 0; i < num_class; i++) {
                        std::cout << "Class " << i << ": " << pre_prob[i] << std::endl;
                    }
                }
                validate_time.stop();
#if _DEBUG
                log << "\n<< Partial result >>" << std::endl;
                log << "num test: " << NUM_TEST << std::endl;
                for (unsigned int i = 0; i < 5; i++)
                    log << "class " << i << ": " << results[i] << std::endl;
#endif
                // release
                delete[] x_node;
                delete[] pre_prob;
                svm_free_and_destroy_model(&model);
                svm_destroy_param(&param);
                delete[] prob.x;
                delete[] prob.y;
                for (int i = 0; i < prob.l; i++)
                    delete[] x_space[i];
                delete[] x_space;
                std::vector<Video>().swap(test_list);
                std::vector<Video>().swap(train_list);
            }

            entire_time.stop();

            log << "\n<< Entire time >>" << std::endl;
            log << "Time: " << entire_time.format();
            log << "\n<< Validation time >>" << std::endl;
            log << "Time: " << validate_time.format();
            
            log << "\n<< Result >>" << std::endl;
            float partial_rate, entire_rate = 0.0;
            for (unsigned int i = 0; i < 5; i++)
            {
                partial_rate = (float)results[i] / repetition / NUM_TEST;
                entire_rate += partial_rate;
                log << "class " << i << ": " << partial_rate * 100 << "[%]\n";
            }
            log << "entire: " << entire_rate / 5.0 * 100 << "[%]\n";

            log3 << num_vw[vw_mode] << "," << entire_rate / 5.0 * 100 << ",";
            log3 << entire_time.format(3,"%w") << "," << validate_time.format(3,"%w") << std::endl;

#if VISUAL_WORDS
        }
#endif
#if ENCODE
    }
#endif
    std::cout << "finished" << std::endl;
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

cv::Mat GetFisherMat(FisherVector* fisherVec, cv::Mat& comp_mat, std::vector<Video>& video_list) {
    cv::Mat fisherMat = cv::Mat_<float>(video_list.size(), fisherVec->GetDimension());

    int idx = 0, j = 0;
    cv::Mat dataToEncode;
    for (auto itr = video_list.begin(); itr != video_list.end(); itr++) {
        dataToEncode = comp_mat.rowRange(cv::Range(itr->GetIdx(), itr->GetIdx() + itr->GetNumData()));
        fisherVec->FvEncode(dataToEncode).copyTo(fisherMat.row(j++));
        idx = itr->GetIdx() + itr->GetNumData();
    }

    return fisherMat;
}

cv::Mat GetVLADMat(VLAD* vlad, cv::Mat& comp_mat, std::vector<Video>& video_list) {
    cv::Mat vladMat = cv::Mat_<float>(video_list.size(), vlad->GetDimension());

    int idx = 0, j = 0;
    cv::Mat dataToEncode;
    for (auto itr = video_list.begin(); itr != video_list.end(); itr++) {
        dataToEncode = comp_mat.rowRange(cv::Range(itr->GetIdx(), itr->GetIdx() + itr->GetNumData()));
        vlad->VladEncode(dataToEncode).copyTo(vladMat.row(j++));
        idx = itr->GetIdx() + itr->GetNumData();
    }

    return vladMat;
}

cv::Mat GetBoVWMat(BoVW* bovw, cv::Mat& comp_mat, std::vector<Video>& video_list) {
    cv::Mat bovwMat = cv::Mat_<float>(video_list.size(), bovw->GetDimension());

    int idx = 0, j = 0;
    cv::Mat dataToEncode;
    for (auto itr = video_list.begin(); itr != video_list.end(); itr++) {
        dataToEncode = comp_mat.rowRange(cv::Range(itr->GetIdx(), itr->GetIdx() + itr->GetNumData()));
        bovw->BuidHistogram(dataToEncode).copyTo(bovwMat.row(j++));
        idx = itr->GetIdx() + itr->GetNumData();
    }

    return bovwMat;
}

void SavePCA(cv::PCA& pca, const std::string& file_dir) {
    cv::Mat eigenvectors = pca.eigenvectors;
    cv::Mat eigenvalues = pca.eigenvalues;
    cv::Mat mean = pca.mean;

    std::ofstream ofs(file_dir.c_str(), std::ios_base::out | std::ios_base::binary);

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

void LoadPCA(cv::Mat& eigenvectors, cv::Mat& eigenvalues, cv::Mat& mean, const std::string& file_dir) {
    std::ifstream ifs(file_dir.c_str(), std::ios_base::in | std::ios_base::binary);

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

bool LoadMat(const std::string& file_dir, cv::Mat& data) {
    std::ifstream ifs(file_dir.c_str(), std::ios_base::binary);
    if (!ifs) { return false; }

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

bool CreateTestSet(cv::Mat& train, cv::Mat& test, std::vector<Video>& train_list, std::vector<Video>& test_list, const int num_test) {
    int train_idx = 0, test_idx = 0;
    int train_size = 0, test_size = 0;
    int count = 0;
    int pre_label = train_list.begin()->GetLabel();
    int tmp_label;

    cv::Mat tmp_train = test.clone();

    if (num_test >= train.rows)
        return false;

    // shuffle vectors on each class
    std::vector<Video>::iterator shuffle_begin = train_list.begin();
    for (auto itr = train_list.begin(); ;) {
        if (itr != train_list.end()) {
            tmp_label = itr->GetLabel();
        }
        if (tmp_label != pre_label || itr == train_list.end()) {
            std::random_shuffle(shuffle_begin, itr);
            shuffle_begin = itr;
            if (itr == train_list.end())
                break;
            else
                pre_label = itr->GetLabel();
        }
        else {
            ++itr;
        }
    }

    // sample some rows as a test data
    for (auto itr = train_list.begin(); itr != train_list.end();) {
        if (itr->GetLabel() != pre_label)
            count = 0;
        if (count < num_test) {
            test_size += itr->GetNumData();
            test.resize(test_size);
            train.rowRange(cv::Range(itr->GetIdx(), itr->GetIdx() + itr->GetNumData())).copyTo(test.rowRange(cv::Range(test_idx, test_size)));
            test_list.push_back(Video(test_idx, itr->GetNumData(), itr->GetLabel()));
            test_idx = test_size;
            itr = train_list.erase(itr); // itr++
            pre_label = itr->GetLabel(); 
            count++;
        }
        else {
            train_size += itr->GetNumData();
            tmp_train.resize(train_size);
            train.rowRange(cv::Range(itr->GetIdx(), itr->GetIdx() + itr->GetNumData())).copyTo(tmp_train.rowRange(cv::Range(train_idx, train_size)));
            train_idx = train_size;
            ++itr;
        }
    }

    train.release();
    train = tmp_train.clone();
    tmp_train.release();

    // reconstruction
    train_idx = 0;
    for (auto itr = train_list.begin(); itr != train_list.end(); itr++) {
        itr->SetIdx(train_idx);
        train_idx += itr->GetNumData();
    }

    return true;
}