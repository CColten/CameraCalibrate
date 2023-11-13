#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <map>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

#include "define_type.h"

#include "yaml-cpp/yaml.h"

class CameraCalibration
{
public:
    CameraCalibration();
    ~CameraCalibration();

    ErrInfo ProcessInternal(const std::vector< cv::Mat >& images, MODETYPE type);
    ErrInfo ProccessSingleImage(const cv::Mat& img, MODETYPE type, int frame_count=0);

    ErrInfo CalAnchors(const cv::Mat& img_src, MODETYPE type, double_t lidar_range, cv::Point3f& camera_coordinate_point);

    cv::Point3f GetCamCoordinatePoint();

    ErrInfo GetLidarParamsFromFile(const std::string lidar_params_file);
    ErrInfo GetCameraInternalParams(const std::string camera_internal_params_file);
    ErrInfo SaveTolog();

public:
    void SetBoardSize(float square_size, int board_rows, int board_cols){
        this->board_size_.width = board_cols - 1;
        this->board_size_.height = board_rows - 1;
        this->square_size_ = square_size;
        return;
    }

    void SetNumReqFrames(int num){
        if (num > 0){
            this->num_req_frames_ = num;
        }
    }

    void SetWindowSize(int win_size){
        this->win_size_ = win_size;
    }

    void SetIntrinsicFile(std::string file){
        this->intrinsic_file_ = file;
        return;
    }

    void SetIsSaveImagePointsImg(bool is_save_image_points){
        this->is_save_image_points_ = is_save_image_points;
    }

public:
    ErrInfo ProcessExternal(const std::vector< cv::Mat >& images, MODETYPE type);



    ErrInfo CalInternalParams(MODETYPE type);

    void SaveInternalParamToFile(std::map< std::string, cv::Mat >& intrinsic_params);

public:
    int num_req_frames_{20};
    int win_size_{11};
    double max_rms_tol_{1.0};

    float square_size_{15.f}; // 棋盘格尺寸
    int board_width_ = 9 - 1;
    int board_height_ = 7 - 1;
    bool is_save_image_points_{false};

    cv::Size board_size_{8,6};
    cv::Size img_size_;

    MODETYPE mode_{MODETYPE::MODE_INVALID};

    bool intrinsic_file_exist_{false};
    std::string intrinsic_file_{"./intrinsic_file.txt"};
    std::vector< std::vector< cv::Point2f > > image_points_;
    std::map< std::string, cv::Mat > intrinsic_params_;

    // cal
    cv::Mat rt_chessboard_to_lidar_;
    cv::Point3f camera_coordinate_point_;

    // for Debug test
    int frame_count_{0};
    std::ofstream fout;

    // yaml files
    YAML::Node yaml_node;
    std::string key_for_lidar_{"RT_chessboard_to_lidar"};
    std::string key_for_cam_k_{"K"};
    std::string key_for_cam_dist_{"dist"};

};

#endif // CAMERACALIBRATION_H
