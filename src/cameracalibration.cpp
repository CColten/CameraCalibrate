#include "cameracalibration.h"

#include "define_type.h"
#include <fstream>
#include <iostream>

static int s_frame_num = 0;

inline ErrInfo GetValueFromYaml(const std::string file, const std::string key_word, const cv::Size& size, cv::Mat& dst)
{
    ErrInfo err;
    try {
        YAML::Node node = YAML::LoadFile(file);

        if (!node[key_word].IsDefined()){
            err.err_str = key_word + " is null";;
            err.ret = ERRTYPE::M_NO_KEY_WORD;
            return err;
        }
        else{
            auto config = node[key_word];

            if (config.size() != static_cast<size_t>(size.height)){
                err.err_str  = key_word + " size(" + std::to_string(config.size()) + ") not equal to " + std::to_string(size.height) + ", please check";
                err.ret = ERRTYPE::M_KEY_WORD_SIZE_NOT_MATCHED;
                return err;
            }

            int count_1 = 0;
            for (auto conf : config) {
                if (conf.size() != static_cast<size_t>(size.width)){
                    err.err_str  = key_word + " size(" + std::to_string(conf.size()) + ") not equal to " + std::to_string(size.width) + ", please check";
                    err.ret = ERRTYPE::M_KEY_WORD_SIZE_NOT_MATCHED;
                    return err;
                }
                int count_2 = 0;
                for (auto item : conf){
                    dst.at<double>(count_1, count_2++) = item.as<double>();
                }
                count_1++;
            }
        }
    } catch (...) { //文件为非yaml格式抛出异常
        err.err_str = "please check params file format, yaml is ok";
        err.ret = ERRTYPE::M_NO_KEY_WORD;
        return err;
    }

    return err;
}


CameraCalibration::CameraCalibration()
{

    rt_chessboard_to_lidar_ = (cv::Mat_<double>(4,4) <<
                                   0.9989342265175298,0.0025906692236498455,0.046083614493639034,39.58549592903438,
                               -0.0030839221875314767,0.9999386852638205,0.010635560013631034,32.92229167931263,
                               -0.04605323567097056,-0.010766343197015865,0.9988809665512695,-14.318462718106565,
                               0.0,0.0,0.0,1.0);
    fout.open("cal_anchor_internal_params.txt");
    if(!fout.is_open()){
        std::cout << "cal_anchor_internal_params.txt not opened!" << std::endl;
    }
}

CameraCalibration::~CameraCalibration(){
    if(fout.is_open()){
        fout.close();
    }
}

void CameraCalibration::SaveInternalParamToFile(std::map< std::string, cv::Mat >& intrinsic_params){

    auto camera_params = intrinsic_params["K"];
    auto dist = intrinsic_params["dist"];

    std::ofstream fout("camera_internal_config.yaml");
    if (!fout.is_open()){
        std::cout << intrinsic_file_ << " not exist" << std::endl;
        return;
    }

    if (!camera_params.empty()){
        fout << "K:" << std::endl; 
        for (size_t i = 0; i < camera_params.rows; i++)
        {
            for (size_t j = 0; j < camera_params.cols; j++)
            {
                if (j==0) {
                        fout << " - ";
                }else{
                    fout << "   ";
                }
                fout << "- " << camera_params.at<double>(i,j) << std::endl;
            }
        }
    }

    if (!dist.empty()){
        fout << "dist:" << std::endl; 
        for (size_t i = 0; i < dist.rows; i++)
        {
            for (size_t j = 0; j < dist.cols; j++)
            {
                if (i==0) { fout << " - ";}
                else{ fout << "   ";}
                fout << "- " << dist.at<double>(i,j) << std::endl;
            }
        }
    }

    fout.close();

    return;
}


ErrInfo CameraCalibration::SaveTolog(){
    ErrInfo err;

    cv::Mat intrinsic_params_k = intrinsic_params_[key_for_cam_k_];
    cv::Mat intrinsic_params_dist = intrinsic_params_[key_for_cam_dist_];
    if(fout.is_open()){
        fout << "key_for_cam_k_:" << std::endl << intrinsic_params_[key_for_cam_k_] << std::endl;
        fout << "key_for_cam_dist_:" << std::endl << intrinsic_params_[key_for_cam_dist_] << std::endl;
    }


    err.err_str = "";
    err.ret = ERRTYPE::M_OK;

    return err;
}

ErrInfo CameraCalibration::ProcessInternal(const std::vector< cv::Mat >& images, MODETYPE type){
    ErrInfo err;
    int frame_count = 0;
    for (const auto& img : images) {
        ProccessSingleImage(img, type, frame_count++);
    }

    CalInternalParams(type);

    {
        cv::Mat intrinsic_params_k = intrinsic_params_[key_for_cam_k_];
        cv::Mat intrinsic_params_dist = intrinsic_params_[key_for_cam_dist_];
        if(fout.is_open()){
            fout << "key_for_cam_k_:" << std::endl << intrinsic_params_[key_for_cam_k_] << std::endl;
            fout << "key_for_cam_dist_:" << std::endl << intrinsic_params_[key_for_cam_dist_] << std::endl;
        }
    }

    err.err_str = "内参标定完成";
    err.ret = ERRTYPE::M_OK;

    return err;
}

ErrInfo CameraCalibration::ProccessSingleImage(const cv::Mat &img, MODETYPE type, int frame_count)
{
    ErrInfo err;

    if (img_size_.empty()){
        this->img_size_ = img.size();
    }

    if (MODETYPE::MODE_INTERNAL == type){
        if (intrinsic_file_exist_){
            err.ret = ERRTYPE::M_OK;
//            err.err_str = "相机内参文件已存在，退出内参标定";
            err.err_str = "llll";
            return err;
        }

        //        int num_valid{0};

        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

        std::vector< cv::Point2f > corners;
        auto ret = cv::findChessboardCorners(img_gray, board_size_, corners);
        if (ret){
            cv::cornerSubPix(img_gray, corners, cv::Size(win_size_, win_size_), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
            image_points_.push_back(corners);

            for (size_t i = 0; i < corners.size(); i++)
            {
                cv::circle(img, cv::Point2f(corners[i].x, corners[i].y), 2, cv::Scalar(0, 0, 255), 2);
            }

            cv::imwrite(std::to_string(frame_count) + ".jpg", img);

            err.err_str = "success";
            err.ret = ERRTYPE::M_OK;
            return err;
            
        }else{
            err.err_str = "find ChessboardCorners failed";
            err.ret = ERRTYPE::M_BOARD_CORNOR_MATCH_FAILD;
            return err;
        }
    }

    return err;
}

ErrInfo CameraCalibration::CalInternalParams(MODETYPE type)
{
    intrinsic_params_.clear();
    ErrInfo err;

    if ((MODETYPE::MODE_INTERNAL == type && image_points_.size() >= static_cast< size_t >(num_req_frames_)) ||
        (MODETYPE::MODE_INTERNAL != type && !intrinsic_file_exist_)){

        cv::Mat camera_mat = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat dist_coeffs_mat = cv::Mat::zeros(5, 1, CV_64F);

        std::vector< std::vector< cv::Point3f > > object_points(1);
        std::vector< cv::Mat> rvecs, tvecs;
        double rms = 0.0;


        //生成棋盘格角点的三维坐标
        for (int row = 0; row < board_size_.height; ++row) {
            for (int col = 0; col < board_size_.width; ++col) {
                object_points[0].push_back(cv::Point3f(col*square_size_, row*square_size_, 0.f));
            }
        }

        object_points.resize(image_points_.size(), object_points[0]);

        rms = cv::calibrateCamera(object_points, image_points_, img_size_, camera_mat, dist_coeffs_mat, rvecs, tvecs);

        if(is_save_image_points_){
            cv::Mat img_save = cv::Mat(this->img_size_, CV_8UC3, cv::Scalar(0,0,0));
            for(const auto& img_points : image_points_){
                for(const auto& pt : img_points){
                    cv::circle(img_save, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)), 1, cv::Scalar(0,0,255), 1);
                }
            }
            cv::imwrite("internal_points.jpg", img_save);
        }

        auto ok = cv::checkRange(camera_mat) && cv::checkRange(dist_coeffs_mat);

        if (rms > max_rms_tol_){
            mode_ = MODETYPE::MODE_INTERNAL;
            err.err_str = "internal projection error is too big, please try again";
            err.ret = ERRTYPE::M_CAMERA_DEVIATION_TOO_BIG;
        }
        else if ( !ok ){
            mode_ = MODETYPE::MODE_INTERNAL;
            err.err_str = "calculate internal params wrong, please try again";
            err.ret = ERRTYPE::M_CAMERA_INTRINSIC_RESULT_VALID;
        }
        else{
            err.ret = ERRTYPE::M_OK;
            err.err_str = "calculate internal params done";

            //            std::map< std::string, cv::Mat > intrinsic_params;
            intrinsic_params_["K"] = camera_mat;
            intrinsic_params_["dist"] = dist_coeffs_mat;

            SaveInternalParamToFile(intrinsic_params_);
        }
    }

    return err;
}


ErrInfo CameraCalibration::CalAnchors(const cv::Mat& img,
                                      MODETYPE type,
                                      double_t lidar_range,
                                      cv::Point3f& camera_coordinate_point){
    ErrInfo err;

    {
        cv::Mat intrinsic_params_k = intrinsic_params_[key_for_cam_k_];
        cv::Mat intrinsic_params_dist = intrinsic_params_[key_for_cam_dist_];
        if(fout.is_open()){
            fout << "key_for_cam_k:" << std::endl << intrinsic_params_[key_for_cam_k_] << std::endl;
            fout << "key_for_cam_dist:" << std::endl << intrinsic_params_[key_for_cam_dist_] << std::endl;
            fout << "rt_chessboard_to_lidar:" << std::endl << rt_chessboard_to_lidar_ <<std::endl;
        }
    }

    if (MODETYPE::MODE_ANCHORS == type){
        std::vector< cv::Point2f > image_points;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        auto foundChessboard = cv::findChessboardCorners(gray, board_size_, image_points);
        bool valid_lidar_range = (lidar_range > 0);

        if (!foundChessboard){
            err.err_str = "chessboard corners matched failed, please try again";
            err.ret = ERRTYPE::M_BOARD_CORNOR_MATCH_FAILD;
            return err;
        }
        else if (!valid_lidar_range){
            err.err_str = "lidar failed, please try aggin";
            err.ret = ERRTYPE::M_LIDAR_CALCULATE_FAILD;
            return err;
        }
        else{
            cv::cornerSubPix(gray, image_points, cv::Size(win_size_, win_size_), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
        }


        std::vector< cv::Point3f > object_points;

        // 假设imagePoints的顺序是从左上角先向右，再向下
        // 生成棋盘格的三维坐标，注意棋盘格三维坐标的原点和方向
        for (int row = 0; row < board_size_.height; ++row) {
            for (int col = 0; col < board_size_.width; ++col) {
                object_points.push_back(cv::Point3f(-col*square_size_, row*square_size_, 0.f));
            }
        }

        // 用PnP求解位姿
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

        cv::Mat intrinsic_params_k = intrinsic_params_[key_for_cam_k_];
        cv::Mat intrinsic_params_dist = intrinsic_params_[key_for_cam_dist_];


        auto ret = cv::solvePnP(object_points, image_points, intrinsic_params_k, intrinsic_params_dist, rvec, tvec);

        if (!ret){
            err.err_str = "call solvePnP error";
            err.ret = ERRTYPE::M_OK;
            return err;
        }

        cv::Mat rotation;
        cv::Rodrigues(rvec, rotation);

        cv::Mat RT_camera_to_chessboard = cv::Mat::zeros(4, 4, CV_64FC1);

        for (int row = 0; row < 3; row++){
            for (int col = 0; col < 3; col++){
                RT_camera_to_chessboard.at<double>(row, col) = rotation.at<double>(row, col);
            }
            RT_camera_to_chessboard.at<double>(row, 3) = tvec.at<double>(row, 0);
        }
        RT_camera_to_chessboard.at<double>(3, 3) = 1.0;

        cv::Mat anchorLidarCoord = cv::Mat::zeros(4, 1, CV_64FC1);
        anchorLidarCoord.at<double>(2, 0) = lidar_range;
        anchorLidarCoord.at<double>(3, 0) = 1.0;

        cv::Mat anchorChessboardCoord = cv::Mat::zeros(4, 1, CV_64FC1);
        anchorChessboardCoord = rt_chessboard_to_lidar_ * anchorLidarCoord;

        cv::Mat anchorCameraCoord = cv::Mat::zeros(4, 1, CV_64FC1);
        anchorCameraCoord = RT_camera_to_chessboard * anchorChessboardCoord;

        camera_coordinate_point.x = anchorCameraCoord.at<double_t>(0, 0);
        camera_coordinate_point.y = anchorCameraCoord.at<double_t>(1, 0);
        camera_coordinate_point.z = anchorCameraCoord.at<double_t>(2, 0);

        err.err_str = "cal camera coordinate point done";
        err.ret = ERRTYPE::M_OK;
    }


    return err;
}


ErrInfo CameraCalibration::ProcessExternal(const std::vector< cv::Mat >& images, MODETYPE type){
    ErrInfo err;

    float lidar_range[] = {690.0842,
                           476.4111,
                           516.4476,
                           791.0750,
                           781.3857,
                           379.3407,
                           358.7429,
                           299.9722,
                           335.2562,
                           353.5429,
                           635.6429,
                           532.7409,
                           735.7357,
                           800.0391,
                           851.4045,
                           720.5111,
                           644.2417,
                           589.4091};

    for (size_t i = 0; i < images.size(); i++)
    {
        cv::Point3f pt;
        CalAnchors(images[i], MODETYPE::MODE_ANCHORS, lidar_range[i], pt);
    }


    return err;
}

cv::Point3f CameraCalibration::GetCamCoordinatePoint(){

    return this->camera_coordinate_point_;
}


ErrInfo CameraCalibration::GetLidarParamsFromFile(const std::string file){
    ErrInfo err;
    err = GetValueFromYaml(file, key_for_lidar_, cv::Size(4,4), rt_chessboard_to_lidar_);

    return err;
}

ErrInfo CameraCalibration::GetCameraInternalParams(const std::string file){
    ErrInfo err;

    std::map< std::string, cv::Mat > intrinsic_params_2;

    cv::Mat mat_k = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat mat_dist = cv::Mat::zeros(8, 1, CV_64F);

    err = GetValueFromYaml(file, key_for_cam_k_, cv::Size(3,3), mat_k);

    err = GetValueFromYaml(file, key_for_cam_dist_, cv::Size(5,1), mat_dist);

    intrinsic_params_.clear();
    intrinsic_params_[key_for_cam_k_] = mat_k;
    intrinsic_params_[key_for_cam_dist_] = mat_dist;

    return err;
}
