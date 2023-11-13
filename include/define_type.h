#ifndef DEFINE_TYPE_H
#define DEFINE_TYPE_H

#pragma once
#include <string>


enum class MODETYPE: int{
    MODE_INVALID = 0,
    MODE_INTERNAL,      // 内参模式
    MODE_ANCHORS,       // anchors模式
    MODE_AIM,           // 瞄准模式
};

enum class ERRTYPE: int{
    M_OK = 0,  // success
    M_CAMERA_DEVIATION_TOO_BIG,
    M_CAMERA_INTRINSIC_RESULT_VALID,
    M_BOARD_CORNOR_MATCH_FAILD,
    M_LIDAR_CALCULATE_FAILD,
    M_NO_KEY_WORD,
    M_KEY_WORD_SIZE_NOT_MATCHED,
    M_CALIBRATION_DONE,
    M_FILE_NOT_EXIST
};

struct ErrInfo{
    ERRTYPE ret;
    std::string err_str;
};

struct ConfigInfo{
    float chess_board_size;
    int chess_board_rows;
    int chess_board_cols;

//    # 内参标定图片数量
    int internal_calibrate_image_num;

    int window_size;

//    # 标定图片保存路径
    std::string internal_calibrate_image_save_path;

//    # 相机内参文件
    std::string camera_internal_params_file;

//    # lidar测量参数文件
    std::string  lidar_params_file;

//    # anchors文件
    std::string anchors_file;

//    # 瞄准模式下定时测试时间间隔(ms)
    int lidar_interval_time;

//    # anchors标定结果json保存文件路径
    std::string anchors_result_json_save_path;

//    #anchors标定结果txt保存文件路径
    std::string anchors_result_txt_save_path;

    bool is_save_image_points = false;

//    # 非直连车机，使用视频测试
    bool is_video_test = false;
    std::string test_video_path;

//    # 相机内参标定测试，使用本地的图片
    bool is_internal_params_test = false;

    std::string internal_calibrate_image_test_path;
};


#endif // DEFINE_TYPE_H
