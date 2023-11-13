#include "yamlhelp.h"
#include<iostream>

#include"opencv2/opencv.hpp"

inline bool ReadBool(std::string value){
    if(value=="true" || value=="True" || value=="TRUE")
        return true;
    else
        return false;
}

YamlHelp::YamlHelp()
{

}

//ErrInfo YamlHelp::GetYaml(const std::string& file, std::map<std::string, std::string>& key_vals)
//{
//    ErrInfo err;
//    ConfigInfo info;
//    err = GetYamlFromCV(file, info);
////    try {
////        YAML::Node node = YAML::LoadFile(file);

////        for(YAML::const_iterator it= node.begin(); it != node.end(); ++it)
////        {
////            auto key = it->first.as< std::string >();
////            auto value = it->second.as< std::string >();
////            std::cout << "key:" << key << ", value:" << value << std::endl;
////            key_vals[it->first.as< std::string >()] = it->second.as< std::string >();
////        }

////        err.err_str = "yaml read ok";
////        err.ret = ERRTYPE::M_OK;

////    } catch (...) { //文件为非yaml格式抛出异常
////        err.err_str = "please check params file format, yaml is ok";
////        err.ret = ERRTYPE::M_NO_KEY_WORD;
////        return err;
////    }

//    return err;
//}


ErrInfo YamlHelp::GetYamlFromCV(const std::string& file, ConfigInfo& info)
{
    ErrInfo err;

    cv::FileStorage fin(file, cv::FileStorage::READ);
    if(!fin.isOpened()){
        std::cout << "config.json open failed" << std::endl;
        err.err_str = "read yaml config file:" + file + " faild";
        err.ret = ERRTYPE::M_FILE_NOT_EXIST;
        return err;
    }

    fin["chess_board_size"] >> info.chess_board_size;
    fin["chess_board_rows"] >> info.chess_board_rows;
    fin["chess_board_cols"] >> info.chess_board_cols;
    fin["window_size"] >> info.window_size;
    fin["internal_calibrate_image_num"] >> info.internal_calibrate_image_num;
    fin["internal_calibrate_image_save_path"] >> info.internal_calibrate_image_save_path;
    fin["camera_internal_params_file"] >> info.camera_internal_params_file;
    fin["lidar_params_file"] >> info.lidar_params_file;
    fin["anchors_file"] >> info.anchors_file;
    fin["lidar_interval_time"] >> info.lidar_interval_time;
    fin["anchors_result_json_save_path"] >> info.anchors_result_json_save_path;
    fin["anchors_result_txt_save_path"] >> info.anchors_result_txt_save_path;
    std::string temp_str;
    fin["is_save_image_points"] >> temp_str;
    info.is_save_image_points = ReadBool(temp_str);
    fin["is_video_test"] >> temp_str;
    info.is_video_test =ReadBool(temp_str);
    fin["test_video_path"] >> info.test_video_path;
    fin["is_internal_params_test"] >> temp_str;
    info.is_internal_params_test = ReadBool(temp_str);
    fin["internal_calibrate_image_test_path"] >> info.internal_calibrate_image_test_path;

    fin.release();

    err.err_str = "read yaml config success";
    err.ret = ERRTYPE::M_OK;

    return err;
}
