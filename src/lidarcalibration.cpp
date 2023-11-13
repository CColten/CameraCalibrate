#include "lidarcalibration.h"

LidarCalibration::LidarCalibration()
{

}

/*
 * 发送信号，对lidar处理，获取结果
*/
ErrInfo LidarCalibration::CalLidarRange(double& lidar_range){
    ErrInfo err;

    lidar_range = 10.f;
    return err;
}
