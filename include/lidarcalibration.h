#ifndef LIDARCALIBRATION_H
#define LIDARCALIBRATION_H

#include "define_type.h"

class LidarCalibration
{
public:
    LidarCalibration();

    ErrInfo CalLidarRange(double& lidar_range);

};

#endif // LIDARCALIBRATION_H
