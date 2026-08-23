#pragma once
#include <opencv2/opencv.hpp>
#include "json.hpp"

inline cv::Matx<double, 3, 3> CAM;
inline cv::Matx<double, 1, 5> DIS;

inline auto initCamAndDis = []() 
{
    J_POWER_RUNE.config_["camera"]["cam"] >> CAM;
    J_POWER_RUNE.config_["camera"]["dis"] >> DIS;
    return 0;
};

inline auto INIT_CAM_AND_DIS = initCamAndDis();