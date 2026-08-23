#pragma once
#include<opencv2/opencv.hpp>
#include <vector>
#include "time/time.hpp"
#include "transform_tools/transform_tools.h"

namespace power_rune
{

//符的输入
struct RuneInput
{
    bool is_big_rune;// 是否是大符
    cv::Mat ori_mat;//原图
    transform_tools::TFTree tf_tree;//tf_tree
    timetool::Timestamp timestamp;//时间戳
    int cd_my_color;
    struct NNRuneInfo
    {
        cv::Point top, left, right, bottom, point_R;
        int class_id; // 0是未击打，1是已击打
    };
    std::vector<NNRuneInfo> nn_rune_infos;//符数据

};


//符的输出
struct RuneSendData
{
    float yaw;              // 需要转到的yaw
    float pitch;            // 需要转到的pitch
    uint8_t is_find_buff;   // 是否发现能量机关
    uint8_t mode;           // aim: 1   small_buuf: 2   big_buff: 3  outpost: 4   hero: 5
    uint8_t is_enable_fire; // 是否允许开火
};


void process_power_rune(const RuneInput& input);
RuneSendData get_rune_data(bool is_big_rune);

}
