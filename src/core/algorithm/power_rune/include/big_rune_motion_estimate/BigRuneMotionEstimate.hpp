#pragma once

#include "common/power_rune_global.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//候选目标
struct CandidateTarget
{
    //符心
    Eigen::Vector3d rune_center;

    //靶心
    Eigen::Vector3d armor_module_center;

    //相位(弧度)
    double phase;

    //起始向量(0弧度对应的单位向量)
    Eigen::Vector3d start_vector;

    //平面法向量(世界系)
    Eigen::Vector3d rune_plane_world_normal;

    //对应的原图时间
    foxglove::Time capture_timestamp;
};

//大符中的连续追踪目标
struct TrackedTarget : public CandidateTarget
{
    TrackedTarget() = delete;
    explicit TrackedTarget(const CandidateTarget &candidate);
    int switch_num = 0;//切换的符叶数
    double continuous_phase;//连续相位(-inf到inf)用于拟合运动方程
    double decentralized_timestamp;//去中心化的时间戳,用于拟合方程
};

//旋转方向
enum class RotationDirection
{
    clockwise,     // 顺时针
    anticlockwise, // 逆时针
    Unknown        // 不确定
};
