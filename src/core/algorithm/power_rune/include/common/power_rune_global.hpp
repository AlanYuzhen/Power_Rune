#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <optional>
#include "transform_tools/transform_tools.h"
#include "Time_generated.h"
#include "json/ReJson.hpp"

enum class RuneKind 
{
    SMALL_BUFF = 2,
    BIG_BUFF = 3
};


//符叶的基本信息
struct RuneInfo
{
    enum Color
    {
        RED = 0,
        BLUE = 1
    };
    cv::Point top, left, right, bottom , point_R;//网络识别点
    int class_id;//0是未击打，1是已击打
    Color color;//符的颜色
    cv::RotatedRect rotated_rect;//根据网络结果拟合的椭圆
    cv::Rect view_rect; //椭圆的外接矩形，也是视图在原图上的位置
    cv::Mat view;   //根据网络网络结果截取的视图
};

//符的观测
struct RuneObservation
{
    bool is_big_rune;//是否是大符
    std::vector<RuneInfo> rune_infos;//符叶的基本信息
    cv::Mat ori_img;//原图
    foxglove::Time timestamp;//原图对应的时间戳
    transform_tools::TFTree tf_tree;//tf_tree
};

//约束后的轮廓
struct ConstrainedContours
{
    std::optional<std::vector<cv::Point>> armor_module_opt;//装甲板模块
    std::optional<std::vector<cv::Point>> light_arm_opt;//灯臂
    std::optional<std::vector<cv::Point>> center_R_opt;//中心R标

};

//符的状态
enum class RuneState
{
    BigInactive,//大符未激活
    BigActivated,//大符已激活
    SmallInactive,//小符未激活
    SmallActivated//小符已激活
};

//单片符叶
struct SingleRuneBlade2D
{
    RuneInfo rune_info;//网络结果
    RuneState rune_state;//状态
    ConstrainedContours constrained_contours;//约束后的轮廓
    bool is_armor_module_usable = false;//装甲板模块轮廓是否可以用于投影优化
    bool is_light_arm_usable = false;//灯臂是否可以用于投影优化
    bool is_center_R_usable = false;//中心R标是否可以用于投影优化
};

//经过传统算法得到的符的信息
struct RefinedRuneObservation
{
    std::vector<SingleRuneBlade2D> rune_blade_2D;//单个符叶
    cv::Mat ori_img;//原图
    foxglove::Time timestamp;//原图对应的时间戳
    transform_tools::TFTree tf_tree;//tf_tree
    bool is_big_rune;//是否是大符
};

//未激活的目标
struct InactiveTargets
{
    struct RunePiece
    {
        Eigen::Vector3d rune_center;//符心
        Eigen::Vector3d armor_center;//靶心
    };
    
    std::vector<RunePiece> rune_pieces;//单片未激活符
    Eigen::Hyperplane<double,3> power_rune_plane;//所在的平面
    foxglove::Time capture_timestamp;//对应原图的拍摄时间
    bool is_big_rune;//是否是大符
    bool is_vaild = false;//是否有效
};

//经过滤波后的目标
struct RuneTarget
{
    Eigen::Vector3d rune_center;//符心
    Eigen::Vector3d armor_module_center;//靶心
    double phase;//相位
    Eigen::Vector3d start_vector;//起始向量(0弧度对应的单位向量)
    Eigen::Vector3d rune_plane_world_normal;//平面的法向量(世界系)
    bool is_big_rune;//是否是大符
    foxglove::Time capture_timestamp;//原图对应的时间戳
    int inactivate_target_num;//本帧的目标数量

    //大符运动方程的参数
    struct BigRuneMotionModelParams
    {
        // 根据规则：
        // spd = a * sin(ωt + δ) + b //或逆时针的时候 -spd = a * sin(ωt + δ) + b
        // a ∈ (0.78 , 1.045) 
        // ω ∈ (1.884 , 2.000)
        // b = 2.090 - a
        // δ ∈ (0 , 2π)

        // 积分可得
        // phase = -(a/ω) * cos(ωt + δ) + bt + C
        // phase = -(a/ω) * cos(ωt)cos(δ) + (a/ω) * sin(ωt)sin(δ) + bt + C
        // phase = Acos(ωt) + Bsin(ωt) + bt + C
        // A = -(a/ω) * cos(δ)
        // B = (a/ω) * sin(δ)
        // b = 2.090 - a
        // => ω * (A^2 + B^2)^(1/2) + b = 2.090
        // C ∈ R

        //参考时间戳(0点)
        foxglove::Time reference_timestamp;

        // 拟合主参数(直接拟合得到) phase = A*cos(ωt) + B*sin(ωt) + b*t + C
        double phase_cos_coefficient;         // A(相位变化余弦分量)
        double phase_sin_coefficient;         // B(相位变化正弦分量)
        double phase_linear_velocity;         // b(相位变化恒定分量)
        double phase_constant_offset_radians; // C(相位初始值)
        double speed_angular_frequency;       // ω(角速度振动频率)

        // 派生物理参数（由拟合主参数换算得到）：spd = a*sin(ωt + δ) + b
        double speed_amplitude;               // a = ω * sqrt(A^2 + B^2) 角速度振幅
        double speed_phase_shift;             // δ = atan2(B, -A) 角速度初相位
        
    };

    //大符的运动方程
    BigRuneMotionModelParams big_rune_motion_model;

    //小符运动方程(仅有角速度即可)
    double angular_velocity;
};


