#pragma once
#include "big_rune_motion_estimate/BigRuneMotionEstimate.hpp"
#include "big_rune_motion_estimate/LM_IRLS_BigRunePhaseMotionFilter.hpp"

#include "common/power_rune_global.hpp"
#include <deque>
#include <optional>
#include <vector>

//小符的卡尔曼滤波器(运动观测器)
class SmallRuneKalmanFilter
{
private:
    

public:
    SmallRuneKalmanFilter() = default;
    std::optional<CandidateTarget> get_filtered_target(CandidateTarget &&candidate_target);
    const double get_angular_velocity() const;
    const RotationDirection get_rotation_direction() const;
    void reinit();//清空队列，重置旋转方向
private:
    void predict(double dt); //计算先验
    void update(double observed_phase);//得到后验
    void reset(double phase);//重置卡尔曼
    void update_candidate_targets(CandidateTarget &&candidate_target);//更新m_candidate_targets，同时有判断转动方向和滤波的作用


    double m_priori_phase;//本次的先验(由上次的后验预测得到)
    double m_posteriori_phase;//本次的后验(由本次的先验更新得到，可用于计算下一次的先验)
    double m_angular_velocity;//小符的角速度(弧度制)
    double m_P;//协方差
    double m_Q;//过程噪声
    double m_R;//观测噪声

    RotationDirection m_rotation_direction = RotationDirection::Unknown;//旋转方向
    int m_life;//迭代次数
    std::deque<CandidateTarget> m_candidate_targets;//候选目标队列
};

//相位估计器
class PhaseMotionEstimator
{
public:
    PhaseMotionEstimator();
    void estimate_phase_motion(const InactiveTargets &inactive_targets);//更新当前识别的目标的位置，拟合运动方程
    std::optional<std::vector<RuneTarget>> try_get_rune_targets();

private:
    RotationDirection m_rotation_direction = RotationDirection::Unknown;//旋转方向
    bool m_is_big_rune;//大符标志位
    SmallRuneKalmanFilter m_small_rune_kalman_filter;//小符卡尔曼滤波器
    LM_IRLS_BigRunePhaseMotionFilter m_big_rune_phase_motion_filter;//大符的相位运动滤波器

    void generate_candidate_targets(const InactiveTargets &inactive_targets);//生成候选目标
    void complete_candidate_target(CandidateTarget& candidate_target, const Eigen::Hyperplane<double,3> &rune_plane_car);//完善候选目标的起始相位和起始向量
    std::vector<CandidateTarget> m_candidate_targets;//候选目标

    void generate_big_power_rune_targets(int inactivate_target_num);//生成大符目标
    void generate_small_power_rune_targets();//生成小符目标
    std::vector<RuneTarget> m_rune_targets;//目标//TODO：由于追踪放在了这里，所以这个vector可以去除，改为RuneTarget就可以了

};
