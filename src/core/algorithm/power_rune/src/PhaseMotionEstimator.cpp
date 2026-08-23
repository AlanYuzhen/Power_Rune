#include "PhaseMotionEstimator.hpp"
#include "function.hpp"
#include "json.hpp"
#include "common/PowerRuneDiagnostics.hpp"
#include "common/PowerRuneVisualizeManager.hpp"
#include "common/power_rune_function.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

PhaseMotionEstimator::PhaseMotionEstimator()
{
}

void PhaseMotionEstimator::estimate_phase_motion(const InactiveTargets &inactive_targets)
{
    //标志位赋值
    m_is_big_rune = inactive_targets.is_big_rune;

    //判断是否是空的或者出现误识别
    if (inactive_targets.rune_pieces.empty())
    {
        return;
    }
    if (m_is_big_rune && inactive_targets.rune_pieces.size() > 2)
    {
        return;
    }
    if (!m_is_big_rune && inactive_targets.rune_pieces.size() > 1)
    {
        return;
    }

    //清空原本的候选目标
    m_candidate_targets.clear();

    //生成候选目标
    generate_candidate_targets(inactive_targets);

    if (VizTopic::RuneOriPhase::enabled())
    {
        //foxglove可视化
        if (m_candidate_targets.size() == 1)
        {
            Viz::log_with_time<VizTopic::RuneOriPhase>(
                function::to_nanoseconds_since_epoch(inactive_targets.capture_timestamp),
                m_candidate_targets[0].phase,
                m_candidate_targets[0].phase);
        }
        else if (m_candidate_targets.size() == 2)
        {
            Viz::log_with_time<VizTopic::RuneOriPhase>(
                function::to_nanoseconds_since_epoch(inactive_targets.capture_timestamp),
                m_candidate_targets[0].phase,
                m_candidate_targets[1].phase);
        }
    }

    //准备输出队列
    m_rune_targets.clear();

    if (m_is_big_rune)
    {
        generate_big_power_rune_targets(inactive_targets.rune_pieces.size());
    }
    else
    {
        generate_small_power_rune_targets();
    }
}

std::optional<std::vector<RuneTarget>> PhaseMotionEstimator::try_get_rune_targets()
{
    if (m_rune_targets.empty())
    {
        return std::optional<std::vector<RuneTarget>>();
    }
    else
    {
        return m_rune_targets;
    }
}

void PhaseMotionEstimator::generate_candidate_targets(const InactiveTargets &inactive_targets)
{
    for (const auto &rune_piece : inactive_targets.rune_pieces)
    {
        CandidateTarget candidate_target;
        candidate_target.rune_center = rune_piece.rune_center;
        candidate_target.armor_module_center = rune_piece.armor_center;
        candidate_target.capture_timestamp = inactive_targets.capture_timestamp;
        complete_candidate_target(candidate_target, inactive_targets.power_rune_plane);

        m_candidate_targets.emplace_back(std::move(candidate_target));
    }
}

void PhaseMotionEstimator::complete_candidate_target(CandidateTarget &candidate_target,const Eigen::Hyperplane<double, 3> &rune_plane_car)
{
    Eigen::Vector3d rune_plane_world_normal = rune_plane_car.normal();
    rune_plane_world_normal.normalize();

    if (rune_plane_world_normal.dot(candidate_target.armor_module_center) < 0.0)
    {
        rune_plane_world_normal = -rune_plane_world_normal;
    }

    Eigen::Vector3d up(0, -1, 0);
    Eigen::Vector3d start_vector = rune_plane_world_normal.cross(up);
    start_vector.normalize();

    Eigen::Vector3d rune_center2armor_center_normal = candidate_target.armor_module_center - candidate_target.rune_center;
    rune_center2armor_center_normal.normalize();

    double sin_theta = rune_plane_world_normal.dot(start_vector.cross(rune_center2armor_center_normal));
    double cos_theta = start_vector.dot(rune_center2armor_center_normal);
    double phase = atan2(sin_theta, cos_theta);

    candidate_target.rune_plane_world_normal = rune_plane_world_normal;
    candidate_target.start_vector = start_vector;
    candidate_target.phase = phase;

    if (VizTopic::RunePlaneVector::enabled())
    {
        auto dirtion_vector2quaternion = [&](const Eigen::Vector3d &dir) -> foxglove::schemas::Quaternion
        {
            Eigen::Vector3d from(1.0, 0.0, 0.0);   // Foxglove 默认箭头方向 +X
            Eigen::Vector3d to = dir.normalized(); // 目标方向

            Eigen::Quaterniond q;
            if (to.dot(from) < -0.999999)
            {
                // 反向（180度）特判，任选一个不共线轴
                q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0, 1.0, 0.0));
            }
            else
            {
                q.setFromTwoVectors(from, to);
            }

            foxglove::schemas::Quaternion out;
            out.x = q.x();
            out.y = q.y();
            out.z = q.z();
            out.w = q.w();
            return out;
        };

        std::array<foxglove::schemas::ArrowPrimitive, 2> fox_rune_plane_vectors;

        for (int i = 0; i < 2; ++i)
        {
            fox_rune_plane_vectors[i].pose = foxglove::schemas::Pose{};
            fox_rune_plane_vectors[i].pose->position = foxglove::schemas::Vector3{};
            fox_rune_plane_vectors[i].pose->orientation = foxglove::schemas::Quaternion{};
        }

        fox_rune_plane_vectors[0].pose->position->x = candidate_target.rune_center.z();
        fox_rune_plane_vectors[0].pose->position->y = -candidate_target.rune_center.x();
        fox_rune_plane_vectors[0].pose->position->z = -candidate_target.rune_center.y();
        fox_rune_plane_vectors[0].pose->orientation =
            dirtion_vector2quaternion(Eigen::Vector3d(
                rune_plane_world_normal.z(),
                -rune_plane_world_normal.x(),
                -rune_plane_world_normal.y()));
        fox_rune_plane_vectors[0].shaft_length = 0.25;
        fox_rune_plane_vectors[0].shaft_diameter = 0.01;
        fox_rune_plane_vectors[0].head_length = 0.08;
        fox_rune_plane_vectors[0].head_diameter = 0.03;
        fox_rune_plane_vectors[0].color = foxglove::schemas::Color{1.0f, 0.5f, 0.0f, 1.0f};

        fox_rune_plane_vectors[1].pose->position->x = candidate_target.rune_center.z();
        fox_rune_plane_vectors[1].pose->position->y = -candidate_target.rune_center.x();
        fox_rune_plane_vectors[1].pose->position->z = -candidate_target.rune_center.y();
        fox_rune_plane_vectors[1].pose->orientation =
            dirtion_vector2quaternion(Eigen::Vector3d(
                start_vector.z(),
                -start_vector.x(),
                -start_vector.y()));
        fox_rune_plane_vectors[1].shaft_length = 0.25;
        fox_rune_plane_vectors[1].shaft_diameter = 0.01;
        fox_rune_plane_vectors[1].head_length = 0.08;
        fox_rune_plane_vectors[1].head_diameter = 0.03;
        fox_rune_plane_vectors[1].color = foxglove::schemas::Color{1.0f, 1.0f, 0.0f, 1.0f};
        Viz::publish_arrows<VizTopic::RunePlaneVector>(fox_rune_plane_vectors);
    }
}

void PhaseMotionEstimator::generate_big_power_rune_targets(int inactivate_target_num)
{
    if (!m_big_rune_phase_motion_filter.calculate_motion(std::move(m_candidate_targets)))
    {
        return;
    }

    CandidateTarget tracked_target = m_big_rune_phase_motion_filter.get_target();
    RuneTarget::BigRuneMotionModelParams motion_model = m_big_rune_phase_motion_filter.get_motion();

    RuneTarget rune_target;
    rune_target.inactivate_target_num = inactivate_target_num;
    rune_target.is_big_rune = true;
    rune_target.rune_plane_world_normal = tracked_target.rune_plane_world_normal;
    rune_target.armor_module_center = tracked_target.armor_module_center;
    rune_target.rune_center = tracked_target.rune_center;
    rune_target.capture_timestamp = tracked_target.capture_timestamp;
    rune_target.phase = tracked_target.phase;
    rune_target.start_vector = tracked_target.start_vector;
    rune_target.big_rune_motion_model = motion_model;
    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PREDICT_ERROR"])
    {
        PowerRuneDiagnostics::instance().push_back_real_phase(rune_target.capture_timestamp, rune_target.phase);
    }
    m_rune_targets.emplace_back(std::move(rune_target));
}

void PhaseMotionEstimator::generate_small_power_rune_targets()
{
    std::optional<CandidateTarget> filtered_target_opt =
        m_small_rune_kalman_filter.get_filtered_target(std::move(m_candidate_targets[0]));
    if (!filtered_target_opt.has_value())
    {
        return;
    }

    if (m_rotation_direction != m_small_rune_kalman_filter.get_rotation_direction())
    {
        m_rotation_direction = m_small_rune_kalman_filter.get_rotation_direction();
        //m_big_rune_phase_motion_filter.set_rotation_direction(m_rotation_direction);
    }

    const auto &filtered_target = filtered_target_opt.value();

    if (VizTopic::RuneFilteredPhase::enabled())
    {
        Viz::log_with_time<VizTopic::RuneFilteredPhase>(
            function::to_nanoseconds_since_epoch(filtered_target.capture_timestamp),
            filtered_target.phase,
            filtered_target.phase);
    }

    RuneTarget rune_target;
    rune_target.is_big_rune = false;
    rune_target.angular_velocity = m_small_rune_kalman_filter.get_angular_velocity();
    rune_target.rune_plane_world_normal = filtered_target.rune_plane_world_normal;
    rune_target.armor_module_center = filtered_target.armor_module_center;
    rune_target.rune_center = filtered_target.rune_center;
    rune_target.capture_timestamp = filtered_target.capture_timestamp;
    rune_target.phase = filtered_target.phase;
    rune_target.start_vector = filtered_target.start_vector;
    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PREDICT_ERROR"])
    {
        PowerRuneDiagnostics::instance().push_back_real_phase(rune_target.capture_timestamp, rune_target.phase);
    }
    m_rune_targets.emplace_back(std::move(rune_target));
}

std::optional<CandidateTarget> SmallRuneKalmanFilter::get_filtered_target(CandidateTarget &&candidate_target)
{
    update_candidate_targets(std::move(candidate_target));

    if (m_rotation_direction == RotationDirection::Unknown)
    {
        LOG(WARNING) << "[SmallRuneKalmanFilter]旋转方向未确定" << m_candidate_targets.size();
        m_life = 0;
        return std::optional<CandidateTarget>();
    }

    if (m_candidate_targets.size() > (int)J_POWER_RUNE.config_["small_phase_estimate"]["min_size_to_filter"])
    {
        LOG(ERROR) << "[get_filtered_target]候选目标数目异常";
        throw std::runtime_error("[get_filtered_target]候选目标数目异常");
    }

    if (m_candidate_targets.size() < (int)J_POWER_RUNE.config_["small_phase_estimate"]["min_size_to_filter"])
    {
        m_life = 0;
        return std::optional<CandidateTarget>();
    }

    if (m_life == 0)
    {
        for (int i = 0; i < m_candidate_targets.size() - 1; i++)
        {
            if (i == 0)
            {
                reset(m_candidate_targets.front().phase);
            }

            double dt = function::timestampMinus(
                            m_candidate_targets[i + 1].capture_timestamp,
                            m_candidate_targets[i].capture_timestamp) *
                        0.001;
            double observed_phase = m_candidate_targets[i + 1].phase;
            predict(dt);
            update(observed_phase);
        }

        m_life = m_candidate_targets.size() - 1;

        CandidateTarget filtered_target = m_candidate_targets.back();
        filtered_target.phase = PRF::normalize_phase<PRF::rad>(m_posteriori_phase);
        return filtered_target;
    }

    double dt = function::timestampMinus(
                    m_candidate_targets.back().capture_timestamp,
                    m_candidate_targets[m_candidate_targets.size() - 2].capture_timestamp) *
                0.001;
    double observed_phase = m_candidate_targets.back().phase;
    predict(dt);
    update(observed_phase);
    m_life++;

    CandidateTarget filtered_target = m_candidate_targets.back();
    filtered_target.phase = PRF::normalize_phase<PRF::rad>(m_posteriori_phase);
    return filtered_target;
}

const double SmallRuneKalmanFilter::get_angular_velocity() const
{
    return m_angular_velocity;
}

const RotationDirection SmallRuneKalmanFilter::get_rotation_direction() const
{
    return m_rotation_direction;
}

void SmallRuneKalmanFilter::reinit()
{
    m_rotation_direction = RotationDirection::Unknown;
    m_candidate_targets.clear();
    m_life = 0;
}

void SmallRuneKalmanFilter::predict(double dt)
{
    m_priori_phase = m_posteriori_phase + m_angular_velocity * dt;
    m_P += m_Q * dt * dt;
}

void SmallRuneKalmanFilter::update(double observed_phase)
{
    double innovation = PRF::calculate_delta_phase<PRF::rad>(observed_phase, m_priori_phase);
    double S = m_P + m_R;
    double K = m_P / S;

    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_BULLISTIC_DEBUG"])
    {
        K = 1.0;
    }

    m_posteriori_phase = m_priori_phase + K * innovation;
    m_P = (1.0 - K) * m_P;
}

void SmallRuneKalmanFilter::reset(double phase)
{
    m_posteriori_phase = phase;
    m_P = (double)J_POWER_RUNE.config_["small_phase_estimate"]["P_init"];
    m_Q = (double)J_POWER_RUNE.config_["small_phase_estimate"]["Q_base"];
    m_R = J_POWER_RUNE.config_["small_phase_estimate"]["R_base"];

    if (m_rotation_direction == RotationDirection::clockwise)
    {
        m_angular_velocity = (double)J_POWER_RUNE.config_["small_phase_estimate"]["expect_angular_velocity"];
    }
    else if (m_rotation_direction == RotationDirection::anticlockwise)
    {
        m_angular_velocity = -(double)J_POWER_RUNE.config_["small_phase_estimate"]["expect_angular_velocity"];
    }
    else
    {
        LOG(ERROR) << "[reset]未初始化的旋转方向";
        throw std::runtime_error("[reset]未初始化的旋转方向");
    }
}

void SmallRuneKalmanFilter::update_candidate_targets(CandidateTarget &&candidate_target)
{
    if (m_candidate_targets.empty())
    {
        m_candidate_targets.emplace_back(std::move(candidate_target));
        return;
    }

    if (function::timestampMinus(candidate_target.capture_timestamp, m_candidate_targets.back().capture_timestamp) * 0.001 >
        (double)J_POWER_RUNE.config_["small_phase_estimate"]["prepare_time"])
    {
        reinit();
        m_candidate_targets.clear();
        m_candidate_targets.emplace_back(std::move(candidate_target));
        LOG(WARNING) << "[update_candidate_targets]超时";
        return;
    }

    if (function::timestampMinus(candidate_target.capture_timestamp, m_candidate_targets.back().capture_timestamp) * 0.001 >
        (double)J_POWER_RUNE.config_["small_phase_estimate"]["max_data_interval"])
    {
        LOG(WARNING) << "[update_candidate_targets]超时"
                     << function::timestampMinus(candidate_target.capture_timestamp, m_candidate_targets.back().capture_timestamp) *
                            0.001;
        m_candidate_targets.clear();
        m_candidate_targets.emplace_back(std::move(candidate_target));
    }
    else
    {
        m_candidate_targets.emplace_back(std::move(candidate_target));
    }

    if (m_rotation_direction == RotationDirection::Unknown)
    {
        int size = m_candidate_targets.size();
        if (size < (int)J_POWER_RUNE.config_["small_phase_estimate"]["min_size_to_confirm_rotation"])
        {
            return;
        }

        int clock_wise_num = 0;
        int anti_clock_wise_num = 0;
        for (int i = 0; i < size - 1; i++)
        {
            double delta_phase = PRF::calculate_delta_phase<PRF::rad>(m_candidate_targets[i + 1].phase, m_candidate_targets[i].phase);
            if (fabs(delta_phase) > (double)J_POWER_RUNE.config_["small_phase_estimate"]["target_switch_threshold"])
            {
                continue;
            }
            if (delta_phase > 0)
            {
                clock_wise_num++;
            }
            else if (delta_phase < 0)
            {
                anti_clock_wise_num++;
            }
        }

        if (clock_wise_num == anti_clock_wise_num)
        {
            return;
        }
        else if (clock_wise_num > anti_clock_wise_num)
        {
            m_rotation_direction = RotationDirection::clockwise;
            LOG(INFO) << "\033[32m[update_candidate_targets]顺时针\033[0m";
        }
        else
        {
            m_rotation_direction = RotationDirection::anticlockwise;
            LOG(INFO) << "\033[32m[update_candidate_targets]逆时针\033[0m";
        }
    }

    if (m_candidate_targets.size() < (int)J_POWER_RUNE.config_["small_phase_estimate"]["min_size_to_filter"])
    {
        return;
    }

    while (m_candidate_targets.size() > (int)J_POWER_RUNE.config_["small_phase_estimate"]["min_size_to_filter"])
    {
        m_candidate_targets.pop_front();
    }

    if (!((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
          (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_BULLISTIC_DEBUG"]))
    {
        for (int i = 0; i < (int)J_POWER_RUNE.config_["small_phase_estimate"]["min_size_to_filter"] - 1; i++)
        {
            double delta_phase = PRF::calculate_delta_phase<PRF::rad>(m_candidate_targets[i + 1].phase, m_candidate_targets[i].phase);

            double max_delta_phase =
                (double)J_POWER_RUNE.config_["small_phase_estimate"]["max_data_interval"] *
                (double)J_POWER_RUNE.config_["small_phase_estimate"]["expect_angular_velocity"] *
                (double)J_POWER_RUNE.config_["small_phase_estimate"]["angular_tolerance_factor"];
            if (fabs(delta_phase) > max_delta_phase)
            {
                m_candidate_targets.erase(m_candidate_targets.begin(), m_candidate_targets.begin() + i + 1);
                LOG(WARNING) << "[update_candidate_targets]观测大幅抖动";
                return;
            }
        }
    }
}
