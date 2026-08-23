#include "big_rune_motion_estimate/BigRunePhaseMotionFilter.hpp"

#include "function.hpp"
#include "json.hpp"
#include "common/PowerRuneDiagnostics.hpp"
#include "common/PowerRuneVisualizeManager.hpp"
#include "common/power_rune_function.hpp"
#include <cstdint>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

TrackedTarget::TrackedTarget(const CandidateTarget &candidate)
    : CandidateTarget(candidate)
{
    continuous_phase = phase > 0 ? phase : phase + CV_2PI;
}

BigRunePhaseMotionFilter::BigRunePhaseMotionFilter()
{
}

bool BigRunePhaseMotionFilter::calculate_motion(std::vector<CandidateTarget> &&candidate_targets)
{
    //1.获取相位数据
    //2.判断运动方程是否已经建立
        //没有建立：
            //将数据加入队列
            //判断队列数目是否足以拟合方程
            //不允许则继续
            //允许则对数据进行分离，然后进行数据连续化
            //拟合方程
        //已经建立：
            //判断是单个目标还是两个目标
                //单个目标：
                    //对数据进行连续化处理，迭代方程
                //两个目标：
                    //进行预测后分别与两个目标的相位进行对比
                        //如果有在预测连续范围内的目标，则选择该目标，然后进行数据连续化
                        //如果没有则选择跳变较小的目标进行数据连续化,然后对数据进行连续化
                    //迭代运动方程

    //更新原始数据队列
    update_ori_candidate_targets(std::move(candidate_targets));

    if (!m_is_motion_model_vaild)
    {
        return try_build_motion_model();
    }

    try_update_motion_model();
    return m_is_motion_model_vaild;
}

const CandidateTarget BigRunePhaseMotionFilter::get_target() const
{
    return m_tracked_target_deque.back();
}

const RuneTarget::BigRuneMotionModelParams BigRunePhaseMotionFilter::get_motion() const
{
    return m_big_rune_motion_model;
}

void BigRunePhaseMotionFilter::set_rotation_direction(RotationDirection rotation_direction)
{
    if (m_rotation_direction == RotationDirection::Unknown && rotation_direction != RotationDirection::Unknown)
    {
        m_rotation_direction = rotation_direction;
    }
}

void BigRunePhaseMotionFilter::update_ori_candidate_targets(std::vector<CandidateTarget> &&ori_candidate_targets)
{

    if (m_ori_candidate_targets_deque.empty())
    {
        //没有数据，直接放入后返回
        m_ori_candidate_targets_deque.emplace_back(std::move(ori_candidate_targets));
        return;
    }

    // 检查与上一帧的时间间隔是否过大,
    double time_interval = function::timestampMinus(ori_candidate_targets[0].capture_timestamp,
                                                    m_ori_candidate_targets_deque.back()[0].capture_timestamp) *0.001;
    if (time_interval > (double)J_POWER_RUNE.config_["big_phase_motion_estimate"]["prepare_time"])
    {
        // 说明运动方程不可信,在这种情况下会出现轨迹被污染的情况
        m_rotation_direction = RotationDirection::Unknown;
        m_is_motion_model_vaild = false;
        m_ori_candidate_targets_deque.clear();
        m_tracked_target_deque.clear();
        m_jump_deque.clear();
        m_ori_candidate_targets_deque.emplace_back(std::move(ori_candidate_targets));//放入新数据
        LOG(WARNING)<<"[update_ori_candidate_targets重置大符旋转方向";
        return;
    }
    if (time_interval > (double)J_POWER_RUNE.config_["big_phase_motion_estimate"]["max_time_interval"])
    {
        //说明运动方程不可信,在这种情况下会出现轨迹被污染的情况
        m_is_motion_model_vaild = false;
        m_ori_candidate_targets_deque.clear();
        m_tracked_target_deque.clear();
        m_jump_deque.clear();
        m_ori_candidate_targets_deque.emplace_back(std::move(ori_candidate_targets));//放入新数据
        return;
    }
    

    //放入新数据
    m_ori_candidate_targets_deque.emplace_back(std::move(ori_candidate_targets));

    //删除不在窗口内的数据
    const double window_settling_time = J_POWER_RUNE.config_["big_phase_motion_estimate"]["window_settling_time"];
    while (!m_ori_candidate_targets_deque.empty())
    {
        double max_window_time =
            function::timestampMinus(
                m_ori_candidate_targets_deque.back()[0].capture_timestamp,
                m_ori_candidate_targets_deque.front()[0].capture_timestamp) /
            1000.0;

        if (max_window_time > window_settling_time)
        {
            m_ori_candidate_targets_deque.pop_front();
        }
        else
        {
            break;
        }
    }
}

bool BigRunePhaseMotionFilter::try_build_motion_model()
{
    //如果数据量未达标,那么不能开始拟合
    if (m_ori_candidate_targets_deque.size() < (int)J_POWER_RUNE.config_["big_phase_motion_estimate"]["min_data_size_to_build_motion_model"])
    {
        //std::cout<<"m_ori_candidate_targets_deque"<<m_ori_candidate_targets_deque.size()<<std::endl;
        //std::cout<<"m_tracked_target_deque"<<m_tracked_target_deque.size()<<std::endl;
        return false;
    }

    //运行到此处说明数据量达标,如果旋转方向未知，需要先确定旋转方向
    if (m_rotation_direction == RotationDirection::Unknown)
    {
        if (!confirm_rotation())
        {
            //无法确定旋转
            return false;
        }
    }

    //进行数据连续化
    init_tracked_target_deque();
    //进行拟合
    fit_motion_model();
    return m_is_motion_model_vaild;
}

bool BigRunePhaseMotionFilter::try_update_motion_model()
{
    //如果数据量未达标,那么不能开始拟合
    if (m_ori_candidate_targets_deque.size() <(int)J_POWER_RUNE.config_["big_phase_motion_estimate"]["min_data_size_to_build_motion_model"])
    {
        return false;
    }

    //旋转方向必然是确定的,应此不需要判断
    //进行数据连续化
    update_tracked_target_deque();

    //说明方程迭代出现了错误，需要重置
    if (need_reset_motion_for_abnormal_jump())
    {
        //不需要清空m_ori_candidate_targets_deque因为可以认为原始数据没有出现异常
        m_is_motion_model_vaild = false;
        m_jump_deque.clear();
        m_tracked_target_deque.clear();
        //m_ori_candidate_targets_deque.clear();
        LOG(ERROR) << "[BigRunePhaseMotionFilter]高频地判定为目标跳变";
        return false;
    }
    

    //进行拟合
    fit_motion_model();
    return m_is_motion_model_vaild;
}

bool BigRunePhaseMotionFilter::confirm_rotation()
{
    //判断趋势
    int clock_wise_num = 0;
    int anti_clock_wise_num = 0;
    for (int i = 1; i < m_ori_candidate_targets_deque.size(); i++)
    {
        double time_interval = function::timestampMinus(
                                   m_ori_candidate_targets_deque[i][0].capture_timestamp,
                                   m_ori_candidate_targets_deque[i - 1][0].capture_timestamp) *
                               0.001;
        if (time_interval >
            (double)J_POWER_RUNE.config_["big_phase_motion_estimate"]["acceptable_time_interval_to_confirm_rotation"])
        {
            //间隔太大的数据没有判断价值
            continue;
        }

        //投票决定顺逆时针
        auto vote = [&](double delta_phase) -> void
        {
            if (fabs(delta_phase) > (double)J_POWER_RUNE.config_["small_phase_estimate"]["target_switch_threshold"])
            {
                return;
            }

            if (delta_phase > 0)
            {
                clock_wise_num++;
            }
            else if (delta_phase < 0)
            {
                anti_clock_wise_num++;
            }
        };

        if (m_ori_candidate_targets_deque[i].size() == 2 && m_ori_candidate_targets_deque[i - 1].size() == 2)
        {
            //2-2
            // 两种合法配对：
            // 1. cur[0] <- last[0], cur[1] <- last[1]
            // 2. cur[0] <- last[1], cur[1] <- last[0]
            // 选择总相位跳变绝对值更小的一组，认为它更符合连续运动
            double delta_phase_00 = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][0].phase,
                m_ori_candidate_targets_deque[i - 1][0].phase);
            double delta_phase_11 = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][1].phase,
                m_ori_candidate_targets_deque[i - 1][1].phase);
            double cost_01 = std::fabs(delta_phase_00) + std::fabs(delta_phase_11);

            double delta_phase_01 = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][0].phase,
                m_ori_candidate_targets_deque[i - 1][1].phase);
            double delta_phase_10 = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][1].phase,
                m_ori_candidate_targets_deque[i - 1][0].phase);
            double cost_10 = std::fabs(delta_phase_01) + std::fabs(delta_phase_10);

            if (cost_01 <= cost_10)
            {
                vote(delta_phase_00);
                vote(delta_phase_11);
            }
            else
            {
                vote(delta_phase_01);
                vote(delta_phase_10);
            }
        }
        else if (m_ori_candidate_targets_deque[i].size() == 2 && m_ori_candidate_targets_deque[i - 1].size() == 1)
        {
            //2-1
            //选择角度较小的
            double delta_phase_1 = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][0].phase,
                m_ori_candidate_targets_deque[i - 1][0].phase);
            double delta_phase_2 = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][1].phase,
                m_ori_candidate_targets_deque[i - 1][0].phase);
            double delta_phase = std::fabs(delta_phase_1) < std::fabs(delta_phase_2) ? delta_phase_1 : delta_phase_2;
            vote(delta_phase);
        }
        else if (m_ori_candidate_targets_deque[i].size() == 1 && m_ori_candidate_targets_deque[i - 1].size() == 2)
        {
            //1-2
            //选择角度较小的
            double delta_phase_1 = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][0].phase,
                m_ori_candidate_targets_deque[i - 1][0].phase);
            double delta_phase_2 = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][0].phase,
                m_ori_candidate_targets_deque[i - 1][1].phase);
            double delta_phase = std::fabs(delta_phase_1) < std::fabs(delta_phase_2) ? delta_phase_1 : delta_phase_2;
            vote(delta_phase);
        }
        else
        {
            //1-1
            double delta_phase = PRF::calculate_delta_phase<PRF::rad>(
                m_ori_candidate_targets_deque[i][0].phase,
                m_ori_candidate_targets_deque[i - 1][0].phase);
            vote(delta_phase);
        }
    }

    if (clock_wise_num > anti_clock_wise_num)
    {
        LOG(INFO) << "\033[32m[BigRunePhaseMotionFilter]顺时针\033[0m";
        m_rotation_direction = RotationDirection::clockwise;
    }
    else if (anti_clock_wise_num > clock_wise_num)
    {
        LOG(INFO) << "\033[32m[BigRunePhaseMotionFilter]逆时针\033[0m";
        m_rotation_direction = RotationDirection::anticlockwise;
    }
    else
    {
        LOG(WARNING) << "[BigRunePhaseMotionFilter]无法确认旋转方向";
        return false;
    }

    return true;
}

void BigRunePhaseMotionFilter::init_tracked_target_deque()
{
    // 对数据进行连续化
        //从最老的数据中随机选择一个数据作为起始观测
        //下一帧数据是否有任意一个数据在预测允许的期望内
                //有：说明是连续追踪。将相位连续化
                //没有：说明发生了相位切换。需要判断切换的数目然后对旧的数据作修正

    //清空旧数据
    m_tracked_target_deque.clear();

    //随机选择最老的数据作为起始追踪(默认选必然存在的第一个元素)
    TrackedTarget root_tracked_target(m_ori_candidate_targets_deque[0][0]);
    m_tracked_target_deque.emplace_back(std::move(root_tracked_target));

    //选择追踪目标并进行数据连续化
    for (int i = 1; i < m_ori_candidate_targets_deque.size(); i++)
    {

        //选择目标
        int track_index = choose_tracked_target(m_ori_candidate_targets_deque[i]);
        const bool is_jump = is_phase_jump(m_ori_candidate_targets_deque[i][track_index]);
        if (is_jump)
        {
            //处理跳变情况
            resolve_track_after_phase_jump(m_ori_candidate_targets_deque[i][track_index]);
        }
        else
        {
            // 正常进行角度连续化然后放入追踪队列
            TrackedTarget track_target(m_ori_candidate_targets_deque[i][track_index]);
            track_target.continuous_phase = accumulate_phase(m_tracked_target_deque.back().continuous_phase, track_target.phase);
            m_tracked_target_deque.emplace_back(std::move(track_target));
        }
    }


    
    


    if (VizTopic::RuneTrackPhase::enabled())
    {
    
    //可视化连续化相位
    if (!m_tracked_target_deque.empty())
    {
        static bool has_visualized_init_tracked_target_deque = false;
        static uint64_t last_visualized_latest_timestamp_ns = 0;
        const uint64_t oldest_timestamp_ns = function::to_nanoseconds_since_epoch(m_tracked_target_deque.front().capture_timestamp);
        const uint64_t latest_timestamp_ns = function::to_nanoseconds_since_epoch(m_tracked_target_deque.back().capture_timestamp);

        if (!has_visualized_init_tracked_target_deque || oldest_timestamp_ns > last_visualized_latest_timestamp_ns)
        {
            const double aligned_latest_phase = PRF::normalize_phase<PRF::rad>(m_tracked_target_deque.back().phase);
            const double phase_compensate = aligned_latest_phase - m_tracked_target_deque.back().continuous_phase;

            for (const auto &tracked_target : m_tracked_target_deque)
            {
                const double vis_phase = tracked_target.continuous_phase + phase_compensate;
                Viz::log_with_time<VizTopic::RuneTrackContinuousPhase>(
                    function::to_nanoseconds_since_epoch(tracked_target.capture_timestamp),
                    vis_phase,
                    int64_t{tracked_target.switch_num});
            }

            has_visualized_init_tracked_target_deque = true;
            last_visualized_latest_timestamp_ns = latest_timestamp_ns;
        }
    }

    //可视化追踪的相位
    if (!m_tracked_target_deque.empty())
    {
        Viz::log_with_time<VizTopic::RuneTrackPhase>(
            function::to_nanoseconds_since_epoch(m_tracked_target_deque.back().capture_timestamp),
            m_tracked_target_deque.back().phase);
    }
    }
}

int BigRunePhaseMotionFilter::choose_tracked_target(const std::vector<CandidateTarget> &ori_candidate_targets)
{
    //选择目标的逻辑
    //如果只有一个目标:直接选择这个目标
    //如果有两个目标：选择在相位上更加领先的目标

    int target_index = 0;
    if (ori_candidate_targets.size() == 1)
    {
        //只有一个目标,直接选择这个目标
        target_index =0;
    }
    else
    {
        //计算差角
        double delta_phase_between = PRF::calculate_delta_phase<PRF::rad>(ori_candidate_targets[0].phase, ori_candidate_targets[1].phase);
        if (delta_phase_between > 0)
        {
            //说明0在相位上超前
            m_rotation_direction == RotationDirection::clockwise ? target_index = 0 : target_index = 1;
        }
        else
        {
            //说明1在相位上超前
            m_rotation_direction == RotationDirection::clockwise ? target_index = 1 : target_index = 0;
        }
    }

    return target_index;

}

bool BigRunePhaseMotionFilter::is_phase_jump(const CandidateTarget &ori_candidate_target)
{
    //判断相位跳变的逻辑：预测相位与观测相位的差值是否超出目标切换阈值
    //如果运动方程已经建立,那么用运动方程进行预测
    //如果没有,那么根据历史数据进行猜测


    if (m_tracked_target_deque.empty())
    {
        //说明是第一次追踪，默认不是跳变
        return false;
    }

    //目前在追踪的目标
    const TrackedTarget &tracked_target = m_tracked_target_deque.back();
    
    //预测相位
    double predict_phase;
    if (m_is_motion_model_vaild)
    {
        //如果有运动方程那么采用运动方程进行预测
        const double t = function::timestampMinus(ori_candidate_target.capture_timestamp, m_big_rune_motion_model.reference_timestamp) * 0.001;
        const double omega_t = m_big_rune_motion_model.speed_angular_frequency * t;
        predict_phase =
            m_big_rune_motion_model.phase_cos_coefficient * std::cos(omega_t) +
            m_big_rune_motion_model.phase_sin_coefficient * std::sin(omega_t) +
            m_big_rune_motion_model.phase_linear_velocity * t +
            m_big_rune_motion_model.phase_constant_offset_radians;
    }
    else
    {
        //根据数据的数目采用不同额的猜测策略
        predict_phase = guess_phase(ori_candidate_target.capture_timestamp);
    }
    
    //计算和预测角度的误差
    double delta_phase = std::fabs(PRF::calculate_delta_phase<PRF::rad>(predict_phase, ori_candidate_target.phase));
    if (delta_phase > (double)J_POWER_RUNE.config_["big_phase_motion_estimate"]["target_switch_threshold"])
    {        
        return true;
    }
    
    return false;
    
}

double BigRunePhaseMotionFilter::guess_phase(foxglove::Time future_time)
{
    if (m_tracked_target_deque.empty())
    {
        LOG(ERROR) << "[BigRunePhaseMotionFilter]在没有追踪数据时却尝试猜测相位";
        throw std::runtime_error("[BigRunePhaseMotionFilter]在没有追踪数据时却尝试猜测相位");
    }

    //ps：使用线性量预测鲁棒性很好，后面的两种算法在连续切换目标的时候会出现bug(加速度异常/速度异常)
    //后续多数据猜测算法可以优化一下
    if (true)
    {
        // 只有一个数据,那么直接用线性量进行预测
        double phase_linear_velocity = (double)J_POWER_RUNE.config_["big_phase_motion_estimate"]["expect_phase_linear_velocity"];
        m_rotation_direction == RotationDirection::anticlockwise ? phase_linear_velocity = -phase_linear_velocity: phase_linear_velocity = phase_linear_velocity;
        double time_interval =function::timestampMinus(future_time,m_tracked_target_deque.back().capture_timestamp) * 0.001;
        return m_tracked_target_deque.back().continuous_phase + phase_linear_velocity * time_interval;
    }
    else if (m_tracked_target_deque.size() == 2)
    {
        //有两个数据,使用帧差法来进行预测
        double delta_phase = PRF::calculate_delta_phase<PRF::rad>(m_tracked_target_deque.back().continuous_phase,m_tracked_target_deque.front().continuous_phase);
        double omega = delta_phase / (function::timestampMinus(m_tracked_target_deque.back().capture_timestamp, m_tracked_target_deque.front().capture_timestamp) * 0.001);
        double time_interval =function::timestampMinus(future_time,m_tracked_target_deque.back().capture_timestamp) * 0.001;
        return m_tracked_target_deque.back().continuous_phase + omega * time_interval;
    }
    else
    {
        const TrackedTarget &target_0 = m_tracked_target_deque[m_tracked_target_deque.size() - 3];
        const TrackedTarget &target_1 = m_tracked_target_deque[m_tracked_target_deque.size() - 2];
        const TrackedTarget &target_2 = m_tracked_target_deque.back();

        const double dt_01 = function::timestampMinus(target_1.capture_timestamp, target_0.capture_timestamp) * 0.001;
        const double dt_12 = function::timestampMinus(target_2.capture_timestamp, target_1.capture_timestamp) * 0.001;
        const double dt_future = function::timestampMinus(future_time, target_2.capture_timestamp) * 0.001;

        const double velocity_01 = (target_1.continuous_phase - target_0.continuous_phase) / dt_01;
        const double velocity_12 = (target_2.continuous_phase - target_1.continuous_phase) / dt_12;
        const double acceleration = (velocity_12 - velocity_01) / ((dt_01 + dt_12) * 0.5);
        const double latest_velocity = velocity_12 + 0.5 * acceleration * dt_12;

        return target_2.continuous_phase + latest_velocity * dt_future + 0.5 * acceleration * dt_future * dt_future;
    }
    
}

void BigRunePhaseMotionFilter::update_tracked_target_deque()
{
    // 弹出老的数据防止爆栈
    while (m_tracked_target_deque.size() >= m_ori_candidate_targets_deque.size() && !m_tracked_target_deque.empty())
    {
        //如果大于m_ori_candidate_targets_deque的数据,那么pop。
        //即最多存储6s的数据
        m_tracked_target_deque.pop_front();
    }

    // 选择目标
    int track_index = choose_tracked_target(m_ori_candidate_targets_deque.back());
    const bool is_jump = is_phase_jump(m_ori_candidate_targets_deque.back()[track_index]);
    m_jump_deque.emplace_back(Jump(is_jump,m_ori_candidate_targets_deque.back()[track_index].capture_timestamp));

    if (is_jump)
    {
        // 处理跳变情况
        resolve_track_after_phase_jump(m_ori_candidate_targets_deque.back()[track_index]);
    }
    else
    {
        // 正常进行角度连续化然后放入追踪队列
        TrackedTarget track_target(m_ori_candidate_targets_deque.back()[track_index]);
        track_target.continuous_phase = accumulate_phase(m_tracked_target_deque.back().continuous_phase, track_target.phase);
        m_tracked_target_deque.emplace_back(std::move(track_target));
    }
    
    //滤波
    //filter_with_motion();
    

    if (VizTopic::RuneTrackPhase::enabled())
    {
    if (!m_tracked_target_deque.empty())
    {
        static bool has_visualized_updated_tracked_target_deque = false;
        static uint64_t last_visualized_latest_timestamp_ns = 0;
        const uint64_t oldest_timestamp_ns =
            function::to_nanoseconds_since_epoch(m_tracked_target_deque.front().capture_timestamp);
        const uint64_t latest_timestamp_ns =
            function::to_nanoseconds_since_epoch(m_tracked_target_deque.back().capture_timestamp);

        if (!has_visualized_updated_tracked_target_deque || oldest_timestamp_ns > last_visualized_latest_timestamp_ns)
        {
            const double aligned_latest_phase = PRF::normalize_phase<PRF::rad>(m_tracked_target_deque.back().phase);
            const double phase_compensate = aligned_latest_phase - m_tracked_target_deque.back().continuous_phase;

            for (const auto &tracked_target : m_tracked_target_deque)
            {
                const double vis_phase = tracked_target.continuous_phase + phase_compensate;
                Viz::log_with_time<VizTopic::RuneTrackContinuousPhase>(
                    function::to_nanoseconds_since_epoch(tracked_target.capture_timestamp),
                    vis_phase,
                    int64_t{tracked_target.switch_num});
            }

            has_visualized_updated_tracked_target_deque = true;
            last_visualized_latest_timestamp_ns = latest_timestamp_ns;
        }

        Viz::log_with_time<VizTopic::RuneTrackPhase>(
            function::to_nanoseconds_since_epoch(m_tracked_target_deque.back().capture_timestamp),
            m_tracked_target_deque.back().phase);
    }
    }
}

void BigRunePhaseMotionFilter::resolve_track_after_phase_jump(const CandidateTarget &ori_candidate_target)
{


    //预测相位
    double predict_phase;
    if (m_is_motion_model_vaild)
    {
        //如果有运动方程那么采用运动方程进行预测
        const double t = function::timestampMinus(ori_candidate_target.capture_timestamp, m_big_rune_motion_model.reference_timestamp) * 0.001;
        const double omega_t = m_big_rune_motion_model.speed_angular_frequency * t;
        predict_phase =
            m_big_rune_motion_model.phase_cos_coefficient * std::cos(omega_t) +
            m_big_rune_motion_model.phase_sin_coefficient * std::sin(omega_t) +
            m_big_rune_motion_model.phase_linear_velocity * t +
            m_big_rune_motion_model.phase_constant_offset_radians;
    }
    else
    {
        //根据数据的数目采用不同额的猜测策略
        predict_phase = guess_phase(ori_candidate_target.capture_timestamp);
    }
   

    //计算从旧目标到新目标的差角
    double delta_phase = PRF::calculate_delta_phase<PRF::rad>(ori_candidate_target.phase, predict_phase);

    //判断切换的数目(4种情况:正负1/正负2)
    constexpr double k_sector_phase = CV_2PI / 5.0;
    int best_switch_count = 0;
    double best_residual = std::numeric_limits<double>::infinity();
    for (int k = -2; k <= 2; ++k)
    {
        if (k == 0)
        {
            continue;
        }

        //计算残差(差角与理想差角的差)
        double residual = std::fabs(delta_phase - k * k_sector_phase);
        if (residual < best_residual)
        {
            best_residual = residual;
            best_switch_count = k;
        }
    }

    //对历史的追踪作修正
    double phase_compensate = CV_2PI / 5.0 * best_switch_count;
    for (auto &tracked_target_item : m_tracked_target_deque)
    {
        tracked_target_item.continuous_phase += phase_compensate;
    }

    // 将新的目标加入追踪队列
    TrackedTarget continuous_target(ori_candidate_target);
    continuous_target.switch_num = best_switch_count;
    continuous_target.continuous_phase = accumulate_phase(m_tracked_target_deque.back().continuous_phase, continuous_target.phase);
    m_tracked_target_deque.emplace_back(std::move(continuous_target));
    
    //滤波



    //为了防止double出现溢出问题,强制限制最旧的数据的相位需要落在0-2pi
    const double first_phase = m_tracked_target_deque.front().continuous_phase;
    double normalized_first_phase = std::fmod(first_phase, CV_2PI);
    if (normalized_first_phase <= 0.0)
    {
        normalized_first_phase += CV_2PI;
    }
    const double overflow_compensate = normalized_first_phase - first_phase;
    if (overflow_compensate != 0.0)
    {
        for (auto &tracked_target_item : m_tracked_target_deque)
        {
            tracked_target_item.continuous_phase += overflow_compensate;
        }
    }
}

double BigRunePhaseMotionFilter::accumulate_phase(const double &old_phase, const double &new_phase)
{
    //计算差角(-pi到pi)
    double delta_phase = PRF::calculate_delta_phase<PRF::rad>(new_phase, old_phase);
    return old_phase + delta_phase;//增量累加
}

void BigRunePhaseMotionFilter::filter_with_motion()

{
    if (!m_is_motion_model_vaild)
    {
       LOG(ERROR) << "[BigRunePhaseMotionFilter]filter_with_motion在运动没有初始化时调用";
       throw std::runtime_error("[BigRunePhaseMotionFilter]filter_with_motion在运动没有初始化时调用");
    }

    if (m_tracked_target_deque.empty())
    {
        LOG(ERROR) << "[BigRunePhaseMotionFilter]filter_with_motion在没有追踪数据时调用";
        throw std::runtime_error("[BigRunePhaseMotionFilter]filter_with_motion在没有追踪数据时调用");
    }

    //平滑窗口大小
    int filter_window_size = J_POWER_RUNE.config_["big_phase_motion_estimate"]["filter_window_size"];
    const int tracked_target_size = static_cast<int>(m_tracked_target_deque.size());
    if (filter_window_size < 3)
    {
        filter_window_size = 3;//至少为3才能开始计算
        LOG(WARNING) << "[BigRunePhaseMotionFilter]过小的filter_window_size";

    }
    if (filter_window_size > tracked_target_size)
    {
        filter_window_size = tracked_target_size;
        LOG(WARNING) << "[BigRunePhaseMotionFilter]过大的filter_window_size";
    }
    if (std::fmod(filter_window_size, 2) != 1)
    {
        filter_window_size -= 1;
        LOG(WARNING) << "[BigRunePhaseMotionFilter]filter_window_size不能为偶数";

    }
    if (filter_window_size < 3)
    {
        return;
    }
    
    //相位预测函数
    auto predict_phase = [this](const foxglove::Time &timestamp) -> double
    {
        const double t = function::timestampMinus(timestamp, m_big_rune_motion_model.reference_timestamp) * 0.001;
        const double omega_t = m_big_rune_motion_model.speed_angular_frequency * t;
        return m_big_rune_motion_model.phase_cos_coefficient * std::cos(omega_t) +
               m_big_rune_motion_model.phase_sin_coefficient * std::sin(omega_t) +
               m_big_rune_motion_model.phase_linear_velocity * t +
               m_big_rune_motion_model.phase_constant_offset_radians;
    };

    //取尾部窗口中心数据,计算其残差
    int center_target_index = m_tracked_target_deque.size() -1 - filter_window_size / 2;
    auto &center_target = m_tracked_target_deque[center_target_index];
    double center_expect_phase = predict_phase(center_target.capture_timestamp);
    const double original_center_phase = center_target.continuous_phase;
    double center_target_residual = std::fabs(center_expect_phase - center_target.continuous_phase);

    
    int window_begin_index = center_target_index - filter_window_size / 2;
    int window_end_index = window_begin_index + filter_window_size;
    double total_residual = 0;
    for (int i = window_begin_index; i < window_end_index; i++)
    {
        const auto &track_target = m_tracked_target_deque[i];
        
        //首先需要计算该点与预测的残差
        double expect_phase = predict_phase(track_target.capture_timestamp);
        double residual = std::fabs(expect_phase -track_target.continuous_phase);
        total_residual += residual;
    }
    double avg_residual = total_residual / filter_window_size;

    if (avg_residual > center_target_residual || avg_residual == 0.0)
    {
        //说明预期和观测数据差异大或者完全不存在误差
        //无法保证可以修改数据或者无需修改数据
    }
    else
    {
        //说明该数据相比平均残差更大,那么可以修正
        //越偏离越相信模型
        double expect_phase_weight = std::min(((center_target_residual - avg_residual) / avg_residual) * 10, 1.0);
        center_target.continuous_phase = expect_phase_weight * center_expect_phase + (1 - expect_phase_weight) * center_target.continuous_phase;
    }

    if (VizTopic::RuneBigFilteredPhase::enabled())
    {
        Viz::log_with_time<VizTopic::RuneBigFilteredPhase>(
            function::to_nanoseconds_since_epoch(center_target.capture_timestamp),
            PRF::normalize_phase<PRF::rad>(original_center_phase),
            PRF::normalize_phase<PRF::rad>(center_target.continuous_phase));
    }
}

bool BigRunePhaseMotionFilter::need_reset_motion_for_abnormal_jump()
{

    //保证队列中的数据长度不多于1s,不少于0.7s
    const auto &newest_time = m_jump_deque.back().jump_time;
    while (function::timestampMinus(newest_time, m_jump_deque.front().jump_time) * 0.001 > 1.0)
    {
        m_jump_deque.pop_front();
    }
    if (function::timestampMinus(m_jump_deque.back().jump_time, m_jump_deque.front().jump_time)*0.001 < 0.7  || m_jump_deque.size() < 10)
    {
        //说明数据量不足
        return false;
    }
    
    
    //统计跳变次数
    int jump_count = 0;
    for (const auto &jump : m_jump_deque)
    {
        if (jump.is_jump)
        {
            jump_count++;
        }
    }

    //如果跳变的数据占了30%,那么需要重置运动方程
    if ((float)jump_count / (float)m_jump_deque.size() > 0.3)
    {
        return true;
    }
    return false;
}
