#include "RuneDecisionModule.hpp"
#include "function.hpp"
#include "json.hpp"
#include "common/PowerRuneDiagnostics.hpp"
#include "common/PowerRuneVisualizeManager.hpp"
#include "common/power_rune_function.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <span>

namespace
{
inline foxglove::Time add_seconds(const foxglove::Time &t, double seconds)
{
    const int64_t base_ns =
        static_cast<int64_t>(t.sec()) * 1'000'000'000LL + static_cast<int64_t>(t.nsec());
    const int64_t delta_ns = static_cast<int64_t>(std::llround(seconds * 1e9));
    int64_t out_ns = base_ns + delta_ns;
    if (out_ns < 0)
    {
        out_ns = 0;
    }
    const uint64_t out = static_cast<uint64_t>(out_ns);
    return foxglove::Time(out / 1'000'000'000ULL, out % 1'000'000'000ULL);
}

} // namespace

using namespace transform_tools;

RuneDecisionModule::RuneDecisionModule()
    : m_send_time(function::getNowTimestamp())
{
    m_built_time = function::getNowTimestamp();
    m_is_valid = false;
    reset_cooldown();
}

void RuneDecisionModule::update_decision_module(std::vector<RuneTarget> &&rune_targets)
{
    //防止出现异常情况
    if (rune_targets.empty())
    {
        LOG(ERROR) << "[update_decision_module]外部未保证有值";
        return;
    }

    //由于框架原因,火控决策不再承担目标选择功能，当前只使用滤波后的第一个目标
    RuneTarget rune_target = std::move(rune_targets[0]);
    const bool is_big_rune = rune_target.is_big_rune;
    const auto &decision_config = J_POWER_RUNE.config_[decision_config_key(is_big_rune)];

    std::lock_guard<std::mutex> lock(m_data_mutex);

    // 模式切换时直接重建状态，避免沿用另一种模式的冷却和追踪历史。
    if (!m_is_valid || m_is_big_rune != is_big_rune)
    {
        m_is_big_rune = is_big_rune;
        m_built_time = rune_target.capture_timestamp;
        m_rune_target = std::move(rune_target);
        m_is_valid = true;
        reset_cooldown();
        return;
    }

    //判断是否超时后重新发现目标
    double time_interval = function::timestampMinus(rune_target.capture_timestamp, m_built_time) * 0.001;
    if (time_interval > (double)decision_config["max_life_time"])
    {
        m_built_time = rune_target.capture_timestamp;
        m_rune_target = std::move(rune_target);
        reset_cooldown();
        return;
    }

    //判断是否出现大角度跳变(目标切换)
    if (is_big_rune)
    {
        // 对于大符来说,可能会出现如下的情况：
        // 由于大符的两片符叶亮起和熄灭的存在短暂的先后上的区别,所以可能会出现在短时间内反复切换目标的情况。
        // 因此对于大符,如果出现大角度的变化,不能立即进行目标的切换,而是要暂时进行缓存,等到不出现连续的跳变时才进行切换

        double t = function::timestampMinus(rune_target.capture_timestamp, m_rune_target.big_rune_motion_model.reference_timestamp) * 0.001;
        const auto &motion_model = m_rune_target.big_rune_motion_model;
        const double omega_t = motion_model.speed_angular_frequency * t;
        const double predict_phase =
        motion_model.phase_cos_coefficient * std::cos(omega_t) +
        motion_model.phase_sin_coefficient * std::sin(omega_t) +
        motion_model.phase_linear_velocity * t +
        motion_model.phase_constant_offset_radians;

        double delta_phase = PRF::calculate_delta_phase<PRF::rad>(rune_target.phase, predict_phase);
        if (std::fabs(delta_phase) > (double)decision_config["target_switch_threshold"])
        {
            //出现跳变,那么需要放入缓冲队列
            m_pending_targets.emplace_back(std::move(rune_target));

            //判断缓冲队列是否是连续的无跳变
            bool is_need_change = need_change_target();
            if (!is_need_change)
            {
                //不需要,那么不作处理,保持目标不变
                return;
            }
            else
            {
                m_built_time = m_pending_targets.back().capture_timestamp;
                m_rune_target = m_pending_targets.back();
                reset_cooldown();
                return;
            }
            
        }
        else
        {
            //没有跳变,放入缓冲队伍队列作为参考值
            
            m_pending_targets.emplace_back(rune_target);
            while (m_pending_targets.size() > (int)decision_config["switch_confirm_count"])
            {
                m_pending_targets.pop_front();
            }

            //直接更新目标
            //这里有一种特殊情况就是上次的目标是一个,而这次的目标是两个。
            //在这种情况下，说明必然是新的一组符，那么必须要重置开火冷却
            //如果不重置开火冷却，那么会出现：上一次打灭的目标这次正好重新亮起。这种情况单靠跳变判断是判断不出来的
            //在这种情况下如果不重置冷却，可能会导致开火时间被白白浪费
            //但是又会引入一个新的问题：如果因为相机视野问题，一开始只看到1片，然后突然正好看到两片，那么可能会重复开火导致鞭尸问题出现
            //这个逻辑需要测试
            if (m_rune_target.inactivate_target_num == 1 && rune_target.inactivate_target_num == 2)
            {
                //reset_cooldown();
            }
            


            m_rune_target = std::move(rune_target);
            return;
        }

    }
    else
    {
        // 对于小符来说，直接切换即可(并且由于小符运动速度较慢，不需要用预测的相位进行判断)
        double delta_phase = PRF::calculate_delta_phase<PRF::rad>(rune_target.phase, m_rune_target.phase);
        if (std::fabs(delta_phase) > (double)decision_config["target_switch_threshold"])
        {
            m_built_time = rune_target.capture_timestamp;
            m_rune_target = std::move(rune_target);
            reset_cooldown();
            return;
        }

        // 运行到此处说明不需要重新建立(连续追踪)
        m_rune_target = std::move(rune_target);
    }
}

power_rune::RuneSendData RuneDecisionModule::get_send_data(bool is_big_rune)
{
    // 配置文件中的云台偏置以角度保存，而火控输出统一使用弧度。
    // 放在此处可覆盖正常弹道和超时回符心两条输出路径。
    const auto send_data_with_angle_offset = [this]()
    {
        power_rune::RuneSendData send_data = m_send_data;
        send_data.yaw += Angle::from_degree(
            static_cast<double>(J_POWER_RUNE.config_["offset"]["yaw"])).to_rad();
        send_data.pitch += Angle::from_degree(
            static_cast<double>(J_POWER_RUNE.config_["offset"]["pitch"])).to_rad();
        return send_data;
    };

    //准备获取快照
    RuneTarget rune_target;
    double time_interval;//追踪器建立到现在的时间
    const auto &decision_config = J_POWER_RUNE.config_[decision_config_key(is_big_rune)];

    //进行严重超时判断
    {
        std::lock_guard<std::mutex> lock(m_data_mutex);
        time_interval = function::timestampMinus(function::getNowTimestamp(), m_built_time) * 0.001;
        if (time_interval > (double)decision_config["critical_timeout"])
        {
            //严重超时，数据已经没有意义(后续符心归位算法允许数据超时，但是不能严重超时)
            m_is_valid = false;
        }

        //当前缓存目标与请求模式不一致时，直接按未发现返回，避免误用错误运动方程。
        if (!m_is_valid || m_is_big_rune != is_big_rune)
        {
            // 无效直接说明没有目标
            m_send_data.mode = rune_mode(is_big_rune);
            m_send_data.is_find_buff = false;
            m_send_data.is_enable_fire = false;
            m_send_time = function::getNowTimestamp();
            return send_data_with_angle_offset();
        }

        //运行到此处说明数据至少没有严重超时，进行快照
        rune_target = m_rune_target;
    }


    double cap2now_interval = function::timestampMinus(function::getNowTimestamp(),rune_target.capture_timestamp)*0.001;
    if (cap2now_interval>(double)J_POWER_RUNE.config_["temp"]["data_life"])
    {
        recover2rune_center(rune_target,is_big_rune,cap2now_interval);
        //m_send_data.is_find_buff = false;
        //m_send_data.is_enable_fire = false;
        return send_data_with_angle_offset();

    }
    

    //如果到达了追踪器的寿命
    if (time_interval > (double)decision_config["max_life_time"])
    {
        //recover2rune_center(rune_target, is_big_rune);
        //可能会抽搐，所以暂时禁用这个函数
        m_send_data.is_find_buff = false;
        m_send_data.is_enable_fire = false;
        return send_data_with_angle_offset();

    }
    else
    {
        //运行到此处说明是连续击打同一个目标， 正常调用弹道模型进行解算
        solve_ballistic(rune_target, function::timestampMinus(function::getNowTimestamp(), rune_target.capture_timestamp) * 0.001, is_big_rune);
    }

    /* 
    如果认为允许开火
        如果冷却不是0，那么降低冷却。不允许开火
        如果冷却是0，那么检查连续开火时间
            如果超出最大允许连续开火时间，那么不允许开火。连续开火时间归零，冷却时间达到最大
            如果没有超出连续开火时间，那么允许开火，累加连续开火时间。
    如果认为不允许开火
        减少冷却，将连续开火时间归0
    */
    double dt = function::timestampMinus(function::getNowTimestamp(), m_send_time) * 0.001;
    if (m_send_data.is_enable_fire)
    {
        //如果还在冷却期：禁止开火，只减少冷却
        double cooldown = m_fire_enable_time_until.load();
        if (cooldown > 0.0)
        {
            cooldown -= dt;
            m_fire_enable_time_until.store(std::max(0.0, cooldown));
            m_send_data.is_enable_fire = false;
        }
        else
        {
            //不在冷却期：累加连续开火的时间
            double fire_time = m_fire_remaining_time.load();
            fire_time += dt;

            //超出连续开火上限：不允许开火，进入冷却
            if (fire_time >= (double)decision_config["fire_remaining_time_threshold"])
            {
                m_fire_remaining_time.store(0.0);
                m_fire_enable_time_until.store((double)decision_config["fire_cooldown_time"]);
                m_send_data.is_enable_fire = false;
            }
            else//未超出连续开火上限：允许开火
            {
                m_fire_remaining_time.store(fire_time);
                m_send_data.is_enable_fire = true;
            }
        }
    }
    else//不允许开火
    {
        // 连续开火时间直接归零
        m_fire_remaining_time.store(0.0);

        //减少冷却
        double cooldown = m_fire_enable_time_until.load();
        if (cooldown > 0.0)
        {
            cooldown -= dt;
            m_fire_enable_time_until.store(std::max(0.0, cooldown));
        }

        m_send_data.is_enable_fire = false;
    }

    m_send_time = function::getNowTimestamp();
    return send_data_with_angle_offset();
}

inline Eigen::Vector3d RuneDecisionModule::calculate_armor_module_center(const Eigen::Vector3d &rune_center, const Eigen::Vector3d &start_vector, const Eigen::Vector3d &plane_normal, double rune_radius, double phase)
{
    Eigen::Vector3d u0 = start_vector.normalized();
    Eigen::Vector3d n = plane_normal.normalized();

    //Rodrigues公式在平面内的简化形式
    Eigen::Vector3d direction = u0 * std::cos(phase) + n.cross(u0) * std::sin(phase);

    return rune_center + rune_radius * direction;
}

const char *RuneDecisionModule::decision_config_key(bool is_big_rune) const
{
    return is_big_rune ? "big_rune_decision_module" : "small_rune_decision_module";
}

uint8_t RuneDecisionModule::rune_mode(bool is_big_rune) const
{
    return is_big_rune ? (uint8_t)RuneKind::BIG_BUFF : (uint8_t)RuneKind::SMALL_BUFF;
}

const char *RuneDecisionModule::log_prefix(bool is_big_rune) const
{
    return is_big_rune ? "BigRuneDecisionModule" : "SmallRuneDecisionModule";
}

void RuneDecisionModule::solve_ballistic(const RuneTarget &rune_target, double algorithmic_time, bool is_big_rune)
{
    if (is_big_rune)
    {
        solve_big_rune_ballistic(rune_target, algorithmic_time);
        return;
    }

    solve_small_rune_ballistic(rune_target, algorithmic_time);
}

void RuneDecisionModule::solve_small_rune_ballistic(const RuneTarget &rune_target, double algorithmic_time)
{
    //准备参数
    const auto &ballistic_config = J_POWER_RUNE.config_["rune_ballistic_model"];

    Eigen::Vector3d rune_center = rune_target.rune_center;//符心
    Eigen::Vector3d start_vector = rune_target.start_vector.normalized();//0相位方向
    Eigen::Vector3d rune_plane_normal = rune_target.rune_plane_world_normal;//转轴
    double rune_radius = ballistic_config["radius"];//半径
    double rune_w = rune_target.angular_velocity;//角速度
    double armor_phase = rune_target.phase;//初相位
    double gune_length = ballistic_config["gune_length"];//枪管长度
    double V_bullet = ballistic_config["bullet_flying_speed"];//弹丸的初速度
    double k_total_coefficient = (double)ballistic_config["drag_coefficient"] *
                                 (double)ballistic_config["air_density"] *
                                 (double)ballistic_config["reference_area"] /
                                 (2 * (double)ballistic_config["small_bullet_mass"]);//弹丸质量
    double gravity = ballistic_config["grivaty"];//重力加速度
    double magnus_acceleration = ballistic_config["magnus_acceleration"];//马格努斯力的加速度系数
    double delay_time = ballistic_config["delay_time"];//留给云台的响应时间

    //计算初始值
    double param[3];
    const Eigen::Vector3d armor_now = rune_target.armor_module_center;
    const double distance = armor_now.norm();
    param[0] = distance / V_bullet;// 飞行时间初值
    param[1] = std::atan2(armor_now.x(), armor_now.z());// yaw 初值
    param[2] = std::atan2(-armor_now.y(), std::sqrt(armor_now.x() * armor_now.x() + armor_now.z() * armor_now.z()));//pitch 初值

    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_BULLISTIC_DEBUG"])
    {
        //弹道模式
        rune_w = 0;
    }

    //构建问题
    ceres::Problem problem;
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<SmallRuneBallisticModel, 3, 3>(
            new SmallRuneBallisticModel(
                rune_center,
                start_vector,
                rune_plane_normal,
                rune_radius,
                rune_w,
                armor_phase,
                gune_length,
                V_bullet,
                k_total_coefficient,
                gravity,
                magnus_acceleration,
                algorithmic_time,
                delay_time));
    problem.AddResidualBlock(cost_function, nullptr, param);

    //约束
    problem.SetParameterLowerBound(param, 0, 0.0);//最小飞行时间
    problem.SetParameterUpperBound(param, 0, (double)ballistic_config["flying_time_max"]);//最大飞行时间
    problem.SetParameterUpperBound(param, 2, (double)ballistic_config["gimbal_pitch_max"]);//云台最大的pitc

    //求解
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 20;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //检查是否求解成功
    if (!summary.IsSolutionUsable())
    {
        m_send_data.mode = rune_mode(false);
        m_send_data.is_find_buff = false;
        m_send_data.is_enable_fire = false;
        LOG(ERROR) << "[" << log_prefix(false) << "]弹道求解失败";
        return;
    }

    //求解成功
    m_send_data.mode = rune_mode(false);
    m_send_data.is_find_buff = true;
    m_send_data.is_enable_fire = true;
    m_send_data.yaw = param[1];

    //计算弹道补偿
    double predict_phase = PRF::normalize_phase<PRF::rad>(armor_phase + rune_w * (algorithmic_time + delay_time + param[0]));
    Eigen::Vector3d predict_armor_center = calculate_armor_module_center(rune_center, start_vector, rune_plane_normal, rune_radius, predict_phase);
    double compensate = compensate_pitch(predict_armor_center);
    double compensated_pitch = param[2] + compensate;
    (void)compensated_pitch;
    m_send_data.pitch = param[2];//不采纳补偿

    if (VizTopic::RuneExpectYawPitch::enabled())
    {
        double expect_param[3];
        expect_param[0] = distance / V_bullet;
        expect_param[1] = std::atan2(armor_now.x(), armor_now.z());
        expect_param[2] = std::atan2(-armor_now.y(), std::sqrt(armor_now.x() * armor_now.x() + armor_now.z() * armor_now.z()));

        ceres::Problem expect_problem;
        ceres::CostFunction *expect_cost_function =
            new ceres::AutoDiffCostFunction<SmallRuneBallisticModel, 3, 3>(
                new SmallRuneBallisticModel(
                    rune_center,
                    start_vector,
                    rune_plane_normal,
                    rune_radius,
                    rune_w,
                    armor_phase,
                    gune_length,
                    V_bullet,
                    k_total_coefficient,
                    gravity,
                    magnus_acceleration,
                    algorithmic_time,
                    0.0));
        expect_problem.AddResidualBlock(expect_cost_function, nullptr, expect_param);

        expect_problem.SetParameterLowerBound(expect_param, 0, 0.0);
        expect_problem.SetParameterUpperBound(expect_param, 0, (double)ballistic_config["flying_time_max"]);
        expect_problem.SetParameterUpperBound(expect_param, 2, (double)ballistic_config["gimbal_pitch_max"]);

        ceres::Solver::Options expect_options;
        expect_options.linear_solver_type = ceres::DENSE_QR;
        expect_options.max_num_iterations = 20;
        expect_options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary expect_summary;
        ceres::Solve(expect_options, &expect_problem, &expect_summary);

        if (expect_summary.IsSolutionUsable())
        {
            Viz::log<VizTopic::RuneExpectYawPitch>(
                Angle::from_rad(-expect_param[1]).to_degree() -
                    static_cast<double>(J_POWER_RUNE.config_["offset"]["yaw"]),
                Angle::from_rad(expect_param[2]).to_degree() +
                    static_cast<double>(J_POWER_RUNE.config_["offset"]["pitch"]));
        }
    }

    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PREDICT_ERROR"])
    {
        // 预测误差调试只在弹道求解完成后记录一次“预测请求”。
        PowerRuneDiagnostics::instance().push_back_predict_phase(
            rune_target,
            algorithmic_time,
            delay_time,
            param[0]);
    }

    if (VizTopic::RunePredictTarget::enabled())
    {
        //foxglove可视化
        //预测未来的相位(在当前时刻预测经过控制延迟,飞行时间后的相位,即在控制延迟后从枪口出射的弹丸应该命中的目标的对应相位。)
        //这个未来的相位反解出的yaw和pitch就是当前时刻发给电控的指令
        const foxglove::Time predict_timestamp = add_seconds(rune_target.capture_timestamp, algorithmic_time + delay_time + param[0]);
        Viz::log_with_time<VizTopic::RunePredictPhase>(
            function::to_nanoseconds_since_epoch(predict_timestamp),
            predict_phase,
            0.0);
        std::array<foxglove::schemas::SpherePrimitive, 2> fox_plane_points;
        int num = 0;
        if (num >= static_cast<int>(fox_plane_points.size()))
        {
            LOG(ERROR) << "[RunePredictTarget]超出array大小";
        }
        else
        {
            fox_plane_points[num].pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{predict_armor_center.z(), -predict_armor_center.x(), -predict_armor_center.y()}};
            fox_plane_points[num].size = foxglove::schemas::Vector3{0.05, 0.05, 0.05};
            fox_plane_points[num].color = foxglove::schemas::Color{0, 1, 0, 1};
            num++;
        }
        Viz::publish_spheres<VizTopic::RunePredictTarget>(
            std::span<const foxglove::schemas::SpherePrimitive>(
                fox_plane_points.data(),
                static_cast<std::size_t>(num)));
    }
}

void RuneDecisionModule::solve_big_rune_ballistic(const RuneTarget &rune_target, double algorithmic_time)
{
    //准备参数
    const auto &ballistic_config = J_POWER_RUNE.config_["rune_ballistic_model"];

    const Eigen::Vector3d rune_center = rune_target.rune_center;
    const Eigen::Vector3d start_vector = rune_target.start_vector.normalized();
    const Eigen::Vector3d rune_plane_normal = rune_target.rune_plane_world_normal;
    const double rune_radius = ballistic_config["radius"];
    const auto &motion_model = rune_target.big_rune_motion_model;
    const double capture_to_reference_time_s =
        function::timestampMinus(rune_target.capture_timestamp, motion_model.reference_timestamp) * 0.001;

    const double gune_length = ballistic_config["gune_length"];
    const double V_bullet = ballistic_config["bullet_flying_speed"];
    const double k_total_coefficient =
        (double)ballistic_config["drag_coefficient"] *
        (double)ballistic_config["air_density"] *
        (double)ballistic_config["reference_area"] /
        (2 * (double)ballistic_config["small_bullet_mass"]);
    const double gravity = ballistic_config["grivaty"];
    const double magnus_acceleration = ballistic_config["magnus_acceleration"];
    const double delay_time = ballistic_config["delay_time"];

    //计算初始值
    double param[3];
    const Eigen::Vector3d armor_now = rune_target.armor_module_center;
    const double distance = armor_now.norm();
    param[0] = distance / V_bullet;
    param[1] = std::atan2(armor_now.x(), armor_now.z());
    param[2] = std::atan2(-armor_now.y(), std::sqrt(armor_now.x() * armor_now.x() + armor_now.z() * armor_now.z()));

    //构建问题
    ceres::Problem problem;
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<BigRuneBallisticModel, 3, 3>(
            new BigRuneBallisticModel(
                rune_center,
                start_vector,
                rune_plane_normal,
                rune_radius,
                motion_model,
                capture_to_reference_time_s,
                gune_length,
                V_bullet,
                k_total_coefficient,
                gravity,
                magnus_acceleration,
                algorithmic_time,
                delay_time));
    problem.AddResidualBlock(cost_function, nullptr, param);

    //约束
    problem.SetParameterLowerBound(param, 0, 0.0);
    problem.SetParameterUpperBound(param, 0, (double)ballistic_config["flying_time_max"]);
    problem.SetParameterUpperBound(param, 2, (double)ballistic_config["gimbal_pitch_max"]);

    //求解
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 20;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //检查是否求解成功
    if (!summary.IsSolutionUsable())
    {
        m_send_data.mode = rune_mode(true);
        m_send_data.is_find_buff = false;
        m_send_data.is_enable_fire = false;
        LOG(ERROR) << "[" << log_prefix(true) << "]弹道求解失败";
        return;
    }

    //求解成功
    m_send_data.mode = rune_mode(true);
    m_send_data.is_find_buff = true;
    m_send_data.is_enable_fire = true;
    m_send_data.yaw = param[1];

    const double t_predict_from_ref = capture_to_reference_time_s + algorithmic_time + delay_time + param[0];
    const double omega_t = motion_model.speed_angular_frequency * t_predict_from_ref;
    const double predict_phase_cont =
        motion_model.phase_cos_coefficient * std::cos(omega_t) +
        motion_model.phase_sin_coefficient * std::sin(omega_t) +
        motion_model.phase_linear_velocity * t_predict_from_ref +
        motion_model.phase_constant_offset_radians;
    const double predict_phase = PRF::normalize_phase<PRF::rad>(predict_phase_cont);
    const Eigen::Vector3d predict_armor_center =
        calculate_armor_module_center(rune_center, start_vector, rune_plane_normal, rune_radius, predict_phase);

    const double compensate = compensate_pitch(predict_armor_center);
    (void)compensate;
    m_send_data.pitch = param[2];//不采纳补偿

    if (VizTopic::RuneExpectYawPitch::enabled())
    {
        double expect_param[3];
        expect_param[0] = distance / V_bullet;
        expect_param[1] = std::atan2(armor_now.x(), armor_now.z());
        expect_param[2] = std::atan2(-armor_now.y(), std::sqrt(armor_now.x() * armor_now.x() + armor_now.z() * armor_now.z()));

        ceres::Problem expect_problem;
        ceres::CostFunction *expect_cost_function =
            new ceres::AutoDiffCostFunction<BigRuneBallisticModel, 3, 3>(
                new BigRuneBallisticModel(
                    rune_center,
                    start_vector,
                    rune_plane_normal,
                    rune_radius,
                    motion_model,
                    capture_to_reference_time_s,
                    gune_length,
                    V_bullet,
                    k_total_coefficient,
                    gravity,
                    magnus_acceleration,
                    algorithmic_time,
                    0.0));
        expect_problem.AddResidualBlock(expect_cost_function, nullptr, expect_param);

        expect_problem.SetParameterLowerBound(expect_param, 0, 0.0);
        expect_problem.SetParameterUpperBound(expect_param, 0, (double)ballistic_config["flying_time_max"]);
        expect_problem.SetParameterUpperBound(expect_param, 2, (double)ballistic_config["gimbal_pitch_max"]);

        ceres::Solver::Options expect_options;
        expect_options.linear_solver_type = ceres::DENSE_QR;
        expect_options.max_num_iterations = 20;
        expect_options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary expect_summary;
        ceres::Solve(expect_options, &expect_problem, &expect_summary);

        if (expect_summary.IsSolutionUsable())
        {
            Viz::log<VizTopic::RuneExpectYawPitch>(
                Angle::from_rad(-expect_param[1]).to_degree() +
                    static_cast<double>(J_POWER_RUNE.config_["offset"]["yaw"]),
                Angle::from_rad(expect_param[2]).to_degree() +
                    static_cast<double>(J_POWER_RUNE.config_["offset"]["pitch"]));
        }
    }

    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PREDICT_ERROR"])
    {
        // 大符同样只在弹道求解完成后记录，后续由真实相位驱动匹配。
        PowerRuneDiagnostics::instance().push_back_predict_phase(
            rune_target,
            algorithmic_time,
            delay_time,
            param[0]);
    }

    if (VizTopic::RunePredictTarget::enabled())
    {
        //预测未来的相位(在当前时刻预测经过控制延迟,飞行时间后的相位,即在控制延迟后从枪口出射的弹丸应该命中的目标的对应相位。)
        //这个未来的相位反解出的yaw和pitch就是当前时刻发给电控的指令
        const foxglove::Time predict_timestamp = add_seconds(rune_target.capture_timestamp, algorithmic_time + delay_time + param[0]);
        Viz::log_with_time<VizTopic::RunePredictPhase>(
            function::to_nanoseconds_since_epoch(predict_timestamp),
            predict_phase,
            0.0);
        std::array<foxglove::schemas::SpherePrimitive, 2> fox_plane_points;
        int num = 0;
        if (num >= static_cast<int>(fox_plane_points.size()))
        {
            LOG(ERROR) << "[RunePredictTarget]超出array大小";
        }
        else
        {
            fox_plane_points[num].pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{predict_armor_center.z(), -predict_armor_center.x(), -predict_armor_center.y()}};
            fox_plane_points[num].size = foxglove::schemas::Vector3{0.05, 0.05, 0.05};
            fox_plane_points[num].color = foxglove::schemas::Color{0, 1, 0, 1};
            num++;
        }
        Viz::publish_spheres<VizTopic::RunePredictTarget>(
            std::span<const foxglove::schemas::SpherePrimitive>(
                fox_plane_points.data(),
                static_cast<std::size_t>(num)));
    }
}

double RuneDecisionModule::compensate_pitch(const Eigen::Vector3d &predict_armor_center)
{
    //todo:根据参数列表中的数据自动计算补偿公式
    // 提取高度
    double h = predict_armor_center.y(); // y 高度

    // 计算距离 d = sqrt(x^2 + z^2)
    double d = std::sqrt(predict_armor_center.x() * predict_armor_center.x() +
                         predict_armor_center.z() * predict_armor_center.z());

    // 拟合模型参数(此处硬编码都是测试后拟合得到的)
    const double a = -0.08548;
    const double b = -0.02966;
    const double c = 0.006992;
    const double e = -0.004465;
    const double f = 0.00228;
    const double g = 0.02849;

    // 计算补偿
    double compensation = a * h + b * d + c * h * d + e * h * h + f * d * d + g;
    return compensation;
}

void RuneDecisionModule::recover2rune_center(const RuneTarget &rune_target, bool is_big_rune, double cap2now_interval)
{
    //计算超时的时间
    double timeout = cap2now_interval - (double)J_POWER_RUNE.config_["temp"]["data_life"];
    if (timeout > (double)J_POWER_RUNE.config_["temp"]["recover_time"])
    {
        //超过恢复时间，防止控死云台
        m_send_data.is_find_buff = false;
        m_send_data.is_enable_fire = false;
        return;
    }

    //计算弹道来求出yaw和pitch原始位置
    solve_ballistic(rune_target,cap2now_interval,is_big_rune);
    if (!m_send_data.is_find_buff)
    {
        //说明弹道求解失败，那么应该直接返回
        return;
    }
    
    double start_yaw = m_send_data.yaw;
    double start_pitch = m_send_data.pitch;

    // 计算让枪口指向符心的yaw和pitch
    double end_yaw = std::atan2(rune_target.rune_center.x(), rune_target.rune_center.z());
    double d = std::sqrt(rune_target.rune_center.x() * rune_target.rune_center.x() + rune_target.rune_center.z() * rune_target.rune_center.z());
    double end_pitch = std::atan2(-rune_target.rune_center.y(), d);

    //计算当前应该到的yaw和pitch
    double recover_k = std::min(1.0,timeout/(double)J_POWER_RUNE.config_["temp"]["recover_time"]);//恢复系数
    double yaw = start_yaw + recover_k * PRF::calculate_delta_phase<PRF::rad>(end_yaw, start_yaw);
    double pitch = start_pitch + recover_k * PRF::calculate_delta_phase<PRF::rad>(end_pitch, start_pitch);


    m_send_data.mode = rune_mode(is_big_rune);
    m_send_data.is_find_buff = true;
    m_send_data.is_enable_fire = false;
    m_send_data.yaw = yaw;
    m_send_data.pitch = pitch;
}

inline void RuneDecisionModule::reset_cooldown()
{
    const auto &decision_config = J_POWER_RUNE.config_[decision_config_key(m_is_big_rune)];

    //重置已经连续开火的时间
    m_fire_remaining_time.store(0.0);

    //重置初始冷却，即重新观察到目标之后等待多久才会开火
    m_fire_enable_time_until.store((double)decision_config["fire_cooldown_time_init"]);
}

bool RuneDecisionModule::need_change_target()
{
    if (m_pending_targets.size() < (int)J_POWER_RUNE.config_["big_rune_decision_module"]["switch_confirm_count"])
    {
        //缓存数据太少,不允许切换
        return false;
    }

    //去除无效的缓冲数据
    while (m_pending_targets.size() > (int)J_POWER_RUNE.config_["big_rune_decision_module"]["switch_confirm_count"])
    {
        m_pending_targets.pop_front();
    }
    
    bool need_switch = true;
    for (int i = 1; i < m_pending_targets.size(); i++)
    {
        //如果均没有出现跳变,说明确实需要切换目标
        //如果出现了任意一次跳变,那么不允许切换目标
       
        const auto &new_target = m_pending_targets[i];
        const auto &old_target = m_pending_targets[i - 1];

        double t = function::timestampMinus(new_target.capture_timestamp, old_target.big_rune_motion_model.reference_timestamp) * 0.001;
        const auto &motion_model = old_target.big_rune_motion_model;
        const double omega_t = motion_model.speed_angular_frequency * t;
        const double predict_phase =
        motion_model.phase_cos_coefficient * std::cos(omega_t) +
        motion_model.phase_sin_coefficient * std::sin(omega_t) +
        motion_model.phase_linear_velocity * t +
        motion_model.phase_constant_offset_radians;

        double delta_phase = PRF::calculate_delta_phase<PRF::rad>(new_target.phase, predict_phase);
        if (std::fabs(delta_phase) > (double)J_POWER_RUNE.config_["big_rune_decision_module"]["target_switch_threshold"])
        {
            //说明切换还不能确定
            need_switch = false;
        }

    }
    return need_switch;

    
}
