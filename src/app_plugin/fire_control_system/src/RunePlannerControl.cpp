#include "RunePlannerControl.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include "class_loader.hpp"
#include "img_viz.hpp"
#include "json.hpp"
#include "power_rune_interface.hpp"
#include "transform_tools/transform_tools.h"

namespace app_plugin
{

void RunePlannerControl::initialize(const app::Context &context)
{
    if (!m_ecs_input)
        m_ecs_input.emplace(context.get_channel_subscriber<ECSData>(this, {}, true));
    if (!m_track_input)
        m_track_input.emplace(context.get_buffer_subscriber<TrackResult>(this));
    if (!m_output)
        m_output.emplace(context.get_buffer_publisher<FireResult>(this));

    J_FIRE_CONTROL.updateJson();
    if (!J_FIRE_CONTROL.config_.isOpened())
        throw std::runtime_error(
            "failed to open fire control config: " +
            (FIRE_CONTROL_CONFIG_DIR / "fire_control_system.json").string());
    m_visualize = static_cast<int>(
        J_FIRE_CONTROL.config_["visualize"]["enabled"]) != 0;
    m_initialized = true;
}

void RunePlannerControl::visualize_result(const TrackResult &track,
                                          const FireResult &result) const
{
    if (!m_visualize || track.img.empty())
        return;

    cv::Mat canvas = track.img.clone();
    const cv::Scalar state_color = result.is_find_buff
        ? cv::Scalar(80, 255, 80)
        : cv::Scalar(80, 80, 255);

    std::ostringstream angles;
    angles << std::fixed << std::setprecision(2)
           << "yaw=" << result.yaw << " deg  pitch=" << result.pitch << " deg";
    std::ostringstream flags;
    flags << "ready=" << m_latest_ecs->is_ready
          << "  found=" << result.is_find_buff
          << "  fire=" << result.is_enable_fire;

    cv::putText(canvas,
                result.mode == AimMode::BigRune ? "BIG RUNE DEMO" : "SMALL RUNE DEMO",
                cv::Point(30, 45), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                state_color, 2, cv::LINE_AA);
    cv::putText(canvas, angles.str(), cv::Point(30, 82),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2, cv::LINE_AA);
    cv::putText(canvas, flags.str(), cv::Point(30, 117),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2, cv::LINE_AA);

    // ImgViz owns the process-wide single HighGUI worker. Plugins never call imshow/waitKey.
    ImgViz::enqueue_image_zero_copy("RuneDemo/EndToEnd", canvas);
}

void RunePlannerControl::process(const app::Context &context)
{
    if (!m_initialized)
        initialize(context);

    // 严格保持参考流程：先等待最新 ECS，再等待本轮 TrackResult。
    m_latest_ecs = *m_ecs_input->wait_next();
    TrackResult track = m_track_input->wait_pop();

    FireResult result;
    result.mode = m_latest_ecs->mode;
    result.is_keep_shooting = false;

    if (result.mode == AimMode::SmallRune || result.mode == AimMode::BigRune)
    {
        const bool is_big_rune = result.mode == AimMode::BigRune;
        const power_rune::RuneSendData rune_data = power_rune::get_rune_data(is_big_rune);

        result.is_find_target = static_cast<bool>(rune_data.is_find_buff);
        result.is_find_buff = static_cast<bool>(rune_data.is_find_buff);
        result.is_enable_fire =
            static_cast<bool>(rune_data.is_enable_fire) && result.is_find_buff && m_latest_ecs->is_ready;

        // RuneSendData 在未找到目标时不保证 yaw/pitch 有效，因此只在 found 后读取。
        if (result.is_find_buff)
        {
            // 与 26-Auto-aim 一致：yaw 取反后由 rad 转 degree，pitch 正号转换。
            result.yaw = transform_tools::Angle::from_rad(-rune_data.yaw).to_degree();
            result.pitch = transform_tools::Angle::from_rad(rune_data.pitch).to_degree();
        }
    }

    const bool finite = std::isfinite(result.yaw) && std::isfinite(result.pitch);
    if (!finite)
    {
        LOG(ERROR) << "[PlannerControl] non-finite rune output rejected: yaw="
                   << result.yaw << ", pitch=" << result.pitch;
        result.yaw = 0.0;
        result.pitch = 0.0;
        result.is_find_target = false;
        result.is_find_buff = false;
        result.is_enable_fire = false;
    }

    visualize_result(track, result);
    m_output->push(std::move(result));
}

} // namespace app_plugin

REGISTER_PLUGIN("PlannerControl", app_plugin::RunePlannerControl)
