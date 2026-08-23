#include "SimulatedReceiveDecoder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>

#include <glog/logging.h>

#include "class_loader.hpp"
#include "json.hpp"

namespace
{

double wrap_degrees(double degrees)
{
    return std::remainder(degrees, 360.0);
}

AimMode parse_mode(int mode)
{
    if (mode == static_cast<int>(AimMode::BigRune))
        return AimMode::BigRune;
    if (mode == static_cast<int>(AimMode::SmallRune))
        return AimMode::SmallRune;
    throw std::runtime_error("serial_simulation.mode must be 2 (SmallRune) or 3 (BigRune)");
}

MyColor parse_color(int color)
{
    if (color == static_cast<int>(MyColor::Blue))
        return MyColor::Blue;
    if (color == static_cast<int>(MyColor::Red))
        return MyColor::Red;
    throw std::runtime_error("serial_simulation.my_color must match MyColor (Red=0, Blue=1)");
}

} // namespace

namespace app_plugin
{

void SimulatedReceiveDecoder::load_config()
{
    J_RECEIVE_DECODER.updateJson();
    if (!J_RECEIVE_DECODER.config_.isOpened())
        throw std::runtime_error(
            "failed to open receive decoder config: " +
            (RECEIVE_DECODER_CONFIG_DIR / "receive_decoder.json").string());

    const cv::FileStorage &serial = J_RECEIVE_DECODER.config_;
    m_template.my_color = parse_color(static_cast<int>(serial["my_color"]));
    m_template.mode = parse_mode(static_cast<int>(serial["mode"]));
    m_template.is_start = static_cast<int>(serial["is_start"]) != 0;
    m_template.is_ready = static_cast<int>(serial["is_ready"]) != 0;
    m_template.yaw = static_cast<double>(serial["yaw"]);
    m_template.pitch = static_cast<double>(serial["pitch"]);
    m_template.roll = static_cast<double>(serial["roll"]);
    m_yaw_rate_deg_s = static_cast<double>(serial["yaw_rate_deg_s"]);
    m_pitch_rate_deg_s = static_cast<double>(serial["pitch_rate_deg_s"]);
    m_roll_rate_deg_s = static_cast<double>(serial["roll_rate_deg_s"]);

    const int period_ms = static_cast<int>(serial["period_ms"]);
    if (period_ms <= 0)
        throw std::runtime_error("serial_simulation.period_ms must be positive");

    m_publish_period = std::chrono::milliseconds(period_ms);
    m_started_at = std::chrono::steady_clock::now();
    m_next_publish = m_started_at;
    m_config_loaded = true;

    LOG(INFO) << "[ReceiveDecoder] simulated ECS enabled: mode="
              << (m_template.mode == AimMode::BigRune ? "BigRune" : "SmallRune")
              << ", color=" << (m_template.my_color == MyColor::Blue ? "Blue" : "Red")
              << ", ready=" << m_template.is_ready
              << ", period_ms=" << period_ms;
}

void SimulatedReceiveDecoder::initialize_endpoint(const app::Context &context)
{
    if (!m_output)
        m_output.emplace(context.get_channel_publisher<ECSData>(this));
}

void SimulatedReceiveDecoder::process(const app::Context &context)
{
    if (!m_config_loaded)
        load_config();
    initialize_endpoint(context);

    const auto now = std::chrono::steady_clock::now();
    if (now < m_next_publish)
        std::this_thread::sleep_until(m_next_publish);

    const auto publish_time = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(publish_time - m_started_at).count();

    ECSData data = m_template;
    data.yaw = wrap_degrees(m_template.yaw + m_yaw_rate_deg_s * elapsed);
    data.pitch = wrap_degrees(m_template.pitch + m_pitch_rate_deg_s * elapsed);
    data.roll = wrap_degrees(m_template.roll + m_roll_rate_deg_s * elapsed);
    m_output->publish(std::make_shared<const ECSData>(data));

    m_next_publish += m_publish_period;
    if (publish_time > m_next_publish + 4 * m_publish_period)
        m_next_publish = publish_time + m_publish_period;
}

} // namespace app_plugin

REGISTER_PLUGIN("ReceiveDecoder", app_plugin::SimulatedReceiveDecoder)
