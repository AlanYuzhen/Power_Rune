#include "ReplayCameraManager.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <glog/logging.h>

#include "class_loader.hpp"
#include "json.hpp"
#include "time/time.hpp"

namespace app_plugin
{

void ReplayCameraManager::load_config_and_open_video()
{
    J_CAMERA.updateJson();
    if (!J_CAMERA.config_.isOpened())
        throw std::runtime_error(
            "failed to open camera config: " +
            (CAMERA_CONFIG_DIR / "camera.json").string());

    const cv::FileStorage &video = J_CAMERA.config_;
    const std::string configured_path = static_cast<std::string>(video["video"]);
    if (configured_path.empty())
        throw std::runtime_error("camera.video cannot be empty");

    const std::filesystem::path path(configured_path);
    m_video_path = (path.is_absolute() ? path : CAMERA_CONFIG_DIR / path).lexically_normal();
    m_loop = static_cast<int>(video["loop"]) != 0;
    m_eof_pause = std::chrono::milliseconds(
        std::max(0, static_cast<int>(video["eof_pause_ms"])));
    m_expected_width = static_cast<int>(video["expected_width"]);
    m_expected_height = static_cast<int>(video["expected_height"]);

    if (!m_capture.open(m_video_path.string()))
    {
        std::ostringstream message;
        message << "failed to open demo video: " << m_video_path;
        throw std::runtime_error(message.str());
    }

    double source_fps = m_capture.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(source_fps) || source_fps <= 0.0)
        source_fps = 30.0;
    const double replay_speed = static_cast<double>(video["replay_speed"]);
    if (!std::isfinite(replay_speed) || replay_speed <= 0.0)
        throw std::runtime_error("replay.replay_speed must be finite and positive");
    const double fps = source_fps * replay_speed;

    m_frame_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / fps));
    m_next_frame = std::chrono::steady_clock::now();
    m_initialized = true;

    LOG(INFO) << "[CameraManager] replay opened: path=" << m_video_path
              << ", fps=" << fps
              << ", loop=" << m_loop
              << ", eof_pause_ms=" << m_eof_pause.count();
}

void ReplayCameraManager::initialize_endpoints(const app::Context &context)
{
    if (!m_ecs_input)
        m_ecs_input.emplace(context.get_channel_subscriber<ECSData>(this, {}, true));
    if (!m_output)
        m_output.emplace(context.get_buffer_publisher<InputFrame>(this));
}

void ReplayCameraManager::throttle()
{
    const auto now = std::chrono::steady_clock::now();
    if (now < m_next_frame)
        std::this_thread::sleep_until(m_next_frame);

    const auto wake_time = std::chrono::steady_clock::now();
    m_next_frame += m_frame_period;
    if (wake_time > m_next_frame + 4 * m_frame_period)
        m_next_frame = wake_time + m_frame_period;
}

bool ReplayCameraManager::read_frame(cv::Mat &frame)
{
    if (m_capture.read(frame) && !frame.empty())
    {
        validate_frame(frame);
        return true;
    }

    std::this_thread::sleep_for(m_eof_pause);
    if (!m_loop)
        return false;

    bool restarted = false;
    if (m_capture.set(cv::CAP_PROP_POS_FRAMES, 0.0))
        restarted = m_capture.read(frame) && !frame.empty();

    // 有些 VideoCapture 后端声称 seek 成功，但紧接着仍然读不到首帧。
    if (!restarted)
    {
        m_capture.release();
        if (!m_capture.open(m_video_path.string()))
            throw std::runtime_error("failed to reopen demo video after EOF");
        restarted = m_capture.read(frame) && !frame.empty();
    }

    if (!restarted)
        throw std::runtime_error("demo video has no decodable frame");

    validate_frame(frame);
    m_next_frame = std::chrono::steady_clock::now() + m_frame_period;
    return true;
}

void ReplayCameraManager::validate_frame(const cv::Mat &frame) const
{
    if ((m_expected_width > 0 && frame.cols != m_expected_width) ||
        (m_expected_height > 0 && frame.rows != m_expected_height))
    {
        std::ostringstream message;
        message << "demo frame size is " << frame.cols << 'x' << frame.rows
                << ", expected " << m_expected_width << 'x' << m_expected_height;
        throw std::runtime_error(message.str());
    }
}

void ReplayCameraManager::process(const app::Context &context)
{
    if (!m_initialized)
        load_config_and_open_video();
    initialize_endpoints(context);

    throttle();

    cv::Mat image;
    if (!read_frame(image))
        return;

    // 姿态必须在节流和解码之后获取，尤其不能复用 EOF 暂停前的旧数据。
    const std::shared_ptr<const ECSData> ecs_data = m_ecs_input->wait_next();
    const timetool::Timestamp timestamp = timetool::now();

    InputFrame frame;
    frame.ecs_data = *ecs_data;
    frame.timestamp = timestamp;
    frame.img = std::move(image);
    m_output->push(std::move(frame));
}

} // namespace app_plugin

REGISTER_PLUGIN("CameraManager", app_plugin::ReplayCameraManager)
