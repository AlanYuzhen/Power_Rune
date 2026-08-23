#pragma once

#include <chrono>
#include <filesystem>
#include <optional>

#include <opencv2/videoio.hpp>

#include "CameraManager.hpp"

namespace app_plugin
{

class ReplayCameraManager final : public CameraManager
{
public:
    void process(const app::Context &context) override;

private:
    using EcsSubscriber = LatestChannel<ECSData>::Subscriber;
    using FramePublisher = LatestBuffer<InputFrame>::Publisher;

    void load_config_and_open_video();
    void initialize_endpoints(const app::Context &context);
    bool read_frame(cv::Mat &frame);
    void validate_frame(const cv::Mat &frame) const;
    void throttle();

    std::optional<EcsSubscriber> m_ecs_input;
    std::optional<FramePublisher> m_output;
    cv::VideoCapture m_capture;
    std::filesystem::path m_video_path;
    std::chrono::steady_clock::time_point m_next_frame{};
    std::chrono::nanoseconds m_frame_period{std::chrono::milliseconds(33)};
    std::chrono::milliseconds m_eof_pause{1000};
    int m_expected_width = 0;
    int m_expected_height = 0;
    bool m_loop = true;
    bool m_initialized = false;
};

} // namespace app_plugin
