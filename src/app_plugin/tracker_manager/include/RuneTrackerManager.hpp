#pragma once

#include <optional>

#include "TrackerManager.hpp"

namespace app_plugin
{

class RuneTrackerManager final : public ::TrackerManager
{
public:
    void process(const app::Context &context) override;

private:
    using InputSubscriber = LatestBuffer<InputFrameWithNNResults>::Subscriber;
    using OutputPublisher = LatestBuffer<TrackResult>::Publisher;

    void initialize_endpoints(const app::Context &context);
    void initialize_tf_tree();
    void update_tf_tree(const ECSData &ecs_data);

    std::optional<InputSubscriber> m_input;
    std::optional<OutputPublisher> m_output;
    transform_tools::TFTree m_tf_tree;
    double m_x_offset_m = 0.0;
    double m_y_offset_m = 0.0;
    double m_z_offset_m = 0.0;
    double m_gimbal_true_y_degree = 0.0;
    double m_gimbal_true_x_degree = 0.0;
    bool m_tf_initialized = false;
};

} // namespace app_plugin
