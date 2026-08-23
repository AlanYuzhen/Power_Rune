#pragma once

#include <chrono>
#include <memory>
#include <optional>

#include "ReceiveDecoder.hpp"

namespace app_plugin
{

class SimulatedReceiveDecoder final : public ReceiveDecoder
{
public:
    void process(const app::Context &context) override;

private:
    using Publisher = LatestChannel<ECSData>::Publisher;

    void load_config();
    void initialize_endpoint(const app::Context &context);

    std::optional<Publisher> m_output;
    ECSData m_template{};
    std::chrono::steady_clock::time_point m_started_at{};
    std::chrono::steady_clock::time_point m_next_publish{};
    std::chrono::nanoseconds m_publish_period{std::chrono::milliseconds(10)};
    double m_yaw_rate_deg_s = 0.0;
    double m_pitch_rate_deg_s = 0.0;
    double m_roll_rate_deg_s = 0.0;
    bool m_config_loaded = false;
};

} // namespace app_plugin
