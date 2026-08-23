#pragma once

#include <optional>

#include "FireControlSystem.hpp"

namespace app_plugin
{

class RunePlannerControl final : public FireControlSystem
{
public:
    void process(const app::Context &context) override;

private:
    using EcsSubscriber = LatestChannel<ECSData>::Subscriber;
    using TrackSubscriber = LatestBuffer<TrackResult>::Subscriber;
    using FirePublisher = LatestBuffer<FireResult>::Publisher;

    void initialize(const app::Context &context);
    void visualize_result(const TrackResult &track, const FireResult &result) const;

    std::optional<EcsSubscriber> m_ecs_input;
    std::optional<TrackSubscriber> m_track_input;
    std::optional<FirePublisher> m_output;
    std::optional<ECSData> m_latest_ecs;
    bool m_visualize = true;
    bool m_initialized = false;
};

} // namespace app_plugin
