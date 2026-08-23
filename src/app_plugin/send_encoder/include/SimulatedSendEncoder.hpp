#pragma once

#include <optional>

#include "SendEncoder.hpp"

namespace app_plugin
{

class SimulatedSendEncoder final : public SendEncoder
{
public:
    void process(const app::Context &context) override;

private:
    using FireSubscriber = LatestBuffer<FireResult>::Subscriber;

    void initialize(const app::Context &context);

    std::optional<FireSubscriber> m_input;
    bool m_initialized = false;
};

} // namespace app_plugin
