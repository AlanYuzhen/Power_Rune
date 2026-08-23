#include "SimulatedSendEncoder.hpp"

#include <cmath>

#include <glog/logging.h>

#include "class_loader.hpp"

namespace app_plugin
{

void SimulatedSendEncoder::initialize(const app::Context &context)
{
    if (!m_input)
        m_input.emplace(context.get_buffer_subscriber<FireResult>(this));
    m_initialized = true;
}

void SimulatedSendEncoder::process(const app::Context &context)
{
    if (!m_initialized)
        initialize(context);

    const FireResult result = m_input->wait_pop();
    const bool finite = std::isfinite(result.yaw) && std::isfinite(result.pitch);

    if (!finite)
    {
        LOG(ERROR) << "[SendEncoder] rejected non-finite result: yaw_deg=" << result.yaw
                   << ", pitch_deg=" << result.pitch;
        return;
    }

    // 模拟发送端只消费结果，不输出逐帧数据。错误仍通过上面的 ERROR 日志报告。
}

} // namespace app_plugin

REGISTER_PLUGIN("SendEncoder", app_plugin::SimulatedSendEncoder)
