// PowerRuneInterface.cpp：内部实现
#include "PowerRuneProcessor.hpp"
#include "RuneDecisionModule.hpp"
#include "power_rune_interface.hpp"
#include "json.hpp"
namespace power_rune
{

static RuneDecisionModule rune_decision_module;
static PowerRuneProcessor rune_processor(rune_decision_module);

void process_power_rune(const RuneInput &input)
{
    rune_processor.process_power_rune(input);
}

RuneSendData get_rune_data(bool is_big_rune)
{
    return rune_decision_module.get_send_data(is_big_rune);
}

}
