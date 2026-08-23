#pragma once
#include "RuneObservationRefiner.hpp"
#include "PowerRunePlane.hpp"
#include "PhaseMotionEstimator.hpp"
#include "RuneDecisionModule.hpp"
#include "power_rune_interface.hpp"
class PowerRuneProcessor
{
public:
    PowerRuneProcessor(RuneDecisionModule &rune_decision_module);
    void process_power_rune(const power_rune::RuneInput &detect_input);

private:
    RuneObservation convert2rune_observation(const power_rune::RuneInput &detect_input);//构建符链路的结构体
    RuneObservationRefiner m_rune_observation_refiner;
    PowerRunePlane m_power_rune_plane;
    PhaseMotionEstimator m_phase_motion_estimator;
    RuneDecisionModule &m_rune_decision_module;
    std::vector<RuneTarget> m_debug_rune_targets;
};
