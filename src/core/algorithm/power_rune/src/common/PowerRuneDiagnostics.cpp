#include "common/PowerRuneDiagnostics.hpp"

#include "function.hpp"
#include "json.hpp"
#include "common/PowerRuneVisualizeManager.hpp"
#include "common/power_rune_function.hpp"

#include <cmath>
#include <limits>

namespace
{
constexpr double kPredictMatchToleranceMs = 1.0;
} // namespace

PowerRuneDiagnostics &PowerRuneDiagnostics::instance()
{
    static PowerRuneDiagnostics instance;
    return instance;
}

double PowerRuneDiagnostics::calculate_predict_phase_at_time(const PredictPhase &predict_phase,
                                                             const foxglove::Time &target_time)
{
    if (predict_phase.rune_target.is_big_rune)
    {
        const auto &motion_model = predict_phase.rune_target.big_rune_motion_model;
        const double t_from_ref =
            function::timestampMinus(target_time, motion_model.reference_timestamp) * 0.001;
        const double omega_t = motion_model.speed_angular_frequency * t_from_ref;
        const double phase =
            motion_model.phase_cos_coefficient * std::cos(omega_t) +
            motion_model.phase_sin_coefficient * std::sin(omega_t) +
            motion_model.phase_linear_velocity * t_from_ref +
            motion_model.phase_constant_offset_radians;
        return PRF::normalize_phase<PRF::rad>(phase);
    }

    double angular_velocity = predict_phase.rune_target.angular_velocity;
    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_BULLISTIC_DEBUG"])
    {
        angular_velocity = 0.0;
    }

    const double delta_t =
        function::timestampMinus(target_time, predict_phase.rune_target.capture_timestamp) * 0.001;
    return PRF::normalize_phase<PRF::rad>(predict_phase.rune_target.phase + angular_velocity * delta_t);
}

void PowerRuneDiagnostics::push_back_predict_phase(const RuneTarget &rune_target,
                                                   const double &algorithmic_time,
                                                   const double &delay_time,
                                                   const double &flying_time)
{
    PredictPhase predict_phase;
    predict_phase.reference_time = rune_target.capture_timestamp;
    predict_phase.algorithmic_time = algorithmic_time;
    predict_phase.delay_time = delay_time;
    predict_phase.flying_time = flying_time;
    predict_phase.rune_target = rune_target;

    {
        std::lock_guard lock(m_predict_phase_deque_mutex);
        m_predict_phase_deque.emplace_back(std::move(predict_phase));
        while (m_predict_phase_deque.size() > static_cast<size_t>(m_max_predict_phase_deque_size))
        {
            m_predict_phase_deque.pop_front();
        }
    }
}

void PowerRuneDiagnostics::push_back_real_phase(const foxglove::Time &capture_time, const double &phase)
{
    RealPhase real_phase;
    real_phase.capture_time = capture_time;
    real_phase.real_phase = phase;

    {
        std::lock_guard lock(m_real_phase_deque_mutex);
        m_real_phase_opt = std::move(real_phase);
    }
    m_real_phase_cv.notify_all();
    try_calculate_predict_error();
}

void PowerRuneDiagnostics::try_calculate_predict_error()
{

    const auto to_target_timestamp_ns = [](const PredictPhase &predict_phase)
    {
        return static_cast<int64_t>(function::to_nanoseconds_since_epoch(predict_phase.reference_time)) +
               static_cast<int64_t>(std::llround(
                   (predict_phase.algorithmic_time + predict_phase.delay_time + predict_phase.flying_time) * 1e9));
    };

    RealPhase real_phase;
    {
        std::unique_lock<std::mutex> lock(m_real_phase_deque_mutex);
        m_real_phase_cv.wait(lock, [this] { return m_real_phase_opt.has_value(); });

        real_phase = std::move(*m_real_phase_opt);
        m_real_phase_opt.reset();
    }

    PredictPhase matched_predict_phase;
    double matched_dt_ms = 0.0;
    bool has_match = false;
    const int64_t real_phase_time_ns =
        static_cast<int64_t>(function::to_nanoseconds_since_epoch(real_phase.capture_time));
    {
        std::lock_guard lock(m_predict_phase_deque_mutex);

        while (!m_predict_phase_deque.empty())
        {
            const double front_dt_ms = (real_phase_time_ns - to_target_timestamp_ns(m_predict_phase_deque.front())) / 1e6;
            if (front_dt_ms > kPredictMatchToleranceMs)
            {
                m_predict_phase_deque.pop_front();
                continue;
            }
            break;
        }
        if (m_predict_phase_deque.empty())
        {
            return;
        }

        size_t best_index = 0;
        double best_abs_dt_ms = std::numeric_limits<double>::max();
        for (size_t i = 0; i < m_predict_phase_deque.size(); ++i)
        {
            const double dt_ms = (real_phase_time_ns - to_target_timestamp_ns(m_predict_phase_deque[i])) / 1e6;
            const double abs_dt_ms = std::abs(dt_ms);
            if (abs_dt_ms < best_abs_dt_ms)
            {
                best_abs_dt_ms = abs_dt_ms;
                matched_dt_ms = dt_ms;
                best_index = i;
            }
        }
        if (best_abs_dt_ms > kPredictMatchToleranceMs)
        {
            return;
        }

        matched_predict_phase = m_predict_phase_deque[best_index];
        m_predict_phase_deque.erase(
            m_predict_phase_deque.begin(),
            m_predict_phase_deque.begin() + static_cast<std::ptrdiff_t>(best_index + 1));
        has_match = true;
    }

    if (!has_match)
    {
        return;
    }

    const double predict_phase = calculate_predict_phase_at_time(matched_predict_phase, real_phase.capture_time);
    const double phase_error = PRF::calculate_delta_phase<PRF::rad>(real_phase.real_phase, predict_phase);
    
    Viz::log<VizTopic::RunePredictError>(predict_phase, real_phase.real_phase, phase_error, matched_dt_ms);
}
