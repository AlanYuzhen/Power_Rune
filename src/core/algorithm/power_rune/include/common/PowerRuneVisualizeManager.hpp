#pragma once

#include "json.hpp"
#include "common/power_rune_function.hpp"
#include "foxglove_viz/foxglove_viz.hpp"

#include <iostream>
#include <array>
#include <concepts>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace VizTopic
{

    struct RuneOriPhase
    {
        using Publisher = foxglove_viz::DataPublisher<double, double>;

        static constexpr std::string_view topic = "/rune_ori_phase";
        static constexpr std::array<std::string_view, 2> fields = {
            "phase0", "phase1"};

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PHASE_FOXGLOVE"];
        }
    };

    struct RuneFilteredPhase
    {
        using Publisher = foxglove_viz::DataPublisher<double, double>;

        static constexpr std::string_view topic = "/rune_filtered_phase";
        static constexpr std::array<std::string_view, 2> fields = {
            "observed_phase", "filtered_phase"};

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PHASE_FOXGLOVE"];
        }
    };

    struct RuneTrackContinuousPhase
    {
        using Publisher = foxglove_viz::DataPublisher<double, int64_t>;

        static constexpr std::string_view topic = "/rune_track_continuous_phase";
        static constexpr std::array<std::string_view, 2> fields = {
            "phase", "switch_num"};

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PHASE_FOXGLOVE"];
        }
    };

    struct RuneTrackPhase
    {
        using Publisher = foxglove_viz::DataPublisher<double>;

        static constexpr std::string_view topic = "/rune_track_phase";
        static constexpr std::array<std::string_view, 1> fields = {"phase"};

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PHASE_FOXGLOVE"];
        }
    };

    struct RuneBigFilteredPhase
    {
        using Publisher = foxglove_viz::DataPublisher<double, double>;

        static constexpr std::string_view topic = "/rune_big_filtered_phase";
        static constexpr std::array<std::string_view, 2> fields = {
            "observed_phase", "filtered_phase"};

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PHASE_FOXGLOVE"];
        }
    };

    struct RunePlaneVector
    {
        using Publisher = foxglove_viz::EntityPublisher;

        static constexpr std::string_view topic = "/scene_car";
        static constexpr std::string_view frame_id = "car_frame";
        static constexpr std::string_view entity_id = "rune_plane_vector";

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_RUNE_PLANE_VECTOR_FOXGLOVE"];
        }
    };

    struct RunePredictError
    {
        using Publisher = foxglove_viz::DataPublisher<double, double, double, double>;
        static constexpr std::string_view topic = "/rune_predict_error";
        static constexpr std::array<std::string_view, 4> fields = {
            "predict_phase", "real_phase", "phase_error", "matched_dt_ms"};

        static bool enabled()
        {   
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_TRACK_PREDICT_ERROR"];
        }
    };

    struct RuneExpectYawPitch
    {
        using Publisher = foxglove_viz::DataPublisher<double, double>;

        static constexpr std::string_view topic = "/rune_expect_yaw_pitch";
        static constexpr std::array<std::string_view, 2> fields = {"yaw", "pitch"};

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_FIRE"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_FIRE_EXPECT_YAW_PITCH_FOXGLOVE"];
        }
    };

    struct RunePredictPhase
    {
        using Publisher = foxglove_viz::DataPublisher<double, double>;

        static constexpr std::string_view topic = "/rune_predict_phase";
        static constexpr std::array<std::string_view, 2> fields = {
            "predict_phase", "reference_phase"};

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_FIRE"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_FIRE_PREDICT_TARGET_FOXGLOVE"];
        }
    };

    struct RunePredictTarget
    {
        using Publisher = foxglove_viz::EntityPublisher;

        static constexpr std::string_view topic = "/scene_car";
        static constexpr std::string_view frame_id = "car_frame";
        static constexpr std::string_view entity_id = "rune_predict_target";

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_FIRE"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_FIRE_PREDICT_TARGET_FOXGLOVE"];
        }
    };

    struct PowerRuneCar
    {
        using Publisher = foxglove_viz::EntityPublisher;

        static constexpr std::string_view topic = "/scene_car";
        static constexpr std::string_view frame_id = "car_frame";
        static constexpr std::string_view entity_id = "power_rune_car";

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD_3D_CAR_FOXGLOVE"];
        }
    };

    struct PowerRuneNormalCar
    {
        using Publisher = foxglove_viz::EntityPublisher;

        static constexpr std::string_view topic = "/scene_car";
        static constexpr std::string_view frame_id = "car_frame";
        static constexpr std::string_view entity_id = "power_rune_normal_car";

        static bool enabled()
        {
            return PowerRuneCar::enabled();
        }
    };

    struct PowerRuneCamera
    {
        using Publisher = foxglove_viz::EntityPublisher;

        static constexpr std::string_view topic = "/scene_camera";
        static constexpr std::string_view frame_id = "camera_frame";
        static constexpr std::string_view entity_id = "power_rune_camera";

        static bool enabled()
        {
            return (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD"] &&
                   (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD_3D_CAMERA_FOXGLOVE"];
        }
    };

    struct PowerRuneNormalCamera
    {
        using Publisher = foxglove_viz::EntityPublisher;

        static constexpr std::string_view topic = "/scene_camera";
        static constexpr std::string_view frame_id = "camera_frame";
        static constexpr std::string_view entity_id = "power_rune_normal_camera";

        static bool enabled()
        {
            return PowerRuneCamera::enabled();
        }
    };
} // namespace VizTopic

class Viz
{
public:
    template <typename Topic, typename... Args>
    static void log(Args &&...args)
    {
        if (!Topic::enabled())
        {
            return;
        }

        publisher<Topic>()->publish(std::forward<Args>(args)...);
    }

    template <typename Topic, typename... Args>
    static void log_with_time(uint64_t timestamp_ns, Args &&...args)
    {
        if (!Topic::enabled())
        {
            return;
        }

        publisher<Topic>()->publish_with_time(
            PRF::timestamp_from_nanoseconds(timestamp_ns),
            std::forward<Args>(args)...);
    }

    template <typename Topic>
    static void publish_spheres(std::span<const foxglove::schemas::SpherePrimitive> spheres)
    {
        if (!Topic::enabled())
        {
            return;
        }

        publisher<Topic>()->publish_spheres(spheres);
    }

    template <typename Topic>
    static void publish_arrows(std::span<const foxglove::schemas::ArrowPrimitive> arrows)
    {
        if (!Topic::enabled())
        {
            return;
        }

        publisher<Topic>()->publish_arrows(arrows);
    }

private:
    static Viz &instance();

    template <typename Topic>
    static const typename Topic::Publisher::Ptr &publisher();

    template <typename Topic>
    static std::string publisher_key();

    template <typename Topic>
    static typename Topic::Publisher::Ptr create_publisher();

    template <typename Topic>
    typename Topic::Publisher::Ptr get_or_create();

    std::unordered_map<std::string, std::shared_ptr<void>> m_publishers;
    std::mutex m_mutex;
};

inline Viz &Viz::instance()
{
    static Viz viz;
    return viz;
}

template <typename Topic>
const typename Topic::Publisher::Ptr &Viz::publisher()
{
    static const auto publisher = instance().get_or_create<Topic>();
    return publisher;
}

template <typename Topic>
std::string Viz::publisher_key()
{
    if constexpr (requires { Topic::frame_id; Topic::entity_id; })
    {
        std::string key(Topic::topic);
        key.push_back('\n');
        key += Topic::frame_id;
        key.push_back('\n');
        key += Topic::entity_id;
        return key;
    }
    else
    {
        return std::string(Topic::topic);
    }
}

template <typename Topic>
typename Topic::Publisher::Ptr Viz::create_publisher()
{
    if constexpr (std::same_as<typename Topic::Publisher, foxglove_viz::EntityPublisher>)
    {
        return foxglove_viz::global_foxglove_server()
            .create_entity_publisher(Topic::topic, Topic::frame_id, Topic::entity_id);
    }
    else
    {
        std::vector<std::string> fields;
        fields.reserve(Topic::fields.size());
        for (const std::string_view field : Topic::fields)
        {
            fields.emplace_back(field);
        }

        return std::make_shared<typename Topic::Publisher>(
            foxglove_viz::global_foxglove_server(),
            Topic::topic,
            std::move(fields));
    }
}

template <typename Topic>
typename Topic::Publisher::Ptr Viz::get_or_create()
{
    const std::string key = publisher_key<Topic>();
    std::lock_guard<std::mutex> lock(m_mutex);

    if (const auto iter = m_publishers.find(key); iter != m_publishers.end())
    {
        return std::static_pointer_cast<typename Topic::Publisher>(iter->second);
    }

    auto publisher = create_publisher<Topic>();
    m_publishers.emplace(key, publisher);
    return publisher;
}
