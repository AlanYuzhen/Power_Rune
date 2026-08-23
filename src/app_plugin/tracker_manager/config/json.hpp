#pragma once

#include <filesystem>

#include "json/ReJson.hpp"

inline const std::filesystem::path TRACKER_CONFIG_DIR =
    std::filesystem::path(__FILE__).parent_path();
inline ReJson J_TRACK((TRACKER_CONFIG_DIR / "track.json").string());
