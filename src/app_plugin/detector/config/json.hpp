#pragma once

#include <filesystem>

#include "json/ReJson.hpp"

inline const std::filesystem::path DETECTOR_CONFIG_DIR =
    std::filesystem::path(__FILE__).parent_path();
inline ReJson J_DETECT((DETECTOR_CONFIG_DIR / "detect.json").string());
