#pragma once

#include <filesystem>

#include "json/ReJson.hpp"

inline const std::filesystem::path CAMERA_CONFIG_DIR =
    std::filesystem::path(__FILE__).parent_path();
inline ReJson J_CAMERA((CAMERA_CONFIG_DIR / "camera.json").string());
