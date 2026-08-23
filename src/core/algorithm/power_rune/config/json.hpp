#pragma once

#include <filesystem>

#include "json/ReJson.hpp"

inline ReJson J_POWER_RUNE(
    (std::filesystem::canonical("/proc/self/exe").parent_path().parent_path() / "config" / "power_rune.json").string());
