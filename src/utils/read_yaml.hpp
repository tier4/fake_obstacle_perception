#pragma once

#include <yaml-cpp/yaml.h>

// User defined includes
#include "obstacle.hpp"

// C++ includes

namespace rviz_plugins
{

std::pair<bool, Obstacle> parseObstacleYAML(const std::string &filepath);
std::tuple<bool , std::string, int32_t, std::vector<std::string>> parsePresetYAML(const std::string &filepath);
}