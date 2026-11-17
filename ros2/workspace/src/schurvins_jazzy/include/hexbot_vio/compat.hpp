#pragma once
#include <functional>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <yaml-cpp/yaml.h>

namespace hexbot_vio {
template<class T>
T param(const YAML::Node& node, const std::string& key,
        const T& fallback, bool = false) {
  return node[key] ? node[key].as<T>() : fallback;
}
}
