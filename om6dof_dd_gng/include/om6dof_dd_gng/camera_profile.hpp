// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace om6dof_dd_gng
{

struct CameraProfile
{
  std::string version;
  std::string model;
  std::string depth_frame;
};

inline std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return value;
}

inline CameraProfile cameraProfile(
  const std::string & version, const std::string & model = "auto",
  const std::string & frame = "auto")
{
  if (version != "v1" && version != "v2") {
    throw std::invalid_argument("model_version must be v1 or v2");
  }
  std::string selected = model == "auto" ? (version == "v2" ? "D435I" : "D405") : upper(model);
  if ((version == "v1" && selected != "D405") ||
    (version == "v2" && selected != "D435" && selected != "D435I"))
  {
    throw std::invalid_argument("camera_model does not match model_version");
  }
  const std::string expected = version == "v2" ? "d435_depth_optical_frame" : "d405_depth_optical_frame";
  if (frame != "auto" && frame != expected) {
    throw std::invalid_argument("camera_frame does not match the selected model's native depth frame");
  }
  return {version, selected == "D435I" ? "D435i" : selected, expected};
}

struct CameraIdentity
{
  std::string name;
  std::string serial;
};

inline bool cameraNameMatches(const std::string & name, const std::string & model)
{
  const auto full = upper(name);
  const auto token = upper(model);
  size_t pos = full.find(token);
  const auto word = [](unsigned char c) {return std::isalnum(c) || c == '_';};
  while (pos != std::string::npos) {
    const auto end = pos + token.size();
    if ((pos == 0 || !word(full[pos - 1])) && (end == full.size() || !word(full[end]))) {
      return true;
    }
    pos = full.find(token, pos + 1);
  }
  return false;
}

inline std::string selectCameraSerial(
  const std::vector<CameraIdentity> & devices, const CameraProfile & profile,
  const std::string & requested = "")
{
  std::vector<std::string> matches;
  for (const auto & device : devices) {
    if (cameraNameMatches(device.name, profile.model) && !device.serial.empty() &&
      (requested.empty() || requested == device.serial))
    {
      matches.push_back(device.serial);
    }
  }
  if (matches.empty()) {
    throw std::runtime_error("No " + profile.model + " matches camera_serial; check the connected model");
  }
  if (matches.size() != 1) {
    throw std::runtime_error("Multiple " + profile.model + " cameras: set camera_serial");
  }
  return matches.front();
}

inline bool freshCapture(int64_t stamp, int64_t now, double max_age_sec)
{
  return stamp > 0 && now >= stamp && std::isfinite(max_age_sec) && max_age_sec > 0 &&
         static_cast<double>(now - stamp) / 1e9 <= max_age_sec;
}

}  // namespace om6dof_dd_gng
