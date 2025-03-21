#pragma once

namespace lesta {
namespace layers {

struct Feature {
  static constexpr const char *STEP = "feature/step";
  static constexpr const char *SLOPE = "feature/slope";
  static constexpr const char *ROUGHNESS = "feature/roughness";
  static constexpr const char *CURVATURE = "feature/curvature";
  static constexpr const char *NORMAL_X = "feature/normal_x";
  static constexpr const char *NORMAL_Y = "feature/normal_y";
  static constexpr const char *NORMAL_Z = "feature/normal_z";
};

struct Label {
  static constexpr const char *FOOTPRINT = "label/footprint";
  static constexpr const char *TRAVERSABILITY = "label/traversability";
};

struct Traversability {
  static constexpr const char *PROBABILITY = "traversability/probability";
  static constexpr const char *BINARY = "traversability/binary";
  static constexpr const char *LOG_ODDS = "mapping/log_odds";
  static constexpr const char *LOG_ODDS_PROBABILITY = "mapping/probability";
  static constexpr const char *LOG_ODDS_BINARY = "mapping/binary";
};

} // namespace layers
} // namespace lesta