/*
 * TraversabilityEstimator.h
 *
 *  Created on: Mar 09, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "lesta/core/TraversabilityNetwork.h"
#include "lesta/types/traversability.h"
#include "lesta/types/layer_definitions.h"

#include <height_mapping_core/height_mapping_core.h>

namespace lesta {

class TraversabilityEstimator {
public:
  struct Config {
    std::string model_path;
    int input_dimension;
    float binary_threshold;
    std::vector<std::string> feature_fields;
  } cfg;

  using Traversability = lesta_types::Traversability;

  TraversabilityEstimator(const Config &cfg);

  void estimateTraversability(HeightMap &map);
  void estimateTraversability(HeightMap &map,
                              const std::vector<grid_map::Index> &measured_indices);
  const TraversabilityNetwork &getModel() const { return network_; }

  void ensureTraversabilityLayers(HeightMap &map);

  // Numerically stable sigmoid function
  inline float sigmoid(float x) const {
    if (x >= 0) {
      return 1.0f / (1.0f + std::exp(-x));
    } else {
      float exp_x = std::exp(x);
      return exp_x / (1.0f + exp_x);
    }
  }

private:
  void estimateTraversabilityImpl(HeightMap &map,
                                  const std::vector<grid_map::Index> &indices);

  TraversabilityNetwork network_;
  std::map<std::string, std::string> featurefield_to_layer_ = {
      {"step", layers::Feature::STEP},
      {"slope", layers::Feature::SLOPE},
      {"roughness", layers::Feature::ROUGHNESS},
      {"curvature", layers::Feature::CURVATURE},
      {"elevation_variance", height_mapping::layers::Height::ELEVATION_VARIANCE},
  };
};

} // namespace lesta
