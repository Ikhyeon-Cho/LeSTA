/*
 * TraversabilityMapper.cpp
 *
 *  Created on: Feb 15, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/core/TraversabilityMapper.h"

namespace lesta {

TraversabilityMapper::TraversabilityMapper(const Config &cfg)
    : TraversabilityEstimator(cfg), cfg(cfg) {

  std::cout << "\033[1;32m[lesta::TraversabilityMapper]: Log odds update parameters --> "
            << " learning_rate=" << cfg.learning_rate << " prob_min=" << cfg.prob_min
            << " prob_max=" << cfg.prob_max << "\033[0m" << std::endl;
}

void TraversabilityMapper::traversabilityMapping(
    HeightMap &map,
    const std::vector<grid_map::Index> &measured_indices) {

  // 1. Update the prediction layers
  estimateTraversability(map, measured_indices);

  // 2. Update the mapping layers
  ensureTraversabilityMappingLayers(map);
  if (cfg.use_log_odds_update)
    traversabilityMappingImpl(map, measured_indices);
}

void TraversabilityMapper::ensureTraversabilityMappingLayers(HeightMap &map) {

  map.addLayer(layers::Traversability::LOG_ODDS, 0.0f); // 50% probability
  map.addLayer(layers::Traversability::LOG_ODDS_PROBABILITY, 0.0f);
  map.addLayer(layers::Traversability::LOG_ODDS_BINARY);
}

void TraversabilityMapper::traversabilityMappingImpl(
    HeightMap &map,
    const std::vector<grid_map::Index> &indices) {

  for (const auto &index : indices) {
    if (map.isEmptyAt(layers::Traversability::PROBABILITY, index)) {
      continue;
    }

    // Get model prediction probability and clamp it
    float observation_prob = map.at(layers::Traversability::PROBABILITY, index);
    observation_prob = clampValue(observation_prob, 0.2, 0.8);
    float observation_log_odds = log_odds(observation_prob);
    const auto &current_log_odds = map.at(layers::Traversability::LOG_ODDS, index);

    float updated_log_odds =
        current_log_odds + (cfg.learning_rate * observation_log_odds);

    // Clamp log odds to configured min/max values
    updated_log_odds =
        clampValue(updated_log_odds, log_odds(cfg.prob_min), log_odds(cfg.prob_max));

    // Convert back to probability for visualization
    const auto &updated_prob = sigmoid(updated_log_odds);

    // Update map layers with the log odds-based probability and binary classification
    map.at(layers::Traversability::LOG_ODDS, index) = updated_log_odds;
    map.at(layers::Traversability::LOG_ODDS_PROBABILITY, index) = updated_prob;
    map.at(layers::Traversability::LOG_ODDS_BINARY, index) =
        (updated_prob >= cfg.binary_threshold)
            ? static_cast<float>(Traversability::TRAVERSABLE)
            : static_cast<float>(Traversability::NON_TRAVERSABLE);
  }
}

} // namespace lesta
