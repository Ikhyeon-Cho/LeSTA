/*
 * TraversabilityEstimator.cpp
 *
 *  Created on: Mar 09, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/core/TraversabilityEstimator.h"

namespace lesta {

TraversabilityEstimator::TraversabilityEstimator(const Config &cfg) : cfg(cfg) {

  // Load model checkpoint
  if (!network_.loadCheckpoint(cfg.model_path)) {
    std::cerr << "\033[1;31m[lesta::TraversabilityEstimator]: Failed to load "
                 "traversability network weights from "
              << cfg.model_path << "\033[0m" << std::endl;
    throw std::runtime_error("Failed to load traversability model weights");
  }

  // Set network input dimension
  network_.setInputDimension(cfg.input_dimension);

  // Make sure input_dimension matches the number of feature fields
  if (cfg.input_dimension != cfg.feature_fields.size()) {
    std::cout << "\033[1;33m[lesta::TraversabilityEstimator]: Input dimension ("
              << cfg.input_dimension << ") doesn't match number of feature fields ("
              << cfg.feature_fields.size() << "). Using feature fields count."
              << std::endl;
    network_.setInputDimension(cfg.feature_fields.size());
  }

  // Print feature lists
  std::cout << "\033[1;32m[lesta::TraversabilityEstimator]: Feature selection --> ";
  for (const auto &field : cfg.feature_fields) {
    std::cout << field << " ";
  }
  std::cout << "\033[0m" << std::endl;
}

void TraversabilityEstimator::estimateTraversability(HeightMap &map) {

  if (network_.inputDimension() < 0) {
    std::cerr
        << "\033[1;31m[lesta::TraversabilityEstimator]: Input dimension is not set\033[0m"
        << std::endl;
    throw std::runtime_error("Input dimension is not set");
  }

  // Collect all cell indices from the map
  std::vector<grid_map::Index> all_indices;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    all_indices.push_back(*it);
  }

  ensureTraversabilityLayers(map);
  estimateTraversabilityImpl(map, all_indices);
}

void TraversabilityEstimator::estimateTraversability(
    HeightMap &map,
    const std::vector<grid_map::Index> &measured_indices) {

  if (network_.inputDimension() < 0) {
    std::cerr
        << "\033[1;31m[lesta::TraversabilityEstimator]: Input dimension is not set\033[0m"
        << std::endl;
    throw std::runtime_error("Input dimension is not set");
  }

  ensureTraversabilityLayers(map);

  if (measured_indices.empty())
    return;

  estimateTraversabilityImpl(map, measured_indices);
}

void TraversabilityEstimator::estimateTraversabilityImpl(
    HeightMap &map,
    const std::vector<grid_map::Index> &measured_indices) {

  std::vector<Eigen::VectorXf> features;
  std::vector<grid_map::Index> valid_indices;
  features.reserve(measured_indices.size());
  valid_indices.reserve(measured_indices.size());

  for (const auto &index : measured_indices) {
    // Check if all required features are valid
    bool all_feature_values_valid = true;
    for (const auto &feature : cfg.feature_fields) {
      if (map.isEmptyAt(featurefield_to_layer_[feature], index)) {
        all_feature_values_valid = false;
        break;
      }
    }
    if (!all_feature_values_valid)
      continue;

    // Create feature vector with the correct dimension
    Eigen::VectorXf feature(cfg.feature_fields.size());

    // Fill feature vector based on configured fields
    for (size_t i = 0; i < cfg.feature_fields.size(); i++) {
      const std::string &field_name = cfg.feature_fields[i];
      feature(i) = map.at(featurefield_to_layer_[field_name], index);
    }

    features.push_back(std::move(feature));
    valid_indices.push_back(index);
  }

  // Perform batch inference if we have valid cells
  if (!features.empty()) {
    std::vector<float> predictions = network_.inference(features);

    // Update map with predictions
    for (size_t i = 0; i < valid_indices.size(); ++i) {
      const auto &index = valid_indices[i];
      float prob = sigmoid(predictions[i]);

      map.at(layers::Traversability::PROBABILITY, index) = prob;
      map.at(layers::Traversability::BINARY, index) =
          (prob >= cfg.binary_threshold)
              ? static_cast<float>(Traversability::TRAVERSABLE)
              : static_cast<float>(Traversability::NON_TRAVERSABLE);
    }
  }
}

void TraversabilityEstimator::ensureTraversabilityLayers(HeightMap &map) {

  map.addLayer(layers::Traversability::PROBABILITY);
  map.addLayer(layers::Traversability::BINARY);
}

} // namespace lesta
