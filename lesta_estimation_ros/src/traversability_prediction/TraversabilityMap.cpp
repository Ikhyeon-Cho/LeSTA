/*
 * TraversabilityMap.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "traversability_prediction/TraversabilityMap.h"

namespace grid_map
{
TraversabilityMap::TraversabilityMap(double map_length_x, double map_length_y, double resolution)
  : HeightMap(map_length_x, map_length_y, resolution)
{
  // Add layers for traversability map
  for (const auto& trav_layer : traversability_layers_)
  {
    addLayer(trav_layer, (float)Traversability::TRAVERSABLE);
    std::cout << "[LeSTA @TraversabilityMap] Added " << trav_layer << " layer to the height map" << std::endl;
  }
}
TraversabilityMap::TraversabilityMap(const HeightMap& map)
  : TraversabilityMap(map.getLength().x(), map.getLength().y(), map.getResolution())
{
}

bool TraversabilityMap::loadTraversabilityModel(const std::string& model_path)
{
  bool success = classifier_->loadModel(model_path);
  if (success)
  {
    is_classifier_loaded_ = true;
    std::cout << "[LeSTA @TraversabilityMap] Loaded classifier model from " << model_path << std::endl;
  }
  else
  {
    std::cerr << "[LeSTA @TraversabilityMap] Failed to load classifier model from " << model_path << std::endl;
  }
  return success;
}

void TraversabilityMap::estimateTraversability()
{
  if (!is_classifier_loaded_)
  {
    std::cerr << "[LeSTA @TraversabilityMap] Classifier model is not loaded. Skip traversability estimation." << std::endl;
    return;
  }

  maskInvalidCells();

  getValidInputs();

  inference();

  mapping();
}

void TraversabilityMap::maskInvalidCells()
{
  if (validity_mask_.size() != getSize().prod())
  {
    validity_mask_.resize(getSize().prod(), CellValidity::VALID);
    valid_inputs_.reserve(getSize().prod());
  }

  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    auto& grid_index = iterator.getLinearIndex();
    if (isEmptyAt(getHeightLayer(), *iterator))
    {
      validity_mask_[grid_index] = CellValidity::INVALID;
    }
    else if (!std::isfinite(at("step", *iterator)))
    {
      validity_mask_[grid_index] = CellValidity::INVALID;
    }
    else if (!std::isfinite(at("slope", *iterator)))
    {
      validity_mask_[grid_index] = CellValidity::INVALID;
    }
    else if (!std::isfinite(at("roughness", *iterator)))
    {
      validity_mask_[grid_index] = CellValidity::INVALID;
    }
    else if (!std::isfinite(at("curvature", *iterator)))
    {
      validity_mask_[grid_index] = CellValidity::INVALID;
    }
    else if (!std::isfinite(at("variance", *iterator)))
    {
      validity_mask_[grid_index] = CellValidity::INVALID;
    }
    else
    {
      validity_mask_[grid_index] = CellValidity::VALID;
    }
  }
}

void TraversabilityMap::getValidInputs()
{
  valid_inputs_.clear();

  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    const auto& grid_index = iterator.getLinearIndex();
    if (validity_mask_[grid_index] == CellValidity::VALID)
    {
      std::vector<float> input(5);
      input[0] = at("step", *iterator);
      input[1] = at("slope", *iterator);
      input[2] = at("roughness", *iterator);
      input[3] = at("curvature", *iterator);
      input[4] = at("variance", *iterator);
      valid_inputs_.push_back(input);
    }
  }
}

void TraversabilityMap::inference()
{
  std::vector<float> predictions = classifier_->inference(valid_inputs_);

  int valid_prediction_index = 0;
  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    const auto& grid_index = iterator.getLinearIndex();
    if (validity_mask_[grid_index] == CellValidity::INVALID)
      continue;

    const auto& traversability_prediction = predictions[valid_prediction_index++];
    at("traversability_prediction", *iterator) = traversability_prediction;
  }
}

void TraversabilityMap::mapping()
{
  const float TRAVERSABILITY_PRIOR = 0.8;
  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    if (!std::isfinite(at("traversability_prediction", *iterator)))
      continue;

    const auto& likelihood = at("traversability_prediction", *iterator);
    auto& probability = at("traversability_mapping", *iterator);

    // Initialize probability update
    if (std::isnan(probability))
    {
      probability = TRAVERSABILITY_PRIOR;
      continue;
    }

    // log-odds probability update
    auto index = iterator.getLinearIndex();
    // if (index == 10000)
    probability = logOddsUpdate(probability, likelihood);
  }
}

float TraversabilityMap::logOddsUpdate(float prior, float likelihood)
{
  // Convert probability to log-odds
  float log_prior = std::log(prior / (1 - prior));
  float log_likelihood_ratio = std::log(likelihood / (1 - likelihood));
  float log_posterior = log_prior + log_likelihood_ratio;

  // Probability Clipping
  const float LOG_PROB_MAX = std::log(0.80 / (1 - 0.80));  // 0.80
  const float LOG_PROB_MIN = std::log(0.20 / (1 - 0.20));  // 0.20
  log_likelihood_ratio = std::max(std::min(log_likelihood_ratio, LOG_PROB_MAX), LOG_PROB_MIN);
  log_posterior = std::max(std::min(log_posterior, LOG_PROB_MAX), LOG_PROB_MIN);

  // std::cout << "prior: " << prior << std::endl;
  // std::cout << "likelihood2: " << 1 / (1 + std::exp(-log_likelihood_ratio)) << std::endl;
  // std::cout << "posterior: " << 1 / (1 + std::exp(-log_posterior)) << std::endl;

  // Convert log-odds back to probability
  return 1 / (1 + std::exp(-log_posterior));
}
}  // namespace grid_map