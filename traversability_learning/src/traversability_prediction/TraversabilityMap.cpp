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
TraversabilityMap::TraversabilityMap(const HeightMap& map)
  : TraversabilityMap(map.getLength().x(), map.getLength().y(), map.getResolution())
{
  setFrameId(map.getFrameId());
}

TraversabilityMap::TraversabilityMap(double map_length_x, double map_length_y, double resolution)
  : HeightMap(map_length_x, map_length_y, resolution)
{
  setFrameId("map");
  setGeometry(grid_map::Length(map_length_x, map_length_y), resolution);

  // Basic layers for traversability map
  std::vector<std::string> traversability_layers{ "traversability_pred", "traversability_prob" };
  for (const auto& trav_layer : traversability_layers)
  {
    addLayer(trav_layer, (float)Traversability::UNKNOWN);
    std::cout << "[@ TraversabilityMap] Added " << trav_layer << " layer to the height map" << std::endl;
  }
  // setBasicLayers(traversability_layers);
}

void TraversabilityMap::update()
{
  const auto& height_matrix = getHeightMatrix();

  TraversabilityClassifier classifier;
  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    if (isEmptyAt(getHeightLayer(), *iterator))
      continue;

    if (!predict(*iterator, classifier))
      continue;

    recursiveUpdate(*iterator);
  }
}

bool TraversabilityMap::predict(const grid_map::Index& index, const TraversabilityClassifier& classifier)
{
  bool has_valid_feature_set = std::isfinite(at("step", index)) && std::isfinite(at("slope", index)) &&
                               std::isfinite(at("roughness", index)) && std::isfinite(at("curvature", index)) &&
                               std::isfinite(at("variance", index));
  if (!has_valid_feature_set)
    return false;

  auto& pred = at("traversability_pred", index);
  pred = classifier.classifyTraversabilityAt(*this, index);

  // Probability Clipping
  const float PROB_MAX = 0.95;
  const float PROB_MIN = 0.05;
  pred = std::max(std::min(pred, PROB_MAX), PROB_MIN);

  return true;
}

void TraversabilityMap::recursiveUpdate(const grid_map::Index& index)
{
  const auto& pred = at("traversability_pred", index);
  auto& prob = at("traversability_prob", index);

  // Initialize probability update
  if (!std::isfinite(prob))
  {
    prob = pred;
    return;
  }

  // Logarithmic probability update
  double log_pred = std::log(pred);
  double log_prob = std::log(prob);

  // Calculate the geometric mean in the log space
  log_prob = (log_prob * (10 - 1) + log_pred) / 10;

  // Probability Clipping
  const double LOG_PROB_MAX = std::log(0.95);
  const double LOG_PROB_MIN = std::log(0.05);
  log_prob = std::max(std::min(log_prob, LOG_PROB_MAX), LOG_PROB_MIN);

  // Revert the logarithmic probability
  prob = std::exp(log_prob);
}

}  // namespace grid_map