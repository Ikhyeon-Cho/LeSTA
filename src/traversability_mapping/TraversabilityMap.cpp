/*
 * TraversabilityMap.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "traversability_mapping/TraversabilityMap.h"
#include <iostream>

TraversabilityMap::TraversabilityMap() : GridMap({ "elevation", "traversability_raw", "traversability_prob" })
{
  setBasicLayers({ "elevation" });
}

void TraversabilityMap::updateFrom(const grid_map::GridMap& elevation_map)
{
  // Create map with fixed geometry
  setFrameId(elevation_map.getFrameId());
  setGeometry(elevation_map.getLength(), elevation_map.getResolution());
  this->move(elevation_map.getPosition());

  // Copy elevation and terrain feature layer
  get("elevation") = grid_map::GridMap::Matrix(elevation_map.get("elevation"));
  add("step", elevation_map.get("step"));
  add("slope", elevation_map.get("slope"));
  add("roughness", elevation_map.get("roughness"));
  add("curvature", elevation_map.get("curvature"));
  add("variance", elevation_map.get("variance"));

  const auto& step_layer = get("step");
  const auto& slope_layer = get("slope");
  const auto& roughness_layer = get("roughness");
  const auto& curvature_layer = get("curvature");
  const auto& variance_layer = get("variance");

  auto& traversabilityRaw_layer = get("traversability_raw");
  auto& traversabilityProb_layer = get("traversability_prob");

  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    if (!isValid(*iterator))
      continue;

    const auto& index = *iterator;

    const auto& step = step_layer(index(0), index(1));
    const auto& slope = slope_layer(index(0), index(1));
    const auto& roughness = roughness_layer(index(0), index(1));
    const auto& curvature = curvature_layer(index(0), index(1));
    const auto& variance = variance_layer(index(0), index(1));

    Eigen::VectorXf feature(5);
    feature << step, slope, roughness, curvature, variance;

    traversabilityRaw_layer(index(0), index(1)) = estimateTraversability(feature);
    // traversabilityProb_layer(index(0), index(1)) = 3;
  }

  erase("step");
  erase("slope");
  erase("roughness");
  erase("curvature");
}

float TraversabilityMap::estimateTraversability(const Eigen::VectorXf& feature)
{
  auto step(feature(0)), slope(feature(1)), roughness(feature(2)), curvature(feature(3)), variance(feature(4));

  if (feature.hasNaN())
    return NAN;

  // Baseline method (for comparison)
  // Refer to: https://ieeexplore.ieee.org/abstract/document/7759199
  return 0.2 * getLinearStepRatio(step) + 0.8 * getLinearSlopeRatio(slope);
}

float TraversabilityMap::getLinearStepRatio(float step)
{
  if (step > max_traversable_step_)
    return NON_TRAVERSABLE;

  return (max_traversable_step_ - step) / (max_traversable_step_);
}

float TraversabilityMap::getLinearSlopeRatio(float slope)
{
  if (slope > max_traversable_slope_)
    return NON_TRAVERSABLE;

  return (max_traversable_slope_ - slope) / (max_traversable_slope_);
}
