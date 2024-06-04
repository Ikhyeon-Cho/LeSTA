/*
 * TraversabilityMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TRAVERSABILITY_MAP_H
#define TRAVERSABILITY_MAP_H

#include <height_map_core/height_map_core.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include "label_generation/Traversability.h"

class TraversabilityClassifier;

namespace grid_map
{
class TraversabilityMap : public HeightMap
{
public:
  using Ptr = std::shared_ptr<TraversabilityMap>;

  TraversabilityMap(const HeightMap& map);
  TraversabilityMap(double map_length_x, double map_length_y, double resolution);
  bool initializeFrom(const HeightMap& map);

  void update();

private:
  bool predict(const grid_map::Index& index, const TraversabilityClassifier& classifier);
  void recursiveUpdate(const grid_map::Index& index);
};
}  // namespace grid_map

class TraversabilityClassifier
{
public:
  TraversabilityClassifier() = default;

  float classifyTraversabilityAt(const grid_map::TraversabilityMap& map, const grid_map::Index& index) const
  {
    const auto& step = map.at("step", index);
    const auto& slope = map.at("slope", index);
    const auto& roughness = map.at("roughness", index);
    const auto& curvature = map.at("curvature", index);
    const auto& variance = map.at("variance", index);

    // [1] TODO: Bring MLP classifier here

    // [2] Comparison Method: Rule-based classifier -> use slope
    float min_slope = 0;
    float max_slope = 30;
    // scaled slope: 1 for 0, 0 for 30
    auto scaled_slope = (slope - min_slope) / (max_slope - min_slope);
    if (scaled_slope < 0)
      scaled_slope = 0;
    if (scaled_slope > 1)
      scaled_slope = 1;

    return 1 - scaled_slope;
  }
};

#endif  // TRAVERSABILITY_MAP_H