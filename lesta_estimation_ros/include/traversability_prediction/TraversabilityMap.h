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
#include "label_generation/TraversabilityInfo.h"
#include "traversability_prediction/classifiers/LearnedTraversability.h"

namespace grid_map
{
class TraversabilityMap : public HeightMap
{
public:
  using Ptr = std::shared_ptr<TraversabilityMap>;

  TraversabilityMap(const HeightMap& map);
  TraversabilityMap(double map_length_x, double map_length_y, double resolution);

  bool loadTraversabilityModel(const std::string& model_path);

  void estimateTraversability();

private:
  void maskInvalidCells();

  void getValidInputs();

  void inference();

  void mapping();

  float logOddsUpdate(float prior, float likelihood);

  enum CellValidity
  {
    INVALID = 0,
    VALID = 1
  };

  std::vector<std::string> traversability_layers_{ "traversability_prediction", "traversability_mapping" };
  TraversabilityClassifierBase::Ptr classifier_{ std::make_unique<LearnedTraversability>() };
  bool is_classifier_loaded_{ false };
  std::vector<int> validity_mask_;
  std::vector<std::vector<float>> valid_inputs_;
};
}  // namespace grid_map

#endif  // TRAVERSABILITY_MAP_H