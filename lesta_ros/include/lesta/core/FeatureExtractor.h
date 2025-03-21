/*
 * FeatureExtractor.h
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>
#include "lesta/types/layer_definitions.h"

namespace lesta {

class FeatureExtractor {
public:
  struct Config {
    double pca_radius;
  } cfg;

  FeatureExtractor(const Config &cfg);

  void extractFeatures(HeightMap &map);
  void extractFeatures(HeightMap &map,
                       const std::vector<grid_map::Index> &measured_indices);

private:
  void ensureFeatureLayers(HeightMap &map);
};

} // namespace lesta
