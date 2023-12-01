/*
 * TraversabilityMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TRAVERSABILITY_MAPPING_TRAVERSABILITY_MAP_H
#define TRAVERSABILITY_MAPPING_TRAVERSABILITY_MAP_H

#include <grid_map_core/grid_map_core.hpp>

class TraversabilityMap : public grid_map::GridMap
{
  const int NON_TRAVERSABLE = 0;
  const int TRAVERSABLE = 1;

public:
  TraversabilityMap();

  void updateFrom(const grid_map::GridMap& map);

private:
  float estimateTraversability(const Eigen::VectorXf& feature);

  float getLinearStepRatio(float step);
  float getLinearSlopeRatio(float slope);

  float max_traversable_step_{ 0.2 };    // [m]
  float max_traversable_slope_{ 30.0 };  // [degree]
};

#endif
