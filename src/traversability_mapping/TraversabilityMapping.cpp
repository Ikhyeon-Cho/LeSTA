/*
 * TraversabilityMapping.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "traversability_mapping/TraversabilityMapping.h"

namespace ros
{
TraversabilityMapping::TraversabilityMapping()
{
  // timer start
}

void TraversabilityMapping::estimateTraversability(const grid_map_msgs::GridMapConstPtr& msg)
{
  // message conversion
  grid_map::GridMap elevation_map;
  grid_map::GridMapRosConverter::fromMessage(*msg, elevation_map);

  map_.updateFrom(elevation_map);

  // Predicted map visualization
}

}  // namespace ros