/*
 * TraversabilityMapping.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TRAVERSABILITY_MAPPING_TRAVERSABILITY_MAPPING_H
#define TRAVERSABILITY_MAPPING_TRAVERSABILITY_MAPPING_H

#include <ros/ros.h>
#include <isr_ros_utils/core/core.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "traversability_mapping/TraversabilityMap.h"

namespace ros
{
class TraversabilityMapping
{
public:
  TraversabilityMapping();

  void estimateTraversability(const grid_map_msgs::GridMapConstPtr& msg);

public:
  // ROS Parameters: Node
  roscpp::Parameter<std::string> elevation_map_topic{ "terrain_mapping/SubscribedTopic/descriptor_map",
                                                      "map" };  // Check this when has no action
  roscpp::Parameter<std::string> traversability_map_topic{ "traversability_mapping/PublishingTopic/traversability_map",
                                                           "map" };

  // ROS Parameters : Framd Ids
  roscpp::Parameter<std::string> frameId_robot{ "frameId_robot", "base_link" };
  roscpp::Parameter<std::string> frameId_map{ "frameId_map", "map" };

  // Traversability Map
  roscpp::Subscriber<grid_map_msgs::GridMap> elevation_map_subscriber{ elevation_map_topic.param(),
                                                                      &TraversabilityMapping::estimateTraversability, this };

private:
  TraversabilityMap map_;
};
}  // namespace ros

#endif  // TRAVERSABILITY_MAPPING_TRAVERSABILITY_MAPPING_H