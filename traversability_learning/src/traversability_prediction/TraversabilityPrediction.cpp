/*
 * TraversabilityPrediction.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "traversability_prediction/TraversabilityPrediction.h"

void TraversabilityPrediction::featureCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  HeightMapConverter::fromPointCloud2(*msg, map_);

  map_.update();

  // Publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, map_.getLayers(), message);
  pub_map.publish(message);
}

void TraversabilityPrediction::updateMapPosition(const ros::TimerEvent& event)
{
  // Get Transform from base_link to map (localization pose)
  auto [get_transform_b2m, base_to_map] = tf_tree_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
    return;

  // Update map position
  map_.move(grid_map::Position(base_to_map.transform.translation.x, base_to_map.transform.translation.y));
}