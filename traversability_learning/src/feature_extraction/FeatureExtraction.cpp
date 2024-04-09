/*
 * FeatureExtraction.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "feature_extraction/FeatureExtraction.h"

void FeatureExtraction::elevationCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  HeightMapConverter::fromPointCloud2(*msg, feature_map_);
  // if (!feature_map_.initializeFrom(map_))
  // {
    // ROS_WARN("Invalid elevation cloud. Skip this cloud.");
    // return;
  // }

  feature_map_.update(local_patch_radius_);

  // Publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(feature_map_, message);
  pub_map.publish(message);
}

void FeatureExtraction::updateMapPosition(const ros::TimerEvent& event)
{
  // Get Transform from base_link to map (localization pose)
  auto [get_transform_b2m, base_to_map] = tf_tree_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
    return;

  // Update map position
  map_.move(grid_map::Position(base_to_map.transform.translation.x, base_to_map.transform.translation.y));
  feature_map_.move(grid_map::Position(base_to_map.transform.translation.x, base_to_map.transform.translation.y));
}