/*
 * FeatureExtraction.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <height_map_core/height_map_core.h>
#include <height_map_msgs/HeightMapConverter.h>
#include <height_map_msgs/HeightMapMsgs.h>

#include "feature_extraction/FeatureMap.h"

class FeatureExtraction
{
public:
  FeatureExtraction();

  void HeightMapCallback(const grid_map_msgs::GridMapConstPtr& msg);

private:
  ros::NodeHandle nh_priv_{ "~" };

  // Topics
  std::string heightmap_topic_{ nh_priv_.param<std::string>("heightMapTopic", "/height_mapping/map/gridmap") };
  // Frame Ids
  std::string baselink_frame{ nh_priv_.param<std::string>("/frame_id/base_link", "base_link") };
  std::string map_frame{ nh_priv_.param<std::string>("/frame_id/map", "map") };

  // Feature Extraction Parameters
  double normal_estimation_radius_{ nh_priv_.param<double>("normalEstimationRadius", 0.2) };

  // ROS
  ros::Subscriber sub_heightmap_{ nh_priv_.subscribe(heightmap_topic_, 10, &FeatureExtraction::HeightMapCallback,
                                                     this) };
  ros::Publisher pub_featuremap_{ nh_priv_.advertise<grid_map_msgs::GridMap>(
      "/traversability_estimation/features/gridmap", 1) };

private:
  // arbitrary values: real size will be determined by the input map
  grid_map::FeatureMap::Ptr map_{ std::make_shared<grid_map::FeatureMap>(10, 10, 0.1) };
};

#endif  // FEATURE_EXTRACTION_H