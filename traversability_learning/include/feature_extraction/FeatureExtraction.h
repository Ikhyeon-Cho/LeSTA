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

#include "ros_utils/TransformHandler.h"

class FeatureExtraction
{
public:
  FeatureExtraction() = default;

  void elevationCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void updateMapPosition(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_{ "height_mapping" };
  ros::NodeHandle pnh_{ "~" };

  utils::TransformHandler tf_tree_;  // For pose update

  // Topics
  std::string elevation_cloud_topic_{ nh_.param<std::string>("heightMapCloudTopic", "/elevation_cloud") };
  std::string featuremap_topic_{ pnh_.param<std::string>("featureMapTopic", "/feature_grid") };

  // Frame Ids
  std::string baselink_frame{ nh_.param<std::string>("baseLinkFrame", "base_link") };
  std::string map_frame{ nh_.param<std::string>("mapFrame", "map") };

  // Height Map Parameters
  double grid_resolution_{ nh_.param<double>("gridResolution", 0.1) };
  double map_length_x_{ nh_.param<double>("mapLengthX", 10.0) };
  double map_length_y_{ nh_.param<double>("mapLengthY", 10.0) };

  // Duration
  double pose_update_rate_{ nh_.param<double>("poseUpdateRate", 20) };

  // Feature Extraction Parameters
  double local_patch_radius_{ pnh_.param<double>("localPatchRadius", 0.2) };

  // ROS
  ros::Subscriber sub_elevation_cloud_{ nh_.subscribe(elevation_cloud_topic_, 10,
                                                      &FeatureExtraction::elevationCloudCallback, this) };
  ros::Publisher pub_map{ pnh_.advertise<grid_map_msgs::GridMap>(featuremap_topic_, 1) };
  ros::Timer pose_update_timer_{ nh_.createTimer(pose_update_rate_, &FeatureExtraction::updateMapPosition, this) };

private:
  // Since the map size is fixed, it is more efficient to assign memory before start
  grid_map::HeightMap map_{ map_length_x_, map_length_y_, grid_resolution_ };
  grid_map::FeatureMap feature_map_{ map_length_x_, map_length_y_, grid_resolution_ };
};

#endif  // FEATURE_EXTRACTION_H