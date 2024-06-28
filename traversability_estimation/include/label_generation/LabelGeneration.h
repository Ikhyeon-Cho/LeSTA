/*
 * TraversabilityLabelGeneration.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TRAVERSABILITY_LABEL_GENERATION_H
#define TRAVERSABILITY_LABEL_GENERATION_H

#include <ros/ros.h>
#include "ros_utils/TransformHandler.h"
#include <sensor_msgs/PointCloud2.h>
#include <height_map_core/height_map_core.h>
#include <height_map_msgs/HeightMapConverter.h>
#include <height_map_msgs/HeightMapMsgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>

#include "label_generation/TraversabilityInfo.h"
#include "feature_extraction/FeatureMap.h"

class TraversabilityLabelGeneration
{
public:
  TraversabilityLabelGeneration() = default;

  void featureMapCallback(const grid_map_msgs::GridMapConstPtr& msg);

  void generateTraversabilityLabels();

  void generateTraversabilityLabels(const sensor_msgs::PointCloud2ConstPtr& cloud);

  bool saveLabeledData(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:
  ros::NodeHandle nh_{ "height_mapping" };
  ros::NodeHandle nh2_{ "feature_extraction" };
  ros::NodeHandle pnh_{ "~" };

  utils::TransformHandler tf_;  // For pose update

  std::string baselink_frame{ nh_.param<std::string>("baseLinkFrame", "base_link") };
  std::string map_frame{ nh_.param<std::string>("mapFrame", "map") };

  // Labeling Parameters
  double footprint_radius_{ pnh_.param<double>("footprintRadius", 0.5) };               // meters
  double max_acceptable_step_{ pnh_.param<double>("maxAcceptableTerrainStep", 0.1) };   // meters
  double max_acceptable_slope_{ pnh_.param<double>("maxAcceptableTerrainSlope", 20) };  // degrees

  // ROS
  ros::Subscriber sub_feature_map_{ nh_.subscribe("/traversability_estimation/features/gridmap", 1,
                                                  &TraversabilityLabelGeneration::featureMapCallback, this) };
  ros::Subscriber sub_globalmap_{ nh_.subscribe("/height_mapping/globalmap/pointcloud", 1,
                                                &TraversabilityLabelGeneration::generateTraversabilityLabels, this) };
  ros::Publisher pub_labelmap_{ pnh_.advertise<grid_map_msgs::GridMap>("/traversability_learning/label/gridmap", 10) };

  ros::ServiceServer labeled_csv_saver_{ pnh_.advertiseService("save_labeled_data",
                                                               &TraversabilityLabelGeneration::saveLabeledData, this) };

private:
  bool is_labelmap_initialized_{ false };

  bool is_labeling_callback_activated_{ false };

  void initializeLabelMap();

  void recordFootprint(grid_map::HeightMap& labelmap);

  void recordOverThresholdAreas(grid_map::HeightMap& labelmap);

  void recordUnknownAreas(grid_map::HeightMap& labelmap);

  void saveLabeledDataToCSV(const grid_map::HeightMap& labelmap);

  void saveUnlabeledDataToCSV(const grid_map::HeightMap& labelmap);

  grid_map::HeightMap::Ptr labelmap_;
  grid_map::GridMap featuremap_;
};

#endif /* TRAVERSABILITY_LABEL_GENERATION_H */