/*
 * TraversabilityLabelGeneration.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef LABEL_GENERATION_H
#define LABEL_GENERATION_H

#include <ros/ros.h>
#include "ros_utils/TransformHandler.h"
#include <sensor_msgs/PointCloud2.h>
#include <height_map_core/height_map_core.h>
#include <height_map_msgs/HeightMapConverter.h>
#include <height_map_msgs/HeightMapMsgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include "label_generation/Traversability.h"

class TraversabilityLabelGeneration
{
public:
  TraversabilityLabelGeneration();

  bool updateMeasuredIndices(const grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZ>& cloud);

  void featureCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void generateLabels(const ros::TimerEvent& event);

  void publishLabeledMap(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_{ "height_mapping" };
  ros::NodeHandle nh2_{ "feature_extraction" };
  ros::NodeHandle pnh_{ "~" };

  utils::TransformHandler tf_tree_;  // For pose update

  // Topics
  std::string feature_cloud_topic_{ nh2_.param<std::string>("featureMapCloudTopic", "/feature_cloud") };
  std::string label_cloud_topic_{ pnh_.param<std::string>("labelMapTopic", "/label_cloud") };

  // Frame Ids
  std::string baselink_frame{ nh_.param<std::string>("baseLinkFrame", "base_link") };
  std::string map_frame{ nh_.param<std::string>("mapFrame", "map") };

  // Height Map Parameters
  double grid_resolution_{ nh_.param<double>("gridResolution", 0.1) };   // meter per grid
  double map_length_x_{ nh_.param<double>("mapLengthXGlobal", 400.0) };  // meters
  double map_length_y_{ nh_.param<double>("mapLengthYGlobal", 400.0) };  // meters

  // Duration
  double footprint_recording_rate_{ pnh_.param<double>("footprintRecordingRate", 10) };  // Hz
  double visualization_rate_{ pnh_.param<double>("visualizationRate", 10) };             // Hz

  // Labeling Parameters
  double footprint_radius_{ pnh_.param<double>("footprintRadius", 0.5) };               // meters
  double max_acceptable_step_{ pnh_.param<double>("maxAcceptableTerrainStep", 0.1) };   // meters
  double max_acceptable_slope_{ pnh_.param<double>("maxAcceptableTerrainSlope", 20) };  // degrees

  // ROS
  ros::Subscriber sub_feature_cloud_{ nh2_.subscribe(feature_cloud_topic_, 10,
                                                     &TraversabilityLabelGeneration::featureCloudCallback, this) };
  // ros::Publisher pub_labelmap_{ pnh_.advertise<grid_map_msgs::GridMap>(label_cloud_topic_, 10) };
  ros::Publisher pub_label_cloud_{ pnh_.advertise<sensor_msgs::PointCloud2>(label_cloud_topic_, 10) };
  ros::Timer footprint_record_timer_{ nh_.createTimer(footprint_recording_rate_,
                                                      &TraversabilityLabelGeneration::generateLabels, this) };
  ros::Timer visualization_timer_{ nh_.createTimer(visualization_rate_,
                                                   &TraversabilityLabelGeneration::publishLabeledMap, this) };

private:
  void recordFootprintAt(const grid_map::Position& robot_position);

  void recordOverThresholdAreas();

  void recordUnknownAreas();

  // Since the map size is fixed, it is more efficient to assign memory before start
  grid_map::HeightMap labeled_map_{ map_length_x_, map_length_y_, grid_resolution_ };
  std::vector<grid_map::Index> measured_indices_;
};

#endif /* LABEL_GENERATION_H */