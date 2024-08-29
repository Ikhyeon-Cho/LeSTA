/*
 * LabelGeneration.h
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
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_set>

#include <height_map_core/height_map_core.h>
#include <height_map_msgs/HeightMapConverter.h>
#include <height_map_msgs/HeightMapMsgs.h>
#include <std_srvs/Empty.h>

#include "label_generation/TraversabilityInfo.h"

class LabelGeneration
{
public:
  LabelGeneration() = default;

  void terrainFeatureCallback(const grid_map_msgs::GridMapConstPtr& msg);

  void updateLabelmapFrom(const grid_map::HeightMap& featuremap);

  void generateTraversabilityLabels();

  void generateTraversabilityLabels(const sensor_msgs::PointCloud2ConstPtr& cloud);

  bool saveLabeledData(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  struct IndexHash
  {
    std::size_t operator()(const grid_map::Index& index) const
    {
      return std::hash<int>()(index.x()) ^ std::hash<int>()(index.y());
    }
  };
  struct IndexEqual
  {
    bool operator()(const grid_map::Index& a, const grid_map::Index& b) const
    {
      return a.x() == b.x() && a.y() == b.y();
    }
  };

private:
  ros::NodeHandle nh_{ "height_mapping" };
  ros::NodeHandle pnh_{ "~" };

  utils::TransformHandler tf_;  // For pose update

  std::string baselink_frame{ nh_.param<std::string>("frame_id/base_link", "base_link") };
  std::string map_frame{ nh_.param<std::string>("frame_id/map", "map") };

  // Labeling region
  double map_length_x_{ pnh_.param<double>("mapLengthX", 400) };
  double map_length_y_{ pnh_.param<double>("mapLengthY", 400) };

  // Labeling Parameters
  double footprint_radius_{ pnh_.param<double>("footprintRadius", 0.5) };              // meters
  double max_acceptable_step_{ pnh_.param<double>("maxAcceptableTerrainStep", 0.1) };  // meters
  std::string labeled_data_path_{ pnh_.param<std::string>("pathToLabeledData", "/home/ikhyeon/Downloads") };
  std::string unlabeled_data_path_{ pnh_.param<std::string>("pathToUnlabeledData", "/home/ikhyeon/Downloads") };

  // ROS
  ros::Subscriber sub_feature_map_{ pnh_.subscribe("/lesta/feature/gridmap", 1,
                                                   &LabelGeneration::terrainFeatureCallback, this) };
  ros::Publisher pub_labelmap_local_{ pnh_.advertise<grid_map_msgs::GridMap>("/lesta/label/gridmap", 10) };
  ros::Publisher pub_labelmap_global_{ pnh_.advertise<sensor_msgs::PointCloud2>("/lesta/label/cloud_global", 10) };
  ros::Publisher pub_labelmap_region_{ pnh_.advertise<visualization_msgs::Marker>("/lesta/label/map_region", 10) };

  // ros::ServiceServer
  ros::ServiceServer labeled_csv_saver_{ pnh_.advertiseService("save_labeled_data", &LabelGeneration::saveLabeledData,
                                                               this) };

private:
  bool is_labelmap_initialized_{ false };

  bool is_labeling_callback_working_{ false };

  void initializeLabelMap(const grid_map::HeightMap& featuremap);

  // Note: grid_map_msgs are not visualizable in large scale map -> use PointCloud2 instead
  void visualizeLabelMap();

  void toPointCloud2(const grid_map::HeightMap& map, const std::vector<std::string>& layers,
                     const std::unordered_set<grid_map::Index, IndexHash, IndexEqual>& measured_indices,
                     sensor_msgs::PointCloud2& cloud);

  void visualizeLabelSubmap(const grid_map::Length& length);

  void recordFootprints();

  void recordNegativeLabel();

  void recordUnknownAreas(grid_map::HeightMap& labelmap);

  void saveLabeledDataToCSV(const grid_map::HeightMap& labelmap);

  void saveUnlabeledDataToCSV(const grid_map::HeightMap& labelmap);

  // grid_map::HeightMap::Ptr labelmap_;
  grid_map::HeightMap::Ptr featuremap_ptr_{ nullptr };
  grid_map::HeightMap labelmap_{ map_length_x_, map_length_y_, 0.1 };
  std::unordered_set<grid_map::Index, IndexHash, IndexEqual> valid_indices_;
};

#endif /* TRAVERSABILITY_LABEL_GENERATION_H */