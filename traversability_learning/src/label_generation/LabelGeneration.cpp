/*
 * TraversabilityLabelGeneration.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "label_generation/LabelGeneration.h"

TraversabilityLabelGeneration::TraversabilityLabelGeneration()
{
  labeled_map_.setFrameId(map_frame);
  labeled_map_.setPosition(grid_map::Position(labeled_map_.getLength().x() / 2, labeled_map_.getLength().y() / 2));

  labeled_map_.addLayer("traversability_label");
  std::cout << "[@ LabelMap] Added traversability_label layer to the label map \n";
}

void TraversabilityLabelGeneration::featureCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Convert PointCloud2 to PCL
  auto feature_cloud_pcl = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *feature_cloud_pcl);

  // Update measured indices -> make large-scale visualization process more efficient
  if (!updateMeasuredIndices(labeled_map_, *feature_cloud_pcl))
  {
    ROS_WARN("Feature cloud is empty. Skip this cloud.");
    return;
  }

  // Convert PointCloud2 to HeightMap with features
  HeightMapConverter::fromPointCloud2(*msg, labeled_map_);
}

bool TraversabilityLabelGeneration::updateMeasuredIndices(const grid_map::HeightMap& map,
                                                          const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  if (cloud.empty())
    return false;

  // Save measured indices for efficient visualization
  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (auto& point : cloud.points)
  {
    // Skip if the point is out of the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    if (!map.isEmptyAt(cell_index))
      continue;

    measured_indices_.push_back(cell_index);
  }

  return true;
}

void TraversabilityLabelGeneration::generateLabels(const ros::TimerEvent& event)
{
  // Get Transform from base_link to map (localization pose)
  auto [get_transform_b2m, base_to_map] = tf_tree_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
    return;

  // Record the footprints (positive label)
  grid_map::Position robot_position(base_to_map.transform.translation.x, base_to_map.transform.translation.y);
  recordFootprintAt(robot_position);

  // Record the over-threshold areas (negative label)
  recordOverThresholdAreas();
}

void TraversabilityLabelGeneration::recordFootprintAt(const grid_map::Position& footprint_position)
{
  for (grid_map::CircleIterator iterator(labeled_map_, footprint_position, footprint_radius_); !iterator.isPastEnd();
       ++iterator)
  {
    Eigen::Vector3d point;
    if (!labeled_map_.getPosition3(labeled_map_.getHeightLayer(), *iterator, point))
      continue;

    labeled_map_.at("traversability_label", *iterator) = TRAVERSABLE;
  }  // circle iterator ends
}

void TraversabilityLabelGeneration::recordOverThresholdAreas()
{
  for (auto index : measured_indices_)
  {
    // Skip if the cell is already labeled, else relabel
    if (labeled_map_.at("traversability_label", index) == TRAVERSABLE)
      continue;

    // Negative labeling
    if (labeled_map_.at("slope", index) > max_acceptable_slope_ &&
        labeled_map_.at("step", index) > max_acceptable_step_)
      labeled_map_.at("traversability_label", index) = NON_TRAVERSABLE;
    else
      labeled_map_.at("traversability_label", index) = UNKNOWN;
  }
}

void TraversabilityLabelGeneration::publishLabeledMap(const ros::TimerEvent& event)
{
  // Publish the labeled cloud
  sensor_msgs::PointCloud2 labeled_cloud_msg;
  auto layer_to_visualize =
      std::vector<std::string>{ "elevation", "step", "slope", "roughness", "curvature", "variance", "traversability_label" };
  HeightMapConverter::toPointCloud2(labeled_map_, layer_to_visualize, measured_indices_, labeled_cloud_msg);
  pub_label_cloud_.publish(labeled_cloud_msg);
}