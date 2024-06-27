/*
 * TraversabilityLabelGeneration.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "label_generation/LabelGeneration.h"

void TraversabilityLabelGeneration::featureMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, featuremap_);
  if (!is_labelmap_initialized_)
  {
    initializeLabelMap();
    is_labelmap_initialized_ = true;
  }

  // Align the label map with the feature map
  labelmap_->move(featuremap_.getPosition());

  // Copy the feature map values
  labelmap_->getHeightMatrix() = featuremap_["elevation"];
  labelmap_->getVarianceMatrix() = featuremap_["variance"];
  labelmap_->get("step") = featuremap_["step"];
  labelmap_->get("slope") = featuremap_["slope"];
  labelmap_->get("roughness") = featuremap_["roughness"];
  labelmap_->get("curvature") = featuremap_["curvature"];

  generateTraversabilityLabels();

  // Publish the labeled map
  grid_map_msgs::GridMap labelmap_msg;
  grid_map::GridMapRosConverter::toMessage(*labelmap_, labelmap_msg);
  pub_labelmap_.publish(labelmap_msg);
}

void TraversabilityLabelGeneration::initializeLabelMap()
{
  // Define map geometry
  auto map_length = featuremap_.getLength();
  auto map_resolution = featuremap_.getResolution();
  labelmap_ = std::make_shared<grid_map::HeightMap>(map_length.x(), map_length.y(), map_resolution);
  labelmap_->setFrameId(featuremap_.getFrameId());

  // Add layers
  labelmap_->addLayer("elevation");
  labelmap_->addLayer("variance");
  labelmap_->addLayer("step");
  labelmap_->addLayer("slope");
  labelmap_->addLayer("roughness");
  labelmap_->addLayer("curvature");
  labelmap_->addLayer("footprints");
  labelmap_->addLayer("traversability_label");
  labelmap_->setBasicLayers({ "elevation", "variance" });
}

void TraversabilityLabelGeneration::generateTraversabilityLabels()
{
  // negative label
  recordOverThresholdAreas(*labelmap_);

  // positive label
  recordFootprint(*labelmap_);

  // recordUnknownAreas();
}

void TraversabilityLabelGeneration::recordFootprint(grid_map::HeightMap& labelmap)
{
  // Get Transform from base_link to map (typically provided by 3D pose estimator)
  auto [get_transform_b2m, base2map] = tf_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
    return;

  grid_map::Position footprint(base2map.transform.translation.x, base2map.transform.translation.y);
  grid_map::CircleIterator iterator(labelmap, footprint, footprint_radius_);

  for (iterator; !iterator.isPastEnd(); ++iterator)
  {
    Eigen::Vector3d grid_with_elevation;
    if (!labelmap.getPosition3(labelmap.getHeightLayer(), *iterator, grid_with_elevation))
      continue;

    labelmap.at("traversability_label", *iterator) = (float)Traversability::TRAVERSABLE;
    labelmap.at("footprints", *iterator) = (float)Traversability::TRAVERSABLE;
  }
}

void TraversabilityLabelGeneration::recordOverThresholdAreas(grid_map::HeightMap& labelmap)
{
  for (grid_map::GridMapIterator iter(labelmap); !iter.isPastEnd(); ++iter)
  {
    Eigen::Vector3d grid_with_elevation;
    if (!labelmap.getPosition3(labelmap.getHeightLayer(), *iter, grid_with_elevation))
      continue;

    // bool non_traversable = (labelmap.at("slope", *iter) > max_acceptable_slope_ &&
    //                            labelmap.at("step", *iter) > max_acceptable_step_);

    if (!std::isfinite(labelmap.at("step", *iter)))
      continue;

    bool non_traversable = labelmap.at("step", *iter) > max_acceptable_step_;
    if (non_traversable)
    {
      labelmap.at("traversability_label", *iter) = (float)Traversability::NON_TRAVERSABLE;
      continue;
    }

    bool LABEL_EQUALS_NON_TRAVERSABLE =
        std::abs(labelmap.at("traversability_label", *iter) - (float)Traversability::NON_TRAVERSABLE) < 1e-6;
    if (LABEL_EQUALS_NON_TRAVERSABLE)
    {
      labelmap.at("traversability_label", *iter) = NAN;
    }
  }
}

void TraversabilityLabelGeneration::recordUnknownAreas(grid_map::HeightMap& labelmap)
{
  for (grid_map::GridMapIterator iter(labelmap); !iter.isPastEnd(); ++iter)
  {
    Eigen::Vector3d grid_with_elevation;
    if (!labelmap.getPosition3(labelmap.getHeightLayer(), *iter, grid_with_elevation))
      continue;

    const auto& traversability_label = labelmap.at("traversability_label", *iter);
    bool has_label = std::isfinite(traversability_label);
    if (has_label)
      continue;

    labelmap.at("traversability_label", *iter) = (float)Traversability::UNKNOWN;
  }
}

bool TraversabilityLabelGeneration::saveLabeledData(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  is_labeling_callback_activated_ = true;
  std::cout << "Generating Traversability Labels..." << std::endl;

  // See generateTraversabilityLabels callback
  return true;
}

void TraversabilityLabelGeneration::generateTraversabilityLabels(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if (!is_labeling_callback_activated_)
    return;

  // Load globalmap info
  double map_length_x;
  double map_length_y;
  double grid_resolution;
  if (!ros::param::get("/height_mapping/global_mapping/mapLengthXGlobal", map_length_x))
  {
    ROS_ERROR("[LabelGenerationService] Failed to get global map parameter");
    return;
  }
  ros::param::get("/height_mapping/global_mapping/mapLengthYGlobal", map_length_y);
  ros::param::get("/height_mapping/global_mapping/gridResolution", grid_resolution);

  // Load feature extraction parameter
  double normal_estimation_radius;
  if (!ros::param::get("/feature_extraction/normalEstimationRadius", normal_estimation_radius))
  {
    ROS_ERROR("[LabelGenerationService] Failed to get feature extraction parameter");
    return;
  }

  // Feature extraction of Global map
  auto pointcloud = *cloud;
  grid_map::FeatureMap featuremap(map_length_x, map_length_y, grid_resolution);
  featuremap.setNormalEstimationRadius(normal_estimation_radius);
  HeightMapConverter::fromPointCloud2(pointcloud, featuremap);
  featuremap.extractFeatures();

  // Label generation: Non-traversable areas
  recordOverThresholdAreas(featuremap);

  // Save labeled data to csv
  saveLabeledDataToCSV(featuremap);

  recordUnknownAreas(featuremap);

  saveUnlabeledDataToCSV(featuremap);

  // Publish featuremap
  grid_map_msgs::GridMap featuremap_msg;
  grid_map::GridMapRosConverter::toMessage(featuremap, featuremap_msg);
  pub_labelmap_.publish(featuremap_msg);
  ros::Duration(10).sleep();

  std::cout << "sub test" << std::endl;

  is_labeling_callback_activated_ = false;
}

void TraversabilityLabelGeneration::saveLabeledDataToCSV(const grid_map::HeightMap& labelmap)
{
  std::string filename = "/home/ikhyeon/labeled_data.csv";
  std::ofstream file(filename);
  file << "step,slope,roughness,curvature,variance,traversability_label\n";  // header

  // Iterate over the grid map
  for (grid_map::GridMapIterator iter(labelmap); !iter.isPastEnd(); ++iter)
  {
    if (labelmap.isEmptyAt(*iter))
      continue;

    // Write data to csv
    float step = labelmap.at("step", *iter);
    float slope = labelmap.at("slope", *iter);
    float roughness = labelmap.at("roughness", *iter);
    float curvature = labelmap.at("curvature", *iter);
    float variance = labelmap.at("variance", *iter);
    float traversability_label = labelmap.at("traversability_label", *iter);

    if (!std::isfinite(traversability_label))
      continue;

    file << step << "," << slope << "," << roughness << "," << curvature << "," << variance << ","
         << traversability_label << "\n";
  }

  file.close();
}

void TraversabilityLabelGeneration::saveUnlabeledDataToCSV(const grid_map::HeightMap& labelmap)
{
  std::string filename = "/home/ikhyeon/unlabeled_data.csv";
  std::ofstream file(filename);
  file << "step,slope,roughness,curvature,variance,traversability_label\n";  // header

  // Iterate over the grid map
  for (grid_map::GridMapIterator iter(labelmap); !iter.isPastEnd(); ++iter)
  {
    if (labelmap.isEmptyAt(*iter))
      continue;

    // Write data to csv
    float step = labelmap.at("step", *iter);
    float slope = labelmap.at("slope", *iter);
    float roughness = labelmap.at("roughness", *iter);
    float curvature = labelmap.at("curvature", *iter);
    float variance = labelmap.at("variance", *iter);
    float traversability_label = labelmap.at("traversability_label", *iter);

    if (traversability_label < (float)Traversability::UNKNOWN - 1e-6)
      continue;

    file << step << "," << slope << "," << roughness << "," << curvature << "," << variance << ","
         << traversability_label << "\n";
  }
  file.close();
}