/*
 * label_generation_node.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/ros/label_generation_node.h"
#include "lesta/ros/config.h"
#include "lesta/types/label_point.h"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <ros/package.h>

namespace lesta_ros {

LabelGenerationNode::LabelGenerationNode() : nh_("~") {

  // Configs from label_generation_node.yaml, feature_extraction_params.yaml
  ros::NodeHandle cfg_node(nh_, "node");
  ros::NodeHandle cfg_frame_id(nh_, "frame_id");
  ros::NodeHandle cfg_mapper(nh_, "height_mapper");
  ros::NodeHandle cfg_feature_extractor(nh_, "feature_extractor");
  ros::NodeHandle cfg_label_generator(nh_, "label_generator");

  // ROS node
  LabelGenerationNode::loadConfig(cfg_node);
  initializePubSubs();
  initializeServices();
  initializeTimers();

  frame_id_ = FrameID::loadFromConfig(cfg_frame_id);

  // Global Mapper
  mapper_ = std::make_unique<lesta::GlobalMapper>(global_mapper::loadConfig(cfg_mapper));

  // Feature Extractor
  feature_extractor_ = std::make_unique<lesta::FeatureExtractor>(
      feature_extractor::loadConfig(cfg_feature_extractor));

  // Label Generator
  label_generator_ = std::make_unique<lesta::LabelGenerator>(
      label_generator::loadConfig(cfg_label_generator));

  std::cout << "\033[1;33m[lesta_ros::LabelGenerationNode]: "
               "Label generation node initialized. Waiting for scan inputs...\033[0m\n";
}

void LabelGenerationNode::loadConfig(const ros::NodeHandle &nh) {

  cfg_.lidarscan_topic = nh.param<std::string>("lidarscan_topic", "/velodyne_points");
  cfg_.map_pub_rate = nh.param<double>("map_publish_rate", 10.0);
  cfg_.pose_update_rate = nh.param<double>("pose_update_rate", 10.0);
  cfg_.remove_backpoints = nh.param<bool>("remove_backpoints", true);
  cfg_.debug_mode = nh.param<bool>("debug_mode", false);
}

void LabelGenerationNode::initializePubSubs() {

  sub_lidarscan_ = nh_.subscribe(cfg_.lidarscan_topic,
                                 1,
                                 &LabelGenerationNode::lidarScanCallback,
                                 this);
  pub_filtered_scan_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/lesta/label_generation/scan_filtered", 1);
  pub_downsampled_scan_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/lesta/label_generation/scan_downsampled",
                                              1);
  pub_rasterized_scan_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/lesta/label_generation/scan_rasterized",
                                              1);
  pub_labelmap_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/lesta/label_generation/map_labeled", 1);
  pub_mapping_region_ =
      nh_.advertise<visualization_msgs::Marker>("/lesta/label_generation/mapping_region",
                                                1);

  if (cfg_.debug_mode) {
    // TODO: Add debug publishers
  }
}

void LabelGenerationNode::initializeServices() {
  srv_save_map_ = nh_.advertiseService("/lesta/save_label_map",
                                       &LabelGenerationNode::saveLabelMap,
                                       this);
}

void LabelGenerationNode::initializeTimers() {

  ros::Duration pose_update_dt(1.0 / cfg_.pose_update_rate);
  ros::Duration map_pub_dt(1.0 / cfg_.map_pub_rate);

  pose_update_timer_ = nh_.createTimer(pose_update_dt,
                                       &LabelGenerationNode::recordFootprints,
                                       this,
                                       false,
                                       false);
  map_publish_timer_ = nh_.createTimer(map_pub_dt,
                                       &LabelGenerationNode::publishLabelMap,
                                       this,
                                       false,
                                       false);
}

void LabelGenerationNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!lidarscan_received_) {
    lidarscan_received_ = true;
    frame_id_.sensor = msg->header.frame_id;
    pose_update_timer_.start();
    map_publish_timer_.start();
    std::cout << "\033[1;32m[lesta_ros::LabelGenerationNode]: Pointcloud Received! "
              << "Use LiDAR scans for label generation... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frame_id_.robot, frame_id_.sensor, sensor2base) ||
      !tf_.lookupTransform(frame_id_.map, frame_id_.robot, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // 3. Preprocess scan data: ready for terrain mapping
  auto scan_preprocessed = preprocessScan(scan_raw, sensor2base, base2map);
  if (!scan_preprocessed)
    return;

  // 4. Terrain mapping: sensor position is required for raycasting
  auto sensor2map = TransformOps::multiplyTransforms(sensor2base, base2map);
  Eigen::Vector3f sensor_position3d(sensor2map.transform.translation.x,
                                    sensor2map.transform.translation.y,
                                    sensor2map.transform.translation.z);
  auto measured_indices = terrainMapping(scan_preprocessed, sensor_position3d);

  // 5. Feature extraction
  feature_extractor_->extractFeatures(mapper_->getHeightMap(), measured_indices);

  // 6. Record non-traversable regions
  label_generator_->addObstacles(mapper_->getHeightMap(), measured_indices);
}

pcl::PointCloud<Laser>::Ptr
LabelGenerationNode::preprocessScan(const pcl::PointCloud<Laser>::Ptr &scan_raw,
                                    const geometry_msgs::TransformStamped &sensor2base,
                                    const geometry_msgs::TransformStamped &base2map) {

  // 1. Transform pointcloud to base frame
  auto scan_base = PointCloudOps::applyTransform<Laser>(scan_raw, sensor2base);

  // For visualization: downsample pointcloud
  auto scan_downsampled = PointCloudOps::downsampleVoxel<Laser>(scan_base, 0.4);
  publishDownsampledScan(scan_downsampled);

  // 2. Fast height filtering
  auto scan_preprocessed = boost::make_shared<pcl::PointCloud<Laser>>();
  mapper_->fastHeightFilter(scan_base, scan_preprocessed);

  // 3. Pass through filter
  scan_preprocessed =
      PointCloudOps::passThrough<Laser>(scan_preprocessed, "x", -5.0, 5.0);
  scan_preprocessed =
      PointCloudOps::passThrough<Laser>(scan_preprocessed, "y", -5.0, 5.0);

  // (Optional) Remove remoter points
  if (cfg_.remove_backpoints)
    scan_preprocessed =
        PointCloudOps::filterAngle2D<Laser>(scan_preprocessed, -135.0, 135.0);

  // 4. Publish filtered scan
  publishFilteredScan(scan_preprocessed);

  // 5. Transform pointcloud to map frame
  scan_preprocessed = PointCloudOps::applyTransform<Laser>(scan_preprocessed, base2map);

  if (scan_preprocessed->empty())
    return nullptr;
  return scan_preprocessed;
}

std::vector<grid_map::Index>
LabelGenerationNode::terrainMapping(const pcl::PointCloud<Laser>::Ptr &cloud_input,
                                    const Eigen::Vector3f &sensor_origin) {

  // 1. Rasterize scan and height mapping
  auto cloud_rasterized = mapper_->heightMapping(cloud_input);

  // 2. Publish rasterized scan
  publishRasterizedScan(cloud_rasterized);

  // 3. Raycasting to remove dynamic obstacles
  mapper_->raycasting(sensor_origin, cloud_rasterized);

  // 4. Get measured indices
  return getMeasuredIndices(mapper_->getHeightMap(), cloud_rasterized);
}

void LabelGenerationNode::recordFootprints(const ros::TimerEvent &event) {

  geometry_msgs::TransformStamped base2map;
  if (!tf_.lookupTransform(frame_id_.map, frame_id_.robot, base2map))
    return;

  // Record footprints
  grid_map::Position robot_position(base2map.transform.translation.x,
                                    base2map.transform.translation.y);
  auto &height_map = mapper_->getHeightMap();
  label_generator_->addFootprint(height_map, robot_position);
}

void LabelGenerationNode::publishLabelMap(const ros::TimerEvent &event) {

  std::vector<std::string> layers = {height_mapping::layers::Height::ELEVATION,
                                     lesta::layers::Feature::STEP,
                                     lesta::layers::Feature::SLOPE,
                                     lesta::layers::Feature::ROUGHNESS,
                                     lesta::layers::Feature::CURVATURE,
                                     height_mapping::layers::Height::ELEVATION_VARIANCE,
                                     lesta::layers::Label::FOOTPRINT,
                                     lesta::layers::Label::TRAVERSABILITY};
  sensor_msgs::PointCloud2 cloud_msg;
  const auto &height_map = mapper_->getHeightMap();
  const auto &valid_indices = mapper_->getMeasuredGridIndices();
  toPointCloud2(height_map, layers, valid_indices, cloud_msg);
  pub_labelmap_.publish(cloud_msg);

  // Publish map region
  visualization_msgs::Marker marker;
  toMapRegion(height_map, marker);
  pub_mapping_region_.publish(marker);
}

void LabelGenerationNode::publishDownsampledScan(
    const pcl::PointCloud<Laser>::Ptr &scan) {

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*scan, cloud_msg);
  pub_downsampled_scan_.publish(cloud_msg);
}

void LabelGenerationNode::publishFilteredScan(const pcl::PointCloud<Laser>::Ptr &scan) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*scan, cloud_msg);
  pub_filtered_scan_.publish(cloud_msg);
}

void LabelGenerationNode::publishRasterizedScan(const pcl::PointCloud<Laser>::Ptr &scan) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*scan, cloud_msg);
  pub_rasterized_scan_.publish(cloud_msg);
}

std::vector<grid_map::Index>
LabelGenerationNode::getMeasuredIndices(const HeightMap &map,
                                        const pcl::PointCloud<Laser>::Ptr &input_scan) {
  std::vector<grid_map::Index> indices;
  indices.reserve(input_scan->size());
  for (const auto &point : input_scan->points) {
    grid_map::Position position(point.x, point.y);
    grid_map::Index index;
    if (map.getIndex(position, index))
      indices.push_back(index);
  }
  return indices;
}

bool LabelGenerationNode::saveLabelMap(lesta::save_training_data::Request &req,
                                       lesta::save_training_data::Response &res) {

  std::cout
      << "\033[1;32m[lesta_ros::LabelGenerationNode]: Saving labeled map...\033[0m\n";

  // Get directory path from request (use parent directory of package if empty)
  std::string directory;
  if (req.directory.empty()) {
    // Get package path
    std::string package_path = ros::package::getPath("lesta");

    // Get parent directory by finding the last '/' and truncating
    size_t last_slash_pos = package_path.find_last_of('/');
    if (last_slash_pos != std::string::npos) {
      directory = package_path.substr(0, last_slash_pos + 1);
    } else {
      // Fallback to package directory if we can't determine parent
      directory = package_path + "/";
    }
  } else {
    directory = req.directory;

    // Expand tilde if present (for home directory)
    if (directory.length() > 0 && directory[0] == '~') {
      const char *home = std::getenv("HOME");
      if (home) {
        directory.replace(0, 1, home);
      }
    }

    // Ensure directory ends with a slash
    if (!directory.empty() && directory.back() != '/') {
      directory += '/';
    }
  }

  // Create filename based on whether a custom filename was provided
  std::string filename;
  if (req.filename.empty()) {
    // No filename provided, use default with timestamp
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    filename = directory + "labelmap_" + ss.str() + ".pcd";
  } else {
    // Use exactly the filename provided by the user
    filename = directory + req.filename;

    // Add .pcd extension if not already present
    if (filename.length() < 4 || filename.substr(filename.length() - 4) != ".pcd") {
      filename += ".pcd";
    }
  }

  // 1. Get the height map
  const auto &height_map = mapper_->getHeightMap();
  const auto &valid_indices = mapper_->getMeasuredGridIndices();

  // 2. Create PCL pointcloud with custom point type
  pcl::PointCloud<lesta_types::LabelPoint> label_cloud;
  label_cloud.header.frame_id = height_map.getFrameId();
  label_cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  label_cloud.points.reserve(valid_indices.size()); // reserve memory for efficiency

  // 3. Fill the cloud with the height map data
  for (const auto &index : valid_indices) {
    grid_map::Position3 position;
    height_map.getPosition3(height_mapping::layers::Height::ELEVATION, index, position);

    lesta_types::LabelPoint point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    point.elevation = height_map.at(height_mapping::layers::Height::ELEVATION, index);
    point.step = height_map.at(lesta::layers::Feature::STEP, index);
    point.slope = height_map.at(lesta::layers::Feature::SLOPE, index);
    point.roughness = height_map.at(lesta::layers::Feature::ROUGHNESS, index);
    point.curvature = height_map.at(lesta::layers::Feature::CURVATURE, index);
    point.variance =
        height_map.at(height_mapping::layers::Height::ELEVATION_VARIANCE, index);
    point.footprint = height_map.at(lesta::layers::Label::FOOTPRINT, index);
    point.traversability_label =
        height_map.at(lesta::layers::Label::TRAVERSABILITY, index);
    label_cloud.push_back(point);
  }
  label_cloud.width = label_cloud.points.size();
  label_cloud.height = 1;
  label_cloud.is_dense = false;

  // Save label map as .pcd
  if (pcl::io::savePCDFileASCII(filename, label_cloud) == -1) {
    std::string error_msg = "Failed to save label map to " + filename;
    std::cout << "\033[33m[lesta_ros::LabelGenerationNode]: " << error_msg << "\033[0m\n";
    res.success = false;
    return false;
  }

  std::string success_msg = "Label map saved to " + filename;
  std::cout << "\033[1;32m[lesta_ros::LabelGenerationNode]: " << success_msg
            << "\033[0m\n";
  res.success = true;
  return true;
}

void LabelGenerationNode::toPointCloud2(
    const HeightMap &map,
    const std::vector<std::string> &layers,
    const std::unordered_set<grid_map::Index> &measuredIndices,
    sensor_msgs::PointCloud2 &cloud) {

  // Setup cloud header
  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp.fromNSec(map.getTimestamp());
  cloud.is_dense = false;

  // Setup field names and cloud structure
  std::vector<std::string> fieldNames;
  fieldNames.reserve(layers.size());

  // Setup field names
  fieldNames.insert(fieldNames.end(), {"x", "y", "z"});
  for (const auto &layer : layers) {
    if (layer == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(layer);
    }
  }

  // Setup point field structure
  cloud.fields.clear();
  cloud.fields.reserve(fieldNames.size());
  int offset = 0;

  for (const auto &name : fieldNames) {
    sensor_msgs::PointField field;
    field.name = name;
    field.count = 1;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = offset;
    cloud.fields.push_back(field);
    offset += sizeof(float);
  }

  // Initialize cloud size
  const size_t num_points = measuredIndices.size();
  cloud.height = 1;
  cloud.width = num_points;
  cloud.point_step = offset;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);

  // Setup point field iterators
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> iterators;
  for (const auto &name : fieldNames) {
    iterators.emplace(name, sensor_msgs::PointCloud2Iterator<float>(cloud, name));
  }

  // Fill point cloud data
  size_t validPoints = 0;
  for (const auto &index : measuredIndices) {
    grid_map::Position3 position;
    if (!map.getPosition3(height_mapping::layers::Height::ELEVATION, index, position)) {
      continue;
    }

    // Update each field
    for (auto &[fieldName, iterator] : iterators) {
      if (fieldName == "x")
        *iterator = static_cast<float>(position.x());
      else if (fieldName == "y")
        *iterator = static_cast<float>(position.y());
      else if (fieldName == "z")
        *iterator = static_cast<float>(position.z());
      else if (fieldName == "rgb")
        *iterator = static_cast<float>(map.at("color", index));
      else
        *iterator = static_cast<float>(map.at(fieldName, index));
      ++iterator;
    }
    ++validPoints;
  }

  // Adjust final cloud size
  cloud.width = validPoints;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
}

void LabelGenerationNode::toMapRegion(const HeightMap &map,
                                      visualization_msgs::Marker &marker) {

  marker.ns = "height_map";
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.header.frame_id = map.getFrameId();
  marker.header.stamp = ros::Time::now();

  float length_x_half = (map.getLength().x() - 0.5 * map.getResolution()) / 2.0;
  float length_y_half = (map.getLength().y() - 0.5 * map.getResolution()) / 2.0;

  marker.points.resize(5);
  marker.points[0].x = map.getPosition().x() + length_x_half;
  marker.points[0].y = map.getPosition().y() + length_x_half;
  marker.points[0].z = 0;

  marker.points[1].x = map.getPosition().x() + length_x_half;
  marker.points[1].y = map.getPosition().y() - length_x_half;
  marker.points[1].z = 0;

  marker.points[2].x = map.getPosition().x() - length_x_half;
  marker.points[2].y = map.getPosition().y() - length_x_half;
  marker.points[2].z = 0;

  marker.points[3].x = map.getPosition().x() - length_x_half;
  marker.points[3].y = map.getPosition().y() + length_x_half;
  marker.points[3].z = 0;

  marker.points[4] = marker.points[0];
}
} // namespace lesta_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "label_generation_node");
  lesta_ros::LabelGenerationNode node;
  ros::spin();

  return 0;
}