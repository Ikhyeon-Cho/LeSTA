#pragma once

#include <string>
#include <ros/node_handle.h>

#include "lesta/core/core.h"

namespace lesta_ros::height_mapper {

static lesta::HeightMapper::Config loadConfig(const ros::NodeHandle &nh) {

  lesta::HeightMapper::Config cfg;
  nh.param<std::string>("estimator_type", cfg.estimator_type, "StatMean");
  nh.param<std::string>("frame_id", cfg.frame_id, "map");
  nh.param<double>("map_length_x", cfg.map_length_x, 10.0);
  nh.param<double>("map_length_y", cfg.map_length_y, 10.0);
  nh.param<double>("grid_resolution", cfg.grid_resolution, 0.1);
  nh.param<double>("min_height_threshold", cfg.min_height, -0.2);
  nh.param<double>("max_height_threshold", cfg.max_height, 1.5);
  return cfg;
}
} // namespace lesta_ros::height_mapper

namespace lesta_ros::global_mapper {

static lesta::GlobalMapper::Config loadConfig(const ros::NodeHandle &nh) {

  lesta::GlobalMapper::Config cfg;
  nh.param<std::string>("estimator_type", cfg.estimator_type, "StatMean");
  nh.param<std::string>("frame_id", cfg.frame_id, "map");
  nh.param<double>("map_length_x", cfg.map_length_x, 400.0);
  nh.param<double>("map_length_y", cfg.map_length_y, 400.0);
  nh.param<double>("grid_resolution", cfg.grid_resolution, 0.1);
  nh.param<double>("min_height_threshold", cfg.min_height, -0.2);
  nh.param<double>("max_height_threshold", cfg.max_height, 1.5);

  nh.param<std::string>("map_save_dir",
                        cfg.map_save_dir,
                        std::string("/home/") + std::getenv("USER") + "/Downloads");
  return cfg;
}
} // namespace lesta_ros::global_mapper

namespace lesta_ros::feature_extractor {

static lesta::FeatureExtractor::Config loadConfig(const ros::NodeHandle &nh) {

  lesta::FeatureExtractor::Config cfg;
  nh.param<double>("pca_radius", cfg.pca_radius, 0.2);

  return cfg;
}
} // namespace lesta_ros::feature_extractor

namespace lesta_ros::label_generator {

static lesta::LabelGenerator::Config loadConfig(const ros::NodeHandle &nh) {

  lesta::LabelGenerator::Config cfg;
  nh.param<double>("footprint_radius", cfg.footprint_radius, 0.5);
  nh.param<double>("max_traversable_step", cfg.max_traversable_step, 0.1);

  return cfg;
}
} // namespace lesta_ros::label_generator

namespace lesta_ros::traversability_estimator {

static lesta::TraversabilityEstimator::Config loadConfig(const ros::NodeHandle &nh) {

  lesta::TraversabilityEstimator::Config cfg;

  nh.param<std::string>("model_path", cfg.model_path, "");
  nh.param<int>("input_dimension", cfg.input_dimension, 4);
  nh.param<std::vector<std::string>>(
      "feature_fields",
      cfg.feature_fields,
      {"step", "slope", "roughness", "curvature", "elevation_variance"});
  nh.param<float>("binary_threshold", cfg.binary_threshold, 0.5);
  return cfg;
}
} // namespace lesta_ros::traversability_estimator

namespace lesta_ros::traversability_mapper {

static lesta::TraversabilityMapper::Config loadConfig(const ros::NodeHandle &nh) {

  lesta::TraversabilityMapper::Config cfg;
  nh.param<std::string>("model_path", cfg.model_path, "");
  nh.param<std::vector<std::string>>("feature_fields",
                                     cfg.feature_fields,
                                     {"step", "slope", "roughness", "curvature"});
  nh.param<int>("input_dimension", cfg.input_dimension, 4);
  nh.param<float>("binary_threshold", cfg.binary_threshold, 0.5);

  // mapping parameters
  nh.param<bool>("use_log_odds_update", cfg.use_log_odds_update, true);
  nh.param<float>("learning_rate", cfg.learning_rate, 0.7);
  nh.param<float>("prob_min", cfg.prob_min, 0.01);
  nh.param<float>("prob_max", cfg.prob_max, 0.99);
  return cfg;
}
} // namespace lesta_ros::traversability_mapper
