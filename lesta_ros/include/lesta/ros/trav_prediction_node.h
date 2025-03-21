/*
 * trav_prediction_node.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "common/ros/common.h"
#include "lesta/core/core.h"

namespace lesta_ros {

class TravPredictionNode {
public:
  struct Config {
    std::string lidarscan_topic;
    double pose_update_rate;
    double map_pub_rate;
    bool remove_backpoints;
    bool debug_mode;
  } cfg_;

  TravPredictionNode();
  ~TravPredictionNode() = default;
  void loadConfig(const ros::NodeHandle &nh);

private:
  // init functions
  void initializeTimers();
  void initializePubSubs();
  void initializeServices();

  // main logic
  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  pcl::PointCloud<Laser>::Ptr
  preprocessScan(const pcl::PointCloud<Laser>::Ptr &scan_raw,
                 const geometry_msgs::TransformStamped &sensor2base,
                 const geometry_msgs::TransformStamped &base2map);
  std::vector<grid_map::Index>
  terrainMapping(const pcl::PointCloud<Laser>::Ptr &inputcloud,
                 const Eigen::Vector3f &sensorOrigin3D);

  // helper functions
  std::vector<grid_map::Index>
  getMeasuredIndices(const HeightMap &map, const pcl::PointCloud<Laser>::Ptr &scan);
  void publishDownsampledScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishFilteredScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishRasterizedScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishHeightmap(const HeightMap &map);
  void publishTravMap(const HeightMap &map);

  // Timer functions
  void updateMapOrigin(const ros::TimerEvent &event);
  void publishMaps(const ros::TimerEvent &event);

  ros::NodeHandle nh_;

  // Subscribers & Publishers
  ros::Subscriber sub_lidarscan_;
  ros::Publisher pub_downsampled_scan_;
  ros::Publisher pub_filtered_scan_;
  ros::Publisher pub_rasterized_scan_;
  ros::Publisher pub_heightmap_;
  ros::Publisher pub_travmap_;

  // Timers
  ros::Timer pose_update_timer_;
  ros::Timer map_publish_timer_;

  // Core objects
  std::unique_ptr<lesta::HeightMapper> mapper_;
  std::unique_ptr<lesta::FeatureExtractor> feature_extractor_;
  std::unique_ptr<lesta::TraversabilityEstimator> trav_estimator_;
  TransformOps tf_;
  FrameID frame_id_;

  // State variables
  bool lidarscan_received_{false};
};
} // namespace lesta_ros