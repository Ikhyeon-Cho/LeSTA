/*
 * traversability_mapping_node.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "terrain_mapping/TerrainMapping.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability_mapping_node");
  ros::NodeHandle nh("~");

  ros::TerrainMapping node;

  ros::spin();

  return 0;
}