/*
 * feature_extraction_node.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "feature_extraction/FeatureExtraction.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_extraction");

  FeatureExtraction node;

  ros::spin();

  return 0;
}