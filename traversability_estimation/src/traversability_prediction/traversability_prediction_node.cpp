/*
 * traversability_prediction_node.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "traversability_prediction/TraversabilityPrediction.h"
#include <chrono>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability_prediction");

  TraversabilityPrediction node;

  ros::spin();

  return 0;
}