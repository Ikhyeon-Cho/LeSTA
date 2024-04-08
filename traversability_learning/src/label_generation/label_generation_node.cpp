/*
 * label_generation_node.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "label_generation/LabelGeneration.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "label_generation");

  TraversabilityLabelGeneration label_generation_node;

  ros::spin();

  return 0;
}