/*
 * ClassifierBase.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TRAVERSABILITY_CLASSIFIER_BASE_H
#define TRAVERSABILITY_CLASSIFIER_BASE_H

#include <vector>
#include <memory>

class TraversabilityClassifierBase
{
public:
  using Ptr = std::unique_ptr<TraversabilityClassifierBase>;

  TraversabilityClassifierBase() = default;
  virtual ~TraversabilityClassifierBase() = default;

  virtual std::vector<float> inference(const std::vector<std::vector<float>>& inputs) = 0;
  virtual bool loadModel(const std::string& model_path) = 0;
};

#endif  // TRAVERSABILITY_CLASSIFIER_BASE_H