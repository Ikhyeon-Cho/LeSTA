/*
 * LearnedTraversability.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef LEARNED_TRAVERSABILITY_H
#define LEARNED_TRAVERSABILITY_H

#include "traversability_prediction/classifiers/ClassifierBase.h"
#include <iostream>
#include <torch/script.h>  // One-stop header.

class LearnedTraversability : public TraversabilityClassifierBase
{
public:
  LearnedTraversability() = default;
  virtual ~LearnedTraversability() = default;

  std::vector<float> inference(const std::vector<std::vector<float>>& inputs) override;
  bool loadModel(const std::string& model_path);

private:
  torch::jit::script::Module model_;
  bool is_model_loaded_ = false;
};

#endif  // LEARNED_TRAVERSABILITY_H