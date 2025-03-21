/*
 * TraversabilityNetwork.cpp
 *
 *  Created on: Feb 15, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/core/TraversabilityNetwork.h"
#include <torch/torch.h>
#include <iostream>

namespace lesta {

bool TraversabilityNetwork::loadCheckpoint(const std::string &model_path) {
  try {
    // Load the TorchScript model
    torch::Device device = torch::kCPU;
    // torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    model_ = torch::jit::load(model_path, device);
    model_.eval();
    model_loaded_ = true;

    // Extract just the filename from the full path
    std::string model_name = model_path;
    size_t last_slash_pos = model_path.find_last_of("/\\");
    if (last_slash_pos != std::string::npos) {
      model_name = model_path.substr(last_slash_pos + 1);
    }

    // Print model information
    std::cout << "\033[1;32m[lesta::TraversabilityNetwork]: Model loaded to "
              << (device.is_cuda() ? "GPU" : "CPU") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m[lesta::TraversabilityNetwork]: Model checkpoint: "
              << model_name << "\033[0m" << std::endl;
    return true;
  } catch (const c10::Error &e) {
    // std::cerr << "\033[1;31m[lesta::TraversabilityNetwork]: Error loading the model: "
    // << e.what() << "\033[0m" << std::endl;
    return false;
  }
}

std::vector<float>
TraversabilityNetwork::inference(const std::vector<Eigen::VectorXf> &features) {
  if (!model_loaded_) {
    throw std::runtime_error("Model not loaded");
  }

  std::vector<float> predictions;
  predictions.reserve(features.size());

  try {
    // Set to evaluation mode
    torch::NoGradGuard no_grad;

    for (const auto &feature : features) {
      // Convert Eigen vector to torch tensor
      std::vector<float> feature_vec(feature.data(), feature.data() + feature.size());
      auto options = torch::TensorOptions().dtype(torch::kFloat32);
      torch::Tensor input_tensor =
          torch::from_blob(feature_vec.data(), {1, input_dimension_}, options).clone();

      // Perform inference
      std::vector<torch::jit::IValue> inputs;
      inputs.push_back(input_tensor);

      // Forward pass
      torch::Tensor output = model_.forward(inputs).toTensor();

      // Get prediction value
      float prediction = output.item<float>();
      predictions.push_back(prediction);
    }
  } catch (const c10::Error &e) {
    std::cerr << "Error during inference: " << e.what() << std::endl;
    throw;
  }

  return predictions;
}

} // namespace lesta