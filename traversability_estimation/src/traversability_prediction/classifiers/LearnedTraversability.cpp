/*
 * LearnedTraversability.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "traversability_prediction/classifiers/LearnedTraversability.h"

bool LearnedTraversability::loadModel(const std::string& model_path)
{
  try
  {
    model_ = torch::jit::load(model_path);
    is_model_loaded_ = true;
  }
  catch (const c10::Error& e)
  {
    std::cerr << "Error loading the model: " << e.what() << "\n";
    return false;
  }
  return true;
}

std::vector<float> LearnedTraversability::inference(const std::vector<std::vector<float>>& inputs)
{
  if (!is_model_loaded_)
  {
    std::cerr << "Model is not loaded yet. \n";
    return {};
  }

  // std::vector<torch::jit::IValue> inputs_torch;
  // inputs_torch.push_back(torch::tensor(inputs).view({ inputs[0].size() , static_cast<long>(inputs.size()) }));

  // at::Tensor output = model_.forward(inputs_torch).toTensor();
  // auto tensor_data = output.accessor<float, 2>();
  // std::vector<float> prediction_values;
  // for (int i = 0; i < tensor_data.size(1); i++)
  // {
  //   prediction_values.push_back(tensor_data[0][i]);
  // }

  // return prediction_values;

  // Flatten the 2D vector into a 1D vector and determine dimensions
  std::vector<float> flat_inputs;
  size_t rows = inputs.size();
  size_t cols = 0;
  if (rows > 0)
  {
    cols = inputs[0].size();
    for (const auto& row : inputs)
    {
      flat_inputs.insert(flat_inputs.end(), row.begin(), row.end());
    }
  }

  std::vector<torch::jit::IValue> inputs_torch;
  inputs_torch.push_back(torch::tensor(flat_inputs).view({ static_cast<long>(rows), static_cast<long>(cols) }));

  at::Tensor output = model_.forward(inputs_torch).toTensor();
  auto tensor_data = output.accessor<float, 2>();
  std::vector<float> prediction_values;

  for (int i = 0; i < tensor_data.size(0); ++i)
  {
    prediction_values.push_back(tensor_data[i][0]);
  }

  return prediction_values;
}